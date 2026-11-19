"""Observe the original PnP FlightLink without replacing its synchronization decisions."""
import json
import os
import time


def wall_us():
    return time.monotonic_ns() // 1000


class ObservedTransport:
    """Delegate I/O; timestamp the original write API, including partial writes."""
    def __init__(self, transport, link):
        self.transport, self.link = transport, link

    def any(self):
        return self.transport.any()

    def read(self, size):
        return self.transport.read(size)

    def write(self, data):
        link = self.link
        active = link.active
        token = None
        if active is not None:
            packet, position, created = active[:3]
            if packet[7:10] == b'\x6f\x00\x00':
                payload = link._diag_module.padded(packet[10:10+packet[1]],16)
                if link._diag_module.unpack_i64(payload) == 0:
                    token = link._diag_module.unpack_i64(payload,8)
        start = link.clock.now_us() if token is not None else None
        result = self.transport.write(data)
        if token is not None and result is not None and result > 0:
            link._sync_diag('REQUEST_TX',start,token_ns=token,queued_us=created,
                queue_wait_us=start-created,bytes_before=position,bytes_written=result,
                packet_bytes=len(packet),complete=position+result >= len(packet))
        return result

    def close(self):
        return self.transport.close()


class SyncDiagnosticsMixin:
    def __init__(self, uart, clock, buffer_size=128):
        super().__init__(uart=uart,clock=clock)
        self._sync_events = []
        self._diag_limit = max(1,buffer_size)
        self.sync_events_dropped = self._diag_sequence = 0
        self._diag_poll_us = self._diag_poll_wall_us = None
        self._diag_gap_us = self._diag_gap_wall_us = None
        self._diag_max_gap_us = 0
        self._diag_status = None
        self._diag_pending_token = None
        self.uart = ObservedTransport(self.uart,self)

    def sync_limits(self):
        c = self._diag_config
        return {'interval_us':c.SYNC_INTERVAL_MS*1000,'max_rtt_us':c.SYNC_MAX_RTT_MS*1000,
            'max_jitter_us':c.SYNC_MAX_JITTER_US,'min_samples':c.SYNC_MIN_SAMPLES,
            'timeout_us':c.SYNC_TIMEOUT_MS*1000,'reset_innovation_us':50000,
            'buffer_size':self._diag_limit}

    def sync_snapshot(self, now=None):
        now = self.clock.now_us() if now is None else now
        age = None if self.offset_us is None else now-self.sync_us
        c = self._diag_config
        rx_buffer = getattr(self.uart.transport,'buffer',None)
        reason = ('NO_OFFSET' if self.offset_us is None else
                  'INSUFFICIENT_SAMPLES' if self.sync_count < c.SYNC_MIN_SAMPLES else
                  'CLOCK_BACKWARD' if age < 0 else
                  'ACCEPT_TIMEOUT' if age > c.SYNC_TIMEOUT_MS*1000 else 'SYNCED')
        return {'local_us':now,'generation':self.generation,'reason':reason,
            'offset_us':self.offset_us,'sync_count':self.sync_count,'sync_age_us':age,
            'boot_ms':self.boot_ms,'main_mode':self.main_mode,'sub_mode':self.sub_mode,
            'armed':self.armed,'mission':self.mission,'offboard':self.offboard,
            'pending_token_ns':self.pending_sync[0] if self.pending_sync else None,
            'pending_age_us':now-self.pending_sync[1] if self.pending_sync else None,
            'queue_depth':len(self.queue),'tx_active':self.active is not None,
            'poll_gap_us':self._diag_gap_us,'poll_wall_gap_us':self._diag_gap_wall_us,
            'max_poll_gap_us':self._diag_max_gap_us,
            'bad_crc':self.parser.bad_crc,'signed_rejected':self.parser.signed_rejected,
            'parser_buffer_bytes':len(self.parser.buffer),
            'rx_buffer_bytes':len(rx_buffer) if rx_buffer is not None else None,
            'events_dropped':self.sync_events_dropped}

    def _sync_diag(self, event, now, **values):
        if len(self._sync_events) >= self._diag_limit:
            self._sync_events.pop(0)
            self.sync_events_dropped += 1
        self._diag_sequence += 1
        item = self.sync_snapshot(now)
        item.update(values)
        item.update(event=event,event_sequence=self._diag_sequence,
                    observed_local_us=self.clock.now_us(),wall_us=wall_us())
        self._sync_events.append(item)

    def drain_sync_events(self):
        events,self._sync_events = self._sync_events,[]
        return events

    def _queue(self, mid, payload):
        result = super()._queue(mid,payload)
        if mid == 111:
            p = self._diag_module.padded(payload,16)
            if self._diag_module.unpack_i64(p) == 0:
                token = self._diag_module.unpack_i64(p,8)
                if self._diag_pending_token is not None:
                    self._sync_diag('REQUEST_REPLACED',self.clock.now_us(),token_ns=self._diag_pending_token)
                self._diag_pending_token = token
                self._sync_diag('REQUEST_QUEUED' if result else 'REQUEST_QUEUE_FULL',
                                self.clock.now_us(),token_ns=token)
        return result

    def _restart(self):
        previous = self.generation
        result = super()._restart()
        self._diag_pending_token = None
        self._sync_diag('LINK_RESTART',self.clock.now_us(),previous_generation=previous)
        return result

    def _receive(self, mid, sysid, compid, payload, now):
        selected = self.target == (sysid,compid)
        pending = self.pending_sync
        before_offset,before_count,before_sync = self.offset_us,self.sync_count,self.sync_us
        before_generation,before_boot = self.generation,self.boot_ms
        result = super()._receive(mid,sysid,compid,payload,now)
        if mid == 31 and selected and self.generation != before_generation:
            self._sync_diag('BOOT_BACKWARD',now,previous_boot_ms=before_boot,
                            received_boot_ms=self.boot_ms)
        if mid != 111:
            return result
        if not selected:
            self._sync_diag('RX_FOREIGN_SOURCE',now,source_system=sysid,source_component=compid)
            return result
        p = self._diag_module.padded(payload,16)
        tc1,ts1 = self._diag_module.unpack_i64(p),self._diag_module.unpack_i64(p,8)
        if tc1 == 0:
            self._sync_diag('REMOTE_REQUEST',now,token_ns=ts1)
        elif pending is not None and ts1 == pending[0]:
            self._diag_pending_token = None
            sent = pending[1]
            rtt = now-sent
            sample = tc1//1000-(sent+now)//2
            innovation = None if before_offset is None else sample-before_offset
            # Classification observes the original method's result. No branch here
            # updates offset/count/pending or makes a synchronization decision.
            accepted = (self.offset_us != before_offset or self.sync_count != before_count
                        or self.sync_us != before_sync or self.generation != before_generation)
            if accepted:
                event = ('ACCEPT_INITIAL' if before_offset is None else
                         'ACCEPT_RESET' if self.generation != before_generation else 'ACCEPT_FILTER')
            elif not 0 <= rtt <= self._diag_config.SYNC_MAX_RTT_MS*1000:
                event = 'REJECT_RTT'
            elif innovation is not None and abs(innovation) > self._diag_config.SYNC_MAX_JITTER_US:
                event = 'REJECT_JITTER'
            else:
                event = 'REPLY_IGNORED_UNCLASSIFIED'
            self._sync_diag(event,now,token_ns=ts1,remote_us=tc1//1000,request_us=sent,
                rtt_us=rtt,sample_offset_us=sample,innovation_us=innovation,
                offset_before_us=before_offset)
        else:
            self._sync_diag('REPLY_NO_PENDING' if pending is None else 'REPLY_TOKEN_MISMATCH',
                            now,token_ns=ts1,remote_us=tc1//1000)
        return result

    def poll(self, service_sync=False):
        now,wall = self.clock.now_us(),wall_us()
        if self._diag_poll_us is not None:
            self._diag_gap_us = now-self._diag_poll_us
            self._diag_gap_wall_us = wall-self._diag_poll_wall_us
            self._diag_max_gap_us = max(self._diag_max_gap_us,self._diag_gap_us)
            if self._diag_gap_us < 0:
                self._sync_diag('LOCAL_CLOCK_BACKWARD',now,previous_poll_us=self._diag_poll_us)
        self._diag_poll_us,self._diag_poll_wall_us = now,wall
        result = super().poll(service_sync=service_sync)
        now = self.clock.now_us()
        reason = self.sync_snapshot(now)['reason']
        if reason != self._diag_status:
            self._diag_status = reason
            self._sync_diag('SYNC_STATUS',now)
        return result


def diagnostic_link(link_module, config, uart, clock, buffer_size=128):
    class DiagnosticFlightLink(SyncDiagnosticsMixin,link_module.FlightLink):
        _diag_module = link_module
        _diag_config = config
    return DiagnosticFlightLink(uart=uart,clock=clock,buffer_size=buffer_size)


class SyncJSONLogger:
    def __init__(self, link, path=None, directory=".", metadata=None):
        self.link = link
        self.file = None
        self.path = path
        self.error = None
        self.last_snapshot_tick = self.last_flush_tick = None
        self.written_records = 0
        self.last_export_wall_us = self.max_export_wall_us = 0
        try:
            if path is None:
                for index in range(1,10000):
                    candidate = directory.rstrip("/")+"/pnp_sync_%04d.jsonl" % index
                    if not self._exists(candidate):
                        self.path = candidate
                        break
                else:
                    raise OSError("SYNC_LOG_INDEX_EXHAUSTED")
            elif self._exists(path):
                raise OSError("SYNC_LOG_ALREADY_EXISTS")
            self.file = open(self.path,"w")
            self._write({"record_type":"config","schema_version":1,
                         "clock_note":"local_us follows FlightLink clock; wall_us is host monotonic time when available",
                         "tx_note":"REQUEST_TX brackets UART/socket write; not physical wire completion",
                         "rx_note":"reply local_us is poll timestamp; observed_local_us/wall_us are parser processing observations, not hardware arrival",
                         "context_note":"event drain_context describes export time, not event time",
                         "limits":link.sync_limits(),"source":metadata})
            self.file.flush()
        except Exception as error:
            self._fail(error)

    @staticmethod
    def _exists(path):
        try:
            os.stat(path)
            return True
        except OSError as error:
            if error.args and error.args[0] == 2:
                return False
            raise

    def _write(self, item):
        line = json.dumps(item)+"\n"
        written = self.file.write(line)
        if written is not None and written != len(line):
            raise OSError("SYNC_LOG_SHORT_WRITE")
        self.written_records += 1

    def _fail(self, error):
        self.error = str(error)
        if self.file is not None:
            try:
                self.file.close()
            except Exception:
                pass
        self.file = None

    def record(self, now_us=None, context=None, force=False):
        if self.file is None:
            return
        started,previous_records = wall_us(),self.written_records
        try:
            now_us = self.link.clock.now_us() if now_us is None else now_us
            wall = time.monotonic_ns()//1000 if hasattr(time,"monotonic_ns") else None
            tick = now_us if wall is None else wall
            for event in self.link.drain_sync_events():
                event["record_type"] = "event"
                if context is not None:
                    event["drain_context"] = context
                self._write(event)
            if (force or self.last_snapshot_tick is None or
                    not 0 <= tick-self.last_snapshot_tick < 1000000):
                item = self.link.sync_snapshot(now_us)
                item.update(record_type="snapshot",wall_us=wall,
                            limits=self.link.sync_limits(),
                            prior_export_wall_us=self.last_export_wall_us,
                            max_export_wall_us=self.max_export_wall_us)
                if context is not None:
                    item["context"] = context
                self._write(item)
                self.last_snapshot_tick = tick
            if (force or self.last_flush_tick is None or
                    not 0 <= tick-self.last_flush_tick < 1000000):
                self.file.flush()
                self.last_flush_tick = tick
        except Exception as error:
            self._fail(error)
        finally:
            if self.written_records != previous_records:
                self.last_export_wall_us = wall_us()-started
                self.max_export_wall_us = max(self.max_export_wall_us,self.last_export_wall_us)

    def close(self):
        self.record(force=True)
        if self.file is not None:
            try:
                self.file.flush()
                self.file.close()
            except Exception as error:
                self._fail(error)
            self.file = None
