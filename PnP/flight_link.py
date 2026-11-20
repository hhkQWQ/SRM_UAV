"""最小MAVLink1/2链路，支持显式启用后的任务协议及模式命令。
默认仅遥测；不写参数。签名帧拒绝，需签名链路不适用。
"""
import math
import struct
import time
import config as C
from pose_solver import finite, norm, unit, quaternion_rotation

CRC_EXTRA = {0:50,31:246,32:185,33:104,39:254,40:230,41:28,42:28,
             43:132,44:221,45:232,47:153,51:196,73:38,
             76:152,77:143,84:143,111:34,230:163,242:104}
EVENT_MESSAGES = (33,40,42,47,51,77,242)

class Clock:
    def __init__(self):
        self.value = 0
        self.previous = time.ticks_us() if hasattr(time,"ticks_us") else None
    def now_us(self):
        if self.previous is None:
            return time.monotonic_ns()//1000
        current = time.ticks_us()
        self.value += time.ticks_diff(current,self.previous)
        self.previous = current
        return self.value

def crc(data, extra):
    value = 65535
    for byte in data:
        tmp = byte^(value&255)
        tmp = (tmp^(tmp<<4))&255
        value = ((value>>8)^(tmp<<8)^(tmp<<3)^(tmp>>4))&65535
    tmp = extra^(value&255)
    tmp = (tmp^(tmp<<4))&255
    return ((value>>8)^(tmp<<8)^(tmp<<3)^(tmp>>4))&65535

def pack_message(mid, payload, seq, sysid, compid):
    payload = payload.rstrip(b"\0") or b"\0"
    head = bytes((len(payload),0,0,seq&255,sysid,compid,mid&255,(mid>>8)&255,(mid>>16)&255))
    return b"\xfd"+head+payload+struct.pack("<H",crc(head+payload,CRC_EXTRA[mid]))

def padded(p, size):
    return (p+b"\0"*size)[:size]

def pack_i64(v):
    return struct.pack("<II",v&0xffffffff,(v>>32)&0xffffffff)

def unpack_i64(p, offset=0):
    lo,hi = struct.unpack("<II",p[offset:offset+8])
    value = lo+(hi<<32)
    return value-(1<<64) if hi&0x80000000 else value

class Parser:
    def __init__(self):
        self.buffer = bytearray()
        self.bad_crc = self.signed_rejected = self.skipped = 0
    def feed(self, data):
        self.buffer.extend(data)
        buf, start, packets = self.buffer,0,[]
        while start < len(buf):
            if buf[start] not in (253,254):
                start += 1
                continue
            if len(buf)-start < 2:
                break
            v2 = buf[start] == 253
            header = 10 if v2 else 6
            if len(buf)-start < header:
                break
            if v2 and buf[start+2]&254:
                start += 1
                continue
            signed = v2 and buf[start+2]&1
            begin = start+header
            end = begin+buf[start+1]
            next_start = end+2+(13 if signed else 0)
            if next_start > len(buf):
                break
            if v2:
                sysid,compid = buf[start+5],buf[start+6]
                mid = buf[start+7]|(buf[start+8]<<8)|(buf[start+9]<<16)
            else:
                sysid,compid,mid = buf[start+3],buf[start+4],buf[start+5]
            if signed:
                self.signed_rejected += 1
            elif mid not in CRC_EXTRA:
                self.skipped += 1
            elif crc(buf[start+1:end],CRC_EXTRA[mid]) != (buf[end]|buf[end+1]<<8):
                self.bad_crc += 1
                start += 1
                continue
            else:
                packets.append((mid,sysid,compid,bytes(buf[begin:end])))
            start = next_start
        if start:
            self.buffer = buf[start:]
        if len(self.buffer) > 1024:
            self.buffer = bytearray()
        return packets

def _qmul(a,b):
    w,x,y,z = a
    v,i,j,k = b
    return (w*v-x*i-y*j-z*k,w*i+x*v+y*k-z*j,w*j-x*k+y*v+z*i,w*k+x*j-y*i+z*v)

class FlightLink:
    """Bounded MAVLink service shared by the board and the simulation.

    Local physical time comes exclusively from ``clock``. The host clock is
    used only to diagnose service gaps and to bound a synchronous I/O window.
    Queue times never participate in the TIMESYNC offset equation.
    """
    def __init__(self, uart=None, clock=None):
        self.clock = clock or Clock()
        if uart is None:
            from pyb import UART
            uart = UART(C.UART_ID,C.BAUDRATE,bits=8,parity=None,stop=1,
                        timeout=0,timeout_char=0,read_buf_len=4096)
        self.uart,self.parser = uart,Parser()
        self.target = None
        self.sequence = self.generation = 0
        self.hb_us = self.hb_sent = self.sync_sent = self.sync_us = -100000000
        self.request_sent = -100000000
        self.armed = self.offboard = self.mission = False
        self.main_mode = self.sub_mode = 0
        self.mission_seq = None
        self.events = []
        self.attitudes,self.velocities = [],[]
        self.estimator = self.offset_us = self.pending_sync = self.boot_ms = None
        self.sync_count = 0
        self.queue = []
        self.active = self.latest_setpoint = None
        self.last_rx_us = 0
        self.dropped = self.rx_out_of_order = self.reply_mismatch = 0
        self.sync_timeouts = self.request_timeouts = self.events_dropped = 0
        self._comm_events = []
        self._sync_samples = []
        self._expired_tokens = []
        self._last_poll_us = self._last_poll_wall_us = None
        self._clock_generation = getattr(self.clock,'generation',None)
        self._max_poll_gap = self._max_poll_wall_gap = 0
        self._last_rtt = self._last_innovation = self._last_queue_wait = None
        self._control_capture_us = None
        self._control_measurement_us = None
        self._tx_status = 'NONE'
        self._tx_capture_us = self._tx_us = self._tx_age_us = None
        self._tx_control_us = None
        self._streams = {mid:{'hz':hz,'attempts':0,'sent':None,'ack':None,
                              'seen':None,'stamp':None,'period':None,'failed':False}
                         for mid,hz in ((31,50),(32,30),(230,5))}
        self._stream_pending = None

    @staticmethod
    def _wall_us():
        return time.monotonic_ns()//1000 if hasattr(time,'monotonic_ns') else time.ticks_us()

    @staticmethod
    def _wall_diff(now,previous):
        return now-previous if hasattr(time,'monotonic_ns') else time.ticks_diff(now,previous)

    @staticmethod
    def _median(values):
        values = sorted(values)
        middle = len(values)//2
        return values[middle] if len(values)&1 else (values[middle-1]+values[middle])//2

    def _event(self,event,now=None,**values):
        if len(self._comm_events) >= C.COMM_EVENT_BUFFER_SIZE:
            self._comm_events.pop(0)
            self.events_dropped += 1
        item = {'local_us':self.clock.now_us() if now is None else now,
                'event':event,'generation':self.generation}
        item.update(values)
        self._comm_events.append(item)

    def drain_comm_events(self):
        result,self._comm_events = self._comm_events,[]
        return result

    def connected(self,now):
        return self.target is not None and 0 <= now-self.hb_us <= C.HEARTBEAT_TIMEOUT_MS*1000

    def synced(self,now):
        return (self.offset_us is not None and self.sync_count >= C.SYNC_MIN_SAMPLES
                and 0 <= now-self.sync_us <= C.SYNC_TIMEOUT_MS*1000)

    def _stamp(self,boot_ms):
        value = boot_ms*1000
        if self.offset_us is not None:
            reference = self.clock.now_us()+self.offset_us
            wrap = (1<<32)*1000
            value += ((reference-value+wrap//2)//wrap)*wrap
        return int(value)

    def _history(self,history,item):
        if history and item[0] <= history[-1][0]:
            self.rx_out_of_order += 1
            return False
        if len(history) >= C.HISTORY_SIZE:
            history.pop(0)
        history.append(item)
        return True

    def _clear_commands(self):
        if self.active is not None or self.latest_setpoint is not None:
            self.dropped += 1
        self.active = self.latest_setpoint = None
        self.queue = [x for x in self.queue if x[4] is None]
        self._tx_status = 'CANCELLED'
        self._control_capture_us = None
        self._control_measurement_us = None

    def _restart(self):
        self.generation += 1
        self.attitudes,self.velocities = [],[]
        self.estimator = self.offset_us = self.pending_sync = self.boot_ms = None
        self.sync_count = 0
        self._sync_samples = []
        self._clear_commands()
        self.queue = []
        self.events = []
        self.mission_seq = None
        self.hb_us = self.hb_sent = self.sync_sent = self.sync_us = self.request_sent = -100000000
        self._last_poll_us = self.clock.now_us()
        self._last_poll_wall_us = self._wall_us()
        self._clock_generation = getattr(self.clock,'generation',None)
        self.parser.buffer = bytearray()
        self._tx_capture_us = self._tx_us = self._tx_age_us = None
        self._tx_control_us = None
        self._stream_pending = None
        for stream in self._streams.values():
            stream.update(attempts=0,sent=None,ack=None,seen=None,stamp=None,period=None,failed=False)
        self._event('LINK_RESET')

    def _stream_seen(self,mid,now,stamp):
        stream = self._streams[mid]
        if stream['stamp'] is not None and stamp > stream['stamp']:
            period = stamp-stream['stamp']
            stream['period'] = period if stream['period'] is None else (stream['period']*3+period)//4
        stream['seen'] = now
        stream['stamp'] = stamp

    def _receive(self,mid,sysid,compid,payload,now):
        # Timestamp each decoded message, never the entry of a receive batch.
        now = self.clock.now_us()
        if mid == 0:
            custom,kind,autopilot,base,status,version = struct.unpack('<IBBBBB',padded(payload,9))
            if autopilot != 12 or compid != C.TARGET_COMPONENT:
                return
            if C.TARGET_SYSTEM is not None and sysid != C.TARGET_SYSTEM:
                return
            if self.target is None:
                self.target = (sysid,compid)
            if self.target == (sysid,compid):
                self.hb_us = now
                self.armed = bool(base&128)
                self.main_mode = (custom>>16)&255 if base&1 else 0
                self.sub_mode = (custom>>24)&255 if base&1 else 0
                self.offboard = self.main_mode == 6
                self.mission = self.main_mode == 4 and self.sub_mode == 4
        if self.target != (sysid,compid):
            return
        if mid == 77:
            p = padded(payload,10)
            if p[8] not in (0,C.SOURCE_SYSTEM) or p[9] not in (0,C.SOURCE_COMPONENT):
                return
            command,result = struct.unpack('<HB',p[:3])
            if command == 511 and self._stream_pending is not None:
                stream = self._streams[self._stream_pending]
                stream['ack'] = result
                if result not in (0,5) and stream['sent'] is not None and 0<=now-stream['sent']<=C.STREAM_RETRY_MS*1000:
                    # COMMAND_ACK does not echo the requested message ID.
                    # Actual stream receipt, not a possibly delayed NACK,
                    # decides success; retries remain bounded below.
                    self._event('STREAM_REJECTED',now,reason=str(self._stream_pending)+':'+str(result))
        if mid == 31:
            values = struct.unpack('<I7f',padded(payload,32))
            boot,q,gyro = values[0],tuple(values[1:5]),tuple(values[5:8])
            if not finite(q+gyro) or abs(norm(q)-1.) > .15:
                return
            if self.boot_ms is not None:
                delta = ((boot-self.boot_ms+(1<<31))&0xffffffff)-(1<<31)
                if delta < -1000:
                    previous = self.boot_ms
                    self._restart()
                    self._event('BOOT_BACKWARD',now,reason=str(previous)+'->'+str(boot))
                elif delta <= 0:
                    self.rx_out_of_order += 1
                    return
            self.boot_ms = boot
            self._history(self.attitudes,(self._stamp(boot),unit(q),gyro,now))
            self._stream_seen(31,now,self.attitudes[-1][0])
        elif mid == 32:
            values = struct.unpack('<I6f',padded(payload,28))
            if finite(values[4:7]):
                if self._history(self.velocities,(self._stamp(values[0]),tuple(values[4:7]),now)):
                    self._stream_seen(32,now,self.velocities[-1][0])
        elif mid == 230:
            values = struct.unpack('<8fH',padded(payload,42)[8:42])
            flags = values[-1]
            ok = ((flags&7)==7 and not(flags&(128|2048)) and
                  math.isfinite(values[0]) and 0. <= values[0] < 1.)
            self.estimator = (now,ok,flags)
            self._stream_seen(230,now,unpack_i64(padded(payload,42)))
        elif mid == 111:
            p = padded(payload,16)
            tc1,token = unpack_i64(p),unpack_i64(p,8)
            if tc1 == 0:
                self._queue(111,pack_i64(max(1,now*1000))+pack_i64(token))
            elif self.pending_sync is not None and token == self.pending_sync[0]:
                sent = self.pending_sync[1]
                self.pending_sync = None
                self._accept_sync(tc1//1000,sent,now)
            else:
                self.reply_mismatch += 1
                self._event('SYNC_LATE_REPLY' if token in self._expired_tokens else 'SYNC_TOKEN_MISMATCH',now)

    def _expire_sync(self,now):
        if self.offset_us is not None and now-self.sync_us>C.SYNC_TIMEOUT_MS*1000:
            self.sync_timeouts += 1
            self.offset_us,self.sync_count = None,0
            self._sync_samples = []
            self.attitudes,self.velocities = [],[]
            self._clear_commands()
            self._event('SYNC_TIMEOUT',now,sync_age_us=now-self.sync_us)

    def _accept_sync(self,remote,sent,now):
        # A reply arriving in the first poll after an idle period must not
        # revive an expired map with the old accumulated sample count.
        self._expire_sync(now)
        rtt = now-sent
        sample = remote-(sent+now)//2
        innovation = None if self.offset_us is None else sample-self.offset_us
        self._last_rtt,self._last_innovation = rtt,innovation
        if not 0 <= rtt <= C.SYNC_MAX_RTT_MS*1000:
            self._event('REJECT_RTT',now,rtt_us=rtt,innovation_us=innovation)
            return
        rows = [x for x in self._sync_samples if 0 <= now-x[0] <= C.SYNC_WINDOW_MS*1000]
        rows.append((now,rtt,sample))
        self._sync_samples = rows[-C.SYNC_WINDOW_SIZE:]
        rows = self._sync_samples
        best = sorted(rows,key=lambda x:(x[1],-x[0]))[:5]
        estimate = self._median([x[2] for x in best])
        jitter = C.SYNC_MAX_JITTER_US
        required = C.SYNC_MIN_SAMPLES
        if self.offset_us is None:
            consistent = [x[2] for x in best if abs(x[2]-estimate) <= jitter]
            self.sync_count = len(consistent)
            if len(consistent) < required:
                return
            self.offset_us = self._median(consistent)
            self.attitudes,self.velocities = [],[]
            event = 'ACCEPT_INITIAL'
        elif abs(sample-self.offset_us) <= jitter:
            inliers = [x[2] for x in best if abs(x[2]-self.offset_us) <= jitter]
            estimate = self._median(inliers) if inliers else sample
            self.offset_us += (estimate-self.offset_us)//5
            self.sync_count += 1
            self.sync_us = now
            return
        else:
            tail = rows[-required:]
            stable = (len(tail)>=required and
                      max(x[2] for x in tail)-min(x[2] for x in tail)<=jitter and
                      max(x[1] for x in tail)<=min(x[1] for x in rows)+C.SYNC_RTT_MARGIN_US)
            if not stable:
                self._event('REJECT_JITTER',now,rtt_us=rtt,innovation_us=innovation,offset_us=self.offset_us)
                return
            estimate = self._median([x[2] for x in tail])
            event = 'ACCEPT_RECONVERGE'
            if abs(estimate-self.offset_us)>C.SYNC_RESET_US:
                self.generation += 1
                self.attitudes,self.velocities = [],[]
                self._clear_commands()
                event = 'ACCEPT_RESET'
            self.offset_us = estimate
            self.sync_count = required
            self._sync_samples = tail
        self.sync_us = now
        self._event(event,now,offset_us=self.offset_us,rtt_us=rtt,innovation_us=innovation)

    def _attitude_sample(self,local_us):
        info = {'attitude_status':'UNSYNCED','attitude_before_dt_ms':None,
                'attitude_after_dt_ms':None,'velocity_age_ms':None}
        if not self.synced(self.clock.now_us()):
            return None,info
        if not self.attitudes:
            info['attitude_status'] = 'NO_HISTORY'
            return None,info
        stamp = local_us+self.offset_us
        history = self.attitudes
        exact = next((row for row in history if row[0]==stamp),None)
        if exact is not None:
            info.update(attitude_status='EXACT',attitude_before_dt_ms=0.,attitude_after_dt_ms=0.)
            return exact[1],info
        if stamp < history[0][0]:
            info.update(attitude_status='BEFORE_HISTORY',attitude_after_dt_ms=(history[0][0]-stamp)*.001)
            return None,info
        for i in range(1,len(history)):
            a,b = history[i-1],history[i]
            if a[0] <= stamp <= b[0]:
                info.update(attitude_before_dt_ms=(a[0]-stamp)*.001,
                            attitude_after_dt_ms=(b[0]-stamp)*.001)
                if b[0]-a[0]>C.TELEMETRY_TIMEOUT_MS*1000:
                    info['attitude_status'] = 'GAP_TOO_WIDE'
                    return None,info
                f = (stamp-a[0])/(b[0]-a[0])
                sign = -1. if sum(a[1][j]*b[1][j] for j in range(4))<0 else 1.
                info['attitude_status'] = 'INTERPOLATED'
                return unit(tuple((1-f)*a[1][j]+f*sign*b[1][j] for j in range(4))),info
        a = history[-1]
        elapsed = stamp-a[0]
        info['attitude_before_dt_ms'] = -elapsed*.001
        if not 0 <= elapsed <= C.ATTITUDE_EXTRAPOLATE_MS*1000:
            info['attitude_status'] = 'EXTRAPOLATION_LIMIT'
            return None,info
        dt = elapsed*1e-6
        half = tuple(v*dt*.5 for v in a[2])
        info['attitude_status'] = 'EXACT' if elapsed==0 else 'EXTRAPOLATED'
        return unit(_qmul(a[1],(1.,half[0],half[1],half[2]))),info

    def attitude_at(self,local_us):
        return self._attitude_sample(local_us)[0]

    def _velocity_sample(self,local_us):
        if not self.synced(self.clock.now_us()) or not self.velocities:
            return None,None
        stamp = local_us+self.offset_us
        history = self.velocities
        exact = next((row for row in history if row[0]==stamp),None)
        if exact is not None:
            return exact[1],0.
        if stamp<history[0][0]:
            return None,None
        for a,b in zip(history,history[1:]):
            if a[0]<=stamp<=b[0]:
                if b[0]-a[0]>C.TELEMETRY_TIMEOUT_MS*1000:
                    return None,None
                f = (stamp-a[0])/(b[0]-a[0])
                return tuple(a[1][i]+f*(b[1][i]-a[1][i]) for i in range(3)),0.
        a = history[-1]
        age = stamp-a[0]
        if 0<=age<=C.TELEMETRY_TIMEOUT_MS*1000:
            return a[1],age*.001
        return None,age*.001

    def velocity_at(self,local_us):
        return self._velocity_sample(local_us)[0]

    def observation_at(self,local_us):
        q,info = self._attitude_sample(local_us)
        velocity,age = self._velocity_sample(local_us)
        info['velocity_age_ms'] = age
        return q,velocity,info,self.generation

    def displacement(self,capture_us,control_us):
        if control_us<capture_us or not self.synced(self.clock.now_us()):
            return None
        knots = [capture_us]
        knots.extend(row[0]-self.offset_us for row in self.velocities
                     if capture_us<row[0]-self.offset_us<control_us)
        knots.append(control_us)
        velocities = [self.velocity_at(stamp) for stamp in knots]
        if any(v is None for v in velocities):
            return None
        # Even if an endpoint is exact, never integrate across an uncovered gap.
        for left,right in zip(knots,knots[1:]):
            if right-left>C.TELEMETRY_TIMEOUT_MS*1000:
                return None
        return tuple(sum((a[i]+b[i])*.5*(right-left)*1e-6
                         for left,right,a,b in zip(knots,knots[1:],velocities,velocities[1:]))
                     for i in range(3))

    def state(self,now=None):
        now = self.clock.now_us() if now is None else now
        reason = 'OK'
        if not self.connected(now): reason = 'HEARTBEAT_STALE'
        elif not self.synced(now): reason = 'TIME_UNSYNCED'
        elif not self.attitudes or not 0<=now-self.attitudes[-1][3]<=C.TELEMETRY_TIMEOUT_MS*1000: reason = 'ATTITUDE_STALE'
        elif not self.velocities or not 0<=now-self.velocities[-1][2]<=C.TELEMETRY_TIMEOUT_MS*1000: reason = 'VELOCITY_STALE'
        elif self.estimator is None or not 0<=now-self.estimator[0]<=C.ESTIMATOR_TIMEOUT_MS*1000 or not self.estimator[1]: reason = 'ESTIMATOR_UNREADY'
        if reason=='OK':
            q_age = now+self.offset_us-self.attitudes[-1][0]
            v_age = now+self.offset_us-self.velocities[-1][0]
            if not -30000<=q_age<=C.TELEMETRY_TIMEOUT_MS*1000: reason = 'ATTITUDE_SAMPLE_STALE'
            elif not -30000<=v_age<=C.TELEMETRY_TIMEOUT_MS*1000: reason = 'VELOCITY_SAMPLE_STALE'
        q = self.attitudes[-1][1] if self.attitudes else None
        yaw = 0.
        if q is not None:
            r = quaternion_rotation(q)
            yaw = math.atan2(r[1][0],r[0][0])
        return {'ready':reason=='OK','reason':reason,'armed':self.armed,'offboard':self.offboard,
                'q':q,'yaw':yaw,'mission':self.mission,'main_mode':self.main_mode,'sub_mode':self.sub_mode,
                'mission_seq':self.mission_seq,'generation':self.generation,
                'velocity':self.velocities[-1][1] if self.velocities else None,
                'local_position_ready':reason=='OK' and self.estimator is not None and (self.estimator[2]&40)==40,
                'tx_status':self._tx_status,'tx_capture_us':self._tx_capture_us,
                'tx_us':self._tx_us,'tx_age_us':self._tx_age_us,'tx_control_us':self._tx_control_us}

    def comm_snapshot(self,now=None):
        now = self.clock.now_us() if now is None else now
        def age(value): return now-value if value is not None and value>-100000000 else None
        return {'local_us':now,'reason':self.state(now)['reason'],'generation':self.generation,
                'offset_us':self.offset_us,'rtt_us':self._last_rtt,'innovation_us':self._last_innovation,
                'sync_age_us':age(self.sync_us),'queue_wait_us':self._last_queue_wait,
                'poll_gap_us':self._max_poll_gap,'poll_wall_gap_us':self._max_poll_wall_gap,
                'heartbeat_age_us':age(self.hb_us),
                'attitude_age_us':None if not self.attitudes or self.offset_us is None else now+self.offset_us-self.attitudes[-1][0],
                'velocity_age_us':None if not self.velocities or self.offset_us is None else now+self.offset_us-self.velocities[-1][0],
                'bad_crc':self.parser.bad_crc,'rx_out_of_order':self.rx_out_of_order,
                'reply_mismatch':self.reply_mismatch,'sync_timeouts':self.sync_timeouts,
                'tx_dropped':self.dropped,'events_dropped':self.events_dropped,
                'tx_capture_us':self._tx_capture_us,'tx_us':self._tx_us,'tx_age_us':self._tx_age_us}

    def _packet(self,mid,payload):
        result = pack_message(mid,payload,self.sequence,C.SOURCE_SYSTEM,C.SOURCE_COMPONENT)
        self.sequence = (self.sequence+1)&255
        return result

    def _queue(self,mid,payload):
        if len(self.queue)>=8:
            self._event('QUEUE_FULL',reason=str(mid))
            return False
        control,tag = None,None
        if mid==76 and len(payload)>=30:
            command = struct.unpack('<H',payload[28:30])[0]
            if command==176:
                main_mode = struct.unpack('<f',payload[4:8])[0]
                control = 'OFFBOARD' if main_mode==6. else 'MODE'
            elif command==400: control = 'ARM'
        if mid==111:
            tag = 'SYNC_REQUEST' if unpack_i64(padded(payload,16))==0 else 'SYNC_REPLY'
        self.queue.append([self._packet(mid,payload),0,self.clock.now_us(),False,control,tag])
        return True

    def drain_events(self):
        result,self.events = self.events,[]
        return result

    def cancel_mission_transfer(self):
        def transfer(x): return (x[0][7]|x[0][8]<<8|x[0][9]<<16) in (39,44,73)
        self.queue = [x for x in self.queue if not transfer(x)]
        if self.active is not None and self.active[1]==0 and transfer(self.active): self.active = None

    def send_message(self,mid,payload):
        return bool(C.NAV_ENABLE and self.connected(self.clock.now_us()) and
                    mid in (39,41,43,44,45,47,73) and self._queue(mid,payload))

    def send_command(self,command,params):
        if not C.NAV_ENABLE or not self.connected(self.clock.now_us()) or len(params)>7 or not finite(params):
            return False
        params = tuple(params)+(0.,)*(7-len(params))
        return self._queue(76,struct.pack('<7fHBBB',*(params+(command,self.target[0],self.target[1],0))))

    def request_mode(self,main_mode,sub_mode=0):
        return self.send_command(176,(1.,float(main_mode),float(sub_mode)))

    def cancel_offboard_requests(self):
        self.queue = [x for x in self.queue if x[4]!='OFFBOARD']
        if self.active is not None and self.active[4]=='OFFBOARD' and self.active[1]==0: self.active = None

    def cancel_setpoint(self):
        self.latest_setpoint = None
        if self.active is not None and self.active[3] and self.active[1]==0: self.active = None
        self.cancel_offboard_requests()
        if self._tx_status=='QUEUED': self._tx_status = 'CANCELLED'

    def set_control_source(self,capture_us,measurement_us=None):
        self._control_capture_us = capture_us
        self._control_measurement_us = measurement_us

    def send_velocity(self,velocity,yaw,now=None):
        now = self.clock.now_us() if now is None else now
        state = self.state(now)
        if not C.NAV_ENABLE or C.readiness_errors() or not state['ready'] or not state['armed']:
            self.cancel_setpoint()
            return False
        if len(velocity)!=3 or not finite(tuple(velocity)+(yaw,)):
            self.cancel_setpoint()
            return False
        mask = 1|2|4|64|128|256|2048
        stamp = int(now+self.offset_us)//1000
        payload = struct.pack('<I11fHBBB',stamp&0xffffffff,0.,0.,0.,velocity[0],velocity[1],velocity[2],
                              0.,0.,0.,yaw,0.,mask,self.target[0],self.target[1],1)
        self.latest_setpoint = [self._packet(84,payload),0,now,True,None,
                                ('SETPOINT',self._control_capture_us,self.generation,now,self._control_measurement_us)]
        self._tx_status = 'QUEUED'
        return True

    def _flush(self,now):
        if self.active is None:
            control = next((i for i,x in enumerate(self.queue) if x[4] is not None),None)
            sync = next((i for i,x in enumerate(self.queue) if len(x)>5 and x[5] in ('SYNC_REPLY','SYNC_REQUEST')),None)
            if control is not None: self.active = self.queue.pop(control)
            elif self.latest_setpoint is not None: self.active,self.latest_setpoint = self.latest_setpoint,None
            elif sync is not None: self.active = self.queue.pop(sync)
            elif self.queue: self.active = self.queue.pop(0)
            else: return
        packet,position,created,is_setpoint,control = self.active[:5]
        tag = self.active[5] if len(self.active)>5 else None
        now = self.clock.now_us()
        expired = now-created>C.MAX_TX_AGE_MS*1000
        sync_expired = tag in ('SYNC_REPLY','SYNC_REQUEST') and now-created>C.SYNC_QUEUE_TIMEOUT_MS*1000
        if position==0 and is_setpoint:
            health = self.state(now)
            measurement = tag[4] if isinstance(tag,tuple) and len(tag)>4 else None
            stale_measurement = measurement is not None and not 0<=now-measurement<=C.MAX_PREDICTION_MS*1000
            if not health['ready'] or not health['armed'] or stale_measurement:
                self.active = None
                self.dropped += 1
                self._tx_status = 'REJECTED'
                reason = ('OBSERVATION_STALE' if stale_measurement else
                          'DISARMED' if not health['armed'] else health['reason'])
                self._event('TX_REJECTED',now,reason=reason,queue_wait_us=now-created)
                return
        if position==0 and (((is_setpoint or control) and (not C.NAV_ENABLE or expired or not self.connected(now))) or sync_expired):
            self.active = None
            self.dropped += 1
            if is_setpoint: self._tx_status = 'EXPIRED'
            self._event('TX_EXPIRED',now,reason='SETPOINT' if is_setpoint else str(tag or control),queue_wait_us=now-created)
            return
        if position==0 and tag in ('SYNC_REQUEST','SYNC_REPLY'):
            p = padded(packet[10:10+packet[1]],16)
            if tag=='SYNC_REQUEST':
                # The generation occupies <1 us, preventing old-epoch replies
                # from matching after a local-clock rewind to the same instant.
                token = now*1000+self.generation%1000
                p = pack_i64(0)+pack_i64(token)
            else:
                token = None
                p = pack_i64(max(1,now*1000))+p[8:16]
            packet = pack_message(111,p,packet[4],packet[5],packet[6])
            self.active[0] = packet
        else: token = None
        # No event formatting or logging between the sampling above and write.
        sent = self.uart.write(packet[position:])
        if sent is None or sent<=0: return
        if position==0:
            if tag=='SYNC_REQUEST':
                self.pending_sync = (token,now)
                self._last_queue_wait = now-created
            elif isinstance(tag,tuple) and tag[0]=='STREAM':
                stream = self._streams[tag[1]]
                stream['attempts'] += 1
                stream['sent'] = now
                self.request_sent = now
                self._stream_pending = tag[1]
        self.active[1] += sent
        if self.active[1]>=len(packet):
            self.active = None
            if is_setpoint:
                self._tx_status = 'WRITTEN'
                self._tx_us = self.clock.now_us()
                self._tx_capture_us = tag[1] if isinstance(tag,tuple) else None
                self._tx_control_us = tag[3] if isinstance(tag,tuple) and len(tag)>3 else None
                self._tx_age_us = None if self._tx_capture_us is None else self._tx_us-self._tx_capture_us

    def _service_streams(self,now):
        if not C.REQUEST_STREAMS or not self.connected(now): return
        waiting = self.queue+([self.active] if self.active is not None else [])
        if any(len(x)>5 and isinstance(x[5],tuple) and x[5][0]=='STREAM' for x in waiting): return
        for mid in (31,32,230):
            s = self._streams[mid]
            if s['failed']: continue
            if s['sent'] is not None and now-s['sent']<C.STREAM_RETRY_MS*1000: return
            recent = s['seen'] is not None and 0<=now-s['seen']<=max(3e6/s['hz'],C.TELEMETRY_TIMEOUT_MS*1000)
            rate_ok = s['period'] is not None and s['period']<=2e6/s['hz']
            if s['attempts'] and recent and rate_ok:
                if self._stream_pending==mid: self._stream_pending = None
                continue
            if s['attempts']>=C.STREAM_MAX_ATTEMPTS:
                s['failed'] = True
                if self._stream_pending==mid: self._stream_pending = None
                self._event('STREAM_UNCONFIRMED',now,reason=str(mid))
                continue
            payload = struct.pack('<7fHBBB',float(mid),1e6/s['hz'],0.,0.,0.,0.,0.,511,self.target[0],self.target[1],0)
            if self._queue(76,payload):
                self.queue[-1][5] = ('STREAM',mid)
            return

    def poll(self,service_sync=False):
        now,wall = self.clock.now_us(),self._wall_us()
        epoch = getattr(self.clock,'generation',None)
        if self._last_poll_us is not None:
            gap = now-self._last_poll_us
            wall_gap = self._wall_diff(wall,self._last_poll_wall_us)
            if gap<0 or epoch!=self._clock_generation:
                self._restart()
                self._event('LOCAL_CLOCK_BACKWARD',now)
            else:
                self._max_poll_gap = max(self._max_poll_gap,gap)
                self._max_poll_wall_gap = max(self._max_poll_wall_gap,wall_gap)
        self._last_poll_us,self._last_poll_wall_us = now,wall
        self._clock_generation = epoch
        budget = C.RX_BYTES_PER_POLL
        while budget>0 and self.uart.any():
            data = self.uart.read(min(self.uart.any(),budget,512))
            if not data: break
            budget -= len(data)
            self.last_rx_us = self.clock.now_us()
            for packet in self.parser.feed(data):
                self._receive(packet[0],packet[1],packet[2],packet[3],self.clock.now_us())
        now = self.clock.now_us()
        if self.parser.buffer and now-self.last_rx_us>1000000: self.parser.buffer = bytearray()
        if not self.connected(now): self.cancel_setpoint()
        if self.pending_sync is not None and now-self.pending_sync[1]>C.SYNC_MAX_RTT_MS*1000:
            token,sent = self.pending_sync
            self.pending_sync = None
            self._expired_tokens = (self._expired_tokens+[token])[-4:]
            self.request_timeouts += 1
            self._event('SYNC_REPLY_TIMEOUT',now,rtt_us=now-sent)
        self._expire_sync(now)
        if self.target is not None:
            if now-self.hb_sent>=1000000:
                if self._queue(0,struct.pack('<IBBBBB',0,18,8,0,4,3)): self.hb_sent = now
            interval = C.SYNC_INTERVAL_MS if self.synced(now) and (self._last_innovation is None or abs(self._last_innovation)<=C.SYNC_MAX_JITTER_US) else C.SYNC_RECOVERY_INTERVAL_MS
            waiting = self.queue+([self.active] if self.active is not None else [])
            queued = any(len(x)>5 and x[5]=='SYNC_REQUEST' for x in waiting)
            if self.connected(now) and self.pending_sync is None and not queued and now-self.sync_sent>=interval*1000:
                if self._queue(111,pack_i64(0)+pack_i64(now*1000)): self.sync_sent = now
            self._service_streams(now)
        self._flush(self.clock.now_us())
        if service_sync and self.pending_sync is not None and self.connected(self.clock.now_us()):
            token,sent = self.pending_sync
            start_wall = self._wall_us()
            limit = C.SYNC_MAX_RTT_MS*1000
            while (self.pending_sync is not None and self.pending_sync[0]==token and
                   0<=self.clock.now_us()-sent<limit and
                   self._wall_diff(self._wall_us(),start_wall)<limit):
                self.poll(False)
                if hasattr(time,'sleep_ms'): time.sleep_ms(1)
                else: time.sleep(.001)
