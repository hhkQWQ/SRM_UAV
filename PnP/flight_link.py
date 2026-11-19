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
    def __init__(self, uart=None, clock=None):
        self.clock = clock or Clock()
        if uart is None:
            from pyb import UART
            uart = UART(C.UART_ID,C.BAUDRATE,bits=8,parity=None,stop=1,
                        timeout=0,timeout_char=0,read_buf_len=4096)
        self.uart, self.parser = uart,Parser()
        self.target = None
        self.sequence = 0
        self.hb_us = -100000000
        self.armed = self.offboard = False
        self.main_mode = self.sub_mode = 0
        self.mission = False
        self.mission_seq = None
        self.events = []
        self.attitudes, self.velocities = [],[]
        self.estimator = None
        self.offset_us = None
        self.sync_count = 0
        self.sync_us = -100000000
        self.pending_sync = None
        self.sync_sent = -100000000
        self.hb_sent = -100000000
        self.request_sent = -100000000
        self.stream_index = 0
        self.boot_ms = None
        self.generation = 0
        self.queue = []
        self.active = None
        self.latest_setpoint = None
        self.last_rx_us = 0
        self.dropped = 0

    def connected(self, now):
        return self.target is not None and 0 <= now-self.hb_us <= C.HEARTBEAT_TIMEOUT_MS*1000

    def synced(self, now):
        return (self.offset_us is not None and self.sync_count >= C.SYNC_MIN_SAMPLES
                and 0 <= now-self.sync_us <= C.SYNC_TIMEOUT_MS*1000)

    def _stamp(self, boot_ms):
        value = boot_ms*1000
        if self.offset_us is not None:
            reference = self.clock.now_us()+self.offset_us
            wrap = (1<<32)*1000
            value += ((reference-value+wrap//2)//wrap)*wrap
        return int(value)

    def _history(self, history, item):
        if history and item[0] <= history[-1][0]:
            return
        if len(history) >= C.HISTORY_SIZE:
            history.pop(0)
        history.append(item)

    def _restart(self):
        self.generation += 1
        self.attitudes, self.velocities = [],[]
        self.estimator = None
        self.offset_us, self.sync_count, self.pending_sync = None,0,None
        self.stream_index = 0
        self.events = []
        self.mission_seq = None
        self.queue = []
        self.cancel_setpoint()

    def _receive(self, mid, sysid, compid, payload, now):
        if mid == 0:
            p = padded(payload,9)
            custom,kind,autopilot,base,status,version = struct.unpack("<IBBBBB",p)
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
        # Mission/ACK可能与QGC共享链路；只交给本机或广播接收者。
        recipient = None
        if mid in (40,51):
            p = padded(payload,5)
            recipient = (p[2],p[3])
        elif mid == 47:
            p = padded(payload,4)
            recipient = (p[0],p[1])
        elif mid == 77:
            p = padded(payload,10)
            recipient = (p[8],p[9])
        if recipient is not None and (recipient[0] not in (0,C.SOURCE_SYSTEM)
                                      or recipient[1] not in (0,C.SOURCE_COMPONENT)):
            return
        if mid in EVENT_MESSAGES:
            if len(self.events) >= 32:
                self.events.pop(0)
            self.events.append((mid,sysid,compid,payload,now))
        if mid == 42:
            self.mission_seq = struct.unpack("<H",padded(payload,2))[0]
        if mid == 31:
            values = struct.unpack("<I7f",padded(payload,32))
            boot,q,gyro = values[0],tuple(values[1:5]),tuple(values[5:8])
            if not finite(q+gyro) or abs(norm(q)-1.) > 0.15:
                return
            if self.boot_ms is not None:
                delta = ((boot-self.boot_ms+(1<<31))&0xffffffff)-(1<<31)
                if delta < -1000:
                    self._restart()
                elif delta <= 0:
                    return
            self.boot_ms = boot
            self._history(self.attitudes,(self._stamp(boot),unit(q),gyro,now))
        elif mid == 32:
            values = struct.unpack("<I6f",padded(payload,28))
            if finite(values[1:]):
                self._history(self.velocities,(self._stamp(values[0]),tuple(values[4:7]),now))
        elif mid == 230:
            p = padded(payload,42)
            values = struct.unpack("<8fH",p[8:42])
            flags = values[-1]
            # 姿态、水平/垂直速度有效；排除常位置假定和加速度异常。
            ok = (flags&7) == 7 and not (flags&(128|2048))
            ok = ok and math.isfinite(values[0]) and 0. <= values[0] < 1.
            self.estimator = (now,ok,flags)
        elif mid == 111:
            p = padded(payload,16)
            tc1,ts1 = unpack_i64(p),unpack_i64(p,8)
            if tc1 == 0:
                self._queue(111,pack_i64(now*1000)+pack_i64(ts1))
            elif self.pending_sync is not None and ts1 == self.pending_sync[0]:
                sent = self.pending_sync[1]
                self.pending_sync = None
                rtt = now-sent
                if not 0 <= rtt <= C.SYNC_MAX_RTT_MS*1000:
                    return
                sample = tc1//1000-(sent+now)//2
                if self.offset_us is None or abs(sample-self.offset_us) > 50000:
                    if self.offset_us is not None:
                        self.generation += 1
                    self.offset_us,self.sync_count = sample,1
                    self.attitudes,self.velocities = [],[]
                elif abs(sample-self.offset_us) <= C.SYNC_MAX_JITTER_US:
                    self.offset_us += (sample-self.offset_us)//5
                    self.sync_count += 1
                else:
                    return
                self.sync_us = now

    def attitude_at(self, local_us):
        now = self.clock.now_us()
        if not self.synced(now) or not self.attitudes:
            return None
        stamp = local_us+self.offset_us
        history = self.attitudes
        if stamp < history[0][0]:
            return None
        for i in range(1,len(history)):
            a,b = history[i-1],history[i]
            if a[0] <= stamp <= b[0]:
                if b[0]-a[0] > C.TELEMETRY_TIMEOUT_MS*1000:
                    return None
                f = (stamp-a[0])/(b[0]-a[0])
                sign = -1. if sum(a[1][j]*b[1][j] for j in range(4)) < 0 else 1.
                return unit(tuple((1-f)*a[1][j]+f*sign*b[1][j] for j in range(4)))
        a = history[-1]
        dt = (stamp-a[0])*1e-6
        if dt < 0 or dt > C.ATTITUDE_EXTRAPOLATE_MS*.001:
            return None
        half = tuple(v*dt*.5 for v in a[2])
        return unit(_qmul(a[1],(1.,half[0],half[1],half[2])))

    def velocity_at(self, local_us):
        if self.offset_us is None or not self.velocities:
            return None
        stamp = local_us+self.offset_us
        for item in reversed(self.velocities):
            if 0 <= stamp-item[0] <= C.TELEMETRY_TIMEOUT_MS*1000:
                return item[1]
        return None

    def state(self, now=None):
        now = self.clock.now_us() if now is None else now
        reason = "OK"
        if not self.connected(now):
            reason = "HEARTBEAT_STALE"
        elif not self.synced(now):
            reason = "TIME_UNSYNCED"
        elif not self.attitudes or not 0 <= now-self.attitudes[-1][3] <= C.TELEMETRY_TIMEOUT_MS*1000:
            reason = "ATTITUDE_STALE"
        elif not self.velocities or not 0 <= now-self.velocities[-1][2] <= C.TELEMETRY_TIMEOUT_MS*1000:
            reason = "VELOCITY_STALE"
        elif self.estimator is None or not 0 <= now-self.estimator[0] <= C.ESTIMATOR_TIMEOUT_MS*1000 or not self.estimator[1]:
            reason = "ESTIMATOR_UNREADY"
        if reason == "OK":
            fc_now = now+self.offset_us
            q_age = fc_now-self.attitudes[-1][0]
            v_age = fc_now-self.velocities[-1][0]
            if not -30000 <= q_age <= C.TELEMETRY_TIMEOUT_MS*1000:
                reason = "ATTITUDE_SAMPLE_STALE"
            elif not -30000 <= v_age <= C.TELEMETRY_TIMEOUT_MS*1000:
                reason = "VELOCITY_SAMPLE_STALE"
        q = self.attitudes[-1][1] if self.attitudes else None
        yaw = 0.
        if q is not None:
            r = quaternion_rotation(q)
            yaw = math.atan2(r[1][0],r[0][0])
        # ESTIMATOR_POS_HORIZ_REL(8) + ESTIMATOR_POS_VERT_ABS(32)。
        # 仅供室内故障退出请求Position；不要求全球位置标志。
        local_position_ready = (reason == "OK" and self.estimator is not None
                                and (self.estimator[2]&40) == 40)
        return {"ready":reason == "OK","reason":reason,"armed":self.armed,
                "offboard":self.offboard,"q":q,"yaw":yaw,
                "mission":self.mission,"main_mode":self.main_mode,
                "sub_mode":self.sub_mode,"mission_seq":self.mission_seq,
                "velocity":self.velocities[-1][1] if self.velocities else None,
                "generation":self.generation,"local_position_ready":local_position_ready}

    def _packet(self, mid, payload):
        packet = pack_message(mid,payload,self.sequence,C.SOURCE_SYSTEM,C.SOURCE_COMPONENT)
        self.sequence = (self.sequence+1)&255
        return packet

    def _queue(self, mid, payload):
        if len(self.queue) < 8:
            control = None
            if mid == 76 and len(payload) >= 30:
                command = struct.unpack("<H",payload[28:30])[0]
                if command == 176:
                    main_mode = struct.unpack("<f",payload[4:8])[0]
                    control = "OFFBOARD" if main_mode == 6. else "MODE"
                elif command == 400:
                    control = "ARM"
            self.queue.append([self._packet(mid,payload),0,self.clock.now_us(),False,control])
            return True
        return False

    def drain_events(self):
        events,self.events = self.events,[]
        return events

    def cancel_mission_transfer(self):
        def upload(item):
            packet = item[0]
            return (packet[7]|packet[8]<<8|packet[9]<<16) in (39,44,73)
        self.queue = [item for item in self.queue if not upload(item)]
        if self.active is not None and self.active[1] == 0 and upload(self.active):
            self.active = None

    def send_message(self, mid, payload):
        """显式启用导航后才允许外部任务管理器写入任务协议。"""
        if not C.NAV_ENABLE or not self.connected(self.clock.now_us()):
            return False
        if mid not in (39,41,43,44,45,47,73):
            return False
        return self._queue(mid,payload)

    def send_command(self, command, params):
        """仅调用者明确启用导航时发送；ACK/心跳由调用者验证。"""
        if (not C.NAV_ENABLE or not self.connected(self.clock.now_us())
                or len(params) > 7 or not finite(params)):
            return False
        params = tuple(params)+(0.,)*(7-len(params))
        payload = struct.pack("<7fHBBB",*(params+(command,self.target[0],self.target[1],0)))
        return self._queue(76,payload)

    def request_mode(self, main_mode, sub_mode=0):
        # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED；PX4 DO_SET_MODE 使用独立主/子模式。
        return self.send_command(176,(1.,float(main_mode),float(sub_mode)))

    def cancel_setpoint(self):
        self.latest_setpoint = None
        if self.active is not None and self.active[3] and self.active[1] == 0:
            self.active = None
        self.cancel_offboard_requests()

    def cancel_offboard_requests(self):
        self.queue = [item for item in self.queue if item[4] != "OFFBOARD"]
        if self.active is not None and len(self.active) > 4 and self.active[4] == "OFFBOARD" and self.active[1] == 0:
            self.active = None

    def send_velocity(self, velocity, yaw, now=None):
        now = self.clock.now_us() if now is None else now
        state = self.state(now)
        if not C.NAV_ENABLE or C.readiness_errors() or not state["ready"] or not state["armed"]:
            self.cancel_setpoint()
            return False
        if len(velocity) != 3 or not finite(tuple(velocity)+(yaw,)):
            self.cancel_setpoint()
            return False
        # LOCAL_NED，位置/加速度/yaw_rate忽略，速度+yaw有效。
        mask = 1|2|4|64|128|256|2048
        stamp = int(now+self.offset_us)//1000
        payload = struct.pack("<I11fHBBB",stamp&0xffffffff,
                    0.,0.,0.,velocity[0],velocity[1],velocity[2],
                    0.,0.,0.,yaw,0.,mask,self.target[0],self.target[1],1)
        self.latest_setpoint = [self._packet(84,payload),0,now,True,None]
        return True

    def _flush(self, now):
        if self.active is None:
            control = next((i for i,item in enumerate(self.queue) if item[4] is not None),None)
            if control is not None:
                self.active = self.queue.pop(control)
            elif self.latest_setpoint is not None:
                self.active,self.latest_setpoint = self.latest_setpoint,None
            elif self.queue:
                self.active = self.queue.pop(0)
            else:
                return
        packet,position,created,is_setpoint = self.active[:4]
        is_control = len(self.active) > 4 and self.active[4] is not None
        if position == 0 and ((is_setpoint or is_control) and (not C.NAV_ENABLE or now-created > C.MAX_TX_AGE_MS*1000)
                              or is_control and not self.connected(now)):
            self.active = None
            self.dropped += 1
            return
        sent = self.uart.write(packet[position:])
        if sent is not None and sent > 0:
            self.active[1] += sent
            if self.active[1] >= len(packet):
                self.active = None

    def poll(self, service_sync=False):
        now = self.clock.now_us()
        budget = C.RX_BYTES_PER_POLL
        while budget > 0 and self.uart.any():
            data = self.uart.read(min(self.uart.any(),budget,512))
            if not data:
                break
            budget -= len(data)
            self.last_rx_us = now
            for packet in self.parser.feed(data):
                self._receive(packet[0],packet[1],packet[2],packet[3],now)
        if self.parser.buffer and now-self.last_rx_us > 1000000:
            self.parser.buffer = bytearray()
        if not self.connected(now):
            self.cancel_setpoint()
        if self.target is not None:
            if now-self.hb_sent >= 1000000:
                self._queue(0,struct.pack("<IBBBBB",0,18,8,0,4,3))
                self.hb_sent = now
            if self.connected(now) and now-self.sync_sent >= C.SYNC_INTERVAL_MS*1000:
                token = now*1000
                self.pending_sync = (token,now)
                self._queue(111,pack_i64(0)+pack_i64(token))
                self.sync_sent = now
            if C.REQUEST_STREAMS and self.connected(now) and now-self.request_sent >= 500000 and self.stream_index < 3:
                mid,hz = ((31,50),(32,30),(230,5))[self.stream_index]
                self._queue(76,struct.pack("<7fHBBB",float(mid),1e6/hz,0.,0.,0.,0.,0.,
                                           511,self.target[0],self.target[1],0))
                self.stream_index += 1
                self.request_sent = now
        self._flush(now)
        # 首次同步短暂等待UART回复，避免snapshot阻塞令每次RTT都超限。
        if service_sync and self.pending_sync is not None and self.connected(now):
            token = self.pending_sync[0]
            deadline = self.pending_sync[1]+C.SYNC_MAX_RTT_MS*1000
            while self.pending_sync is not None and self.pending_sync[0] == token and self.clock.now_us() < deadline:
                self.poll(False)
                if hasattr(time,"sleep_ms"):
                    time.sleep_ms(1)
                else:
                    time.sleep(.001)
