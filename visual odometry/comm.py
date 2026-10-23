"""通信模块：MAVLink1/2接收、MAVLink2发送、PX4发现、时钟同步及数据缓存。

仅请求数据，不发送解锁/模式/EKF参数命令。默认不发送ODOMETRY。
无需pymavlink；只对CRC_EXTRA中的消息进行校验和解码。
"""
import struct
import time
import config as C

CRC_EXTRA = {0: 50, 31: 246, 76: 152, 77: 143,
             111: 34, 132: 85, 241: 90, 331: 91}


class Clock:
    """将MicroPython ticks回绕展开；PC使用monotonic_ns。"""
    def __init__(self):
        self.micro = hasattr(time, "ticks_us")
        self.raw = time.ticks_us() if self.micro else time.monotonic_ns()//1000
        self.total = self.raw

    def now_us(self):
        raw = time.ticks_us() if self.micro else time.monotonic_ns()//1000
        self.total += time.ticks_diff(raw, self.raw) if self.micro else raw-self.raw
        self.raw = raw
        return self.total

    def from_ticks(self, raw):
        now = self.now_us()
        return now + (time.ticks_diff(raw, self.raw) if self.micro else raw-self.raw)


def pack_i64(value):
    return struct.pack("<II", value & 0xFFFFFFFF, (value >> 32) & 0xFFFFFFFF)


def unpack_i64(data, offset=0):
    lo, hi = struct.unpack("<II", data[offset:offset+8])
    value = lo+(hi << 32)
    return value-(1 << 64) if hi & 0x80000000 else value


def payload_padded(payload, size):
    return (payload + b"\x00"*size)[:size]


def crc(data, extra):
    value = 0xFFFF
    for byte in data:
        tmp = byte ^ (value & 255)
        tmp = (tmp ^ (tmp << 4)) & 255
        value = ((value >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 65535
    tmp = extra ^ (value & 255)
    tmp = (tmp ^ (tmp << 4)) & 255
    return ((value >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 65535


def pack_message(msgid, payload, seq, sysid, compid):
    payload = payload.rstrip(b"\x00") or b"\x00"
    head = bytes((len(payload), 0, 0, seq & 255, sysid, compid,
                  msgid & 255, (msgid >> 8) & 255, (msgid >> 16) & 255))
    return b"\xfd"+head+payload+struct.pack("<H", crc(head+payload, CRC_EXTRA[msgid]))


class Parser:
    def __init__(self):
        self.buffer = bytearray()
        self.bad_crc = 0
        self.skipped = 0
        self.signed_rejected = 0

    def feed(self, data):
        self.buffer.extend(data)
        packets = []
        buffer = self.buffer
        start, size = 0, len(buffer)
        # 先移动游标，最后只复制一次余量；兼容不支持bytearray删除的固件。
        while start < size:
            if buffer[start] not in (0xFD, 0xFE):
                start += 1
                continue
            if size-start < 2:
                break
            v2 = buffer[start] == 0xFD
            head = 10 if v2 else 6
            if size-start < head:
                break
            if v2 and buffer[start+2] & 0xFE:
                start += 1
                continue
            signed = v2 and (buffer[start+2] & 1)
            payload = start+head
            end = payload+buffer[start+1]
            next_start = end+2+(13 if signed else 0)
            if size < next_start:
                break
            if v2:
                sysid, compid = buffer[start+5], buffer[start+6]
                msgid = buffer[start+7] | buffer[start+8] << 8 | buffer[start+9] << 16
            else:
                sysid, compid, msgid = buffer[start+3], buffer[start+4], buffer[start+5]
            # 不宣称已认证签名帧；需签名的链路应接入完整签名实现。
            if signed:
                self.signed_rejected += 1
                start = next_start
                continue
            if msgid not in CRC_EXTRA:
                self.skipped += 1
                start = next_start
                continue
            checksum = buffer[end] | buffer[end+1] << 8
            if crc(buffer[start+1:end], CRC_EXTRA[msgid]) != checksum:
                self.bad_crc += 1
                start += 1
                continue
            packets.append((msgid, sysid, compid, bytes(buffer[payload:end])))
            start = next_start
        if start:
            self.buffer = buffer[start:]
        # 噪声不会导致无限增长；带签名的MAVLink2帧最大280字节。
        if len(self.buffer) > 4096:
            self.buffer = bytearray()
        return packets


def odometry_payload(state):
    """common.xml ODOMETRY: 233字节，XY-only，NaN姿态/速度，有限z。"""
    nan = float("nan")
    pose = [0.0]*21
    pose[0], pose[6], pose[11] = state["variance"][0], state["variance"][1], C.Z_PLACEHOLDER_VARIANCE
    pose[15] = pose[18] = pose[20] = nan
    values = (state["position"][0], state["position"][1], 0.0)+(nan,)*10
    return (pack_i64(int(state["timestamp_us"])) + struct.pack("<13f", *values)
            + struct.pack("<21f", *pose) + struct.pack("<21f", *((nan,)*21))
            + struct.pack("<BBBBb", 1, 12, state["reset_counter"] & 255, 2,
                          max(1, min(100, int(state["quality"])))))


class FlightLink:
    def __init__(self, uart=None, clock=None):
        self.clock = clock or Clock()
        if uart is None:
            from pyb import UART
            uart = UART(C.UART_ID, C.BAUDRATE, bits=8, parity=None, stop=1,
                        timeout=0, timeout_char=0, read_buf_len=4096, flow=0)
        self.uart = uart
        self.parser = Parser()
        self.target = None
        self.sequence = 0
        self.attitudes = []         # (飞控us, q_wxyz, gyro_b, 本地接收us)
        self.ranges = []            # (飞控us, m, min, max, quality, variance, receive_us)
        self.vibration = None       # (本地us, 三项PX4指标, clipping0/1/2)
        self.boot_generation = 0
        self.time_generation = 0
        self.last_boot_ms = None
        self.heartbeat_us = -10000000
        self.last_rx_us = 0
        self.last_hb_tx = -10000000
        self.last_sync_tx = -10000000
        self.offset_us = None       # 飞控时钟 - OpenMV时钟
        self.sync_count = 0
        self.sync_us = 0
        self.pending_sync = None
        self.last_sync_rtt_us = None
        self.sync_rtt_rejected = 0
        self.serviced_sync_token = None
        self.tx_queue = []
        self.tx_active = None        # [packet, position, created_us, is_odometry]
        self.odom_latest = None
        self.tx_dropped = 0
        self.last_ack = None
        self.range_id = None
        self.range_candidates = set()
        self.range_ambiguous = False
        self.requests = []
        self.pending_command = None
        self.desired_streams = []
        self.request_after_us = 0
        self.next_stream_check_us = 0

    def connected(self, now=None):
        now = self.clock.now_us() if now is None else now
        return self.target is not None and now-self.heartbeat_us <= C.HEARTBEAT_TIMEOUT_US

    def synced(self, now=None):
        now = self.clock.now_us() if now is None else now
        return (self.offset_us is not None and self.sync_count >= C.SYNC_MIN_SAMPLES
                and now-self.sync_us <= C.SYNC_TIMEOUT_US)

    def fc_time(self, local_us):
        return None if self.offset_us is None else int(local_us+self.offset_us)

    def _stamp(self, boot_ms):
        # 32位毫秒时间戳展开到TIMESYNC的64位飞控时间附近。
        if self.offset_us is None:
            return boot_ms*1000
        reference = self.clock.now_us()+self.offset_us
        wrap = (1 << 32)*1000
        value = boot_ms*1000
        return int(value+round((reference-value)/wrap)*wrap)

    def _queue(self, msgid, payload, is_odometry=False, created=None):
        if self.target is None:
            return
        packet = pack_message(msgid, payload, self.sequence, self.target[0], C.SOURCE_COMPONENT)
        self.sequence = (self.sequence+1) & 255
        now = self.clock.now_us() if created is None else created
        item = [packet, 0, now, is_odometry]
        if is_odometry:
            self.odom_latest = item  # 未开始发送的旧观测可被最新观测替代
        elif len(self.tx_queue) < 8:
            self.tx_queue.append(item)
        else:
            self.tx_dropped += 1

    def _flush(self, now):
        if self.tx_active is None:
            if self.tx_queue:
                self.tx_active = self.tx_queue.pop(0)
            elif self.odom_latest is not None:
                self.tx_active, self.odom_latest = self.odom_latest, None
                if now-self.tx_active[2] > C.MAX_TX_AGE_US:
                    self.tx_dropped += 1
                    self.tx_active = None
                    return
            else:
                return
        packet, position, _, _ = self.tx_active
        sent = self.uart.write(packet[position:])
        if sent:
            self.tx_active[1] += sent
            if self.tx_active[1] == len(packet):
                self.tx_active = None

    def cancel_odometry(self):
        self.odom_latest = None
        if self.tx_active is not None and self.tx_active[3] and self.tx_active[1] == 0:
            self.tx_active = None
        # 已开始的帧必须发完，不能截断字节流；其时间戳仍是原始采集时刻。

    def send_odometry(self, state):
        self._queue(331, odometry_payload(state), True, state["timestamp_us"])

    def _history(self, target, item):
        if target and item[0] <= target[-1][0]:
            return
        target.append(item)
        if len(target) > C.SENSOR_HISTORY_SIZE:
            del target[0]

    def _restart(self):
        self.boot_generation += 1
        self.offset_us = None
        self.sync_count = 0
        self.pending_sync = None
        self.attitudes = []
        self.ranges = []
        self.vibration = None
        self.range_candidates = set()
        self.range_id = None
        self.range_ambiguous = False
        self.cancel_odometry()
        self.tx_queue = []
        self._make_requests()

    def _make_requests(self):
        attitude_hz = C.ATTITUDE_HZ if C.ATTITUDE_HZ is not None else min(100, max(30, 1.5*C.TARGET_FPS))
        range_hz = C.RANGE_HZ if C.RANGE_HZ is not None else min(50, max(10, C.TARGET_FPS))
        self.desired_streams = [(31, int(1000000/attitude_hz)),
                                (132, int(1000000/range_hz))]
        if C.REQUEST_VIBRATION:
            self.desired_streams.append((241, int(1000000/C.VIBRATION_HZ)))
        self.requests = [] if C.STREAM_POLICY == "listen" else list(self.desired_streams)
        self.pending_command = None
        now = self.clock.now_us()
        self.request_after_us = now+(800000 if C.STREAM_POLICY == "auto" else 0)
        self.next_stream_check_us = now+5000000

    def _have_stream(self, msgid, interval, now):
        if msgid == 241:
            return self.vibration is not None and now-self.vibration[0] < 1000000
        history = self.attitudes if msgid == 31 else self.ranges
        if len(history) < 3:
            return False
        span = history[-1][0]-history[0][0]
        return (span > 0 and now-history[-1][-1] < max(250000, 3*interval)
                and (len(history)-1)*1000000.0/span >= .7*1000000.0/interval)

    def _receive(self, msgid, sysid, compid, payload, now):
        if msgid == 0:
            hb = payload_padded(payload, 9)
            if hb[5] != 12 or compid != C.TARGET_COMPONENT:
                return
            if C.TARGET_SYSTEM is not None and sysid != C.TARGET_SYSTEM:
                return
            if self.target is None:
                self.target = (sysid, compid)
                self._make_requests()
            if (sysid, compid) == self.target:
                self.heartbeat_us = now
        if self.target != (sysid, compid):
            return
        if msgid == 31:
            values = struct.unpack("<Ifffffff", payload_padded(payload, 32))
            boot = values[0]
            if self.last_boot_ms is not None:
                delta = ((boot-self.last_boot_ms+(1 << 31)) & 0xFFFFFFFF)-(1 << 31)
                if delta < -1000:
                    self._restart()
                elif delta <= 0:
                    return
            self.last_boot_ms = boot
            self._history(self.attitudes, (self._stamp(boot)+C.ATTITUDE_TIME_OFFSET_US,
                          tuple(values[1:5]), tuple(values[5:8]), now))
        elif msgid == 132:
            p = payload_padded(payload, 39)
            stamp, minimum, maximum, distance, kind, sid, orient, variance = struct.unpack("<IHHHBBBB", p[:14])
            if orient != C.TOF_ORIENTATION:
                return
            if C.TOF_SENSOR_ID is not None:
                if sid != C.TOF_SENSOR_ID:
                    return
                self.range_id = sid
            else:
                self.range_candidates.add(sid)
                self.range_ambiguous = len(self.range_candidates) > 1
                if self.range_ambiguous:
                    self.ranges = []
                    return
                self.range_id = sid
            quality = p[38] if len(payload) >= 39 else 0
            self._history(self.ranges, (self._stamp(stamp)+C.RANGE_TIME_OFFSET_US,
                          distance*0.01, minimum*0.01, maximum*0.01,
                          quality, None if variance == 255 else variance*0.0001, now))
        elif msgid == 111:
            p = payload_padded(payload, 16)
            tc1, ts1 = unpack_i64(p), unpack_i64(p, 8)
            if tc1 == 0:
                self._queue(111, pack_i64(now*1000)+pack_i64(ts1))
            elif self.pending_sync is not None and ts1 == self.pending_sync[0]:
                sent = self.pending_sync[1]
                self.pending_sync = None
                rtt = now-sent
                self.last_sync_rtt_us = rtt
                if rtt < 0 or rtt > C.SYNC_MAX_RTT_US:
                    self.sync_rtt_rejected += 1
                else:
                    sample = tc1/1000.0-(sent+now)*0.5
                    if self.offset_us is None or abs(sample-self.offset_us) > 50000:
                        if self.offset_us is not None:
                            self.time_generation += 1
                        self.offset_us, self.sync_count = sample, 1
                        self.attitudes = []
                        self.ranges = []
                    elif abs(sample-self.offset_us) <= C.SYNC_MAX_JITTER_US:
                        self.offset_us += 0.2*(sample-self.offset_us)
                        self.sync_count += 1
                    else:
                        return
                    self.sync_us = now
        elif msgid == 77:
            p = payload_padded(payload, 10)
            command = struct.unpack("<H", p[:2])[0]
            result = p[2]
            addressed = p[8] in (0, self.target[0]) and p[9] in (0, C.SOURCE_COMPONENT)
            if addressed and command == 511 and self.pending_command is not None:
                self.last_ack = (self.pending_command[0], result)
                if result != 5:
                    self.pending_command = None
        elif msgid == 241:
            p = payload_padded(payload, 32)
            values = struct.unpack("<fffIII", p[8:32])
            self.vibration = (now, tuple(values[:3]), tuple(values[3:]))

    def _service_sync(self):
        # snapshot/特征计算会阻塞主循环。每个同步请求只短暂等待一次，
        # 避免已到UART的回复等到下一帧才处理，造成虚假的长RTT。
        pending = self.pending_sync
        if pending is None or pending[0] == self.serviced_sync_token:
            return
        self.serviced_sync_token = pending[0]
        deadline = pending[1]+C.SYNC_MAX_RTT_US
        while self.pending_sync is not None and self.clock.now_us() < deadline:
            self.poll()  # 不递归进入_service_sync
            if self.pending_sync is None:
                break
            if hasattr(time, "sleep_ms"):
                time.sleep_ms(1)
            else:
                time.sleep(0.001)

    def poll(self, service_sync=False):
        now = self.clock.now_us()
        budget = C.RX_BYTES_PER_POLL
        while budget > 0:
            available = self.uart.any()
            if not available:
                break
            data = self.uart.read(min(available, budget, 512))
            if not data:
                break
            budget -= len(data)
            self.last_rx_us = now
            for packet in self.parser.feed(data):
                self._receive(packet[0], packet[1], packet[2], packet[3], now)
        if self.parser.buffer and now-self.last_rx_us > 1000000:
            self.parser.buffer = bytearray()
        if self.target is None:
            return
        if now-self.last_hb_tx >= 1000000:
            self._queue(0, struct.pack("<IBBBBB", 0, 18, 8, 0, 4, 3))
            self.last_hb_tx = now
        if now-self.last_sync_tx >= C.SYNC_INTERVAL_US:
            ts = now*1000
            self.pending_sync = (ts, now)
            self._queue(111, pack_i64(0)+pack_i64(ts))
            self.last_sync_tx = now
        if self.connected(now):
            if (C.STREAM_POLICY == "auto" and now >= self.next_stream_check_us
                    and self.pending_command is None and not self.requests):
                self.requests = [(mid, interval) for mid, interval in self.desired_streams
                                 if not self._have_stream(mid, interval, now)]
                self.next_stream_check_us = now+5000000
            if self.pending_command is None and now >= self.request_after_us:
                while self.requests:
                    msgid, interval = self.requests.pop(0)
                    if C.STREAM_POLICY == "auto" and self._have_stream(msgid, interval, now):
                        continue
                    self.pending_command = [msgid, interval, 0, -10000000]
                    break
            p = self.pending_command
            if p is not None and now-p[3] >= C.COMMAND_TIMEOUT_US:
                if p[2] >= 3:
                    self.last_ack = (p[0], "TIMEOUT")
                    self.pending_command = None
                else:
                    body = struct.pack("<fffffffHBBB", float(p[0]), float(p[1]),
                        0.0, 0.0, 0.0, 0.0, 0.0, 511, self.target[0], self.target[1], p[2])
                    self._queue(76, body)
                    p[2] += 1
                    p[3] = now
        else:
            self.cancel_odometry()
        self._flush(now)
        if service_sync:
            self._service_sync()
