"""MODE1轻量观察及总结；MODE2退出后保存汇总和限量逐帧诊断。"""
MODE = 1
DEBUG_VERSION = 'sparse-vo-4'
RATE_WINDOW_US = 2000000
EVENT_DISPLAY_INTERVAL_US = 1000000  # 状态变化摘要的最小间隔；不定时刷新，q立即查询
LOG_DIRECTORY = None
EVENT_BUFFER_SIZE = 16
TRACE_CAPACITY = 512
TRACE_RECORD_BYTES = 64

import gc
import math
import os
import struct
import sys
from array import array
from algorithm import _us, _elapsed, euler_degrees
from output import BLOCK_CODES, publication_block_mask, PublicationGate
from main import run
import config as C

PHASES = (('capture', '采图/相机调节'), ('communication', '通信/输入等待'),
          ('vision', '视觉总处理'), ('gc', '显式GC'), ('debug', '调试统计/输入输出'),
          ('cycle', '完整循环'),
          ('native_warp', '  补偿/窗口重采样'), ('native_klt', '  KLT正反向'),
          ('native_sad', '  SAD搜索/歧义检查'), ('native_detect', '  FAST补点'),
          ('native_fit', '  平移拟合'), ('native_copy', '  参考图复制'))
INDEX = {name: i for i, (name, _) in enumerate(PHASES)}
DETAIL_NAMES = ('匹配数', '内点数', '像素残差')
BLOCK_NAMES = ('时序未确认', '位置无效', '外参未确认或机体位置无效', '置信度不足',
               '飞控未连接', '时钟未同步', '观测过期或时间在未来', '非新视觉测量')
FAILURES = (
    ('NO_FEATURES', '特征不足', 0), ('LOST', '视觉跟踪失败，原因未细分', 0),
    ('MATCH_COUNT', '可靠匹配不足', 0), ('INLIER_RATIO', '一致匹配比例不足', 0),
    ('AMBIGUOUS_MATCH', '少点匹配有歧义', 0),
    ('INPUT_INVALID', '视觉输入无效', 1), ('LOW_TEXTURE', '图像纹理不足', 0), ('DYNAMIC_LIMIT', '动态限制', 0),
    ('VIBRATION', '振动超限', 0), ('WAIT_HEARTBEAT', '等待飞控连接', 1),
    ('TIME_UNSYNCED', '时间未同步', 1), ('WAIT_ATTITUDE', '图像时刻缺少姿态', 1),
    ('EXTRINSICS_PENDING', '安装参数未确认', 1), ('WAIT_RANGE', '等待测距', 1),
    ('WAIT_RANGE_ATTITUDE', '测距时刻缺少姿态', 1),
    ('RANGE_ATTITUDE_GAP', '测距缺少对应姿态', 1), ('RANGE_INVALID', '测距无效', 1),
    ('RANGE_TILT', '测距倾角或地面交点无效', 1), ('RANGE_JUMP', '测距突变', 1),
    ('RANGE_STALE', '测距过期', 1), ('SELECT_TOF_ID', '测距器ID不明确', 1),
    ('ATTITUDE_RESET', '飞控姿态跳变', 1), ('OTHER', '其他未分类失效', 2))
FAILURE_INDEX = {item[0]: i for i, item in enumerate(FAILURES)}
INITIAL_STATES = ('INITIALIZING', 'REINITIALIZING')
RECOVERY_STATES = ('POSE_RECOVERING', 'RANGE_RECOVERING')
WARMUP_STATES = INITIAL_STATES+RECOVERY_STATES
DISPLAY_STATES = ('初始化', '跟踪正常', '位置无效', '少点跟踪', '短时预测')
EVENT_NAMES = ('连接变化', '同步变化', '进入失跟', '恢复跟踪', '轨迹重置', '测距异常', '程序异常')
TRACE_REASON_NAMES = ('TRACKING', 'SPARSE_TRACKING', 'PREDICTING',
    'POSE_RECOVERING', 'INITIALIZING', 'REINITIALIZING') + tuple(item[0] for item in FAILURES)
TRACE_REASON_INDEX = {name: i for i, name in enumerate(TRACE_REASON_NAMES)}
VISION_REASON_NAMES = ('OK', 'MATCH_COUNT', 'INLIER_RATIO', 'AMBIGUOUS_MATCH',
                       'INPUT_INVALID', 'INITIALIZING', 'TRACK_FAILED')
VISION_REASON_INDEX = {name: i for i, name in enumerate(VISION_REASON_NAMES)}
TRACE_HEADER_FORMAT = '<IIffhhhhHhHfIHH'
TRACE_BYTES_FORMAT = '<22B'
assert struct.calcsize(TRACE_HEADER_FORMAT) + struct.calcsize(TRACE_BYTES_FORMAT) == TRACE_RECORD_BYTES


def exception_text(exception):
    try:
        from io import StringIO
        buffer = StringIO()
        if hasattr(sys, 'print_exception'):
            sys.print_exception(exception, buffer)
        else:
            import traceback
            traceback.print_exception(type(exception), exception, exception.__traceback__, file=buffer)
        return buffer.getvalue()
    except Exception:
        return repr(exception)


class Console:
    """复用USB接收缓冲；只使用原始终端，不读取IDE调试协议。"""
    def __init__(self):
        self.vcp = self.poller = None
        self.rx_buffer = bytearray(64)
        self.rx_view = memoryview(self.rx_buffer)
        self.after_cr = False
        try:
            from pyb import USB_VCP
            self.vcp = USB_VCP()
            if self.vcp.debug_mode_enabled():
                raise RuntimeError('请断开IDE主调试连接，通过原始USB串口终端运行test.main()')
        except ImportError:
            import select
            self.poller = select.poll()
            self.poller.register(sys.stdin, select.POLLIN)

    def read(self):
        if self.vcp is not None:
            if not self.vcp.any():
                return b''
            count = self.vcp.recv(self.rx_buffer, timeout=0)
            return self.rx_view[:count] if count else b''
        if not self.poller.poll(0):
            return b''
        return sys.stdin.read(1).encode()

    def commands(self, data):
        if not data:
            return ()
        actions = []
        for value in data:
            if value == 10 and self.after_cr:
                self.after_cr = False
                continue
            self.after_cr = value == 13
            if value in (10, 13):
                actions.append('origin')
            elif value in (ord('q'), ord('Q')):
                actions.append('query')
            elif value in (ord('x'), ord('X')):
                actions.append('exit')
        return actions

    def poll(self):
        return self.commands(self.read())


class EventBuffer:
    """固定数值环：每条17字节载荷，不缓存文本，相邻重复事件合并。"""
    def __init__(self, capacity=EVENT_BUFFER_SIZE):
        if capacity < 1:
            raise ValueError('event capacity must be positive')
        self.capacity = capacity
        self.low = array('I', [0]*capacity)
        self.high = array('I', [0]*capacity)
        self.codes = bytearray(capacity)
        self.arguments = array('I', [0]*capacity)
        self.repeats = array('I', [0]*capacity)
        self.start = self.count = self.overwritten = self.total = 0

    def add(self, offset_ms, code, argument=0):
        self.total += 1
        argument = max(0, min(0xFFFFFFFF, int(argument)))
        if self.count:
            last = (self.start+self.count-1) % self.capacity
            if self.codes[last] == code and self.arguments[last] == argument:
                self.repeats[last] = min(0xFFFFFFFF, self.repeats[last]+1)
                return
        if self.count < self.capacity:
            slot = (self.start+self.count) % self.capacity
            self.count += 1
        else:
            slot = self.start
            self.start = (self.start+1) % self.capacity
            self.overwritten += 1
        offset_ms = max(0, int(offset_ms))
        self.low[slot], self.high[slot] = offset_ms & 0xFFFFFFFF, offset_ms >> 32
        self.codes[slot], self.arguments[slot], self.repeats[slot] = code, argument, 1

    def lines(self):
        for i in range(self.count):
            slot = (self.start+i) % self.capacity
            stamp = self.low[slot]+(self.high[slot] << 32)
            code, argument = self.codes[slot], self.arguments[slot]
            if code in (0, 1):
                detail = '恢复' if argument else '失效'
            elif code in (2, 5):
                detail = FAILURES[argument][1]
            elif code == 3:
                detail = '失跟持续%dms' % argument
            elif code == 4:
                detail = 'reset_counter=%d' % argument
            else:
                detail = '见结束原因'
            yield '[%.3fs] %s：%s；重复=%d\n' % (stamp*.001, EVENT_NAMES[code], detail, self.repeats[slot])


def trace_u8(value):
    return 255 if value is None else max(0, min(254, int(value)))


def trace_u16(value):
    return 65535 if value is None else max(0, min(65534, int(value)))


def trace_i16(value, scale=1):
    return -32768 if value is None or not math.isfinite(value) else max(-32767, min(32767, int(value*scale+(.5 if value >= 0 else -.5))))


def trace_float(value):
    return float('nan') if value is None or not math.isfinite(value) else float(value)


def trace_cell(value):
    return '' if value == 255 else str(value)


def trace_name(values, code):
    return values[code] if code < len(values) else ''


def trace_scaled(value, missing, divisor, digits):
    return '' if value == missing else ('%.'+str(digits)+'f') % (value/divisor)


def trace_decimal(value, digits=4):
    return '' if not math.isfinite(value) else ('%.'+str(digits)+'f') % value


class FrameTrace:
    """Fixed 64-byte records; writes strings only after a MODE2 run ends."""
    def __init__(self, capacity=TRACE_CAPACITY):
        if capacity < 1:
            raise ValueError('trace capacity must be positive')
        self.capacity = capacity
        self.buffer = bytearray(capacity*TRACE_RECORD_BYTES)
        self.start = self.count = self.total = self.overwritten = 0

    def reserve(self):
        self.total += 1
        if self.count < self.capacity:
            slot = (self.start+self.count) % self.capacity
            self.count += 1
        else:
            slot = self.start
            self.start = (self.start+1) % self.capacity
            self.overwritten += 1
        return slot*TRACE_RECORD_BYTES

    def rows(self):
        for i in range(self.count):
            slot = (self.start+i) % self.capacity
            offset = slot*TRACE_RECORD_BYTES
            yield (struct.unpack_from(TRACE_HEADER_FORMAT, self.buffer, offset) +
                   struct.unpack_from(TRACE_BYTES_FORMAT, self.buffer, offset+42))


def is_directory(path):
    try:
        return bool(os.stat(path)[0] & 0x4000)
    except OSError:
        return False


class SessionLog:
    """只在退出后创建，逐段写报告，不拼接整份文本。"""
    def __init__(self, directory=None):
        self.error = None
        if directory is None:
            directory = '/sd/logs' if is_directory('/sd') else '/flash/logs' if is_directory('/flash') else 'logs'
        if not is_directory(directory):
            os.mkdir(directory)
        index = 1
        while True:
            self.path = '%s/vo_test_%04d.txt' % (directory, index)
            try:
                os.stat(self.path)
            except OSError:
                break
            index += 1
        self.file = open(self.path, 'wb')

    def write(self, text):
        if self.error is None:
            try:
                self.file.write(text.encode('utf-8'))
            except OSError as error:
                self.error = repr(error)

    def close(self):
        try:
            self.file.close()
        except OSError as error:
            self.error = repr(error)


def number(value, digits=2):
    return '--' if value is None else ('%.'+str(digits)+'f') % value


class TestSession:
    def __init__(self, mode=MODE, console=None, log_directory=None):
        if mode not in (1, 2):
            raise ValueError('MODE must be 1 or 2')
        self.mode, self.timing = mode, mode == 2
        self.console, self.log_directory = console, log_directory
        self.link = self.log = None
        self.closed = self.active = False
        self.start = self.end = None
        self.frames = self.valid_frames = self.publish_frames = 0
        self.quality_sum = self.quality_count = 0
        self.initial_frames = self.recovery_frames = self.no_new_loops = 0
        self.last_stamp = self.last_sequence = None
        self.last_result = self.epoch = None
        self.gate = PublicationGate()
        self.failure_counts = array('I', [0]*len(FAILURES))
        self.loss_counts = array('I', [0]*len(FAILURES))
        self.had_tracking = self.in_loss = False
        self.loss_start = None
        self.lost_episodes = self.reset_events = 0
        self.origin = None
        self.origin_id = 0
        self.pending_origin = False
        self.rate_start = None
        self.rate_counts = array('I', [0, 0, 0])
        self.rates = [0.0, 0.0, 0.0]  # 处理Hz、可发布Hz、有效率；无有效定位Hz。
        self.rate_ready = False
        self.last_display = -EVENT_DISPLAY_INTERVAL_US
        self.display_state = self.health = None
        self.pending_display = False
        self.last_range_failure = None
        self.events = EventBuffer() if self.timing else None
        self.block_counts = array('I', [0]*len(BLOCK_CODES)) if self.timing else None
        self.values = array('I', [0]*len(PHASES)) if self.timing else None
        self.phase_counts = array('I', [0]*len(PHASES)) if self.timing else None
        # Python整数累计微秒，避免uint32约71分钟后溢出。
        self.phase_totals = [0]*len(PHASES) if self.timing else None
        self.phase_maximum = array('I', [0]*len(PHASES)) if self.timing else None
        self.seen = 0
        self.detail_counts = array('I', [0, 0, 0]) if self.timing else None
        self.detail_sums = [0.0, 0.0, 0.0] if self.timing else None
        self.detail_minimum = [0.0, 0.0, 0.0] if self.timing else None
        self.detail_maximum = [0.0, 0.0, 0.0] if self.timing else None
        self.minimum_heap = None
        self.last_heap_sample = -1000000
        self.sparse_frames = 0
        self.predicted_frames = 0
        self.trace = FrameTrace() if self.timing else None
        self.image_metrics = array('f', [0.0]*5) if self.timing else None
        self.image_metrics_method = None
        self.image_metrics_valid = False
        self.frame_exposure_us = self.frame_gain_db = None

    def prepare(self, sensor, link, controller, capture, adapter, output):
        self.link = link
        output.publish = output.display = False
        if self.console is None:
            self.console = Console()
        if self.timing:
            self.image_metrics_method = getattr(controller.engine.core, 'image_metrics', None)
            self.rx_start = (getattr(link, 'attitude_received', 0), getattr(link, 'range_received', 0),
                             link.parser.bad_crc, link.sync_rtt_rejected)
            self.start_config = (C.IMAGE_WIDTH, C.IMAGE_HEIGHT, C.MAX_KEYPOINTS,
                C.FAST_THRESHOLD, C.KLT_MAX_ITERATIONS, C.EXPOSURE_US, C.GAIN_DB,
                C.MIN_PUBLISH_QUALITY, C.ODOMETRY_HZ, C.SENSOR_TIMING_CONFIRMED,
                C.CAMERA_ORIENTATION_CONFIRMED,C.CONFIDENCE_SIGMA0_M,C.TRACK_HOLD_US,C.LOST_RESET_US)
        print('MODE%d | %s | Enter=更新原点 q=查询 x=结束 | 测试不发送ODOMETRY' % (self.mode, DEBUG_VERSION))
        print('ΔN/ΔE沿飞控NED轴；有效定位时按Enter建立参考。退出时输出总结。')
        if self.timing:
            print('MODE2只在退出后保存日志；断电/硬复位会丢失未保存统计。')

    def begin_frame(self):
        now = self.link.clock.now_us()
        if self.start is None:
            self.start = self.end = self.rate_start = now
        if self.timing:
            self.frame_exposure_us = self.frame_gain_db = None
            self.image_metrics_valid = False
            for i in range(len(PHASES)):
                self.values[i] = 0
            self.seen = 0
            self.frame_start = self.cursor = _us()
            self.active = True

    def observe_image(self, image, sensor):
        """Read camera state before the adapter changes settings for a later frame."""
        if not self.timing:
            return
        try:
            self.frame_exposure_us = sensor.get_exposure_us()
            self.frame_gain_db = sensor.get_gain_db()
        except (AttributeError, OSError):
            self.frame_exposure_us = self.frame_gain_db = None
        if self.image_metrics_method is not None:
            self.image_metrics_method(image, self.image_metrics)
            self.image_metrics_valid = True

    def record(self, name, duration):
        if self.timing and self.active:
            i = INDEX[name]
            self.values[i] += max(0, int(duration))
            self.seen |= 1 << i

    def mark(self, name):
        if self.timing:
            self.record(name, _elapsed(self.cursor))
            self.cursor = _us()

    def native(self, timing):
        # C phases are disjoint children of vision, not additional loop cost.
        names = ('native_warp', 'native_klt', 'native_sad', 'native_detect', 'native_fit', 'native_copy')
        for i in range(6):
            if timing[i]:
                self.record(names[i], timing[i])

    def event(self, now, code, argument=0):
        if self.events is not None:
            self.events.add((now-self.start)//1000, code, argument)

    def observe_health(self, now):
        if not self.timing:
            return
        health = int(self.link.connected(now)) | (int(self.link.synced(now)) << 1)
        if self.health is not None:
            changed = self.health ^ health
            if changed & 1:
                self.event(now, 0, health & 1)
            if changed & 2:
                self.event(now, 1, (health >> 1) & 1)
        self.health = health

    def add_detail(self, i, value):
        if value is None or not math.isfinite(value):
            return
        if not self.detail_counts[i]:
            self.detail_minimum[i] = self.detail_maximum[i] = value
        else:
            self.detail_minimum[i] = min(self.detail_minimum[i], value)
            self.detail_maximum[i] = max(self.detail_maximum[i], value)
        self.detail_counts[i] += 1
        self.detail_sums[i] += value

    def observe(self, result, now):
        self.observe_health(now)
        if result is None:
            self.no_new_loops += 1
            return False
        stamp, sequence = result['timestamp_us'], result.get('frame_sequence')
        if ((self.last_stamp is not None and stamp <= self.last_stamp) or
                (sequence is not None and self.last_sequence is not None and sequence == self.last_sequence)):
            self.no_new_loops += 1
            return False
        self.last_stamp, self.last_sequence, self.last_result = stamp, sequence, result
        epoch = result['reset_counter']
        if self.epoch is not None and epoch != self.epoch:
            self.reset_events += 1
            self.origin = None
            self.pending_origin = False
            self.event(now, 4, epoch)
            print('轨迹重置：旧原点已清除，请重新按Enter。')
        self.epoch = epoch
        self.frames += 1
        self.rate_counts[0] += 1
        valid, reason = result['valid'], result['reason']
        predicted = valid and result.get('predicted', False)
        measured = valid and result.get('measured', False) and not predicted
        if measured:
            self.valid_frames += 1
            self.sparse_frames += int(result.get('sparse', False))
            self.rate_counts[1] += 1
            quality = result.get('quality')
            if quality is not None and math.isfinite(quality) and 0 <= quality <= 100:
                self.quality_sum += quality
                self.quality_count += 1
            if self.in_loss:
                self.event(now, 3, (now-self.loss_start)//1000)
                self.in_loss = False
            self.had_tracking = True
        elif predicted:
            self.predicted_frames += 1
        else:
            if self.had_tracking and not self.in_loss:
                self.in_loss, self.loss_start = True, now
                self.lost_episodes += 1
                self.event(now, 2, FAILURE_INDEX.get(reason, len(FAILURES)-1))
            if reason in INITIAL_STATES:
                self.initial_frames += 1
            elif reason in RECOVERY_STATES:
                self.recovery_frames += 1
            else:
                failure = result.get('failure_reason', reason)
                i = FAILURE_INDEX.get(failure, len(FAILURES)-1)
                self.failure_counts[i] += 1
                if self.in_loss:
                    self.loss_counts[i] += 1
                if self.timing and FAILURES[i][2] == 1 and reason.startswith('RANGE_') and i != self.last_range_failure:
                    self.event(now, 5, i)
                self.last_range_failure = i
        if measured:
            self.last_range_failure = None
        mask = publication_block_mask(result, self.link, now)
        if self.gate.accept(result, now, mask):
            self.publish_frames += 1
            self.rate_counts[2] += 1
        if self.timing:
            for i in range(len(BLOCK_CODES)):
                if mask & (1 << i):
                    self.block_counts[i] += 1
            if result.get('vision') is not None:
                self.add_detail(0, result.get('matches'))
                self.add_detail(1, result.get('inliers'))
                self.add_detail(2, result.get('residual_px'))
        return True

    def record_trace(self, result):
        if self.trace is None or result is None:
            return
        vision = result.get('vision')
        has_vision = vision is not None
        ref_count = vision.get('ref_count') if has_vision else None
        ref_kind = trace_u8(vision.get('base')) if ref_count else 255
        flags = (2 if result.get('approximate') else 0)
        position = result.get('position')
        if position is None:
            north = east = float('nan')
        elif self.origin is not None:
            north, east = position[0]-self.origin[0], position[1]-self.origin[1]
            flags |= 1  # N/E values are relative to the displayed origin.
        else:
            north, east = position
        q = result.get('q')
        try:
            roll, pitch, yaw = euler_degrees(q) if q is not None else (None, None, None)
        except (ValueError, TypeError):
            roll = pitch = yaw = None
        exposure = trace_u16(self.frame_exposure_us)
        gain = trace_i16(self.frame_gain_db, 100)
        if exposure != 65535:
            flags |= 4
        if gain != -32768:
            flags |= 8
        overexposure = 65535
        overexposure_mask = 0
        gradient = float('nan')
        gradient_cells = 255
        if self.image_metrics_valid:
            m = self.image_metrics
            overexposure = trace_u16(max(0.0, min(1.0, m[0]))*1000+0.5)
            overexposure_mask = int(m[1]) & 0xFFFFFF
            gradient = trace_float(m[2])
            gradient_cells = trace_u8(m[3])
            flags |= 16
        if result.get('predicted') and result.get('valid'):
            state = 2
        elif result.get('measured') and result.get('valid'):
            state = 1
        elif result.get('measured'):
            state = 3  # Native measurement awaiting pose confirmation.
        else:
            state = 0
        failure = result.get('failure_reason')
        quality = trace_u8(result.get('quality'))
        offset = self.trace.reserve()
        struct.pack_into(TRACE_HEADER_FORMAT, self.trace.buffer, offset,
            max(0, min(0xFFFFFFFF, (result['timestamp_us']-self.start)//1000)),
            max(0, min(0xFFFFFFFF, result.get('frame_sequence') or 0)),
            north, east, trace_i16(roll, 100), trace_i16(pitch, 100),
            trace_i16(yaw, 100), trace_i16(result.get('height'), 1000),
            exposure, gain, overexposure, gradient, overexposure_mask,
            trace_u16(vision.get('fast_candidates') if has_vision else None),
            trace_u16(vision.get('weak_candidates') if has_vision else None))
        struct.pack_into(TRACE_BYTES_FORMAT, self.trace.buffer, offset+42,
            trace_u8(result.get('reset_counter')), trace_u8(self.origin_id), state,
            TRACE_REASON_INDEX.get(result.get('reason'), 255),
            FAILURE_INDEX.get(failure, 255), quality, ref_kind,
            trace_u8(ref_count),
            trace_u8(vision.get('projected') if has_vision else None),
            trace_u8(result.get('matches') if has_vision else None),
            trace_u8(result.get('inliers') if has_vision else None),
            trace_u8(vision.get('added') if has_vision else None),
            trace_u8(vision.get('klt_eig_reject') if has_vision else None),
            trace_u8(vision.get('klt_fail') if has_vision else None),
            trace_u8(vision.get('fb_reject') if has_vision else None),
            trace_u8(vision.get('sad_fail') if has_vision else None),
            trace_u8(vision.get('new_prev_count') if ref_kind == 1 else None),
            trace_u8(vision.get('new_prev_survived') if ref_kind == 1 else None),
            gradient_cells,
            trace_u8(vision.get('detect_cells') if has_vision else None),
            flags,
            VISION_REASON_INDEX.get(vision.get('vision_reason'), 255) if has_vision else 255)

    def update_rates(self, now):
        elapsed = now-self.rate_start
        if elapsed >= RATE_WINDOW_US:
            self.rates[0] = self.rate_counts[0]*1000000.0/elapsed
            self.rates[1] = self.rate_counts[2]*1000000.0/elapsed
            self.rates[2] = 100.0*self.rate_counts[1]/max(1, self.rate_counts[0])
            self.rate_counts[0] = self.rate_counts[1] = self.rate_counts[2] = 0
            self.rate_start, self.rate_ready = now, True

    def fresh(self, result, now):
        return result is not None and 0 <= now-result['timestamp_us'] <= C.MAX_TX_AGE_US

    def relative(self, result, now):
        if self.origin is None or not self.fresh(result, now) or not result['valid']:
            return None
        return result['position'][0]-self.origin[0], result['position'][1]-self.origin[1]

    def state(self, result, now):
        if result is None:
            return 2
        state = ((4 if result.get('predicted') else 3 if result.get('sparse') else 1)
                 if result['valid'] and self.fresh(result, now) else
                 0 if result['reason'] in WARMUP_STATES else 2)
        return state | (8 if result['approximate'] else 0)

    def summary(self, result, now):
        state = self.state(result, now)
        label = DISPLAY_STATES[state & 7]+('（近似）' if state & 8 else '')
        relative = self.relative(result, now)
        position = 'ΔN=-- ΔE=--' if relative is None else 'ΔN=%+.3fm ΔE=%+.3fm' % relative
        fresh = self.fresh(result, now)
        height = result['height'] if fresh else None
        q = result.get('q') if fresh else None
        attitude = '--'
        if q is not None:
            try:
                attitude = '%+.1f/%+.1f/%+.1f°' % euler_degrees(q)
            except (ValueError, TypeError):
                pass
        warming = '（窗口统计中）' if not self.rate_ready else ''
        print('状态=%s | %s | 高度=%sm | 横滚/俯仰/航向=%s | 处理=%.2fFPS | 可发布=%.2fHz | 有效率=%.1f%%%s' % (
            label, position, number(height, 3), attitude, self.rates[0], self.rates[1], self.rates[2], warming))
        self.last_display, self.pending_display = now, False

    def commit_phase(self, i, duration):
        self.phase_counts[i] += 1
        self.phase_totals[i] += duration
        self.phase_maximum[i] = max(self.phase_maximum[i], duration)

    def interact(self, result, now, new):
        """交互与数据统计分开；交互异常也能完成本帧统计收尾。"""
        stop = query = False
        for action in self.console.poll():
            if action == 'exit':
                stop = True
                break
            if action == 'origin':
                self.pending_origin = True
                if not new or not result['valid'] or not result.get('measured') or not self.fresh(result, now):
                    print('已收到Enter：等待下一有效新图像建立原点。')
            elif action == 'query':
                query = True
        if (self.pending_origin and new and result['valid'] and result.get('measured')
                and not result.get('predicted') and self.fresh(result, now)):
            self.origin = result['position'][0], result['position'][1]
            self.origin_id += 1
            self.pending_origin = False
            self.rate_counts[0] = self.rate_counts[1] = self.rate_counts[2] = 0
            self.rate_start, self.rate_ready = now, False
            self.rates[0] = self.rates[1] = self.rates[2] = 0.0
            print('初始点%d已建立，ΔN/ΔE归零；视觉跟踪与全程统计保留。' % self.origin_id)
        self.update_rates(now)
        state = self.state(self.last_result, now)
        if state != self.display_state:
            self.display_state, self.pending_display = state, True
        if query or (self.pending_display and now-self.last_display >= EVENT_DISPLAY_INTERVAL_US):
            self.summary(self.last_result, self.link.clock.now_us())
        return stop

    def finish_frame(self, result, fps):
        debug_start = _us() if self.timing else None
        now = self.link.clock.now_us()
        new = self.observe(result, now)
        try:
            return self.interact(result, now, new)
        finally:
            if self.timing and new:
                self.record_trace(result)
            self.finish_statistics(now, debug_start)

    def finish_statistics(self, now, debug_start):
        if self.timing:
            if now-self.last_heap_sample >= 1000000:
                self.last_heap_sample = now
                if hasattr(gc, 'mem_free'):
                    free = gc.mem_free()
                    self.minimum_heap = free if self.minimum_heap is None else min(self.minimum_heap, free)
            for i in range(len(PHASES)):
                if self.seen & (1 << i):
                    self.commit_phase(i, self.values[i])
            # 固定表更新也纳入debug/cycle；不再放在完整循环计时之后。
            self.commit_phase(INDEX['debug'], _elapsed(debug_start))
            self.commit_phase(INDEX['cycle'], _elapsed(self.frame_start))
            self.active = False
        self.end = self.link.clock.now_us()

    def snapshot(self):
        elapsed = 0 if self.start is None else max(0, self.end-self.start)
        seconds = elapsed*1e-6
        return dict(seconds=seconds, frames=self.frames, stable=self.valid_frames, predicted=self.predicted_frames,
            publishable=self.publish_frames, fps=None if not seconds else self.frames/seconds,
            publish_hz=None if not seconds else self.publish_frames/seconds,
            quality=None if not self.quality_count else self.quality_sum/self.quality_count,
            valid_ratio=None if not self.frames else 100.0*self.valid_frames/self.frames)

    def common_lines(self, snapshot, reason, error):
        yield '结束原因=%s | 测量时长=%.3fs\n' % (reason, snapshot['seconds'])
        yield '总接收图像=%d | 真实稳定追踪=%d | 短时预测=%d | 可发布帧=%d\n' % (
            snapshot['frames'], snapshot['stable'], snapshot['predicted'], snapshot['publishable'])
        confidence = '无有效样本' if snapshot['quality'] is None else number(snapshot['quality'])+'/100'
        yield '平均处理=%sFPS | 平均可发布=%sHz | 平均置信度=%s | 有效率=%s%%\n' % (
            number(snapshot['fps']), number(snapshot['publish_hz']),
            confidence, number(snapshot['valid_ratio']))
        yield '失跟段=%d | 轨迹重置=%d | 初始化帧=%d | 恢复确认帧=%d\n' % (
            self.lost_episodes, self.reset_events, self.initial_frames, self.recovery_frames)
        if not self.frames and self.no_new_loops:
            yield '未获得可统计的新图像；无新图像/重复循环=%d。\n' % self.no_new_loops
        counts = self.loss_counts if self.lost_episodes else self.failure_counts
        total = sum(counts)
        if self.lost_episodes:
            yield '主要失跟原因（仅已建立追踪之后；比例分母为已分类原因帧）\n'
        elif not self.had_tracking:
            yield '尚未建立稳定追踪；首次定位受阻原因（初始化/恢复确认单列）\n'
        else:
            yield '本次未发生失跟；启动及其他输入受阻原因\n'
        ranked = sorted(range(len(FAILURES)), key=lambda i: counts[i], reverse=True)
        emitted = 0
        for i in ranked:
            if counts[i] and emitted < 3:
                group = ('视觉/运行限制', '输入不可用', '其他')[FAILURES[i][2]]
                yield '%s：%s=%d帧（%.1f%%）\n' % (group, FAILURES[i][1], counts[i], 100.0*counts[i]/total)
                emitted += 1
        if not total:
            yield '无已分类失效原因。\n'
        yield '置信度沿用算法0~100评分；可发布为准入+限频模拟，ODOMETRY实际发送=0。\n'
        if error:
            yield '异常信息：%s\n' % error

    def report(self, write, snapshot, reason, error):
        write('MODE2精简定位日志 | 版本=%s\n' % DEBUG_VERSION)
        for line in self.common_lines(snapshot, reason, error):
            write(line)
        seconds = snapshot['seconds']
        write('有效定位=%.3fHz（仅MODE2）；无新图像/重复循环=%d\n' % (
            self.valid_frames/seconds if seconds else 0.0, self.no_new_loops))
        write('关键启动配置：宽/高/特征目标/FAST/KLT迭代/曝光/增益/发布置信度/发布限频/时序确认/方向确认/sigma0(m)/预测上限(us)/重置上限(us)=%r\n' % (self.start_config,))
        write('\n视觉可靠性：样本数；最低/平均/最大\n')
        for i, name in enumerate(DETAIL_NAMES):
            count = self.detail_counts[i]
            write('%s | %s\n' % (name, '未执行' if not count else '%d；%.3f/%.3f/%.3f' % (
                count, self.detail_minimum[i], self.detail_sums[i]/count, self.detail_maximum[i])))
        write('少点有效跟踪=%d帧；纯FAST/KLT，无ORB恢复\n' % self.sparse_frames)
        write('\n全程失效帧（初始化/恢复确认单列；与失跟段数不同）\n')
        for i, (_, label, _) in enumerate(FAILURES):
            if self.failure_counts[i]:
                write('%s=%d帧\n' % (label, self.failure_counts[i]))
        write('\n发布受阻帧（同一帧可有多个原因）\n')
        for i, label in enumerate(BLOCK_NAMES):
            if self.block_counts[i]:
                write('%s=%d帧\n' % (label, self.block_counts[i]))
        link = self.link
        att = getattr(link, 'attitude_received', 0)-self.rx_start[0]
        ran = getattr(link, 'range_received', 0)-self.rx_start[1]
        write('\n输入：姿态=%d包/%.2fHz ToF=%d包/%.2fHz CRC错误=%d RTT拒绝=%d\n' % (
            att, att/seconds if seconds else 0, ran, ran/seconds if seconds else 0,
            link.parser.bad_crc-self.rx_start[2], link.sync_rtt_rejected-self.rx_start[3]))
        write('最低可用堆=%s（每秒采样，不代表瞬时分配峰值）\n' % (
            '未提供' if self.minimum_heap is None else '%d字节' % self.minimum_heap))
        write('\n阶段：次数；平均/最大ms；累计ms；占完整循环比例\n')
        cycle = self.phase_totals[INDEX['cycle']]
        for i, (_, label) in enumerate(PHASES):
            count = self.phase_counts[i]
            if count:
                total = self.phase_totals[i]
                write('%s | %d；%.3f/%.3f；%.3f；%.2f%%\n' % (
                    label, count, total/count*.001, self.phase_maximum[i]*.001, total*.001, 100.0*total/max(1, cycle)))
            else:
                write('%s | 未执行\n' % label)
        write('原生六阶段属于视觉子项，不重复计入完整循环；GC只统计显式调用。采图含等待。未完成的中断循环不纳入阶段表。\n')
        write('\n重要事件：总数=%d 保留=%d 被覆盖记录=%d；重复事件合并\n' % (
            self.events.total, self.events.count, self.events.overwritten))
        for line in self.events.lines():
            write(line)
        write('\n逐帧诊断：总数=%d 保留=%d 覆盖=%d；时间为本次测试开始后的毫秒。'
              'N/E在origin_id>0且position_relative=1时相对该原点，否则为VO原始坐标；'
              '255/空白表示无测量。新点留存只统计上一帧参考，关键帧参考留空；'
              '失跟帧的added_points表示补入旧参考图的点。\n' % (
                  self.trace.total, self.trace.count, self.trace.overwritten))
        write('frame_ms,sequence,reset_id,origin_id,position_relative,state,reason,vision_reason,'
              'failure,quality,n_m,e_m,roll_deg,pitch_deg,yaw_deg,height_m,exposure_us,gain_db,'
              'overexposed_fraction,overexposed_cell_mask_hex,gradient2d_min_eigen,'
              'gradient_usable_cells,reference,reference_points,projected_points,warp_reject,'
              'matches,inliers,fast_candidates,weak_candidates,added_points,scanned_cells,'
              'finding_outcome,klt_eigen_reject,'
              'klt_fail,forward_backward_reject,sad_fail,new_previous_points,'
              'new_previous_survived,new_previous_retention\n')
        states = ('lost', 'measured', 'predicted', 'pose_confirming')
        for row in self.trace.rows():
            flags = row[35]
            retained = (trace_decimal(row[32]/row[31], 3)
                        if row[21] == 1 and row[31] not in (0, 255) and row[32] != 255 else '')
            if row[34] == 255 or row[26] == 255:
                finding = ''
            elif row[34] == 0:
                finding = 'not_scanned'
            elif row[26] > 0:
                finding = 'added'
            elif row[13] == 0 and row[14] == 0:
                finding = 'no_2d_candidate'
            else:
                finding = 'candidates_not_added'
            warp_reject = (row[22]-row[23]
                           if row[22] != 255 and row[23] != 255 else '')
            values = (
                row[0], row[1], trace_cell(row[15]), trace_cell(row[16]), int(bool(flags & 1)),
                trace_name(states, row[17]), trace_name(TRACE_REASON_NAMES, row[18]),
                trace_name(VISION_REASON_NAMES, row[36]),
                trace_name(tuple(item[0] for item in FAILURES), row[19]), trace_cell(row[20]),
                trace_decimal(row[2]), trace_decimal(row[3]),
                trace_scaled(row[4], -32768, 100, 2), trace_scaled(row[5], -32768, 100, 2),
                trace_scaled(row[6], -32768, 100, 2), trace_scaled(row[7], -32768, 1000, 3),
                trace_scaled(row[8], 65535, 1, 0), trace_scaled(row[9], -32768, 100, 2),
                trace_scaled(row[10], 65535, 1000, 3),
                ('%06X' % row[12]) if flags & 16 else '', trace_decimal(row[11], 3),
                trace_cell(row[33]), ('previous' if row[21] == 1 else 'key' if row[21] == 0 else ''),
                trace_cell(row[22]), trace_cell(row[23]), warp_reject,
                trace_cell(row[24]), trace_cell(row[25]),
                trace_cell(row[13]), trace_cell(row[14]), trace_cell(row[26]), trace_cell(row[34]),
                finding,
                trace_cell(row[27]), trace_cell(row[28]), trace_cell(row[29]), trace_cell(row[30]),
                trace_cell(row[31]), trace_cell(row[32]), retained)
            write(','.join(str(value) for value in values)+'\n')
        write('最终报告保存发生在测量结束后；运行中无文件写入。日志完成\n')

    def close(self, reason='x正常结束', error=None):
        if self.closed:
            return
        self.closed, self.active = True, False
        # 最终写盘及异常中断的半个循环不进入测量分母。
        snapshot = self.snapshot()
        if error and self.start is not None:
            self.event(self.end, 6)
        for line in self.common_lines(snapshot, reason, error):
            print(line, end='')
        if not self.timing:
            return
        try:
            self.log = SessionLog(self.log_directory if self.log_directory is not None else LOG_DIRECTORY)
            if self.start is None:
                self.log.write('MODE2未进入测试循环 | 版本=%s\n' % DEBUG_VERSION)
                for line in self.common_lines(snapshot, reason, error):
                    self.log.write(line)
                self.log.write('日志完成\n')
            else:
                self.report(self.log.write, snapshot, reason, error)
        except Exception as save_error:
            if self.log is not None:
                self.log.error = repr(save_error)
            print('日志保存失败：%r' % save_error)
        finally:
            if self.log is not None:
                self.log.close()
                print('日志=%s | 保存状态=%s' % (self.log.path, self.log.error or '完成'))


def main():
    session = TestSession(MODE)
    reason, error = 'x正常结束', None
    try:
        run(session)
    except KeyboardInterrupt:
        reason = 'KeyboardInterrupt中断'
    except BaseException as exception:
        reason, error = '程序异常', exception_text(exception)
        raise
    finally:
        try:
            session.close(reason, error)
        except BaseException as save_error:
            print('总结保存失败：%r；结束原因=%s 原始错误=%s' % (save_error, reason, error))


if __name__ == '__main__':
    main()
