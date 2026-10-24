"""事件触发测试：MODE1简略观察；MODE2同样显示，退出时保存全程统计。"""
MODE = 1
DEBUG_VERSION = 'usb-buffer-2'
RATE_WINDOW_US = 2000000
EVENT_DISPLAY_INTERVAL_US = 1000000
LOG_DIRECTORY = None       # None: 优先/sd/logs，否则/flash/logs；PC使用./logs
EVENT_BUFFER_SIZE = 8      # 有界事件缓冲；不保存图像或逐帧历史

import gc
import math
import os
import sys
from array import array
import config as C
from algorithm import _us, _elapsed, euler_degrees
from output import _state, publication_blockers, PublicationGate
from control import range_invalid_detail
from main import run

PHASES = (
    ("poll_before", "采图前通信/同步"), ("capture", "采图等待"),
    ("poll_after", "采图后通信/同步"), ("wait_attitude", "等待姿态"),
    ("camera", "曝光调节/统计"), ("vo_total", "VO总处理（含子项）"),
    ("features", "  按需ORB检测/描述子"), ("match", "  ORB恢复匹配"),
    ("track_fit", "  原生跟踪/投影/拟合"), ("vision_poll", "  视觉内通信"),
    ("image_stats", "  VO内纹理统计"), ("vo_other", "  VO其余逻辑"),
    ("housekeeping", "循环维护（含GC）"), ("gc", "  垃圾回收"),
    ("output", "发布检查"), ("test_output", "调试输入/输出及日志"),
    ("cycle", "完整循环（含统计前的调试开销）"))
INDEX = {name: i for i, (name, _) in enumerate(PHASES)}
VO_PARTS = ("features", "match", "track_fit", "vision_poll", "image_stats")
BLOCK_NAMES = {"TIMING_UNCONFIRMED": "时序未确认", "POSITION_INVALID": "位置无效",
    "EXTRINSICS_UNCONFIRMED": "外参未确认或机体位置无效", "LOW_QUALITY": "质量不足",
    "DISCONNECTED": "飞控未连接", "UNSYNCED": "时钟未同步", "OBSERVATION_STALE": "观测过期或时间在未来"}


def increment(table, key, count=1):
    table[key] = table.get(key, 0)+count


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


class Metric:
    """常量空间全程统计；无效数值单独计数。"""
    def __init__(self):
        self.count = self.invalid = 0
        self.total = 0.0
        self.minimum = self.maximum = self.last = None

    def add(self, value):
        if value is None or not math.isfinite(value):
            self.invalid += 1
            return
        self.count += 1
        self.total += value
        self.minimum = value if self.minimum is None else min(self.minimum, value)
        self.maximum = value if self.maximum is None else max(self.maximum, value)
        self.last = value

    def text(self, scale=1.0):
        if not self.count:
            return "无有效样本/未执行 | 无效=%d" % self.invalid
        return "样本=%d 无效=%d | 最小/平均/最大/最后=%.4f/%.4f/%.4f/%.4f" % (
            self.count, self.invalid, self.minimum*scale, self.total/self.count*scale,
            self.maximum*scale, self.last*scale)


# 保留原验证脚本所使用的固定容量Samples接口；新日志使用全程Metric。
class Samples:
    def __init__(self, capacity=256):
        self.values = array("I", [0]*capacity)
        self.count = self.total = self.maximum = 0

    def add(self, value):
        value = max(0, int(value))
        self.values[self.count % len(self.values)] = value
        self.count += 1
        self.total += value
        self.maximum = max(self.maximum, value)


class Console:
    """USB VCP原始终端非阻塞输入；绝不读取IDE调试协议字节。"""
    def __init__(self):
        self.vcp = self.poller = None
        self.rx_buffer = bytearray(64)
        self.rx_view = memoryview(self.rx_buffer)
        self.after_cr = False
        try:
            from pyb import USB_VCP
            self.vcp = USB_VCP()
            if self.vcp.debug_mode_enabled():
                raise RuntimeError("请断开IDE主调试连接，再通过工具→打开终端的串口连接运行test.main()")
        except ImportError:
            import select
            self.poller = select.poll()
            self.poller.register(sys.stdin, select.POLLIN)

    def read(self):
        if self.vcp is not None:
            # any()只判断是否有输入；recv使用复用的可写缓冲区并返回实际字节数。
            if not self.vcp.any():
                return b""
            count = self.vcp.recv(self.rx_buffer, timeout=0)
            return self.rx_view[:count] if count else b""
        if not self.poller.poll(0):
            return b""
        # stdin非阻塞就绪后只取一个字符；不使用阻塞readline。
        return sys.stdin.read(1).encode()

    def commands(self, data):
        actions = []
        for value in data:
            if value == 10 and self.after_cr:
                self.after_cr = False
                continue
            self.after_cr = value == 13
            if value in (10, 13):
                actions.append("origin")
            elif value in (ord('q'), ord('Q')):
                actions.append("query")
            elif value in (ord('x'), ord('X')):
                actions.append("exit")
        return actions

    def poll(self):
        return self.commands(self.read())


def is_directory(path):
    try:
        return bool(os.stat(path)[0] & 0x4000)
    except OSError:
        return False


class SessionLog:
    def __init__(self, directory=None):
        self.file = None
        self.pending = []
        self.error = None
        if directory is None:
            if is_directory('/sd'):
                directory = '/sd/logs'
            elif is_directory('/flash'):
                directory = '/flash/logs'
            else:
                directory = 'logs'
        if not is_directory(directory):
            os.mkdir(directory)
        index = 1
        while True:
            path = '%s/vo_test_%04d.txt' % (directory, index)
            try:
                os.stat(path)
            except OSError:
                break
            index += 1
        self.path = path
        self.file = open(path, 'w')
        self.write('MODE2测试日志\n调试版本=%s\n状态=运行中；缺少“日志完成”表示未完成最终汇总。\n' % DEBUG_VERSION)
        self.flush()

    def write(self, text):
        if self.error is not None:
            return
        try:
            self.file.write(text)
        except OSError as error:
            self.error = repr(error)

    def event(self, text):
        self.pending.append(text)
        if len(self.pending) >= EVENT_BUFFER_SIZE:
            self.flush()

    def flush(self):
        if self.pending:
            self.write(''.join(self.pending))
            self.pending = []
        if self.error is None:
            try:
                self.file.flush()
            except OSError as error:
                self.error = repr(error)

    def close(self):
        self.flush()
        try:
            self.file.close()
        except OSError as error:
            self.error = repr(error)


class TestSession:
    def __init__(self, mode=MODE, console=None, log_directory=None):
        if mode not in (1, 2):
            raise ValueError('MODE must be 1 (summary) or 2 (summary + final log)')
        self.mode, self.timing = mode, mode == 2
        self.console, self.log_directory = console, log_directory
        self.log = self.link = None
        self.closed = self.active = False
        self.values = [0]*len(PHASES)
        self.seen = [False]*len(PHASES)
        self.phases = [Metric() for _ in PHASES] if self.timing else None
        self.metrics, self.reasons, self.blockers, self.durations = {}, {}, {}, {}
        self.frames = self.valid_frames = self.publish_frames = self.loops = 0
        self.gate = PublicationGate()
        self.rate_start = None
        self.rate_counts = [0, 0, 0]
        self.rates = (0.0, 0.0, 0.0)
        self.rate_ready = False
        self.origin = self.segment = None
        self.origin_id = 0
        self.pending_origin = False
        self.epoch = self.last_result = self.state_key = None
        self.last_display = -EVENT_DISPLAY_INTERVAL_US
        self.pending_display = False
        self.start = self.last_state_at = None
        self.events = self.lost_episodes = self.orb_attempts = self.orb_success = 0
        self.reset_events = 0
        self.health_changes = {}
        self.last_vision = None
        self.last_camera_sample = -1000000
        self.last_sync_received = 0
        self.last_ack = None
        self.latest_blockers = ()

    def prepare(self, sensor, link, controller, capture, adapter, output):
        self.sensor, self.link, self.controller = sensor, link, controller
        self.output = output
        output.publish = output.display = False
        self.start = self.rate_start = self.last_state_at = link.clock.now_us()
        self.rx_start = (getattr(link, 'attitude_received', 0), getattr(link, 'range_received', 0))
        if self.timing:
            self.log = SessionLog(self.log_directory if self.log_directory is not None else LOG_DIRECTORY)
            self.log.write('配置快照（启动值）\n')
            for key in sorted(dir(C)):
                if key.isupper():
                    self.log.write('%s=%r\n' % (key, getattr(C, key)))
            self.log.flush()
        if self.console is None:
            self.console = Console()
        print('MODE%d | 调试版本=%s | Enter=建立/更新初始点 q=查询 x=结束 | 测试不发送ODOMETRY' % (
            self.mode, DEBUG_VERSION))
        print('ΔN/ΔE为飞控NED轴的相对位移；等待Enter建立初始点。请使用原始终端，q/x无需回车。')
        if self.log is not None:
            print('日志=%s | 运行中累计统计，结束后写入全程报告。' % self.log.path)

    def begin_frame(self):
        if not self.timing:
            return
        for i in range(len(PHASES)):
            self.values[i], self.seen[i] = 0, False
        self.frame_start = self.cursor = _us()
        self.active = True

    def record(self, name, duration):
        if self.timing and self.active:
            i = INDEX[name]
            self.values[i] += max(0, int(duration))
            self.seen[i] = True

    def mark(self, name):
        if self.timing:
            self.record(name, _elapsed(self.cursor))
            self.cursor = _us()

    def add(self, name, value):
        metric = self.metrics.get(name)
        if metric is None:
            metric = self.metrics[name] = Metric()
        metric.add(value)

    def event(self, now, message):
        self.events += 1
        if self.log is not None:
            self.log.event('[%.3fs] %s\n' % ((now-self.start)*1e-6, message))

    def update_rates(self, now):
        elapsed = now-self.rate_start
        if elapsed >= RATE_WINDOW_US:
            self.rates = tuple(count*1000000.0/elapsed for count in self.rate_counts)
            self.rate_ready = True
            self.rate_counts = [0, 0, 0]
            self.rate_start = now

    def relative(self, result):
        if self.origin is None or result is None or not result['valid']:
            return None
        return tuple(result['position'][i]-self.origin[i] for i in range(2))

    def end_segment(self, now, why):
        segment = self.segment
        if segment is None:
            return
        if self.log is not None:
            self.log.flush()
            self.log.write('\n初始点区间%d | 开始=%.3fs 时长=%.3fs 结束原因=%s\n' % (
                segment['id'], (segment['start']-self.start)*1e-6,
                (now-segment['start'])*1e-6, why))
            self.log.write('帧数=%d 有效=%d 有效率=%.2f%% 最后有效ΔN/ΔE=%r 最终位置有效=%s\n' % (
                segment['frames'], segment['valid'], 100.0*segment['valid']/max(1, segment['frames']),
                segment['last'], segment['current_valid']))
            self.log.write('内部参考原点N/E=%r 所属轨迹reset_counter=%d；方向为飞控NED轴。\n' % (
                segment['origin'], segment['epoch']))
            for name in ('N', 'E'):
                self.log.write('Δ%s(m) | %s\n' % (name, segment[name].text()))
            self.log.flush()
        self.segment = None

    def set_origin(self, result, now):
        self.end_segment(now, 'Enter更新初始点')
        self.origin = tuple(result['position'])
        self.origin_id += 1
        self.pending_origin = False
        self.rate_start, self.rate_counts = now, [0, 0, 0]
        self.rates = (0.0, 0.0, 0.0)
        self.rate_ready = False
        self.segment = dict(id=self.origin_id, start=now, frames=0, valid=0,
                            origin=self.origin, epoch=result['reset_counter'],
                            last=None, current_valid=True, N=Metric(), E=Metric())
        self.event(now, '已建立初始点%d；ΔN/ΔE归零，保留视觉跟踪' % self.origin_id)
        print('已建立初始点%d，ΔN=0 ΔE=0；帧率窗口重新统计。' % self.origin_id)

    def observe_details(self, result, now):
        link = self.link
        if result is not None:
            vision = result.get('vision') or {}
            for key in ('height', 'quality', 'residual_px', 'attitude_age_us', 'range_age_us', 'residual_yaw'):
                self.add(key, result.get(key))
            self.add('range_skipped_attitude_cumulative_current_epoch', result.get('range_skipped_attitude'))
            if result['q'] is not None:
                # 角度有环绕，只记录最终值和范围，报告时不输出普通平均。
                for name, value in zip(('roll_deg', 'pitch_deg', 'yaw_deg'), euler_degrees(result['q'])):
                    self.add(name, value)
            if vision:
                self.last_vision = vision
                for key in ('keypoints', 'matches', 'inliers', 'cells', 'spread', 'klt', 'sad', 'search_px'):
                    self.add('vision_'+key, vision.get(key))
                if result['matches']:
                    self.add('inlier_ratio', result['inliers']/result['matches'])
                if self.seen[INDEX['match']]:
                    self.orb_attempts += 1
                self.orb_success += int(bool(vision.get('orb')))
                increment(self.reasons, 'reference_'+str(vision.get('reference', 'unknown')))
            variance = result.get('variance')
            if result['valid'] and variance is not None:
                self.add('variance_N_m2', variance[0]); self.add('variance_E_m2', variance[1])
            increment(self.reasons, 'height_'+str(result.get('range_state', 'unknown')))
            if result.get('range_detail'):
                increment(self.reasons, 'range_'+result['range_detail'])
        for name, history in (('attitude', link.attitudes), ('range', link.ranges)):
            self.add(name+'_receive_age_ms', None if not history else (now-history[-1][-1])*.001)
        fc_stamp = None if result is None else link.fc_time(result['timestamp_us'])
        self.add('image_minus_latest_attitude_ms', None if fc_stamp is None or not link.attitudes
                 else (fc_stamp-link.attitudes[-1][0])*.001)
        received = getattr(link, 'sync_received', 0)
        if received != self.last_sync_received:
            self.add('sync_RTT_ms', None if link.last_sync_rtt_us is None else link.last_sync_rtt_us*.001)
            self.last_sync_received = received
        if link.last_ack != self.last_ack:
            self.event(now, '消息频率请求结果=%r' % (link.last_ack,))
            self.last_ack = link.last_ack
        if now-self.last_camera_sample >= 1000000:
            self.last_camera_sample = now
            self.add('exposure_us', self.sensor.get_exposure_us())
            self.add('gain_db', self.sensor.get_gain_db())
            if link.ranges:
                sample = link.ranges[-1]
                self.add('raw_range_m', sample[1])
                self.add('range_min_m', sample[2]); self.add('range_max_m', sample[3])
                self.add('range_signal_raw', sample[4])
                increment(self.reasons, 'raw_range_'+str(range_invalid_detail(*sample[1:5]) or 'OK'))
            if hasattr(gc, 'mem_free'):
                self.add('heap_free_bytes', gc.mem_free())

    def observe(self, result, now):
        self.loops += 1
        if result is not None:
            self.last_result = result
            epoch = result['reset_counter']
            if self.epoch is not None and epoch != self.epoch:
                self.reset_events += 1
                self.end_segment(now, '算法轨迹重置')
                self.origin = None
                self.pending_origin = False
                self.event(now, '算法轨迹重置；旧初始点失效，请重新按Enter')
                print('算法轨迹重置：旧初始点已清除，请重新按Enter。')
            self.epoch = epoch
            self.frames += 1
            self.valid_frames += int(result['valid'])
            blockers = publication_blockers(result, self.link, now)
            published = self.gate.accept(result, now, blockers)
            self.latest_blockers = blockers
            self.publish_frames += int(published)
            for i, value in enumerate((1, int(result['valid']), int(published))):
                self.rate_counts[i] += value
            if self.timing:
                for name in blockers:
                    increment(self.blockers, name)
        reason = 'NO_NEW_FRAME' if result is None else result['reason']
        if self.timing:
            increment(self.reasons, reason)
            self.observe_details(result, now)
        state = (self.link.connected(now), self.link.synced(now), reason,
                 None if result is None else result.get('range_detail'),
                 None if result is None else result['approximate'])
        if state != self.state_key:
            if self.state_key is not None:
                self.add_duration(now)
                for i, name in ((0, '连接'), (1, '同步')):
                    if state[i] != self.state_key[i]:
                        increment(self.health_changes, name+('建立' if state[i] else '失效'))
            if reason in ('LOST', 'NO_FEATURES') and (self.state_key is None or self.state_key[2] not in ('LOST', 'NO_FEATURES')):
                self.lost_episodes += 1
            self.state_key, self.last_state_at = state, now
            self.event(now, '状态：连接=%s 同步=%s %s 原因=%s 近似=%s' % (
                state[0], state[1], _state(reason), state[3], state[4]))
            self.pending_display = True
        self.update_rates(now)

    def add_duration(self, now):
        if not self.timing or self.state_key is None:
            return
        duration = max(0, now-self.last_state_at)
        increment(self.durations, self.state_key[2], duration)
        if not self.state_key[0]:
            increment(self.durations, 'DISCONNECTED', duration)
        if not self.state_key[1]:
            increment(self.durations, 'UNSYNCED', duration)
        self.last_state_at = now

    def summary(self, result, now):
        fresh = result is not None and 0 <= now-result['timestamp_us'] <= C.MAX_TX_AGE_US
        valid = fresh and result['valid']
        reason = '暂无新图像' if result is None else _state(result['reason'])
        if result is not None and result['approximate']:
            reason += '（近似）'
        if result is not None and result.get('range_detail'):
            reason += '：'+_state(result['range_detail'])
        if not fresh:
            reason += '（当前无新鲜观测）'
        attitude = '--' if not fresh or result['q'] is None else '%+.1f/%+.1f/%+.1f°' % euler_degrees(result['q'])
        height = '--' if not fresh or result['height'] is None else '%.3fm' % result['height']
        relative = self.relative(result) if valid else None
        position = 'ΔN=-- ΔE=--' if relative is None else 'ΔN=%+.3fm ΔE=%+.3fm' % relative
        origin = '未建立，按Enter' if self.origin is None else str(self.origin_id)
        warming = '（窗口统计中）' if not self.rate_ready else ''
        print('状态=%s | 初始点=%s | 有效定位=%.2fHz 可发布=%.2fHz%s | 横滚/俯仰/航向=%s | 离地高度=%s | %s' % (
            reason, origin, self.rates[1], self.rates[2], warming, attitude, height, position))
        self.last_display, self.pending_display = now, False

    def finish_frame(self, result, fps):
        now = self.link.clock.now_us()
        debug_start = _us() if self.timing else None
        self.observe(result, now)
        actions = self.console.poll()
        candidate = (result is not None and result['valid']
                     and 0 <= now-result['timestamp_us'] <= C.MAX_TX_AGE_US)
        stop = False
        query = False
        for action in actions:
            if action == 'exit':
                stop = True
                break
            if action == 'origin':
                self.pending_origin = True
                query = True
                if not candidate:
                    print('已收到Enter：等待下一有效定位帧建立初始点。')
                    self.event(now, 'Enter请求；等待下一有效定位帧')
            elif action == 'query':
                query = True
                self.event(now, 'q查询摘要')
        if self.pending_origin and candidate:
            self.set_origin(result, now)
            query = True
        if self.segment is not None:
            self.segment['frames'] += int(result is not None)
            relative = self.relative(result) if candidate else None
            self.segment['current_valid'] = relative is not None
            if relative is not None:
                self.segment['valid'] += 1
                self.segment['last'] = relative
                self.segment['N'].add(relative[0]); self.segment['E'].add(relative[1])
        if query or (self.pending_display and now-self.last_display >= EVENT_DISPLAY_INTERVAL_US):
            self.summary(result, now)
        if self.timing:
            self.record('test_output', _elapsed(debug_start))
            if self.seen[INDEX['vo_total']]:
                self.record('vo_other', max(0, self.values[INDEX['vo_total']]-sum(self.values[INDEX[n]] for n in VO_PARTS)))
            self.record('cycle', _elapsed(self.frame_start))
            self.active = False
            for i, metric in enumerate(self.phases):
                if self.seen[i]:
                    metric.add(self.values[i])
        return stop

    def report(self, now, reason, error):
        write = self.log.write
        seconds = max(0, now-self.start)*1e-6
        divisor = max(.000001, seconds)
        write('\n=== 整次测试最终统计 ===\n结束原因=%s\n错误=%s\n运行时长=%.3fs 循环=%d 图像=%d 有效=%d 可发布=%d\n' % (
            reason, error or '无', seconds, self.loops, self.frames, self.valid_frames, self.publish_frames))
        write('全程处理/有效定位/可发布=%.3f/%.3f/%.3fHz 有效率=%.2f%%\n' % (
            self.frames/divisor, self.valid_frames/divisor, self.publish_frames/divisor,
            100.0*self.valid_frames/max(1, self.frames)))
        write('结束时最近窗口处理/有效/可发布=%r Hz；不满首个窗口时为0，非全程统计。\n' % (self.rates,))
        write('可发布为准入+限频干跑，测试ODOMETRY实际发送=0；不代表UART吞吐或飞控融合率。\n')
        result = self.last_result
        write('最终观测=%r\n' % (None if result is None else {k: result.get(k) for k in (
            'timestamp_us', 'reason', 'valid', 'q', 'height', 'quality', 'reset_counter', 'keyframe', 'approximate')},))
        fresh = result is not None and 0 <= now-result['timestamp_us'] <= C.MAX_TX_AGE_US
        write('最终观测新鲜=%s；最后有效值与当前有效性分别标记。\n' % fresh)
        write('最后一次已执行视觉诊断=%r\n' % (self.last_vision,))
        write('\n数值统计：每帧/执行时采样；曝光、增益、原始ToF、堆每秒采样。\n')
        for name in sorted(self.metrics):
            metric = self.metrics[name]
            if name.endswith('_deg'):
                write('%s | 样本=%d 最小/最大/最后=%r/%r/%r；角度环绕，不计算普通均值\n' % (
                    name, metric.count, metric.minimum, metric.maximum, metric.last))
            else:
                write('%s | %s\n' % (name, metric.text()))
        write('\n状态/检查结果出现帧数（子项可重叠，不是测距拒绝包数）\n')
        for name in sorted(self.reasons):
            write('%s=%d\n' % (name, self.reasons[name]))
        write('\n状态持续时间（最后观察状态维持至下一次观察，子项可重叠）\n')
        for name in sorted(self.durations):
            write('%s=%.4fs\n' % (name, self.durations[name]*1e-6))
        write('\n发布受阻帧数（同一帧可能有多个原因）\n')
        for name in sorted(self.blockers):
            write('%s=%d\n' % (BLOCK_NAMES.get(name, name), self.blockers[name]))
        link = self.link
        att_count = getattr(link, 'attitude_received', 0)-self.rx_start[0]
        range_count = getattr(link, 'range_received', 0)-self.rx_start[1]
        write('\n输入全程：选定飞控姿态包=%d(%.3fHz) 选定ToF包=%d(%.3fHz)\n' % (
            att_count, att_count/divisor, range_count, range_count/divisor))
        write('最终链路：连接=%s 同步=%s 同步计数=%d 同步回复累计=%d RTT拒绝累计=%d CRC错误累计=%d 发送丢弃累计=%d\n' % (
            link.connected(now), link.synced(now), link.sync_count, getattr(link, 'sync_received', 0),
            link.sync_rtt_rejected, link.parser.bad_crc, link.tx_dropped))
        write('最终ToF_ID=%r 请求结果=%r 飞控重启累计=%d 时钟域重置累计=%d\n' % (
            link.range_id, link.last_ack, link.boot_generation, link.time_generation))
        write('连接/同步状态转换次数=%r（首次观察不计为转换）\n' % self.health_changes)
        write('状态变化/操作事件=%d 失跟段=%d 观察到轨迹重置=%d ORB尝试/成功=%d/%d\n' % (
            self.events, self.lost_episodes, self.reset_events, self.orb_attempts, self.orb_success))
        write('\n阶段耗时：次数；最小/平均/最大/最后(ms)；累计(ms)；占循环比例\n')
        cycle_total = self.phases[INDEX['cycle']].total
        for i, (name, label) in enumerate(PHASES):
            metric = self.phases[i]
            write('%s | %s | 累计=%.3fms 占循环=%.2f%%\n' % (
                label, metric.text(.001), metric.total*.001, 100.0*metric.total/max(1, cycle_total)))
        write('VO总处理包含视觉子项；维护包含GC，父子项不能相加。采图含等待，不等于CPU占用。\n')
        write('统计管理及未分项间隙=%.3fms；异常中断的未完成循环不纳入完整循环表。\n' % max(0, (now-self.start-cycle_total)*.001))
        write('\n=== 本次测试分析 ===\n')
        write('有效定位=%d/%d(%.2f%%)，可发布=%d。\n' % (
            self.valid_frames, self.frames, 100.0*self.valid_frames/max(1, self.frames), self.publish_frames))
        ranked = sorted(((self.phases[INDEX[n]].total, n) for n in (
            'poll_before', 'capture', 'poll_after', 'wait_attitude', 'camera', 'features', 'match',
            'track_fit', 'vision_poll', 'image_stats', 'vo_other', 'housekeeping', 'output', 'test_output')), reverse=True)
        ranked = [(total, name) for total, name in ranked if total > 0]
        if not ranked:
            write('没有可比较的完整阶段耗时，无法判断主要耗时项。\n')
        for total, name in ranked[:3]:
            write('耗时项：%s=%.2f%%完整循环\n' % (PHASES[INDEX[name]][1].strip(), 100.0*total/max(1, cycle_total)))
        if self.blockers:
            for name, count in sorted(self.blockers.items(), key=lambda item: item[1], reverse=True)[:3]:
                write('主要发布限制：%s，占图像帧%.2f%%。\n' % (BLOCK_NAMES.get(name, name), 100.0*count/max(1, self.frames)))
        write('输入平均接收/请求目标：姿态%.2f/%.2fHz，ToF %.2f/%.2fHz；平均值含启动和断流时段。\n' % (
            att_count/divisor, C.ATTITUDE_HZ, range_count/divisor, C.RANGE_HZ))
        write('输入异常依据：断连%.3fs 未同步%.3fs CRC错误%d RTT拒绝%d。\n' % (
            self.durations.get('DISCONNECTED', 0)*1e-6, self.durations.get('UNSYNCED', 0)*1e-6,
            link.parser.bad_crc, link.sync_rtt_rejected))
        write('恢复情况：失跟段%d ORB尝试/成功%d/%d 轨迹重置%d。\n' % (
            self.lost_episodes, self.orb_attempts, self.orb_success, self.reset_events))
        if not self.valid_frames:
            write('无有效定位，不能据此代表正常跟踪负载；先排查输入和视觉状态。\n')
        if not self.origin_id:
            write('未按Enter成功建立初始点，本次没有相对位置区间。\n')
        write('未记录图像或无限逐帧序列；最后统计覆盖本次全程，区间结束即写入以限制内存。\n')
        write('RTT为帧循环观察到的新同步回复样本；同一循环多个回复只保留最后一个RTT。\n')
        write('打印、事件写盘和计时会影响实测帧率；最终报告写盘发生在测量结束后。\n日志完成\n')

    def close(self, reason='x正常结束', error=None):
        if self.closed:
            return
        self.closed = True
        if self.link is None:
            # 相机/固件初始化失败也尽量留下原因，不伪造运行统计。
            if self.timing:
                self.log = self.log or SessionLog(self.log_directory if self.log_directory is not None else LOG_DIRECTORY)
                self.log.write('初始化失败：%s %s\n日志完成（未进入测试循环）\n' % (reason, error or ''))
                self.log.close()
                print('初始化日志=%s | 保存错误=%s' % (self.log.path, self.log.error or '无'))
            return
        now = self.link.clock.now_us()
        self.add_duration(now)
        if self.segment is not None and (self.last_result is None or now-self.last_result['timestamp_us'] > C.MAX_TX_AGE_US):
            self.segment['current_valid'] = False
        self.end_segment(now, reason)
        if self.timing:
            self.log = self.log or SessionLog(self.log_directory if self.log_directory is not None else LOG_DIRECTORY)
            self.log.flush()
            self.report(now, reason, error)
            self.log.close()
            if self.log.error:
                print('日志保存失败/不完整：%s | %s' % (self.log.path, self.log.error))
            else:
                print('测试结束，完整日志=%s' % self.log.path)
        else:
            print('测试结束：%s；MODE1不生成详细日志。' % reason)


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
        # 保存失败不能掩盖原始异常，也不向飞控发送测试观测。
        try:
            session.close(reason, error)
        except BaseException as save_error:
            print('最终日志保存失败：%r；结束原因=%s 原始错误=%s' % (save_error, reason, error))


if __name__ == '__main__':
    main()
