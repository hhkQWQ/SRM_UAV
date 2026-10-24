"""PC regression checks for event UI, origin epochs, dry-run rates and final logs.
Run from project: python3 -B native/test_debug.py
No board, UART or native module is needed for these scheduling checks.
"""
import contextlib
import io
from pathlib import Path
import sys
import tempfile
import unittest
import struct
from types import SimpleNamespace
from unittest.mock import patch

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
import test as debug
import config as C
from output import Output, PublicationGate, publication_blockers
from comm import FlightLink, History
import main as runtime


class Clock:
    now = 100000
    def now_us(self): return self.now


class Link:
    def __init__(self):
        self.clock = Clock()
        self.attitudes, self.ranges = History(), History()
        self.attitude_received = self.range_received = self.sync_received = 0
        self.last_sync_rtt_us = None
        self.sync_count = self.sync_rtt_rejected = self.tx_dropped = 0
        self.boot_generation = self.time_generation = 0
        self.last_ack = self.range_id = None
        self.parser = SimpleNamespace(bad_crc=0)
        self.sent, self.cancelled = [], 0
        self.connected_value = self.synced_value = True
    def connected(self, now=None): return self.connected_value
    def synced(self, now=None): return self.synced_value
    def fc_time(self, stamp): return stamp
    def send_odometry(self, result): self.sent.append(result['timestamp_us'])
    def cancel_odometry(self): self.cancelled += 1


class Input:
    def __init__(self): self.actions = []
    def poll(self):
        value, self.actions = self.actions, []
        return value


def result(now, position=(0, 0), valid=True, reset=0):
    return dict(timestamp_us=now, position=position if valid else None,
        valid=valid, body_valid=valid, approximate=False, reason='TRACKING' if valid else 'LOST',
        q=(1, 0, 0, 0), height=1.5, quality=80 if valid else 0, reset_counter=reset,
        keyframe=1, matches=30, inliers=25, residual_px=.5, residual_yaw=.01, attitude_age_us=1000,
        range_age_us=2000, range_state='OK', range_detail=None, variance=(.01, .02),
        vision=dict(keypoints=48, matches=30, inliers=25, cells=6, spread=.2,
                    klt=25, sad=0, search_px=6, reference='key', orb=False))


class DebugChecks(unittest.TestCase):
    def setUp(self):
        self.saved = C.SENSOR_TIMING_CONFIRMED, C.SEND_ODOMETRY
        C.SENSOR_TIMING_CONFIRMED = C.SEND_ODOMETRY = True
        self.folder = tempfile.TemporaryDirectory()
        self.input, self.link = Input(), Link()
        self.sensor = SimpleNamespace(get_exposure_us=lambda: 6000, get_gain_db=lambda: 18)
        self.stdout = io.StringIO()
        self.redirect = contextlib.redirect_stdout(self.stdout)
        self.redirect.__enter__()
        self.sessions = []

    def tearDown(self):
        for session in self.sessions:
            session.close()
        self.redirect.__exit__(None, None, None)
        self.folder.cleanup()
        C.SENSOR_TIMING_CONFIRMED, C.SEND_ODOMETRY = self.saved

    def session(self, mode=2):
        session = debug.TestSession(mode, self.input, self.folder.name)
        self.sessions.append(session)
        output = Output(self.link)
        session.prepare(self.sensor, self.link, None, None, None, output)
        self.assertFalse(output.publish)
        self.assertFalse(output.display)
        return session

    def frame(self, session, position=(0, 0), valid=True, reset=0, actions=(), dt=50000):
        self.link.clock.now += dt
        now = self.link.clock.now
        self.link.attitudes.append((now, (1, 0, 0, 0), (0, 0, 0), now))
        self.link.ranges.append((now, 1.5, .1, 8, 100, None, now))
        self.link.attitude_received += 1
        self.link.range_received += 1
        self.input.actions = list(actions)
        session.begin_frame()
        session.record('vo_total', 0)
        return session.finish_frame(result(now, position, valid, reset), 99)

    def test_quiet_query_and_modes(self):
        for mode in (1, 2):
            session = self.session(mode)
            self.frame(session)
            initial = self.stdout.getvalue()
            self.assertIn('调试版本='+debug.DEBUG_VERSION, initial)
            for _ in range(60): self.frame(session, position=(1, 2))
            self.assertEqual(initial, self.stdout.getvalue(), 'stable numeric changes must stay quiet')
            self.frame(session, actions=('query',))
            self.assertEqual(self.stdout.getvalue().count('状态='), initial.count('状态=')+1)
            self.assertFalse(session.relative(session.last_result))
            self.assertFalse(self.link.sent)
            self.assertEqual(bool(session.phases), mode == 2)
            session.close()

    def test_origins_pending_rebase_reset_and_whole_run(self):
        session = self.session()
        self.frame(session, valid=False, actions=('origin',))
        self.assertTrue(session.pending_origin)
        self.assertIsNone(session.origin)
        self.frame(session, (10, 20))
        self.assertEqual(session.origin, (10, 20))
        self.frame(session, (11, 18))
        self.assertEqual(session.relative(session.last_result), (1, -2))
        count = session.frames
        self.frame(session, (12, 19), actions=('origin',))
        self.assertEqual(session.origin_id, 2)
        self.assertGreater(session.frames, count)
        self.assertEqual(session.relative(session.last_result), (0, 0))
        self.frame(session, valid=False, reset=1)
        self.assertIsNone(session.origin)
        self.assertIsNone(session.segment)
        session.close()
        text = Path(session.log.path).read_text()
        self.assertIn('初始点区间1', text)
        self.assertIn('最后有效ΔN/ΔE=(1, -2)', text)
        self.assertIn('初始点区间2', text)
        self.assertIn('算法轨迹重置', text)
        self.assertIn('图像=5 有效=3', text)
        self.assertIn('日志完成', text)

    def test_gate_matches_formal_output_and_limit(self):
        output = Output(self.link, display=False)
        gate = PublicationGate()
        simulated = 0
        for i in range(300):
            self.link.clock.now += 10000
            item = result(self.link.clock.now)
            if i % 9 == 0: item['quality'] = 0
            if i % 11 == 0: item['timestamp_us'] -= C.MAX_TX_AGE_US+1
            blockers = publication_blockers(item, self.link, self.link.clock.now)
            simulated += int(gate.accept(item, self.link.clock.now, blockers))
            output.update(item, 0)
            output.update(item, 0)  # duplicate must not publish
        self.assertEqual(simulated, len(self.link.sent))
        self.assertLessEqual(simulated, 3*C.ODOMETRY_HZ)
        self.assertEqual(len(self.link.sent), len(set(self.link.sent)))

    def test_rate_window_is_not_query_interval(self):
        session = self.session()
        for _ in range(200): self.frame(session, dt=10000)
        rates = session.rates
        self.assertAlmostEqual(rates[1], 100)
        self.assertLessEqual(rates[2], C.ODOMETRY_HZ)
        self.frame(session, actions=('query',), dt=10000)
        self.assertEqual(session.rates, rates)
        self.frame(session, actions=('origin',), dt=10000)
        self.assertEqual(session.rates, (0, 0, 0))
        self.assertEqual(session.frames, 202)

    def test_status_coalescing_and_current_stale(self):
        session = self.session()
        self.frame(session)
        lines = self.stdout.getvalue().count('状态=')
        self.frame(session, valid=False)
        self.frame(session)
        self.assertEqual(lines, self.stdout.getvalue().count('状态='))
        for _ in range(20): self.frame(session)
        self.assertEqual(lines+1, self.stdout.getvalue().count('状态='))
        self.link.clock.now += C.MAX_TX_AGE_US+1
        session.summary(session.last_result, self.link.clock.now)
        self.assertIn('ΔN=-- ΔE=--', self.stdout.getvalue().splitlines()[-1])
        self.assertIn('当前无新鲜观测', self.stdout.getvalue().splitlines()[-1])

    def test_keyboard_crlf_and_exit(self):
        console = debug.Console.__new__(debug.Console)
        console.after_cr = False
        self.assertEqual(console.commands(b'\r'), ['origin'])
        self.assertEqual(console.commands(b'\nq'), ['query'])
        self.assertEqual(console.commands(b'\nx'), ['origin', 'exit'])
        session = self.session()
        self.assertTrue(self.frame(session, actions=('exit',)))

    def test_usb_console_boolean_any_and_nonblocking_recv(self):
        class VCP:
            data = b'\r\nqx'
            calls = []
            def debug_mode_enabled(self): return False
            def any(self): return bool(self.data)  # 与板上固件一致，不返回字节数。
            def recv(self, buffer, *, timeout):
                if not isinstance(buffer, bytearray):
                    raise TypeError('object with buffer protocol required')
                size = len(buffer)
                self.calls.append((size, timeout))
                value, self.data = self.data[:size], self.data[size:]
                buffer[:len(value)] = value
                return len(value)
        vcp = VCP()
        with patch.dict(sys.modules, {'pyb': SimpleNamespace(USB_VCP=lambda: vcp)}):
            console = debug.Console()
        self.assertEqual(console.poll(), ['origin', 'query', 'exit'])
        self.assertEqual(vcp.calls, [(64, 0)])
        self.assertEqual(console.poll(), [])
        self.assertEqual(vcp.calls, [(64, 0)], 'no input must not call recv')

    def test_usb_console_bounds_reads_and_keeps_remaining_input(self):
        class VCP:
            data = b'q'*65
            def debug_mode_enabled(self): return False
            def any(self): return bool(self.data)
            buffers = []
            def recv(self, buffer, *, timeout):
                if not isinstance(buffer, bytearray):
                    raise TypeError('object with buffer protocol required')
                self.buffers.append(buffer)
                size = len(buffer)
                value, self.data = self.data[:size], self.data[size:]
                buffer[:len(value)] = value
                return len(value)
        vcp = VCP()
        with patch.dict(sys.modules, {'pyb': SimpleNamespace(USB_VCP=lambda: vcp)}):
            console = debug.Console()
        self.assertEqual(console.poll(), ['query']*64)
        self.assertEqual(console.poll(), ['query'])
        self.assertEqual(console.poll(), [])
        self.assertIs(vcp.buffers[0], vcp.buffers[1], 'USB input must reuse its buffer')

    def test_usb_console_empty_recv_does_not_replay_old_keys(self):
        class VCP:
            reads = 0
            def debug_mode_enabled(self): return False
            def any(self): return True
            def recv(self, buffer, *, timeout):
                self.reads += 1
                if self.reads == 1:
                    buffer[0] = ord('q')
                    return 1
                return 0
        vcp = VCP()
        with patch.dict(sys.modules, {'pyb': SimpleNamespace(USB_VCP=lambda: vcp)}):
            console = debug.Console()
        self.assertEqual(console.poll(), ['query'])
        self.assertEqual(console.poll(), [])
        self.assertEqual(console.poll(), [])

    def test_metric_and_bounded_buffer_all_segments(self):
        metric = debug.Metric()
        for value in (1, 3, None, float('nan')): metric.add(value)
        self.assertEqual((metric.count, metric.invalid, metric.total), (2, 2, 4))
        session = self.session()
        for i in range(40):
            self.frame(session, (i, i+1), actions=('origin',))
            self.assertLess(len(session.log.pending), debug.EVENT_BUFFER_SIZE)
        session.close()
        text = Path(session.log.path).read_text()
        self.assertEqual(text.count('初始点区间'), 40)
        self.assertIn('图像=40 有效=40', text)
        self.assertIn('最小/平均/最大/最后', text)
        self.assertIn('未执行', text)
        self.assertNotIn('水平距离', text)
        session.close()  # idempotent

    def test_log_unique_paths(self):
        a, b = debug.SessionLog(self.folder.name), debug.SessionLog(self.folder.name)
        self.assertNotEqual(a.path, b.path)
        a.close(); b.close()

    def test_interrupt_and_error_finalize(self):
        original_run, original_session = debug.run, debug.TestSession
        for exception in (KeyboardInterrupt(), ValueError('synthetic failure')):
            session = self.session()
            def fake_run(observer):
                self.frame(observer, actions=('origin',))
                raise exception
            debug.run = fake_run
            debug.TestSession = lambda mode: session
            try:
                if isinstance(exception, KeyboardInterrupt): debug.main()
                else:
                    with self.assertRaises(ValueError): debug.main()
            finally:
                debug.run, debug.TestSession = original_run, original_session
            text = Path(session.log.path).read_text()
            self.assertIn('日志完成', text)
            self.assertIn('KeyboardInterrupt中断' if isinstance(exception, KeyboardInterrupt) else 'synthetic failure', text)

    def test_initialization_failure_report(self):
        session = debug.TestSession(2, self.input, self.folder.name)
        session.close('程序异常', 'missing rm_vo')
        text = Path(session.log.path).read_text()
        self.assertIn('初始化失败', text)
        self.assertIn('missing rm_vo', text)

    def test_receive_counters_survive_history_wrap_and_fc_restart(self):
        link = FlightLink(uart=object(), clock=self.link.clock)
        link.target = (1, C.TARGET_COMPONENT)
        for i in range(130):
            payload = struct.pack('<Ifffffff', 5000+i*20, 1, 0, 0, 0, 0, 0, 0)
            link._receive(31, 1, C.TARGET_COMPONENT, payload, 100000+i*20000)
        self.assertEqual(link.attitude_received, 130)
        self.assertEqual(len(link.attitudes), C.SENSOR_HISTORY_SIZE)
        link._receive(31, 1, C.TARGET_COMPONENT, struct.pack('<Ifffffff', 100, 1, 0, 0, 0, 0, 0, 0), 4000000)
        self.assertEqual(link.boot_generation, 1)
        self.assertEqual(link.attitude_received, 131)
        payload = struct.pack('<IHHHBBBB', 100, 10, 800, 150, 0, 2, C.TOF_ORIENTATION, 255)
        link._receive(132, 1, C.TARGET_COMPONENT, payload, 4000000)
        self.assertEqual(link.range_received, 1)
        link._restart()
        self.assertEqual(link.range_received, 1)

    def test_actual_main_loop_observer_and_exit(self):
        for mode in (1, 2):
            session = debug.TestSession(mode, self.input, self.folder.name)
            self.sessions.append(session)
            inputs = iter([['origin'], [], ['query'], ['exit']])
            self.input.poll = lambda: next(inputs)
            link = self.link
            calls = []
            def poll(service_sync=False): calls.append(service_sync)
            link.poll = poll
            def snapshot():
                link.clock.now += 50000
                now = link.clock.now
                link.attitudes.append((now, (1, 0, 0, 0), (0, 0, 0), now))
                return object()
            self.sensor.snapshot = snapshot
            engine = SimpleNamespace(predicted_pixel_speed=lambda *args: 0)
            controller = SimpleNamespace(engine=engine,
                process=lambda image, stamp, link, statistics=None: result(stamp))
            adapter = SimpleNamespace(update=lambda *args: None)
            with patch.object(runtime, 'setup_camera', return_value=self.sensor), \
                 patch.object(runtime, 'FlightLink', return_value=link), \
                 patch.object(runtime, 'VOController', return_value=controller), \
                 patch.object(runtime, 'CameraAdapter', return_value=adapter):
                runtime.run(session)
            session.close()
            self.assertEqual(session.frames, 4)
            self.assertEqual(session.origin_id, 1)
            self.assertEqual(engine.probe is not None, mode == 2)
            self.assertEqual(len(calls), 8)
            self.assertFalse(link.sent)
            if mode == 2:
                self.assertEqual(session.phases[debug.INDEX['cycle']].count, 4)
                self.assertIn('日志完成', Path(session.log.path).read_text())


if __name__ == '__main__':
    unittest.main()
