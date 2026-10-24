"""PC checks for logging only: python -B verify_logging.py (no camera/UART/C build)."""
import contextlib
import io
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))
import test as logging
import main as entry
import config as C
from output import Output, publication_block_mask, publication_blockers, PublicationGate


class Clock:
    def __init__(self): self.now = 1000000
    def now_us(self): return self.now


class Link:
    def __init__(self):
        self.clock = Clock()
        self.up = self.sync = True
        self.attitude_received = self.range_received = self.sync_rtt_rejected = 0
        self.parser = types.SimpleNamespace(bad_crc=0)
        self.attitudes = [(0, (1, 0, 0, 0), (0, 0, 0), 0)]
        self.sent = 0
    def connected(self, now=None): return self.up
    def synced(self, now=None): return self.sync
    def poll(self, service_sync=False): self.clock.now += 1000
    def cancel_odometry(self): pass
    def send_odometry(self, result):
        self.sent += 1
        raise AssertionError('test mode submitted ODOMETRY')


class Console:
    def __init__(self, actions=()): self.actions = list(actions)
    def poll(self): return self.actions.pop(0) if self.actions else ()


def observation(stamp, valid=True, reason=None, quality=80, **extra):
    result = dict(timestamp_us=stamp, valid=valid, body_valid=True,
        reason=reason or ('TRACKING' if valid else 'LOST'), quality=quality,
        approximate=False, reset_counter=0, position=(1.0, 2.0), height=1.5,
        matches=12, inliers=10, residual_px=.5, vision={'sparse': False})
    result.update(extra)
    return result


class LoggingChecks(unittest.TestCase):
    def setUp(self):
        self.stdout = io.StringIO()
        self.output_capture = contextlib.redirect_stdout(self.stdout)
        self.output_capture.__enter__()
        self.addCleanup(self.output_capture.__exit__, None, None, None)
        for name, value in (('SENSOR_TIMING_CONFIRMED', True), ('SEND_ODOMETRY', True)):
            setting = patch.object(C, name, value)
            setting.start()
            self.addCleanup(setting.stop)
        self.folder = tempfile.TemporaryDirectory(prefix='_logging_check_', dir=ROOT)
        self.assertEqual(Path(self.folder.name).resolve().parent, ROOT)
        self.addCleanup(self.folder.cleanup)

    def session(self, mode=1, actions=()):
        session = logging.TestSession(mode, Console(actions), self.folder.name)
        link, output = Link(), Output(Link())
        output.link = link
        session.prepare(None, link, None, None, None, output)
        self.assertFalse(output.publish)
        self.assertFalse(output.display)
        return session

    def feed(self, session, valid=True, reason=None, quality=80, step=100000, stamp=None, **extra):
        session.begin_frame()
        result = observation(session.link.clock.now if stamp is None else stamp,
                             valid, reason, quality, **extra)
        session.link.clock.now += step
        session.finish_frame(result, 0)
        return result

    def test_counts_quality_dedup_and_loss_recovery(self):
        session = self.session()
        self.feed(session, False, 'INITIALIZING', 0)
        first = self.feed(session, quality=40)
        self.feed(session, quality=100, stamp=first['timestamp_us'])  # duplicate: ignored
        self.feed(session, quality=80)
        self.feed(session, False, 'LOST', 0)
        self.feed(session, False, 'POSE_RECOVERING', 0)
        self.feed(session, quality=60)
        result = session.snapshot()
        self.assertEqual((result['frames'], result['stable'], result['publishable']), (6, 3, 3))
        self.assertEqual(result['quality'], 60)
        self.assertEqual(result['valid_ratio'], 50)
        self.assertAlmostEqual(result['fps'], 6/result['seconds'])
        self.assertAlmostEqual(result['publish_hz'], 3/result['seconds'])
        self.assertEqual((session.lost_episodes, session.initial_frames, session.recovery_frames), (1, 1, 1))
        self.assertEqual(session.loss_counts[logging.FAILURE_INDEX['LOST']], 1)
        session.close()
        text = self.stdout.getvalue()
        self.assertIn('原因未细分', text)
        self.assertNotIn('有效定位=', text)
        self.assertNotIn('横滚', text)
        self.assertIsNone(session.phase_counts)
        self.assertIsNone(session.events)
        self.assertFalse(list(Path(self.folder.name).iterdir()))

    def test_sequence_dedup_warmup_and_input_causes(self):
        session = self.session()
        self.feed(session, False, 'WAIT_ATTITUDE', 0, frame_sequence=1)
        self.feed(session, frame_sequence=2)
        self.feed(session, frame_sequence=2)  # same image, different timestamp
        self.feed(session, False, 'RANGE_INVALID', 0, frame_sequence=3)
        self.feed(session, False, 'RANGE_RECOVERING', 0, frame_sequence=4)
        self.feed(session, frame_sequence=5)
        self.assertEqual(session.frames, 5)
        self.assertEqual(session.lost_episodes, 1)
        self.assertEqual(session.failure_counts[logging.FAILURE_INDEX['WAIT_ATTITUDE']], 1)
        self.assertEqual(session.loss_counts[logging.FAILURE_INDEX['WAIT_ATTITUDE']], 0)
        self.assertEqual(session.loss_counts[logging.FAILURE_INDEX['RANGE_INVALID']], 1)

    def test_gate_limits_and_all_publication_checks(self):
        session = self.session(2)
        self.feed(session, step=1000)
        self.feed(session, step=1000)  # below 30Hz rate interval
        self.feed(session, step=40000)
        self.assertEqual((session.valid_frames, session.publish_frames), (3, 2))
        self.feed(session, quality=10)
        self.feed(session, body_valid=False)
        self.feed(session, step=C.MAX_TX_AGE_US+1)
        session.link.up = False
        self.feed(session)
        session.link.up, session.link.sync = True, False
        self.feed(session)
        with patch.object(C, 'SENSOR_TIMING_CONFIRMED', False):
            self.feed(session)
        self.assertEqual(session.publish_frames, 2)
        for index in (0, 2, 3, 4, 5, 6):
            self.assertGreater(session.block_counts[index], 0)
        self.assertEqual(session.link.sent, 0)
        sample = observation(session.link.clock.now, quality=float('nan'))
        self.assertTrue(publication_block_mask(sample, session.link, session.link.clock.now) & 8)
        self.assertIn('LOW_QUALITY', publication_blockers(sample, session.link, session.link.clock.now))

    def test_no_runtime_file_io_and_bounded_events(self):
        session = self.session(2, (('origin',), ('query',)))
        with patch('builtins.open', side_effect=AssertionError('runtime file open')):
            with patch.object(logging.os, 'mkdir', side_effect=AssertionError('runtime mkdir')):
                for i in range(45):
                    session.link.sync = i % 2 == 0
                    self.feed(session, i % 3 == 0, 'TRACKING' if i % 3 == 0 else 'LOST')
        self.assertIsNone(session.log)
        self.assertEqual(session.events.count, logging.EVENT_BUFFER_SIZE)
        self.assertGreater(session.events.overwritten, 0)
        measured = session.snapshot()
        session.close()
        self.assertEqual(session.snapshot(), measured)
        files = list(Path(self.folder.name).glob('vo_test_*.txt'))
        self.assertEqual(len(files), 1)
        text = files[0].read_text(encoding='utf-8')
        self.assertIn('日志完成', text)
        self.assertIn('可发布为准入+限频模拟', text)
        self.assertIn('发布受阻帧', text)
        self.assertIn('被覆盖记录=', text)
        for obsolete in ('yaw_deg', 'variance_N', 'search_px', '最终观测=', '区间开始', '横滚'):
            self.assertNotIn(obsolete, text)

    def test_event_merging_and_64_bit_time(self):
        events = logging.EventBuffer(2)
        events.add((1 << 32)+3, 0, 1)
        events.add((1 << 32)+4, 0, 1)
        self.assertEqual((events.count, events.total, events.repeats[0]), (1, 2, 2))
        events.add((1 << 32)+5, 1, 0)
        events.add((1 << 32)+6, 4, 2)
        self.assertEqual((events.count, events.overwritten), (2, 1))
        lines = ''.join(events.lines())
        self.assertIn('4294967.', lines)
        self.assertIn('轨迹重置', lines)

    def test_summary_hides_reasons_and_origin_reset_preserves_totals(self):
        session = self.session(actions=(('origin',), (), (), ('query',)))
        self.feed(session)
        self.assertEqual(session.origin, (1, 2))
        self.feed(session, False, 'WAIT_ATTITUDE', 0)
        self.feed(session, False, 'REINITIALIZING', 0, reset_counter=1)
        self.assertIsNone(session.origin)
        self.feed(session, reset_counter=1)
        self.assertEqual((session.frames, session.valid_frames, session.reset_events), (4, 2, 1))
        live = self.stdout.getvalue()
        self.assertNotIn('图像时刻缺少姿态', live)
        self.assertNotIn('有效定位=', live)
        self.assertIn('可发布=', live)
        session.close()
        self.assertIn('图像时刻缺少姿态', self.stdout.getvalue())

    def test_empty_initial_failure_and_zero_stable_frames(self):
        for mode in (1, 2):
            session = self.session(mode)
            session.close()
            self.assertIsNone(session.snapshot()['fps'])
            self.assertIsNone(session.snapshot()['quality'])
            session.close()  # idempotent
        session = self.session()
        self.feed(session, False, 'NO_FEATURES', 0)
        self.assertEqual(session.lost_episodes, 0)
        session.close()
        self.assertIn('首次定位受阻原因', self.stdout.getvalue())
        self.assertIn('无有效样本', self.stdout.getvalue())
        failed = logging.TestSession(2, Console(), self.folder.name)
        failed.close('程序异常', 'camera setup failed')
        self.assertIn('camera setup failed', self.stdout.getvalue())

    def test_measurement_excludes_startup_save_and_partial_visual_cycle(self):
        session = self.session(2)
        session.link.clock.now += 9000000  # startup is outside measurement
        self.feed(session)
        saved = session.snapshot()
        session.begin_frame()
        session.link.clock.now += 10000000  # exception before a result/finish_frame
        session.close('程序异常', 'incomplete visual call')
        self.assertEqual(session.snapshot(), saved)
        self.assertEqual(session.phase_counts[logging.INDEX['cycle']], 1)

    def test_console_exception_still_finalizes_counted_result(self):
        session = self.session(2)
        session.console.poll = lambda: (_ for _ in ()).throw(RuntimeError('console failure'))
        with self.assertRaisesRegex(RuntimeError, 'console failure'):
            self.feed(session)
        self.assertEqual(session.frames, 1)
        self.assertGreater(session.snapshot()['seconds'], 0)
        self.assertEqual(session.phase_counts[logging.INDEX['cycle']], 1)
        session.close('程序异常', 'console failure')

    def test_save_errors_do_not_claim_completion_or_hide_original_error(self):
        session = self.session(2)
        self.feed(session)
        with patch.object(logging, 'SessionLog', side_effect=OSError('disk full')):
            session.close('程序异常', 'original failure')
        text = self.stdout.getvalue()
        self.assertIn('original failure', text)
        self.assertIn('日志保存失败', text)
        self.assertNotIn('保存状态=完成', text)

    def test_usb_commands_and_empty_input(self):
        console = object.__new__(logging.Console)
        console.after_cr = False
        self.assertEqual(console.commands(b''), ())
        self.assertEqual(console.commands(b'\r\nqX'), ['origin', 'query', 'exit'])

    def test_write_failure_is_reported_and_original_exception_survives(self):
        class BrokenFile:
            closed = False
            def write(self, data): raise OSError('write failed')
            def close(self): self.closed = True
        session = self.session(2)
        self.feed(session)
        file = BrokenFile()
        with patch.object(logging, 'is_directory', return_value=True), \
                patch.object(logging.os, 'stat', side_effect=FileNotFoundError), \
                patch('builtins.open', return_value=file):
            session.close('程序异常', 'original error')
        self.assertTrue(file.closed)
        self.assertIn('write failed', session.log.error)
        self.assertNotIn('保存状态=完成', self.stdout.getvalue())
        with patch.object(logging, 'run', side_effect=ValueError('vision error')), \
                patch.object(logging, 'MODE', 2), \
                patch.object(logging, 'SessionLog', side_effect=OSError('disk error')):
            with self.assertRaisesRegex(ValueError, 'vision error'):
                logging.main()

    def test_frontend_reuses_diagnostics_profiles_native_and_never_calls_orb(self):
        import algorithm as algorithm
        class Core:
            status = 1
            def __init__(self, settings): pass
            def reset(self): pass
            def profile(self, enabled): self.enabled = enabled
            def timings(self, out):
                for i in range(6): out[i] = i+1
            def step(self, image, meta, stamp, prediction, out):
                for i in range(len(out)): out[i] = 0
                out[0], out[15], out[17] = self.status, int(self.status == 1), 1.5
                out[28] = 1 if self.status == 0 else 0
        class Image:
            def width(self): return 320
            def height(self): return 240
            def format(self): return 1
            def find_keypoints(self, **kwargs): raise AssertionError('ORB was called')
        module = types.SimpleNamespace(GRAYSCALE=1, CORNER_FAST=0, CORNER_AGAST=1)
        samples = []
        probe = types.SimpleNamespace(native=lambda values: samples.append(tuple(values)))
        with patch.dict(sys.modules, rm_vo=types.SimpleNamespace(Core=Core, ABI_VERSION=2), image=module):
            engine = algorithm.FeatureEngine(probe)
            diagnostics = engine.diagnostics
            first = engine.process(Image(), (1, 0, 0, 0), 1.5, (0, 0, 0), 100000)
            engine.core.status = 2
            second = engine.process(Image(), (1, 0, 0, 0), 1.5, (0, 0, 0), 133333)
            engine.core.status = 0
            failed = engine.process(Image(), (1, 0, 0, 0), 1.5, (0, 0, 0), 166666)
        self.assertEqual((first['status'], second['status'], failed['status']), (1, 2, 0))
        self.assertIs(engine.diagnostics, diagnostics)
        self.assertEqual(len(diagnostics), 2)
        self.assertEqual(diagnostics['vision_reason'], 'MATCH_COUNT')
        self.assertEqual(len(samples), 3)
        self.assertEqual(samples[0], (1,2,3,4,5,6))
        self.assertTrue(engine.core.enabled)

    def test_sparse_tracking_remains_valid_and_is_not_a_loss(self):
        session = self.session(2)
        self.feed(session)
        before = session.reset_events
        sparse = self.feed(session, reason='SPARSE_TRACKING', quality=15, sparse=True, inliers=2)
        self.assertEqual(session.state(sparse, session.link.clock.now), 3)
        self.feed(session, reason='SPARSE_TRACKING', quality=25, sparse=True, inliers=3)
        self.feed(session)
        self.assertEqual((session.valid_frames, session.sparse_frames, session.lost_episodes), (4,2,0))
        self.assertEqual(session.reset_events, before)
        self.assertEqual(session.publish_frames, 2)
        self.assertGreater(session.block_counts[3], 0)
        session.close()
        text = next(Path(self.folder.name).glob('vo_test_*.txt')).read_text(encoding='utf-8')
        self.assertIn('少点有效跟踪=2帧', text)

    def test_shared_main_loop_in_both_modes_never_transmits(self):
        class Engine:
            def predicted_pixel_speed(self, height, stamp): return 0
        class Controller:
            def __init__(self): self.engine = Engine()
            def process(self, image, stamp, link, statistics=None): return observation(stamp)
        class Capture:
            def __init__(self, sensor, clock): self.clock, self.last_sequence = clock, 0
            def get(self):
                self.clock.now += 40000
                self.last_sequence += 1
                return object(), self.clock.now
        class Adapter:
            def __init__(self, *args): pass
            def update(self, *args): return None
        for mode in (1, 2):
            link = Link()
            session = logging.TestSession(mode, Console(((), ('origin',), ('query',), ('exit',))), self.folder.name)
            with patch.multiple(entry, setup_camera=lambda: None, FlightLink=lambda: link,
                    VOController=Controller, Capture=Capture, CameraAdapter=Adapter,
                    wait_for_attitude=lambda *args: None):
                entry.run(session)
            self.assertEqual((session.frames, session.valid_frames), (4, 4))
            self.assertEqual(link.sent, 0)
            if mode == 2:
                self.assertEqual(session.phase_counts[logging.INDEX['cycle']], 4)
                self.assertEqual(session.phase_counts[logging.INDEX['vision']], 4)
            session.close()


if __name__ == '__main__':
    unittest.main(verbosity=2)
