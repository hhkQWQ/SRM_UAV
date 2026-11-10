"""OpenMV入口。正常运行main.py；事件摘要与全程测试日志运行test.py。

只显示位姿、离地高度、帧率和置信度摘要。默认不回传有效ODOMETRY。
"""
import gc
import math
import time
import config as C
from comm import FlightLink
from control import VOController, attitude_at
from output import Output
from algorithm import _us, _elapsed


def validate_config():
    if C.STREAM_POLICY not in ("auto", "listen", "request"):
        raise ValueError("invalid STREAM_POLICY")
    if (C.IMAGE_WIDTH, C.IMAGE_HEIGHT) != (320, 240):
        raise ValueError("native VO requires calibrated QVGA")
    if C.FRAME_BUFFERS != 1:
        raise ValueError("4.5.9 requires one capture buffer for unambiguous image timestamps")
    if not C.MIN_MATCHES <= C.MAX_KEYPOINTS <= 64:
        raise ValueError("native feature capacity is 64")
    if any(v <= 0 for v in (C.ATTITUDE_HZ, C.RANGE_HZ, C.ODOMETRY_HZ, C.DISPLAY_HZ)):
        raise ValueError("stream and output frequencies must be positive")
    if not 0 < C.EXPOSURE_MIN_US <= C.EXPOSURE_MAX_US or C.MAX_BLUR_PX <= 0:
        raise ValueError("invalid exposure or blur settings")
    if C.MAX_KEYPOINTS < C.MIN_MATCHES or not 2 <= C.MIN_INLIERS <= C.MIN_MATCHES:
        raise ValueError("invalid minimum/maximum feature counts")
    if not 1 <= C.SEARCH_MIN_PX <= C.SEARCH_MAX_PX <= 8:
        raise ValueError("local search must stay within eight pixels")
    if (not 1 <= C.FEATURE_TOPUP_CELLS <= 24 or not 0 <= C.FEATURE_TOPUP_POINTS <= 64
            or not 1 <= C.KEYFRAME_CHECK_INTERVAL <= 1000 or C.SPARSE_MAX_INLIERS != 3 or not .5 < C.MIN_INLIER_RATIO <= 1
            or not 0 < C.SAD_UNIQUENESS_RATIO < 1):
        raise ValueError("invalid tracking work budget/consensus")
    if (not 1 <= C.FEATURE_CELL_MAX_POINTS <= 3 or not 3 <= C.FEATURE_MIN_DISTANCE_PX <= 12
            or not 1 <= C.WEAK_CORNER_STRIDE_PX <= 8 or not 1 <= C.INIT_FULL_SCAN_INTERVAL <= 100):
        raise ValueError("invalid weak-feature detection settings")
    if (not 0 < C.TRACK_HOLD_US < C.LOST_RESET_US or not 2 <= C.PREDICTION_HISTORY_SIZE <= 16
            or not 1 <= C.CONFIDENCE_FULL_POINTS <= 64 or not 1 <= C.CONFIDENCE_FULL_CELLS <= 9
            or not math.isfinite(C.CONFIDENCE_SIGMA0_M) or C.CONFIDENCE_SIGMA0_M <= 0
            or not math.isfinite(C.PREDICTION_VELOCITY_STD_M_S) or C.PREDICTION_VELOCITY_STD_M_S <= 0):
        raise ValueError("invalid confidence/prediction parameters")
    r = C.R_BC
    if len(r) != 9:
        raise ValueError("R_BC must contain 9 values")
    for i in range(3):
        for j in range(3):
            dot = sum(r[k*3+i]*r[k*3+j] for k in range(3))
            if abs(dot-(1.0 if i == j else 0.0)) > 1e-4:
                raise ValueError("R_BC must be a rotation, not a mirror")
    det = (r[0]*(r[4]*r[8]-r[5]*r[7])-r[1]*(r[3]*r[8]-r[5]*r[6])
           +r[2]*(r[3]*r[7]-r[4]*r[6]))
    if abs(det-1.0) > 1e-4:
        raise ValueError("R_BC determinant must be +1")
    if abs(sum(v*v for v in C.TOF_AXIS_B)-1.0) > 1e-4:
        raise ValueError("TOF_AXIS_B must be a unit vector")
    for pos in (C.CAMERA_POS_B_M, C.TOF_POS_B_M):
        if pos is not None and (len(pos) != 3 or not all(math.isfinite(v) for v in pos)):
            raise ValueError("sensor position must be None or 3 finite metres")
    if C.HMIRROR or C.VFLIP:
        raise ValueError("restore calibration image direction or recalibrate before enabling flips")


def setup_camera():
    import sensor
    sensor.reset()
    sensor.set_pixformat(sensor.GRAYSCALE)
    sensor.set_framesize(sensor.QVGA)
    sensor.set_framebuffers(C.FRAME_BUFFERS)
    sensor.set_hmirror(C.HMIRROR)
    sensor.set_vflip(C.VFLIP)
    # 在首次稳定等待前应用曝光，避免自动曝光先拉长帧周期。
    if C.EXPOSURE_US is None:
        sensor.set_auto_exposure(True)
    else:
        sensor.set_auto_exposure(False, exposure_us=C.EXPOSURE_US)
    if C.GAIN_DB is None:
        if C.GAIN_CEILING_DB is None:
            sensor.set_auto_gain(True)
        else:
            sensor.set_auto_gain(True, gain_db_ceiling=C.GAIN_CEILING_DB)
    else:
        sensor.set_auto_gain(False, gain_db=C.GAIN_DB)
    sensor.skip_frames(time=C.CAMERA_SETTLE_MS)
    if C.AUTOFOCUS_ON_START:
        try:
            sensor.ioctl(sensor.IOCTL_TRIGGER_AUTO_FOCUS)
            sensor.ioctl(sensor.IOCTL_WAIT_ON_AUTO_FOCUS, 5000)
            sensor.skip_frames(time=500)
        except (AttributeError, OSError) as error:
            print("相机对焦未完成，请检查清晰度：%s" % error)
    # 不设置帧率上限；真实帧率在循环中统计。
    return sensor


class Capture:
    def __init__(self, sensor, clock):
        self.sensor = sensor
        self.clock = clock
        self.done_ticks = None
        self.done_sequence = 0
        self.last_sequence = -1
        if C.CAMERA_TIMESTAMP_MODE == "frame_callback":
            # 回调只记录时间/编号，不进行串口、打印、图像处理。
            sensor.set_frame_callback(self._done)
        elif C.CAMERA_TIMESTAMP_MODE != "snapshot_return":
            raise ValueError("invalid CAMERA_TIMESTAMP_MODE")

    def _done(self):
        self.done_ticks = time.ticks_us()
        self.done_sequence = (self.done_sequence+1) & 0x3FFFFFFF

    def get(self):
        image = self.sensor.snapshot()
        if C.CAMERA_TIMESTAMP_MODE == "frame_callback":
            raw, sequence = self.done_ticks, self.done_sequence
            if raw is None or sequence == self.last_sequence:
                return None, None
            self.last_sequence = sequence
            stamp = self.clock.from_ticks(raw)
        else:
            stamp = self.clock.now_us()
            self.done_sequence = (self.done_sequence+1) & 0x3FFFFFFF
            self.last_sequence = self.done_sequence
        # 返回/帧完成时刻不等于曝光中点，必须测量并填写偏移。
        return image, stamp+C.CAMERA_TIME_OFFSET_US


class CameraAdapter:
    """低频闭环调节；不依赖OV5640在固定曝光时自动增益是否继续工作。"""
    def __init__(self, sensor):
        self.sensor = sensor
        self.last_us = -10000000
        self.manual = False

    def update(self, image, now, gyro=(0.0, 0.0, 0.0), pixel_speed=0.0):
        if not C.CAMERA_AUTO_TUNE or now-self.last_us < 500000:
            return
        self.last_us = now
        statistics = image.get_statistics()
        mean = statistics.mean()
        old_exp, old_gain = self.sensor.get_exposure_us(), self.sensor.get_gain_db()
        speed = math.sqrt(sum(v*v for v in gyro))
        cap = C.EXPOSURE_MAX_US
        motion = max(0.0, pixel_speed)+max(C.FX, C.FY)*speed
        if math.isfinite(motion) and motion > .01:
            cap = min(cap, C.MAX_BLUR_PX*1000000.0/motion)
        cap = max(C.EXPOSURE_MIN_US, int(cap))
        ratio = max(.8, min(1.25, 100.0/max(1, mean)))
        exposure = max(C.EXPOSURE_MIN_US, min(cap, int(old_exp*ratio)))
        gain = max(0.0, min(30.0, old_gain+20*math.log10(max(.01, ratio*old_exp/exposure))))
        # 首次接管自动模式；之后忽略很小的变化，减少重复寄存器写入。
        if not self.manual or abs(exposure-old_exp) >= max(100, old_exp*.02):
            self.sensor.set_auto_exposure(False, exposure_us=exposure)
        if not self.manual or abs(gain-old_gain) >= .2:
            self.sensor.set_auto_gain(False, gain_db=gain)
        self.manual = True
        return statistics  # 同一帧的纹理检查复用统计，避免再次扫描全图。


def wait_for_attitude(link, stamp):
    start = link.clock.now_us()
    while True:
        now = link.clock.now_us()
        if not link.connected(now) or not link.synced(now):
            return
        attitude = attitude_at(link.attitudes, link.fc_time(stamp))
        if attitude is not None:
            return attitude
        if now-start >= C.ATTITUDE_WAIT_US:
            return
        link.poll()
        if hasattr(time, "sleep_ms"):
            time.sleep_ms(1)
        else:
            time.sleep(0.001)


def run(observer=None):
    """两个入口复用同一循环；正式入口不创建测试对象、不采集阶段计时。"""
    validate_config()
    sensor = setup_camera()
    link = FlightLink()
    controller = VOController()
    probe = observer if observer is not None and getattr(observer, "timing", True) else None
    controller.engine.service = link.poll
    controller.engine.probe = probe
    output = Output(link)
    capture = Capture(sensor, link.clock)
    camera_adapter = CameraAdapter(sensor)
    if observer is not None:
        observer.prepare(sensor, link, controller, capture, camera_adapter, output)
    window_start = link.clock.now_us()
    frames, fps = 0, 0.0
    last_height = None
    while True:
        if observer is not None:
            observer.begin_frame()
        link.poll(service_sync=True)
        if probe is not None:
            probe.mark("communication")
        image, stamp = capture.get()
        if probe is not None:
            probe.mark("capture")
        link.poll(service_sync=True)
        if image is None:
            if probe is not None:
                probe.mark("communication")
            if observer is not None and observer.finish_frame(None, fps):
                break
            continue
        attitude = wait_for_attitude(link, stamp)
        if probe is not None:
            probe.mark("communication")
        process_start = link.clock.now_us()
        gyro = link.attitudes[-1][2] if link.attitudes else (0.0, 0.0, 0.0)
        pixel_speed = controller.engine.predicted_pixel_speed(last_height, stamp)
        if probe is not None and hasattr(probe, 'observe_image'):
            probe.observe_image(image, sensor)
        statistics = camera_adapter.update(image, process_start, gyro, pixel_speed)
        if probe is not None:
            probe.mark("capture")
        result = controller.process(image, stamp, link, statistics=statistics, attitude=attitude)
        last_height = result["height"]
        result["frame_sequence"] = capture.last_sequence
        if probe is not None:
            probe.mark("vision")
        frames += 1
        now = link.clock.now_us()
        if now-window_start >= 1000000:
            fps = frames*1000000.0/(now-window_start)
            frames, window_start = 0, now
        if hasattr(gc, "mem_free") and gc.mem_free() < C.GC_LOW_WATER_BYTES:
            if probe is not None:
                gc_start = _us()
            gc.collect()
            if probe is not None:
                probe.record("gc", _elapsed(gc_start))
        output.update(result, fps)
        if observer is not None:
            if observer.finish_frame(result, fps):
                break


def main():
    run()


if __name__ == "__main__":
    main()
