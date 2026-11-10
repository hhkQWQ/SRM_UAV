"""算法接口：姿态辅助函数与原生QVGA视觉核心。

固定缓冲传递；纯FAST检测、KLT跟踪和二维平移均在C中完成。
"""
import math
import time
import config as C
_INF = float("inf")
_VISION_REASONS = ('OK', 'MATCH_COUNT', 'INLIER_RATIO', 'AMBIGUOUS_MATCH',
                   'INPUT_INVALID', 'INITIALIZING')
RESULT_FLOATS = 34


def _us():
    return time.ticks_us() if hasattr(time, "ticks_us") else time.monotonic_ns()//1000


def _elapsed(start):
    now = _us()
    return time.ticks_diff(now, start) if hasattr(time, "ticks_diff") else now-start


def finite(x):
    return x == x and abs(x) != _INF


def normalize_q(q):
    n = math.sqrt(sum(v * v for v in q))
    if not finite(n) or n < 1e-9:
        raise ValueError("invalid quaternion")
    return tuple(v / n for v in q)


def q_matrix(q):
    w, x, y, z = normalize_q(q)
    return (1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w),
            2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w),
            2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y))


def rotate(r, v):
    x, y, z = v
    return (r[0]*x+r[1]*y+r[2]*z, r[3]*x+r[4]*y+r[5]*z,
            r[6]*x+r[7]*y+r[8]*z)


def matmul(a, b):
    return tuple(sum(a[i*3+k]*b[k*3+j] for k in range(3))
                 for i in range(3) for j in range(3))


def q_angle(a, b):
    dot = abs(sum(x*y for x, y in zip(a, b)))
    return 2.0 * math.acos(min(1.0, max(0.0, dot)))


def slerp(a, b, t):
    dot = sum(x*y for x, y in zip(a, b))
    if dot < 0:
        b = tuple(-x for x in b)
        dot = -dot
    dot = min(1.0, max(-1.0, dot))
    if dot > 0.9995:
        return normalize_q(tuple(x+t*(y-x) for x, y in zip(a, b)))
    angle = math.acos(dot)
    den = math.sin(angle)
    wa, wb = math.sin((1-t)*angle)/den, math.sin(t*angle)/den
    return tuple(wa*x+wb*y for x, y in zip(a, b))


def predict_q(q, gyro, dt):
    speed = math.sqrt(sum(v*v for v in gyro))
    if speed < 1e-9 or dt == 0:
        return q
    half = speed*dt*0.5
    scale = math.sin(half)/speed
    b = (math.cos(half), gyro[0]*scale, gyro[1]*scale, gyro[2]*scale)
    w, x, y, z = q
    a, b1, c, d = b
    return normalize_q((w*a-x*b1-y*c-z*d, w*b1+x*a+y*d-z*c,
                        w*c-x*d+y*a+z*b1, w*d+x*c-y*b1+z*a))


def euler_degrees(q):
    w, x, y, z = normalize_q(q)
    return (math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))*180/math.pi,
            math.asin(max(-1.0, min(1.0, 2*(w*y-z*x))))*180/math.pi,
            math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))*180/math.pi)


class MotionHistory:
    """最近真实位置的定容时间回归；预测值永不反馈进历史。"""
    def __init__(self):
        from comm import History
        self.samples = History(C.PREDICTION_HISTORY_SIZE)
        self.reset()

    def reset(self):
        self.samples.start = self.samples.count = 0
        self.stamp = self.position = None
        self.velocity = (0.0, 0.0)
        self.velocity_variance = (C.PREDICTION_VELOCITY_STD_M_S**2,)*2

    def add(self, stamp, position):
        if self.stamp is not None:
            if stamp <= self.stamp:
                return
            if stamp-self.stamp > C.TRACK_HOLD_US:
                self.reset()
        self.samples.append((stamp, position[0], position[1]))
        self.stamp, self.position = stamp, position
        n = len(self.samples)
        if n < 2:
            return
        # Relative time/position avoid cancellation with long uint64 uptime.
        mt = sum((p[0]-stamp)*1e-6 for p in self.samples)/n
        mp = tuple(sum(p[i+1]-position[i] for p in self.samples)/n for i in range(2))
        tt = sum(((p[0]-stamp)*1e-6-mt)**2 for p in self.samples)
        if tt <= 1e-12:
            return
        self.velocity = tuple(sum(((p[0]-stamp)*1e-6-mt)*
                                  (p[i+1]-position[i]-mp[i]) for p in self.samples)/tt
                              for i in range(2))
        self.velocity_variance = tuple(max(C.PREDICTION_VELOCITY_STD_M_S**2,
            (sum((p[i+1]-position[i]-mp[i]-self.velocity[i]*
                  ((p[0]-stamp)*1e-6-mt))**2 for p in self.samples)/max(1,n-2)
             +C.POSITION_NOISE_FLOOR_M**2)/tt) for i in range(2))

    def project(self, stamp):
        if self.stamp is None or not 0 <= stamp-self.stamp <= C.TRACK_HOLD_US:
            return None
        dt = (stamp-self.stamp)*1e-6
        position = tuple(self.position[i]+self.velocity[i]*dt for i in range(2))
        extra = tuple(self.velocity_variance[i]*dt*dt for i in range(2))
        return position, extra


class FeatureEngine:
    """Native QVGA frontend; Python schedules sensors and uncertainty."""
    def __init__(self, probe=None, predictor=None):
        from array import array
        try:
            import rm_vo
        except ImportError:
            raise RuntimeError("缺少rm_vo原生模块：请按README编译并安装固件")
        if rm_vo.ABI_VERSION != 4 or rm_vo.RESULT_FLOATS != RESULT_FLOATS:
            raise RuntimeError("rm_vo接口版本不匹配，请更新固件和全部Python文件")
        import image
        self.image_module = image
        self.core = rm_vo.Core(native_settings())
        self.meta = array('f', [0.0]*13)
        self.timestamp = array('I', [0, 0])
        self.prediction = array('f', [0.0]*3)
        self.raw = array('f', [0.0]*RESULT_FLOATS)
        self.native_timing = array('I', [0]*6)
        self.profile_enabled = False
        self.probe, self.service = probe, None
        self.pixel_sigma = C.PIXEL_STD
        self.owns_predictor = predictor is None
        self.predictor = MotionHistory() if predictor is None else predictor
        self.reset()

    def reset(self):
        self.core.reset()
        if self.owns_predictor:
            self.predictor.reset()
        self.key_variance = self.previous_variance = (0.0, 0.0)
        # 两个必要诊断字段跨帧复用；详细数值由控制结果直接提供。
        self.diagnostics = {'vision_reason': 'INITIALIZING', 'sparse': False}

    def _service(self):
        if self.service is not None:
            self.service()

    def process(self, image, q, height, camera_pos, stamp, rotation=None):
        if image.width()!=320 or image.height()!=240 or image.format()!=self.image_module.GRAYSCALE:
            raise ValueError('rm_vo requires calibrated QVGA GRAYSCALE')
        r = q_matrix(q) if rotation is None else rotation
        rotation,arm = matmul(r,C.R_BC),rotate(r,camera_pos)
        for i in range(9):
            self.meta[i] = rotation[i]
        for i in range(3):
            self.meta[9+i] = arm[i]
        self.meta[12] = height
        self.timestamp[0], self.timestamp[1] = stamp & 0xFFFFFFFF, stamp >> 32
        projection = self.predictor.project(stamp)
        position = projection[0] if projection is not None else self.predictor.position or (0.0,0.0)
        for i in range(2):
            self.prediction[i] = position[i]
        self.prediction[2] = C.SEARCH_MIN_PX if projection is not None else C.SEARCH_MAX_PX
        profiling = self.probe is not None
        if profiling != self.profile_enabled:
            self.core.profile(profiling)
            self.profile_enabled = profiling
        self.core.step(image,self.meta,self.timestamp,self.prediction,self.raw)
        if profiling:
            self.core.timings(self.native_timing)
            self.probe.native(self.native_timing)
        self._service()
        raw = self.raw
        status,key_changed,base = int(raw[0]),bool(raw[13]),int(raw[14])
        self.diagnostics['sparse'] = bool(raw[16]) and status==2
        reason = int(raw[17])
        self.diagnostics['vision_reason'] = _VISION_REASONS[reason] if 0<=reason<len(_VISION_REASONS) else 'TRACK_FAILED'
        self.diagnostics['base'] = base
        self.diagnostics['ref_count'] = int(raw[23])
        self.diagnostics['projected'] = int(raw[24])
        self.diagnostics['klt_eig_reject'] = int(raw[25])
        self.diagnostics['klt_fail'] = int(raw[26])
        self.diagnostics['fb_reject'] = int(raw[27])
        self.diagnostics['sad_fail'] = int(raw[28])
        self.diagnostics['fast_candidates'] = int(raw[29])
        self.diagnostics['weak_candidates'] = int(raw[30])
        self.diagnostics['new_prev_count'] = int(raw[31])
        self.diagnostics['new_prev_survived'] = int(raw[32])
        self.diagnostics['added'] = int(raw[22])
        self.diagnostics['detect_cells'] = int(raw[21])
        base_variance = self.previous_variance if base else self.key_variance
        if status==2 and self.owns_predictor:
            self.predictor.add(stamp,(raw[1],raw[2]))
        return dict(status=status,position=(raw[1],raw[2]),delta=(raw[3],raw[4]),
            matches=int(raw[5]),inliers=int(raw[6]),ratio=raw[7],
            cells=int(raw[8]),residual_px=raw[9],
            sample_variance=(raw[10],raw[11]),keyframe=int(raw[12]),key_changed=key_changed,
            reference_height=raw[15],base_variance=base_variance,
            effective_count=raw[18],match_weight_sum=raw[33],
            sparse=bool(raw[16]),attempts=int(raw[19]),
            sad_candidates=int(raw[20]),detect_cells=int(raw[21]),added=int(raw[22]),
            ref_count=int(raw[23]),projected=int(raw[24]),
            klt_eig_reject=int(raw[25]),klt_fail=int(raw[26]),
            fb_reject=int(raw[27]),sad_fail=int(raw[28]),
            fast_candidates=int(raw[29]),weak_candidates=int(raw[30]),
            new_prev_count=int(raw[31]),new_prev_survived=int(raw[32]))

    def accept_variance(self, motion, variance):
        self.previous_variance = variance
        if motion['key_changed']:
            self.key_variance = variance

    def predicted_pixel_speed(self, height, now):
        if height is None or height<=0 or self.predictor.project(now) is None:
            return 0.0
        return max(C.FX,C.FY)*math.sqrt(sum(v*v for v in self.predictor.velocity))/height


def native_settings():
    from array import array
    return array('f',(C.FX,C.FY,C.CX,C.CY)+C.DIST_COEFFS+(
        C.MAX_KEYPOINTS,C.FAST_THRESHOLD,C.MIN_MATCHES,C.MIN_INLIERS,
        C.MIN_INLIER_RATIO,C.MIN_RAY_DOWN,C.TRANSLATION_GATE_PX,
        C.SEARCH_MIN_PX,C.SEARCH_MAX_PX,C.KLT_MAX_ITERATIONS,
        C.KLT_MAX_ERROR,C.KLT_FB_ERROR_PX,C.KEYFRAME_DISTANCE_HEIGHT_RATIO,
        C.KEYFRAME_HEIGHT_RATIO,C.KEYFRAME_ROTATION_RAD,C.FEATURE_TOPUP_RATIO,
        C.KLT_MIN_EIGENVALUE,C.KEYFRAME_REFRESH_INLIER_RATIO,
        C.FEATURE_TOPUP_CELLS,C.FEATURE_TOPUP_POINTS,C.KEYFRAME_CHECK_INTERVAL,
        C.TEMPLATE_WARP_ANGLE_RAD,C.TEMPLATE_WARP_HEIGHT_RATIO,C.SAD_UNIQUENESS_RATIO,
        C.FEATURE_CELL_MAX_POINTS,C.FEATURE_MIN_DISTANCE_PX,C.WEAK_CORNER_STRIDE_PX,
        C.INIT_FULL_SCAN_INTERVAL))
