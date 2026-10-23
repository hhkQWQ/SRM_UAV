"""算法接口：姿态辅助函数与原生QVGA视觉核心。

正常跟踪传递紧凑缓冲；ORB只在关键帧建立与恢复时调用。
"""
import math
import time
import config as C
_INF = float("inf")


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


def inverse_rotate(r, v):
    x, y, z = v
    return (r[0]*x+r[3]*y+r[6]*z, r[1]*x+r[4]*y+r[7]*z,
            r[2]*x+r[5]*y+r[8]*z)


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


def undistort(u, v):
    xd, yd = (u-C.CX)/C.FX, (v-C.CY)/C.FY
    x, y = xd, yd
    k1, k2, p1, p2, k3 = C.DIST_COEFFS
    for _ in range(C.UNDISTORT_ITERATIONS):
        old_x, old_y = x, y
        r2 = x*x+y*y
        radial = 1+k1*r2+k2*r2*r2+k3*r2*r2*r2
        if abs(radial) < 1e-9:
            raise ValueError("invalid distortion")
        dx = 2*p1*x*y+p2*(r2+2*x*x)
        dy = p1*(r2+2*y*y)+2*p2*x*y
        x, y = (xd-dx)/radial, (yd-dy)/radial
        if abs(x-old_x)+abs(y-old_y) < 1e-8:
            break
    return x, y


class FeatureEngine:
    """Native QVGA frontend; Python schedules sensors and ORB recovery."""
    def __init__(self, probe=None):
        from array import array
        try:
            import rm_vo
        except ImportError:
            raise RuntimeError("缺少rm_vo原生模块：请按README编译并安装固件")
        if rm_vo.ABI_VERSION != 1:
            raise RuntimeError("rm_vo接口版本不匹配，请更新固件和全部Python文件")
        import image
        self.image_module, self.array = image, array
        self.core = rm_vo.Core(native_settings())
        self.meta = array('f', [0.0]*13)
        self.timestamp = array('I', [0, 0])
        self.prediction = array('f', [0.0]*3)
        self.raw = array('f', [0.0]*23)
        self.probe, self.service = probe, None
        self.pixel_sigma = C.PIXEL_STD
        self.reset()

    def reset(self):
        self.core.reset()
        self.key_descriptor = None
        self.last_position = self.last_stamp = None
        self.velocity = (0.0, 0.0)
        self.velocity_stamp = None
        self.key_variance = self.previous_variance = (0.0, 0.0)
        self.empty_frames = 0
        self.diagnostics = {}

    def _record(self, name, start):
        if self.probe is not None:
            self.probe.record(name, _elapsed(start))

    def _service(self):
        if self.service is not None:
            start = _us()
            self.service()
            self._record('vision_poll', start)

    def _descriptor(self, image):
        detector = C.FEATURE_DETECTOR
        if detector == 'auto':
            detector = 'AGAST' if self.empty_frames >= 3 else 'FAST'
        corner = (self.image_module.CORNER_FAST if detector == 'FAST'
                  else self.image_module.CORNER_AGAST)
        threshold = C.KEYPOINT_THRESHOLD
        start = _us()
        descriptor = image.find_keypoints(threshold=threshold,
            normalized=C.KEYPOINT_NORMALIZED, scale_factor=C.KEYPOINT_SCALE_FACTOR,
            max_keypoints=C.MAX_KEYPOINTS, corner_detector=corner)
        if (C.FEATURE_DETECTOR == 'auto' and detector == 'FAST'
                and (descriptor is None or len(descriptor)<C.MIN_MATCHES)):
            # A keyframe needs usable recovery descriptors now; waiting for
            # future frames would lose the original reference image.
            detector = 'AGAST'
            descriptor = image.find_keypoints(threshold=threshold,
                normalized=C.KEYPOINT_NORMALIZED, scale_factor=C.KEYPOINT_SCALE_FACTOR,
                max_keypoints=C.MAX_KEYPOINTS, corner_detector=self.image_module.CORNER_AGAST)
        self._record('features', start)
        self.empty_frames = self.empty_frames+1 if descriptor is None or len(descriptor)<C.MIN_MATCHES else 0
        self.diagnostics.update(detector_name=detector,detector=threshold,
            normalized=C.KEYPOINT_NORMALIZED,limit=C.MAX_KEYPOINTS)
        self._service()
        return descriptor

    def process(self, image, q, height, camera_pos, stamp):
        if image.width()!=320 or image.height()!=240 or image.format()!=self.image_module.GRAYSCALE:
            raise ValueError('rm_vo requires calibrated QVGA GRAYSCALE')
        r = q_matrix(q)
        rotation,arm = matmul(r,C.R_BC),rotate(r,camera_pos)
        for i in range(9):
            self.meta[i] = rotation[i]
        for i in range(3):
            self.meta[9+i] = arm[i]
        self.meta[12] = height
        self.timestamp[0], self.timestamp[1] = stamp & 0xFFFFFFFF, stamp >> 32
        dt = 0.0 if self.last_stamp is None else max(0.0,(stamp-self.last_stamp)*1e-6)
        age = None if self.velocity_stamp is None else stamp-self.velocity_stamp
        predictive = age is not None and 0<=age<=C.PREDICTION_MAX_AGE_US
        velocity = self.velocity if predictive else (0.0,0.0)
        position = self.last_position or (0.0,0.0)
        for i in range(2):
            self.prediction[i] = position[i]+velocity[i]*dt
        uncertainty = .5*C.PREDICTION_ACCEL_M_S2*dt*dt*max(C.FX,C.FY)/max(.05,height)
        self.prediction[2] = (min(C.SEARCH_MAX_PX,C.SEARCH_MIN_PX+uncertainty)
                             if predictive else C.SEARCH_MAX_PX)
        self.diagnostics = dict(keypoints=0,matches=0,inliers=0,work_size=(320,240),
            limit=C.MAX_KEYPOINTS,normalized=None,detector_name='KLT/SAD',
            detector=C.FAST_THRESHOLD,vision_reason='MATCH_COUNT')
        start = _us()
        self.core.step(image,self.meta,self.timestamp,self.prediction,self.raw)
        self._record('track_fit',start)
        self._service()
        used_orb = False
        if int(self.raw[0])==0 and self.key_descriptor is not None:
            descriptor = self._descriptor(image)
            if descriptor is not None and len(descriptor)>=C.MIN_MATCHES:
                start = _us()
                match = self.image_module.match_descriptor(self.key_descriptor,descriptor,
                    threshold=C.MATCH_THRESHOLD,filter_outliers=False)
                packed = self.array('f')
                used_source,used_target = set(),set()
                for i,j in match.match():
                    if i in used_source or j in used_target:
                        continue
                    used_source.add(i);used_target.add(j)
                    a,b = self.key_descriptor[i],descriptor[j]
                    # MicroPython array.extend requires a buffer, not a tuple.
                    for value in (a[0],a[1],b[0],b[1]):
                        packed.append(value)
                    if len(used_source)>=64:
                        break
                self._record('match',start)
                start = _us()
                self.core.recover(image,self.meta,self.timestamp,packed,self.raw)
                self._record('track_fit',start)
                used_orb = int(self.raw[0])==2
                self._service()
        raw = self.raw
        status,key_changed,base = int(raw[0]),bool(raw[15]),int(raw[16])
        self.diagnostics.update(keypoints=int(raw[19]),matches=int(raw[6]),
            inliers=int(raw[7]),cells=int(raw[9]),spread=raw[10],
            klt=int(raw[20]),sad=int(raw[21]),orb=used_orb,reference='near' if base else 'key',
            search_px=self.prediction[2],vision_reason='OK' if status==2 else 'MATCH_COUNT')
        base_variance = self.previous_variance if base else self.key_variance
        if key_changed:
            self.key_descriptor = self._descriptor(image)
        if status==2:
            measured = (raw[1],raw[2])
            if self.last_position is not None and 0<dt<=C.PREDICTION_MAX_AGE_US*1e-6:
                self.velocity = tuple((measured[i]-self.last_position[i])/dt for i in range(2))
                self.velocity_stamp = stamp
            else:
                self.velocity,self.velocity_stamp = (0.0,0.0),None
            self.last_position,self.last_stamp = measured,stamp
        elif status==1:
            self.last_position,self.last_stamp = (0.0,0.0),stamp
        return dict(status=status,position=(raw[1],raw[2]),delta=(raw[3],raw[4]),
            residual_yaw=raw[5],matches=int(raw[6]),inliers=int(raw[7]),ratio=raw[8],
            cells=int(raw[9]),spread=raw[10],residual_px=raw[11],
            sample_variance=(raw[12],raw[13]),keyframe=int(raw[14]),key_changed=key_changed,
            reference_height=raw[17],reference_dt=raw[18],base_variance=base_variance)

    def accept_variance(self, motion, variance):
        self.previous_variance = variance
        if motion['key_changed']:
            self.key_variance = variance

    def predicted_pixel_speed(self, height, now):
        if (height is None or height<=0 or self.velocity_stamp is None
                or now-self.velocity_stamp>C.PREDICTION_MAX_AGE_US):
            return 0.0
        return max(C.FX,C.FY)*math.sqrt(sum(v*v for v in self.velocity))/height


def native_settings():
    from array import array
    return array('f',(C.FX,C.FY,C.CX,C.CY)+C.DIST_COEFFS+(
        C.MAX_KEYPOINTS,C.FAST_THRESHOLD,C.MIN_MATCHES,C.MIN_INLIERS,
        C.MIN_INLIER_RATIO,C.MIN_OCCUPIED_CELLS,C.MIN_RAY_DOWN,C.RANSAC_THRESHOLD_PX,
        C.MAX_RESIDUAL_YAW_RAD,C.SEARCH_MIN_PX,C.SEARCH_MAX_PX,C.KLT_MAX_ITERATIONS,
        C.KLT_MAX_ERROR,C.KLT_FB_ERROR_PX,C.KEYFRAME_DISTANCE_HEIGHT_RATIO,
        C.KEYFRAME_HEIGHT_RATIO,C.KEYFRAME_ROTATION_RAD,C.NEAR_REFERENCE_FLOW_PX,
        C.FEATURE_TOPUP_RATIO,C.KLT_MIN_EIGENVALUE,C.RANSAC_MAX_ITERATIONS,
        C.KEYFRAME_REFRESH_INLIER_RATIO))
