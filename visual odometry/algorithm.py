"""算法计算：四元数、稀疏去畸变、平地投影、二维平移RANSAC、ORB。

只在FeatureEngine中调用OpenMV的image；其余函数可在PC直接验证。
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


class GroundFrame:
    """保存描述子和关键帧投影；不保存被绘图污染的相机framebuffer。"""
    def __init__(self, descriptor, q, height, camera_pos, stamp, pixels=None,
                 pixel_transform=(1.0, 1.0, 0.0, 0.0), lazy=False):
        self.descriptor = descriptor
        self.q = q
        self.height = height
        self.stamp = stamp
        r_nb = q_matrix(q)
        self.r_nc = matmul(r_nb, C.R_BC)
        self.arm = rotate(r_nb, camera_pos)
        count = len(pixels) if pixels is not None else len(descriptor)
        sx, sy, ox, oy = pixel_transform
        self.pixels = []
        for i in range(count):
            u, v = pixels[i] if pixels is not None else descriptor[i][:2]
            self.pixels.append((u*sx+ox, v*sy+oy))
        self.points = [None]*count
        self.projected = [False]*count
        if not lazy:
            for i in range(count):
                self.point(i)

    def point(self, index):
        if not self.projected[index]:
            u, v = self.pixels[index]
            xn, yn = undistort(u, v)
            ray = rotate(self.r_nc, (xn, yn, 1.0))
            if ray[2] > C.MIN_RAY_DOWN:
                self.points[index] = (u, v, self.arm[0]+self.height*ray[0]/ray[2],
                                      self.arm[1]+self.height*ray[1]/ray[2], xn, yn)
            self.projected[index] = True
        return self.points[index]

    def spread(self):
        n = len(self.pixels)
        if n < 2:
            return 0, 0.0
        cells = set()
        x = y = xx = yy = xy = 0.0
        for u, v in self.pixels:
            cells.add((min(2, int(u*3/C.IMAGE_WIDTH)), min(2, int(v*3/C.IMAGE_HEIGHT))))
            x += u
            y += v
            xx += u*u
            yy += v*v
            xy += u*v
        xx, yy, xy = max(0.0, xx-x*x/n), max(0.0, yy-y*y/n), xy-x*y/n
        trace = xx+yy
        root = math.sqrt(max(0.0, (xx-yy)**2+4*xy*xy))
        return len(cells), max(0.0, trace-root)/max(1e-9, trace+root)


def _rows(reference, current, pairs):
    """为每对点预计算相机坐标，RANSAC内环不再求旋转或去畸变。"""
    rows = []
    for i, j in pairs:
        if i < 0 or j < 0 or i >= len(reference.points) or j >= len(current.points):
            continue
        a, b = reference.point(i), current.point(j)
        if a is None or b is None:
            continue
        base = inverse_rotate(current.r_nc,
                              (a[2]-current.arm[0], a[3]-current.arm[1], current.height))
        # dx,dy, baseX/Y/Z, observed normalized x/y, current raw u/v, source indices
        rows.append((a[2]-b[2], a[3]-b[3], base[0], base[1], base[2],
                     b[4], b[5], b[0], b[1], i, j))
    return rows


def _residual2(row, dx, dy, r):
    # R_nc^T * (ground_NE - body_delta_NE - arm_NE, camera_height)
    x = row[2]-r[0]*dx-r[3]*dy
    y = row[3]-r[1]*dx-r[4]*dy
    z = row[4]-r[2]*dx-r[5]*dy
    if z <= 1e-5:
        return 1e30
    ex, ey = C.FX*(x/z-row[5]), C.FY*(y/z-row[6])
    return ex*ex+ey*ey


def ransac_translation(reference, current, pairs, seed=1, diagnostics=None, probe=None):
    """已知姿态/高度的平面平移模型。返回None或几何估计dict。

    单个点对能提出2DoF假设；必须由足够多、空间分散的内点验证。
    对内点加权精化；公共姿态/高度误差由control额外计入方差。
    """
    if probe is not None:
        start = _us()
    rows = _rows(reference, current, pairs)
    n = len(rows)
    if diagnostics is not None:
        diagnostics.update(matches=n, inliers=0, vision_reason="MATCH_COUNT")
    if probe is not None:
        probe.record("project", _elapsed(start))
    if n < C.MIN_MATCHES:
        return None
    if probe is None:
        return _fit_translation(rows, current, seed, diagnostics)
    start = _us()
    try:
        return _fit_translation(rows, current, seed, diagnostics)
    finally:
        probe.record("fit", _elapsed(start))


def _fit_translation(rows, current, seed, diagnostics):
    n = len(rows)
    r, gate2 = current.r_nc, C.RANSAC_THRESHOLD_PX**2
    best_count, best_cost, best = 0, 1e30, None
    limit, iteration = min(n, C.RANSAC_MAX_ITERATIONS), 0
    seen = set()
    while iteration < limit:
        seed = (1664525*seed+1013904223) & 0xFFFFFFFF
        idx = seed % n
        while idx in seen:
            idx = (idx+1) % n
        seen.add(idx)
        dx, dy = rows[idx][:2]
        count, cost = 0, 0.0
        for row in rows:
            error = _residual2(row, dx, dy, r)
            if error <= gate2:
                count += 1
                cost += error
        if count > best_count or (count == best_count and cost < best_cost):
            best_count, best_cost, best = count, cost, (dx, dy)
            ratio = count/n
            if ratio >= 1.0:
                need = C.RANSAC_MIN_ITERATIONS
            elif ratio > 0:
                need = int(math.ceil(math.log(1-C.RANSAC_CONFIDENCE)/math.log(1-ratio)))
            else:
                need = limit
            limit = min(limit, max(C.RANSAC_MIN_ITERATIONS, need))
        iteration += 1
    if best is None:
        return None
    dx, dy = best
    for _ in range(2):
        sx, sy, sw = 0.0, 0.0, 0.0
        for row in rows:
            error = _residual2(row, dx, dy, r)
            if error <= gate2:
                weight = 1.0/max(1.0, math.sqrt(error))
                sx += weight*row[0]
                sy += weight*row[1]
                sw += weight
        if sw == 0:
            return None
        dx, dy = sx/sw, sy/sw
    inliers, cells, errors, vx, vy = [], set(), 0.0, 0.0, 0.0
    for row in rows:
        error = _residual2(row, dx, dy, r)
        if error <= gate2:
            inliers.append((row[9], row[10]))
            cells.add((min(2, int(row[7]*3/C.IMAGE_WIDTH)),
                       min(2, int(row[8]*3/C.IMAGE_HEIGHT))))
            errors += error
            vx += (row[0]-dx)**2
            vy += (row[1]-dy)**2
    count = len(inliers)
    if (count < C.MIN_INLIERS or count/n < C.MIN_INLIER_RATIO
            or len(cells) < C.MIN_OCCUPIED_CELLS):
        if diagnostics is not None:
            diagnostics.update(inliers=count, cells=len(cells),
                vision_reason=("INLIERS" if count < C.MIN_INLIERS else
                               "INLIER_RATIO" if count/n < C.MIN_INLIER_RATIO else "COVERAGE"))
        return None
    if diagnostics is not None:
        diagnostics.update(inliers=count, cells=len(cells), vision_reason="OK")
    return {"delta": (dx, dy), "inliers": inliers, "matches": n,
            "ratio": count/n, "cells": len(cells), "iterations": iteration,
            "residual_px": math.sqrt(errors/count),
            "sample_variance": (vx/max(1, count-1), vy/max(1, count-1))}


class FeatureEngine:
    """固件原生AGAST/ORB；没有在MicroPython中逐像素扫描。"""
    def __init__(self, probe=None):
        import image
        self.image_module = image
        self.sequence = 0
        self.work = None
        self.threshold = C.KEYPOINT_THRESHOLD
        self.limit = C.MAX_KEYPOINTS
        self.diagnostics = {}
        self.pixel_sigma = C.PIXEL_STD
        self.service = None
        self.probe = probe
        self.detector = "FAST" if C.FEATURE_DETECTOR == "auto" else C.FEATURE_DETECTOR
        self.empty_frames = 0
        self.reference_height = None

    def _service(self):
        if self.service is not None:
            if self.probe is None:
                self.service()
            else:
                start = _us()
                self.service()
                self.probe.record("vision_poll", _elapsed(start))

    def extract(self, image, q, height, camera_pos, stamp):
        if image.width() != C.IMAGE_WIDTH or image.height() != C.IMAGE_HEIGHT:
            raise ValueError("image resolution differs from calibration")
        start = _us()
        scale = C.FEATURE_SCALE
        if scale == 1.0:
            work = image
        elif self.work is None:
            self.work = image.copy(x_scale=scale, y_scale=scale, hint=self.image_module.AREA)
            work = self.work
        else:
            self.work.draw_image(image, 0, 0, x_scale=scale, y_scale=scale, hint=self.image_module.AREA)
            work = self.work
        if self.probe is not None:
            self.probe.record("resize", _elapsed(start))
        used_threshold, used_limit = self.threshold, self.limit
        detector_name = self.detector
        detector = (self.image_module.CORNER_FAST if detector_name == "FAST"
                    else self.image_module.CORNER_AGAST)
        if detector_name == "FAST":
            used_threshold = min(90, used_threshold*3)
        normalized = C.KEYPOINT_NORMALIZED
        if self.reference_height is not None and abs(math.log(height/self.reference_height)) > .12:
            normalized = False
        if self.probe is not None:
            detect_start = _us()
        descriptor = work.find_keypoints(threshold=used_threshold,
            normalized=normalized, scale_factor=C.KEYPOINT_SCALE_FACTOR,
            max_keypoints=used_limit,
            corner_detector=detector)
        elapsed = _elapsed(start)
        if self.probe is not None:
            self.probe.record("features", _elapsed(detect_start))
        count = 0 if descriptor is None else len(descriptor)
        self.diagnostics = dict(keypoints=count, detector=used_threshold, detector_name=detector_name,
            normalized=normalized, limit=used_limit,
            matches=0, inliers=0, vision_reason="NO_FEATURES", work_size=(work.width(), work.height()))
        self._service()
        if C.FEATURE_ADAPTIVE:
            self.empty_frames = self.empty_frames+1 if count < C.MIN_MATCHES else 0
            if C.FEATURE_DETECTOR == "auto" and self.empty_frames >= 3:
                self.detector = "AGAST"
            if count < max(C.MIN_MATCHES+4, used_limit//2):
                self.threshold = max(5, self.threshold-2)
            elif count >= int(used_limit*.9):
                self.threshold = min(40, self.threshold+1)
            budget = 1000000/max(1, C.TARGET_FPS)*.4
            if elapsed > budget and count >= C.MIN_MATCHES+4:
                floor = min(C.MAX_KEYPOINTS, max(24, C.MIN_MATCHES+4))
                self.limit = max(floor, self.limit-4)
            elif elapsed < budget*.6:
                self.limit = min(C.MAX_KEYPOINTS, self.limit+2)
            if (C.FEATURE_DETECTOR == "auto" and self.detector == "AGAST"
                    and count >= used_limit*.8 and elapsed > budget*1.5):
                self.detector = "FAST"
        if count < C.MIN_MATCHES:
            return None
        sx, sy = C.IMAGE_WIDTH/work.width(), C.IMAGE_HEIGHT/work.height()
        self.pixel_sigma = C.PIXEL_STD*max(sx, sy)
        frame = GroundFrame(descriptor, q, height, camera_pos, stamp,
            pixel_transform=(sx, sy, (sx-1)*.5, (sy-1)*.5), lazy=True)
        cells, spread = frame.spread()
        self.diagnostics.update(cells=cells, spread=spread, vision_reason="READY")
        if cells < C.MIN_OCCUPIED_CELLS or spread < .01:
            self.diagnostics["vision_reason"] = "FEATURES_1D" if spread < .01 else "FEATURES_CLUSTERED"
            return None
        return frame

    def estimate(self, reference, current):
        if self.probe is not None:
            start = _us()
        matches = self.image_module.match_descriptor(reference.descriptor,
            current.descriptor, threshold=C.MATCH_THRESHOLD, filter_outliers=False)
        if self.probe is not None:
            self.probe.record("match", _elapsed(start))
        self._service()
        self.sequence += 1
        return ransac_translation(reference, current, matches.match(), self.sequence,
                                  self.diagnostics, self.probe)
