"""控制决定逻辑：输入健康、测距、时间插值、关键帧、失跟与输出准入。

此处的“控制”是算法状态管理，不控制电机、不切飞行模式、不修改EKF参数。
"""
import math
import config as C
import algorithm as A
from comm import History


def attitude_at(history, stamp):
    """飞控时钟域；有括号时SLERP，否则只在短窗口内用机体系角速度预测。"""
    if not history:
        return None
    before, after = None, None
    for item in reversed(history):
        if item[0] <= stamp:
            before = item
            break
        else:
            after = item
    try:
        if before is not None and after is not None:
            if not all(A.finite(x) for x in before[2]+after[2]):
                return None
            gap = after[0]-before[0]
            if gap > C.MAX_ATTITUDE_GAP_US:
                return None
            fraction = (stamp-before[0])/gap
            q = A.slerp(A.normalize_q(before[1]), A.normalize_q(after[1]), fraction)
            gyro = tuple(x+fraction*(y-x) for x, y in zip(before[2], after[2]))
            return q, gyro, min(stamp-before[0], after[0]-stamp)
        item = before if before is not None else after
        delta = stamp-item[0]
        if abs(delta) > C.MAX_ATTITUDE_EXTRAPOLATION_US:
            return None
        if not all(A.finite(x) for x in item[2]):
            return None
        return A.predict_q(A.normalize_q(item[1]), item[2], delta*1e-6), item[2], abs(delta)
    except (ValueError, ZeroDivisionError):
        return None


def range_bounds(low, high):
    return (low if C.MIN_RANGE_M is None else max(low, C.MIN_RANGE_M),
            high if C.MAX_RANGE_M is None else min(high, C.MAX_RANGE_M))


def range_invalid_detail(distance, low, high, quality):
    if not A.finite(distance):
        return "NOT_FINITE"
    if not A.finite(low) or not A.finite(high) or high <= low:
        return "LIMITS_INVALID"
    if quality == 1:
        return "SIGNAL_INVALID"
    lower, upper = range_bounds(low, high)
    if distance <= 0 or distance < lower:
        return "TOO_NEAR"
    if distance > upper:
        return "TOO_FAR"
    return None


class HeightTracker:
    def __init__(self):
        self.samples = History()  # (fc_us, body_height, sigma_m)
        self.last_stamp = -1
        self.valid = False
        self.good = 0
        self.reason = "WAIT_RANGE"
        self.reject_detail = None
        self.skipped_attitude = 0

    def reject(self, reason, detail=None):
        self.valid = False
        self.good = 0
        self.reason = reason
        self.reject_detail = detail

    def consume(self, link, tof_pos):
        if link.range_ambiguous:
            self.reject("SELECT_TOF_ID")
            return
        for item in link.ranges:
            stamp, distance, low, high, quality, variance, received = item
            if stamp <= self.last_stamp:
                continue
            detail = range_invalid_detail(distance, low, high, quality)
            if detail is not None:
                self.last_stamp = stamp
                self.reject("RANGE_INVALID", detail)
                continue
            # 原始距离已通过检查，不能继续显示更早样本的TOO_NEAR等原因。
            self.reject_detail = None
            attitude = attitude_at(link.attitudes, stamp)
            if attitude is None:
                if link.attitudes and stamp <= link.attitudes[-1][0]:
                    # 姿态按时间有序入队；已落在旧数据或已知间隙中的测距，
                    # 未来新姿态无法补齐。消费它并继续，避免永久挡住更新样本。
                    self.last_stamp = stamp
                    self.skipped_attitude += 1
                    if not self.valid:
                        self.reason = "RANGE_ATTITUDE_GAP"
                    continue
                # 只有尚未到达姿态覆盖范围的测距需要等待。
                if not self.valid:
                    self.reason = "WAIT_RANGE_ATTITUDE"
                break
            self.last_stamp = stamp
            r = A.q_matrix(attitude[0])
            down = A.rotate(r, C.TOF_AXIS_B)[2]
            if (down <= C.MIN_RAY_DOWN or (C.MAX_TILT_DEG is not None
                    and down < math.cos(C.MAX_TILT_DEG*math.pi/180))):
                self.reject("RANGE_TILT")
                continue
            height = distance*down+A.rotate(r, tof_pos)[2]
            sigma = max(C.RANGE_NOISE_M+C.RANGE_RELATIVE_NOISE*distance,
                        math.sqrt(max(0.0, variance)) if variance is not None else 0.0)
            if self.samples:
                previous = self.samples[-1]
                dt = max(0.0, (stamp-previous[0])*1e-6)
                gate = C.RANGE_JUMP_BASE_M+3*sigma+C.MAX_VERTICAL_SPEED_M_S*dt
                if dt < 0.5 and abs(height-previous[1]) > gate:
                    self.reject("RANGE_JUMP")
                    continue
            self.samples.append((stamp, height, sigma))
            self.good += 1
            self.valid = self.good >= C.HEIGHT_RECOVERY_SAMPLES
            self.reason = "OK" if self.valid else "RANGE_RECOVERING"
            self.reject_detail = None

    def at(self, stamp):
        if not self.valid or not self.samples:
            return None
        before, after = None, None
        for item in reversed(self.samples):
            if item[0] <= stamp:
                before = item
                break
            else:
                after = item
        if before is not None and after is not None:
            if after[0]-before[0] > 2*C.MAX_RANGE_AGE_US:
                return None
            f = (stamp-before[0])/(after[0]-before[0])
            return (before[1]+f*(after[1]-before[1]),
                    max(before[2], after[2]), min(stamp-before[0], after[0]-stamp))
        item = before if before is not None else after
        delta = stamp-item[0]
        if abs(delta) > C.MAX_RANGE_AGE_US:
            return None
        velocity = 0.0
        if len(self.samples) >= 2:
            a, b = self.samples[-2], self.samples[-1]
            dt = (b[0]-a[0])*1e-6
            if dt > 0:
                velocity = max(-C.MAX_VERTICAL_SPEED_M_S,
                               min(C.MAX_VERTICAL_SPEED_M_S, (b[1]-a[1])/dt))
        sigma = item[2]+abs(delta)*1e-6*0.3
        return item[1]+velocity*delta*1e-6, sigma, abs(delta)


class VOController:
    def __init__(self, engine=None):
        self.engine = engine if engine is not None else A.FeatureEngine()
        self.height = HeightTracker()
        self.reset_counter = 0
        self.keyframe_id = 0
        self.good_frames = 0
        self.generation = None
        self.time_generation = None
        self.previous_attitude = None
        self.previous_clipping = None
        self.clipping_until = 0
        self.lost_since = None

    def reset(self):
        self.engine.reset()
        self.good_frames = 0
        self.previous_attitude = None
        self.reset_counter = (self.reset_counter+1) & 255
        self.keyframe_id = 0  # IDs are local to the reset_counter epoch.
        self.lost_since = None

    def _result(self, stamp, reason, q=None, height=None, approximate=False):
        result = {"timestamp_us": stamp, "position": None, "q": q,
                  "height": height, "quality": 0, "valid": False,
                  "body_valid": False, "reason": reason, "approximate": approximate,
                  "reset_counter": self.reset_counter, "variance": (1.0, 1.0),
                  "keyframe": self.keyframe_id, "matches": 0, "inliers": 0,
                  "residual_px": None, "sparse": False}
        result["range_detail"] = self.height.reject_detail
        result["range_state"] = self.height.reason
        result["range_skipped_attitude"] = self.height.skipped_attitude
        result["vision"] = None
        return result

    def process(self, image, stamp, link, statistics=None):
        now = link.clock.now_us()
        if self.generation is None:
            self.generation = link.boot_generation
        elif link.boot_generation != self.generation:
            self.reset()
            self.height = HeightTracker()
            self.previous_clipping = None
            self.generation = link.boot_generation
        if self.time_generation is None:
            self.time_generation = link.time_generation
        elif link.time_generation != self.time_generation:
            self.reset()
            self.height = HeightTracker()
            self.time_generation = link.time_generation
        if not link.connected(now):
            self.good_frames = 0
            return self._result(stamp, "WAIT_HEARTBEAT")
        if not link.synced(now):
            self.good_frames = 0
            return self._result(stamp, "TIME_UNSYNCED")
        fc_stamp = link.fc_time(stamp)
        attitude = attitude_at(link.attitudes, fc_stamp)
        if attitude is None:
            self.good_frames = 0
            return self._result(stamp, "WAIT_ATTITUDE")
        q, gyro, attitude_age = attitude
        r = A.q_matrix(q)
        camera_pos = C.CAMERA_POS_B_M
        tof_pos = C.TOF_POS_B_M
        approximate = (camera_pos is None or tof_pos is None or not C.CAMERA_ORIENTATION_CONFIRMED)
        if approximate and not C.ALLOW_APPROXIMATE_EXTRINSICS:
            return self._result(stamp, "EXTRINSICS_PENDING", q=q, approximate=True)
        camera_pos = (0.0, 0.0, 0.0) if camera_pos is None else camera_pos
        tof_pos = (0.0, 0.0, 0.0) if tof_pos is None else tof_pos
        self.height.consume(link, tof_pos)
        height_sample = self.height.at(fc_stamp)
        hc = None if height_sample is None else height_sample[0]-A.rotate(r, camera_pos)[2]
        # 调试显示机体参考点的离地高度；相机高度hc仅供投影计算。
        body_height = None if height_sample is None else height_sample[0]
        result = self._result(stamp, "WAIT_RANGE", q, body_height, approximate)
        if hc is None or hc <= 0.001:
            self.good_frames = 0
            result["height"] = None
            result["reason"] = self.height.reason if not self.height.valid else "RANGE_STALE"
            return result
        speed = math.sqrt(sum(x*x for x in gyro))
        if self.previous_attitude is not None:
            old_t, old_q, old_gyro = self.previous_attitude
            dt = max(0.0, (fc_stamp-old_t)*1e-6)
            expected = 1.5*max(speed, math.sqrt(sum(x*x for x in old_gyro)))*dt
            if dt < 0.5 and A.q_angle(q, old_q) > expected+C.ATTITUDE_RESET_MARGIN_RAD:
                self.reset()
                result.update(reset_counter=self.reset_counter, reason="ATTITUDE_RESET", height=None)
                self.height = HeightTracker()
                return result
        self.previous_attitude = (fc_stamp, q, gyro)
        if ((C.MAX_TILT_DEG is not None and r[8] < math.cos(C.MAX_TILT_DEG*math.pi/180))
                or (C.MAX_GYRO_RAD_S is not None and speed > C.MAX_GYRO_RAD_S)):
            self.good_frames = 0
            result["reason"] = "DYNAMIC_LIMIT"
            result["height"] = None
            return result
        if link.vibration is not None and now-link.vibration[0] < 1000000:
            _, metrics, clipping = link.vibration
            if self.previous_clipping is not None and any(x > y for x, y in zip(clipping, self.previous_clipping)):
                self.clipping_until = now+C.CLIPPING_HOLD_US
            self.previous_clipping = clipping
            excessive = (C.VIBRATION_LIMITS is not None
                         and any(x > limit for x, limit in zip(metrics, C.VIBRATION_LIMITS)))
            if now < self.clipping_until or excessive:
                self.good_frames = 0
                result["reason"] = "VIBRATION"
                result["height"] = None
                return result
        if C.MIN_IMAGE_STDEV is not None and statistics is None:
            statistics = image.get_statistics()
        if C.MIN_IMAGE_STDEV is not None and statistics.stdev() < C.MIN_IMAGE_STDEV:
            self.good_frames = 0
            result["reason"] = "LOW_TEXTURE"
            return result
        motion = self.engine.process(image, q, hc, camera_pos, stamp)
        result["vision"] = getattr(self.engine, "diagnostics", None)
        result["matches"] = motion["matches"]
        result["inliers"] = motion["inliers"]
        result["keyframe"] = self.keyframe_id = motion["keyframe"]
        if motion["status"] == 1:
            self.good_frames = 0
            self.lost_since = None
            result["reason"] = "INITIALIZING"
            return result
        if motion["status"] != 2:
            self.good_frames = 0
            detail = (result["vision"] or {}).get("vision_reason")
            result["reason"] = ("NO_FEATURES" if motion["status"] == -1 else detail
                                if detail in ("MATCH_COUNT", "INLIER_RATIO", "AMBIGUOUS_MATCH",
                                              "PATCH_UNRELIABLE", "INPUT_INVALID") else "LOST")
            if self.lost_since is None:
                self.lost_since = stamp
            elif stamp-self.lost_since >= C.LOST_RESET_US:
                # Unknown displacement is never attached to the last position.
                # A new zero origin is explicit and visible in reset_counter.
                self.reset()
                result.update(reason="REINITIALIZING", reset_counter=self.reset_counter,
                              keyframe=self.keyframe_id)
            return result
        self.lost_since = None
        delta = motion["delta"]
        position = motion["position"]
        count = motion["inliers"]
        # 公共姿态/测距误差不按内点数量消掉。
        distance = math.sqrt(delta[0]**2+delta[1]**2)
        rotation_sigma = C.ATTITUDE_STD_RAD+speed*(attitude_age+C.SYNC_MAX_JITTER_US)*1e-6
        reference_height = motion["reference_height"]
        sigma_common2 = ((max(hc, reference_height)*rotation_sigma)**2
                         +(distance*height_sample[1]/max(hc, 0.1))**2
                         +C.POSITION_NOISE_FLOOR_M**2)
        # A tiny consensus residual does not prove precise tracking. Independent
        # pixel noise has a floor; shared pose/range/time errors are not divided
        # by feature count. Sparse observations receive extra conservatism.
        pixel_var = (max(hc, reference_height)*getattr(self.engine, "pixel_sigma", C.PIXEL_STD)/min(C.FX, C.FY))**2
        effective = max(0.5, motion.get("effective_count", count))
        inflation = (C.TWO_POINT_VARIANCE_FACTOR if count == 2 else
                     C.THREE_POINT_VARIANCE_FACTOR if count == 3 else 1.0)
        variance = tuple(motion["base_variance"][i]+sigma_common2
                         +inflation*max(pixel_var, motion["sample_variance"][i])/effective
                         for i in range(2))
        freshness = max(0.1, 1.0-0.5*height_sample[2]/C.MAX_RANGE_AGE_US)
        geometric = min(1.0, count/30.0)*motion["ratio"]
        geometric *= max(0.1, 1.0-0.5*motion["residual_px"]/C.RANSAC_THRESHOLD_PX)
        geometric *= (.3+.7*min(1.0, motion["cells"]/6.0))
        geometric *= (.3+.7*min(1.0, motion["spread"]/.12))
        geometric *= min(1.0, effective/count)
        quality = max(1, min(100, int(100*geometric*freshness)))
        sparse = count <= C.SPARSE_MAX_INLIERS
        if count == 2:
            quality = min(quality, C.TWO_POINT_QUALITY_CAP)
        elif count == 3:
            quality = min(quality, C.THREE_POINT_QUALITY_CAP)
        if approximate:
            quality = min(quality, C.APPROXIMATE_QUALITY_CAP)
            variance = tuple(v+0.1**2 for v in variance)
        self.engine.accept_variance(motion, variance)
        self.good_frames += 1
        result.update(position=position, variance=variance, matches=motion["matches"],
                      inliers=count, residual_px=motion["residual_px"], quality=quality,
                      reason="SPARSE_TRACKING" if sparse else "TRACKING", sparse=sparse,
                      attitude_age_us=attitude_age,
                      range_age_us=height_sample[2], residual_yaw=motion["residual_yaw"])
        if self.good_frames < C.POSE_RECOVERY_FRAMES:
            result.update(position=None, quality=0, reason="POSE_RECOVERING")
            return result
        result["valid"] = True
        result["body_valid"] = not approximate
        return result
