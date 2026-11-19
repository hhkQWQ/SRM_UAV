"""单个装甲板的中心关联与静止目标相对位置跟踪。时间为展开后的本地ms。"""
import config as C
from pose_solver import finite, norm, hypot2

class TargetTracker:
    def __init__(self):
        self.center = None
        self.pixel_velocity = (0.,0.)
        self.color = None
        self.size = 0.
        self.pending = None
        self.center_misses = 0
        self.p = None
        self.pose_misses = 0
        self.measurement_ms = None
        self.last_ms = None
        self.velocity = None
        self.generation = 0
        self.position_pending = None

    def _pending_step(self, detection):
        if detection is None:
            self.pending = None
            return False
        center, color, size = detection["center"], detection["color"], detection["size"]
        p = self.pending
        gate = min(110.,35.+1.2*size)
        if p is not None and p[1] == color and hypot2(center[0]-p[0][0],center[1]-p[0][1]) <= gate:
            count = p[2]+1
        else:
            count = 1
        self.pending = (center,color,count)
        if count < C.CONFIRM_FRAMES:
            return False
        self.center, self.color, self.size = tuple(center), color, size
        self.pixel_velocity = (0.,0.)
        self.center_misses = 0
        self.pending = None
        self.p, self.measurement_ms = None, None
        self.position_pending = None
        self.pose_misses = 0
        self.generation += 1
        return True

    def update(self, detection, now_ms, relative_local=None, velocity_local=None):
        if detection is not None:
            center,size = detection.get("center"),detection.get("size")
            if (center is None or len(center) != 2 or not finite(center) or
                    size is None or not finite((size,)) or size <= 0):
                detection = None
                relative_local = None
        if velocity_local is not None and (len(velocity_local) != 3 or not finite(velocity_local)):
            velocity_local = None
        dt = 0. if self.last_ms is None else (now_ms-self.last_ms)*0.001
        # 同一帧不能重复累计确认次数或刷新真实观测时间。
        if self.last_ms is not None and now_ms == self.last_ms:
            return self._result(now_ms,False,False,velocity_local is not None)
        # 控制延迟或时钟跳变不能被当作短时预测。
        if dt < 0 or dt > C.MAX_PREDICTION_MS*0.001:
            self.center, self.p, self.pending = None,None,None
            self.measurement_ms = None
            self.velocity = None
            self.position_pending = None
            self.generation += 1
            dt = 0.
        self.last_ms = now_ms
        if self.p is not None and velocity_local is not None:
            previous_v = velocity_local if self.velocity is None else self.velocity
            self.p = tuple(self.p[i]-0.5*(previous_v[i]+velocity_local[i])*dt for i in range(3))
        self.velocity = velocity_local
        if self.center is not None:
            predicted = tuple(self.center[i]+self.pixel_velocity[i]*dt for i in range(2))
            gate = min(100.,10.+0.6*self.size+0.9*norm(self.pixel_velocity)*dt)
            matched = (detection is not None and detection["color"] == self.color
                       and hypot2(detection["center"][0]-predicted[0],
                                      detection["center"][1]-predicted[1]) <= gate)
            if matched:
                residual = tuple(detection["center"][i]-predicted[i] for i in range(2))
                self.center = tuple(predicted[i]+C.TRACK_ALPHA*residual[i] for i in range(2))
                if dt > 1e-4:
                    self.pixel_velocity = tuple(self.pixel_velocity[i]+C.TRACK_BETA*residual[i]/dt for i in range(2))
                self.size = detection["size"]
                self.center_misses = 0
                self.pending = None
            else:
                self.center = predicted
                self.pixel_velocity = tuple(v*0.92 for v in self.pixel_velocity)
                self.center_misses += 1
                matched = self._pending_step(detection)
                if not matched and self.center_misses > C.MAX_LOST_FRAMES:
                    self.center = None
        else:
            matched = self._pending_step(detection)
        measured = False
        if matched and relative_local is not None and len(relative_local) == 3 and finite(relative_local):
            fresh_track = self.p is None or self.measurement_ms is None
            accepted = not fresh_track and norm(tuple(relative_local[i]-self.p[i] for i in range(3))) <= C.POSITION_JUMP_M
            if fresh_track:
                pending = self.position_pending
                count = 1
                estimate = tuple(relative_local)
                if pending is not None and 0 < now_ms-pending[1] <= C.MAX_PREDICTION_MS:
                    prediction = pending[0]
                    if velocity_local is not None:
                        elapsed = (now_ms-pending[1])*0.001
                        prediction = tuple(prediction[i]-velocity_local[i]*elapsed for i in range(3))
                    if norm(tuple(relative_local[i]-prediction[i] for i in range(3))) <= C.POSITION_JUMP_M:
                        count = pending[2]+1
                        estimate = tuple(prediction[i]+(relative_local[i]-prediction[i])/count for i in range(3))
                self.position_pending = (estimate,now_ms,count)
                accepted = count >= C.POSE_CONFIRM_FRAMES
            if accepted:
                a = 1. if fresh_track else C.POSITION_ALPHA
                self.p = estimate if fresh_track else tuple((1-a)*self.p[i]+a*relative_local[i] for i in range(3))
                self.measurement_ms, self.pose_misses, measured = now_ms,0,True
                self.position_pending = None
        else:
            self.position_pending = None
        if not measured:
            self.pose_misses += 1
        return self._result(now_ms,measured,matched,velocity_local is not None)

    def _result(self, now_ms, measured, matched, velocity_available):
        age = None if self.measurement_ms is None else now_ms-self.measurement_ms
        usable = (self.center is not None and self.p is not None and age is not None
                  and 0 <= age <= C.MAX_PREDICTION_MS and self.pose_misses <= C.MAX_LOST_FRAMES
                  and (measured or velocity_available))
        if not usable:
            self.p = None
        # 尚在确认中心或只有二维中心时，都属于数据不足的ACQUIRING。
        state = "MEASURED" if measured else ("PREDICTED" if usable else
                ("ACQUIRING" if self.center is not None or self.pending is not None else "LOST"))
        return {"state":state, "position":self.p if usable else None,
                "center":self.center, "size":self.size, "color":self.color, "age_ms":age,
                "measurement_ms":self.measurement_ms, "generation":self.generation,
                "pose_misses":self.pose_misses, "matched":matched}
