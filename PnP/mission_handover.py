"""Mission开关触发视觉接管；不解锁、不起飞、不设置目标绝对坐标。

Mission阶段仅预热零速度参考，收到真实Offboard心跳后才让视觉外环运动。
人工退出、遥测异常或长时间丢靶后锁止；操作者切回Position后才能重新待命。
"""
import config as C
from pose_solver import finite, norm
from visual_servo import VisualServo


class MissionHandover:
    def __init__(self, servo=None, mission_seq=None, enabled=None):
        self.servo = servo or VisualServo()
        self.enabled = enabled
        self.mission_seq = mission_seq
        self.state = "WAIT_MISSION"
        self.since = self.prime_since = self.request_since = None
        self.last_ms = None
        self.generation = None
        self.track_generation = None
        self.last_real_measurement_ms = None
        self.real_count = 0
        self.attempts = 0
        self.engaged = self.locked = False
        self.yaw = None
        self.lock_reason = None
        self.rearm_since = self.rearm_last_ms = self.rearm_measurement = None
        self.rearm_generation = None
        self.rearm_count = 0
        self.hold_requested_ms = None
        self.hold_attempts = 0

    def _result(self, velocity=None, request_mode=None):
        return {"velocity":velocity,"yaw":self.yaw,"request_mode":request_mode,
                "state":self.state}

    def _clear_acquisition(self):
        self.since = self.prime_since = self.last_real_measurement_ms = None
        self.real_count = 0

    def _release(self, flight, reason):
        self.locked = True
        self.engaged = False
        self.state = reason
        self.lock_reason = reason
        self.rearm_since = self.rearm_last_ms = self.rearm_measurement = None
        self.rearm_count = 0
        self._clear_acquisition()
        # 模式命令仍需调用者通过FlightLink连接/显式启用门控。
        request = (4,3) if flight.get("armed") and (flight.get("offboard") or flight.get("mission")) else None
        self.hold_requested_ms = self.last_ms if request else None
        self.hold_attempts = 1 if request else 0
        return self._result(request_mode=request)

    def _locked_update(self, track, flight, now_ms, configured):
        # 只有操作者明确切回Position并重新稳定观测，才允许开始下一次任务。
        # 自动故障处置进入的Hold不解除锁止；留在Mission也不能重新抢控。
        p,v = track.get("position"),flight.get("velocity")
        measurement = track.get("measurement_ms")
        manual = flight.get("main_mode") == 3 and not flight.get("mission") and not flight.get("offboard")
        real = track.get("state") == "MEASURED"
        predicted = (track.get("state") == "PREDICTED" and self.rearm_since is not None
                     and self.rearm_measurement is not None
                     and 0 <= now_ms-self.rearm_measurement <= C.MAX_PREDICTION_MS)
        fresh = (manual and configured and flight.get("ready") and flight.get("armed")
                 and (real or predicted) and measurement is not None
                 and 0 <= now_ms-measurement <= C.MAX_PREDICTION_MS
                 and p is not None and len(p) == 3 and finite(p)
                 and C.MIN_DEPTH_M < p[2] <= C.MAX_DEPTH_M
                 and v is not None and len(v) == 3 and finite(v) and norm(v) <= .20)
        generation = (flight.get("generation"),track.get("generation"))
        if (not fresh or self.rearm_generation != generation or
                self.rearm_last_ms is not None and not 0 <= now_ms-self.rearm_last_ms <= 250):
            self.rearm_since = self.rearm_measurement = None
            self.rearm_count = 0
        self.rearm_generation,self.rearm_last_ms = generation,now_ms
        if fresh:
            if self.rearm_since is None:
                self.rearm_since = now_ms
            if real and (self.rearm_measurement is None or measurement > self.rearm_measurement):
                self.rearm_measurement = measurement
                self.rearm_count += 1
            self.state = "REARMING_POSITION"
            if real and now_ms-self.rearm_since >= 1000 and self.rearm_count >= 3:
                enabled,sequence = self.enabled,self.mission_seq
                self.__init__(mission_seq=sequence,enabled=enabled)
                self.last_ms,self.generation = now_ms,flight.get("generation")
                return self._result()
        else:
            self.state = self.lock_reason
        request = None
        if flight.get("armed") and flight.get("mission") and self.hold_attempts < 3:
            if self.hold_requested_ms is None or now_ms-self.hold_requested_ms >= 1000:
                self.hold_requested_ms = now_ms
                self.hold_attempts += 1
                request = (4,3)
        elif not flight.get("mission"):
            self.hold_requested_ms,self.hold_attempts = None,0
        return self._result(request_mode=request)

    def update(self, track, flight, now_ms, configured, mission_prepared=None):
        enabled = (C.NAV_ENABLE and getattr(C,"MISSION_ENABLE",False)
                   if self.enabled is None else self.enabled)
        if not enabled:
            self.state = "DISABLED"
            return None
        if self.locked:
            return self._locked_update(track,flight,now_ms,configured)
        if self.last_ms is not None and not 0 <= now_ms-self.last_ms <= 250:
            self.last_ms = now_ms
            if self.engaged:
                return self._release(flight,"LOOP_STALE")
            self._clear_acquisition()
        self.last_ms = now_ms
        generation = flight.get("generation")
        if self.generation is not None and generation != self.generation:
            if self.engaged:
                return self._release(flight,"LINK_RESET")
            self._clear_acquisition()
            self.request_since = None
            self.track_generation = None
            self.attempts = 0
        self.generation = generation
        if not configured or not flight.get("ready") or not flight.get("armed"):
            if self.engaged or flight.get("armed") and flight.get("mission"):
                return self._release(flight,flight.get("reason","FLIGHT_UNREADY"))
            self.state = "WAIT_READY"
            self._clear_acquisition()
            return self._result()
        if self.state == "ACTIVE":
            if not flight.get("offboard"):
                return self._release(flight,"PILOT_OVERRIDE")
            output = self.servo.update(track,flight,now_ms,configured,enabled=True)
            if output is None:
                return self._release(flight,self.servo.state)
            return dict(output,request_mode=None,phase="ACTIVE")
        if flight.get("offboard") and self.request_since is not None:
            self.state = "ACTIVE"
            self.servo = VisualServo()
            output = self.servo.update(track,flight,now_ms,configured,enabled=True)
            if output is None:
                return self._release(flight,self.servo.state)
            return dict(output,request_mode=None,phase="ACTIVE")
        if not flight.get("mission"):
            if self.engaged:
                return self._release(flight,"PILOT_OVERRIDE")
            self.state = "WAIT_MISSION"
            self._clear_acquisition()
            return self._result()
        if mission_prepared is False:
            return self._release(flight,"MISSION_NOT_READY")
        if self.mission_seq is not None and flight.get("mission_seq") != self.mission_seq:
            if self.engaged:
                return self._release(flight,"MISSION_ITEM_CHANGED")
            self.state = "WAIT_CAPTURE_WAYPOINT"
            self._clear_acquisition()
            return self._result()
        p,v = track.get("position"),flight.get("velocity")
        measurement = track.get("measurement_ms")
        track_gen = track.get("generation")
        if self.track_generation != track_gen:
            if self.request_since is not None:
                return self._release(flight,"TARGET_CHANGED_DURING_TRANSFER")
            self._clear_acquisition()
            self.track_generation = track_gen
        real = track.get("state") == "MEASURED"
        short_prediction = (track.get("state") == "PREDICTED" and self.since is not None
                            and self.last_real_measurement_ms is not None
                            and 0 <= now_ms-self.last_real_measurement_ms <= C.MAX_PREDICTION_MS)
        fresh = ((real or short_prediction) and measurement is not None
                 and 0 <= now_ms-measurement <= C.MAX_PREDICTION_MS
                 and p is not None and len(p) == 3 and finite(p)
                 and C.MIN_DEPTH_M < p[2] <= C.MAX_DEPTH_M
                 and v is not None and len(v) == 3 and finite(v) and norm(v) <= .20)
        if not fresh:
            self._clear_acquisition()
            self.state = "WAIT_TARGET"
            return self._release(flight,"TARGET_LOST_DURING_TRANSFER" if self.request_since is not None
                                 else "MISSION_TARGET_UNREADY")
        if real and (self.last_real_measurement_ms is None or measurement > self.last_real_measurement_ms):
            self.last_real_measurement_ms = measurement
            self.real_count += 1
        if self.since is None:
            self.since = now_ms
        self.state = "ACQUIRING"
        if not real:
            # 短预测只保留已建立的稳定窗/零速度预热；不能启动或触发模式切换。
            if self.prime_since is not None:
                self.state = "PRIMING"
                return self._result((0.,0.,0.))
            return self._result()
        if now_ms-self.since < 1000 or self.real_count < 3:
            return self._result()
        self.engaged = True
        if self.prime_since is None:
            self.prime_since = now_ms
            self.yaw = flight["yaw"]
        self.state = "PRIMING"
        request = None
        if now_ms-self.prime_since >= 1200:
            self.state = "REQUEST_OFFBOARD"
            if self.request_since is None or now_ms-self.request_since >= 1000:
                if self.attempts >= 3:
                    return self._release(flight,"OFFBOARD_REJECTED")
                request = (6,0)
                self.request_since = now_ms
                self.attempts += 1
        return self._result((0.,0.,0.),request)
