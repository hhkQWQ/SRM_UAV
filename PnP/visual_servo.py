"""相对中心1m悬停外环；只产生NED速度/yaw参考，不解锁、不切换模式。"""
import config as C
from pose_solver import finite, hypot2

def _bounded_xy(x, y, limit):
    length = hypot2(x,y)
    scale = min(1.,limit/length) if length > 0 else 1.
    return x*scale,y*scale

class VisualServo:
    def __init__(self):
        self.command = (0.,0.,0.)
        self.last_ms = None
        self.yaw = None
        self.lost_since = None
        self.arrival_since = None
        self.generation = None
        self.state = "DISABLED"

    def update(self, track, flight, now_ms, configured, enabled=None):
        enabled = C.NAV_ENABLE if enabled is None else enabled
        dt = 0. if self.last_ms is None else (now_ms-self.last_ms)*0.001
        self.last_ms = now_ms
        reason = None
        if not enabled:
            reason = "DISABLED"
        elif not configured:
            reason = "CONFIG_UNCONFIRMED"
        elif not flight.get("ready"):
            reason = flight.get("reason","FLIGHT_UNREADY")
        elif not flight.get("armed"):
            reason = "DISARMED"
        if reason is not None:
            self.state, self.command, self.yaw = reason,(0.,0.,0.),None
            self.lost_since = self.arrival_since = None
            return None
        if dt < 0 or dt > 0.25:
            self.command = (0.,0.,0.)
            self.state = "LOOP_STALE"
            self.lost_since = self.arrival_since = None
            self.yaw = None
            return None
        if self.yaw is None:
            self.yaw = flight["yaw"]
        if not flight.get("offboard"):
            # 启用后先用零速度预热；由操作者切入Offboard，避免进入条件循环依赖。
            self.state, self.command = "PRIMING",(0.,0.,0.)
            self.lost_since = self.arrival_since = None
            return self._output()
        p = track.get("position")
        if p is not None and (len(p) != 3 or not finite(p)):
            p = None
        if track.get("generation") != self.generation:
            self.generation = track.get("generation")
            self.arrival_since = None
        if p is not None and track.get("state") in ("MEASURED","PREDICTED"):
            self.lost_since = None
            ex,ey,ez = p[0],p[1],p[2]-C.TARGET_HEIGHT_M
            vx,vy = _bounded_xy(C.KP_XY*ex,C.KP_XY*ey,C.MAX_SPEED_XY)
            vz = max(-C.MAX_SPEED_Z,min(C.MAX_SPEED_Z,C.KP_Z*ez))
            desired = (vx,vy,vz)
            arrived = (hypot2(ex,ey) <= C.ARRIVAL_XY_M and abs(ez) <= C.ARRIVAL_Z_M)
            # 仅真实观测可建立/维持到位确认。
            if arrived and track["state"] == "MEASURED":
                if self.arrival_since is None:
                    self.arrival_since = now_ms
            else:
                self.arrival_since = None
            holding = self.arrival_since is not None and now_ms-self.arrival_since >= C.ARRIVAL_DWELL_MS
            self.state = "HOLD" if holding else ("PREDICT" if track["state"] == "PREDICTED" else "ALIGN")
        else:
            if self.lost_since is None:
                self.lost_since = now_ms
            self.arrival_since = None
            elapsed = now_ms-self.lost_since
            if elapsed > C.LOST_BRAKE_MS:
                self.state, self.command = "LOST_RELEASE",(0.,0.,0.)
                return None  # 不再发送；由PX4已配置的Offboard丢失处置接管
            desired = (0.,0.,0.)
            self.state = "BRAKING"
        dx,dy = _bounded_xy(desired[0]-self.command[0],desired[1]-self.command[1],C.MAX_ACCEL_XY*dt)
        dz = max(-C.MAX_ACCEL_Z*dt,min(C.MAX_ACCEL_Z*dt,desired[2]-self.command[2]))
        self.command = (self.command[0]+dx,self.command[1]+dy,self.command[2]+dz)
        return self._output()

    def _output(self):
        return {"velocity":self.command,"yaw":self.yaw,"state":self.state}
