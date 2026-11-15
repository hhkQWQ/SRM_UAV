"""手动粗导航后的Mission保持点准备；纯MicroPython，不解锁、不切模式。

显式NAV_ENABLE+MISSION_ENABLE后，稳定观测期间只上传当前位置的单项
LOITER_UNLIM，供操作者切Mission。手动移动后重新准备，故障锁止时禁止提示就绪。
"""
import math
import struct
import config as C
from pose_solver import finite, norm


class MissionStandby:
    STABLE_US = 1200000
    GLOBAL_MAX_AGE_US = 1000000
    RETRY_US = 1500000
    UPLOAD_TIMEOUT_US = 20000000
    MAX_RETRIES = 5
    ANCHOR_MAX_AGE_US = 10000000
    ANCHOR_XY_REFRESH_M = .25
    ANCHOR_Z_REFRESH_M = .15

    def __init__(self, link):
        self.link = link
        self.state = "WAIT_MANUAL_FLIGHT"
        self.error = None
        self.anchor = None
        self.global_position = None
        self.stable_since = self.last_measurement_ms = None
        self.last_real_measurement_ms = None
        self.real_count = 0
        self.track_generation = None
        self.link_generation = link.generation
        self.link_target = link.target
        self.upload_target = None
        self.upload_started_us = self.last_action_us = None
        self.last_global_request_us = None
        self.last_update_us = None
        self.item_sent = self.integer_item = False
        self.complete = self.locked = False
        self.retries = self.attempts = 0
        self.blocked_reason = None

    def _result(self):
        return {"state":self.state,"error":self.error,"ready":self.complete,
                "anchor":self.anchor,"attempts":self.attempts}

    def _reset_stability(self):
        self.stable_since = self.last_measurement_ms = None
        self.last_real_measurement_ms = None
        self.real_count = 0
        self.track_generation = None

    def _cancel(self):
        cancel = getattr(self.link,"cancel_mission_transfer",None)
        if cancel is not None:
            cancel()
        # PX4 honours OPERATION_CANCELLED while receiving a mission. This
        # abandons an incomplete transfer; it does not clear stored missions.
        if self.upload_target is not None and self.upload_target == self.link.target:
            self.link.send_message(47, struct.pack("<4B",self.upload_target[0],
                                                  self.upload_target[1],15,0))
        self.upload_target = None
        self.upload_started_us = self.last_action_us = None
        self.item_sent = False

    def _interrupt(self, reason):
        if self.upload_started_us is not None:
            self._cancel()
            self.error = "UPLOAD_ABORTED_"+reason
        self.anchor = None
        self._reset_stability()
        self.state = reason

    def _fail(self, reason):
        self._cancel()
        self.locked = True
        self.error = reason
        self.state = "ABORTED"
        self._reset_stability()

    def _count(self, now):
        system,component = self.upload_target
        accepted = self.link.send_message(44,struct.pack("<HBBB",1,system,component,0))
        self.last_action_us = now
        self.attempts += 1
        return accepted

    def _item(self, now, integer):
        system,component = self.upload_target
        point = self.anchor
        # LOITER_UNLIM has no automatic continuation. Its yaw is the measured
        # aircraft heading at staging, with no target map position required.
        params = (0.,0.,0.,point["yaw_deg"])
        tail = (0,17,system,component,6 if integer else 3,1,0,0)
        if integer:
            payload = struct.pack("<4fiifHH6B",*(params+(
                point["latitude_int"],point["longitude_int"],point["relative_alt_m"])+tail))
            mid = 73
        else:
            payload = struct.pack("<7fHH6B",*(params+(
                point["latitude_int"]/1e7,point["longitude_int"]/1e7,
                point["relative_alt_m"])+tail))
            mid = 39
        accepted = self.link.send_message(mid,payload)
        self.last_action_us = now
        self.attempts += 1
        if accepted:
            self.item_sent,self.integer_item = True,integer
        return accepted

    def _events(self, events, now, allow_transfer):
        for mid,system,component,payload,received in events:
            if self.link.target != (system,component) or not 0 <= now-received <= self.GLOBAL_MAX_AGE_US:
                continue
            if mid == 33:
                values = struct.unpack("<I4i",(payload+b"\0"*20)[:20])
                boot_ms = getattr(self.link,"boot_ms",None)
                if boot_ms is not None:
                    sample_delta_ms = ((values[0]-boot_ms+(1<<31))&0xffffffff)-(1<<31)
                    if not -self.GLOBAL_MAX_AGE_US//1000 <= sample_delta_ms <= 50:
                        continue
                latitude,longitude,relative_alt = values[1],values[2],values[4]
                if (-900000000 <= latitude <= 900000000 and
                        -1800000000 <= longitude <= 1800000000 and
                        -1000000 <= relative_alt <= 100000000):
                    self.global_position = (received,latitude,longitude,relative_alt)
            elif allow_transfer and self.state == "UPLOADING" and mid in (40,51):
                # mission_type is a zero-valued extension for ordinary flight
                # plans; MAVLink2 may truncate that and other trailing zeros.
                p = (payload+b"\0"*5)[:5]
                sequence,target_system,target_component,mission_type = struct.unpack("<HBBB",p)
                if (target_system != C.SOURCE_SYSTEM or target_component != C.SOURCE_COMPONENT
                        or mission_type != 0):
                    continue
                if received < self.upload_started_us:
                    continue
                if sequence != 0:
                    self._fail("MISSION_REQUEST_OUT_OF_RANGE")
                    return
                self._item(now,mid == 51)
                self.retries = 0
            elif allow_transfer and self.state == "UPLOADING" and mid == 47:
                p = (payload+b"\0"*4)[:4]
                target_system,target_component,result,mission_type = struct.unpack("<4B",p)
                if (target_system != C.SOURCE_SYSTEM or target_component != C.SOURCE_COMPONENT
                        or mission_type != 0 or received < self.upload_started_us):
                    continue
                if result != 0:
                    self._fail("MISSION_ACK_%d" % result)
                    return
                if not self.item_sent:
                    self._fail("MISSION_ACK_BEFORE_ITEM")
                    return
                self.complete = True
                self.state,self.error = "READY_FOR_MISSION",None
                self.upload_target = None
                self.upload_started_us = self.last_action_us = None

    def _healthy_reason(self, track, flight, now, configured):
        if not configured or not flight.get("ready") or self.link.target is None:
            return "WAIT_READY"
        if not flight.get("armed"):
            return "WAIT_MANUAL_FLIGHT"
        if flight.get("mission") or flight.get("offboard"):
            return "WAIT_PILOT_MODE"
        position,velocity = track.get("position"),flight.get("velocity")
        measurement = track.get("measurement_ms")
        short_prediction = (track.get("state") == "PREDICTED" and self.stable_since is not None
                            and self.last_real_measurement_ms is not None
                            and 0 <= now//1000-self.last_real_measurement_ms <= C.MAX_PREDICTION_MS)
        if ((track.get("state") != "MEASURED" and not short_prediction) or measurement is None
                or not 0 <= now//1000-measurement <= C.MAX_PREDICTION_MS
                or position is None or len(position) != 3 or not finite(position)
                or not C.MIN_DEPTH_M < position[2] <= C.MAX_DEPTH_M
                or velocity is None or len(velocity) != 3 or not finite(velocity)
                or norm(velocity) > .20 or not math.isfinite(flight.get("yaw",float("nan")))):
            return "WAIT_STABLE_TARGET"
        return None

    def _anchor_stale(self, now):
        if self.anchor is None or self.global_position is None:
            return True
        received,latitude,longitude,altitude = self.global_position
        if not 0 <= now-received <= self.GLOBAL_MAX_AGE_US:
            return True
        if not 0 <= now-self.anchor["captured_us"] <= self.ANCHOR_MAX_AGE_US:
            return True
        radians = math.pi/(180.*1e7)
        north = (latitude-self.anchor["latitude_int"])*radians*6378137.
        east = ((longitude-self.anchor["longitude_int"])*radians*6378137.*
                math.cos(latitude*radians))
        return (norm((north,east)) > self.ANCHOR_XY_REFRESH_M or
                abs(altitude/1000.-self.anchor["relative_alt_m"]) > self.ANCHOR_Z_REFRESH_M)

    def update(self, track, flight, now_us, configured, events=None, handover=None):
        """events可共享FlightLink.drain_events的结果，避免多个消费者抢占。"""
        events = self.link.drain_events() if events is None else events
        enabled = C.NAV_ENABLE and getattr(C,"MISSION_ENABLE",False)
        if not enabled:
            if self.upload_started_us is not None:
                self._cancel()
            self._reset_stability()
            # Preserve a terminal lock across enable/disable toggles.
            if self.locked:
                return dict(self._result(),state="DISABLED")
            self.state = "DISABLED"
            return self._result()
        changed = (self.link_generation != self.link.generation or self.link_target != self.link.target)
        if changed:
            self.link_generation,self.link_target = self.link.generation,self.link.target
            if self.complete:
                self.locked,self.complete = True,False
                self.state,self.error = "LINK_RESET","STAGED_MISSION_REQUIRES_RECHECK"
                return self._result()
            self._interrupt("WAIT_READY")
            self.global_position = None
            self.last_global_request_us = self.last_update_us = None
        # 就绪必须同时意味着视觉接管器可用，不能仅表示曾上传过一个任务。
        self._events(events,now_us,False)
        if handover is not None and handover.locked:
            if self.upload_started_us is not None:
                self._cancel()
            self.complete = False
            self.anchor = None
            self._reset_stability()
            self.blocked_reason = handover.lock_reason or handover.state
            self.state,self.error = "BLOCKED_"+self.blocked_reason,self.blocked_reason
            return self._result()
        if self.blocked_reason is not None:
            # 操作者切Position、健康恢复后重新准备当前位置，不复用上一轮的保持点。
            self.blocked_reason = None
            self.complete,self.locked = False,False
            self.anchor = None
            self.error = None
            self._reset_stability()
            self.state = "WAIT_STABLE_TARGET"
        if self.locked:
            return self._result()
        if self.complete:
            if not flight.get("armed"):
                self.locked,self.complete = True,False
                self.state,self.error = "STOPPED","STAGED_MISSION_REQUIRES_RECHECK_AFTER_DISARM"
            elif flight.get("mission"):
                self.state = "MISSION_SELECTED"
            elif flight.get("offboard"):
                self.state = "VISION_ACTIVE"
            else:
                measurement = track.get("measurement_ms")
                if (track.get("state") == "MEASURED" and measurement is not None and
                        0 <= now_us//1000-measurement <= C.MAX_PREDICTION_MS):
                    self.last_real_measurement_ms = measurement
                reason = self._healthy_reason(track,flight,now_us,configured)
                if reason is not None:
                    self.state = reason
                    return dict(self._result(),ready=False)
                if self._anchor_stale(now_us):
                    self.complete = False
                    self.anchor = None
                    self._reset_stability()
                    self.state = "WAIT_STABLE_TARGET"
                else:
                    self.state = "READY_FOR_MISSION"
            if self.complete or self.locked:
                return self._result()
        reason = self._healthy_reason(track,flight,now_us,configured)
        measurement = track.get("measurement_ms")
        generation = track.get("generation")
        track_changed = (generation != self.track_generation or
                         (self.last_measurement_ms is not None and measurement is not None and
                          not 0 <= measurement-self.last_measurement_ms <= C.MAX_PREDICTION_MS))
        if self.state == "UPLOADING" and track_changed:
            reason = "WAIT_STABLE_TARGET"
        loop_stale = (self.last_update_us is not None and
                      not 0 <= now_us-self.last_update_us <= C.MAX_PREDICTION_MS*1000)
        self.last_update_us = now_us
        if reason is not None or loop_stale:
            self._interrupt(reason or "WAIT_STABLE_TARGET")
            # Continue receiving GPS, but never fulfil mission requests when
            # manual flight, health, observation, or timing gates are absent.
            self._events(events,now_us,False)
            return self._result()
        if track_changed:
            self._reset_stability()
        self.track_generation,self.last_measurement_ms = generation,measurement
        real = track.get("state") == "MEASURED"
        if real and (self.last_real_measurement_ms is None or measurement > self.last_real_measurement_ms):
            self.last_real_measurement_ms = measurement
            self.real_count += 1
        self._events(events,now_us,real)
        if self.complete or self.locked:
            return self._result()
        if (self.last_global_request_us is None or now_us-self.last_global_request_us >= 2000000):
            if (self.global_position is None or
                    not 0 <= now_us-self.global_position[0] <= self.GLOBAL_MAX_AGE_US):
                self.link.send_command(511,(33.,100000.))
                self.last_global_request_us = now_us
        if self.state == "UPLOADING":
            if not real:
                return self._result()
            if now_us-self.upload_started_us > self.UPLOAD_TIMEOUT_US:
                self._fail("MISSION_UPLOAD_TIMEOUT")
            elif now_us-self.last_action_us >= self.RETRY_US:
                self.retries += 1
                if self.retries > self.MAX_RETRIES:
                    self._fail("MISSION_UPLOAD_TIMEOUT")
                elif self.item_sent:
                    self._item(now_us,self.integer_item)
                else:
                    self._count(now_us)
            return self._result()
        if not real:
            self.state = "WAIT_STABLE_TARGET"
            return self._result()
        if self.stable_since is None:
            self.stable_since = now_us
        self.state = "WAIT_STABLE_TARGET"
        if now_us-self.stable_since < self.STABLE_US or self.real_count < 3:
            return self._result()
        if (self.global_position is None or
                not 0 <= now_us-self.global_position[0] <= self.GLOBAL_MAX_AGE_US):
            self.state = "WAIT_GLOBAL_POSITION"
            return self._result()
        _,latitude,longitude,relative_alt = self.global_position
        self.anchor = {"latitude_int":latitude,"longitude_int":longitude,
                       "relative_alt_m":relative_alt/1000.,
                       "captured_us":now_us,
                       "yaw_deg":flight["yaw"]*180./math.pi % 360.}
        self.upload_target = self.link.target
        self.state,self.error = "UPLOADING",None
        self.upload_started_us = now_us
        self.retries = 0
        self.item_sent = False
        self._count(now_us)
        return self._result()
