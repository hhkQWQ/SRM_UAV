"""将本目录运行模块放到OpenMV根目录；config默认仅观察。"""
import math
import os
import config as C
from pose_solver import (PlanarPoseSolver, finite, rotation_valid, local_to_camera,
                         relative_measurement, quaternion_rotation, mv)
from target_tracker import TargetTracker
from visual_servo import VisualServo
from flight_link import Clock, FlightLink

# 只记录诊断数据，不改变视觉、跟踪或导航参数。
POSE_LOG_ENABLE = True
POSE_LOG_INTERVAL_MS = 200
POSE_LOG_FLUSH_MS = 1000

POSE_LOG_FIELDS = (
    "frame_id", "capture_time_us", "log_time_us", "capture_ms",
    "frame_pre_log_ms", "fps", "solve_ms", "attitude_valid",
    "q_w", "q_x", "q_y", "q_z", "roll_deg", "pitch_deg", "yaw_deg",
    "pose_valid", "pose_reason", "reprojection_px",
    "p_C_x_m", "p_C_y_m", "p_C_z_m",
) + tuple("r_CT_%d%d" % (i, j) for i in range(3) for j in range(3)) + (
    "relative_valid", "relative_reason", "normal_tilt_deg",
    "p_local_raw_x_m", "p_local_raw_y_m", "p_local_raw_z_m",
    "p_local_x_m", "p_local_y_m", "p_local_z_m",
    "velocity_N_mps", "velocity_E_mps", "velocity_D_mps",
    "track", "nav", "flight", "track_age_ms", "observation_age_ms",
    "configured", "config_errors", "color",
) + tuple("%s%d_px" % (axis, i) for i in range(1, 5) for axis in ("u", "v"))


def attitude_rpy_deg(q):
    """该帧实际使用的B->NED四元数，转换为ZYX欧拉角；缺失时不补旧值。"""
    if q is None:
        return None
    r = quaternion_rotation(q)
    degrees = 180. / math.pi
    return (math.atan2(r[2][1], r[2][2]) * degrees,
            math.asin(max(-1., min(1., -r[2][0]))) * degrees,
            math.atan2(r[1][0], r[0][0]) * degrees)


def _csv_cell(value):
    if value is None:
        return ""
    if isinstance(value, bool):
        return "1" if value else "0"
    if isinstance(value, float):
        return "%.7g" % value if math.isfinite(value) else ""
    value = str(value)
    if any(c in value for c in (",", '"', "\r", "\n")):
        return '"' + value.replace('"', '""') + '"'
    return value


def _values(value, count):
    return (None,) * count if value is None else tuple(value)


class PoseCSVLogger:
    """自动编号CSV；默认5Hz采样、每秒刷新，写入失败只停止记录。"""
    def __init__(self, directory="/", enabled=POSE_LOG_ENABLE, start_us=0):
        self.file = None
        self.path = None
        self.last_sample_us = None
        self.last_flush_us = start_us
        self.last_error = None
        if not enabled:
            return
        try:
            for index in range(1, 10000):
                path = directory.rstrip("/") + "/pnp_pose_%04d.csv" % index
                try:
                    os.stat(path)
                except OSError as error:
                    if not error.args or error.args[0] != 2:
                        raise
                    self.path = path
                    self.file = open(path, "w")
                    break
            else:
                raise OSError("LOG_INDEX_EXHAUSTED")
            self._write(",".join(POSE_LOG_FIELDS) + "\n")
            self.file.flush()
            print("LOG=%s" % self.path)
        except Exception as error:
            self.fail(error)

    def _write(self, line):
        written = self.file.write(line)
        if written is not None and written != len(line):
            raise OSError("CSV_SHORT_WRITE")

    def due(self, now_us):
        return (self.file is not None and
                (self.last_sample_us is None or
                 now_us-self.last_sample_us >= POSE_LOG_INTERVAL_MS*1000))

    def write(self, row, now_us):
        if not self.due(now_us):
            return False
        try:
            if len(row) != len(POSE_LOG_FIELDS):
                raise ValueError("CSV_FIELD_COUNT")
            self._write(",".join(_csv_cell(v) for v in row) + "\n")
            if now_us-self.last_flush_us >= POSE_LOG_FLUSH_MS*1000:
                self.file.flush()
                self.last_flush_us = now_us
            self.last_sample_us = now_us
            return True
        except Exception as error:
            self.fail(error)
            return False

    def fail(self, error):
        self.last_error = str(error)
        if self.file is not None:
            try:
                self.file.close()
            except Exception:
                pass
            self.file = None
        print("LOG_ERROR=%s; recording stopped" % self.last_error)

    def close(self):
        if self.file is not None:
            try:
                self.file.flush()
                self.file.close()
                self.file = None
            except Exception as error:
                self.fail(error)


def pose_log_row(frame_id, stamp, now, capture_us, pre_log_us, fps,
                 solve_us, q, pose, relative, track, velocity, flight,
                 nav_state, points, color):
    pose_valid = bool(pose and pose.get("valid"))
    p = pose.get("position") if pose_valid else None
    rotation = pose.get("rotation") if pose_valid else None
    is_configured = configured()
    normal_tilt = None
    if not pose_valid:
        reason = "PNP_INVALID" if pose is not None else "NO_POSE"
    elif not is_configured:
        reason = "CONFIG_UNREADY"
    elif q is None:
        reason = "ATTITUDE_UNAVAILABLE"
    else:
        r_nb = quaternion_rotation(q)
        normal = mv(r_nb, mv(C.R_BC, tuple(row[2] for row in rotation)))
        normal_tilt = math.acos(max(-1., min(1., normal[2]))) * 180./math.pi
        if relative is not None:
            reason = "OK"
        elif normal[2] < math.cos(C.MAX_NORMAL_TILT_DEG*math.pi/180.):
            reason = "PLANE_NORMAL_TILT"
        else:
            reason = "RELATIVE_DEPTH_REJECTED"
    pose_reason = pose.get("reason", "UNKNOWN") if pose is not None else (
        "MODEL_MISSING" if C.TARGET_POINTS_T_M is None else "NO_PAIR")
    measurement_ms = track.get("measurement_ms")
    return (
        frame_id, stamp, now, capture_us*.001, pre_log_us*.001, fps,
        None if solve_us is None else solve_us*.001, q is not None,
    ) + _values(q, 4) + _values(attitude_rpy_deg(q), 3) + (
        pose_valid, pose_reason,
        pose.get("reprojection_px") if pose is not None else None,
    ) + _values(p, 3) + (
        (None,) * 9 if rotation is None else tuple(v for row in rotation for v in row)
    ) + (
        relative is not None, reason, normal_tilt,
    ) + _values(relative, 3) + _values(track.get("position"), 3) + _values(velocity, 3) + (
        track["state"], nav_state, flight["reason"], track.get("age_ms"),
        None if measurement_ms is None else now//1000-measurement_ms,
        is_configured, "|".join(C.readiness_errors()), color,
    ) + ((None,) * 8 if points is None else tuple(v for point in points for v in point))


def terminal_report(fps, position, q, solve_mean_ms, solve_peak_ms):
    p = "None" if position is None else "(%.3f,%.3f,%.3f)" % tuple(position)
    angles = attitude_rpy_deg(q)
    rpy = "None" if angles is None else "(%.1f,%.1f,%.1f)" % angles
    mean = "None" if solve_mean_ms is None else "%.2f" % solve_mean_ms
    peak = "None" if solve_peak_ms is None else "%.2f" % solve_peak_ms
    return "FPS=%.1f p_local=%s m RPY=%s deg solve_ms(mean/peak)=%s/%s" % (fps, p, rpy, mean, peak)


def build_solver():
    if C.TARGET_POINTS_T_M is None:
        return None
    return PlanarPoseSolver(C.TARGET_POINTS_T_M)

def configured():
    return (not C.readiness_errors() and rotation_valid(C.R_BC)
            and C.T_BC_M is not None and len(C.T_BC_M) == 3 and finite(C.T_BC_M))

def run():
    import sensor
    import armor_detect as detector
    clock = Clock()
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA)
    sensor.skip_frames(time=1500)
    sensor.set_auto_gain(False)
    sensor.set_auto_whitebal(False)
    sensor.set_auto_exposure(False,exposure_us=C.EXPOSURE_US)
    # 不自动重新对焦；运行焦点须与所用标定一致。
    solver = build_solver()
    tracker,servo = TargetTracker(),VisualServo()
    link = FlightLink(clock=clock) if C.LINK_ENABLE else None
    last_report,last_control = clock.now_us(),-10000000
    logger = PoseCSVLogger(start_us=last_report)
    frames,solves,solve_sum,solve_peak,frame_id = 0,0,0,0,0
    previous,previous_color = None,None
    generation = link.generation if link else 0
    try:
        while True:
            start = clock.now_us()
            if link:
                link.poll(service_sync=True)
                if generation != link.generation:
                    tracker,servo = TargetTracker(),VisualServo()
                    previous = None
                    generation = link.generation
            capture_start = clock.now_us()
            img = sensor.snapshot()
            capture_end = clock.now_us()
            stamp = (capture_start+capture_end)//2-C.CAMERA_CAPTURE_DELAY_US
            red,blue = detector.detect_lights(img)
            debug = img if C.DEBUG_MODE >= 2 else None
            rp = detector.find_best_pair(red,debug) if C.TARGET_COLOR != "BLUE" else None
            bp = detector.find_best_pair(blue,debug) if C.TARGET_COLOR != "RED" else None
            pair,color = detector.select_best_pair(rp,bp)
            if link:
                link.poll()
            q = link.attitude_at(stamp) if link else None
            velocity = link.velocity_at(stamp) if link else None
            detection,pose,relative,points,spent = None,None,None,None,None
            if pair is not None:
                cx,cy,_,_,w,h = detector.get_pair_geometry(pair)
                center = detector.pair_center(pair)
                detection = {"center":center,"color":color,"size":float(max(w,h))}
                if solver is not None:
                    before = clock.now_us()
                    hint = previous if previous_color == color else None
                    points = detector.get_pair_points(pair)
                    pose = solver.solve(points,hint)
                    spent = clock.now_us()-before
                    solves += 1
                    solve_sum += spent
                    solve_peak = max(solve_peak,spent)
                    if pose["valid"]:
                        detection["center"] = pose["center"]
                        previous,previous_color = pose,color
                        if configured() and q is not None:
                            relative = relative_measurement(pose,q,C.R_BC,C.T_BC_M)
            track = tracker.update(detection,stamp//1000,relative,velocity)
            now = clock.now_us()
            flight = link.state(now) if link else {"ready":False,"reason":"LINK_DISABLED"}
            if now-last_control >= C.CONTROL_INTERVAL_MS*1000:
                control_track = dict(track)
                delay = max(0.,(now-stamp)*1e-6)
                if track["position"] is not None:
                    current_v = flight.get("velocity")
                    if current_v is not None:
                        control_track["position"] = tuple(track["position"][i]-current_v[i]*delay for i in range(3))
                    if (track["age_ms"] is None or track["age_ms"]+delay*1000 > C.MAX_PREDICTION_MS):
                        control_track["position"],control_track["state"] = None,"LOST"
                output = servo.update(control_track,flight,now//1000,configured())
                if link:
                    if output is None:
                        link.cancel_setpoint()
                    else:
                        link.send_velocity(output["velocity"],output["yaw"],now)
                last_control = now
            if link:
                link.poll()
            if C.DEBUG_MODE:
                center = None
                cross_color = (255,160,0)  # 中心关联或预测，不代表新鲜三维测量
                if pose is not None and pose["valid"] and track["matched"]:
                    center,cross_color = pose["center"],(0,255,0)
                elif solver is not None and track["position"] is not None and q is not None and configured():
                    pc = local_to_camera(track["position"],q,C.R_BC,C.T_BC_M)
                    center = solver.project(pc)
                elif detection is not None:
                    center = detection["center"]
                if center is not None and 0 <= center[0] < img.width() and 0 <= center[1] < img.height():
                    img.draw_cross(int(round(center[0])),int(round(center[1])),size=8,color=cross_color,thickness=2)
            pre_log_end = clock.now_us()
            frame_id += 1
            if logger.due(pre_log_end):
                try:
                    seconds = max(1e-6, (pre_log_end-last_report)*1e-6)
                    row = pose_log_row(
                        frame_id, stamp, pre_log_end, capture_end-capture_start,
                        pre_log_end-start, (frames+1)/seconds, spent, q, pose,
                        relative, track, velocity, flight, servo.state, points, color)
                    logger.write(row, pre_log_end)
                except Exception as error:
                    logger.fail(error)
            end = clock.now_us()
            frames += 1
            if end-last_report >= C.DEBUG_PRINT_INTERVAL_MS*1000:
                seconds = (end-last_report)*1e-6
                print(terminal_report(frames/seconds, track["position"], q,
                                      solve_sum/solves*.001 if solves else None,
                                      solve_peak*.001 if solves else None))
                frames,solves,solve_sum,solve_peak = 0,0,0,0
                last_report = end
                detector.reset_stats()
    finally:
        try:
            if link:
                link.cancel_setpoint()
        finally:
            logger.close()

if __name__ == "__main__":
    run()
