"""将本目录运行模块放到OpenMV根目录；config默认仅观察。"""
import time
import config as C
from pose_solver import (PlanarPoseSolver, finite, rotation_valid, local_to_camera,
                         relative_measurement)
from target_tracker import TargetTracker
from visual_servo import VisualServo
from flight_link import Clock, FlightLink

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
    print("PnP observe; readiness:",C.readiness_errors())
    print("MODEL REQUIRED" if solver is None else "model loaded")
    last_report,last_control = clock.now_us(),-10000000
    frames,solves,solve_sum,solve_peak,frame_peak = 0,0,0,0,0
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
            detection,pose,relative = None,None,None
            if pair is not None:
                cx,cy,_,_,w,h = detector.get_pair_geometry(pair)
                center = detector.pair_center(pair)
                detection = {"center":center,"color":color,"size":float(max(w,h))}
                if solver is not None:
                    before = clock.now_us()
                    hint = previous if previous_color == color else None
                    pose = solver.solve(detector.get_pair_points(pair),hint)
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
            end = clock.now_us()
            frames += 1
            frame_peak = max(frame_peak,end-start)
            if end-last_report >= C.DEBUG_PRINT_INTERVAL_MS*1000:
                seconds = (end-last_report)*1e-6
                p = pose["position"] if pose and pose["valid"] else None
                print("FPS=%.1f solve_mean_ms=%.2f solve_peak_ms=%.2f frame_peak_ms=%.2f track=%s nav=%s flight=%s p_C=%s p_local=%s age_ms=%s" %
                      (frames/seconds,solve_sum/max(1,solves)*.001,solve_peak*.001,frame_peak*.001,
                       track["state"],servo.state,flight["reason"],p,track["position"],None if track["measurement_ms"] is None else end//1000-track["measurement_ms"]))
                frames,solves,solve_sum,solve_peak,frame_peak = 0,0,0,0,0
                last_report = end
                detector.reset_stats()
    finally:
        if link:
            link.cancel_setpoint()

if __name__ == "__main__":
    run()
