"""OpenMV室内视觉任务：人工切Offboard后开始识别与导航。"""
import math
import os
import json
import time
import config as C
from pose_solver import (PlanarPoseSolver, finite, rotation_valid, local_to_camera,
                         relative_measurement, quaternion_rotation,
                         camera_down_normal)
from target_tracker import TargetTracker
from flight_link import Clock, FlightLink
from visual_servo import VisualServo
from comm_log import CommCSVLogger

# 只记录诊断数据，不改变视觉、跟踪或导航参数。
POSE_LOG_ENABLE = True
POSE_LOG_INTERVAL_MS = 200
POSE_LOG_FLUSH_MS = 1000

POSE_LOG_FIELDS = (
    "frame_id", "capture_time_us", "control_time_us", "control_capture_time_us", "link_generation",
    "pose_valid", "pose_reason", "reprojection_px", "relative_valid", "relative_reason",
    "p_local_raw_x_m", "p_local_raw_y_m", "p_local_raw_z_m",
    "track", "control_track", "nav", "flight", "armed", "offboard", "vision_enabled",
    "attitude_status", "attitude_before_dt_ms", "attitude_after_dt_ms", "velocity_age_ms",
    "observation_age_ms", "control_p_local_x_m", "control_p_local_y_m", "control_p_local_z_m",
    "command_vn_mps", "command_ve_mps", "command_vd_mps", "setpoint_submitted", "tx_status", "tx_image_age_ms",
)


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


def log_state_key(track,nav,flight,alignment):
    """Normal interpolation/extrapolation changes stay in 5 Hz samples."""
    attitude = alignment.get('attitude_status')
    if attitude in ('EXACT','INTERPOLATED','EXTRAPOLATED'):
        attitude = 'OK'
    return (track,nav,flight.get('reason'),flight.get('armed'),flight.get('offboard'),attitude)


class PoseCSVLogger:
    """自动编号CSV；默认5Hz采样、每秒刷新，写入失败只停止记录。"""
    def __init__(self, directory="/", enabled=POSE_LOG_ENABLE, start_us=0):
        self.file = None
        self.path = None
        self.last_sample_us = None
        self.last_flush_us = start_us
        self.last_error = None
        self.last_state = None
        if not enabled:
            return
        try:
            for index in range(1, 10000):
                path = directory.rstrip("/") + "/pnp_pose_%04d.csv" % index
                occupied = False
                for candidate in (path,path[:-4]+'.json',path[:-4]+'_sync.csv'):
                    try:
                        os.stat(candidate)
                        occupied = True
                        break
                    except OSError as error:
                        if not error.args or error.args[0] != 2:
                            raise
                if not occupied:
                    self.path = path
                    self.file = open(path, "w")
                    break
            else:
                raise OSError("LOG_INDEX_EXHAUSTED")
            self._write(",".join(POSE_LOG_FIELDS) + "\n")
            self.file.flush()
            with open(self.path[:-4]+'.json','w') as metadata:
                metadata.write(json.dumps(run_metadata()))
            print("LOG=%s" % self.path)
        except Exception as error:
            self.fail(error)

    def _write(self, line):
        written = self.file.write(line)
        if written is not None and written != len(line):
            raise OSError("CSV_SHORT_WRITE")

    def due(self, now_us, state=None):
        return (self.file is not None and
                (state is not None and state != self.last_state or
                 self.last_sample_us is None or
                 not 0 <= now_us-self.last_sample_us < POSE_LOG_INTERVAL_MS*1000))

    def write(self, row, now_us, state=None):
        if not self.due(now_us,state):
            return False
        try:
            if len(row) != len(POSE_LOG_FIELDS):
                raise ValueError("CSV_FIELD_COUNT")
            self._write(",".join(_csv_cell(v) for v in row) + "\n")
            if not self.flush_due(now_us):
                return False
            self.last_sample_us = now_us
            self.last_state = state
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

    def flush_due(self, now_us):
        """Flush even when no camera frame arrives; caller keeps I/O low priority."""
        if self.file is not None and not 0 <= now_us-self.last_flush_us < POSE_LOG_FLUSH_MS*1000:
            try:
                self.file.flush()
                self.last_flush_us = now_us
            except Exception as error:
                self.fail(error)
        return self.file is not None

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
                 nav_state, points, color,
                 vision_enabled=True, control_time_us=None, control_track=None,
                 output=None, setpoint_submitted=False):
    """Compact frame diagnostics plus explicitly identified control input.

    Raw position/truth refer to ``stamp``. The compensated control position
    refers to ``control_time_us`` and its own ``control_capture_time_us``.
    """
    valid = bool(pose and pose.get('valid'))
    if not valid: reason = 'PNP_INVALID' if pose else 'NO_POSE'
    elif not configured(): reason = 'CONFIG_UNREADY'
    elif q is None: reason = 'ATTITUDE_UNAVAILABLE'
    elif relative is None: reason = 'RELATIVE_REJECTED'
    else: reason = 'OK'
    control = control_track or {}
    alignment = track.get('_alignment',{})
    measurement = control.get('measurement_ms')
    age = None if measurement is None or control_time_us is None else control_time_us//1000-measurement
    source = control.get('_capture_us')
    tx_age = flight.get('tx_age_us') if flight.get('tx_control_us')==control_time_us else None
    return (frame_id,stamp,control_time_us,source,track.get('_link_generation'),
            valid,pose.get('reason','UNKNOWN') if pose else 'NO_PAIR',
            pose.get('reprojection_px') if pose else None,
            relative is not None,reason) + _values(relative,3) + (
            track.get('state'),control.get('state'),nav_state,flight.get('reason'),
            flight.get('armed'),flight.get('offboard'),vision_enabled,
            alignment.get('attitude_status'),alignment.get('attitude_before_dt_ms'),
            alignment.get('attitude_after_dt_ms'),alignment.get('velocity_age_ms'),age,
            ) + _values(control.get('position'),3) + _values(output.get('velocity') if output else None,3) + (
            setpoint_submitted,flight.get('tx_status'),None if tx_age is None else tx_age*.001)


def run_metadata():
    """Static facts are written once, never repeated in each frame/event."""
    controls = ('TARGET_HEIGHT_M','ARRIVAL_XY_M','ARRIVAL_Z_M','ARRIVAL_DWELL_MS',
                'KP_XY','KP_Z','MAX_SPEED_XY','MAX_SPEED_Z','MAX_ACCEL_XY','MAX_ACCEL_Z',
                'CONTROL_INTERVAL_MS','LOST_BRAKE_MS')
    errors = C.readiness_errors()
    if not rotation_valid(C.R_BC):
        errors.append('CAMERA_ROTATION_INVALID')
    if C.T_BC_M is None or len(C.T_BC_M) != 3 or not finite(C.T_BC_M):
        errors.append('CAMERA_TRANSLATION_INVALID')
    metadata = {'schema_version':3,'implementation':'shared_pnp_cooperative_v3',
            'clock':'local unfolded microseconds; remote=local+offset_us',
            'log_interval_ms':POSE_LOG_INTERVAL_MS,
            'control_config':{key:getattr(C,key) for key in controls},
            'configured':configured(),'config_errors':errors,
            'scheduling':{'control_interval_ms':C.CONTROL_INTERVAL_MS,
                          'communication_service_ms':C.COMM_SERVICE_INTERVAL_MS,
                          'diagnostic_delay_ms':C.DIAGNOSTIC_DELAY_MS,
                          'policy':'Actual-time deadlines; latest complete image; no catch-up burst'},
            'timing_limits':{key:getattr(C,key) for key in (
                'SYNC_INTERVAL_MS','SYNC_RECOVERY_INTERVAL_MS','SYNC_MAX_RTT_MS',
                'SYNC_MAX_JITTER_US','SYNC_MIN_SAMPLES','SYNC_TIMEOUT_MS',
                'SYNC_WINDOW_MS','SYNC_WINDOW_SIZE','SYNC_RESET_US',
                'ATTITUDE_EXTRAPOLATE_MS','TELEMETRY_TIMEOUT_MS','MAX_TX_AGE_MS')},
            'camera':{'intrinsics':C.INTRINSICS,'distortion':C.DIST_COEFFS,
                      'R_BC':C.R_BC,'T_BC_M':C.T_BC_M,
                      'capture_delay_us':C.CAMERA_CAPTURE_DELAY_US},
            'transmit_note':'WRITTEN is a completed local write, not a PX4 execution acknowledgement'}
    try:
        import hashlib
        import binascii
        paths = {'config':getattr(C,'__file__','config.py'),
                 'main':globals().get('__file__','main.py'),
                 'flight_link':getattr(__import__(FlightLink.__module__),'__file__','flight_link.py')}
        hashes = {}
        for name,path in paths.items():
            checksum = hashlib.sha256()
            with open(path,'rb') as source:
                while True:
                    data = source.read(512)
                    if not data: break
                    checksum.update(data)
            hashes[name] = binascii.hexlify(checksum.digest()).decode()
        metadata['source_sha256'] = hashes
    except (ImportError,OSError,AttributeError) as error:
        metadata['source_hash_error'] = str(error)
    return metadata

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

def vision_enabled(flight):
    """同一模式门控供OpenMV和仿真使用；观察模式始终识别。"""
    return (not C.NAV_ENABLE or
            bool(flight.get("ready") and flight.get("armed") and flight.get("offboard")))


def process_image(img, stamp, solver, tracker, detector, clock, link,
                  previous=None, previous_color=None, vision_active=True, service=None):
    """共享识别、PnP、坐标变换和跟踪；图像与计时接口由平台提供。"""
    pair,color = None,None
    if vision_active:
        red,blue = detector.detect_lights(img,service=service)
        debug = img if C.DEBUG_MODE >= 2 else None
        rp = detector.find_best_pair(red,debug,service=service) if C.TARGET_COLOR != "BLUE" else None
        bp = detector.find_best_pair(blue,debug,service=service) if C.TARGET_COLOR != "RED" else None
        pair,color = detector.select_best_pair(rp,bp)
    if link:
        link.poll()
    if service:
        service('detect')
    if link:
        q,velocity,alignment,link_generation = link.observation_at(stamp)
    else:
        q,velocity,alignment,link_generation = None,None,{'attitude_status':'NO_LINK'},None
    previous_generation = getattr(tracker,'flight_generation',link_generation)
    if previous_generation != link_generation:
        tracker.__init__()
        previous,previous_color = None,None
    tracker.flight_generation = link_generation
    detection,pose,relative,points,spent = None,None,None,None,None
    if pair is not None:
        cx,cy,_,_,w,h = detector.get_pair_geometry(pair)
        detection = {"center":detector.pair_center(pair),"color":color,"size":float(max(w,h))}
        if solver is not None:
            before = clock.now_us()
            hint = previous if previous_color == color else None
            points = detector.get_pair_points(pair)
            normal = camera_down_normal(q,C.R_BC) if C.HORIZONTAL_TARGET and q is not None else None
            pose = solver.solve(points,hint,normal_camera=normal,service=service)
            spent = clock.now_us()-before
            if pose["valid"]:
                detection["center"] = pose["center"]
                previous,previous_color = pose,color
                if configured() and q is not None:
                    relative = relative_measurement(pose,q,C.R_BC,C.T_BC_M)
    if service:
        service('pnp')
    track = tracker.update(detection,stamp//1000,relative,velocity)
    track['_alignment'],track['_link_generation'] = alignment,link_generation
    if not vision_active:
        track["state"] = "IDLE"
    return ({"detection":detection,"pose":pose,"relative":relative,"points":points,
             "track":track,"color":color,"solve_us":spent,"stamp_us":stamp,
             "q":q,"velocity":velocity,"vision_enabled":vision_active,
             "alignment":alignment,"link_generation":link_generation},previous,previous_color)


def update_control(track, flight, stamp, now, servo, link):
    """共享延迟补偿、控制更新和发送/释放逻辑，不增加独立接管状态。"""
    control_track = dict(track)
    control_track['_capture_us'] = stamp
    invalid_epoch = link is not None and track.get('_link_generation') != flight.get('generation')
    if stamp > now or now-stamp > C.MAX_PREDICTION_MS*1000 or invalid_epoch:
        control_track.update(position=None,state='LOST')
    if control_track.get('position') is not None:
        displacement = link.displacement(stamp,now) if link else None
        if displacement is None:
            control_track.update(position=None,state='LOST')
        else:
            control_track['position'] = tuple(track['position'][i]-displacement[i] for i in range(3))
        measurement = track.get('measurement_ms')
        age = None if measurement is None else now//1000-measurement
        control_track['age_ms'] = age
        if age is None or not 0 <= age <= C.MAX_PREDICTION_MS:
            control_track["position"],control_track["state"] = None,"LOST"
    previous_nav = servo.state
    output = servo.update(control_track,flight,now//1000,configured())
    submitted = False
    if link:
        has_position = control_track.get('position') is not None
        measurement = control_track.get('measurement_ms')
        link.set_control_source(stamp if has_position else None,
                                measurement*1000 if has_position and measurement is not None else None)
        if output is None or output.get("velocity") is None:
            link.cancel_setpoint(servo.state)
            if (servo.state in ("LOST_RELEASE","LOOP_STALE") and servo.state != previous_nav
                    and flight.get("ready") and flight.get("armed")
                    and flight.get("offboard") and flight.get("local_position_ready")):
                link.request_mode(3,0,reason=servo.state)
        else:
            submitted = link.send_velocity(output["velocity"],output["yaw"],now)
    return control_track,output,submitted


def service_runtime(runtime, clock, servo, link, stage='idle', force=False):
    """Small cooperative scheduler, not a handover state machine.

    ``revision`` invalidates in-flight image work after a gate/clock change.
    Only complete images are published in runtime['result']; this function never
    changes tracker history, image stamps or real measurement times.
    """
    now = clock.now_us()
    if link:
        link.service_stage = stage
    last_service = runtime.get('service_time_us')
    if (not force and last_service is not None and
            0 <= now-last_service < C.COMM_SERVICE_INTERVAL_MS*1000):
        return
    if link:
        link.poll()
    now = clock.now_us()
    runtime['service_time_us'] = now
    flight = link.state(now) if link else {'ready':False,'reason':'LINK_DISABLED'}
    active,generation = vision_enabled(flight),flight.get('generation')
    if active != runtime.get('vision_active') or generation != runtime.get('generation'):
        runtime['result'] = None
        runtime['revision'] = runtime.get('revision',0)+1
    runtime['vision_active'],runtime['generation'],runtime['flight'] = active,generation,flight
    gate = (generation,flight.get('reason'),flight.get('armed'),flight.get('offboard'),
            flight.get('main_mode'),flight.get('sub_mode'))
    last_control = runtime.get('control_time_us')
    if (last_control is None or not 0 <= now-last_control < C.CONTROL_INTERVAL_MS*1000
            or gate != runtime.get('gate')):
        if link and last_control is not None and now-last_control >= C.DIAGNOSTIC_DELAY_MS*1000:
            link._event('CONTROL_DELAY',now,reason=stage,delay_us=now-last_control)
        result = runtime.get('result')
        track = result['track'] if result is not None else {
            'position':None,'state':'LOST' if active else 'IDLE','generation':0,
            '_link_generation':generation}
        stamp = result['stamp_us'] if result is not None else now
        control,output,submitted = update_control(track,flight,stamp,now,servo,link)
        runtime.update(control_track=control,output=output,submitted=submitted,
                       control_time_us=now,gate=gate)
        if link:
            link.poll()


def record_log_delay(link, clock, started, label):
    if link:
        elapsed = clock.now_us()-started
        if elapsed >= C.DIAGNOSTIC_DELAY_MS*1000:
            link._event('LOG_DELAY',reason=label,delay_us=elapsed)


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
    last_report = clock.now_us()
    logger = PoseCSVLogger(start_us=last_report)
    comm_logger = (CommCSVLogger(link,logger.path[:-4]+'_sync.csv',clock=clock)
                   if link and logger.path else None)
    comm_error = None
    frames,solves,solve_sum,solve_peak,frame_id = 0,0,0,0,0
    previous,previous_color = None,None
    runtime = {'revision':0}
    tracker_revision = -1
    frame_available = getattr(sensor,'get_frame_available',None)
    last_maintenance = None

    def service(stage='idle', force=False):
        service_runtime(runtime,clock,servo,link,stage,force)

    def maintenance():
        nonlocal last_maintenance,comm_error
        now = clock.now_us()
        if last_maintenance is not None and 0 <= now-last_maintenance < C.CONTROL_INTERVAL_MS*1000:
            return
        service('before_log',True)
        started = clock.now_us()
        logger.flush_due(started)
        if comm_logger:
            comm_logger.record()
            if comm_logger.error and comm_logger.error != comm_error:
                comm_error = comm_logger.error
                print('COMM_LOG_ERROR=%s' % comm_error)
        record_log_delay(link,clock,started,'MAINTENANCE')
        service('log',True)
        last_maintenance = clock.now_us()

    try:
        while True:
            start = clock.now_us()
            service('wait_frame')
            if frame_available is not None and not frame_available():
                maintenance()
                if hasattr(time,'sleep_ms'): time.sleep_ms(1)
                else: time.sleep(.001)
                continue
            service('wait_frame',True)
            if tracker_revision != runtime['revision']:
                tracker = TargetTracker()
                previous,previous_color = None,None
                tracker_revision = runtime['revision']
            frame_revision = runtime['revision']
            vision_active = runtime['vision_active']
            capture_start = clock.now_us()
            img = sensor.snapshot()
            capture_end = clock.now_us()
            stamp = (capture_start+capture_end)//2-C.CAMERA_CAPTURE_DELAY_US
            service('capture',True)
            if frame_revision != runtime['revision']:
                maintenance()
                continue
            result,previous,previous_color = process_image(
                img,stamp,solver,tracker,detector,clock,link,
                previous,previous_color,vision_active,service)
            if (frame_revision == runtime['revision'] and
                    result['link_generation'] == runtime['generation']):
                runtime['result'] = result
            service('image',True)
            if runtime.get('result') is not result:
                maintenance()
                continue
            q,velocity = result["q"],result["velocity"]
            detection,pose,relative,points = (result[key] for key in ("detection","pose","relative","points"))
            track,color,spent = result["track"],result["color"],result["solve_us"]
            if spent is not None:
                solves += 1
                solve_sum += spent
                solve_peak = max(solve_peak,spent)
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
            service('display',True)
            pre_log_end = clock.now_us()
            flight = runtime['flight']
            nav_state = servo.state
            frame_id += 1
            log_state = log_state_key(track['state'],nav_state,flight,track.get('_alignment',{}))
            if logger.due(pre_log_end,log_state):
                service('before_pose_log',True)
                flight,nav_state = runtime['flight'],servo.state
                log_state = log_state_key(track['state'],nav_state,flight,track.get('_alignment',{}))
                log_started = clock.now_us()
                try:
                    seconds = max(1e-6, (pre_log_end-last_report)*1e-6)
                    row = pose_log_row(
                        frame_id, stamp, pre_log_end, capture_end-capture_start,
                        pre_log_end-start, (frames+1)/seconds, spent, q, pose,
                        relative, track, velocity, flight, nav_state, points, color,
                        vision_active, runtime.get('control_time_us'),runtime.get('control_track'),
                        runtime.get('output'),runtime.get('submitted',False))
                    logger.write(row, pre_log_end,log_state)
                except Exception as error:
                    logger.fail(error)
                record_log_delay(link,clock,log_started,'POSE')
                service('pose_log',True)
            maintenance()
            end = clock.now_us()
            frames += 1
            if end-last_report >= C.DEBUG_PRINT_INTERVAL_MS*1000:
                seconds = (end-last_report)*1e-6
                print(terminal_report(frames/seconds, track["position"], q,
                                      solve_sum/solves*.001 if solves else None,
                                      solve_peak*.001 if solves else None)
                      + " nav=%s track=%s vision=%s flight=%s" %
                      (nav_state,track["state"],vision_active,flight["reason"]))
                frames,solves,solve_sum,solve_peak = 0,0,0,0
                last_report = end
                detector.reset_stats()
    finally:
        try:
            if link:
                link.cancel_setpoint('PROGRAM_EXIT')
        finally:
            if comm_logger:
                comm_logger.close()
            logger.close()

if __name__ == "__main__":
    run()
