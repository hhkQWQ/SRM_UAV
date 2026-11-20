#!/usr/bin/env python3
"""ROS 2 Gazebo input and diagnostics for the original OpenMV PnP algorithm."""
import argparse
from collections import deque
import csv
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import signal
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, PoseStamped
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from algorithm_core import Algorithm, quaternion_from_rotation
from camera_mount import read_camera_mount
from gazebo_truth import GroundTruth
from udp_uart import UdpUart
from link_service import ThreadSafeLink


def stamp_us(stamp):
    return int(stamp.sec) * 1_000_000 + int(stamp.nanosec) // 1000


def finite_json(value):
    if isinstance(value, dict):
        return {k: finite_json(v) for k, v in value.items()}
    if isinstance(value, (tuple, list, np.ndarray)):
        return [finite_json(v) for v in value]
    if isinstance(value, (float, np.floating)):
        return float(value) if math.isfinite(value) else None
    if isinstance(value, np.integer):
        return int(value)
    return value


# All operational fields come from PnP. Only aligned ground truth is appended.
TRUTH_FIELDS = (
    'true_p_local_x_m','true_p_local_y_m','true_p_local_z_m',
    'truth_dt_us','truth_source',
)


class PnpNode(Node):
    def __init__(self, options):
        super().__init__('pnp_sim_algorithm')
        self.options = options
        self.algorithm = Algorithm(options.algorithm_dir, options.enable_control)
        self.runtime = self.algorithm.modules.runtime
        options.enable_control = bool(self.algorithm.config.NAV_ENABLE)
        if options.enable_control and options.no_mavlink:
            raise ValueError('Offboard control requires real MAVLink telemetry; use --observe for diagnostics')
        camera_position, camera_rotation = read_camera_mount(
            options.px4_dir, self.algorithm.config.R_BC, self.algorithm.config.T_BC_M)
        self.truth = GroundTruth(options.world, options.model,
                                 camera_position=camera_position,
                                 camera_rotation=camera_rotation)
        self.bridge = CvBridge()
        self.pending = deque(maxlen=1)
        self.last_result = None
        self.last_image_us = None
        self.last_image_wall = None
        self.last_frame_us = None
        self.last_control_us = -1_000_000
        self.last_report_wall = 0.
        self.last_warning_wall = 0.
        self.last_csv_us = None
        self.last_log_state = None
        self.last_flush_wall = 0.
        self.frame_count = 0
        self.drop_count = 0
        self.generation = self.truth.generation
        self.setpoint_submitted = False
        self.last_command = None
        self.vision_enabled = False
        self.control_track = None
        self.control_time_us = None
        self.transport = self.link = None
        if not options.no_mavlink:
            try:
                self.transport = UdpUart(options.mavlink_port)
                self.link = ThreadSafeLink(self.algorithm.modules.link.FlightLink(
                    uart=self.transport,clock=self.truth))
            except OSError as error:
                if options.enable_control:
                    raise
                self.get_logger().warning(f'MAVLink unavailable: {error}; continuing image observation')
        elif options.enable_control:
            raise ValueError('--enable-control requires MAVLink')
        self.link_generation = self.link.generation if self.link else 0
        self.image_publisher = self.create_publisher(Image, '/pnp/debug_image', qos_profile_sensor_data)
        self.pose_publisher = self.create_publisher(PoseStamped, '/pnp/target_camera', 10)
        self.relative_publisher = self.create_publisher(PointStamped, '/pnp/relative_ned', 10)
        self.status_publisher = self.create_publisher(String, '/pnp/status', 10)
        self.info_subscription = self.create_subscription(
            CameraInfo, options.camera_info_topic, self.on_info, qos_profile_sensor_data)
        self.image_subscription = self.create_subscription(
            Image, options.image_topic, self.on_image, qos_profile_sensor_data)
        log_dir = Path(options.log_dir).expanduser()
        log_dir.mkdir(parents=True, exist_ok=True)
        name = datetime.now().strftime('pnp_sim_%Y%m%d_%H%M%S_%f')
        self.csv_path = log_dir / f'{name}.csv'
        self.csv_file = self.csv_path.open('x', newline='')
        self.csv_fields = self.runtime.POSE_LOG_FIELDS + TRUTH_FIELDS
        self.csv = csv.DictWriter(self.csv_file, fieldnames=self.csv_fields)
        self.csv.writeheader()
        settings = self.runtime.run_metadata()
        settings.update({
            'clock': 'Gazebo simulation microseconds; remote=local+offset_us',
            'scheduling_clock': 'host monotonic; 2 ms target communication polling interval',
            'mode': 'manual_offboard' if options.enable_control else 'observe',
            'algorithm_dir': str(Path(options.algorithm_dir).resolve()),
            'simulation_overrides': {
                'R_BC': self.algorithm.config.R_BC,
                'T_BC_M': self.algorithm.config.T_BC_M,
            },
            'camera_mount': {
                'source': 'original config.py extrinsics; verified against actual Gazebo model',
                'position_in_model_m': camera_position,
                'sensor_rotation_in_model': camera_rotation,
            },
            'detector_config': {
                key: getattr(self.algorithm.config,key) for key in
                ('LIGHT_EXTRACT_MODE','TARGET_COLOR','LIGHT_L_MIN','RED_THRESHOLD','BLUE_THRESHOLD')
            },
            'processing_resolution': self.algorithm.processing_size,
            'ground_truth_policy': 'aligned error statistics only; never an attitude, position or control input',
            'options': vars(options),
            'control_policy': 'Shared PnP VisualServo; Position zero-velocity priming, pilot selects Offboard; fresh MEASURED/PREDICTED may navigate; no mission upload or automatic Offboard request',
            'log_schema': 'Compact shared PnP control fields plus 5 aligned truth fields; no aliases',
            'csv_fields': self.csv_fields,
            'log_interval_ms': self.runtime.POSE_LOG_INTERVAL_MS,
            'control_config': {key:getattr(self.algorithm.config,key) for key in
                ('TARGET_HEIGHT_M','KP_XY','KP_Z','MAX_SPEED_XY','MAX_SPEED_Z',
                 'MAX_ACCEL_XY','MAX_ACCEL_Z','CONTROL_INTERVAL_MS','LOST_BRAKE_MS',
                 'MAX_PREDICTION_MS','ARRIVAL_XY_M','ARRIVAL_Z_M','ARRIVAL_DWELL_MS')},
        })
        self.settings = settings
        self.settings_path = log_dir / f'{name}.json'
        self.settings_path.write_text(json.dumps(settings, indent=2, ensure_ascii=False))
        self.sync_logger = None
        self.last_sync_log_error = None
        if self.link:
            sync_path = str(log_dir / f'{name}_sync.csv')
            source_info = {'algorithm_dir':str(Path(options.algorithm_dir).resolve())}
            try:
                for label,path in (
                    ('flight_link_sha256',Path(options.algorithm_dir)/'flight_link.py'),
                    ('config_sha256',Path(options.algorithm_dir)/'config.py'),
                    ('main_sha256',Path(options.algorithm_dir)/'main.py'),
                    ('tracker_sha256',Path(options.algorithm_dir)/'target_tracker.py'),
                    ('servo_sha256',Path(options.algorithm_dir)/'visual_servo.py'),
                    ('detector_sha256',Path(options.algorithm_dir)/'armor_detect.py'),
                    ('pose_solver_sha256',Path(options.algorithm_dir)/'pose_solver.py'),
                    ('link_service_sha256',Path(__file__).with_name('link_service.py'))):
                    source_info[label] = hashlib.sha256(path.read_bytes()).hexdigest()
            except OSError as error:
                source_info['hash_error'] = str(error)
            self.sync_logger = self.algorithm.modules.comm_log.CommCSVLogger(
                self.link,sync_path,clock=self.truth,interval_ms=1000)
            self.settings['sync_diagnostics'] = {'path':sync_path,
                'source':source_info,
                'policy':'Shared PnP synchronization decisions; compact 1 Hz summaries plus anomaly events',
                'time_observation':'Request stamped at write and fresh receive processing time; not hardware timestamps'}
            self.settings_path.write_text(json.dumps(settings, indent=2, ensure_ascii=False))
            self.get_logger().info(f'TIMESYNC diagnostics={sync_path}')
            self.link.start()
        self.timer = self.create_timer(.01, self.on_tick)
        self.get_logger().info(f'Mode={settings["mode"]}; CSV={self.csv_path}')
        self.get_logger().info('Waiting for image, CameraInfo and Gazebo simulation clock')
        if options.enable_control:
            self.get_logger().info('Offboard待命：在Position中人工起飞、靠近目标并减速；PRIMING至少约1.2秒后手动切Offboard。')

    def on_info(self, message):
        try:
            previous = self.algorithm.calibration
            self.algorithm.set_camera(message.width, message.height,
                                      message.k, message.d, message.distortion_model)
            if self.algorithm.calibration != previous:
                self.get_logger().info(
                    f'CameraInfo native={message.width}x{message.height}; '
                    f'algorithm={self.algorithm.processing_size}; '
                    f'scaled fx/fy/cx/cy={self.algorithm.config.INTRINSICS}; '
                    f'd={list(message.d)}; original detector configuration')
                self.settings['camera_calibration'] = {
                    'native_size': [message.width,message.height],
                    'native_k': list(message.k), 'native_d': list(message.d),
                    'processing_size': self.algorithm.processing_size,
                    'processing_intrinsics': self.algorithm.config.INTRINSICS,
                    'resize': 'INTER_AREA; pixel-center mapping',
                }
                # The static metadata is written before CameraInfo arrives;
                # refresh its camera values to the actual processing K/D.
                self.settings['camera'].update(
                    intrinsics=self.algorithm.config.INTRINSICS,
                    distortion=self.algorithm.config.DIST_COEFFS)
                self.settings_path.write_text(json.dumps(self.settings,indent=2,ensure_ascii=False))
                self.last_result = None
        except (ValueError, TypeError) as error:
            self.algorithm.solver = None
            self.algorithm.config.CALIBRATION_CONFIRMED = False
            self.warn(str(error))

    def on_image(self, message):
        self.pending.append(message)
        self.last_image_wall = time.monotonic()

    def warn(self, message):
        now = time.monotonic()
        if now - self.last_warning_wall > 2:
            self.get_logger().warning(message)
            self.last_warning_wall = now

    def flight_state(self, now):
        if self.link:
            with self.link.lock:
                # The worker may have received newer telemetry since the
                # caller sampled time. Never compare it to an older timestamp.
                return self.link.state(self.truth.now_us())
        return {'ready': False, 'reason': 'MAVLINK_UNAVAILABLE', 'armed': False,
                'offboard': False, 'yaw': 0., 'velocity': None}

    def on_tick(self):
        tick_started = time.monotonic()
        now = self.truth.now_us()
        if self.truth.generation != self.generation:
            self.generation = self.truth.generation
            self.algorithm.reset(reset_control=False)
            self.last_result = self.last_frame_us = self.last_image_us = None
            self.last_control_us = now
            self.last_csv_us = None
            self.last_log_state = None
            self.pending.clear()
            if self.link:
                self.link.reset_clock_generation(self.generation)
        if now <= 0:
            self.record_sync_diagnostics(tick_started)
            self.warn('Waiting for Gazebo /clock')
            return
        if self.link:
            self.link.poll()
            if self.link.generation != self.link_generation:
                self.link_generation = self.link.generation
                self.algorithm.reset(reset_control=False)
                self.last_result = None
        enabled = self.runtime.vision_enabled(self.flight_state(now))
        if enabled != self.vision_enabled:
            self.algorithm.reset(reset_control=False)
            self.last_result = None
            self.vision_enabled = enabled
        if self.pending and self.algorithm.solver is not None:
            message = self.pending.pop()
            self.pending.clear()
            capture = stamp_us(message.header.stamp)
            age = now-capture
            if capture <= 0 or age > self.algorithm.config.MAX_PREDICTION_MS*1000:
                self.drop_count += 1
                self.warn(f'Dropping stale image: age={age/1000:.1f} ms')
            elif age < 0:
                self.pending.append(message)  # Wait for the Gazebo clock, not for truth.
            elif self.last_frame_us is None or capture > self.last_frame_us:
                try:
                    self.process_frame(message,capture,self.truth.sample(capture))
                except (ValueError,cv2.error) as error:
                    self.warn(f'Frame rejected: {error}')
                    self.last_result = None
                self.last_frame_us = capture
        now = self.truth.now_us()
        if now-self.last_control_us >= self.algorithm.config.CONTROL_INTERVAL_MS*1000:
            self.control_tick(now)
            self.last_control_us = self.control_time_us
        if self.link:
            self.link.poll()
        self.record_sync_diagnostics(tick_started)
        wall = time.monotonic()
        if wall-self.last_flush_wall >= self.runtime.POSE_LOG_FLUSH_MS*.001:
            self.csv_file.flush()
            self.last_flush_wall = wall
        if self.last_image_wall is None or wall-self.last_image_wall > 3:
            self.warn('No new image; check camera bridge and simulation playback')
        if self.algorithm.solver is None:
            self.warn('Waiting for /pnp/camera_info')

    def record_sync_diagnostics(self, tick_started):
        if self.sync_logger is None:
            return
        self.sync_logger.record()
        if self.sync_logger.error and self.sync_logger.error != self.last_sync_log_error:
            self.get_logger().warning(f'TIMESYNC diagnostic log stopped: {self.sync_logger.error}')
            self.last_sync_log_error = self.sync_logger.error

    def process_frame(self, message, capture, truth):
        started = time.perf_counter()
        frame = self.algorithm.prepare_image(
            self.bridge.imgmsg_to_cv2(message,desired_encoding='bgr8'))
        result = self.algorithm.process(frame,capture,self.link,self.vision_enabled)
        fps = (1e6/(capture-self.last_image_us)
               if self.last_image_us is not None and capture > self.last_image_us else 0.)
        self.last_result = result
        self.last_image_us = capture
        self.frame_count += 1
        if self.link:
            self.link.poll()
        now = self.truth.now_us()
        if now-self.last_control_us >= self.algorithm.config.CONTROL_INTERVAL_MS*1000:
            self.control_tick(now)
            self.last_control_us = self.control_time_us
        if self.link:
            self.link.poll()
        now = self.truth.now_us()
        flight = self.flight_state(now)
        values = self.runtime.pose_log_row(
            self.frame_count,capture,now,None,(time.perf_counter()-started)*1e6,
            fps,result['solve_us'],result['q'],result['pose'],result['relative'],
            result['track'],result['velocity'],flight,self.controller_state(),
            result['points'],result['color'],self.vision_enabled,self.control_time_us,
            self.control_track,self.last_command,self.setpoint_submitted)
        row = dict(zip(self.runtime.POSE_LOG_FIELDS,values))
        row.update({key:None for key in TRUTH_FIELDS})
        if truth is not None:
            for axis,component in zip(('x','y','z'),truth['relative_ned_true']):
                row[f'true_p_local_{axis}_m'] = component
            row['truth_dt_us'],row['truth_source'] = truth.get('truth_dt_us'),truth.get('source')
        self.status_publisher.publish(String(data=json.dumps(finite_json(row),allow_nan=False)))
        pose = result['pose']
        if pose and pose.get('valid'):
            output = PoseStamped()
            output.header.stamp = message.header.stamp
            output.header.frame_id = 'pnp_camera_optical'
            output.pose.position.x,output.pose.position.y,output.pose.position.z = pose['position']
            w,x,y,z = quaternion_from_rotation(pose['rotation'])
            output.pose.orientation.w,output.pose.orientation.x = w,x
            output.pose.orientation.y,output.pose.orientation.z = y,z
            self.pose_publisher.publish(output)
        if result['relative'] is not None:
            output = PointStamped()
            output.header.stamp = message.header.stamp
            output.header.frame_id = 'pnp_local_ned'
            output.point.x,output.point.y,output.point.z = result['relative']
            self.relative_publisher.publish(output)
        debug_frame = cv2.resize(frame,self.algorithm.capture_size,interpolation=cv2.INTER_NEAREST)
        self.draw(debug_frame,result,row)
        debug = self.bridge.cv2_to_imgmsg(debug_frame,encoding='bgr8')
        debug.header = message.header
        self.image_publisher.publish(debug)
        state = self.runtime.log_state_key(row.get('track'),row.get('nav'),flight,
                                           result.get('alignment',{}))
        if (self.last_csv_us is None or state != self.last_log_state or
                now-self.last_csv_us >= self.runtime.POSE_LOG_INTERVAL_MS*1000):
            self.record(row)
            self.last_csv_us = now
            self.last_log_state = state
        wall = time.monotonic()
        if wall-self.last_report_wall >= self.algorithm.config.DEBUG_PRINT_INTERVAL_MS*.001:
            self.get_logger().info(
                f'track={row["track"]} nav={row["nav"]} flight={row["flight"]} '
                f'offboard={row["offboard"]} vision_enabled={row["vision_enabled"]}')
            self.algorithm.modules.detector.reset_stats()
            self.last_report_wall = wall

    def draw(self, frame, result, row):
        config = self.algorithm.config
        scale = np.array((frame.shape[1]/config.IMAGE_WIDTH,frame.shape[0]/config.IMAGE_HEIGHT))
        if config.DEBUG_MODE >= 2 and result['points'] is not None:
            polygon = np.rint((np.asarray(result['points'])+.5)*scale-.5).astype(np.int32)
            cv2.polylines(frame,[polygon],True,(0,220,0),2)
        if config.DEBUG_MODE:
            center,color = None,(0,160,255)
            pose,track,q = result['pose'],result['track'],result['q']
            if pose and pose.get('valid') and track.get('matched'):
                center,color = pose['center'],(0,255,0)
            elif track.get('position') is not None and q is not None and self.runtime.configured():
                point = self.algorithm.modules.pose.local_to_camera(track['position'],q,config.R_BC,config.T_BC_M)
                center = self.algorithm.solver.project(point)
            elif result['detection'] is not None:
                center = result['detection']['center']
            if center is not None and 0 <= center[0] < config.IMAGE_WIDTH and 0 <= center[1] < config.IMAGE_HEIGHT:
                pixel = tuple(np.rint((np.asarray(center)+.5)*scale-.5).astype(int))
                cv2.drawMarker(frame,pixel,color,cv2.MARKER_CROSS,20,2)
        def value(key):
            v = row.get(key)
            return '--' if v is None else f'{v:.3f}'
        def position(axis):
            v = row.get(f'control_p_local_{axis}_m')
            if v is None:
                v = row.get(f'p_local_raw_{axis}_m')
            return '--' if v is None else f'{v:.3f}'
        solve_ms = None if result.get('solve_us') is None else result['solve_us']*.001
        solve_text = '--' if solve_ms is None else f'{solve_ms:.3f}'
        lines = [f'track={row["track"]} | nav={row["nav"]}',
                 f'armed={row["armed"]} offboard={row["offboard"]} | flight={row["flight"]}',
                 f'vision_enabled={row["vision_enabled"]} | observation_age_ms={value("observation_age_ms")}',
                 f'p_local N/E/D: {position("x")} / {position("y")} / {position("z")} m',
                 f'command N/E/D: {value("command_vn_mps")} / {value("command_ve_mps")} / {value("command_vd_mps")} m/s',
                 f'attitude={row.get("attitude_status","--")} solve_ms={solve_text} submitted={row["setpoint_submitted"]}']
        for i,line in enumerate(lines):
            position=(12,26+i*27)
            cv2.putText(frame,line,position,cv2.FONT_HERSHEY_SIMPLEX,.56,(0,0,0),4,cv2.LINE_AA)
            cv2.putText(frame,line,position,cv2.FONT_HERSHEY_SIMPLEX,.56,(255,255,255),1,cv2.LINE_AA)

    def controller_state(self):
        return self.algorithm.control_servo.state

    def control_tick(self, now):
        if self.link:
            with self.link.lock:
                # State, history integration and submitted reference must use
                # one consistent clock generation. This lock covers only the
                # short control calculation, never image processing or I/O logs.
                return self._control_tick(self.truth.now_us())
        return self._control_tick(now)

    def _control_tick(self, now):
        result = self.last_result
        track = (result['track'] if result is not None else
                 {'position':None,'state':'IDLE' if not self.vision_enabled else 'LOST','generation':0})
        if result is not None and self.link and result.get('link_generation') != self.link.generation:
            track = dict(track,position=None,state='LOST')
        stamp = result['stamp_us'] if result is not None else now
        self.control_track,self.last_command,self.setpoint_submitted = self.runtime.update_control(
            track,self.flight_state(now),stamp,now,self.algorithm.control_servo,self.link)
        self.control_time_us = now

    def record(self, row):
        self.csv.writerow(row)

    def close(self):
        if self.link:
            self.link.close()
        if self.sync_logger:
            self.sync_logger.close()
        if self.link:
            self.link.cancel_setpoint()
        if self.transport:
            self.transport.close()
        self.truth.close()
        self.csv_file.flush()
        self.csv_file.close()


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    sim_dir = Path(__file__).resolve().parent.parent
    parser.add_argument('--algorithm-dir', default=str(sim_dir.parent / 'PnP'))
    parser.add_argument('--px4-dir', default='/home/hhkqwq/PX4-Autopilot')
    parser.add_argument('--world', default='pnp_pillar')
    parser.add_argument('--model', default='x500_mono_cam_down_0')
    parser.add_argument('--image-topic', default='/pnp/image_raw')
    parser.add_argument('--camera-info-topic', default='/pnp/camera_info')
    parser.add_argument('--log-dir', default=str(sim_dir / 'logs'))
    parser.add_argument('--opencv-threads', type=int, default=2)
    parser.add_argument('--mavlink-port', type=int, default=14540)
    parser.add_argument('--no-mavlink', action='store_true')
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--enable-control', dest='enable_control', action='store_const', const=True, default=None,
                      help='Enable manual Offboard control; default follows PnP/config.py NAV_ENABLE')
    mode.add_argument('--observe', dest='enable_control', action='store_const', const=False,
                      help='Observation only: continuous detection, no control setpoints')
    options, ros_args = parser.parse_known_args()
    if any(arg == '--mission' or arg.startswith('--mission=') for arg in ros_args):
        parser.error('--mission has been removed; use manual Offboard or --observe')
    if any(arg in ('--truth-attitude','--max-fps') or arg.startswith(('--truth-attitude=','--max-fps=')) for arg in ros_args):
        parser.error('Simulation-only attitude assistance and frame caps were removed; use the PnP configuration')
    if options.no_mavlink and options.enable_control is not False:
        parser.error('--no-mavlink requires explicit --observe')
    return options, ros_args


def main():
    options, ros_args = arguments()
    cv2.setNumThreads(max(1, options.opencv_threads))
    def shutdown_signal(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, shutdown_signal)
    rclpy.init(args=ros_args)
    node = None
    try:
        node = PnpNode(options)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
