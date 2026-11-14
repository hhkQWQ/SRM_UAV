#!/usr/bin/env python3
"""ROS 2 Gazebo input and diagnostics for the original OpenMV PnP algorithm."""
import argparse
from collections import deque
import csv
from datetime import datetime
import json
import math
from pathlib import Path
import signal
import time
import xml.etree.ElementTree as ET

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, PoseStamped, TwistStamped
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from algorithm_core import Algorithm, quaternion_from_rotation
from gazebo_truth import GroundTruth
from udp_uart import UdpUart


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


def check_stock_camera(px4):
    models = Path(px4) / 'Tools/simulation/gz/models'
    top = ET.parse(models / 'x500_mono_cam_down/model.sdf').getroot().find('model')
    include = next(x for x in top.findall('include')
                   if x.findtext('uri') == 'model://mono_cam')
    camera = tuple(float(x) for x in include.findtext('pose').split())
    base = ET.parse(models / 'x500_base/model.sdf').find('model')
    base_pose = tuple(float(x) for x in base.findtext('pose').split())
    if not (np.allclose(camera, (0., 0., .1, 0., 1.5707, 0.), atol=1e-8)
            and np.allclose(base_pose, (0., 0., .24, 0., 0., 0.), atol=1e-8)):
        raise ValueError('Camera/base installation changed; update simulation extrinsics before running')
    imager = ET.parse(models / 'mono_cam/model.sdf').find('./model/link/sensor')
    sensor_pose = tuple(float(x) for x in imager.findtext('pose', '0 0 0 0 0 0').split())
    if not np.allclose(sensor_pose, np.zeros(6), atol=1e-8):
        raise ValueError('Camera sensor has a nonzero local pose; update extrinsics')


CSV_FIELDS = (
    'frame', 'sim_time_s', 'image_age_ms', 'pipeline_ms', 'pose_valid', 'pose_reason',
    'track_state', 'attitude_source', 'flight_reason', 'armed', 'offboard',
    'p_camera_x_m', 'p_camera_y_m', 'p_camera_z_m', 'reprojection_px',
    'relative_n_m', 'relative_e_m', 'relative_d_m',
    'true_camera_x_m', 'true_camera_y_m', 'true_camera_z_m',
    'camera_error_m', 'relative_error_m', 'truth_dt_us', 'truth_source',
    'body_world_x_m', 'body_world_y_m', 'body_world_z_m',
    'shadow_state', 'shadow_vn_mps', 'shadow_ve_mps', 'shadow_vd_mps',
    'control_enabled', 'control_state', 'control_submitted',
    'u1', 'v1', 'u2', 'v2', 'u3', 'v3', 'u4', 'v4',
)


class PnpNode(Node):
    def __init__(self, options):
        super().__init__('pnp_sim_algorithm')
        self.options = options
        check_stock_camera(options.px4_dir)
        self.algorithm = Algorithm(options.algorithm_dir, options.enable_control)
        self.truth = GroundTruth(options.world, options.model)
        self.bridge = CvBridge()
        self.pending = deque(maxlen=1)
        self.last_result = None
        self.last_image_us = None
        self.last_image_wall = None
        self.last_frame_us = None
        self.last_process_wall = 0.
        self.last_control_us = -1_000_000
        self.last_report_wall = 0.
        self.last_warning_wall = 0.
        self.last_csv_wall = 0.
        self.last_flush_wall = 0.
        self.frame_count = 0
        self.drop_count = 0
        self.generation = self.truth.generation
        self.attitude_source = 'NONE'
        self.control_submitted = False
        self.transport = self.link = None
        if not options.no_mavlink:
            try:
                self.transport = UdpUart(options.mavlink_port)
                self.link = self.algorithm.modules.link.FlightLink(
                    uart=self.transport, clock=self.truth)
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
        self.shadow_publisher = self.create_publisher(TwistStamped, '/pnp/suggested_velocity_ned', 10)
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
        self.csv = csv.DictWriter(self.csv_file, fieldnames=CSV_FIELDS)
        self.csv.writeheader()
        settings = {
            'mode': 'control' if options.enable_control else 'observe',
            'algorithm_dir': str(Path(options.algorithm_dir).resolve()),
            'simulation_overrides': {
                'R_BC': self.algorithm.config.R_BC,
                'T_BC_M': self.algorithm.config.T_BC_M,
            },
            'detector_config': {
                key: getattr(self.algorithm.config,key) for key in
                ('LIGHT_EXTRACT_MODE','TARGET_COLOR','LIGHT_L_MIN','RED_THRESHOLD','BLUE_THRESHOLD')
            },
            'processing_resolution': self.algorithm.processing_size,
            'ground_truth_policy': 'metrics and labelled attitude assistance only; never a control position input',
            'shadow_policy': 'simulated ready/armed/offboard gates; diagnostic output only',
            'options': vars(options),
        }
        self.settings = settings
        self.settings_path = log_dir / f'{name}.json'
        self.settings_path.write_text(json.dumps(settings, indent=2, ensure_ascii=False))
        self.timer = self.create_timer(.01, self.on_tick)
        self.get_logger().info(f'Mode={settings["mode"]}; CSV={self.csv_path}')
        self.get_logger().info('Waiting for image, CameraInfo and Gazebo simulation clock')

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
        return (self.link.state(now) if self.link else
                {'ready': False, 'reason': 'MAVLINK_UNAVAILABLE', 'armed': False,
                 'offboard': False, 'yaw': 0., 'velocity': None})

    def on_tick(self):
        now = self.truth.now_us()
        if self.truth.generation != self.generation:
            self.generation = self.truth.generation
            self.algorithm.reset()
            self.last_result = None
            self.last_frame_us = None
            self.last_control_us = now
            if self.link:
                self.link._restart()
        if now <= 0:
            self.warn('Waiting for Gazebo /clock; check that the pnp_pillar world is running')
            return
        if self.link:
            self.link.poll()
            if self.link.generation != self.link_generation:
                self.link_generation = self.link.generation
                self.algorithm.reset()
                self.last_result = None
                self.attitude_source = 'NONE'
        wall = time.monotonic()
        if (self.pending and self.algorithm.solver is not None
                and (self.options.max_fps is None or
                     wall-self.last_process_wall >= 1./self.options.max_fps)):
            message = self.pending.pop()
            self.pending.clear()
            capture = stamp_us(message.header.stamp)
            age = now - capture
            if capture <= 0 or age < -50_000 or age > 250_000:
                self.drop_count += 1
                self.warn(f'Dropping stale image: age={age / 1000:.1f} ms')
            elif self.last_frame_us is None or capture > self.last_frame_us:
                # Wait up to 40 ms for a bracketing world pose; after that the
                # camera pose still runs and unavailable truth is left blank.
                truth = self.truth.sample(capture)
                if truth is None and age < 40_000:
                    self.pending.append(message)
                else:
                    try:
                        self.process_frame(message, capture, now, truth)
                    except (ValueError, cv2.error) as error:
                        self.warn(f'Frame rejected: {error}')
                        self.last_result = None
                    self.last_frame_us = capture
                    self.last_process_wall = time.monotonic()
        if now - self.last_control_us >= 50_000:
            self.control_tick(now)
            self.last_control_us = now
        if self.link:
            self.link.poll()
        if wall - self.last_flush_wall > 1:
            self.csv_file.flush()
            self.last_flush_wall = wall
        if self.last_image_wall is None or wall - self.last_image_wall > 3:
            self.warn('No new image: keep start_camera_bridge.sh running and simulation playing')
        if self.algorithm.solver is None:
            self.warn('Waiting for /pnp/camera_info; keep both image and CameraInfo bridges running')

    def process_frame(self, message, capture, now, truth):
        frame = self.algorithm.prepare_image(
            self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8'))
        q = self.link.attitude_at(capture) if self.link else None
        velocity = self.link.velocity_at(capture) if self.link else None
        previous_source = self.attitude_source
        self.attitude_source = 'PX4_MAVLINK' if q is not None else 'NONE'
        if (q is None and truth is not None and self.options.truth_attitude
                and not self.options.enable_control):
            q = truth['q_nb']
            velocity = truth.get('velocity_ned')
            self.attitude_source = 'GAZEBO_TRUTH_ASSISTED_OBSERVATION'
        if self.attitude_source != previous_source:
            # Do not blend tracking or prediction history across attitude
            # sources. Control mode never uses truth-assisted observations.
            self.algorithm.reset()
        started = time.perf_counter()
        result = self.algorithm.process(frame, capture, q, velocity)
        result['pipeline_ms'] = (time.perf_counter() - started) * 1000.
        result['stamp_us'] = capture
        result['attitude_source'] = self.attitude_source
        flight = self.flight_state(now)
        shadow = self.algorithm.shadow(result, capture, flight.get('yaw', 0.))
        result['shadow'] = shadow
        self.last_result = result
        self.last_image_us = capture
        self.frame_count += 1
        pose = result['pose']
        valid = bool(pose and pose.get('valid'))
        camera = pose.get('position') if valid else None
        relative = result['relative_ned']
        true_camera = truth['p_camera_true'] if truth else None
        camera_error = (float(np.linalg.norm(np.asarray(camera) - true_camera))
                        if camera is not None and true_camera is not None else None)
        relative_error = (float(np.linalg.norm(np.asarray(relative) - truth['relative_ned_true']))
                          if relative is not None and truth else None)
        status = {'frame': self.frame_count, 'sim_time_s': capture / 1e6,
                  'pose_valid': valid, 'pose_reason': pose.get('reason') if pose else 'NO_PAIR',
                  'p_camera_m': camera, 'relative_ned_m': relative,
                  'track': result['track']['state'], 'attitude_source': self.attitude_source,
                  'flight_reason': flight['reason'], 'armed': flight['armed'],
                  'offboard': flight['offboard'], 'camera_error_m': camera_error,
                  'relative_error_m': relative_error,
                  'truth_dt_us': truth.get('truth_dt_us') if truth else None,
                  'truth_source': truth.get('source') if truth else None,
                  'true_camera_m': true_camera,
                  'body_world_m': truth['body_world'] if truth else None,
                  'shadow_simulated_gates': shadow,
                  'control_enabled': self.options.enable_control,
                  'control_state': self.algorithm.control_servo.state,
                  'image_age_ms': (now - capture) / 1000,
                  'pipeline_ms': result['pipeline_ms'],
                  'red_lights': result['lights_red'],
                  'dropped_stale_images': self.drop_count}
        status['processing_resolution'] = self.algorithm.processing_size
        status['native_resolution'] = self.algorithm.capture_size
        self.status_publisher.publish(String(data=json.dumps(finite_json(status), allow_nan=False)))
        if valid:
            output = PoseStamped()
            output.header = message.header
            output.header.frame_id = 'pnp_camera_optical'
            output.pose.position.x, output.pose.position.y, output.pose.position.z = camera
            w, x, y, z = quaternion_from_rotation(pose['rotation'])
            output.pose.orientation.w, output.pose.orientation.x = w, x
            output.pose.orientation.y, output.pose.orientation.z = y, z
            self.pose_publisher.publish(output)
        if relative is not None:
            output = PointStamped()
            output.header = message.header
            output.header.frame_id = 'pnp_local_ned'
            output.point.x, output.point.y, output.point.z = relative
            self.relative_publisher.publish(output)
        if shadow is not None:
            output = TwistStamped()
            output.header = message.header
            output.header.frame_id = 'pnp_local_ned'
            output.twist.linear.x, output.twist.linear.y, output.twist.linear.z = shadow['velocity']
            self.shadow_publisher.publish(output)
        # Upscaling is for the display only; detection and PnP used the original
        # algorithm's 320x240 resolution and the corresponding scaled K.
        debug_frame = cv2.resize(frame, self.algorithm.capture_size,
                                 interpolation=cv2.INTER_NEAREST)
        self.draw(debug_frame, result, status)
        debug = self.bridge.cv2_to_imgmsg(debug_frame, encoding='bgr8')
        debug.header = message.header
        self.image_publisher.publish(debug)
        wall = time.monotonic()
        if wall - self.last_csv_wall >= .1:
            self.record(result, status, flight, truth)
            self.last_csv_wall = wall
        if wall - self.last_report_wall > 1:
            p = 'none' if camera is None else ','.join(f'{x:.3f}' for x in camera)
            err = 'n/a' if camera_error is None else f'{camera_error:.3f}'
            self.get_logger().info(
                f'{status["pose_reason"]} track={status["track"]} '
                f'p_C=({p})m err={err}m '
                f'flight={flight["reason"]} control={self.options.enable_control}')
            self.last_report_wall = wall

    def draw(self, frame, result, status):
        scale = np.array((frame.shape[1]/self.algorithm.config.IMAGE_WIDTH,
                          frame.shape[0]/self.algorithm.config.IMAGE_HEIGHT))
        points = result['points']
        if points is not None:
            polygon = np.rint((np.asarray(points)+.5)*scale-.5).astype(np.int32)
            cv2.polylines(frame, [polygon], True, (0, 220, 0), 2)
            for i, point in enumerate(polygon):
                cv2.circle(frame, tuple(point), 4, (0, 240, 255), -1)
                cv2.putText(frame, str(i + 1), tuple(point + (5, -5)),
                            cv2.FONT_HERSHEY_SIMPLEX, .55, (0, 240, 255), 1)
        pose = result['pose']
        if pose and pose.get('valid'):
            center = tuple(np.rint((np.asarray(pose['center'])+.5)*scale-.5).astype(int))
            cv2.drawMarker(frame, center, (0, 255, 0), cv2.MARKER_CROSS, 20, 2)
        lines = [f'{"CONTROL" if self.options.enable_control else "OBSERVE"} | '
                 f'{status["pose_reason"]} | {status["track"]}',
                 f'attitude: {self.attitude_source}',
                 f'PX4: {status["flight_reason"]} | '
                 f'controller: {self.algorithm.control_servo.state}',
                 f'pipeline: {status["pipeline_ms"]:.1f} ms | '
                 f'detection input: {self.algorithm.processing_size} | original config']
        if status['p_camera_m'] is not None:
            p = status['p_camera_m']
            lines.append(f'target in camera xyz: {p[0]:+.3f}, {p[1]:+.3f}, {p[2]:+.3f} m')
        if status['relative_ned_m'] is not None:
            p = status['relative_ned_m']
            lines.append(f'target from body N/E/D: {p[0]:+.3f}, {p[1]:+.3f}, {p[2]:+.3f} m')
        if status['camera_error_m'] is not None:
            lines.append(f'Gazebo camera-position error: {status["camera_error_m"]:.4f} m')
        if status['body_world_m'] is not None:
            p = status['body_world_m']
            lines.append(f'body world ENU: {p[0]:.2f}, {p[1]:.2f}, {p[2]:.2f} m')
        for i, line in enumerate(lines):
            y = 26 + i * 27
            cv2.putText(frame, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX, .59,
                        (0, 0, 0), 4, cv2.LINE_AA)
            cv2.putText(frame, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX, .59,
                        (255, 255, 255), 1, cv2.LINE_AA)

    def control_tick(self, now):
        self.control_submitted = False
        if not self.link or not self.options.enable_control:
            return
        flight = self.link.state(now)
        result = self.last_result
        fresh = (result is not None and 0 <= now - result['stamp_us'] <= 250_000
                 and result['attitude_source'] == 'PX4_MAVLINK'
                 and self.last_image_wall is not None
                 and time.monotonic() - self.last_image_wall < .5)
        if not fresh:
            flight = dict(flight, ready=False, reason='IMAGE_OR_PX4_ATTITUDE_STALE')
        track = dict(result['track']) if result else {'position': None, 'state': 'LOST', 'generation': 0}
        if fresh and track.get('position') is not None:
            delay = (now - result['stamp_us']) / 1e6
            v = flight.get('velocity')
            if v is not None:
                track['position'] = tuple(track['position'][i] - v[i] * delay for i in range(3))
            if track.get('age_ms') is None or track['age_ms'] + delay * 1000 > self.algorithm.config.MAX_PREDICTION_MS:
                track['position'] = None
                track['state'] = 'LOST'
        output = self.algorithm.control_servo.update(
            track, flight, now // 1000, not self.algorithm.config.readiness_errors())
        if output is None:
            self.link.cancel_setpoint()
        else:
            self.control_submitted = self.link.send_velocity(output['velocity'], output['yaw'], now)

    def record(self, result, status, flight, truth):
        row = {key: status.get(key) for key in ('pose_valid', 'pose_reason', 'attitude_source',
               'flight_reason', 'armed', 'offboard', 'camera_error_m', 'relative_error_m',
               'truth_dt_us', 'truth_source', 'control_enabled')}
        row.update(frame=self.frame_count, sim_time_s=status['sim_time_s'],
                   image_age_ms=status['image_age_ms'], pipeline_ms=result['pipeline_ms'],
                   track_state=result['track']['state'],
                   reprojection_px=result['pose'].get('reprojection_px') if result['pose'] else None,
                   shadow_state=self.algorithm.shadow_servo.state,
                   control_state=self.algorithm.control_servo.state,
                   control_submitted=self.control_submitted)
        vectors = [('p_camera', status['p_camera_m'], ('x', 'y', 'z')),
                   ('relative', status['relative_ned_m'], ('n', 'e', 'd')),
                   ('true_camera', status['true_camera_m'], ('x', 'y', 'z')),
                   ('body_world', status['body_world_m'], ('x', 'y', 'z'))]
        for prefix, value, axes in vectors:
            for i, axis in enumerate(axes):
                row[f'{prefix}_{axis}_m'] = value[i] if value is not None else None
        for i, axis in enumerate(('n', 'e', 'd')):
            row[f'shadow_v{axis}_mps'] = result['shadow']['velocity'][i] if result['shadow'] else None
        if result['points'] is not None:
            for i, (u, v) in enumerate(result['points'], 1):
                row[f'u{i}'], row[f'v{i}'] = u, v
        self.csv.writerow(row)

    def close(self):
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
    parser.add_argument('--max-fps', type=float, default=None,
                        help='Optional explicit processing cap; default processes incoming frames')
    parser.add_argument('--opencv-threads', type=int, default=2)
    parser.add_argument('--mavlink-port', type=int, default=14540)
    parser.add_argument('--no-mavlink', action='store_true')
    parser.add_argument('--truth-attitude', action='store_true',
                        help='Explicit diagnostic-only Gazebo attitude assistance in observation mode')
    parser.add_argument('--enable-control', action='store_true',
                        help='Allow original controller velocity setpoints; never arms or switches modes')
    options, ros_args = parser.parse_known_args()
    if options.max_fps is not None and (not math.isfinite(options.max_fps) or options.max_fps <= 0):
        parser.error('--max-fps must be positive')
    if options.no_mavlink and options.enable_control:
        parser.error('--enable-control requires MAVLink; remove --no-mavlink')
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
