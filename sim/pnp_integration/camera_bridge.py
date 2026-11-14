#!/usr/bin/env python3
"""Publish a camera view with the original OpenMV K/D and image size."""
import argparse
from copy import deepcopy
from pathlib import Path
import signal

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image

from camera_model import CalibratedCamera, load_calibration


class CameraBridge(Node):
    def __init__(self, directory):
        super().__init__('pnp_openmv_camera')
        size, matrix, distortion = load_calibration(directory)
        self.camera = CalibratedCamera(size, matrix, distortion)
        self.bridge = CvBridge()
        self.pending = None
        self.last_warning = None
        self.image_pub = self.create_publisher(Image, '/pnp/image_raw', qos_profile_sensor_data)
        self.info_pub = self.create_publisher(CameraInfo, '/pnp/camera_info', qos_profile_sensor_data)
        self.info = CameraInfo()
        self.info.width, self.info.height = size
        self.info.distortion_model = 'plumb_bob'
        self.info.k = list(matrix.flat)
        self.info.d = list(distortion)
        self.info.r = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
        self.info.p = [matrix[0, 0], 0., matrix[0, 2], 0.,
                       0., matrix[1, 1], matrix[1, 2], 0., 0., 0., 1., 0.]
        self.image_sub = self.create_subscription(
            Image, '/pnp/gazebo_image_raw', self.on_image, qos_profile_sensor_data)
        self.info_sub = self.create_subscription(
            CameraInfo, '/pnp/gazebo_camera_info', self.on_info, qos_profile_sensor_data)
        self.timer = self.create_timer(.005, self.process_latest)
        self.get_logger().info(
            f'OpenMV camera {size[0]}x{size[1]}; '
            f'fx/fy/cx/cy={(matrix[0,0],matrix[1,1],matrix[0,2],matrix[1,2])}; '
            f'D={list(distortion)}; calibration={Path(directory)/"config.py"}')

    def warn(self, error):
        message = str(error)
        if message != self.last_warning:
            self.get_logger().warning(message)
            self.last_warning = message

    def on_info(self, message):
        try:
            if self.camera.configure(message.width, message.height, message.k,
                                     message.d, message.distortion_model):
                self.get_logger().info(
                    f'Gazebo source {message.width}x{message.height}; '
                    'calibrated image mapping ready; target truth is not used')
            self.last_warning = None
        except (ValueError, TypeError, cv2.error) as error:
            self.camera.source = self.camera.maps = None
            self.warn(error)

    def on_image(self, message):
        self.pending = message

    def process_latest(self):
        if self.pending is None or self.camera.source is None:
            return
        message, self.pending = self.pending, None
        try:
            frame = self.camera.render(self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8'))
            output = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            output.header = deepcopy(message.header)
            info = deepcopy(self.info)
            info.header = deepcopy(message.header)
            # Preserve exposure timestamp/frame ID; the remap adds no artificial time shift.
            self.info_pub.publish(info)
            self.image_pub.publish(output)
            self.last_warning = None
        except (ValueError, TypeError, cv2.error) as error:
            self.warn(error)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--algorithm-dir', default=str(Path(__file__).resolve().parents[2]/'PnP'))
    parser.add_argument('--opencv-threads', type=int, default=2)
    options, _ = parser.parse_known_args()
    cv2.setNumThreads(max(1, options.opencv_threads))
    # The shared launcher forwards algorithm options; they are not ROS arguments.
    rclpy.init(args=[])
    def stop(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, stop)
    node = None
    try:
        node = CameraBridge(options.algorithm_dir)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
