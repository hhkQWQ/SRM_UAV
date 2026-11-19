"""Gazebo input adapter for the unmodified OpenMV PnP modules.

The OpenMV image operations are approximated by openmv_compat. Pair scoring,
four-point ordering, planar pose estimation, tracking and servo are the actual
modules loaded from the user's PnP directory, with a separate in-memory config.
"""
from contextlib import contextmanager
import importlib.util
import math
from pathlib import Path
import sys
from types import SimpleNamespace

import cv2
import numpy as np

from openmv_compat import ImageAdapter, make_image_module


@contextmanager
def module_aliases(values):
    missing = object()
    previous = {key: sys.modules.get(key, missing) for key in values}
    sys.modules.update(values)
    try:
        yield
    finally:
        for key, value in previous.items():
            if value is missing:
                sys.modules.pop(key, None)
            else:
                sys.modules[key] = value


def load_file(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    original = sys.dont_write_bytecode
    try:
        sys.dont_write_bytecode = True
        spec.loader.exec_module(module)
    finally:
        sys.dont_write_bytecode = original
    return module


def load_original(directory, enable_control=None):
    directory = Path(directory).resolve()
    config = load_file('_pnp_sim_config', directory / 'config.py')
    # Only platform/camera adaptations. Detector settings remain exactly as
    # loaded from the user's config.py, including GRAY/ANY and both thresholds.
    if enable_control is not None:
        config.NAV_ENABLE = bool(enable_control)
    config.CALIBRATION_CONFIRMED = False
    # The Gazebo mount is generated from, and checked against, the original
    # R_BC/T_BC_M. Do not replace the user's physical installation calibration.
    image_module = make_image_module()
    with module_aliases({'config': config, 'image': image_module}):
        pose = load_file('_pnp_sim_pose_solver', directory / 'pose_solver.py')
        with module_aliases({'pose_solver': pose}):
            tracker = load_file('_pnp_sim_target_tracker', directory / 'target_tracker.py')
            servo = load_file('_pnp_sim_visual_servo', directory / 'visual_servo.py')
            link = load_file('_pnp_sim_flight_link', directory / 'flight_link.py')
            detector = load_file('_pnp_sim_armor_detect', directory / 'armor_detect.py')
            with module_aliases({'target_tracker': tracker, 'visual_servo': servo, 'flight_link': link}):
                runtime = load_file('_pnp_sim_main', directory / 'main.py')
    return SimpleNamespace(config=config, pose=pose, tracker=tracker,
                           servo=servo, link=link, detector=detector, runtime=runtime)


class Algorithm:
    def __init__(self, directory, enable_control=None):
        self.modules = load_original(directory, enable_control)
        self.config = self.modules.config
        self.processing_size = (int(self.config.IMAGE_WIDTH), int(self.config.IMAGE_HEIGHT))
        self.capture_size = None
        self.solver = None
        self.calibration = None
        self.work_clock = self.modules.link.Clock()
        self.reset()

    def reset(self, reset_control=True):
        self.tracker = self.modules.tracker.TargetTracker()
        if reset_control or not hasattr(self, 'control_servo'):
            self.control_servo = self.modules.servo.VisualServo()
        self.previous = None
        self.previous_color = None

    def set_camera(self, width, height, k, distortion, model='plumb_bob'):
        k = tuple(float(x) for x in k)
        d = tuple(float(x) for x in distortion)
        if len(k) != 9 or not all(math.isfinite(x) for x in k + d):
            raise ValueError('CameraInfo contains invalid calibration values')
        if model not in ('', 'plumb_bob') or len(d) not in (0, 4, 5):
            raise ValueError('Only zero distortion / plumb_bob 4 or 5 terms are supported')
        if width <= 0 or height <= 0 or min(k[0], k[4]) <= 0:
            raise ValueError('CameraInfo has invalid image size or focal length')
        d = (d + (0.,) * 5)[:5]
        calibration = (int(width), int(height), k, d)
        if calibration != self.calibration or self.solver is None:
            self.capture_size = (int(width), int(height))
            target_w, target_h = self.processing_size
            sx, sy = target_w / width, target_h / height
            # OpenCV resize maps source pixel centers to (u+0.5)*s-0.5.
            self.config.INTRINSICS = (k[0]*sx, k[4]*sy,
                                     (k[2]+.5)*sx-.5, (k[5]+.5)*sy-.5)
            self.config.DIST_COEFFS = d
            self.config.CALIBRATION_CONFIRMED = True
            self.solver = self.modules.pose.PlanarPoseSolver(
                self.config.TARGET_POINTS_T_M,
                self.config.INTRINSICS, self.config.DIST_COEFFS)
            self.calibration = calibration
            self.reset(reset_control=False)

    def prepare_image(self, bgr):
        """Match the original algorithm's resolution, with calibrated resizing."""
        if self.capture_size is None or bgr.shape[:2] != self.capture_size[::-1]:
            raise ValueError('Image size differs from native CameraInfo')
        if self.capture_size == self.processing_size:
            return bgr.copy()
        return cv2.resize(bgr, self.processing_size, interpolation=cv2.INTER_AREA)

    def process(self, bgr, stamp_us, link=None, vision_enabled=True):
        if self.solver is None:
            raise ValueError('Waiting for CameraInfo')
        if bgr.shape[:2] != (self.config.IMAGE_HEIGHT, self.config.IMAGE_WIDTH):
            raise ValueError('Image size differs from CameraInfo')
        result,self.previous,self.previous_color = self.modules.runtime.process_image(
            ImageAdapter(bgr,rgb565=True),stamp_us,
            self.solver,self.tracker,self.modules.detector,self.work_clock,link,
            self.previous,self.previous_color,vision_enabled)
        return result


def quaternion_from_rotation(rotation):
    r = np.asarray(rotation, dtype=float)
    # Stable symmetric eigendecomposition, scalar-last eigenvector.
    k = np.array([
        [r[0, 0]-r[1, 1]-r[2, 2], r[1, 0]+r[0, 1], r[2, 0]+r[0, 2], r[2, 1]-r[1, 2]],
        [r[1, 0]+r[0, 1], r[1, 1]-r[0, 0]-r[2, 2], r[2, 1]+r[1, 2], r[0, 2]-r[2, 0]],
        [r[2, 0]+r[0, 2], r[2, 1]+r[1, 2], r[2, 2]-r[0, 0]-r[1, 1], r[1, 0]-r[0, 1]],
        [r[2, 1]-r[1, 2], r[0, 2]-r[2, 0], r[1, 0]-r[0, 1], r.trace()],
    ]) / 3.
    _, vectors = np.linalg.eigh(k)
    x, y, z, w = vectors[:, -1]
    q = np.array([w, x, y, z])
    if q[0] < 0:
        q = -q
    return tuple(float(x) for x in q)
