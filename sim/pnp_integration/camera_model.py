"""Render the original OpenMV calibration from Gazebo's wide camera image.

This adapter changes image rays as well as CameraInfo. It does not simply
replace the calibration of an unchanged image. Geometry comes only from the
two camera calibrations; no target/vehicle truth is used.
"""
from pathlib import Path
import runpy

import cv2
import numpy as np


def load_calibration(directory):
    config = runpy.run_path(str(Path(directory) / 'config.py'))
    if not config.get('CALIBRATION_CONFIRMED'):
        raise ValueError('Original OpenMV calibration is not confirmed')
    width, height = int(config['IMAGE_WIDTH']), int(config['IMAGE_HEIGHT'])
    fx, fy, cx, cy = (float(x) for x in config['INTRINSICS'])
    matrix = np.array(((fx, 0., cx), (0., fy, cy), (0., 0., 1.)))
    distortion = np.asarray(config['DIST_COEFFS'], dtype=float)
    if (min(width, height) <= 0 or min(fx, fy) <= 0
            or distortion.shape != (5,) or not np.all(np.isfinite(matrix))
            or not np.all(np.isfinite(distortion))):
        raise ValueError('Invalid original OpenMV calibration')
    return (width, height), matrix, distortion


class CalibratedCamera:
    """Inverse-map output distorted pixels to the ideal Gazebo source image.

    Each OpenMV pixel is integrated from 2x2 subpixel samples. Keeping Gazebo's
    1280x960 render provides sufficient samples for the narrower 320x240 view;
    the actual detector and PnP still receive exactly IMAGE_WIDTH x HEIGHT.
    """
    SUPERSAMPLE = 2

    def __init__(self, size, matrix, distortion):
        self.size = tuple(size)
        self.matrix = np.asarray(matrix, dtype=float).reshape(3, 3)
        self.distortion = np.asarray(distortion, dtype=float).reshape(5)
        self.source = None
        self.maps = None

    def configure(self, width, height, matrix, distortion, model='plumb_bob'):
        matrix = np.asarray(matrix, dtype=float).reshape(3, 3)
        distortion = np.asarray(distortion, dtype=float)
        if (model not in ('', 'plumb_bob') or distortion.size not in (0, 4, 5)
                or not np.all(np.isfinite(matrix))
                or not np.all(np.isfinite(distortion))
                or min(width, height, matrix[0, 0], matrix[1, 1]) <= 0
                or not np.allclose(matrix[[0, 1, 2, 2, 2], [1, 0, 0, 1, 2]],
                                   (0., 0., 0., 0., 1.))):
            raise ValueError('Invalid Gazebo CameraInfo')
        if np.any(np.abs(distortion) > 1e-12):
            raise ValueError('OpenMV adapter expects the stock, distortion-free Gazebo camera')
        source = (int(width), int(height), tuple(matrix.flat))
        if source == self.source:
            return False
        self.source = self.maps = None
        w, h = self.size
        scale = self.SUPERSAMPLE
        # Pixel centers stay in the original OpenMV calibration coordinates.
        u, v = np.meshgrid((np.arange(w*scale)+.5)/scale-.5,
                           (np.arange(h*scale)+.5)/scale-.5)
        pixels = np.stack((u, v), axis=-1).reshape(-1, 1, 2)
        rays = cv2.undistortPointsIter(
            pixels, self.matrix, self.distortion, None, None,
            (cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 30, 1e-12))
        rays = np.column_stack((rays.reshape(-1, 2), np.ones(len(rays))))
        source_pixels = (rays @ matrix.T)[:, :2].reshape(h*scale, w*scale, 2)
        if (not np.all(np.isfinite(source_pixels))
                or np.any(source_pixels < 0.)
                or np.any(source_pixels[..., 0] > width-1)
                or np.any(source_pixels[..., 1] > height-1)):
            raise ValueError('Gazebo view does not cover the complete calibrated OpenMV view')
        self.maps = tuple(np.ascontiguousarray(source_pixels[..., i], dtype=np.float32)
                          for i in (0, 1))
        self.source = source
        return True

    def render(self, image):
        if self.source is None or self.maps is None:
            raise ValueError('Waiting for valid Gazebo CameraInfo')
        if image.shape != (self.source[1], self.source[0], 3) or image.dtype != np.uint8:
            raise ValueError('Gazebo image differs from its CameraInfo')
        sampled = cv2.remap(image, *self.maps, interpolation=cv2.INTER_LINEAR)
        return cv2.resize(sampled, self.size, interpolation=cv2.INTER_AREA)
