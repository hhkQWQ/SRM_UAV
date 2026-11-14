"""OpenCV adapter for the OpenMV v4.5 image API used by PnP/armor_detect.py.

This supplies image/blob measurements, not a replacement armor detector. The
original light filtering, color selection, pairing, scoring, deduplication and
four-endpoint ordering run unchanged in armor_detect.py.

The adapter is deliberately a *subset*, not an OpenMV emulator. Color conversion
uses OpenCV CIE Lab (L 0..100, signed a/b), components use 8-connectivity, rectangle
axes use cv2.minAreaRect with integer corner/midpoint rounding, and blob merge
uses bounding-box proximity. RGB565 can
be quantized before processing. These differ from OpenMV's integer lookup tables,
flood-fill details, axis rounding and sensor/exposure behavior; desktop results
must not be presented as bit-identical hardware detection or hardware timing.

OpenMV v4.5.9 API reference:
https://docs.openmv.io/v4.5.9/library/omv.image.html
"""

from __future__ import annotations

import math
import types
from dataclasses import dataclass

import cv2
import numpy as np


def lab_to_rgb(lab):
    """Return 8-bit RGB from OpenMV-style Lab channel ranges."""
    value = np.asarray(lab, dtype=np.float32).reshape(1, 1, 3)
    rgb = cv2.cvtColor(value, cv2.COLOR_LAB2RGB)[0, 0]
    return tuple(int(v) for v in np.rint(np.clip(rgb, 0., 1.) * 255.))


def make_image_module(name="image"):
    """Create the small module object needed by an isolated original-code loader."""
    module = types.ModuleType(name)
    module.lab_to_rgb = lab_to_rgb
    module.Image = ImageAdapter
    return module


def _threshold_mask(lab, threshold):
    if len(threshold) != 6:
        raise ValueError("Only six-channel RGB Lab thresholds are supported")
    bounds = np.asarray(threshold, dtype=np.float32)
    if np.any(~np.isfinite(bounds)):
        raise ValueError("Non-finite Lab threshold")
    lower, upper = bounds[[0, 2, 4]], bounds[[1, 3, 5]]
    if np.any(lower > upper):
        raise ValueError("Reversed Lab threshold bounds")
    return cv2.inRange(lab, lower, upper)


@dataclass
class StatisticsAdapter:
    mean: tuple[float, float, float]

    def l_mean(self):
        return self.mean[0]

    def a_mean(self):
        return self.mean[1]

    def b_mean(self):
        return self.mean[2]


class BlobAdapter:
    """A mask plus the blob methods used by the existing detector.

    Pixel-center rectangles and integer edge midpoints follow the v4.5.9 API
    convention. OpenCV's continuous minimum-area rectangle replaces OpenMV's
    sampled-angle corner search, so thin/rotated blob endpoints can still differ
    by a pixel. The original PnP receives those measured endpoints unchanged.
    """

    def __init__(self, x, y, mask, code):
        self._x, self._y = int(x), int(y)
        self._mask = np.ascontiguousarray(mask, dtype=np.uint8)
        self._code = int(code)
        self._pixels = int(cv2.countNonZero(self._mask))
        self._moments = None
        self._rect = None

    def x(self):
        return self._x

    def y(self):
        return self._y

    def w(self):
        return int(self._mask.shape[1])

    def h(self):
        return int(self._mask.shape[0])

    def rect(self):
        return self.x(), self.y(), self.w(), self.h()

    def pixels(self):
        return self._pixels

    def code(self):
        return self._code

    def _statistics(self):
        if self._moments is None:
            m = cv2.moments(self._mask, binaryImage=True)
            n = max(float(m['m00']), 1.)
            cx, cy = m['m10'] / n, m['m01'] / n
            dx, dy = math.floor(cx + .5) - cx, math.floor(cy + .5) - cy
            # OpenMV v4.5.9 accumulates central moments about a rounded centroid.
            a = m['mu20'] + n * dx * dx
            b = m['mu11'] + n * dx * dy
            c = m['mu02'] + n * dy * dy
            angle = (.5 * math.atan2(2. * b, a - c) if a != c else 0.) % math.pi
            # Algebraic simplification of v4.5.9 blob.c calc_roundness.
            spread, trace = math.hypot(a - c, b), a + c
            ratio = (trace - spread) / (trace + spread) if trace + spread else 1.
            self._moments = (self._x + cx, self._y + cy,
                             angle, min(1., max(0., ratio)))
        return self._moments

    def cx(self):
        return int(math.floor(self._statistics()[0] + .5))

    def cy(self):
        return int(math.floor(self._statistics()[1] + .5))

    def rotation(self):
        return self._statistics()[2]

    def elongation(self):
        return 1. - self._statistics()[3]

    def _rectangle(self):
        if self._rect is None:
            contours, _ = cv2.findContours(self._mask, cv2.RETR_EXTERNAL,
                                           cv2.CHAIN_APPROX_SIMPLE)
            if not contours:
                raise ValueError('Empty blob')
            centers = np.concatenate(contours).reshape(-1, 2).astype(np.float32)
            centers += (self._x, self._y)
            rectangle = cv2.minAreaRect(centers)
            corners = np.floor(cv2.boxPoints(rectangle) + .5).astype(np.int64)
            midpoints = np.trunc((corners + np.roll(corners, -1, axis=0)) / 2).astype(int)
            first = (midpoints[0], midpoints[2])
            second = (midpoints[1], midpoints[3])
            first_length = np.linalg.norm(first[1] - first[0])
            second_length = np.linalg.norm(second[1] - second[0])
            major, minor = (first, second) if first_length >= second_length else (second, first)
            area = abs(float(np.dot(corners[:, 0], np.roll(corners[:, 1], -1)) -
                             np.dot(corners[:, 1], np.roll(corners[:, 0], -1)))) / 2.
            self._rect = (tuple(int(v) for point in major for v in point),
                          tuple(int(v) for point in minor for v in point),
                          max(area, 1.))
        return self._rect

    def major_axis_line(self):
        return self._rectangle()[0]

    def minor_axis_line(self):
        return self._rectangle()[1]

    def solidity(self):
        # OpenMV v4.5.9 uses minimum-area-rectangle fill for this method.
        return min(1., self.pixels() / self._rectangle()[2])

    def near(self, other, margin):
        return (self.x() <= other.x() + other.w() + margin
                and other.x() <= self.x() + self.w() + margin
                and self.y() <= other.y() + other.h() + margin
                and other.y() <= self.y() + self.h() + margin)

    def merged(self, other):
        x, y = min(self.x(), other.x()), min(self.y(), other.y())
        width = max(self.x() + self.w(), other.x() + other.w()) - x
        height = max(self.y() + self.h(), other.y() + other.h()) - y
        mask = np.zeros((height, width), np.uint8)
        for blob in (self, other):
            x0, y0 = blob.x() - x, blob.y() - y
            destination = mask[y0:y0 + blob.h(), x0:x0 + blob.w()]
            cv2.bitwise_or(destination, blob._mask, dst=destination)
        return BlobAdapter(x, y, mask, self.code() | other.code())


class ImageAdapter:
    """Wrap a uint8 BGR image without changing its resolution.

    ``bgr`` exposes the working image, including any debug drawings. No global
    ``image`` module is installed automatically; use make_image_module() in the
    original-code loader so module-name overrides have an explicit lifetime.
    """

    def __init__(self, bgr, rgb565=True):
        array = np.asarray(bgr)
        if array.dtype != np.uint8 or array.ndim != 3 or array.shape[2] != 3:
            raise ValueError('Expected uint8 BGR image with shape (H,W,3)')
        self.bgr = np.array(array, copy=True, order='C')
        if rgb565:
            # BGR channel order: blue/red 5-bit and green 6-bit, bit replication.
            for channel in (0, 2):
                values = self.bgr[:, :, channel]
                values &= np.uint8(0xf8)
                values |= values >> np.uint8(5)
            values = self.bgr[:, :, 1]
            values &= np.uint8(0xfc)
            values |= values >> np.uint8(6)
        self._lab = None

    def width(self):
        return int(self.bgr.shape[1])

    def height(self):
        return int(self.bgr.shape[0])

    @property
    def lab(self):
        if self._lab is None:
            self._lab = cv2.cvtColor(self.bgr.astype(np.float32) / 255.,
                                     cv2.COLOR_BGR2LAB)
        return self._lab

    def find_blobs(self, thresholds, *, x_stride=1, y_stride=1,
                   area_threshold=1, pixels_threshold=1, merge=False, margin=0):
        if x_stride != 1 or y_stride != 1:
            raise ValueError('Only full-resolution x_stride=y_stride=1 is supported')
        if margin < 0 or area_threshold < 0 or pixels_threshold < 0:
            raise ValueError('Negative blob limits')
        blobs = []
        for index, threshold in enumerate(thresholds):
            mask = _threshold_mask(self.lab, threshold)
            count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
            for label in range(1, count):
                x, y, width, height, pixels = (int(v) for v in stats[label])
                if pixels < pixels_threshold or width * height < area_threshold:
                    continue
                component = (labels[y:y + height, x:x + width] == label).astype(np.uint8)
                blobs.append(BlobAdapter(x, y, component, 1 << index))
        if merge:
            # Re-scan after every union to include transitive bounding-box merges.
            changed = True
            while changed:
                changed = False
                for i in range(len(blobs)):
                    for j in range(i + 1, len(blobs)):
                        if blobs[i].near(blobs[j], margin):
                            blobs[i] = blobs[i].merged(blobs[j])
                            blobs.pop(j)
                            changed = True
                            break
                    if changed:
                        break
        blobs.sort(key=lambda blob: (blob.y(), blob.x(), blob.code()))
        return blobs

    def get_statistics(self, *, thresholds=None, roi=None):
        if roi is None:
            x, y, width, height = 0, 0, self.width(), self.height()
        else:
            x, y, width, height = (int(v) for v in roi)
        x0, y0 = max(0, x), max(0, y)
        x1, y1 = min(self.width(), x + width), min(self.height(), y + height)
        if x1 <= x0 or y1 <= y0:
            return StatisticsAdapter((0., 0., 0.))
        lab = self.lab[y0:y1, x0:x1]
        mask = None
        if thresholds:
            mask = np.zeros(lab.shape[:2], np.uint8)
            for threshold in thresholds:
                cv2.bitwise_or(mask, _threshold_mask(lab, threshold), dst=mask)
        return StatisticsAdapter(tuple(float(v) for v in cv2.mean(lab, mask=mask)[:3]))

    def draw_rectangle(self, rect, *, color=(255, 255, 255), thickness=1):
        x, y, width, height = (int(round(v)) for v in rect)
        cv2.rectangle(self.bgr, (x, y), (x + width - 1, y + height - 1),
                      tuple(int(v) for v in reversed(color)), int(thickness))
        return self

    def draw_line(self, *coordinates, color=(255, 255, 255), thickness=1):
        line = coordinates[0] if len(coordinates) == 1 else coordinates
        if len(line) != 4:
            raise ValueError('Expected a four-coordinate line')
        x1, y1, x2, y2 = (int(round(v)) for v in line)
        cv2.line(self.bgr, (x1, y1), (x2, y2),
                 tuple(int(v) for v in reversed(color)), int(thickness))
        return self
