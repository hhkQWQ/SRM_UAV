"""Time-aligned Gazebo truth for the stock PX4 X500 down-facing camera.

The camera's pixel coordinates are optical (right, down, forward). Gazebo's
camera link is FLU; PX4 body coordinates are FRD and its local frame is NED.
No values from this module are detection inputs, and it never publishes.

``GroundTruth(connect=False)`` and ``compute_truth`` require only the standard
library, allowing the geometry and interpolation to be checked without Gazebo.
``truth_dt_us`` means selected truth time minus requested image time; it is
zero for interpolation and signed for a nearest-sample result.
"""

from bisect import bisect_left
from collections import deque
import math
import threading
from typing import NamedTuple


CAMERA_PITCH = 1.5707
BASE_IN_MODEL = (0.0, 0.0, 0.24)
CAMERA_IN_MODEL = (0.0, 0.0, 0.10)
ENU_TO_NED = ((0.0, 1.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, -1.0))
FRD_TO_FLU = ((1.0, 0.0, 0.0), (0.0, -1.0, 0.0), (0.0, 0.0, -1.0))
OPTICAL_TO_SENSOR = ((0.0, 0.0, 1.0), (-1.0, 0.0, 0.0), (0.0, -1.0, 0.0))


class PoseSample(NamedTuple):
    time_us: int
    position: tuple
    quaternion: tuple
    source: str = "gazebo"


def _vector(value, length):
    value = tuple(float(v) for v in value)
    if len(value) != length or not all(math.isfinite(v) for v in value):
        raise ValueError("invalid finite vector")
    return value


def normalize_quaternion(value):
    """Normalize a quaternion in w,x,y,z order."""
    value = _vector(value, 4)
    length = math.sqrt(sum(v * v for v in value))
    if length < 1e-12:
        raise ValueError("zero quaternion")
    return tuple(v / length for v in value)


def quaternion_slerp(first, second, fraction):
    """Shortest-arc SLERP, with normalized-linear interpolation near zero."""
    first = normalize_quaternion(first)
    second = normalize_quaternion(second)
    fraction = max(0.0, min(1.0, float(fraction)))
    cosine = sum(a * b for a, b in zip(first, second))
    if cosine < 0.0:
        second = tuple(-v for v in second)
        cosine = -cosine
    cosine = max(-1.0, min(1.0, cosine))
    if cosine > 0.9995:
        return normalize_quaternion(tuple(
            a + fraction * (b - a) for a, b in zip(first, second)))
    angle = math.acos(cosine)
    denominator = math.sin(angle)
    left = math.sin((1.0 - fraction) * angle) / denominator
    right = math.sin(fraction * angle) / denominator
    return normalize_quaternion(tuple(
        left * a + right * b for a, b in zip(first, second)))


def quaternion_rotation(quaternion):
    w, x, y, z = normalize_quaternion(quaternion)
    return (
        (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)),
        (2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)),
        (2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)),
    )


def rotation_quaternion(rotation):
    """Convert a proper 3x3 rotation matrix to normalized w,x,y,z."""
    r = rotation
    trace = r[0][0] + r[1][1] + r[2][2]
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        q = (0.25 * scale, (r[2][1] - r[1][2]) / scale,
             (r[0][2] - r[2][0]) / scale, (r[1][0] - r[0][1]) / scale)
    elif r[0][0] > r[1][1] and r[0][0] > r[2][2]:
        scale = math.sqrt(max(0.0, 1.0 + r[0][0] - r[1][1] - r[2][2])) * 2.0
        q = ((r[2][1] - r[1][2]) / scale, 0.25 * scale,
             (r[0][1] + r[1][0]) / scale, (r[0][2] + r[2][0]) / scale)
    elif r[1][1] > r[2][2]:
        scale = math.sqrt(max(0.0, 1.0 + r[1][1] - r[0][0] - r[2][2])) * 2.0
        q = ((r[0][2] - r[2][0]) / scale, (r[0][1] + r[1][0]) / scale,
             0.25 * scale, (r[1][2] + r[2][1]) / scale)
    else:
        scale = math.sqrt(max(0.0, 1.0 + r[2][2] - r[0][0] - r[1][1])) * 2.0
        q = ((r[1][0] - r[0][1]) / scale, (r[0][2] + r[2][0]) / scale,
             (r[1][2] + r[2][1]) / scale, 0.25 * scale)
    return normalize_quaternion(q)


def _mv(matrix, vector):
    return tuple(sum(a * b for a, b in zip(row, vector)) for row in matrix)


def _transpose(matrix):
    return tuple(tuple(row[column] for row in matrix) for column in range(3))


def _mm(first, second):
    return tuple(tuple(sum(first[i][k] * second[k][j] for k in range(3))
                       for j in range(3)) for i in range(3))


def _plus(first, second):
    return tuple(a + b for a, b in zip(first, second))


def _minus(first, second):
    return tuple(a - b for a, b in zip(first, second))


def camera_extrinsics():
    """Return stock optical-to-base FRD rotation and translation."""
    s, c = math.sin(CAMERA_PITCH), math.cos(CAMERA_PITCH)
    return (((0.0, -s, c), (1.0, 0.0, 0.0), (0.0, c, s)),
            (0.0, 0.0, 0.14))


def compute_truth(model_position, model_quaternion, target_position=(5.0, 0.0, 1.5),
                  velocity_enu=None):
    """Compute optical and body-relative truth from a top-level model pose.

    Model translation must be the world pose of x500_mono_cam_down_0, not the
    local pose of camera_link or base_link from SceneBroadcaster's Pose_V.
    ``velocity_enu``, when provided, is the velocity of the base reference point.
    """
    model_position = _vector(model_position, 3)
    target_position = _vector(target_position, 3)
    r_wm = quaternion_rotation(model_quaternion)
    s, c = math.sin(CAMERA_PITCH), math.cos(CAMERA_PITCH)
    r_mount = ((c, 0.0, s), (0.0, 1.0, 0.0), (-s, 0.0, c))
    r_wc = _mm(_mm(r_wm, r_mount), OPTICAL_TO_SENSOR)
    body_world = _plus(model_position, _mv(r_wm, BASE_IN_MODEL))
    camera_world = _plus(model_position, _mv(r_wm, CAMERA_IN_MODEL))
    r_nb = _mm(_mm(ENU_TO_NED, r_wm), FRD_TO_FLU)
    return {
        "p_camera_true": _mv(_transpose(r_wc), _minus(target_position, camera_world)),
        "relative_ned_true": _mv(ENU_TO_NED, _minus(target_position, body_world)),
        "body_world": body_world,
        "camera_world": camera_world,
        "target_world": target_position,
        "q_nb": rotation_quaternion(r_nb),
        "velocity_ned": (None if velocity_enu is None else
                         _mv(ENU_TO_NED, _vector(velocity_enu, 3))),
    }


def _body_position(sample):
    return _plus(sample.position, _mv(quaternion_rotation(sample.quaternion), BASE_IN_MODEL))


def _stamp_us(stamp):
    return int(stamp.sec) * 1_000_000 + int(stamp.nsec) // 1000


class GroundTruth:
    """Read-only, thread-safe history of Gazebo model poses and sim clock.

    Nearest fallback is limited to ``max_nearest_us``. Interpolation requires
    bracketing poses separated by at most 100 ms, to avoid bridging a data gap.
    Static target poses have no age limit; a missing target uses the explicitly
    declared map fallback. A backwards simulation clock starts a new generation.
    """

    MAX_INTERPOLATION_GAP_US = 100_000
    MAX_VELOCITY_SPAN_US = 150_000

    def __init__(self, world="pnp_pillar", model="x500_mono_cam_down_0",
                 target="pnp_target_pillar", target_fallback=(5.0, 0.0, 1.5),
                 *, connect=True):
        self.world = world
        self.model = model
        self.target = target
        self.target_fallback = _vector(target_fallback, 3)
        self._lock = threading.RLock()
        self._poses = deque(maxlen=600)
        self._target_pose = None
        self._clock_us = None
        self._generation = 0
        self._closed = False
        self._node = None
        self._topics = []
        if connect:
            # Importing this file, and connect=False, do not create sockets or
            # depend on Gazebo's binary Python packages.
            from gz.transport13 import Node
            from gz.msgs10.pose_v_pb2 import Pose_V
            from gz.msgs10.clock_pb2 import Clock
            self._node = Node()
            try:
                for suffix in ("pose/info", "dynamic_pose/info"):
                    topic = f"/world/{world}/{suffix}"
                    callback = lambda message, name=suffix: self._on_pose_message(message, name)
                    if not self._node.subscribe(Pose_V, topic, callback):
                        raise RuntimeError(f"Could not subscribe to {topic}")
                    self._topics.append(topic)
                topic = f"/world/{world}/clock"
                if not self._node.subscribe(Clock, topic, self._on_clock_message):
                    raise RuntimeError(f"Could not subscribe to {topic}")
                self._topics.append(topic)
            except Exception:
                self.close()
                raise

    @property
    def generation(self):
        with self._lock:
            return self._generation

    def now_us(self):
        with self._lock:
            return 0 if self._clock_us is None else self._clock_us

    def _reset_locked(self):
        self._poses.clear()
        self._target_pose = None
        self._generation += 1

    def add_clock_sample(self, time_us):
        """Ingest a simulation clock; also usable in offline verification."""
        time_us = int(time_us)
        with self._lock:
            if self._closed:
                return
            if self._clock_us is not None and time_us < self._clock_us:
                self._reset_locked()
            self._clock_us = time_us

    def _on_clock_message(self, message):
        self.add_clock_sample(_stamp_us(message.sim))

    def _on_pose_message(self, message, source="pose/info"):
        time_us = _stamp_us(message.header.stamp)
        for pose in message.pose:
            if pose.name not in (self.model, self.target):
                continue
            position = (pose.position.x, pose.position.y, pose.position.z)
            quaternion = (pose.orientation.w, pose.orientation.x,
                          pose.orientation.y, pose.orientation.z)
            try:
                if pose.name == self.model:
                    self.add_pose_sample(time_us, position, quaternion, source=source)
                else:
                    self.add_target_sample(time_us, position, quaternion, source=source)
            except (ValueError, TypeError):
                # One malformed entry must not kill the transport thread.
                continue

    def _time_plausible_locked(self, time_us):
        if self._clock_us is None:
            return True
        # Pose and clock callbacks are on separate streams. Permit ordinary
        # callback reordering, but reject late packets from the previous epoch.
        return self._clock_us - 2_000_000 <= time_us <= self._clock_us + 250_000

    def add_pose_sample(self, time_us, position, quaternion, *, source="offline"):
        """Ingest one top-level world pose; duplicate timestamps are replaced."""
        sample = PoseSample(int(time_us), _vector(position, 3),
                            normalize_quaternion(quaternion), source)
        with self._lock:
            if self._closed or not self._time_plausible_locked(sample.time_us):
                return
            if (self._clock_us is None and self._poses and
                    sample.time_us < self._poses[-1].time_us - 250_000):
                self._reset_locked()
            rows = list(self._poses)
            times = [row.time_us for row in rows]
            index = bisect_left(times, sample.time_us)
            if index < len(rows) and rows[index].time_us == sample.time_us:
                rows[index] = sample
            else:
                rows.insert(index, sample)
            self._poses.clear()
            self._poses.extend(rows[-self._poses.maxlen:])

    def add_target_sample(self, time_us, position, quaternion=(1.0, 0.0, 0.0, 0.0),
                          *, source="offline"):
        """Ingest the static target model origin (the light-strip top plane)."""
        sample = PoseSample(int(time_us), _vector(position, 3),
                            normalize_quaternion(quaternion), source)
        with self._lock:
            if self._closed or not self._time_plausible_locked(sample.time_us):
                return
            if self._target_pose is None or sample.time_us >= self._target_pose.time_us:
                self._target_pose = sample

    def sample(self, image_us, max_nearest_us=20_000):
        """Return time-aligned truth, or None when no sufficiently close pose exists."""
        image_us = int(image_us)
        max_nearest_us = max(0, int(max_nearest_us))
        with self._lock:
            rows = list(self._poses)
            target = self._target_pose
            generation = self._generation
        if not rows:
            return None
        times = [row.time_us for row in rows]
        index = bisect_left(times, image_us)
        pair = None
        if index < len(rows) and rows[index].time_us == image_us:
            chosen = rows[index]
            mode, time_difference = "exact", 0
        elif (0 < index < len(rows) and
              rows[index].time_us - rows[index - 1].time_us <= self.MAX_INTERPOLATION_GAP_US):
            left, right = rows[index - 1], rows[index]
            fraction = (image_us - left.time_us) / (right.time_us - left.time_us)
            chosen = PoseSample(image_us,
                tuple(a + fraction * (b - a) for a, b in zip(left.position, right.position)),
                quaternion_slerp(left.quaternion, right.quaternion, fraction),
                "interpolated")
            pair = (left, right)
            mode, time_difference = "interpolated", 0
        else:
            candidates = [i for i in (index - 1, index) if 0 <= i < len(rows)]
            index = min(candidates, key=lambda i: abs(rows[i].time_us - image_us))
            chosen = rows[index]
            time_difference = chosen.time_us - image_us
            if abs(time_difference) > max_nearest_us:
                return None
            mode = "nearest"
        if pair is None and len(rows) >= 2:
            left_index, right_index = max(0, index - 1), min(len(rows) - 1, index + 1)
            if left_index != right_index:
                pair = (rows[left_index], rows[right_index])
        velocity_enu = None
        if pair is not None:
            first, second = pair
            duration_us = second.time_us - first.time_us
            if 0 < duration_us <= self.MAX_VELOCITY_SPAN_US:
                difference = _minus(_body_position(second), _body_position(first))
                velocity_enu = tuple(v * 1_000_000.0 / duration_us for v in difference)
        target_position = target.position if target is not None else self.target_fallback
        result = compute_truth(chosen.position, chosen.quaternion, target_position, velocity_enu)
        target_source = "gazebo_target_pose" if target is not None else "map_fallback"
        result.update({
            "truth_dt_us": time_difference,
            "source": f"gazebo_pose_{mode};target={target_source}",
            "target_source": target_source,
            "sample_time_us": chosen.time_us,
            "image_time_us": image_us,
            "generation": generation,
        })
        return result

    def close(self):
        """Unsubscribe from all native Gazebo topics; safe to call repeatedly."""
        with self._lock:
            if self._closed:
                return
            self._closed = True
            node, topics = self._node, tuple(self._topics)
            self._node = None
            self._topics.clear()
        if node is not None:
            for topic in topics:
                try:
                    node.unsubscribe(topic)
                except Exception:
                    pass

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()
