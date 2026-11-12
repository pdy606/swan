"""Timestamped single-camera contexts and planar odometry, without ML imports."""
from collections import deque
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Pose2D:
    stamp: float
    x: float
    y: float
    yaw: float


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_yaw(x, y, z, w):
    if not all(math.isfinite(value) for value in (x, y, z, w)):
        raise ValueError('Nonfinite odometry orientation')
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    if norm < 1e-8:
        raise ValueError('Zero odometry quaternion')
    x, y, z, w = (value / norm for value in (x, y, z, w))
    return math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))


class FrameBuffer:
    def __init__(self, capacity=40):
        self.frames = deque(maxlen=capacity)

    def clear(self):
        self.frames.clear()

    def append(self, frame):
        stamp = frame['stamp']
        if not math.isfinite(stamp) or stamp < 0:
            raise ValueError('Invalid camera timestamp')
        if self.frames and stamp <= self.frames[-1]['stamp']:
            return False
        self.frames.append(frame)
        return True

    def context(self, count=6, interval=0.2, tolerance=0.075):
        if len(self.frames) < count:
            return None
        newest = self.frames[-1]['stamp']
        selected = []
        for index in range(count):
            target = newest - (count - 1 - index) * interval
            frame = min(self.frames, key=lambda candidate: abs(candidate['stamp'] - target))
            if abs(frame['stamp'] - target) > tolerance:
                return None
            if selected and frame['stamp'] <= selected[-1]['stamp']:
                return None
            selected.append(frame)
        return selected


class OdomBuffer:
    def __init__(self, capacity=180):
        self.poses = deque(maxlen=capacity)

    def clear(self):
        self.poses.clear()

    def append(self, pose):
        if not all(math.isfinite(value) for value in (pose.stamp, pose.x, pose.y, pose.yaw)):
            raise ValueError('Nonfinite odometry')
        if pose.stamp < 0:
            raise ValueError('Negative odometry timestamp')
        if self.poses and pose.stamp <= self.poses[-1].stamp:
            return False
        self.poses.append(pose)
        return True

    def pose_at(self, stamp, max_skew=0.15):
        if not self.poses:
            return None
        nearest = min(self.poses, key=lambda pose: abs(pose.stamp - stamp))
        if abs(nearest.stamp - stamp) > max_skew:
            return None
        for before, after in zip(self.poses, list(self.poses)[1:]):
            if before.stamp <= stamp <= after.stamp:
                if after.stamp - before.stamp > 2 * max_skew:
                    return None
                fraction = (stamp - before.stamp) / (after.stamp - before.stamp)
                return Pose2D(stamp, before.x + fraction * (after.x - before.x),
                              before.y + fraction * (after.y - before.y),
                              wrap_angle(before.yaw + fraction * wrap_angle(after.yaw - before.yaw)))
        return nearest
