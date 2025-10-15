import math
from typing import Tuple


def wrap_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    wrapped = (angle + math.pi) % (2.0 * math.pi)
    if wrapped < 0.0:
        wrapped += 2.0 * math.pi
    return wrapped - math.pi


def clamp(value: float, minimum: float, maximum: float) -> float:
    """Clamp value to [minimum, maximum]."""
    return max(minimum, min(maximum, value))


def quat_to_euler_xyz(x: float, y: float, z: float, w: float) -> Tuple[float, float, float]:
    """Return roll, pitch, yaw from a quaternion using the XYZ convention."""
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = 1.0 if t2 > 1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)

    return roll, pitch, yaw

