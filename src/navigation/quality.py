"""Navigation goal validation."""

import math


def planar_pose(values):
    if not isinstance(values, (list, tuple)) or len(values) != 3:
        raise ValueError("pose must be [x, y, yaw]")
    if any(
        isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v)
        for v in values
    ):
        raise ValueError("pose values must be finite numbers")
    x, y, yaw = map(float, values)
    return [x, y, math.atan2(math.sin(yaw), math.cos(yaw))]
