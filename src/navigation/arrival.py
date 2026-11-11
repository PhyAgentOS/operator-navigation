"""Verify Nav2 success against the requested pose and independent motion evidence."""

import math

from navigation.quality import planar_pose


def angle_error(a, b):
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


def check_arrival(
    target, start_pose, current_pose, start_odom, current_odom, *, wheel_motion, require_motion=True
):
    target, start_pose, current_pose, start_odom, current_odom = map(
        planar_pose, [target, start_pose, current_pose, start_odom, current_odom]
    )
    distance = math.hypot(target[0] - current_pose[0], target[1] - current_pose[1])
    heading = angle_error(target[2], current_pose[2])
    displacement = math.hypot(current_odom[0] - start_odom[0], current_odom[1] - start_odom[1])
    turn = angle_error(current_odom[2], start_odom[2])
    start_distance = math.hypot(target[0] - start_pose[0], target[1] - start_pose[1])
    reason = "已到达目标位置和朝向"
    verified = True
    if distance > 0.15 or heading > 0.18:
        verified, reason = False, "Nav2 返回到达，但实际定位仍偏离请求的目标，请重新设点"
    elif require_motion and start_distance > 0.15 and (not wheel_motion or displacement < 0.03):
        verified, reason = False, "Nav2 返回到达，但未确认底盘实际位移，请检查定位"
    elif (
        require_motion
        and angle_error(target[2], start_pose[2]) > 0.25
        and (not wheel_motion or turn < 0.03)
    ):
        verified, reason = False, "Nav2 返回到达，但未确认底盘实际转向，请检查定位"
    return {
        "verified": verified,
        "reason": reason,
        "position_error_m": distance,
        "heading_error_rad": heading,
        "odometry_displacement_m": displacement,
        "odometry_turn_rad": turn,
        "wheel_motion_observed": wheel_motion,
    }
