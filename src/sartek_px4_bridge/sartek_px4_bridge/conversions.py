"""Shared coordinate, quaternion, QoS, and PX4 mode helpers for sartek_px4_bridge."""

import math
from typing import Tuple

from px4_msgs.msg import VehicleStatus
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


# PX4 custom main/sub modes (from PX4-Autopilot/src/modules/commander/px4_custom_mode.h)
PX4_CUSTOM_MAIN_MODE_MANUAL = 1
PX4_CUSTOM_MAIN_MODE_ALTCTL = 2
PX4_CUSTOM_MAIN_MODE_POSCTL = 3
PX4_CUSTOM_MAIN_MODE_AUTO = 4
PX4_CUSTOM_MAIN_MODE_ACRO = 5
PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6
PX4_CUSTOM_MAIN_MODE_STABILIZED = 7

PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF = 2
PX4_CUSTOM_SUB_MODE_AUTO_LOITER = 3
PX4_CUSTOM_SUB_MODE_AUTO_MISSION = 4
PX4_CUSTOM_SUB_MODE_AUTO_RTL = 5
PX4_CUSTOM_SUB_MODE_AUTO_LAND = 6

_NAV_STATE_NAMES = {
    VehicleStatus.NAVIGATION_STATE_MANUAL: 'MANUAL',
    VehicleStatus.NAVIGATION_STATE_ALTCTL: 'ALTCTL',
    VehicleStatus.NAVIGATION_STATE_POSCTL: 'POSCTL',
    VehicleStatus.NAVIGATION_STATE_AUTO_MISSION: 'AUTO.MISSION',
    VehicleStatus.NAVIGATION_STATE_AUTO_LOITER: 'AUTO.LOITER',
    VehicleStatus.NAVIGATION_STATE_AUTO_RTL: 'AUTO.RTL',
    VehicleStatus.NAVIGATION_STATE_POSITION_SLOW: 'POSCTL.SLOW',
    VehicleStatus.NAVIGATION_STATE_GUIDED_COURSE: 'GUIDED_COURSE',
    VehicleStatus.NAVIGATION_STATE_ALTITUDE_CRUISE: 'ALTITUDE_CRUISE',
    VehicleStatus.NAVIGATION_STATE_ACRO: 'ACRO',
    VehicleStatus.NAVIGATION_STATE_DESCEND: 'DESCEND',
    VehicleStatus.NAVIGATION_STATE_TERMINATION: 'TERMINATION',
    VehicleStatus.NAVIGATION_STATE_OFFBOARD: 'OFFBOARD',
    VehicleStatus.NAVIGATION_STATE_STAB: 'STABILIZED',
    VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF: 'AUTO.TAKEOFF',
    VehicleStatus.NAVIGATION_STATE_AUTO_LAND: 'AUTO.LAND',
    VehicleStatus.NAVIGATION_STATE_AUTO_FOLLOW_TARGET: 'AUTO.FOLLOW_TARGET',
    VehicleStatus.NAVIGATION_STATE_AUTO_PRECLAND: 'AUTO.PRECLAND',
    VehicleStatus.NAVIGATION_STATE_ORBIT: 'ORBIT',
}

# Mode string -> (custom_main_mode, custom_sub_mode)
MODE_STRING_TO_PX4 = {
    'MANUAL': (PX4_CUSTOM_MAIN_MODE_MANUAL, 0),
    'ALTCTL': (PX4_CUSTOM_MAIN_MODE_ALTCTL, 0),
    'POSCTL': (PX4_CUSTOM_MAIN_MODE_POSCTL, 0),
    'ACRO': (PX4_CUSTOM_MAIN_MODE_ACRO, 0),
    'OFFBOARD': (PX4_CUSTOM_MAIN_MODE_OFFBOARD, 0),
    'STABILIZED': (PX4_CUSTOM_MAIN_MODE_STABILIZED, 0),
    'AUTO.TAKEOFF': (PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF),
    'TAKEOFF': (PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF),
    'AUTO.LOITER': (PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_LOITER),
    'LOITER': (PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_LOITER),
    'AUTO.MISSION': (PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_MISSION),
    'AUTO.RTL': (PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_RTL),
    'RTL': (PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_RTL),
    'AUTO.LAND': (PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_LAND),
    'LAND': (PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_LAND),
}


def px4_qos_profile(depth: int = 10) -> QoSProfile:
    """Return QoS profile compatible with PX4 uXRCE-DDS /fmu/* topics."""
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


def ned_to_enu(north: float, east: float, down: float) -> Tuple[float, float, float]:
    """Convert NED (North, East, Down) coordinates or velocities to ENU (East, North, Up)."""
    return float(east), float(north), float(-down)


def enu_to_ned(east: float, north: float, up: float) -> Tuple[float, float, float]:
    """Convert ENU (East, North, Up) coordinates or velocities to NED (North, East, Down)."""
    return float(north), float(east), float(-up)


def quaternion_to_euler_deg(q) -> Tuple[float, float, float]:
    """
    Convert Hamilton quaternion [w, x, y, z] (FRD body -> NED earth) to Euler angles (deg).

    Returns:
        (roll_deg, pitch_deg, yaw_deg) where:
        - roll_deg: right wing down positive [-180, 180]
        - pitch_deg: nose up positive [-90, 90]
        - yaw_deg: clockwise from North [0, 360)
    """
    w, x, y, z = float(q[0]), float(q[1]), float(q[2]), float(q[3])

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll_rad = math.atan2(sinr_cosp, cosr_cosp)

    sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch_rad = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw_rad = math.atan2(siny_cosp, cosy_cosp)

    roll_deg = math.degrees(roll_rad)
    pitch_deg = math.degrees(pitch_rad)
    yaw_deg = math.degrees(yaw_rad) % 360.0
    return roll_deg, pitch_deg, yaw_deg


def euler_deg_to_quaternion(
    roll_deg: float, pitch_deg: float, yaw_deg: float
) -> Tuple[float, float, float, float]:
    """
    Convert aerospace ZYX Euler angles (deg, FRD -> NED) to Hamilton quaternion (w, x, y, z).

    Exact inverse of quaternion_to_euler_deg:
    - roll_deg: right wing down positive
    - pitch_deg: nose up positive
    - yaw_deg: clockwise from North
    """
    half_r = math.radians(roll_deg) * 0.5
    half_p = math.radians(pitch_deg) * 0.5
    half_y = math.radians(yaw_deg) * 0.5

    cr, sr = math.cos(half_r), math.sin(half_r)
    cp, sp = math.cos(half_p), math.sin(half_p)
    cy, sy = math.cos(half_y), math.sin(half_y)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return float(w), float(x), float(y), float(z)


def nav_state_to_str(nav_state: int) -> str:
    """Map PX4 VehicleStatus.nav_state enum value to a human-readable mode string."""
    return _NAV_STATE_NAMES.get(int(nav_state), f'UNKNOWN({int(nav_state)})')
