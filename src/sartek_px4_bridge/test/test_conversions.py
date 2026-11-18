"""Unit tests for ENU/NED and quaternion/Euler conversion helpers."""

import math
import pytest

from sartek_px4_bridge.conversions import (
    enu_to_ned,
    euler_deg_to_quaternion,
    nav_state_to_str,
    ned_to_enu,
    quaternion_to_euler_deg,
)
from px4_msgs.msg import VehicleStatus


def test_ned_enu_roundtrip():
    east, north, up = ned_to_enu(120.0, -45.0, -80.0)
    assert (east, north, up) == (-45.0, 120.0, 80.0)
    assert enu_to_ned(east, north, up) == (120.0, -45.0, -80.0)


@pytest.mark.parametrize(
    'roll,pitch,yaw',
    [
        (0.0, 0.0, 0.0),
        (15.0, -25.0, 90.0),
        (-35.0, 12.0, 245.0),
        (45.0, -45.0, 359.0),
    ],
)
def test_euler_quaternion_roundtrip(roll, pitch, yaw):
    q = euler_deg_to_quaternion(roll, pitch, yaw)
    r_out, p_out, y_out = quaternion_to_euler_deg(q)
    assert math.isclose(r_out, roll, abs_tol=1e-4)
    assert math.isclose(p_out, pitch, abs_tol=1e-4)
    yaw_diff = abs((y_out - yaw + 180.0) % 360.0 - 180.0)
    assert yaw_diff < 1e-4


def test_nav_state_mapping():
    assert nav_state_to_str(VehicleStatus.NAVIGATION_STATE_OFFBOARD) == 'OFFBOARD'
    assert nav_state_to_str(VehicleStatus.NAVIGATION_STATE_AUTO_LOITER) == 'AUTO.LOITER'
    assert nav_state_to_str(VehicleStatus.NAVIGATION_STATE_AUTO_RTL) == 'AUTO.RTL'
