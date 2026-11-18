#!/usr/bin/env python3
"""
PX4 Telemetry Bridge — Micro XRCE-DDS -> sartek_msgs.

Subscribes to PX4 /fmu/out/* topics (NED / FRD / uORB) and publishes unified,
ENU-converted `sartek_msgs/TelemetryStatus` and `sartek_msgs/SystemHealth`
messages so mission nodes never need to depend on `px4_msgs` directly.
"""

import math
import time

from px4_msgs.msg import (
    AirspeedValidated,
    BatteryStatus,
    FailsafeFlags,
    HomePosition,
    SensorCombined,
    VehicleAttitude,
    VehicleGlobalPosition,
    VehicleGnss,
    VehicleLocalPosition,
    VehicleStatus,
)
import rclpy
from rclpy.node import Node
from sartek_msgs.msg import SystemHealth, TelemetryStatus

from sartek_px4_bridge.conversions import (
    nav_state_to_str,
    ned_to_enu,
    px4_qos_profile,
    quaternion_to_euler_deg,
)


class Px4TelemetryBridge(Node):
    """Aggregate PX4 uXRCE-DDS telemetry into TelemetryStatus and SystemHealth."""

    def __init__(self) -> None:
        super().__init__('px4_telemetry_bridge')

        self.declare_parameter('fmu_prefix', '')
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('connection_timeout_s', 2.0)
        self.declare_parameter('stall_speed_ms', 12.0)
        self.declare_parameter('battery_critical_pct', 15.0)
        self.declare_parameter('battery_critical_volt', 14.0)
        self.declare_parameter('gps_min_satellites', 6)
        self.declare_parameter('imu_vibration_critical', 60.0)

        fmu_prefix = self.get_parameter('fmu_prefix').value.rstrip('/')
        rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self._conn_timeout_s = float(self.get_parameter('connection_timeout_s').value)
        self._stall_speed = float(self.get_parameter('stall_speed_ms').value)
        self._bat_crit_pct = float(self.get_parameter('battery_critical_pct').value)
        self._bat_crit_volt = float(self.get_parameter('battery_critical_volt').value)
        self._gps_min_sats = int(self.get_parameter('gps_min_satellites').value)
        self._imu_vib_crit = float(self.get_parameter('imu_vibration_critical').value)

        qos = px4_qos_profile(depth=10)

        def fmu_topic(suffix: str) -> str:
            return f'{fmu_prefix}/fmu/out/{suffix}' if fmu_prefix else f'/fmu/out/{suffix}'

        # PX4 /fmu/out/* Subscribers
        self.create_subscription(
            VehicleLocalPosition,
            fmu_topic('vehicle_local_position_v1'),
            self._local_pos_cb,
            qos,
        )
        self.create_subscription(
            VehicleAttitude,
            fmu_topic('vehicle_attitude'),
            self._attitude_cb,
            qos,
        )
        self.create_subscription(
            VehicleGlobalPosition,
            fmu_topic('vehicle_global_position'),
            self._global_pos_cb,
            qos,
        )
        self.create_subscription(
            AirspeedValidated,
            fmu_topic('airspeed_validated_v1'),
            self._airspeed_cb,
            qos,
        )
        self.create_subscription(
            BatteryStatus,
            fmu_topic('battery_status_v1'),
            self._battery_cb,
            qos,
        )
        self.create_subscription(
            VehicleStatus,
            fmu_topic('vehicle_status_v5'),
            self._status_cb,
            qos,
        )
        self.create_subscription(
            HomePosition,
            fmu_topic('home_position_v2'),
            self._home_cb,
            qos,
        )
        self.create_subscription(
            VehicleGnss,
            fmu_topic('vehicle_gnss'),
            self._gnss_cb,
            qos,
        )
        self.create_subscription(
            SensorCombined,
            fmu_topic('sensor_combined'),
            self._sensor_cb,
            qos,
        )
        self.create_subscription(
            FailsafeFlags,
            fmu_topic('failsafe_flags'),
            self._failsafe_cb,
            qos,
        )

        # Unified sartek_msgs Publishers
        self._telemetry_pub = self.create_publisher(TelemetryStatus, 'gorev/telemetry_status', 10)
        self._health_pub = self.create_publisher(SystemHealth, 'gorev/system_health', 10)

        # Cached state (ENU + degrees + WGS84)
        self._last_msg_mono = 0.0
        self._pos_x = 0.0
        self._pos_y = 0.0
        self._pos_z = 0.0
        self._vel_x = 0.0
        self._vel_y = 0.0
        self._vel_z = 0.0
        self._groundspeed = 0.0
        self._airspeed = 0.0
        self._airspeed_valid = False

        self._roll_deg = 0.0
        self._pitch_deg = 0.0
        self._yaw_deg = 0.0

        self._latitude = 0.0
        self._longitude = 0.0
        self._altitude_amsl = 0.0
        self._global_valid = False

        self._home_lat = 0.0
        self._home_lon = 0.0
        self._home_alt = 0.0
        self._home_set = False

        self._battery_voltage = 0.0
        self._battery_remaining = 1.0
        self._battery_received = False

        self._gps_fix_type = 0
        self._gps_satellites = 0
        self._gnss_received = False

        self._imu_vib_x = 0.0
        self._imu_vib_y = 0.0
        self._imu_vib_z = 0.0

        self._is_armed = False
        self._flight_mode = 'UNKNOWN'
        self._px4_failsafe = False
        self._rc_connected = True
        self._manual_control_lost = False

        period = 1.0 / max(rate_hz, 1.0)
        self.create_timer(period, self._publish_loop)

        self.get_logger().info(
            f'px4_telemetry_bridge started (fmu_prefix="{fmu_prefix}", rate={rate_hz:.1f} Hz)'
        )

    def _touch_connection(self) -> None:
        self._last_msg_mono = time.monotonic()

    def _local_pos_cb(self, msg: VehicleLocalPosition) -> None:
        self._touch_connection()
        if msg.xy_valid:
            self._pos_x, self._pos_y, _ = ned_to_enu(msg.x, msg.y, msg.z)
        if msg.z_valid:
            self._pos_z = float(-msg.z)
        if msg.v_xy_valid:
            self._vel_x, self._vel_y, _ = ned_to_enu(msg.vx, msg.vy, msg.vz)
            self._groundspeed = math.hypot(msg.vx, msg.vy)
        if msg.v_z_valid:
            self._vel_z = float(-msg.vz)

    def _attitude_cb(self, msg: VehicleAttitude) -> None:
        self._touch_connection()
        self._roll_deg, self._pitch_deg, self._yaw_deg = quaternion_to_euler_deg(msg.q)

    def _global_pos_cb(self, msg: VehicleGlobalPosition) -> None:
        self._touch_connection()
        self._global_valid = bool(msg.lat_lon_valid)
        if msg.lat_lon_valid:
            self._latitude = float(msg.lat)
            self._longitude = float(msg.lon)
        if msg.alt_valid:
            self._altitude_amsl = float(msg.alt)
        if not self._gnss_received and msg.lat_lon_valid and self._gps_fix_type < 2:
            self._gps_fix_type = 3

    def _airspeed_cb(self, msg: AirspeedValidated) -> None:
        self._touch_connection()
        if msg.airspeed_source == AirspeedValidated.SOURCE_DISABLED:
            self._airspeed_valid = False
            return
        for candidate in (
            msg.calibrated_airspeed_m_s,
            msg.indicated_airspeed_m_s,
            msg.true_airspeed_m_s,
        ):
            if math.isfinite(candidate) and candidate >= 0.0:
                self._airspeed = float(candidate)
                self._airspeed_valid = True
                return
        self._airspeed_valid = False

    def _battery_cb(self, msg: BatteryStatus) -> None:
        self._touch_connection()
        self._battery_received = True
        if math.isfinite(msg.voltage_v) and msg.voltage_v > 0.0:
            self._battery_voltage = float(msg.voltage_v)
        if math.isfinite(msg.remaining) and msg.remaining >= 0.0:
            self._battery_remaining = max(0.0, min(1.0, float(msg.remaining)))

    def _status_cb(self, msg: VehicleStatus) -> None:
        self._touch_connection()
        self._is_armed = msg.arming_state == VehicleStatus.ARMING_STATE_ARMED
        self._flight_mode = nav_state_to_str(msg.nav_state)
        self._px4_failsafe = bool(msg.failsafe)

    def _home_cb(self, msg: HomePosition) -> None:
        if msg.valid_hpos:
            self._home_lat = float(msg.lat)
            self._home_lon = float(msg.lon)
        if msg.valid_alt:
            self._home_alt = float(msg.alt)
        self._home_set = bool(msg.valid_hpos and msg.valid_alt)

    def _gnss_cb(self, msg: VehicleGnss) -> None:
        self._gnss_received = True
        self._gps_fix_type = int(msg.receiver.fix_type)
        self._gps_satellites = int(msg.receiver.satellites_used)

    def _sensor_cb(self, msg: SensorCombined) -> None:
        self._imu_vib_x = float(msg.accelerometer_m_s2[0])
        self._imu_vib_y = float(msg.accelerometer_m_s2[1])
        # FRD Z points down (+9.81 m/s^2 reaction -> -9.81 measured at rest)
        self._imu_vib_z = float(msg.accelerometer_m_s2[2]) + 9.81

    def _failsafe_cb(self, msg: FailsafeFlags) -> None:
        self._manual_control_lost = bool(msg.manual_control_signal_lost)
        self._rc_connected = not self._manual_control_lost

    def _evaluate_warnings(self, airspeed: float, is_connected: bool) -> list[str]:
        warnings = []
        if not is_connected:
            warnings.append('PX4_BAGLANTI_YOK')
            return warnings

        is_flying = self._is_armed and self._pos_z > 10.0
        if is_flying and 0.5 < airspeed < self._stall_speed:
            warnings.append(f'STALL_RISK:{airspeed:.1f}m/s')

        bat_pct = self._battery_remaining * 100.0
        if self._battery_received and (
            bat_pct <= self._bat_crit_pct
            or (self._battery_voltage > 0.0 and self._battery_voltage <= self._bat_crit_volt)
        ):
            warnings.append(f'BATARYA_KRITIK:{bat_pct:.0f}%')

        if self._gps_fix_type < 2 and not self._global_valid:
            warnings.append(f'GPS_YOK:Fix={self._gps_fix_type}')
        elif self._gnss_received and 0 < self._gps_satellites < self._gps_min_sats:
            warnings.append(f'GPS_ZAYIF:{self._gps_satellites}uydu')

        max_vib = max(abs(self._imu_vib_x), abs(self._imu_vib_y), abs(self._imu_vib_z))
        if is_flying and max_vib >= self._imu_vib_crit:
            warnings.append(f'IMU_VIBRASYON:{max_vib:.1f}')

        if self._px4_failsafe:
            warnings.append('PX4_FAILSAFE')

        return warnings

    def _publish_loop(self) -> None:
        now_stamp = self.get_clock().now().to_msg()
        is_connected = (
            self._last_msg_mono > 0.0
            and (time.monotonic() - self._last_msg_mono) <= self._conn_timeout_s
        )
        airspeed = self._airspeed if self._airspeed_valid else self._groundspeed
        warnings = self._evaluate_warnings(airspeed, is_connected)
        has_issue = len(warnings) > 0
        reason = ' | '.join(warnings)

        tel = TelemetryStatus()
        tel.header.stamp = now_stamp
        tel.header.frame_id = 'map'
        tel.airspeed = float(airspeed)
        tel.groundspeed = float(self._groundspeed)
        tel.vel_x = float(self._vel_x)
        tel.vel_y = float(self._vel_y)
        tel.vel_z = float(self._vel_z)
        tel.altitude = float(self._pos_z)
        tel.altitude_amsl = float(self._altitude_amsl)
        tel.pos_x = float(self._pos_x)
        tel.pos_y = float(self._pos_y)
        tel.pos_z = float(self._pos_z)
        tel.roll_deg = float(self._roll_deg)
        tel.pitch_deg = float(self._pitch_deg)
        tel.yaw_deg = float(self._yaw_deg)
        tel.latitude = float(self._latitude)
        tel.longitude = float(self._longitude)
        tel.home_latitude = float(self._home_lat)
        tel.home_longitude = float(self._home_lon)
        tel.home_altitude = float(self._home_alt)
        tel.home_set = bool(self._home_set)
        tel.battery_voltage = float(self._battery_voltage)
        tel.battery_remaining = float(self._battery_remaining)
        tel.gps_fix_type = max(0, min(255, int(self._gps_fix_type)))
        tel.gps_satellites = int(self._gps_satellites)
        tel.imu_vibration_x = float(self._imu_vib_x)
        tel.imu_vibration_y = float(self._imu_vib_y)
        tel.imu_vibration_z = float(self._imu_vib_z)
        tel.is_critical = has_issue
        tel.warning_message = reason
        tel.is_connected = bool(is_connected)
        self._telemetry_pub.publish(tel)

        health = SystemHealth()
        health.header.stamp = now_stamp
        health.header.frame_id = 'base_link'
        health.battery_voltage = float(self._battery_voltage)
        health.battery_percent = float(self._battery_remaining * 100.0)
        health.gps_fix_type = max(0, min(255, int(self._gps_fix_type)))
        health.gps_satellites = max(0, min(255, int(self._gps_satellites)))
        health.imu_vibration_x = float(self._imu_vib_x)
        health.imu_vibration_y = float(self._imu_vib_y)
        health.imu_vibration_z = float(self._imu_vib_z)
        health.is_armed = bool(self._is_armed)
        health.flight_mode = self._flight_mode
        health.rc_connected = bool(self._rc_connected)
        health.is_emergency = has_issue
        health.emergency_reason = reason
        self._health_pub.publish(health)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Px4TelemetryBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
