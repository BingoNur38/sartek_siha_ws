#!/usr/bin/env python3
"""
SARTEK Simülasyon — Sanal (Hedef) İHA Otonom Uçuş Düğümü (PX4 v1.18 / Micro XRCE-DDS)
======================================================================================
Her bir sanal sabit kanatlı İHA'yı kendi DDS isim uzayı (/gz_advanced_plane_1..5)
üzerinden otonom olarak:
  1. GPS ve lokal pozisyon hazır olmasını bekler
  2. AUTO.TAKEOFF modunu ayarlar ve ARM eder (pistten otonom kalkış)
  3. Güvenli tırmanma irtifasına ulaşınca OFFBOARD moduna geçer
  4. Kendi irtifa katmanında rastgele (veya tanımlı) waypoint'leri sürekli takip eder

Kullanım:
  ros2 run sartek_sim target_offboard --ros-args \
      -p namespace:=gz_advanced_plane_1 -p target_system:=2 -p cruise_altitude:=60.0
"""

import math
import random
import time

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleGlobalPosition,
    VehicleLocalPosition,
    VehicleStatus,
)
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class FlightState:
    WAIT_GPS = 0
    TAKEOFF_CMD = 1
    ARM = 2
    CLIMBING = 3
    PRE_OFFBOARD = 4
    SET_OFFBOARD = 5
    WAYPOINT = 6

    NAMES = {
        0: 'WAIT_GPS',
        1: 'TAKEOFF_CMD',
        2: 'ARM',
        3: 'CLIMBING',
        4: 'PRE_OFFBOARD',
        5: 'SET_OFFBOARD',
        6: 'WAYPOINT',
    }


class TargetOffboardNode(Node):
    """PX4 v1.18 Micro XRCE-DDS tabanlı otonom sanal sabit kanat kontrolcüsü."""

    def __init__(self) -> None:
        super().__init__('target_offboard')

        self.declare_parameter('namespace', 'gz_advanced_plane_1')
        self.declare_parameter('target_system', 2)
        self.declare_parameter('takeoff_altitude', 45.0)
        self.declare_parameter('cruise_altitude', 65.0)
        self.declare_parameter('waypoint_threshold', 45.0)
        self.declare_parameter('area_min_n', -220.0)
        self.declare_parameter('area_max_n', 320.0)
        self.declare_parameter('area_min_e', -220.0)
        self.declare_parameter('area_max_e', 320.0)
        self.declare_parameter('seed', 0)

        self.ns = str(self.get_parameter('namespace').value).strip('/')
        self.target_sys = int(self.get_parameter('target_system').value)
        self.takeoff_alt_rel = float(self.get_parameter('takeoff_altitude').value)
        self.cruise_alt = float(self.get_parameter('cruise_altitude').value)
        self.wp_threshold = float(self.get_parameter('waypoint_threshold').value)
        self.area_min_n = float(self.get_parameter('area_min_n').value)
        self.area_max_n = float(self.get_parameter('area_max_n').value)
        self.area_min_e = float(self.get_parameter('area_min_e').value)
        self.area_max_e = float(self.get_parameter('area_max_e').value)
        seed = int(self.get_parameter('seed').value)

        self.rng = random.Random(seed if seed != 0 else (self.target_sys * 97 + int(time.time())))

        self.current_wp = self._generate_random_waypoint()
        self.wp_count = 1

        self.vehicle_status = None
        self.vehicle_local_pos = None
        self.home_alt_amsl = None
        self.flight_state = FlightState.WAIT_GPS
        self.setpoint_count = 0
        self.tick = 0
        self.state_ticks = 0

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        prefix = f'/{self.ns}' if self.ns else ''

        # PX4 v1.18 versiyonlu topic'ler
        self.create_subscription(
            VehicleStatus,
            f'{prefix}/fmu/out/vehicle_status_v5',
            self._status_cb,
            px4_qos,
        )
        self.create_subscription(
            VehicleLocalPosition,
            f'{prefix}/fmu/out/vehicle_local_position_v1',
            self._local_pos_cb,
            px4_qos,
        )
        self.create_subscription(
            VehicleGlobalPosition,
            f'{prefix}/fmu/out/vehicle_global_position',
            self._global_pos_cb,
            px4_qos,
        )

        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode,
            f'{prefix}/fmu/in/offboard_control_mode',
            px4_qos,
        )
        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint,
            f'{prefix}/fmu/in/trajectory_setpoint',
            px4_qos,
        )
        self.vehicle_cmd_pub = self.create_publisher(
            VehicleCommand,
            f'{prefix}/fmu/in/vehicle_command_v1',
            px4_qos,
        )

        self.create_timer(0.05, self._control_loop)  # 20 Hz
        self.create_timer(2.0, self._report)         # 0.5 Hz log

        self.get_logger().info(
            f'[{self.ns}] Başlatıldı | sys_id={self.target_sys} | '
            f'kalkış=+{self.takeoff_alt_rel:.0f}m | seyir={self.cruise_alt:.0f}m'
        )

    @staticmethod
    def _timestamp_us() -> int:
        # 0 -> PX4 uxrce_dds_client mesajı kendi simülasyon saatiyle damgalar
        # (Gazebo yavaşladığında / RTF düştüğünde zaman kaymasını önler)
        return 0

    def _generate_random_waypoint(self, prev_wp=None):
        """Yarışma sahası içinde güvenli, uçuşa uygun rastgele NED waypoint üret."""
        for _ in range(20):
            n = self.rng.uniform(self.area_min_n, self.area_max_n)
            e = self.rng.uniform(self.area_min_e, self.area_max_e)
            alt = self.cruise_alt + self.rng.uniform(-5.0, 5.0)
            if prev_wp is None:
                return (n, e, -alt)
            dist = math.hypot(n - prev_wp[0], e - prev_wp[1])
            if dist >= 140.0:  # Sabit kanat dönüş yarıçapı için yeterince uzak olsun
                return (n, e, -alt)
        return (n, e, -alt)

    def _status_cb(self, msg: VehicleStatus) -> None:
        self.vehicle_status = msg

    def _local_pos_cb(self, msg: VehicleLocalPosition) -> None:
        self.vehicle_local_pos = msg

    def _global_pos_cb(self, msg: VehicleGlobalPosition) -> None:
        if self.home_alt_amsl is None and abs(msg.lat) > 1e-4:
            self.home_alt_amsl = float(msg.alt)
            self.get_logger().info(
                f'[{self.ns}] Home GPS alındı: lat={msg.lat:.6f}, '
                f'lon={msg.lon:.6f}, amsl={self.home_alt_amsl:.1f}m'
            )

    def _publish_vehicle_command(
        self,
        command: int,
        param1: float = 0.0,
        param2: float = 0.0,
        param3: float = 0.0,
        param4: float = 0.0,
        param5: float = 0.0,
        param6: float = 0.0,
        param7: float = 0.0,
    ) -> None:
        msg = VehicleCommand()
        msg.timestamp = self._timestamp_us()
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.param3 = float(param3)
        msg.param4 = float(param4)
        msg.param5 = float(param5)
        msg.param6 = float(param6)
        msg.param7 = float(param7)
        msg.command = int(command)
        msg.target_system = self.target_sys
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_cmd_pub.publish(msg)

    def _send_takeoff(self) -> None:
        target_amsl = self.home_alt_amsl + self.takeoff_alt_rel
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF,
            param1=math.nan,
            param2=math.nan,
            param3=math.nan,
            param4=math.nan,
            param5=0.0,
            param6=0.0,
            param7=target_amsl,
        )
        self.get_logger().info(
            f'[{self.ns}] TAKEOFF gönderildi (hedef {target_amsl:.0f}m AMSL)'
        )

    def _send_arm(self) -> None:
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=1.0,
            param2=0.0,
        )
        self.get_logger().info(f'[{self.ns}] ARM gönderildi')

    def _send_offboard(self) -> None:
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,  # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
            param2=6.0,  # PX4_CUSTOM_MAIN_MODE_OFFBOARD
        )
        self.get_logger().info(f'[{self.ns}] OFFBOARD modu istendi')

    def _current_alt_rel(self) -> float:
        if self.vehicle_local_pos is None:
            return 0.0
        return float(-self.vehicle_local_pos.z)

    def _dist_to_wp_2d(self) -> float:
        if self.vehicle_local_pos is None:
            return float('inf')
        dn = self.vehicle_local_pos.x - self.current_wp[0]
        de = self.vehicle_local_pos.y - self.current_wp[1]
        return math.hypot(dn, de)

    def _publish_offboard_setpoint(self) -> None:
        ts = self._timestamp_us()

        ocm = OffboardControlMode()
        ocm.timestamp = ts
        ocm.position = True
        ocm.velocity = False
        ocm.acceleration = False
        ocm.attitude = False
        ocm.body_rate = False
        self.offboard_mode_pub.publish(ocm)

        sp = TrajectorySetpoint()
        sp.timestamp = ts
        sp.position = [
            float(self.current_wp[0]),
            float(self.current_wp[1]),
            float(self.current_wp[2]),
        ]
        sp.velocity = [math.nan, math.nan, math.nan]
        sp.acceleration = [math.nan, math.nan, math.nan]
        sp.jerk = [math.nan, math.nan, math.nan]
        sp.yaw = math.nan
        sp.yawspeed = math.nan
        self.trajectory_pub.publish(sp)
        self.setpoint_count += 1

    def _set_state(self, new_state: int) -> None:
        self.flight_state = new_state
        self.state_ticks = 0

    def _control_loop(self) -> None:
        self.tick += 1
        self.state_ticks += 1

        # Offboard kalp atışını yalnızca kalkış tamamlandıktan sonra gönder
        if self.flight_state >= FlightState.PRE_OFFBOARD:
            self._publish_offboard_setpoint()

        if self.flight_state == FlightState.WAIT_GPS:
            if (
                self.home_alt_amsl is not None
                and self.vehicle_status is not None
                and self.vehicle_local_pos is not None
                and self.state_ticks >= 40  # EKF stabilize olması için ~2s bekle
            ):
                self.get_logger().info(f'[{self.ns}] Sensörler hazır → TAKEOFF başlatılıyor')
                self._set_state(FlightState.TAKEOFF_CMD)
            return

        if self.vehicle_status is None:
            return

        nav = int(self.vehicle_status.nav_state)
        armed = self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED

        if self.flight_state == FlightState.TAKEOFF_CMD:
            if self.state_ticks % 40 == 1:
                self._send_takeoff()
            if nav == VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF:
                self.get_logger().info(f'[{self.ns}] AUTO.TAKEOFF aktif → ARM ediliyor')
                self._set_state(FlightState.ARM)
            elif armed and self._current_alt_rel() > 15.0:
                self._set_state(FlightState.CLIMBING)

        elif self.flight_state == FlightState.ARM:
            if not armed and self.state_ticks % 40 == 1:
                if nav != VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF:
                    self._send_takeoff()
                self._send_arm()
            if armed:
                self.get_logger().info(f'[{self.ns}] ARMED! Pistten kalkış ve tırmanma başladı')
                self._set_state(FlightState.CLIMBING)

        elif self.flight_state == FlightState.CLIMBING:
            alt = self._current_alt_rel()
            # Kalkış yüksekliğine ulaşıldıysa veya PX4 AUTO.LOITER'a geçtiyse Offboard'a hazırlan
            if alt >= (self.takeoff_alt_rel - 8.0) or (
                alt >= 25.0 and nav == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER
            ):
                self.get_logger().info(
                    f'[{self.ns}] Tırmanma tamamlandı ({alt:.0f}m) → OFFBOARD hazırlığı'
                )
                self.setpoint_count = 0
                self._set_state(FlightState.PRE_OFFBOARD)

        elif self.flight_state == FlightState.PRE_OFFBOARD:
            if self.setpoint_count >= 40:  # 2 saniye boyunca 20Hz setpoint akışı
                self._set_state(FlightState.SET_OFFBOARD)

        elif self.flight_state == FlightState.SET_OFFBOARD:
            if not armed:
                self.get_logger().warn(f'[{self.ns}] Disarm oldu → TAKEOFF adımı yenileniyor')
                self._set_state(FlightState.TAKEOFF_CMD)
                return
            if self.state_ticks % 30 == 1:
                self._send_offboard()
            if nav == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self.get_logger().info(
                    f'[{self.ns}] OFFBOARD aktif! Rastgele waypoint uçuşu başladı → '
                    f'WP#{self.wp_count} (N={self.current_wp[0]:.0f}, '
                    f'E={self.current_wp[1]:.0f}, Alt={-self.current_wp[2]:.0f}m)'
                )
                self._set_state(FlightState.WAYPOINT)

        elif self.flight_state == FlightState.WAYPOINT:
            if not armed:
                self.get_logger().warn(f'[{self.ns}] Disarm oldu → TAKEOFF adımı yenileniyor')
                self._set_state(FlightState.TAKEOFF_CMD)
                return
            if nav != VehicleStatus.NAVIGATION_STATE_OFFBOARD and self.state_ticks % 40 == 1:
                self._send_offboard()

            if self._dist_to_wp_2d() < self.wp_threshold:
                self.current_wp = self._generate_random_waypoint(self.current_wp)
                self.wp_count += 1
                self.get_logger().info(
                    f'[{self.ns}] Yeni rastgele hedef → WP#{self.wp_count}: '
                    f'N={self.current_wp[0]:.0f}, E={self.current_wp[1]:.0f}, '
                    f'Alt={-self.current_wp[2]:.0f}m'
                )

    def _report(self) -> None:
        if self.vehicle_status is None or self.vehicle_local_pos is None:
            return
        armed = self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED
        state_name = FlightState.NAMES.get(self.flight_state, '?')
        self.get_logger().info(
            f'[{self.ns}] {"ARMED" if armed else "DISARM"} | {state_name} | '
            f'Nav={self.vehicle_status.nav_state} | Alt={self._current_alt_rel():.1f}m | '
            f'WP#{self.wp_count} dist={self._dist_to_wp_2d():.0f}m'
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TargetOffboardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
