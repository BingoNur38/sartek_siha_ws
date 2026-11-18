#!/usr/bin/env python3
"""
PX4 Control Bridge — sartek_msgs -> PX4 Micro XRCE-DDS.

Translates ENU `sartek_msgs/MissionCommand` and `sartek_msgs/SafetyOverride`
commands into PX4 `/fmu/in/offboard_control_mode`, `/fmu/in/trajectory_setpoint`,
`/fmu/in/vehicle_attitude_setpoint_v1`, `/fmu/in/vehicle_rates_setpoint`, and
`/fmu/in/vehicle_command_v1`.
"""

import math
import time

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleAttitudeSetpoint,
    VehicleCommand,
    VehicleCommandAck,
    VehicleRatesSetpoint,
    VehicleStatus,
)
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
from sartek_msgs.msg import MissionCommand, SafetyOverride
from std_msgs.msg import String

from sartek_px4_bridge.conversions import (
    MODE_STRING_TO_PX4,
    enu_to_ned,
    euler_deg_to_quaternion,
    nav_state_to_str,
    px4_qos_profile,
)

_CMD_RESULT_NAMES = {
    VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED: 'ACCEPTED',
    VehicleCommandAck.VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED: 'TEMPORARILY_REJECTED',
    VehicleCommandAck.VEHICLE_CMD_RESULT_DENIED: 'DENIED',
    VehicleCommandAck.VEHICLE_CMD_RESULT_UNSUPPORTED: 'UNSUPPORTED',
    VehicleCommandAck.VEHICLE_CMD_RESULT_FAILED: 'FAILED',
    VehicleCommandAck.VEHICLE_CMD_RESULT_IN_PROGRESS: 'IN_PROGRESS',
    VehicleCommandAck.VEHICLE_CMD_RESULT_CANCELLED: 'CANCELLED',
}


class Px4ControlBridge(Node):
    """Bridge MissionCommand / SafetyOverride / terminal commands to PX4 uXRCE-DDS."""

    def __init__(self) -> None:
        super().__init__('px4_control_bridge')

        self.declare_parameter('fmu_prefix', '')
        self.declare_parameter('active_commander', '')
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('default_altitude_m', 80.0)
        self.declare_parameter('default_speed_ms', 18.0)
        self.declare_parameter('target_system', 1)
        self.declare_parameter('target_component', 1)

        fmu_prefix = self.get_parameter('fmu_prefix').value.rstrip('/')
        self._active_commander = str(self.get_parameter('active_commander').value)
        rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self._default_alt = float(self.get_parameter('default_altitude_m').value)
        self._default_speed = float(self.get_parameter('default_speed_ms').value)
        self._target_system = int(self.get_parameter('target_system').value)
        self._target_component = int(self.get_parameter('target_component').value)

        self.add_on_set_parameters_callback(self._on_param_change)

        def fmu_in(suffix: str) -> str:
            return f'{fmu_prefix}/fmu/in/{suffix}' if fmu_prefix else f'/fmu/in/{suffix}'

        def fmu_out(suffix: str) -> str:
            return f'{fmu_prefix}/fmu/out/{suffix}' if fmu_prefix else f'/fmu/out/{suffix}'

        qos = px4_qos_profile(depth=10)

        # High-level subscribers
        self.create_subscription(
            MissionCommand, 'gorev/mission_command', self._mission_cmd_cb, 10
        )
        self.create_subscription(
            SafetyOverride, 'gorev/safety_override', self._override_cb, 10
        )
        self.create_subscription(
            String, 'gorev/vehicle_command', self._vehicle_cmd_str_cb, 10
        )

        # PX4 feedback subscribers
        self.create_subscription(
            VehicleStatus, fmu_out('vehicle_status_v5'), self._status_cb, qos
        )
        self.create_subscription(
            VehicleCommandAck, fmu_out('vehicle_command_ack_v2'), self._ack_cb, qos
        )

        # PX4 /fmu/in/* publishers
        self._offboard_mode_pub = self.create_publisher(
            OffboardControlMode, fmu_in('offboard_control_mode'), qos
        )
        self._traj_sp_pub = self.create_publisher(
            TrajectorySetpoint, fmu_in('trajectory_setpoint'), qos
        )
        self._att_sp_pub = self.create_publisher(
            VehicleAttitudeSetpoint, fmu_in('vehicle_attitude_setpoint_v1'), qos
        )
        self._rates_sp_pub = self.create_publisher(
            VehicleRatesSetpoint, fmu_in('vehicle_rates_setpoint'), qos
        )
        self._vehicle_cmd_pub = self.create_publisher(
            VehicleCommand, fmu_in('vehicle_command_v1'), qos
        )

        # Internal setpoint state (ENU)
        self._control_mode = 'IDLE'  # IDLE, POSITION, ATTITUDE, RATES
        self._target_x = 0.0
        self._target_y = 0.0
        self._target_z = self._default_alt
        self._target_speed = self._default_speed
        self._last_sent_speed = 0.0

        self._roll_deg = 0.0
        self._pitch_deg = 0.0
        self._yaw_deg = 0.0
        self._thrust = 0.5

        self._override_active = False
        self._override_mode = ''
        self._nav_state = -1
        self._is_armed = False

        self.create_timer(1.0 / max(rate_hz, 2.0), self._control_loop)
        self.get_logger().info(
            f'px4_control_bridge started (active_commander="{self._active_commander}", '
            f'rate={rate_hz:.1f} Hz)'
        )

    @staticmethod
    def _px4_timestamp_us() -> int:
        return int(time.time_ns() // 1000)

    def _on_param_change(self, params) -> SetParametersResult:
        for param in params:
            if param.name == 'active_commander':
                new_cmd = str(param.value)
                if new_cmd != self._active_commander:
                    self.get_logger().info(
                        f'active_commander updated: "{self._active_commander}" -> "{new_cmd}"'
                    )
                    self._active_commander = new_cmd
        return SetParametersResult(successful=True)

    def _status_cb(self, msg: VehicleStatus) -> None:
        self._nav_state = int(msg.nav_state)
        self._is_armed = msg.arming_state == VehicleStatus.ARMING_STATE_ARMED

    def _ack_cb(self, msg: VehicleCommandAck) -> None:
        if msg.command in (
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF,
            VehicleCommand.VEHICLE_CMD_DO_CHANGE_SPEED,
        ):
            res_name = _CMD_RESULT_NAMES.get(msg.result, str(msg.result))
            self.get_logger().info(
                f'PX4 CMD ACK cmd={msg.command} -> {res_name} (param1={msg.result_param1})'
            )

    def _mission_cmd_cb(self, msg: MissionCommand) -> None:
        if self._override_active:
            return

        if (
            self._active_commander
            and msg.commander_id
            and msg.commander_id != self._active_commander
            and self._active_commander != 'coordinator'
        ):
            return

        cmd_type = msg.command_type.strip().upper() or 'GOTO'

        if cmd_type in ('RTL', 'LOITER', 'LAND', 'OFFBOARD', 'TAKEOFF'):
            self._handle_mode_or_action(cmd_type)
            return

        if cmd_type in ('ATTITUDE', 'ATTITUDE_BYPASS'):
            self._control_mode = 'ATTITUDE'
            self._roll_deg = float(msg.roll_deg)
            self._pitch_deg = float(msg.pitch_deg)
            self._yaw_deg = float(msg.yaw_deg)
            self._thrust = max(0.0, min(1.0, float(msg.thrust)))
            return

        if cmd_type == 'RATES':
            self._control_mode = 'RATES'
            self._roll_deg = float(msg.roll_deg)
            self._pitch_deg = float(msg.pitch_deg)
            self._yaw_deg = float(msg.yaw_deg)
            self._thrust = max(0.0, min(1.0, float(msg.thrust)))
            return

        # Default: GOTO / ORBIT -> Offboard position control (ENU -> NED)
        self._control_mode = 'POSITION'
        self._target_x = float(msg.target_x)
        self._target_y = float(msg.target_y)
        self._target_z = float(msg.target_z) if abs(msg.target_z) > 0.01 else self._default_alt
        if msg.speed > 0.0 and abs(msg.speed - self._last_sent_speed) > 0.25:
            self._target_speed = float(msg.speed)
            self._send_speed_command(self._target_speed)

    def _override_cb(self, msg: SafetyOverride) -> None:
        if msg.override_active:
            was_inactive = not self._override_active or self._override_mode != msg.override_mode
            self._override_active = True
            self._override_mode = msg.override_mode.strip().upper()
            if was_inactive:
                self.get_logger().error(
                    f'SAFETY OVERRIDE ACTIVE: {self._override_mode} — {msg.reason}'
                )

            if self._override_mode in ('LOITER', 'RTL', 'LAND'):
                self._send_mode_command(self._override_mode)
                self._control_mode = 'IDLE'
            elif self._override_mode == 'CLIMB':
                self._control_mode = 'POSITION'
                self._target_z = float(msg.override_alt) if msg.override_alt > 0.0 else max(
                    self._target_z, 100.0
                )
            elif self._override_mode == 'FULL_THROTTLE':
                self._control_mode = 'ATTITUDE'
                self._roll_deg = 0.0
                self._pitch_deg = 5.0
                self._thrust = 1.0
        elif self._override_active:
            self._override_active = False
            self._override_mode = ''
            self.get_logger().info('SafetyOverride cleared; MissionCommand control resumed.')

    def _vehicle_cmd_str_cb(self, msg: String) -> None:
        raw = msg.data.strip().upper()
        if not raw:
            return
        parts = raw.split()
        cmd = parts[0]
        force = len(parts) > 1 and parts[1] == 'FORCE'

        if cmd == 'ARM':
            self._send_arm_command(True, force=force)
        elif cmd == 'DISARM':
            self._send_arm_command(False, force=force)
        elif cmd == 'RESET':
            self._control_mode = 'IDLE'
            self._override_active = False
            self.get_logger().info('Control setpoint reset to IDLE.')
        else:
            self._handle_mode_or_action(cmd)

    def _handle_mode_or_action(self, cmd: str) -> None:
        if cmd == 'TAKEOFF':
            self._send_takeoff_command()
            return
        if cmd == 'OFFBOARD':
            if self._control_mode == 'IDLE':
                self._control_mode = 'POSITION'
                self.get_logger().info(
                    f'No prior MissionCommand; streaming default ENU setpoint '
                    f'({self._target_x:.1f}, {self._target_y:.1f}, {self._target_z:.1f})'
                )
            self._publish_setpoint_once()
        self._send_mode_command(cmd)

    def _send_vehicle_command(
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
        msg.timestamp = self._px4_timestamp_us()
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.param3 = float(param3)
        msg.param4 = float(param4)
        msg.param5 = float(param5)
        msg.param6 = float(param6)
        msg.param7 = float(param7)
        msg.command = int(command)
        msg.target_system = self._target_system
        msg.target_component = self._target_component
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self._vehicle_cmd_pub.publish(msg)

    def _send_arm_command(self, arm: bool, force: bool = False) -> None:
        self.get_logger().info(f'Sending {"ARM" if arm else "DISARM"} command (force={force})')
        self._send_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=1.0 if arm else 0.0,
            param2=21196.0 if force else 0.0,
        )

    def _send_takeoff_command(self) -> None:
        self.get_logger().info('Sending NAV_TAKEOFF command')
        self._send_vehicle_command(
            VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF,
            param1=math.nan,
            param2=math.nan,
            param3=math.nan,
            param4=math.nan,
            param5=0.0,
            param6=0.0,
            param7=math.nan,
        )

    def _send_mode_command(self, mode_name: str) -> None:
        if mode_name not in MODE_STRING_TO_PX4:
            self.get_logger().warn(f'Unsupported flight mode command: "{mode_name}"')
            return
        main_mode, sub_mode = MODE_STRING_TO_PX4[mode_name]
        self.get_logger().info(
            f'Sending DO_SET_MODE "{mode_name}" (main={main_mode}, sub={sub_mode}) '
            f'[current={nav_state_to_str(self._nav_state)}]'
        )
        self._send_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,  # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
            param2=float(main_mode),
            param3=float(sub_mode),
        )

    def _send_speed_command(self, speed_ms: float) -> None:
        self._last_sent_speed = speed_ms
        self._send_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_CHANGE_SPEED,
            param1=0.0,  # SPEED_TYPE_AIRSPEED
            param2=float(speed_ms),
            param3=-1.0,
        )

    def _publish_setpoint_once(self) -> None:
        ts = self._px4_timestamp_us()

        mode_msg = OffboardControlMode()
        mode_msg.timestamp = ts
        mode_msg.position = self._control_mode == 'POSITION'
        mode_msg.attitude = self._control_mode == 'ATTITUDE'
        mode_msg.body_rate = self._control_mode == 'RATES'
        self._offboard_mode_pub.publish(mode_msg)

        if self._control_mode == 'POSITION':
            north, east, down = enu_to_ned(self._target_x, self._target_y, self._target_z)
            sp = TrajectorySetpoint()
            sp.timestamp = ts
            sp.position = [north, east, down]
            sp.velocity = [math.nan, math.nan, math.nan]
            sp.acceleration = [math.nan, math.nan, math.nan]
            sp.jerk = [math.nan, math.nan, math.nan]
            sp.yaw = math.nan
            sp.yawspeed = math.nan
            self._traj_sp_pub.publish(sp)

        elif self._control_mode == 'ATTITUDE':
            att = VehicleAttitudeSetpoint()
            att.timestamp = ts
            att.yaw_sp_move_rate = math.nan
            att.q_d = list(euler_deg_to_quaternion(self._roll_deg, self._pitch_deg, self._yaw_deg))
            att.thrust_body = [float(self._thrust), 0.0, 0.0]
            self._att_sp_pub.publish(att)

        elif self._control_mode == 'RATES':
            rates = VehicleRatesSetpoint()
            rates.timestamp = ts
            rates.roll = math.radians(self._roll_deg)
            rates.pitch = math.radians(self._pitch_deg)
            rates.yaw = math.radians(self._yaw_deg)
            rates.thrust_body = [float(self._thrust), 0.0, 0.0]
            self._rates_sp_pub.publish(rates)

    def _control_loop(self) -> None:
        if self._control_mode == 'IDLE':
            return
        self._publish_setpoint_once()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Px4ControlBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
