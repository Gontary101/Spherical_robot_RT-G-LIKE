import math
from typing import Dict, Optional, Tuple, Sequence

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64, Float64MultiArray
from tf2_msgs.msg import TFMessage

from ..controllers.attitude import DualAxisAttitudeController
from ..controllers.navigation import GoalNavigator
from ..controllers.utils import clamp, quat_to_euler_xyz


class PositionControllerNode(Node):
    """Goal-to-(x,y) controller driving reaction wheels and forward velocity."""

    def __init__(self) -> None:
        super().__init__('spherical_position_controller')

        # --- Parameters ---
        entity_name = self.declare_parameter('entity_name', 'spherical_robot').get_parameter_value().string_value
        link_name = self.declare_parameter('link_name', 'frame').get_parameter_value().string_value
        self._entity_name = entity_name

        self._imu_topic = self.declare_parameter('imu_topic', '/spherical_robot/frame_imu').get_parameter_value().string_value
        self._pose_topic = self.declare_parameter('pose_topic', '/world/default/pose/info').get_parameter_value().string_value
        self._joint_states_topic = self.declare_parameter('joint_states_topic', '/joint_states').get_parameter_value().string_value
        self._roll_effort_topic = self.declare_parameter(
            'roll_effort_topic',
            '/spherical_robot/inertial_wheel_roll_effort_controller/commands',
        ).get_parameter_value().string_value
        self._pitch_effort_topic = self.declare_parameter(
            'pitch_effort_topic',
            '/spherical_robot/inertial_wheel_pitch_effort_controller/commands',
        ).get_parameter_value().string_value
        self._drive_cmd_topic = self.declare_parameter(
            'drive_command_topic',
            '/spherical_robot/drive_velocity_command',
        ).get_parameter_value().string_value

        goal_x = self.declare_parameter('goal_x', 0.0).get_parameter_value().double_value
        goal_y = self.declare_parameter('goal_y', 0.0).get_parameter_value().double_value
        stop_distance = self.declare_parameter('stop_distance', 0.15).get_parameter_value().double_value

        motor_rad_max = self.declare_parameter('motor_rad_max', 7.5).get_parameter_value().double_value
        k_motor = self.declare_parameter('k_motor', 0.35).get_parameter_value().double_value
        k_heading = self.declare_parameter('k_heading', 2.5).get_parameter_value().double_value
        k_heading_rate_damp = self.declare_parameter('k_heading_rate_damp', 0.6).get_parameter_value().double_value
        heading_gate_rad = self.declare_parameter('heading_gate_rad', math.radians(60.0)).get_parameter_value().double_value
        gravity = self.declare_parameter('gravity', 9.81).get_parameter_value().double_value
        roll_max_rad = self.declare_parameter('roll_max_rad', math.radians(15.0)).get_parameter_value().double_value
        friction_mu_max = self.declare_parameter('friction_mu_max', 0.8).get_parameter_value().double_value
        roll_enable_shell_rad = self.declare_parameter('roll_enable_shell_rad', 0.5).get_parameter_value().double_value
        creep_motor_rad = self.declare_parameter('creep_motor_rad', 0.8).get_parameter_value().double_value
        motor_rad_acc_limit = self.declare_parameter('motor_rad_acc_limit', 3.0).get_parameter_value().double_value
        self._drive_radius = self.declare_parameter('drive_wheel_radius', 0.26926).get_parameter_value().double_value

        kp_roll = self.declare_parameter('kp_roll', 6.0).get_parameter_value().double_value
        ki_roll = self.declare_parameter('ki_roll', 1.5).get_parameter_value().double_value
        kd_roll = self.declare_parameter('kd_roll', 2.0).get_parameter_value().double_value
        kp_pitch = self.declare_parameter('kp_pitch', 20.0).get_parameter_value().double_value
        ki_pitch = self.declare_parameter('ki_pitch', 8.0).get_parameter_value().double_value
        kd_pitch = self.declare_parameter('kd_pitch', 6.0).get_parameter_value().double_value
        rate_lpf_alpha = self.declare_parameter('rate_lpf_alpha', 0.2).get_parameter_value().double_value
        deadband_rad = self.declare_parameter('deadband_rad', 0.02).get_parameter_value().double_value
        torque_limit_nm = self.declare_parameter('torque_limit_nm', 50.0).get_parameter_value().double_value
        i_limit_nm = self.declare_parameter('i_limit_nm', 20.0).get_parameter_value().double_value
        gyro_ff_enable = self.declare_parameter('gyro_ff_enable', True).get_parameter_value().bool_value
        gyro_ff_gain = self.declare_parameter('gyro_ff_gain', 1.0).get_parameter_value().double_value
        wheel_inertia = self.declare_parameter('wheel_inertia_kgm2', 0.0530374).get_parameter_value().double_value
        roll_pair_sign = self.declare_parameter('roll_pair_sign', 1.0).get_parameter_value().double_value
        pitch_pair_sign = self.declare_parameter('pitch_pair_sign', 1.0).get_parameter_value().double_value

        control_rate_hz = self.declare_parameter('control_rate_hz', 200.0).get_parameter_value().double_value
        self._control_period = 1.0 / max(control_rate_hz, 1.0)
        self._log_period = self.declare_parameter('log_period', 0.5).get_parameter_value().double_value

        self._target_frames = (
            f'{entity_name}::{link_name}',
            f'{entity_name}/{link_name}',
            entity_name,
            link_name,
        )

        # --- Controllers ---
        self._navigator = GoalNavigator(
            goal_xy=(goal_x, goal_y),
            stop_distance=stop_distance,
            motor_rad_max=motor_rad_max,
            k_motor=k_motor,
            k_heading=k_heading,
            k_heading_rate_damp=k_heading_rate_damp,
            heading_gate_rad=heading_gate_rad,
            gravity=gravity,
            roll_max_rad=roll_max_rad,
            friction_mu_max=friction_mu_max,
            roll_enable_shell_rad=roll_enable_shell_rad,
            creep_motor_rad=creep_motor_rad,
            acc_limit=motor_rad_acc_limit,
            drive_radius=self._drive_radius,
        )
        self._attitude = DualAxisAttitudeController(
            kp_roll=kp_roll,
            ki_roll=ki_roll,
            kd_roll=kd_roll,
            kp_pitch=kp_pitch,
            ki_pitch=ki_pitch,
            kd_pitch=kd_pitch,
            rate_lpf_alpha=rate_lpf_alpha,
            deadband=deadband_rad,
            torque_limit=torque_limit_nm,
            integral_limit=i_limit_nm,
            gyro_feedforward_enable=gyro_ff_enable,
            gyro_feedforward_gain=gyro_ff_gain,
            wheel_inertia=wheel_inertia,
            roll_pair_sign=roll_pair_sign,
            pitch_pair_sign=pitch_pair_sign,
        )

        # --- Publishers & subscriptions ---
        self._pub_roll = self.create_publisher(Float64MultiArray, self._roll_effort_topic, 10)
        self._pub_pitch = self.create_publisher(Float64MultiArray, self._pitch_effort_topic, 10)
        self._pub_drive = self.create_publisher(Float64, self._drive_cmd_topic, 10)

        self.create_subscription(Imu, self._imu_topic, self._on_imu, 100)
        self.create_subscription(TFMessage, self._pose_topic, self._on_pose, 20)
        self.create_subscription(JointState, self._joint_states_topic, self._on_joint_state, 50)

        self._timer = self.create_timer(self._control_period, self._on_timer)

        # --- State ---
        self._pose_available = False
        self._position_xy: Tuple[float, float] = (0.0, 0.0)

        self._imu_available = False
        self._roll = self._pitch = self._yaw = 0.0
        self._roll_rate = self._pitch_rate = self._yaw_rate = 0.0

        self._wheel_velocities: Dict[str, float] = {}
        self._outer_shell_velocity: float = 0.0
        self._last_motor_command: float = 0.0

        self._last_control_time: Optional[Time] = None
        self._last_log_time: Optional[float] = None
        self._missing_pose_warned = False
        self._pose_frame_logged = False

        self.get_logger().info(
            'Position controller running:\n'
            f'  goals: ({goal_x:.3f}, {goal_y:.3f}) m  stop ≤ {stop_distance:.2f} m\n'
            f'  topics: imu={self._imu_topic}, pose={self._pose_topic}, joint_states={self._joint_states_topic}\n'
            f'          roll_effort={self._roll_effort_topic}, pitch_effort={self._pitch_effort_topic}, drive={self._drive_cmd_topic}'
        )

    # ------------------------------------------------------------------
    # Subscriptions
    # ------------------------------------------------------------------

    def _on_imu(self, msg: Imu) -> None:
        qx, qy, qz, qw = msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
        self._roll, self._pitch, self._yaw = quat_to_euler_xyz(qx, qy, qz, qw)
        self._roll_rate = msg.angular_velocity.x
        self._pitch_rate = msg.angular_velocity.y
        self._yaw_rate = msg.angular_velocity.z
        self._imu_available = True

    def _on_pose(self, msg: TFMessage) -> None:
        resolved = self._resolve_world_position(msg, self._target_frames)
        if resolved is None:
            if not self._missing_pose_warned:
                sample = ', '.join(sorted(t.child_frame_id for t in msg.transforms)[:10])
                self.get_logger().warn(
                    f'No world pose for target frames {self._target_frames}. Sample children: [{sample}]'
                )
                self._missing_pose_warned = True
            return
        position, frame_id = resolved

        if not self._pose_frame_logged:
            self.get_logger().info(
                f'Using pose frame "{frame_id}" for world position (x={position[0]:+.3f}, y={position[1]:+.3f}).'
            )
            self._pose_frame_logged = True

        x, y, _ = position
        self._position_xy = (x, y)
        self._pose_available = True

    def _on_joint_state(self, msg: JointState) -> None:
        for name, vel in zip(msg.name, msg.velocity):
            if name in ('rw_x', 'rw_y', 'rw_z', 'rw_w'):
                self._wheel_velocities[name] = vel
            elif name == 'outer_2':
                self._outer_shell_velocity = vel

    # ------------------------------------------------------------------
    # Control loop
    # ------------------------------------------------------------------

    def _on_timer(self) -> None:
        if not self._pose_available or not self._imu_available:
            return

        now = self.get_clock().now()
        if self._last_control_time is None:
            self._last_control_time = now
            return

        dt = (now - self._last_control_time).nanoseconds * 1e-9
        if dt <= 1e-6:
            return
        self._last_control_time = now

        shell_rad = self._outer_shell_velocity
        if abs(shell_rad) < 1e-6:
            # Fallback: last commanded shell speed (bridge ratio is 1.0)
            shell_rad = self._last_motor_command

        nav_cmd = self._navigator.compute(
            position_xy=self._position_xy,
            yaw=self._yaw,
            yaw_rate=self._yaw_rate,
            shell_rad=shell_rad,
            dt=dt,
        )

        drive_msg = Float64()
        drive_msg.data = -nav_cmd.motor_velocity
        self._pub_drive.publish(drive_msg)
        self._last_motor_command = drive_msg.data

        wheel_cmds = self._attitude.step(
            roll=self._roll,
            pitch=self._pitch,
            roll_target=nav_cmd.roll_target,
            pitch_target=nav_cmd.pitch_target,
            dt=dt,
            roll_rate_meas=self._roll_rate,
            pitch_rate_meas=self._pitch_rate,
            yaw_rate=self._yaw_rate,
            wheel_velocities=self._wheel_velocities,
        )

        roll_msg = Float64MultiArray()
        roll_msg.data = [wheel_cmds.roll_pair[0], wheel_cmds.roll_pair[1]]
        pitch_msg = Float64MultiArray()
        pitch_msg.data = [wheel_cmds.pitch_pair[0], wheel_cmds.pitch_pair[1]]

        self._pub_roll.publish(roll_msg)
        self._pub_pitch.publish(pitch_msg)

        if self._log_period > 0.0:
            now_sec = now.nanoseconds * 1e-9
            if self._last_log_time is None or (now_sec - self._last_log_time) >= self._log_period:
                self.get_logger().info(
                    f'err=({nav_cmd.error_x:+.3f},{nav_cmd.error_y:+.3f})m '
                    f'dist={nav_cmd.distance:.3f}m '
                    f'psi_err={math.degrees(nav_cmd.heading_error):+.1f}deg '
                    f'psi_dot_cmd={nav_cmd.yaw_rate_command:+.3f}rad/s '
                    f'roll_tgt={math.degrees(nav_cmd.roll_target):+.1f}deg '
                    f'cmd_shell={nav_cmd.motor_velocity:.3f}rad/s '
                    f'shell_rad={shell_rad:.3f}rad/s '
                    f'roll_eff=[{wheel_cmds.roll_pair[0]:+.2f},{wheel_cmds.roll_pair[1]:+.2f}] '
                    f'pitch_eff=[{wheel_cmds.pitch_pair[0]:+.2f},{wheel_cmds.pitch_pair[1]:+.2f}]'
                )
                self._last_log_time = now_sec

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _resolve_world_position(
        self,
        msg: TFMessage,
        candidates: Sequence[str],
    ) -> Optional[Tuple[Tuple[float, float, float], str]]:
        tf_map: Dict[str, TransformStamped] = {t.child_frame_id: t for t in msg.transforms}
        roots = {'', 'world', 'map', 'odom'}

        # First pass: exact candidates whose parent is already world-like
        for candidate in candidates:
            tf = tf_map.get(candidate)
            if tf is None:
                continue
            parent = tf.header.frame_id or ''
            if parent not in roots:
                continue
            pos = (
                tf.transform.translation.x,
                tf.transform.translation.y,
                tf.transform.translation.z,
            )
            return pos, candidate

        # Second pass: any transform belonging to this entity already rooted in world
        for tf in msg.transforms:
            child = tf.child_frame_id
            parent = tf.header.frame_id or ''
            if parent not in roots:
                continue
            if child.startswith(self._entity_name):
                pos = (
                    tf.transform.translation.x,
                    tf.transform.translation.y,
                    tf.transform.translation.z,
                )
                return pos, child

        return None


def main() -> None:
    rclpy.init()
    node = PositionControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
