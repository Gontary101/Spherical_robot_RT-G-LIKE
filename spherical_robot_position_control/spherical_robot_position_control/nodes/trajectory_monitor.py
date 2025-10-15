import math
from typing import Dict, Optional, Sequence, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped


class TrajectoryMonitor(Node):
    """Periodically log goal vs. actual pose and drivetrain velocities."""

    def __init__(self) -> None:
        super().__init__('trajectory_monitor')

        self._goal_x = self.declare_parameter('goal_x', 0.0).get_parameter_value().double_value
        self._goal_y = self.declare_parameter('goal_y', 0.0).get_parameter_value().double_value
        self._pose_topic = self.declare_parameter('pose_topic', '/world/default/pose/info').get_parameter_value().string_value
        self._joint_states_topic = self.declare_parameter('joint_states_topic', '/joint_states').get_parameter_value().string_value
        self._drive_cmd_topic = self.declare_parameter('drive_command_topic', '/spherical_robot/drive_velocity_command').get_parameter_value().string_value
        self._drive_radius = self.declare_parameter('drive_wheel_radius', 0.26926).get_parameter_value().double_value
        self._outer_velocity_ratio = self.declare_parameter('outer_velocity_ratio', 0.25).get_parameter_value().double_value
        self._motor_2_sign = self.declare_parameter('motor_2_sign', -1.0).get_parameter_value().double_value
        self._log_period = max(0.1, self.declare_parameter('log_period', 0.5).get_parameter_value().double_value)

        entity_name = self.declare_parameter('entity_name', 'spherical_robot').get_parameter_value().string_value
        link_name = self.declare_parameter('link_name', 'frame').get_parameter_value().string_value
        self._target_frames: Sequence[str] = (
            f'{entity_name}::{link_name}',
            f'{entity_name}/{link_name}',
            entity_name,
            link_name,
        )
        self._entity_name = entity_name

        self._position_xy: Optional[Tuple[float, float]] = None
        self._cmd_motor: float = 0.0
        self._motor_velocity_1: float = 0.0
        self._motor_velocity_2: float = 0.0
        self._outer_velocity: float = 0.0
        self._last_log_time: Optional[float] = None
        self._pose_frame_logged = False

        self.create_subscription(TFMessage, self._pose_topic, self._on_pose, 20)
        self.create_subscription(JointState, self._joint_states_topic, self._on_joint_state, 50)
        self.create_subscription(Float64, self._drive_cmd_topic, self._on_drive_command, 10)

        self.create_timer(self._log_period, self._on_timer)

        self.get_logger().info(
            f'TrajectoryMonitor tracking goal=({self._goal_x:.2f}, {self._goal_y:.2f}) using pose topic {self._pose_topic}.'
        )

    def _on_pose(self, msg: TFMessage) -> None:
        resolved = self._resolve_world_position(msg, self._target_frames)
        if resolved is None:
            if not self._pose_frame_logged:
                sample = ', '.join(sorted(t.child_frame_id for t in msg.transforms)[:8])
                self.get_logger().warn(
                    f'TrajectoryMonitor: no world pose for {self._target_frames}. Sample children: [{sample}]'
                )
                self._pose_frame_logged = True  # avoid spamming
            return

        position, frame_id = resolved
        if not self._pose_frame_logged:
            self.get_logger().info(f'TrajectoryMonitor: using pose frame "{frame_id}".')
            self._pose_frame_logged = True

        self._position_xy = (position[0], position[1])

    def _on_joint_state(self, msg: JointState) -> None:
        for name, vel in zip(msg.name, msg.velocity):
            if name == 'motor_joint_1':
                self._motor_velocity_1 = vel
            elif name == 'motor_joint_2':
                self._motor_velocity_2 = vel
            elif name == 'outer_2':
                self._outer_velocity = vel

    def _on_drive_command(self, msg: Float64) -> None:
        # Command sent to the velocity controller (rad/s)
        self._cmd_motor = msg.data

    def _on_timer(self) -> None:
        if self._position_xy is None:
            return

        now_sec = self.get_clock().now().nanoseconds * 1e-9
        shell_rad = self._outer_velocity
        if abs(shell_rad) < 1e-6:
            shell_rad = self._outer_velocity_ratio * 0.5 * (
                self._motor_velocity_1 - self._motor_2_sign * self._motor_velocity_2
            )
        shell_linear = shell_rad * self._drive_radius

        err_x = self._goal_x - self._position_xy[0]
        err_y = self._goal_y - self._position_xy[1]

        self.get_logger().info(
            f't={now_sec:7.2f}s goal=({self._goal_x:+.3f},{self._goal_y:+.3f}) '
            f'pose=({self._position_xy[0]:+.3f},{self._position_xy[1]:+.3f}) '
            f'err=({err_x:+.3f},{err_y:+.3f}) '
            f'cmd_motor={self._cmd_motor:+.3f}rad/s '
            f'motor=[{self._motor_velocity_1:+.3f},{self._motor_velocity_2:+.3f}]rad/s '
            f'shell_rad={shell_rad:+.3f}rad/s shell_lin={shell_linear:+.3f}m/s'
        )
        self._last_log_time = now_sec

    def _resolve_world_position(
        self,
        msg: TFMessage,
        candidates: Sequence[str],
    ) -> Optional[Tuple[Tuple[float, float, float], str]]:
        tf_map: Dict[str, TransformStamped] = {t.child_frame_id: t for t in msg.transforms}
        roots = {'', 'world', 'map', 'odom'}

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

        for tf in msg.transforms:
            parent = tf.header.frame_id or ''
            if parent not in roots:
                continue
            child = tf.child_frame_id
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
    node = TrajectoryMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
