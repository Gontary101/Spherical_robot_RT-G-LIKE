import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray


class DriveCommandBridge(Node):
    """Republish a single drive velocity command into joint-group commands with gear ratios."""

    def __init__(self) -> None:
        super().__init__('drive_command_bridge')

        input_topic = self.declare_parameter('input_topic', 'drive_velocity_command').get_parameter_value().string_value
        controller_topic = self.declare_parameter(
            'controller_topic',
            '/spherical_robot/drive_motor_velocity_controller/commands',
        ).get_parameter_value().string_value
        outer_ratio = self.declare_parameter('outer_ratio', 1.0).get_parameter_value().double_value

        if math.isclose(outer_ratio, 0.0):
            self.get_logger().warn('outer_ratio parameter is 0.0; outer shell will not receive commands.')
        self._outer_ratio = outer_ratio

        self._publisher = self.create_publisher(Float64MultiArray, controller_topic, 10)
        self.create_subscription(Float64, input_topic, self._on_velocity_command, 10)

        self.get_logger().info(
            f'drive_command_bridge listening on {input_topic} → {controller_topic} (outer_ratio={outer_ratio:.3f})'
        )

    def _on_velocity_command(self, msg: Float64) -> None:
        command = msg.data
        out_msg = Float64MultiArray()
        # Single joint: outer_2 only (JointGroupVelocityController with one joint).
        out_msg.data = [self._outer_ratio * command]
        self._publisher.publish(out_msg)


def main() -> None:
    rclpy.init()
    node = DriveCommandBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
