import math
from typing import Optional, Sequence

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Float64MultiArray
from tf2_msgs.msg import TFMessage


def quat_to_euler_xyz(x: float, y: float, z: float, w: float):
    # Returns roll, pitch, yaw
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return roll, pitch, yaw


class PitchStabilizer(Node):
    def __init__(self) -> None:
        super().__init__('pitch_stabilizer')

        # Params
        self._entity_name = self.declare_parameter('entity_name', 'spherical_robot').get_parameter_value().string_value
        self._link_name = self.declare_parameter('link_name', 'frame').get_parameter_value().string_value
        self._kp = self.declare_parameter('kp', 10.0).get_parameter_value().double_value
        self._kd = self.declare_parameter('kd', 2.0).get_parameter_value().double_value
        self._max_torque = self.declare_parameter('max_torque', 50.0).get_parameter_value().double_value
        # Reaction wheel W is inverted relative to Y
        self._w_sign = self.declare_parameter('w_sign', -1.0).get_parameter_value().double_value
        self._topic_cmd = self.declare_parameter(
            'effort_command_topic', '/spherical_robot/inertial_wheel_effort_controller/commands'
        ).get_parameter_value().string_value
        self._target_frame = self.declare_parameter('target_frame', f'{self._entity_name}::{self._link_name}').get_parameter_value().string_value
        self._pose_topic = self.declare_parameter('pose_topic', '/spherical_robot/pose_info').get_parameter_value().string_value
        self._alt_target_frames: Sequence[str] = (
            self._target_frame,
            f'{self._entity_name}/{self._link_name}',
            self._link_name,
        )

        self._publisher = self.create_publisher(Float64MultiArray, self._topic_cmd, 10)
        self.create_subscription(TFMessage, self._pose_topic, self._on_pose_message, 10)

        self._last_pitch: Optional[float] = None
        self._last_time: Optional[Time] = None

        self.get_logger().info(
            f'PitchStabilizer stabilizing pitch for {self._entity_name}::{self._link_name} using {self._pose_topic} '
            f'publishing torques to {self._topic_cmd} (kp={self._kp}, kd={self._kd}, max={self._max_torque}, w_sign={self._w_sign})'
        )

    def _on_pose_message(self, msg: TFMessage) -> None:
        target_tf = None
        fallback_tf = None
        for transform in msg.transforms:
            child = transform.child_frame_id
            if child in self._alt_target_frames:
                target_tf = transform
                break
            # Keep the first transform whose child contains the link name
            if fallback_tf is None and self._link_name in child:
                fallback_tf = transform

        tf = target_tf or fallback_tf
        if tf is None:
            return

        qx = tf.transform.rotation.x
        qy = tf.transform.rotation.y
        qz = tf.transform.rotation.z
        qw = tf.transform.rotation.w
        _, pitch, _ = quat_to_euler_xyz(qx, qy, qz, qw)

        now = self.get_clock().now()
        pitch_rate = 0.0
        if self._last_pitch is not None and self._last_time is not None:
            dt = (now - self._last_time).nanoseconds * 1e-9
            if dt > 1e-6:
                pitch_rate = (pitch - self._last_pitch) / dt

        self._last_pitch = pitch
        self._last_time = now

        # PD control: torque tries to drive pitch -> 0 (upright)
        torque = -self._kp * pitch - self._kd * pitch_rate
        torque = max(-self._max_torque, min(self._max_torque, torque))

        # Command order: [rw_x, rw_y, rw_z, rw_w]
        y_torque = torque
        w_torque = self._w_sign * (-torque)

        out = Float64MultiArray()
        out.data = [0.0, y_torque, 0.0, w_torque]
        self._publisher.publish(out)


def main() -> None:
    rclpy.init()
    node = PitchStabilizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
