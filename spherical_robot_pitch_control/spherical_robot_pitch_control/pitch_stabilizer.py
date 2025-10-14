import math
from typing import Dict, Optional, Sequence, Tuple

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Imu
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


def _quat_multiply(q1: Tuple[float, float, float, float], q2: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def _quat_normalize(q: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
    x, y, z, w = q
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-9:
        return (0.0, 0.0, 0.0, 1.0)
    inv = 1.0 / norm
    return (x * inv, y * inv, z * inv, w * inv)


def _transform_to_quat(transform: TransformStamped) -> Tuple[float, float, float, float]:
    rot = transform.transform.rotation
    return (rot.x, rot.y, rot.z, rot.w)


class PitchStabilizer(Node):
    def __init__(self) -> None:
        super().__init__('pitch_stabilizer')

        # Params
        self._entity_name = self.declare_parameter('entity_name', 'spherical_robot').get_parameter_value().string_value
        self._link_name = self.declare_parameter('link_name', 'frame').get_parameter_value().string_value
        self._use_imu = self.declare_parameter('use_imu', True).get_parameter_value().bool_value
        self._imu_topic = self.declare_parameter('imu_topic', '/spherical_robot/frame_imu').get_parameter_value().string_value
        # PID gains (torque = -(Kp*e + Kd*rate + Ki*∫e))
        self._kp = self.declare_parameter('kp', 12.0).get_parameter_value().double_value
        self._kd = self.declare_parameter('kd', 3.0).get_parameter_value().double_value
        self._ki = self.declare_parameter('ki', 4.0).get_parameter_value().double_value
        # Limit the total torque commanded to each reaction wheel
        self._max_torque = self.declare_parameter('max_torque', 50.0).get_parameter_value().double_value
        # The left reaction wheel axis is mirrored w.r.t the right-hand Y wheel → default +1.0 so we send opposite values (u_left = -u_right)
        self._w_sign = self.declare_parameter('w_sign', 1.0).get_parameter_value().double_value
        # Target upright pitch (rad). Keep 0.0 if “frame Y” should be level.
        self._target_pitch = self.declare_parameter('target_pitch_rad', 0.0).get_parameter_value().double_value
        # Small deadband in rad around the target to reduce chatter
        self._deadband = self.declare_parameter('deadband_rad', 0.02).get_parameter_value().double_value  # ~1.1°
        # First-order low-pass filter for pitch rate (0..1). Higher → more smoothing.
        self._rate_lpf_alpha = self.declare_parameter('rate_lpf_alpha', 0.2).get_parameter_value().double_value
        # Cap the integral contribution (N·m)
        self._i_limit = self.declare_parameter('i_limit', 20.0).get_parameter_value().double_value
        # Period for debugging orientation logs (seconds). Set ≤0 to disable.
        self._log_period = self.declare_parameter('orientation_log_period', 0.2).get_parameter_value().double_value
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
        if self._use_imu:
            self.create_subscription(Imu, self._imu_topic, self._on_imu_message, 50)
        else:
            self.create_subscription(TFMessage, self._pose_topic, self._on_pose_message, 10)

        self._last_pitch: Optional[float] = None
        self._last_time: Optional[Time] = None
        self._rate_filt: float = 0.0
        self._i_term: float = 0.0  # integral contribution already scaled by Ki (units: N·m)
        self._last_debug_sec: Optional[float] = None
        self._debug_frame_logged = False

        self.get_logger().info(
            'PitchStabilizer → pitch axis control\n'
            f'  link:          {self._entity_name}::{self._link_name}\n'
            f'  source:        {"IMU " + self._imu_topic if self._use_imu else "TF " + self._pose_topic}\n'
            f'  cmd topic:     {self._topic_cmd}\n'
            f'  gains:         Kp={self._kp:.2f}  Ki={self._ki:.2f}  Kd={self._kd:.2f}\n'
            f'  limits:        |wheel torque| ≤ {self._max_torque:.1f} N·m, |I| ≤ {self._i_limit:.1f} N·m\n'
            f'  target pitch:  {self._target_pitch:.4f} rad, deadband={self._deadband:.4f} rad\n'
            f'  pairing:       w_sign={self._w_sign:.1f} (u_left = w_sign * (−u_right))'
        )

    def _on_imu_message(self, msg: Imu) -> None:
        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w
        roll, pitch, yaw = quat_to_euler_xyz(qx, qy, qz, qw)
        self._update_control_from_angles(roll, pitch, yaw, source='IMU')

    def _on_pose_message(self, msg: TFMessage) -> None:
        transform_map: Dict[str, TransformStamped] = {}
        for transform in msg.transforms:
            transform_map[transform.child_frame_id] = transform

        tf = None
        exact_match = False
        for candidate in self._alt_target_frames:
            tf = transform_map.get(candidate)
            if tf is not None:
                exact_match = True
                break

        if tf is None:
            # Provide a one-time dump of available frames to help with debugging
            if not self._debug_frame_logged:
                self._debug_frame_logged = True
                available = ', '.join(sorted(transform_map.keys())[:10])
                self.get_logger().warn(
                    f'No transform found for target frames {self._alt_target_frames}. Sample children: [{available}]'
                )
            return

        if not self._debug_frame_logged:
            parent = tf.header.frame_id if hasattr(tf, 'header') else '<no_header>'
            self.get_logger().info(
                f'Using pose transform child="{tf.child_frame_id}" parent="{parent}" '
                f'(exact_match={"yes" if exact_match else "no"})'
            )
            self._debug_frame_logged = True

        # Resolve orientation back to the world/root frame by walking ancestors.
        q_total = _transform_to_quat(tf)
        parent_name = tf.header.frame_id or ''
        visited = set()
        while parent_name and parent_name not in ('world', 'map', 'odom'):
            if parent_name in visited:
                break  # Prevent cycles; fall back to current value
            visited.add(parent_name)
            parent_tf = transform_map.get(parent_name)
            if parent_tf is None:
                break
            q_parent = _transform_to_quat(parent_tf)
            q_total = _quat_multiply(q_parent, q_total)
            parent_name = parent_tf.header.frame_id or ''

        q_total = _quat_normalize(q_total)
        roll, pitch, yaw = quat_to_euler_xyz(*q_total)
        self._update_control_from_angles(roll, pitch, yaw, source='TF')

    def _update_control_from_angles(self, roll: float, pitch: float, yaw: float, source: str) -> None:
        now = self.get_clock().now()
        dt = None
        raw_rate = 0.0
        if self._last_pitch is not None and self._last_time is not None:
            dt_candidate = (now - self._last_time).nanoseconds * 1e-9
            if dt_candidate > 1e-6:
                dt = dt_candidate
                raw_rate = (pitch - self._last_pitch) / dt
                # Low-pass the derivative to reduce noise
                self._rate_filt = (
                    self._rate_lpf_alpha * raw_rate + (1.0 - self._rate_lpf_alpha) * self._rate_filt
                )
        else:
            # First sample init
            self._rate_filt = 0.0

        # Error (target upright is 0 by default)
        e = pitch - self._target_pitch
        # Apply a small deadband
        if abs(e) < self._deadband and abs(self._rate_filt) < 2.0 * self._deadband:
            e = 0.0
            self._rate_filt = 0.0

        # ----- PID with conditional anti-windup -----
        # Proportional & Derivative parts (note the overall negative sign)
        u_lin = -(self._kp * e + self._kd * self._rate_filt)
        # Provisional control including integral contribution
        u_pre = u_lin - self._i_term

        # Saturate the provisional command to per-wheel limit (this is the base wheel command magnitude)
        u = max(-self._max_torque, min(self._max_torque, u_pre))

        # Conditional integration: integrate only if we're not saturating in the same direction as the error.
        # This prevents windup while still allowing integral to unwind when saturated.
        saturated = abs(u_pre) > self._max_torque + 1e-9
        if not saturated or (saturated and ((u_pre > 0.0 and e < 0.0) or (u_pre < 0.0 and e > 0.0))):
            if dt is not None:
                self._i_term += self._ki * e * dt
                # Limit the integral torque contribution directly
                self._i_term = max(-self._i_limit, min(self._i_limit, self._i_term))

            # Recompute final control with updated integral
            u = max(-self._max_torque, min(self._max_torque, u_lin - self._i_term))

        if self._log_period > 0.0:
            now_sec = now.nanoseconds * 1e-9
            if self._last_debug_sec is None or (now_sec - self._last_debug_sec) >= self._log_period:
                self.get_logger().info(
                    f'[{source}] RPY (rad): roll={roll:+.3f}, pitch={pitch:+.3f}, yaw={yaw:+.3f}'
                )
                self._last_debug_sec = now_sec

        self._last_pitch = pitch
        self._last_time = now

        # Interpret u as the **wheel Y** torque command.
        # To make frame torques add, the mirrored wheel gets the opposite sign (scaled by w_sign).
        # Command order expected by the JointGroupEffortController: [rw_back, rw_right, rw_front, rw_left, rw_yaw]
        y_torque = u
        w_torque = self._w_sign * (-u)

        out = Float64MultiArray()
        # Index → joint mapping (per ros2_control list):
        #   0: rw_back   → rolls left when positive
        #   1: rw_right  → pitches back when positive
        #   2: rw_front  → rolls right when positive
        #   3: rw_left   → pitches forward when positive
        #   4: rw_yaw    → yaws right when positive
        out.data = [0.0, y_torque, 0.0, w_torque, 0.0]
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
