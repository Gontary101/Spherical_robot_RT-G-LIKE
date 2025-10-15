import math
from dataclasses import dataclass
from typing import Tuple

from .utils import clamp, wrap_angle


@dataclass
class NavigationCommand:
    motor_velocity: float
    roll_target: float
    pitch_target: float
    arrived: bool
    distance: float
    heading_error: float
    yaw_rate_command: float
    error_x: float
    error_y: float


class GoalNavigator:
    """Compute motor velocity and roll target for a goal pose."""

    def __init__(
        self,
        goal_xy: Tuple[float, float],
        stop_distance: float,
        motor_rad_max: float,
        k_motor: float,
        k_heading: float,
        k_heading_rate_damp: float,
        heading_gate_rad: float,
        gravity: float,
        roll_max_rad: float,
        friction_mu_max: float,
        roll_enable_shell_rad: float,
        creep_motor_rad: float,
        acc_limit: float,
        drive_radius: float,
    ) -> None:
        self._goal_x, self._goal_y = goal_xy
        self._stop_distance = max(0.0, stop_distance)
        self._motor_rad_max = max(0.0, motor_rad_max)
        self._k_motor = max(0.0, k_motor)
        self._k_heading = k_heading
        self._k_heading_rate_damp = k_heading_rate_damp
        self._heading_gate = max(0.0, heading_gate_rad)
        self._gravity = max(1e-6, gravity)
        self._roll_cap = max(0.0, min(roll_max_rad, math.atan(max(1e-6, friction_mu_max))))
        self._roll_enable_shell_rad = max(0.0, roll_enable_shell_rad)
        self._creep_motor = max(0.0, creep_motor_rad)
        self._acc_limit = max(0.0, acc_limit)
        self._drive_radius = max(1e-6, drive_radius)

        self._prev_motor_velocity = 0.0

    def compute(
        self,
        position_xy: Tuple[float, float],
        yaw: float,
        yaw_rate: float,
        shell_rad: float,
        dt: float,
    ) -> NavigationCommand:
        x, y = position_xy
        dx = self._goal_x - x
        dy = self._goal_y - y
        distance = math.hypot(dx, dy)
        arrived = distance <= self._stop_distance

        desired_heading = math.atan2(dy, dx) if not arrived else yaw
        heading_error = wrap_angle(desired_heading - yaw)

        yaw_rate_cmd = 0.0 if arrived else (self._k_heading * heading_error - self._k_heading_rate_damp * yaw_rate)

        if arrived:
            motor_target = 0.0
        else:
            base = min(self._k_motor * distance, self._motor_rad_max)
            if abs(heading_error) > self._heading_gate:
                base = max(base, self._creep_motor)
            sign = 1.0 if math.cos(heading_error) >= 0.0 else -1.0
            motor_target = clamp(sign * base, -self._motor_rad_max, self._motor_rad_max)

        dv_max = self._acc_limit * max(dt, 0.0)
        motor_cmd = clamp(motor_target, self._prev_motor_velocity - dv_max, self._prev_motor_velocity + dv_max)
        self._prev_motor_velocity = motor_cmd

        roll_target = 0.0
        if not arrived:
            if abs(shell_rad) > self._roll_enable_shell_rad:
                shell_speed = abs(shell_rad) * self._drive_radius
                arg = (shell_speed * yaw_rate_cmd) / self._gravity
                roll_target = clamp(math.atan(arg), -self._roll_cap, self._roll_cap)

        return NavigationCommand(
            motor_velocity=motor_cmd,
            roll_target=roll_target,
            pitch_target=0.0,
            arrived=arrived,
            distance=distance,
            heading_error=heading_error,
            yaw_rate_command=yaw_rate_cmd,
            error_x=dx,
            error_y=dy,
        )
