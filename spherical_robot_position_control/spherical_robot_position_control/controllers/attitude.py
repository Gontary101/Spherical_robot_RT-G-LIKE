from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Tuple

from .utils import clamp


@dataclass
class WheelCommands:
    roll_pair: Tuple[float, float]
    pitch_pair: Tuple[float, float]
    roll_torque: float
    pitch_torque: float


class DualAxisAttitudeController:
    """Independent roll & pitch PID controllers with gyro feed-forward."""

    def __init__(
        self,
        kp_roll: float,
        ki_roll: float,
        kd_roll: float,
        kp_pitch: float,
        ki_pitch: float,
        kd_pitch: float,
        rate_lpf_alpha: float,
        deadband: float,
        torque_limit: float,
        integral_limit: float,
        gyro_feedforward_enable: bool,
        gyro_feedforward_gain: float,
        wheel_inertia: float,
        roll_pair_sign: float,
        pitch_pair_sign: float,
    ) -> None:
        self._kp_roll = kp_roll
        self._ki_roll = ki_roll
        self._kd_roll = kd_roll

        self._kp_pitch = kp_pitch
        self._ki_pitch = ki_pitch
        self._kd_pitch = kd_pitch

        self._alpha = rate_lpf_alpha
        self._deadband = abs(deadband)
        self._torque_limit = abs(torque_limit)
        self._integral_limit = abs(integral_limit)

        self._ff_enable = gyro_feedforward_enable
        self._ff_gain = gyro_feedforward_gain
        self._wheel_inertia = wheel_inertia

        self._roll_pair_sign = 1.0 if roll_pair_sign >= 0.0 else -1.0
        self._pitch_pair_sign = 1.0 if pitch_pair_sign >= 0.0 else -1.0

        self._roll_rate_filt = 0.0
        self._pitch_rate_filt = 0.0

        self._roll_integral = 0.0
        self._pitch_integral = 0.0

    def reset(self) -> None:
        self._roll_rate_filt = 0.0
        self._pitch_rate_filt = 0.0
        self._roll_integral = 0.0
        self._pitch_integral = 0.0

    def step(
        self,
        roll: float,
        pitch: float,
        roll_target: float,
        pitch_target: float,
        dt: float,
        roll_rate_meas: float,
        pitch_rate_meas: float,
        yaw_rate: float,
        wheel_velocities: Dict[str, float],
    ) -> WheelCommands:
        dt = max(dt, 1e-6)

        # Low-pass the measured body rates to tame noise
        self._roll_rate_filt = self._alpha * roll_rate_meas + (1.0 - self._alpha) * self._roll_rate_filt
        self._pitch_rate_filt = self._alpha * pitch_rate_meas + (1.0 - self._alpha) * self._pitch_rate_filt

        roll_cmd = self._axis_pid(
            angle=roll,
            target=roll_target,
            rate=self._roll_rate_filt,
            kp=self._kp_roll,
            ki=self._ki_roll,
            kd=self._kd_roll,
            integral=self._roll_integral,
            dt=dt,
        )
        self._roll_integral = roll_cmd.integral
        roll_torque = roll_cmd.command

        pitch_cmd = self._axis_pid(
            angle=pitch,
            target=pitch_target,
            rate=self._pitch_rate_filt,
            kp=self._kp_pitch,
            ki=self._ki_pitch,
            kd=self._kd_pitch,
            integral=self._pitch_integral,
            dt=dt,
        )
        self._pitch_integral = pitch_cmd.integral
        pitch_torque = pitch_cmd.command

        if self._ff_enable:
            hx = self._wheel_inertia * (
                wheel_velocities.get('rw_x', 0.0) - wheel_velocities.get('rw_z', 0.0)
            )
            hy = self._wheel_inertia * (
                wheel_velocities.get('rw_y', 0.0) - wheel_velocities.get('rw_w', 0.0)
            )
            tau_g_roll = -yaw_rate * hy
            tau_g_pitch = yaw_rate * hx
            roll_torque += -self._ff_gain * tau_g_roll
            pitch_torque += -self._ff_gain * tau_g_pitch
            roll_torque = clamp(roll_torque, -self._torque_limit, self._torque_limit)
            pitch_torque = clamp(pitch_torque, -self._torque_limit, self._torque_limit)

        # Map frame torques to wheel effort pairs (roll → [rw_x, rw_z], pitch → [rw_y, rw_w])
        tau_rw_x = clamp(roll_torque, -self._torque_limit, self._torque_limit)
        tau_rw_z = clamp(self._roll_pair_sign * (-roll_torque), -self._torque_limit, self._torque_limit)
        tau_rw_y = clamp(pitch_torque, -self._torque_limit, self._torque_limit)
        tau_rw_w = clamp(self._pitch_pair_sign * (-pitch_torque), -self._torque_limit, self._torque_limit)

        return WheelCommands(
            roll_pair=(tau_rw_x, tau_rw_z),
            pitch_pair=(tau_rw_y, tau_rw_w),
            roll_torque=roll_torque,
            pitch_torque=pitch_torque,
        )

    @dataclass
    class _AxisPIDResult:
        command: float
        integral: float

    def _axis_pid(
        self,
        angle: float,
        target: float,
        rate: float,
        kp: float,
        ki: float,
        kd: float,
        integral: float,
        dt: float,
    ) -> "_AxisPIDResult":
        error = angle - target
        if abs(error) < self._deadband and abs(rate) < 2.0 * self._deadband:
            error = 0.0
            rate = 0.0

        proportional = -(kp * error + kd * rate)
        tentative = clamp(proportional - integral, -self._torque_limit, self._torque_limit)
        saturated = abs(proportional - integral) > self._torque_limit + 1e-9

        if not saturated or (saturated and ((proportional - integral > 0.0 and error < 0.0) or (proportional - integral < 0.0 and error > 0.0))):
            integral += ki * error * dt
            integral = clamp(integral, -self._integral_limit, self._integral_limit)
            tentative = clamp(proportional - integral, -self._torque_limit, self._torque_limit)

        return self._AxisPIDResult(command=tentative, integral=integral)

