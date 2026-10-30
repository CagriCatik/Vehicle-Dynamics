"""Virtual chassis sensors: corruption, sampling, latency and dropout.

Signal chain per call of :meth:`SensorSuite.measure` (ground truth in, measurement out):

1. sampling     a new sample is taken when ``t`` reaches the next sample instant
                (``sample_period``; 0 = every call); otherwise the previous output is held
2. corruption   bias (initial value + random walk), white noise, wheel-encoder and
                steering quantisation, first-order steering-sensor lag (``steer_lag_tau``)
3. latency      the delivered sample is the newest one at least ``latency`` old (FIFO)
4. dropout      a channel inside a scheduled outage window, or dropped with probability
                ``dropout_probability`` per sample, keeps its last valid value and is
                flagged invalid in ``SensorMeasurements.valid``

All randomness comes from one seeded generator, so runs are repeatable. The
defaults (no sampling, latency, lag or dropout) reproduce the original sensor model
exactly, including its random-number sequence.

Estimators receive only :class:`SensorMeasurements`; it contains no ground-truth
quantities (positions, sideslip, tyre slips or forces).
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
import math
from pathlib import Path
from typing import Any
import numpy as np


@dataclass
class SensorConfig:
    """Configuration for automotive chassis sensor suite."""

    # IMU Accelerometer
    accel_noise_std: float = 0.15          # [m/s^2]
    accel_bias_init: float = 0.05          # [m/s^2]
    accel_bias_drift_std: float = 0.001    # [m/s^2/sqrt(s)]

    # IMU Gyroscope (yaw rate)
    gyro_noise_std: float = 0.015          # [rad/s] (~0.85 deg/s)
    gyro_bias_init: float = 0.008          # [rad/s] (~0.45 deg/s)
    gyro_bias_drift_std: float = 0.0001    # [rad/s/sqrt(s)]

    # Wheel Speed Sensors (WSS)
    encoder_ticks_per_rev: int = 48        # Standard ABS tone ring tick count
    wheel_noise_std: float = 0.20          # [rad/s]
    quantize_wheel_speed: bool = True

    # Steering Wheel Angle Sensor
    steer_noise_std: float = 0.0017        # [rad] (~0.1 deg)
    steer_quantization_rad: float = 0.001  # [rad] (~0.05 deg)
    steer_lag_tau: float = 0.0             # [s] first-order sensor filter (0: none)

    # Timing and availability (all channels)
    sample_period: float = 0.0             # [s] 0: a new sample on every call
    latency: float = 0.0                   # [s] transport delay of every channel
    dropout_probability: float = 0.0       # [-] per sample and channel
    outages: tuple = ()                    # ((channel, t_start, t_end), ...) scheduled dropouts

    @classmethod
    def from_yaml(cls, path: str | Path | None = None) -> SensorConfig:
        """Load sensor noise and bias parameters from centralized YAML."""
        import yaml
        if path is None:
            from vehicle_model.config.loader import get_configs_root
            p = get_configs_root() / "estimators" / "sensors.yaml"
        else:
            p = Path(path)

        with open(p, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}

        imu = data.get("imu", {})
        wss = data.get("wheel_speed_sensors", {})
        steer = data.get("steering_angle_sensor", {})
        timing = data.get("timing", {})

        return cls(
            accel_noise_std=float(imu.get("accel_noise_std", 0.15)),
            accel_bias_init=float(imu.get("accel_bias_init", 0.05)),
            accel_bias_drift_std=float(imu.get("accel_bias_drift_std", 0.001)),
            gyro_noise_std=float(imu.get("gyro_noise_std", 0.015)),
            gyro_bias_init=float(imu.get("gyro_bias_init", 0.008)),
            gyro_bias_drift_std=float(imu.get("gyro_bias_drift_std", 0.0001)),
            encoder_ticks_per_rev=int(wss.get("encoder_ticks_per_rev", 48)),
            wheel_noise_std=float(wss.get("noise_std", 0.20)),
            quantize_wheel_speed=bool(wss.get("quantization_enabled", True)),
            steer_noise_std=float(steer.get("noise_std", 0.0017)),
            steer_quantization_rad=float(steer.get("quantization_rad", 0.001)),
            steer_lag_tau=float(steer.get("lag_tau", 0.0)),
            sample_period=float(timing.get("sample_period", 0.0)),
            latency=float(timing.get("latency", 0.0)),
            dropout_probability=float(timing.get("dropout_probability", 0.0)),
        )


@dataclass
class SensorMeasurements:
    """Clean measurement container accessible to onboard control & state estimators.

    Notice: Does NOT contain ground-truth global positions (X, Y), true sideslip,
    or ground-truth tyre slip to preserve strict physical realism.
    """

    t: float
    ax: float                      # Measured longitudinal acceleration [m/s^2]
    ay: float                      # Measured lateral acceleration [m/s^2]
    r: float                       # Measured yaw rate [rad/s]
    omega: np.ndarray              # Measured 4-wheel angular speeds [rad/s] (FL, FR, RL, RR)
    steer_f: float                 # Measured front road wheel steering angle [rad]
    steer_r: float                 # Measured rear road wheel steering angle [rad]
    valid: dict = field(default_factory=lambda: {c: True for c in CHANNELS})  # False: dropped, value held


CHANNELS = ("ax", "ay", "r", "omega", "steer")


class SensorSuite:
    """Simulates realistic automotive chassis sensor corruption and sampling."""

    def __init__(self, config: SensorConfig | None = None, seed: int = 42):
        self.config = config or SensorConfig()
        self.rng = np.random.RandomState(seed)
        c = self.config
        if c.sample_period < 0 or c.latency < 0 or c.steer_lag_tau < 0 or not 0.0 <= c.dropout_probability <= 1.0:
            raise ValueError("sample_period, latency and steer_lag_tau must be >= 0, dropout_probability in [0, 1]")
        for ch, *_ in c.outages:
            if ch not in CHANNELS:
                raise ValueError(f"unknown sensor channel {ch!r}; channels: {CHANNELS}")
        self.reset()

    def reset(self) -> None:
        """Reset biases, filters, sampling and delay state."""
        self.bias_ax = self.config.accel_bias_init
        self.bias_ay = self.config.accel_bias_init
        self.bias_r = self.config.gyro_bias_init
        self._steer_f_lag = None
        self._steer_r_lag = None
        self._next_sample = -np.inf
        self._held: SensorMeasurements | None = None
        self._queue: deque[SensorMeasurements] = deque()
        self._last_valid: SensorMeasurements | None = None

    def _corrupt(self, t, dt, true_ax, true_ay, true_r, true_omega, true_steer_f, true_steer_r) -> SensorMeasurements:
        c = self.config
        # 1. Random walk bias drift: b_{k+1} = b_k + N(0, drift_std * sqrt(dt))
        sq_dt = math.sqrt(max(1e-6, dt))
        self.bias_ax += float(self.rng.normal(0.0, c.accel_bias_drift_std * sq_dt))
        self.bias_ay += float(self.rng.normal(0.0, c.accel_bias_drift_std * sq_dt))
        self.bias_r += float(self.rng.normal(0.0, c.gyro_bias_drift_std * sq_dt))

        # 2. Corrupt IMU measurements: z = x_true + bias + noise
        ax_meas = true_ax + self.bias_ax + float(self.rng.normal(0.0, c.accel_noise_std))
        ay_meas = true_ay + self.bias_ay + float(self.rng.normal(0.0, c.accel_noise_std))
        r_meas = true_r + self.bias_r + float(self.rng.normal(0.0, c.gyro_noise_std))

        # 3. Corrupt Wheel Speed Sensors (WSS)
        w_noisy = np.asarray(true_omega, dtype=float) + self.rng.normal(0.0, c.wheel_noise_std, size=4)
        if c.quantize_wheel_speed and c.encoder_ticks_per_rev > 0:
            rad_per_tick = 2.0 * math.pi / c.encoder_ticks_per_rev
            w_meas = np.round(w_noisy / rad_per_tick) * rad_per_tick
        else:
            w_meas = w_noisy

        # 4. Steering angle sensor: optional first-order lag, noise, quantisation
        if c.steer_lag_tau > 0.0:
            a = dt / (c.steer_lag_tau + dt)
            self._steer_f_lag = true_steer_f if self._steer_f_lag is None else self._steer_f_lag + a * (true_steer_f - self._steer_f_lag)
            self._steer_r_lag = true_steer_r if self._steer_r_lag is None else self._steer_r_lag + a * (true_steer_r - self._steer_r_lag)
            true_steer_f, true_steer_r = self._steer_f_lag, self._steer_r_lag
        q_rad = c.steer_quantization_rad
        sf = true_steer_f + float(self.rng.normal(0.0, c.steer_noise_std))
        sr = true_steer_r + float(self.rng.normal(0.0, c.steer_noise_std))
        if q_rad > 0.0:
            sf, sr = round(sf / q_rad) * q_rad, round(sr / q_rad) * q_rad
        return SensorMeasurements(t=t, ax=ax_meas, ay=ay_meas, r=r_meas, omega=w_meas, steer_f=sf, steer_r=sr)

    def measure(
        self,
        t: float,
        dt: float,
        true_ax: float,
        true_ay: float,
        true_r: float,
        true_omega: np.ndarray,
        true_steer_f: float,
        true_steer_r: float = 0.0,
    ) -> SensorMeasurements:
        """Measurement delivered at time ``t`` (module docstring: sampling, corruption, latency, dropout)."""
        c = self.config
        # 1-2 sampling and corruption
        if c.sample_period <= 0.0 or t >= self._next_sample - 1e-12:
            period = c.sample_period if c.sample_period > 0.0 else dt
            if c.sample_period > 0.0:
                self._next_sample = (t if self._next_sample == -np.inf else self._next_sample) + c.sample_period
            sample = self._corrupt(t, period, true_ax, true_ay, true_r, true_omega, true_steer_f, true_steer_r)
            self._queue.append(sample)
        # 3 latency: newest sample that is at least `latency` old
        while len(self._queue) > 1 and self._queue[1].t <= t - c.latency + 1e-12:
            self._queue.popleft()
        out = self._queue[0]
        if c.latency > 0.0 and out.t > t - c.latency + 1e-12:
            out = self._held if self._held is not None else out  # nothing old enough yet
        self._held = out

        # 4 dropout: hold the last valid value per channel
        dropped = {ch: any(o_ch == ch and t0 <= t < t1 for o_ch, t0, t1 in c.outages) for ch in CHANNELS}
        if c.dropout_probability > 0.0:
            for ch in CHANNELS:
                dropped[ch] = dropped[ch] or bool(self.rng.random_sample() < c.dropout_probability)
        if not any(dropped.values()) or self._last_valid is None:
            self._last_valid = out
            return SensorMeasurements(out.t, out.ax, out.ay, out.r, out.omega.copy(), out.steer_f, out.steer_r)
        lv = self._last_valid
        pick = lambda ch, new, old: old if dropped[ch] else new
        result = SensorMeasurements(
            t=out.t, ax=pick("ax", out.ax, lv.ax), ay=pick("ay", out.ay, lv.ay), r=pick("r", out.r, lv.r),
            omega=pick("omega", out.omega, lv.omega).copy(),
            steer_f=pick("steer", out.steer_f, lv.steer_f), steer_r=pick("steer", out.steer_r, lv.steer_r),
            valid={ch: not dropped[ch] for ch in CHANNELS},
        )
        self._last_valid = result
        return result


__all__ = ("CHANNELS", "SensorConfig", "SensorMeasurements", "SensorSuite")
