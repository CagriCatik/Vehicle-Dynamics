"""Automotive sensor models, corruption dynamics, and measurement containers.

Fulfills Plan.md §11 (Milestone M6):
Realistic synthetic sensor measurements with configurable Gaussian noise,
constant biases, random walk drift, encoder tick quantization, and latency.
"""

from __future__ import annotations

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


class SensorSuite:
    """Simulates realistic automotive chassis sensor corruption and sampling."""

    def __init__(self, config: SensorConfig | None = None, seed: int = 42):
        self.config = config or SensorConfig()
        self.rng = np.random.RandomState(seed)

        # Persistent biases
        self.bias_ax = self.config.accel_bias_init
        self.bias_ay = self.config.accel_bias_init
        self.bias_r = self.config.gyro_bias_init

    def reset(self) -> None:
        """Reset sensor biases to initial values."""
        self.bias_ax = self.config.accel_bias_init
        self.bias_ay = self.config.accel_bias_init
        self.bias_r = self.config.gyro_bias_init

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
        """Corrupt ground-truth simulation variables with realistic sensor physics."""
        # 1. Random walk bias drift: b_{k+1} = b_k + N(0, drift_std * sqrt(dt))
        sq_dt = math.sqrt(max(1e-6, dt))
        self.bias_ax += float(self.rng.normal(0.0, self.config.accel_bias_drift_std * sq_dt))
        self.bias_ay += float(self.rng.normal(0.0, self.config.accel_bias_drift_std * sq_dt))
        self.bias_r += float(self.rng.normal(0.0, self.config.gyro_bias_drift_std * sq_dt))

        # 2. Corrupt IMU measurements: z = x_true + bias + noise
        ax_meas = true_ax + self.bias_ax + float(self.rng.normal(0.0, self.config.accel_noise_std))
        ay_meas = true_ay + self.bias_ay + float(self.rng.normal(0.0, self.config.accel_noise_std))
        r_meas = true_r + self.bias_r + float(self.rng.normal(0.0, self.config.gyro_noise_std))

        # 3. Corrupt Wheel Speed Sensors (WSS)
        w_true = np.asarray(true_omega, dtype=float)
        w_noisy = w_true + self.rng.normal(0.0, self.config.wheel_noise_std, size=4)

        if self.config.quantize_wheel_speed and self.config.encoder_ticks_per_rev > 0:
            # Quantize speed based on ticks per revolution in discrete time
            rad_per_tick = 2.0 * math.pi / self.config.encoder_ticks_per_rev
            w_meas = np.round(w_noisy / rad_per_tick) * rad_per_tick
        else:
            w_meas = w_noisy

        # 4. Corrupt Steering Angle Sensor
        sf_noisy = true_steer_f + float(self.rng.normal(0.0, self.config.steer_noise_std))
        q_rad = self.config.steer_quantization_rad
        if q_rad > 0.0:
            sf_meas = round(sf_noisy / q_rad) * q_rad
        else:
            sf_meas = sf_noisy

        sr_noisy = true_steer_r + float(self.rng.normal(0.0, self.config.steer_noise_std))
        if q_rad > 0.0:
            sr_meas = round(sr_noisy / q_rad) * q_rad
        else:
            sr_meas = sr_noisy

        return SensorMeasurements(
            t=t,
            ax=ax_meas,
            ay=ay_meas,
            r=r_meas,
            omega=w_meas,
            steer_f=sf_meas,
            steer_r=sr_meas,
        )


__all__ = ("SensorConfig", "SensorMeasurements", "SensorSuite")
