"""Extended Kalman Filter (EKF) sideslip angle and chassis state observer.

Implements Plan.md §11 (Milestone M6):
Fuses IMU acceleration (ax, ay), gyro yaw rate (r), wheel speed encoders,
and steering angle to estimate longitudinal speed vx, lateral speed vy,
sideslip angle beta = arctan(vy/vx), and sensor biases without accessing
unobservable ground-truth simulation variables.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Any
import numpy as np

from vehicle_model.core.types import SevenDOFParams
from vehicle_model.estimation.sensors import SensorMeasurements


@dataclass
class EKFConfig:
    """Tuning covariance parameters for the EKF sideslip observer."""

    # Process noise variances Q
    q_vx: float = 0.10
    q_vy: float = 0.25
    q_r: float = 0.05
    q_bias_ay: float = 0.0005
    q_bias_r: float = 0.00005

    # Measurement noise variances R
    r_vx: float = 0.25
    r_yaw_rate: float = 0.002
    r_ay: float = 0.10

    # Initial covariance P0
    p_vx: float = 1.0
    p_vy: float = 1.0
    p_r: float = 0.1
    p_bias_ay: float = 0.1
    p_bias_r: float = 0.01

    min_speed_for_update: float = 1.0  # [m/s] Below this, kinematics are singular

    @classmethod
    def from_yaml(cls, path: str | Path | None = None) -> EKFConfig:
        """Load EKF tuning parameters from centralized YAML."""
        import yaml
        if path is None:
            from vehicle_model.config.loader import get_configs_root
            p = get_configs_root() / "estimators" / "ekf.yaml"
        else:
            p = Path(path)

        with open(p, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}

        pn = data.get("process_noise", {})
        mn = data.get("measurement_noise", {})
        ic = data.get("initial_covariance", {})
        fs = data.get("filter_settings", {})

        return cls(
            q_vx=float(pn.get("q_vx", 0.10)),
            q_vy=float(pn.get("q_vy", 0.25)),
            q_r=float(pn.get("q_r", 0.05)),
            q_bias_ay=float(pn.get("q_bias_ay", 0.0005)),
            q_bias_r=float(pn.get("q_bias_r", 0.00005)),
            r_vx=float(mn.get("r_vx", 0.25)),
            r_yaw_rate=float(mn.get("r_yaw_rate", 0.002)),
            r_ay=float(mn.get("r_ay", 0.10)),
            p_vx=float(ic.get("p_vx", 1.0)),
            p_vy=float(ic.get("p_vy", 1.0)),
            p_r=float(ic.get("p_r", 0.1)),
            p_bias_ay=float(ic.get("p_bias_ay", 0.1)),
            p_bias_r=float(ic.get("p_bias_r", 0.01)),
            min_speed_for_update=float(fs.get("min_speed_for_update", 1.0)),
        )


@dataclass
class EstimatedState:
    """State estimate emitted by the EKF observer."""

    vx: float                      # Estimated longitudinal velocity [m/s]
    vy: float                      # Estimated lateral velocity [m/s]
    r: float                       # Estimated yaw rate [rad/s]
    sideslip: float                # Estimated chassis sideslip angle beta [rad]
    sideslip_deg: float            # Estimated sideslip angle beta [deg]
    bias_ay: float                 # Estimated accelerometer bias [m/s^2]
    bias_r: float                  # Estimated gyro bias [rad/s]
    covariance_diag: np.ndarray    # Diagonal variances of the state covariance matrix P


class EKFSideslipObserver:
    """Nonlinear Extended Kalman Filter for vehicle sideslip and body velocity estimation.

    States x = [vx, vy, r, bias_ay, bias_r]^T (dim 5)
    Measurements z = [vx_ref, r_meas, ay_meas]^T (dim 3)
    """

    def __init__(
        self,
        params: SevenDOFParams,
        config: EKFConfig | None = None,
        cornering_stiffness_f: float = 85000.0,
        cornering_stiffness_r: float = 95000.0,
    ):
        self.params = params
        self.config = config or EKFConfig()
        self.Caf = cornering_stiffness_f  # Front axle cornering stiffness [N/rad]
        self.Car = cornering_stiffness_r  # Rear axle cornering stiffness [N/rad]

        # State vector: [vx, vy, r, bias_ay, bias_r]
        self.x = np.zeros(5, dtype=float)

        # Covariance matrix P (5x5)
        self.P = np.diag([
            self.config.p_vx,
            self.config.p_vy,
            self.config.p_r,
            self.config.p_bias_ay,
            self.config.p_bias_r,
        ])

        # Process noise covariance Q (5x5)
        self.Q = np.diag([
            self.config.q_vx,
            self.config.q_vy,
            self.config.q_r,
            self.config.q_bias_ay,
            self.config.q_bias_r,
        ])

        # Measurement noise covariance R (3x3)
        self.R = np.diag([
            self.config.r_vx,
            self.config.r_yaw_rate,
            self.config.r_ay,
        ])

    def reset(self, init_vx: float = 0.0, init_vy: float = 0.0, init_r: float = 0.0) -> None:
        """Reset estimator state and covariance."""
        self.x = np.array([init_vx, init_vy, init_r, 0.0, 0.0], dtype=float)
        self.P = np.diag([
            self.config.p_vx,
            self.config.p_vy,
            self.config.p_r,
            self.config.p_bias_ay,
            self.config.p_bias_r,
        ])

    def update(
        self,
        meas: SensorMeasurements,
        dt: float,
    ) -> EstimatedState:
        """Execute one EKF Predict and Update step.

        Parameters:
            meas: SensorMeasurements from onboard chassis sensors
            dt: Sample time step [s]

        Returns:
            EstimatedState with updated vehicle velocity and sideslip angle.
        """
        vx, vy, r, b_ay, b_r = self.x
        mass = self.params.mass
        lf = self.params.lf
        lr = self.params.lr

        # ── 1. PREDICT STEP (Nonlinear Dynamic Tyre Force Propagation) ──
        vx_hat = max(self.config.min_speed_for_update, vx)
        alpha_f = meas.steer_f - (vy + lf * r) / vx_hat
        alpha_r = meas.steer_r - (vy - lr * r) / vx_hat

        Fyf = self.Caf * alpha_f
        Fyr = self.Car * alpha_r
        Iz = self.params.Iz

        # Equations of motion
        vx_dot = meas.ax + r * vy
        vy_dot = (Fyf + Fyr) / mass - r * vx
        r_dot = (Fyf * lf - Fyr * lr) / Iz

        # State propagation (Euler integration step)
        vx_new = max(0.1, vx + vx_dot * dt)
        vy_new = vy + vy_dot * dt
        r_new = r + r_dot * dt
        b_ay_new = b_ay
        b_r_new = b_r

        self.x = np.array([vx_new, vy_new, r_new, b_ay_new, b_r_new], dtype=float)

        # Process Jacobian F = df / dx
        C1 = (self.Caf + self.Car) / (mass * vx_hat)
        C2 = (self.Caf * lf - self.Car * lr) / (mass * vx_hat)
        D1 = (self.Caf * lf - self.Car * lr) / (Iz * vx_hat)
        D2 = (self.Caf * (lf ** 2) + self.Car * (lr ** 2)) / (Iz * vx_hat)

        F = np.zeros((5, 5), dtype=float)
        F[0, 1] = r
        F[0, 2] = vy
        F[1, 1] = -C1
        F[1, 2] = -C2 - vx
        F[2, 1] = -D1
        F[2, 2] = -D2

        # State transition matrix Phi = I + F * dt
        Phi = np.eye(5) + F * dt
        self.P = Phi @ self.P @ Phi.T + self.Q * dt

        # ── 2. MEASUREMENT UPDATE STEP ──
        # Synthesize reference longitudinal speed from average wheel speed
        w_speeds = np.sort(meas.omega)
        vx_ref = float(np.mean(w_speeds[1:3]) * self.params.Reff)
        vx_ref = max(0.1, vx_ref)

        # Expected measurements h(x):
        # h1: vx
        # h2: r + b_r
        # h3: lateral acceleration ay_pred = (Fyf + Fyr) / mass + b_ay
        alpha_f_hat = meas.steer_f - (self.x[1] + lf * self.x[2]) / max(self.config.min_speed_for_update, self.x[0])
        alpha_r_hat = meas.steer_r - (self.x[1] - lr * self.x[2]) / max(self.config.min_speed_for_update, self.x[0])
        Fyf_hat = self.Caf * alpha_f_hat
        Fyr_hat = self.Car * alpha_r_hat
        ay_pred = (Fyf_hat + Fyr_hat) / mass + self.x[3]

        z_pred = np.array([
            self.x[0],
            self.x[2] + self.x[4],
            ay_pred,
        ], dtype=float)

        z_meas = np.array([
            vx_ref,
            meas.r,
            meas.ay,
        ], dtype=float)

        # Innovation
        y = z_meas - z_pred

        # Measurement Jacobian H = dh / dx (3x5)
        H = np.zeros((3, 5), dtype=float)
        H[0, 0] = 1.0   # dh1/dvx
        H[1, 2] = 1.0   # dh2/dr
        H[1, 4] = 1.0   # dh2/db_r

        H[2, 1] = -C1   # dh3/dvy
        H[2, 2] = -C2   # dh3/dr
        H[2, 3] = 1.0   # dh3/db_ay

        # Innovation covariance S = H P H^T + R
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)     # Kalman gain (5x3)

        # State and covariance update
        self.x = self.x + K @ y
        self.P = (np.eye(5) - K @ H) @ self.P

        # Enforce physically positive longitudinal velocity
        self.x[0] = max(0.01, self.x[0])

        # Compute estimated sideslip angle beta
        est_vx = self.x[0]
        est_vy = self.x[1]
        est_r = self.x[2]
        sideslip = math.atan2(est_vy, est_vx)

        return EstimatedState(
            vx=est_vx,
            vy=est_vy,
            r=est_r,
            sideslip=sideslip,
            sideslip_deg=math.degrees(sideslip),
            bias_ay=self.x[3],
            bias_r=self.x[4],
            covariance_diag=np.diag(self.P).copy(),
        )


__all__ = ("EKFConfig", "EstimatedState", "EKFSideslipObserver")
