"""Sideslip observer blending kinematic integration with a model-based estimate (Plan.md, Abschnitt 11; M7).

Two classic sideslip observers fail in complementary ranges:

* **model-based** (here the EKF of :mod:`.ekf_sideslip` with linear tyres): accurate
  while the tyres are linear, wrong at the limit (its tyre forces keep growing);
* **kinematic** (``v_y_dot = a_y - r v_x`` from the IMU): independent of tyre
  models, so it works at the limit, but integrates sensor bias and drifts.

The blended observer integrates the kinematic equation and pulls it towards the
model-based estimate only while the vehicle is in its linear range:

    v_y_hat_dot = a_y,m - r_m v_x,m + K w (v_y,EKF - v_y_hat)
    w = exp(-(|a_y,m| / a_lin)^4) * exp(-((|r_m - r_lin| / max(|r_lin|, r_floor)) / r_tol)^2)
    r_lin = v_x,m delta_m / (L + EG v_x,m^2)                  (linear yaw response to the measured steer)

Inputs: sensor measurements only (``information = "sensors_only"``); ``v_x,m`` from
the mean of the two middle wheel speeds.

Tuning (sine with dwell 80 km/h, reference car, MF tyres, default sensor noise;
RMS / max sideslip error in degrees):

    case                      EKF          kinematic    blended K=2, r_tol=0.3
    4 deg, mu 1.0 (linear)    0.23 / 0.71  1.08 / 1.95  0.46 / 0.70
    6 deg, mu 1.0             1.31 / 4.0   1.13 / 2.03  0.63 / 0.99
    4 deg, mu 0.5 (sliding)   24.8 / 51    0.96 / 1.85  1.18 / 2.16
    8 deg, mu 1.0 (spinning)  29.7 / 53    1.26 / 4.69  1.19 / 4.38

Larger K improves the linear case (K=10: 0.27) but lets the failing model leak in at
the limit (K=10: 1.81 on mu 0.5); K=2 has the best worst case.

Limits: the linearity weight uses fixed thresholds (no friction estimate); after a
long excursion beyond the linear range the kinematic part carries the integrated
bias until the vehicle is linear again; no estimate of the road bank or slope.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from vehicle_model.core.types import SevenDOFParams

from .ekf_sideslip import EKFSideslipObserver, EstimatedState
from .sensors import SensorMeasurements


@dataclass(frozen=True)
class BlendedObserverConfig:
    gain: float = 2.0                 # [1/s] K, pull towards the model estimate in the linear range
    a_lin: float = 4.0                # [m/s^2] lateral acceleration scale of the linear range
    r_tol: float = 0.3                # [-] tolerated relative yaw-rate deviation from the linear response
    r_floor: float = 0.05             # [rad/s] floor of the relative yaw-rate deviation
    understeer_gradient: float = 0.0009  # [rad/(m/s^2)] of the linear yaw-rate reference (~0.5 deg/g)
    min_speed: float = 1.0            # [m/s]


class KinematicSideslipObserver:
    """Pure kinematic integration ``v_y_dot = a_y,m - r_m v_x,m`` (deterministic baseline; drifts with bias)."""

    information = "sensors_only"

    def __init__(self, params: SevenDOFParams, min_speed: float = 1.0):
        self.params = params
        self.min_speed = min_speed
        self.reset()

    def reset(self, init_vx: float = 0.0, init_vy: float = 0.0) -> None:
        self.vy = float(init_vy)

    def update(self, meas: SensorMeasurements, dt: float) -> EstimatedState:
        vx = max(float(np.mean(np.sort(meas.omega)[1:3])) * self.params.Reff, self.min_speed)
        self.vy += (meas.ay - meas.r * vx) * dt
        beta = math.atan2(self.vy, vx)
        return EstimatedState(vx=vx, vy=self.vy, r=meas.r, sideslip=beta, sideslip_deg=math.degrees(beta),
                              bias_ay=0.0, bias_r=0.0, covariance_diag=np.zeros(5))


class BlendedSideslipObserver:
    """Kinematic sideslip integration corrected by an EKF in the linear range."""

    information = "sensors_only"

    def __init__(self, params: SevenDOFParams, config: BlendedObserverConfig | None = None,
                 model_observer: EKFSideslipObserver | None = None):
        self.params = params
        self.config = config or BlendedObserverConfig()
        self.model = model_observer or EKFSideslipObserver(params)
        self.reset()

    def reset(self, init_vx: float = 0.0, init_vy: float = 0.0) -> None:
        self.model.reset(init_vx=init_vx, init_vy=init_vy)
        self.vy = float(init_vy)
        self.weight = 1.0

    def update(self, meas: SensorMeasurements, dt: float) -> EstimatedState:
        c, p = self.config, self.params
        model = self.model.update(meas, dt)
        vx = max(float(np.mean(np.sort(meas.omega)[1:3])) * p.Reff, c.min_speed)
        r_lin = vx * meas.steer_f / (p.wheelbase + c.understeer_gradient * vx**2)
        i_ay = abs(meas.ay) / c.a_lin
        i_r = abs(meas.r - r_lin) / max(abs(r_lin), c.r_floor)
        self.weight = math.exp(-i_ay**4) * math.exp(-(i_r / c.r_tol) ** 2)
        self.vy += (meas.ay - meas.r * vx + c.gain * self.weight * (model.vy - self.vy)) * dt
        beta = math.atan2(self.vy, vx)
        return EstimatedState(vx=vx, vy=self.vy, r=meas.r, sideslip=beta, sideslip_deg=math.degrees(beta),
                              bias_ay=model.bias_ay, bias_r=model.bias_r, covariance_diag=model.covariance_diag)


__all__ = ("BlendedObserverConfig", "BlendedSideslipObserver", "KinematicSideslipObserver")
