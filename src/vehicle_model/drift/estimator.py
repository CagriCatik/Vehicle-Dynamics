"""Drift-capable velocity observer from sensors only (follow-up of Drift plan D7).

Why a separate observer: the general sideslip observers (``estimation.blended_sideslip``)
take the longitudinal speed from the two middle wheel speeds and start at ``beta = 0``. In a
rear-wheel-drive drift the inner rear wheel spins (kappa ~ 0.9): the speed reference rises to
about twice the true speed, and the controller, which starts in the drift, is fed a sideslip of
0 instead of -20 deg. D7 measured the result: drift lost in all estimated runs.

This observer uses only what stays valid in a drift:

* the IMU kinematics in the body frame (process model, no tyre model)::

      vx_dot = ax,m + r_m vy,     vy_dot = ay,m - r_m vx

* the two **front** wheel speeds (no drive and no brake torque in the drift controller, so the
  front wheels roll freely, kappa ~ 0). A free-rolling wheel measures the velocity of its centre
  along the steered wheel heading::

      omega_i R = (vx -+ r w_f / 2) cos(delta) + (vy + l_f r) sin(delta)        (FL: -, FR: +)

  i.e. ``y_i = omega_i R +- r w_f/2 cos(delta) - l_f r sin(delta) = [cos delta, sin delta] [vx, vy]``.

One wheel-speed direction alone does not give both velocity components; the rotation does:
with ``r != 0`` the kinematics turn a ``vy`` error into a ``vx`` error, which the wheels see
(the pair is observable whenever ``r != 0`` or ``delta != 0``). A time-varying Kalman filter
(discrete, linearised exactly: the model is linear in ``[vx, vy]`` for given ``r``) does the
fusion. Initialisation from the first front wheel measurement with ``vy = 0`` and a large
``vy`` variance: no assumed sideslip, the filter converges at a rate set by ``r``.

Limits: front-wheel slip (braking, front drive, or a locked wheel) biases the measurement; in
straight driving (``r ~ 0``, ``delta ~ 0``) ``vy`` is not observable and follows the integrated
accelerometer bias; the accelerometer and gyro biases are not estimated (an ``ay`` bias ``b``
leaves a ``vx`` error of about ``b / r``). Planar motion only (no bank, no pitch).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from vehicle_model.estimation.ekf_sideslip import EstimatedState


@dataclass(frozen=True)
class DriftObserverConfig:
    accel_noise_std: float = 0.5      # [m/s^2] process noise of the kinematics (sensor noise, unmodelled terms)
    wheel_speed_std: float = 0.10     # [m/s] per front wheel (wheel noise 0.2 rad/s x R, plus residual slip)
    init_vx_std: float = 0.5          # [m/s]
    init_vy_std: float = 6.0          # [m/s] no prior on the sideslip
    min_speed: float = 1.0            # [m/s]


class DriftVelocityObserver:
    """Kalman filter on ``[vx, vy]``: IMU kinematics, corrected by the free-rolling front wheels."""

    information = "sensors_only"

    def __init__(self, params, config: DriftObserverConfig | None = None):
        self.params = params
        self.config = config or DriftObserverConfig()
        self.reset()

    def reset(self, init_vx: float = 0.0, init_vy: float = 0.0) -> None:
        # init_vx from the caller is not used: it comes from the middle wheel speeds (spinning rear
        # wheel in a drift); the filter initialises itself from the front wheels on the first update
        self.x = None
        self.P = None

    def _front(self, meas):
        p, d = self.params, float(meas.steer_f)
        c, s = math.cos(d), math.sin(d)
        y = np.array([meas.omega[0] * p.Reff + meas.r * p.wf / 2.0 * c - p.lf * meas.r * s,
                      meas.omega[1] * p.Reff - meas.r * p.wf / 2.0 * c - p.lf * meas.r * s])
        return y, np.array([[c, s], [c, s]])

    def update(self, meas, dt: float) -> EstimatedState:
        cfg = self.config
        y, H = self._front(meas)
        if self.x is None:
            self.x = np.array([float(np.mean(y)) / max(H[0, 0], 0.5), 0.0])
            self.P = np.diag([cfg.init_vx_std**2, cfg.init_vy_std**2])
        else:
            r = float(meas.r)
            F = np.array([[1.0, r * dt], [-r * dt, 1.0]])
            self.x = F @ self.x + np.array([meas.ax, meas.ay]) * dt
            self.P = F @ self.P @ F.T + np.eye(2) * (cfg.accel_noise_std * dt) ** 2
        R = np.eye(2) * cfg.wheel_speed_std**2
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.pinv(S)
        self.x = self.x + K @ (y - H @ self.x)
        self.P = (np.eye(2) - K @ H) @ self.P
        vx = max(float(self.x[0]), cfg.min_speed)
        vy = float(self.x[1])
        beta = math.atan2(vy, vx)
        return EstimatedState(vx=vx, vy=vy, r=float(meas.r), sideslip=beta, sideslip_deg=math.degrees(beta),
                              bias_ay=0.0, bias_r=0.0, covariance_diag=np.r_[np.diag(self.P), 0.0, 0.0, 0.0])


__all__ = ("DriftObserverConfig", "DriftVelocityObserver")
