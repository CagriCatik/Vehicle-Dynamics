"""Road-friction estimation from virtual sensors only (G02 / state estimation Mode C).

Information used (``information = "sensors_only"``): the sensor measurements
(IMU accelerations, yaw rate, wheel speeds, steering angle), the controller's
own commanded wheel torques, and nominal vehicle data (mass, geometry, wheel
inertia, a nominal brush tyre). No tyre forces, slips, wheel loads or road
friction from the simulation.

Per step and wheel:

1. **Vehicle speed reference** ``v_ref``: integrated measured ``a_x``, corrected one-sidedly
   towards the wheel speeds: without drive torque the vehicle is at least as fast as the
   fastest wheel (upward correction only; downward only after ``free_rolling_time`` without
   any wheel torque, when the wheels roll freely); with drive torque at most as fast as
   the slowest wheel. During sustained all-wheel braking the speed is therefore only
   bounded from below and relies on the integrated ``a_x`` (a physical observability
   limit: no wheel rolls freely). It is
   initialised from freely rolling wheels: ``reset(init_vx)`` before a manoeuvre, or the
   first step without wheel torque (with braking from the first step the wheels already
   slip and would start the reference several m/s too low).
2. **Longitudinal tyre force** from the wheel dynamics,
   ``F_x = (T_drive - T_brake sgn(omega) - I_w omega_dot - d omega) / R``, with ``omega_dot``
   from a first-order filtered derivative of the measured wheel speed.
3. **Wheel loads** quasi-statically from the measured ``a_x``, ``a_y`` and the geometry.
4. **Axle lateral forces** from ``m a_y = F_yf + F_yr`` and ``I_z r_dot = l_f F_yf - l_r F_yr``
   (filtered derivative of the measured yaw rate), split left/right by wheel load.
5. **Slips** ``kappa = (omega R - v_wx)/v_wx`` and ``alpha`` from ``v_ref``, the yaw rate,
   the measured steering angle and an external sideslip estimate (optional).
6. **Friction** by inverting a nominal combined-slip brush (Fiala) model: with the
   theoretical slip ``psi = |(C_k s_x, C_a s_y)|``,
   ``F(mu) = psi - psi^2/(3 mu Fz) + psi^3/(27 mu^2 Fz^2)`` (``mu Fz`` beyond ``psi = 3 mu Fz``),
   the measured force magnitude gives ``mu``.

Identifiability: in the linear tyre range ``F ~ psi`` regardless of ``mu``, so the
force carries no information on ``mu`` (``dF/dmu ~ 0``). Each update is weighted by
the normalised sensitivity ``s = (mu / Fz) dF/dmu`` and only used if ``s``, the
utilisation ``|F|/Fz`` and the theoretical slip ``psi/Fz`` exceed their thresholds.
The last one matters: with little slip, wheel-speed noise inflates ``psi`` relative
to the force and biases ``mu`` low (seen on the hardly braked high-friction front
wheel on split mu). The estimate never drops below the observed utilisation
``|F| / Fz`` (friction is at least what is being used). Without excitation the
estimate keeps its prior and ``confidence`` stays low: it does not claim a value.

Limits: nominal brush tyre (mismatch with the plant tyre is intended); quasi-static
loads (no suspension dynamics); the lateral-force split assumes forces proportional
to load; no bank or slope compensation; wheel-speed quantisation limits ``omega_dot``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from vehicle_model.core.types import FL, FR, RL, RR, SevenDOFParams
from vehicle_model.vehicle_models.normal_loads import quasi_static_normal_loads, wheel_contact_positions

from .friction_estimator import FrictionEstimate
from .sensors import SensorMeasurements


@dataclass(frozen=True)
class SensorFrictionConfig:
    c_kappa: float = 18.0               # [-] nominal brush longitudinal stiffness per unit load
    c_alpha: float = 16.0               # [1/rad] nominal brush cornering stiffness per unit load
    mu_min: float = 0.05
    mu_max: float = 1.5
    initial_mu: float = 1.0             # prior until the road has been identified
    min_sensitivity: float = 0.05       # [-] below: mu not identifiable from this sample
    forgetting: float = 0.995           # [-] per step, lets the estimate follow road changes
    derivative_tau: float = 0.03        # [s] filter of the wheel-speed and yaw-rate derivatives
    speed_gain: float = 5.0             # [1/s] correction towards free-rolling wheels
    bound_gain: float = 20.0            # [1/s] enforcement of the wheel-speed bounds (vehicle >= fastest / <= slowest)
    free_rolling_time: float = 0.1      # [s] without wheel torque before wheels count as free rolling
    min_utilisation: float = 0.1        # [-] |F|/Fz below: force within the noise, no update
    min_slip_excitation: float = 0.25   # [-] psi/Fz below: slip within the wheel-speed noise, no update
    min_speed: float = 3.0              # [m/s] no estimation below


def brush_force(psi, mu_fz):
    """Combined-slip brush (Fiala) force magnitude for theoretical slip ``psi`` [N] and limit ``mu Fz``."""
    q = psi / (3.0 * mu_fz)
    return np.where(q < 1.0, psi - psi * q + psi * q * q / 3.0, mu_fz)


class SensorFrictionEstimator:
    """Per-wheel road-friction estimate with identifiability weighting (module docstring)."""

    information = "sensors_only"

    def __init__(self, params: SevenDOFParams, config: SensorFrictionConfig | None = None):
        self.params = params
        self.config = config or SensorFrictionConfig()
        self.positions = wheel_contact_positions(params)
        self.reset()

    def reset(self, init_vx: float | None = None) -> None:
        c = self.config
        self.v_ref = init_vx
        self.mu = np.full(4, c.initial_mu)
        self.weight = np.zeros(4)           # accumulated information per wheel
        self.utilisation = np.zeros(4)
        self._omega_prev = None
        self._r_prev = None
        self._no_torque_time = 0.0
        self._omega_dot = np.zeros(4)
        self._r_dot = 0.0
        self.last_sensitivity = np.zeros(4)
        self.last_forces = (np.zeros(4), np.zeros(4), np.zeros(4))
        self.last_slips = (np.zeros(4), np.zeros(4))
        self.last_wheel_velocity = (np.zeros(4), np.zeros(4))
        self.valid = False                    # True once a step with a speed reference has been processed

    # ------------------------------------------------------------------ pieces
    def _derivatives(self, meas: SensorMeasurements, dt: float) -> None:
        a = dt / (self.config.derivative_tau + dt)
        if self._omega_prev is not None:
            self._omega_dot += a * ((meas.omega - self._omega_prev) / dt - self._omega_dot)
            self._r_dot += a * ((meas.r - self._r_prev) / dt - self._r_dot)
        self._omega_prev, self._r_prev = meas.omega.copy(), meas.r

    def _speed_reference(self, meas: SensorMeasurements, braking: bool, driving: bool, dt: float) -> float:
        p, c = self.params, self.config
        v_wheels = meas.omega * p.Reff + meas.r * self.positions[:, 1]   # wheel speed at the CoG line
        if self.v_ref is None:
            if braking or driving:
                return None                      # not initialised: wheels under torque are slipping
            self.v_ref = float(np.mean(v_wheels))
        self.v_ref += meas.ax * dt
        self._no_torque_time = 0.0 if (braking or driving) else self._no_torque_time + dt
        if driving:
            target = float(np.min(v_wheels))      # vehicle <= slowest (driven) wheel
            if target < self.v_ref:
                self.v_ref += c.bound_gain * (target - self.v_ref) * dt
        else:
            target = float(np.max(v_wheels))      # vehicle >= fastest wheel
            if target > self.v_ref:
                self.v_ref += c.bound_gain * (target - self.v_ref) * dt
            elif self._no_torque_time >= c.free_rolling_time:
                self.v_ref += c.speed_gain * (target - self.v_ref) * dt
        return self.v_ref

    def _forces(self, meas, T_drive, T_brake):
        p = self.params
        Fx = (T_drive - T_brake * np.sign(meas.omega) - p.Iw * self._omega_dot - p.wheel_damping * meas.omega) / p.Reff
        Fz = quasi_static_normal_loads(p, meas.ax, meas.ay).Fz
        L = p.wheelbase
        Fy_f = p.mass * meas.ay * p.lr / L + p.Iz * self._r_dot / L
        Fy_r = p.mass * meas.ay * p.lf / L - p.Iz * self._r_dot / L
        Fy = np.array([Fy_f * Fz[FL] / (Fz[FL] + Fz[FR]), Fy_f * Fz[FR] / (Fz[FL] + Fz[FR]),
                       Fy_r * Fz[RL] / (Fz[RL] + Fz[RR]), Fy_r * Fz[RR] / (Fz[RL] + Fz[RR])])
        return Fx, Fy, np.maximum(Fz, 1.0)

    def _slips(self, meas, v_ref, vy):
        pos = self.positions
        steer = np.array([meas.steer_f, meas.steer_f, meas.steer_r, meas.steer_r])
        vxb = v_ref - meas.r * pos[:, 1]
        vyb = vy + meas.r * pos[:, 0]
        v_wx = vxb * np.cos(steer) + vyb * np.sin(steer)
        v_wy = -vxb * np.sin(steer) + vyb * np.cos(steer)
        kappa = (meas.omega * self.params.Reff - v_wx) / np.maximum(np.abs(v_wx), 0.5)
        alpha = -np.arctan2(v_wy, np.maximum(np.abs(v_wx), 0.5))
        self.last_wheel_velocity = (v_wx, v_wy)
        return kappa, alpha

    def _invert(self, F, psi, Fz):
        """``mu`` with brush_force(psi, mu Fz) = F (bisection) and the normalised sensitivity there."""
        c = self.config
        lo, hi = np.full(4, c.mu_min), np.full(4, c.mu_max)
        for _ in range(40):
            mid = 0.5 * (lo + hi)
            too_small = brush_force(psi, mid * Fz) < F       # force grows with mu
            lo, hi = np.where(too_small, mid, lo), np.where(too_small, hi, mid)
        mu = 0.5 * (lo + hi)
        h = 1e-3
        dF = (brush_force(psi, (mu + h) * Fz) - brush_force(psi, (mu - h) * Fz)) / (2 * h)
        return mu, mu * dF / Fz

    # ------------------------------------------------------------------ update
    def update(self, meas: SensorMeasurements, T_drive, T_brake, dt: float, vy: float = 0.0) -> FrictionEstimate:
        """One step; ``T_drive``, ``T_brake`` are the commanded wheel torques, ``vy`` a sideslip-observer estimate."""
        c = self.config
        T_drive, T_brake = np.asarray(T_drive, float), np.asarray(T_brake, float)
        self._derivatives(meas, dt)
        braking, driving = bool(np.any(T_brake > 1.0)), bool(np.any(T_drive > 1.0) and not np.any(T_brake > 1.0))
        v_ref = self._speed_reference(meas, braking, driving, dt)
        self.weight *= c.forgetting
        if v_ref is None or v_ref < c.min_speed:
            return self._estimate()
        Fx, Fy, Fz = self._forces(meas, T_drive, T_brake)
        kappa, alpha = self._slips(meas, v_ref, vy)
        F = np.hypot(Fx, Fy)
        self.utilisation = F / Fz
        sx = kappa / np.maximum(1.0 + kappa, 1e-3)
        sy = np.tan(alpha) / np.maximum(1.0 + kappa, 1e-3)
        psi = np.hypot(c.c_kappa * Fz * sx, c.c_alpha * Fz * sy)
        mu_sample, sens = self._invert(F, psi, Fz)
        self.last_sensitivity = sens
        self.last_forces = (Fx, Fy, Fz)
        self.last_slips = (kappa, alpha)
        self.valid = True
        use = (sens > c.min_sensitivity) & (self.utilisation > c.min_utilisation) & (psi / Fz > c.min_slip_excitation)
        w = np.where(use, sens, 0.0)
        new_weight = self.weight + w
        self.mu = np.where(use, (self.weight * self.mu + w * mu_sample) / np.maximum(new_weight, 1e-12), self.mu)
        self.weight = new_weight
        self.mu = np.clip(np.maximum(self.mu, np.minimum(self.utilisation, c.mu_max)), c.mu_min, c.mu_max)
        return self._estimate()

    def as_telemetry(self, meas: SensorMeasurements):
        """Wheel quantities for allocation and slip control, from sensors and this estimator only (Mode C).

        ``None`` until a speed reference exists (consumers then fall back to nominal loads
        and the target's friction estimate). Approximations, declared: wheel loads
        quasi-static from measured accelerations; longitudinal forces from the wheel
        dynamics and the *commanded* torques; lateral forces are the axle forces of the
        single-track balance split by load and used as wheel-frame forces; slips from
        the speed reference and the sideslip estimate; friction is the estimate (prior
        until excited, see ``confidence_per_wheel``).
        """
        from vehicle_model.core.types import WheelTelemetry

        if not self.valid:
            return None
        Fx, Fy, Fz = self.last_forces
        kappa, alpha = self.last_slips
        v_wx, v_wy = self.last_wheel_velocity
        steer = np.array([meas.steer_f, meas.steer_f, meas.steer_r, meas.steer_r])
        c, s = np.cos(steer), np.sin(steer)
        return WheelTelemetry(omega=meas.omega.copy(), v_wx=v_wx.copy(), v_wy=v_wy.copy(), kappa=kappa.copy(),
                              alpha=alpha.copy(), Fz=Fz.copy(), mu=self.mu.copy(), Fx_wheel=Fx.copy(), Fy_wheel=Fy.copy(),
                              Fx_body=c * Fx - s * Fy, Fy_body=s * Fx + c * Fy, steer_angles=steer)

    def _estimate(self) -> FrictionEstimate:
        conf = 1.0 - np.exp(-self.weight)          # 0 without information, -> 1 with accumulated excitation
        w = np.maximum(conf, 1e-6)
        side = lambda i, j: float((w[i] * self.mu[i] + w[j] * self.mu[j]) / (w[i] + w[j]))   # confidence-weighted
        mu_left, mu_right = side(FL, RL), side(FR, RR)
        left_known, right_known = max(conf[FL], conf[RL]) > 0.5, max(conf[FR], conf[RR]) > 0.5
        return FrictionEstimate(mu_overall=float(np.sum(w * self.mu) / np.sum(w)), mu_wheels=self.mu.copy(),
                                mu_left=mu_left, mu_right=mu_right,
                                is_split_mu=bool(abs(mu_left - mu_right) >= 0.25 and left_known and right_known),
                                confidence=float(np.mean(conf)))

    def confidence_per_wheel(self) -> np.ndarray:
        return 1.0 - np.exp(-self.weight)


__all__ = ("SensorFrictionConfig", "SensorFrictionEstimator", "brush_force")
