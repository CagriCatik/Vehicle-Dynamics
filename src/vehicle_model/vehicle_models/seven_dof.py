"""Seven Degree-of-Freedom (7-DOF) two-track vehicle dynamics model.

Dynamic degrees of freedom (7):
  vx, vy, r                      chassis longitudinal/lateral velocity [m/s], yaw rate [rad/s]
  omega_FL, omega_FR, omega_RL, omega_RR   wheel spin rates [rad/s]
Navigation states (3, integrated, not extra DOFs):
  X, Y [m], psi [rad]

Modelling assumptions (documented in docs/VEHICLE_MODELS.md, section 2):
  * Planar motion on a level road, ISO 8855 axes (x forward, y left, z up).
  * Quasi-static longitudinal/lateral load transfer; no suspension dynamics.
    The algebraic loop ``ax, ay -> Fz -> tyre forces -> ax, ay`` is solved by
    fixed-point iteration, so the derivative is a pure function of the state.
    Non-convergence is reported in the telemetry (``load_transfer_converged``).
  * Unilateral wheel contact: a lifting wheel carries zero load and the chassis
    rests on three wheels with exact force/moment balance; tip-over is flagged
    (see :mod:`vehicle_model.vehicle_models.normal_loads`).
  * Steering angles are inputs (no steering actuator dynamics here). Left and
    right wheels share the axle angle unless a ``chassis`` (quasi-static K&C,
    :mod:`vehicle_model.suspension.kinematics`) adds Ackermann geometry, bump
    and compliance steer and roll-centre load transfer; without it the model
    is unchanged.
  * Tyre forces come from any :class:`~vehicle_model.tire_model.TyreModel`.
  * Rolling resistance and aerodynamic drag act on the chassis.
  * Brake torque opposes wheel rotation via ``tanh(omega/omega_eps)``; static
    brake hold at standstill is not modelled.
"""

from __future__ import annotations

import dataclasses
import math

import numpy as np

from vehicle_model.core.types import (
    FL,
    FR,
    RL,
    RR,
    SevenDOFControlInput,
    SevenDOFParams,
    SevenDOFState,
    WheelTelemetry,
)
from vehicle_model.roads.surface import RoadSurface, UniformRoad
from vehicle_model.steering.geometry import wheel_steer_angles
from vehicle_model.vehicle_models.normal_loads import (
    NormalLoads,
    quasi_static_normal_loads,
    wheel_contact_positions,
)
from vehicle_model.tire_model.combined_slip import SimplifiedCombinedSlipTyre
from vehicle_model.tire_model.framework import TyreModel

_V_EPS = 0.20          # [m/s] slip regularisation speed
_OMEGA_EPS = 0.50      # [rad/s] brake-torque sign smoothing
_LT_TOL = 1e-6         # [m/s^2] load-transfer fixed-point tolerance
_LT_MAX_ITER = 25
_RK4_STABILITY = 2.0   # safety-reduced RK4 stability bound (exact: ~2.78) for lambda*dt


def _vehicle_path(path: str | None):
    """Vehicle YAML path (same resolution as ``SevenDOFParams.from_yaml``)."""
    from pathlib import Path

    from vehicle_model.config.loader import get_configs_root

    return Path(path) if path is not None else get_configs_root() / "vehicles" / "seven_dof_passenger.yaml"


def _vehicle_yaml(path: str | None) -> dict:
    """Raw vehicle YAML."""
    import yaml

    p = _vehicle_path(path)
    with open(p, "r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


class SevenDOFModel:
    """Nonlinear 7-DOF two-track vehicle model with exchangeable tyre models."""

    n_states = 10
    n_inputs = 10  # steer_f, steer_r, 4x T_drive, 4x T_brake
    state_type = SevenDOFState

    def __init__(
        self,
        params: SevenDOFParams | None = None,
        tyre: TyreModel | None = None,
        road: RoadSurface | None = None,
        tyre_rear: TyreModel | None = None,
        chassis=None,
    ):
        self.params = params or SevenDOFParams.from_yaml()
        self.chassis = chassis  # QuasiStaticChassis or None
        self._load_params = self.params
        if chassis is not None:
            if chassis.params != self.params:
                raise ValueError("chassis was built for different vehicle parameters")
            self._load_params = dataclasses.replace(
                self.params, k_roll_f=chassis.front_lateral_transfer_share)
        from vehicle_model.tire_model import load_tyre
        self.tyre_front: TyreModel = tyre or load_tyre("pacejka_front_synthetic")
        self.tyre_rear: TyreModel = tyre_rear or load_tyre("pacejka_rear_synthetic")
        for t in (self.tyre_front, self.tyre_rear):
            if not isinstance(t, TyreModel):
                raise TypeError(f"{type(t).__name__} does not implement the TyreModel protocol")
        self.road = road or UniformRoad(mu=1.0)
        self._accel_guess = (0.0, 0.0)  # warm start for the load-transfer iteration only

    @classmethod
    def from_yaml(
        cls,
        vehicle_config: str | None = None,
        tyre_name: str | None = None,
        road: RoadSurface | None = None,
        tyre_rear_name: str | None = None,
        kc_config: str | None = None,
    ) -> SevenDOFModel:
        """Instantiate SevenDOFModel loaded from centralized YAML configuration files.

        Tyre selection per axle: explicit argument, else the vehicle YAML's
        ``tyres: {front, rear}`` section, else ``pacejka_{front,rear}_synthetic``.
        ``kc_config``: optional K&C set (configs/suspension/kc_*.yaml); it needs the
        vehicle YAML's ``chassis_extended`` section. Without it the model has no K&C.
        """
        from vehicle_model.tire_model import load_tyre
        p = SevenDOFParams.from_yaml(vehicle_config)
        yaml_tyres = _vehicle_yaml(vehicle_config).get("tyres") or {}
        front = tyre_name or yaml_tyres.get("front") or "pacejka_front_synthetic"
        rear = tyre_rear_name or yaml_tyres.get("rear") or "pacejka_rear_synthetic"
        chassis = None
        if kc_config is not None:
            from vehicle_model.suspension.kinematics import QuasiStaticChassis
            chassis = QuasiStaticChassis.from_yaml(p, _vehicle_path(vehicle_config), kc_config)
        return cls(params=p, tyre=load_tyre(front), tyre_rear=load_tyre(rear), road=road, chassis=chassis)


    # ------------------------------------------------------------------ geometry
    @property
    def wheel_positions(self) -> np.ndarray:
        """Wheel centre positions [x, y] in the body frame, rows FL, FR, RL, RR."""
        return wheel_contact_positions(self.params)

    # ------------------------------------------------------------- normal loads
    def compute_normal_loads(self, ax: float, ay: float) -> np.ndarray:
        """Quasi-static wheel loads [N]; ``Fz >= 0`` and ``sum(Fz) == m g`` on level ground.

        See :meth:`compute_normal_load_state` for the contact state.
        """
        return quasi_static_normal_loads(self._load_params, ax, ay).Fz

    def compute_normal_load_state(self, ax: float, ay: float) -> NormalLoads:
        """Wheel loads with unilateral contact: lift-off and tip-over are reported, not hidden.

        Force, pitch- and roll-moment balance are exact unless ``support == "tip_over"``.
        """
        return quasi_static_normal_loads(self._load_params, ax, ay)

    # ---------------------------------------------------------------- dynamics
    def wheel_velocities(self, state: SevenDOFState, u: SevenDOFControlInput, steer: np.ndarray | None = None):
        """``(steer, cos, sin, v_wx, v_wy)``: wheel-centre velocity in each wheel frame [m/s].

        ``steer`` defaults to the axle angles of ``u`` (left = right).
        """
        pos = self.wheel_positions
        if steer is None:
            steer = np.array([u.steer_f, u.steer_f, u.steer_r, u.steer_r], dtype=float)
        cos_d, sin_d = np.cos(steer), np.sin(steer)
        vxb = state.vx - state.r * pos[:, 1]
        vyb = state.vy + state.r * pos[:, 0]
        return steer, cos_d, sin_d, vxb * cos_d + vyb * sin_d, -vxb * sin_d + vyb * cos_d

    def _slips(self, state, v_wx, v_wy, v_wheel) -> tuple[np.ndarray, np.ndarray]:
        """Quasi-static regularised slips ``(kappa, alpha)``; overridden by transient-tyre variants."""
        kappa = (v_wheel - v_wx) / np.maximum(np.maximum(np.abs(v_wx), np.abs(v_wheel)), _V_EPS)
        alpha = -np.arctan2(v_wy, np.maximum(np.abs(v_wx), _V_EPS))
        return kappa, alpha

    def _extra_derivatives(self, state, v_wx, v_wy, v_wheel) -> np.ndarray:
        """Derivatives of states appended to the 10 base states (none here)."""
        return np.empty(0)

    def _tyre_forces(self, kappa, alpha, Fz, mu):
        Fx_f, Fy_f = self.tyre_front.forces(kappa[:2], alpha[:2], Fz[:2], mu[:2])
        Fx_r, Fy_r = self.tyre_rear.forces(kappa[2:], alpha[2:], Fz[2:], mu[2:])
        return np.concatenate([Fx_f, Fx_r]), np.concatenate([Fy_f, Fy_r])

    def evaluate_dynamics(
        self,
        state: SevenDOFState,
        u: SevenDOFControlInput,
        road: RoadSurface | None = None,
    ) -> tuple[np.ndarray, WheelTelemetry, float, float]:
        """Return ``(dstate_dt, telemetry, ax, ay)`` for the given state and input."""
        p = self.params
        road = road or self.road
        vx, vy, r, omega, psi = state.vx, state.vy, state.r, state.omega, state.psi

        pos = self.wheel_positions
        v_wheel = omega * p.Reff
        chassis = self.chassis
        if chassis is None:
            steer, cos_d, sin_d, v_wx, v_wy = self.wheel_velocities(state, u)
            kappa, alpha = self._slips(state, v_wx, v_wy, v_wheel)
        else:
            kc = chassis.kc
            steer_geo = wheel_steer_angles(u.steer_f, u.steer_r, pos, p.lf, p.lr, kc.ackermann_front, kc.ackermann_rear)

        # Friction at contact patches
        c_psi, s_psi = math.cos(psi), math.sin(psi)
        X_c = state.X + pos[:, 0] * c_psi - pos[:, 1] * s_psi
        Y_c = state.Y + pos[:, 0] * s_psi + pos[:, 1] * c_psi
        mu = road.get_friction_batch(X_c, Y_c)

        F_aero = 0.5 * p.rho * p.Cd * p.Af * vx * abs(vx)
        F_roll = p.c_rr * p.mass * p.g * math.tanh(vx / 0.5)

        # Fixed-point solution of the quasi-static load-transfer loop
        ax, ay = self._accel_guess
        Fy_axle = (p.mass * p.lr / p.wheelbase * ay, p.mass * p.lf / p.wheelbase * ay)  # first guess
        converged = False
        for _ in range(_LT_MAX_ITER):
            if chassis is not None:
                # K&C steer depends on (ax, ay) and the axle lateral forces: part of the fixed point
                steer = steer_geo + chassis.steer_change(ax, ay, *Fy_axle)
                steer, cos_d, sin_d, v_wx, v_wy = self.wheel_velocities(state, u, steer)
                kappa, alpha = self._slips(state, v_wx, v_wy, v_wheel)
            loads = quasi_static_normal_loads(self._load_params, ax, ay)
            Fz = loads.Fz
            Fx_w, Fy_w = self._tyre_forces(kappa, alpha, Fz, mu)
            Fx_b = Fx_w * cos_d - Fy_w * sin_d
            Fy_b = Fx_w * sin_d + Fy_w * cos_d
            ax_new = (Fx_b.sum() - F_aero - F_roll) / p.mass
            ay_new = Fy_b.sum() / p.mass
            converged = abs(ax_new - ax) < _LT_TOL and abs(ay_new - ay) < _LT_TOL
            if chassis is not None:
                Fy_new = (Fy_w[:2].sum(), Fy_w[2:].sum())
                converged = converged and max(abs(Fy_new[0] - Fy_axle[0]), abs(Fy_new[1] - Fy_axle[1])) < _LT_TOL * p.mass
                Fy_axle = Fy_new
            ax, ay = ax_new, ay_new
            if converged:
                break
        # The loads belong to the previous iterate; on non-convergence they lag (ax, ay).
        self._accel_guess = (ax, ay)

        # Body equations: m(vx_dot - r vy) = sum Fx, m(vy_dot + r vx) = sum Fy
        dvx = ax + r * vy
        dvy = ay - r * vx
        Mz = np.sum(pos[:, 0] * Fy_b - pos[:, 1] * Fx_b)
        dr = Mz / p.Iz

        # Wheel spin: Iw omega_dot = T_d - T_b sgn(omega) - Fx R - d omega
        T_brake_eff = u.T_brake * np.tanh(omega / _OMEGA_EPS)
        domega = (u.T_drive - T_brake_eff - Fx_w * p.Reff - p.wheel_damping * omega) / p.Iw

        dX = vx * c_psi - vy * s_psi
        dY = vx * s_psi + vy * c_psi

        deriv = np.concatenate([[dvx, dvy, dr, *domega, dX, dY, r],
                                self._extra_derivatives(state, v_wx, v_wy, v_wheel)])
        telemetry = WheelTelemetry(
            omega=omega.copy(), v_wx=v_wx, v_wy=v_wy, kappa=kappa, alpha=alpha, Fz=Fz, mu=mu,
            Fx_wheel=Fx_w, Fy_wheel=Fy_w, Fx_body=Fx_b, Fy_body=Fy_b, steer_angles=steer,
            in_contact=loads.in_contact, support=loads.support,
            load_transfer_converged=converged, contact_xy=np.column_stack([X_c, Y_c]),
        )
        return deriv, telemetry, float(ax), float(ay)

    # ------------------------------------------------------------- integration
    def wheel_eigenvalue(self, state: SevenDOFState, telemetry: WheelTelemetry) -> float:
        """Estimate of the fastest wheel-slip eigenvalue ``C_kappa R^2 / (Iw v)`` [1/s]."""
        p = self.params
        C_k = np.concatenate([
            self.tyre_front.slip_stiffness(telemetry.Fz[:2], telemetry.mu[:2]),
            self.tyre_rear.slip_stiffness(telemetry.Fz[2:], telemetry.mu[2:]),
        ])
        v_ref = np.maximum(np.maximum(np.abs(telemetry.v_wx), np.abs(state.omega * p.Reff)), _V_EPS)
        return float(np.max(C_k * p.Reff**2 / (p.Iw * v_ref)))

    def _rk4(self, y0: np.ndarray, u: SevenDOFControlInput, h: float, road: RoadSurface) -> np.ndarray:
        def f(y):
            return self.evaluate_dynamics(self.state_type.from_array(y), u, road)[0]
        k1 = f(y0)
        k2 = f(y0 + 0.5 * h * k1)
        k3 = f(y0 + 0.5 * h * k2)
        k4 = f(y0 + h * k3)
        return y0 + (h / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

    def step(
        self,
        state: SevenDOFState,
        u: SevenDOFControlInput,
        dt: float,
        road: RoadSurface | None = None,
    ) -> tuple[SevenDOFState, WheelTelemetry]:
        """Advance by ``dt`` with RK4 and automatic sub-stepping.

        The wheel-slip dynamics become stiff at low speed (eigenvalue
        ``~ C_kappa R^2 / (Iw v)``). The step is split into ``n`` RK4 sub-steps
        so that ``lambda * dt/n <= 2``. Returns the new state and the telemetry
        evaluated at the new state.
        """
        road = road or self.road
        u_c = u.clamped(self.params)
        _, tel0, _, _ = self.evaluate_dynamics(state, u_c, road)
        n_sub = max(1, math.ceil(self.wheel_eigenvalue(state, tel0) * dt / _RK4_STABILITY))
        h = dt / n_sub
        y = state.to_array()
        for _ in range(n_sub):
            y = self._rk4(y, u_c, h, road)
        next_state = self.state_type.from_array(y)
        _, tel, _, _ = self.evaluate_dynamics(next_state, u_c, road)
        return next_state, tel
