"""Nonlinear Model Predictive Control (NMPC) path tracking controller (SLSQP, single shooting).

Algorithm Overview
------------------
At each control step the controller solves a finite-horizon optimal control problem (OCP):
    min  sum_{i=0}^{N-1} [ Q_Y * (y_i - y_ref)^2
         + Q_YAW * (yaw_i - yaw_ref)^2
         + Q_V * (v_i - v_ref)^2
         + obstacle_barrier_i ]
         + R_steer * ||u_sf||^2 + R_accel * ||u_a||^2

subject to:
    -U_A_MAX     <= u_a_i   <= U_A_MAX
    -U_STEER_MAX <= u_sf_i  <= U_STEER_MAX

Transcribed as a single-shooting nonlinear program: the decision variables are only the 2N
control inputs, the states are rolled out by forward Euler inside the cost function. Solved with
``scipy.optimize.minimize(method="SLSQP")`` warm-started from the previous solution.

This is *not* a C/GMRES (Ohtsuka continuation / GMRES) implementation. The class name
``NMPCControllerCGMRES`` is kept only as a deprecated alias of :class:`SLSQPPathTrackingNMPC`
for backward compatibility.

The prediction model is a kinematic bicycle whose rear-axle steer angle is slaved to the
front steer via a synthetic look-up table (LUT).

Known limitations (fixed constants, not configurable yet):
  * front steer clamp +-30 deg inside the prediction model, speed clamp 0..15 m/s,
    LUT speed clamp 0..20 m/s
  * obstacle barrier weight 80, minimum horizon step 0.05 s

Solver outcome: every solve is classified by :mod:`.solver_diagnostics`
(nominal / suboptimal / fallback / degraded). Separately, the wrapper
:class:`NMPCPathTrackingController` checks the plan against its hard input bounds
(:mod:`.constraints`) and acts through the shared
:class:`.motion_control.ActuationLayer` (allocation of ``m a_x`` with ``M_z = 0``, then
ABS/TCS; Plan V3.0 M10 5.3). Output status as in
:class:`.motion_control.MotionController`: solver status if not nominal, otherwise the
allocation status.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from pathlib import Path
import time
from typing import Any, Sequence
import warnings

import numpy as np
from scipy.optimize import minimize
import yaml

from vehicle_model.controllers.framework import (
    ControlOutput,
    ControlTarget,
    VehicleController,
)
from vehicle_model.allocation import ActuatorHealth
from vehicle_model.controllers.constraints import HARD, Constraint, ConstraintSet, FeasibilityMonitor
from vehicle_model.controllers.motion_control import INFORMATION_FULL_STATE, ActuationConfig, ActuationLayer
from vehicle_model.controllers.solver_diagnostics import NOMINAL, SolverMonitor
from vehicle_model.core.types import SevenDOFParams, SevenDOFState, WheelTelemetry
from vehicle_model.steering.lut import LookupTable, load_lookup_table
from vehicle_model.vehicle_models.normal_loads import quasi_static_normal_loads


@dataclass
class NMPCConfig:
    """Configuration parameters for NMPC controller, vehicle, and trajectory."""

    # Vehicle parameters
    wheelbase: float = 0.60
    max_accel: float = 4.5
    max_steer_front_rad: float = math.radians(25.0)
    vehicle_radius: float = 0.30

    # Controller weights and horizon
    N: int = 8
    tf: float = 1.5
    alpha: float = 0.5
    q_x: float = 0.0
    q_y: float = 25.0
    q_yaw: float = 3.0
    q_v: float = 20.0
    r_steer: float = 0.05
    r_accel: float = 0.01

    # Trajectory
    target_v: float = 2.5
    amplitude: float = 1.5
    frequency: float = 0.15
    wavelength: float = 16.6666

    # Obstacles: list of [x, y, radius]
    obstacles: list[list[float]] = field(default_factory=list)

    # Path to LUT
    lut_path: str = "configs/steering/lut.json"

    @classmethod
    def from_yaml(cls, path: str | Path | None = None) -> NMPCConfig:
        """Load configuration from YAML."""
        if path is None:
            from vehicle_model.config.loader import get_configs_root
            candidate = get_configs_root() / "controllers" / "nmpc_path_tracking.yaml"
            if not candidate.exists():
                candidate = Path("configs/controllers/nmpc_path_tracking.yaml").resolve()
            path = candidate
        else:
            path = Path(path)

        with open(path, "r", encoding="utf-8") as f:
            cfg = yaml.safe_load(f) or {}

        veh = cfg.get("vehicle", {})
        ctrl = cfg.get("controller", {})
        traj = cfg.get("trajectory", {})
        paths = cfg.get("paths", {})

        target_v = float(traj.get("target_v", 2.5))
        freq = float(traj.get("frequency", 0.15))
        wavelength = target_v / freq if freq > 0 else 16.6666

        return cls(
            wheelbase=float(veh.get("wheelbase", 0.60)),
            max_accel=float(veh.get("max_accel", 4.5)),
            max_steer_front_rad=math.radians(float(veh.get("max_steer_front_deg", 25.0))),
            vehicle_radius=float(veh.get("radius", 0.30)),
            N=int(ctrl.get("N", 8)),
            tf=float(ctrl.get("tf", 1.5)),
            alpha=float(ctrl.get("alpha", 0.5)),
            q_x=float(ctrl.get("q_x", 0.0)),
            q_y=float(ctrl.get("q_y", 25.0)),
            q_yaw=float(ctrl.get("q_yaw", 3.0)),
            q_v=float(ctrl.get("q_v", 20.0)),
            r_steer=float(ctrl.get("r_steer", 0.05)),
            r_accel=float(ctrl.get("r_accel", 0.01)),
            target_v=target_v,
            amplitude=float(traj.get("amplitude", 1.5)),
            frequency=freq,
            wavelength=wavelength,
            obstacles=list(cfg.get("obstacles", [])),
            lut_path=str(paths.get("lut_json", "configs/steering/lut.json")),
        )


def _angle_diff(a: float, b: float) -> float:
    """Shortest signed angle difference wrapped to (-pi, pi]."""
    d = a - b
    while d > math.pi:
        d -= 2.0 * math.pi
    while d < -math.pi:
        d += 2.0 * math.pi
    return d


def get_ref_state(
    t: float,
    init_x: float,
    vehicle_x: float | None = None,
    target_v: float = 2.5,
    amplitude: float = 1.5,
    wavelength: float = 16.6666,
) -> tuple[float, float, float, float]:
    """Compute the spatial reference state along sinusoidal path.

    Returns: (x_pos, y_ref, yaw_ref, v_ref)
    """
    x_pos = vehicle_x if vehicle_x is not None else (init_x + target_v * t)
    x_rel = x_pos - init_x
    k = 2.0 * math.pi / wavelength
    y_ref = amplitude * math.sin(k * x_rel)
    dy_dx = amplitude * k * math.cos(k * x_rel)
    yaw_ref = math.atan2(dy_dx, 1.0)
    v_ref = target_v
    return x_pos, y_ref, yaw_ref, v_ref


def bicycle_kinematic_model(
    v: float,
    yaw: float,
    u_a: float,
    u_steer_f: float,
    wheelbase: float = 0.60,
    steering_lut: LookupTable | None = None,
) -> tuple[float, float, float, float, float]:
    """Evaluate continuous kinematic bicycle state derivatives with LUT rear-axle steer.

    Returns: (dx, dy, d_yaw, dv, u_steer_r)

    Yaw rate uses the small-sideslip approximation ``v / L * (sin(delta_f) - sin(delta_r))``.
    """
    steer_f_clamped = max(-math.radians(30.0), min(math.radians(30.0), u_steer_f))
    if steering_lut is not None:
        steer_f_deg = math.degrees(steer_f_clamped)
        v_clamped = max(0.0, min(v, 20.0))
        u_steer_r = steering_lut.desired_rear_rad(v_clamped, steer_f_deg)
    else:
        u_steer_r = 0.0

    dx = v * math.cos(yaw)
    dy = v * math.sin(yaw)
    dv = u_a
    d_yaw = (v / max(wheelbase, 1e-4)) * (math.sin(steer_f_clamped) - math.sin(u_steer_r))
    return dx, dy, d_yaw, dv, u_steer_r


class KinematicBicycleSystem:
    """Stateful kinematic bicycle plant for simulation and benchmarking."""

    def __init__(
        self,
        init_x: float = 0.0,
        init_y: float = 0.0,
        init_yaw: float = 0.0,
        init_v: float = 0.05,
        wheelbase: float = 0.60,
        steering_lut: LookupTable | None = None,
    ):
        self.x = float(init_x)
        self.y = float(init_y)
        self.yaw = float(init_yaw)
        self.v = float(init_v)
        self.wheelbase = wheelbase
        self.steering_lut = steering_lut

        if self.steering_lut is None:
            try:
                self.steering_lut = load_lookup_table()
            except Exception:
                self.steering_lut = None

        self.history_x = [self.x]
        self.history_y = [self.y]
        self.history_yaw = [self.yaw]
        self.history_v = [self.v]
        self.history_steer_f = [0.0]
        self.history_steer_r = [0.0]
        self.history_a = [0.0]

    def update_state(self, u_a: float, u_steer_f: float, dt: float = 0.05) -> None:
        """Advance the vehicle state by dt using forward Euler integration."""
        dx, dy, d_yaw, dv, u_steer_r = bicycle_kinematic_model(
            self.v,
            self.yaw,
            u_a,
            u_steer_f,
            wheelbase=self.wheelbase,
            steering_lut=self.steering_lut,
        )
        self.x += dt * dx
        self.y += dt * dy
        self.yaw += dt * d_yaw
        self.v = max(0.0, min(self.v + dt * dv, 15.0))

        self.history_x.append(self.x)
        self.history_y.append(self.y)
        self.history_yaw.append(self.yaw)
        self.history_v.append(self.v)
        self.history_steer_f.append(u_steer_f)
        self.history_steer_r.append(u_steer_r)
        self.history_a.append(u_a)


def _mpc_cost(
    u_seq: np.ndarray,
    x0: float,
    y0: float,
    yaw0: float,
    v0: float,
    dt: float,
    init_x: float,
    config: NMPCConfig,
    steering_lut: LookupTable | None,
) -> float:
    """Evaluate the finite-horizon single-shooting cost (states rolled out by forward Euler)."""
    N = config.N
    xi, yi, yawi, vi = x0, y0, yaw0, v0
    cost = 0.0

    for i in range(N):
        u1_cmd = float(u_seq[i])
        usf = float(u_seq[N + i])

        _, yr, yawr, _ = get_ref_state(
            0.0,
            init_x,
            vehicle_x=xi,
            target_v=config.target_v,
            amplitude=config.amplitude,
            wavelength=config.wavelength,
        )

        cost += config.q_y * (yi - yr) ** 2
        cost += config.q_yaw * _angle_diff(yawi, yawr) ** 2
        cost += config.q_v * (vi - config.target_v) ** 2

        # Obstacle avoidance log-barrier
        for ox, oy, or_ in config.obstacles:
            r_safe = or_ + config.vehicle_radius
            d2 = (xi - ox) ** 2 + (yi - oy) ** 2 + 1e-6
            if d2 < (4.0 * r_safe) ** 2:
                cost += 80.0 * r_safe**2 / d2

        dx, dy, dyaw, dv, _ = bicycle_kinematic_model(
            vi,
            yawi,
            u1_cmd,
            usf,
            wheelbase=config.wheelbase,
            steering_lut=steering_lut,
        )
        xi += dt * dx
        yi += dt * dy
        yawi += dt * dyaw
        vi = max(0.0, min(vi + dt * dv, 15.0))

    cost += config.r_steer * float(np.dot(u_seq[N:], u_seq[N:]))
    cost += config.r_accel * float(np.dot(u_seq[:N], u_seq[:N]))
    return cost


class SLSQPPathTrackingNMPC:
    """Receding-horizon NMPC path tracker solved via SLSQP (single shooting) with warm starting.

    Keeps the original simulator interface ``calc_input(x, y, yaw, v, time)``.
    """

    def __init__(
        self,
        init_x: float = 0.0,
        config: NMPCConfig | None = None,
        steering_lut: LookupTable | None = None,
    ):
        self.config = config or NMPCConfig()
        self.init_x = float(init_x)
        self.N = self.config.N
        self.tf = self.config.tf
        self.alpha = self.config.alpha

        self.steering_lut = steering_lut
        if self.steering_lut is None:
            try:
                self.steering_lut = load_lookup_table(self.config.lut_path)
            except Exception:
                self.steering_lut = None

        self.u1 = np.zeros(self.N, dtype=float)  # Accel sequence
        self.u2 = np.zeros(self.N, dtype=float)  # Front steer sequence
        self.history_f: list[float] = []
        self.monitor = SolverMonitor()
        self.last_report = None

    def set_target_v(self, v: float) -> None:
        """Update desired vehicle longitudinal speed."""
        self.config.target_v = float(v)

    def reset(self) -> None:
        """Reset warm-start state and solve history."""
        self.u1 = np.zeros(self.N, dtype=float)
        self.u2 = np.zeros(self.N, dtype=float)
        self.history_f.clear()
        self.monitor.reset()
        self.last_report = None

    def calc_input(
        self,
        x: float,
        y: float,
        yaw: float,
        v: float,
        time_s: float,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Compute optimal control input sequences over prediction horizon."""
        dt = self.tf * (1.0 - math.exp(-self.alpha * max(0.0, time_s))) / self.N
        dt = max(dt, 0.05)

        u_a_max = self.config.max_accel
        u_steer_max = self.config.max_steer_front_rad
        bounds = [(-u_a_max, u_a_max)] * self.N + [(-u_steer_max, u_steer_max)] * self.N
        u_guess = np.concatenate([self.u1, self.u2])

        args = (x, y, yaw, v, dt, self.init_x, self.config, self.steering_lut)
        t_solve = time.perf_counter()
        res = minimize(
            _mpc_cost,
            u_guess,
            args=args,
            method="SLSQP",
            bounds=bounds,
            options={"maxiter": 100, "ftol": 1e-5},
        )
        solve_ms = (time.perf_counter() - t_solve) * 1e3

        # Use the solution if it converged or improves on the warm start (see solver_diagnostics)
        self.last_report = self.monitor.classify(
            bool(res.success), res.x, float(res.fun), float(_mpc_cost(u_guess, *args)),
            str(res.message), int(getattr(res, "nit", 0)), solve_ms,
        )
        if self.last_report.accepted:
            u_opt = res.x
            self.u1 = np.clip(u_opt[: self.N], -u_a_max, u_a_max)
            self.u2 = np.clip(u_opt[self.N :], -u_steer_max, u_steer_max)

        self.history_f.append(float(res.fun))
        return self.u1, self.u2


class NMPCControllerCGMRES(SLSQPPathTrackingNMPC):
    """Deprecated alias of :class:`SLSQPPathTrackingNMPC`.

    The controller never implemented the C/GMRES algorithm; use ``SLSQPPathTrackingNMPC``.
    """

    def __init__(self, *args: Any, **kwargs: Any):
        warnings.warn(
            "NMPCControllerCGMRES is deprecated: the solver is SLSQP, not C/GMRES. "
            "Use SLSQPPathTrackingNMPC instead.",
            DeprecationWarning,
            stacklevel=2,
        )
        super().__init__(*args, **kwargs)


class NMPCPathTrackingController:
    """NMPC path-tracking controller conforming to VehicleController protocol."""

    def __init__(
        self,
        config: NMPCConfig | None = None,
        params: SevenDOFParams | None = None,
        steering_lut: LookupTable | None = None,
        actuation: ActuationConfig | None = None,
        health: ActuatorHealth | None = None,
    ):
        self.config = config or NMPCConfig()
        self.params = params or SevenDOFParams()
        self.actuation = ActuationLayer(self.params, actuation, health)
        self.feasibility = FeasibilityMonitor()
        self.constraints = ConstraintSet([
            Constraint("accel", HARD, self.config.max_accel, unit="m/s^2", note="SLSQP bounds"),
            Constraint("steer_front", HARD, self.config.max_steer_front_rad, unit="rad", note="SLSQP bounds"),
        ])
        self.controller = SLSQPPathTrackingNMPC(
            init_x=0.0,
            config=self.config,
            steering_lut=steering_lut,
        )
        self._current_time = 0.0

    @classmethod
    def from_yaml(cls, path: str | Path | None = None) -> NMPCPathTrackingController:
        cfg = NMPCConfig.from_yaml(path)
        return cls(config=cfg)

    @property
    def name(self) -> str:
        return "NMPC Path Tracking Controller (SLSQP)"

    information = INFORMATION_FULL_STATE

    def reset(self) -> None:
        self.controller.reset()
        self.actuation.reset()
        self.feasibility.reset()
        self._current_time = 0.0

    def compute_control(
        self,
        state: SevenDOFState,
        target: ControlTarget,
        telemetry: WheelTelemetry | None = None,
        dt: float = 0.01,
    ) -> ControlOutput:
        t0 = time.perf_counter()
        self._current_time += dt

        if target.target_vx > 0:
            self.controller.set_target_v(target.target_vx)

        x_val = getattr(state, "X", getattr(state, "x", 0.0))
        y_val = getattr(state, "Y", getattr(state, "y", 0.0))
        u1s, u2s = self.controller.calc_input(
            x=x_val,
            y=y_val,
            yaw=state.psi,
            v=max(0.01, state.vx),
            time_s=self._current_time,
        )
        cmd_ax = float(u1s[0])
        cmd_steer_f = float(u2s[0])

        # Derivation of rear steering from calibrated LUT
        if self.controller.steering_lut is not None:
            cmd_steer_r = self.controller.steering_lut.compute_rear_angle(state.vx, cmd_steer_f)
        else:
            cmd_steer_r = 0.0

        feas = self.feasibility.record(self.constraints.check({"accel": u1s, "steer_front": u2s}))

        # Shared actuation chain: allocation of m a_x (no yaw moment), then ABS/TCS
        p = self.params
        if telemetry is not None:
            Fz, mu, Fy, wheel_steer = telemetry.Fz, telemetry.mu, telemetry.Fy_wheel, telemetry.steer_angles
        else:
            Fz = quasi_static_normal_loads(p, 0.0, 0.0).Fz
            mu, Fy = np.full(4, max(target.mu_estimated, 0.05)), np.zeros(4)
            wheel_steer = np.array([cmd_steer_f, cmd_steer_f, cmd_steer_r, cmd_steer_r])
        Fx_des = p.mass * cmd_ax
        t_drive, t_brake, alloc, mz_issued, slip_active = self.actuation.actuate(
            Fx_des, 0.0, wheel_steer, Fz, mu, Fy, telemetry, state.vx, dt)
        report = self.controller.last_report
        status = "saturated" if (alloc.status != "infeasible" and slip_active) else alloc.status
        if report.status != NOMINAL:
            status = report.status

        t_elapsed = (time.perf_counter() - t0) * 1000.0
        return ControlOutput(
            steer_f=cmd_steer_f,
            steer_r=cmd_steer_r,
            T_drive=t_drive,
            T_brake=t_brake,
            desired_yaw_moment=0.0,
            allocated_yaw_moment=mz_issued,
            status=status,
            computation_time_ms=t_elapsed,
            diagnostics={
                "cmd_ax": cmd_ax,
                "history_f": self.controller.history_f[-1] if self.controller.history_f else 0.0,
                "solver": report.as_dict(),
                "feasibility": feas.as_dict(),
                "actuation": {"demand": {"Fx": Fx_des, "Mz": 0.0},
                              "allocation_achievable": {"Fx": float(alloc.achieved[0]), "Mz": float(alloc.achieved[1])},
                              "allocation_status": alloc.status,
                              "issued": {"Fx": float(self.actuation.allocator.effectiveness(wheel_steer)[0] @ ((t_drive - t_brake) / p.Reff)),
                                         "Mz": mz_issued,
                                         "slip_control_active": slip_active}},
                "information": self.information,
            },
        )


__all__ = (
    "NMPCConfig",
    "get_ref_state",
    "bicycle_kinematic_model",
    "KinematicBicycleSystem",
    "SLSQPPathTrackingNMPC",
    "NMPCControllerCGMRES",
    "NMPCPathTrackingController",
)
