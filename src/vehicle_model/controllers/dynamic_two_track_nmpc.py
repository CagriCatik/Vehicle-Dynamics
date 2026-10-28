"""Dynamic Two-Track Nonlinear Model Predictive Control (Dynamic NMPC).

Solves a finite-horizon optimal control problem (OCP) on a planar two-track
prediction model with combined-slip tyre forces, using
``scipy.optimize.minimize(method="SLSQP")``.

State Vector (x in R^6):
    x = [X, Y, psi, vx, vy, r]^T

Control Input Vector (u in R^4):
    u = [delta_f, delta_r, Mz_direct, ax_demand]^T
    where delta_f is front steer, delta_r is active rear steer,
    Mz_direct is direct yaw moment from torque vectoring, and ax_demand is acceleration.

Features:
1. Dynamic two-track non-linear prediction model with dynamic normal load transfer.
2. Combined-slip Magic Formula tyre model; friction-circle utilisation above 0.90 is
   penalised in the cost (soft penalty, not a hard constraint).
3. Single-shooting discretisation (decision variables are the 4N inputs, states are
   rolled out with RK4) with a shifted warm start.
4. Active four-wheel steering (4WS) and direct yaw moment coordination.
5. Hard box bounds on input amplitudes; input rates are only penalised in the cost.

Conforms to Plan.md §4.3, §5, §10.2, §19.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from pathlib import Path
import time
from typing import Any, Callable, Optional, Sequence, Tuple

import numpy as np
from scipy.optimize import minimize
import yaml

from vehicle_model.controllers.framework import (
    ControlOutput,
    ControlTarget,
    VehicleController,
)
from vehicle_model.core.types import FL, FR, RL, RR, SevenDOFParams, SevenDOFState, WheelTelemetry


@dataclass
class DynamicNMPCConfig:
    """Configuration parameters for Dynamic Two-Track NMPC."""

    # Horizon and timing
    N: int = 12                          # Prediction horizon steps
    dt_pred: float = 0.05                # Prediction sampling time [s] (0.6s total lookahead)

    # Actuator limits
    max_steer_front_rad: float = math.radians(30.0) # Maximum front steer [rad]
    max_steer_rear_rad: float = math.radians(10.0)  # Maximum rear steer [rad]
    max_yaw_moment_nm: float = 4000.0               # Maximum direct yaw moment [N*m]
    max_accel_mps2: float = 4.5                     # Maximum acceleration [m/s^2]
    max_decel_mps2: float = 8.0                     # Maximum deceleration [m/s^2]
    max_steer_rate_radps: float = math.radians(60.0)# Steer slew rate limit [rad/s]
    cornering_stiffness_f: float = 45000.0          # Per-wheel front cornering stiffness [N/rad]
    cornering_stiffness_r: float = 48000.0          # Per-wheel rear cornering stiffness [N/rad]

    # Objective weights
    q_lat_err: float = 12.0              # Lateral error weight (Y - Y_ref)^2
    q_heading_err: float = 8.0           # Heading error weight (psi - psi_ref)^2
    q_speed_err: float = 2.0             # Speed tracking weight (vx - vx_ref)^2
    q_sideslip: float = 15.0             # Sideslip penalty (beta^2)
    q_yaw_rate: float = 0.5              # Yaw rate damping (r^2)
    r_steer_f: float = 0.8               # Front steering control effort (delta_f^2)
    r_steer_r: float = 1.2               # Rear steering control effort (delta_r^2)
    r_yaw_moment: float = 0.0001         # Yaw moment effort (Mz^2)
    r_accel: float = 0.2                 # Longitudinal accel effort (ax^2)
    r_dsteer_f: float = 4.0              # Front steering slew rate penalty
    r_dsteer_r: float = 6.0              # Rear steering slew rate penalty
    w_kamm_circle: float = 50.0          # Kamm friction circle barrier weight

    # Solver parameters
    max_iter: int = 25                   # Max optimizer iterations
    tolerance: float = 1e-3              # Solver tolerance

    @classmethod
    def from_yaml(cls, path: str | Path) -> "DynamicNMPCConfig":
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        cfg = data.get("dynamic_nmpc", data)
        return cls(
            N=int(cfg.get("N", 12)),
            dt_pred=float(cfg.get("dt_pred", 0.05)),
            max_steer_front_rad=math.radians(float(cfg.get("max_steer_front_deg", 30.0))),
            max_steer_rear_rad=math.radians(float(cfg.get("max_steer_rear_deg", 10.0))),
            max_yaw_moment_nm=float(cfg.get("max_yaw_moment_nm", 4000.0)),
            max_accel_mps2=float(cfg.get("max_accel_mps2", 4.5)),
            max_decel_mps2=float(cfg.get("max_decel_mps2", 8.0)),
            max_steer_rate_radps=math.radians(float(cfg.get("max_steer_rate_degps", 60.0))),
            cornering_stiffness_f=float(cfg.get("cornering_stiffness_f", 45000.0)),
            cornering_stiffness_r=float(cfg.get("cornering_stiffness_r", 48000.0)),
            q_lat_err=float(cfg.get("q_lat_err", 12.0)),
            q_heading_err=float(cfg.get("q_heading_err", 8.0)),
            q_speed_err=float(cfg.get("q_speed_err", 2.0)),
            q_sideslip=float(cfg.get("q_sideslip", 15.0)),
            q_yaw_rate=float(cfg.get("q_yaw_rate", 0.5)),
            r_steer_f=float(cfg.get("r_steer_f", 0.8)),
            r_steer_r=float(cfg.get("r_steer_r", 1.2)),
            r_yaw_moment=float(cfg.get("r_yaw_moment", 0.0001)),
            r_accel=float(cfg.get("r_accel", 0.2)),
            r_dsteer_f=float(cfg.get("r_dsteer_f", 4.0)),
            r_dsteer_r=float(cfg.get("r_dsteer_r", 6.0)),
            w_kamm_circle=float(cfg.get("w_kamm_circle", 50.0)),
            max_iter=int(cfg.get("max_iter", 25)),
            tolerance=float(cfg.get("tolerance", 1e-3)),
        )


class DynamicTwoTrackNMPC(VehicleController):
    """Dynamic Two-Track Nonlinear Model Predictive Controller."""

    def __init__(
        self,
        params: Optional[SevenDOFParams] = None,
        config: Optional[DynamicNMPCConfig] = None,
    ):
        self.params = params or SevenDOFParams.from_yaml()
        self.config = config or DynamicNMPCConfig()

        # Input dimension: 4 per step [delta_f, delta_r, Mz, ax]
        self.n_inputs = 4
        self.n_dec = self.config.N * self.n_inputs

        # Warm start solution cache
        self.u_prev = np.zeros(self.n_dec, dtype=np.float64)
        self.last_u = np.zeros(self.n_inputs, dtype=np.float64)

    @property
    def name(self) -> str:
        return "Dynamic Two-Track NMPC (Combined-Slip & 4WS)"

    def reset(self) -> None:
        """Reset warm-start state."""
        self.u_prev.fill(0.0)
        self.last_u.fill(0.0)

    def continuous_dynamics(
        self,
        state: np.ndarray,
        control: np.ndarray,
        mu: float = 1.0,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Evaluate continuous two-track state derivatives and tyre friction utilizations.

        State: [X, Y, psi, vx, vy, r]
        Control: [delta_f, delta_r, Mz_tv, ax_req]
        """
        X, Y, psi, vx, vy, r = state
        delta_f, delta_r, Mz_tv, ax_req = control

        p = self.params
        v_lon = max(1.5, abs(vx))

        # Kinematic slip angles at 4 wheel stations
        # Corner coordinates
        corner_x = np.array([p.lf, p.lf, -p.lr, -p.lr])
        corner_y = np.array([p.wf / 2.0, -p.wf / 2.0, p.wr / 2.0, -p.wr / 2.0])
        steer_arr = np.array([delta_f, delta_f, delta_r, delta_r])

        # Velocities in wheel frames
        vx_w = (v_lon - r * corner_y) * np.cos(steer_arr) + (vy + r * corner_x) * np.sin(steer_arr)
        vy_w = -(v_lon - r * corner_y) * np.sin(steer_arr) + (vy + r * corner_x) * np.cos(steer_arr)
        alpha = -np.arctan2(vy_w, np.maximum(1.0, np.abs(vx_w)))

        # Dynamic vertical normal loads Fz_i
        # Quasi-static load transfer based on ax and estimated ay = vx * r
        ay_est = v_lon * r
        fz_fl = 0.5 * (p.mass * p.g * p.lr / p.wheelbase - p.mass * ax_req * p.hcog / p.wheelbase) - p.mass * ay_est * p.hcog / p.wf * p.k_roll_f
        fz_fr = 0.5 * (p.mass * p.g * p.lr / p.wheelbase - p.mass * ax_req * p.hcog / p.wheelbase) + p.mass * ay_est * p.hcog / p.wf * p.k_roll_f
        fz_rl = 0.5 * (p.mass * p.g * p.lf / p.wheelbase + p.mass * ax_req * p.hcog / p.wheelbase) - p.mass * ay_est * p.hcog / p.wr * (1.0 - p.k_roll_f)
        fz_rr = 0.5 * (p.mass * p.g * p.lf / p.wheelbase + p.mass * ax_req * p.hcog / p.wheelbase) + p.mass * ay_est * p.hcog / p.wr * (1.0 - p.k_roll_f)
        Fz = np.maximum(100.0, np.array([fz_fl, fz_fr, fz_rl, fz_rr]))

        # Longitudinal forces distributed from ax_req and Mz_tv
        # Fx_total = m * ax_req
        Fx_base = 0.25 * p.mass * ax_req
        # Torque vectoring moment differential: delta_Fx = Mz_tv / track
        dFx_f = 0.5 * Mz_tv / p.wf
        dFx_r = 0.5 * Mz_tv / p.wr
        Fx = np.array([
            Fx_base - dFx_f,
            Fx_base + dFx_f,
            Fx_base - dFx_r,
            Fx_base + dFx_r,
        ])

        # Pacejka lateral tyre forces with combined-slip friction circle limitation
        # Cornering stiffness
        C_alpha = np.array([
            self.config.cornering_stiffness_f,
            self.config.cornering_stiffness_f,
            self.config.cornering_stiffness_r,
            self.config.cornering_stiffness_r,
        ])
        Fy = np.zeros(4)
        kamm_ratios = np.zeros(4)

        for i in range(4):
            mu_fzi = max(100.0, mu * Fz[i])
            # Pure lateral peak
            Fy0 = mu_fzi * math.sin(1.3 * math.atan((C_alpha[i] / (1.3 * mu_fzi)) * alpha[i]))
            # Kamm circle combined slip coupling
            fx_ratio = min(0.98, abs(Fx[i]) / mu_fzi)
            Fy[i] = Fy0 * math.sqrt(max(0.01, 1.0 - (fx_ratio ** 2)))
            # Kamm utilization ratio
            kamm_ratios[i] = math.sqrt(Fx[i] ** 2 + Fy[i] ** 2) / mu_fzi

        # Transform forces to body frame
        Fx_body = Fx * np.cos(steer_arr) - Fy * np.sin(steer_arr)
        Fy_body = Fx * np.sin(steer_arr) + Fy * np.cos(steer_arr)

        # Planar equations of motion
        F_drag = 0.5 * 1.225 * 0.30 * 2.2 * (v_lon ** 2)
        ax = (np.sum(Fx_body) - F_drag) / p.mass
        ay = np.sum(Fy_body) / p.mass

        dvx = ax + vy * r
        dvy = ay - vx * r

        # Total yaw acceleration (including tire moment and active direct yaw moment)
        Mz_tires = np.sum(corner_x * Fy_body - corner_y * Fx_body)
        dr = Mz_tires / p.Iz

        # Kinematic pose derivatives
        dX = vx * math.cos(psi) - vy * math.sin(psi)
        dY = vx * math.sin(psi) + vy * math.cos(psi)
        dpsi = r

        x_dot = np.array([dX, dY, dpsi, dvx, dvy, dr], dtype=np.float64)
        return x_dot, kamm_ratios

    def rk4_step(
        self,
        state: np.ndarray,
        control: np.ndarray,
        dt: float,
        mu: float = 1.0,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """RK4 numerical integration of dynamic two-track plant."""
        k1, kamm1 = self.continuous_dynamics(state, control, mu)
        k2, _ = self.continuous_dynamics(state + 0.5 * dt * k1, control, mu)
        k3, _ = self.continuous_dynamics(state + 0.5 * dt * k2, control, mu)
        k4, _ = self.continuous_dynamics(state + dt * k3, control, mu)
        state_next = state + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
        return state_next, kamm1

    def compute_control(
        self,
        state: SevenDOFState,
        target: ControlTarget,
        telemetry: Optional[WheelTelemetry] = None,
        dt: float = 0.01,
    ) -> ControlOutput:
        """Execute Dynamic NMPC optimization step at current state."""
        t0 = time.perf_counter()

        # Convert SevenDOFState to 6-DOF planar state [X, Y, psi, vx, vy, r]
        x0 = np.array([
            state.X,
            state.Y,
            state.psi,
            state.vx,
            state.vy,
            state.r,
        ], dtype=np.float64)

        # Setup reference trajectory vectors across prediction horizon
        N = self.config.N
        dt_p = self.config.dt_pred
        mu_cur = target.mu_estimated if target.mu_estimated > 0 else 1.0

        # Construct reference states [X_ref, Y_ref, psi_ref, vx_ref]
        X_ref = np.zeros(N)
        Y_ref = np.zeros(N)
        psi_ref = np.zeros(N)
        vx_ref = np.full(N, target.target_vx)

        for k in range(N):
            t_k = (k + 1) * dt_p
            X_ref[k] = state.X + max(2.0, state.vx) * t_k
            Y_ref[k] = target.target_y
            psi_ref[k] = target.target_psi

        # Warm start: shift previous solution by 1 step
        u_init = np.zeros(self.n_dec, dtype=np.float64)
        u_init[:-self.n_inputs] = self.u_prev[self.n_inputs:]
        u_init[-self.n_inputs:] = self.u_prev[-self.n_inputs:]

        # Smart warm-start initialization if previous control was zero
        if np.all(self.u_prev == 0.0):
            e_y0 = state.Y - target.target_y
            u_init[0::4] = float(np.clip(-0.03 * e_y0, -self.config.max_steer_front_rad, self.config.max_steer_front_rad))

        # High-speed zero-sideslip 4WS coordination ratio: beta = 0 -> delta_r / delta_f
        p = self.params
        cf_axle = 2.0 * self.config.cornering_stiffness_f
        cr_axle = 2.0 * self.config.cornering_stiffness_r
        v_sq = (max(2.0, state.vx) ** 2)
        denom_4ws = p.lf + (p.mass * v_sq * p.lr) / (cr_axle * p.wheelbase)
        num_4ws = -p.lr + (p.mass * v_sq * p.lf) / (cf_axle * p.wheelbase)
        k_4ws_ratio = float(num_4ws / max(0.1, denom_4ws)) if state.vx > 10.0 else 0.0

        # Bounds on decision variables
        bounds = []
        for _ in range(N):
            bounds.append((-self.config.max_steer_front_rad, self.config.max_steer_front_rad))
            bounds.append((-self.config.max_steer_rear_rad, self.config.max_steer_rear_rad))
            bounds.append((-self.config.max_yaw_moment_nm, self.config.max_yaw_moment_nm))
            bounds.append((-self.config.max_decel_mps2, self.config.max_accel_mps2))

        # Objective function
        cfg = self.config

        def cost_function(u_vec: np.ndarray) -> float:
            cost = 0.0
            x_k = x0.copy()
            u_last_step = self.last_u.copy()

            for k_step in range(N):
                u_k = u_vec[k_step * 4 : (k_step + 1) * 4]
                x_k, kamm_ratios = self.rk4_step(x_k, u_k, dt_p, mu_cur)

                # Tracking errors
                e_y = x_k[1] - Y_ref[k_step]
                e_psi = x_k[2] - psi_ref[k_step]
                e_v = x_k[3] - vx_ref[k_step]
                beta = math.atan2(x_k[4], max(1.5, abs(x_k[3])))

                cost += cfg.q_lat_err * (e_y ** 2)
                cost += cfg.q_heading_err * (e_psi ** 2)
                cost += cfg.q_speed_err * (e_v ** 2)
                cost += cfg.q_sideslip * (beta ** 2)
                cost += cfg.q_yaw_rate * (x_k[5] ** 2)

                # Control effort
                cost += cfg.r_steer_f * (u_k[0] ** 2)
                cost += cfg.r_steer_r * (u_k[1] ** 2)
                cost += cfg.r_yaw_moment * (u_k[2] ** 2)
                cost += cfg.r_accel * (u_k[3] ** 2)

                # Control rate of change
                cost += cfg.r_dsteer_f * ((u_k[0] - u_last_step[0]) ** 2)
                cost += cfg.r_dsteer_r * ((u_k[1] - u_last_step[1]) ** 2)
                u_last_step = u_k

                # 4WS zero-sideslip coordination regularization at highway speeds
                if state.vx > 12.0:
                    delta_r_opt = k_4ws_ratio * u_k[0]
                    cost += 0.8 * cfg.r_steer_r * ((u_k[1] - delta_r_opt) ** 2)

                # Heuristic terminal weighting (no formal stability guarantee)
                if k_step == N - 1:
                    cost += 2.0 * cfg.q_lat_err * (e_y ** 2)
                    cost += 2.0 * cfg.q_heading_err * (e_psi ** 2)
                    cost += 1.5 * cfg.q_sideslip * (beta ** 2)

                # Kamm circle friction barrier penalty
                for ratio in kamm_ratios:
                    if ratio > 0.90:
                        cost += cfg.w_kamm_circle * ((ratio - 0.90) ** 2)

            return float(cost)

        # Solve nonlinear OCP
        res = minimize(
            fun=cost_function,
            x0=u_init,
            method="SLSQP",
            bounds=bounds,
            options={"maxiter": cfg.max_iter, "ftol": cfg.tolerance, "disp": False},
        )

        # In real-time NMPC / RTI, accept candidate solution if it improved or maintained cost
        cost_init = cost_function(u_init)
        if hasattr(res, "x") and not np.isnan(res.x).any() and cost_function(res.x) <= cost_init + 1e-3:
            u_opt = res.x
        elif res.success:
            u_opt = res.x
        else:
            u_opt = u_init
        self.u_prev = u_opt.copy()

        # First control action: [delta_f, delta_r, Mz_tv, ax_req]
        u_0 = u_opt[:4]
        self.last_u = u_0.copy()

        delta_f_cmd = float(u_0[0])
        delta_r_cmd = float(u_0[1])
        Mz_tv_cmd = float(u_0[2])
        ax_req_cmd = float(u_0[3])

        # Allocate wheel torques from Mz_tv and ax_req
        total_wheel_torque = (self.params.mass * ax_req_cmd * self.params.Reff)
        T_drive = np.zeros(4)
        T_brake = np.zeros(4)

        if total_wheel_torque >= 0.0:
            base_drive = 0.25 * total_wheel_torque
            diff_drive = (Mz_tv_cmd / (self.params.wf)) * self.params.Reff * 0.5
            T_drive[FL] = max(0.0, base_drive - diff_drive)
            T_drive[FR] = max(0.0, base_drive + diff_drive)
            T_drive[RL] = max(0.0, base_drive - diff_drive)
            T_drive[RR] = max(0.0, base_drive + diff_drive)
        else:
            base_brake = 0.25 * abs(total_wheel_torque)
            diff_brake = (Mz_tv_cmd / (self.params.wf)) * self.params.Reff * 0.5
            T_brake[FL] = max(0.0, base_brake + diff_brake)
            T_brake[FR] = max(0.0, base_brake - diff_brake)
            T_brake[RL] = max(0.0, base_brake + diff_brake)
            T_brake[RR] = max(0.0, base_brake - diff_brake)

        t_elapsed = (time.perf_counter() - t0) * 1000.0
        _, kamm_0 = self.rk4_step(x0, u_0, dt_p, mu_cur)
        return ControlOutput(
            steer_f=delta_f_cmd,
            steer_r=delta_r_cmd,
            T_drive=T_drive,
            T_brake=T_brake,
            desired_yaw_moment=Mz_tv_cmd,
            allocated_yaw_moment=Mz_tv_cmd,
            status="nominal",
            computation_time_ms=t_elapsed,
            diagnostics={
                "kamm_utilization": kamm_0,
                "peak_kamm": float(np.max(kamm_0)),
                "solver_iterations": int(getattr(res, "nit", 0)),
            },
        )
