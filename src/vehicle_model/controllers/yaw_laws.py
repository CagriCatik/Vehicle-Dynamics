"""Yaw motion control laws: PI, LQR, gain-scheduled LQR and LTV-MPC.

All laws act through :class:`~vehicle_model.controllers.motion_control.MotionController`
(same reference, allocation and slip control). Actuated inputs, selectable:
``("yaw_moment",)`` (torque vectoring / ESC) or ``("rear_steer", "yaw_moment")``
(integrated 4WS + TV).

Design model (independent of the simulated plant, chassis_control_platform_plan.md 10.2): the linear single
track of :mod:`vehicle_model.handling_analysis` with states ``x = [v_y, r]`` and
inputs ``[delta_f, delta_r, M_z]`` (``M_z`` enters as ``r_dot += M_z/I_z``), augmented
with the integral of the yaw-rate error ``z_dot = r - r_ref`` for steady-state
accuracy under model mismatch.

Feedforward: the equilibrium ``(x_ss, u_ss)`` with ``r = r_ref`` at the driver's
``delta_f``. With yaw moment only, ``v_y`` is free (2 equations, unknowns ``v_y, M_z``);
with rear steer as well, the target sideslip is zero (unknowns ``delta_r, M_z``).

LQR:      u = u_ss - K [x - x_ss; z],  K from the continuous algebraic Riccati equation.
          Fixed design speed or gains scheduled over speed (linear interpolation).
          Saturated inputs stop the integrator (anti-windup).
LTV-MPC:  the model at the current speed, discretised with zero-order hold, is held
          over the horizon (time-varying from step to step). With the cost
          ``sum q (x - x_ss)^2 + q_z z^2 + (u - u_ss)'R(u - u_ss) + du'S du`` and box
          limits on the inputs, the condensed problem is a bounded linear least-squares
          problem, solved with ``scipy.optimize.lsq_linear`` (BVLS); the solver result
          is classified by :mod:`.solver_diagnostics`. Separately, the plan is checked
          against the declared limits (:mod:`.constraints`): input amplitudes are hard
          (BVLS bounds); predicted sideslip and yaw rate against the heuristic stability
          limits are advisory; input rates are only penalised (S), no limit is claimed.
"""

from __future__ import annotations

import time
from dataclasses import dataclass

import numpy as np
from scipy.linalg import expm, solve_continuous_are
from scipy.optimize import lsq_linear

from vehicle_model.controllers.motion_control import (
    INFORMATION_FULL_STATE,
    MotionContext,
    MotionRequest,
)
from vehicle_model.controllers.constraints import ADVISORY, HARD, Constraint, ConstraintSet, FeasibilityMonitor, stability_limits
from vehicle_model.controllers.solver_diagnostics import NOMINAL, NO_KKT, SolverMonitor, kkt_residuals
from vehicle_model.core.types import SevenDOFState
from vehicle_model.handling_analysis import LinearSingleTrack, LinearSingleTrackParams

INPUTS_TV = ("yaw_moment",)
INPUTS_4WS_TV = ("rear_steer", "yaw_moment")
INPUTS_AFS = ("front_steer",)                    # active front steering overlay
INPUTS_AFS_TV = ("front_steer", "yaw_moment")
_INPUT_SETS = (INPUTS_TV, INPUTS_4WS_TV, INPUTS_AFS, INPUTS_AFS_TV)
_INDEX = {"front_steer": 0, "rear_steer": 1, "yaw_moment": 2}  # columns of the full input matrix [delta_f, delta_r, M_z]
_LABEL = {INPUTS_TV: "TV", INPUTS_4WS_TV: "4WS+TV", INPUTS_AFS: "AFS", INPUTS_AFS_TV: "AFS+TV"}


# ----------------------------------------------------------------------------- PI
@dataclass
class NoYawLaw:
    """No yaw-moment or rear-steer request: the motion chain runs only its brake/traction functions
    (allocation, EBD, ABS/TCS, ...). Reference for brake-function comparisons."""

    name: str = "brake functions only"
    information: str = INFORMATION_FULL_STATE

    def reset(self) -> None:
        pass

    def request(self, state: SevenDOFState, ctx: MotionContext) -> MotionRequest:
        return MotionRequest(yaw_moment=None)          # no yaw request: the allocation leaves M_z free


@dataclass
class YawRatePI:
    """PI on the yaw-rate error with anti-windup (the slip control and allocation stability controller law)."""

    kp: float = 15000.0       # [N m/(rad/s)]
    ki: float = 100000.0      # [N m/rad]
    deadband: float = 0.0     # [rad/s]
    name: str = "PI (TV)"
    information: str = INFORMATION_FULL_STATE

    def __post_init__(self) -> None:
        self._integral = 0.0

    def reset(self) -> None:
        self._integral = 0.0

    def request(self, state: SevenDOFState, ctx: MotionContext) -> MotionRequest:
        e = ctx.r_ref - state.r
        if abs(e) <= self.deadband:
            self._integral = 0.0
            return MotionRequest()
        e_eff = e - np.sign(e) * self.deadband
        if abs(self.kp * e_eff + self.ki * self._integral) < ctx.max_yaw_moment:
            self._integral += e_eff * ctx.dt
        return MotionRequest(yaw_moment=self.kp * e_eff + self.ki * self._integral)


# ----------------------------------------------------------------------------- model
class YawDesignModel:
    """Linear single track + yaw-moment input + yaw-rate integrator, for a chosen input set."""

    def __init__(self, params: LinearSingleTrackParams, inputs=INPUTS_TV):
        if tuple(inputs) not in _INPUT_SETS:
            raise ValueError(f"inputs must be one of {_INPUT_SETS}")
        self.params = params
        self.inputs = tuple(inputs)
        self.cols = [_INDEX[i] for i in self.inputs]
        self.model = LinearSingleTrack(params)

    def matrices(self, vx: float):
        """``A`` (2x2), ``B_all`` (2x3, columns delta_f, delta_r, M_z) at speed ``vx``."""
        ss = self.model.state_space(vx)
        B_all = np.column_stack([ss.B, [0.0, 1.0 / self.params.Iz]])
        return ss.A, B_all

    def augmented(self, vx: float):
        """``A_a`` (3x3), ``B_a`` (3xm) with states ``[v_y, r, z]``."""
        A, B_all = self.matrices(vx)
        A_a = np.zeros((3, 3))
        A_a[:2, :2] = A
        A_a[2, 1] = 1.0
        B_a = np.vstack([B_all[:, self.cols], np.zeros((1, len(self.cols)))])
        return A_a, B_a

    def equilibrium(self, vx: float, delta_f: float, r_ref: float):
        """``(x_ss = [v_y, r], u_ss)`` holding ``r = r_ref`` (and ``v_y = 0`` with rear steer)."""
        A, B_all = self.matrices(vx)
        b_f = B_all[:, 0] * delta_f
        if len(self.cols) == 1:
            # unknowns v_y and the single input (M_z, or the front-steer overlay for AFS)
            M = np.column_stack([A[:, 0], B_all[:, self.cols[0]]])
            vy, u = np.linalg.solve(M, -(A[:, 1] * r_ref + b_f))
            return np.array([vy, r_ref]), np.array([u])
        M = B_all[:, self.cols]  # two inputs: unknowns are the inputs, with v_y = 0
        u = np.linalg.solve(M, -(A[:, 1] * r_ref + b_f))
        return np.array([0.0, r_ref]), u


def _input_limits(inputs, ctx: MotionContext):
    lim = {"rear_steer": ctx.max_rear_steer, "yaw_moment": ctx.max_yaw_moment, "front_steer": ctx.max_front_overlay}
    return np.array([lim[i] for i in inputs])


def _to_request(inputs, u, status=NOMINAL, diagnostics=None) -> MotionRequest:
    d = dict(zip(inputs, map(float, u)))
    return MotionRequest(yaw_moment=d.get("yaw_moment", 0.0), rear_steer=d.get("rear_steer", 0.0),
                         front_steer=d.get("front_steer", 0.0), status=status, diagnostics=diagnostics or {})


# ----------------------------------------------------------------------------- LQR
@dataclass(frozen=True)
class LQRWeights:
    q_vy: float = 0.0          # [1/(m/s)^2]   0 for TV (sideslip not targeted)
    q_r: float = 50.0          # [1/(rad/s)^2]
    q_z: float = 5000.0        # [1/rad^2]     integral of the yaw-rate error (see tuning note below)
    r_rear_steer: float = 400.0     # [1/rad^2]
    r_front_steer: float = 400.0    # [1/rad^2]  AFS overlay
    r_yaw_moment: float = 1.0e-6    # [1/(N m)^2]

    def matrices(self, inputs):
        Q = np.diag([self.q_vy, self.q_r, self.q_z])
        R = np.diag([{"rear_steer": self.r_rear_steer, "yaw_moment": self.r_yaw_moment,
                      "front_steer": self.r_front_steer}[i] for i in inputs])
        return Q, R


def lqr_gain(A, B, Q, R) -> np.ndarray:
    """Continuous-time LQR gain ``K = R^-1 B' P``."""
    P = solve_continuous_are(A, B, Q, R)
    return np.linalg.solve(R, B.T @ P)


class LQRYawLaw:
    """LQR with integral action; fixed design speed or gain-scheduled over speed."""

    information = INFORMATION_FULL_STATE

    def __init__(self, design: LinearSingleTrackParams, inputs=INPUTS_TV, weights: LQRWeights | None = None,
                 design_speed: float | None = 20.0, schedule_speeds=None):
        self.model = YawDesignModel(design, inputs)
        self.weights = weights or LQRWeights(q_vy=0.0 if tuple(inputs) == INPUTS_TV else 20.0)
        if (design_speed is None) == (schedule_speeds is None):
            raise ValueError("give either design_speed (fixed LQR) or schedule_speeds (gain scheduling)")
        Q, R = self.weights.matrices(self.model.inputs)
        self.speeds = np.array([design_speed] if schedule_speeds is None else sorted(schedule_speeds), dtype=float)
        self.gains = np.array([lqr_gain(*self.model.augmented(v), Q, R) for v in self.speeds])
        kind = "LQR" if schedule_speeds is None else "GS-LQR"
        self.name = f"{kind} ({_LABEL[self.model.inputs]})"
        self._z = 0.0

    def reset(self) -> None:
        self._z = 0.0

    def gain(self, vx: float) -> np.ndarray:
        if len(self.speeds) == 1:
            return self.gains[0]
        v = float(np.clip(abs(vx), self.speeds[0], self.speeds[-1]))
        i = int(np.clip(np.searchsorted(self.speeds, v) - 1, 0, len(self.speeds) - 2))
        w = (v - self.speeds[i]) / (self.speeds[i + 1] - self.speeds[i])
        return (1 - w) * self.gains[i] + w * self.gains[i + 1]

    def request(self, state: SevenDOFState, ctx: MotionContext) -> MotionRequest:
        vx = max(abs(state.vx), 1.0)
        x_ss, u_ss = self.model.equilibrium(vx, ctx.steer_f, ctx.r_ref)
        e = np.array([state.vy - x_ss[0], state.r - x_ss[1], self._z])
        u = u_ss - self.gain(vx) @ e
        lim = _input_limits(self.model.inputs, ctx)
        u_sat = np.clip(u, -lim, lim)
        if np.allclose(u_sat, u):  # anti-windup: integrate only when not saturated
            self._z += (state.r - ctx.r_ref) * ctx.dt
        return _to_request(self.model.inputs, u_sat, diagnostics={"u_ss": u_ss.tolist(), "z": self._z})


# ----------------------------------------------------------------------------- LTV-MPC
# Tuning note (step steer 72 km/h, reference car with K&C, design model without K&C):
# the integral weight decides how fast the model mismatch is removed. LQR: q_z = 500
# needs ~2 s to reach 95 % of the reference, 5000 reaches it in 0.28 s (TV) / 0.07 s
# (4WS+TV) without overshoot, 5e4 overshoots by 5-9 %. The MPC needs a stronger
# integral weight because its horizon cost also pulls the predicted integral back to
# zero: q_z = 1.5e4 gives 0.22 s / 0.07 s with <= 6 % overshoot.
_MPC_Q_Z = 1.5e4


@dataclass(frozen=True)
class MPCSettings:
    horizon: int = 15
    dt: float = 0.02                 # [s] prediction step
    weights: LQRWeights = LQRWeights(q_vy=0.0, q_z=_MPC_Q_Z)
    rate_weight: float = 0.1         # S = rate_weight * R  (input-rate penalty)
    max_consecutive_failures: int = 5
    time_budget_ms: float | None = None


class LTVMPCYawLaw:
    """Linear time-varying MPC with box input limits, solved as bounded least squares."""

    information = INFORMATION_FULL_STATE

    def __init__(self, design: LinearSingleTrackParams, inputs=INPUTS_TV, settings: MPCSettings | None = None):
        self.model = YawDesignModel(design, inputs)
        self.settings = settings or MPCSettings(
            weights=LQRWeights(q_vy=0.0 if tuple(inputs) == INPUTS_TV else 20.0, q_z=_MPC_Q_Z))
        self.name = f"LTV-MPC ({_LABEL[self.model.inputs]})"
        self.monitor = SolverMonitor(self.settings.max_consecutive_failures, self.settings.time_budget_ms)
        self.feasibility = FeasibilityMonitor()
        self.reset()

    def constraints(self, ctx: MotionContext, vx: float) -> ConstraintSet:
        beta_max, r_max = stability_limits(ctx.mu_hat, vx)
        units = {"rear_steer": "rad", "yaw_moment": "N m"}
        return ConstraintSet([
            *(Constraint(i, HARD, float(lim), unit=units[i], note="BVLS bounds")
              for i, lim in zip(self.model.inputs, _input_limits(self.model.inputs, ctx))),
            Constraint("predicted_sideslip", ADVISORY, beta_max, unit="rad", note="heuristic, not enforced"),
            Constraint("predicted_yaw_rate", ADVISORY, r_max, unit="rad/s", note="mu g / v, not enforced"),
        ])

    def reset(self) -> None:
        m, N = len(self.model.inputs), self.settings.horizon
        self._z = 0.0
        self._plan = np.zeros((N, m))   # deviations from u_ss
        self._u_prev = None
        self.monitor.reset()
        self.feasibility.reset()

    def _discrete(self, vx):
        A_a, B_a = self.model.augmented(vx)
        n, m = B_a.shape
        M = np.zeros((n + m, n + m))
        M[:n, :n], M[:n, n:] = A_a, B_a
        E = expm(M * self.settings.dt)
        return E[:n, :n], E[:n, n:]

    def request(self, state: SevenDOFState, ctx: MotionContext) -> MotionRequest:
        s = self.settings
        N, inputs = s.horizon, self.model.inputs
        m = len(inputs)
        vx = max(abs(state.vx), 1.0)
        x_ss, u_ss = self.model.equilibrium(vx, ctx.steer_f, ctx.r_ref)
        Ad, Bd = self._discrete(vx)
        e0 = np.array([state.vy - x_ss[0], state.r - x_ss[1], self._z])

        # Condensed prediction of the deviation e_k = Phi_k e0 + sum_j Gamma_kj du_j
        n = 3
        Phi = np.zeros((N * n, n))
        Gam = np.zeros((N * n, N * m))
        powers = [np.eye(n)]
        for _ in range(N):
            powers.append(Ad @ powers[-1])
        for k in range(N):
            Phi[k * n:(k + 1) * n] = powers[k + 1]
            for j in range(k + 1):
                Gam[k * n:(k + 1) * n, j * m:(j + 1) * m] = powers[k - j] @ Bd
        Q, R = s.weights.matrices(inputs)
        sq, sr = np.sqrt(np.diag(Q)), np.sqrt(np.diag(R))
        Wq = np.kron(np.eye(N), np.diag(sq))
        Wr = np.kron(np.eye(N), np.diag(sr))
        D = np.eye(N * m) - np.eye(N * m, k=-m)           # input differences
        Ws = np.sqrt(s.rate_weight) * Wr
        du0 = np.zeros(N * m)
        if self._u_prev is not None:
            du0[:m] = self._u_prev - u_ss                 # first difference against the last applied input
        A_ls = np.vstack([Wq @ Gam, Wr, Ws @ D])
        b_ls = np.concatenate([-Wq @ Phi @ e0, np.zeros(N * m), Ws @ du0])
        lim = np.tile(_input_limits(inputs, ctx), N)
        lo, hi = -lim - np.tile(u_ss, N), lim - np.tile(u_ss, N)  # bounds on the deviation from u_ss

        warm = np.clip(np.concatenate([self._plan[1:].ravel(), self._plan[-1]]), lo, hi)
        cost = lambda U: float(np.sum((A_ls @ U - b_ls) ** 2))
        t0 = time.perf_counter()
        try:
            res = lsq_linear(A_ls, b_ls, bounds=(lo, hi), method="bvls", tol=1e-10)
            x, ok, msg, nit = res.x, bool(res.success), str(res.message), int(res.nit)
        except (ValueError, np.linalg.LinAlgError) as exc:  # solver failure is a reportable outcome
            x, ok, msg, nit = None, False, f"{type(exc).__name__}: {exc}", 0
        solve_ms = (time.perf_counter() - t0) * 1e3
        c_x = cost(x) if x is not None and np.all(np.isfinite(x)) else float("nan")
        kkt = kkt_residuals(2.0 * A_ls.T @ (A_ls @ x - b_ls), x, lo, hi, cost=c_x) if np.isfinite(c_x) else NO_KKT
        report = self.monitor.classify(ok, x, c_x, cost(warm), msg, nit, solve_ms, x_warm_start=warm, kkt=kkt)
        U = x if report.accepted else warm
        self._plan = U.reshape(N, m)
        planned = u_ss + self._plan                                  # (N, m) absolute inputs
        e_pred = (Phi @ e0 + Gam @ U).reshape(N, n)
        values = {i: planned[:, k] for k, i in enumerate(inputs)}
        values["predicted_sideslip"] = (e_pred[:, 0] + x_ss[0]) / vx
        values["predicted_yaw_rate"] = e_pred[:, 1] + x_ss[1]
        feas = self.feasibility.record(self.constraints(ctx, vx).check(values))
        u = u_ss + self._plan[0]
        u_sat = np.clip(u, -_input_limits(inputs, ctx), _input_limits(inputs, ctx))
        if np.allclose(u_sat, u, rtol=0, atol=1e-9 * (1 + np.abs(u).max())):  # anti-windup
            self._z += (state.r - ctx.r_ref) * ctx.dt
        u = u_sat
        self._u_prev = u
        return _to_request(inputs, u, status=report.status,
                           diagnostics={"solver": report.as_dict(), "feasibility": feas.as_dict(),
                                        "u_ss": u_ss.tolist(), "z": self._z})


__all__ = (
    "NoYawLaw",
    "INPUTS_4WS_TV",
    "INPUTS_AFS",
    "INPUTS_AFS_TV",
    "INPUTS_TV",
    "LQRWeights",
    "LQRYawLaw",
    "LTVMPCYawLaw",
    "MPCSettings",
    "YawDesignModel",
    "YawRatePI",
    "lqr_gain",
)
