"""Robust and adaptive yaw motion laws (variants A-D).

All laws use the design model of :mod:`.yaw_laws` (linear single track with yaw-moment
input and yaw-rate integrator, states ``e = [v_y - v_y,ss, r - r_ss, z]``), are designed
once from *nominal* parameters and act through :class:`.motion_control.MotionController`.
None of them is re-tuned to the simulated plant; the plant may deviate on purpose
(:mod:`vehicle_model.simulation.uncertainty`).

A. Prescribed-degree-of-stability LQR (:class:`RobustLQRYawLaw`)
   Design on the shifted system ``(A + alpha I, B)``. With ``P`` the stabilising solution
   of ``(A + alpha I)'P + P(A + alpha I) - P B R^-1 B' P + Q = 0`` and ``K = R^-1 B' P``,
   ``V = e'Pe`` satisfies along ``e_dot = (A - BK) e``

       V_dot = e'((A-BK)'P + P(A-BK)) e = -2 alpha V - e'(Q + K'RK) e <= -2 alpha V,

   so every nominal closed-loop eigenvalue has ``Re <= -alpha``. The margin ``alpha`` is
   room for model error: a perturbation that moves the eigenvalues by less than
   ``alpha`` keeps the loop stable. Whether the declared parameter box stays inside that
   room is *checked*, not assumed: :func:`robust_stability` evaluates the closed loop
   over the box. Gains are scheduled over speed like the GS-LQR.

B. Extended-state disturbance observer (:class:`DisturbanceObserverLaw`)
   Yaw channel of the nominal model with a lumped disturbance ``d`` (model error, road
   and actuator effects, all in rad/s^2):

       r_dot = f_nom(v_y, r, delta, M_z) + d,   d_dot = 0,
       f_nom = (l_f F_yf - l_r F_yr + M_z) / I_z,
       F_yf = sat(C_f alpha_f, mu_hat F_zf),  F_yr = sat(C_r alpha_r, mu_hat F_zr)

   with static axle loads and the estimated friction. The saturation is part of the known
   model on purpose: with linear axle forces the observer would read the missing force of
   a saturated front axle as a disturbance, and cancelling it would force the yaw response
   of an unsaturated vehicle (turning the car in; found in the first test, sideslip 15 deg
   in sine with dwell). ``d`` then holds parameter errors (stiffness, inertia, friction
   estimate), not the known tyre limit.

   Observer (Luenberger on the extended state ``[r, d]`` with measured ``r``):

       r_hat_dot = f_nom + d_hat + l1 (r - r_hat),   d_hat_dot = l2 (r - r_hat).

   Error ``(e_r, e_d) = (r - r_hat, d - d_hat)``: ``e_r_dot = e_d - l1 e_r``, ``e_d_dot =
   -l2 e_r`` (for constant ``d``), characteristic polynomial ``s^2 + l1 s + l2``. With
   ``l1 = 2 w_o``, ``l2 = w_o^2`` both poles are at ``-w_o``. Compensation: the base law's
   yaw moment plus ``-I_z d_hat`` cancels the estimated disturbance within the actuator
   limit. The observer is fed the *applied* (clipped) moment; the allocation may still
   deliver less, which then appears as disturbance (and is compensated only within the
   limit). Discretised with forward Euler at the control period (``w_o dt << 1``).

C. Constraint-tightening MPC (:class:`TighteningMPCYawLaw`)
   Discrete design model ``e+ = A_d e + B_d u + E w`` with an additive disturbance
   ``|w_i| <= w_max,i`` per step on ``(v_y, r)``. Inputs are parametrised with the
   LQR gain as ancillary feedback, ``u_k = u_ss - K e_k + c_k``; the decision variables
   are ``c``. Nominal prediction: ``e+ = A_K e + B_d c`` with ``A_K = A_d - B_d K``. The
   true error deviates from the nominal one by ``sum_{j<k} A_K^j E w``, so a constraint
   ``|y| <= y_max`` on an output ``y = C e + y_ss`` holds for every admissible ``w`` if
   the nominal prediction satisfies the tightened bound

       |C e_k + y_ss| <= y_max - h_k,   h_k = sum_{j=0}^{k-1} |C A_K^j E| w_max

   (the same for the inputs with ``C -> K``). Constrained outputs: sideslip ``v_y / v_x``
   and yaw rate against :func:`.constraints.stability_limits`. The QP (quadratic cost as
   the LTV-MPC, linear inequalities) is solved with SLSQP and analytic derivatives.

   Infeasibility. When the vehicle is already beyond a declared limit (e.g. the yaw rate
   above ``mu g / v``, possible because the tyres reach more than ``mu`` at light loads) no
   input can satisfy the hard state constraints. The law then solves the relaxed problem

       min J(c) + w1 (s_x + s_u) + w2 (s_x^2 + s_u^2)   s.t. state rows <= rhs + s_x,
                                                            input rows <= rhs + s_u,  s >= 0,

   (an exact penalty: zero slack whenever the tightened problem is feasible). Reporting
   follows the state estimation separation: the solver status says how well the relaxed problem was
   solved; the plan's violation of the hard limits appears in the feasibility report
   (counted as infeasibility rate), together with ``tightened_problem_feasible = False``
   and the slack. Only if the relaxed solve fails as well is ``c = 0`` (the ancillary
   LQR) applied with status fallback.
   Guarantee scope: the tightening covers disturbances inside ``w_max`` for the linear
   design model; the nonlinear plant is not covered by it.

D. Adaptive cornering-stiffness LQR (:class:`AdaptiveLQRYawLaw`)
   One parameter, a common factor ``theta`` on both axle cornering stiffnesses. Lateral
   balance of the linear single track: ``a_y = theta phi`` with the regressor

       phi = (C_f alpha_f + C_r alpha_r) / m,  alpha_f = delta_f - (v_y + l_f r)/v_x,
                                               alpha_r = delta_r - (v_y - l_r r)/v_x,

   and ``a_y = v_y_dot + r v_x`` from consecutive states. Recursive least squares with
   forgetting factor ``lambda``:

       k = P phi / (lambda + phi^2 P),  theta += k (a_y - theta phi),  P = (P - k phi P)/lambda,

   updated only with excitation (``|phi| >= phi_min``) and only where the model structure
   holds: measured and predicted lateral acceleration both in the linear tyre range
   (``|a_y|, |theta phi| <= linear_range * mu_hat g``; the measured value alone passes samples
   from steering reversals with saturated tyres, where ``a_y`` crosses zero while the linear
   prediction is large) and little yaw moment in the last step (``|M_z| <= max_moment_share
   M_z,max``; brake-based yaw moments take lateral force through combined slip, which the
   regressor does not model). The estimate is projected onto
   ``[theta_min, theta_max]``. The initial covariance encodes the nominal design as prior
   (``P0 = 0.05``, standard deviation ~0.22 around ``theta = 1``); with a vague prior
   (``P0 = 10``) the first few samples, taken in fast transients, dominated the estimate
   (``theta = 0.59`` on the nominal vehicle after sine with dwell). Without the range limit (``linear_range = inf``) the estimate
   reads tyre saturation as soft tyres: in sine with dwell ``theta`` fell to its lower bound,
   and the controller designed for that soft vehicle destabilised the real one (worn rear
   tyres: 89 deg sideslip, with frozen ``theta = 1``: 2.3 deg) -- certainty equivalence outside
   the validity range of the model. The gains come from a table designed beforehand over
   ``speed x theta`` (no optimisation at run time) and follow a low-pass filtered
   ``theta`` (time constant ``tau``), so the adaptation is slow compared with the yaw
   dynamics. Near the friction limit ``theta < 1`` stands for the reduced effective
   stiffness of the saturating tyres.
"""

from __future__ import annotations

import dataclasses
import math
import time
from dataclasses import dataclass

import numpy as np
from scipy.linalg import expm
from scipy.optimize import minimize

from vehicle_model.controllers.constraints import ADVISORY, HARD, Constraint, ConstraintSet, FeasibilityMonitor, stability_limits
from vehicle_model.controllers.motion_control import INFORMATION_FULL_STATE, MotionContext, MotionRequest
from vehicle_model.controllers.solver_diagnostics import NO_KKT, SolverMonitor, kkt_residuals
from vehicle_model.controllers.yaw_laws import (
    INPUTS_4WS_TV,
    INPUTS_TV,
    LQRWeights,
    LQRYawLaw,
    YawDesignModel,
    _input_limits,
    _to_request,
    lqr_gain,
)
from vehicle_model.core.types import SevenDOFState
from vehicle_model.handling_analysis import LinearSingleTrackParams

DEFAULT_SPEEDS = (5.0, 10.0, 15.0, 20.0, 25.0, 30.0, 40.0)


def _weights(inputs, **over) -> LQRWeights:
    return LQRWeights(q_vy=0.0 if tuple(inputs) == INPUTS_TV else 20.0, **over)


def _tag(inputs) -> str:
    return "4WS+TV" if tuple(inputs) == INPUTS_4WS_TV else "TV"


# ----------------------------------------------------------------------------- A
class RobustLQRYawLaw(LQRYawLaw):
    """Gain-scheduled LQR with prescribed degree of stability ``alpha`` (closed-loop ``Re <= -alpha``)."""

    def __init__(self, design: LinearSingleTrackParams, inputs=INPUTS_TV, alpha: float = 3.0,
                 weights: LQRWeights | None = None, schedule_speeds=DEFAULT_SPEEDS):
        if alpha < 0.0:
            raise ValueError("alpha must be >= 0")
        super().__init__(design, inputs, weights, design_speed=None, schedule_speeds=schedule_speeds)
        self.alpha = float(alpha)
        Q, R = self.weights.matrices(self.model.inputs)
        n = 3
        self.gains = np.array([lqr_gain(self.model.augmented(v)[0] + self.alpha * np.eye(n),
                                        self.model.augmented(v)[1], Q, R) for v in self.speeds])
        self.name = f"Robust LQR alpha={self.alpha:g} ({_tag(self.model.inputs)})"


def closed_loop_abscissa(design: LinearSingleTrackParams, inputs, K: np.ndarray, vx: float) -> float:
    """Largest real part of ``A_a - B_a K`` for the design model ``design`` at speed ``vx``."""
    A, B = YawDesignModel(design, inputs).augmented(vx)
    return float(np.max(np.linalg.eigvals(A - B @ K).real))


def robust_stability(law, nominal: LinearSingleTrackParams, box: dict, speeds=DEFAULT_SPEEDS,
                     points: int = 3) -> dict:
    """Closed-loop spectral abscissa of ``law``'s gains over a parameter box of the design model.

    ``box`` maps ``mass``, ``Iz``, ``C_f``, ``C_r`` to ``(low, high)`` factors on the
    nominal value; every combination of ``points`` values per factor (including the
    vertices) is evaluated at every speed. Returns the worst case per speed and overall.
    This is a sampled check of the linear design model, not a proof for all parameters
    in the box, and says nothing about the nonlinear plant.
    """
    names = ("mass", "Iz", "C_f", "C_r")
    grids = [np.linspace(*box.get(k, (1.0, 1.0)), points if k in box else 1) for k in names]
    mesh = np.array(np.meshgrid(*grids, indexing="ij")).reshape(4, -1).T
    per_speed = []
    worst = {"abscissa": -math.inf}
    for v in speeds:
        K = law.gain(v)
        w = {"speed": float(v), "abscissa": -math.inf, "factors": None,
             "nominal": closed_loop_abscissa(nominal, law.model.inputs, K, v)}
        for f in mesh:
            p = dataclasses.replace(nominal, mass=nominal.mass * f[0], Iz=nominal.Iz * f[1], C_f=nominal.C_f * f[2],
                                    C_r=nominal.C_r * f[3])
            a = closed_loop_abscissa(p, law.model.inputs, K, v)
            if a > w["abscissa"]:
                w["abscissa"], w["factors"] = a, dict(zip(names, map(float, f)))
        per_speed.append(w)
        if w["abscissa"] > worst["abscissa"]:
            worst = {**w}
    return {"box": {k: list(v) for k, v in box.items()}, "points_per_factor": points, "models": int(len(mesh)),
            "per_speed": per_speed, "worst": worst, "stable": bool(worst["abscissa"] < 0.0)}


# ----------------------------------------------------------------------------- B
class DisturbanceObserverLaw:
    """Base law plus yaw-moment compensation ``-I_z d_hat`` from an extended-state observer."""

    def __init__(self, base, design: LinearSingleTrackParams, bandwidth: float = 12.0, max_share: float = 1.0):
        if bandwidth <= 0.0 or not 0.0 < max_share <= 1.0:
            raise ValueError("bandwidth must be > 0 and max_share in (0, 1]")
        self.base = base
        self.params = design
        self.Iz = float(design.Iz)
        self.w_o = float(bandwidth)
        self.l1, self.l2 = 2.0 * self.w_o, self.w_o ** 2
        self.max_share = float(max_share)
        self.name = f"{base.name} + DOB {self.w_o:g} rad/s"
        self.information = getattr(base, "information", INFORMATION_FULL_STATE)
        self.reset()

    def reset(self) -> None:
        self.base.reset()
        self.r_hat = None
        self.d_hat = 0.0
        self._last = (0.0, 0.0, 0.0, 0.0, 1.0)   # (v_y, delta_f, delta_r, M_z, mu_hat) of the last step

    def f_nom(self, vy, r, vx, df, dr, mz, mu) -> float:
        """Nominal yaw acceleration with friction-saturated axle forces [rad/s^2]."""
        p = self.params
        Fzf, Fzr = p.mass * 9.81 * p.lr / (p.lf + p.lr), p.mass * 9.81 * p.lf / (p.lf + p.lr)
        Fyf = float(np.clip(p.C_f * (df - (vy + p.lf * r) / vx), -mu * Fzf, mu * Fzf))
        Fyr = float(np.clip(p.C_r * (dr - (vy - p.lr * r) / vx), -mu * Fzr, mu * Fzr))
        return (p.lf * Fyf * math.cos(df) - p.lr * Fyr + mz) / p.Iz

    def request(self, state: SevenDOFState, ctx: MotionContext) -> MotionRequest:
        vx = max(abs(state.vx), 1.0)
        dt = ctx.dt
        if self.r_hat is None:
            self.r_hat = float(state.r)
        else:   # observer step with the inputs applied over the last period
            vy, df, dr, mz, mu = self._last
            f_nom = self.f_nom(vy, self.r_hat, vx, df, dr, mz, mu)
            e = float(state.r) - self.r_hat
            self.r_hat += dt * (f_nom + self.d_hat + self.l1 * e)
            self.d_hat += dt * self.l2 * e
        req = self.base.request(state, ctx)
        lim = ctx.max_yaw_moment
        comp = float(np.clip(-self.Iz * self.d_hat, -self.max_share * lim, self.max_share * lim))
        mz = float(np.clip(req.yaw_moment + comp, -lim, lim))
        self._last = (float(state.vy), ctx.steer_f, req.rear_steer, mz, ctx.mu_hat)
        diag = {**req.diagnostics, "dob": {"d_hat": self.d_hat, "r_hat": self.r_hat, "compensation": comp,
                                           "base_yaw_moment": req.yaw_moment}}
        return MotionRequest(yaw_moment=mz, rear_steer=req.rear_steer, status=req.status, diagnostics=diag)


# ----------------------------------------------------------------------------- C
@dataclass(frozen=True)
class TighteningSettings:
    horizon: int = 15
    dt: float = 0.02                  # [s] prediction step
    weights: LQRWeights | None = None  # None: LQR defaults with q_z = 1.5e4 (as the LTV-MPC)
    w_max_vy: float = 0.3             # [m/s^2] disturbance bound on v_y_dot
    w_max_r: float = 0.3              # [rad/s^2] disturbance bound on r_dot
    max_iter: int = 100
    tolerance: float = 1e-9
    max_consecutive_failures: int = 5
    slack_l1: float = 1.0e3           # exact-penalty weights of the relaxed problem; slack in units of the limit
    slack_l2: float = 1.0e4


class TighteningMPCYawLaw:
    """Robust MPC by constraint tightening around an LQR ancillary law (sideslip and yaw-rate limits)."""

    information = INFORMATION_FULL_STATE

    def __init__(self, design: LinearSingleTrackParams, inputs=INPUTS_TV, settings: TighteningSettings | None = None,
                 design_speed: float = 20.0):
        self.settings = s = settings or TighteningSettings()
        self.model = YawDesignModel(design, inputs)
        self.weights = s.weights or _weights(inputs, q_z=1.5e4)
        Q, R = self.weights.matrices(self.model.inputs)
        self.speeds = np.array(DEFAULT_SPEEDS)
        self.gains = np.array([lqr_gain(*self.model.augmented(v), Q, R) for v in self.speeds])
        self.name = f"Tightening MPC ({_tag(self.model.inputs)})"
        self.monitor = SolverMonitor(s.max_consecutive_failures)
        self.feasibility = FeasibilityMonitor()
        self.reset()

    gain = LQRYawLaw.gain

    def reset(self) -> None:
        self._z = 0.0
        self._plan = None
        self.monitor.reset()
        self.feasibility.reset()

    def _discrete(self, vx: float):
        A_a, B_a = self.model.augmented(vx)
        n, m = B_a.shape
        M = np.zeros((n + m, n + m))
        M[:n, :n], M[:n, n:] = A_a, B_a
        Ed = expm(M * self.settings.dt)
        return Ed[:n, :n], Ed[:n, n:]

    def tightening(self, vx: float) -> dict:
        """Margins ``h_k`` (k = 1..N) for sideslip, yaw rate and each input at speed ``vx``."""
        s = self.settings
        Ad, Bd = self._discrete(vx)
        K = self.gain(vx)
        AK = Ad - Bd @ K
        E = np.zeros((3, 2))
        E[0, 0] = E[1, 1] = s.dt                    # w acts on (v_y_dot, r_dot) over one step
        w = np.array([s.w_max_vy, s.w_max_r])
        C_beta = np.array([1.0 / vx, 0.0, 0.0])
        C_r = np.array([0.0, 1.0, 0.0])
        h = {"beta": [0.0], "r": [0.0], "u": [np.zeros(K.shape[0])]}
        P = np.eye(3)
        for _ in range(s.horizon - 1):
            h["beta"].append(h["beta"][-1] + float(np.abs(C_beta @ P @ E) @ w))
            h["r"].append(h["r"][-1] + float(np.abs(C_r @ P @ E) @ w))
            h["u"].append(h["u"][-1] + np.abs(K @ P @ E) @ w)
            P = AK @ P
        return {"beta": np.array(h["beta"]), "r": np.array(h["r"]), "u": np.array(h["u"])}

    def request(self, state: SevenDOFState, ctx: MotionContext) -> MotionRequest:
        s = self.settings
        N, inputs = s.horizon, self.model.inputs
        m = len(inputs)
        vx = max(abs(state.vx), 1.0)
        x_ss, u_ss = self.model.equilibrium(vx, ctx.steer_f, ctx.r_ref)
        Ad, Bd = self._discrete(vx)
        K = self.gain(vx)
        AK = Ad - Bd @ K
        e0 = np.array([state.vy - x_ss[0], state.r - x_ss[1], self._z])

        # e_k = Phi_k e0 + sum_{j<k} AK^{k-1-j} Bd c_j, k = 0..N (k = 0: e0)
        powers = [np.eye(3)]
        for _ in range(N):
            powers.append(AK @ powers[-1])
        Phi = np.vstack([powers[k] for k in range(N + 1)])              # (3(N+1), 3)
        Gam = np.zeros((3 * (N + 1), N * m))
        for k in range(1, N + 1):
            for j in range(k):
                Gam[3 * k:3 * k + 3, j * m:(j + 1) * m] = powers[k - 1 - j] @ Bd
        # inputs u_k = u_ss - K e_k + c_k, k = 0..N-1
        Kb = np.kron(np.eye(N), K)
        Ue_phi = -Kb @ Phi[:3 * N]
        Ue_gam = -Kb @ Gam[:3 * N] + np.eye(N * m)
        Q, R = self.weights.matrices(inputs)
        Wq = np.kron(np.eye(N), np.diag(np.sqrt(np.diag(Q))))
        Wr = np.kron(np.eye(N), np.diag(np.sqrt(np.diag(R))))
        A_ls = np.vstack([Wq @ Gam[3:], Wr @ Ue_gam])
        b_ls = np.concatenate([-Wq @ Phi[3:] @ e0, -Wr @ Ue_phi @ e0])

        # tightened constraints G c <= g
        beta_max, r_max = stability_limits(ctx.mu_hat, vx)
        h = self.tightening(vx)
        lim = _input_limits(inputs, ctx)
        rows, rhs = [], []
        for k in range(1, N + 1):
            hk = k - 1
            cb = np.array([1.0 / vx, 0.0, 0.0])
            cr = np.array([0.0, 1.0, 0.0])
            for c, y_ss, ymax, marg in ((cb, x_ss[0] / vx, beta_max, h["beta"][hk]), (cr, x_ss[1], r_max, h["r"][hk])):
                row = c @ Gam[3 * k:3 * k + 3]
                off = c @ Phi[3 * k:3 * k + 3] @ e0 + y_ss
                rows += [row, -row]
                rhs += [ymax - marg - off, ymax - marg + off]
        for k in range(N):
            for i in range(m):
                row = Ue_gam[k * m + i]
                off = Ue_phi[k * m + i] @ e0 + u_ss[i]
                marg = h["u"][k][i]
                rows += [row, -row]
                rhs += [lim[i] - marg - off, lim[i] - marg + off]
        G, g = np.array(rows), np.array(rhs)
        n_state_rows = 4 * N
        # scaling: decision variables in units of the input limits, every row in units of its limit
        # (SLSQP fails on the mixed rad / N m scales otherwise)
        D = np.tile(lim, N)
        row_scale = np.r_[np.tile([beta_max, beta_max, r_max, r_max], N), np.tile(np.repeat(lim, 2), N)]
        G = G * D[None, :] / row_scale[:, None]
        g = g / row_scale
        A_ls = A_ls * D[None, :]

        cost = lambda c: float(np.sum((A_ls @ c - b_ls) ** 2))
        grad = lambda c: 2.0 * A_ls.T @ (A_ls @ c - b_ls)
        warm = np.zeros(N * m) if self._plan is None else np.r_[self._plan[m:], np.zeros(m)]
        tol_g = 1e-6 * (1.0 + float(np.max(np.abs(g))))
        t0 = time.perf_counter()
        res = minimize(cost, warm, jac=grad, method="SLSQP",
                       constraints=[{"type": "ineq", "fun": lambda c: g - G @ c, "jac": lambda c: -G}],
                       options={"maxiter": s.max_iter, "ftol": s.tolerance})
        x = res.x if np.all(np.isfinite(res.x)) else None
        feasible = x is not None and bool(res.success) and float(np.max(G @ x - g)) <= tol_g
        slack = (0.0, 0.0)
        G_k, g_k, x_k, ok, msg, nit = G, g, x, bool(res.success), str(res.message), int(getattr(res, "nit", 0))
        cost_k, grad_k, warm_k = cost, grad, warm
        if not feasible:   # relaxed problem (exact penalty on two slacks: state rows, input rows)
            S = np.zeros((len(g), 2))
            S[:n_state_rows, 0] = S[n_state_rows:, 1] = 1.0
            w1, w2 = s.slack_l1, s.slack_l2
            cost_k = lambda y: cost(y[:-2]) + w1 * float(np.sum(y[-2:])) + w2 * float(np.sum(y[-2:] ** 2))
            grad_k = lambda y: np.r_[grad(y[:-2]), w1 + 2.0 * w2 * y[-2:]]
            G_k = np.hstack([G, -S])
            viol0 = np.maximum(G @ warm - g, 0.0)
            warm_k = np.r_[warm, viol0[:n_state_rows].max(initial=0.0), viol0[n_state_rows:].max(initial=0.0)]
            res = minimize(cost_k, warm_k, jac=grad_k, method="SLSQP", bounds=[(None, None)] * (N * m) + [(0, None)] * 2,
                           constraints=[{"type": "ineq", "fun": lambda y: g - G_k @ y, "jac": lambda y: -G_k}],
                           options={"maxiter": s.max_iter, "ftol": s.tolerance})
            x_k = res.x if np.all(np.isfinite(res.x)) else None
            ok = bool(res.success) and x_k is not None and float(np.max(G_k @ x_k - g)) <= tol_g
            msg = "tightened problem infeasible; relaxed: " + str(res.message)
            nit += int(getattr(res, "nit", 0))
            if x_k is not None:
                slack = (float(x_k[-1 - 1]), float(x_k[-1]))
        solve_ms = (time.perf_counter() - t0) * 1e3
        c_x = cost_k(x_k) if x_k is not None else float("nan")
        kkt = NO_KKT
        if x_k is not None:
            lo = None if feasible else np.r_[np.full(N * m, -np.inf), 0.0, 0.0]
            kkt = kkt_residuals(grad_k(x_k), x_k, lower=lo, ineq_value=g - G_k @ x_k, ineq_jacobian=-G_k, cost=c_x)
        report = self.monitor.classify(ok, x_k if ok else None, c_x, cost_k(warm_k), msg, nit, solve_ms,
                                       x_warm_start=warm_k, kkt=kkt)
        c_hat = x_k[:N * m] if report.accepted else np.zeros(N * m)   # failed relaxed solve: ancillary LQR (c = 0)
        self._plan = c_hat
        c = c_hat * D
        u = u_ss - K @ e0 + c[:m]
        u_sat = np.clip(u, -lim, lim)
        if np.allclose(u_sat, u, rtol=0, atol=1e-9 * (1 + np.abs(u).max())):
            self._z += (state.r - ctx.r_ref) * ctx.dt
        e_all = (Phi @ e0 + Gam @ c).reshape(N + 1, 3)[1:]
        U = (u_ss + (Ue_phi @ e0 + Ue_gam @ c).reshape(N, m))
        values = {i: U[:, k] for k, i in enumerate(inputs)}
        values["predicted_sideslip"] = (e_all[:, 0] + x_ss[0]) / vx
        values["predicted_yaw_rate"] = e_all[:, 1] + x_ss[1]
        cons = ConstraintSet([*(Constraint(i, HARD, float(l), note="tightened, SLSQP") for i, l in zip(inputs, lim)),
                              Constraint("predicted_sideslip", HARD, beta_max, unit="rad", note="tightened by h_k"),
                              Constraint("predicted_yaw_rate", HARD, r_max, unit="rad/s", note="tightened by h_k")])
        feas = self.feasibility.record(cons.check(values))
        return _to_request(inputs, u_sat, status=report.status,
                           diagnostics={"solver": report.as_dict(), "feasibility": feas.as_dict(),
                                        "tightened_problem_feasible": feasible, "slack": {"state": slack[0], "input": slack[1]},
                                        "z": self._z,
                                        "margin_beta_end": float(h["beta"][-1]), "margin_r_end": float(h["r"][-1])})


# ----------------------------------------------------------------------------- D
@dataclass(frozen=True)
class AdaptationSettings:
    forgetting: float = 0.998         # lambda per control step
    p0: float = 0.05                  # initial covariance: prior std ~0.22 around the nominal theta = 1
    phi_min: float = 1.0              # [m/s^2] excitation threshold on the regressor
    linear_range: float = 0.5         # update only for |a_y| <= linear_range * mu_hat * g (inf: always)
    max_moment_share: float = 0.25    # ... and |M_z| of the last step <= share * max yaw moment (1: always)
    theta_min: float = 0.3
    theta_max: float = 1.5
    tau: float = 0.5                  # [s] low-pass time constant of the theta used for the gains
    theta_grid: tuple = (0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.15, 1.3, 1.5)


class AdaptiveLQRYawLaw:
    """LQR whose design model follows an RLS estimate of a cornering-stiffness factor (gain table, slow)."""

    information = INFORMATION_FULL_STATE

    def __init__(self, design: LinearSingleTrackParams, inputs=INPUTS_TV, settings: AdaptationSettings | None = None,
                 weights: LQRWeights | None = None, schedule_speeds=DEFAULT_SPEEDS):
        self.settings = s = settings or AdaptationSettings()
        self.design = design
        self.inputs = tuple(inputs)
        self.weights = weights or _weights(inputs)
        self.thetas = np.array(sorted(s.theta_grid), dtype=float)
        if not (self.thetas[0] <= 1.0 <= self.thetas[-1]):
            raise ValueError("theta_grid must contain 1.0 in its range")
        self.speeds = np.array(sorted(schedule_speeds), dtype=float)
        # table designed once before operation: one LQR per (theta, speed)
        self.models = [YawDesignModel(self._scaled(t), inputs) for t in self.thetas]
        Q, R = self.weights.matrices(self.inputs)
        self.gains = np.array([[lqr_gain(*mdl.augmented(v), Q, R) for v in self.speeds] for mdl in self.models])
        self.name = f"Adaptive LQR ({_tag(self.inputs)})"
        self.reset()

    def _scaled(self, theta: float) -> LinearSingleTrackParams:
        return dataclasses.replace(self.design, C_f=self.design.C_f * theta, C_r=self.design.C_r * theta)

    def reset(self) -> None:
        s = self.settings
        self.theta_rls = 1.0
        self.theta = 1.0
        self.P = s.p0
        self._z = 0.0
        self._prev = None
        self.updates = 0

    def _interp(self, table: np.ndarray, theta: float, v: float):
        t = float(np.clip(theta, self.thetas[0], self.thetas[-1]))
        v = float(np.clip(v, self.speeds[0], self.speeds[-1]))
        i = int(np.clip(np.searchsorted(self.thetas, t) - 1, 0, len(self.thetas) - 2))
        j = int(np.clip(np.searchsorted(self.speeds, v) - 1, 0, len(self.speeds) - 2))
        a = (t - self.thetas[i]) / (self.thetas[i + 1] - self.thetas[i])
        b = (v - self.speeds[j]) / (self.speeds[j + 1] - self.speeds[j])
        return ((1 - a) * (1 - b) * table[i, j] + a * (1 - b) * table[i + 1, j]
                + (1 - a) * b * table[i, j + 1] + a * b * table[i + 1, j + 1])

    def _adapt(self, state: SevenDOFState, ctx: MotionContext, steer_r: float, mz: float) -> None:
        s, d = self.settings, self.design
        vx = max(abs(state.vx), 1.0)
        if self._prev is not None:   # balance over the last interval: state and inputs at its start
            vy_p, r_p, vx_p, df, dr, mz_p = self._prev
            ay = (state.vy - vy_p) / ctx.dt + r_p * vx_p
            a_f = df - (vy_p + d.lf * r_p) / vx_p
            a_r = dr - (vy_p - d.lr * r_p) / vx_p
            phi = (d.C_f * a_f + d.C_r * a_r) / d.mass
            lin = s.linear_range * ctx.mu_hat * 9.81
            if (abs(phi) >= s.phi_min and abs(ay) <= lin and abs(self.theta_rls * phi) <= lin
                    and abs(mz_p) <= s.max_moment_share * ctx.max_yaw_moment):
                k = self.P * phi / (s.forgetting + phi * phi * self.P)
                self.theta_rls = float(np.clip(self.theta_rls + k * (ay - self.theta_rls * phi), s.theta_min, s.theta_max))
                self.P = (self.P - k * phi * self.P) / s.forgetting
                self.updates += 1
        self.theta += (ctx.dt / (s.tau + ctx.dt)) * (self.theta_rls - self.theta)
        self._prev = (float(state.vy), float(state.r), vx, ctx.steer_f, steer_r, mz)

    def request(self, state: SevenDOFState, ctx: MotionContext) -> MotionRequest:
        vx = max(abs(state.vx), 1.0)
        # feedforward and gain from the two neighbouring theta models (interpolated)
        t = float(np.clip(self.theta, self.thetas[0], self.thetas[-1]))
        i = int(np.clip(np.searchsorted(self.thetas, t) - 1, 0, len(self.thetas) - 2))
        a = (t - self.thetas[i]) / (self.thetas[i + 1] - self.thetas[i])
        eq = [self.models[k].equilibrium(vx, ctx.steer_f, ctx.r_ref) for k in (i, i + 1)]
        x_ss = (1 - a) * eq[0][0] + a * eq[1][0]
        u_ss = (1 - a) * eq[0][1] + a * eq[1][1]
        K = self._interp(self.gains, self.theta, vx)
        e = np.array([state.vy - x_ss[0], state.r - x_ss[1], self._z])
        u = u_ss - K @ e
        lim = _input_limits(self.inputs, ctx)
        u_sat = np.clip(u, -lim, lim)
        if np.allclose(u_sat, u):
            self._z += (state.r - ctx.r_ref) * ctx.dt
        steer_r = float(u_sat[self.inputs.index("rear_steer")]) if "rear_steer" in self.inputs else 0.0
        mz = float(u_sat[self.inputs.index("yaw_moment")])
        self._adapt(state, ctx, steer_r, mz)
        return _to_request(self.inputs, u_sat, diagnostics={"theta": self.theta, "theta_rls": self.theta_rls,
                                                            "rls_updates": self.updates, "z": self._z})


__all__ = ("AdaptationSettings", "AdaptiveLQRYawLaw", "DisturbanceObserverLaw", "RobustLQRYawLaw", "TighteningMPCYawLaw",
           "TighteningSettings", "closed_loop_abscissa", "robust_stability")
