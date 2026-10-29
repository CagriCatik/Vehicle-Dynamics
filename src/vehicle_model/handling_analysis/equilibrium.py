"""Nonlinear equilibria and their local stability (Plan V3.0 M11 6.1, 6.2).

States of the analysis are the body-fixed states of the model: ``vx, vy, r``, the four
wheel speeds and, for :class:`~vehicle_model.vehicle_models.TransientSevenDOFModel`,
the tyre relaxation states. The navigation states ``X, Y, psi`` are left out: in a
steady corner they grow with time, and on a uniform road they do not act back on
the dynamics.

Operating point (:class:`OperatingPoint`): longitudinal speed (held), front steer,
rear steer as a ratio of the front steer, brake torques, the split of the drive
torque over the wheels and an optional torque-vectoring yaw moment (differential
drive torque on the rear axle). Friction, centre-of-gravity position and tyres
belong to the model (road, ``params``, tyre objects), so any model of the
:class:`~vehicle_model.vehicle_models.SevenDOFModel` family with any
:class:`~vehicle_model.tire_model.TyreModel` can be analysed.

Equilibrium: unknowns are all body states except ``vx`` and the total drive torque
that holds the speed; equations are "all body-state derivatives are zero". A point
is accepted when every residual group is below :data:`RESIDUAL_TOL`, the load-transfer
fixed point converged and the solver reported success. Wheel contact (four or three
wheels) is reported with it.

Local stability: central-difference Jacobian of the body-state derivative with a
step-size study (:meth:`EquilibriumAnalysis.linearise`), eigenvalues, participation
factors per state group, damping and natural frequency. :meth:`EquilibriumAnalysis.linear_check`
compares the linear prediction ``expm(A t) dx0`` with the nonlinear model started from
a small perturbation, so the local statement is checked against the dynamics it is
derived from.
"""

from __future__ import annotations

import copy
import dataclasses
import math
from dataclasses import dataclass

import numpy as np
from scipy.linalg import expm
from scipy.optimize import least_squares, linear_sum_assignment

from vehicle_model.core.types import SevenDOFControlInput
from vehicle_model.roads.surface import UniformRoad

from .linearization import linear_params_from_seven_dof
from .linear_single_track import LinearSingleTrack

# Residual tolerances per state group, SI units of the derivative.
RESIDUAL_TOL = {
    "translational": 1e-6,   # [m/s^2]  vx_dot, vy_dot
    "yaw": 1e-6,             # [rad/s^2] r_dot
    "wheel": 1e-5,           # [rad/s^2] omega_dot (I_w/R_eff = 4.7 kg: 5e-5 N of tyre force)
    "relaxation": 1e-6,      # [1/s]     tyre relaxation states
}
MARGINAL_TOL = 1e-3          # [1/s] |Re(lambda)| below this: marginal
JACOBIAN_STEPS = (1e-2, 1e-3, 1e-4, 1e-5, 1e-6, 1e-7)   # relative central-difference steps
JACOBIAN_CONVERGED = 1e-4    # accepted relative eigenvalue change between neighbouring steps
SIM_LOAD_TRANSFER_TOL = 1e-9  # [m/s^2] load-transfer tolerance of the simulation in linear_check
_NAV = (7, 8, 9)             # X, Y, psi in the model's state array


@dataclass(frozen=True)
class OperatingPoint:
    speed: float                                   # longitudinal speed vx [m/s], held by the drive torque
    steer_f: float = 0.0                           # front axle steer angle [rad]
    rear_steer_ratio: float = 0.0                  # steer_r = ratio * steer_f [-]
    tv_yaw_moment: float = 0.0                     # [N m] differential drive torque on the rear axle
    drive_split: tuple = (0.25, 0.25, 0.25, 0.25)  # share of the total drive torque per wheel, sums to 1
    brake_torque: tuple = (0.0, 0.0, 0.0, 0.0)     # [N m] per wheel

    def __post_init__(self):
        if not self.speed > 0.5:
            raise ValueError("operating speed must be above 0.5 m/s")
        if len(self.drive_split) != 4 or abs(sum(self.drive_split) - 1.0) > 1e-9:
            raise ValueError("drive_split needs 4 shares that sum to 1")
        if len(self.brake_torque) != 4 or min(self.brake_torque) < 0.0:
            raise ValueError("brake_torque needs 4 non-negative values")

    @property
    def steer_r(self) -> float:
        return self.rear_steer_ratio * self.steer_f

    def inputs(self, params, drive_total: float) -> SevenDOFControlInput:
        dT = self.tv_yaw_moment * params.Reff / params.wr   # yaw moment = w_r * dF on the rear axle
        T_drive = drive_total * np.asarray(self.drive_split, dtype=float) + np.array([0.0, 0.0, -dT, dT])
        return SevenDOFControlInput(steer_f=self.steer_f, steer_r=self.steer_r, T_drive=T_drive,
                                    T_brake=np.asarray(self.brake_torque, dtype=float))

    def to_dict(self) -> dict:
        return {"speed_mps": self.speed, "steer_f_deg": math.degrees(self.steer_f),
                "rear_steer_ratio": self.rear_steer_ratio, "tv_yaw_moment_nm": self.tv_yaw_moment,
                "drive_split": list(self.drive_split), "brake_torque_nm": list(self.brake_torque)}


@dataclass
class Mode:
    eigenvalue: complex
    group: str                       # state group with the largest participation
    participation: dict              # group -> share, sums to 1
    damping_ratio: float             # -Re/|lambda|
    natural_frequency_hz: float      # |lambda|/(2 pi); nan for real eigenvalues

    def to_dict(self) -> dict:
        return {"re": self.eigenvalue.real, "im": self.eigenvalue.imag, "group": self.group,
                "participation": self.participation, "damping_ratio": self.damping_ratio,
                "natural_frequency_hz": self.natural_frequency_hz}


@dataclass
class Linearisation:
    A: np.ndarray
    state_names: tuple
    step: float                      # relative step used for A
    step_study: list                 # [{step, matrix_change, eigenvalue_change}] between neighbouring steps
    converged: bool                  # eigenvalue change at the chosen step <= JACOBIAN_CONVERGED
    modes: list

    @property
    def eigenvalues(self) -> np.ndarray:
        return np.array([m.eigenvalue for m in self.modes])

    @property
    def critical_mode(self) -> Mode:
        """The mode with the largest real part: it decides local stability."""
        return self.modes[0]

    def max_real(self) -> float:
        return self.critical_mode.eigenvalue.real

    def stability(self) -> str:
        """``stable`` / ``marginal`` / ``unstable`` from the largest real part (tolerance MARGINAL_TOL)."""
        re = self.max_real()
        return "unstable" if re > MARGINAL_TOL else ("stable" if re < -MARGINAL_TOL else "marginal")

    def lateral_modes(self) -> list:
        """Modes dominated by ``vy`` and ``r`` (the handling modes)."""
        return [m for m in self.modes if m.group == "lateral_yaw"]

    def to_dict(self) -> dict:
        crit = self.critical_mode
        return {"step": self.step, "converged": self.converged, "step_study": self.step_study,
                "stability": self.stability(), "max_real": self.max_real(),
                "critical_mode_group": crit.group, "critical_mode_participation": crit.participation,
                "modes": [m.to_dict() for m in self.modes]}


@dataclass
class Equilibrium:
    op: OperatingPoint
    mu: float
    state: object                    # model.state_type at X = Y = psi = 0
    drive_total: float               # [N m] total drive torque holding the speed
    residual: dict                   # group -> max |derivative| in SI units
    accepted: bool                   # residuals within RESIDUAL_TOL, load transfer converged, solver success
    support: str                     # "four_wheel" | "three_wheel" | "tip_over"
    ax: float
    ay: float
    max_abs_kappa: float             # largest longitudinal slip [-]
    max_abs_alpha: float             # largest slip angle [rad]
    solver_message: str = ""
    linearisation: Linearisation | None = None
    linear_check: dict | None = None

    @property
    def beta(self) -> float:
        return math.atan2(self.state.vy, self.state.vx)

    @property
    def r(self) -> float:
        return float(self.state.r)

    def to_dict(self) -> dict:
        s = self.state
        return {"operating_point": self.op.to_dict(), "mu": self.mu, "accepted": self.accepted,
                "residual": self.residual, "support": self.support, "solver_message": self.solver_message,
                "max_abs_kappa": self.max_abs_kappa, "max_abs_alpha_deg": math.degrees(self.max_abs_alpha),
                "vx": float(s.vx), "vy": float(s.vy), "r_degps": math.degrees(s.r),
                "beta_deg": math.degrees(self.beta), "ay": self.ay, "ax": self.ax,
                "omega": [float(w) for w in s.omega], "drive_total_nm": self.drive_total,
                "linearisation": self.linearisation.to_dict() if self.linearisation else None,
                "linear_check": self.linear_check}


@dataclass
class EquilibriumAnalysis:
    """Equilibria, Jacobians and modes of a :class:`SevenDOFModel`-family model on a uniform road."""

    model: object
    load_transfer_tol: float = 1e-12
    jacobian_steps: tuple = JACOBIAN_STEPS

    def __post_init__(self):
        if not isinstance(self.model.road, UniformRoad):
            raise ValueError("equilibria need a uniform road (friction must not depend on position)")
        # own copies: tight load-transfer tolerance, so the derivative is reproducible to rounding error;
        # a looser one for the simulation in linear_check (its error is ~1e-5 of the perturbation dynamics)
        self._sim_model = copy.copy(self.model)
        self._sim_model.load_transfer_tol = SIM_LOAD_TRANSFER_TOL
        self._sim_model.load_transfer_max_iter = 500
        self.model = copy.copy(self.model)
        self.model.load_transfer_tol = self.load_transfer_tol
        self.model.load_transfer_max_iter = 500
        n = self.model.n_states
        self.body = np.array([i for i in range(n) if i not in _NAV])
        names = ["vx", "vy", "r", "omega_FL", "omega_FR", "omega_RL", "omega_RR"]
        names += [f"x{i}" for i in range(10, n)]
        self.state_names = tuple(names)
        self.groups = tuple(["longitudinal", "lateral_yaw", "lateral_yaw"] + ["wheel"] * 4
                            + ["relaxation"] * (n - 10))
        # typical magnitudes: Jacobian steps and solver scaling
        self._typical = np.array([1.0, 1.0, 0.1] + [1.0] * 4 + [0.01] * (n - 10))

    # ------------------------------------------------------------------ helpers
    @property
    def params(self):
        return self.model.params

    @property
    def mu(self) -> float:
        return float(self.model.road.mu)

    def _full(self, xb: np.ndarray) -> np.ndarray:
        y = np.zeros(self.model.n_states)
        y[self.body] = xb
        return y

    def derivative(self, xb: np.ndarray, u: SevenDOFControlInput) -> np.ndarray:
        """Body-state derivative ``f(x, u)``."""
        d, _, _, _ = self.model.evaluate_dynamics(self.model.state_type.from_array(self._full(xb)), u)
        return d[self.body]

    def _residual_groups(self, d: np.ndarray) -> dict:
        out = {"translational": float(max(abs(d[0]), abs(d[1]))), "yaw": float(abs(d[2])),
               "wheel": float(np.max(np.abs(d[3:7])))}
        if d.size > 7:
            out["relaxation"] = float(np.max(np.abs(d[7:])))
        return out

    def _free_rolling(self, vx: float, vy: float, r: float, op: OperatingPoint) -> np.ndarray:
        st = self.model.state_type.from_array(self._full(np.r_[vx, vy, r, np.zeros(self.body.size - 3)]))
        _, _, _, v_wx, _ = self.model.wheel_velocities(st, op.inputs(self.params, 0.0))
        return v_wx / self.params.Reff

    def _initial_guess(self, op: OperatingPoint) -> np.ndarray:
        """Linear single-track steady state (yaw rate limited to the friction), free-rolling wheels."""
        p = self.params
        lin = LinearSingleTrack(linear_params_from_seven_dof(p, self.model.tyre_front, self.model.tyre_rear, self.mu))
        v = op.speed
        delta = op.steer_f - op.steer_r
        r_lim = 0.95 * self.mu * p.g / v
        r = float(np.clip(lin.yaw_rate_gain(v) * delta, -r_lim, r_lim))
        beta = lin.sideslip_gain(v) * delta * (r / (lin.yaw_rate_gain(v) * delta) if delta else 1.0)
        return self._guess_from(op, beta, r)

    def _guess_from(self, op: OperatingPoint, beta: float, r: float) -> np.ndarray:
        p = self.params
        v = op.speed
        vy = v * math.tan(beta)
        z = np.zeros(self.body.size)  # unknowns: vy, r, omega x4, extra..., drive_total
        z[0], z[1] = vy, r
        z[2:6] = self._free_rolling(v, vy, r, op)
        drag = 0.5 * p.rho * p.Cd * p.Af * v * v + p.c_rr * p.mass * p.g
        z[-1] = drag * p.Reff + float(np.sum(op.brake_torque))
        return z

    def _residual_scale(self, op: OperatingPoint) -> np.ndarray:
        p = self.params
        s = np.ones(self.body.size)
        s[0:2] = 1.0 / p.g
        s[2] = p.Iz / (p.mass * p.g * p.wheelbase)
        s[3:7] = p.Iw / (p.Reff * p.mass * p.g)
        return s

    # ---------------------------------------------------------------- equilibria
    def solve(self, op: OperatingPoint, guess: np.ndarray | None = None) -> Equilibrium:
        """Equilibrium for ``op`` from ``guess`` (unknown vector, see :meth:`unknowns`) or the linear guess."""
        p = self.params
        z0 = self._initial_guess(op) if guess is None else np.asarray(guess, dtype=float)
        zscale = np.r_[max(op.speed, 1.0), max(p.g / op.speed, 0.05), np.full(4, op.speed / p.Reff),
                       self._typical[7:], max(abs(z0[-1]), 100.0)]
        rs = self._residual_scale(op)

        def unpack(zn):
            z = zn * zscale
            xb = np.r_[op.speed, z[:-1]]
            return xb, op.inputs(p, z[-1])

        def fun(zn):
            xb, u = unpack(zn)
            return self.derivative(xb, u) * rs

        sol = least_squares(fun, z0 / zscale, method="lm", xtol=1e-15, ftol=1e-15, gtol=1e-15, max_nfev=4000)
        xb, u = unpack(sol.x)
        st = self.model.state_type.from_array(self._full(xb))
        d, tel, ax, ay = self.model.evaluate_dynamics(st, u)
        res = self._residual_groups(d[self.body])
        ok = (bool(np.all(np.isfinite(d))) and tel.load_transfer_converged and tel.support != "tip_over"
              and all(res[g] <= RESIDUAL_TOL[g] for g in res) and bool(np.all(st.omega > 0.0)))
        return Equilibrium(op, self.mu, st, float(sol.x[-1] * zscale[-1]), res, ok, tel.support, ax, ay,
                           float(np.max(np.abs(tel.kappa))), float(np.max(np.abs(tel.alpha))), sol.message)

    @staticmethod
    def unknowns(eq: Equilibrium) -> np.ndarray:
        """Unknown vector of ``eq`` (``vy, r, omega x4, extra..., drive_total``): a guess for a nearby point."""
        y = eq.state.to_array()
        return np.r_[y[1:7], y[10:], eq.drive_total]

    def branch(self, ops: list[OperatingPoint], guess: np.ndarray | None = None) -> list[Equilibrium]:
        """Equilibria along a sequence of operating points by natural continuation."""
        out = []
        for op in ops:
            eq = self.solve(op, guess)
            if not eq.accepted and guess is not None:  # continuation lost the branch: retry from the linear guess
                retry = self.solve(op)
                eq = retry if retry.accepted else eq
            out.append(eq)
            guess = self.unknowns(eq) if eq.accepted else guess
        return out

    def find_all(self, op: OperatingPoint, beta_seeds_deg=(-20, -12, -6, 0, 6, 12, 20),
                 r_seeds=(-1.0, -0.5, 0.0, 0.5, 1.0), max_beta_deg: float = 30.0,
                 max_abs_kappa: float = 0.2) -> list[Equilibrium]:
        """All accepted equilibria reached from a seed grid (``r`` seeds in units of mu g / v), deduplicated.

        Seeds cover sideslip and yaw rate; an equilibrium reached from no seed may still exist,
        so the set is "all equilibria found", not a proof of completeness. Solutions with
        ``|kappa| > max_abs_kappa`` are dropped: there the drive torque that holds the speed
        spins the wheels up (several times the road speed at large sideslip), an artefact of
        the speed-holding formulation rather than a driving state.
        """
        p = self.params
        found: list[Equilibrium] = []
        r_unit = self.mu * p.g / op.speed
        seeds = [self._initial_guess(op)] + [self._guess_from(op, math.radians(b), k * r_unit)
                                             for b in beta_seeds_deg for k in r_seeds]
        for z in seeds:
            eq = self.solve(op, z)
            if not eq.accepted or abs(math.degrees(eq.beta)) > max_beta_deg or eq.max_abs_kappa > max_abs_kappa:
                continue
            if any(abs(eq.state.vy - e.state.vy) < 1e-4 * op.speed and abs(eq.r - e.r) < 1e-5 for e in found):
                continue
            found.append(eq)
        return sorted(found, key=lambda e: e.beta)

    # ----------------------------------------------------------- linearisation
    def jacobian(self, eq: Equilibrium, rel_step: float) -> np.ndarray:
        xb = eq.state.to_array()[self.body]
        u = eq.op.inputs(self.params, eq.drive_total)
        h = rel_step * np.maximum(np.abs(xb), self._typical)
        A = np.empty((xb.size, xb.size))
        for j in range(xb.size):
            e = np.zeros(xb.size)
            e[j] = h[j]
            A[:, j] = (self.derivative(xb + e, u) - self.derivative(xb - e, u)) / (2.0 * h[j])
        return A

    def linearise(self, eq: Equilibrium) -> Linearisation:
        """Jacobian with a step-size study; the step after the smallest neighbouring change is used.

        For neighbouring steps ``h_k > h_{k+1}`` the study reports the relative matrix change and
        the largest relative eigenvalue change (eigenvalues paired by minimum distance). Too large
        steps show truncation error, too small ones rounding error; the minimum marks the plateau.
        """
        mats = [self.jacobian(eq, h) for h in self.jacobian_steps]
        study = []
        for k in range(len(mats) - 1):
            a, b = mats[k], mats[k + 1]
            study.append({"step": self.jacobian_steps[k + 1],
                          "matrix_change": float(np.max(np.abs(a - b)) / max(np.max(np.abs(b)), 1e-12)),
                          "eigenvalue_change": _eig_change(a, b)})
        k_best = int(np.argmin([s["eigenvalue_change"] for s in study]))
        A = mats[k_best + 1]
        lin = Linearisation(A, self.state_names, self.jacobian_steps[k_best + 1], study,
                            study[k_best]["eigenvalue_change"] <= JACOBIAN_CONVERGED, self.modes(A))
        eq.linearisation = lin
        return lin

    def modes(self, A: np.ndarray) -> list[Mode]:
        lam, V = np.linalg.eig(A)
        W = np.linalg.inv(V)
        out = []
        groups = sorted(set(self.groups))
        for k in np.argsort(-lam.real):
            pf = np.abs(V[:, k] * W[k, :])
            pf = pf / pf.sum()
            share = {g: float(sum(pf[i] for i, gi in enumerate(self.groups) if gi == g)) for g in groups}
            mag = abs(lam[k])
            out.append(Mode(complex(lam[k]), max(share, key=share.get), share,
                            float(-lam[k].real / mag) if mag > 0 else float("nan"),
                            float(mag / (2 * math.pi)) if abs(lam[k].imag) > 1e-9 else float("nan")))
        return out

    def linear_check(self, eq: Equilibrium, d_vy: float = 0.02, d_r: float = 0.002,
                     duration: float = 1.0, dt: float = 0.005) -> dict:
        """Nonlinear response to a small ``(vy, r)`` perturbation against ``expm(A t) dx0``.

        Returns the largest deviation of the lateral states (``vy``, ``r``) relative to the
        largest linear excursion, and whether both decay or grow alike. For an unstable
        equilibrium the horizon is shortened to one time constant of the fastest growth,
        so the perturbation stays small.
        """
        lin = eq.linearisation or self.linearise(eq)
        if lin.max_real() > MARGINAL_TOL:
            duration = min(duration, 1.0 / lin.max_real())
        u = eq.op.inputs(self.params, eq.drive_total)
        x_star = eq.state.to_array()
        dx0 = np.zeros(self.body.size)
        dx0[1], dx0[2] = d_vy, d_r
        y = x_star.copy()
        y[self.body] += dx0
        st = self.model.state_type.from_array(y)
        n = max(1, int(round(duration / dt)))
        err = np.zeros(2)
        peak = np.abs(dx0[1:3]).copy()
        Phi = expm(lin.A * dt)
        dx_lin = dx0.copy()
        for _ in range(n):
            st, _ = self._sim_model.step(st, u, dt)
            dx_lin = Phi @ dx_lin
            dx_nl = st.to_array()[self.body] - x_star[self.body]
            err = np.maximum(err, np.abs(dx_nl[1:3] - dx_lin[1:3]))
            peak = np.maximum(peak, np.abs(dx_lin[1:3]))
        rel = err / peak
        end_lin = float(np.linalg.norm(dx_lin[1:3] / [d_vy, d_r]))
        end_nl = float(np.linalg.norm(dx_nl[1:3] / [d_vy, d_r]))
        out = {"duration_s": n * dt, "dt": dt, "perturbation": {"vy": d_vy, "r": d_r},
               "relative_error_vy": float(rel[0]), "relative_error_r": float(rel[1]),
               "growth_linear": end_lin, "growth_nonlinear": end_nl,
               "same_tendency": (end_lin < 1.0) == (end_nl < 1.0)}
        eq.linear_check = out
        return out


def _eig_change(a: np.ndarray, b: np.ndarray) -> float:
    la, lb = np.linalg.eigvals(a), np.linalg.eigvals(b)
    cost = np.abs(la[:, None] - lb[None, :])
    i, j = linear_sum_assignment(cost)
    return float(np.max(cost[i, j] / np.maximum(np.abs(lb[j]), 1.0)))


def with_params(model, **changes):
    """Copy of ``model`` with changed vehicle parameters (e.g. ``lf``/``lr`` for the CoG position).

    Not for a model with K&C chassis: its geometry was built for the original parameters.
    """
    if getattr(model, "chassis", None) is not None:
        raise ValueError("with_params cannot change a model with a K&C chassis; build it from a changed vehicle file")
    m = copy.copy(model)
    m.params = dataclasses.replace(model.params, **changes)
    m._load_params = dataclasses.replace(model._load_params, **changes)
    return m


__all__ = ("JACOBIAN_STEPS", "MARGINAL_TOL", "RESIDUAL_TOL", "Equilibrium", "EquilibriumAnalysis", "Linearisation",
           "Mode", "OperatingPoint", "with_params")
