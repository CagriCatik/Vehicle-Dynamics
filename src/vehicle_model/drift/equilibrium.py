"""Stationary drift equilibria of the existing 7-DOF plant (Drift plan D2, section 4.2).

Configuration V1: rear-wheel drive with equal torque on both rear wheels (open differential,
``T_drive = [0, 0, T, T]``), front steer only, no brakes, front wheels free rolling.

Problem. For a reference ``(v, beta)`` the body states are ``q = [vx, vy, r, omega x4]``.
Speed and sideslip are imposed by substitution, ``vx = v cos(beta)``, ``vy = v sin(beta)``;
the unknowns are ``z = [r, omega_FL, omega_FR, omega_RL, omega_RR, delta_f, T]`` (7) and the
equations are the 7 body-state derivatives of the plant (``vx_dot = vy_dot = r_dot = 0``,
``omega_dot_i = 0``). This is the plan's 9x9 problem (9 unknowns ``q, delta_f, T``; 7 dynamic
residuals plus speed and sideslip) with the two kinematic conditions eliminated exactly: square,
no over-determination claimed. The radius ``R = v/|r|`` is a result, not a condition.

Solver. Bounded ``scipy.optimize.least_squares`` (trust region reflective) on residuals scaled
to comparable units (accelerations by ``g``, yaw acceleration by ``m g L / I_z``, wheel
accelerations by ``m g R / I_w``); bounds from the vehicle limits (steer, drive torque) and
``omega >= 0``. Seeds are a fixed, deterministic grid over yaw rate, steer and rear slip;
with ``multistart="all"`` every seed is solved and distinct branches are returned.

Acceptance (``validity``): ``valid`` needs all of

    residual groups within RESIDUAL_TOL (same tolerances as handling_analysis.equilibrium)
    load transfer converged, four wheels in contact
    steer and torque strictly inside their limits (a solution on a bound is reported as
    ``actuator_limit``: the requested drift needs more than the actuator has)
    a drift: |r| >= min_yaw_rate, |beta| >= min_beta, r opposite in sign to beta
    (yaw rate towards the inside, velocity pointing outwards: ``r * beta < 0``), and both
    rear tyres beyond the slip angle of their pure-lateral force peak at their actual load
    (the rear axle saturated). The sign condition alone also admits fast ordinary cornering
    (beta < 0 at r > 0 with the rear below its peak); such equilibria are reported as
    ``rear_axle_below_lateral_peak``, not as drifts.
    non-negative wheel speeds

``valid_extrapolated``: as ``valid``, but a tyre works outside its declared validity range.
Anything else is ``infeasible`` with the list of reasons; no forced solution is reported.
"""

from __future__ import annotations

import itertools
import math
from dataclasses import dataclass

import numpy as np
from scipy.optimize import least_squares

from vehicle_model.core.types import SevenDOFControlInput
from vehicle_model.handling_analysis.equilibrium import RESIDUAL_TOL
from vehicle_model.roads.surface import UniformRoad

from .plant_adapter import DeterministicPlant
from .tyre_audit import extrapolating
from .types import (ACTUATOR_LIMIT, INFEASIBLE, LOAD_TRANSFER_NOT_CONVERGED, NEGATIVE_WHEEL_SPEED, REAR_NOT_SATURATED,
                    RESIDUAL_TOO_LARGE,
                    SOLVER_NOT_CONVERGED, TIP_OVER, TRIVIAL, VALID, VALID_EXTRAPOLATED, WHEEL_LIFT, WRONG_ATTITUDE,
                    DriftEquilibrium, DriftReference)


@dataclass(frozen=True)
class EquilibriumSettings:
    min_yaw_rate: float = 0.05                     # [rad/s] below: trivial (straight driving)
    min_beta: float = math.radians(2.0)            # [rad]
    seed_yaw_factors: tuple = (0.5, 0.7, 0.9)      # r seed = factor * mu g / v
    seed_steer_deg: tuple = (-20.0, -10.0, 0.0, 10.0)   # mirrored for right drifts
    seed_rear_slip: tuple = (0.1, 0.3, 0.6)
    bound_tol: float = 1e-6                        # relative distance to a bound counted as "on the bound"
    max_nfev: int = 3000                           # budget of a refinement solve
    seed_nfev: int = 300                           # budget per seed (a converging seed needs ~150)


def peak_slip_angle(tyre, Fz: float, mu: float) -> float:
    """Slip angle of the pure-lateral force peak at load ``Fz`` [rad] (0.05 deg grid, kappa = 0)."""
    a = np.radians(np.arange(0.05, 45.0, 0.05))
    _, Fy = tyre.forces(np.zeros_like(a), a, np.full_like(a, float(Fz)), np.full_like(a, float(mu)))
    return float(a[int(np.argmax(Fy))])


def rwd_input(steer: float, torque: float) -> SevenDOFControlInput:
    """Configuration V1: front steer, equal drive torque on both rear wheels, no brakes."""
    return SevenDOFControlInput(steer_f=float(steer), T_drive=np.array([0.0, 0.0, torque, torque]))


class DriftEquilibriumSolver:
    """Drift equilibria of a SevenDOFModel on a uniform road (friction from the reference)."""

    def __init__(self, model, settings: EquilibriumSettings | None = None):
        self.settings = settings or EquilibriumSettings()
        self.base_model = model
        self._plants = {}

    def plant(self, mu: float) -> DeterministicPlant:
        if mu not in self._plants:
            import copy
            m = copy.copy(self.base_model)
            m.road = UniformRoad(mu=float(mu))
            self._plants[mu] = DeterministicPlant(m)
        return self._plants[mu]

    # ------------------------------------------------------------------ problem
    def _scales(self, p):
        return np.r_[1.0 / p.g, 1.0 / p.g, p.Iz / (p.mass * p.g * p.wheelbase), np.full(4, p.Iw / (p.Reff * p.mass * p.g))]

    def _unpack(self, z, ref: DriftReference):
        vx, vy = ref.speed_mps * math.cos(ref.beta_rad), ref.speed_mps * math.sin(ref.beta_rad)
        return np.r_[vx, vy, z[0], z[1:5]], rwd_input(z[5], z[6])

    def _bounds(self, p):
        lo = np.r_[-10.0, np.zeros(4), -p.max_steer_f, 0.0]
        hi = np.r_[10.0, np.full(4, 2000.0), p.max_steer_f, p.max_drive_torque]
        return lo, hi

    def residual(self, z, ref: DriftReference) -> np.ndarray:
        P = self.plant(ref.road_mu)
        q, u = self._unpack(z, ref)
        return P.evaluate(P.full_state(q), u).body_derivative * self._scales(P.params)

    def seeds(self, ref: DriftReference) -> list:
        """Deterministic seed grid (yaw rate towards the inside, steer mirrored with the direction)."""
        s, P = self.settings, self.plant(ref.road_mu)
        p, v = P.params, ref.speed_mps
        sgn = -math.copysign(1.0, ref.beta_rad)          # yaw rate sign of a drift
        out = []
        for fr, sd, kr in itertools.product(s.seed_yaw_factors, s.seed_steer_deg, s.seed_rear_slip):
            r0 = sgn * fr * ref.road_mu * p.g / v
            delta = math.radians(sd) * sgn
            q0, u0 = self._unpack(np.r_[r0, np.zeros(4), delta, 0.0], ref)
            _, _, _, v_wx, _ = P.model.wheel_velocities(P.state(P.full_state(q0)), u0)
            w = np.maximum(v_wx, 0.1) / p.Reff
            w[2:] *= 1.0 / (1.0 - kr)                    # plant slip: kappa = (wR - v)/(wR) for wR > v
            out.append(np.r_[r0, w, delta, 0.5 * p.max_drive_torque])
        return out

    # ------------------------------------------------------------------ solve
    def _solve_from(self, ref, z0, max_nfev: int | None = None):
        P = self.plant(ref.road_mu)
        lo, hi = self._bounds(P.params)
        z0 = np.clip(np.asarray(z0, float), lo + 1e-9 * (hi - lo), hi - 1e-9 * (hi - lo))
        sol = least_squares(self.residual, z0, args=(ref,), bounds=(lo, hi), method="trf",
                            x_scale=np.r_[0.5, np.full(4, 50.0), 0.2, 500.0], xtol=1e-15, ftol=1e-15, gtol=1e-15,
                            max_nfev=max_nfev or self.settings.max_nfev)
        return self.evaluate(ref, sol.x, bool(sol.status > 0), str(sol.message))

    def evaluate(self, ref: DriftReference, z, converged: bool = True, message: str = "") -> DriftEquilibrium:
        """Classify the candidate ``z`` (see module docstring); ``converged``: solver success."""
        s, P = self.settings, self.plant(ref.road_mu)
        p = P.params
        q, u = self._unpack(z, ref)
        ev = P.evaluate(P.full_state(q), u)
        d = ev.body_derivative
        groups = {"translational": float(max(abs(d[0]), abs(d[1]))), "yaw": float(abs(d[2])),
                  "wheel": float(np.max(np.abs(d[3:7])))}
        norm = max(groups[g] / RESIDUAL_TOL[g] for g in groups)
        tel = ev.telemetry
        reasons = []
        if not converged:
            reasons.append(SOLVER_NOT_CONVERGED)
        if norm > 1.0:
            reasons.append(RESIDUAL_TOO_LARGE)
        if not ev.load_transfer_converged:
            reasons.append(LOAD_TRANSFER_NOT_CONVERGED)
        if tel.support == "tip_over":
            reasons.append(TIP_OVER)
        elif tel.support != "four_wheel":
            reasons.append(WHEEL_LIFT)
        lo, hi = self._bounds(p)
        tol = s.bound_tol * (hi - lo)
        at_bound = [n for n, i in (("steer", 5), ("torque", 6)) if z[i] <= lo[i] + tol[i] or z[i] >= hi[i] - tol[i]]
        if "steer" in at_bound or ("torque" in at_bound and z[6] > 0.5 * hi[6]):
            reasons.append(ACTUATOR_LIMIT)
        r = float(z[0])
        if abs(r) < s.min_yaw_rate or abs(ref.beta_rad) < s.min_beta:
            reasons.append(TRIVIAL)
        elif r * ref.beta_rad >= 0.0:
            reasons.append(WRONG_ATTITUDE)
        rear_peak = [peak_slip_angle(P.model.tyre_rear, tel.Fz[i], tel.mu[i]) for i in (2, 3)]
        if TRIVIAL not in reasons and WRONG_ATTITUDE not in reasons and \
                any(abs(tel.alpha[i]) < rear_peak[k] for k, i in enumerate((2, 3))):
            reasons.append(REAR_NOT_SATURATED)
        if np.any(np.asarray(z[1:5]) < 0.0):
            reasons.append(NEGATIVE_WHEEL_SPEED)
        extra = extrapolating(P.model.tyre_front, tel.alpha[:2], tel.kappa[:2]).tolist() + \
            extrapolating(P.model.tyre_rear, tel.alpha[2:], tel.kappa[2:]).tolist()
        validity = INFEASIBLE if reasons else (VALID_EXTRAPOLATED if any(extra) else VALID)
        util = np.hypot(tel.Fx_wheel, tel.Fy_wheel) / np.maximum(tel.mu * tel.Fz, 1e-9)
        diag = {"residual": groups, "solver_message": message, "kappa": tel.kappa.tolist(),
                "alpha_deg": np.degrees(tel.alpha).tolist(), "Fz": tel.Fz.tolist(), "Fx": tel.Fx_wheel.tolist(),
                "Fy": tel.Fy_wheel.tolist(), "utilisation": util.tolist(), "extrapolating": extra,
                "support": tel.support, "ax": ev.ax, "ay": ev.ay, "at_bound": at_bound,
                "drive_power_kw": float(np.sum(u.T_drive * q[3:7]) / 1e3),
                "rear_peak_slip_angle_deg": np.degrees(rear_peak).tolist()}
        return DriftEquilibrium(ref, float(q[0]), float(q[1]), r, q[3:7], float(z[5]), u.T_drive, u.T_brake, float(norm),
                                not reasons, validity, tuple(reasons), diag)

    @staticmethod
    def unknowns(eq: DriftEquilibrium) -> np.ndarray:
        return np.r_[eq.yaw_rate, eq.omega, eq.steer_front, eq.wheel_drive_torque[2]]

    def solve(self, ref: DriftReference, guess=None, multistart: str = "first"):
        """Equilibrium for ``ref``.

        ``guess`` (unknown vector): one solve from there, then the seeds if it is not feasible.
        ``multistart="first"``: seeds in fixed order (budget ``seed_nfev`` each) until a feasible
        one is found; otherwise the best candidate is refined with ``max_nfev`` and reported
        (infeasible unless the refinement succeeds). ``"all"``: every seed is
        solved; returns the list of distinct branches (feasible first, then by residual).
        """
        if multistart not in ("first", "all"):
            raise ValueError("multistart must be 'first' or 'all'")
        cands = []
        if guess is not None:
            eq = self._solve_from(ref, guess)
            if eq.physical_feasible and multistart == "first":
                return eq
            cands.append(eq)
        for z0 in self.seeds(ref):
            eq = self._solve_from(ref, z0, self.settings.seed_nfev)
            cands.append(eq)
            if eq.physical_feasible and multistart == "first":
                return eq
        if multistart == "first":
            # no seed converged within its budget: refine the best candidate with the full budget
            best = min(cands, key=lambda e: e.residual_norm)
            refined = self._solve_from(ref, self.unknowns(best))
            return refined if refined.residual_norm <= best.residual_norm else best
        branches = []
        for e in sorted(cands, key=lambda e: (not e.physical_feasible, e.residual_norm)):
            if not any(abs(e.yaw_rate - b.yaw_rate) < 1e-4 and abs(e.steer_front - b.steer_front) < 1e-4
                       and abs(e.wheel_drive_torque[2] - b.wheel_drive_torque[2]) < 1e-2 for b in branches):
                branches.append(e)
        return branches

    def sweep(self, refs) -> list:
        """Equilibria along ``refs`` (in order) with natural continuation from the last feasible point."""
        out, guess, branch = [], None, None
        for ref in refs:
            if (ref.direction, ref.road_mu) != branch:   # continuation only within one direction and friction
                guess, branch = None, (ref.direction, ref.road_mu)
            eq = self.solve(ref, guess)
            out.append(eq)
            if eq.physical_feasible:
                guess = self.unknowns(eq)
        return out


__all__ = ("DriftEquilibriumSolver", "EquilibriumSettings", "peak_slip_angle", "rwd_input")
