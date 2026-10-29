"""Linearisation of the 7-DOF plant at a drift equilibrium (Drift plan D3, section 5.1).

``dq_dot = A dq + B du`` with ``q = [vx, vy, r, omega x4]`` and ``u = [delta_f, T]`` (front
steer, equal drive torque on both rear wheels), both from central differences of the
deterministic plant evaluation (:class:`~vehicle_model.drift.plant_adapter.DeterministicPlant`).

Step size: not fixed. For every relative step of ``steps`` the Jacobians are computed; for
neighbouring steps the largest relative change of the eigenvalues of ``A`` (paired by
minimum distance) and of ``B`` is measured, and the step after the smallest change is used
(truncation error at large steps, rounding at small ones). ``converged``: that change
<= 1e-4.

Checks reported with every linearisation: equilibrium residual (must be within tolerance
before linearising), finiteness, eigenvalues, the unstable modes and the PBH test
``rank [A - lambda I, B] = n`` for each of them (stabilisability), the rank of the Kalman
controllability matrix (informative: stable fast wheel modes may be uncontrollable), and the
condition numbers of ``A`` and ``B`` in scaled coordinates.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.optimize import linear_sum_assignment

from .equilibrium import rwd_input

STEPS = (1e-2, 1e-3, 1e-4, 1e-5, 1e-6, 1e-7)
STATE_SCALE = np.array([1.0, 1.0, 0.1, 10.0, 10.0, 10.0, 10.0])   # typical magnitudes: m/s, m/s, rad/s, rad/s
STABLE_TOL = 1e-6                                                  # [1/s] Re(lambda) above: unstable / marginal


@dataclass(frozen=True)
class DriftLinearisation:
    A: np.ndarray                    # (7, 7)
    B: np.ndarray                    # (7, 2)
    q_star: np.ndarray
    u_star: np.ndarray
    step: float
    step_study: tuple                # ({step, eigenvalue_change, B_change}, ...)
    converged: bool
    eigenvalues: np.ndarray
    unstable: tuple                  # eigenvalues with Re >= -STABLE_TOL
    pbh_ranks: tuple                 # rank [A - lambda I, B] per unstable eigenvalue
    stabilisable: bool
    controllability_rank: int
    condition_A: float
    condition_B: float

    def summary(self) -> dict:
        return {"step": self.step, "converged": self.converged, "step_study": list(self.step_study),
                "eigenvalues": [[float(l.real), float(l.imag)] for l in self.eigenvalues],
                "unstable": [[float(l.real), float(l.imag)] for l in self.unstable], "pbh_ranks": list(self.pbh_ranks),
                "stabilisable": self.stabilisable, "controllability_rank": self.controllability_rank,
                "condition_A": self.condition_A, "condition_B": self.condition_B}


def input_scale(params) -> np.ndarray:
    return np.array([params.max_steer_f, params.max_drive_torque])


def _eig_change(a: np.ndarray, b: np.ndarray) -> float:
    la, lb = np.linalg.eigvals(a), np.linalg.eigvals(b)
    cost = np.abs(la[:, None] - lb[None, :])
    i, j = linear_sum_assignment(cost)
    return float(np.max(cost[i, j] / np.maximum(np.abs(lb[j]), 1.0)))


def jacobians(plant, q_star, u_star, rel_step: float):
    """Central-difference ``(A, B)`` with steps ``rel_step * max(|x|, typical)`` per component."""
    q_star, u_star = np.asarray(q_star, float), np.asarray(u_star, float)
    us = input_scale(plant.params)

    def f(q, u):
        return plant.evaluate(plant.full_state(q), rwd_input(*u)).body_derivative[:7]

    hq = rel_step * np.maximum(np.abs(q_star), STATE_SCALE)
    hu = rel_step * np.maximum(np.abs(u_star), 0.1 * us)
    A = np.column_stack([(f(q_star + e * hq[i], u_star) - f(q_star - e * hq[i], u_star)) / (2 * hq[i])
                         for i, e in enumerate(np.eye(7))])
    B = np.column_stack([(f(q_star, u_star + e * hu[j]) - f(q_star, u_star - e * hu[j])) / (2 * hu[j])
                         for j, e in enumerate(np.eye(2))])
    return A, B


def linearise(plant, equilibrium, steps=STEPS) -> DriftLinearisation:
    """Linearisation of ``plant`` at a feasible drift ``equilibrium`` with step study and stabilisability checks."""
    if not equilibrium.physical_feasible:
        raise ValueError(f"cannot linearise an infeasible equilibrium ({', '.join(equilibrium.reasons)})")
    q_star = equilibrium.body_state()
    u_star = np.array([equilibrium.steer_front, equilibrium.wheel_drive_torque[2]])
    mats = [jacobians(plant, q_star, u_star, h) for h in steps]
    if not all(np.all(np.isfinite(A)) and np.all(np.isfinite(B)) for A, B in mats):
        raise FloatingPointError("non-finite Jacobian")
    study = []
    for k in range(len(mats) - 1):
        (A0, B0), (A1, B1) = mats[k], mats[k + 1]
        study.append({"step": steps[k + 1], "eigenvalue_change": _eig_change(A0, A1),
                      "B_change": float(np.max(np.abs(B0 - B1)) / max(np.max(np.abs(B1)), 1e-12))})
    k = int(np.argmin([max(s["eigenvalue_change"], s["B_change"]) for s in study]))
    A, B = mats[k + 1]
    lam = np.linalg.eigvals(A)
    unstable = tuple(l for l in lam if l.real >= -STABLE_TOL)
    ranks = []
    for l in unstable:
        M = np.hstack([A - l * np.eye(7), B])
        ranks.append(int(np.linalg.matrix_rank(M, tol=1e-9 * np.linalg.norm(M))))
    Ds, Du = np.diag(STATE_SCALE), np.diag(input_scale(plant.params))
    As, Bs = np.linalg.solve(Ds, A @ Ds), np.linalg.solve(Ds, B @ Du)
    ctrb = np.hstack([np.linalg.matrix_power(As, i) @ Bs for i in range(7)])
    return DriftLinearisation(A, B, q_star, u_star, steps[k + 1], tuple(study),
                              max(study[k]["eigenvalue_change"], study[k]["B_change"]) <= 1e-4,
                              lam[np.argsort(-lam.real)], unstable, tuple(ranks), all(r == 7 for r in ranks),
                              int(np.linalg.matrix_rank(ctrb, tol=1e-9 * np.linalg.norm(ctrb))),
                              float(np.linalg.cond(As)), float(np.linalg.cond(Bs)))


__all__ = ("STATE_SCALE", "STEPS", "DriftLinearisation", "input_scale", "jacobians", "linearise")
