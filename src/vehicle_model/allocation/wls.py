"""Box-constrained weighted least-squares control allocation (exact, small problems).

    minimise  J(u) = |W_v (B u - v)|^2 + eps |W_u (u - u_p)|^2
    subject to lo <= u <= hi

(Härkegård 2002, "Efficient active set algorithms for solving constrained least
squares problems in aircraft control allocation".) With ``eps > 0`` and positive
``W_u`` the problem is strictly convex, so the minimiser is unique and is the
solution of the reduced equality problem for exactly one assignment of every
variable to {free, at lower bound, at upper bound}. For the few actuators of a
vehicle (n <= 6, i.e. <= 729 assignments) all assignments are enumerated: the
result is exact and deterministic, without iteration limits or warm starts.
"""

from __future__ import annotations

import itertools
from dataclasses import dataclass

import numpy as np

_MAX_ENUMERATED = 6


@dataclass(frozen=True)
class WLSResult:
    u: np.ndarray            # optimal actuator vector
    achieved: np.ndarray     # B u
    residual: np.ndarray     # B u - v
    at_lower: np.ndarray     # bool per actuator
    at_upper: np.ndarray     # bool per actuator
    cost: float


def solve_box_wls(B, v, lo, hi, W_v=None, W_u=None, u_p=None, eps=1e-6, tol=1e-9) -> WLSResult:
    """Exact minimiser of the box-constrained WLS problem (module docstring)."""
    B = np.atleast_2d(np.asarray(B, dtype=float))
    m, n = B.shape
    v = np.asarray(v, dtype=float).reshape(m)
    lo = np.asarray(lo, dtype=float).reshape(n)
    hi = np.asarray(hi, dtype=float).reshape(n)
    if n > _MAX_ENUMERATED:
        raise ValueError(f"enumeration is limited to {_MAX_ENUMERATED} actuators, got {n}")
    if np.any(lo > hi):
        raise ValueError("lower bound above upper bound")
    if eps <= 0.0:
        raise ValueError("eps must be positive (strict convexity)")
    Wv2 = np.diag(np.ones(m) if W_v is None else np.asarray(W_v, dtype=float) ** 2)
    Wu2 = np.diag(np.ones(n) if W_u is None else np.asarray(W_u, dtype=float) ** 2)
    u_p = np.zeros(n) if u_p is None else np.asarray(u_p, dtype=float).reshape(n)

    H = B.T @ Wv2 @ B + eps * Wu2
    g = B.T @ Wv2 @ v + eps * Wu2 @ u_p  # J = u'Hu - 2 g'u + const

    def cost(u):
        r = B @ u - v
        d = u - u_p
        return float(r @ Wv2 @ r + eps * d @ Wu2 @ d)

    best_u, best_J = None, np.inf
    scale = np.maximum(np.abs(lo), np.abs(hi)) + 1.0
    for assignment in itertools.product((0, 1, 2), repeat=n):  # 0 free, 1 lower, 2 upper
        a = np.array(assignment)
        u = np.where(a == 1, lo, np.where(a == 2, hi, 0.0))
        free = a == 0
        if free.any():
            rhs = g[free] - H[np.ix_(free, ~free)] @ u[~free]
            u[free] = np.linalg.solve(H[np.ix_(free, free)], rhs)
            if np.any(u[free] < lo[free] - tol * scale[free]) or np.any(u[free] > hi[free] + tol * scale[free]):
                continue
        u = np.clip(u, lo, hi)
        J = cost(u)
        if best_u is None or J < best_J - 1e-15 * max(1.0, abs(best_J)):
            best_u, best_J = u, J
    achieved = B @ best_u
    span = np.maximum(hi - lo, 1e-12)
    return WLSResult(
        u=best_u, achieved=achieved, residual=achieved - v,
        at_lower=(best_u - lo) <= 1e-9 * span, at_upper=(hi - best_u) <= 1e-9 * span, cost=best_J,
    )


__all__ = ("WLSResult", "solve_box_wls")
