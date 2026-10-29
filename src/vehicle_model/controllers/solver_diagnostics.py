"""Solver diagnostics and degraded-mode bookkeeping for optimisation-based controllers (Plan.md, Abschnitt 10.4; M6).

Every solve is classified; the classification is the controller's output status:

    nominal     the solver converged and its solution is used
    suboptimal  not converged (e.g. iteration limit), but the solution improves the
                cost of the warm start and is used
    fallback    the solution is rejected (failed, non-finite or worse than the warm
                start); the shifted previous plan is used
    degraded    ``max_consecutive_failures`` or more consecutive non-nominal solves:
                the controller is outside its validated operating mode. The command
                is still computed as for fallback/suboptimal; consumers should treat
                it as unreliable.

A time budget can be set. SciPy solvers cannot be interrupted, so an overrun is
reported (``over_budget``), not prevented.

Solution quality (Plan V3.0 M10 5.2), reported per solve and summarised by
:meth:`SolverMonitor.summary`:

    cost_improvement           J(warm start) - J(solution)
    relative_cost_improvement  the same divided by max(|J(warm start)|, 1e-12): near 0
                               means the warm start was already (locally) optimal
    warm_start_step            max |x - x_warm_start|: how far the solver moved
    kkt_stationarity           first-order optimality ||grad L||_inf, see :func:`kkt_residuals`
    kkt_stationarity_relative  the same divided by max(1, |J(solution)|)
    kkt_primal                 largest violation of the solver's own bounds/constraints

KKT residuals are evaluated after the solve from the objective gradient and the
constraint Jacobians; they are ``nan`` where a controller does not provide them.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.optimize import nnls

NOMINAL, SUBOPTIMAL, FALLBACK, DEGRADED = "nominal", "suboptimal", "fallback", "degraded"


@dataclass(frozen=True)
class KKTResiduals:
    """First-order optimality of a solution of  min f(x)  s.t.  lo <= x <= hi,  g(x) >= 0.

    ``stationarity`` = ||grad f - sum_i mu_i grad c_i||_inf over the active constraints c_i
    (bounds and inequalities within ``active_tol``), with the multipliers mu >= 0 fitted by
    non-negative least squares. Complementarity holds by construction (inactive
    constraints get mu = 0), so a small value together with a small ``primal`` violation is
    a KKT point up to the active-set tolerance. The value has the units of the cost per
    unit of the decision variables; ``stationarity_relative`` divides it by
    max(1, |f(x)|) when the cost is given.
    """

    stationarity: float
    primal: float
    n_active: int
    stationarity_relative: float = float("nan")

    def as_dict(self) -> dict:
        return dict(self.__dict__)


NO_KKT = KKTResiduals(float("nan"), float("nan"), 0)


def kkt_residuals(grad, x, lower=None, upper=None, ineq_value=None, ineq_jacobian=None,
                  active_tol: float = 1e-6, cost: float | None = None) -> KKTResiduals:
    """KKT residuals of ``x`` (see :class:`KKTResiduals`).

    ``lower``/``upper``: variable bounds (scalars or arrays, +-inf or ``None`` for unbounded);
    ``ineq_value``/``ineq_jacobian``: g(x) and dg/dx with g(x) >= 0.
    """
    if grad is None or x is None:
        return NO_KKT
    x = np.asarray(x, float).ravel()
    grad = np.asarray(grad, float).ravel()
    if x.size == 0 or grad.size != x.size or not (np.all(np.isfinite(x)) and np.all(np.isfinite(grad))):
        return NO_KKT
    n = x.size
    lo = np.broadcast_to(np.asarray(-np.inf if lower is None else lower, float), n)
    hi = np.broadcast_to(np.asarray(np.inf if upper is None else upper, float), n)
    primal = max(float(np.max(lo - x, initial=0.0)), float(np.max(x - hi, initial=0.0)))
    cols = [np.eye(1, n, j).ravel()                       # grad of x_j - lo_j >= 0
            for j in np.flatnonzero(np.isfinite(lo) & (x - lo <= active_tol * (1.0 + np.abs(lo))))]
    cols += [-np.eye(1, n, j).ravel()                     # grad of hi_j - x_j >= 0
             for j in np.flatnonzero(np.isfinite(hi) & (hi - x <= active_tol * (1.0 + np.abs(hi))))]
    if ineq_value is not None:
        g = np.asarray(ineq_value, float).ravel()
        J = np.asarray(ineq_jacobian, float).reshape(g.size, n)
        primal = max(primal, float(np.max(-g, initial=0.0)))
        cols += list(J[g <= active_tol * (1.0 + np.abs(J).max(axis=1))])
    r = grad
    if cols:
        A = np.array(cols).T
        r = grad - A @ nnls(A, grad)[0]
    stat = float(np.max(np.abs(r)))
    rel = stat / max(1.0, abs(float(cost))) if cost is not None and np.isfinite(cost) else float("nan")
    return KKTResiduals(stat, primal, len(cols), rel)


@dataclass(frozen=True)
class SolverReport:
    status: str
    converged: bool
    accepted: bool
    message: str
    iterations: int
    cost: float
    cost_warm_start: float
    solve_time_ms: float
    over_budget: bool
    consecutive_failures: int
    cost_improvement: float = float("nan")
    relative_cost_improvement: float = float("nan")
    warm_start_step: float = float("nan")
    kkt_stationarity: float = float("nan")
    kkt_stationarity_relative: float = float("nan")
    kkt_primal: float = float("nan")

    def as_dict(self) -> dict:
        return dict(self.__dict__)


class SolverMonitor:
    """Classifies solves and counts consecutive non-nominal ones."""

    def __init__(self, max_consecutive_failures: int = 5, time_budget_ms: float | None = None):
        if max_consecutive_failures < 1:
            raise ValueError("max_consecutive_failures must be >= 1")
        self.max_consecutive_failures = max_consecutive_failures
        self.time_budget_ms = time_budget_ms
        self.reset()

    def reset(self) -> None:
        self.consecutive_failures = 0
        self.counts = {NOMINAL: 0, SUBOPTIMAL: 0, FALLBACK: 0, DEGRADED: 0}
        self.last: SolverReport | None = None
        self.reports: list[SolverReport] = []

    def classify(self, converged: bool, x, cost: float, cost_warm_start: float, message: str = "",
                 iterations: int = 0, solve_time_ms: float = 0.0, improvement_tol: float = 1e-9,
                 x_warm_start=None, kkt: KKTResiduals | None = None) -> SolverReport:
        """Decide whether the solution ``x`` is used, and the resulting status.

        ``x_warm_start`` and ``kkt`` are optional quality information (module docstring).
        """
        finite = x is not None and bool(np.all(np.isfinite(x))) and np.isfinite(cost)
        accepted = finite and (converged or cost <= cost_warm_start + improvement_tol * max(1.0, abs(cost_warm_start)))
        status = NOMINAL if (accepted and converged) else (SUBOPTIMAL if accepted else FALLBACK)
        self.consecutive_failures = 0 if status == NOMINAL else self.consecutive_failures + 1
        if self.consecutive_failures >= self.max_consecutive_failures:
            status = DEGRADED
        self.counts[status] += 1
        over = self.time_budget_ms is not None and solve_time_ms > self.time_budget_ms
        gain = float(cost_warm_start) - float(cost) if finite else float("nan")
        step = float("nan")
        if finite and x_warm_start is not None:
            step = float(np.max(np.abs(np.asarray(x, float).ravel() - np.asarray(x_warm_start, float).ravel()),
                                initial=0.0))
        kkt = kkt or NO_KKT
        self.last = SolverReport(status, bool(converged), bool(accepted), str(message), int(iterations),
                                 float(cost), float(cost_warm_start), float(solve_time_ms), bool(over),
                                 self.consecutive_failures, gain, gain / max(abs(float(cost_warm_start)), 1e-12),
                                 step, float(kkt.stationarity), float(kkt.stationarity_relative),
                                 float(kkt.primal))
        self.reports.append(self.last)
        return self.last

    def summary(self) -> dict:
        """Aggregate solution quality over all solves since the last reset (Plan V3.0 M10 5.2)."""
        n = len(self.reports)
        if n == 0:
            return {"solves": 0}
        col = lambda k: np.array([getattr(r, k) for r in self.reports], float)
        fin = lambda a: a[np.isfinite(a)]
        t, kkt, prim, rel = col("solve_time_ms"), fin(col("kkt_stationarity_relative")), fin(col("kkt_primal")), \
            fin(col("relative_cost_improvement"))
        out = {"solves": n, **{f"fraction_{k}": v / n for k, v in self.counts.items()},
               "over_budget": int(sum(r.over_budget for r in self.reports)),
               "solve_ms_mean": float(t.mean()), "solve_ms_p95": float(np.percentile(t, 95)),
               "solve_ms_max": float(t.max()), "iterations_mean": float(col("iterations").mean())}
        if rel.size:
            out["relative_cost_improvement_median"] = float(np.median(rel))
        if kkt.size:
            out["kkt_stationarity_relative_median"] = float(np.median(kkt))
            out["kkt_stationarity_relative_max"] = float(kkt.max())
        if prim.size:
            out["kkt_primal_max"] = float(prim.max())
        return out


__all__ = ("DEGRADED", "FALLBACK", "NOMINAL", "NO_KKT", "SUBOPTIMAL", "KKTResiduals", "SolverMonitor", "SolverReport",
           "kkt_residuals")
