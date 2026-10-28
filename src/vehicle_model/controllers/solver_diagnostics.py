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
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

NOMINAL, SUBOPTIMAL, FALLBACK, DEGRADED = "nominal", "suboptimal", "fallback", "degraded"


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

    def classify(self, converged: bool, x, cost: float, cost_warm_start: float, message: str = "",
                 iterations: int = 0, solve_time_ms: float = 0.0, improvement_tol: float = 1e-9) -> SolverReport:
        """Decide whether the solution ``x`` is used, and the resulting status."""
        finite = x is not None and bool(np.all(np.isfinite(x))) and np.isfinite(cost)
        accepted = finite and (converged or cost <= cost_warm_start + improvement_tol * max(1.0, abs(cost_warm_start)))
        status = NOMINAL if (accepted and converged) else (SUBOPTIMAL if accepted else FALLBACK)
        self.consecutive_failures = 0 if status == NOMINAL else self.consecutive_failures + 1
        if self.consecutive_failures >= self.max_consecutive_failures:
            status = DEGRADED
        self.counts[status] += 1
        over = self.time_budget_ms is not None and solve_time_ms > self.time_budget_ms
        self.last = SolverReport(status, bool(converged), bool(accepted), str(message), int(iterations),
                                 float(cost), float(cost_warm_start), float(solve_time_ms), bool(over),
                                 self.consecutive_failures)
        return self.last


__all__ = ("DEGRADED", "FALLBACK", "NOMINAL", "SUBOPTIMAL", "SolverMonitor", "SolverReport")
