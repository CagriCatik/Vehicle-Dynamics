"""Constraint semantics and feasibility checks, separate from the solver status (Rule 4).

A solver that reports success has minimised its cost within *its* model and bounds;
that alone says nothing about whether the plan respects the limits that matter.
Each controller therefore declares its limits with an explicit kind and checks its
plan against them after every solve. The result, a :class:`FeasibilityReport`, is
reported next to the :class:`~.solver_diagnostics.SolverReport`, never merged into it.

Kinds (what a controller promises about a limit):

    hard      enforced by the optimiser (bounds or constraints). An excess beyond the
              tolerance means the plan is infeasible: a defect or a failed fallback.
    soft      allowed to be exceeded at a cost (slack variable). An excess is legal and
              reported with its magnitude.
    penalty   only a cost term above a threshold; no limit is claimed. The excess over
              the threshold is reported as information.
    advisory  monitored and reported; it does not influence the optimisation (e.g.
              heuristic stability limits, configured limits that are not enforced).
    plant     enforced by the physics of the simulated vehicle (tyre saturation). The
              controller cannot violate it in the plant; a plan that exceeds it relies
              on forces the vehicle cannot produce, so the plan is not realisable.

Only ``hard`` decides :attr:`FeasibilityReport.feasible`; every excess of every kind
is listed with its size. A value missing from a check is an error, not a silent pass.
"""

from __future__ import annotations

from collections.abc import Iterable, Mapping
from dataclasses import dataclass

import numpy as np

HARD, SOFT, PENALTY, ADVISORY, PLANT = "hard", "soft", "penalty", "advisory", "plant"
KINDS = (HARD, SOFT, PENALTY, ADVISORY, PLANT)


@dataclass(frozen=True)
class Constraint:
    """``lower <= value <= upper`` element-wise; ``lower=None`` means ``|value| <= upper``."""

    name: str
    kind: str
    upper: float
    lower: float | None = None
    unit: str = ""
    note: str = ""
    rel_tol: float = 1e-6        # excess below rel_tol * max(1, |limit|) counts as satisfied (solver tolerance)

    def __post_init__(self) -> None:
        if self.kind not in KINDS:
            raise ValueError(f"constraint {self.name!r}: unknown kind {self.kind!r}, expected one of {KINDS}")
        lo = -self.upper if self.lower is None else self.lower
        if not lo <= self.upper:
            raise ValueError(f"constraint {self.name!r}: lower {lo} > upper {self.upper}")

    @property
    def bounds(self) -> tuple[float, float]:
        return (-self.upper if self.lower is None else self.lower), self.upper

    def check(self, value) -> "ConstraintCheck":
        v = np.atleast_1d(np.asarray(value, dtype=float)).ravel()
        lo, hi = self.bounds
        if v.size == 0:
            raise ValueError(f"constraint {self.name!r}: empty value")
        if not np.all(np.isfinite(v)):
            excess = float("inf")
        else:
            excess = float(max(0.0, np.max(v - hi), np.max(lo - v)))
        tol = self.rel_tol * max(1.0, abs(lo), abs(hi))
        peak = float(v[np.argmax(np.abs(v))]) if np.all(np.isfinite(v)) else float("nan")
        return ConstraintCheck(self.name, self.kind, peak, lo, hi, excess, excess > tol, self.unit)


@dataclass(frozen=True)
class ConstraintCheck:
    name: str
    kind: str
    peak: float          # value with the largest magnitude
    lower: float
    upper: float
    excess: float        # largest distance outside [lower, upper]; 0 inside
    exceeded: bool       # excess beyond the tolerance
    unit: str = ""


@dataclass(frozen=True)
class FeasibilityReport:
    checks: tuple[ConstraintCheck, ...]

    @property
    def feasible(self) -> bool:
        """No hard constraint exceeded. Soft, penalty, advisory and plant excesses are reported, not judged here."""
        return not any(c.exceeded for c in self.checks if c.kind == HARD)

    @property
    def realisable(self) -> bool:
        """Feasible and no plant limit exceeded: the plan does not rely on forces the vehicle cannot produce."""
        return self.feasible and not any(c.exceeded for c in self.checks if c.kind == PLANT)

    def exceeded(self, kind: str | None = None) -> tuple[str, ...]:
        return tuple(c.name for c in self.checks if c.exceeded and (kind is None or c.kind == kind))

    def max_excess(self, kind: str) -> float:
        return max((c.excess for c in self.checks if c.kind == kind), default=0.0)

    def as_dict(self) -> dict:
        return {"feasible": self.feasible, "realisable": self.realisable,
                "exceeded": {k: list(self.exceeded(k)) for k in KINDS if self.exceeded(k)},
                "checks": [dict(c.__dict__) for c in self.checks]}


class ConstraintSet:
    """A controller's declared limits. :meth:`check` needs exactly one value (array) per constraint."""

    def __init__(self, constraints: Iterable[Constraint]):
        self.constraints = tuple(constraints)
        names = [c.name for c in self.constraints]
        if len(set(names)) != len(names):
            raise ValueError(f"duplicate constraint names in {names}")

    def check(self, values: Mapping[str, object]) -> FeasibilityReport:
        names = {c.name for c in self.constraints}
        missing, unknown = names - set(values), set(values) - names
        if missing or unknown:
            raise KeyError(f"constraint values: missing {sorted(missing)}, unknown {sorted(unknown)}")
        return FeasibilityReport(tuple(c.check(values[c.name]) for c in self.constraints))

    def declaration(self) -> list[dict]:
        """What the controller promises per limit (for reports)."""
        return [{"name": c.name, "kind": c.kind, "lower": c.bounds[0], "upper": c.bounds[1], "unit": c.unit,
                 "note": c.note} for c in self.constraints]


class FeasibilityMonitor:
    """Counts checked steps, infeasible and unrealisable ones, and the largest excess per constraint."""

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self.steps = self.infeasible_steps = self.unrealisable_steps = 0
        self.excess_counts: dict[str, int] = {}
        self.max_excess: dict[str, float] = {}
        self.last: FeasibilityReport | None = None

    def record(self, report: FeasibilityReport) -> FeasibilityReport:
        self.steps += 1
        self.infeasible_steps += not report.feasible
        self.unrealisable_steps += not report.realisable
        for c in report.checks:
            if c.exceeded:
                key = f"{c.kind}:{c.name}"
                self.excess_counts[key] = self.excess_counts.get(key, 0) + 1
                self.max_excess[key] = max(self.max_excess.get(key, 0.0), c.excess)
        self.last = report
        return report

    def summary(self) -> dict:
        return {"steps": self.steps, "infeasible_steps": self.infeasible_steps,
                "unrealisable_steps": self.unrealisable_steps, "excess_counts": dict(self.excess_counts),
                "max_excess": dict(self.max_excess)}


def stability_limits(mu: float, vx: float, g: float = 9.81, beta_gain: float = 0.02) -> tuple[float, float]:
    """Heuristic advisory limits ``(beta_max, r_max)`` [rad, rad/s].

    ``r_max = mu g / v`` (steady-state lateral acceleration within friction) and
    ``beta_max = arctan(beta_gain mu g)`` (about 11 deg at mu = 1, 2 deg at mu = 0.2,
    a common ESC rule of thumb). Not derived from the vehicle dynamics: advisory
    only, until nonlinear stability provides model-based stability boundaries.
    """
    mu = max(float(mu), 0.05)
    return float(np.arctan(beta_gain * mu * g)), float(mu * g / max(abs(vx), 1.0))


__all__ = (
    "ADVISORY",
    "HARD",
    "KINDS",
    "PENALTY",
    "PLANT",
    "SOFT",
    "Constraint",
    "ConstraintCheck",
    "ConstraintSet",
    "FeasibilityMonitor",
    "FeasibilityReport",
    "stability_limits",
)
