"""Monte Carlo robustness studies with documented spread and limit violations.

A study draws ``n`` parameter samples from declared distributions with one seeded
generator (``numpy.random.default_rng(seed)``), builds a fresh plant per sample,
runs one scenario with one controller through
:func:`~vehicle_model.simulation.controller_benchmark.run_scenario` and collects
the scenario KPIs. It reports

* the full sample table (parameters and KPIs per sample),
* statistics per KPI: mean, standard deviation, min, 5/50/95 % percentiles, max,
* limit violations: count, rate and the violating sample indices.

Same seed, same samples, same results (except the measured computation times):
studies are repeatable.

Distributions (``Variation``): ``("normal", mean, std)``, ``("uniform", low, high)``,
truncated to ``[clip_low, clip_high]`` if given. The parameter names are
interpreted by the plant builder passed in, e.g. relative factors on mass or tyre
stiffness, a CoG shift in metres, the road friction.
"""

from __future__ import annotations

import math
import operator
from dataclasses import dataclass, field
from typing import Callable

import numpy as np

from vehicle_model.simulation.controller_benchmark import Scenario, run_scenario

_OPS = {"<=": operator.le, "<": operator.lt, ">=": operator.ge, ">": operator.gt}


@dataclass(frozen=True)
class Variation:
    kind: str                      # "normal" | "uniform"
    a: float                       # mean or low
    b: float                       # std or high
    clip_low: float = -math.inf
    clip_high: float = math.inf

    def __post_init__(self) -> None:
        if self.kind not in ("normal", "uniform"):
            raise ValueError("kind must be 'normal' or 'uniform'")
        if self.kind == "normal" and self.b < 0 or self.kind == "uniform" and self.b < self.a:
            raise ValueError("invalid distribution parameters")

    def sample(self, rng: np.random.Generator) -> float:
        x = rng.normal(self.a, self.b) if self.kind == "normal" else rng.uniform(self.a, self.b)
        return float(np.clip(x, self.clip_low, self.clip_high))


@dataclass(frozen=True)
class Limit:
    """Requirement on a KPI, e.g. ``Limit("yaw_rate_ratio_1s", "<=", 0.35)``; violated if not met."""

    kpi: str
    op: str
    value: float

    def __post_init__(self) -> None:
        if self.op not in _OPS:
            raise ValueError(f"op must be one of {sorted(_OPS)}")

    def met(self, x: float) -> bool:
        return bool(np.isfinite(x)) and _OPS[self.op](x, self.value)


@dataclass
class MonteCarloResult:
    samples: list[dict]                       # parameters per sample
    kpis: list[dict]                          # KPIs per sample
    statistics: dict = field(default_factory=dict)
    violations: dict = field(default_factory=dict)

    def table(self) -> list[dict]:
        return [{**s, **{k: v for k, v in kp.items() if not isinstance(v, dict)}} for s, kp in zip(self.samples, self.kpis)]


def _statistics(kpis: list[dict]) -> dict:
    keys = [k for k, v in kpis[0].items() if isinstance(v, (int, float))]
    out = {}
    for k in keys:
        x = np.array([kp[k] for kp in kpis], dtype=float)
        q = np.nanpercentile(x, [5, 50, 95])
        out[k] = {"mean": float(np.nanmean(x)), "std": float(np.nanstd(x)), "min": float(np.nanmin(x)),
                  "p5": float(q[0]), "p50": float(q[1]), "p95": float(q[2]), "max": float(np.nanmax(x)),
                  "n_nan": int(np.isnan(x).sum())}
    return out


def run_monte_carlo(build_plant: Callable[[dict], Callable], controller_factory, scenario: Scenario,
                    variations: dict[str, Variation], n: int, seed: int, limits=(), dt: float = 0.005,
                    progress: Callable[[int, int], None] | None = None) -> MonteCarloResult:
    """``build_plant(sample) -> plant factory (road -> model)``; one run per sample."""
    if n < 1:
        raise ValueError("n must be >= 1")
    rng = np.random.default_rng(seed)
    samples = [{name: var.sample(rng) for name, var in variations.items()} for _ in range(n)]
    kpis = []
    for i, smp in enumerate(samples):
        kpis.append(run_scenario(build_plant(smp), controller_factory, scenario, dt).kpis)
        if progress is not None:
            progress(i + 1, n)
    result = MonteCarloResult(samples, kpis, _statistics(kpis))
    for lim in limits:
        bad = [i for i, kp in enumerate(kpis) if not lim.met(kp[lim.kpi])]
        result.violations[f"{lim.kpi} {lim.op} {lim.value:g}"] = {"count": len(bad), "rate": len(bad) / n, "samples": bad}
    return result


__all__ = ("Limit", "MonteCarloResult", "Variation", "run_monte_carlo")
