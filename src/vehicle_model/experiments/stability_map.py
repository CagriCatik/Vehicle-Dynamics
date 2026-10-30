"""Experiment type ``stability_map``: numerical recovery regions per controller (6.4).

For each controller the closed loop is started from a grid of initial states
``(beta0, r0)`` at the operating speed while the driver holds the steer, and every
run is classified (:mod:`vehicle_model.simulation.stability_region`). Reported:
2D maps, the share of returned initial states per controller with its uncertainty
(boundary points, re-simulation with smaller step and longer duration, one grid
refinement), phase portraits, time histories of limit cases, the open-loop
equilibria of the operating point, and the heuristic phase-plane boundary of
:mod:`vehicle_model.controllers.phase_plane` as comparison basis.

A map is a numerically determined recovery region for the grid, step and duration
used, not a proven region of attraction.

Example (``configs/experiments/stability_map_cornering.yaml``)::

    type: stability_map
    name: stability_map_cornering
    vehicle: configs/vehicles/seven_dof_passenger.yaml
    road: {type: uniform, mu: 0.6}
    operating_point: {speed_kph: 80, steer_deg: 1.0}
    grid: {beta_deg: [-20, 20, 11], r_degps: [-40, 40, 11]}      # [min, max, points]
    simulation: {dt: 0.01, duration_s: 6.0}
    controllers: [passive, esc, 4ws, tv, ivmc, ivmc_no_phase_plane]
    outputs: {directory: output/experiments/stability_map_cornering}

Optional: ``criteria`` (target_beta_deg, target_r_degps, hold_s, loss_beta_deg,
settle_beta_rate_degps, settle_r_rate_degps2), ``uncertainty`` (dt_factor,
duration_factor, refine), ``portraits`` (controllers, points per axis), ``workers``.
"""

from __future__ import annotations

import csv
import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from .spec import _keys, _require
from .types import provenance, write_json

KIND = "stability_map"
_TOP = {"type", "name", "description", "vehicle", "tyres", "road", "operating_point", "grid", "simulation", "criteria",
        "controllers", "uncertainty", "portraits", "workers", "outputs", "verification"}
_CRITERIA = {"target_beta_deg": "target_beta", "target_r_degps": "target_r", "hold_s": "hold",
             "loss_beta_deg": "loss_beta", "settle_beta_rate_degps": "settle_beta_rate",
             "settle_r_rate_degps2": "settle_r_rate"}
FORMATS = ("csv", "json", "png", "md")


@dataclass(frozen=True)
class StabilityMapSpec:
    name: str
    vehicle: str
    mu: float
    speed_kph: float
    steer_deg: float
    grid: dict                        # {beta_deg: [min, max, n], r_degps: [min, max, n]}
    controllers: tuple
    dt: float = 0.01
    duration_s: float = 6.0
    criteria: dict = field(default_factory=dict)      # YAML keys of _CRITERIA
    uncertainty: dict = field(default_factory=dict)   # dt_factor, duration_factor, refine
    portraits: dict = field(default_factory=dict)     # controllers, points
    workers: int | None = None
    tyres: dict = field(default_factory=dict)
    outputs: dict = field(default_factory=dict)
    verification: bool = True
    description: str = ""
    source: str = ""
    kind = KIND

    @property
    def beta_grid(self) -> np.ndarray:
        lo, hi, n = self.grid["beta_deg"]
        return np.radians(np.linspace(lo, hi, int(n)))

    @property
    def r_grid(self) -> np.ndarray:
        lo, hi, n = self.grid["r_degps"]
        return np.radians(np.linspace(lo, hi, int(n)))

    def problem(self, controller: str):
        from vehicle_model.simulation.stability_region import RecoveryCriteria, RecoveryProblem
        crit = {}
        for key, name in _CRITERIA.items():
            if key in self.criteria:
                v = float(self.criteria[key])
                crit[name] = v if key == "hold_s" else math.radians(v)
        return RecoveryProblem(controller, self.speed_kph / 3.6, math.radians(self.steer_deg), self.mu, self.vehicle,
                               self.tyres.get("front"), self.tyres.get("rear"), self.dt, self.duration_s,
                               RecoveryCriteria(**crit))

    def to_dict(self) -> dict:
        return {"type": KIND, "name": self.name, "description": self.description, "vehicle": self.vehicle,
                "tyres": dict(self.tyres), "road": {"type": "uniform", "mu": self.mu},
                "operating_point": {"speed_kph": self.speed_kph, "steer_deg": self.steer_deg},
                "grid": dict(self.grid), "simulation": {"dt": self.dt, "duration_s": self.duration_s},
                "criteria": dict(self.criteria), "controllers": list(self.controllers),
                "uncertainty": dict(self.uncertainty), "portraits": dict(self.portraits), "workers": self.workers,
                "outputs": dict(self.outputs), "verification": self.verification}


def _num(v, where, lo=None, hi=None, positive=False) -> float:
    _require(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v), f"{where} must be a number")
    _require(not positive or v > 0, f"{where} must be positive")
    _require(lo is None or v >= lo, f"{where} must be >= {lo}")
    _require(hi is None or v <= hi, f"{where} must be <= {hi}")
    return float(v)


def parse(data: dict, source: str = "<dict>") -> StabilityMapSpec:
    from vehicle_model.simulation.stability_region import CONTROLLERS

    _require(data.get("type") == KIND, f"{source}: type must be {KIND!r}")
    _keys(data, _TOP, source, {"type", "name", "vehicle", "road", "operating_point", "grid", "controllers"})
    _require(Path(data["vehicle"]).is_file(), f"{source}: vehicle file not found: {data['vehicle']}")
    tyres = dict(data.get("tyres") or {})
    _keys(tyres, {"front", "rear"}, f"{source}: tyres")
    road = dict(data["road"])
    _keys(road, {"type", "mu"}, f"{source}: road", {"type", "mu"})
    _require(road["type"] == "uniform", f"{source}: road.type must be 'uniform'")
    mu = _num(road["mu"], f"{source}: road.mu", 0.05, 2.0)
    op = dict(data["operating_point"])
    _keys(op, {"speed_kph", "steer_deg"}, f"{source}: operating_point", {"speed_kph"})
    speed = _num(op["speed_kph"], f"{source}: operating_point.speed_kph", 10.0, 250.0)
    steer = _num(op.get("steer_deg", 0.0), f"{source}: operating_point.steer_deg", -10.0, 10.0)
    grid = dict(data["grid"])
    _keys(grid, {"beta_deg", "r_degps"}, f"{source}: grid", {"beta_deg", "r_degps"})
    for k in grid:
        g = grid[k]
        _require(isinstance(g, list) and len(g) == 3, f"{source}: grid.{k} must be [min, max, points]")
        lo, hi = _num(g[0], f"{source}: grid.{k}[0]"), _num(g[1], f"{source}: grid.{k}[1]")
        _require(lo < hi, f"{source}: grid.{k}: min must be below max")
        _require(isinstance(g[2], int) and not isinstance(g[2], bool) and 2 <= g[2] <= 101,
                 f"{source}: grid.{k}[2] must be an integer in [2, 101]")
        grid[k] = [lo, hi, int(g[2])]
    _require(max(abs(grid["beta_deg"][0]), abs(grid["beta_deg"][1])) < 80.0, f"{source}: grid.beta_deg must lie in (-80, 80)")
    sim = dict(data.get("simulation") or {})
    _keys(sim, {"dt", "duration_s"}, f"{source}: simulation")
    dt = _num(sim.get("dt", 0.01), f"{source}: simulation.dt", 0.001, 0.02)
    duration = _num(sim.get("duration_s", 6.0), f"{source}: simulation.duration_s", 0.5, 60.0)
    crit = dict(data.get("criteria") or {})
    _keys(crit, set(_CRITERIA), f"{source}: criteria")
    for k, v in crit.items():
        _num(v, f"{source}: criteria.{k}", positive=True)
    ctrls = data["controllers"]
    _require(isinstance(ctrls, list) and ctrls, f"{source}: controllers must be a non-empty list")
    for c in ctrls:
        _require(c in CONTROLLERS, f"{source}: controller {c!r} unknown; known: {CONTROLLERS}")
    _require(len(ctrls) == len(set(ctrls)), f"{source}: controllers must be unique")
    unc = {"dt_factor": 0.5, "duration_factor": 2.0, "refine": True, **dict(data.get("uncertainty") or {})}
    _keys(unc, {"dt_factor", "duration_factor", "refine"}, f"{source}: uncertainty")
    _num(unc["dt_factor"], f"{source}: uncertainty.dt_factor", 0.05, 0.99)
    _num(unc["duration_factor"], f"{source}: uncertainty.duration_factor", 1.01, 10.0)
    _require(isinstance(unc["refine"], bool), f"{source}: uncertainty.refine must be true or false")
    por = {"controllers": [ctrls[0]], "points": 5, **dict(data.get("portraits") or {})}
    _keys(por, {"controllers", "points"}, f"{source}: portraits")
    _require(isinstance(por["controllers"], list) and set(por["controllers"]) <= set(ctrls),
             f"{source}: portraits.controllers must be a list of the experiment's controllers")
    _require(isinstance(por["points"], int) and 2 <= por["points"] <= 15, f"{source}: portraits.points must be in [2, 15]")
    workers = data.get("workers")
    _require(workers is None or (isinstance(workers, int) and not isinstance(workers, bool) and workers >= 1),
             f"{source}: workers must be a positive integer (omit for automatic)")
    outputs = dict(data.get("outputs") or {})
    _keys(outputs, {"directory", "formats"}, f"{source}: outputs")
    outputs.setdefault("formats", list(FORMATS))
    _require(set(outputs["formats"]) <= set(FORMATS), f"{source}: outputs.formats must be a subset of {FORMATS}")
    outputs.setdefault("directory", f"output/experiments/{data['name']}")
    return StabilityMapSpec(str(data["name"]), str(data["vehicle"]), mu, speed, steer, grid, tuple(ctrls), dt, duration,
                            crit, unc, por, workers, tyres, outputs, bool(data.get("verification", True)),
                            str(data.get("description", "")), source)


def describe(spec: StabilityMapSpec) -> str:
    nb, nr = spec.grid["beta_deg"][2], spec.grid["r_degps"][2]
    return (f"recovery map {nb}x{nr} at {spec.speed_kph:g} km/h, steer {spec.steer_deg:g} deg, mu {spec.mu:g}, "
            f"{len(spec.controllers)} controller(s)")


def _open_loop_equilibria(spec: StabilityMapSpec) -> list[dict]:
    from vehicle_model.handling_analysis.equilibrium import EquilibriumAnalysis, OperatingPoint
    from vehicle_model.simulation.stability_region import _setup

    make_model = _setup(spec.problem(spec.controllers[0]))[0]
    an = EquilibriumAnalysis(make_model())
    out = []
    for e in an.find_all(OperatingPoint(spec.speed_kph / 3.6, math.radians(spec.steer_deg))):
        lin = an.linearise(e)
        out.append({"beta_deg": math.degrees(e.beta), "r_degps": math.degrees(e.r), "stability": lin.stability(),
                    "max_real": lin.max_real(), "ay": e.ay})
    return out


def _heuristic_comparison(S: np.ndarray, outcomes: np.ndarray, s_crit: float) -> dict:
    inside = S < s_crit
    ret = outcomes == "returned"
    return {"agreement": float(np.mean(inside == ret)),
            "inside_and_returned": int(np.sum(inside & ret)), "inside_not_returned": int(np.sum(inside & ~ret)),
            "outside_and_returned": int(np.sum(~inside & ret)), "outside_not_returned": int(np.sum(~inside & ~ret))}


def run(spec: StabilityMapSpec, out: Path, verify: bool | None = None) -> dict:
    from vehicle_model.controllers.phase_plane import PhasePlaneConfig
    from vehicle_model.simulation.stability_region import (CONTROLLER_DESCRIPTIONS, compute_map, heuristic_boundary,
                                                           problem_dict)

    from .verification import verify_model
    formats = set(spec.outputs.get("formats", ()))
    verification = []
    if spec.verification if verify is None else verify:
        from vehicle_model.roads.surface import UniformRoad
        from vehicle_model.vehicle_models import SevenDOFModel
        verification = verify_model(lambda: SevenDOFModel.from_yaml(
            spec.vehicle, tyre_name=spec.tyres.get("front"), road=UniformRoad(mu=1.0), tyre_rear_name=spec.tyres.get("rear")))
    bg, rg = spec.beta_grid, spec.r_grid
    unc = spec.uncertainty
    S = heuristic_boundary(spec.problem(spec.controllers[0]), bg, rg)
    s_crit = PhasePlaneConfig().s_critical
    equilibria = _open_loop_equilibria(spec)
    maps, results = {}, []
    for c in spec.controllers:
        m = compute_map(spec.problem(c), bg, rg, spec.workers, unc["dt_factor"], unc["duration_factor"], unc["refine"])
        maps[c] = m
        results.append({"controller": c, "description": CONTROLLER_DESCRIPTIONS[c], "problem": problem_dict(m.problem),
                        **m.summary(), "heuristic_comparison": _heuristic_comparison(S, m.outcomes, s_crit),
                        "outcomes": m.outcomes.tolist(), "certainty": m.certainty.tolist(),
                        "refinement": m.refinement})
    portraits = _portraits(spec, maps)
    limits = _limit_cases(spec, maps)
    summary = {"experiment": spec.to_dict(), "source": spec.source, "provenance": provenance(),
               "verification": verification, "beta_grid_deg": np.degrees(bg).tolist(),
               "r_grid_degps": np.degrees(rg).tolist(), "open_loop_equilibria": equilibria,
               "heuristic_stability_index": S.tolist(), "heuristic_s_critical": s_crit, "results": results}
    if "json" in formats:
        write_json(out / "results.json", summary)
    if "csv" in formats:
        _csv(maps, S, out / "map.csv")
    if "png" in formats:
        _plot_maps(spec, maps, S, s_crit, equilibria, out / "stability_maps.png")
        _plot_shares(spec, results, out / "region_comparison.png")
        if portraits:
            _plot_portraits(spec, portraits, maps, equilibria, out / "phase_portraits.png")
        if limits:
            _plot_limits(spec, limits, out / "limit_cases.png")
    if "md" in formats:
        (out / "report.md").write_text(_report(spec, summary), encoding="utf-8")
    return summary


def _portraits(spec, maps) -> dict:
    from vehicle_model.simulation.stability_region import simulate
    n = int(spec.portraits["points"])
    out = {}
    for c in spec.portraits["controllers"]:
        m = maps[c]
        ref = (m.reference["beta"], m.reference["r"])
        ib = np.unique(np.linspace(0, m.beta_grid.size - 1, n).round().astype(int))
        ir = np.unique(np.linspace(0, m.r_grid.size - 1, n).round().astype(int))
        out[c] = [simulate(m.problem, m.beta_grid[i], m.r_grid[j], ref, record=True) for i in ib for j in ir]
    return out


def _limit_cases(spec, maps, per_controller: int = 2) -> dict:
    """Neighbouring initial states with ``returned`` vs ``loss_of_control``, re-run with histories."""
    from vehicle_model.simulation.stability_region import simulate
    out = {}
    for c, m in maps.items():
        o = m.outcomes
        pairs = []
        for i in range(o.shape[0]):
            for j in range(o.shape[1]):
                for di, dj in ((1, 0), (0, 1)):
                    k, l = i + di, j + dj
                    if k < o.shape[0] and l < o.shape[1] and {o[i, j], o[k, l]} == {"returned", "loss_of_control"}:
                        pairs.append(((i, j), (k, l)))
        if not pairs:
            continue
        pick = [pairs[int(x)] for x in np.unique(np.linspace(0, len(pairs) - 1, per_controller).round())]
        ref = (m.reference["beta"], m.reference["r"])
        out[c] = [[simulate(m.problem, m.beta_grid[a], m.r_grid[b], ref, record=True) for a, b in pair]
                  for pair in pick]
    return out


def _csv(maps, S, path: Path) -> None:
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["controller", "beta0_deg", "r0_degps", "outcome", "certainty", "t_end_s", "final_beta_deg",
                    "final_r_degps", "max_abs_beta_deg", "heuristic_S", "error"])
        for c, m in maps.items():
            nr = m.r_grid.size
            for k, run in enumerate(m.runs):
                i, j = divmod(k, nr)
                w.writerow([c, math.degrees(run.beta0), math.degrees(run.r0), run.outcome, m.certainty[i, j],
                            run.t_end, math.degrees(run.final_beta), math.degrees(run.final_r),
                            math.degrees(run.max_abs_beta), S[i, j], run.error])


# ----------------------------------------------------------------------------- plots
# Outcome colours: returned / loss reuse the status hues (good / critical); the others are neutral or
# warning hues; every class also has its own hatch-free marker in the legend and the CSV, so colour is not
# the only carrier of the class.
_OUTCOME_COLOURS = {"returned": "#199e70", "large_deviation": "#c98500", "loss_of_control": "#d03b3b",
                    "numerical_failure": "#8a63d2", "inconclusive": "#7d8590"}
_SERIES = ("#3987e5", "#d95926", "#199e70", "#c98500", "#d55181", "#008300")


def _grid_image(ax, bg_deg, rg_deg, outcomes):
    from matplotlib.colors import ListedColormap
    from vehicle_model.simulation.stability_region import OUTCOMES
    codes = np.vectorize(OUTCOMES.index)(outcomes)
    cmap = ListedColormap([_OUTCOME_COLOURS[o] for o in OUTCOMES])
    db = (bg_deg[1] - bg_deg[0]) / 2 if bg_deg.size > 1 else 1.0
    dr = (rg_deg[1] - rg_deg[0]) / 2 if rg_deg.size > 1 else 1.0
    ax.imshow(codes.T, origin="lower", cmap=cmap, vmin=-0.5, vmax=len(OUTCOMES) - 0.5, aspect="auto",
              extent=(bg_deg[0] - db, bg_deg[-1] + db, rg_deg[0] - dr, rg_deg[-1] + dr), interpolation="nearest")


def _plot_maps(spec, maps, S, s_crit, equilibria, path: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D
    from matplotlib.patches import Patch

    from vehicle_model.simulation.stability_region import OUTCOMES
    from vehicle_model.visualization.theme import TEXT_PRIMARY, apply_dark_theme, style_axis
    apply_dark_theme()
    n = len(maps)
    cols = min(3, n)
    rows = math.ceil(n / cols)
    fig, axes = plt.subplots(rows, cols, figsize=(5.6 * cols, 4.9 * rows + 0.8), squeeze=False)
    bg, rg = np.degrees(spec.beta_grid), np.degrees(spec.r_grid)
    for ax, (c, m) in zip(axes.flat, maps.items()):
        _grid_image(ax, bg, rg, m.outcomes)
        B, R = np.meshgrid(bg, rg, indexing="ij")
        sens = m.certainty == "sensitive"
        ax.plot(B[sens], R[sens], "x", color=TEXT_PRIMARY, ms=7, mew=1.8)
        if S.shape[0] > 1 and S.shape[1] > 1:
            ax.contour(B, R, S, levels=[s_crit], colors=[TEXT_PRIMARY], linestyles="--", linewidths=1.5)
        for e in equilibria:
            st = e["stability"] == "stable"
            ax.plot(e["beta_deg"], e["r_degps"], "o", ms=8, mec=TEXT_PRIMARY, mfc=TEXT_PRIMARY if st else "none", mew=2)
        ax.plot(math.degrees(m.reference["beta"]), math.degrees(m.reference["r"]), "*", ms=12, color=TEXT_PRIMARY)
        ax.set_xlim(bg[0] - (bg[1] - bg[0]) / 2, bg[-1] + (bg[1] - bg[0]) / 2)
        ax.set_ylim(rg[0] - (rg[1] - rg[0]) / 2, rg[-1] + (rg[1] - rg[0]) / 2)
        style_axis(ax, f"{c}: returned {m.share():.0%}", "initial sideslip beta0 [deg]", "initial yaw rate r0 [deg/s]")
    for ax in list(axes.flat)[n:]:
        ax.set_visible(False)
    handles = [Patch(color=_OUTCOME_COLOURS[o], label=o.replace("_", " ")) for o in OUTCOMES]
    handles += [Line2D([], [], ls="", marker="x", color=TEXT_PRIMARY, mew=1.8, label="sensitive (step / duration)"),
                Line2D([], [], ls="--", color=TEXT_PRIMARY, label="heuristic phase-plane boundary S = 1"),
                Line2D([], [], ls="", marker="o", mec=TEXT_PRIMARY, mfc=TEXT_PRIMARY, label="stable equilibrium (open loop)"),
                Line2D([], [], ls="", marker="o", mec=TEXT_PRIMARY, mfc="none", mew=2, label="unstable equilibrium (open loop)"),
                Line2D([], [], ls="", marker="*", color=TEXT_PRIMARY, ms=10, label="reference (closed-loop steady state)")]
    fig.legend(handles=handles, loc="lower center", ncol=5, fontsize=8, frameon=False)
    fig.suptitle(f"{spec.name}: numerical recovery regions at {spec.speed_kph:g} km/h, steer {spec.steer_deg:g} deg, "
                 f"mu {spec.mu:g}\n(synthetic parameters; not a proven region of attraction)")
    fig.tight_layout(rect=(0, 0.07, 1, 0.97))
    fig.savefig(path, dpi=130, facecolor=fig.get_facecolor())
    plt.close(fig)


def _plot_shares(spec, results, path: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    from vehicle_model.visualization.theme import TEXT_PRIMARY, apply_dark_theme, style_axis
    apply_dark_theme()
    fig, ax = plt.subplots(figsize=(1.6 * len(results) + 4, 4.8))
    x = np.arange(len(results))
    share = np.array([r["returned_share"] for r in results])
    lo = np.array([r["returned_share_bounds"][0] for r in results])
    hi = np.array([r["returned_share_bounds"][1] for r in results])
    ax.bar(x, share * 100, width=0.6, color=_SERIES[0])
    ax.errorbar(x, share * 100, yerr=[(share - lo) * 100, (hi - share) * 100], fmt="none", ecolor=TEXT_PRIMARY,
                capsize=6, lw=1.5)
    ref = [r["returned_share_refined"] for r in results]
    if all(v is not None for v in ref):
        ax.plot(x, np.array(ref) * 100, "D", color=_SERIES[1], ms=8, label="refined grid")
        ax.legend(fontsize=8)
    for xi, s in zip(x, share):
        ax.text(xi, s * 100 - 4.0, f"{s:.0%}", ha="center", va="top", fontsize=10, color="white", fontweight="bold")
    ax.set_xticks(x, [r["controller"] for r in results])
    ax.set_ylim(0, 105)
    style_axis(ax, "share of returned initial states; bars: bounds from boundary and sensitive points", None,
               "returned [% of grid points]")
    fig.tight_layout()
    fig.savefig(path, dpi=130, facecolor=fig.get_facecolor())
    plt.close(fig)


def _plot_portraits(spec, portraits, maps, equilibria, path: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    from vehicle_model.visualization.theme import TEXT_PRIMARY, apply_dark_theme, style_axis
    apply_dark_theme()
    fig, axes = plt.subplots(len(portraits), 3, figsize=(17, 4.8 * len(portraits)), squeeze=False)
    for row, (c, runs) in zip(axes, portraits.items()):
        for run in runs:
            h = run.history
            col = _OUTCOME_COLOURS[run.outcome]
            b, r = np.degrees(h["beta"]), np.degrees(h["r"])
            bd = np.degrees(h["beta_dot"])
            rd = np.gradient(r, h["t"]) if r.size > 1 else np.zeros_like(r)
            for ax, (x, y) in zip(row, ((b, r), (b, bd), (r, rd))):
                ax.plot(x, y, color=col, lw=1.3, alpha=0.9)
                if x.size:
                    ax.plot(x[0], y[0], "o", color=col, ms=4)
        for e in equilibria:
            st = e["stability"] == "stable"
            row[0].plot(e["beta_deg"], e["r_degps"], "o", ms=8, mec=TEXT_PRIMARY, mfc=TEXT_PRIMARY if st else "none", mew=2)
        style_axis(row[0], f"{c}: sideslip / yaw rate", "beta [deg]", "r [deg/s]")
        style_axis(row[1], f"{c}: sideslip / sideslip rate", "beta [deg]", "beta_dot [deg/s]")
        style_axis(row[2], f"{c}: yaw rate / yaw acceleration", "r [deg/s]", "r_dot [deg/s²]")
    fig.suptitle(f"{spec.name}: phase portraits (dot = start; colour = outcome, as in the maps)")
    fig.tight_layout()
    fig.savefig(path, dpi=130, facecolor=fig.get_facecolor())
    plt.close(fig)


def _plot_limits(spec, limits, path: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    from vehicle_model.visualization.theme import apply_dark_theme, style_axis
    apply_dark_theme()
    fig, axes = plt.subplots(len(limits), 2, figsize=(14, 3.6 * len(limits)), squeeze=False)
    for row, (c, pairs) in zip(axes, limits.items()):
        for k, pair in enumerate(pairs):
            for run in pair:
                h = run.history
                ls = "-" if k == 0 else ":"
                lab = (f"beta0 {math.degrees(run.beta0):.1f}, r0 {math.degrees(run.r0):.1f}: "
                       f"{run.outcome.replace('_', ' ')}")
                row[0].plot(h["t"], np.degrees(h["beta"]), color=_OUTCOME_COLOURS[run.outcome], ls=ls, lw=2, label=lab)
                row[1].plot(h["t"], np.degrees(h["r"]), color=_OUTCOME_COLOURS[run.outcome], ls=ls, lw=2)
        row[0].legend(fontsize=7, loc="best")
        style_axis(row[0], f"{c}: sideslip angle, neighbouring initial states", "time [s]", "beta [deg]")
        style_axis(row[1], f"{c}: yaw rate", "time [s]", "r [deg/s]")
    fig.suptitle(f"{spec.name}: limit cases (neighbouring grid points with different outcomes)")
    fig.tight_layout()
    fig.savefig(path, dpi=130, facecolor=fig.get_facecolor())
    plt.close(fig)


def _report(spec, summary: dict) -> str:
    pv = summary["provenance"]
    nb, nr = spec.grid["beta_deg"][2], spec.grid["r_degps"][2]
    L = [f"# Stability map: {spec.name}", ""]
    if spec.description:
        L += [spec.description, ""]
    L += ["## Setup", "", "| item | value |", "|---|---|", f"| specification | `{spec.source}` |",
          f"| vehicle | `{spec.vehicle}`, tyres {spec.tyres or 'from the vehicle file'} |",
          f"| operating point | {spec.speed_kph:g} km/h, front steer {spec.steer_deg:g} deg held, mu = {spec.mu:g} |",
          f"| grid | beta0 {spec.grid['beta_deg'][:2]} deg x r0 {spec.grid['r_degps'][:2]} deg/s, {nb} x {nr} points |",
          f"| simulation | dt {spec.dt} s, up to {spec.duration_s} s, speed-holding pedal |",
          f"| code | git {pv['git_commit']}, Python {pv['python']}, numpy {pv['numpy']}, scipy {pv['scipy']} |", "",
          "All parameters are synthetic; results describe the virtual vehicle, not a real one. A map is a",
          "numerically determined recovery region for this grid, step and duration, not a proven region of attraction.", ""]
    if summary["verification"]:
        ok = all(c["passed"] for c in summary["verification"])
        L += [f"Model verification: **{'PASS' if ok else 'FAIL'}** "
              f"({sum(c['passed'] for c in summary['verification'])}/{len(summary['verification'])} checks).", ""]
    L += ["## Open-loop equilibria of the operating point", "", "| beta [deg] | r [deg/s] | a_y [m/s²] | stability | max Re [1/s] |",
          "|---|---|---|---|---|"]
    for e in summary["open_loop_equilibria"]:
        L.append(f"| {e['beta_deg']:.2f} | {e['r_degps']:.2f} | {e['ay']:.2f} | {e['stability']} | {e['max_real']:.3f} |")
    L += ["", "## Recovery regions", "",
          "| controller | returned | bounds | refined grid | loss of control | large deviation | inconclusive | "
          "numerical failure | sensitive points | reference beta / r |", "|---" * 10 + "|"]
    for r in summary["results"]:
        c = r["counts"]
        lo, hi = r["returned_share_bounds"]
        ref = r["reference"]
        rs = r["returned_share_refined"]
        L.append(f"| {r['controller']} | {r['returned_share']:.0%} | {lo:.0%} – {hi:.0%} | "
                 f"{'–' if rs is None else f'{rs:.0%}'} | {c['loss_of_control']} | {c['large_deviation']} | "
                 f"{c['inconclusive']} | {c['numerical_failure']} | {r['sensitive_points']} | "
                 f"{math.degrees(ref['beta']):.2f} / {math.degrees(ref['r']):.2f}"
                 f"{'' if ref['settled'] else ' (not settled)'} |")
    L += ["", "## Numerical uncertainty", "", "| controller | re-run with smaller step: changed / re-run | "
          "re-run with longer duration: changed / re-run | refinement: simulated / inherited midpoints |", "|---|---|---|---|"]
    for r in summary["results"]:
        s, d, rf = r["sensitivity"]["step"], r["sensitivity"]["duration"], r["refinement"] or {}
        L.append(f"| {r['controller']} | {s['changed']} / {s['rerun']} (dt x {s['factor']}) | "
                 f"{d['changed']} / {d['rerun']} (x {d['factor']}) | "
                 f"{rf.get('simulated_midpoints', '–')} / {rf.get('inherited_midpoints', '–')} |")
    L += ["", "## Heuristic phase-plane boundary as comparison basis", "",
          f"Stability index S of `controllers/phase_plane.py` at the initial states; inside: S < {summary['heuristic_s_critical']:g}.",
          "", "| controller | agreement with 'returned' | inside, not returned | outside, returned |", "|---|---|---|---|"]
    for r in summary["results"]:
        h = r["heuristic_comparison"]
        L.append(f"| {r['controller']} | {h['agreement']:.0%} | {h['inside_not_returned']} | {h['outside_and_returned']} |")
    L += ["", "## Classification", "",
          "- returned: inside |beta - beta_ref| < target_beta and |r - r_ref| < target_r for hold seconds;",
          "  the reference is the closed-loop steady state of each controller from the open-loop equilibrium.",
          "- loss of control: |beta| >= loss_beta. large deviation: settled outside the target region.",
          "  inconclusive: still moving at the end. numerical failure: non-finite state or exception.",
          "- Bounds: lower = returned and neither boundary nor sensitive; upper = returned or boundary/sensitive.",
          "  Refined grid: midpoints between neighbouring points of different outcome simulated, others inherited.", ""]
    return "\n".join(L)


__all__ = ("KIND", "StabilityMapSpec", "describe", "parse", "run")
