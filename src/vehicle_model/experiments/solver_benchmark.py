"""Experiment type ``solver_benchmark``: computation effort and solution quality of the dynamic NMPC (Plan V3.0 §12, M10 5.2).

Every variant (a set of :class:`~vehicle_model.controllers.dynamic_two_track_nmpc.DynamicNMPCConfig`
overrides on top of ``base``) drives the same closed-loop lane change on a fresh
plant. Reported per variant: solver status shares, KKT residuals, cost improvement
over the warm start, iterations, computation time, plan feasibility and the
tracking result, so effort and quality are read together.

Example (``configs/experiments/dynamic_nmpc_solver_benchmark.yaml``)::

    type: solver_benchmark
    name: dynamic_nmpc_solver_benchmark
    vehicle: configs/vehicles/seven_dof_passenger.yaml
    road: {type: uniform, mu: 0.75}
    manoeuvre: {type: lane_change, speed_kph: 90, offset_m: 3.5, start_s: 0.2, duration_s: 2.0}
    control_dt: 0.05
    base: {N: 10, max_iter: 15, max_steer_rear_deg: 8, max_yaw_moment_nm: 3500, w_kamm_circle: 80}
    variants:
      - {name: "max_iter 15"}
      - {name: "max_iter 100", max_iter: 100}
    outputs: {directory: output/experiments/dynamic_nmpc_solver_benchmark}

Configuration keys are the field names of ``DynamicNMPCConfig``; angles may be given
in degrees with ``_deg`` / ``_degps`` instead of ``_rad`` / ``_radps``. Computation
times are wall clock and not reproducible; everything else is.
"""

from __future__ import annotations

import csv
import dataclasses
import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from .spec import _keys, _require
from .types import provenance, slug, write_json

KIND = "solver_benchmark"
_TOP = {"type", "name", "description", "vehicle", "tyres", "road", "manoeuvre", "control_dt", "base", "variants",
        "outputs", "verification"}
_MANOEUVRE = {"type", "speed_kph", "offset_m", "start_s", "duration_s"}
FORMATS = ("csv", "json", "png", "md")
_STEP_KEYS = ("t", "X", "Y", "psi", "vx", "steer_f", "steer_r", "status", "iterations", "solve_ms",
              "kkt_stationarity_relative", "kkt_primal", "relative_cost_improvement", "warm_start_step",
              "plan_feasible")


@dataclass(frozen=True)
class SolverBenchmarkSpec:
    name: str
    vehicle: str
    road: dict
    manoeuvre: dict
    control_dt: float
    base: dict
    variants: tuple            # ((name, overrides), ...)
    tyres: dict = field(default_factory=dict)
    outputs: dict = field(default_factory=dict)
    verification: bool = True
    description: str = ""
    source: str = ""
    kind = KIND

    def config(self, overrides: dict):
        from vehicle_model.controllers.dynamic_two_track_nmpc import DynamicNMPCConfig
        return dataclasses.replace(DynamicNMPCConfig(), **{**_to_config(self.base), **_to_config(overrides)})

    def to_dict(self) -> dict:
        return {"type": KIND, "name": self.name, "description": self.description, "vehicle": self.vehicle,
                "tyres": dict(self.tyres), "road": dict(self.road), "manoeuvre": dict(self.manoeuvre),
                "control_dt": self.control_dt, "base": dict(self.base),
                "variants": [{"name": n, **o} for n, o in self.variants], "outputs": dict(self.outputs),
                "verification": self.verification}


def _config_fields() -> dict:
    from vehicle_model.controllers.dynamic_two_track_nmpc import DynamicNMPCConfig
    return {f.name: f for f in dataclasses.fields(DynamicNMPCConfig)}


def _canonical(key: str) -> tuple[str, float]:
    """Config field name and factor for a YAML key (degree aliases)."""
    for deg, rad in (("_degps", "_radps"), ("_deg", "_rad")):
        if key.endswith(deg):
            return key[: -len(deg)] + rad, math.pi / 180.0
    return key, 1.0


def _to_config(d: dict) -> dict:
    out = {}
    fields = _config_fields()
    for k, v in d.items():
        name, factor = _canonical(k)
        default = fields[name].default
        if isinstance(default, bool):
            out[name] = bool(v)
        elif isinstance(default, int):
            out[name] = int(v)
        elif v is None:
            out[name] = None
        else:
            out[name] = float(v) * factor
    return out


def _check_config(d: dict, where: str) -> None:
    _require(isinstance(d, dict), f"{where} must be a mapping")
    fields = _config_fields()
    for k, v in d.items():
        name, _ = _canonical(k)
        _require(name in fields, f"{where}: unknown key {k!r}; allowed: DynamicNMPCConfig fields "
                                 f"(angles also as *_deg / *_degps)")
        default = fields[name].default
        if isinstance(default, bool):
            _require(isinstance(v, bool), f"{where}.{k} must be true or false")
        elif isinstance(default, int):
            _require(isinstance(v, int) and not isinstance(v, bool) and v >= 1, f"{where}.{k} must be an integer >= 1")
        elif not (name == "time_budget_ms" and v is None):
            _require(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v),
                     f"{where}.{k} must be a number")
    seen = [_canonical(k)[0] for k in d]
    _require(len(seen) == len(set(seen)), f"{where}: a key is given in degrees and radians")


def parse(data: dict, source: str = "<dict>") -> SolverBenchmarkSpec:
    _require(data.get("type") == KIND, f"{source}: type must be {KIND!r}")
    _keys(data, _TOP, source, {"type", "name", "vehicle", "road", "manoeuvre", "variants"})
    _require(Path(data["vehicle"]).is_file(), f"{source}: vehicle file not found: {data['vehicle']}")
    tyres = data.get("tyres") or {}
    _keys(tyres, {"front", "rear"}, f"{source}: tyres")
    road = dict(data["road"])
    _keys(road, {"type", "mu"}, f"{source}: road", {"type", "mu"})
    _require(road["type"] == "uniform", f"{source}: road.type must be 'uniform'")
    _require(0.0 < float(road["mu"]) <= 2.0, f"{source}: road.mu must lie in (0, 2]")
    man = {"offset_m": 3.5, "start_s": 0.2, "duration_s": 2.0, **dict(data["manoeuvre"])}
    _keys(man, _MANOEUVRE, f"{source}: manoeuvre", {"type", "speed_kph"})
    _require(man["type"] == "lane_change", f"{source}: manoeuvre.type must be 'lane_change'")
    _require(5.0 <= float(man["speed_kph"]) <= 250.0, f"{source}: manoeuvre.speed_kph must lie in [5, 250]")
    _require(abs(float(man["offset_m"])) <= 10.0, f"{source}: manoeuvre.offset_m must lie in [-10, 10]")
    _require(0.0 < float(man["duration_s"]) <= 20.0, f"{source}: manoeuvre.duration_s must lie in (0, 20]")
    _require(0.0 <= float(man["start_s"]) < float(man["duration_s"]), f"{source}: manoeuvre.start_s must lie in [0, duration_s)")
    control_dt = float(data.get("control_dt", 0.05))
    _require(0.0 < control_dt <= 0.1, f"{source}: control_dt must lie in (0, 0.1] s")
    base = dict(data.get("base") or {})
    _check_config(base, f"{source}: base")
    variants = data["variants"]
    _require(isinstance(variants, list) and variants, f"{source}: variants must be a non-empty list")
    parsed = []
    for i, v in enumerate(variants):
        _require(isinstance(v, dict) and isinstance(v.get("name"), str) and v["name"],
                 f"{source}: variants[{i}] needs a name")
        over = {k: x for k, x in v.items() if k != "name"}
        _check_config(over, f"{source}: variants[{i}]")
        parsed.append((v["name"], over))
    names = [n for n, _ in parsed]
    _require(len(names) == len(set(names)), f"{source}: variant names must be unique")
    outputs = dict(data.get("outputs") or {})
    _keys(outputs, {"directory", "formats"}, f"{source}: outputs")
    outputs.setdefault("formats", list(FORMATS))
    _require(set(outputs["formats"]) <= set(FORMATS), f"{source}: outputs.formats must be a subset of {FORMATS}")
    outputs.setdefault("directory", f"output/experiments/{data['name']}")
    spec = SolverBenchmarkSpec(str(data["name"]), str(data["vehicle"]), road, man, control_dt, base, tuple(parsed),
                               dict(tyres), outputs, bool(data.get("verification", True)),
                               str(data.get("description", "")), source)
    for _, over in spec.variants:            # every variant must give a valid configuration
        try:
            spec.config(over)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{source}: invalid configuration: {exc}") from None
    return spec


def describe(spec: SolverBenchmarkSpec) -> str:
    m = spec.manoeuvre
    return f"dynamic NMPC, {m['type']} at {m['speed_kph']} km/h, {len(spec.variants)} variant(s)"


def _plant(spec: SolverBenchmarkSpec):
    from vehicle_model.roads.surface import UniformRoad
    from vehicle_model.vehicle_models import SevenDOFModel
    return SevenDOFModel.from_yaml(spec.vehicle, tyre_name=spec.tyres.get("front"),
                                   road=UniformRoad(mu=float(spec.road["mu"])),
                                   tyre_rear_name=spec.tyres.get("rear"))


def run_variant(spec: SolverBenchmarkSpec, overrides: dict) -> tuple[dict, dict]:
    """Closed-loop lane change with one configuration: (per-step history, KPIs)."""
    from vehicle_model.controllers.dynamic_two_track_nmpc import DynamicTwoTrackNMPC
    from vehicle_model.controllers.framework import ControlTarget
    from vehicle_model.core.types import SevenDOFControlInput, SevenDOFState

    plant = _plant(spec)
    p, mu, m, dt = plant.params, float(spec.road["mu"]), spec.manoeuvre, spec.control_dt
    nmpc = DynamicTwoTrackNMPC(params=p, config=spec.config(overrides))
    v0, offset = float(m["speed_kph"]) / 3.6, float(m["offset_m"])
    s = SevenDOFState(vx=v0, omega=np.full(4, v0 / p.Reff))
    h = {k: [] for k in _STEP_KEYS}
    for i in range(int(round(float(m["duration_s"]) / dt))):
        t = i * dt
        y_ref = offset if t >= float(m["start_s"]) - 1e-9 else 0.0
        out = nmpc.compute_control(s, ControlTarget(target_y=y_ref, target_psi=0.0, target_vx=v0, mu_estimated=mu), dt=dt)
        s, _ = plant.step(s, SevenDOFControlInput(steer_f=out.steer_f, steer_r=out.steer_r, T_drive=out.T_drive,
                                                  T_brake=out.T_brake), dt)
        rep, feas = out.diagnostics["solver"], out.diagnostics["feasibility"]
        for k, v in (("t", t + dt), ("X", s.X), ("Y", s.Y), ("psi", s.psi), ("vx", s.vx), ("steer_f", out.steer_f),
                     ("steer_r", out.steer_r), ("status", out.status), ("iterations", rep["iterations"]),
                     ("solve_ms", rep["solve_time_ms"]), ("kkt_stationarity_relative", rep["kkt_stationarity_relative"]),
                     ("kkt_primal", rep["kkt_primal"]), ("relative_cost_improvement", rep["relative_cost_improvement"]),
                     ("warm_start_step", rep["warm_start_step"]), ("plan_feasible", feas["feasible"])):
            h[k].append(v)
    steer = np.r_[0.0, h["steer_f"]]
    kpis = {"final_lateral_error": abs(s.Y - offset),
            "lateral_overshoot": max(0.0, (max(h["Y"]) - offset) if offset >= 0 else (offset - min(h["Y"]))),
            "max_steer_rate_degps": float(np.degrees(np.max(np.abs(np.diff(steer)) / dt))),
            "plan_infeasible_steps": int(sum(not f for f in h["plan_feasible"])),
            **{f"solver_{k}": v for k, v in nmpc.monitor.summary().items()}}
    return h, kpis


def run(spec: SolverBenchmarkSpec, out: Path, verify: bool | None = None) -> dict:
    from .verification import verify_model
    formats = set(spec.outputs.get("formats", ()))
    verification = verify_model(lambda: _plant(spec)) if (spec.verification if verify is None else verify) else []
    results, histories = [], {}
    for name, over in spec.variants:
        h, kpis = run_variant(spec, over)
        histories[name] = h
        results.append({"variant": name, "overrides": over, "config": dataclasses.asdict(spec.config(over)),
                        "kpis": kpis})
        if "csv" in formats:
            (out / "histories").mkdir(exist_ok=True)
            with open(out / "histories" / f"{slug(name)}.csv", "w", newline="", encoding="utf-8") as f:
                w = csv.writer(f)
                w.writerow(_STEP_KEYS)
                w.writerows(zip(*(h[k] for k in _STEP_KEYS)))
    summary = {"experiment": spec.to_dict(), "source": spec.source, "provenance": provenance(),
               "verification": verification, "results": results}
    if "json" in formats:
        write_json(out / "results.json", summary)
    if "csv" in formats:
        keys = list(dict.fromkeys(k for r in results for k in r["kpis"]))
        with open(out / "kpis.csv", "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["variant", *keys])
            for r in results:
                w.writerow([r["variant"], *(r["kpis"].get(k, "") for k in keys)])
    if "png" in formats:
        _plot(spec, histories, out / "solver_benchmark.png")
    if "md" in formats:
        (out / "report.md").write_text(_report(spec, summary), encoding="utf-8")
    return summary


def _plot(spec, histories: dict, path: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    from vehicle_model.visualization.theme import TEXT_MUTED, apply_dark_theme, style_axis
    colours = ("#3987e5", "#d95926", "#199e70", "#c98500", "#d55181", "#008300")  # validated slots 1-6
    apply_dark_theme()
    fig, ax = plt.subplots(1, 3, figsize=(17, 4.8))
    for (name, h), c in zip(histories.items(), colours * 4):
        t = np.asarray(h["t"])
        ax[0].plot(t, h["Y"], color=c, lw=2, label=name)
        ax[1].plot(t, h["solve_ms"], color=c, lw=2, marker="o", ms=4)
        kkt = np.asarray(h["kkt_stationarity_relative"], float)
        ax[2].semilogy(t, np.maximum(kkt, 1e-16), color=c, lw=2, marker="o", ms=4)
    ax[0].axhline(float(spec.manoeuvre["offset_m"]), color=TEXT_MUTED, lw=1, ls="--")
    ax[0].legend(fontsize=8, loc="lower right")
    style_axis(ax[0], "lateral position", "time [s]", "Y [m]")
    style_axis(ax[1], "computation time per solve (wall clock)", "time [s]", "[ms]")
    style_axis(ax[2], "KKT stationarity / max(1, |J|)", "time [s]", "[-]")
    m = spec.manoeuvre
    fig.suptitle(f"{spec.name}: dynamic NMPC, lane change {m['offset_m']} m at {m['speed_kph']} km/h, "
                 f"mu = {spec.road['mu']} (synthetic parameters)")
    fig.tight_layout()
    fig.savefig(path, dpi=130, facecolor=fig.get_facecolor())
    plt.close(fig)


def _fmt(v) -> str:
    if isinstance(v, float):
        return "–" if math.isnan(v) else f"{v:.3g}"
    return str(v)


def _report(spec: SolverBenchmarkSpec, summary: dict) -> str:
    pv, m = summary["provenance"], spec.manoeuvre
    L = [f"# Solver benchmark: {spec.name}", ""]
    if spec.description:
        L += [spec.description, ""]
    L += ["## Setup", "", "| item | value |", "|---|---|", f"| specification | `{spec.source}` |",
          f"| vehicle | `{spec.vehicle}` |", f"| road | uniform, mu = {spec.road['mu']} |",
          f"| manoeuvre | lane change {m['offset_m']} m at {m['speed_kph']} km/h, step at {m['start_s']} s, "
          f"{m['duration_s']} s |", f"| control period | {spec.control_dt} s |",
          f"| base configuration | {', '.join(f'{k}: {v}' for k, v in spec.base.items()) or 'defaults'} |",
          f"| code | git {pv['git_commit']}, Python {pv['python']}, numpy {pv['numpy']}, scipy {pv['scipy']} |", "",
          "All parameters are synthetic; results describe the virtual vehicle, not a real one.", ""]
    if summary["verification"]:
        ok = all(c["passed"] for c in summary["verification"])
        L += [f"Model verification: **{'PASS' if ok else 'FAIL'}** "
              f"({sum(c['passed'] for c in summary['verification'])}/{len(summary['verification'])} checks).", ""]
    cols = [("final_lateral_error", "final lateral error [m]"), ("lateral_overshoot", "overshoot [m]"),
            ("max_steer_rate_degps", "max steer rate [deg/s]"), ("plan_infeasible_steps", "infeasible plans"),
            ("solver_fraction_nominal", "nominal"), ("solver_fraction_suboptimal", "suboptimal"),
            ("solver_fraction_fallback", "fallback"), ("solver_fraction_degraded", "degraded"),
            ("solver_iterations_mean", "iterations (mean)"),
            ("solver_kkt_stationarity_relative_median", "KKT rel. (median)"),
            ("solver_kkt_stationarity_relative_max", "KKT rel. (max)"), ("solver_kkt_primal_max", "primal (max)"),
            ("solver_relative_cost_improvement_median", "improvement over warm start (median)"),
            ("solver_solve_ms_mean", "solve time mean [ms]"), ("solver_solve_ms_max", "solve time max [ms]")]
    L += ["## Results", "", "| variant | overrides | " + " | ".join(c for _, c in cols) + " |",
          "|---" * (len(cols) + 2) + "|"]
    for r in summary["results"]:
        over = ", ".join(f"{k}: {v}" for k, v in r["overrides"].items()) or "–"
        L.append(f"| {r['variant']} | {over} | " + " | ".join(_fmt(r["kpis"].get(k, float("nan"))) for k, _ in cols) + " |")
    L += ["", "## Measures and limits", "",
          "- Status classes (nominal / suboptimal / fallback / degraded) and the KKT residuals are defined in",
          "  `controllers/solver_diagnostics.py` (docs/MOTION_CONTROL.md 3, 3.2). SLSQP reports convergence when the",
          "  relative cost change falls below `tolerance`; the KKT stationarity shows how far that is from a",
          "  first-order optimum.",
          "- Plan feasibility (hard limits) is checked separately from the solver status.",
          "- Computation times are wall clock on this machine and not reproducible; SciPy's SLSQP cannot be",
          "  interrupted, so a time budget is reported, not enforced. All other results are reproducible.", ""]
    return "\n".join(L)


__all__ = ("KIND", "SolverBenchmarkSpec", "describe", "parse", "run", "run_variant")
