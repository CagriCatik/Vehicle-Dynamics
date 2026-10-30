"""Scenario book: one catalogue of the runnable driving scenarios, with parameters and outputs.

Every entry wraps an existing scenario implementation (nothing is re-implemented
here) and runs it into its own output directory with a ``summary.json``. Parameters
have defaults; ``--set key=value`` overrides them, unknown keys are errors.

Categories:

    parking               4WS vs 2WS at walking pace (event-driven driver phases, space need)
    four_wheel_steering   the 4WS benchmark scenarios, 2WS vs 4WS (look-up table)
    split_mu              split-friction braking and launch scenarios
    yaw_stability         YAML experiments: controllers on step steer, sine with dwell, split-mu braking
    solver                YAML solver benchmarks: NMPC status, KKT residuals, computation time
    nonlinear_stability   YAML equilibrium analyses and recovery maps (equilibria, eigenvalues, stability regions)
    robust_control        YAML robust benchmarks: controllers x feedback modes x uncertainty cases
    critical_scenarios    YAML critical scenario searches (M13): search, verification, minimal cases, replay
    design                YAML design optimisations (M14): NSGA-II Pareto analysis, independent re-check
    roll_stability        YAML roll stability benchmarks on the 14-DOF model (V4 M19): RSC, active roll
    ride_comfort          YAML ride and pitch benchmarks on the 14-DOF model (V4 M21): preview damping, active pitch
    hill_descent          YAML hill descent benchmarks on graded roads (V4 M22): HDC, grade x surface matrix
    fault_tolerance       YAML fault catalogues on the 7-DOF loop (V4 M23): health monitor, degraded modes
    validation            the validation matrix (configs/validation/matrix.yaml), one scenario at a time
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable

CATEGORIES = ("parking", "four_wheel_steering", "split_mu", "yaw_stability", "solver", "nonlinear_stability",
              "robust_control", "critical_scenarios", "design", "roll_stability", "ride_comfort", "hill_descent",
              "fault_tolerance", "validation")
VEHICLE = "configs/vehicles/seven_dof_passenger.yaml"


@dataclass(frozen=True)
class ScenarioEntry:
    name: str
    category: str
    title: str
    description: str
    run: Callable[[Path, dict, bool], dict]     # (output dir, parameters, gif) -> summary
    params: dict = field(default_factory=dict)  # defaults; overridable
    runtime: str = ""
    outputs: str = ""

    def resolve(self, overrides: dict) -> dict:
        unknown = set(overrides) - set(self.params)
        if unknown:
            raise KeyError(f"{self.name}: unknown parameter(s) {sorted(unknown)}; allowed: {sorted(self.params) or 'none'}")
        out = dict(self.params)
        for k, v in overrides.items():
            default = self.params[k]
            if isinstance(default, bool) and not isinstance(v, bool):
                raise TypeError(f"{self.name}.{k}: expected true/false, got {v!r}")
            if isinstance(default, (int, float)) and not isinstance(default, bool):
                if isinstance(v, bool) or not isinstance(v, (int, float)):
                    raise TypeError(f"{self.name}.{k}: expected a number, got {v!r}")
                v = type(default)(v) if isinstance(default, float) or float(v).is_integer() else float(v)
            out[k] = v
        return out


def _model():
    from vehicle_model.vehicle_models import SevenDOFModel
    return SevenDOFModel.from_yaml(VEHICLE)


def _write(out: Path, summary: dict) -> dict:
    out.mkdir(parents=True, exist_ok=True)
    (out / "summary.json").write_text(json.dumps(summary, indent=2, default=float), encoding="utf-8")
    return summary


# ----------------------------------------------------------------------------- parking
def _parking(name: str):
    def run(out: Path, params: dict, gif: bool) -> dict:
        from vehicle_model.simulation.four_wheel_steering.parking import render_parking
        summary = render_parking(name, _model, out, gif=gif, dt=params["dt"])
        return _write(out, {"scenario": name, "results": summary})
    return run


# ----------------------------------------------------------------------------- 4WS benchmark scenarios
def _four_ws(func_name: str, kind: str, gif_kwargs: dict | None = None):
    """kind 'low': enable_4ws flag; 'high': controller 2WS (passive) vs 4WS LUT + yaw damping."""
    def run(out: Path, params: dict, gif: bool) -> dict:
        import vehicle_model.simulation.four_wheel_steering_scenarios as fws
        from vehicle_model.simulation.four_wheel_steering.animation import animate_4ws_detailed_vehicle, plot_4ws_comparison
        sim = getattr(fws, func_name)
        if kind == "low":
            r2 = sim(model=_model(), enable_4ws=False, controller_name="2WS (front only)", **params)
            r4 = sim(model=_model(), enable_4ws=True, controller_name="4WS (LUT)", **params)
        else:
            from vehicle_model.controllers.framework import FourWheelSteeringController, PassiveBaselineController
            from vehicle_model.steering.four_wheel_steering import SteeringMode
            r2 = sim(model=_model(), controller=PassiveBaselineController(), controller_name="2WS (front only)", **params)
            r4 = sim(model=_model(), controller=FourWheelSteeringController(mode=SteeringMode.LOOKUP_TABLE,
                                                                            enable_yaw_damping=True, k_yaw_damping=0.08),
                     controller_name="4WS (LUT + yaw damping)", **params)
        p = _model().params
        title = ENTRY_TITLES[func_name]
        plot_4ws_comparison(r4, r2, out / "comparison.png", title, p.lf, p.lr)
        if gif:
            animate_4ws_detailed_vehicle(r4, r2, out / "animation.gif", title=title, lf=p.lf, lr=p.lr, **(gif_kwargs or {}))
        return _write(out, {"scenario": func_name, "params": params,
                            "results": {r.controller_name: {k: v for k, v in r.metrics.__dict__.items()} for r in (r2, r4)}})
    return run


ENTRY_TITLES = {
    "simulate_low_speed_turning_circle": "Low-speed turning circle: 2WS vs 4WS",
    "simulate_urban_90deg_cornering": "Urban 90 deg corner: 2WS vs 4WS",
    "simulate_parallel_parking": "Reverse parallel parking (time-based driver): 2WS vs 4WS",
    "simulate_high_speed_double_lane_change": "Double lane change: 2WS vs 4WS",
    "simulate_high_speed_slalom": "High-speed slalom: 2WS vs 4WS",
    "simulate_autobahn_lane_change": "Motorway lane change: 2WS vs 4WS",
    "simulate_high_speed_step_steer": "Step steer: 2WS vs 4WS",
    "simulate_sine_with_dwell": "Sine with dwell: 2WS vs 4WS",
    "simulate_split_mu_braking_4ws": "Split-mu braking: 2WS vs 4WS (yaw damping)",
}


# ----------------------------------------------------------------------------- split-mu
def _split_mu_matrix(out: Path, params: dict, gif: bool) -> dict:
    from vehicle_model.simulation.four_wheel_steering.split_mu_matrix import matrix_table, run_split_mu_matrix
    from vehicle_model.vehicle_models import SevenDOFModel
    rows = run_split_mu_matrix(lambda road: SevenDOFModel.from_yaml(road=road), **params)
    out.mkdir(parents=True, exist_ok=True)
    (out / "matrix.md").write_text(matrix_table(rows) + "\n", encoding="utf-8")
    return _write(out, {"params": params, "rows": [{"configuration": r.configuration, "stable": r.stable,
                                                    "yaw_moment_by": r.yaw_moment_by, **r.kpis,
                                                    "peak_brake_yaw_moment": r.peak_brake_yaw_moment,
                                                    "peak_steer_yaw_moment": r.peak_steer_yaw_moment} for r in rows]})


def _split_mu_suite(out: Path, params: dict, gif: bool) -> dict:
    from vehicle_model.simulation.split_mu.suite import run_all_split_mu_scenarios
    res = run_all_split_mu_scenarios(output_dir=out, generate_gifs=gif)
    return _write(out, {"scenarios": {k: sorted(v) for k, v in res.items()}})


# ----------------------------------------------------------------------------- yaw-stability experiments
def _experiment(path: str):
    def run(out: Path, params: dict, gif: bool) -> dict:
        from vehicle_model.experiments.runner import run_experiment
        from vehicle_model.experiments.spec import load_experiment
        summary = run_experiment(load_experiment(path), out)
        return {"experiment": path, "verification_passed": all(c["passed"] for c in summary.get("verification", []))}
    return run


# ----------------------------------------------------------------------------- validation matrix
def _validation(method: str):
    def run(out: Path, params: dict, gif: bool) -> dict:
        from vehicle_model.logging import setup_logger
        from vehicle_model.simulation.validation.runner import ValidationMatrixRunner
        runner = ValidationMatrixRunner(config_path="configs/validation/matrix.yaml", figures_dir=out,
                                        logger=setup_logger("scenario_book"))
        evals = getattr(runner, method)()
        return _write(out, {"scenario": method, "results": [
            {"controller": e.controller_name, "passed": e.overall_passed,
             "criteria": [{"name": c.metric_name, "passed": c.passed, "value": c.value, "threshold": c.threshold} for c in e.criteria]}
            for e in evals]})
    return run


def _build() -> dict[str, ScenarioEntry]:
    from vehicle_model.simulation.four_wheel_steering.parking import PARKING_SCENARIOS

    E = ScenarioEntry
    entries = [E(n, "parking", sc.title, sc.description, _parking(n), {"dt": 0.01}, "~1.5 min with GIF, 20 s without",
                 "<name>.gif (plan view + steering panel), <name>.png, summary.json (swept extent, clearances)")
               for n, sc in PARKING_SCENARIOS.items()]
    fws = [
        ("turning_circle", "simulate_low_speed_turning_circle", "low", {"v_kph": 12.0, "front_steer_deg": 25.0, "duration_s": 7.0},
         "Constant speed, constant front steer: rear counter-steers, the circle shrinks."),
        ("urban_corner", "simulate_urban_90deg_cornering", "low", {"v_kph": 15.0, "duration_s": 5.0},
         "Right-angle turn at a junction with a curb; driver model follows the corner."),
        ("parallel_parking", "simulate_parallel_parking", "low", {"duration_s": 7.2},
         "Reverse S-curve into a bay with a fixed (time-based) steering programme; see parking/ for event-based ones."),
        ("double_lane_change", "simulate_high_speed_double_lane_change", "high", {"v_kph": 80.0, "duration_s": 3.5},
         "ISO 3888-2-like obstacle avoidance; rear steers in phase at speed."),
        ("slalom", "simulate_high_speed_slalom", "high", {"v_kph": 70.0, "duration_s": 4.0}, "Slalom through 30 m cones."),
        ("motorway_lane_change", "simulate_autobahn_lane_change", "high", {"v_kph": 130.0, "duration_s": 3.5},
         "Single lane change at motorway speed."),
        ("step_steer", "simulate_high_speed_step_steer", "high", {"v_kph": 100.0, "step_steer_deg": 3.0, "duration_s": 2.5},
         "Step steer input; yaw-rate response and sideslip."),
        ("sine_with_dwell", "simulate_sine_with_dwell", "high", {"v_kph": 80.0, "steer_amp_deg": 5.0, "duration_s": 4.5},
         "Sine with dwell modelled on FMVSS 126 (simulation, not the regulatory test)."),
        ("split_mu_braking", "simulate_split_mu_braking_4ws", "high", {"v_kph": 100.0, "mu_left": 1.0, "mu_right": 0.2, "duration_s": 3.0},
         "Braking on split friction; rear-steer yaw damping alone cannot prevent the spin (see split_mu_matrix)."),
    ]
    entries += [E(f"4ws_{n}", "four_wheel_steering", ENTRY_TITLES[fn], d, _four_ws(fn, kind), params,
                  "~1 min with GIF", "comparison.png (paths + steering panel), animation.gif, summary.json")
                for n, fn, kind, params, d in fws]
    entries += [
        E("split_mu_matrix", "split_mu", "Split-mu braking across actuator configurations",
          "Six configurations (passive, 4WS, 4WS+ABS, ESC/TV, allocation, IVMC) with the same pedal; which stay straight "
          "and why (brake vs steering yaw moment).", _split_mu_matrix,
          {"v_kph": 100.0, "mu_high": 1.0, "mu_low": 0.2, "pedal": 1.0, "duration": 3.0}, "~30 s",
          "matrix.md, summary.json"),
        E("split_mu_suite", "split_mu", "Split-mu scenario suite",
          "Braking, launch, friction transitions and evasive manoeuvres on split friction for several controllers.",
          _split_mu_suite, {}, "~7 min with GIFs", "dashboards, GIFs, reports, summary.json"),
    ]
    import yaml
    for path in sorted(Path("configs/experiments").glob("*.yaml")):
        kind = (yaml.safe_load(path.read_text(encoding="utf-8")) or {}).get("type", "controller_comparison")
        if kind == "solver_benchmark":
            entries.append(E(f"exp_{path.stem}", "solver", f"Solver benchmark {path.stem}",
                             f"Runs {path} (edit the YAML to change the NMPC variants, manoeuvre, road).",
                             _experiment(str(path)), {}, "~2 min", "report.md, kpis.csv, results.json, histories, plot"))
            continue
        if kind == "equilibrium_analysis":
            entries.append(E(f"exp_{path.stem}", "nonlinear_stability", f"Equilibrium analysis {path.stem}",
                             f"Runs {path} (edit the YAML to change sweep, series, vehicle, tyres, road).",
                             _experiment(str(path)), {}, "~1-3 min",
                             "report.md, equilibria.csv, results.json, equilibria and eigenvalue plots"))
            continue
        if kind == "robust_benchmark":
            entries.append(E(f"exp_{path.stem}", "robust_control", f"Robust benchmark {path.stem}",
                             f"Runs {path} (edit the YAML to change controllers, feedback modes, uncertainty cases).",
                             _experiment(str(path)), {}, "~3-5 min (parallel)",
                             "report.md, kpis.csv, results.json, KPI matrices, yaw-rate histories"))
            continue
        if kind == "critical_scenario_search":
            entries.append(E(f"exp_{path.stem}", "critical_scenarios", f"Critical scenario search {path.stem}",
                             f"Runs {path} (edit the YAML to change parameter space, constraints, criterion, search).",
                             _experiment(str(path)), {}, "~5-10 min (parallel search, serial verification)",
                             "report.md, evaluations.csv, results.json, search.png, critical/<id>/ (replay YAML, "
                             "metadata, CSV, PNG, GIF, report)"))
            continue
        if kind == "design_optimization":
            entries.append(E(f"exp_{path.stem}", "design", f"Design optimisation {path.stem}",
                             f"Runs {path} (edit the YAML to change design parameters, objectives, scenario sets, search).",
                             _experiment(str(path)), {}, "~5-10 min (parallel)",
                             "report.md, evaluations.csv, pareto_designs.csv, design_<label>.yaml, Pareto fronts, "
                             "distributions, sensitivities, results.json"))
            continue
        if kind == "rollover_benchmark":
            entries.append(E(f"exp_{path.stem}", "roll_stability", f"Roll stability benchmark {path.stem}",
                             f"Runs {path} (edit the YAML to change variants, manoeuvres, friction, controllers).",
                             _experiment(str(path)), {}, "~3-5 min (parallel, 14-DOF at 1 ms)",
                             "report.md, kpis.csv, results.json, histories, rollover_timeseries.png"))
            continue
        if kind == "ride_benchmark":
            entries.append(E(f"exp_{path.stem}", "ride_comfort", f"Ride and pitch benchmark {path.stem}",
                             f"Runs {path} (edit the YAML to change roads, manoeuvres, damping modes, preview, active pitch).",
                             _experiment(str(path)), {}, "~1-2 min (parallel, 14-DOF at 1 ms)",
                             "report.md, kpis.csv, results.json, histories"))
            continue
        if kind == "hill_descent_benchmark":
            entries.append(E(f"exp_{path.stem}", "hill_descent", f"Hill descent benchmark {path.stem}",
                             f"Runs {path} (edit the YAML to change grades, surfaces, named cases, controllers).",
                             _experiment(str(path)), {}, "~10 min (parallel, 7-DOF with road grade)",
                             "report.md, kpis.csv, results.json, histories"))
            continue
        if kind == "fault_tolerance_benchmark":
            entries.append(E(f"exp_{path.stem}", "fault_tolerance", f"Fault tolerance benchmark {path.stem}",
                             f"Runs {path} (edit the YAML to change manoeuvres, fault sets, controllers).",
                             _experiment(str(path)), {}, "~5-10 min (parallel, 7-DOF)",
                             "report.md, kpis.csv, results.json, histories"))
            continue
        if kind == "stability_map":
            entries.append(E(f"exp_{path.stem}", "nonlinear_stability", f"Stability map {path.stem}",
                             f"Runs {path} (edit the YAML to change grid, operating point, controllers).",
                             _experiment(str(path)), {}, "~10-20 min (parallel)",
                             "report.md, map.csv, results.json, maps, portraits, limit cases"))
            continue
        entries.append(E(f"exp_{path.stem}", "yaw_stability", f"YAML experiment {path.stem}",
                         f"Runs {path} (edit the YAML to change vehicle, manoeuvre, controllers, sweep).",
                         _experiment(str(path)), {}, "~1 min", "report.md, kpis.csv, results.json, histories, plots"))
    for method, title in (("run_iso_3888_dlc", "ISO 3888-2 double lane change"),
                          ("run_step_steer_iso_7401", "ISO 7401 step steer"),
                          ("run_iso_4138_circle", "ISO 4138 steady-state circle"),
                          ("run_low_speed_slalom_4ws", "Low-speed slalom with 4WS"),
                          ("run_high_speed_lane_change", "High-speed lane change"),
                          ("run_fishhook_ncap", "Fishhook (NCAP-like)"),
                          ("run_mu_split_braking", "Split-mu braking"),
                          ("run_braking_in_a_turn", "Braking in a turn"),
                          ("run_sine_with_dwell", "Sine with dwell"),
                          ("run_ride_comfort_bump", "Ride comfort over a bump")):
        entries.append(E("val_" + method.removeprefix("run_"), "validation", f"Validation matrix: {title}",
                         "Scenario of configs/validation/matrix.yaml with its controllers and pass/fail thresholds.",
                         _validation(method), {}, "~30 s", "figures, summary.json (criteria per controller)"))
    return {e.name: e for e in entries}


_CATALOG: dict[str, ScenarioEntry] | None = None


def catalog() -> dict[str, ScenarioEntry]:
    global _CATALOG
    if _CATALOG is None:
        _CATALOG = _build()
    return _CATALOG


def run_entry(name: str, out_dir: str | Path | None = None, overrides: dict | None = None, gif: bool = True) -> dict:
    entry = catalog()[name]
    params = entry.resolve(overrides or {})
    out = Path(out_dir or Path("output/scenario_book") / name)
    out.mkdir(parents=True, exist_ok=True)
    return entry.run(out, params, gif)


def book_markdown() -> str:
    """The scenario book as Markdown (docs/SCENARIO_BOOK.md is generated from this)."""
    L = ["# Scenario book", "",
         "Generated from `src/vehicle_model/scenario_book/catalog.py` "
         "(`python -m vehicle_model.scenario_book book --write docs/SCENARIO_BOOK.md`). Do not edit by hand.", "",
         "```bash",
         "python -m vehicle_model.scenario_book list [--category parking]",
         "python -m vehicle_model.scenario_book info u_turn",
         "python -m vehicle_model.scenario_book run u_turn [--set dt=0.005] [--no-gif] [--out DIR]",
         "python -m vehicle_model.scenario_book run --category parking",
         "```", "",
         "Outputs go to `output/scenario_book/<name>/` unless `--out` is given. All vehicles and parameters are synthetic; "
         "results are simulation results, not measurements.", ""]
    for cat in CATEGORIES:
        rows = [e for e in catalog().values() if e.category == cat]
        L += [f"## {cat}", "", "| name | scenario | parameters (defaults) | runtime | outputs |", "|---|---|---|---|---|"]
        for e in rows:
            ps = ", ".join(f"`{k}={v}`" for k, v in e.params.items()) or "–"
            L.append(f"| `{e.name}` | **{e.title}** – {e.description} | {ps} | {e.runtime} | {e.outputs} |")
        L.append("")
    return "\n".join(L)


__all__ = ("CATEGORIES", "ScenarioEntry", "book_markdown", "catalog", "run_entry")
