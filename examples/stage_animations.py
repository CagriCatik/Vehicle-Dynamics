"""One comparison GIF for every experiment, in the platform's one renderer (visualization/run_animation.py).

Experiments with recorded histories are drawn from them. Analyses without a vehicle run (stability maps, equilibria,
robust benchmark, design optimisation) get a representative closed-loop simulation of their own operating point:

    stability maps     recovery from a start state, with the map's controllers (stability_region.simulate)
    equilibria         settling onto the stationary state from straight driving
    robust benchmark   the same runs as the benchmark, worst uncertainty case
    design             the two extreme Pareto designs on the design manoeuvre
    critical search    critical and nominal run of each confirmed case (recorded time series)

    python examples/stage_animations.py                # all experiments still without a GIF
    python examples/stage_animations.py --force        # all experiments
    python examples/stage_animations.py hill           # experiments and demos whose name contains "hill"
Out: <experiment output directory>/comparison.gif (critical cases: critical/<case>/scenario.gif); demo scripts without
their own animation get one representative run comparison in their output folder.
"""

from __future__ import annotations

import csv
import math
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np
import yaml

from vehicle_model.visualization.run_animation import animate_runs, load_history

D = 180.0 / math.pi
SPEED, YAW, BETA = ("speed", "speed", 3.6, "km/h"), ("yaw rate", "r", D, "deg/s"), ("sideslip", "beta", D, "deg")
STEER = ("front road-wheel angle", "steer_f", D, "deg")
MAX_RUNS = 4


def _traces(kind: str) -> tuple:
    return (SPEED, ("wheel slip", "max_abs_slip", 1.0, "-"), YAW) if "braking" in kind or kind in ("coast", "launch") else (YAW, BETA, STEER)


def _pick(files: list) -> list:
    """At most MAX_RUNS runs, spread over the list so that first and last variant stay in."""
    if len(files) <= MAX_RUNS:
        return files
    return [files[int(round(i))] for i in np.linspace(0, len(files) - 1, MAX_RUNS)]


def _label(stem: str, point: str) -> str:
    return stem[len(point) + 2:].replace("__", " ").replace("_", " ").strip()


def controller_comparison(cfg: dict, out: Path) -> None:
    files = sorted((out / "histories").glob("*.csv"))
    point = files[0].stem.split("__")[0]
    runs = [(_label(f.stem, point), load_history(f)) for f in _pick([f for f in files if f.stem.split("__")[0] == point])]
    road = cfg["road"] if "sweep" not in cfg or not str(cfg["sweep"]["parameter"]).startswith("road") else None
    animate_runs(runs, out / "comparison.gif", f"{cfg['name'].replace('_', ' ')} ({point})", _traces(cfg["manoeuvre"]["type"]), road=road)


def solver_benchmark(cfg: dict, out: Path) -> None:
    files = _pick(sorted((out / "histories").glob("*.csv")))
    runs = [(f.stem.replace("_", " "), load_history(f)) for f in files]
    animate_runs(runs, out / "comparison.gif", "Dynamic NMPC: solver settings in a closed-loop lane change",
                 (("longitudinal speed", "vx", 3.6, "km/h"), ("solve time", "solve_ms", 1.0, "ms"), STEER))


def _path_from_states(h: dict) -> dict:
    """Plan-view path integrated from the recorded states (the recovery runs record no position)."""
    t = np.asarray(h["t"], float)
    dt = float(t[1] - t[0])
    vx, r, beta = np.asarray(h["vx"], float), np.asarray(h["r"], float), np.asarray(h["beta"], float)
    vy = vx * np.tan(beta)
    psi = np.cumsum(r) * dt
    X, Y = np.cumsum(vx * np.cos(psi) - vy * np.sin(psi)) * dt, np.cumsum(vx * np.sin(psi) + vy * np.cos(psi)) * dt
    return {**h, "vy": vy, "psi": psi, "X": X, "Y": Y, "speed": np.hypot(vx, vy)}


def stability_map(cfg: dict, out: Path) -> None:
    from vehicle_model.experiments.spec import load_experiment
    from vehicle_model.simulation.stability_region import reference_state, simulate
    spec = load_experiment(cfg["_path"])
    runs = []
    for c in cfg["controllers"][:3]:
        problem = spec.problem(c) if hasattr(spec, "problem") else None
        ref = reference_state(problem)
        res = simulate(problem, math.radians(12.0), math.radians(-25.0), (ref["beta"], ref["r"]), record=True, stop_early=False)
        runs.append((f"{c}: {res.outcome}", _path_from_states(res.history)))
    animate_runs(runs, out / "comparison.gif", "Recovery from beta 12 deg, r -25 deg/s (80 km/h, mu 0.6, 1 deg steer held)", (BETA, YAW, STEER))


class _HeldInputs:
    """Constant yaw moment and rear steer ratio: the sweep variables of the TV/4WS equilibrium analysis."""
    name, information = "held inputs", "open loop"

    def __init__(self, yaw_moment: float, rear_ratio: float):
        self.yaw_moment, self.rear_ratio = yaw_moment, rear_ratio

    def reset(self) -> None:
        pass

    def request(self, state, ctx):
        from vehicle_model.controllers.motion_control import MotionRequest
        return MotionRequest(yaw_moment=self.yaw_moment, rear_steer=self.rear_ratio * ctx.steer_f)


def _passive_run(v_kph: float, steer_deg: float, mu: float, duration: float = 6.0, case: dict | None = None, held=None):
    import dataclasses

    from vehicle_model.roads.surface import UniformRoad
    from vehicle_model.simulation.controller_benchmark import run_scenario, step_steer_scenario
    from vehicle_model.simulation.uncertainty import UncertaintyCase, plant_factory
    from vehicle_model.vehicle_models import SevenDOFModel
    sc = dataclasses.replace(step_steer_scenario(v_kph / 3.6, math.radians(steer_deg), duration), road=lambda: UniformRoad(mu=mu))
    nominal = SevenDOFModel.from_yaml("configs/vehicles/seven_dof_passenger.yaml")
    controller = None
    if held is not None:
        from vehicle_model.controllers.motion_control import MotionController
        controller = lambda p: MotionController(p, _HeldInputs(*held))
    return run_scenario(plant_factory(nominal, UncertaintyCase.from_dict(case or {"name": "nominal"})), controller, sc, 0.005).history


def equilibrium(cfg: dict, out: Path) -> None:
    name = cfg["name"]
    if name.endswith("steer_sweep"):
        runs = [(f"{s:g} deg, mu {m:g}", _passive_run(90.0, s, m)) for s, m in ((1.0, 1.0), (4.0, 1.0), (4.0, 0.6))]
        title = "Settling onto the steady cornering state, 90 km/h: main branch towards the limit"
    elif name.endswith("speed_stiffness"):
        runs = [(f"rear stiffness x{k:g}", _passive_run(180.0, 0.3, 1.0, 5.0, {"name": "c", "cornering_scale_rear": k})) for k in (1.0, 0.6, 0.45)]
        title = "Straight running at 180 km/h after 0.3 deg steer: lower rear stiffness: large yaw oscillation instead of settling"
    else:
        runs = [(f"M_z {m:g} N m, rear ratio {k:g}", _passive_run(90.0, 1.0, 0.6, 6.0, held=(m, k)))
                for m, k in ((0.0, 0.0), (750.0, 0.0), (-1000.0, 0.0), (0.0, 0.2))]
        title = "Steady cornering at 90 km/h, mu 0.6, 1 deg, with held yaw moment and rear steer ratio"
    animate_runs(runs, out / "comparison.gif", title, (YAW, BETA, SPEED))


def robust_benchmark(cfg: dict, out: Path) -> None:
    from vehicle_model.experiments.robust_benchmark import _nominal, build_controller, scenario
    from vehicle_model.experiments.spec import load_experiment
    from vehicle_model.simulation.controller_benchmark import run_scenario
    from vehicle_model.simulation.uncertainty import UncertaintyCase, plant_factory
    spec = load_experiment(cfg["_path"])
    case_dict = next(u for u in spec.uncertainty if u["name"] == "actuator_delay")
    case = UncertaintyCase.from_dict(case_dict)
    runs = []
    for c in spec.controllers:
        if c["name"] in ("baseline PI", "baseline GS-LQR", "Robust LQR", "Tightening MPC"):
            res = run_scenario(plant_factory(_nominal(spec), case), build_controller(spec, c, "ideal", case), scenario(spec), spec.dt)
            runs.append((c["name"], res.history))
    animate_runs(runs, out / "comparison.gif", "Sine with dwell 80 km/h, 6 deg, with actuator delay", (BETA, YAW, STEER))


def design(cfg: dict, out: Path) -> None:
    from vehicle_model.experiments.design_optimization import _manoeuvre, _plant_and_controller
    from vehicle_model.experiments.spec import load_experiment
    from vehicle_model.simulation.controller_benchmark import run_scenario
    spec = load_experiment(cfg["_path"])
    rows = list(csv.DictReader(open(out / "pareto_designs.csv", encoding="utf-8")))
    names = [d["name"] for d in spec.design] if isinstance(spec.design[0], dict) else [d.name for d in spec.design]
    picks = (("best yaw tracking", min(rows, key=lambda r: float(r["handling_yaw_error"]))),
             ("best comfort", min(rows, key=lambda r: float(r["comfort_body_accel"]))))
    runs = []
    for lab, r in picks:
        values = {k: float(r[k]) for k in names if k in r}
        plant, factory = _plant_and_controller(spec, values, {"name": "low_mu", "mu": 0.6})
        sc = _manoeuvre({"type": "sine_with_dwell", "speed_kph": 80, "amplitude_deg": 5, "mu": 0.6})
        runs.append((lab, run_scenario(plant, factory, sc, spec.dt).history))
    animate_runs(runs, out / "comparison.gif", "Extreme Pareto designs: sine with dwell 80 km/h, 5 deg, mu 0.6", (BETA, YAW, STEER))


def critical_search(cfg: dict, out: Path) -> None:
    for d in sorted((out / "critical").glob("*")):
        rows = load_history(d / "timeseries.csv")
        cols = {k[:-len("_nominal")] for k in rows if k.endswith("_nominal")}
        nominal = {k: rows[f"{k}_nominal"] for k in cols} if cols else None
        runs = [("critical", rows)] + ([("nominal", nominal)] if nominal else [])
        animate_runs(runs, d / "scenario.gif", f"Critical scenario {d.name.replace('_', ' ')}", (BETA, YAW, SPEED))


# ----------------------------------------------------------------------------- demo scripts without an animation
VEHICLE = "configs/vehicles/seven_dof_passenger.yaml"
REFERENCE = "configs/vehicles/reference_passenger_car.yaml"
DRY = {"type": "uniform", "mu": 1.0}
SPLIT = {"type": "split_mu", "mu_left": 1.0, "mu_right": 0.2}
SWD = {"type": "sine_with_dwell", "speed_kph": 80, "amplitude_deg": 6}


def dsl_run(road: dict, manoeuvre: dict, controller: dict, tyres: dict | None = None, kc: str | None = None, dt: float = 0.005,
            vehicle: str = VEHICLE) -> dict:
    """One closed-loop run through the experiment DSL, with the same builders as the YAML experiments."""
    from vehicle_model.experiments.runner import controller_factories, make_scenario, plant_factory
    from vehicle_model.experiments.spec import parse_controller_comparison
    from vehicle_model.simulation.controller_benchmark import run_scenario
    data = {"name": "demo", "vehicle": vehicle, "road": road, "manoeuvre": manoeuvre, "controllers": [controller], "dt": dt,
            "verification": False, "outputs": {"formats": []}, "tyres": tyres or {}}
    if kc:
        data["kc"] = kc
    spec = parse_controller_comparison(data)
    factory = controller_factories(spec)[spec.controllers[0].label]
    return run_scenario(plant_factory(spec), factory, make_scenario(spec), spec.dt).history


def plan(path: str, title: str, runs: list, traces: tuple = (YAW, BETA, STEER), road: dict | None = None) -> None:
    animate_runs(runs, Path(path), title, traces, road=road)


def _ride(path: str, title: str, road: dict, manoeuvre: dict, modes: tuple) -> None:
    from vehicle_model.simulation.ride import run as run_ride
    from vehicle_model.visualization.run_animation import animate_side_view
    runs = [(m, run_ride("configs/suspension/fourteen_dof.yaml", road, manoeuvre, {"mode": m}, duration=4.0)["history"]) for m in modes]
    animate_side_view(runs, Path(path), title, [("body vertical acceleration", "ddzs", 1.0, "m/s^2"), ("pitch", "theta", D, "deg"),
                                                ("tyre load FL", "Fz_FL", 1e-3, "kN")])


def _drift_csv(path: str) -> dict:
    h = load_history(path)
    return {**h, "beta": np.radians(np.asarray(h["beta_deg"], float)), "speed": np.hypot(np.asarray(h["vx"], float), np.asarray(h["vy"], float))}


def _drift_gif(path: str, title: str, runs: tuple) -> None:
    animate_runs([(lab, _drift_csv(f"output/drift/{f}")) for lab, f in runs], Path(path), title, (BETA, YAW, STEER))


def _tyres(name: str) -> dict:
    return {"front": name, "rear": name}


def demo_recipes() -> dict:
    braking = {"type": "split_mu_braking", "speed_kph": 90}
    passive, pi, lqr = {"law": "passive"}, {"law": "pi"}, {"law": "gs_lqr", "inputs": "4ws_tv"}
    step = {"type": "step_steer", "speed_kph": 80, "steer_deg": 2.0, "duration_s": 3.0}
    lane = {"type": "lane_change", "speed_kph": 80, "amplitude_deg": 3.0, "frequency_hz": 0.5, "duration_s": 5.0}
    limit = {"type": "step_steer", "speed_kph": 80, "steer_deg": 5.0, "duration_s": 4.0}

    def over(items, road, man, **kw):
        return [(lab, dsl_run(road, man, c, **kw)) for lab, c in items]
    return {
        "abs_tcs_esc_benchmark": lambda: plan("output/benchmark/abs_tcs_esc.gif", "ABS, TCS and yaw control: split-mu braking 90 km/h",
                                              over((("passive", passive), ("ABS + yaw PI", pi)), SPLIT, braking), (YAW, SPEED, BETA), SPLIT),
        "advanced_chassis_control_demo": lambda: plan("output/advanced_control/lift_off.gif", "Lift-off in a turn: no yaw control against yaw PI",
                                                      over((("no yaw control", {"law": "no_yaw"}), ("yaw PI", pi)), DRY,
                                                           {"type": "lift_off", "speed_kph": 80, "steer_deg": 3.0, "accel_pedal": 0.3, "lift_s": 2.0, "duration_s": 5.0})),
        "controller_benchmark": lambda: plan("output/controller_benchmark/sine_with_dwell.gif", "Controller library: sine with dwell 80 km/h, 6 deg",
                                             over((("passive", passive), ("PI", pi), ("LQR", {"law": "lqr", "inputs": "tv"}), ("GS-LQR", {"law": "gs_lqr", "inputs": "tv"})), DRY, SWD)),
        "e2e_deep_analysis_and_benchmarking": lambda: plan("output/e2e_analysis/lane_change.gif", "End-to-end comparison: lane change 80 km/h",
                                                           over((("passive", passive), ("PI", pi), ("GS-LQR 4WS+TV", lqr)), DRY, lane)),
        "estimation_and_pso_demo": lambda: plan("output/estimation/feedback_modes.gif", "Same controller with ideal, estimated and sensor-only feedback",
                                                over((("ideal", {"law": "gs_lqr"}), ("estimated", {"law": "gs_lqr", "feedback": "estimated"}),
                                                      ("sensors only", {"law": "gs_lqr", "feedback": "sensor_constrained"})), DRY, SWD)),
        "handling_analysis_reference_vehicle": lambda: plan("output/handling_analysis/step_steer_speeds.gif", "Step steer 2 deg at three speeds, passive",
                                                            [(f"{v} km/h", dsl_run(DRY, dict(step, speed_kph=v), passive)) for v in (60, 100, 140)], (YAW, BETA, SPEED)),
        "handling_limits_and_understeer_demo": lambda: plan("output/handling_limits/towards_the_limit.gif", "Steering towards the limit at 80 km/h, passive",
                                                            [(f"{a} deg", dsl_run(DRY, dict(step, steer_deg=a), passive)) for a in (2, 4, 8)], (YAW, BETA, SPEED)),
        "integrated_control_benchmark": lambda: plan("output/benchmark/integrated_lane_change.gif", "Integrated control: lane change 80 km/h",
                                                     over((("passive", passive), ("PI (TV)", pi), ("GS-LQR 4WS+TV", lqr)), DRY, lane)),
        "kc_understeer_analysis": lambda: plan("output/kc_analysis/step_steer_kc.gif", "Step steer 80 km/h with and without kinematics and compliance",
                                               [("rigid axles", dsl_run(DRY, step, passive, vehicle=REFERENCE)),
                                                ("with K&C", dsl_run(DRY, step, passive, vehicle=REFERENCE, kc="configs/suspension/kc_reference_passenger_car.yaml"))],
                                               (YAW, BETA, SPEED)),
        "monte_carlo_robustness": lambda: plan("output/monte_carlo/mu_variation.gif", "Robustness: sine with dwell with PI on three friction values",
                                               [(f"mu {m:g}", dsl_run({"type": "uniform", "mu": m}, SWD, pi)) for m in (1.0, 0.7, 0.4)]),
        "seven_dof_4ws_tv_demo": lambda: plan("output/demo_7dof/split_mu_4ws_tv.gif", "7-DOF model: split-mu braking with 4WS and torque vectoring",
                                              over((("passive", passive), ("GS-LQR 4WS+TV", lqr)), SPLIT, braking), (YAW, SPEED, BETA), SPLIT),
        "slip_and_allocation_demo": lambda: plan("output/slip_and_allocation/low_mu_braking.gif", "Full braking on mu 0.3: locked wheels against slip control",
                                                 over((("passive", passive), ("allocation + ABS", {"law": "no_yaw"})), {"type": "uniform", "mu": 0.3},
                                                      {"type": "straight_braking", "speed_kph": 80}), (SPEED, ("wheel slip", "max_abs_slip", 1.0, "-"), YAW)),
        "split_mu_matrix_demo": lambda: plan("output/split_mu_matrix/split_mu_matrix.gif", "Split-mu braking with the steering held straight",
                                             over((("passive", passive), ("no yaw control", {"law": "no_yaw"}), ("PI", pi), ("GS-LQR 4WS+TV", lqr)), SPLIT, braking), (YAW, SPEED, BETA), SPLIT),
        "tyre_models_demo": lambda: plan("output/tyres/tyre_models_step_steer.gif", "Same manoeuvre with three tyre models: step steer 5 deg, 80 km/h",
                                         [(n, dsl_run(DRY, limit, passive, tyres=_tyres(t))) for n, t in (("linear", "linear_passenger_synthetic"),
                                          ("Fiala", "fiala_passenger_synthetic"), ("Magic Formula", "magic_formula_passenger_synthetic"))], (YAW, BETA, SPEED)),
        "drift_tyre_audit": lambda: plan("output/drift/tyre_audit.gif", "Large slip: step steer 8 deg, 80 km/h, mu 0.6, with two tyre models",
                                         [(n, dsl_run({"type": "uniform", "mu": 0.6}, dict(limit, steer_deg=8.0), passive, tyres=_tyres(t)))
                                          for n, t in (("Fiala", "fiala_passenger_synthetic"), ("Magic Formula", "magic_formula_passenger_synthetic"))], (YAW, BETA, SPEED)),
        "drift_hold_demo": lambda: _drift_gif("output/drift/lqr_hold/hold_recovery.gif", "Drift hold after a 3 deg sideslip disturbance: LQR against open loop",
                                              (("LQR", "lqr_hold/state_history_left_beta_plus_3deg_lqr.csv"), ("open loop", "lqr_hold/state_history_left_beta_plus_3deg_open_loop.csv"))),
        "drift_equilibria_demo": lambda: _drift_gif("output/drift/equilibrium_sweep/held_equilibria.gif", "Computed drift equilibria held with LQR, left and right",
                                                    (("left", "lqr_hold/state_history_left_hold_lqr.csv"), ("right", "lqr_hold/state_history_right_hold_lqr.csv"))),
        "drift_initiation_demo": lambda: _drift_gif("output/drift/initiation/initiation.gif", "Drift initiation, hold and exit, left and right",
                                                    (("left", "initiation/state_history_left.csv"), ("right", "initiation/state_history_right.csv"))),
        "drift_robustness_demo": lambda: _drift_gif("output/drift/robustness/disturbances.gif", "Drift hold with LQR under four disturbances (the study itself varies the plant)",
                                                    (("sideslip +3 deg", "lqr_hold/state_history_left_beta_plus_3deg_lqr.csv"), ("speed +1 m/s", "lqr_hold/state_history_left_speed_plus_1_lqr.csv"),
                                                     ("yaw rate +0.1 rad/s", "lqr_hold/state_history_left_yaw_plus_0.1_lqr.csv"), ("inner wheel -10 %", "lqr_hold/state_history_left_inner_wheel_minus_10_lqr.csv"))),
        "suspension_dynamics_demo": lambda: _ride("output/suspension/damping_iso_c.gif", "ISO 8608 class C road, 60 km/h: passive, skyhook, groundhook",
                                                  {"iso_class": "C", "seed": 3}, {"type": "cruise", "speed_kph": 60}, ("passive", "skyhook", "groundhook")),
        "vertical_dynamics_demo": lambda: _ride("output/vertical_dynamics/bump_response.gif", "6 cm bump at 60 km/h: passive and skyhook",
                                                {"bumps": [{"x": 30, "length": 0.6, "height": 0.06}]}, {"type": "cruise", "speed_kph": 60}, ("passive", "skyhook")),
    }


HANDLERS = {"controller_comparison": controller_comparison, "solver_benchmark": solver_benchmark, "stability_map": stability_map,
            "equilibrium_analysis": equilibrium, "robust_benchmark": robust_benchmark, "design_optimization": design,
            "critical_scenario_search": critical_search}


def _demo(name: str) -> tuple:
    try:
        demo_recipes()[name]()
        return name, ""
    except Exception as ex:
        return name, f"FAILED {type(ex).__name__}: {ex}"


def main(argv=None) -> int:
    args = list(sys.argv[1:] if argv is None else argv)
    force = "--force" in args
    filters = [a for a in args if not a.startswith("-")]
    failed = []
    for path in sorted(Path("configs/experiments").glob("*.yaml")):
        cfg = yaml.safe_load(path.read_text(encoding="utf-8"))
        cfg["_path"] = str(path)
        kind = cfg.get("type", "controller_comparison")
        out = Path((cfg.get("outputs") or {}).get("directory", f"output/experiments/{cfg['name']}"))
        if filters and not any(f in path.stem for f in filters):
            continue
        if kind not in HANDLERS or not out.is_dir():
            continue
        if not force and any(out.rglob("*.gif")) and kind != "critical_scenario_search":
            continue
        try:
            HANDLERS[kind](cfg, out)
            print(f"  {path.stem}: ok")
        except Exception as ex:
            failed.append(path.stem)
            print(f"  {path.stem}: FAILED {type(ex).__name__}: {ex}")
    names = [n for n in demo_recipes() if not filters or any(f in n for f in filters)]
    with ProcessPoolExecutor(max_workers=4) as pool:
        for name, error in pool.map(_demo, names):
            print(f"  {name}: {error or 'ok'}", flush=True)
            if error:
                failed.append(name)
    print("failed:", failed)
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
