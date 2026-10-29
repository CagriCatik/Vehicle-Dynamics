"""Experiment type ``robust_benchmark``: controllers x feedback modes x uncertainty cases (Plan V3.0 M12 7.4, §12).

Every controller is designed once from the *nominal* vehicle (``vehicle``, ``tyres``) and
runs unchanged against every uncertainty case (:mod:`vehicle_model.simulation.uncertainty`):
the plant deviates, actuators are delayed, sensors corrupted. Nothing is re-tuned per
case. Feedback modes: ``ideal`` (true state and telemetry), ``estimated`` (motion state
from sensors and an observer, Mode B), ``sensor_constrained`` (no simulation telemetry,
Mode C). All runs share the scenario, the yaw reference and the plant model class
(7-DOF two-track, independent of the controllers' linear design model).

KPIs per run (Plan 7.4):

    yaw_rate_rms_error_degps   RMS of r - r_ref (common reference, true friction)
    max_sideslip_deg           max |beta|
    lateral_deviation_m        max |Y - Y_ref|, Y_ref integrated from r_ref and the actual speed
    saturation_share           steps with the yaw-moment demand at its limit or the allocation /
                               slip control saturated
    max_yaw_moment_rate_knmps  max |d M_z,allocated / dt|
    max_rear_steer_rate_degps  max |d delta_r / dt|
    max_tyre_utilisation       max over steps and wheels of sqrt(F_x^2 + F_y^2) / (mu F_z)
    computation_ms_mean/_max   wall clock of compute_control (not reproducible)
    infeasibility_rate         steps whose plan violates a hard constraint (feasibility report)
    fallback_rate              steps with solver status fallback or supervisor mode fallback
    degraded_share             steps with status degraded or supervisor mode degraded
    yaw_moment_effort_knms     integral of |M_z,allocated|
    rear_steer_effort_degs     integral of |delta_r|
    loss_of_control            1 if |beta| >= 30 deg was reached

Baselines: controllers marked ``baseline: true`` are the fixed V2.0 laws (PI, GS-LQR as
of M6, unchanged since); the report gives every controller's KPIs relative to the
first baseline per feedback mode and case, including the cases where an advanced
controller does worse.

Example (``configs/experiments/robust_benchmark_sine_with_dwell.yaml``)::

    type: robust_benchmark
    name: robust_benchmark_sine_with_dwell
    vehicle: configs/vehicles/seven_dof_passenger.yaml
    road: {type: uniform, mu: 1.0}
    manoeuvre: {type: sine_with_dwell, speed_kph: 80, amplitude_deg: 6}
    controllers:
      - {name: V2.0 PI, law: pi, baseline: true}
      - {name: Robust LQR, law: robust_lqr, inputs: 4ws_tv, alpha: 3}
    feedback: [ideal, estimated, sensor_constrained]
    uncertainty:
      - {name: nominal}
      - {name: heavy, mass_factor: 1.2, Iz_factor: 1.25}
"""

from __future__ import annotations

import csv
import math
import os
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from .spec import _keys, _require
from .types import provenance, slug, write_json

KIND = "robust_benchmark"
LAWS = ("passive", "pi", "lqr", "gs_lqr", "ltv_mpc", "robust_lqr", "gs_lqr_dob", "tightening_mpc", "adaptive_lqr")
FEEDBACK = ("ideal", "estimated", "sensor_constrained")
_TOP = {"type", "name", "description", "vehicle", "tyres", "road", "manoeuvre", "controllers", "feedback", "uncertainty",
        "dt", "seed", "workers", "outputs", "verification"}
_CTRL = {"name", "law", "inputs", "baseline", "alpha", "bandwidth", "w_max_vy", "w_max_r", "supervisor"}
FORMATS = ("csv", "json", "png", "md")
KPIS = ("yaw_rate_rms_error_degps", "max_sideslip_deg", "lateral_deviation_m", "saturation_share",
        "max_yaw_moment_rate_knmps", "max_rear_steer_rate_degps", "max_tyre_utilisation", "computation_ms_mean",
        "computation_ms_max", "infeasibility_rate", "fallback_rate", "degraded_share", "yaw_moment_effort_knms",
        "rear_steer_effort_degs", "loss_of_control")
_LOWER_IS_BETTER = ("yaw_rate_rms_error_degps", "max_sideslip_deg", "lateral_deviation_m")
LOSS_BETA = math.radians(30.0)


@dataclass(frozen=True)
class RobustBenchmarkSpec:
    name: str
    vehicle: str
    road: dict
    manoeuvre: dict
    controllers: tuple            # dicts, validated
    feedback: tuple
    uncertainty: tuple            # dicts (UncertaintyCase fields + name)
    tyres: dict = field(default_factory=dict)
    dt: float = 0.005
    seed: int = 0
    workers: int | None = None
    outputs: dict = field(default_factory=dict)
    verification: bool = True
    description: str = ""
    source: str = ""
    kind = KIND

    def to_dict(self) -> dict:
        return {"type": KIND, "name": self.name, "description": self.description, "vehicle": self.vehicle,
                "tyres": dict(self.tyres), "road": dict(self.road), "manoeuvre": dict(self.manoeuvre),
                "controllers": [dict(c) for c in self.controllers], "feedback": list(self.feedback),
                "uncertainty": [dict(u) for u in self.uncertainty], "dt": self.dt, "seed": self.seed,
                "workers": self.workers, "outputs": dict(self.outputs), "verification": self.verification}


def parse(data: dict, source: str = "<dict>") -> RobustBenchmarkSpec:
    from vehicle_model.simulation.uncertainty import UncertaintyCase

    _require(data.get("type") == KIND, f"{source}: type must be {KIND!r}")
    _keys(data, _TOP, source, {"type", "name", "vehicle", "road", "manoeuvre", "controllers", "uncertainty"})
    _require(Path(data["vehicle"]).is_file(), f"{source}: vehicle file not found: {data['vehicle']}")
    tyres = dict(data.get("tyres") or {})
    _keys(tyres, {"front", "rear"}, f"{source}: tyres")
    road = dict(data["road"])
    _keys(road, {"type", "mu"}, f"{source}: road", {"type", "mu"})
    _require(road["type"] == "uniform" and 0.05 <= float(road["mu"]) <= 2.0,
             f"{source}: road must be uniform with mu in [0.05, 2]")
    man = dict(data["manoeuvre"])
    _require(man.get("type") in ("step_steer", "sine_with_dwell"), f"{source}: manoeuvre.type must be step_steer or sine_with_dwell")
    allowed = {"type", "speed_kph", "steer_deg", "duration_s"} if man["type"] == "step_steer" else {"type", "speed_kph", "amplitude_deg"}
    _keys(man, allowed, f"{source}: manoeuvre", {"type", "speed_kph"})
    _require(20.0 <= float(man["speed_kph"]) <= 200.0, f"{source}: manoeuvre.speed_kph must lie in [20, 200]")
    ctrls = data["controllers"]
    _require(isinstance(ctrls, list) and ctrls, f"{source}: controllers must be a non-empty list")
    out_c = []
    for i, c in enumerate(ctrls):
        where = f"{source}: controllers[{i}]"
        _require(isinstance(c, dict), f"{where} must be a mapping")
        _keys(c, _CTRL, where, {"name", "law"})
        _require(c["law"] in LAWS, f"{where}.law must be one of {LAWS}")
        _require(c.get("inputs", "tv") in ("tv", "4ws_tv"), f"{where}.inputs must be tv or 4ws_tv")
        _require(not (c["law"] == "pi" and c.get("inputs", "tv") != "tv"), f"{where}: the PI law has yaw-moment input only")
        sup = c.get("supervisor")
        if sup is not None:
            _require(isinstance(sup, dict), f"{where}.supervisor must be a mapping")
            _keys(sup, {"fallback"}, f"{where}.supervisor", {"fallback"})
            _require(sup["fallback"] in ("pi", "passive"), f"{where}.supervisor.fallback must be pi or passive")
            _require(c["law"] != "passive", f"{where}: the passive controller needs no supervisor")
        for k in ("alpha", "bandwidth", "w_max_vy", "w_max_r"):
            if k in c:
                _require(isinstance(c[k], (int, float)) and not isinstance(c[k], bool) and c[k] >= 0, f"{where}.{k} must be >= 0")
        out_c.append(dict(c))
    names = [c["name"] for c in out_c]
    _require(len(set(names)) == len(names), f"{source}: controller names must be unique")
    fb = tuple(data.get("feedback", ["ideal"]))
    _require(fb and set(fb) <= set(FEEDBACK) and len(set(fb)) == len(fb), f"{source}: feedback must be a list of {FEEDBACK}")
    unc = data["uncertainty"]
    _require(isinstance(unc, list) and unc, f"{source}: uncertainty must be a non-empty list of cases")
    cases = []
    for i, u in enumerate(unc):
        _require(isinstance(u, dict) and isinstance(u.get("name"), str), f"{source}: uncertainty[{i}] needs a name")
        try:
            UncertaintyCase.from_dict(u)
        except (TypeError, ValueError) as ex:
            raise ValueError(f"{source}: uncertainty[{i}]: {ex}") from None
        cases.append(dict(u))
    _require(len({u["name"] for u in cases}) == len(cases), f"{source}: uncertainty case names must be unique")
    dt = float(data.get("dt", 0.005))
    _require(0.001 <= dt <= 0.02, f"{source}: dt must lie in [0.001, 0.02]")
    workers = data.get("workers")
    _require(workers is None or (isinstance(workers, int) and not isinstance(workers, bool) and workers >= 1),
             f"{source}: workers must be a positive integer")
    outputs = dict(data.get("outputs") or {})
    _keys(outputs, {"directory", "formats"}, f"{source}: outputs")
    outputs.setdefault("formats", list(FORMATS))
    _require(set(outputs["formats"]) <= set(FORMATS), f"{source}: outputs.formats must be a subset of {FORMATS}")
    outputs.setdefault("directory", f"output/experiments/{data['name']}")
    return RobustBenchmarkSpec(str(data["name"]), str(data["vehicle"]), road, man, tuple(out_c), fb, tuple(cases), tyres,
                               dt, int(data.get("seed", 0)), workers, outputs, bool(data.get("verification", True)),
                               str(data.get("description", "")), source)


def describe(spec: RobustBenchmarkSpec) -> str:
    return (f"{spec.manoeuvre['type']}: {len(spec.controllers)} controller(s) x {len(spec.feedback)} feedback mode(s) x "
            f"{len(spec.uncertainty)} uncertainty case(s)")


# ----------------------------------------------------------------------------- building blocks
def _nominal(spec):
    from vehicle_model.roads.surface import UniformRoad
    from vehicle_model.vehicle_models import SevenDOFModel
    return SevenDOFModel.from_yaml(spec.vehicle, tyre_name=spec.tyres.get("front"), road=UniformRoad(mu=float(spec.road["mu"])),
                                   tyre_rear_name=spec.tyres.get("rear"))


def _law(c: dict, design):
    from vehicle_model.controllers.robust_laws import (AdaptiveLQRYawLaw, DisturbanceObserverLaw, RobustLQRYawLaw,
                                                       TighteningMPCYawLaw, TighteningSettings)
    from vehicle_model.controllers.yaw_laws import INPUTS_4WS_TV, INPUTS_TV, LQRYawLaw, LTVMPCYawLaw, YawRatePI
    from vehicle_model.controllers.robust_laws import DEFAULT_SPEEDS
    inputs = INPUTS_4WS_TV if c.get("inputs") == "4ws_tv" else INPUTS_TV
    law = c["law"]
    if law == "pi":
        return YawRatePI()
    if law == "lqr":
        return LQRYawLaw(design, inputs, design_speed=20.0)
    if law == "gs_lqr":
        return LQRYawLaw(design, inputs, design_speed=None, schedule_speeds=DEFAULT_SPEEDS)
    if law == "ltv_mpc":
        return LTVMPCYawLaw(design, inputs)
    if law == "robust_lqr":
        return RobustLQRYawLaw(design, inputs, alpha=float(c.get("alpha", 3.0)))
    if law == "gs_lqr_dob":
        return DisturbanceObserverLaw(LQRYawLaw(design, inputs, design_speed=None, schedule_speeds=DEFAULT_SPEEDS), design,
                                      bandwidth=float(c.get("bandwidth", 12.0)))
    if law == "tightening_mpc":
        return TighteningMPCYawLaw(design, inputs, TighteningSettings(w_max_vy=float(c.get("w_max_vy", 0.3)),
                                                                      w_max_r=float(c.get("w_max_r", 0.3))))
    return AdaptiveLQRYawLaw(design, inputs)


def build_controller(spec, c: dict, feedback: str, case):
    """Controller of entry ``c`` for the nominal vehicle, with feedback mode and the case's delays (factory)."""
    from vehicle_model.controllers.estimated_feedback import EstimatedStateFeedback
    from vehicle_model.controllers.framework import PassiveBaselineController
    from vehicle_model.controllers.motion_control import MotionController
    from vehicle_model.controllers.supervisor import SupervisoryController
    from vehicle_model.controllers.yaw_laws import YawRatePI
    from vehicle_model.estimation.blended_sideslip import BlendedSideslipObserver
    from vehicle_model.handling_analysis import linear_params_from_seven_dof
    from vehicle_model.simulation.uncertainty import sensor_config, wrap_controller

    nom = _nominal(spec)
    p = nom.params                     # nominal: the controller never sees the plant's parameters
    design = linear_params_from_seven_dof(p, nom.tyre_front, nom.tyre_rear)

    def factory(_plant_params):
        if c["law"] == "passive":
            ctrl = PassiveBaselineController(p)
        else:
            ctrl = MotionController(p, _law(c, design))
            sup = c.get("supervisor")
            if sup:
                fb_ctrl = MotionController(p, YawRatePI()) if sup["fallback"] == "pi" else PassiveBaselineController(p)
                ctrl = SupervisoryController(ctrl, fb_ctrl, p)
            if feedback != "ideal":
                ctrl = EstimatedStateFeedback(ctrl, BlendedSideslipObserver(p), sensor_config(case), seed=spec.seed,
                                              mode="C" if feedback == "sensor_constrained" else "B", params=p)
        return wrap_controller(ctrl, case)
    return factory


def scenario(spec):
    from vehicle_model.simulation.controller_benchmark import sine_with_dwell_scenario, step_steer_scenario
    m, mu = spec.manoeuvre, float(spec.road["mu"])
    v = float(m["speed_kph"]) / 3.6
    if m["type"] == "step_steer":
        import dataclasses
        from vehicle_model.roads.surface import UniformRoad
        sc = step_steer_scenario(v, math.radians(float(m.get("steer_deg", 2.0))), float(m.get("duration_s", 3.0)))
        return dataclasses.replace(sc, road=lambda: UniformRoad(mu=mu))
    return sine_with_dwell_scenario(v, math.radians(float(m.get("amplitude_deg", 6.0))), mu)


# ----------------------------------------------------------------------------- KPIs
def kpis(h: dict, dt: float, max_yaw_moment: float = 4000.0) -> dict:
    t = np.asarray(h["t"], float)
    r, r_ref = np.asarray(h["r"], float), np.asarray(h["r_ref"], float)
    beta = np.asarray(h["beta"], float)
    psi_ref = np.cumsum(r_ref) * dt
    y_ref = np.cumsum(np.asarray(h["speed"], float) * np.sin(psi_ref)) * dt
    mz_d, mz_a = np.asarray(h["Mz_demand"], float), np.asarray(h["Mz_allocated"], float)
    status = np.asarray(h["status"], dtype=object)
    mode = np.asarray(h["supervisor_mode"], dtype=object)
    feas = [f for f in h["feasibility"]]
    solver = [s for s in h["solver"]]
    sat = (np.abs(mz_d) >= 0.99 * max_yaw_moment) | np.isin(status, ("saturated", "infeasible"))
    solver_status = np.array([(s or {}).get("status") for s in solver], dtype=object)
    infeasible = np.array([f is not None and not f.get("feasible", True) for f in feas])
    fallback = (solver_status == "fallback") | (mode == "fallback") | (status == "fallback")
    degraded = (solver_status == "degraded") | (mode == "degraded") | (status == "degraded")
    steer_r = np.asarray(h["steer_r"], float)
    comp = np.asarray(h["computation_ms"], float)
    return {
        "yaw_rate_rms_error_degps": float(np.degrees(np.sqrt(np.mean((r - r_ref) ** 2)))),
        "max_sideslip_deg": float(np.degrees(np.max(np.abs(beta)))),
        "lateral_deviation_m": float(np.max(np.abs(np.asarray(h["Y"], float) - y_ref))),
        "saturation_share": float(np.mean(sat)),
        "max_yaw_moment_rate_knmps": float(np.max(np.abs(np.diff(mz_a))) / dt / 1e3) if t.size > 1 else 0.0,
        "max_rear_steer_rate_degps": float(np.degrees(np.max(np.abs(np.diff(steer_r))) / dt)) if t.size > 1 else 0.0,
        "max_tyre_utilisation": float(np.nanmax(np.asarray(h["tyre_utilisation"], float))),
        "computation_ms_mean": float(np.mean(comp)), "computation_ms_max": float(np.max(comp)),
        "infeasibility_rate": float(np.mean(infeasible)), "fallback_rate": float(np.mean(fallback)),
        "degraded_share": float(np.mean(degraded)),
        "yaw_moment_effort_knms": float(np.sum(np.abs(mz_a)) * dt / 1e3),
        "rear_steer_effort_degs": float(np.degrees(np.sum(np.abs(steer_r)) * dt)),
        "loss_of_control": float(np.max(np.abs(beta)) >= LOSS_BETA),
        "supervisor_modes": {str(m): int(np.sum(mode == m)) for m in set(mode.tolist()) if m is not None},
    }


def run_one(args) -> dict:
    """One (controller, feedback, case) run; module-level so a process pool can call it."""
    from vehicle_model.simulation.controller_benchmark import run_scenario
    from vehicle_model.simulation.uncertainty import UncertaintyCase, plant_factory

    spec, ci, fb, ui = args
    c, u = spec.controllers[ci], spec.uncertainty[ui]
    case = UncertaintyCase.from_dict(u)
    try:
        res = run_scenario(plant_factory(_nominal(spec), case), build_controller(spec, c, fb, case), scenario(spec), spec.dt)
        k, err = kpis(res.history, spec.dt), ""
        hist = {key: np.asarray(res.history[key], float).tolist() for key in ("t", "r", "r_ref", "beta", "Y", "steer_r",
                                                                              "Mz_allocated")}
    except Exception as ex:   # a crashing controller is a result, reported as such
        k, err, hist = {kk: float("nan") for kk in KPIS}, f"{type(ex).__name__}: {ex}", None
    return {"controller": c["name"], "baseline": bool(c.get("baseline", False)), "feedback": fb, "case": u["name"],
            "kpis": k, "error": err, "history": hist}


def run(spec: RobustBenchmarkSpec, out: Path, verify: bool | None = None) -> dict:
    from .verification import verify_model
    from vehicle_model.controllers.robust_laws import RobustLQRYawLaw, robust_stability
    from vehicle_model.handling_analysis import linear_params_from_seven_dof

    formats = set(spec.outputs.get("formats", ()))
    verification = verify_model(lambda: _nominal(spec)) if (spec.verification if verify is None else verify) else []
    tasks = [(spec, ci, fb, ui) for ci in range(len(spec.controllers)) for fb in spec.feedback
             for ui in range(len(spec.uncertainty))]
    workers = spec.workers or max(1, min(8, (os.cpu_count() or 2) - 1))
    if workers > 1 and len(tasks) > 1:
        with ProcessPoolExecutor(max_workers=workers) as ex:
            runs = list(ex.map(run_one, tasks))
    else:
        runs = [run_one(t) for t in tasks]
    # linear robustness analysis of the LQR-type laws over the plant cases' parameter box (design model only)
    nom = _nominal(spec)
    design = linear_params_from_seven_dof(nom.params, nom.tyre_front, nom.tyre_rear)
    box = _box(spec)
    analysis = {}
    for c in spec.controllers:
        if c["law"] in ("lqr", "gs_lqr", "robust_lqr"):
            law = _law(c, design)
            analysis[c["name"]] = robust_stability(law, design, box)
    comparison = _compare(spec, runs)
    histories = {(r["controller"], r["feedback"], r["case"]): r.pop("history") for r in runs}
    summary = {"experiment": spec.to_dict(), "source": spec.source, "provenance": provenance(),
               "verification": verification, "results": runs, "comparison": comparison,
               "linear_robustness": analysis, "parameter_box": box}
    if "json" in formats:
        write_json(out / "results.json", summary)
    if "csv" in formats:
        with open(out / "kpis.csv", "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["controller", "baseline", "feedback", "case", *KPIS, "error"])
            for r in runs:
                w.writerow([r["controller"], r["baseline"], r["feedback"], r["case"],
                            *(r["kpis"].get(k, "") for k in KPIS), r["error"]])
    if "png" in formats:
        _plot(spec, runs, histories, out)
    if "md" in formats:
        (out / "report.md").write_text(_report(spec, summary), encoding="utf-8")
    return summary


def _box(spec) -> dict:
    """Parameter box of the design model spanned by the plant cases (mass, Iz, cornering stiffness)."""
    lo = {"mass": 1.0, "Iz": 1.0, "C_f": 1.0, "C_r": 1.0}
    hi = dict(lo)
    for u in spec.uncertainty:
        for k, f in (("mass", u.get("mass_factor", 1.0)), ("Iz", u.get("Iz_factor", 1.0)),
                     ("C_f", u.get("cornering_scale_front", 1.0)), ("C_r", u.get("cornering_scale_rear", 1.0))):
            lo[k], hi[k] = min(lo[k], f), max(hi[k], f)
    return {k: (lo[k], hi[k]) for k in lo if lo[k] != hi[k]}


def _compare(spec, runs) -> list:
    """Every controller relative to the first baseline, per feedback mode and case, on the tracking KPIs."""
    base = next((c["name"] for c in spec.controllers if c.get("baseline")), None)
    if base is None:
        return []
    idx = {(r["controller"], r["feedback"], r["case"]): r for r in runs}
    out = []
    for r in runs:
        b = idx.get((base, r["feedback"], r["case"]))
        if b is None or r["controller"] == base:
            continue
        row = {"controller": r["controller"], "feedback": r["feedback"], "case": r["case"], "baseline": base}
        worse = []
        for k in _LOWER_IS_BETTER:
            x, y = r["kpis"].get(k, float("nan")), b["kpis"].get(k, float("nan"))
            row[k] = float(x / y) if y and math.isfinite(x) and math.isfinite(y) else float("nan")
            if math.isfinite(row[k]) and row[k] > 1.05:
                worse.append(k)
        row["worse_than_baseline"] = worse
        row["lost_control"] = bool(r["kpis"].get("loss_of_control") == 1.0)
        out.append(row)
    return out


# ----------------------------------------------------------------------------- outputs
_COLOURS = ("#3987e5", "#d95926", "#199e70", "#c98500", "#d55181", "#008300", "#8a63d2", "#7d8590")


def _plot(spec, runs, histories, out: Path) -> None:
    _plot_matrices(spec, runs, out)
    _plot_histories(spec, histories, out)


def _plot_matrices(spec, runs, out: Path) -> None:
    """KPI matrices per feedback mode; logarithmic colour scale (losses of control would hide the rest)."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.colors import LogNorm

    from vehicle_model.visualization.theme import apply_dark_theme, style_axis
    apply_dark_theme()
    names = [c["name"] for c in spec.controllers]
    cases = [u["name"] for u in spec.uncertainty]
    for kpi, title in (("yaw_rate_rms_error_degps", "yaw-rate RMS error [deg/s]"), ("max_sideslip_deg", "max sideslip [deg]")):
        fig, axes = plt.subplots(1, len(spec.feedback), figsize=(6.5 * len(spec.feedback), 5.2), squeeze=False)
        for ax, fb in zip(axes[0], spec.feedback):
            M = np.full((len(names), len(cases)), np.nan)
            for r in runs:
                if r["feedback"] == fb:
                    M[names.index(r["controller"]), cases.index(r["case"])] = r["kpis"].get(kpi, np.nan)
            lo = max(float(np.nanmin(M)), 0.1) if np.any(np.isfinite(M)) else 0.1
            norm = LogNorm(vmin=lo, vmax=max(float(np.nanmax(M)), lo * 10))
            im = ax.imshow(M, aspect="auto", cmap="viridis", norm=norm)
            for i in range(M.shape[0]):
                for j in range(M.shape[1]):
                    if np.isfinite(M[i, j]):
                        bright = norm(max(M[i, j], lo)) > 0.6
                        ax.text(j, i, f"{M[i, j]:.2g}", ha="center", va="center", fontsize=7,
                                color="black" if bright else "white")
            ax.set_xticks(range(len(cases)), cases, rotation=30, ha="right", fontsize=8)
            ax.set_yticks(range(len(names)), names, fontsize=8)
            style_axis(ax, f"{fb}: {title}")
            ax.grid(False)                                  # no grid lines through the cells
            fig.colorbar(im, ax=ax, shrink=0.8)
        fig.suptitle(f"{spec.name}: {title} per controller and uncertainty case (log colour scale; synthetic parameters)")
        fig.tight_layout()
        fig.savefig(out / f"matrix_{kpi}.png", dpi=130, facecolor=fig.get_facecolor())
        plt.close(fig)


def _plot_histories(spec, histories, out: Path) -> None:
    """Yaw-rate histories for the first feedback mode, one panel per case."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    from vehicle_model.visualization.theme import TEXT_MUTED, apply_dark_theme, style_axis
    apply_dark_theme()
    names = [c["name"] for c in spec.controllers]
    cases = [u["name"] for u in spec.uncertainty]
    fb = spec.feedback[0]
    fig, axes = plt.subplots(1, len(cases), figsize=(5.0 * len(cases), 4.4), squeeze=False, sharey=True)
    for ax, case in zip(axes[0], cases):
        ref_drawn = False
        for name, col in zip(names, _COLOURS * 3):
            h = histories.get((name, fb, case))
            if not h:
                continue
            if not ref_drawn:
                ax.plot(h["t"], np.degrees(h["r_ref"]), color=TEXT_MUTED, lw=1.2, ls="--", label="reference")
                ref_drawn = True
            ax.plot(h["t"], np.degrees(h["r"]), color=col, lw=1.6, label=name)
        style_axis(ax, f"{case} ({fb})", "time [s]", "yaw rate [deg/s]")
    axes[0][0].legend(fontsize=7, loc="best")
    fig.suptitle(f"{spec.name}: yaw rate, {fb} feedback")
    fig.tight_layout()
    fig.savefig(out / "yaw_rate_histories.png", dpi=130, facecolor=fig.get_facecolor())
    plt.close(fig)


def _fmt(v) -> str:
    if isinstance(v, float):
        return "–" if math.isnan(v) else f"{v:.3g}"
    return str(v)


def _report(spec, summary) -> str:
    pv = summary["provenance"]
    L = [f"# Robust controller benchmark: {spec.name}", ""]
    if spec.description:
        L += [spec.description, ""]
    m = spec.manoeuvre
    L += ["## Setup", "", "| item | value |", "|---|---|", f"| specification | `{spec.source}` |",
          f"| nominal vehicle (controller design) | `{spec.vehicle}`, tyres {spec.tyres or 'from the vehicle file'} |",
          f"| manoeuvre | {m['type']} at {m['speed_kph']} km/h, road mu = {spec.road['mu']} |",
          f"| feedback modes | {', '.join(spec.feedback)} |", f"| step | {spec.dt} s, sensor seed {spec.seed} |",
          f"| code | git {pv['git_commit']}, Python {pv['python']}, numpy {pv['numpy']}, scipy {pv['scipy']} |", "",
          "Controllers are designed once from the nominal vehicle and are not re-tuned per case. All parameters are",
          "synthetic; results describe the virtual vehicle, not a real one.", ""]
    if summary["verification"]:
        ok = all(c["passed"] for c in summary["verification"])
        L += [f"Model verification (nominal vehicle): **{'PASS' if ok else 'FAIL'}** "
              f"({sum(c['passed'] for c in summary['verification'])}/{len(summary['verification'])} checks).", ""]
    L += ["## Uncertainty cases (deviation of the simulated plant from the nominal design)", "",
          "| case | deviations |", "|---|---|"]
    for u in spec.uncertainty:
        dev = ", ".join(f"{k}: {v}" for k, v in u.items() if k != "name") or "none"
        L.append(f"| {u['name']} | {dev} |")
    L += ["", "## Controllers", "", "| name | law | inputs | baseline | supervisor | options |", "|---|---|---|---|---|---|"]
    for c in spec.controllers:
        opts = ", ".join(f"{k}: {v}" for k, v in c.items() if k in ("alpha", "bandwidth", "w_max_vy", "w_max_r")) or "–"
        sup = f"fallback {c['supervisor']['fallback']}" if c.get("supervisor") else "–"
        L.append(f"| {c['name']} | {c['law']} | {c.get('inputs', 'tv')} | {'yes (V2.0)' if c.get('baseline') else '–'} | "
                 f"{sup} | {opts} |")
    cols = [("yaw_rate_rms_error_degps", "r RMS err [deg/s]"), ("max_sideslip_deg", "max beta [deg]"),
            ("lateral_deviation_m", "lat. dev. [m]"), ("saturation_share", "saturated"),
            ("max_yaw_moment_rate_knmps", "max dMz/dt [kNm/s]"), ("max_rear_steer_rate_degps", "max d delta_r/dt [deg/s]"),
            ("max_tyre_utilisation", "max tyre util."), ("computation_ms_mean", "comp. mean [ms]"),
            ("computation_ms_max", "comp. max [ms]"), ("infeasibility_rate", "infeasible"), ("fallback_rate", "fallback"),
            ("degraded_share", "degraded"), ("yaw_moment_effort_knms", "Mz effort [kNms]"),
            ("rear_steer_effort_degs", "delta_r effort [deg s]"), ("loss_of_control", "lost")]
    for fb in spec.feedback:
        L += ["", f"## Results: {fb} feedback", "", "| case | controller | " + " | ".join(c for _, c in cols) + " |",
              "|---" * (len(cols) + 2) + "|"]
        for u in spec.uncertainty:
            for r in summary["results"]:
                if r["feedback"] == fb and r["case"] == u["name"]:
                    cells = " | ".join(_fmt(r["kpis"].get(k, float("nan"))) for k, _ in cols)
                    L.append(f"| {u['name']} | {r['controller']}{' (error: ' + r['error'] + ')' if r['error'] else ''} | {cells} |")
    comp = summary["comparison"]
    if comp:
        L += ["", f"## Relative to the V2.0 baseline ({comp[0]['baseline']})", "",
              "Ratios < 1 are better than the baseline; 'worse' lists the tracking KPIs more than 5 % worse.", "",
              "| feedback | case | controller | r RMS err | max beta | lat. dev. | worse | lost |", "|---|---|---|---|---|---|---|---|"]
        for row in comp:
            L.append(f"| {row['feedback']} | {row['case']} | {row['controller']} | {_fmt(row['yaw_rate_rms_error_degps'])} | "
                     f"{_fmt(row['max_sideslip_deg'])} | {_fmt(row['lateral_deviation_m'])} | "
                     f"{', '.join(row['worse_than_baseline']) or '–'} | {'yes' if row['lost_control'] else '–'} |")
        n_worse = sum(bool(r["worse_than_baseline"]) for r in comp)
        L += ["", f"{n_worse} of {len(comp)} comparisons are worse than the baseline on at least one tracking KPI.", ""]
    if summary["linear_robustness"]:
        L += ["## Linear robustness of the LQR-type laws (design model over the parameter box)", "",
              f"Box (factors on the nominal design): {summary['parameter_box'] or 'none (only non-parametric deviations)'}.",
              "Spectral abscissa of the closed loop A - B K; < 0 stable. Sampled grid, not a proof; the nonlinear plant",
              "is not covered.", "", "| controller | worst abscissa [1/s] | at speed [m/s] | factors | nominal at that speed |",
              "|---|---|---|---|---|"]
        for n, a in summary["linear_robustness"].items():
            w = a["worst"]
            L.append(f"| {n} | {w['abscissa']:.3f} | {w['speed']:g} | {w['factors']} | {w['nominal']:.3f} |")
        L.append("")
    L += ["## Measures and limits", "",
          "- KPI definitions: module docstring of `experiments/robust_benchmark.py`. Computation times are wall clock",
          "  and not reproducible; everything else is (fixed sensor seed).",
          "- Ideal feedback reads the true state and telemetry (declared); sensor deviations act only in the estimated",
          "  and sensor-constrained modes. Actuator delay and control latency act in every mode.",
          "- The dynamic NMPC is a path tracker and is benchmarked on its lane change (`solver_benchmark`), not here.", ""]
    return "\n".join(L)


__all__ = ("FEEDBACK", "KIND", "KPIS", "LAWS", "RobustBenchmarkSpec", "build_controller", "describe", "kpis", "parse",
           "run", "run_one")
