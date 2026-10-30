"""Run an experiment specification and write results and report (Plan.md M9).

:func:`run_experiment` dispatches on the experiment type (:mod:`.types`, Plan V3.0
§12). This module runs the default type ``controller_comparison``: for each sweep point (or once without a sweep) every controller runs on a fresh
plant built from the specification, through the controller benchmark of M6
(:mod:`vehicle_model.simulation.controller_benchmark`). Model-based laws use the
linear single track of the configured vehicle without K&C as design model.

Outputs in ``outputs.directory`` (selectable formats):

    results.json      specification, verification checks, KPIs per sweep point and controller,
                      provenance (git commit, package versions)
    kpis.csv          one row per sweep point and controller
    histories/*.csv   time histories per run
    <point>.png       yaw rate, sideslip and lateral position per controller
    <point>.gif       plan-view animation of all controllers (opt-in format "gif")
    report.md         engineering report: setup, verification, KPI tables, information access, limits
"""

from __future__ import annotations

import csv
import dataclasses
import math
from pathlib import Path

import numpy as np

from vehicle_model.controllers.estimated_feedback import EstimatedStateFeedback
from vehicle_model.controllers.framework import PassiveBaselineController
from vehicle_model.controllers.motion_control import MotionController
from vehicle_model.controllers.yaw_laws import (INPUTS_4WS_TV, INPUTS_AFS, INPUTS_AFS_TV, INPUTS_TV, LQRYawLaw, LTVMPCYawLaw,
                                                NoYawLaw, YawRatePI)
from vehicle_model.estimation.blended_sideslip import BlendedSideslipObserver
from vehicle_model.handling_analysis import linear_params_from_seven_dof
from vehicle_model.roads.surface import MuSplitRoad, UniformRoad
from vehicle_model.simulation.controller_benchmark import (
    brake_in_turn_scenario,
    coast_scenario,
    corner_exit_scenario,
    lane_change_scenario,
    lift_off_scenario,
    steer_ramp_scenario,
    launch_scenario,
    run_scenario,
    sine_with_dwell_scenario,
    straight_braking_scenario,
    split_mu_braking_scenario,
    step_steer_scenario,
)
from vehicle_model.vehicle_models import SevenDOFModel

from .spec import ExperimentSpec
from .types import get_type, provenance, report_progress, slug as _slug, write_json
from .verification import verify_model

SCHEDULE_SPEEDS = (5.0, 10.0, 15.0, 20.0, 25.0, 30.0, 40.0)
_HISTORY_KEYS = ("t", "X", "Y", "psi", "vx", "vy", "speed", "r", "r_ref", "beta", "ax", "ay", "steer_f", "steer_r",
                 "Mz_demand", "Mz_allocated", "Mz_requested", "Mz_alloc_pre_slip", "Mz_issued", "Mz_realised",
                 "T_issued_abs", "T_realised_abs", "regen_power_w", "friction_power_w", "abs_active_n",
                 "alloc_residual_fx", "drive_power_w", "tyre_utilisation", "max_abs_slip", "T_brake_total",
                 "status",
                 *(f"{q}_{w}" for q in ("Fz", "Fx", "Fy") for w in ("FL", "FR", "RL", "RR")))


def make_road(spec: ExperimentSpec):
    r = spec.road
    if r["type"] == "uniform":
        return UniformRoad(mu=float(r["mu"]))
    if r["type"] == "mu_step":
        from vehicle_model.roads.surface import TransitionRoad
        return TransitionRoad(mu_before=float(r["mu_before"]), mu_after=float(r["mu_after"]), x_start=float(r["x_step"]),
                              transition_length=0.5)
    return MuSplitRoad(mu_left=float(r["mu_left"]), mu_right=float(r["mu_right"]), split_y=0.0, transition_width=0.05)


def plant_factory(spec: ExperimentSpec):
    def build(road):
        return SevenDOFModel.from_yaml(spec.vehicle, tyre_name=spec.tyres.get("front"), road=road,
                                       tyre_rear_name=spec.tyres.get("rear"), kc_config=spec.kc)
    return build


def make_scenario(spec: ExperimentSpec):
    m = spec.manoeuvre
    v = float(m["speed_kph"]) / 3.6
    if m["type"] == "step_steer":
        sc = step_steer_scenario(v, math.radians(float(m.get("steer_deg", 2.3))), float(m.get("duration_s", 3.0)))
    elif m["type"] == "sine_with_dwell":
        sc = sine_with_dwell_scenario(v, math.radians(float(m.get("amplitude_deg", 8.0))))
    elif m["type"] == "straight_braking":
        sc = straight_braking_scenario(v, float(m.get("brake_pedal", 1.0)))
    elif m["type"] == "brake_in_turn":
        sc = brake_in_turn_scenario(v, math.radians(float(m.get("steer_deg", 2.0))), float(m.get("brake_pedal", 0.4)),
                                    float(m.get("brake_start_s", 2.0)), float(m.get("duration_s", 6.0)))
    elif m["type"] == "coast":
        sc = coast_scenario(v, float(m.get("duration_s", 8.0)))
    elif m["type"] == "steer_ramp":
        sc = steer_ramp_scenario(v, math.radians(float(m.get("steer_deg", 3.0))), math.radians(float(m.get("steer_rate_degps", 20.0))),
                                 float(m.get("accel_pedal", 0.05)), float(m.get("duration_s", 4.0)))
    elif m["type"] == "lane_change":
        sc = lane_change_scenario(v, math.radians(float(m.get("amplitude_deg", 2.5))), float(m.get("frequency_hz", 0.5)),
                                  float(m.get("accel_pedal", 0.05)), float(m.get("duration_s", 5.0)))
    elif m["type"] == "lift_off":
        sc = lift_off_scenario(v, math.radians(float(m.get("steer_deg", 3.0))), float(m.get("accel_pedal", 0.3)),
                               float(m.get("lift_s", 2.0)), float(m.get("duration_s", 5.0)))
    elif m["type"] == "launch":
        sc = launch_scenario(v, float(m.get("accel_pedal", 1.0)), float(m.get("duration_s", 6.0)))
    elif m["type"] == "corner_exit":
        sc = corner_exit_scenario(v, math.radians(float(m.get("steer_deg", 4.0))), float(m.get("accel_pedal", 1.0)),
                                  float(m.get("accel_start_s", 2.0)), float(m.get("duration_s", 6.0)))
    else:
        sc = split_mu_braking_scenario(v)
    return dataclasses.replace(sc, road=lambda: make_road(spec))


def controller_factories(spec: ExperimentSpec) -> dict:
    base = SevenDOFModel.from_yaml(spec.vehicle, tyre_name=spec.tyres.get("front"), tyre_rear_name=spec.tyres.get("rear"))
    design = linear_params_from_seven_dof(base.params, base.tyre_front, base.tyre_rear)
    out = {}
    for c in spec.controllers:
        if c.law == "passive":
            out[c.label] = None if c.actuators == "ideal" else _with_actuators(lambda p: PassiveBaselineController(p))
            continue
        inputs = {"tv": INPUTS_TV, "4ws_tv": INPUTS_4WS_TV, "afs": INPUTS_AFS, "afs_tv": INPUTS_AFS_TV}[c.inputs]

        def factory(p, c=c, inputs=inputs):
            if c.law == "pi":
                law = YawRatePI()
            elif c.law == "esc":                  # ESC-like: PI acting only outside a 3 deg/s yaw-rate band (M24 P1)
                law = YawRatePI(deadband=math.radians(3.0), name="ESC (PI, deadband 3 deg/s)")
            elif c.law == "lqr":
                law = LQRYawLaw(design, inputs, design_speed=20.0)
            elif c.law == "gs_lqr":
                law = LQRYawLaw(design, inputs, design_speed=None, schedule_speeds=SCHEDULE_SPEEDS)
            elif c.law in ("brakes", "no_yaw"):
                law = NoYawLaw()
            else:
                law = LTVMPCYawLaw(design, inputs)
            yt = dict(c.traction).get("yaw_tolerance")
            if yt is not None:                    # ITC: the yaw law only acts outside a yaw-rate band (M18.2)
                from vehicle_model.controllers.integrated_traction import YawToleranceLaw
                law = YawToleranceLaw(law, math.radians(float(yt)), dict(c.traction).get("itc_scope", "always"))
            ctrl = MotionController(p, law, _with_hdc(_with_gvc(_with_steering(_motion_config(c), c), c), c), _health(c))
            if c.feedback == "ideal":
                return ctrl
            return EstimatedStateFeedback(ctrl, BlendedSideslipObserver(p), seed=spec.seed,
                                          mode="C" if c.feedback == "sensor_constrained" else "B", params=p)
        out[c.label] = factory if c.actuators == "ideal" else _with_actuators(factory, c)
    return out


def _with_steering(cfg, c):
    """Add the AFS steering overlay (variable ratio, overlay authority, failure) of the entry (Plan V4.0 M19.2)."""
    st = dict(c.steering)
    if not st and not c.inputs.startswith("afs"):
        return cfg
    from vehicle_model.controllers.active_front_steering import SteeringOverlayConfig
    gains = (1.25, 1.0, 0.85) if st.get("ratio") == "variable" else (1.0, 1.0, 1.0)
    overlay = SteeringOverlayConfig(ratio_gains=gains, max_overlay=0.0 if not c.inputs.startswith("afs") else math.radians(5.0),
                                    failed=bool(st.get("afs_failed", False)))
    return dataclasses.replace(cfg, steering=overlay)


def _with_gvc(cfg, c):
    """Add the GVC-like load-transfer control of the entry (Plan V4.0 M20; enabled: false makes it vanish)."""
    gv = dict(c.gvc)
    if not gv:
        return cfg
    from vehicle_model.controllers.g_vectoring import GVCConfig
    kw = {"enabled": bool(gv.get("enabled", True))}
    if "c_xy" in gv:
        kw["c_xy"] = float(gv["c_xy"])
    if "max_decel_g" in gv:
        kw["max_decel"] = float(gv["max_decel_g"]) * 9.81
    return dataclasses.replace(cfg, gvc=GVCConfig(**kw))


def _with_hdc(cfg, c):
    """Add hill descent control of the entry (Plan V4.0 M22; enabled: false makes it vanish)."""
    hd = dict(c.hdc)
    if not hd:
        return cfg
    from vehicle_model.experiments.hill_descent import _hdc_config
    return dataclasses.replace(cfg, hdc=_hdc_config(hd))


def _health(c):
    """Actuator health of the entry's traction block (failed motors, Plan V4.0 M18) or None."""
    from vehicle_model.allocation import ActuatorHealth
    failed = dict(c.traction).get("failed", ())
    if not failed:
        return None
    names = ("FL", "FR", "RL", "RR")
    return ActuatorHealth(drive_ok=tuple(n not in failed for n in names))


def _motion_config(c):
    """MotionControllerConfig with the brake (M17) and traction (M18) functions of the controller entry."""
    from vehicle_model.allocation import AllocatorConfig
    from vehicle_model.controllers.motion_control import ActuationConfig, MotionControllerConfig
    b, tr = dict(c.brake), dict(c.traction)
    dist = ibc = cbc = None
    traction, tcs, alloc = None, True, AllocatorConfig()
    if tr:
        from vehicle_model.controllers.traction_distribution import TractionDistributionConfig
        if tr.get("distribution", "free") != "free" or tr.get("envelope"):
            traction = TractionDistributionConfig(mode=tr.get("distribution", "free"),
                                                  lock_torque=float(tr.get("lock_torque", 300.0)),
                                                  envelope=bool(tr.get("envelope", False)))
        tcs = bool(tr.get("tcs", True))
        driven = {"awd": (True,) * 4, "fwd": (True, True, False, False), "rwd": (False, False, True, True)}[tr.get("driven", "awd")]
        alloc = AllocatorConfig(driven=driven)
    if not b:
        return MotionControllerConfig(actuation=ActuationConfig(allocator=alloc, traction=traction, enable_tcs=tcs))
    from vehicle_model.controllers.brake_distribution import BrakeDistributionConfig, brake_functions_config
    bf = brake_functions_config()                     # configs/braking/brake_functions.yaml
    if b.get("distribution", "grip") != "grip":
        dist = BrakeDistributionConfig(mode=b["distribution"], **bf["ebd"])
    if b.get("ibc"):
        from vehicle_model.controllers.integrated_brake import IntegratedBrakeConfig
        ibc = IntegratedBrakeConfig(regen_policy=b.get("regen", "cut_on_abs"), **bf["ibc"])
    if b.get("cbc"):
        cbc = bf["cbc"]
    return MotionControllerConfig(actuation=ActuationConfig(allocator=alloc, distribution=dist, ibc=ibc, traction=traction,
                                                            enable_tcs=tcs), cbc=cbc)


def _with_actuators(factory, c=None):
    """Factory of the same controller behind the actuator models (Plan V4.0 M16; configs/braking, powertrains);
    with the regenerative slip control of the entry's brake block (M17.4)."""
    from vehicle_model.simulation.actuator_chain import ActuatorChain, ActuatorChainConfig
    regen = None
    b = dict(c.brake) if c is not None else {}
    if "regen" in b or "coast_torque" in b:
        from vehicle_model.controllers.regenerative_slip import RegenSlipConfig
        from vehicle_model.controllers.brake_distribution import brake_functions_config
        regen = RegenSlipConfig(policy=b.get("regen", "cut_on_abs"), coast_torque=float(b.get("coast_torque", 0.0)),
                                **brake_functions_config()["regenerative_slip"])

    def build(p):
        return ActuatorChain(factory(p), p, ActuatorChainConfig.from_yaml(regen=regen))
    return build


def run_experiment(spec, out_dir: str | Path | None = None, verify: bool | None = None) -> dict:
    """Run a validated specification of any experiment type into ``out_dir`` (default: from the specification)."""
    out = Path(out_dir or spec.outputs["directory"])
    out.mkdir(parents=True, exist_ok=True)
    return get_type(spec.kind).run(spec, out, verify)


def describe_controller_comparison(spec: ExperimentSpec) -> str:
    return (f"{spec.manoeuvre['type']}, {len(spec.controllers)} controller(s)"
            + (f", sweep over {spec.sweep['parameter']}" if spec.sweep else ""))


def run_controller_comparison(spec: ExperimentSpec, out: Path, verify: bool | None = None) -> dict:
    formats = set(spec.outputs.get("formats", ()))
    points = [(None, spec)] if not spec.sweep else [
        (v, spec.with_parameter(spec.sweep["parameter"], v)) for v in spec.sweep["values"]]

    verification = []
    if spec.verification if verify is None else verify:
        verification = verify_model(lambda: plant_factory(spec)(UniformRoad(mu=1.0)))
    total = len(points) * len(spec.controllers)

    results = []
    for value, sp in points:
        sc = make_scenario(sp)
        runs = []
        point = "base" if value is None else "{}={}".format(spec.sweep["parameter"], value)
        for label, factory in controller_factories(sp).items():
            report_progress(len(results), total, f"{point} / {label}")
            r = run_scenario(plant_factory(sp), factory, sc, sp.dt)
            runs.append((label, r))
            results.append({"point": point, "sweep_value": value, "controller": label, "kpis": r.kpis})
            if "csv" in formats:
                (out / "histories").mkdir(exist_ok=True)
                fname = out / "histories" / f"{_slug(point)}__{_slug(label)}.csv"
                with open(fname, "w", newline="", encoding="utf-8") as f:
                    w = csv.writer(f)
                    w.writerow(_HISTORY_KEYS)
                    w.writerows(zip(*(r.history[k] for k in _HISTORY_KEYS)))
        if "png" in formats:
            _plot(runs, out / (_slug(point) + ".png"), sp)
        if "gif" in formats:
            _animate(runs, out / (_slug(point) + ".gif"), sp)

    report_progress(total, total, "writing outputs")
    summary = {"experiment": spec.to_dict(), "source": spec.source, "provenance": provenance(),
               "verification": verification, "results": results}
    if "json" in formats:
        write_json(out / "results.json", summary)
    if "csv" in formats:
        keys = sorted({k for r in results for k, v in r["kpis"].items() if not isinstance(v, dict)})
        with open(out / "kpis.csv", "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["point", "controller", *keys])
            for r in results:
                w.writerow([r["point"], r["controller"], *(r["kpis"].get(k, "") for k in keys)])
    if "md" in formats:
        (out / "report.md").write_text(_report(spec, summary), encoding="utf-8")
    return summary


def _plot(runs, path, spec):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    from vehicle_model.visualization.theme import TEXT_MUTED, apply_dark_theme, style_axis
    colours = ("#3987e5", "#d95926", "#199e70", "#c98500", "#d55181", "#008300")  # validated slots 1-6
    apply_dark_theme()
    fig, ax = plt.subplots(1, 3, figsize=(17, 5))
    cols = iter(colours)
    for label, r in runs:
        h = r.history
        c = TEXT_MUTED if label == "passive" else next(cols, TEXT_MUTED)
        ls = "--" if label == "passive" else "-"
        ax[0].plot(h["t"], h["r"], color=c, ls=ls, lw=1.6, label=label)
        ax[1].plot(h["t"], np.degrees(h["beta"]), color=c, ls=ls, lw=1.6)
        ax[2].plot(h["X"], h["Y"], color=c, ls=ls, lw=1.6)
    ax[0].legend(fontsize=8, loc="best")
    style_axis(ax[0], "yaw rate", "time [s]", "[rad/s]")
    style_axis(ax[1], "sideslip angle", "time [s]", "[deg]")
    style_axis(ax[2], "path", "X [m]", "Y [m]")
    fig.suptitle(f"{spec.name}: {spec.manoeuvre['type']} at {spec.manoeuvre['speed_kph']} km/h (synthetic parameters)")
    fig.tight_layout()
    fig.savefig(path, dpi=130, facecolor=fig.get_facecolor())
    plt.close(fig)


def _kv(d) -> str:
    return ", ".join(f"{k}: {v}" for k, v in d.items())


def _animate(runs, path, spec, frames: int = 50, length: float = 4.6, width: float = 1.8):
    """Plan view: each controller's vehicle as an oriented rectangle on its path (``frames`` frames)."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter

    from vehicle_model.visualization.theme import TEXT_MUTED, apply_dark_theme, style_axis
    colours = ("#3987e5", "#d95926", "#199e70", "#c98500", "#d55181", "#008300")
    apply_dark_theme()
    fig, ax = plt.subplots(figsize=(9, 4.5))
    t_end = max(r.history["t"][-1] for _, r in runs)
    xs = np.concatenate([r.history["X"] for _, r in runs])
    ys = np.concatenate([r.history["Y"] for _, r in runs])
    pad = 4.0
    ax.set_xlim(xs.min() - pad, xs.max() + pad)
    ax.set_ylim(min(ys.min(), -3.0) - pad, max(ys.max(), 3.0) + pad)
    ax.set_aspect("equal")
    style_axis(ax, f"{spec.name}: plan view", "X [m]", "Y [m]")
    body = np.array([[length / 2, width / 2], [-length / 2, width / 2], [-length / 2, -width / 2], [length / 2, -width / 2]])
    cols = iter(colours)
    artists = []
    for label, r in runs:
        c = TEXT_MUTED if label == "passive" else next(cols, TEXT_MUTED)
        trail, = ax.plot([], [], color=c, lw=1.2, alpha=0.8)
        car = plt.Polygon(body, closed=True, fc=c, ec="#0f172a", alpha=0.9, label=label)
        ax.add_patch(car)
        artists.append((r.history, trail, car))
    ax.legend(loc="upper left", fontsize=8)
    stamp = ax.text(0.99, 0.02, "", transform=ax.transAxes, ha="right", color=TEXT_MUTED, fontsize=9)

    def draw(i):
        t = t_end * i / (frames - 1)
        for h, trail, car in artists:
            k = int(np.searchsorted(h["t"], min(t, h["t"][-1])))
            k = min(k, len(h["t"]) - 1)
            trail.set_data(h["X"][:k + 1], h["Y"][:k + 1])
            c, s = np.cos(h["psi"][k]), np.sin(h["psi"][k])
            car.set_xy(body @ np.array([[c, s], [-s, c]]) + [h["X"][k], h["Y"][k]])
        stamp.set_text(f"t = {t:4.2f} s")
        return []

    FuncAnimation(fig, draw, frames=frames).save(path, writer=PillowWriter(fps=12), dpi=80)
    plt.close(fig)


def _fmt(v):
    if isinstance(v, float):
        return "–" if math.isnan(v) else f"{v:.4g}"
    return str(v)


def _report(spec: ExperimentSpec, summary: dict) -> str:
    L = [f"# Experiment report: {spec.name}", ""]
    if spec.description:
        L += [spec.description, ""]
    pv = summary["provenance"]
    L += ["## Setup", "", "| item | value |", "|---|---|",
          f"| specification | `{spec.source}` |", f"| vehicle | `{spec.vehicle}` |",
          f"| tyres | {_kv(spec.tyres) or 'from the vehicle file'} |", f"| K&C | {spec.kc or 'none'} |",
          f"| road | {_kv(spec.road)} |", f"| manoeuvre | {_kv(spec.manoeuvre)} |",
          f"| sweep | {_kv(spec.sweep) if spec.sweep else 'none'} |", f"| step size / seed | {spec.dt} s / {spec.seed} |",
          f"| code | git {pv['git_commit']}, Python {pv['python']}, numpy {pv['numpy']}, scipy {pv['scipy']} |", ""]
    L += ["All parameters are synthetic; results describe the virtual vehicle, not a real one.", ""]
    if summary["verification"]:
        ok = all(c["passed"] for c in summary["verification"])
        L += ["## Model verification", "", f"Overall: **{'PASS' if ok else 'FAIL'}**", "",
              "| check | result | value | tolerance | description |", "|---|---|---|---|---|"]
        for c in summary["verification"]:
            L.append(f"| {c['name']} | {'PASS' if c['passed'] else '**FAIL**'} | {_fmt(c['value'])} | {_fmt(c['tolerance'])} | {c['description']} |")
        L.append("")
    L += ["## Results", ""]
    for point in dict.fromkeys(r["point"] for r in summary["results"]):
        rows = [r for r in summary["results"] if r["point"] == point]
        keys = [k for k in dict.fromkeys(k for r in rows for k in r["kpis"])
                if not isinstance(rows[0]["kpis"].get(k), dict) and not k.startswith("computation")]
        L += [f"### {point}", "", "| controller | " + " | ".join(keys) + " | status counts |", "|---" * (len(keys) + 2) + "|"]
        for r in rows:
            st = ", ".join(f"{k}: {v}" for k, v in r["kpis"]["status_counts"].items())
            L.append(f"| {r['controller']} | " + " | ".join(_fmt(r["kpis"].get(k, float("nan"))) for k in keys) + f" | {st} |")
        L.append("")
    L += ["## Information access and limits", "",
          "- Controllers use the vehicle state and the telemetry (wheel loads, lateral forces, road friction): idealised",
          "  full-state feedback, except where marked `+ observer` (motion state from virtual sensors and the blended",
          "  sideslip observer; allocation and slip control still use the telemetry) and `+ sensors only` (Mode C:",
          "  wheel loads, forces, slips and road friction are estimated from the sensors as well).",
          "- Model-based laws use the linear single track of the configured vehicle without K&C as design model.",
          "- Sine with dwell is modelled on FMVSS 126 (limits 0.35 / 0.20 / 1.83 m); it is a simulation, not the regulatory test.",
          "- Computation times are wall clock and not reproducible; all other results are (same specification, same code).", ""]
    return "\n".join(L)


__all__ = ("controller_factories", "describe_controller_comparison", "make_road", "make_scenario", "plant_factory",
           "run_controller_comparison", "run_experiment")
