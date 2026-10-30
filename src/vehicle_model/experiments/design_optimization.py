"""Experiment type ``design_optimization``: vehicle design and multi-objective optimisation (Plan V3.0 M14).

Lays out the virtual reference vehicle instead of only controlling it: a declared set of
design parameters is searched with a seeded multi-objective evolutionary algorithm
(NSGA-II), the Pareto front is analysed, and the selected designs are re-checked on an
independent evaluation set (other manoeuvres, road, speed and plant deviations).

**Models (existing, unchanged).** Handling, stability, actuator effort and robustness run on the
7-DOF two-track model (:class:`~vehicle_model.vehicle_models.SevenDOFModel`) with the shared
motion controller; ride comfort and roll on the 14-DOF model
(:class:`~vehicle_model.vehicle_models.fourteen_dof.FourteenDOFModel`, springs, dampers,
anti-roll bars, wheel hop, ISO 8608 road). The two are coupled through the roll-stiffness
distribution: the suspension parameters set the 14-DOF model and give the 7-DOF model its
``k_roll_f`` (``K_phi,axle = k_spring t^2 / 2 + k_arb``). Comfort is evaluated only where a
vertical model exists (Plan 9.2): the 7-DOF model has none.

**Design parameters (Plan 9.1)**, each ``{name, low, high}`` (reference value from the models):

    suspension    spring_front, spring_rear [N/m], damper_front, damper_rear [N s/m],
                  arb_front, arb_rear [N m/deg]                          (14-DOF; roll share -> 7-DOF)
    steering      rear_ratio_low, rear_ratio_high (speed-scheduled 4WS feedforward delta_r = k(v) delta_f,
                  counter-phase / same-phase), rear_transition_speed_mps, max_rear_steer_deg,
                  rear_steer_rate_degps, actuator_delay_s
    torque vect.  tv_kp, tv_ki (PI law) or lqr_q_r, lqr_r_yaw_moment (LQR law), max_yaw_moment [N m]
    control       control_period_s (sample and hold of the controller)

**Objectives (Plan 9.2)**, all minimised, each with the model that can represent it:

    handling_yaw_error      mean yaw-rate RMS error over the manoeuvres [deg/s]       7-DOF   handling, tracking
    stability_sideslip      max |beta| over manoeuvres x plant cases [deg]             7-DOF   stability
    agility_response_time   yaw-rate 90 % time in the step-steer manoeuvres [s]        7-DOF   agility
    comfort_body_accel      RMS heave acceleration on the ISO 8608 road [m/s^2]       14-DOF  comfort
    roll_gradient           body roll per lateral acceleration [deg/g]                14-DOF  handling feel
    actuator_effort         integral |M_z| over the manoeuvres [kN m s]               7-DOF   energy / actuator use
    rear_steer_effort       integral |delta_r| over the manoeuvres [deg s]            7-DOF   actuator use
    robustness_yaw_error    worst plant case of the mean yaw-rate RMS error [deg/s]  7-DOF   robustness
    control_rate_hz         controller evaluations per second (deterministic proxy)  -       computation

``objectives`` selects the optimised ones (2-4); ``report`` adds further ones that are only
computed. Wall-clock computation time is not reproducible and therefore not an objective.

**Validity (Plan 9.5).** A design outside its bounds or violating ``validity`` is not
simulated and never enters a Pareto front (constrained domination). Defaults: body bounce
frequency per axle in [0.9, 2.0] Hz, damping ratio in [0.15, 0.7], front roll-stiffness share
in [0.35, 0.75], ``rear_ratio_low <= rear_ratio_high``; a failing controller design (e.g. the
Riccati equation) or a non-finite run marks the design invalid with its reason.

**Pareto analysis (Plan 9.3).** Fast non-dominated sorting over all valid evaluations; outputs:
front plots for every objective pair (reference and selected designs marked), the Pareto
designs' parameters and objectives (CSV), objective distributions, the reference comparison,
sensitivities (standardised regression coefficients of every objective on the normalised
parameters, with R^2), and one YAML file per selected design (knee point and the best design
per objective).

**Robust design (Plan 9.4).** ``optimisation`` and ``evaluation`` are separate scenario sets.
The selected designs and the reference are evaluated on the evaluation set; a design passes if
it does not lose control there (max sideslip below ``max_sideslip_deg``) and every objective it
improves in optimisation is not worse than the reference in evaluation by more than
``tolerance`` (relative).
"""

from __future__ import annotations

import copy
import csv
import dataclasses
import math
import os
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import yaml

from .spec import _keys, _require
from .types import provenance, report_progress, write_json

KIND = "design_optimization"
SUSPENSION = {"spring_front": "k_spring_front", "spring_rear": "k_spring_rear", "damper_front": "c_damper_front",
              "damper_rear": "c_damper_rear", "arb_front": "k_arb_front", "arb_rear": "k_arb_rear"}
STEERING = {"rear_ratio_low": -0.30, "rear_ratio_high": 0.20, "rear_transition_speed_mps": 10.0,
            "max_rear_steer_deg": 10.0, "rear_steer_rate_degps": 35.0, "actuator_delay_s": 0.0}
CONTROL = {"tv_kp": 15000.0, "tv_ki": 100000.0, "lqr_q_r": 50.0, "lqr_r_yaw_moment": 1.0e-6, "max_yaw_moment": 4000.0,
           "control_period_s": 0.005}
PARAMETERS = set(SUSPENSION) | set(STEERING) | set(CONTROL)
OBJECTIVES = ("handling_yaw_error", "stability_sideslip", "agility_response_time", "comfort_body_accel",
              "roll_gradient", "actuator_effort", "rear_steer_effort", "robustness_yaw_error", "control_rate_hz")
UNITS = {"handling_yaw_error": "deg/s", "stability_sideslip": "deg", "agility_response_time": "s",
         "comfort_body_accel": "m/s^2", "roll_gradient": "deg/g", "actuator_effort": "kN m s",
         "rear_steer_effort": "deg s", "robustness_yaw_error": "deg/s", "control_rate_hz": "Hz"}
VALIDITY = {"bounce_hz": (0.9, 2.0), "damping_ratio": (0.15, 0.7), "roll_share_front": (0.35, 0.75)}
_TOP = {"type", "name", "description", "vehicle", "tyres", "ride_vehicle", "controller", "design", "fixed",
        "objectives", "report", "validity", "optimisation", "evaluation", "search", "selection", "dt", "workers",
        "outputs", "recheck"}
FORMATS = ("json", "csv", "png", "yaml", "md")
DEG = math.pi / 180.0


@dataclass(frozen=True)
class DesignSpec:
    name: str
    vehicle: str
    ride_vehicle: str
    controller: str
    design: tuple                 # ({name, low, high}, ...)
    objectives: tuple
    optimisation: dict
    evaluation: dict
    report: tuple = ()
    fixed: dict = field(default_factory=dict)
    validity: dict = field(default_factory=dict)
    search: dict = field(default_factory=dict)
    selection: dict = field(default_factory=dict)
    recheck: dict = field(default_factory=dict)
    tyres: dict = field(default_factory=dict)
    dt: float = 0.005
    workers: int | None = None
    outputs: dict = field(default_factory=dict)
    description: str = ""
    source: str = ""
    kind = KIND

    def to_dict(self) -> dict:
        return {"type": KIND, "name": self.name, "description": self.description, "vehicle": self.vehicle,
                "tyres": dict(self.tyres), "ride_vehicle": self.ride_vehicle, "controller": {"law": self.controller},
                "design": [dict(d) for d in self.design], "fixed": dict(self.fixed), "objectives": list(self.objectives),
                "report": list(self.report), "validity": {k: list(v) for k, v in self.validity.items()},
                "optimisation": copy.deepcopy(self.optimisation), "evaluation": copy.deepcopy(self.evaluation),
                "search": dict(self.search), "selection": dict(self.selection), "recheck": dict(self.recheck),
                "dt": self.dt, "outputs": dict(self.outputs)}


# ----------------------------------------------------------------------------- parsing
def _num(x, where) -> float:
    _require(isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x), f"{where} must be a number")
    return float(x)


def _scenario_set(d, where) -> dict:
    from vehicle_model.simulation.uncertainty import UncertaintyCase
    _require(isinstance(d, dict), f"{where} must be a mapping")
    _keys(d, {"manoeuvres", "ride", "roll", "uncertainty"}, where, {"manoeuvres"})
    mans = d["manoeuvres"]
    _require(isinstance(mans, list) and mans, f"{where}.manoeuvres must be a non-empty list")
    for i, m in enumerate(mans):
        w = f"{where}.manoeuvres[{i}]"
        _require(isinstance(m, dict) and m.get("type") in ("sine_with_dwell", "step_steer"),
                 f"{w}.type must be sine_with_dwell or step_steer")
        allowed = {"type", "speed_kph", "mu", "amplitude_deg"} if m["type"] == "sine_with_dwell" else \
            {"type", "speed_kph", "mu", "steer_deg", "duration_s"}
        _keys(m, allowed, w, {"type", "speed_kph"})
        _require(20 <= _num(m["speed_kph"], f"{w}.speed_kph") <= 200, f"{w}.speed_kph must lie in [20, 200]")
        _require(0.05 <= _num(m.get("mu", 1.0), f"{w}.mu") <= 2.0, f"{w}.mu must lie in [0.05, 2]")
    ride = dict(d.get("ride") or {})
    _keys(ride, {"road_class", "seed", "speed_mps", "duration_s"}, f"{where}.ride")
    _require(ride.get("road_class", "C") in "ABCDEFGH" and len(ride.get("road_class", "C")) == 1,
             f"{where}.ride.road_class must be one of A-H")
    roll = dict(d.get("roll") or {})
    _keys(roll, {"speed_mps", "lateral_accel_mps2"}, f"{where}.roll")
    unc = d.get("uncertainty") or [{"name": "nominal"}]
    _require(isinstance(unc, list) and unc, f"{where}.uncertainty must be a non-empty list")
    for i, u in enumerate(unc):
        _require(isinstance(u, dict) and isinstance(u.get("name"), str), f"{where}.uncertainty[{i}] needs a name")
        try:
            UncertaintyCase.from_dict(u)
        except (TypeError, ValueError) as ex:
            raise ValueError(f"{where}.uncertainty[{i}]: {ex}") from None
    return {"manoeuvres": [dict(m) for m in mans], "ride": ride, "roll": roll, "uncertainty": [dict(u) for u in unc]}


def parse(data: dict, source: str = "<dict>") -> DesignSpec:
    _require(data.get("type") == KIND, f"{source}: type must be {KIND!r}")
    _keys(data, _TOP, source, {"type", "name", "vehicle", "design", "objectives", "optimisation", "evaluation"})
    for k in ("vehicle", "ride_vehicle"):
        if k in data:
            _require(Path(data[k]).is_file(), f"{source}: {k} file not found: {data[k]}")
    ctrl = dict(data.get("controller") or {"law": "pi"})
    _keys(ctrl, {"law"}, f"{source}: controller", {"law"})
    _require(ctrl["law"] in ("pi", "lqr", "passive"), f"{source}: controller.law must be pi, lqr or passive")
    design = []
    for i, d in enumerate(data["design"] if isinstance(data["design"], list) else []):
        w = f"{source}: design[{i}]"
        _require(isinstance(d, dict), f"{w} must be a mapping")
        _keys(d, {"name", "low", "high"}, w, {"name", "low", "high"})
        _require(d["name"] in PARAMETERS, f"{w}.name {d['name']!r} unknown; allowed: {sorted(PARAMETERS)}")
        lo, hi = _num(d["low"], f"{w}.low"), _num(d["high"], f"{w}.high")
        _require(lo < hi, f"{w}: needs low < high")
        design.append({"name": d["name"], "low": lo, "high": hi})
    _require(design, f"{source}: design must be a non-empty list")
    names = [d["name"] for d in design]
    _require(len(set(names)) == len(names), f"{source}: design parameter names must be unique")
    if ctrl["law"] != "lqr":
        _require(not {"lqr_q_r", "lqr_r_yaw_moment"} & set(names), f"{source}: LQR weights need controller.law lqr")
    if ctrl["law"] != "pi":
        _require(not {"tv_kp", "tv_ki"} & set(names), f"{source}: tv_kp/tv_ki need controller.law pi")
    fixed = dict(data.get("fixed") or {})
    for k, v in fixed.items():
        _require(k in PARAMETERS, f"{source}: fixed.{k} unknown")
        _require(k not in names, f"{source}: fixed.{k} is also a design parameter")
        _num(v, f"{source}: fixed.{k}")
    objs = tuple(data["objectives"])
    _require(2 <= len(objs) <= 4 and set(objs) <= set(OBJECTIVES) and len(set(objs)) == len(objs),
             f"{source}: objectives must be 2-4 distinct names of {OBJECTIVES}")
    rep = tuple(data.get("report") or ())
    _require(set(rep) <= set(OBJECTIVES) and not set(rep) & set(objs), f"{source}: report must name other objectives")
    validity = {k: tuple(v) for k, v in VALIDITY.items()}
    for k, v in (data.get("validity") or {}).items():
        _require(k in VALIDITY, f"{source}: validity.{k} unknown; allowed: {sorted(VALIDITY)}")
        _require(isinstance(v, list) and len(v) == 2 and _num(v[0], k) < _num(v[1], k), f"{source}: validity.{k} must be [low, high]")
        validity[k] = (float(v[0]), float(v[1]))
    opt = _scenario_set(data["optimisation"], f"{source}: optimisation")
    ev = _scenario_set(data["evaluation"], f"{source}: evaluation")
    for sname, s in (("optimisation", opt), ("evaluation", ev)):
        if "agility_response_time" in objs + rep:
            _require(any(m["type"] == "step_steer" for m in s["manoeuvres"]),
                     f"{source}: agility_response_time needs a step_steer manoeuvre in {sname}")
    _require(opt != ev, f"{source}: optimisation and evaluation scenario sets must differ (Plan 9.4)")
    search = {"method": "nsga2", "population": 16, "generations": 4, "seed": 0, **(data.get("search") or {})}
    _keys(search, {"method", "population", "generations", "seed"}, f"{source}: search")
    _require(search["method"] == "nsga2", f"{source}: search.method must be nsga2")
    for k, lo in (("population", 4), ("generations", 0)):
        _require(isinstance(search[k], int) and not isinstance(search[k], bool) and lo <= search[k] <= 1000,
                 f"{source}: search.{k} must be an integer >= {lo}")
    sel = {"count": 3, **(data.get("selection") or {})}
    _keys(sel, {"count"}, f"{source}: selection")
    rc = {"max_sideslip_deg": 15.0, "tolerance": 0.05, **(data.get("recheck") or {})}
    _keys(rc, {"max_sideslip_deg", "tolerance"}, f"{source}: recheck")
    dt = _num(data.get("dt", 0.005), f"{source}: dt")
    _require(0.001 <= dt <= 0.01, f"{source}: dt must lie in [0.001, 0.01]")
    workers = data.get("workers")
    _require(workers is None or (isinstance(workers, int) and not isinstance(workers, bool) and workers >= 1),
             f"{source}: workers must be a positive integer")
    outputs = dict(data.get("outputs") or {})
    _keys(outputs, {"directory", "formats"}, f"{source}: outputs")
    outputs.setdefault("formats", list(FORMATS))
    _require(set(outputs["formats"]) <= set(FORMATS), f"{source}: outputs.formats must be a subset of {FORMATS}")
    outputs.setdefault("directory", f"output/experiments/{data['name']}")
    return DesignSpec(str(data["name"]), str(data["vehicle"]),
                      str(data.get("ride_vehicle", "configs/suspension/fourteen_dof.yaml")), ctrl["law"], tuple(design),
                      objs, opt, ev, rep, fixed, validity, search, sel, rc, dict(data.get("tyres") or {}), dt, workers,
                      outputs, str(data.get("description", "")), source)


def describe(spec: DesignSpec) -> str:
    return (f"{len(spec.design)} design parameter(s), objectives {', '.join(spec.objectives)}; NSGA-II "
            f"{spec.search['population']} x {spec.search['generations'] + 1}")


# ----------------------------------------------------------------------------- design -> models
def ride_config(spec):
    from vehicle_model.vehicle_models.fourteen_dof import FourteenDOFConfig
    return FourteenDOFConfig.from_yaml(spec.ride_vehicle)


def reference_values(spec) -> dict:
    """Design parameter values of the unmodified reference vehicle."""
    cfg = ride_config(spec)
    ref = {k: float(getattr(cfg, f)) for k, f in SUSPENSION.items()}
    ref.update(STEERING)
    ref.update(CONTROL)
    ref.update({k: float(v) for k, v in spec.fixed.items()})
    return ref


def full_values(spec, values: dict) -> dict:
    v = reference_values(spec)
    v.update({k: float(x) for k, x in values.items()})
    return v


def roll_stiffness(cfg) -> tuple[float, float]:
    """Axle roll stiffnesses [N m/rad] of the 14-DOF configuration (springs at the track, bar in N m/deg)."""
    kf = cfg.k_spring_front * cfg.track_front**2 / 2.0 + cfg.k_arb_front / DEG
    kr = cfg.k_spring_rear * cfg.track_rear**2 / 2.0 + cfg.k_arb_rear / DEG
    return kf, kr


def derived(spec, values: dict) -> dict:
    """14-DOF configuration, roll share for the 7-DOF model, and the static ride quantities."""
    v = full_values(spec, values)
    cfg = dataclasses.replace(ride_config(spec), **{f: v[k] for k, f in SUSPENSION.items()})
    kf, kr = roll_stiffness(cfg)
    out = {"ride_config": cfg, "roll_share_front": kf / (kf + kr)}
    for axle, share, ks, cs, kt in (("front", cfg.b / cfg.wheelbase, cfg.k_spring_front, cfg.c_damper_front, cfg.k_tyre_front),
                                    ("rear", cfg.a / cfg.wheelbase, cfg.k_spring_rear, cfg.c_damper_rear, cfg.k_tyre_rear)):
        m = cfg.sprung_mass * share / 2.0
        k_ride = ks * kt / (ks + kt)
        out[f"bounce_hz_{axle}"] = math.sqrt(k_ride / m) / (2.0 * math.pi)
        out[f"damping_ratio_{axle}"] = cs / (2.0 * math.sqrt(ks * m))
    return out


def validity_violations(spec, values: dict) -> list:
    out = []
    for p in spec.design:
        x = values.get(p["name"], None)
        if x is None or not p["low"] <= x <= p["high"]:
            out.append(f"{p['name']} outside [{p['low']}, {p['high']}]")
    v = full_values(spec, values)
    d = derived(spec, values)
    lo, hi = spec.validity["bounce_hz"]
    for axle in ("front", "rear"):
        if not lo <= d[f"bounce_hz_{axle}"] <= hi:
            out.append(f"bounce frequency {axle} {d[f'bounce_hz_{axle}']:.2f} Hz outside [{lo}, {hi}]")
    lo, hi = spec.validity["damping_ratio"]
    for axle in ("front", "rear"):
        if not lo <= d[f"damping_ratio_{axle}"] <= hi:
            out.append(f"damping ratio {axle} {d[f'damping_ratio_{axle}']:.2f} outside [{lo}, {hi}]")
    lo, hi = spec.validity["roll_share_front"]
    if not lo <= d["roll_share_front"] <= hi:
        out.append(f"front roll-stiffness share {d['roll_share_front']:.2f} outside [{lo}, {hi}]")
    if v["rear_ratio_low"] > v["rear_ratio_high"]:
        out.append("rear_ratio_low > rear_ratio_high (same-phase below counter-phase)")
    for k in ("max_rear_steer_deg", "rear_steer_rate_degps", "max_yaw_moment", "control_period_s",
              "rear_transition_speed_mps", "tv_kp", "lqr_q_r", "lqr_r_yaw_moment"):
        if v[k] <= 0.0:
            out.append(f"{k} must be > 0")
    if v["actuator_delay_s"] < 0.0 or v["tv_ki"] < 0.0:
        out.append("actuator_delay_s and tv_ki must be >= 0")
    return out


class _DesignController:
    """Motion controller (TV law) + speed-scheduled 4WS feedforward + sample-and-hold of the whole controller."""

    def __init__(self, inner, values: dict, params, period_steps: int):
        self.inner, self.v, self.p, self.n = inner, values, params, max(1, int(period_steps))
        self.name = f"{getattr(inner, 'name', type(inner).__name__)} + 4WS feedforward"
        self.reset()

    def reset(self):
        self.inner.reset()
        self._k, self._held, self._rear = 0, None, 0.0

    def compute_control(self, state, target, telemetry=None, dt: float = 0.01):
        if self._held is None or self._k % self.n == 0:
            out = self.inner.compute_control(state, target, telemetry, dt * self.n)
            v = self.v
            s = 1.0 / (1.0 + math.exp(-max(-30.0, min(30.0, (abs(state.vx) - v["rear_transition_speed_mps"]) / 2.5))))
            k = v["rear_ratio_low"] + (v["rear_ratio_high"] - v["rear_ratio_low"]) * s
            lim, rate = v["max_rear_steer_deg"] * DEG, v["rear_steer_rate_degps"] * DEG * dt * self.n
            want = float(np.clip(k * out.steer_f, -lim, lim))
            self._rear = float(np.clip(want, self._rear - rate, self._rear + rate))
            out.steer_r = self._rear
            self._held = out
        else:
            out = copy.copy(self._held)
            out.computation_time_ms = 0.0
        self._k += 1
        return out


def _plant_and_controller(spec, values: dict, case: dict):
    from vehicle_model.controllers.framework import PassiveBaselineController
    from vehicle_model.controllers.motion_control import MotionController, MotionControllerConfig
    from vehicle_model.controllers.yaw_laws import INPUTS_TV, LQRWeights, LQRYawLaw, YawRatePI
    from vehicle_model.controllers.robust_laws import DEFAULT_SPEEDS
    from vehicle_model.handling_analysis import linear_params_from_seven_dof
    from vehicle_model.simulation.uncertainty import UncertaintyCase, plant_factory, wrap_controller
    from vehicle_model.vehicle_models import SevenDOFModel

    v = full_values(spec, values)
    d = derived(spec, values)
    base = SevenDOFModel.from_yaml(spec.vehicle, tyre_name=spec.tyres.get("front"), tyre_rear_name=spec.tyres.get("rear"))
    params = dataclasses.replace(base.params, k_roll_f=d["roll_share_front"],
                                 max_steer_r=min(base.params.max_steer_r, v["max_rear_steer_deg"] * DEG))
    nominal = SevenDOFModel(params=params, tyre=base.tyre_front, tyre_rear=base.tyre_rear, road=base.road, chassis=base.chassis)
    unc = UncertaintyCase.from_dict({**case, "actuator_delay_s": case.get("actuator_delay_s", 0.0) + v["actuator_delay_s"]})
    n = max(1, int(round(v["control_period_s"] / spec.dt)))
    design = linear_params_from_seven_dof(params, nominal.tyre_front, nominal.tyre_rear)

    def factory(_p):
        if spec.controller == "passive":
            inner = PassiveBaselineController(params)
        else:
            law = (YawRatePI(kp=v["tv_kp"], ki=v["tv_ki"]) if spec.controller == "pi" else
                   LQRYawLaw(design, INPUTS_TV, LQRWeights(q_vy=0.0, q_r=v["lqr_q_r"], r_yaw_moment=v["lqr_r_yaw_moment"]),
                             design_speed=None, schedule_speeds=DEFAULT_SPEEDS))
            inner = MotionController(params, law, MotionControllerConfig(max_yaw_moment=v["max_yaw_moment"]))
        return wrap_controller(_DesignController(inner, v, params, n), unc)
    return plant_factory(nominal, unc), factory


def _manoeuvre(m: dict):
    from vehicle_model.roads.surface import UniformRoad
    from vehicle_model.simulation.controller_benchmark import sine_with_dwell_scenario, step_steer_scenario
    v, mu = float(m["speed_kph"]) / 3.6, float(m.get("mu", 1.0))
    if m["type"] == "sine_with_dwell":
        return sine_with_dwell_scenario(v, math.radians(float(m.get("amplitude_deg", 5.0))), mu)
    sc = step_steer_scenario(v, math.radians(float(m.get("steer_deg", 2.0))), float(m.get("duration_s", 3.0)))
    return dataclasses.replace(sc, road=lambda: UniformRoad(mu=mu))


def _t90(t, r) -> float:
    r = np.abs(np.asarray(r, float))
    target = 0.9 * r[-1]
    idx = np.flatnonzero(r >= target)
    return float(t[idx[0]]) if idx.size and r[-1] > 1e-6 else float("nan")


def _handling(spec, values, sset, need: set) -> dict:
    """7-DOF objectives over the manoeuvres (x plant cases for stability/robustness)."""
    from vehicle_model.simulation.controller_benchmark import run_scenario
    from .robust_benchmark import kpis
    cases = sset["uncertainty"] if need & {"stability_sideslip", "robustness_yaw_error"} else sset["uncertainty"][:1]
    per_case = []
    t90 = []
    eff = rear = 0.0
    beta_max = 0.0
    for ci, case in enumerate(cases):
        errs = []
        for m in sset["manoeuvres"]:
            plant, factory = _plant_and_controller(spec, values, case)
            res = run_scenario(plant, factory, _manoeuvre(m), spec.dt)
            h = res.history
            if not all(np.all(np.isfinite(np.asarray(h[k], float))) for k in ("X", "Y", "r", "beta")):
                raise FloatingPointError("non-finite vehicle state")
            k = kpis(h, spec.dt)
            errs.append(k["yaw_rate_rms_error_degps"])
            beta_max = max(beta_max, k["max_sideslip_deg"])
            if ci == 0:
                eff += k["yaw_moment_effort_knms"]
                rear += k["rear_steer_effort_degs"]
                if m["type"] == "step_steer":
                    t90.append(_t90(h["t"], h["r"]))
        per_case.append(float(np.mean(errs)))
    return {"handling_yaw_error": per_case[0], "stability_sideslip": beta_max,
            "agility_response_time": float(np.mean(t90)) if t90 else float("nan"), "actuator_effort": eff,
            "rear_steer_effort": rear, "robustness_yaw_error": float(np.max(per_case))}


def _ride(spec, values, sset) -> float:
    """RMS heave acceleration of the 14-DOF body on an ISO 8608 road, straight at constant speed."""
    from vehicle_model.roads.road_profile import ISO8608Profile
    from vehicle_model.vehicle_models.fourteen_dof import FourteenDOFModel, FourteenDOFState
    r = sset["ride"]
    cfg = derived(spec, values)["ride_config"]
    v, T, dt = float(r.get("speed_mps", 20.0)), float(r.get("duration_s", 4.0)), 0.001
    seed = int(r.get("seed", 1))
    left, right = ISO8608Profile(r.get("road_class", "C"), seed=seed), ISO8608Profile(r.get("road_class", "C"), seed=seed + 1000)
    xg = np.arange(-5.0, v * T * 1.2 + 10.0, 0.02)
    zl, zr, sl, sr = left.height(xg), right.height(xg), left.slope(xg), right.slope(xg)
    m = FourteenDOFModel(cfg)
    xs = np.array([cfg.a, cfg.a, -cfg.b, -cfg.b])
    s = FourteenDOFState(vx=v)
    acc = []
    T_drive = np.full(4, 0.0)
    t = 0.0
    while t < T - 1e-12:
        x = s.x + xs
        z = np.array([np.interp(x[0], xg, zl), np.interp(x[1], xg, zr), np.interp(x[2], xg, zl), np.interp(x[3], xg, zr)])
        dz = s.vx * np.array([np.interp(x[0], xg, sl), np.interp(x[1], xg, sr), np.interp(x[2], xg, sl),
                              np.interp(x[3], xg, sr)])
        T_drive[:] = max(0.0, 40.0 * (v - s.vx)) + 25.0          # holds the speed against drag and rolling losses
        s, tel = m.step(s, np.zeros(4), T_drive, np.zeros(4), dt, 1.0, z, dz)
        if t >= 1.0:                                              # after the start transient
            acc.append(tel.ddzs)
        t += dt
    a = np.asarray(acc)
    if not np.all(np.isfinite(a)):
        raise FloatingPointError("non-finite 14-DOF ride run")
    return float(np.sqrt(np.mean(a**2)))


def _roll(spec, values, sset) -> float:
    """Roll gradient of the 14-DOF model in a steady turn (flat road) [deg/g]."""
    from vehicle_model.vehicle_models.fourteen_dof import FourteenDOFModel, FourteenDOFState
    r = sset["roll"]
    cfg = derived(spec, values)["ride_config"]
    v, ay = float(r.get("speed_mps", 20.0)), float(r.get("lateral_accel_mps2", 4.0))
    delta = cfg.wheelbase * ay / v**2 * 1.15                      # kinematic steer plus some understeer
    m = FourteenDOFModel(cfg)
    s = FourteenDOFState(vx=v)
    dt, T = 0.002, 3.0
    phis, ays = [], []
    t = 0.0
    while t < T - 1e-12:
        d = delta * min(1.0, t / 0.5)
        T_drive = np.full(4, max(0.0, 60.0 * (v - s.vx)) + 30.0)
        s, tel = m.step(s, np.array([d, d, 0.0, 0.0]), T_drive, np.zeros(4), dt)
        if t >= T - 0.5:
            phis.append(s.phi)
            ays.append(tel.ay)
        t += dt
    a = float(np.mean(ays))
    if not math.isfinite(a) or abs(a) < 0.5:
        raise FloatingPointError("steady turn not reached in the 14-DOF roll run")
    return float(abs(np.degrees(np.mean(phis)) / (a / 9.81)))


def evaluate(spec, values: dict, which: str = "optimisation", objectives=None) -> dict:
    """Objectives of one design on one scenario set (``valid`` False with reasons if not admissible or failed)."""
    sset = spec.optimisation if which == "optimisation" else spec.evaluation
    names = tuple(objectives or (spec.objectives + spec.report))
    out = {"values": {k: float(x) for k, x in values.items()}, "set": which, "objectives": {}, "valid": True,
           "reasons": [], "error": ""}
    bad = validity_violations(spec, values)
    if bad:
        return {**out, "valid": False, "reasons": bad}
    d = derived(spec, values)
    out["derived"] = {k: float(x) for k, x in d.items() if k != "ride_config"}
    try:
        res = {}
        hand = set(names) & {"handling_yaw_error", "stability_sideslip", "agility_response_time", "actuator_effort",
                             "rear_steer_effort", "robustness_yaw_error"}
        if hand:
            res.update(_handling(spec, values, sset, hand))
        if "comfort_body_accel" in names:
            res["comfort_body_accel"] = _ride(spec, values, sset)
        if "roll_gradient" in names:
            res["roll_gradient"] = _roll(spec, values, sset)
        if "control_rate_hz" in names:
            res["control_rate_hz"] = 1.0 / full_values(spec, values)["control_period_s"]
        out["objectives"] = {k: float(res[k]) for k in names}
        if not all(math.isfinite(x) for x in out["objectives"].values()):
            raise FloatingPointError("non-finite objective")
    except Exception as ex:   # a failing design (controller design, numerics) is invalid, with its reason
        out.update({"valid": False, "reasons": [f"{type(ex).__name__}: {ex}"], "error": f"{type(ex).__name__}: {ex}"})
    return out


def _eval_task(args):
    spec, values, which = args
    return evaluate(spec, values, which)


# ----------------------------------------------------------------------------- Pareto
def dominates(a, b) -> bool:
    """``a`` Pareto-dominates ``b`` (minimisation): no worse in every objective, better in at least one."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    return bool(np.all(a <= b) and np.any(a < b))


def non_dominated_sort(F) -> list:
    """Fronts (lists of indices) of the rows of ``F`` (fast non-dominated sorting, Deb et al. 2002)."""
    F = np.asarray(F, float)
    n = len(F)
    S = [[] for _ in range(n)]
    cnt = np.zeros(n, int)
    fronts = [[]]
    for p in range(n):
        for q in range(n):
            if p == q:
                continue
            if dominates(F[p], F[q]):
                S[p].append(q)
            elif dominates(F[q], F[p]):
                cnt[p] += 1
        if cnt[p] == 0:
            fronts[0].append(p)
    i = 0
    while fronts[i]:
        nxt = []
        for p in fronts[i]:
            for q in S[p]:
                cnt[q] -= 1
                if cnt[q] == 0:
                    nxt.append(q)
        i += 1
        fronts.append(nxt)
    return fronts[:-1]


def crowding_distance(F) -> np.ndarray:
    F = np.asarray(F, float)
    n, m = F.shape
    d = np.zeros(n)
    if n <= 2:
        return np.full(n, np.inf)
    for j in range(m):
        o = np.argsort(F[:, j], kind="stable")
        d[o[0]] = d[o[-1]] = np.inf
        span = F[o[-1], j] - F[o[0], j]
        if span > 0:
            d[o[1:-1]] += (F[o[2:], j] - F[o[:-2], j]) / span
    return d


def _rank(pop_F, valid) -> tuple[np.ndarray, np.ndarray]:
    """Rank and crowding with constrained domination: every valid design ranks before every invalid one."""
    n = len(pop_F)
    rank, crowd = np.full(n, 10**6), np.zeros(n)
    vi = [i for i in range(n) if valid[i]]
    if vi:
        for r, front in enumerate(non_dominated_sort(pop_F[vi])):
            idx = [vi[k] for k in front]
            rank[idx] = r
            crowd[idx] = crowding_distance(pop_F[idx])
    return rank, crowd


def nsga2(spec, pool=None) -> list:
    """All evaluations (optimisation set) of a seeded NSGA-II run; each carries ``generation``."""
    s = spec.search
    rng = np.random.default_rng(int(s["seed"]))
    lo = np.array([p["low"] for p in spec.design])
    hi = np.array([p["high"] for p in spec.design])
    N, dim = int(s["population"]), len(spec.design)
    from scipy.stats import qmc
    X = lo + qmc.LatinHypercube(d=dim, seed=int(s["seed"])).random(N) * (hi - lo)
    evals = []

    def run(Xs, gen):
        report_progress(gen, int(s["generations"]) + 1, f"generation {gen}")
        tasks = [(spec, {p["name"]: float(x[j]) for j, p in enumerate(spec.design)}, "optimisation") for x in Xs]
        res = list(pool.map(_eval_task, tasks)) if pool is not None else [_eval_task(t) for t in tasks]
        for r in res:
            r["generation"] = gen
        evals.extend(res)
        return res

    def objs(res):
        return np.array([[r["objectives"][o] if r["valid"] else np.inf for o in spec.objectives] for r in res])

    res = run(X, 0)
    F, V = objs(res), np.array([r["valid"] for r in res])
    eta_c, eta_m = 15.0, 20.0
    for g in range(1, int(s["generations"]) + 1):
        rank, crowd = _rank(F, V)

        def pick():
            a, b = rng.integers(0, len(X), 2)
            return a if (rank[a], -crowd[a]) < (rank[b], -crowd[b]) else b
        kids = []
        while len(kids) < N:
            p1, p2 = X[pick()].copy(), X[pick()].copy()
            if rng.random() < 0.9:                                   # SBX crossover
                u = rng.random(dim)
                beta = np.where(u <= 0.5, (2 * u) ** (1 / (eta_c + 1)), (1 / (2 * (1 - u))) ** (1 / (eta_c + 1)))
                c1 = 0.5 * ((1 + beta) * p1 + (1 - beta) * p2)
                c2 = 0.5 * ((1 - beta) * p1 + (1 + beta) * p2)
            else:
                c1, c2 = p1, p2
            for c in (c1, c2):                                       # polynomial mutation, rate 1/dim
                for j in range(dim):
                    if rng.random() < 1.0 / dim:
                        u = rng.random()
                        dl = (2 * u) ** (1 / (eta_m + 1)) - 1 if u < 0.5 else 1 - (2 * (1 - u)) ** (1 / (eta_m + 1))
                        c[j] += dl * (hi[j] - lo[j])
                kids.append(np.clip(c, lo, hi))
        kids = np.array(kids[:N])
        kres = run(kids, g)
        X2, F2, V2 = np.vstack([X, kids]), np.vstack([F, objs(kres)]), np.r_[V, [r["valid"] for r in kres]]
        rank, crowd = _rank(F2, V2)
        order = sorted(range(len(X2)), key=lambda i: (rank[i], -crowd[i]))[:N]
        X, F, V = X2[order], F2[order], V2[order]
    report_progress(int(s["generations"]) + 1, int(s["generations"]) + 1, "search done; re-check")
    return evals


def pareto_set(spec, evals) -> list:
    """Indices (into ``evals``) of the valid, non-dominated designs, duplicates removed."""
    vi = [i for i, e in enumerate(evals) if e["valid"]]
    if not vi:
        return []
    F = np.array([[evals[i]["objectives"][o] for o in spec.objectives] for i in vi])
    front = [vi[k] for k in non_dominated_sort(F)[0]]
    seen, out = set(), []
    for i in front:
        key = tuple(round(evals[i]["values"][p["name"]], 9) for p in spec.design)
        if key not in seen:
            seen.add(key)
            out.append(i)
    return out


def select(spec, evals, front: list) -> list:
    """Knee point (closest to the utopia point in the normalised front) and the best design per objective."""
    if not front:
        return []
    F = np.array([[evals[i]["objectives"][o] for o in spec.objectives] for i in front])
    span = np.where(F.max(0) > F.min(0), F.max(0) - F.min(0), 1.0)
    Z = (F - F.min(0)) / span
    picks = [("knee", front[int(np.argmin(np.linalg.norm(Z, axis=1)))])]
    for j, o in enumerate(spec.objectives):
        picks.append((f"best_{o}", front[int(np.argmin(F[:, j]))]))
    out, seen = [], set()
    for label, i in picks:
        if i not in seen and len(out) < int(spec.selection["count"]):
            seen.add(i)
            out.append((label, i))
    return out


def sensitivities(spec, evals) -> dict:
    """Standardised regression coefficients of every objective on the normalised design parameters."""
    vi = [e for e in evals if e["valid"]]
    names = [p["name"] for p in spec.design]
    if len(vi) < len(names) + 2:
        return {}
    X = np.array([[(e["values"][p["name"]] - p["low"]) / (p["high"] - p["low"]) for p in spec.design] for e in vi])
    Xs = (X - X.mean(0)) / np.where(X.std(0) > 0, X.std(0), 1.0)
    out = {}
    for o in spec.objectives + spec.report:
        y = np.array([e["objectives"][o] for e in vi], float)
        if not np.all(np.isfinite(y)) or y.std() == 0:
            continue
        ys = (y - y.mean()) / y.std()
        A = np.column_stack([Xs, np.ones(len(ys))])
        coef, *_ = np.linalg.lstsq(A, ys, rcond=None)
        r2 = 1.0 - float(np.sum((A @ coef - ys) ** 2) / np.sum(ys**2))
        out[o] = {"src": dict(zip(names, coef[:-1].tolist())), "r2": r2}
    return out


# ----------------------------------------------------------------------------- run
def run(spec: DesignSpec, out: Path, verify: bool | None = None) -> dict:
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    ref_vals = {p["name"]: reference_values(spec)[p["name"]] for p in spec.design}
    workers = spec.workers or max(1, min(8, (os.cpu_count() or 2) - 1))
    if workers > 1:
        with ProcessPoolExecutor(max_workers=workers) as pool:
            evals = nsga2(spec, pool)
            reference = pool.submit(_eval_task, (spec, ref_vals, "optimisation"))
            front = pareto_set(spec, evals)
            chosen = select(spec, evals, front)
            rechecks = list(pool.map(_eval_task, [(spec, ref_vals, "evaluation")] +
                                     [(spec, evals[i]["values"], "evaluation") for _, i in chosen]))
            reference = reference.result()
    else:
        evals = nsga2(spec)
        reference = _eval_task((spec, ref_vals, "optimisation"))
        front = pareto_set(spec, evals)
        chosen = select(spec, evals, front)
        rechecks = [_eval_task((spec, ref_vals, "evaluation"))] + [_eval_task((spec, evals[i]["values"], "evaluation"))
                                                                     for _, i in chosen]
    ref_eval, sel_eval = rechecks[0], rechecks[1:]
    selected = []
    for (label, i), ev in zip(chosen, sel_eval):
        selected.append({"label": label, "values": evals[i]["values"], "derived": evals[i].get("derived", {}),
                         "optimisation": evals[i]["objectives"], "evaluation": ev["objectives"],
                         "recheck": recheck(spec, evals[i], reference, ev, ref_eval)})
    summary = {"experiment": spec.to_dict(), "source": spec.source, "provenance": provenance(), "verification": [],
               "evaluations": len(evals), "invalid": sum(not e["valid"] for e in evals),
               "reference": {"values": ref_vals, "optimisation": reference["objectives"], "evaluation": ref_eval["objectives"],
                             "derived": reference.get("derived", {}), "valid": reference["valid"],
                             "reasons": reference["reasons"]},
               "pareto": [{"values": evals[i]["values"], "objectives": evals[i]["objectives"],
                           "dominates_reference": reference["valid"] and dominates(
                               [evals[i]["objectives"][o] for o in spec.objectives],
                               [reference["objectives"][o] for o in spec.objectives])} for i in front],
               "selected": selected, "sensitivities": sensitivities(spec, evals),
               "results": [{k: e.get(k) for k in ("generation", "values", "valid", "objectives", "reasons")} for e in evals]}
    formats = set(spec.outputs.get("formats", ()))
    if "json" in formats:
        write_json(out / "results.json", summary)
    if "csv" in formats:
        _csv(spec, evals, front, out)
    if "yaml" in formats:
        for s in selected:
            (out / f"design_{s['label']}.yaml").write_text(_design_yaml(spec, s), encoding="utf-8")
    if "png" in formats:
        _plots(spec, evals, front, summary, out)
    if "md" in formats:
        (out / "report.md").write_text(_report(spec, summary), encoding="utf-8")
    return summary


def recheck(spec, opt, ref_opt, ev, ref_ev) -> dict:
    """Independent re-check on the evaluation set (Plan 9.4)."""
    if not ev["valid"]:
        return {"passed": False, "reasons": ["invalid on the evaluation set: " + "; ".join(ev["reasons"])]}
    reasons = []
    beta = ev["objectives"].get("stability_sideslip")
    if beta is None:
        beta = evaluate(spec, opt["values"], "evaluation", ("stability_sideslip",))["objectives"].get("stability_sideslip")
    if beta is not None and beta >= spec.recheck["max_sideslip_deg"]:
        reasons.append(f"loss of control on the evaluation set (max sideslip {beta:.1f} deg)")
    claims = {}
    for o in spec.objectives:
        if not ref_opt["valid"] or not ref_ev["valid"]:
            break
        if opt["objectives"][o] < ref_opt["objectives"][o]:          # improvement claimed in optimisation
            x, r = ev["objectives"][o], ref_ev["objectives"][o]
            ok = x <= r * (1.0 + spec.recheck["tolerance"]) + 1e-12
            claims[o] = {"evaluation": x, "reference": r, "kept": bool(ok)}
            if not ok:
                reasons.append(f"{o}: improvement not confirmed ({x:.4g} vs reference {r:.4g})")
    return {"passed": not reasons, "reasons": reasons, "claims": claims, "max_sideslip_deg": beta}


# ----------------------------------------------------------------------------- outputs
def _csv(spec, evals, front, out: Path) -> None:
    names = [p["name"] for p in spec.design]
    cols = list(spec.objectives + spec.report)
    with open(out / "evaluations.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["generation", *names, "valid", *cols, "pareto", "reasons"])
        fs = set(front)
        for i, e in enumerate(evals):
            w.writerow([e["generation"], *(e["values"][n] for n in names), e["valid"],
                        *(e["objectives"].get(c, "") for c in cols), i in fs, "; ".join(e["reasons"])])
    with open(out / "pareto_designs.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow([*names, "roll_share_front", *cols])
        for i in front:
            e = evals[i]
            w.writerow([*(e["values"][n] for n in names), e.get("derived", {}).get("roll_share_front", ""),
                        *(e["objectives"][c] for c in cols)])


def _design_yaml(spec, s) -> str:
    v = full_values(spec, s["values"])
    d = s["derived"]
    doc = {"design": s["label"], "source_experiment": spec.name, "provenance": provenance(),
           "note": "Selected Pareto design (synthetic parameters, virtual models only).",
           "seven_dof": {"vehicle": spec.vehicle, "k_roll_f": d.get("roll_share_front")},
           "fourteen_dof": {"vehicle": spec.ride_vehicle,
                            **{f: v[k] for k, f in SUSPENSION.items()}},
           "steering_4ws_feedforward": {k: v[k] for k in STEERING},
           "control": {"law": spec.controller, **{k: v[k] for k in CONTROL}},
           "objectives_optimisation": s["optimisation"], "objectives_evaluation": s["evaluation"],
           "recheck": s["recheck"]}
    return yaml.safe_dump(doc, sort_keys=False)


_C = ("#3987e5", "#d95926", "#199e70", "#c98500", "#d55181")
_SEL = ("#d95926", "#c98500", "#d55181", "#f5f5f5")         # selected designs, one colour each


def _plots(spec, evals, front, summary, out: Path) -> None:
    import itertools
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from vehicle_model.visualization.theme import TEXT_MUTED, apply_dark_theme, style_axis
    apply_dark_theme()
    objs = list(spec.objectives)
    V = [e for e in evals if e["valid"]]
    F = np.array([[e["objectives"][o] for o in objs] for e in V]) if V else np.zeros((0, len(objs)))
    P = np.array([[evals[i]["objectives"][o] for o in objs] for i in front]) if front else np.zeros((0, len(objs)))
    ref = summary["reference"]["optimisation"]
    pairs = list(itertools.combinations(range(len(objs)), 2))
    cols = min(3, len(pairs))
    rows = math.ceil(len(pairs) / cols)
    fig, axes = plt.subplots(rows, cols, figsize=(5.2 * cols, 4.4 * rows), squeeze=False)
    for ax in axes.flat[len(pairs):]:
        ax.axis("off")
    for ax, (i, j) in zip(axes.flat, pairs):
        ax.plot(F[:, i], F[:, j], "o", color=TEXT_MUTED, ms=3, alpha=0.5, label="valid designs")
        if len(P):
            o = np.argsort(P[:, i])
            ax.plot(P[o, i], P[o, j], "o", color=_C[0], ms=6, label="Pareto set (all objectives)")
        if ref:
            ax.plot(ref[objs[i]], ref[objs[j]], "P", color=_C[2], ms=11, label="reference vehicle")
        for k, s in enumerate(summary["selected"]):
            ax.plot(s["optimisation"][objs[i]], s["optimisation"][objs[j]], "*", color=_SEL[k % len(_SEL)], ms=13,
                    mec="#0f172a", label=f"selected: {s['label']}")
        style_axis(ax, f"{objs[i]} vs {objs[j]}", f"{objs[i]} [{UNITS[objs[i]]}]", f"{objs[j]} [{UNITS[objs[j]]}]")
    axes.flat[0].legend(fontsize=7)
    fig.suptitle(f"{spec.name}: Pareto analysis (minimise all; synthetic parameters, virtual models)")
    fig.tight_layout()
    fig.savefig(out / "pareto_fronts.png", dpi=110)
    plt.close(fig)

    allo = objs + list(spec.report)
    fig, axes = plt.subplots(1, len(allo), figsize=(3.6 * len(allo), 3.6), squeeze=False)
    for ax, o in zip(axes.flat, allo):
        x = np.array([e["objectives"][o] for e in V], float)
        ax.hist(x[np.isfinite(x)], bins=15, color=_C[0], alpha=0.8)
        if summary["reference"]["optimisation"]:
            ax.axvline(summary["reference"]["optimisation"][o], color=_C[2], lw=2, label="reference")
        style_axis(ax, o, UNITS[o], "designs")
    axes.flat[0].legend(fontsize=7)
    fig.suptitle(f"{spec.name}: distribution of the objective values ({len(V)} valid designs)")
    fig.tight_layout()
    fig.savefig(out / "objective_distributions.png", dpi=110)
    plt.close(fig)

    sens = summary["sensitivities"]
    if sens:
        names = [p["name"] for p in spec.design]
        fig, ax = plt.subplots(figsize=(1.1 * len(names) + 4, 0.5 * len(sens) + 2.5))
        M = np.array([[sens[o]["src"][n] for n in names] for o in sens])
        im = ax.imshow(M, cmap="RdBu_r", vmin=-1, vmax=1, aspect="auto")
        ax.set_xticks(range(len(names)), names, rotation=35, ha="right")
        ax.set_yticks(range(len(sens)), [f"{o} (R² {sens[o]['r2']:.2f})" for o in sens])
        for a in range(M.shape[0]):
            for b in range(M.shape[1]):
                ax.text(b, a, f"{M[a, b]:+.2f}", ha="center", va="center", fontsize=8,
                        color="#f5f5f5" if abs(M[a, b]) > 0.5 else "#0f172a")
        fig.colorbar(im, ax=ax, label="standardised regression coefficient")
        ax.set_title(f"{spec.name}: sensitivities over all valid designs (linear, normalised)")
        fig.tight_layout()
        fig.savefig(out / "sensitivities.png", dpi=110)
        plt.close(fig)


def _f(x) -> str:
    return "–" if x is None or (isinstance(x, float) and not math.isfinite(x)) else f"{x:.4g}"


def _report(spec, s) -> str:
    objs = list(spec.objectives)
    ref = s["reference"]
    L = [f"# Design optimisation: {spec.name}", "",
         f"Controller law {spec.controller}; objectives (minimised): {', '.join(objs)}"
         + (f"; reported: {', '.join(spec.report)}" if spec.report else "") +
         f". NSGA-II, population {spec.search['population']}, {spec.search['generations']} generations, seed "
         f"{spec.search['seed']}: {s['evaluations']} evaluations, {s['invalid']} invalid (excluded). Synthetic parameters, "
         "virtual models only (7-DOF handling, 14-DOF ride and roll).", "",
         "| design parameter | low | high | reference |", "|---|---|---|---|"]
    L += [f"| {p['name']} | {p['low']:g} | {p['high']:g} | {_f(ref['values'][p['name']])} |" for p in spec.design]
    L += ["", f"Pareto set: {len(s['pareto'])} design(s); {sum(p['dominates_reference'] for p in s['pareto'])} dominate the "
          "reference vehicle in every optimised objective.", "",
          "## Selected designs: optimisation set and independent evaluation set", "",
          "| design | " + " | ".join(f"{o} opt / eval" for o in objs) + " | re-check |", "|---" * (len(objs) + 2) + "|",
          "| reference | " + " | ".join(f"{_f(ref['optimisation'].get(o))} / {_f(ref['evaluation'].get(o))}" for o in objs)
          + " | – |"]
    for d in s["selected"]:
        rc = d["recheck"]
        L.append(f"| {d['label']} | " + " | ".join(f"{_f(d['optimisation'].get(o))} / {_f(d['evaluation'].get(o))}"
                                                  for o in objs)
                 + f" | {'passed' if rc['passed'] else 'FAILED: ' + '; '.join(rc['reasons'])} |")
    L += ["", "## Trade-offs (relative to the reference, optimisation set)", ""]
    for d in s["selected"]:
        ch = [f"{o} {100 * (d['optimisation'][o] / ref['optimisation'][o] - 1):+.1f} %" for o in objs
              if ref["optimisation"].get(o)]
        L.append(f"- {d['label']}: " + ", ".join(ch))
    if s["sensitivities"]:
        L += ["", "## Sensitivities (standardised regression coefficients)", ""]
        for o, v in s["sensitivities"].items():
            top = sorted(v["src"].items(), key=lambda kv: -abs(kv[1]))[:3]
            L.append(f"- {o} (R² {v['r2']:.2f}): " + ", ".join(f"{k} {c:+.2f}" for k, c in top))
    L += ["", "Files: evaluations.csv, pareto_designs.csv, design_<label>.yaml, pareto_fronts.png, "
          "objective_distributions.png, sensitivities.png, results.json.", ""]
    return "\n".join(L)


__all__ = ("DesignSpec", "crowding_distance", "derived", "describe", "dominates", "evaluate", "non_dominated_sort",
           "nsga2", "parse", "pareto_set", "recheck", "reference_values", "roll_stiffness", "run", "select",
           "sensitivities", "validity_violations")
