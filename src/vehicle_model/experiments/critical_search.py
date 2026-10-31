"""Experiment type ``critical_scenario_search``: adversarial virtual testing.

Finds the manoeuvres and vehicle/periphery deviations under which one controller does worst,
offline, from a declared parameter space, then reduces every critical case to a minimal,
reproducible reference scenario.

**Parameter space.** Every searched parameter has a bound and a nominal value::

    parameters:
      - {name: amplitude_deg, low: 4, high: 10, nominal: 5}

Allowed names (unknown names fail hard):

    manoeuvre   speed_kph, amplitude_deg, frequency_hz (sine with dwell: steer gradient),
                steer_deg, steer_rate_degps (step steer / brake in turn: steer gradient),
                brake_start_s, brake_pedal (brake in turn)
    road        mu, zone_start_m (position of the friction boundary), zone_length_m, zone_mu
                (full-width low-friction zone along X; length 0: no zone)
    vehicle     the plant and periphery fields of UncertaintyCase: mass_factor, Iz_factor,
                cog_shift_m, cornering_scale_front/_rear, actuator_delay_s, actuator_tau_s,
                control_latency_s, sensor_noise_scale, gyro_bias_radps, accel_bias_mps2,
                sensor_latency_s

``fixed`` sets unsearched values (including ``actuator_failure``: ``none``, ``rear_steer_stuck``
(rear steer held at 0) or ``brake_vectoring_off`` (brake torques equalised per axle: no yaw moment
from differential braking)). ``constraints`` keep combinations physical; a sample that violates
one is not simulated:

    {type: linear, coefficients: {speed_kph: 1, amplitude_deg: 10}, max: 200, reason: ...}
    {type: lateral_demand, max_factor: 2.0}   steady-state lateral acceleration of the steer input
                                              (linear single-track estimate v^2 delta / (L + K_us v^2))
                                              at most max_factor * mu * g

**Objective and criterion.** ``objective: {kpi, sense}`` names one KPI of the
robust_benchmark set (:data:`.robust_benchmark.KPIS`: max sideslip, yaw-rate RMS error, lateral
deviation, saturation share, degraded share, fallback rate, infeasibility rate, tyre utilisation,
...) and maximises (``sense: max``) or minimises it. ``criterion: {kpi, threshold, direction}``
defines "critical" (``direction: above``: KPI >= threshold).

**Search (offline only).** ``search`` is a sequence of stages: ``sweep`` (full grid,
``points`` per parameter), ``lhs`` (Latin hypercube, ``samples``) and ``differential_evolution``
(scipy, ``maxiter``, ``popsize``; started from the best samples of the earlier stages). All
stages use ``seed``; a sample's result does not depend on the number of worker processes.

**Numerical failures are not physical failures.** A run that raises or ends non-finite is
classified ``numerical`` and never counts as critical. Every critical case is re-run (a) with
the same seed (bit-identical KPI expected), (b) at half the step and (c) with a 1000x tighter
wheel-load fixed-point tolerance; it is ``confirmed`` only if the criterion holds in (b) and (c)
and the KPI agrees within ``verification_tolerance`` (relative), else ``numerical_artefact``.

**Counterexample minimisation.** For each parameter in ``minimise`` (in order), a
bisection between the nominal and the critical value finds the value closest to nominal at
which the criterion still holds (others kept); the result is verified again.

**Collection.** Per critical case ``<out>/critical/<id>/``: ``scenario.yaml`` (a
self-contained replay specification of this type: ``replay: {values}``), ``metadata.json``
(seed, provenance, trigger, violated criterion, verification), ``timeseries.csv``,
``comparison.json`` (nominal parameters with the same controller, and the ``baseline``
controller on the critical scenario), ``timeseries.png``, ``plan_view.png``, ``scenario.gif``
and ``report.md``. A replay specification (``replay:`` instead of ``search:``) re-evaluates one
case and checks the criterion; copied to ``configs/critical/`` it is a regression test.
"""

from __future__ import annotations

import copy
import csv
import itertools
import json
import math
import os
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import yaml

from .spec import _keys, _require
from .types import provenance, report_progress, slug, write_json

KIND = "critical_scenario_search"
MANOEUVRES = {
    "sine_with_dwell": {"speed_kph", "amplitude_deg", "frequency_hz"},
    "step_steer": {"speed_kph", "steer_deg", "steer_rate_degps"},
    "brake_in_turn": {"speed_kph", "steer_deg", "steer_rate_degps", "brake_start_s", "brake_pedal"},
}
ROAD = {"mu", "zone_start_m", "zone_length_m", "zone_mu"}
PLANT = {"mass_factor", "Iz_factor", "cog_shift_m", "cornering_scale_front", "cornering_scale_rear", "actuator_delay_s",
         "actuator_tau_s", "control_latency_s", "sensor_noise_scale", "gyro_bias_radps", "accel_bias_mps2",
         "sensor_latency_s"}
FAILURES = ("none", "rear_steer_stuck", "brake_vectoring_off")
DEFAULTS = {"speed_kph": 80.0, "amplitude_deg": 6.0, "frequency_hz": 0.7, "steer_deg": 3.0, "steer_rate_degps": 200.0,
            "brake_start_s": 1.5, "brake_pedal": 0.5, "mu": 1.0, "zone_start_m": 30.0, "zone_length_m": 0.0,
            "zone_mu": 0.3, "actuator_failure": "none"}
_TOP = {"type", "name", "description", "vehicle", "tyres", "manoeuvre", "controller", "baseline", "feedback",
        "parameters", "fixed", "constraints", "objective", "criterion", "search", "minimise", "replay", "dt", "seed",
        "workers", "verification_tolerance", "outputs", "max_critical_cases"}
FORMATS = ("json", "csv", "png", "gif", "md")
NUMERICAL = "numerical"
_UNDERSTEER_GRADIENT = 0.0009   # [rad/(m/s^2)] of the steady lateral-demand estimate (as the blended observer's reference)


@dataclass(frozen=True)
class CriticalSearchSpec:
    name: str
    vehicle: str
    manoeuvre: str
    controller: dict
    parameters: tuple                     # ({name, low, high, nominal}, ...)
    objective: dict
    criterion: dict
    tyres: dict = field(default_factory=dict)
    baseline: dict | None = None
    feedback: str = "ideal"
    fixed: dict = field(default_factory=dict)
    constraints: tuple = ()
    search: tuple = ()
    minimise: tuple = ()
    replay: dict | None = None
    dt: float = 0.005
    seed: int = 0
    workers: int | None = None
    verification_tolerance: float = 0.1
    max_critical_cases: int = 3
    outputs: dict = field(default_factory=dict)
    description: str = ""
    source: str = ""
    kind = KIND

    def to_dict(self) -> dict:
        d = {"type": KIND, "name": self.name, "description": self.description, "vehicle": self.vehicle,
             "tyres": dict(self.tyres), "manoeuvre": self.manoeuvre, "controller": dict(self.controller),
             "feedback": self.feedback, "parameters": [dict(p) for p in self.parameters], "fixed": dict(self.fixed),
             "constraints": [dict(c) for c in self.constraints], "objective": dict(self.objective),
             "criterion": dict(self.criterion), "dt": self.dt, "seed": self.seed,
             "verification_tolerance": self.verification_tolerance, "outputs": dict(self.outputs)}
        if self.baseline:
            d["baseline"] = dict(self.baseline)
        if self.replay is not None:
            d["replay"] = dict(self.replay)
        else:
            d.update({"search": [dict(s) for s in self.search], "minimise": list(self.minimise),
                      "max_critical_cases": self.max_critical_cases})
        return d


# ----------------------------------------------------------------------------- parsing
def _num(x, where):
    _require(isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x), f"{where} must be a number")
    return float(x)


def parse(data: dict, source: str = "<dict>") -> CriticalSearchSpec:
    from .robust_benchmark import FEEDBACK, KPIS, LAWS
    from vehicle_model.simulation.uncertainty import UncertaintyCase

    _require(data.get("type") == KIND, f"{source}: type must be {KIND!r}")
    _keys(data, _TOP, source, {"type", "name", "vehicle", "manoeuvre", "controller", "parameters", "objective",
                               "criterion"})
    _require(Path(data["vehicle"]).is_file(), f"{source}: vehicle file not found: {data['vehicle']}")
    _require(("search" in data) != ("replay" in data), f"{source}: give exactly one of search (find cases) or replay")
    man = data["manoeuvre"]
    _require(man in MANOEUVRES, f"{source}: manoeuvre must be one of {sorted(MANOEUVRES)}")
    allowed = MANOEUVRES[man] | ROAD | PLANT
    for key in ("controller", "baseline"):
        c = data.get(key)
        if c is None:
            continue
        _require(isinstance(c, dict), f"{source}: {key} must be a mapping")
        _keys(c, {"name", "law", "inputs", "alpha", "bandwidth", "w_max_vy", "w_max_r", "supervisor"}, f"{source}: {key}",
              {"name", "law"})
        _require(c["law"] in LAWS, f"{source}: {key}.law must be one of {LAWS}")
    fb = data.get("feedback", "ideal")
    _require(fb in FEEDBACK, f"{source}: feedback must be one of {FEEDBACK}")
    params = data["parameters"]
    _require(isinstance(params, list), f"{source}: parameters must be a list")
    out_p = []
    for i, p in enumerate(params):
        where = f"{source}: parameters[{i}]"
        _require(isinstance(p, dict), f"{where} must be a mapping")
        _keys(p, {"name", "low", "high", "nominal"}, where, {"name", "low", "high", "nominal"})
        _require(p["name"] in allowed, f"{where}.name {p['name']!r} is not a parameter of {man}; allowed: {sorted(allowed)}")
        lo, hi, nom = (_num(p[k], f"{where}.{k}") for k in ("low", "high", "nominal"))
        _require(lo < hi and lo <= nom <= hi, f"{where}: needs low < high and low <= nominal <= high")
        out_p.append({"name": p["name"], "low": lo, "high": hi, "nominal": nom})
    names = [p["name"] for p in out_p]
    _require(len(set(names)) == len(names), f"{source}: parameter names must be unique")
    fixed = dict(data.get("fixed") or {})
    for k, v in fixed.items():
        _require(k in allowed or k == "actuator_failure", f"{source}: fixed.{k} is not a parameter of {man}")
        _require(k not in names, f"{source}: fixed.{k} is also searched")
        if k == "actuator_failure":
            _require(v in FAILURES, f"{source}: fixed.actuator_failure must be one of {FAILURES}")
        else:
            _num(v, f"{source}: fixed.{k}")
    cons = []
    for i, c in enumerate(data.get("constraints") or []):
        where = f"{source}: constraints[{i}]"
        _require(isinstance(c, dict) and c.get("type") in ("linear", "lateral_demand"),
                 f"{where}.type must be linear or lateral_demand")
        if c["type"] == "linear":
            _keys(c, {"type", "coefficients", "max", "reason"}, where, {"type", "coefficients", "max"})
            _require(isinstance(c["coefficients"], dict) and c["coefficients"], f"{where}.coefficients must be a mapping")
            for k, v in c["coefficients"].items():
                _require(k in allowed, f"{where}.coefficients: unknown parameter {k!r}")
                _num(v, f"{where}.coefficients.{k}")
            _num(c["max"], f"{where}.max")
        else:
            _keys(c, {"type", "max_factor", "reason"}, where, {"type", "max_factor"})
            _require(_num(c["max_factor"], f"{where}.max_factor") > 0, f"{where}.max_factor must be > 0")
        cons.append(dict(c))
    obj = dict(data["objective"])
    _keys(obj, {"kpi", "sense"}, f"{source}: objective", {"kpi"})
    _require(obj["kpi"] in KPIS, f"{source}: objective.kpi must be one of {KPIS}")
    obj.setdefault("sense", "max")
    _require(obj["sense"] in ("max", "min"), f"{source}: objective.sense must be max or min")
    crit = dict(data["criterion"])
    _keys(crit, {"kpi", "threshold", "direction"}, f"{source}: criterion", {"kpi", "threshold"})
    _require(crit["kpi"] in KPIS, f"{source}: criterion.kpi must be one of {KPIS}")
    crit["threshold"] = _num(crit["threshold"], f"{source}: criterion.threshold")
    crit.setdefault("direction", "above")
    _require(crit["direction"] in ("above", "below"), f"{source}: criterion.direction must be above or below")
    stages, minimise, replay = [], (), None
    if "search" in data:
        _require(out_p, f"{source}: a search needs at least one parameter")
        _require(isinstance(data["search"], list) and data["search"], f"{source}: search must be a non-empty list")
        for i, s in enumerate(data["search"]):
            where = f"{source}: search[{i}]"
            _require(isinstance(s, dict) and s.get("method") in ("sweep", "lhs", "differential_evolution"),
                     f"{where}.method must be sweep, lhs or differential_evolution")
            spec_keys = {"sweep": {"method", "points"}, "lhs": {"method", "samples"},
                         "differential_evolution": {"method", "maxiter", "popsize"}}[s["method"]]
            _keys(s, spec_keys, where, spec_keys)
            for k in spec_keys - {"method"}:
                _require(isinstance(s[k], int) and not isinstance(s[k], bool) and 1 <= s[k] <= 10000,
                         f"{where}.{k} must be an integer in [1, 10000]")
            stages.append(dict(s))
        minimise = tuple(data.get("minimise") or ())
        for k in minimise:
            _require(k in names, f"{source}: minimise.{k} is not a searched parameter")
    else:
        replay = dict(data["replay"])
        _keys(replay, {"values", "expect_critical"}, f"{source}: replay", {"values"})
        vals = dict(replay["values"])
        for k in names:
            _require(k in vals, f"{source}: replay.values misses {k}")
        for k, v in vals.items():
            _require(k in names, f"{source}: replay.values.{k} is not a declared parameter")
            _num(v, f"{source}: replay.values.{k}")
        replay["values"] = vals
        replay.setdefault("expect_critical", True)
    UncertaintyCase.from_dict({k: v for k, v in _values(out_p, fixed, {}).items() if k in PLANT})
    dt = _num(data.get("dt", 0.005), f"{source}: dt")
    _require(0.001 <= dt <= 0.02, f"{source}: dt must lie in [0.001, 0.02]")
    tol = _num(data.get("verification_tolerance", 0.1), f"{source}: verification_tolerance")
    _require(0 < tol < 1, f"{source}: verification_tolerance must lie in (0, 1)")
    workers = data.get("workers")
    _require(workers is None or (isinstance(workers, int) and not isinstance(workers, bool) and workers >= 1),
             f"{source}: workers must be a positive integer")
    outputs = dict(data.get("outputs") or {})
    _keys(outputs, {"directory", "formats"}, f"{source}: outputs")
    outputs.setdefault("formats", list(FORMATS))
    _require(set(outputs["formats"]) <= set(FORMATS), f"{source}: outputs.formats must be a subset of {FORMATS}")
    outputs.setdefault("directory", f"output/experiments/{data['name']}")
    return CriticalSearchSpec(str(data["name"]), str(data["vehicle"]), man, dict(data["controller"]), tuple(out_p), obj,
                              crit, dict(data.get("tyres") or {}), dict(data["baseline"]) if data.get("baseline") else None,
                              fb, fixed, tuple(cons), tuple(stages), minimise, replay, dt, int(data.get("seed", 0)),
                              workers, tol, int(data.get("max_critical_cases", 3)), outputs,
                              str(data.get("description", "")), source)


def describe(spec: CriticalSearchSpec) -> str:
    what = "replay" if spec.replay is not None else " -> ".join(s["method"] for s in spec.search)
    return (f"{spec.manoeuvre}, {spec.controller['name']}: {len(spec.parameters)} parameter(s), {what}; "
            f"critical: {spec.criterion['kpi']} {'>=' if spec.criterion['direction'] == 'above' else '<='} "
            f"{spec.criterion['threshold']}")


# ----------------------------------------------------------------------------- one evaluation
def _values(parameters, fixed, values) -> dict:
    v = dict(DEFAULTS)
    v.update({p["name"]: p["nominal"] for p in parameters})
    v.update(fixed)
    v.update(values)
    return v


def nominal_values(spec) -> dict:
    return {p["name"]: p["nominal"] for p in spec.parameters}


def constraint_violations(spec, values: dict) -> list:
    """Names of the violated constraints (``[]``: physically admissible)."""
    from vehicle_model.core.types import SevenDOFParams
    v = _values(spec.parameters, spec.fixed, values)
    out = []
    for c in spec.constraints:
        if c["type"] == "linear":
            if sum(float(k) * float(v[n]) for n, k in c["coefficients"].items()) > float(c["max"]) + 1e-12:
                out.append(c.get("reason") or f"linear {c['coefficients']} <= {c['max']}")
        else:
            p = SevenDOFParams()
            speed = float(v["speed_kph"]) / 3.6
            delta = math.radians(float(v["amplitude_deg"] if spec.manoeuvre == "sine_with_dwell" else v["steer_deg"]))
            ay = speed**2 * delta / (p.wheelbase + _UNDERSTEER_GRADIENT * speed**2)
            if ay > float(c["max_factor"]) * float(v["mu"]) * p.g:
                out.append(c.get("reason") or f"lateral demand {ay:.1f} m/s^2 > {c['max_factor']} mu g")
    return out


def scenario(spec, values: dict):
    """The manoeuvre and road for ``values`` (a :class:`~vehicle_model.simulation.controller_benchmark.Scenario`)."""
    from vehicle_model.controllers.framework import ControlTarget
    from vehicle_model.roads.surface import FrictionPatchRoad, UniformRoad
    from vehicle_model.simulation.controller_benchmark import Scenario, sine_with_dwell

    v = _values(spec.parameters, spec.fixed, values)
    v0, mu = float(v["speed_kph"]) / 3.6, float(v["mu"])
    zl = float(v["zone_length_m"])
    if zl > 0.0:
        z0, zmu = float(v["zone_start_m"]), float(v["zone_mu"])
        road = lambda: FrictionPatchRoad(default_mu=mu, patches=[(z0, z0 + zl, -1e3, 1e3, zmu)])
    else:
        road = lambda: UniformRoad(mu=mu)
    if spec.manoeuvre == "sine_with_dwell":
        steer, t_end = sine_with_dwell(math.radians(float(v["amplitude_deg"])), float(v["frequency_hz"]))
        return Scenario("sine_with_dwell", road, v0, t_end + 2.5, lambda t, s: ControlTarget(driver_steer_f=steer(t)),
                        params={"t_end": t_end, "t_start": 0.5})
    d, rate, t0 = math.radians(float(v["steer_deg"])), math.radians(float(v["steer_rate_degps"])), 0.5
    ramp = lambda t: 0.0 if t < t0 else math.copysign(min(abs(d), rate * (t - t0)), d)
    if spec.manoeuvre == "step_steer":
        return Scenario("step_steer", road, v0, 4.0,
                        lambda t, s: ControlTarget(driver_steer_f=ramp(t),
                                                   driver_accel_pedal=max(0.08 + 0.05 * (v0 - s.vx), 0.0)))
    tb, pedal = float(v["brake_start_s"]), float(v["brake_pedal"])
    return Scenario("brake_in_turn", road, v0, tb + 3.0,
                    lambda t, s: ControlTarget(driver_steer_f=ramp(t),
                                               driver_accel_pedal=0.0 if t >= tb else max(0.08 + 0.05 * (v0 - s.vx), 0.0),
                                               driver_brake_pedal=pedal if t >= tb else 0.0),
                    stop_speed=1.0)


class _ActuatorFailure:
    """``rear_steer_stuck``: rear steer held at 0; ``brake_vectoring_off``: brake torques equal per axle."""

    def __init__(self, inner, mode: str):
        self.inner, self.mode = inner, mode

    @property
    def name(self) -> str:
        return f"{getattr(self.inner, 'name', type(self.inner).__name__)} [{self.mode}]"

    def reset(self) -> None:
        self.inner.reset()

    def compute_control(self, state, target, telemetry=None, dt: float = 0.01):
        out = self.inner.compute_control(state, target, telemetry, dt)
        if self.mode == "rear_steer_stuck":
            out.steer_r = 0.0
        elif self.mode == "brake_vectoring_off":
            b = np.asarray(out.T_brake, float)
            out.T_brake = np.array([b[:2].mean()] * 2 + [b[2:].mean()] * 2)
        return out


class _BenchSpec:
    """The fields of a robust_benchmark spec that its controller factory reads."""

    def __init__(self, spec, mu: float):
        self.vehicle, self.tyres, self.road, self.seed = spec.vehicle, spec.tyres, {"type": "uniform", "mu": mu}, spec.seed


def evaluate(spec, values: dict, controller: dict | None = None, dt: float | None = None,
             load_transfer_tol: float | None = None, keep_history: bool = False) -> dict:
    """One simulation: KPIs, objective value and class (``critical`` / ``nominal`` / ``numerical`` / ``rejected``)."""
    from vehicle_model.simulation.controller_benchmark import run_scenario
    from vehicle_model.simulation.uncertainty import UncertaintyCase, plant_factory

    from .robust_benchmark import _nominal, build_controller, kpis

    v = _values(spec.parameters, spec.fixed, values)
    res = {"values": {k: float(values[k]) for k in values}, "objective": float("nan"), "kpis": {}, "error": ""}
    bad = constraint_violations(spec, values)
    if bad:
        return {**res, "class": "rejected", "constraints_violated": bad}
    dt = dt or spec.dt
    c = controller or spec.controller
    case = UncertaintyCase.from_dict({"name": "search", **{k: v[k] for k in PLANT if k in v}})
    bench = _BenchSpec(spec, float(v["mu"]))
    try:
        nominal = _nominal(bench)
        build = plant_factory(nominal, case)

        def plant(road):
            m = build(road)
            if load_transfer_tol is not None:
                m.load_transfer_tol = load_transfer_tol
                m.load_transfer_max_iter = 200
            return m
        ctrl_factory = build_controller(bench, c, spec.feedback, case)
        failure = v.get("actuator_failure", "none")
        factory = ctrl_factory if failure == "none" else (lambda p: _ActuatorFailure(ctrl_factory(p), failure))
        run = run_scenario(plant, factory, scenario(spec, values), dt)
        h = run.history
        finite = all(np.all(np.isfinite(np.asarray(h[k], float))) for k in ("X", "Y", "vx", "r", "beta"))
        if not finite:
            raise FloatingPointError("non-finite vehicle state")
        k = kpis(h, dt)
        k.pop("supervisor_modes", None)
        res["kpis"] = k
        x = float(k[spec.objective["kpi"]])
        res["objective"] = x if spec.objective["sense"] == "max" else -x
        res["class"] = "critical" if is_critical(spec, k) else "nominal"
        if keep_history:
            res["history"] = {key: np.asarray(h[key], float) for key in ("t", "X", "Y", "psi", "vx", "r", "r_ref", "beta",
                                                                          "steer_f", "steer_r", "Mz_allocated", "speed",
                                                                          "tyre_utilisation")}
    except Exception as ex:   # solver / numerics: reported, never counted as a physical failure
        res.update({"class": NUMERICAL, "error": f"{type(ex).__name__}: {ex}"})
    return res


def is_critical(spec, k: dict) -> bool:
    x, c = float(k[spec.criterion["kpi"]]), spec.criterion
    if not math.isfinite(x):
        return False
    return x >= c["threshold"] if c["direction"] == "above" else x <= c["threshold"]


def _eval_task(args):
    spec, values = args
    return evaluate(spec, values)


# ----------------------------------------------------------------------------- search
def _to_values(spec, unit_row) -> dict:
    return {p["name"]: p["low"] + float(u) * (p["high"] - p["low"]) for p, u in zip(spec.parameters, unit_row)}


def _map(pool, tasks):
    return list(pool.map(_eval_task, tasks)) if pool is not None else [_eval_task(t) for t in tasks]


def search(spec, pool=None) -> list:
    """All evaluations of all stages, in order; each carries its ``stage``."""
    from scipy.optimize import differential_evolution
    from scipy.stats import qmc

    d = len(spec.parameters)
    evals = []
    for si, st in enumerate(spec.search):
        report_progress(si, len(spec.search), st["method"])
        if st["method"] == "sweep":
            grid = np.linspace(0.0, 1.0, st["points"])
            rows = list(itertools.product(grid, repeat=d))
        elif st["method"] == "lhs":
            rows = qmc.LatinHypercube(d=d, seed=spec.seed + si).random(st["samples"])
        else:
            pop = st["popsize"] * d
            seen = sorted((e for e in evals if np.isfinite(e["objective"])), key=lambda e: -e["objective"])[:pop]
            init = np.array([[(e["values"][p["name"]] - p["low"]) / (p["high"] - p["low"]) for p in spec.parameters]
                             for e in seen]) if seen else None
            if init is None or len(init) < 5:
                init = "latinhypercube"
            stage = []

            def vec_map(func, xs):
                rows_ = [np.clip(x, 0.0, 1.0) for x in xs]
                out = _map(pool, [(spec, _to_values(spec, r)) for r in rows_])
                for o in out:
                    o["stage"] = f"{si}:differential_evolution"
                stage.extend(out)
                return [-(o["objective"]) if np.isfinite(o["objective"]) else 1e6 for o in out]

            differential_evolution(_noop, [(0.0, 1.0)] * d, maxiter=st["maxiter"], popsize=st["popsize"], init=init,
                                   seed=spec.seed + si, polish=False, updating="deferred", workers=vec_map, tol=0.0)
            evals.extend(stage)
            continue
        out = _map(pool, [(spec, _to_values(spec, r)) for r in rows])
        for o in out:
            o["stage"] = f"{si}:{st['method']}"
        evals.extend(out)
    return evals


def _noop(x):   # pragma: no cover - differential_evolution evaluates through ``workers`` (vec_map)
    raise RuntimeError("evaluated through the workers map")


# ----------------------------------------------------------------------------- verification and minimisation
def verify(spec, values: dict, reference: dict | None = None) -> dict:
    """Re-run with the same seed, at dt/2 and with a 1000x tighter load-transfer tolerance."""
    ref = reference or evaluate(spec, values)
    kpi = spec.criterion["kpi"]
    same = evaluate(spec, values)
    half = evaluate(spec, values, dt=spec.dt / 2.0)
    tight = evaluate(spec, values, load_transfer_tol=1e-9)
    x0 = ref["kpis"].get(kpi, float("nan"))
    checks = {}
    for name, r in (("same_seed", same), ("half_step", half), ("tight_load_transfer", tight)):
        x = r["kpis"].get(kpi, float("nan"))
        rel = abs(x - x0) / max(abs(x0), 1e-9) if math.isfinite(x) and math.isfinite(x0) else float("inf")
        checks[name] = {"class": r["class"], kpi: x, "relative_change": rel, "error": r["error"]}
    reproducible = checks["same_seed"]["relative_change"] == 0.0
    numerics_ok = all(checks[n]["class"] == "critical" and checks[n]["relative_change"] <= spec.verification_tolerance
                      for n in ("half_step", "tight_load_transfer"))
    verdict = ("confirmed" if ref["class"] == "critical" and reproducible and numerics_ok else
               "not_critical" if ref["class"] != "critical" else "numerical_artefact")
    return {"verdict": verdict, "reproducible": reproducible, "checks": checks, "kpi": kpi, "value": x0}


def robustly_critical(spec, values: dict) -> bool:
    """Critical at the step ``dt`` and at ``dt / 2`` with the criterion KPI within the verification tolerance."""
    a = evaluate(spec, values)
    if a["class"] != "critical":
        return False
    b = evaluate(spec, values, dt=spec.dt / 2.0)
    k = spec.criterion["kpi"]
    x, y = a["kpis"][k], b["kpis"].get(k, float("nan"))
    return b["class"] == "critical" and abs(y - x) <= spec.verification_tolerance * max(abs(x), 1e-9)


def minimise(spec, values: dict, iterations: int = 6, passes: int = 3) -> tuple[dict, list]:
    """Move each ``minimise`` parameter towards nominal as far as the case stays critical (bisection).

    Repeated (up to ``passes``) until no parameter moves: a boundary bisected while the later parameters
    still had their critical values shifts once they are back at nominal.

    A point counts as critical here only if it is **robustly** critical (:func:`robustly_critical`: also at
    half the step, KPI within ``verification_tolerance``). Measured without this: the bisection
    walks onto the stability boundary, where the outcome depends on the step (actuator delay rounded to
    whole control periods, controller rate) and the minimal case fails its own verification.
    """
    cur = dict(values)
    trace = []
    for _ in range(passes):
        before = dict(cur)
        cur = _minimise_pass(spec, cur, iterations, trace)
        if cur == before:
            break
    return cur, trace


def _minimise_pass(spec, cur: dict, iterations: int, trace: list) -> dict:
    cur = dict(cur)
    for name in spec.minimise:
        p = next(q for q in spec.parameters if q["name"] == name)
        crit_v, nom_v = cur[name], p["nominal"]
        if crit_v == nom_v:
            continue
        trial = {**cur, name: nom_v}
        ok = robustly_critical(spec, trial)
        trace.append({"parameter": name, "value": nom_v, "class": "critical" if ok else "not_robustly_critical"})
        if ok:
            cur = trial
            continue
        lo, hi = nom_v, crit_v            # lo: not critical, hi: critical
        for _ in range(iterations):
            mid = 0.5 * (lo + hi)
            ok = robustly_critical(spec, {**cur, name: mid})
            trace.append({"parameter": name, "value": mid, "class": "critical" if ok else "not_robustly_critical"})
            if ok:
                hi = mid
            else:
                lo = mid
        cur[name] = hi
    return cur


# ----------------------------------------------------------------------------- run
def run(spec: CriticalSearchSpec, out: Path, verify_model: bool | None = None) -> dict:
    """Search (or replay); ``verify_model`` is accepted for the common interface: the model checks of this
    type are the per-case re-runs (same seed, half step, tight load-transfer tolerance)."""
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    if spec.replay is not None:
        return _replay(spec, out)
    workers = spec.workers or max(1, min(8, (os.cpu_count() or 2) - 1))
    if workers > 1:
        with ProcessPoolExecutor(max_workers=workers) as pool:
            evals = search(spec, pool)
    else:
        evals = search(spec)
    counts = {c: sum(e["class"] == c for e in evals) for c in ("critical", "nominal", NUMERICAL, "rejected")}
    ranked = sorted((e for e in evals if e["class"] == "critical"), key=lambda e: -e["objective"])
    import shutil
    shutil.rmtree(out / "critical", ignore_errors=True)       # this run's own case folders only
    cases, seen, duplicates = [], [], []
    for e in ranked[:3 * spec.max_critical_cases]:
        if len(cases) >= spec.max_critical_cases:
            break
        # distinct cases are distinct after minimisation (5 % of each range)
        minimal, trace = minimise(spec, e["values"]) if spec.minimise else (dict(e["values"]), [])
        key = tuple(round(20 * (minimal[p["name"]] - p["low"]) / (p["high"] - p["low"])) for p in spec.parameters)
        if key in seen:
            duplicates.append({"trigger_values": e["values"], "minimal_values": minimal})
            continue
        seen.append(key)
        cases.append(_case(spec, e, out, len(cases), minimal, trace))
    nominal = evaluate(spec, nominal_values(spec))
    summary = {"experiment": spec.to_dict(), "source": spec.source, "provenance": provenance(), "verification": [],
               "evaluations": len(evals), "counts": counts, "nominal": nominal, "critical_cases": cases,
               "duplicates_after_minimisation": duplicates,
               "results": [{k: e[k] for k in ("stage", "values", "class", "objective", "error")} for e in evals]}
    formats = set(spec.outputs.get("formats", ()))
    if "json" in formats:
        write_json(out / "results.json", summary)
    if "csv" in formats:
        names = [p["name"] for p in spec.parameters]
        with open(out / "evaluations.csv", "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["stage", *names, "class", spec.objective["kpi"], "error"])
            for e in evals:
                w.writerow([e["stage"], *(e["values"][n] for n in names), e["class"],
                            e["kpis"].get(spec.objective["kpi"], ""), e["error"]])
    if "png" in formats:
        _plot_search(spec, evals, cases, out)
    if "md" in formats:
        (out / "report.md").write_text(_report(spec, summary), encoding="utf-8")
    return summary


def _replay_spec(spec, values: dict, name: str) -> dict:
    d = spec.to_dict()
    for k in ("search", "minimise", "max_critical_cases"):
        d.pop(k, None)
    d["name"] = name
    d["replay"] = {"values": {k: float(v) for k, v in values.items()}, "expect_critical": True}
    d["outputs"] = {"directory": f"output/experiments/{name}", "formats": ["json", "md"]}
    d["workers"] = 1
    return d


def _case(spec, found: dict, out: Path, index: int, minimal: dict, trace: list) -> dict:
    trigger = found["values"]
    v_found = verify(spec, trigger, found)
    final = evaluate(spec, minimal, keep_history=True)
    v_min = verify(spec, minimal, final) if final["class"] == "critical" else {"verdict": "not_critical"}
    cid = f"{index + 1:02d}_{slug(spec.criterion['kpi'])}"
    d = out / "critical" / cid
    d.mkdir(parents=True, exist_ok=True)
    nominal = evaluate(spec, nominal_values(spec), keep_history=True)
    base = evaluate(spec, minimal, controller=spec.baseline, keep_history=True) if spec.baseline else None
    comparison = {"nominal_parameters": _brief(nominal), "baseline_controller": _brief(base) if base else None,
                  "baseline_name": spec.baseline["name"] if spec.baseline else None}
    case = {"id": cid, "trigger_values": trigger, "trigger_class": found["class"], "trigger_stage": found.get("stage"),
            "trigger_kpis": found["kpis"], "minimal_values": minimal, "minimal_kpis": final["kpis"],
            "violated_criterion": dict(spec.criterion), "verification_trigger": v_found, "verification_minimal": v_min,
            "minimisation_trace": trace, "seed": spec.seed, "comparison": comparison,
            "replay_spec": f"critical/{cid}/scenario.yaml"}
    formats = set(spec.outputs.get("formats", ()))
    (d / "scenario.yaml").write_text(yaml.safe_dump(_replay_spec(spec, minimal, f"{spec.name}_{cid}"), sort_keys=False),
                                     encoding="utf-8")
    write_json(d / "metadata.json", {**{k: v for k, v in case.items() if k != "comparison"}, "provenance": provenance(),
                                     "model": "SevenDOFModel (7-DOF two-track), synthetic parameters"})
    write_json(d / "comparison.json", comparison)
    if "csv" in formats and "history" in final:
        _write_history(d / "timeseries.csv", final["history"])
    if "png" in formats and "history" in final:
        _plot_case(spec, final, nominal, base, d)
    if "gif" in formats and "history" in final:
        _gif_case(spec, final, nominal, d / "scenario.gif")
    (d / "report.md").write_text(_case_report(spec, case), encoding="utf-8")
    return case


def _brief(r: dict | None) -> dict | None:
    if r is None:
        return None
    return {"class": r["class"], "kpis": r["kpis"], "error": r["error"]}


def _replay(spec, out: Path) -> dict:
    r = evaluate(spec, spec.replay["values"])
    v = verify(spec, spec.replay["values"], r) if r["class"] == "critical" else {"verdict": r["class"]}
    expected = bool(spec.replay.get("expect_critical", True))
    ok = (v["verdict"] == "confirmed") == expected
    summary = {"experiment": spec.to_dict(), "source": spec.source, "provenance": provenance(), "verification": [],
               "results": [{"values": r["values"], "class": r["class"], "kpis": r["kpis"], "error": r["error"]}],
               "replay": {"verdict": v["verdict"], "expected_critical": expected, "passed": ok, "checks": v.get("checks")}}
    formats = set(spec.outputs.get("formats", ()))
    if "json" in formats:
        write_json(out / "results.json", summary)
    if "md" in formats:
        k = spec.criterion["kpi"]
        (out / "report.md").write_text(
            f"# Replay {spec.name}\n\nCriterion {k} {spec.criterion['direction']} {spec.criterion['threshold']}: value "
            f"{r['kpis'].get(k, float('nan')):.4g}, class {r['class']}, verdict {v['verdict']} "
            f"({'as expected' if ok else 'NOT as expected'}).\n", encoding="utf-8")
    return summary


# ----------------------------------------------------------------------------- outputs
def _write_history(path: Path, h: dict) -> None:
    keys = list(h)
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(keys)
        for row in zip(*(h[k] for k in keys)):
            w.writerow([float(x) for x in row])


def _plot_search(spec, evals, cases, out: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from vehicle_model.visualization.theme import apply_dark_theme, style_axis
    apply_dark_theme()
    ps = spec.parameters
    pairs = list(itertools.combinations(range(len(ps)), 2))[:5] or [(0, 0)]
    cols = 3 if len(pairs) > 2 else len(pairs) + 1
    rows = math.ceil((len(pairs) + 1) / cols)
    fig, axes = plt.subplots(rows, cols, figsize=(4.6 * cols, 4.0 * rows), squeeze=False)
    for ax in axes.flat[len(pairs) + 1:]:
        ax.axis("off")
    axes = axes.flatten()
    obj = spec.objective["kpi"]
    k = np.array([e["kpis"].get(obj, np.nan) for e in evals], float)
    ax = axes[0]
    ax.plot(np.arange(len(k)), k, ".", color="#7d8590", ms=4, label="evaluation")
    ax.plot(np.arange(len(k)), np.fmax.accumulate(np.nan_to_num(k, nan=-np.inf)) if spec.objective["sense"] == "max"
            else np.fmin.accumulate(np.nan_to_num(k, nan=np.inf)), color="#3987e5", lw=2, label="best so far")
    ax.axhline(spec.criterion["threshold"], color="#d95926", ls="--", lw=1, label="criterion")
    ax.legend(fontsize=7)
    style_axis(ax, "search progress", "evaluation", obj)
    cls_col = {"critical": "#d95926", "nominal": "#3987e5", NUMERICAL: "#c98500", "rejected": "#555555"}
    for ax, (i, j) in zip(axes[1:], pairs):
        for c, col in cls_col.items():
            sel = [e for e in evals if e["class"] == c]
            if sel:
                ax.plot([e["values"][ps[i]["name"]] for e in sel], [e["values"][ps[j]["name"]] for e in sel], "o",
                        color=col, ms=3.5, alpha=0.7, label=c)
        for cs in cases:
            ax.plot(cs["minimal_values"][ps[i]["name"]], cs["minimal_values"][ps[j]["name"]], "*", color="#f5f5f5", ms=12)
        ax.plot(ps[i]["nominal"], ps[j]["nominal"], "P", color="#199e70", ms=10, label="nominal")
        style_axis(ax, f"{ps[i]['name']} x {ps[j]['name']}", ps[i]["name"], ps[j]["name"])
    axes[1].legend(fontsize=7) if len(axes) > 1 else None
    fig.suptitle(f"{spec.name}: {spec.controller['name']} (stars: minimal critical cases)")
    fig.tight_layout()
    fig.savefig(out / "search.png", dpi=110)
    plt.close(fig)


def _plot_case(spec, crit, nominal, base, d: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from vehicle_model.visualization.theme import TEXT_MUTED, apply_dark_theme, style_axis
    apply_dark_theme()
    runs = [("critical (minimal)", crit, "#d95926"), ("nominal parameters", nominal, "#3987e5")]
    if base is not None and "history" in base:
        runs.append((f"baseline {spec.baseline['name']}, critical", base, "#199e70"))
    fig, axes = plt.subplots(2, 2, figsize=(13, 7.5), sharex=True)
    for lab, r, col in runs:
        h = r["history"]
        axes[0, 0].plot(h["t"], np.degrees(h["r"]), color=col, lw=1.4, label=lab)
        axes[0, 0].plot(h["t"], np.degrees(h["r_ref"]), color=col, lw=0.8, ls=":")
        axes[0, 1].plot(h["t"], np.degrees(h["beta"]), color=col, lw=1.4)
        axes[1, 0].plot(h["t"], np.degrees(h["steer_f"]), color=col, lw=1.2)
        axes[1, 0].plot(h["t"], np.degrees(h["steer_r"]), color=col, lw=0.9, ls="--")
        axes[1, 1].plot(h["t"], h["Mz_allocated"] / 1e3, color=col, lw=1.2)
    if spec.criterion["kpi"] == "max_sideslip_deg":
        for s in (1, -1):
            axes[0, 1].axhline(s * spec.criterion["threshold"], color=TEXT_MUTED, ls="--", lw=1)
    axes[0, 0].legend(fontsize=7)
    style_axis(axes[0, 0], "yaw rate (dotted: reference)", None, "r [deg/s]")
    style_axis(axes[0, 1], "sideslip", None, "beta [deg]")
    style_axis(axes[1, 0], "steer front (solid), rear (dashed)", "time [s]", "delta [deg]")
    style_axis(axes[1, 1], "allocated yaw moment", "time [s]", "M_z [kN m]")
    fig.suptitle(f"{spec.name}: minimal critical case vs nominal (synthetic parameters, virtual model)")
    fig.tight_layout()
    fig.savefig(d / "timeseries.png", dpi=110)
    plt.close(fig)
    fig, ax = plt.subplots(figsize=(11, 5))
    for lab, r, col in runs:
        ax.plot(r["history"]["X"], r["history"]["Y"], color=col, lw=1.5, label=lab)
    _zone(spec, crit["values"], ax)
    ax.set_aspect("equal", adjustable="datalim")
    ax.legend(fontsize=7)
    style_axis(ax, "CoG paths", "X [m]", "Y [m]")
    fig.tight_layout()
    fig.savefig(d / "plan_view.png", dpi=110)
    plt.close(fig)


def _zone(spec, values, ax) -> None:
    v = _values(spec.parameters, spec.fixed, values)
    if float(v["zone_length_m"]) > 0.0:
        ax.axvspan(float(v["zone_start_m"]), float(v["zone_start_m"]) + float(v["zone_length_m"]), color="#5a7fa8",
                   alpha=0.18, label=f"low-friction zone mu {float(v['zone_mu']):.2f}")


def _gif_case(spec, crit, nominal, path: Path, fps: int = 10) -> None:
    """Critical against nominal run in the platform's one plan-view renderer."""
    from vehicle_model.visualization.run_animation import animate_runs
    animate_runs([("critical", crit["history"]), ("nominal", nominal["history"])], path, f"critical scenario: {spec.name}", fps=fps)


def _fmt_vals(vals: dict) -> str:
    return ", ".join(f"{k} {v:.3g}" for k, v in vals.items())


def _report(spec, s) -> str:
    c = spec.criterion
    L = [f"# Critical scenario search: {spec.name}", "",
         f"Controller **{spec.controller['name']}** ({spec.controller['law']}, feedback {spec.feedback}) on "
         f"{spec.manoeuvre}. Objective: {spec.objective['sense']} {spec.objective['kpi']}; critical: {c['kpi']} "
         f"{'>=' if c['direction'] == 'above' else '<='} {c['threshold']}. Offline search, seed {spec.seed}; synthetic "
         "parameters, virtual model only.", "",
         "| parameter | low | high | nominal |", "|---|---|---|---|"]
    L += [f"| {p['name']} | {p['low']:g} | {p['high']:g} | {p['nominal']:g} |" for p in spec.parameters]
    n = s["counts"]
    L += ["", f"Evaluations: {s['evaluations']} ({' -> '.join(st['method'] for st in spec.search)}): "
          f"{n['critical']} critical, {n['nominal']} not critical, {n['rejected']} rejected by constraints, "
          f"{n[NUMERICAL]} numerical failures (not counted as physical).", "",
          f"Nominal parameters: {c['kpi']} = {s['nominal']['kpis'].get(c['kpi'], float('nan')):.3g} ({s['nominal']['class']}).",
          "", "## Critical cases", ""]
    if not s["critical_cases"]:
        L.append("None found in the searched space.")
    if s.get("duplicates_after_minimisation"):
        L += [f"{len(s['duplicates_after_minimisation'])} further critical sample(s) minimised to an already listed "
              "case (not repeated).", ""]
    for cs in s["critical_cases"]:
        vt, vm = cs["verification_trigger"], cs["verification_minimal"]
        L += [f"### {cs['id']}", "",
              f"- found ({cs['trigger_stage']}): {_fmt_vals(cs['trigger_values'])}; {c['kpi']} "
              f"{cs['trigger_kpis'].get(c['kpi'], float('nan')):.3g}; verification: {vt['verdict']}",
              f"- minimal: {_fmt_vals(cs['minimal_values'])}; {c['kpi']} "
              f"{cs['minimal_kpis'].get(c['kpi'], float('nan')):.3g}; verification: {vm['verdict']}",
              f"- replay: `{cs['replay_spec']}`", ""]
    return "\n".join(L)


def _case_report(spec, cs) -> str:
    c, k = spec.criterion, spec.criterion["kpi"]
    vm = cs["verification_minimal"]
    cmp_ = cs["comparison"]
    L = [f"# Critical case {cs['id']} ({spec.name})", "",
         f"Controller {spec.controller['name']} ({spec.controller['law']}, feedback {spec.feedback}), manoeuvre "
         f"{spec.manoeuvre}, seed {cs['seed']}. Synthetic parameters, virtual 7-DOF model only.", "",
         f"**Violated criterion:** {k} {'>=' if c['direction'] == 'above' else '<='} {c['threshold']}.", "",
         f"**Trigger (found by {cs['trigger_stage']}):** {_fmt_vals(cs['trigger_values'])} -> {k} "
         f"{cs['trigger_kpis'].get(k, float('nan')):.4g}.", "",
         f"**Minimal case:** {_fmt_vals(cs['minimal_values'])} -> {k} {cs['minimal_kpis'].get(k, float('nan')):.4g}.", "",
         "**Numerical diagnosis:** " + (", ".join(f"{n}: {ch['class']}, {k} {ch[k]:.4g} (rel. change {ch['relative_change']:.2%})"
                                         for n, ch in vm.get("checks", {}).items()) or "-")
         + f" -> **{vm['verdict']}**.", "",
         "**Comparison:** nominal parameters, same controller: "
         f"{cmp_['nominal_parameters']['class']}, {k} {cmp_['nominal_parameters']['kpis'].get(k, float('nan')):.4g}."]
    if cmp_["baseline_controller"]:
        b = cmp_["baseline_controller"]
        L[-1] += f" Baseline {cmp_['baseline_name']} on the minimal case: {b['class']}, {k} {b['kpis'].get(k, float('nan')):.4g}."
    L += ["", "| KPI | minimal case |", "|---|---|"]
    L += [f"| {kk} | {vv:.4g} |" for kk, vv in cs["minimal_kpis"].items() if isinstance(vv, float)]
    L += ["", "Files: scenario.yaml (replay), metadata.json, comparison.json, timeseries.csv, timeseries.png, "
          "plan_view.png, scenario.gif.", ""]
    return "\n".join(L)


__all__ = ("CriticalSearchSpec", "constraint_violations", "describe", "evaluate", "is_critical", "minimise", "parse", "robustly_critical",
           "run", "scenario", "search", "verify")
