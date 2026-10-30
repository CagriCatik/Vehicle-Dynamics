"""Experiment type ``hill_descent_benchmark``: Hill Descent Control on graded roads (Plan V4.0 M22, 7-DOF).

Scenarios (a synthetic grade x surface matrix plus named cases) x controllers, each run on the 7-DOF model with a
:class:`~vehicle_model.roads.surface.GradedRoad` (gravity in the plant) through the shared MotionController chain::

    type: hill_descent_benchmark
    name: ...
    vehicle: configs/vehicles/seven_dof_passenger.yaml
    matrix: {grades_percent: [5, 10, 20], surfaces: {dry: 1.0, ice: 0.12}, speed_kph: 10, duration: 20}
    scenarios:                                  # optional named cases
      - {name: grade_change, profile: [[15, -5], [50, -20]], surface: {mu: 1.0}}
      - {name: split_mu, profile: [[15, -10]], surface: {mu_left: 1.0, mu_right: 0.2}}
      - {name: driver_brakes, profile: [[15, -10]], surface: {mu: 1.0}, driver: [{t0: 8, t1: 11, brake_pedal: 0.15}]}
    controllers:
      - {name: HDC off, hdc: {enabled: false}}
      - {name: HDC, hdc: {v_set_kph: 10}}
      - {name: HDC + actuator dynamics, hdc: {}, actuators: dynamic}
      - {name: HDC, RR brake failed, hdc: {}, brake_failed: [RR]}

``profile``: ``[x_start_m, grade_percent]`` pairs (negative = downhill in the driving direction), level before the
first, 8 m transitions. Matrix scenarios: level 15 m, then the grade. The driver steers straight (no correction).

KPIs: maximum and final speed, speed RMS error from 10 s (after the slope entry), overshoot over the set speed, maximum wheel slip,
maximum brake deceleration (tyre forces), ABS share, friction brake energy, time and first time ``infeasible``,
HDC status counts, lateral drift and heading change.
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
from .types import provenance, write_json

KIND = "hill_descent_benchmark"
HDC_KEYS = {"enabled", "v_set_kph", "max_decel_g", "kp", "ki"}
KPIS = ("v_max_kph", "v_end_kph", "speed_rms_error_kph", "overshoot_kph", "max_abs_slip", "max_brake_decel",
        "abs_share", "brake_energy_kj", "regen_energy_kj", "infeasible_s", "t_infeasible", "max_lateral_drift_m", "max_heading_deg",
        "distance_m")
HIST = ("t", "X", "Y", "psi", "speed", "ax", "max_abs_slip", "abs_active_n", "T_brake_total", "friction_power_w", "regen_power_w",
        "r", "beta", *(f"F{q}_{w}" for q in ("x", "z") for w in ("FL", "FR", "RL", "RR")))


@dataclass(frozen=True)
class HDCSpec:
    name: str
    vehicle: str
    scenarios: tuple
    controllers: tuple
    dt: float = 0.005
    workers: int | None = None
    outputs: dict = field(default_factory=dict)
    description: str = ""
    source: str = ""
    kind = KIND

    def to_dict(self) -> dict:
        return {"type": KIND, "name": self.name, "description": self.description, "vehicle": self.vehicle, "dt": self.dt,
                "scenarios": [dict(s) for s in self.scenarios], "controllers": [dict(c) for c in self.controllers],
                "outputs": dict(self.outputs)}


def _num(x, lo, hi, where):
    _require(isinstance(x, (int, float)) and not isinstance(x, bool) and lo <= x <= hi, f"{where} must lie in [{lo}, {hi}]")


def _scenario(s, where):
    _keys(s, {"name", "profile", "surface", "speed_kph", "duration", "driver"}, where, {"name", "profile", "surface"})
    _require(isinstance(s["profile"], list) and s["profile"], f"{where}.profile must be a non-empty list of [x_m, grade_percent]")
    xs = []
    for k, seg in enumerate(s["profile"]):
        _require(isinstance(seg, (list, tuple)) and len(seg) == 2, f"{where}.profile[{k}] must be [x_m, grade_percent]")
        _num(seg[0], 0.0, 5000.0, f"{where}.profile[{k}][0]")
        _num(seg[1], -40.0, 40.0, f"{where}.profile[{k}][1]")
        xs.append(seg[0])
    _require(xs == sorted(xs), f"{where}.profile must be sorted by x")
    sf = s["surface"]
    _require(isinstance(sf, dict) and (set(sf) == {"mu"} or set(sf) == {"mu_left", "mu_right"}),
             f"{where}.surface must be {{mu}} or {{mu_left, mu_right}}")
    for k, v in sf.items():
        _num(v, 0.05, 1.5, f"{where}.surface.{k}")
    _num(s.get("speed_kph", 10.0), 3.0, 60.0, f"{where}.speed_kph")
    _num(s.get("duration", 20.0), 2.0, 120.0, f"{where}.duration")
    for k, d in enumerate(s.get("driver", [])):
        w = f"{where}.driver[{k}]"
        _keys(d, {"t0", "t1", "brake_pedal", "accel_pedal"}, w, {"t0", "t1"})
        _require(0.0 <= d["t0"] < d["t1"], f"{w}: need 0 <= t0 < t1")
        for p in ("brake_pedal", "accel_pedal"):
            if p in d:
                _num(d[p], 0.0, 1.0, f"{w}.{p}")


def parse(data: dict, source: str = "<dict>") -> HDCSpec:
    _require(data.get("type") == KIND, f"{source}: type must be {KIND!r}")
    _keys(data, {"type", "name", "description", "vehicle", "matrix", "scenarios", "controllers", "dt", "workers", "outputs"},
          source, {"type", "name", "vehicle", "controllers"})
    _require(Path(data["vehicle"]).is_file(), f"{source}: vehicle file not found: {data['vehicle']}")
    scen = []
    m = data.get("matrix")
    if m is not None:
        _keys(m, {"grades_percent", "surfaces", "speed_kph", "duration"}, f"{source}: matrix", {"grades_percent", "surfaces"})
        _require(isinstance(m["surfaces"], dict) and m["surfaces"], f"{source}: matrix.surfaces must map name -> mu")
        for sname, mu in m["surfaces"].items():
            for g in m["grades_percent"]:
                _num(g, 0.0, 40.0, f"{source}: matrix.grades_percent (downhill magnitude)")
                scen.append({"name": f"{sname}_{g:g}pct", "profile": [[15.0, -float(g)]], "surface": {"mu": mu},
                             "speed_kph": m.get("speed_kph", 10.0), "duration": m.get("duration", 20.0)})
    scen += [dict(s) for s in data.get("scenarios") or []]
    _require(scen, f"{source}: need a matrix or scenarios")
    for i, s in enumerate(scen):
        _scenario(s, f"{source}: scenarios[{i}]")
    ctrls = data["controllers"]
    _require(isinstance(ctrls, list) and ctrls, f"{source}: controllers must be a non-empty list")
    for key, items in (("scenarios", scen), ("controllers", ctrls)):
        names = [x.get("name") for x in items]
        _require(len(set(names)) == len(names), f"{source}: {key} need unique names")
    for i, c in enumerate(ctrls):
        w = f"{source}: controllers[{i}]"
        _keys(c, {"name", "hdc", "actuators", "brake_failed"}, w, {"name", "hdc"})
        _keys(c["hdc"], HDC_KEYS, f"{w}.hdc")
        _require(isinstance(c["hdc"].get("enabled", True), bool), f"{w}.hdc.enabled must be true or false")
        for k, lo, hi in (("v_set_kph", 3.0, 40.0), ("max_decel_g", 0.05, 1.0), ("kp", 0.0, 20.0), ("ki", 0.0, 20.0)):
            if k in c["hdc"]:
                _num(c["hdc"][k], lo, hi, f"{w}.hdc.{k}")
        _require(c.get("actuators", "ideal") in ("ideal", "dynamic"), f"{w}.actuators must be ideal or dynamic")
        _require(set(c.get("brake_failed", [])) <= {"FL", "FR", "RL", "RR"}, f"{w}.brake_failed: wheel names FL, FR, RL, RR")
    dt = float(data.get("dt", 0.005))
    _require(0.0 < dt <= 0.01, f"{source}: dt must lie in (0, 0.01] s")
    outputs = dict(data.get("outputs") or {})
    _keys(outputs, {"directory", "formats"}, f"{source}: outputs")
    outputs.setdefault("formats", ["csv", "json", "md"])
    outputs.setdefault("directory", f"output/experiments/{data['name']}")
    return HDCSpec(str(data["name"]), str(data["vehicle"]), tuple(scen), tuple(dict(c) for c in ctrls), dt,
                   data.get("workers"), outputs, str(data.get("description", "")), source)


def describe(spec: HDCSpec) -> str:
    return f"hill descent: {len(spec.scenarios)} scenario(s) x {len(spec.controllers)} controller(s)"


def make_road(s: dict):
    from vehicle_model.roads.surface import GradedRoad, MuSplitRoad, UniformRoad
    sf = s["surface"]
    base = UniformRoad(mu=float(sf["mu"])) if "mu" in sf else MuSplitRoad(
        mu_left=float(sf["mu_left"]), mu_right=float(sf["mu_right"]), split_y=0.0, transition_width=0.05)
    return GradedRoad(base, tuple((float(x), float(g) / 100.0) for x, g in s["profile"]), 8.0)


def _hdc_config(h: dict):
    from vehicle_model.controllers.hill_descent import HDCConfig
    kw = {"enabled": bool(h.get("enabled", True))}
    if "v_set_kph" in h:
        kw["v_set"] = float(h["v_set_kph"]) / 3.6
    if "max_decel_g" in h:
        kw["max_decel"] = float(h["max_decel_g"]) * 9.81
    for k in ("kp", "ki"):
        if k in h:
            kw[k] = float(h[k])
    return HDCConfig(**kw)


def controller_factory(c: dict):
    from vehicle_model.allocation import ActuatorHealth
    from vehicle_model.controllers.motion_control import MotionController, MotionControllerConfig
    from vehicle_model.controllers.yaw_laws import NoYawLaw
    failed = set(c.get("brake_failed", []))
    health = ActuatorHealth(brake_ok=tuple(w not in failed for w in ("FL", "FR", "RL", "RR"))) if failed else None
    cfg = MotionControllerConfig(hdc=_hdc_config(c["hdc"]))

    def build(p):
        ctrl = MotionController(p, NoYawLaw(), cfg, health)
        if c.get("actuators", "ideal") == "dynamic":
            from vehicle_model.simulation.actuator_chain import ActuatorChain, ActuatorChainConfig
            return ActuatorChain(ctrl, p, ActuatorChainConfig.from_yaml())
        return ctrl
    return build


def scenario(s: dict):
    from vehicle_model.controllers.framework import ControlTarget
    from vehicle_model.simulation.controller_benchmark import Scenario
    events = list(s.get("driver", []))

    def driver(t, state):
        b = a = 0.0
        for e in events:
            if e["t0"] <= t < e["t1"]:
                b, a = max(b, float(e.get("brake_pedal", 0.0))), max(a, float(e.get("accel_pedal", 0.0)))
        return ControlTarget(driver_brake_pedal=b, driver_accel_pedal=a)
    return Scenario(s["name"], lambda: make_road(s), float(s.get("speed_kph", 10.0)) / 3.6, float(s.get("duration", 20.0)),
                    driver)


def kpis(h: dict, s: dict, c: dict, dt: float) -> dict:
    t, v = h["t"], h["speed"]
    v_set = float(c["hdc"].get("v_set_kph", 10.0)) / 3.6
    st = [None if d is None else d.get("hdc") for d in h["function_status"]]
    inf = np.array([x == "infeasible" for x in st])
    late = t >= 10.0                            # after the slope entry transient (overshoot is its own KPI)
    return {
        "v_max_kph": float(v.max() * 3.6), "v_end_kph": float(v[-1] * 3.6),
        "speed_rms_error_kph": float(np.sqrt(np.mean((v[late] - v_set) ** 2)) * 3.6) if np.any(late) else float("nan"),
        "overshoot_kph": float(max(0.0, v.max() - v_set) * 3.6),
        "max_abs_slip": float(np.max(h["max_abs_slip"])), "max_brake_decel": float(max(0.0, -np.min(h["ax"]))),
        "abs_share": float(np.mean(h["abs_active_n"] > 0)),
        "brake_energy_kj": float(np.sum(h["friction_power_w"]) * dt / 1e3),
        "regen_energy_kj": float(np.sum(h["regen_power_w"]) * dt / 1e3),
        "infeasible_s": float(np.sum(inf) * dt), "t_infeasible": float(t[np.argmax(inf)]) if inf.any() else float("nan"),
        "max_lateral_drift_m": float(np.max(np.abs(h["Y"]))), "max_heading_deg": math.degrees(float(np.max(np.abs(h["psi"])))),
        "distance_m": float(h["X"][-1]),
        "status_counts": {str(k): int(sum(1 for x in st if x == k)) for k in set(st)},
    }


def _task(args):
    from vehicle_model.simulation.controller_benchmark import run_scenario
    from vehicle_model.vehicle_models import SevenDOFModel
    spec, s, c = args
    try:
        road = make_road(s)
        r = run_scenario(lambda rd: SevenDOFModel.from_yaml(spec.vehicle, road=rd), controller_factory(c), scenario(s), spec.dt)
        h = r.history
        hist = {k: np.asarray(h[k], float) for k in HIST if k in h}
        hist["Fx_FL"] = np.asarray(h["Fx_FL"], float)
        hist["grade_pct"] = np.array([road.grade(x) * 100.0 for x in h["X"]])
        hist["Z"] = np.array([road.height(x) for x in h["X"]])
        hist["hdc_infeasible"] = np.array([float(d is not None and d.get("hdc") == "infeasible") for d in h["function_status"]])
        hist["hdc_active"] = np.array([float(d is not None and d.get("hdc") in ("active", "infeasible")) for d in h["function_status"]])
        return {"scenario": s["name"], "controller": c["name"], "kpis": kpis(h, s, c, spec.dt), "error": "", "history": hist}
    except Exception as ex:   # a failing run is a result
        return {"scenario": s["name"], "controller": c["name"], "kpis": {}, "error": f"{type(ex).__name__}: {ex}", "history": None}


def run(spec: HDCSpec, out: Path, verify: bool | None = None) -> dict:
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    tasks = [(spec, s, c) for s in spec.scenarios for c in spec.controllers]
    workers = spec.workers or max(1, min(8, (os.cpu_count() or 2) - 1))
    if workers > 1:
        with ProcessPoolExecutor(max_workers=workers) as ex:
            res = list(ex.map(_task, tasks))
    else:
        res = [_task(t) for t in tasks]
    hist = {(r["scenario"], r["controller"]): r.pop("history") for r in res}
    summary = {"experiment": spec.to_dict(), "source": spec.source, "provenance": provenance(), "verification": [],
               "information_mode": "A (vehicle speed, ABS flags, previous allocation); no road-grade information in HDC",
               "results": res}
    fm = set(spec.outputs.get("formats", ()))
    if "json" in fm:
        write_json(out / "results.json", summary)
    if "csv" in fm:
        with open(out / "kpis.csv", "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["scenario", "controller", *KPIS, "status_counts", "error"])
            for r in res:
                k = r["kpis"]
                w.writerow([r["scenario"], r["controller"], *(k.get(x, "") for x in KPIS),
                            ";".join(f"{a}:{b}" for a, b in sorted(k.get("status_counts", {}).items())), r["error"]])
        (out / "histories").mkdir(exist_ok=True)
        for (s, c), h in hist.items():
            if h is None:
                continue
            with open(out / "histories" / f"{s}__{c}.csv".replace(" ", "_").replace("/", "_"), "w", newline="", encoding="utf-8") as f:
                w = csv.writer(f)
                w.writerow(list(h))
                w.writerows(zip(*h.values()))
    if "md" in fm:
        f3 = lambda x: "–" if x in ("", None) or (isinstance(x, float) and math.isnan(x)) else f"{x:.3g}"
        L = [f"# Hill descent benchmark: {spec.name}", "", "7-DOF model with road grade (gravity in the plant), shared "
             "MotionController chain, information mode A. Synthetic parameters and test matrix, virtual model only.", "",
             "| scenario | controller | v max [km/h] | v end | speed RMS error (from 10 s) | max slip | max brake decel [m/s^2] | ABS share | friction / regen energy [kJ] | infeasible [s] |",
             "|---" * 10 + "|"]
        for r in res:
            k = r["kpis"]
            L.append(f"| {r['scenario']} | {r['controller']} | " + " | ".join(f3(k.get(x)) for x in (
                "v_max_kph", "v_end_kph", "speed_rms_error_kph", "max_abs_slip", "max_brake_decel", "abs_share"))
                     + f" | {f3(k.get('brake_energy_kj'))} / {f3(k.get('regen_energy_kj'))} | {f3(k.get('infeasible_s'))} |" + (f" {r['error']}" if r["error"] else ""))
        (out / "report.md").write_text("\n".join(L) + "\n", encoding="utf-8")
    return summary


__all__ = ("KIND", "HDCSpec", "describe", "make_road", "parse", "run")
