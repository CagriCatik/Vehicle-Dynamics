"""Experiment type ``fault_tolerance_benchmark``: fault catalogue on the 7-DOF closed loop.

Manoeuvres x fault sets x controllers. Every controller runs behind the deterministic fault injection
(:mod:`vehicle_model.simulation.fault_injection`); ``aware: true`` adds the health monitor and degraded modes
(:mod:`vehicle_model.controllers.fault_tolerance`), ``aware: false`` is the same controller without them::

    type: fault_tolerance_benchmark
    name: ...
    vehicle: configs/vehicles/seven_dof_passenger.yaml
    manoeuvres:
      - {name: split_mu_braking_90, type: split_mu_braking, speed_kph: 90}
      - {name: sine_with_dwell_80, type: sine_with_dwell, speed_kph: 80, amplitude_deg: 6}
    fault_sets:
      - {name: none, faults: []}
      - {name: brake_RR_failed, faults: [{t_start: 0.5, channel: brake, mode: failed, wheel: RR}]}
    controllers:
      - {name: PI unaware, law: pi, aware: false}
      - {name: PI fault tolerant, law: pi, aware: true}

KPIs: the manoeuvre KPIs of the controller benchmark, plus command-leak time to failed actuators before and after
their self-report, worst system health state and time per state, time with the system ``unavailable`` (no brake
left), and
whether every injected fault was detected (``undetected``: faults never seen by the monitor).
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

KIND = "fault_tolerance_benchmark"
MANOEUVRES = {"split_mu_braking": {"speed_kph"}, "straight_braking": {"speed_kph", "brake_pedal"},
              "sine_with_dwell": {"speed_kph", "amplitude_deg"}, "brake_in_turn": {"speed_kph", "steer_deg", "brake_pedal"},
              "step_steer": {"speed_kph", "steer_deg"}, "lane_change": {"speed_kph", "amplitude_deg"}}
LAWS = ("no_yaw", "pi", "gs_lqr")
FAULT_KPIS = ("leak_before_report_s", "leak_after_report_s", "worst_state", "unavailable_s", "undetected")
STATE_CHANNEL = {"brake": "brake", "motor": "motor", "steer_front": "steer_front", "steer_rear": "steer_rear", "imu": "imu",
                 "wheel_speed": "wheel_speed", "load_sensor": "load_sensor", "delay": "comm", "stale": "comm",
                 "mu_estimate": None}


@dataclass(frozen=True)
class FaultSpec:
    name: str
    vehicle: str
    manoeuvres: tuple
    fault_sets: tuple
    controllers: tuple
    dt: float = 0.005
    detection_delay: float = 0.05
    workers: int | None = None
    outputs: dict = field(default_factory=dict)
    description: str = ""
    source: str = ""
    kind = KIND

    def to_dict(self) -> dict:
        return {"type": KIND, "name": self.name, "description": self.description, "vehicle": self.vehicle, "dt": self.dt,
                "detection_delay": self.detection_delay, "manoeuvres": [dict(m) for m in self.manoeuvres],
                "fault_sets": [dict(f) for f in self.fault_sets], "controllers": [dict(c) for c in self.controllers],
                "outputs": dict(self.outputs)}


def parse(data: dict, source: str = "<dict>") -> FaultSpec:
    from vehicle_model.simulation.fault_injection import parse_faults
    _require(isinstance(data, dict) and data.get("type") == KIND, f"{source}: type must be {KIND!r}")
    _keys(data, {"type", "name", "description", "vehicle", "manoeuvres", "fault_sets", "controllers", "dt",
                 "detection_delay", "workers", "outputs"}, source, {"type", "name", "vehicle", "manoeuvres", "fault_sets", "controllers"})
    _require(isinstance(data["vehicle"], str) and Path(data["vehicle"]).is_file(), f"{source}: vehicle file not found: {data['vehicle']}")
    for key in ("manoeuvres", "fault_sets", "controllers"):
        _require(isinstance(data[key], list) and data[key], f"{source}: {key} must be a non-empty list")
        _require(all(isinstance(x, dict) for x in data[key]), f"{source}: {key} entries must be mappings")
        names = [x.get("name") for x in data[key]]
        _require(all(isinstance(n, str) for n in names) and len(set(names)) == len(names), f"{source}: {key} need unique names")
    for i, m in enumerate(data["manoeuvres"]):
        w = f"{source}: manoeuvres[{i}]"
        _require(m.get("type") in MANOEUVRES, f"{w}.type must be one of {sorted(MANOEUVRES)}")
        _keys(m, MANOEUVRES[m["type"]] | {"name", "type"}, w, {"name", "type", "speed_kph"})
        for k, v in m.items():
            if k not in ("name", "type"):
                _require(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v), f"{w}.{k} must be a number")
        _require(10.0 <= m["speed_kph"] <= 200.0, f"{w}.speed_kph must lie in [10, 200]")
    for i, f in enumerate(data["fault_sets"]):
        _keys(f, {"name", "faults"}, f"{source}: fault_sets[{i}]", {"name", "faults"})
        parse_faults(f["faults"], f"{source}: fault_sets[{i}].faults")
    for i, c in enumerate(data["controllers"]):
        w = f"{source}: controllers[{i}]"
        _keys(c, {"name", "law", "aware", "actuators"}, w, {"name", "law", "aware"})
        _require(c["law"] in LAWS, f"{w}.law must be one of {LAWS}")
        _require(isinstance(c["aware"], bool), f"{w}.aware must be true or false")
        _require(c.get("actuators", "ideal") in ("ideal", "dynamic"), f"{w}.actuators must be ideal or dynamic")
    dt = data.get("dt", 0.005)
    _require(isinstance(dt, (int, float)) and 0.0 < dt <= 0.01, f"{source}: dt must lie in (0, 0.01] s")
    dd = data.get("detection_delay", 0.05)
    _require(isinstance(dd, (int, float)) and 0.0 <= dd <= 1.0, f"{source}: detection_delay must lie in [0, 1] s")
    outputs = dict(data.get("outputs") or {})
    _keys(outputs, {"directory", "formats"}, f"{source}: outputs")
    outputs.setdefault("formats", ["csv", "json", "md"])
    outputs.setdefault("directory", f"output/experiments/{data['name']}")
    return FaultSpec(str(data["name"]), data["vehicle"], tuple(dict(m) for m in data["manoeuvres"]),
                     tuple(dict(f) for f in data["fault_sets"]), tuple(dict(c) for c in data["controllers"]), float(dt),
                     float(dd), data.get("workers"), outputs, str(data.get("description", "")), source)


def describe(spec: FaultSpec) -> str:
    return (f"fault tolerance: {len(spec.manoeuvres)} manoeuvre(s) x {len(spec.fault_sets)} fault set(s) x "
            f"{len(spec.controllers)} controller(s)")


def scenario(m: dict):
    from vehicle_model.simulation import controller_benchmark as cb
    v = float(m["speed_kph"]) / 3.6
    t = m["type"]
    if t == "split_mu_braking":
        return cb.split_mu_braking_scenario(v)
    if t == "straight_braking":
        return cb.straight_braking_scenario(v, float(m.get("brake_pedal", 1.0)))
    if t == "sine_with_dwell":
        return cb.sine_with_dwell_scenario(v, math.radians(float(m.get("amplitude_deg", 6.0))))
    if t == "brake_in_turn":
        return cb.brake_in_turn_scenario(v, math.radians(float(m.get("steer_deg", 2.0))), float(m.get("brake_pedal", 0.4)))
    if t == "step_steer":
        return cb.step_steer_scenario(v, math.radians(float(m.get("steer_deg", 2.3))))
    return cb.lane_change_scenario(v, math.radians(float(m.get("amplitude_deg", 2.5))))


def controller_factory(spec: FaultSpec, c: dict, faults, sink: list):
    from vehicle_model.controllers.fault_tolerance import FaultTolerantController
    from vehicle_model.controllers.motion_control import MotionController
    from vehicle_model.controllers.yaw_laws import INPUTS_TV, LQRYawLaw, NoYawLaw, YawRatePI
    from vehicle_model.handling_analysis import linear_params_from_seven_dof
    from vehicle_model.simulation.fault_injection import FaultInjectedController
    from vehicle_model.vehicle_models import SevenDOFModel
    from .runner import SCHEDULE_SPEEDS
    base = SevenDOFModel.from_yaml(spec.vehicle)
    design = linear_params_from_seven_dof(base.params, base.tyre_front, base.tyre_rear)

    def build(p):
        law = {"no_yaw": NoYawLaw, "pi": YawRatePI}.get(c["law"])
        law = law() if law else LQRYawLaw(design, INPUTS_TV, design_speed=None, schedule_speeds=SCHEDULE_SPEEDS)
        ctrl = MotionController(p, law)
        if c.get("actuators", "ideal") == "dynamic":
            from vehicle_model.simulation.actuator_chain import ActuatorChain, ActuatorChainConfig
            ctrl = ActuatorChain(ctrl, p, ActuatorChainConfig.from_yaml())
        if c["aware"]:
            ctrl = FaultTolerantController(ctrl, p)
        a = MotionController(p, NoYawLaw()).mc_config.actuation.allocator
        w = FaultInjectedController(ctrl, faults, spec.detection_delay, p.Reff, a.max_brake_torque, a.max_drive_torque)
        sink.append(w)
        return w
    return build


def fault_kpis(h: dict, log: list, faults, dt: float) -> dict:
    from vehicle_model.controllers.fault_tolerance import STATES
    health = [None if d is None else d.get("health") for d in h["function_status"]]
    leak_before = leak_after = 0.0
    for e in log:
        for lab, leaked in e["leaks"].items():
            if leaked:
                reported = any(r.replace(" ", "") == lab.replace(" ", "") for r in e["reported"])
                leak_before += 0.0 if reported else dt
                leak_after += dt if reported else 0.0
    seen = [x for x in health if x is not None]
    undetected = []
    for f in faults:
        ch = STATE_CHANNEL[f.channel]
        if not seen:
            break
        if ch is None or all(x == "nominal" for x, t in zip(health, h["t"]) if f.active(t - dt)):
            undetected.append(f.label)
    return {"leak_before_report_s": leak_before, "leak_after_report_s": leak_after,
            "worst_state": max(seen, key=STATES.index) if seen else "not monitored",
            "unavailable_s": float(sum(1 for x in seen if x == "unavailable") * dt),
            "undetected": ";".join(undetected) if seen else "not monitored",
            "state_s": {s: float(sum(1 for x in seen if x == s) * dt) for s in STATES if s in seen}}


def _task(args):
    from vehicle_model.simulation.controller_benchmark import run_scenario
    from vehicle_model.simulation.fault_injection import parse_faults
    from vehicle_model.vehicle_models import SevenDOFModel
    spec, m, fs, c = args
    faults = parse_faults(fs["faults"])
    sink = []
    try:
        sc = scenario(m)
        r = run_scenario(lambda rd: SevenDOFModel.from_yaml(spec.vehicle, road=rd), controller_factory(spec, c, faults, sink), sc, spec.dt)
        h = r.history
        finite = bool(np.all(np.isfinite(np.asarray(h["vx"], float))))
        k = {kk: v for kk, v in r.kpis.items() if not isinstance(v, dict) and not kk.startswith("computation")}
        k.update(fault_kpis(h, sink[-1].log, faults, spec.dt))
        k["finite"] = finite
        hist = {key: np.asarray(h[key], float) for key in ("t", "X", "Y", "psi", "vx", "vy", "speed", "r", "r_ref", "beta",
                                                             "steer_f", "steer_r", "Mz_demand", "abs_active_n", "T_brake_total")}
        for i, w in enumerate(("FL", "FR", "RL", "RR")):
            hist[f"Fx_{w}"] = np.asarray(h[f"Fx_{w}"], float)
            hist[f"Fy_{w}"] = np.asarray(h[f"Fy_{w}"], float)
        from vehicle_model.controllers.fault_tolerance import STATES
        hist["health_rank"] = np.array([STATES.index(d["health"]) if d and d.get("health") else -1 for d in h["function_status"]], float)
        hist["faults_active"] = np.array([len(e["active"]) for e in sink[-1].log][:len(h["t"])], float)
        return {"manoeuvre": m["name"], "fault_set": fs["name"], "controller": c["name"], "kpis": k, "error": "", "history": hist}
    except Exception as ex:   # a failing run (e.g. NaN reaching the allocation) is a result
        return {"manoeuvre": m["name"], "fault_set": fs["name"], "controller": c["name"], "kpis": {},
                "error": f"{type(ex).__name__}: {ex}", "history": None}


def run(spec: FaultSpec, out: Path, verify: bool | None = None) -> dict:
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    tasks = [(spec, m, f, c) for m in spec.manoeuvres for f in spec.fault_sets for c in spec.controllers]
    workers = spec.workers or max(1, min(8, (os.cpu_count() or 2) - 1))
    if workers > 1:
        with ProcessPoolExecutor(max_workers=workers) as ex:
            res = list(ex.map(_task, tasks))
    else:
        res = [_task(t) for t in tasks]
    hist = {(r["manoeuvre"], r["fault_set"], r["controller"]): r.pop("history") for r in res}
    summary = {"experiment": spec.to_dict(), "source": spec.source, "provenance": provenance(), "verification": [],
               "information": "monitor sees the corrupted time-stamped controller input and actuator self-reports only",
               "results": res}
    fm = set(spec.outputs.get("formats", ()))
    if "json" in fm:
        write_json(out / "results.json", summary)
    keys = sorted({k for r in res for k, v in r["kpis"].items() if not isinstance(v, dict)})
    if "csv" in fm:
        with open(out / "kpis.csv", "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["manoeuvre", "fault_set", "controller", *keys, "error"])
            for r in res:
                w.writerow([r["manoeuvre"], r["fault_set"], r["controller"], *(r["kpis"].get(k, "") for k in keys), r["error"]])
        (out / "histories").mkdir(exist_ok=True)
        for (m, fs, c), h in hist.items():
            if h is None:
                continue
            with open(out / "histories" / f"{m}__{fs}__{c}.csv".replace(" ", "_").replace("/", "_"), "w", newline="", encoding="utf-8") as f:
                w = csv.writer(f)
                w.writerow(list(h))
                w.writerows(zip(*h.values()))
    if "md" in fm:
        f3 = lambda x: "–" if x in ("", None) else (f"{x:.3g}" if isinstance(x, float) else str(x))
        L = [f"# Fault tolerance benchmark: {spec.name}", "", "7-DOF closed loop, deterministic fault injection, actuator "
             f"self-reports after {spec.detection_delay * 1e3:.0f} ms. Synthetic parameters, virtual model only.", ""]
        for m in spec.manoeuvres:
            rows = [r for r in res if r["manoeuvre"] == m["name"]]
            mk = [k for k in keys if k not in FAULT_KPIS and k != "finite" and any(k in r["kpis"] for r in rows)]
            L += [f"## {m['name']}", "", "| fault set | controller | " + " | ".join(mk + list(FAULT_KPIS)) + " |",
                  "|---" * (len(mk) + len(FAULT_KPIS) + 2) + "|"]
            for r in rows:
                L.append(f"| {r['fault_set']} | {r['controller']} | " + " | ".join(f3(r["kpis"].get(k)) for k in mk + list(FAULT_KPIS))
                         + " |" + (f" **{r['error']}**" if r["error"] else ""))
            L.append("")
        (out / "report.md").write_text("\n".join(L) + "\n", encoding="utf-8")
    return summary


__all__ = ("KIND", "FaultSpec", "describe", "parse", "run")
