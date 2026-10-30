"""Experiment type ``ride_benchmark``: predictive damping and active pitch on the 14-DOF model (Plan V4.0 M21).

A matrix of scenarios (road + manoeuvre) x controllers on the 14-DOF closed loop of
:mod:`vehicle_model.simulation.ride`::

    type: ride_benchmark
    name: ...
    vehicle: configs/suspension/fourteen_dof.yaml
    scenarios:
      - {name: iso_C_60, road: {iso_class: C, seed: 3}, manoeuvre: {type: cruise, speed_kph: 60}, duration: 4.0}
    controllers:
      - {name: passive, damping: {mode: passive}}
      - {name: preview skyhook, damping: {mode: preview_skyhook}, preview: {latency: 0.02}}
      - {name: active pitch, damping: {mode: skyhook}, active_pitch: true}

KPIs per run: body vertical acceleration RMS and peak, pitch RMS (from the manoeuvre start) and peak, pitch-rate
peak, suspension travel and time beyond the declared travel limit (bottoming proxy: the model has no bump-stop
force), tyre-load variation, time with a wheel without load, active energy and largest active force.
"""

from __future__ import annotations

import csv
import itertools
import os
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass, field
from pathlib import Path

from .spec import _keys, _require
from .types import provenance, write_json

KIND = "ride_benchmark"
KPIS = ("body_accel_rms", "body_accel_peak", "pitch_rms_deg", "pitch_peak_deg", "pitch_rate_peak_degps", "travel_max_m",
        "travel_over_limit_s", "tyre_load_variation", "contact_loss_s", "active_energy_kj", "active_force_max_n")


@dataclass(frozen=True)
class RideSpec:
    name: str
    vehicle: str
    scenarios: tuple
    controllers: tuple
    workers: int | None = None
    outputs: dict = field(default_factory=dict)
    description: str = ""
    source: str = ""
    kind = KIND

    def to_dict(self) -> dict:
        return {"type": KIND, "name": self.name, "description": self.description, "vehicle": self.vehicle,
                "scenarios": [dict(s) for s in self.scenarios], "controllers": [dict(c) for c in self.controllers],
                "outputs": dict(self.outputs)}


def parse(data: dict, source: str = "<dict>") -> RideSpec:
    from vehicle_model.suspension.preview_control import DAMPING_MODES
    _require(data.get("type") == KIND, f"{source}: type must be {KIND!r}")
    _keys(data, {"type", "name", "description", "vehicle", "scenarios", "controllers", "workers", "outputs"}, source,
          {"type", "name", "vehicle", "scenarios", "controllers"})
    _require(Path(data["vehicle"]).is_file(), f"{source}: vehicle file not found: {data['vehicle']}")
    for key in ("scenarios", "controllers"):
        _require(isinstance(data[key], list) and data[key], f"{source}: {key} must be a non-empty list")
        names = [x.get("name") if isinstance(x, dict) else None for x in data[key]]
        _require(all(isinstance(n, str) for n in names) and len(set(names)) == len(names), f"{source}: {key} need unique names")
    for i, s in enumerate(data["scenarios"]):
        w = f"{source}: scenarios[{i}]"
        _keys(s, {"name", "road", "manoeuvre", "duration"}, w, {"name", "road", "manoeuvre"})
        _keys(s["road"], {"iso_class", "seed", "bumps", "grooves"}, f"{w}.road")
        _require(s["road"].get("iso_class", "A") in "ABCDEFGH", f"{w}.road.iso_class must be one of A-H")
        _require(s["manoeuvre"].get("type") in ("cruise", "brake", "accel", "corner"),
                 f"{w}.manoeuvre.type must be cruise, brake, accel or corner")
        _keys(s["manoeuvre"], {"type", "speed_kph", "decel_g", "start_s", "drive_torque", "steer_deg"}, f"{w}.manoeuvre", {"type"})
        _require(1.0 <= float(s.get("duration", 4.0)) <= 30.0, f"{w}.duration must lie in [1, 30] s")
    for i, c in enumerate(data["controllers"]):
        w = f"{source}: controllers[{i}]"
        _keys(c, {"name", "damping", "active_pitch", "preview"}, w, {"name", "damping"})
        _require(c["damping"].get("mode") in DAMPING_MODES, f"{w}.damping.mode must be one of {DAMPING_MODES}")
        _keys(c["damping"], {"mode", "c_min", "c_max", "c_sky", "v_soft", "window", "damper_failed"}, f"{w}.damping")
        if "preview" in c:
            _require(c["damping"]["mode"] == "preview_skyhook", f"{w}.preview needs damping mode preview_skyhook")
            _keys(c["preview"], {"distance", "latency", "noise_std", "seed", "failed"}, f"{w}.preview")
        _require(set(c["damping"].get("damper_failed", [])) <= {"FL", "FR", "RL", "RR"}, f"{w}.damping.damper_failed: wheel names")
    outputs = dict(data.get("outputs") or {})
    _keys(outputs, {"directory", "formats"}, f"{source}: outputs")
    outputs.setdefault("formats", ["csv", "json", "md"])
    outputs.setdefault("directory", f"output/experiments/{data['name']}")
    return RideSpec(str(data["name"]), str(data["vehicle"]), tuple(dict(s) for s in data["scenarios"]),
                    tuple(dict(c) for c in data["controllers"]), data.get("workers"), outputs,
                    str(data.get("description", "")), source)


def describe(spec: RideSpec) -> str:
    return f"14-DOF ride/pitch: {len(spec.scenarios)} scenario(s) x {len(spec.controllers)} controller(s)"


def _task(args):
    from vehicle_model.simulation.ride import run as run_ride
    spec, s, c = args
    d = dict(c["damping"])
    if "damper_failed" in d:
        d["damper_failed"] = tuple(d["damper_failed"])
    try:
        r = run_ride(spec.vehicle, s["road"], s["manoeuvre"], d, bool(c.get("active_pitch", False)), c.get("preview"),
                     duration=float(s.get("duration", 4.0)))
        return {"scenario": s["name"], "controller": c["name"], "kpis": r["kpis"], "error": "", "history": r["history"]}
    except Exception as ex:   # a failing run is a result
        return {"scenario": s["name"], "controller": c["name"], "kpis": {}, "error": f"{type(ex).__name__}: {ex}", "history": None}


def run(spec: RideSpec, out: Path, verify: bool | None = None) -> dict:
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    tasks = [(spec, s, c) for s, c in itertools.product(spec.scenarios, spec.controllers)]
    workers = spec.workers or max(1, min(8, (os.cpu_count() or 2) - 1))
    if workers > 1:
        with ProcessPoolExecutor(max_workers=workers) as ex:
            res = list(ex.map(_task, tasks))
    else:
        res = [_task(t) for t in tasks]
    hist = {(r["scenario"], r["controller"]): r.pop("history") for r in res}
    summary = {"experiment": spec.to_dict(), "source": spec.source, "provenance": provenance(), "verification": [],
               "information_mode": "A (14-DOF body/wheel velocities); preview from an external road sensor model",
               "results": res}
    fm = set(spec.outputs.get("formats", ()))
    if "json" in fm:
        write_json(out / "results.json", summary)
    if "csv" in fm:
        with open(out / "kpis.csv", "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["scenario", "controller", *KPIS, "error"])
            for r in res:
                w.writerow([r["scenario"], r["controller"], *(r["kpis"].get(k, "") for k in KPIS), r["error"]])
        (out / "histories").mkdir(exist_ok=True)
        for (s, c), h in hist.items():
            if h is None:
                continue
            with open(out / "histories" / f"{s}__{c}.csv".replace(" ", "_").replace("/", "_"), "w", newline="", encoding="utf-8") as f:
                w = csv.writer(f)
                w.writerow(list(h))
                for i in range(len(h["t"])):
                    w.writerow([h[k][i] for k in h])
    if "md" in fm:
        L = [f"# Ride and pitch benchmark: {spec.name}", "", "14-DOF model, information mode A, preview from an external "
             "road sensor model. Synthetic parameters, virtual model only.", "",
             "| scenario | controller | body acc RMS [m/s^2] | body acc peak | pitch RMS [deg] | pitch peak [deg] | travel max [m] | tyre-load variation | active energy [kJ] |",
             "|---" * 9 + "|"]
        f3 = lambda x: "–" if x in ("", None) else f"{x:.3g}"
        for r in res:
            k = r["kpis"]
            L.append(f"| {r['scenario']} | {r['controller']} | {f3(k.get('body_accel_rms'))} | {f3(k.get('body_accel_peak'))} | "
                     f"{f3(k.get('pitch_rms_deg'))} | {f3(k.get('pitch_peak_deg'))} | {f3(k.get('travel_max_m'))} | "
                     f"{f3(k.get('tyre_load_variation'))} | {f3(k.get('active_energy_kj'))} |" + (f" {r['error']}" if r["error"] else ""))
        (out / "report.md").write_text("\n".join(L) + "\n", encoding="utf-8")
    return summary


__all__ = ("KIND", "RideSpec", "describe", "parse", "run")
