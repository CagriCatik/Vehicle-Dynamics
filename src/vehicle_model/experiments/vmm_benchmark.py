"""Experiment type ``vmm_benchmark``: integrated vehicle-motion benchmark matrix P0-P7 (Plan V4.0 M24.3).

Configuration levels x manoeuvres x paired variation draws::

    type: vmm_benchmark
    name: ...
    vehicle: configs/vehicles/seven_dof_passenger.yaml          # 7-DOF manoeuvres
    vehicle_14dof: configs/suspension/fourteen_dof.yaml          # ride and rollover manoeuvres
    levels:
      - {name: P0, controller: {law: passive}}
      - {name: P5, controller: {law: gs_lqr, inputs: afs_tv, ...}, rsc: true}
      - {name: P6, same_as_7dof: P5, controller: {...}, rsc: true, vertical: {damping: preview_skyhook, active_pitch: true}}
      - {name: P7, ..., fault_tolerant: true, supervisor: true}
    manoeuvres:
      - {name: split_mu_braking, model: 7dof, road: {type: split_mu, mu_left: 1.0, mu_right: 0.2},
         manoeuvre: {type: split_mu_braking, speed_kph: 90}, primary: [stopping_distance, max_lateral_offset]}
      - {name: hill_descent_20, model: 7dof, grade: [[15, -20]], road: {type: uniform, mu: 1.0},
         manoeuvre: {type: coast, speed_kph: 10, duration_s: 20}, primary: [v_max_kph]}
      - {name: actuator_fault, model: 7dof, ..., faults: [{t_start: 0.5, channel: brake, mode: failed, wheel: RR}]}
      - {name: uneven_road, model: 14dof_ride, road: {iso_class: C}, manoeuvre: {type: cruise, speed_kph: 60}}
      - {name: fishhook, model: 14dof_rollover, manoeuvre: {type: fishhook, speed_kph: 80, amplitude_deg: 8}, mu: 1.0,
         variant: {cg_height: 0.95, sprung_mass: 1520}}
    variations: {draws: 4, seed: 1, speed_pct: 5, mu_pct: 10, mass_pct: 10, hcog_pct: 10}

**Model boundary** (Plan V4.0): 7-DOF manoeuvres run the shared MotionController chain of each level; 14-DOF
manoeuvres vary only what exists there: ride (damping mode, preview, active pitch: ``vertical``) and rollover
(``rsc``). A level that does not change anything on a manoeuvre's model (P6 vs P5 on 7-DOF; P0-P5 on ride) is run
once and **reported as identical by construction**, not run again.

**Paired variations**: draw 0 is nominal; draws 1..n-1 vary speed, friction, mass (and yaw inertia with it) and CoG
height uniformly within the declared percentages, identically for every level. Controllers are designed on the
**nominal** parameters, the plant gets the drawn ones. Ride draws change the road seed and speed, rollover draws
the CoG height and speed.

Statistics per manoeuvre, level and KPI: median, quartiles, worst case, mean with a bootstrap 95 % interval (with
few draws the interval is wide and reported as such), and the median paired difference to P0.
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

from .spec import _keys, _require
from .types import provenance, write_json

KIND = "vmm_benchmark"
MODELS = ("7dof", "14dof_ride", "14dof_rollover")
LOWER_IS_BETTER = True   # every primary KPI is a cost (distances, errors, peaks, times)


@dataclass(frozen=True)
class VMMSpec:
    name: str
    vehicle: str
    vehicle_14dof: str
    levels: tuple
    manoeuvres: tuple
    variations: dict
    dt: float = 0.005
    workers: int | None = None
    outputs: dict = field(default_factory=dict)
    description: str = ""
    source: str = ""
    kind = KIND

    def to_dict(self) -> dict:
        return {"type": KIND, "name": self.name, "description": self.description, "vehicle": self.vehicle,
                "vehicle_14dof": self.vehicle_14dof, "dt": self.dt, "levels": [copy.deepcopy(x) for x in self.levels],
                "manoeuvres": [copy.deepcopy(m) for m in self.manoeuvres], "variations": dict(self.variations),
                "outputs": dict(self.outputs)}


def _dsl_spec(vehicle, m, controller, dt, name="vmm"):
    from .spec import parse_controller_comparison
    return parse_controller_comparison({"name": name, "vehicle": vehicle, "road": m["road"], "manoeuvre": m["manoeuvre"],
                                        "controllers": [controller], "dt": dt, "verification": False,
                                        "outputs": {"formats": []}})


def parse(data: dict, source: str = "<dict>") -> VMMSpec:
    from vehicle_model.simulation.fault_injection import parse_faults
    _require(isinstance(data, dict) and data.get("type") == KIND, f"{source}: type must be {KIND!r}")
    _keys(data, {"type", "name", "description", "vehicle", "vehicle_14dof", "levels", "manoeuvres", "variations", "dt",
                 "workers", "outputs"}, source, {"type", "name", "vehicle", "vehicle_14dof", "levels", "manoeuvres"})
    for k in ("vehicle", "vehicle_14dof"):
        _require(isinstance(data[k], str) and Path(data[k]).is_file(), f"{source}: {k} file not found: {data[k]}")
    dt = data.get("dt", 0.005)
    _require(isinstance(dt, (int, float)) and 0.0 < dt <= 0.01, f"{source}: dt must lie in (0, 0.01] s")
    for key in ("levels", "manoeuvres"):
        _require(isinstance(data[key], list) and data[key] and all(isinstance(x, dict) for x in data[key]),
                 f"{source}: {key} must be a non-empty list of mappings")
        names = [x.get("name") for x in data[key]]
        _require(all(isinstance(n, str) for n in names) and len(set(names)) == len(names), f"{source}: {key} need unique names")
    names = [lv["name"] for lv in data["levels"]]
    for i, lv in enumerate(data["levels"]):
        w = f"{source}: levels[{i}]"
        _keys(lv, {"name", "controller", "rsc", "vertical", "fault_tolerant", "supervisor", "same_as_7dof", "description"},
              w, {"name", "controller"})
        for k in ("rsc", "fault_tolerant", "supervisor"):
            _require(isinstance(lv.get(k, False), bool), f"{w}.{k} must be true or false")
        if "same_as_7dof" in lv:
            _require(lv["same_as_7dof"] in names[:i], f"{w}.same_as_7dof must name an earlier level")
        if "vertical" in lv:
            _keys(lv["vertical"], {"damping", "active_pitch", "preview"}, f"{w}.vertical", {"damping"})
        _require(not (lv.get("fault_tolerant") and lv["controller"].get("law") == "passive"),
                 f"{w}: the passive car has no health monitor")
    for i, m in enumerate(data["manoeuvres"]):
        w = f"{source}: manoeuvres[{i}]"
        _require(m.get("model") in MODELS, f"{w}.model must be one of {MODELS}")
        _require(isinstance(m.get("primary"), list) and m["primary"] and all(isinstance(k, str) for k in m["primary"]),
                 f"{w}.primary must be a non-empty list of KPI names")
        if m["model"] == "7dof":
            _keys(m, {"name", "model", "road", "manoeuvre", "grade", "faults", "primary", "description"}, w,
                  {"name", "model", "road", "manoeuvre", "primary"})
            for lv in data["levels"]:                 # every level must be a valid controller on this manoeuvre
                _dsl_spec(data["vehicle"], m, lv["controller"], dt)
            if "faults" in m:
                parse_faults(m["faults"], f"{w}.faults")
            if "grade" in m:
                _require(isinstance(m["grade"], list) and m["grade"], f"{w}.grade must be a list of [x_m, grade_percent]")
        elif m["model"] == "14dof_ride":
            _keys(m, {"name", "model", "road", "manoeuvre", "duration", "primary", "description"}, w,
                  {"name", "model", "road", "manoeuvre", "primary"})
        else:
            _keys(m, {"name", "model", "manoeuvre", "mu", "variant", "duration", "primary", "description"}, w,
                  {"name", "model", "manoeuvre", "primary"})
    var = dict(data.get("variations") or {})
    _keys(var, {"draws", "seed", "speed_pct", "mu_pct", "mass_pct", "hcog_pct"}, f"{source}: variations")
    var.setdefault("draws", 1)
    var.setdefault("seed", 0)
    _require(isinstance(var["draws"], int) and 1 <= var["draws"] <= 50, f"{source}: variations.draws must lie in [1, 50]")
    for k in ("speed_pct", "mu_pct", "mass_pct", "hcog_pct"):
        var.setdefault(k, 0.0)
        _require(isinstance(var[k], (int, float)) and 0.0 <= var[k] <= 30.0, f"{source}: variations.{k} must lie in [0, 30]")
    outputs = dict(data.get("outputs") or {})
    _keys(outputs, {"directory", "formats"}, f"{source}: outputs")
    outputs.setdefault("formats", ["csv", "json", "md", "png"])
    outputs.setdefault("directory", f"output/experiments/{data['name']}")
    return VMMSpec(str(data["name"]), data["vehicle"], data["vehicle_14dof"], tuple(copy.deepcopy(x) for x in data["levels"]),
                   tuple(copy.deepcopy(m) for m in data["manoeuvres"]), var, float(dt), data.get("workers"), outputs,
                   str(data.get("description", "")), source)


def describe(spec: VMMSpec) -> str:
    return (f"VMM matrix: {len(spec.levels)} level(s) x {len(spec.manoeuvres)} manoeuvre(s) x "
            f"{spec.variations['draws']} paired draw(s)")


def draws(spec: VMMSpec) -> list:
    """Paired variation draws (draw 0 nominal); factors per quantity."""
    v = spec.variations
    rng = np.random.default_rng(v["seed"])
    out = [{"draw": 0, "speed": 1.0, "mu": 1.0, "mass": 1.0, "hcog": 1.0, "road_seed": 0}]
    for k in range(1, v["draws"]):
        u = rng.uniform(-1.0, 1.0, 4)
        out.append({"draw": k, "speed": 1 + u[0] * v["speed_pct"] / 100, "mu": 1 + u[1] * v["mu_pct"] / 100,
                    "mass": 1 + u[2] * v["mass_pct"] / 100, "hcog": 1 + u[3] * v["hcog_pct"] / 100, "road_seed": k})
    return out


def _effective_level(spec, m, lv):
    """The level that actually runs on this manoeuvre's model (identical-by-construction levels share runs)."""
    if m["model"] == "7dof":
        return lv.get("same_as_7dof", lv["name"])
    if m["model"] == "14dof_ride":
        vert = lv.get("vertical")
        return "vertical:" + (repr(sorted(vert.items())) if vert else "passive")
    return "rsc" if lv.get("rsc") else "passive"


def _vary_road(road: dict, f: float) -> dict:
    r = dict(road)
    for k in ("mu", "mu_left", "mu_right", "mu_before", "mu_after"):
        if k in r:
            r[k] = float(min(2.0, max(0.05, r[k] * f)))
    return r


def _run_7dof(spec, m, lv, d):
    from vehicle_model.controllers.framework import PassiveBaselineController
    from vehicle_model.simulation.controller_benchmark import run_scenario
    from vehicle_model.vehicle_models import SevenDOFModel
    from .runner import controller_factories, make_scenario
    mm = dict(m, road=_vary_road(m["road"], d["mu"]),
              manoeuvre=dict(m["manoeuvre"], speed_kph=float(m["manoeuvre"]["speed_kph"]) * d["speed"]))
    sp = _dsl_spec(spec.vehicle, mm, lv["controller"], spec.dt)
    base = SevenDOFModel.from_yaml(spec.vehicle)
    p_nom = base.params
    p_var = dataclasses.replace(p_nom, mass=p_nom.mass * d["mass"], Iz=p_nom.Iz * d["mass"], hcog=p_nom.hcog * d["hcog"])
    if "grade" in m:
        from vehicle_model.experiments.hill_descent import make_road as graded_road
        from vehicle_model.experiments.hill_descent import scenario as hd_scenario
        sf = {"mu": sp.road["mu"]} if sp.road["type"] == "uniform" else {"mu_left": sp.road["mu_left"], "mu_right": sp.road["mu_right"]}
        hd = {"name": m["name"], "profile": m["grade"], "surface": sf, "speed_kph": sp.manoeuvre["speed_kph"],
              "duration": float(sp.manoeuvre.get("duration_s", 20.0))}
        sc = hd_scenario(hd)
        road_fn = lambda: graded_road(hd)
        sc = dataclasses.replace(sc, road=road_fn)
    else:
        sc = make_scenario(sp)
    factory = controller_factories(sp)[sp.controllers[0].label]
    sink = []

    def ctrl_factory(_p_plant):
        c = PassiveBaselineController(p_nom) if factory is None else factory(p_nom)   # designed on nominal parameters
        if lv.get("fault_tolerant"):
            from vehicle_model.controllers.fault_tolerance import FaultTolerantController
            c = FaultTolerantController(c, p_nom)
        if lv.get("supervisor"):
            from vehicle_model.controllers.motion_supervisor import VehicleMotionSupervisor
            c = VehicleMotionSupervisor(c)
            sink.append(c)
        if m.get("faults"):
            from vehicle_model.simulation.fault_injection import FaultInjectedController, parse_faults
            c = FaultInjectedController(c, parse_faults(m["faults"]), 0.05, p_nom.Reff)
            sink.append(c)
        return c

    def plant(road):
        return SevenDOFModel(params=p_var, tyre=base.tyre_front, road=road, tyre_rear=base.tyre_rear, chassis=base.chassis)
    r = run_scenario(plant, ctrl_factory, sc, spec.dt)
    h = r.history
    k = {kk: float(v) for kk, v in r.kpis.items() if isinstance(v, (int, float)) and not kk.startswith("computation")}
    t, speed = h["t"], np.asarray(h["speed"], float)
    k.update({
        "max_sideslip_deg": math.degrees(float(np.nanmax(np.abs(np.asarray(h["beta"], float))))),
        "yaw_error_rms_degps": math.degrees(float(np.sqrt(np.nanmean((np.asarray(h["r"], float) - np.asarray(h["r_ref"], float)) ** 2)))),
        "max_lateral_offset_m": float(np.nanmax(np.abs(np.asarray(h["Y"], float)))),
        "v_max_kph": float(np.nanmax(speed) * 3.6), "v_end_kph": float(speed[-1] * 3.6),
        "max_abs_slip": float(np.nanmax(h["max_abs_slip"])),
        "regen_energy_kj": float(np.sum(h["regen_power_w"]) * spec.dt / 1e3),
        "max_jerk": float(np.nanmax(np.abs(np.diff(np.asarray(h["ax"], float)))) / spec.dt) if len(t) > 2 else 0.0,
        "distance_m": float(h["X"][-1]),
        "finite": float(np.all(np.isfinite(np.asarray(h["X"], float)))),
    })
    sup = [c for c in sink if hasattr(c, "summary")]
    if sup:
        s = sup[0].summary()
        k.update({"supervisor_transitions": float(s["transitions"]), "supervisor_conflicts": float(s["conflicts"]),
                  "allocation_condition_median": float(s["condition_median"]),
                  "allocation_ill_conditioned_share": float(s["ill_conditioned_share"])})
    hist = {key: np.asarray(h[key], float) for key in ("t", "X", "Y", "psi", "vx", "vy", "speed", "r", "r_ref", "beta",
                                                       "steer_f", "steer_r", "ax", "abs_active_n")}
    for w in ("FL", "FR", "RL", "RR"):
        for q in ("Fx", "Fy", "Fz"):
            hist[f"{q}_{w}"] = np.asarray(h[f"{q}_{w}"], float)
    return k, hist


def _run_ride(spec, m, lv, d):
    from vehicle_model.simulation.ride import run as run_ride
    vert = lv.get("vertical") or {"damping": "passive"}
    road = dict(m["road"])
    if "iso_class" in road:
        road["seed"] = int(road.get("seed", 1)) + d["road_seed"]
    man = dict(m["manoeuvre"], speed_kph=float(m["manoeuvre"].get("speed_kph", 60.0)) * d["speed"])
    r = run_ride(spec.vehicle_14dof, road, man, {"mode": vert["damping"]}, bool(vert.get("active_pitch", False)),
                 vert.get("preview") if vert["damping"] == "preview_skyhook" else None, duration=float(m.get("duration", 4.0)))
    hist = {k: r["history"][k] for k in ("t", "zs", "theta", "phi", "ddzs", "s_road", "zr_FL", "zu_FL", "zu_RL", "c_FL", "Fz_FL")}
    return {k: float(v) for k, v in r["kpis"].items()}, hist


def _run_rollover(spec, m, lv, d):
    from vehicle_model.simulation.roll_stability import run as run_roll
    var = dict(m.get("variant") or {})
    var["cg_height"] = float(var.get("cg_height", 0.55)) * d["hcog"]
    man = dict(m["manoeuvre"], speed_kph=float(m["manoeuvre"]["speed_kph"]) * d["speed"])
    r = run_roll(spec.vehicle_14dof, var, man, float(m.get("mu", 1.0)) * d["mu"], "rsc" if lv.get("rsc") else "passive",
                 duration=float(m.get("duration", 5.0)))
    k = {kk: float(v) for kk, v in r["kpis"].items() if isinstance(v, (int, float, bool))}
    hist = {key: np.asarray(r["history"][key], float) for key in ("t", "phi", "ltr", "vx", "r") if key in r["history"]}
    return k, hist


def _task(args):
    spec, m, lv, d = args
    try:
        fn = {"7dof": _run_7dof, "14dof_ride": _run_ride, "14dof_rollover": _run_rollover}[m["model"]]
        k, hist = fn(spec, m, lv, d)
        return {"manoeuvre": m["name"], "level": lv["name"], "draw": d["draw"], "kpis": k, "error": "", "history": hist}
    except Exception as ex:   # a failing run is a result
        return {"manoeuvre": m["name"], "level": lv["name"], "draw": d["draw"], "kpis": {},
                "error": f"{type(ex).__name__}: {ex}", "history": None}


def _bootstrap_ci(x, rng, n=2000):
    if len(x) < 2:
        return float("nan"), float("nan")
    means = rng.choice(x, size=(n, len(x)), replace=True).mean(axis=1)
    return float(np.percentile(means, 2.5)), float(np.percentile(means, 97.5))


def statistics(spec: VMMSpec, results: list) -> list:
    rng = np.random.default_rng(12345)
    rows = []
    for m in spec.manoeuvres:
        for kpi in m["primary"]:
            base = {r["draw"]: r["kpis"].get(kpi) for r in results if r["manoeuvre"] == m["name"] and r["level"] == spec.levels[0]["name"]}
            for lv in spec.levels:
                rs = [r for r in results if r["manoeuvre"] == m["name"] and r["level"] == lv["name"]]
                vals = np.array([r["kpis"].get(kpi, np.nan) for r in rs], float)
                ok = vals[np.isfinite(vals)]
                errors = sum(1 for r in rs if r["error"])
                diffs = np.array([r["kpis"].get(kpi, np.nan) - (base.get(r["draw"]) if base.get(r["draw"]) is not None else np.nan)
                                  for r in rs], float)
                diffs = diffs[np.isfinite(diffs)]
                lo, hi = _bootstrap_ci(ok, rng)
                rows.append({"manoeuvre": m["name"], "kpi": kpi, "level": lv["name"], "n": int(ok.size), "errors": errors,
                             "median": float(np.median(ok)) if ok.size else float("nan"),
                             "q25": float(np.percentile(ok, 25)) if ok.size else float("nan"),
                             "q75": float(np.percentile(ok, 75)) if ok.size else float("nan"),
                             "worst": float(np.max(ok)) if ok.size else float("nan"),
                             "mean": float(np.mean(ok)) if ok.size else float("nan"), "ci95_lo": lo, "ci95_hi": hi,
                             "paired_median_diff_vs_P0": float(np.median(diffs)) if diffs.size else float("nan"),
                             "shared_run": rs[0].get("shared_with", "") if rs else ""})
    return rows


def run(spec: VMMSpec, out: Path, verify: bool | None = None) -> dict:
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    ds = draws(spec)
    tasks, share = [], {}
    for m in spec.manoeuvres:
        done = {}
        for lv in spec.levels:
            eff = _effective_level(spec, m, lv)
            if eff in done:
                share[(m["name"], lv["name"])] = done[eff]
                continue
            done[eff] = lv["name"]
            tasks += [(spec, m, lv, d) for d in ds]
    workers = spec.workers or max(1, min(8, (os.cpu_count() or 2) - 1))
    if workers > 1:
        with ProcessPoolExecutor(max_workers=workers) as ex:
            res = list(ex.map(_task, tasks))
    else:
        res = [_task(t) for t in tasks]
    for (mname, lname), src in share.items():      # identical by construction: copy, flagged
        for r in [r for r in res if r["manoeuvre"] == mname and r["level"] == src]:
            res.append({**r, "level": lname, "shared_with": src, "history": r["history"]})
    order = {lv["name"]: i for i, lv in enumerate(spec.levels)}
    mord = {m["name"]: i for i, m in enumerate(spec.manoeuvres)}
    res.sort(key=lambda r: (mord[r["manoeuvre"]], order[r["level"]], r["draw"]))
    hist = {(r["manoeuvre"], r["level"], r["draw"]): r.pop("history") for r in res}
    stats = statistics(spec, res)
    summary = {"experiment": spec.to_dict(), "source": spec.source, "provenance": provenance(), "draws": ds, "verification": [],
               "information": "7-DOF: shared chain, information mode A, controllers on nominal parameters; 14-DOF: ride/rollover loops",
               "results": res, "statistics": stats}
    fm = set(spec.outputs.get("formats", ()))
    if "json" in fm:
        write_json(out / "results.json", summary)
    if "csv" in fm:
        keys = sorted({k for r in res for k in r["kpis"]})
        with open(out / "kpis.csv", "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["manoeuvre", "level", "draw", "shared_with", *keys, "error"])
            for r in res:
                w.writerow([r["manoeuvre"], r["level"], r["draw"], r.get("shared_with", ""), *(r["kpis"].get(k, "") for k in keys), r["error"]])
        with open(out / "stats.csv", "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=list(stats[0]))
            w.writeheader()
            w.writerows(stats)
        (out / "histories").mkdir(exist_ok=True)
        for (mn, ln, dr), h in hist.items():
            if h is None or dr != 0:
                continue
            with open(out / "histories" / f"{mn}__{ln}.csv", "w", newline="", encoding="utf-8") as f:
                w = csv.writer(f)
                w.writerow(list(h))
                w.writerows(zip(*h.values()))
    if "md" in fm:
        (out / "report.md").write_text(_report(spec, ds, res, stats), encoding="utf-8")
    if "png" in fm:
        _plots(spec, stats, res, out)
    return summary


def _report(spec, ds, res, stats) -> str:
    f3 = lambda x: "–" if x is None or (isinstance(x, float) and not math.isfinite(x)) else f"{x:.3g}"
    L = [f"# Vehicle motion management benchmark: {spec.name}", "", spec.description, "",
         f"Paired draws: {len(ds)} (draw 0 nominal); variations: {spec.variations}. Synthetic parameters, virtual models only.",
         "Levels marked *shared* are identical by construction on that model and were not run again.", ""]
    for m in spec.manoeuvres:
        L += [f"## {m['name']} ({m['model']})", "", m.get("description", ""), "",
              "| level | KPI | median | q25 | q75 | worst | mean [95 % CI] | paired median diff vs P0 | runs failed | shared |",
              "|---" * 10 + "|"]
        for s in [s for s in stats if s["manoeuvre"] == m["name"]]:
            L.append(f"| {s['level']} | {s['kpi']} | {f3(s['median'])} | {f3(s['q25'])} | {f3(s['q75'])} | {f3(s['worst'])} | "
                     f"{f3(s['mean'])} [{f3(s['ci95_lo'])}, {f3(s['ci95_hi'])}] | {f3(s['paired_median_diff_vs_P0'])} | "
                     f"{s['errors']} | {s['shared_run'] or ''} |")
        errs = sorted({r["error"] for r in res if r["manoeuvre"] == m["name"] and r["error"]})
        if errs:
            L += ["", "Failed runs: " + "; ".join(errs)]
        L.append("")
    return "\n".join(L) + "\n"


def _plots(spec, stats, res, out: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    from vehicle_model.visualization.theme import TEXT_MUTED, apply_dark_theme, style_axis
    apply_dark_theme()
    levels = [lv["name"] for lv in spec.levels]
    rows = [(m["name"], m["primary"][0]) for m in spec.manoeuvres]
    M = np.full((len(rows), len(levels)), np.nan)
    for i, (mn, kpi) in enumerate(rows):
        base = next((s["median"] for s in stats if s["manoeuvre"] == mn and s["kpi"] == kpi and s["level"] == levels[0]), np.nan)
        for j, ln in enumerate(levels):
            med = next((s["median"] for s in stats if s["manoeuvre"] == mn and s["kpi"] == kpi and s["level"] == ln), np.nan)
            M[i, j] = (med - base) / abs(base) * 100.0 if np.isfinite(base) and abs(base) > 1e-9 else np.nan
    fig, ax = plt.subplots(figsize=(1.3 * len(levels) + 5, 0.55 * len(rows) + 2))
    lim = np.nanmax(np.abs(np.clip(M, -100, 100))) if np.any(np.isfinite(M)) else 1.0
    im = ax.imshow(np.clip(M, -100, 100), cmap="RdBu_r", vmin=-lim, vmax=lim, aspect="auto")
    for i in range(len(rows)):
        for j in range(len(levels)):
            ax.text(j, i, "n/a" if not np.isfinite(M[i, j]) else f"{M[i, j]:+.0f}%", ha="center", va="center", fontsize=8,
                    color="#f5f5f5")
    ax.set_xticks(range(len(levels)), levels)
    ax.set_yticks(range(len(rows)), [f"{mn}: {k}" for mn, k in rows], fontsize=8)
    fig.colorbar(im, ax=ax, label="median change vs P0 [%] (lower is better, blue)")
    style_axis(ax, "Primary KPI per manoeuvre: median change against P0 over the paired draws", None, None)
    fig.tight_layout()
    fig.savefig(out / "matrix.png", dpi=110)
    plt.close(fig)


__all__ = ("KIND", "VMMSpec", "describe", "draws", "parse", "run", "statistics")
