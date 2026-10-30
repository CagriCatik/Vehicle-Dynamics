"""Experiment type ``rollover_benchmark``: roll stability control on the 14-DOF model.

A matrix of vehicle variants x manoeuvres x friction values x controllers, each run on the 14-DOF closed
loop of :mod:`vehicle_model.simulation.roll_stability` (1 ms plant step, 5 ms control, information mode A)::

    type: rollover_benchmark
    name: ...
    vehicle: configs/suspension/fourteen_dof.yaml
    variants:   [{name: nominal}, {name: high_cog, cg_height: 0.95, sprung_mass: 1520}]
    manoeuvres: [{name: fishhook_80, type: fishhook, speed_kph: 80, amplitude_deg: 8, rate_degps: 45}]
    mu: [1.0, 0.6]
    controllers:
      - {name: passive, controller: passive}
      - {name: RSC, controller: rsc}
      - {name: RSC + active roll (failed), controller: rsc_active_roll, active_suspension: failed}
    duration: 5.0

KPIs per run: largest LTR, smallest wheel load, time with one / two wheels without contact, largest roll angle
and roll rate, largest yaw-rate error, speed loss, RSC active and emergency time, activations, and the
outcome class (no wheel lift / wheel lift, recovered / outside the model's validity). A lifted wheel or
LTR = 1 is not reported as a rollover; roll angles beyond the validity of the small-angle suspension
kinematics are flagged instead.
"""

from __future__ import annotations

import csv
import itertools
import math
import os
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from .spec import _keys, _require
from .types import provenance, write_json

KIND = "rollover_benchmark"
VARIANT_KEYS = {"name", "cg_height", "sprung_mass", "Ixx", "k_arb_front", "k_arb_rear"}
_TOP = {"type", "name", "description", "vehicle", "variants", "manoeuvres", "mu", "controllers", "duration", "workers",
        "outputs"}
KPIS = ("ltr_max", "min_wheel_load_n", "lift_time_s", "two_wheel_lift_time_s", "roll_max_deg", "roll_rate_max_degps",
        "yaw_error_max_degps", "speed_loss_kph", "rsc_active_time_s", "rsc_emergency_time_s", "rsc_activations")


@dataclass(frozen=True)
class RolloverSpec:
    name: str
    vehicle: str
    variants: tuple
    manoeuvres: tuple
    mu: tuple
    controllers: tuple
    duration: float = 5.0
    workers: int | None = None
    outputs: dict = field(default_factory=dict)
    description: str = ""
    source: str = ""
    kind = KIND

    def to_dict(self) -> dict:
        return {"type": KIND, "name": self.name, "description": self.description, "vehicle": self.vehicle,
                "variants": [dict(v) for v in self.variants], "manoeuvres": [dict(m) for m in self.manoeuvres],
                "mu": list(self.mu), "controllers": [dict(c) for c in self.controllers], "duration": self.duration,
                "outputs": dict(self.outputs)}


def parse(data: dict, source: str = "<dict>") -> RolloverSpec:
    from vehicle_model.simulation.roll_stability import CONTROLLERS
    _require(data.get("type") == KIND, f"{source}: type must be {KIND!r}")
    _keys(data, _TOP, source, {"type", "name", "vehicle", "variants", "manoeuvres", "controllers"})
    _require(Path(data["vehicle"]).is_file(), f"{source}: vehicle file not found: {data['vehicle']}")
    for key in ("variants", "manoeuvres", "controllers"):
        _require(isinstance(data[key], list) and data[key], f"{source}: {key} must be a non-empty list")
        names = [x.get("name") if isinstance(x, dict) else None for x in data[key]]
        _require(all(isinstance(n, str) for n in names) and len(set(names)) == len(names),
                 f"{source}: every entry of {key} needs a unique name")
    for i, v in enumerate(data["variants"]):
        _keys(v, VARIANT_KEYS, f"{source}: variants[{i}]", {"name"})
        for k, x in v.items():
            if k != "name":
                _require(isinstance(x, (int, float)) and x > 0, f"{source}: variants[{i}].{k} must be > 0")
    for i, m in enumerate(data["manoeuvres"]):
        _require(m.get("type") in ("fishhook", "evasive"), f"{source}: manoeuvres[{i}].type must be fishhook or evasive")
        allowed = {"name", "type", "speed_kph", "amplitude_deg"} | ({"rate_degps", "hold_s"} if m["type"] == "fishhook"
                                                                    else {"frequency_hz"})
        _keys(m, allowed, f"{source}: manoeuvres[{i}]", {"name", "type", "speed_kph", "amplitude_deg"})
        _require(10 <= float(m["speed_kph"]) <= 200 and 0 < float(m["amplitude_deg"]) <= 30,
                 f"{source}: manoeuvres[{i}]: speed_kph in [10, 200], amplitude_deg in (0, 30]")
    mus = tuple(float(x) for x in data.get("mu", [1.0]))
    _require(mus and all(0.05 <= x <= 1.5 for x in mus), f"{source}: mu values must lie in [0.05, 1.5]")
    for i, c in enumerate(data["controllers"]):
        _keys(c, {"name", "controller", "active_suspension"}, f"{source}: controllers[{i}]", {"name", "controller"})
        _require(c["controller"] in CONTROLLERS, f"{source}: controllers[{i}].controller must be one of {CONTROLLERS}")
        _require(c.get("active_suspension", "healthy") in ("healthy", "failed"),
                 f"{source}: controllers[{i}].active_suspension must be healthy or failed")
    dur = float(data.get("duration", 5.0))
    _require(1.0 <= dur <= 30.0, f"{source}: duration must lie in [1, 30] s")
    outputs = dict(data.get("outputs") or {})
    _keys(outputs, {"directory", "formats"}, f"{source}: outputs")
    outputs.setdefault("formats", ["csv", "json", "png", "md"])
    outputs.setdefault("directory", f"output/experiments/{data['name']}")
    return RolloverSpec(str(data["name"]), str(data["vehicle"]), tuple(dict(v) for v in data["variants"]),
                        tuple(dict(m) for m in data["manoeuvres"]), mus, tuple(dict(c) for c in data["controllers"]),
                        dur, data.get("workers"), outputs, str(data.get("description", "")), source)


def describe(spec: RolloverSpec) -> str:
    return (f"14-DOF roll stability: {len(spec.variants)} variant(s) x {len(spec.manoeuvres)} manoeuvre(s) x "
            f"{len(spec.mu)} mu x {len(spec.controllers)} controller(s)")


def _task(args):
    from vehicle_model.simulation.roll_stability import run as run14
    spec, v, m, mu, c = args
    try:
        res = run14(spec.vehicle, {k: x for k, x in v.items() if k != "name"}, m, mu, c["controller"],
                    c.get("active_suspension", "healthy"), duration=spec.duration)
        k, err, hist = res["kpis"], "", res["history"]
    except Exception as ex:   # a failing run is a result
        k, err, hist = {}, f"{type(ex).__name__}: {ex}", None
    return {"variant": v["name"], "manoeuvre": m["name"], "mu": mu, "controller": c["name"], "kpis": k, "error": err,
            "history": hist}


def run(spec: RolloverSpec, out: Path, verify: bool | None = None) -> dict:
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    tasks = [(spec, v, m, mu, c) for v, m, mu, c in itertools.product(spec.variants, spec.manoeuvres, spec.mu, spec.controllers)]
    workers = spec.workers or max(1, min(8, (os.cpu_count() or 2) - 1))
    if workers > 1:
        with ProcessPoolExecutor(max_workers=workers) as ex:
            res = list(ex.map(_task, tasks))
    else:
        res = [_task(t) for t in tasks]
    hist = {(r["variant"], r["manoeuvre"], r["mu"], r["controller"]): r.pop("history") for r in res}
    summary = {"experiment": spec.to_dict(), "source": spec.source, "provenance": provenance(), "verification": [],
               "information_mode": "A (14-DOF plant telemetry)", "results": res}
    fm = set(spec.outputs.get("formats", ()))
    if "json" in fm:
        write_json(out / "results.json", summary)
    if "csv" in fm:
        with open(out / "kpis.csv", "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["variant", "manoeuvre", "mu", "controller", *KPIS, "outcome", "error"])
            for r in res:
                w.writerow([r["variant"], r["manoeuvre"], r["mu"], r["controller"], *(r["kpis"].get(k, "") for k in KPIS),
                            r["kpis"].get("outcome", ""), r["error"]])
        (out / "histories").mkdir(exist_ok=True)
        for (v, m, mu, c), h in hist.items():
            if h is None:
                continue
            keys = [k for k in h if k != "mode"]
            with open(out / "histories" / f"{v}__{m}__mu{mu}__{c}.csv".replace(" ", "_").replace("/", "_"), "w",
                      newline="", encoding="utf-8") as f:
                w = csv.writer(f)
                w.writerow([*keys, "mode"])
                for i in range(len(h["t"])):
                    w.writerow([*(h[k][i] for k in keys), h["mode"][i]])
    if "png" in fm:
        _plot(spec, res, hist, out)
    if "md" in fm:
        (out / "report.md").write_text(_report(spec, res), encoding="utf-8")
    return summary


_C = ("#3987e5", "#d95926", "#199e70", "#c98500", "#d55181", "#8a63d2")


def _plot(spec, res, hist, out: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from vehicle_model.visualization.theme import TEXT_MUTED, apply_dark_theme, style_axis
    apply_dark_theme()
    v, m, mu = spec.variants[-1]["name"], spec.manoeuvres[0]["name"], spec.mu[0]
    fig, ax = plt.subplots(2, 3, figsize=(18, 9))
    for i, c in enumerate(spec.controllers):
        h = hist.get((v, m, mu, c["name"]))
        if h is None:
            continue
        col = _C[i % len(_C)]
        ax[0, 0].plot(h["t"], h["ltr"], color=col, lw=1.4, label=c["name"])
        ax[0, 1].plot(h["t"], np.degrees(h["phi"]), color=col, lw=1.4)
        lifted = 4 - sum(h[f"contact_{w}"] for w in "FL FR RL RR".split())
        ax[0, 2].plot(h["t"], lifted, color=col, lw=1.4)
        ax[1, 0].plot(h["Y"], h["X"], color=col, lw=1.4)
        ax[1, 1].plot(h["t"], np.degrees(h["r"]), color=col, lw=1.4)
        ax[1, 2].plot(h["t"], h["vx"] * 3.6, color=col, lw=1.4)
    ax[0, 0].axhline(1.0, color=TEXT_MUTED, ls=":", lw=1)
    ax[0, 0].legend(fontsize=7)
    style_axis(ax[0, 0], f"LTR ({v}, {m}, mu {mu})", "t [s]", "-")
    style_axis(ax[0, 1], "roll angle", "t [s]", "deg")
    style_axis(ax[0, 2], "wheels without contact", "t [s]", "count")
    style_axis(ax[1, 0], "path", "Y [m]", "X [m]")
    style_axis(ax[1, 1], "yaw rate", "t [s]", "deg/s")
    style_axis(ax[1, 2], "speed", "t [s]", "km/h")
    fig.suptitle(f"{spec.name}: roll stability on the 14-DOF model (synthetic parameters, virtual model)")
    fig.tight_layout()
    fig.savefig(out / "rollover_timeseries.png", dpi=110)
    plt.close(fig)


def _report(spec, res) -> str:
    L = [f"# Roll stability benchmark: {spec.name}", "", "14-DOF model, information mode A, throttle released, manoeuvres "
         "fishhook-/evasive-inspired (not the normative procedures). Synthetic parameters, virtual model only. A lifted "
         "wheel or LTR = 1 is not a rollover statement.", "",
         "| variant | manoeuvre | mu | controller | LTR max | lift [s] | two-wheel lift [s] | roll max [deg] | yaw error max [deg/s] | speed loss [km/h] | RSC active [s] | outcome |",
         "|---" * 12 + "|"]
    f = lambda x: "–" if x == "" or x is None else (f"{x:.3g}" if isinstance(x, float) else str(x))
    for r in res:
        k = r["kpis"]
        L.append(f"| {r['variant']} | {r['manoeuvre']} | {r['mu']} | {r['controller']} | {f(k.get('ltr_max'))} | "
                 f"{f(k.get('lift_time_s'))} | {f(k.get('two_wheel_lift_time_s'))} | {f(k.get('roll_max_deg'))} | "
                 f"{f(k.get('yaw_error_max_degps'))} | {f(k.get('speed_loss_kph'))} | {f(k.get('rsc_active_time_s'))} | "
                 f"{k.get('outcome', r['error'])} |")
    return "\n".join(L) + "\n"


__all__ = ("KIND", "RolloverSpec", "describe", "parse", "run")
