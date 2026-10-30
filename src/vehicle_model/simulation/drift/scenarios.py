"""Initiation from normal driving, hold, exit and recovery (scenarios S04, S05) with success rates.

Every run starts in straight driving at the drift speed (wheels rolling freely) and goes
through the supervised controller (:class:`~vehicle_model.drift.control_adapter.DriftSupervisedController`)
on the existing 7-DOF plant via the ScenarioRunner: no state is set by hand after t = 0.

S04: drift requested at ``request_s``; success = HOLD reached and held until the exit request
(or the end) without FAILED. S05: at ``exit_s`` the supervised exit is compared with an
immediate shutdown (drive torque 0 and steer ramped to 0 at the rate limit, no supervisor).

The sweep covers ``speed_mps x beta_deg x mu x directions``; runs are independent and run in a
process pool (same results as serial). Outputs: ``kpi.json``, ``kpi.csv``, ``mode_log.json``,
state histories of the first run per direction, ``initiation_sequence.png``, ``report.md``,
``run_config_resolved.yaml``, ``metadata.json``.
"""

from __future__ import annotations

import csv
import json
import math
import os
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path

import numpy as np
import yaml

from .config import keys, load_yaml, numbers, require, vehicle_model

_TOP = {"name", "description", "vehicle", "tyres", "powertrain", "sweep", "directions", "approach", "supervisor",
        "simulation", "workers", "outputs"}
_SUP = {"request_s", "exit_s", "capture_beta_deg", "capture_r_degps", "capture_v_mps", "capture_hold_s",
        "initiate_timeout_s", "loss_beta_deg", "exit_beta_deg", "recover_timeout_s", "spin_beta_deg"}


@dataclass(frozen=True)
class InitiationSpec:
    name: str
    vehicle: str
    tyres: tuple
    speeds: tuple
    betas_deg: tuple
    mus: tuple
    directions: tuple
    approach_ay_fraction: float
    supervisor: tuple                 # sorted items of the supervisor keys
    dt: float
    duration: float
    workers: int | None
    output_dir: str
    source: str = ""

    def cases(self):
        for mu in self.mus:
            for d in self.directions:
                for v in self.speeds:
                    for b in self.betas_deg:
                        yield {"mu": mu, "direction": d, "speed": v, "beta_deg": b}


def parse(data: dict, source: str = "<dict>") -> InitiationSpec:
    keys(data, _TOP, source, {"name", "vehicle", "sweep"})
    tyres = dict(data.get("tyres") or {})
    vehicle_model(data["vehicle"], tyres, source)
    pt = dict(data.get("powertrain") or {"layout": "rwd"})
    keys(pt, {"layout"}, f"{source}: powertrain")
    require(pt.get("layout", "rwd") == "rwd", f"{source}: powertrain.layout: only 'rwd' is implemented")
    sw = data["sweep"]
    keys(sw, {"speed_mps", "beta_deg", "mu"}, f"{source}: sweep", {"speed_mps", "beta_deg", "mu"})
    speeds = numbers(sw["speed_mps"], f"{source}: sweep.speed_mps", 5.0, 40.0)
    betas = numbers(sw["beta_deg"], f"{source}: sweep.beta_deg", 5.0, 45.0)
    mus = numbers(sw["mu"], f"{source}: sweep.mu", 0.2, 1.5)
    dirs = data.get("directions", ["left", "right"])
    require(isinstance(dirs, list) and dirs and set(dirs) <= {"left", "right"} and len(set(dirs)) == len(dirs),
            f"{source}: directions must be a list of 'left' / 'right'")
    ap = dict(data.get("approach") or {})
    keys(ap, {"ay_fraction"}, f"{source}: approach")
    ayf = float(ap.get("ay_fraction", 0.6))
    require(0.05 <= ayf <= 0.9, f"{source}: approach.ay_fraction must lie in [0.05, 0.9]")
    sup = dict(data.get("supervisor") or {})
    keys(sup, _SUP, f"{source}: supervisor")
    sim = dict(data.get("simulation") or {})
    keys(sim, {"dt_s", "duration_s"}, f"{source}: simulation")
    dt, dur = float(sim.get("dt_s", 0.005)), float(sim.get("duration_s", 45.0))
    require(0.001 <= dt <= 0.02 and 5.0 <= dur <= 200.0, f"{source}: simulation out of range")
    workers = data.get("workers")
    require(workers is None or (isinstance(workers, int) and workers >= 1), f"{source}: workers must be >= 1")
    out = dict(data.get("outputs") or {})
    keys(out, {"directory"}, f"{source}: outputs")
    return InitiationSpec(str(data["name"]), str(data["vehicle"]), tuple(sorted(tyres.items())), tuple(speeds),
                          tuple(betas), tuple(mus), tuple(dirs), ayf, tuple(sorted(sup.items())), dt, dur, workers,
                          str(out.get("directory", f"output/drift/{data['name']}")), source)


def load(path):
    return parse(load_yaml(path), str(path))


_SUP_MAP = {   # YAML key -> (SupervisorSettings field, factor to SI)
    "request_s": ("request_s", 1.0), "exit_s": ("exit_s", 1.0), "capture_beta_deg": ("capture_beta", math.pi / 180),
    "capture_r_degps": ("capture_r", math.pi / 180), "capture_v_mps": ("capture_v", 1.0),
    "capture_hold_s": ("capture_hold_s", 1.0), "initiate_timeout_s": ("initiate_timeout_s", 1.0),
    "loss_beta_deg": ("loss_beta", math.pi / 180), "exit_beta_deg": ("exit_beta", math.pi / 180),
    "recover_timeout_s": ("recover_timeout_s", 1.0), "spin_beta_deg": ("spin_beta", math.pi / 180)}


def supervisor_settings(spec: InitiationSpec):
    from vehicle_model.drift.supervisor import SupervisorSettings
    kw = {}
    for k, v in spec.supervisor:
        name, f = _SUP_MAP[k]
        kw[name] = None if v is None else float(v) * f
    return SupervisorSettings(**kw)


@lru_cache(maxsize=32)
def _design(vehicle: str, tyres: tuple, speed: float, beta_deg: float, mu: float, direction: str, ayf: float):
    """Drift equilibrium + LQR and the approach cornering equilibrium (cached per worker)."""
    from vehicle_model.drift import DriftEquilibriumSolver, DriftReference
    from vehicle_model.drift.equilibrium import approach_equilibrium
    from vehicle_model.drift.linearization import linearise
    from vehicle_model.drift.lqr import design_lqr
    m = vehicle_model(vehicle, dict(tyres), "<design>")
    sol = DriftEquilibriumSolver(m)
    sign = -1.0 if direction == "left" else 1.0
    de = sol.solve(DriftReference(speed, sign * math.radians(beta_deg), mu))
    if not de.physical_feasible:
        return m, de, None, None
    dl = design_lqr(linearise(sol.plant(mu), de), m.params)
    ae = approach_equilibrium(m, speed, mu, ayf, direction)
    return m, de, dl, ae


def run_case(args) -> dict:
    """One sweep point: supervised sequence and, if an exit time is set, the immediate-shutdown comparison."""
    from vehicle_model.core.types import SevenDOFState
    from vehicle_model.drift.control_adapter import DriftSupervisedController
    from vehicle_model.drift.supervisor import DriftSupervisor
    from vehicle_model.roads.surface import UniformRoad

    from .hold import simulate
    spec, case, keep_history = args
    m, de, dl, ae = _design(spec.vehicle, spec.tyres, case["speed"], case["beta_deg"], case["mu"], case["direction"],
                            spec.approach_ay_fraction)
    res = {**case}
    if dl is None:
        res.update({"status": "INFEASIBLE", "reasons": list(de.reasons)})
        return res
    import copy
    plant = copy.copy(m)
    plant.road = UniformRoad(mu=case["mu"])
    st = supervisor_settings(spec)
    p = m.params
    x0 = SevenDOFState(vx=case["speed"], omega=np.full(4, case["speed"] / p.Reff))
    sup = DriftSupervisor(st, de.beta, de.yaw_rate, de.speed, ae.yaw_rate)
    ctrl = DriftSupervisedController(p, de, dl, ae, sup, case["speed"])
    modes = []
    orig = ctrl.compute_control

    def logged(*a, **k):
        out = orig(*a, **k)
        modes.append(out.diagnostics["mode"])
        return out
    ctrl.compute_control = logged
    h, sat = simulate(plant, ctrl, x0, spec.duration, spec.dt)
    res.update(_kpis(h, np.array(modes, dtype=object), sat, de, st, spec.dt))
    res["transitions"] = [{"t": tr.t, "from": tr.source, "to": tr.target, "reason": tr.reason} for tr in sup.transitions]
    res["equilibrium"] = {"radius_m": de.radius, "yaw_rate_degps": math.degrees(de.yaw_rate),
                          "steer_deg": math.degrees(de.steer_front), "validity": de.validity}
    if st.exit_s is not None and res["hold_reached"]:
        res["shutdown"] = _shutdown(plant, ctrl, sup, de, ae, x0, spec, st)
    if keep_history:
        res["history"] = h
    return res


def _kpis(h, modes, sat, de, st, dt) -> dict:
    from vehicle_model.drift.metrics import hold_kpis
    t = h.time
    beta = np.arctan2(h.vy, h.vx)
    hold = modes == "HOLD"
    out = {"final_mode": str(modes[-1]), "hold_reached": bool(np.any(hold)),
           "failed": bool(np.any(modes == "FAILED")), "saturation_fraction": float(np.mean(sat))}
    if out["hold_reached"]:
        i0 = int(np.argmax(hold))
        out["time_to_hold_s"] = float(t[i0] - st.request_s)
        k = hold_kpis(t[hold], h.vx[hold], h.vy[hold], h.r[hold], de.beta, de.yaw_rate, de.speed,
                      t[i0] + 2.0 if np.sum(hold) * dt > 4.0 else t[i0])
        out.update({f"hold_{n}": v for n, v in k.items() if n in ("beta_rmse_deg", "yaw_rmse_degps", "speed_rmse_mps")})
    ex = modes == "EXIT"
    if np.any(ex):
        after = t >= t[int(np.argmax(ex))]
        out["exit_peak_beta_deg"] = float(np.degrees(np.max(np.abs(beta[after]))))
        out["exit_peak_yaw_degps"] = float(np.degrees(np.max(np.abs(h.r[after]))))
        out["exit_min_speed_mps"] = float(np.min(np.hypot(h.vx[after], h.vy[after])))
        dis = after & (modes == "DISABLED")
        out["exit_to_normal_s"] = float(t[int(np.argmax(dis))] - t[int(np.argmax(ex))]) if np.any(dis) else None
    out["success"] = out["hold_reached"] and not out["failed"] and (
        st.exit_s is None or out.get("exit_to_normal_s") is not None)
    return out


class _Shutdown:
    """Immediate shutdown at the exit time: torque 0, steer ramped to 0 at the rate limit (no supervisor)."""

    def __init__(self, inner, t_off: float, params):
        self.inner, self.t_off, self.p = inner, t_off, params
        self.t, self.steer = 0.0, 0.0

    def compute_control(self, state, target, telemetry, dt):
        from vehicle_model.controllers.framework import ControlOutput
        self.t += dt
        if self.t < self.t_off:
            out = self.inner.compute_control(state, target, telemetry, dt)
            self.steer = out.steer_f
            return out
        step = self.p.max_steer_rate * dt
        self.steer -= float(np.clip(self.steer, -step, step))
        return ControlOutput(steer_f=self.steer, T_drive=np.zeros(4), T_brake=np.zeros(4))


def _shutdown(plant, ctrl, sup, de, ae, x0, spec, st) -> dict:
    from vehicle_model.drift.control_adapter import DriftSupervisedController
    from vehicle_model.drift.supervisor import DriftSupervisor

    from .hold import simulate
    import dataclasses
    st2 = dataclasses.replace(st, exit_s=None)
    sup2 = DriftSupervisor(st2, de.beta, de.yaw_rate, de.speed, ae.yaw_rate)
    inner = DriftSupervisedController(ctrl.params, de, ctrl.drift.lqr, ae, sup2, ctrl.cruise_speed)
    h, _ = simulate(plant, _Shutdown(inner, st.exit_s, ctrl.params), x0, spec.duration, spec.dt)
    after = h.time >= st.exit_s
    beta = np.arctan2(h.vy[after], h.vx[after])
    return {"peak_beta_deg": float(np.degrees(np.max(np.abs(np.unwrap(beta))))),
            "peak_yaw_degps": float(np.degrees(np.max(np.abs(h.r[after])))),
            "min_speed_mps": float(np.min(np.hypot(h.vx[after], h.vy[after]))),
            "spin": bool(np.max(np.abs(np.unwrap(beta))) >= st.spin_beta),
            "final_beta_deg": float(np.degrees(beta[-1]))}


def run(spec: InitiationSpec, out_dir=None) -> dict:
    from vehicle_model.experiments.types import provenance
    from vehicle_model.visualization.drift_plots import plot_initiation

    out = Path(out_dir or spec.output_dir)
    out.mkdir(parents=True, exist_ok=True)
    cases = list(spec.cases())
    first = {}
    for i, c in enumerate(cases):
        first.setdefault(c["direction"], i)
    tasks = [(spec, c, i in first.values()) for i, c in enumerate(cases)]
    workers = spec.workers or max(1, min(8, (os.cpu_count() or 2) - 1))
    if workers > 1 and len(tasks) > 1:
        with ProcessPoolExecutor(max_workers=workers) as ex:
            res = list(ex.map(run_case, tasks))
    else:
        res = [run_case(t) for t in tasks]
    hist = {r["direction"]: r.pop("history") for r in res if "history" in r}
    for d, h in hist.items():
        from .hold import _write_history
        _write_history(out / f"state_history_{d}.csv", h)
    ok = [r for r in res if r.get("success")]
    summary = {"runs": len(res), "success": len(ok), "success_rate": len(ok) / len(res) if res else 0.0,
               "infeasible": sum(r.get("status") == "INFEASIBLE" for r in res)}
    (out / "run_config_resolved.yaml").write_text(yaml.safe_dump(load_yaml(spec.source) if Path(spec.source).is_file()
                                                                 else {}, sort_keys=False), encoding="utf-8")
    (out / "metadata.json").write_text(json.dumps({"source": spec.source, "provenance": provenance(), **summary},
                                                  indent=2), encoding="utf-8")
    (out / "kpi.json").write_text(json.dumps(res, indent=2, default=float), encoding="utf-8")
    (out / "mode_log.json").write_text(json.dumps([{k: r.get(k) for k in ("mu", "direction", "speed", "beta_deg",
                                                                          "transitions")} for r in res], indent=2,
                                                  default=float), encoding="utf-8")
    cols = ("mu", "direction", "speed", "beta_deg", "status", "success", "hold_reached", "failed", "final_mode",
            "time_to_hold_s", "hold_beta_rmse_deg", "hold_yaw_rmse_degps", "hold_speed_rmse_mps", "exit_peak_beta_deg",
            "exit_peak_yaw_degps", "exit_min_speed_mps", "exit_to_normal_s", "saturation_fraction")
    with open(out / "kpi.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow([*cols, "shutdown_spin", "shutdown_peak_beta_deg"])
        for r in res:
            sd = r.get("shutdown") or {}
            w.writerow([r.get(c, "") for c in cols] + [sd.get("spin", ""), sd.get("peak_beta_deg", "")])
    if hist:
        d0 = next(iter(hist))
        r0 = next(r for r in res if r["direction"] == d0)
        plot_initiation(hist[d0], r0, out / "initiation_sequence.png", spec.name)
    (out / "report.md").write_text(_report(spec, res, summary), encoding="utf-8")
    return {"results": res, "summary": summary, "directory": str(out)}


def _report(spec, res, summary) -> str:
    L = [f"# Drift initiation, hold and exit: {spec.name}", "",
         f"Start in straight driving; drift requested at {dict(spec.supervisor).get('request_s', 0.5)} s, exit at "
         f"{dict(spec.supervisor).get('exit_s')} s; {spec.duration:g} s per run, dt {spec.dt} s. Approach: stable "
         f"cornering at {spec.approach_ay_fraction:g} mu g. Synthetic parameters, virtual model only.", "",
         f"**Success: {summary['success']} of {summary['runs']} ({summary['success_rate']:.0%})**, infeasible drift "
         f"targets: {summary['infeasible']}.", "",
         "| mu | dir | v [m/s] | beta* [deg] | result | final mode | time to HOLD [s] | HOLD beta / r RMSE | "
         "exit: peak beta [deg] / peak r [deg/s] / min v [m/s] / to normal [s] | shutdown instead |", "|---" * 10 + "|"]
    f = lambda x, n=1: "–" if x is None else f"{x:.{n}f}"
    for r in res:
        if r.get("status") == "INFEASIBLE":
            L.append(f"| {r['mu']:g} | {r['direction']} | {r['speed']:g} | {r['beta_deg']:g} | INFEASIBLE "
                     f"({', '.join(r['reasons'])}) | | | | | |")
            continue
        sd = r.get("shutdown")
        sdt = "–" if not sd else ("spin" if sd["spin"] else f"no spin, peak beta {sd['peak_beta_deg']:.0f} deg")
        L.append(f"| {r['mu']:g} | {r['direction']} | {r['speed']:g} | {r['beta_deg']:g} | "
                 f"{'PASS' if r['success'] else 'FAIL'} | {r['final_mode']} | {f(r.get('time_to_hold_s'))} | "
                 f"{f(r.get('hold_beta_rmse_deg'), 2)} / {f(r.get('hold_yaw_rmse_degps'), 2)} | "
                 f"{f(r.get('exit_peak_beta_deg'), 0)} / {f(r.get('exit_peak_yaw_degps'), 0)} / "
                 f"{f(r.get('exit_min_speed_mps'))} / {f(r.get('exit_to_normal_s'))} | {sdt} |")
    L += ["", "Success: HOLD reached, no FAILED, and (with an exit time) normal driving restored. Mode sequences with",
          "reasons: `mode_log.json`. Shutdown: at the exit time drive torque 0 and steer ramped to 0, no supervisor.", ""]
    return "\n".join(L)


def main(argv=None) -> int:
    import argparse
    ap = argparse.ArgumentParser(description="Drift initiation, hold and exit")
    ap.add_argument("config", nargs="?", default="configs/drift/initiation.yaml")
    ap.add_argument("--out", default=None)
    a = ap.parse_args(argv)
    res = run(load(a.config), a.out)
    s = res["summary"]
    print(f"{s['success']} of {s['runs']} runs successful ({s['success_rate']:.0%}) -> {res['directory']}")
    return 0


__all__ = ("InitiationSpec", "load", "main", "parse", "run", "run_case", "supervisor_settings")
