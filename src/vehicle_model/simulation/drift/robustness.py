"""Robustness, model variation and numerical checks of the drift hold.

The drift LQR is designed once from the nominal vehicle (``configs/drift/robustness.yaml``:
speed, sideslip, friction). Every case runs the same scenario on a deviating plant and is
classified; nothing is re-tuned per case. The deviations reuse the robust control uncertainty model
(:class:`~vehicle_model.simulation.uncertainty.UncertaintyCase`: mass, inertia, CoG,
cornering stiffness, friction, actuator delay/lag, control latency, sensor noise/bias/latency)
plus three drift-specific ones:

    hcog_m            centre-of-gravity height of the plant [m]
    tyre_model        "transient": the TransientSevenDOFModel (tyre relaxation) as plant
    split_road        friction ``mu_low`` above the line ``Y = split_y`` (the circle crosses it twice per lap)

Feedback modes: ``ideal`` (true state), ``estimated`` (motion state from sensors and an observer,
Mode B) and ``sensor_constrained`` (Mode C); sensor cases run with several seeds and report the
spread (mean, min, max). Observer per case (``observer``): ``drift`` (default,
:class:`~vehicle_model.drift.estimator.DriftVelocityObserver`: IMU kinematics + free-rolling front
wheels) or ``blended`` (the general :class:`~vehicle_model.estimation.blended_sideslip.BlendedSideslipObserver`,
kept as the reference that fails in a drift: speed from the spinning rear wheel, start at beta = 0).

Controller (top-level ``controller``): ``integral: true`` adds the integral of the sideslip error
to the LQR (``q_integral`` its scaled weight), default false (the drift LQR design).

Scenario: start on the drift equilibrium with a sideslip perturbation, hold for ``duration_s``.
Classes per run:

    held        no drift loss and |beta - beta*| < hold_tol at the end
    degraded    no drift loss but the final sideslip error >= hold_tol
    lost        |beta - beta*| >= 30 deg at some time (spin or exit)
    failed      non-finite state or exception

Numerical checks (always run): RK4 step-size convergence of the closed loop (final state at
dt, dt/2, dt/4; observed order) and the wheel-load balance ``sum F_z = m g`` along the run.
"""

from __future__ import annotations

import copy
import dataclasses
import json
import math
import os
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np
import yaml

from .config import keys, load_yaml, numbers, require, vehicle_model

_TOP = {"name", "description", "vehicle", "tyres", "powertrain", "reference", "scenario", "cases", "seeds", "workers",
        "numerics", "outputs", "controller"}
_EXTRA = {"name", "feedback", "hcog_m", "tyre_model", "split_road", "speed_factor", "sensor_seeds", "observer"}


def load(path) -> dict:
    from vehicle_model.simulation.uncertainty import FIELDS
    data = load_yaml(path)
    src = str(path)
    keys(data, _TOP, src, {"name", "vehicle", "reference", "scenario", "cases"})
    vehicle_model(data["vehicle"], dict(data.get("tyres") or {}), src)
    keys(data["reference"], {"speed_mps", "beta_deg", "mu"}, f"{src}: reference", {"speed_mps", "beta_deg", "mu"})
    keys(data["scenario"], {"duration_s", "dt_s", "perturbation_beta_deg", "hold_tol_deg"}, f"{src}: scenario")
    require(isinstance(data["cases"], list) and data["cases"], f"{src}: cases must be a non-empty list")
    names = []
    for i, c in enumerate(data["cases"]):
        keys(c, set(FIELDS) | _EXTRA, f"{src}: cases[{i}]", {"name"})
        require(c.get("feedback", "ideal") in ("ideal", "estimated", "sensor_constrained"),
                f"{src}: cases[{i}].feedback must be ideal, estimated or sensor_constrained")
        require(c.get("tyre_model", "quasi_static") in ("quasi_static", "transient"),
                f"{src}: cases[{i}].tyre_model must be quasi_static or transient")
        require(c.get("observer", "drift") in ("drift", "blended"), f"{src}: cases[{i}].observer must be drift or blended")
        if "split_road" in c:
            keys(c["split_road"], {"mu_low", "split_y"}, f"{src}: cases[{i}].split_road", {"mu_low", "split_y"})
        names.append(c["name"])
    require(len(set(names)) == len(names), f"{src}: case names must be unique")
    numbers(data.get("seeds", [0]), f"{src}: seeds", 0, 1e9)
    ctl = data.get("controller") or {}
    keys(ctl, {"integral", "q_integral"}, f"{src}: controller")
    require(isinstance(ctl.get("integral", False), bool), f"{src}: controller.integral must be true or false")
    numbers([ctl.get("q_integral", 10.0)], f"{src}: controller.q_integral", 0.0, 1e6)
    data["_source"] = src
    return data


def _design(cfg):
    from vehicle_model.drift import DriftEquilibriumSolver, DriftReference
    from vehicle_model.drift.linearization import linearise
    from vehicle_model.drift.lqr import LQRWeights, design_lqr
    ref, ctl = cfg["reference"], cfg.get("controller") or {}
    m = vehicle_model(cfg["vehicle"], dict(cfg.get("tyres") or {}), cfg["_source"])
    sol = DriftEquilibriumSolver(m)
    eq = sol.solve(DriftReference(float(ref["speed_mps"]), -math.radians(float(ref["beta_deg"])), float(ref["mu"])))
    if not eq.physical_feasible:
        raise ValueError(f"nominal drift equilibrium infeasible: {eq.reasons}")
    integral = bool(ctl.get("integral", False))
    w = LQRWeights(q_integral=float(ctl.get("q_integral", LQRWeights.q_integral)))
    return m, eq, design_lqr(linearise(sol.plant(float(ref["mu"])), eq), m.params, w, integral=integral)


def _plant(m, case: dict, mu: float):
    from vehicle_model.roads.surface import MuSplitRoad, UniformRoad
    from vehicle_model.simulation.uncertainty import UncertaintyCase, plant_factory
    from vehicle_model.vehicle_models import TransientSevenDOFModel
    unc = UncertaintyCase.from_dict({k: v for k, v in case.items() if k not in _EXTRA or k == "name"})
    road = UniformRoad(mu=mu)
    if "split_road" in case:
        sr = case["split_road"]
        road = MuSplitRoad(mu_left=float(sr["mu_low"]), mu_right=mu, split_y=float(sr["split_y"]), transition_width=0.5)
    p = plant_factory(m, unc)(road)
    if "hcog_m" in case:
        p.params = dataclasses.replace(p.params, hcog=float(case["hcog_m"]))
        p._load_params = dataclasses.replace(p._load_params, hcog=float(case["hcog_m"]))
    if case.get("tyre_model") == "transient":
        p = TransientSevenDOFModel(params=p.params, tyre=p.tyre_front, tyre_rear=p.tyre_rear, road=p.road)
    return p, unc


class _SteerSensor:
    """Feeds the last applied steer command as the measured steering angle to the sensor wrapper.

    ``EstimatedStateFeedback`` measures the steering angle from ``target.driver_steer_f`` (the driver's
    command in the motion-control setting); in the drift loop the controller itself steers, so the sensor
    sees the command applied in the previous step, as a steering-angle sensor would.
    """

    def __init__(self, inner):
        from vehicle_model.controllers.framework import ControlTarget
        self.inner, self.steer, self._target = inner, None, ControlTarget

    def reset(self):
        self.inner.reset()
        self.steer = None

    def compute_control(self, state, target, telemetry, dt):
        tgt = self._target(driver_steer_f=0.0 if self.steer is None else self.steer)
        out = self.inner.compute_control(state, tgt, telemetry, dt)
        self.steer = out.steer_f
        return out


def run_case(args) -> dict:
    """One case and seed (module level, for the process pool)."""
    from vehicle_model.controllers.estimated_feedback import EstimatedStateFeedback
    from vehicle_model.core.types import SevenDOFState
    from vehicle_model.drift.control_adapter import DriftLQRController
    from vehicle_model.drift.equilibrium import rwd_input
    from vehicle_model.drift.estimator import DriftVelocityObserver
    from vehicle_model.drift.metrics import hold_kpis
    from vehicle_model.estimation.blended_sideslip import BlendedSideslipObserver
    from vehicle_model.simulation.uncertainty import sensor_config, wrap_controller

    from .hold import simulate
    cfg, case, seed = args
    sc = cfg["scenario"]
    dt, dur = float(sc.get("dt_s", 0.005)), float(sc.get("duration_s", 10.0))
    tol = math.radians(float(sc.get("hold_tol_deg", 5.0)))
    m, eq, lqr = _design(cfg)
    mu = float(cfg["reference"]["mu"])
    out = {"case": case["name"], "seed": seed, "feedback": case.get("feedback", "ideal")}
    if out["feedback"] != "ideal":
        out["observer"] = case.get("observer", "drift")
    try:
        plant, unc = _plant(m, case, mu)
        ctrl = DriftLQRController(m.params, eq, lqr)               # nominal design, never re-tuned
        if out["feedback"] != "ideal":
            obs = DriftVelocityObserver(m.params) if out["observer"] == "drift" else BlendedSideslipObserver(m.params)
            ctrl = _SteerSensor(EstimatedStateFeedback(ctrl, obs, sensor_config(unc),
                                                       seed=int(seed), params=m.params,
                                                       mode="C" if out["feedback"] == "sensor_constrained" else "B"))
        ctrl = wrap_controller(ctrl, unc)
        v = eq.speed * float(case.get("speed_factor", 1.0))
        b = eq.beta - math.radians(float(sc.get("perturbation_beta_deg", 3.0)))
        x0 = SevenDOFState(vx=v * math.cos(b), vy=v * math.sin(b), r=eq.yaw_rate, omega=eq.omega * v / eq.speed)
        if hasattr(plant, "steady_slip_state"):
            x0 = plant.steady_slip_state(x0, rwd_input(eq.steer_front, eq.wheel_drive_torque[2]))
        h, sat = simulate(plant, ctrl, x0, dur, dt)
        if not (np.all(np.isfinite(h.vx)) and np.all(np.isfinite(h.vy))):
            raise FloatingPointError("non-finite state")
        k = hold_kpis(h.time, h.vx, h.vy, h.r, eq.beta, eq.yaw_rate, eq.speed, dur / 2.0, sat)
        final_err = abs(math.radians(k["final_beta_error_deg"]))
        out.update(k)
        out["final_speed_error_mps"] = float(math.hypot(h.vx[-1], h.vy[-1]) - eq.speed)
        out["class"] = "lost" if k["stability_loss"] else ("held" if final_err < tol else "degraded")
        out["load_balance_max_rel_error"] = float(np.max(np.abs(np.sum(h.Fz, axis=1) - plant.params.mass * plant.params.g))
                                                  / (plant.params.mass * plant.params.g))
    except Exception as ex:  # an exception is a result of the case, not a crash of the study
        out.update({"class": "failed", "error": f"{type(ex).__name__}: {ex}"})
    return out


def numerics(cfg) -> dict:
    """RK4 step-size convergence of the closed-loop hold (nominal plant)."""
    from vehicle_model.core.types import SevenDOFState
    from vehicle_model.drift.control_adapter import DriftLQRController
    from vehicle_model.roads.surface import UniformRoad

    from .hold import simulate
    m, eq, lqr = _design(cfg)
    plant = copy.copy(m)
    plant.road = UniformRoad(mu=float(cfg["reference"]["mu"]))
    nc = cfg.get("numerics") or {}
    T, dt0 = float(nc.get("duration_s", 2.0)), float(nc.get("dt_s", 0.01))
    b = eq.beta - math.radians(3.0)
    x0 = SevenDOFState(vx=eq.speed * math.cos(b), vy=eq.speed * math.sin(b), r=eq.yaw_rate, omega=eq.omega.copy())
    finals = []
    steps = [dt0, dt0 / 2, dt0 / 4, dt0 / 8]
    for dt in steps:
        h, _ = simulate(plant, DriftLQRController(m.params, eq, lqr), x0, T, dt)
        finals.append(np.r_[h.vx[-1], h.vy[-1], h.r[-1]])
    e = [float(np.max(np.abs(finals[i] - finals[-1]))) for i in range(len(steps) - 1)]
    orders = [math.log2(e[i] / e[i + 1]) if e[i + 1] > 0 else float("inf") for i in range(len(e) - 1)]
    note = ("closed loop: the controller is sampled with the same step and holds its input over the step, so the "
            "observed order stays below the order 4 of the RK4 plant integration")
    if lqr.integral:
        note += ("; with the sideslip integral, the conditional integration (anti-windup) switches with saturation, "
                 "the final state is not smooth in the step and the observed order is not meaningful: the error level "
                 "is the result")
    return {"steps": steps, "errors_vs_finest": e, "observed_orders": orders, "note": note}


def run(cfg, out_dir=None) -> dict:
    from vehicle_model.experiments.types import provenance
    out = Path(out_dir or cfg.get("outputs", {}).get("directory", "output/drift/robustness"))
    out.mkdir(parents=True, exist_ok=True)
    seeds = [int(s) for s in cfg.get("seeds", [0])]
    tasks = []
    for c in cfg["cases"]:
        fb = c.get("feedback", "ideal")
        for s in (seeds if fb != "ideal" else seeds[:1]):
            tasks.append((cfg, c, s))
    workers = int(cfg.get("workers") or max(1, min(8, (os.cpu_count() or 2) - 1)))
    if workers > 1:
        with ProcessPoolExecutor(max_workers=workers) as ex:
            res = list(ex.map(run_case, tasks))
    else:
        res = [run_case(t) for t in tasks]
    num = numerics(cfg)
    summary = _summary(cfg, res)
    clean = {k: v for k, v in cfg.items() if not k.startswith("_")}
    (out / "run_config_resolved.yaml").write_text(yaml.safe_dump(clean, sort_keys=False), encoding="utf-8")
    (out / "metadata.json").write_text(json.dumps({"source": cfg["_source"], "provenance": provenance()}, indent=2),
                                       encoding="utf-8")
    (out / "kpi.json").write_text(json.dumps({"runs": res, "summary": summary, "numerics": num}, indent=2, default=float),
                                  encoding="utf-8")
    (out / "report.md").write_text(_report(cfg, summary, num), encoding="utf-8")
    return {"runs": res, "summary": summary, "numerics": num, "directory": str(out)}


def _summary(cfg, res) -> list:
    rows = []
    for c in cfg["cases"]:
        rs = [r for r in res if r["case"] == c["name"]]
        cls = [r["class"] for r in rs]
        ok = [r for r in rs if r["class"] != "failed"]

        def stat(key):
            x = np.array([r[key] for r in ok], float)
            return (float(np.mean(x)), float(np.min(x)), float(np.max(x))) if x.size else (float("nan"),) * 3
        rows.append({"case": c["name"], "feedback": c.get("feedback", "ideal"), "runs": len(rs),
                     "classes": {k: cls.count(k) for k in ("held", "degraded", "lost", "failed") if cls.count(k)},
                     "beta_rmse_deg": stat("beta_rmse_deg"), "final_beta_error_deg": stat("final_beta_error_deg"),
                     "max_beta_error_deg": stat("max_beta_error_deg"), "saturation_fraction": stat("saturation_fraction"),
                     "final_speed_error_mps": stat("final_speed_error_mps"),
                     "deviation": {k: v for k, v in c.items() if k != "name"},
                     "errors": sorted({r.get("error", "") for r in rs if r.get("error")})})
    return rows


def _report(cfg, summary, num) -> str:
    ref, sc = cfg["reference"], cfg["scenario"]
    ctl = cfg.get("controller") or {}
    law = (f"drift LQR with sideslip integral, q_integral {ctl.get('q_integral', 10.0)}" if ctl.get("integral")
           else "drift LQR without integral action")
    L = [f"# Drift robustness: {cfg['name']}", "",
         f"Nominal design: {ref['speed_mps']} m/s, beta {ref['beta_deg']} deg, mu {ref['mu']} ({law}, never re-tuned). "
         f"Scenario: equilibrium + {sc.get('perturbation_beta_deg', 3.0)} deg sideslip, {sc.get('duration_s', 10.0)} s; "
         f"held: final |beta - beta*| < {sc.get('hold_tol_deg', 5.0)} deg; lost: >= 30 deg at any time. Synthetic "
         "parameters, virtual model only.", "",
         "| case | deviation | feedback | runs | held / degraded / lost / failed | beta RMSE [deg] mean (min-max) | "
         "final beta error [deg] mean (min-max) | final speed error [m/s] | saturated |", "|---" * 9 + "|"]
    for r in summary:
        c = r["classes"]
        dev = ", ".join(f"{k}: {v}" for k, v in r["deviation"].items() if k not in ("feedback", "observer")) or "none"
        fb = r["feedback"] + (f" ({r['deviation'].get('observer', 'drift')} observer)" if r["feedback"] != "ideal" else "")
        f3 = lambda t: "–" if math.isnan(t[0]) else (f"{t[0]:.2f}" if t[1] == t[2] else f"{t[0]:.2f} ({t[1]:.2f}–{t[2]:.2f})")
        L.append(f"| {r['case']} | {dev} | {fb} | {r['runs']} | {c.get('held', 0)} / {c.get('degraded', 0)} / "
                 f"{c.get('lost', 0)} / {c.get('failed', 0)} | {f3(r['beta_rmse_deg'])} | {f3(r['final_beta_error_deg'])} | "
                 f"{f3(r['final_speed_error_mps'])} | {r['saturation_fraction'][0]:.0%} |" + (f" errors: {r['errors']}" if r["errors"] else ""))
    L += ["", "## Numerics", "", f"RK4 closed-loop step study (nominal plant, {num['steps']} s): errors of the final "
          f"(vx, vy, r) against the finest step {[f'{e:.2e}' for e in num['errors_vs_finest']]}, observed orders "
          f"{[round(o, 2) for o in num['observed_orders']]}. {num['note']}.", ""]
    return "\n".join(L)


def main(argv=None) -> int:
    import argparse
    ap = argparse.ArgumentParser(description="Drift robustness study")
    ap.add_argument("config", nargs="?", default="configs/drift/robustness.yaml")
    ap.add_argument("--out", default=None)
    a = ap.parse_args(argv)
    res = run(load(a.config), a.out)
    print(f"{len(res['runs'])} runs -> {res['directory']}")
    return 0


__all__ = ("load", "main", "numerics", "run", "run_case")
