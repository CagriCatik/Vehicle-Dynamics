"""Drift plan D3: hold and perturbation recovery around a drift equilibrium (scenarios S01-S03).

For each direction: solve the equilibrium, linearise (step study, stabilisability), design the
LQR, then simulate the existing 7-DOF plant through the existing ScenarioRunner from the
equilibrium state (``hold``) and from defined perturbations, once with constant ``u*`` (open
loop) and once with the LQR. Nothing is teleported: the runner integrates the plant from the
initial state; the controller only sets steer and rear torque within the vehicle limits.

Outputs: ``run_config_resolved.yaml``, ``metadata.json``, ``linearisation.json``, ``kpi.json``,
``state_history_<direction>_<perturbation>_<variant>.csv``, ``sideslip_and_yaw.png``, ``report.md``.
"""

from __future__ import annotations

import csv
import json
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml

from .config import keys, load_yaml, require, vehicle_model

_TOP = {"name", "description", "vehicle", "tyres", "powertrain", "reference", "directions", "controller", "simulation",
        "perturbations", "outputs"}
_PERT = {"name", "beta_deg", "r_radps", "speed_mps", "omega_inner_radps"}


@dataclass(frozen=True)
class HoldSpec:
    name: str
    vehicle: str
    tyres: dict
    speed: float
    beta_deg: float
    mu: float
    directions: tuple
    controller: dict
    dt: float
    duration: float
    settle: float
    perturbations: tuple
    output_dir: str
    source: str = ""
    raw: dict | None = None


def parse(data: dict, source: str = "<dict>") -> HoldSpec:
    from vehicle_model.drift.lqr import LQRWeights
    keys(data, _TOP, source, {"name", "vehicle", "reference", "controller"})
    tyres = dict(data.get("tyres") or {})
    vehicle_model(data["vehicle"], tyres, source)
    pt = dict(data.get("powertrain") or {"layout": "rwd"})
    keys(pt, {"layout"}, f"{source}: powertrain")
    require(pt.get("layout", "rwd") == "rwd", f"{source}: powertrain.layout: only 'rwd' is implemented (V1)")
    ref = dict(data["reference"])
    keys(ref, {"speed_mps", "beta_deg", "mu"}, f"{source}: reference", {"speed_mps", "beta_deg", "mu"})
    require(2.0 <= float(ref["speed_mps"]) <= 60.0, f"{source}: reference.speed_mps outside [2, 60]")
    require(0.5 <= float(ref["beta_deg"]) <= 60.0, f"{source}: reference.beta_deg (magnitude) outside [0.5, 60]")
    require(0.1 <= float(ref["mu"]) <= 1.5, f"{source}: reference.mu outside [0.1, 1.5]")
    dirs = data.get("directions", ["left", "right"])
    require(isinstance(dirs, list) and dirs and set(dirs) <= {"left", "right"} and len(set(dirs)) == len(dirs),
            f"{source}: directions must be a list of 'left' / 'right'")
    ctrl = dict(data["controller"])
    keys(ctrl, {"type", "information_mode", "steer_rate_design", "rate_limit", "q_scaled", "r_scaled", "q_steer"},
         f"{source}: controller", {"type"})
    require(ctrl["type"] == "equilibrium_lqr", f"{source}: controller.type must be 'equilibrium_lqr'")
    require(ctrl.get("information_mode", "ideal_full_state") == "ideal_full_state",
            f"{source}: controller.information_mode: only 'ideal_full_state' is implemented in D3")
    LQRWeights(tuple(ctrl.get("q_scaled", LQRWeights.q_scaled)), tuple(ctrl.get("r_scaled", LQRWeights.r_scaled)),
               float(ctrl.get("q_steer", 1.0)))    # dimension and sign checks fail hard here
    sim = dict(data.get("simulation") or {})
    keys(sim, {"dt_s", "duration_s", "settle_s"}, f"{source}: simulation")
    dt, dur, settle = float(sim.get("dt_s", 0.005)), float(sim.get("duration_s", 6.0)), float(sim.get("settle_s", 2.0))
    require(0.001 <= dt <= 0.02 and 0.5 <= dur <= 60.0 and 0.0 <= settle < dur, f"{source}: simulation out of range")
    perts = data.get("perturbations", [{"name": "hold"}])
    require(isinstance(perts, list) and perts, f"{source}: perturbations must be a non-empty list")
    for i, pt_ in enumerate(perts):
        keys(pt_, _PERT, f"{source}: perturbations[{i}]", {"name"})
    require(len({p["name"] for p in perts}) == len(perts), f"{source}: perturbation names must be unique")
    out = dict(data.get("outputs") or {})
    keys(out, {"directory"}, f"{source}: outputs")
    return HoldSpec(str(data["name"]), str(data["vehicle"]), tyres, float(ref["speed_mps"]), float(ref["beta_deg"]),
                    float(ref["mu"]), tuple(dirs), ctrl, dt, dur, settle, tuple(dict(p) for p in perts),
                    str(out.get("directory", f"output/drift/{data['name']}")), source, data)


def load(path: str | Path) -> HoldSpec:
    return parse(load_yaml(path), str(path))


def initial_state(eq, pert: dict):
    """Equilibrium state plus a perturbation, mirrored for right drifts (inner wheel: RL left, RR right)."""
    from vehicle_model.core.types import SevenDOFState
    s = 1.0 if eq.reference.direction == "left" else -1.0
    v = eq.speed + float(pert.get("speed_mps", 0.0))
    beta = eq.beta + s * math.radians(float(pert.get("beta_deg", 0.0)))
    omega = eq.omega.copy()
    omega[2 if s > 0 else 3] += float(pert.get("omega_inner_radps", 0.0))
    return SevenDOFState(vx=v * math.cos(beta), vy=v * math.sin(beta), r=eq.yaw_rate + s * float(pert.get("r_radps", 0.0)),
                         omega=omega)


def fresh_plant(model):
    """Independent copy of the plant with a defined load-transfer start value.

    The plant warm-starts its load-transfer iteration from the previous evaluation
    (``_accel_guess``). Reusing one plant object across runs made results depend on the run
    order (found in D5: identical initial states of an unstable drift fell off the equilibrium
    at different times). Every run therefore starts from a copy with the same start value.
    """
    import copy
    m = copy.copy(model)
    m._accel_guess = (0.0, 0.0)
    return m


def simulate(model, controller, x0, duration: float, dt: float):
    """Closed loop through the existing ScenarioRunner on a fresh plant copy (telemetry passed to the controller)."""
    from vehicle_model.core.types import SevenDOFControlInput
    from vehicle_model.simulation.scenario_runner import ScenarioRunner
    model = fresh_plant(model)
    sat = []

    def cb(t, s, tel):
        out = controller.compute_control(s, None, tel, dt)
        sat.append(out.status == "saturated")
        return SevenDOFControlInput(steer_f=out.steer_f, steer_r=out.steer_r, T_drive=out.T_drive, T_brake=out.T_brake)
    return ScenarioRunner(model, model.road).run(cb, x0, duration, dt), np.array(sat)


class _OpenLoop:
    """Constant equilibrium input u* (the plan's open-loop comparison)."""

    def __init__(self, eq):
        from vehicle_model.drift import rwd_input
        self.u = rwd_input(eq.steer_front, eq.wheel_drive_torque[2])

    def compute_control(self, state, target, telemetry, dt):
        from vehicle_model.controllers.framework import ControlOutput
        return ControlOutput(steer_f=self.u.steer_f, T_drive=self.u.T_drive.copy(), T_brake=self.u.T_brake.copy())


def run(spec: HoldSpec, out_dir: str | Path | None = None) -> dict:
    import copy

    from vehicle_model.drift import DriftEquilibriumSolver, DriftReference
    from vehicle_model.drift.control_adapter import DriftLQRController
    from vehicle_model.drift.linearization import linearise
    from vehicle_model.drift.lqr import LQRWeights, design_lqr
    from vehicle_model.drift.metrics import hold_kpis
    from vehicle_model.experiments.types import provenance
    from vehicle_model.roads.surface import UniformRoad
    from vehicle_model.visualization.drift_plots import plot_hold_runs

    out = Path(out_dir or spec.output_dir)
    out.mkdir(parents=True, exist_ok=True)
    base = vehicle_model(spec.vehicle, spec.tyres, spec.source)
    plant_model = copy.copy(base)
    plant_model.road = UniformRoad(mu=spec.mu)
    solver = DriftEquilibriumSolver(base)
    c = spec.controller
    w = LQRWeights(tuple(c.get("q_scaled", LQRWeights.q_scaled)), tuple(c.get("r_scaled", LQRWeights.r_scaled)),
                   float(c.get("q_steer", 1.0)))
    results, lin_out, histories = [], {}, {}
    for d in spec.directions:
        ref = DriftReference(spec.speed, (-1.0 if d == "left" else 1.0) * math.radians(spec.beta_deg), spec.mu)
        eq = solver.solve(ref)
        if not eq.physical_feasible:
            results.append({"direction": d, "status": "INFEASIBLE", "reasons": list(eq.reasons)})
            continue
        lin = linearise(solver.plant(spec.mu), eq)
        lqr = design_lqr(lin, base.params, w, steer_rate=bool(c.get("steer_rate_design", True)))
        lin_out[d] = {"equilibrium": eq.to_dict(), "linearisation": lin.summary(),
                      "closed_loop_eigenvalues": [[float(l.real), float(l.imag)] for l in lqr.closed_loop_eigenvalues],
                      "K": lqr.K.tolist()}
        for pert in spec.perturbations:
            x0 = initial_state(eq, pert)
            for variant in ("open_loop", "lqr"):
                ctrl = _OpenLoop(eq) if variant == "open_loop" else DriftLQRController(
                    base.params, eq, lqr, rate_limit=bool(c.get("rate_limit", True)), equilibrium_id=f"{d}")
                h, sat = simulate(plant_model, ctrl, x0, spec.duration, spec.dt)
                k = hold_kpis(h.time, h.vx, h.vy, h.r, eq.beta, eq.yaw_rate, eq.speed, spec.settle, sat)
                results.append({"direction": d, "perturbation": pert["name"], "variant": variant, "kpis": k})
                histories[(d, pert["name"], variant)] = h
                _write_history(out / f"state_history_{d}_{pert['name']}_{variant}.csv", h)
    (out / "run_config_resolved.yaml").write_text(yaml.safe_dump(spec.raw, sort_keys=False), encoding="utf-8")
    meta = {"source": spec.source, "provenance": provenance(), "plant": "SevenDOFModel via ScenarioRunner",
            "information_mode": "ideal_full_state"}
    (out / "metadata.json").write_text(json.dumps(meta, indent=2), encoding="utf-8")
    (out / "linearisation.json").write_text(json.dumps(lin_out, indent=2, default=float), encoding="utf-8")
    (out / "kpi.json").write_text(json.dumps(results, indent=2, default=float), encoding="utf-8")
    plot_hold_runs(histories, {d: lin_out[d]["equilibrium"] for d in lin_out}, out / "sideslip_and_yaw.png", spec.name)
    (out / "report.md").write_text(_report(spec, results, lin_out, meta), encoding="utf-8")
    return {"results": results, "linearisation": lin_out, "directory": str(out), "histories": histories}


def _write_history(path: Path, h) -> None:
    beta = np.degrees(np.arctan2(h.vy, h.vx))
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["t", "X", "Y", "psi", "vx", "vy", "r", "beta_deg", "omega_FL", "omega_FR", "omega_RL", "omega_RR",
                    "steer_f", "T_RL", "T_RR", "kappa_RL", "kappa_RR", "alpha_RL_deg", "alpha_RR_deg"])
        for k in range(h.time.size):
            w.writerow([h.time[k], h.X[k], h.Y[k], h.psi[k], h.vx[k], h.vy[k], h.r[k], beta[k], *h.omega[k], h.steer_f[k],
                        h.T_drive[k, 2], h.T_drive[k, 3], h.kappa[k, 2], h.kappa[k, 3], np.degrees(h.alpha[k, 2]),
                        np.degrees(h.alpha[k, 3])])


def _report(spec, results, lin_out, meta) -> str:
    pv = meta["provenance"]
    L = [f"# Drift hold and perturbation recovery: {spec.name}", "",
         f"Reference: {spec.speed:g} m/s, |beta| {spec.beta_deg:g} deg, mu {spec.mu:g}; RWD, front steer; information "
         f"mode ideal_full_state. Plant: 7-DOF via ScenarioRunner, dt {spec.dt} s, {spec.duration:g} s; RMS over "
         f"t >= {spec.settle:g} s. Code: git {pv['git_commit']}. Synthetic parameters, virtual model only.", ""]
    for d, lo in lin_out.items():
        e, l = lo["equilibrium"], lo["linearisation"]
        L += [f"## {d}", "", f"Equilibrium: R {e['radius_m']:.1f} m, r {e['yaw_rate_degps']:.1f} deg/s, delta_f "
              f"{e['steer_front_deg']:.1f} deg, T {e['wheel_drive_torque'][2]:.0f} N m per rear wheel, validity {e['validity']}.",
              f"Open loop: unstable eigenvalues {[round(u[0], 3) for u in l['unstable']]} 1/s, PBH ranks {l['pbh_ranks']} "
              f"(stabilisable: {l['stabilisable']}), Jacobian step {l['step']:g} (converged {l['converged']}). "
              f"Closed loop (linear): slowest eigenvalue {lo['closed_loop_eigenvalues'][0][0]:.2f} 1/s.", "",
              "| perturbation | variant | beta RMSE [deg] | yaw RMSE [deg/s] | speed RMSE [m/s] | max beta error [deg] | "
              "error norm start -> end | saturated | drift lost |", "|---|---|---|---|---|---|---|---|---|"]
        for r in results:
            if r.get("direction") == d and "kpis" in r:
                k = r["kpis"]
                L.append(f"| {r['perturbation']} | {r['variant']} | {k['beta_rmse_deg']:.2f} | {k['yaw_rmse_degps']:.2f} | "
                         f"{k['speed_rmse_mps']:.2f} | {k['max_beta_error_deg']:.1f} | {k['error_norm_initial']:.2f} -> "
                         f"{k['error_norm_final']:.2f} | {k['saturation_fraction']:.0%} | "
                         f"{'yes' if k['stability_loss'] else 'no'} |")
        L.append("")
    for r in results:
        if r.get("status") == "INFEASIBLE":
            L += [f"## {r['direction']}: INFEASIBLE ({', '.join(r['reasons'])})", ""]
    L += ["Error norm: sqrt((d_beta/0.05 rad)^2 + (d_r/0.1 rad/s)^2 + (d_v/1 m/s)^2). Drift lost: |beta - beta_ref| >= 30 deg.",
          "Local result only: the LQR is designed on the linearisation; no global stability is claimed.", ""]
    return "\n".join(L)


def main(argv=None) -> int:
    import argparse
    ap = argparse.ArgumentParser(description="Drift LQR hold and perturbation recovery (Drift plan D3)")
    ap.add_argument("config", nargs="?", default="configs/drift/drift_lqr.yaml")
    ap.add_argument("--out", default=None)
    a = ap.parse_args(argv)
    res = run(load(a.config), a.out)
    lost = [(r["direction"], r["perturbation"]) for r in res["results"] if r.get("variant") == "lqr" and r["kpis"]["stability_loss"]]
    print(f"{len(res['results'])} runs -> {res['directory']}; LQR runs with lost drift: {lost or 'none'}")
    return 0


__all__ = ("HoldSpec", "initial_state", "load", "main", "parse", "run", "simulate")
