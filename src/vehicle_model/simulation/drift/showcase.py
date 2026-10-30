"""Drift plan D6: the animated and fully exported runs of ``configs/drift/animation.yaml``.

Scenarios (each from recorded 7-DOF data via :mod:`.runner`):

    circle_hold     equilibrium + sideslip perturbation, reference circle shifted radially:
                    ``open_loop`` (constant u*) vs ``drift_tracking`` (scheduled LQR + radial loop);
                    exported per variant and as one comparison GIF on shared axes
    full_sequence   supervised drift from straight driving: approach, initiation, hold, exit
    figure_eight    supervised left drift, exit, right drift, exit (sequence; geometry see D5)
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np

from .config import keys, load_yaml, numbers, require, vehicle_model

_TOP = {"name", "description", "vehicle", "tyres", "powertrain", "reference", "scenarios", "visualization", "simulation",
        "outputs"}


def load(path) -> dict:
    data = load_yaml(path)
    keys(data, _TOP, str(path), {"name", "vehicle", "reference", "scenarios"})
    vehicle_model(data["vehicle"], dict(data.get("tyres") or {}), str(path))
    keys(data["reference"], {"speed_mps", "beta_deg", "mu", "branch_beta_deg"}, f"{path}: reference",
         {"speed_mps", "beta_deg", "mu", "branch_beta_deg"})
    numbers(data["reference"]["branch_beta_deg"], f"{path}: reference.branch_beta_deg", 1.0, 45.0)
    sc = data["scenarios"]
    keys(sc, {"circle_hold", "full_sequence", "figure_eight"}, f"{path}: scenarios")
    vis = dict(data.get("visualization") or {})
    keys(vis, {"enable_gif", "enable_plots", "fps", "dpi", "playback_speed"}, f"{path}: visualization")
    require(1 <= int(vis.get("fps", 20)) <= 60, f"{path}: visualization.fps must lie in [1, 60]")
    data["_source"] = str(path)
    return data


def run(cfg: dict, out_dir=None) -> dict:
    from vehicle_model.core.types import SevenDOFState
    from vehicle_model.drift import DriftEquilibriumSolver, DriftReference
    from vehicle_model.drift.control_adapter import ScheduledDriftController
    from vehicle_model.drift.metrics import hold_kpis
    from vehicle_model.drift.path_reference import CircleReference
    from vehicle_model.drift.schedule import BranchSchedule
    from vehicle_model.roads.surface import UniformRoad
    from vehicle_model.visualization.drift_animation import animate_drift

    from .hold import _OpenLoop
    from .runner import DriftRunResult, export_run, record
    from .tracking import _supervised

    out = Path(out_dir or cfg.get("outputs", {}).get("directory", "output/drift/runs"))
    ref = cfg["reference"]
    v, b, mu = float(ref["speed_mps"]), float(ref["beta_deg"]), float(ref["mu"])
    dt = float(cfg.get("simulation", {}).get("dt_s", 0.005))
    vis = cfg.get("visualization", {})
    gif, plots = bool(vis.get("enable_gif", True)), bool(vis.get("enable_plots", True))
    fps, dpi = int(vis.get("fps", 20)), int(vis.get("dpi", 80))
    pb = dict(vis.get("playback_speed") or {})
    m = vehicle_model(cfg["vehicle"], dict(cfg.get("tyres") or {}), cfg["_source"])
    plant = m.__class__(params=m.params, tyre=m.tyre_front, tyre_rear=m.tyre_rear, road=UniformRoad(mu=mu))
    eq = DriftEquilibriumSolver(m).solve(DriftReference(v, -math.radians(b), mu))
    eqd = eq.to_dict()
    resolved = {k: v_ for k, v_ in cfg.items() if not k.startswith("_")}
    exported = {}
    sc = cfg["scenarios"]

    if "circle_hold" in sc:
        c = sc["circle_hold"]
        sch = BranchSchedule(m, v, mu, [-x for x in ref["branch_beta_deg"]])
        circ = CircleReference.through_state(0.0, 0.0, eq.beta, eq.radius, "left", offset=float(c.get("circle_offset_m", 2.0)))
        beta0 = eq.beta - math.radians(float(c.get("perturbation_beta_deg", 3.0)))
        x0 = SevenDOFState(vx=v * math.cos(beta0), vy=v * math.sin(beta0), r=eq.yaw_rate, omega=eq.omega.copy())
        runs = []
        for variant, ctrl in (("open_loop", _OpenLoop(eq)),
                              ("drift_tracking", ScheduledDriftController(m.params, sch, eq.beta, circle=circ,
                                                                          k_p=math.radians(1.0), k_d=math.radians(0.5)))):
            h, diags = record(plant, ctrl, x0, float(c.get("duration_s", 14.0)), dt)
            k = hold_kpis(h.time, h.vx, h.vy, h.r, eq.beta, eq.yaw_rate, eq.speed, 4.0)
            err = circ.radial_error(h.X, h.Y)
            k["radial_rmse_last_5s_m"] = float(np.sqrt(np.mean(err[h.time >= h.time[-1] - 5.0] ** 2)))
            label = "drift lost" if k["stability_loss"] else "PASS"
            r = DriftRunResult("circle_hold", variant, h, diags, m.params, circ, eqd, eq.beta, eq.yaw_rate, k, label, resolved)
            exported[("circle_hold", variant)] = export_run(r, out, gif, plots, fps, float(pb.get("circle_hold", 1.0)), dpi)
            runs.append(r)
        if gif:
            animate_drift(runs[::-1], out / "circle_hold" / "comparison.gif", fps=fps,
                          playback_speed=float(pb.get("circle_hold", 1.0)), dpi=dpi,
                          title="circle_hold: drift_tracking vs open_loop (same start, same axes)")

    for name in ("full_sequence", "figure_eight"):
        if name not in sc:
            continue
        c = sc[name]
        x0 = SevenDOFState(vx=v, omega=np.full(4, v / m.params.Reff))
        if name == "full_sequence":
            ctrl, sup = _supervised(m, v, b, mu, "left", float(c.get("exit_s", 20.0)))
            h, diags = record(plant, ctrl, x0, float(c.get("duration_s", 30.0)), dt)
            modes = [d["mode"] for d in diags]
            ok = "HOLD" in modes and modes[-1] == "DISABLED" and "FAILED" not in modes
            k = {"final_mode": modes[-1], "hold_reached": "HOLD" in modes,
                 "transitions": [f"{t.t:.2f} s {t.target} ({t.reason})" for t in sup.transitions]}
        else:
            from .tracking import TrackingSpec, _figure_eight
            spec = TrackingSpec("figure_eight", cfg["vehicle"], tuple(sorted((cfg.get("tyres") or {}).items())), v, b, mu,
                                tuple(ref["branch_beta_deg"]), (0.0,), 1.0, 0.5, 1.0, None, dict(c), dt, str(out), cfg["_source"])
            import tempfile
            with tempfile.TemporaryDirectory() as tmp:     # the sequence's own CSV is replaced by the full export
                k, h, modes = _figure_eight(spec, m, Path(tmp))
            diags = [{"mode": md} for md in modes]
            ok = k["sequence_success"]
            k = {**{kk: vv for kk, vv in k.items() if kk != "sections"}, **{f"{s}_{kk}": vv for s, sec in k["sections"].items()
                                                                            for kk, vv in sec.items()}}
        if name == "figure_eight":
            label = ("sequence PASS" if ok else "sequence FAIL") + (" / geometry PASS" if k.get("geometry_success")
                                                                    else " / geometry FAIL")
        else:
            label = "PASS" if ok else "FAIL"
        r = DriftRunResult(name, "supervised", h, diags, m.params, None, eqd, eq.beta, eq.yaw_rate, k, label, resolved)
        exported[(name, "supervised")] = export_run(r, out, gif, plots, fps, float(pb.get(name, 2.0)), dpi)
    return {"directory": str(out), "exported": {f"{a}/{b_}": str(p) for (a, b_), p in exported.items()}}


def main(argv=None) -> int:
    import argparse
    ap = argparse.ArgumentParser(description="Drift animations and full exports (Drift plan D6)")
    ap.add_argument("config", nargs="?", default="configs/drift/animation.yaml")
    ap.add_argument("--out", default=None)
    a = ap.parse_args(argv)
    res = run(load(a.config), a.out)
    for k, p in res["exported"].items():
        print(f"{k}: {p}")
    return 0


__all__ = ("load", "main", "run")
