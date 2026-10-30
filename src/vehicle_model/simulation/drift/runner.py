"""Drift plan D6: recorded drift runs and their complete export.

:func:`record` runs a controller on a fresh copy of the 7-DOF plant through the existing
ScenarioRunner (via :func:`.hold.simulate`) and keeps, per step, the controller diagnostics
(mode, status, saturation, reference values). :func:`export_run` writes the plan's outputs
for one run into ``<out>/<scenario>/<variant>/``:

    run_config_resolved.yaml  metadata.json  state_history.csv  wheel_forces.csv
    controller_diagnostics.csv  kpi.json  drift_animation.gif  trajectory.png
    sideslip_and_yaw.png  wheel_torques_and_slip.png  phase_plane.png
    equilibrium_residuals.png  report.md

CSV, plots and GIF are all made from the same recorded history object, so path and time
axes agree by construction (checked in the tests).
"""

from __future__ import annotations

import csv
import json
import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import yaml


@dataclass
class DriftRunResult:
    scenario: str
    variant: str
    history: object                       # SimulationHistory
    diagnostics: list                     # per step: mode, status, saturated, beta_ref, ...
    params: object                        # SevenDOFParams of the plant
    circle: object | None = None          # CircleReference shown in plots and GIF
    equilibrium: dict | None = None       # DriftEquilibrium.to_dict() of the target
    reference_beta: float | None = None   # [rad]
    reference_r: float | None = None      # [rad/s]
    kpis: dict = field(default_factory=dict)
    label: str = ""                       # "PASS", "drift lost", "FAILED", ...
    config: dict = field(default_factory=dict)


def record(plant, controller, x0, duration: float, dt: float):
    """Simulate and return ``(history, diagnostics)``; diagnostics are taken from each ControlOutput."""
    from .hold import simulate
    diags = []

    class _Log:
        def compute_control(self, state, target, telemetry, dt_):
            out = controller.compute_control(state, target, telemetry, dt_)
            d = out.diagnostics or {}
            drift = d.get("drift") or {}
            diags.append({"mode": d.get("mode", drift.get("mode", "")), "status": out.status,
                          "saturated": out.status == "saturated", "beta_ref": d.get("beta_ref", drift.get("beta_ref")),
                          "radial_error": d.get("radial_error"), "exit_phase": d.get("exit_phase")})
            return out
    h, _ = simulate(plant, _Log(), x0, duration, dt)
    return h, diags


def _f(x):
    return "" if x is None else x


def export_run(res: DriftRunResult, out_root: str | Path, gif: bool = True, plots: bool = True, fps: int = 20,
               playback_speed: float = 1.0, dpi: int = 90) -> Path:
    from vehicle_model.experiments.types import provenance
    out = Path(out_root) / res.scenario / res.variant
    out.mkdir(parents=True, exist_ok=True)
    h = res.history
    beta = np.arctan2(h.vy, h.vx)
    chi = h.psi + beta
    with open(out / "state_history.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["t", "X", "Y", "psi", "chi", "vx", "vy", "speed", "r", "beta_deg", "omega_FL", "omega_FR", "omega_RL",
                    "omega_RR", "steer_f", "steer_r", "T_drive_FL", "T_drive_FR", "T_drive_RL", "T_drive_RR", "T_brake_FL",
                    "T_brake_FR", "T_brake_RL", "T_brake_RR"])
        for k in range(h.time.size):
            w.writerow([h.time[k], h.X[k], h.Y[k], h.psi[k], chi[k], h.vx[k], h.vy[k], math.hypot(h.vx[k], h.vy[k]), h.r[k],
                        math.degrees(beta[k]), *h.omega[k], h.steer_f[k], h.steer_r[k], *h.T_drive[k], *h.T_brake[k]])
    with open(out / "wheel_forces.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        wn = ("FL", "FR", "RL", "RR")
        w.writerow(["t", *(f"{q}_{n}" for q in ("kappa", "alpha_deg", "Fz", "mu", "Fx", "Fy", "utilisation") for n in wn)])
        util = np.hypot(h.Fx_wheel, h.Fy_wheel) / np.maximum(h.mu * h.Fz, 1e-9)
        for k in range(h.time.size):
            w.writerow([h.time[k], *h.kappa[k], *np.degrees(h.alpha[k]), *h.Fz[k], *h.mu[k], *h.Fx_wheel[k], *h.Fy_wheel[k],
                        *util[k]])
    with open(out / "controller_diagnostics.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["t", "mode", "status", "saturated", "beta_ref_deg", "radial_error_m", "exit_phase"])
        for k, d in enumerate(res.diagnostics):
            br = d.get("beta_ref")
            w.writerow([h.time[k], d.get("mode", ""), d.get("status", ""), d.get("saturated", ""),
                        "" if br is None else math.degrees(br), _f(d.get("radial_error")), _f(d.get("exit_phase"))])
    (out / "kpi.json").write_text(json.dumps(res.kpis, indent=2, default=float), encoding="utf-8")
    (out / "run_config_resolved.yaml").write_text(yaml.safe_dump(res.config, sort_keys=False), encoding="utf-8")
    (out / "metadata.json").write_text(json.dumps({"scenario": res.scenario, "variant": res.variant, "label": res.label,
                                                   "samples": int(h.time.size), "dt": float(h.time[1] - h.time[0]),
                                                   "equilibrium": res.equilibrium, "provenance": provenance(),
                                                   "information_mode": "ideal_full_state",
                                                   "source": "recorded SimulationHistory of the 7-DOF plant"},
                                                  indent=2, default=float), encoding="utf-8")
    if plots:
        from vehicle_model.visualization.drift_plots import plot_run_set
        plot_run_set(res, out)
    if gif:
        from vehicle_model.visualization.drift_animation import animate_drift
        animate_drift([res], out / "drift_animation.gif", fps=fps, playback_speed=playback_speed, dpi=dpi,
                      title=f"{res.scenario} / {res.variant}" + (f" [{res.label}]" if res.label else ""))
    (out / "report.md").write_text(_report(res), encoding="utf-8")
    return out


def _report(res: DriftRunResult) -> str:
    L = [f"# Drift run {res.scenario} / {res.variant}", "", f"Result: {res.label or '-'}", "",
         "All files in this directory come from one recorded simulation of the 7-DOF plant (information mode",
         "ideal_full_state); the GIF draws recorded samples only. Synthetic parameters, virtual model only.", "",
         "| KPI | value |", "|---|---|"]
    for k, v in res.kpis.items():
        if isinstance(v, (int, float, bool, str)):
            L.append(f"| {k} | {v:.4g} |" if isinstance(v, float) else f"| {k} | {v} |")
    if res.equilibrium:
        e = res.equilibrium
        L += ["", f"Target equilibrium: beta {e['beta_deg']:.1f} deg, r {e['yaw_rate_degps']:.1f} deg/s, R {e['radius_m']:.1f} m, "
              f"delta_f {e['steer_front_deg']:.1f} deg, validity {e['validity']}."]
    L += ["", "Files: state_history.csv, wheel_forces.csv, controller_diagnostics.csv, kpi.json, metadata.json,",
          "run_config_resolved.yaml, drift_animation.gif, trajectory.png, sideslip_and_yaw.png,",
          "wheel_torques_and_slip.png, phase_plane.png, equilibrium_residuals.png.", ""]
    return "\n".join(L)


__all__ = ("DriftRunResult", "export_run", "record")
