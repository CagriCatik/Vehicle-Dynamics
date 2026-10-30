"""Equilibrium sweep from ``configs/drift/equilibrium_sweep.yaml`` with exports.

Outputs (``outputs.directory``): ``run_config_resolved.yaml``, ``metadata.json``,
``equilibria.csv`` (one row per operating point: residuals, steer, four slips and slip
angles, loads, utilisation, torques, radius, direction, validity class and reasons),
``equilibria.json``, ``equilibrium_sweep.png``, ``report.md``.
"""

from __future__ import annotations

import csv
import json
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml

from .config import keys, load_yaml, numbers, require, vehicle_model

_TOP = {"name", "description", "vehicle", "tyres", "powertrain", "sweep", "directions", "outputs"}


@dataclass(frozen=True)
class SweepSpec:
    name: str
    vehicle: str
    tyres: dict
    speeds: tuple
    betas_deg: tuple
    mus: tuple
    directions: tuple
    output_dir: str
    description: str = ""
    source: str = ""

    def references(self):
        """Operating points in continuation order: friction, direction, speed, increasing |beta|."""
        from vehicle_model.drift import DriftReference
        for mu in self.mus:
            for d in self.directions:
                sign = -1.0 if d == "left" else 1.0
                for v in self.speeds:
                    for b in sorted(self.betas_deg):
                        yield DriftReference(v, sign * math.radians(b), mu)

    def resolved(self) -> dict:
        return {"name": self.name, "description": self.description, "vehicle": self.vehicle, "tyres": dict(self.tyres),
                "powertrain": {"layout": "rwd"}, "sweep": {"speed_mps": list(self.speeds), "beta_deg": list(self.betas_deg),
                                                           "mu": list(self.mus)},
                "directions": list(self.directions), "outputs": {"directory": self.output_dir}}


def parse(data: dict, source: str = "<dict>") -> SweepSpec:
    keys(data, _TOP, source, {"name", "vehicle", "sweep"})
    tyres = dict(data.get("tyres") or {})
    vehicle_model(data["vehicle"], tyres, source)           # files and tyre names must resolve
    pt = dict(data.get("powertrain") or {"layout": "rwd"})
    keys(pt, {"layout"}, f"{source}: powertrain")
    require(pt.get("layout", "rwd") == "rwd", f"{source}: powertrain.layout: only 'rwd' is implemented")
    sw = data["sweep"]
    keys(sw, {"speed_mps", "beta_deg", "mu"}, f"{source}: sweep", {"speed_mps", "beta_deg", "mu"})
    speeds = numbers(sw["speed_mps"], f"{source}: sweep.speed_mps", 2.0, 60.0)
    betas = numbers(sw["beta_deg"], f"{source}: sweep.beta_deg", 0.5, 60.0)
    mus = numbers(sw["mu"], f"{source}: sweep.mu", 0.1, 1.5)
    dirs = data.get("directions", ["left", "right"])
    require(isinstance(dirs, list) and dirs and set(dirs) <= {"left", "right"} and len(set(dirs)) == len(dirs),
            f"{source}: directions must be a list of 'left' / 'right'")
    out = dict(data.get("outputs") or {})
    keys(out, {"directory"}, f"{source}: outputs")
    return SweepSpec(str(data["name"]), str(data["vehicle"]), tyres, tuple(speeds), tuple(betas), tuple(mus), tuple(dirs),
                     str(out.get("directory", f"output/drift/{data['name']}")), str(data.get("description", "")), source)


def load(path: str | Path) -> SweepSpec:
    return parse(load_yaml(path), str(path))


_CSV = ("direction", "mu", "speed_mps", "beta_ref_deg", "validity", "reasons", "residual_norm", "yaw_rate_degps", "radius_m",
        "steer_front_deg", "rear_torque_nm", "kappa_FL", "kappa_FR", "kappa_RL", "kappa_RR", "alpha_FL_deg", "alpha_FR_deg",
        "alpha_RL_deg", "alpha_RR_deg", "Fz_FL", "Fz_FR", "Fz_RL", "Fz_RR", "util_FL", "util_FR", "util_RL", "util_RR",
        "ax", "ay", "drive_power_kw", "residual_translational", "residual_yaw", "residual_wheel", "extrapolating")


def _row(eq) -> list:
    d, ref = eq.diagnostics, eq.reference
    return [ref.direction, ref.road_mu, ref.speed_mps, math.degrees(ref.beta_rad), eq.validity, ";".join(eq.reasons),
            eq.residual_norm, math.degrees(eq.yaw_rate), eq.radius, math.degrees(eq.steer_front), eq.wheel_drive_torque[2],
            *d["kappa"], *d["alpha_deg"], *d["Fz"], *d["utilisation"], d["ax"], d["ay"], d["drive_power_kw"],
            d["residual"]["translational"], d["residual"]["yaw"], d["residual"]["wheel"],
            ";".join(n for n, x in zip(("FL", "FR", "RL", "RR"), d["extrapolating"]) if x)]


def run(spec: SweepSpec, out_dir: str | Path | None = None) -> dict:
    from vehicle_model.drift import DriftEquilibriumSolver
    from vehicle_model.experiments.types import provenance
    from vehicle_model.visualization.drift_plots import plot_equilibrium_sweep

    out = Path(out_dir or spec.output_dir)
    out.mkdir(parents=True, exist_ok=True)
    solver = DriftEquilibriumSolver(vehicle_model(spec.vehicle, spec.tyres, spec.source))
    eqs = solver.sweep(list(spec.references()))
    counts = {}
    for e in eqs:
        counts[e.validity] = counts.get(e.validity, 0) + 1
    (out / "run_config_resolved.yaml").write_text(yaml.safe_dump(spec.resolved(), sort_keys=False), encoding="utf-8")
    meta = {"source": spec.source, "provenance": provenance(), "points": len(eqs), "counts": counts,
            "plant": "SevenDOFModel (unchanged), DeterministicPlant evaluation", "powertrain": "rwd, equal rear torques"}
    (out / "metadata.json").write_text(json.dumps(meta, indent=2), encoding="utf-8")
    with open(out / "equilibria.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(_CSV)
        w.writerows(_row(e) for e in eqs)
    (out / "equilibria.json").write_text(json.dumps([e.to_dict() for e in eqs], indent=2, default=float), encoding="utf-8")
    plot_equilibrium_sweep(eqs, out / "equilibrium_sweep.png", spec.name)
    (out / "report.md").write_text(_report(spec, eqs, meta), encoding="utf-8")
    return {"equilibria": eqs, "metadata": meta, "directory": str(out)}


def _report(spec: SweepSpec, eqs, meta) -> str:
    pv = meta["provenance"]
    L = [f"# Drift equilibrium sweep: {spec.name}", "", spec.description or "", "",
         f"Vehicle `{spec.vehicle}`, tyres {spec.tyres}, rear-wheel drive with equal rear torques, front steer only. "
         f"Code: git {pv['git_commit']}. Synthetic parameters; virtual model only.", "",
         "| validity | points |", "|---|---|"]
    for k, n in sorted(meta["counts"].items()):
        L.append(f"| {k} | {n} |")
    L += ["", "| dir | mu | v [m/s] | beta [deg] | validity | R [m] | r [deg/s] | delta_f [deg] | T_rear [N m] | "
          "kappa RL / RR | alpha RL / RR [deg] | reasons |", "|---" * 12 + "|"]
    for e in eqs:
        d = e.diagnostics
        if e.physical_feasible:
            L.append(f"| {e.reference.direction} | {e.reference.road_mu:g} | {e.reference.speed_mps:g} | "
                     f"{math.degrees(e.reference.beta_rad):g} | {e.validity} | {e.radius:.1f} | {math.degrees(e.yaw_rate):.1f} | "
                     f"{math.degrees(e.steer_front):.1f} | {e.wheel_drive_torque[2]:.0f} | "
                     f"{d['kappa'][2]:.2f} / {d['kappa'][3]:.2f} | {d['alpha_deg'][2]:.1f} / {d['alpha_deg'][3]:.1f} | – |")
        else:
            L.append(f"| {e.reference.direction} | {e.reference.road_mu:g} | {e.reference.speed_mps:g} | "
                     f"{math.degrees(e.reference.beta_rad):g} | infeasible | | | | | | | {', '.join(e.reasons)} |")
    L += ["", "Validity classes and reasons: `drift/equilibrium.py` (module docstring). `valid_extrapolated`: a tyre works",
          "outside its declared validity range (MF: |alpha| <= 20 deg); the result rests on the extrapolated tyre model.", ""]
    return "\n".join(L)


def main(argv=None) -> int:
    import argparse
    ap = argparse.ArgumentParser(description="Drift equilibrium sweep")
    ap.add_argument("config", nargs="?", default="configs/drift/equilibrium_sweep.yaml")
    ap.add_argument("--out", default=None)
    a = ap.parse_args(argv)
    res = run(load(a.config), a.out)
    print(f"{len(res['equilibria'])} operating points, {res['metadata']['counts']} -> {res['directory']}")
    return 0


__all__ = ("SweepSpec", "load", "main", "parse", "run")
