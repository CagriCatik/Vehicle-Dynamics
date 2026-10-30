"""Monte Carlo robustness of the yaw stability control.

Sine with dwell (80 km/h, 8 deg, modelled on FMVSS 126) on the reference car with
MF tyres, passive and with the PI yaw controller, under random parameter
variations. Every sample gets a fresh plant; the study is repeatable (seed).

    mass_factor            normal(1, 0.05), yaw inertia scaled alike
    cog_shift_m            normal(0, 0.03), + = rearwards (wheelbase unchanged)
    front/rear_stiffness   normal(1, 0.08) on the MF cornering stiffness PKY1
    mu                     uniform(0.8, 1.0) road friction

Outputs (output/monte_carlo/):
    monte_carlo.json      samples, KPIs, statistics, limit violations
    monte_carlo.png       distributions of the yaw-rate ratio and lateral displacement

Run:
    python examples/monte_carlo_robustness.py [n_samples]     (default 25; several minutes)
"""

from __future__ import annotations

import dataclasses
import json
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from vehicle_model.controllers.motion_control import MotionController
from vehicle_model.controllers.yaw_laws import YawRatePI
from vehicle_model.roads.surface import UniformRoad
from vehicle_model.simulation.controller_benchmark import sine_with_dwell_scenario
from vehicle_model.simulation.robustness import Limit, Variation, run_monte_carlo
from vehicle_model.tire_model import MagicFormulaTyre, load_tyre
from vehicle_model.vehicle_models import SevenDOFModel
from vehicle_model.visualization.theme import TEXT_MUTED, apply_dark_theme, style_axis

VEHICLE = "configs/vehicles/reference_passenger_car.yaml"
MF = "magic_formula_passenger_synthetic"
OUT = Path("output/monte_carlo")
PASSIVE, CONTROLLED = "#d95926", "#3987e5"   # validated slots (dark surface); same entities as in the slip control and allocation demo
SEED = 2026

VARIATIONS = {
    "mass_factor": Variation("normal", 1.0, 0.05, 0.8, 1.2),
    "cog_shift_m": Variation("normal", 0.0, 0.03, -0.1, 0.1),
    "front_stiffness": Variation("normal", 1.0, 0.08, 0.7, 1.3),
    "rear_stiffness": Variation("normal", 1.0, 0.08, 0.7, 1.3),
    "mu": Variation("uniform", 0.8, 1.0),
}
LIMITS = (Limit("yaw_rate_ratio_1s", "<=", 0.35), Limit("yaw_rate_ratio_1_75s", "<=", 0.20),
          Limit("lateral_displacement", ">=", 1.83))


def build_plant(s: dict):
    base = SevenDOFModel.from_yaml(VEHICLE, tyre_name=MF, tyre_rear_name=MF)
    p = base.params
    p = dataclasses.replace(p, mass=p.mass * s["mass_factor"], Iz=p.Iz * s["mass_factor"],
                            lf=p.lf + s["cog_shift_m"], lr=p.lr - s["cog_shift_m"])
    mf = load_tyre(MF)
    tf = MagicFormulaTyre(dataclasses.replace(mf.params, PKY1=mf.params.PKY1 * s["front_stiffness"]), mf.provenance)
    tr = MagicFormulaTyre(dataclasses.replace(mf.params, PKY1=mf.params.PKY1 * s["rear_stiffness"]), mf.provenance)
    road = UniformRoad(mu=s["mu"])
    return lambda _scenario_road: SevenDOFModel(p, tf, road, tr)


def main(n: int = 25) -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    sc = sine_with_dwell_scenario()
    controllers = {"passive": None, "PI (TV)": lambda p: MotionController(p, YawRatePI())}
    results = {}
    for name, factory in controllers.items():
        print(f"{name}: {n} samples ...")
        results[name] = run_monte_carlo(build_plant, factory, sc, VARIATIONS, n, SEED, LIMITS)
    dump = {"seed": SEED, "n": n, "variations": {k: dataclasses.asdict(v) for k, v in VARIATIONS.items()},
            "results": {name: {"statistics": r.statistics, "violations": r.violations, "table": r.table()}
                        for name, r in results.items()}}
    (OUT / "monte_carlo.json").write_text(json.dumps(dump, indent=2), encoding="utf-8")

    apply_dark_theme()
    fig, ax = plt.subplots(1, 2, figsize=(14, 5.5))
    for a, kpi, lim, xlab in ((ax[0], "yaw_rate_ratio_1s", 0.35, "yaw-rate ratio 1.0 s after steer end [-]"),
                              (ax[1], "lateral_displacement", 1.83, "lateral displacement at 1.07 s [m]")):
        data = {name: np.array([k[kpi] for k in r.kpis]) for name, r in results.items()}
        lo, hi = min(d.min() for d in data.values()), max(d.max() for d in data.values())
        bins = np.linspace(lo, hi + 1e-9, 21)
        for (name, d), c in zip(data.items(), (PASSIVE, CONTROLLED)):
            a.hist(d, bins=bins, color=c, alpha=0.85, label=name, edgecolor="#0f172a", linewidth=1.0)
        a.axvline(lim, color=TEXT_MUTED, ls=":", lw=1.2)
        a.annotate("limit", (lim, a.get_ylim()[1] * 0.92), textcoords="offset points", xytext=(4, 0), color=TEXT_MUTED, fontsize=8)
        a.legend(loc="best")
        style_axis(a, f"{kpi} over {n} plant samples", xlab, "samples")
    fig.suptitle(f"Monte Carlo robustness, sine with dwell 80 km/h / 8 deg (seed {SEED}, synthetic parameters)")
    fig.tight_layout()
    fig.savefig(OUT / "monte_carlo.png", dpi=150, facecolor=fig.get_facecolor())
    plt.close(fig)
    for name, r in results.items():
        print(name, json.dumps({k: (v["count"], v["rate"]) for k, v in r.violations.items()}),
              {k: (round(s["p5"], 3), round(s["p50"], 3), round(s["p95"], 3)) for k, s in r.statistics.items()
               if k in ("yaw_rate_ratio_1s", "lateral_displacement", "max_sideslip_deg")})


if __name__ == "__main__":
    main(int(sys.argv[1]) if len(sys.argv) > 1 else 25)
