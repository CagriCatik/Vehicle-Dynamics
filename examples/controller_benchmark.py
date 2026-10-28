"""Controller comparison on one virtual plant (Plan.md, M6).

Plant: reference car, MF tyres, synthetic K&C set. Controllers: passive, PI,
LQR, gain-scheduled LQR (TV and 4WS+TV) and LTV-MPC (TV and 4WS+TV); all
model-based laws use the linear single track of the car *without* K&C as design
model, so plant and controller model differ on purpose (Plan.md 10.2).

Part 1  scenarios step steer, sine with dwell, split-mu braking on the nominal plant
Part 2  robustness (Plan.md 10.4): sine with dwell on plant variants
        (+15 % mass and yaw inertia; rear cornering stiffness -20 %)

Outputs (output/controller_benchmark/):
    kpis.md, kpis.json           KPI tables
    controller_benchmark.png     yaw rate in the sine with dwell, nominal and variants

Run:
    python examples/controller_benchmark.py     (several minutes)
"""

from __future__ import annotations

import dataclasses
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from vehicle_model.controllers.motion_control import MotionController
from vehicle_model.controllers.yaw_laws import INPUTS_4WS_TV, INPUTS_TV, LQRYawLaw, LTVMPCYawLaw, YawRatePI
from vehicle_model.handling_analysis import linear_params_from_seven_dof
from vehicle_model.simulation.controller_benchmark import (
    kpi_table,
    run_benchmark,
    sine_with_dwell_scenario,
    split_mu_braking_scenario,
    step_steer_scenario,
)
from vehicle_model.suspension.kinematics import QuasiStaticChassis, load_kc
from vehicle_model.tire_model import MagicFormulaTyre, load_tyre
from vehicle_model.vehicle_models import SevenDOFModel
from vehicle_model.visualization.theme import TEXT_MUTED, apply_dark_theme, style_axis

VEHICLE = "configs/vehicles/reference_passenger_car.yaml"
KC = "configs/suspension/kc_reference_passenger_car.yaml"
MF = "magic_formula_passenger_synthetic"
OUT = Path("output/controller_benchmark")
SPEEDS = (5, 10, 15, 20, 25, 30, 40)
# validated categorical slots 1-6 (dark surface); passive is drawn in muted ink
COLORS = ("#3987e5", "#d95926", "#199e70", "#c98500", "#d55181", "#008300")


def make_plant(variant: str = "nominal"):
    base = SevenDOFModel.from_yaml(VEHICLE, tyre_name=MF, tyre_rear_name=MF)
    p, tyre_f, tyre_r = base.params, base.tyre_front, base.tyre_rear
    masses = QuasiStaticChassis.from_yaml(p, VEHICLE, None).masses
    if variant == "heavy":
        extra = 0.15 * p.mass
        p = dataclasses.replace(p, mass=p.mass + extra, Iz=1.15 * p.Iz)
        masses = dataclasses.replace(masses, sprung_mass=masses.sprung_mass + extra)
    elif variant == "soft_rear_tyres":
        mf = load_tyre(MF)
        tyre_r = MagicFormulaTyre(dataclasses.replace(mf.params, PKY1=0.8 * mf.params.PKY1), mf.provenance)
    elif variant != "nominal":
        raise ValueError(variant)
    chassis = QuasiStaticChassis(p, masses, load_kc(KC))
    return lambda road: SevenDOFModel(p, tyre_f, road, tyre_r, chassis=chassis)


def controllers():
    src = SevenDOFModel.from_yaml(VEHICLE, tyre_name=MF, tyre_rear_name=MF)
    d = linear_params_from_seven_dof(src.params, src.tyre_front, src.tyre_rear)
    return {
        "passive": None,
        "PI (TV)": lambda p: MotionController(p, YawRatePI()),
        "LQR (TV)": lambda p: MotionController(p, LQRYawLaw(d, INPUTS_TV, design_speed=20.0)),
        "GS-LQR (TV)": lambda p: MotionController(p, LQRYawLaw(d, INPUTS_TV, design_speed=None, schedule_speeds=SPEEDS)),
        "GS-LQR (4WS+TV)": lambda p: MotionController(p, LQRYawLaw(d, INPUTS_4WS_TV, design_speed=None, schedule_speeds=SPEEDS)),
        "LTV-MPC (TV)": lambda p: MotionController(p, LTVMPCYawLaw(d, INPUTS_TV)),
        "LTV-MPC (4WS+TV)": lambda p: MotionController(p, LTVMPCYawLaw(d, INPUTS_4WS_TV)),
    }


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    ctrls = controllers()
    nominal = run_benchmark(make_plant(), ctrls, [step_steer_scenario(), sine_with_dwell_scenario(),
                                                   split_mu_braking_scenario()])
    variants = {v: run_benchmark(make_plant(v), ctrls, [sine_with_dwell_scenario()]) for v in ("heavy", "soft_rear_tyres")}

    md = ["# Controller benchmark (synthetic reference car, MF tyres, K&C)\n", "## Nominal plant", kpi_table(nominal)]
    for v, res in variants.items():
        md += [f"\n## Plant variant: {v}", kpi_table(res)]
    (OUT / "kpis.md").write_text("\n".join(md) + "\n", encoding="utf-8")
    dump = {"nominal": [{"scenario": r.scenario, "controller": r.controller, "kpis": r.kpis} for r in nominal]}
    for v, res in variants.items():
        dump[v] = [{"scenario": r.scenario, "controller": r.controller, "kpis": r.kpis} for r in res]
    (OUT / "kpis.json").write_text(json.dumps(dump, indent=2), encoding="utf-8")

    apply_dark_theme()
    fig, ax = plt.subplots(1, 3, figsize=(18, 6), sharey=True)
    panels = [("nominal", [r for r in nominal if r.scenario == "sine_with_dwell"])]
    panels += [(v, res) for v, res in variants.items()]
    for a, (title, res) in zip(ax, panels):
        c_iter = iter(COLORS)
        for r in res:
            color = TEXT_MUTED if r.controller == "passive" else next(c_iter)
            a.plot(r.history["t"], r.history["r"], color=color, lw=1.2 if r.controller == "passive" else 1.8,
                   ls="--" if r.controller == "passive" else "-", label=r.controller)
        a.set_ylim(-0.8, 0.8)
        style_axis(a, f"Sine with dwell 80 km/h, 8 deg – {title}", "time [s]", "yaw rate [rad/s]" if a is ax[0] else "")
    ax[0].legend(loc="lower left", fontsize=8)
    fig.suptitle("Yaw-rate controllers on the same plant (synthetic parameters, simulation modelled on FMVSS 126)")
    fig.tight_layout()
    fig.savefig(OUT / "controller_benchmark.png", dpi=150, facecolor=fig.get_facecolor())
    plt.close(fig)
    print("\n".join(md))


if __name__ == "__main__":
    main()
