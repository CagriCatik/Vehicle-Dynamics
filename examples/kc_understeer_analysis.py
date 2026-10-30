"""Influence of suspension kinematics & compliance on understeer.

Quasi-steady steer ramp at constant speed (ISO 4138, constant-speed method) with
the 7-DOF reference car, switching the K&C effects on one by one, plus a sweep
of the front roll-centre height and the roll-stiffness split. The understeer
angle ``delta - L a_y/v^2`` separates the geometric steer demand from what the
tyres and the suspension add.

Outputs (output/kc_analysis/):
    kc_understeer.png         four-panel figure
    kc_understeer.json        linear budget and simulated gradients per variant

Run:
    python examples/kc_understeer_analysis.py
"""

from __future__ import annotations

import dataclasses
import json
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from vehicle_model.core.types import SevenDOFControlInput, SevenDOFState
from vehicle_model.handling_analysis import understeer_budget
from vehicle_model.suspension.kinematics import QuasiStaticChassis, SuspensionKC, ToeMap, load_kc
from vehicle_model.vehicle_models import SevenDOFModel
from vehicle_model.visualization.theme import TEXT_MUTED, apply_dark_theme, style_axis

VEHICLE = "configs/vehicles/reference_passenger_car.yaml"
KC = "configs/suspension/kc_reference_passenger_car.yaml"
OUT = Path("output/kc_analysis")
COLORS = ("#3987e5", "#d95926", "#199e70", "#c98500")  # validated categorical slots 1-4 (dark surface)
V = 25.0                 # [m/s] constant speed
RAMP = math.radians(0.25)  # [rad/s] front steer rate; results identical at 0.1 deg/s (quasi-steady)
DT = 0.01
G = 9.81


def steer_ramp(model: SevenDOFModel, ay_max: float = 0.75 * G):
    """Steer ramp at constant speed; returns (a_y, delta) sampled until a_y_max or saturation."""
    p = model.params
    s = SevenDOFState(vx=V, omega=np.full(4, V / p.Reff))
    ay_hist, d_hist, t = [], [], 0.0
    while True:
        delta = RAMP * t
        f = 0.5 * p.rho * p.Cd * p.Af * s.vx**2 + p.c_rr * p.mass * p.g + 3.0 * p.mass * (V - s.vx)
        s, tel = model.step(s, SevenDOFControlInput(steer_f=delta, T_drive=np.full(4, f * p.Reff / 4)), DT)
        t += DT
        ay = s.vx * s.r  # steady-state lateral acceleration on the path
        ay_hist.append(ay)
        d_hist.append(delta)
        if ay > ay_max or (len(ay_hist) > 50 and ay < max(ay_hist) - 0.2) or t > 60:
            break
    return np.array(ay_hist), np.array(d_hist)


def understeer_angle(model, ay, delta):
    return delta - model.params.wheelbase * ay / V**2


def gradient(ay, ua, lo, hi):
    m = (ay > lo * G) & (ay < hi * G)
    return float(np.polyfit(ay[m], ua[m], 1)[0])


def model_with(kc: SuspensionKC | None, params=None) -> SevenDOFModel:
    base = SevenDOFModel.from_yaml(VEHICLE)
    p = params or base.params
    chassis = None
    if kc is not None:
        masses = QuasiStaticChassis.from_yaml(p, VEHICLE, None).masses
        if params is not None:  # keep the elastic roll split consistent with k_roll_f
            K = masses.roll_stiffness_front + masses.roll_stiffness_rear
            masses = dataclasses.replace(masses, roll_stiffness_front=p.k_roll_f * K,
                                         roll_stiffness_rear=(1 - p.k_roll_f) * K)
        chassis = QuasiStaticChassis(p, masses, kc)
    return SevenDOFModel(p, base.tyre_front, base.road, base.tyre_rear, chassis=chassis)


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    full = load_kc(KC)
    no_steer = dict(bump_steer_front=ToeMap(), bump_steer_rear=ToeMap(), compliance_steer_front=0.0, compliance_steer_rear=0.0)
    variants = {
        "tyres only (no K&C)": None,
        "+ roll centres, Ackermann": dataclasses.replace(full, **no_steer),
        "+ roll steer": dataclasses.replace(full, compliance_steer_front=0.0, compliance_steer_rear=0.0),
        "+ compliance steer (full K&C)": full,
    }
    results = {}
    apply_dark_theme()
    fig, ax = plt.subplots(2, 2, figsize=(15, 10))
    a = ax.ravel()
    for (name, kc), c in zip(variants.items(), COLORS):
        model = model_with(kc)
        ay, delta = steer_ramp(model)
        ua = understeer_angle(model, ay, delta)
        b = understeer_budget(model).deg_per_g(G)
        results[name] = {
            "linear_budget_deg_per_g": b,
            "simulated_EG_deg_per_g": {
                "0.05-0.15 g": math.degrees(gradient(ay, ua, 0.05, 0.15)) * G,
                "0.3-0.5 g": math.degrees(gradient(ay, ua, 0.3, 0.5)) * G,
            },
            "max_ay_g": float(ay.max() / G),
        }
        a[0].plot(ay / G, np.degrees(ua), color=c, lw=2, label=name)
        # local gradient over a_y (smoothed by a moving fit)
        centres = np.linspace(0.05, min(0.65, ay.max() / G - 0.05), 25)
        local = [math.degrees(gradient(ay, ua, x - 0.05, x + 0.05)) * G for x in centres]
        a[1].plot(centres, local, color=c, lw=2, label=name)
        a[1].plot([0.0], [b["total"]], "o", ms=8, color=c)
    a[0].legend(loc="upper left")
    style_axis(a[0], f"Understeer angle, steer ramp at {V * 3.6:.0f} km/h", "lateral acceleration [g]", "delta - L a_y/v² [deg]")
    a[1].text(0.01, 0.97, "dots: linear budget at a_y -> 0\n(roll centres and Ackermann do not enter it:\nfirst two dots coincide)", transform=a[1].transAxes, va="top", color=TEXT_MUTED, fontsize=9)
    a[1].legend(loc="lower right")
    style_axis(a[1], "Local understeer gradient", "lateral acceleration [g]", "EG [deg/g]")

    # budget components of the full K&C car
    b = results["+ compliance steer (full K&C)"]["linear_budget_deg_per_g"]
    parts = ["tyres", "roll_steer", "compliance_steer"]
    bars = a[2].barh(range(3), [b[k] for k in parts], color=COLORS[0], height=0.5)
    for rect, k in zip(bars, parts):
        a[2].text(rect.get_width() + 0.02, rect.get_y() + rect.get_height() / 2, f"{b[k]:.2f}", va="center", color=TEXT_MUTED)
    a[2].set_yticks(range(3), ["tyres", "roll steer", "compliance steer"])
    a[2].invert_yaxis()
    a[2].set_xlim(0, 1.25 * max(b[k] for k in parts))
    style_axis(a[2], f"Linear understeer budget, full K&C (total {b['total']:.2f} deg/g)", "[deg/g]", "")

    # sensitivity at 0.4-0.6 g: front roll-centre height and roll stiffness split (full K&C otherwise)
    sweep = {}
    base_p = SevenDOFModel.from_yaml(VEHICLE).params
    for label, c, xs, make in (
        ("front roll-centre height [m]", COLORS[0], (0.0, 0.05, 0.10, 0.15),
         lambda x: model_with(dataclasses.replace(full, roll_centre_height_front=x))),
        ("front roll-stiffness share k_roll_f [-]", COLORS[1], (0.45, 0.55, 0.60, 0.70),
         lambda x: model_with(full, dataclasses.replace(base_p, k_roll_f=x))),
    ):
        eg = []
        for x in xs:
            m = make(x)
            ay, delta = steer_ramp(m)
            eg.append(math.degrees(gradient(ay, understeer_angle(m, ay, delta), 0.4, 0.6)) * G)
        sweep[label] = dict(zip(map(str, xs), eg))
        xn = (np.array(xs) - xs[0]) / (xs[-1] - xs[0])
        a[3].plot(xn, eg, "o-", color=c, lw=2, ms=7, label=f"{label}: {xs[0]} … {xs[-1]}")
    results["sensitivity_EG_0.4-0.6g_deg_per_g"] = sweep
    a[3].legend(loc="upper left", fontsize=8)
    style_axis(a[3], "Understeer gradient at 0.4-0.6 g vs parameter (full K&C)", "parameter, normalised to its range [-]", "EG [deg/g]")

    fig.suptitle("Suspension K&C and understeer – reference car, synthetic K&C set")
    fig.tight_layout()
    fig.savefig(OUT / "kc_understeer.png", dpi=150, facecolor=fig.get_facecolor())
    plt.close(fig)
    (OUT / "kc_understeer.json").write_text(json.dumps(results, indent=2), encoding="utf-8")
    print(json.dumps(results, indent=2))


if __name__ == "__main__":
    main()
