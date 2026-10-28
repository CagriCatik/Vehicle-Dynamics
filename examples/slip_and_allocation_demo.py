"""Slip control, split-mu braking, torque vectoring and a failed brake (Plan.md, M5).

Reference car with the Magic Formula tyre set (MF). Compares
    braking efficiency  no ABS / legacy ABS (controllers/abs.py) / PI slip control
    split-mu braking    passive (60/40 brake bias) / allocated stability control
    torque vectoring    passive / allocated, understeering K&C car
    failed FL brake     passive / allocated

Outputs (output/m5/):
    slip_and_allocation.png    four-panel figure
    slip_and_allocation.json   key figures

Run:
    python examples/slip_and_allocation_demo.py      (about three minutes)
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from vehicle_model.allocation import ActuatorHealth
from vehicle_model.controllers.abs import AntilockBrakingSystem
from vehicle_model.controllers.framework import ControlTarget
from vehicle_model.controllers.slip_control import WheelSlipController
from vehicle_model.controllers.stability_control import AllocatedStabilityController
from vehicle_model.core.types import SevenDOFControlInput, SevenDOFState
from vehicle_model.roads.surface import MuSplitRoad, UniformRoad
from vehicle_model.vehicle_models import SevenDOFModel
from vehicle_model.visualization.theme import TEXT_MUTED, apply_dark_theme, style_axis

VEHICLE = "configs/vehicles/reference_passenger_car.yaml"
KC = "configs/suspension/kc_reference_passenger_car.yaml"
MF = "magic_formula_passenger_synthetic"
OUT = Path("output/m5")
COLORS = ("#3987e5", "#d95926", "#199e70")  # validated categorical slots 1-3 (dark surface)
NEW, PASSIVE, LEGACY = COLORS                # colour follows the entity in every panel
DT = 0.005
BIAS = np.array([0.3, 0.3, 0.2, 0.2])      # passive 60/40 brake distribution


def model(road, kc=None):
    return SevenDOFModel.from_yaml(VEHICLE, tyre_name=MF, tyre_rear_name=MF, road=road, kc_config=kc)


def braking_efficiency(mu, mode):
    m = model(UniformRoad(mu=mu))
    p = m.params
    s, tel, v = SevenDOFState(vx=25.0, omega=np.full(4, 25.0 / p.Reff)), None, []
    legacy, pi = AntilockBrakingSystem(), WheelSlipController()
    while s.vx > 3.0:
        Tb = np.full(4, p.max_brake_torque)
        if mode == "legacy ABS":
            Tb = legacy.modulate_brake_torques(Tb, tel, s.vx)
        elif mode == "PI slip control":
            Tb = pi.update(Tb, tel, s.vx, DT)
        s, tel = m.step(s, SevenDOFControlInput(T_brake=Tb), DT)
        v.append(s.vx)
    v = np.array(v)
    i0, i1 = np.argmax(v < 20.0), np.argmax(v < 5.0)
    return float((v[i0] - v[i1]) / ((i1 - i0) * DT) / (mu * p.g))


def run(m, controller, target_fn, duration, v0, passive_brake=None, stop_speed=None):
    p = m.params
    s, tel, t = SevenDOFState(vx=v0, omega=np.full(4, v0 / p.Reff)), None, 0.0
    hist = {"t": [], "psi": [], "r": [], "X": [], "Y": [], "status": []}
    while t < duration and (stop_speed is None or s.vx > stop_speed):
        tgt = target_fn(s)
        if controller is None:
            Tb = (passive_brake if passive_brake is not None else BIAS) * 4 * p.max_brake_torque * tgt.driver_brake_pedal
            u = SevenDOFControlInput(steer_f=tgt.driver_steer_f,
                                     T_drive=np.full(4, tgt.driver_accel_pedal * p.max_drive_torque), T_brake=Tb)
        else:
            out = controller.compute_control(s, tgt, tel, DT)
            hist["status"].append(out.status)
            u = SevenDOFControlInput(steer_f=out.steer_f, T_drive=out.T_drive, T_brake=out.T_brake)
        s, tel = m.step(s, u, DT)
        t += DT
        for k, val in (("t", t), ("psi", s.psi), ("r", s.r), ("X", s.X), ("Y", s.Y)):
            hist[k].append(val)
    return {k: np.array(v) if k != "status" else v for k, v in hist.items()}


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    res: dict = {}
    apply_dark_theme()
    fig, ax = plt.subplots(2, 2, figsize=(15, 10))
    a = ax.ravel()

    # 1 braking efficiency
    mus, modes = (1.0, 0.5, 0.2), ("no ABS", "legacy ABS", "PI slip control")
    eff = {mode: [braking_efficiency(mu, mode) for mu in mus] for mode in modes}
    res["braking_efficiency_20_to_5_mps"] = {mode: dict(zip(map(str, mus), e)) for mode, e in eff.items()}
    x = np.arange(len(mus))
    for j, (mode, c) in enumerate(zip(modes, (PASSIVE, LEGACY, NEW))):
        bars = a[0].bar(x + (j - 1) * 0.26, eff[mode], width=0.24, color=c, label=mode)
        for b, val in zip(bars, eff[mode]):
            a[0].text(b.get_x() + b.get_width() / 2, val + 0.01, f"{val:.2f}", ha="center", color=TEXT_MUTED, fontsize=8)
    a[0].set_xticks(x, [f"mu = {mu}" for mu in mus])
    a[0].set_ylim(0, 1.25)
    a[0].legend(loc="upper left", ncol=3, fontsize=8)
    style_axis(a[0], "Straight braking from 90 km/h, full brake torque", "", "mean deceleration / (mu g) [-]")

    # 2 split-mu braking
    road = MuSplitRoad(mu_left=1.0, mu_right=0.2, split_y=0.0, transition_width=0.05)
    brake = lambda s: ControlTarget(driver_brake_pedal=1.0)
    pas = run(model(road), None, brake, 15.0, 25.0, stop_speed=1.0)
    m = model(road)
    act = run(m, AllocatedStabilityController(m.params), brake, 15.0, 25.0, stop_speed=1.0)
    for h, c, lab in ((pas, PASSIVE, "passive (60/40)"), (act, NEW, "allocated stability control")):
        a[1].plot(h["t"], np.degrees(h["psi"]), color=c, lw=2, label=lab)
    a[1].legend(loc="center right")
    style_axis(a[1], "Split-mu braking (left 1.0, right 0.2), steering fixed", "time [s]", "heading [deg]")
    res["split_mu"] = {
        "passive": {"final_heading_deg": float(np.degrees(pas["psi"][-1])), "time_s": float(pas["t"][-1])},
        "allocated": {"final_heading_deg": float(np.degrees(act["psi"][-1])), "max_abs_Y_m": float(np.max(np.abs(act["Y"]))),
                      "stopping_distance_m": float(act["X"][-1]), "time_s": float(act["t"][-1]),
                      "select_low_distance_m": 25.0**2 / (2 * 0.2 * m.params.g)},
    }

    # 3 torque vectoring on the understeering K&C car
    delta, v0 = 0.04, 20.0
    tv_target = lambda s: ControlTarget(driver_steer_f=delta, driver_accel_pedal=max(0.08 + 0.05 * (v0 - s.vx), 0.0))
    pas = run(model(UniformRoad(), KC), None, tv_target, 3.0, v0)
    m = model(UniformRoad(), KC)
    ctrl = AllocatedStabilityController(m.params)
    act = run(m, ctrl, tv_target, 3.0, v0)
    r_ref = ctrl.reference_yaw_rate(delta, v0, 1.0)
    a[2].plot(pas["t"], pas["r"], color=PASSIVE, lw=2, label="passive")
    a[2].plot(act["t"], act["r"], color=NEW, lw=2, label="torque vectoring")
    a[2].axhline(r_ref, color=TEXT_MUTED, lw=1, ls=":", label="reference yaw rate")
    a[2].legend(loc="lower right")
    style_axis(a[2], "Step steer 72 km/h, K&C car (EG 2.2 deg/g, reference 0.56 deg/g)", "time [s]", "yaw rate [rad/s]")
    res["torque_vectoring"] = {"r_ref": r_ref, "r_passive": float(pas["r"][-1]), "r_tv": float(act["r"][-1])}

    # 4 failed FL brake
    failed = ActuatorHealth(brake_ok=(False, True, True, True))
    half = lambda s: ControlTarget(driver_brake_pedal=0.5)
    pas = run(model(UniformRoad()), None, half, 1.5, 25.0, passive_brake=BIAS * np.array([0, 1, 1, 1]))
    m = model(UniformRoad())
    act = run(m, AllocatedStabilityController(m.params, health=failed), half, 1.5, 25.0)
    a[3].plot(pas["t"], np.degrees(pas["psi"]), color=PASSIVE, lw=2, label="passive, FL brake failed")
    a[3].plot(act["t"], np.degrees(act["psi"]), color=NEW, lw=2, label="allocated, FL brake failed")
    a[3].legend(loc="lower left")
    style_axis(a[3], "Braking (pedal 0.5) with the front-left brake failed", "time [s]", "heading [deg]")
    res["failed_brake"] = {"passive_heading_deg": float(np.degrees(pas["psi"][-1])),
                           "allocated_heading_deg": float(np.degrees(act["psi"][-1])),
                           "allocated_status": sorted(set(act["status"]))}

    fig.suptitle("Slip control and bounded control allocation – reference car, MF tyres (synthetic)")
    fig.tight_layout()
    fig.savefig(OUT / "slip_and_allocation.png", dpi=150, facecolor=fig.get_facecolor())
    plt.close(fig)
    (OUT / "slip_and_allocation.json").write_text(json.dumps(res, indent=2), encoding="utf-8")
    print(json.dumps(res, indent=2))


if __name__ == "__main__":
    main()
