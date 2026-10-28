"""Handling analysis of the virtual reference vehicle (Plan.md, M1).

Linearises the 7-DOF reference car (configs/vehicles/reference_passenger_car.yaml),
reports the analytical handling characteristics and compares the linear step
response with the nonlinear 7-DOF model.

Outputs (output/handling_analysis/):
    reference_vehicle_handling.png   six-panel figure
    reference_vehicle_kpis.json      scalar characteristics + 7-DOF vs linear deviations
    reference_vehicle_vs_speed.csv   linear characteristics over speed

Run:
    python examples/handling_analysis_reference_vehicle.py
"""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from vehicle_model.core.types import SevenDOFControlInput, SevenDOFState
from vehicle_model.handling_analysis import (
    LinearSingleTrack,
    linear_params_from_seven_dof,
    step_response_metrics,
)
from vehicle_model.vehicle_models import SevenDOFModel
from vehicle_model.visualization.theme import TEXT_MUTED, apply_dark_theme, style_axis

VEHICLE = "configs/vehicles/reference_passenger_car.yaml"
OUT = Path("output/handling_analysis")
SPEEDS = (10.0, 20.0, 30.0)                       # [m/s] compared operating points
COLORS = ("#3987e5", "#d95926", "#199e70")        # validated categorical slots 1-3 (dark surface)
AY_STEP = 0.3                                     # [m/s^2] small-signal step amplitude
DT, DURATION = 0.005, 3.0


def simulate_7dof_step(model: SevenDOFModel, vx0: float, delta: float):
    """Front-steer step at t = 0 with speed-holding drive torque (feed-forward + P)."""
    p = model.params
    state = SevenDOFState(vx=vx0, omega=np.full(4, vx0 / p.Reff))
    t, r = [0.0], [0.0]
    for k in range(int(round(DURATION / DT))):
        f_res = 0.5 * p.rho * p.Cd * p.Af * state.vx**2 + p.c_rr * p.mass * p.g
        f_req = f_res + 2.0 * p.mass * (vx0 - state.vx)
        u = SevenDOFControlInput(steer_f=delta, T_drive=np.full(4, f_req * p.Reff / 4.0))
        state, _ = model.step(state, u, DT)
        t.append((k + 1) * DT)
        r.append(state.r)
    return np.array(t), np.array(r)


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    nl = SevenDOFModel.from_yaml(VEHICLE)
    lin = LinearSingleTrack(linear_params_from_seven_dof(nl.params, nl.tyre_front, nl.tyre_rear))
    lp = lin.params
    g = nl.params.g

    # ---------------------------------------------------------------- linear characteristics
    v = np.linspace(5.0, 60.0, 111)  # below ~5 m/s the linear tyre model is not meaningful
    gain = np.array([lin.yaw_rate_gain(x) for x in v])
    modal = [lin.modal_properties(x) for x in v]
    wn = np.array([m.natural_frequency for m in modal])
    zeta = np.array([m.damping_ratio for m in modal])

    with open(OUT / "reference_vehicle_vs_speed.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["vx_mps", "yaw_rate_gain_1ps", "lateral_accel_gain_mps2", "sideslip_gain", "omega_n_radps", "zeta", "stable"])
        for x, m in zip(v, modal):
            w.writerow([f"{x:.3f}", f"{lin.yaw_rate_gain(x):.6f}", f"{lin.lateral_accel_gain(x):.6f}",
                        f"{lin.sideslip_gain(x):.6f}", f"{m.natural_frequency:.6f}", f"{m.damping_ratio:.6f}", m.stable])

    # ---------------------------------------------------------------- 7-DOF vs linear steps
    steps = {}
    for vx in SPEEDS:
        delta = AY_STEP / lin.lateral_accel_gain(vx)
        t, r_nl = simulate_7dof_step(nl, vx, delta)
        r_lin = lin.state_space(vx).step_response(t, np.array([delta, 0.0]))[:, 0]
        m_nl, m_lin = step_response_metrics(t, r_nl), step_response_metrics(t, r_lin)
        r_ss = lin.yaw_rate_gain(vx) * delta
        steps[vx] = dict(t=t, r_nl=r_nl, r_lin=r_lin, r_ss=r_ss, delta=delta, m_nl=m_nl, m_lin=m_lin)

    kpis = {
        "vehicle": VEHICLE,
        "linear_params": {"mass": lp.mass, "Iz": lp.Iz, "lf": lp.lf, "lr": lp.lr, "C_f": lp.C_f, "C_r": lp.C_r},
        "understeer_gradient_rad_per_mps2": lin.understeer_gradient,
        "understeer_gradient_deg_per_g": lin.understeer_gradient_deg_per_g(g),
        "characteristic_speed_kph": None if lin.characteristic_speed is None else lin.characteristic_speed * 3.6,
        "critical_speed_kph": None if lin.critical_speed is None else lin.critical_speed * 3.6,
        "step_ay_mps2": AY_STEP,
        "operating_points": {
            f"{vx:.0f}_mps": {
                "omega_n_radps": lin.modal_properties(vx).natural_frequency,
                "zeta": lin.modal_properties(vx).damping_ratio,
                "yaw_rate_gain_1ps": lin.yaw_rate_gain(vx),
                "response_time_s": {"seven_dof": s["m_nl"].response_time, "linear": s["m_lin"].response_time},
                "overshoot": {"seven_dof": s["m_nl"].overshoot, "linear": s["m_lin"].overshoot},
                "steady_yaw_rate_deviation_rel": float(s["m_nl"].steady_state / s["r_ss"] - 1.0),
                "max_transient_deviation_rel": float(np.max(np.abs(s["r_nl"] - s["r_lin"])) / s["r_ss"]),
            }
            for vx, s in steps.items()
        },
        "validity": "linear range only (a_y = 0.3 m/s^2); tyre-only understeer, no elastokinematics",
    }
    (OUT / "reference_vehicle_kpis.json").write_text(json.dumps(kpis, indent=2), encoding="utf-8")

    # ---------------------------------------------------------------- figure
    apply_dark_theme()
    fig, ax = plt.subplots(2, 3, figsize=(16, 9))
    a = ax.ravel()

    a[0].plot(v * 3.6, gain, color=COLORS[0], lw=2)
    a[0].plot(v * 3.6, v / lp.wheelbase, color=TEXT_MUTED, lw=1, ls=":", label="neutral steer v/L")
    if lin.characteristic_speed:
        vc = lin.characteristic_speed
        a[0].plot([vc * 3.6], [lin.yaw_rate_gain(vc)], "o", ms=8, color=COLORS[0])
        a[0].annotate(f"v_char = {vc * 3.6:.0f} km/h", (vc * 3.6, lin.yaw_rate_gain(vc)),
                      textcoords="offset points", xytext=(-90, 10), color=TEXT_MUTED)
    a[0].set_ylim(0, 1.3 * gain.max())
    a[0].legend(loc="lower right")
    style_axis(a[0], "Steady-state yaw rate gain r/δ_f", "speed [km/h]", "[1/s]")

    a[1].plot(v * 3.6, wn / (2 * math.pi), color=COLORS[0], lw=2)
    style_axis(a[1], "Undamped natural frequency", "speed [km/h]", "f_n [Hz]")

    a[2].plot(v * 3.6, zeta, color=COLORS[0], lw=2)
    style_axis(a[2], "Damping ratio of lateral dynamics", "speed [km/h]", "ζ [-]")

    freq = np.logspace(-1, 1, 200)  # [Hz]
    for vx, c in zip(SPEEDS, COLORS):
        H = lin.state_space(vx).frequency_response(2 * math.pi * freq)[:, 0, 0]
        mag = np.abs(H) / abs(H[0])
        a[3].semilogx(freq, 20 * np.log10(mag), color=c, lw=2, label=f"{vx * 3.6:.0f} km/h")
        a[4].semilogx(freq, np.degrees(np.unwrap(np.angle(H))), color=c, lw=2, label=f"{vx * 3.6:.0f} km/h")
    a[3].legend()
    a[4].legend()
    style_axis(a[3], "Yaw rate response |r/δ_f| (normalised to 0.1 Hz)", "frequency [Hz]", "[dB]")
    style_axis(a[4], "Yaw rate phase", "frequency [Hz]", "[deg]")

    for (vx, s), c in zip(steps.items(), COLORS):
        a[5].plot(s["t"], s["r_nl"] / s["r_ss"], color=c, lw=2, label=f"7-DOF {vx * 3.6:.0f} km/h")
        a[5].plot(s["t"], s["r_lin"] / s["r_ss"], color=c, lw=1.2, ls="--", label=f"linear {vx * 3.6:.0f} km/h")
    a[5].set_xlim(0, 1.0)
    a[5].legend(loc="lower right", ncol=2, fontsize=8)
    style_axis(a[5], f"Step steer (a_y = {AY_STEP} m/s²): 7-DOF vs linear", "time [s]", "r / r_ss,linear [-]")

    fig.suptitle("Reference passenger car – linear handling analysis (synthetic parameters)")
    fig.tight_layout()
    fig.savefig(OUT / "reference_vehicle_handling.png", dpi=150, facecolor=fig.get_facecolor())
    plt.close(fig)

    # ---------------------------------------------------------------- console report
    print(f"Vehicle: {VEHICLE}")
    print(f"  C_f = {lp.C_f / 1e3:.1f} kN/rad, C_r = {lp.C_r / 1e3:.1f} kN/rad")
    print(f"  understeer gradient EG = {lin.understeer_gradient_deg_per_g(g):+.3f} deg/g")
    if lin.characteristic_speed:
        print(f"  characteristic speed  = {lin.characteristic_speed * 3.6:.1f} km/h")
    print(f"  {'v [km/h]':>8} | {'f_n [Hz]':>8} | {'zeta':>6} | {'t90 7DOF/lin [s]':>17} | {'dev r_ss':>8} | {'max dev r(t)':>12}")
    for vx, s in steps.items():
        mp = lin.modal_properties(vx)
        print(f"  {vx * 3.6:8.0f} | {mp.natural_frequency / (2 * math.pi):8.2f} | {mp.damping_ratio:6.3f} | "
              f"{s['m_nl'].response_time:7.3f}/{s['m_lin'].response_time:<7.3f}   | "
              f"{s['m_nl'].steady_state / s['r_ss'] - 1:+8.3%} | {np.max(np.abs(s['r_nl'] - s['r_lin'])) / s['r_ss']:12.3%}")
    print(f"Saved results to {OUT}/")


if __name__ == "__main__":
    main()
