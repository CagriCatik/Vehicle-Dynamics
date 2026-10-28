"""Magic Formula tyre and tyre relaxation (Plan.md, M3).

Shows what the generic tyre plot suite (``python -m vehicle_model.tire_model.plots``)
does not: the aligning moment, the friction self-similarity of the combined-slip
map, and the transient (relaxation) behaviour alone and in the 7-DOF vehicle.

Outputs (output/tyres/):
    tyre_models_m3.png          six-panel figure
    tyre_models_m3_kpis.json    stiffnesses, peaks and transient deviations

Run:
    python examples/tyre_models_m3_demo.py
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from vehicle_model.core.types import SevenDOFControlInput, SevenDOFState
from vehicle_model.tire_model import TyreRelaxation, load_tyre
from vehicle_model.vehicle_models import SevenDOFModel, TransientSevenDOFModel
from vehicle_model.visualization.theme import TEXT_MUTED, apply_dark_theme, style_axis

OUT = Path("output/tyres")
VEHICLE = "configs/vehicles/reference_passenger_car.yaml"
COLORS = ("#3987e5", "#d95926", "#199e70")   # validated categorical slots 1-3 (dark surface)
LOADS = (2000.0, 4000.0, 7000.0)             # [N]
SIGMAS = (0.2, 0.4, 0.8)                     # [m] lateral relaxation lengths compared


def step_steer(model, s0, T=0.6, dt=0.005):
    u = SevenDOFControlInput(steer_f=0.03, T_drive=np.full(4, 60.0))
    t, r = [0.0], [s0.r]
    s = s0
    for k in range(int(round(T / dt))):
        s, _ = model.step(s, u, dt)
        t.append((k + 1) * dt)
        r.append(s.r)
    return np.array(t), np.array(r)


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    mf = load_tyre("magic_formula_passenger_synthetic")
    fiala = load_tyre("fiala_passenger_synthetic")
    apply_dark_theme()
    fig, ax = plt.subplots(2, 3, figsize=(17, 9.5))
    a = ax.ravel()
    kpis: dict = {"tyre": mf.provenance.describe(), "loads_N": {}}

    # 1  lateral force over slip angle, three loads (Fiala at Fz0 as reference)
    alpha = np.radians(np.linspace(0.0, 15.0, 301))
    for Fz, c in zip(LOADS, COLORS):
        _, Fy, Mz = mf.forces_and_moment(0.0, alpha, Fz, 1.0)
        a[0].plot(np.degrees(alpha), Fy / Fz, color=c, lw=2, label=f"MF  Fz = {Fz / 1e3:.0f} kN")
        a[1].plot(np.degrees(alpha), Mz, color=c, lw=2, label=f"Fz = {Fz / 1e3:.0f} kN")
        i = int(np.argmax(Fy))
        kpis["loads_N"][f"{Fz:.0f}"] = {
            "C_alpha_per_Fz_1_per_rad": float(mf.cornering_stiffness(Fz) / Fz),
            "C_kappa_per_Fz": float(mf.longitudinal_stiffness(Fz) / Fz),
            "peak_mu_y": float(Fy[i] / Fz), "peak_alpha_deg": float(np.degrees(alpha[i])),
            "peak_Mz_Nm": float(-Mz.min()), "trail_at_zero_slip_m": float(-Mz[1] / Fy[1]),
        }
    _, Fy_f = fiala.forces(0.0, alpha, 4000.0, 1.0)
    a[0].plot(np.degrees(alpha), Fy_f / 4000.0, color=TEXT_MUTED, lw=1.2, ls="--", label="Fiala  Fz = 4 kN")
    a[0].legend(loc="lower right")
    style_axis(a[0], "Lateral force (pure side slip, mu = 1)", "slip angle [deg]", "Fy / Fz [-]")
    a[1].legend(loc="lower right")
    style_axis(a[1], "Self-aligning moment (pneumatic trail)", "slip angle [deg]", "Mz [N m]")

    # 2  combined slip: Fy/Fz over kappa at fixed slip angles
    kappa = np.linspace(-0.4, 0.4, 401)
    for deg, c in zip((2.0, 5.0, 10.0), COLORS):
        Fx, Fy = mf.forces(kappa, math.radians(deg), 4000.0, 1.0)
        a[2].plot(Fx / 4000.0, Fy / 4000.0, color=c, lw=2, label=f"alpha = {deg:.0f} deg, kappa -0.4..0.4")
    th = np.linspace(0, 2 * np.pi, 361)
    a[2].plot(np.cos(th), np.sin(th), color=TEXT_MUTED, lw=1, ls=":", label="friction ellipse (mu_x = mu_y = 1)")
    a[2].set_aspect("equal")
    a[2].set_xlim(-1.15, 1.15)
    a[2].set_ylim(0, 1.15)
    a[2].legend(loc="lower center", fontsize=8)
    style_axis(a[2], "Combined slip at Fz0 (G weighting + ellipse limit)", "Fx / Fz [-]", "Fy / Fz [-]")

    # 3  self-similarity in mu: normalised curves coincide
    for mu, c in zip((1.0, 0.6, 0.3), COLORS):
        Fx, Fy = mf.forces(mu * kappa, np.arctan(mu * math.tan(math.radians(5.0))), 4000.0, mu)
        a[3].plot(kappa, Fy / (mu * 4000.0), color=c, lw=2 if mu == 1.0 else 1.5,
                  ls="-" if mu == 1.0 else "--", label=f"mu = {mu}")
    a[3].legend(loc="lower center")
    style_axis(a[3], "Combined slip is self-similar in mu (alpha = 5 deg at mu = 1)",
               "kappa / mu [-]", "Fy / (mu Fz) [-]")

    # 4  relaxation: lateral slip build-up over rolled distance
    x = np.linspace(0.0, 2.5, 251)
    for sig, c in zip(SIGMAS, COLORS):
        a[4].plot(x, 1.0 - np.exp(-x / sig), color=c, lw=2, label=f"sigma_alpha = {sig} m")
    a[4].axhline(1.0 - math.exp(-1.0), color=TEXT_MUTED, lw=1, ls=":")
    a[4].annotate("63 % after one relaxation length", (1.5, 1.0 - math.exp(-1.0)), textcoords="offset points",
                  xytext=(0, -14), color=TEXT_MUTED, fontsize=8)
    a[4].legend(loc="lower right")
    style_axis(a[4], "Slip step response (same at every speed)", "rolled distance [m]", "alpha' / alpha_ss [-]")

    # 5  vehicle: step steer, quasi-static vs relaxation
    base = SevenDOFModel.from_yaml(VEHICLE, tyre_name="magic_formula_passenger_synthetic",
                                   tyre_rear_name="magic_formula_passenger_synthetic")
    p = base.params
    s0 = SevenDOFState(vx=20.0, omega=np.full(4, 20.0 / p.Reff))
    t, r_b = step_steer(base, s0)
    a[5].plot(t, r_b, color=TEXT_MUTED, lw=1.5, ls="--", label="quasi-static slips")
    kpis["step_steer_20mps"] = {"r_ss_quasi_static": float(r_b[-1])}
    for sig, c in zip(SIGMAS, COLORS):
        tr = TransientSevenDOFModel(base.params, base.tyre_front, base.road, base.tyre_rear,
                                    relaxation=TyreRelaxation(sigma_kappa=sig / 4, sigma_alpha=sig))
        _, r_t = step_steer(tr, tr.steady_slip_state(s0, SevenDOFControlInput()))
        a[5].plot(t, r_t, color=c, lw=2, label=f"sigma_alpha = {sig} m")
        kpis["step_steer_20mps"][f"sigma_{sig}"] = {
            "r_at_20ms_rel": float(r_t[4] / r_b[4]),
            "peak_rel": float(r_t.max() / r_b.max()),
            "final_rel": float(r_t[-1] / r_b[-1]),
        }
    a[5].set_xlim(0, 0.6)
    a[5].legend(loc="lower right")
    style_axis(a[5], "7-DOF step steer, 20 m/s (MF tyres)", "time [s]", "yaw rate [rad/s]")

    fig.suptitle("Magic Formula tyre and tyre relaxation – synthetic parameters, not a real tyre")
    fig.tight_layout()
    fig.savefig(OUT / "tyre_models_m3.png", dpi=150, facecolor=fig.get_facecolor())
    plt.close(fig)
    (OUT / "tyre_models_m3_kpis.json").write_text(json.dumps(kpis, indent=2), encoding="utf-8")
    print(json.dumps(kpis, indent=2))
    print(f"Saved results to {OUT}/")


if __name__ == "__main__":
    main()
