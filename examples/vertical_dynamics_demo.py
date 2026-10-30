"""Quarter-car frequency responses and ISO 8608 roads.

Analytical frequency responses of the passive quarter car against simulated
points (nonlinear model, harmonic road, steady-state amplitude), the same for
skyhook and groundhook semi-active damping, and the ISO 8608 road classes with
the spectrum of a generated class C profile.

Outputs (output/vertical_dynamics/): vertical_dynamics.png, vertical_dynamics.json

Run:
    python examples/vertical_dynamics_demo.py     (about one minute)
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import welch

from vehicle_model.roads.road_profile import ISO8608_CLASSES, ISO8608Profile
from vehicle_model.suspension.controllers import GroundhookController, SkyhookController
from vehicle_model.suspension.quarter_car import QuarterCarConfig, QuarterCarModel, QuarterCarState
from vehicle_model.suspension.vertical_analysis import frequency_response, modes
from vehicle_model.visualization.theme import TEXT_MUTED, apply_dark_theme, style_axis

OUT = Path("output/vertical_dynamics")
SKY, PASSIVE, GROUND = "#3987e5", "#d95926", "#199e70"   # validated slots 1-3 (dark surface)
CFG = QuarterCarConfig()
Z = 0.003  # [m] road amplitude


def simulated_gain(f_hz, controller=None, dt=2e-4, T=4.0):
    model = QuarterCarModel(CFG)
    c = controller() if controller else None
    w = 2 * math.pi * f_hz
    road = lambda t: (Z * math.sin(w * t), Z * w * math.cos(w * t))
    s, t, acc, load, ts = QuarterCarState(), 0.0, [], [], []
    while t < T:
        damper = c.compute_damping(s.dzs, s.dzu, dt) if c else None
        s_next, out = model.step_with_road(s, t, dt, road, damper_c=damper)
        if t >= T / 2:
            ts.append(t); acc.append(out.ddzs); load.append(out.dynamic_tyre_load)
        s, t = s_next, t + dt
    ts = np.array(ts)
    basis = np.column_stack([np.sin(w * ts), np.cos(w * ts), np.ones_like(ts)])
    amp = lambda y: math.hypot(*np.linalg.lstsq(basis, np.array(y), rcond=None)[0][:2])  # fundamental harmonic
    return amp(acc) / Z, amp(load) / Z


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    f_line = np.logspace(np.log10(0.5), np.log10(25.0), 300)
    H = frequency_response(CFG, f_line)
    f_pts = np.array([0.8, 1.45, 2.5, 4.0, 6.0, 9.0, 11.5, 15.0, 20.0])
    sims = {name: np.array([simulated_gain(f, c) for f in f_pts])
            for name, c in (("passive", None), ("skyhook", SkyhookController), ("groundhook", GroundhookController))}
    (fb, zb), (fh, zh) = modes(CFG)

    apply_dark_theme()
    fig, ax = plt.subplots(1, 3, figsize=(18, 5.5))
    for a, key, col, ylab in ((ax[0], "body_acceleration", 0, "|a_s / z_r| [1/s^2]"),
                              (ax[1], "dynamic_tyre_load", 1, "|F_dyn / z_r| [N/m]")):
        a.loglog(f_line, np.abs(H[key]), color=PASSIVE, lw=1.5, label="passive, analytical")
        for name, c, mk in (("passive", PASSIVE, "o"), ("skyhook", SKY, "s"), ("groundhook", GROUND, "^")):
            a.loglog(f_pts, sims[name][:, col], mk, color=c, ms=8, mec="#0f172a", label=f"{name}, simulated")
        for f, lab in ((fb, "bounce"), (fh, "wheel hop")):
            a.axvline(f, color=TEXT_MUTED, ls=":", lw=1)
            a.text(f * 1.05, 0.95, lab, transform=a.get_xaxis_transform(), color=TEXT_MUTED, fontsize=8, va="top")
        a.legend(loc="lower center", fontsize=8)
        style_axis(a, key.replace("_", " ") + " per road amplitude", "frequency [Hz]", ylab)

    prof = ISO8608Profile("C", seed=1)
    n = np.logspace(-2, np.log10(prof.n[-1]), 200)
    for cls in "ABCDE":
        ax[2].loglog(n, ISO8608_CLASSES[cls] * (n / 0.1) ** -2, color=TEXT_MUTED, lw=1, ls="--" if cls != "C" else "-")
        ax[2].text(n[-1] * 1.08, ISO8608_CLASSES[cls] * (n[-1] / 0.1) ** -2, cls, color=TEXT_MUTED, fontsize=9, va="center")
    x = np.arange(0.0, 4000.0, 0.05)
    nn, pxx = welch(prof.height(x), fs=20.0, nperseg=2**14)
    band = (nn >= prof.n[0]) & (nn <= prof.n[-1])                  # generated frequency range only
    ax[2].loglog(nn[band], pxx[band], color=SKY, lw=1.2, label="generated class C profile (Welch)")
    ax[2].set_xlim(0.01, 4.0)
    ax[2].set_ylim(1e-9, 1e-1)
    ax[2].legend(loc="lower left", fontsize=8)
    style_axis(ax[2], "ISO 8608 road classes A-E", "spatial frequency [cycles/m]", "G_d [m^3]")
    fig.suptitle("Quarter car: analytical vs simulated frequency response, semi-active damping, ISO 8608 roads (synthetic parameters)")
    fig.tight_layout()
    fig.savefig(OUT / "vertical_dynamics.png", dpi=150, facecolor=fig.get_facecolor())
    plt.close(fig)
    res = {"modes_hz_zeta": {"bounce": [fb, zb], "wheel_hop": [fh, zh]}, "frequencies_hz": f_pts.tolist(),
           "simulated_gain": {k: v.tolist() for k, v in sims.items()},
           "analytical_passive": {k: np.abs(frequency_response(CFG, f_pts)[k]).tolist() for k in ("body_acceleration", "dynamic_tyre_load")}}
    (OUT / "vertical_dynamics.json").write_text(json.dumps(res, indent=2), encoding="utf-8")
    dev = np.max(np.abs(sims["passive"][:, 0] / np.abs(frequency_response(CFG, f_pts)["body_acceleration"]) - 1))
    print(f"bounce {fb:.2f} Hz (zeta {zb:.2f}), hop {fh:.2f} Hz (zeta {zh:.2f}); passive simulated vs analytical max deviation {dev:.2%}")


if __name__ == "__main__":
    main()
