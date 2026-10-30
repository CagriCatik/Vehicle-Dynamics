"""Plan V4.0 M16: actuation trace requested -> allocated -> issued -> realised on split-mu braking.

Reads the histories of ``configs/experiments/v4_m16_split_mu_actuators.yaml`` (run it first) and draws, for the
yaw-rate PI with ideal and with dynamic actuators: the yaw moment at the four stages, issued against realised
wheel torque (actuator lag), the realised longitudinal tyre forces per wheel, yaw rate and recovered regen power.

Run:
    python -m vehicle_model.experiments run configs/experiments/v4_m16_split_mu_actuators.yaml
    python examples/v4_m16_actuation_trace.py
Out: output/v4/m16/split_mu_actuators/actuation_trace.png, metadata.json
"""

from __future__ import annotations

import csv
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from vehicle_model.experiments.types import provenance
from vehicle_model.visualization.theme import TEXT_MUTED, apply_dark_theme, style_axis

OUT = Path("output/v4/m16/split_mu_actuators")
CFG = "configs/experiments/v4_m16_split_mu_actuators.yaml"
C = ("#3987e5", "#d95926", "#199e70", "#c98500")


def load(name: str) -> dict:
    rows = list(csv.DictReader(open(OUT / "histories" / name, encoding="utf-8")))
    out = {}
    for k in rows[0]:
        try:
            out[k] = np.array([float(r[k]) for r in rows])
        except ValueError:
            out[k] = np.array([r[k] for r in rows])
    return out


def main() -> None:
    ideal, dyn = load("base__PI__TV.csv"), load("base__PI__TV___actuator_dynamics.csv")
    apply_dark_theme()
    fig, ax = plt.subplots(2, 3, figsize=(17, 8.5))
    win = lambda h: h["t"] <= 0.6
    for a, h, title in ((ax[0, 0], ideal, "ideal actuators"), (ax[0, 1], dyn, "dynamic actuators")):
        w = win(h)
        for k, lab, col, ls in (("Mz_requested", "requested (law)", C[0], "-"), ("Mz_alloc_pre_slip", "allocated (before ABS/TCS)", C[2], "--"),
                                ("Mz_issued", "issued (controller output)", C[1], "-"), ("Mz_realised", "realised (plant input)", C[3], ":")):
            a.plot(h["t"][w], h[k][w], color=col, ls=ls, lw=1.6, label=lab)
        style_axis(a, f"PI yaw moment chain, {title} (first 0.6 s)", "t [s]", "M_z [N m]")
    ax[0, 0].legend(fontsize=8)
    w = win(dyn)
    ax[0, 2].plot(dyn["t"][w], dyn["T_issued_abs"][w] / 1e3, color=C[1], lw=1.6, label="issued Σ|T|")
    ax[0, 2].plot(dyn["t"][w], dyn["T_realised_abs"][w] / 1e3, color=C[3], lw=1.6, ls=":", label="realised Σ|T|")
    ax[0, 2].legend(fontsize=8)
    style_axis(ax[0, 2], "wheel torque, dynamic actuators: hydraulic build-up", "t [s]", "kN m")
    for i, wname in enumerate(("FL", "FR", "RL", "RR")):
        ax[1, 0].plot(dyn["t"], dyn[f"Fx_{wname}"] / 1e3, color=C[i], lw=1.3, label=wname)
    ax[1, 0].legend(fontsize=8)
    style_axis(ax[1, 0], "realised longitudinal tyre forces (dynamic; left mu 1.0, right mu 0.2)", "t [s]", "F_x [kN]")
    for h, lab, col in ((ideal, "ideal", C[0]), (dyn, "dynamic", C[1])):
        ax[1, 1].plot(h["t"], np.degrees(h["r"]), color=col, lw=1.4, label=lab)
    ax[1, 1].legend(fontsize=8)
    style_axis(ax[1, 1], "yaw rate", "t [s]", "r [deg/s]")
    ax[1, 2].plot(dyn["t"], dyn["regen_power_w"] / 1e3, color=C[2], lw=1.4)
    style_axis(ax[1, 2], "recovered mechanical regen power at the wheels (dynamic)", "t [s]", "kW")
    for a in ax.flat:
        a.axhline(0.0, color=TEXT_MUTED, lw=0.6)
    fig.suptitle("M16: actuation trace on split-mu braking, 90 km/h, yaw-rate PI (synthetic parameters, virtual 7-DOF model)")
    fig.tight_layout()
    fig.savefig(OUT / "actuation_trace.png", dpi=110)
    plt.close(fig)
    res = json.loads((OUT / "results.json").read_text(encoding="utf-8"))
    meta = {"milestone": "M16", "experiment": res["experiment"]["name"], "config": CFG, "seed": res["experiment"]["seed"],
            "model": "SevenDOFModel (7-DOF two-track), MF tyres (synthetic)", "provenance": provenance(),
            "experiment_provenance": res["provenance"],
            "controllers": [r["controller"] for r in res["results"]],
            "kpis": {r["controller"]: {k: v for k, v in r["kpis"].items() if not isinstance(v, dict)} for r in res["results"]}}
    (OUT / "metadata.json").write_text(json.dumps(meta, indent=2, default=float), encoding="utf-8")
    print(f"wrote {OUT / 'actuation_trace.png'}")


if __name__ == "__main__":
    main()
