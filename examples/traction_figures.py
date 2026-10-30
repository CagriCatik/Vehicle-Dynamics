"""Result figure of the traction functions (run the v4_m18_* experiments first).

    python -m vehicle_model.experiments run configs/experiments/traction_split_mu_launch.yaml   (and the others)
    python examples/traction_figures.py
Out: output/traction_control/traction_functions.png, output/traction_control/metadata.json
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
from vehicle_model.visualization.theme import apply_dark_theme, style_axis

OUT = Path("output/traction_control")
C = ("#3987e5", "#d95926", "#199e70", "#c98500", "#d55181", "#8a63d2", "#f5f5f5", "#7d8590")


def hist(path):
    rows = list(csv.DictReader(open(path, encoding="utf-8")))
    return {k: np.array([float(r[k]) if r[k] not in ("", "nan") else np.nan for r in rows]) for k in rows[0] if k != "status"}


def kpis(name):
    return list(csv.DictReader(open(OUT / name / "kpis.csv", encoding="utf-8")))


def main() -> None:
    apply_dark_theme()
    fig, ax = plt.subplots(2, 3, figsize=(19, 10))
    H = OUT / "split_mu_launch" / "histories"
    runs = [("base__passive__actuator_dynamics.csv", "passive (equal torques, no TCS)"),
            ("base__no_yaw_control__actuator_dynamics___open.csv", "open coupling + TCS"),
            ("base__no_yaw_control__actuator_dynamics___elsd_300_N_m.csv", "eLSD 300 N m"),
            ("base__no_yaw_control__actuator_dynamics.csv", "free 4-motor"),
            ("base__PI__TV___actuator_dynamics.csv", "TV only (yaw-rate PI)"),
            ("base__PI__TV___actuator_dynamics___elsd_300_N_m__envelope__ITC_band_0.5_deg_s.csv", "ITC, band 0.5 deg/s"),
            ("base__PI__TV___actuator_dynamics___elsd_300_N_m__envelope__ITC_band_2.0_deg_s.csv", "ITC, band 2 deg/s")]
    for i, (f, lab) in enumerate(runs):
        h = hist(H / f)
        ax[0, 0].plot(h["t"], h["speed"] * 3.6, color=C[i], lw=1.5, label=lab)
        ax[0, 1].plot(h["t"], np.degrees(h["r"]), color=C[i], lw=1.3, label=lab)
    # one colour per variant in every panel; information mode C hollow, open without TCS grey
    colour = {"passive": C[0], "open": C[1], "elsd": C[2], "free": C[3], "pi": C[4], "itc0.5": C[5], "itc2": C[6]}

    def key(name):
        n = name.replace(" [actuator dynamics]", "")
        if n.startswith("passive"):
            return "passive"
        if "ITC band 0.5" in n:
            return "itc0.5"
        if "ITC band" in n:
            return "itc2"
        if n.startswith("PI"):
            return "pi"
        if "elsd" in n:
            return "elsd"
        if "open" in n:
            return "open"
        return "free"
    ax[0, 0].legend(fontsize=7)
    style_axis(ax[0, 0], "split-mu launch (left 1.0, right 0.2), full pedal: speed", "t [s]", "km/h")
    ax[0, 1].set_ylim(-8, 8)
    style_axis(ax[0, 1], "same: yaw rate (steering held straight; passive leaves the plot)", "t [s]", "r [deg/s]")
    for r in kpis("split_mu_launch"):
        col = "#7d8590" if "no TCS" in r["controller"] else colour[key(r["controller"])]
        hollow = "sensors only" in r["controller"]
        ax[0, 2].plot(abs(float(r["final_heading_deg"])) + 1e-3, float(r["speed_gain_kph"]), "o", ms=10, color=col,
                      mfc="none" if hollow else col, mew=2, label=r["controller"].replace(" [actuator dynamics]", ""))
    ax[0, 2].set_xscale("log")
    ax[0, 2].legend(fontsize=6.5)
    style_axis(ax[0, 2], "traction against heading change (no driver; hollow: mode C)", "|heading change| [deg]", "speed gain [km/h]")
    h = hist(H / runs[-1][0])
    for i, w in enumerate(("FL", "FR", "RL", "RR")):
        ax[1, 0].plot(h["t"], h[f"Fx_{w}"] / 1e3, color=C[i], lw=1.3, label=w)
    ax[1, 0].legend(fontsize=7)
    style_axis(ax[1, 0], "ITC (band 2 deg/s): realised longitudinal tyre forces", "t [s]", "F_x [kN]")
    rows = kpis("drive_faults")
    labels = [r["controller"].split("[")[-1].rstrip("]").replace("elsd, envelope, ", "").replace(", ITC band 2.0 deg/s", "")
              .replace("ITC band 2.0 deg/s", "AWD") for r in rows]
    labels[-1] = "no yaw control, failed FL+RL"
    ax[1, 1].barh(range(len(rows)), [float(r["speed_gain_kph"]) for r in rows], color=C[0])
    for i, r in enumerate(rows):
        ax[1, 1].text(float(r["speed_gain_kph"]) + 1, i, f"heading {float(r['final_heading_deg']):.1f} deg", va="center", fontsize=8)
    ax[1, 1].set_yticks(range(len(rows)), labels, fontsize=8)
    style_axis(ax[1, 1], "launch on mu 0.5, ITC: drive layouts and motor failures", "speed gain in 6 s [km/h]", None)
    markers = iter("os^D")
    for r in kpis("corner_exit"):
        k = key(r["controller"])
        ax[1, 2].plot(float(r["max_sideslip_deg"]), float(r["speed_gain_kph"]), "o" if k != "itc2" else next(markers), ms=10,
                      color=colour[k], label=r["controller"].replace(" [actuator dynamics]", ""))
    ax[1, 2].set_xscale("log")
    ax[1, 2].legend(fontsize=6.5)
    style_axis(ax[1, 2], "corner exit, mu 0.5, 0.8 mu g, full pedal: a saturated limit case", "max sideslip [deg]", "speed gain [km/h]")
    fig.suptitle("traction: virtual eLSD, TCS, TV and ITC (synthetic parameters, virtual 7-DOF model, dynamic actuators)")
    fig.tight_layout()
    fig.savefig(OUT / "traction_functions.png", dpi=110)
    plt.close(fig)
    meta = {"topic": "traction control", "provenance": provenance(),
            "experiments": {d: json.loads((OUT / d / "results.json").read_text())["provenance"]
                            for d in ("split_mu_launch", "corner_exit", "mu_step_launch", "drive_faults")},
            "configs": [str(p) for p in sorted(Path("configs/experiments").glob("v4_m18_*.yaml"))]}
    (OUT / "metadata.json").write_text(json.dumps(meta, indent=2), encoding="utf-8")
    print(f"wrote {OUT / 'traction_functions.png'}")


if __name__ == "__main__":
    main()
