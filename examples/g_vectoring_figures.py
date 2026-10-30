"""Result figure of the GVC-like load-transfer control (own simplified reference strategy).

    python -m vehicle_model.experiments run configs/experiments/g_vectoring_steer_ramp.yaml   (and the others)
    python examples/g_vectoring_figures.py
Out: output/g_vectoring/gvc.png, output/g_vectoring/metadata.json
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

OUT = Path("output/g_vectoring")
C = ("#3987e5", "#d95926", "#199e70", "#c98500")


def hist(path):
    rows = list(csv.DictReader(open(path, encoding="utf-8")))
    out = {}
    for k in rows[0]:
        try:
            out[k] = np.array([float(r[k]) for r in rows])
        except ValueError:
            out[k] = np.array([r[k] for r in rows])
    return out


def main() -> None:
    apply_dark_theme()
    fig, ax = plt.subplots(2, 3, figsize=(18, 9))
    runs = (("no_yaw_control.csv", "GVC off"), ("no_yaw_control__GVC.csv", "GVC (C_xy 0.15 s, 0.05 g)"),
            ("no_yaw_control__GVC_C_xy_0.3.csv", "GVC (C_xy 0.3 s, 0.1 g)"))
    for i, (f, lab) in enumerate(runs):
        h = hist(OUT / "steer_ramp" / "histories" / f"road.mu_1.0__{f}")
        fz = h["Fz_FL"] + h["Fz_FR"]
        ax[0, 0].plot(h["t"], h["ax"], color=C[i], lw=1.4, label=lab)
        ax[0, 1].plot(h["t"], fz - fz[np.searchsorted(h["t"], 0.5)], color=C[i], lw=1.4, label=lab)
        ax[0, 2].plot(h["t"], np.degrees(h["r"]), color=C[i], lw=1.4, label=lab)
        h5 = hist(OUT / "steer_ramp" / "histories" / f"road.mu_0.5__{f}")
        ax[1, 0].plot(h5["t"], np.degrees(h5["beta"]), color=C[i], lw=1.4, label=lab)
        hl = hist(OUT / "lane_change" / "histories" / f"base__{f}")
        ax[1, 1].plot(hl["t"], hl["ax"], color=C[i], lw=1.4, label=lab)
        ax[1, 2].plot(hl["t"], np.degrees(hl["r"]), color=C[i], lw=1.4, label=lab)
    ax[0, 0].legend(fontsize=8)
    style_axis(ax[0, 0], "steer ramp 80 km/h, 3 deg, mu 1: longitudinal acceleration", "t [s]", "m/s^2")
    style_axis(ax[0, 1], "front axle load change from the steer start", "t [s]", "N")
    style_axis(ax[0, 2], "yaw rate (hardly changed)", "t [s]", "deg/s")
    style_axis(ax[1, 0], "same ramp on mu 0.5 (beyond the limit): sideslip", "t [s]", "deg")
    style_axis(ax[1, 1], "lane change 80 km/h, 2.5 deg, 0.5 Hz: longitudinal acceleration", "t [s]", "m/s^2")
    style_axis(ax[1, 2], "same: yaw rate", "t [s]", "deg/s")
    fig.suptitle("GVC-like load-transfer control (own simplified reference strategy, not Mazda's algorithm); "
                 "7-DOF model, constant pedal, no yaw control")
    fig.tight_layout()
    fig.savefig(OUT / "gvc.png", dpi=110)
    plt.close(fig)
    meta = {"topic": "G-vectoring", "provenance": provenance(),
            "experiments": {d: json.loads((OUT / d / "results.json").read_text())["provenance"]
                            for d in ("steer_ramp", "lane_change", "lift_off", "emergency")},
            "configs": [str(p) for p in sorted(Path("configs/experiments").glob("v4_m20_*.yaml"))]}
    (OUT / "metadata.json").write_text(json.dumps(meta, indent=2), encoding="utf-8")
    print(f"wrote {OUT / 'gvc.png'}")


if __name__ == "__main__":
    main()
