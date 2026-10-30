"""Result figure of roll stability control (14-DOF) and active front steering (7-DOF).

Run the experiments first:
    python -m vehicle_model.experiments run configs/experiments/roll_stability_fishhook_evasive.yaml
    python -m vehicle_model.experiments run configs/experiments/active_front_steering_step_steer.yaml
    python -m vehicle_model.experiments run configs/experiments/active_front_steering_sine_with_dwell.yaml
    python examples/roll_stability_steering_figures.py
Out: output/roll_stability/rsc_afs.png, output/roll_stability/metadata.json
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

OUT = Path("output/roll_stability")
AFS = Path("output/active_front_steering")
C = ("#3987e5", "#d95926", "#199e70", "#c98500", "#d55181", "#8a63d2", "#f5f5f5", "#7d8590")


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
    fig, ax = plt.subplots(2, 3, figsize=(19, 10))
    R = OUT / "fishhook_evasive" / "histories"
    for i, (c, lab) in enumerate((("passive", "passive"), ("active_roll", "active roll only"), ("RSC", "RSC (brakes)"),
                                  ("RSC_+_active_roll", "RSC + active roll"))):
        h = hist(R / f"extreme_cog__fishhook_80__mu1.0__{c}.csv")
        ax[0, 0].plot(h["t"], h["ltr"], color=C[i], lw=1.4, label=lab)
        lifted = 4 - sum(h[f"contact_{w}"] for w in "FL FR RL RR".split())
        ax[0, 1].plot(h["t"], lifted + 0.04 * i, color=C[i], lw=1.4, label=lab)
        ax[0, 2].plot(h["t"], np.degrees(h["phi"]), color=C[i], lw=1.4, label=lab)
    ax[0, 0].axhline(1.0, color=TEXT_MUTED, ls=":", lw=1)
    ax[0, 0].legend(fontsize=7)
    style_axis(ax[0, 0], "14-DOF fishhook 80 km/h, 8 deg, h_cg 1.1 m: LTR", "t [s]", "-")
    style_axis(ax[0, 1], "wheels without contact (offset per curve for visibility)", "t [s]", "count")
    style_axis(ax[0, 2], "roll angle (model trusted up to 15 deg)", "t [s]", "deg")
    S = AFS / "step_steer" / "histories"
    W = AFS / "sine_with_dwell" / "histories"
    runs = (("base__passive.csv", "2WS passive"), ("base__GS-LQR__TV.csv", "TV"), ("base__GS-LQR__4WS_TV.csv", "4WS+TV"),
            ("base__GS-LQR__AFS.csv", "AFS"), ("base__GS-LQR__AFS___actuator_dynamics.csv", "AFS, steering actuator"),
            ("base__GS-LQR__AFS___AFS_failed.csv", "AFS failed"))
    for i, (f, lab) in enumerate(runs):
        h = hist(S / f)
        ax[1, 0].plot(h["t"], np.degrees(h["r"]), color=C[i], lw=1.3, label=lab)
        hw = hist(W / f)
        ax[1, 1].plot(hw["t"], np.degrees(hw["beta"]), color=C[i], lw=1.3, label=lab)
        ax[1, 2].plot(hw["t"], np.degrees(hw["steer_f"]), color=C[i], lw=1.3, label=lab)
    ax[1, 0].plot(h["t"], np.degrees(h["r_ref"]), color=TEXT_MUTED, ls=":", lw=1, label="reference")
    ax[1, 0].set_xlim(0, 1.5)
    ax[1, 0].legend(fontsize=7)
    style_axis(ax[1, 0], "step steer 80 km/h, 2 deg: yaw rate", "t [s]", "deg/s")
    ax[1, 1].set_ylim(-15, 15)
    style_axis(ax[1, 1], "sine with dwell 80 km/h, 6 deg: sideslip (passive, AFS failed spin)", "t [s]", "deg")
    style_axis(ax[1, 2], "same: realised front road-wheel angle (driver 6 deg)", "t [s]", "deg")
    fig.suptitle("roll stability control (14-DOF) and active front steering (7-DOF); synthetic parameters, virtual models")
    fig.tight_layout()
    fig.savefig(OUT / "rsc_afs.png", dpi=110)
    plt.close(fig)
    meta = {"topic": "roll stability and active steering", "provenance": provenance(),
            "experiments": {str(d): json.loads((d / "results.json").read_text())["provenance"]
                            for d in (OUT / "fishhook_evasive", AFS / "step_steer", AFS / "sine_with_dwell")},
            "configs": [str(p) for p in [*Path("configs/experiments").glob("roll_stability_*.yaml"), *Path("configs/experiments").glob("active_front_steering_*.yaml")]]}
    (OUT / "metadata.json").write_text(json.dumps(meta, indent=2), encoding="utf-8")
    print(f"wrote {OUT / 'rsc_afs.png'}")


if __name__ == "__main__":
    main()
