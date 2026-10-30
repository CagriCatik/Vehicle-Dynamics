"""Plan V4.0 M17: result figure of the brake functions (run the v4_m17_* experiments first).

    python -m vehicle_model.experiments run configs/experiments/v4_m17_straight_braking.yaml   (and the others)
    python examples/v4_m17_braking_figures.py
Out: output/v4/m17/braking_functions.png, output/v4/m17/metadata.json
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

OUT = Path("output/v4/m17")
C = ("#3987e5", "#d95926", "#199e70", "#c98500", "#d55181", "#8a63d2", "#7d8590")


def hist(path):
    rows = list(csv.DictReader(open(path, encoding="utf-8")))
    return {k: np.array([float(r[k]) if r[k] not in ("", "nan") else np.nan for r in rows]) for k in rows[0] if k != "status"}


def kpis(name):
    return list(csv.DictReader(open(OUT / name / "kpis.csv", encoding="utf-8")))


def main() -> None:
    apply_dark_theme()
    fig, ax = plt.subplots(2, 3, figsize=(19, 10))
    # (a) straight braking: stopping distance per friction and strategy
    rows = kpis("straight_braking")
    ctrls = list(dict.fromkeys(r["controller"] for r in rows))
    mus = list(dict.fromkeys(r["point"] for r in rows))
    w = 0.8 / len(ctrls)
    for i, c in enumerate(ctrls):
        vals = [float(next(r for r in rows if r["controller"] == c and r["point"] == m)["stopping_distance"]) for m in mus]
        ax[0, 0].bar(np.arange(len(mus)) + (i - len(ctrls) / 2 + 0.5) * w, vals, w * 0.92, color=C[i % len(C)], label=c)
    ax[0, 0].set_xticks(range(len(mus)), [m.replace("road.mu=", "mu ") for m in mus])
    ax[0, 0].legend(fontsize=7)
    style_axis(ax[0, 0], "straight braking 90 km/h: stopping distance (A: ideal info, C: sensors only)", None, "m")
    # (b) split-mu: stability against deceleration
    for i, r in enumerate(kpis("split_mu")):
        ax[0, 1].plot(float(r["stopping_distance"]), abs(float(r["final_heading_deg"])), "o", ms=10, color=C[i % len(C)],
                      label=r["controller"])
    ax[0, 1].set_yscale("symlog", linthresh=1.0)
    ax[0, 1].legend(fontsize=7)
    style_axis(ax[0, 1], "split-mu 1.0/0.2: heading change vs stopping distance", "stopping distance [m]", "|heading| [deg]")
    # (c) braking in a turn: sideslip
    for i, (f, lab) in enumerate((("brake_functions.csv", "no yaw control"), ("brake_functions__CBC.csv", "CBC only"),
                                  ("PI__TV.csv", "PI"), ("PI__TV___CBC.csv", "PI + CBC"))):
        h = hist(OUT / "brake_in_turn" / "histories" / f"base__{f}")
        ax[0, 2].plot(h["t"], np.degrees(h["beta"]), color=C[i], lw=1.5, label=lab)
    ax[0, 2].axvline(2.0, color=TEXT_MUTED, ls=":", lw=1)
    ax[0, 2].legend(fontsize=7)
    style_axis(ax[0, 2], "braking in a turn, mu 1.0 (80 km/h, 3 deg, pedal 0.5 from 2 s): sideslip", "t [s]", "beta [deg]")
    # (d) MSR: coast on mu 0.2 at 80 km/h
    base = OUT / "regen" / "histories" / "manoeuvre.speed_kph_80__brake_functions__actuator_dynamics___regen_{}__coast_300_N_m.csv"
    for i, pol in enumerate(("cut_on_abs", "modulated")):
        h = hist(str(base).format(pol))
        ax[1, 0].plot(h["t"], h["max_abs_slip"], color=C[i], lw=1.4, label=f"max |kappa|, {pol}")
        ax[1, 1].plot(h["t"], h["regen_power_w"] / 1e3, color=C[i], lw=1.4, label=pol)
    ax[1, 0].legend(fontsize=7)
    style_axis(ax[1, 0], "coast with 300 N m drag per wheel, mu 0.2, 80 km/h: wheel slip", "t [s]", "-")
    ax[1, 1].legend(fontsize=7)
    style_axis(ax[1, 1], "same: recovered mechanical regen power", "t [s]", "kW")
    # (e) regen braking onto a friction drop
    for i, pol in enumerate(("cut_on_abs", "modulated")):
        h = hist(OUT / "regen_braking" / "histories" / f"base__brake_functions__actuator_dynamics___IBC__regen_{pol}.csv")
        ax[1, 2].plot(h["t"], h["speed"] * 3.6, color=C[i], lw=1.5, label=f"speed, {pol}")
        ax[1, 2].plot(h["t"], h["abs_active_n"] * 10, color=C[i], lw=0.8, ls=":", label=f"ABS wheels x10, {pol}")
    ax[1, 2].legend(fontsize=7)
    style_axis(ax[1, 2], "regen braking (pedal 0.25) onto mu 1.0 -> 0.2 at 20 m", "t [s]", "km/h")
    fig.suptitle("M17 brake functions: EBD, IBC, CBC, regenerative slip control (synthetic parameters, virtual 7-DOF model)")
    fig.tight_layout()
    fig.savefig(OUT / "braking_functions.png", dpi=110)
    plt.close(fig)
    meta = {"milestone": "M17", "provenance": provenance(),
            "experiments": {d: json.loads((OUT / d / "results.json").read_text())["provenance"]
                            for d in ("straight_braking", "split_mu", "mu_step", "brake_in_turn", "regen", "regen_braking")},
            "configs": [str(p) for p in sorted(Path("configs/experiments").glob("v4_m17_*.yaml"))],
            "parameters": "configs/braking/brake_functions.yaml"}
    (OUT / "metadata.json").write_text(json.dumps(meta, indent=2), encoding="utf-8")
    print(f"wrote {OUT / 'braking_functions.png'}")


if __name__ == "__main__":
    main()
