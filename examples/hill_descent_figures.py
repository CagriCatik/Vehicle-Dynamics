"""Result figure of the hill descent benchmark (7-DOF with road grade).

    python -m vehicle_model.experiments run configs/experiments/hill_descent_control.yaml
    python examples/hill_descent_figures.py
Out: output/hill_descent/benchmark.png, output/hill_descent/metadata.json
"""

from __future__ import annotations

import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from vehicle_model.experiments.types import provenance
from vehicle_model.visualization.run_animation import load_history
from vehicle_model.visualization.theme import TEXT_MUTED, apply_dark_theme, style_axis

OUT = Path("output/hill_descent")
H = OUT / "hdc" / "histories"
C = ("#3987e5", "#d95926", "#199e70", "#c98500")


def main() -> None:
    apply_dark_theme()
    fig, ax = plt.subplots(2, 2, figsize=(16, 9.5))
    for i, g in enumerate((5, 10, 20)):
        on = load_history(H / f"dry_{g}pct__HDC.csv")
        off = load_history(H / f"dry_{g}pct__HDC_off.csv")
        ax[0, 0].plot(on["t"], on["speed"] * 3.6, color=C[i], lw=1.5, label=f"HDC, {g} %")
        ax[0, 0].plot(off["t"], off["speed"] * 3.6, color=C[i], lw=1.0, ls="--", label=f"off, {g} %")
    ax[0, 0].axhline(10.0, color=TEXT_MUTED, lw=1.0, ls=":")
    style_axis(ax[0, 0], "dry: speed with and without HDC (set speed 10 km/h)", "t [s]", "km/h")
    ax[0, 0].legend(fontsize=8, ncol=2)
    for i, (s, lab) in enumerate((("dry", "dry mu 1.0"), ("wet", "wet 0.6"), ("snow", "snow 0.3"), ("ice", "ice 0.12"))):
        h = load_history(H / f"{s}_20pct__HDC.csv")
        ax[0, 1].plot(h["t"], h["speed"] * 3.6, color=C[i], lw=1.5, label=lab)
        inf = h["hdc_infeasible"] > 0
        if inf.any():
            ax[0, 1].fill_between(h["t"], 0, 40, where=inf, color=C[i], alpha=0.12, label=f"{lab}: infeasible reported")
    style_axis(ax[0, 1], "20 % downhill with HDC: surfaces (ice: mu < tan of the grade)", "t [s]", "km/h")
    ax[0, 1].set_ylim(0, 40)
    ax[0, 1].legend(fontsize=8)
    for i, (f, lab) in enumerate((("grade_change_dry__HDC.csv", "HDC"), ("grade_change_dry__HDC_+_actuator_dynamics.csv", "HDC + actuator dynamics"),
                                  ("grade_change_dry__HDC,_RR_brake_failed.csv", "HDC, RR brake failed"))):
        h = load_history(H / f)
        ax[1, 0].plot(h["t"], h["speed"] * 3.6, color=C[i], lw=1.4, label=lab)
    h = load_history(H / "grade_change_dry__HDC.csv")
    tw = ax[1, 0].twinx()
    tw.plot(h["t"], h["grade_pct"], color=TEXT_MUTED, lw=1.0, ls=":")
    tw.set_ylabel("grade [%] (dotted)")
    style_axis(ax[1, 0], "grade change 5 -> 20 -> 5 %, dry", "t [s]", "km/h")
    ax[1, 0].legend(fontsize=8)
    for i, (f, lab) in enumerate((("driver_brakes_10pct__HDC.csv", "driver brakes 8-11 s (pedal 0.1)"),
                                  ("driver_accelerates_10pct__HDC.csv", "driver accelerates 8-10 s (override)"),
                                  ("entry_30kph_20pct__HDC.csv", "entry at 30 km/h, 20 %"), ("split_mu_10pct__HDC.csv", "split-mu 10 %"))):
        h = load_history(H / f)
        ax[1, 1].plot(h["t"], h["speed"] * 3.6, color=C[i], lw=1.4, label=lab)
    style_axis(ax[1, 1], "named cases with HDC (10 km/h)", "t [s]", "km/h")
    ax[1, 1].legend(fontsize=8)
    fig.suptitle("hill descent control on graded roads (7-DOF, gravity in the plant), synthetic parameters")
    fig.tight_layout()
    fig.savefig(OUT / "hdc.png", dpi=110)
    json.dump({"figure": "hdc.png", "source": "configs/experiments/hill_descent_control.yaml", "provenance": provenance()},
              open(OUT / "metadata.json", "w"), indent=2)
    print(OUT / "hdc.png")


if __name__ == "__main__":
    main()
