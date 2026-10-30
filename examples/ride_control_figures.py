"""Result figure of the ride/pitch benchmark (14-DOF).

    python -m vehicle_model.experiments run configs/experiments/ride_preview_damping_active_pitch.yaml
    python examples/ride_control_figures.py
Out: output/ride_control/benchmark.png, output/ride_control/metadata.json
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
from vehicle_model.visualization.run_animation import load_history
from vehicle_model.visualization.theme import apply_dark_theme, style_axis

OUT = Path("output/ride_control")
C = ("#3987e5", "#d95926", "#199e70", "#c98500", "#9d5bd2")
CTRL = ("passive", "skyhook", "groundhook", "preview skyhook", "active pitch + skyhook")


def main() -> None:
    apply_dark_theme()
    rows = list(csv.DictReader(open(OUT / "ride" / "kpis.csv", encoding="utf-8")))
    k = {(r["scenario"], r["controller"]): r for r in rows}
    scen = list(dict.fromkeys(r["scenario"] for r in rows))
    fig, ax = plt.subplots(2, 2, figsize=(17, 10))
    w = 0.8 / len(CTRL)
    x = np.arange(len(scen))
    for (i, j), key, ttl, unit in (((0, 0), "body_accel_rms", "body vertical acceleration RMS (comfort)", "m/s^2"),
                                   ((0, 1), "pitch_rms_deg", "pitch RMS from the manoeuvre start", "deg"),
                                   ((1, 0), "tyre_load_variation", "tyre-load variation (std / static, handling)", "-")):
        for c, name in enumerate(CTRL):
            ax[i, j].bar(x + (c - (len(CTRL) - 1) / 2) * w, [float(k[(s, name)][key]) for s in scen], w * 0.9,
                         color=C[c], label=name)
        ax[i, j].set_xticks(x, scen, rotation=25, ha="right", fontsize=8)
        style_axis(ax[i, j], ttl, None, unit)
    ax[0, 0].legend(fontsize=8)
    H = OUT / "ride" / "histories"
    for c, name in enumerate(CTRL):
        h = load_history(H / f"brake_100_wavy__{name}.csv".replace(" ", "_"))
        ax[1, 1].plot(h["t"], np.degrees(h["theta"]), color=C[c], lw=1.3, label=name)
    style_axis(ax[1, 1], "braking 0.6 g from 100 km/h on ISO B: pitch (nose-down negative)", "t [s]", "deg")
    ax[1, 1].legend(fontsize=8)
    fig.suptitle("14-DOF ride and pitch benchmark, synthetic parameters (virtual model only)")
    fig.tight_layout()
    OUT.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUT / "ride.png", dpi=110)
    json.dump({"figure": "ride.png", "source": "configs/experiments/ride_preview_damping_active_pitch.yaml", "provenance": provenance()},
              open(OUT / "metadata.json", "w"), indent=2)
    print(OUT / "ride.png")


if __name__ == "__main__":
    main()
