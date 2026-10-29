"""4WS split-mu braking across actuator configurations (Plan.md V3.0 M10 section 5.5).

Six configurations brake from 100 km/h on split friction (1.0 / 0.2) with the
same pedal and the steering held straight: partial (0.3) and full (1.0) braking.
Which ones stay straight, and why the others spin, is explained in
``vehicle_model.simulation.four_wheel_steering.split_mu_matrix``.

Outputs (output/split_mu_matrix/):
    matrix.md                     KPI and yaw-moment table per pedal
    split_mu_matrix.png           paths and yaw angle at full braking

Run:
    python examples/split_mu_matrix_demo.py     (about a minute)
"""

from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from vehicle_model.simulation.four_wheel_steering.split_mu_matrix import matrix_table, run_split_mu_matrix
from vehicle_model.vehicle_models import SevenDOFModel
from vehicle_model.visualization.theme import TEXT_MUTED, apply_dark_theme, style_axis

OUT = Path("output/split_mu_matrix")
# validated categorical slots 1-5 (dark surface); passive is drawn in muted ink
COLORS = ("#3987e5", "#d95926", "#199e70", "#c98500", "#d55181")


def plant(road):
    return SevenDOFModel.from_yaml(road=road)


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    results = {pedal: run_split_mu_matrix(plant, pedal=pedal) for pedal in (0.3, 1.0)}
    md = ["# 4WS split-mu braking matrix (100 km/h, mu 1.0 / 0.2, steering held straight)\n",
          "Yaw moments: peak in the first 0.3 s, split into longitudinal (brake) and lateral (steering) tyre forces.\n"]
    for pedal, rows in results.items():
        md += [f"\n## Brake pedal {pedal:g}\n", matrix_table(rows)]
    (OUT / "matrix.md").write_text("\n".join(md) + "\n", encoding="utf-8")

    apply_dark_theme()
    fig, (ax_path, ax_psi) = plt.subplots(1, 2, figsize=(16, 6))
    c_iter = iter(COLORS)
    for r in results[1.0]:
        h = r.result.history
        passive = r.configuration == "passive"
        color = TEXT_MUTED if passive else next(c_iter)
        style = {"color": color, "lw": 1.2 if passive else 1.8, "ls": "--" if passive else "-",
                 "label": f"{r.configuration} ({'stable' if r.stable else 'spins'})"}
        ax_path.plot(h["X"], h["Y"], **style)
        ax_psi.plot(h["t"], np.degrees(h["psi"]), **style)
    ax_path.set_aspect("equal", adjustable="datalim")
    style_axis(ax_path, "Path at full braking (high friction on the left)", "X [m]", "Y [m]")
    style_axis(ax_psi, "Yaw angle at full braking (a run ends when v_x < 1 m/s: the vehicle is sideways)",
               "time [s]", "yaw angle [deg]")
    ax_psi.legend(loc="upper left", fontsize=8)
    fig.suptitle("Split-mu braking: only the allocation, which reduces brake torque on the high-friction side, stays straight")
    fig.tight_layout()
    fig.savefig(OUT / "split_mu_matrix.png", dpi=150, facecolor=fig.get_facecolor())
    plt.close(fig)
    print("\n".join(md))


if __name__ == "__main__":
    main()
