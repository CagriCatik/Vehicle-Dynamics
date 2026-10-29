"""Parking and low-speed manoeuvring: rear-axle steering (4WS) against front steering only (2WS).

Scenarios (``vehicle_model.simulation.four_wheel_steering.parking``): reverse
perpendicular parking, U-turn on a narrow street, pulling out of a tight parallel
bay, 90 deg turn between narrow alleys. Each GIF shows the plan view of both
vehicles and, below, the road-wheel angles over time with a cursor: the rear axle
steers against the front (counter-phase) at walking pace.

Outputs (output/four_wheel_steering/parking/):
    <scenario>.gif      plan view + steering panel
    summary.md          space need per scenario (swept extent, clearances, gap needed)

Run:
    python examples/parking_4ws_demo.py [scenario ...]     (about 90 s per scenario)
"""

from __future__ import annotations

import sys
from pathlib import Path

from vehicle_model.simulation.four_wheel_steering.parking import PARKING_SCENARIOS, render_parking
from vehicle_model.vehicle_models import SevenDOFModel

OUT = Path("output/four_wheel_steering/parking")


def main(names: list[str]) -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    md = ["# Parking and low-speed manoeuvring: 4WS vs 2WS", "",
          "Synthetic passenger car (configs/vehicles/seven_dof_passenger.yaml), dry asphalt, driver phases end on events "
          "(heading, lateral offset, distance), so both variants reach the same target pose. Body 4.50 x 1.80 m.", ""]
    for name in names:
        sc = PARKING_SCENARIOS[name]
        s = render_parking(name, SevenDOFModel.from_yaml, OUT)
        md += [f"## {sc.title}", "", sc.description, "",
               "| variant | duration [s] | final heading [deg] | max rear steer [deg] | extent x [m] | extent y [m] | extras |",
               "|---|---|---|---|---|---|---|"]
        for label, d in s.items():
            extras = []
            if "required_gap_ahead_m" in d:
                extras.append(f"gap needed ahead {d['required_gap_ahead_m']:.2f} m")
            if "clearances_m" in d:
                extras.append("clearances " + " / ".join(f"{c:.2f}" for c in d["clearances_m"]) + " m")
            md.append(f"| {label} | {d['duration_s']:.1f} | {d['final_heading_deg']:.1f} | {d['max_rear_steer_deg']:.1f} "
                      f"| {d['extent_x']:.2f} | {d['extent_y']:.2f} | {'; '.join(extras) or '–'} |")
        md.append("")
        print(f"{name}: done")
    (OUT / "summary.md").write_text("\n".join(md) + "\n", encoding="utf-8")
    print("\n".join(md))


if __name__ == "__main__":
    main(sys.argv[1:] or list(PARKING_SCENARIOS))
