"""Headless smoke test for the CGMRES-NMPC controller."""

from __future__ import annotations

from pathlib import Path
import sys

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[2]
CGMRES_ROOT = PROJECT_ROOT / "python" / "cgmres_nmpc"
if str(CGMRES_ROOT) not in sys.path:
    sys.path.insert(0, str(CGMRES_ROOT))

from src.config import global_config
from src.controllers.cgmres import NMPCControllerCGMRES, get_ref_state
from src.models.vehicle import KinematicBicycleSystem


def run_controller_steps(n_steps: int = 40, print_every: int | None = None):
    cfg = global_config
    dt = cfg["simulation"]["dt"]
    plant = KinematicBicycleSystem(0.0, 0.0, 0.0, 0.5)
    ctrl = NMPCControllerCGMRES(init_x=0.0)
    rows = []

    for i in range(n_steps):
        t = (i + 1) * dt
        u1s, u2s = ctrl.calc_input(plant.x, plant.y, plant.yaw, plant.v, t)
        plant.update_state(u1s[0], u2s[0], dt)
        _, y_ref, _, _ = get_ref_state(0, 0.0, vehicle_x=plant.x)
        row = {
            "t": t,
            "x": plant.x,
            "y": plant.y,
            "y_ref": y_ref,
            "ey": plant.y - y_ref,
            "yaw": plant.yaw,
            "v": plant.v,
            "steer": u2s[0],
        }
        rows.append(row)
        if print_every and (i + 1) % print_every == 0:
            print(
                f"t={t:4.1f} x={plant.x:6.2f} y={plant.y:7.3f} "
                f"yr={y_ref:7.3f} ey={row['ey']:7.3f} "
                f"yaw={np.degrees(plant.yaw):6.1f}deg v={plant.v:5.3f} "
                f"usf={np.degrees(u2s[0]):6.1f}deg"
            )
    return rows


def test_cgmres_controller_smoke():
    rows = run_controller_steps(n_steps=5)
    assert len(rows) == 5
    assert np.isfinite([rows[-1]["x"], rows[-1]["y"], rows[-1]["yaw"], rows[-1]["v"]]).all()


if __name__ == "__main__":
    print(f"{'t':>5}  {'x':>6}  {'y':>7}  {'yr':>7}  {'ey':>7}  {'yaw':>7}  {'v':>6}  {'usf':>7}")
    print("-" * 65)
    run_controller_steps(n_steps=200, print_every=20)
