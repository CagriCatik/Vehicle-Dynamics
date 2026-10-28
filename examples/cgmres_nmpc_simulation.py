"""Closed-loop Nonlinear Model Predictive Control (NMPC) path tracking simulation.

Tracks a sinusoidal slalom path using the C/GMRES formulation with calibrated
Ackermann rear-axle steering lookup table (LUT).

Run:
    python examples/cgmres_nmpc_simulation.py
    python examples/cgmres_nmpc_simulation.py --headless
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np

from vehicle_model.controllers.nmpc import (
    KinematicBicycleSystem,
    NMPCConfig,
    NMPCControllerCGMRES,
    get_ref_state,
)
from vehicle_model.visualization.nmpc_dashboard import RealTimeDashboard
from vehicle_model.visualization.plan_view import VehicleParams, draw_vehicle_trajectory
from vehicle_model.visualization.theme import (
    apply_dark_theme,
    style_axis,
    BG_COLOR,
    PANEL_BG,
    TEXT_COLOR,
    TEXT_MUTED,
    GRID_COLOR,
    ACCENT_CYAN,
    ACCENT_EMERALD,
    ACCENT_AMBER,
    ACCENT_ROSE,
)


def run_simulation(headless: bool = False, n_steps: int = 120, output_dir: str = "output"):
    config = NMPCConfig.from_yaml()
    dt = 0.05

    plant = KinematicBicycleSystem(
        init_x=0.0,
        init_y=0.0,
        init_yaw=0.0,
        init_v=config.target_v,
        wheelbase=config.wheelbase,
    )
    controller = NMPCControllerCGMRES(init_x=0.0, config=config)

    if not headless:
        print("Launching interactive NMPC RealTimeDashboard...")
        dashboard = RealTimeDashboard(
            plant=plant,
            controller=controller,
            dt=dt,
            max_iter=n_steps,
            init_x=0.0,
        )
        dashboard.run()
        return

    print(f"Running headless NMPC simulation for {n_steps} steps...")
    rows = []
    for i in range(n_steps):
        t = (i + 1) * dt
        u1s, u2s = controller.calc_input(plant.x, plant.y, plant.yaw, plant.v, t)
        plant.update_state(float(u1s[0]), float(u2s[0]), dt=dt)
        _, y_ref, yaw_ref, _ = get_ref_state(
            t=0.0,
            init_x=0.0,
            vehicle_x=plant.x,
            target_v=config.target_v,
            amplitude=config.amplitude,
            wavelength=config.wavelength,
        )
        rows.append({
            "t": t,
            "x": plant.x,
            "y": plant.y,
            "y_ref": y_ref,
            "ey": plant.y - y_ref,
            "yaw": plant.yaw,
            "yaw_ref": yaw_ref,
            "v": plant.v,
            "steer_f": float(u2s[0]),
            "steer_r": plant.history_steer_r[-1],
            "accel": float(u1s[0]),
        })

    out_p = Path(output_dir)
    out_p.mkdir(parents=True, exist_ok=True)
    apply_dark_theme()
    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=False, facecolor=BG_COLOR)
    for ax in axes:
        style_axis(ax)

    # 1. Trajectory vs Reference with 2D planar vehicle footprints
    x_ref = np.linspace(0, plant.x + 1.0, 300)
    y_ref_curve = [
        get_ref_state(0.0, 0.0, vehicle_x=xi, target_v=config.target_v, amplitude=config.amplitude, wavelength=config.wavelength)[1]
        for xi in x_ref
    ]
    axes[0].plot(x_ref, y_ref_curve, color=TEXT_MUTED, ls="--", label="Reference Path", lw=1.5)

    v_params = VehicleParams(
        wheelbase=config.wheelbase,
        width=0.36,
        wheel_track=0.30,
        tire_radius=0.07,
        tire_width=0.04,
        rear_overhang=0.12,
        front_overhang=0.12,
    )
    draw_vehicle_trajectory(
        ax=axes[0],
        X=np.array(plant.history_x),
        Y=np.array(plant.history_y),
        psi=np.array(plant.history_yaw),
        steer_f=np.array(plant.history_steer_f),
        steer_r=np.array(plant.history_steer_r),
        params=v_params,
        n_snapshots=8,
        color=ACCENT_CYAN,
        path_style="-",
        label="NMPC Tracked (2D Vehicle)",
        alpha=0.85,
    )
    axes[0].set_xlabel("X [m]", color=TEXT_COLOR)
    axes[0].set_ylabel("Y [m]", color=TEXT_COLOR)
    axes[0].set_aspect("equal", adjustable="datalim")
    axes[0].grid(True, linestyle="--", alpha=0.3, color=GRID_COLOR)
    axes[0].legend(loc="upper left", facecolor=PANEL_BG, edgecolor=GRID_COLOR, labelcolor=TEXT_COLOR)
    axes[0].set_title("NMPC Slalom Trajectory Tracking (Plan View with 2D Vehicle & LUT Rear Steer)", fontweight="bold", color=TEXT_COLOR)

    # 2. Steering Angles
    t_hist = np.arange(len(plant.history_steer_f)) * dt
    axes[1].plot(t_hist, np.degrees(plant.history_steer_f), color=ACCENT_CYAN, lw=1.8, label="Front Steer [deg]")
    axes[1].plot(t_hist, np.degrees(plant.history_steer_r), color=ACCENT_AMBER, lw=1.8, label="Rear Steer (LUT) [deg]")
    axes[1].set_xlabel("Time [s]", color=TEXT_COLOR)
    axes[1].set_ylabel("Steer Angle [deg]", color=TEXT_COLOR)
    axes[1].grid(True, alpha=0.3, color=GRID_COLOR)
    axes[1].legend(facecolor=PANEL_BG, edgecolor=GRID_COLOR, labelcolor=TEXT_COLOR)

    # 3. Speed & Tracking Error
    axes[2].plot([r["t"] for r in rows], [r["ey"] for r in rows], color=ACCENT_EMERALD, lw=1.8, label="Lateral Error e_y [m]")
    axes[2].plot(t_hist, plant.history_v, color="#e879f9", ls="--", lw=1.8, label="Vehicle Speed v [m/s]")
    axes[2].axhline(config.target_v, color=TEXT_MUTED, ls=":", label="Target Speed")
    axes[2].set_xlabel("Time [s]", color=TEXT_COLOR)
    axes[2].set_ylabel("Error / Speed", color=TEXT_COLOR)
    axes[2].grid(True, alpha=0.3, color=GRID_COLOR)
    axes[2].legend(facecolor=PANEL_BG, edgecolor=GRID_COLOR, labelcolor=TEXT_COLOR)

    plt.tight_layout()
    save_path = out_p / "cgmres_nmpc_simulation.png"
    fig.savefig(save_path, dpi=150, facecolor=fig.get_facecolor(), edgecolor="none")
    plt.close(fig)
    print(f"Saved headless simulation summary plot to {save_path}")


def main():
    parser = argparse.ArgumentParser(description="CGMRES-NMPC Path Tracking Simulation")
    parser.add_argument("--headless", action="store_true", help="Run without opening GUI and save plot")
    parser.add_argument("--steps", type=int, default=120, help="Number of simulation steps")
    args = parser.parse_args()
    run_simulation(headless=args.headless, n_steps=args.steps)


if __name__ == "__main__":
    main()
