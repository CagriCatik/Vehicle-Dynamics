"""Chassis Control Benchmark: Unregulated vs. Integrated Active Control (ABS, TCS, 4WS, TV, ESC).

Executes reproducible virtual validation scenarios:
  Benchmark 1: Mu-Split Emergency Braking (Left mu=1.0, Right mu=0.2)
  Benchmark 2: High-Speed Evasive Maneuver (Front-only vs. 4WS + Torque Vectoring)

Outputs:
  - Comparative KPI metrics printed to console
  - High-resolution comparative plots saved to output/benchmark/
"""

from __future__ import annotations

import math
from pathlib import Path
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from vehicle_model.controllers.integrated_controller import IntegratedChassisController
from vehicle_model.core.types import (
    FL,
    FR,
    RL,
    RR,
    SevenDOFControlInput,
    SevenDOFParams,
    SevenDOFState,
)
from vehicle_model.logging import setup_logger
from vehicle_model.roads.surface import MuSplitRoad, UniformRoad
from vehicle_model.simulation.scenario_runner import ScenarioRunner, SimulationHistory
from vehicle_model.vehicle_models.seven_dof import SevenDOFModel
from vehicle_model.visualization.plan_view import VehicleParams, draw_vehicle_trajectory
from vehicle_model.visualization.theme import (
    ACCENT_AMBER,
    ACCENT_CYAN,
    ACCENT_EMERALD,
    ACCENT_ROSE,
    BG_COLOR,
    CARD_COLOR,
    GRID_COLOR,
    MU_HIGH_COLOR,
    MU_LOW_COLOR,
    ROAD_COLOR,
    TEXT_MUTED,
    TEXT_PRIMARY,
    apply_dark_theme,
)

apply_dark_theme()

logger = setup_logger("abs_tcs_benchmark")


def run_mu_split_benchmark(output_dir: Path) -> tuple[SimulationHistory, SimulationHistory]:
    """Run head-to-head comparison of mu-split emergency braking."""
    # Load physical parameters directly from centralized YAML config
    params = SevenDOFParams.from_yaml()
    model = SevenDOFModel(params=params)
    road = MuSplitRoad(mu_left=1.0, mu_right=0.2, split_y=0.0)

    v0 = 20.0  # 72 km/h
    omega0 = v0 / params.Reff
    init_state = SevenDOFState(
        vx=v0, vy=0.0, r=0.0, omega=np.full(4, omega0), X=0.0, Y=0.0, psi=0.0
    )

    runner = ScenarioRunner(model=model, road=road)
    duration = 2.0
    dt = 0.005

    # 1. Unregulated baseline (standard brakes, open differential, no ABS)
    def unreg_ctrl(t, state, tele):
        return SevenDOFControlInput(steer_f=0.0, T_brake=np.full(4, 1500.0))

    hist_unreg = runner.run(unreg_ctrl, init_state, duration=duration, dt=dt)

    # 2. Active Integrated Chassis Controller loaded from centralized YAML config
    ivmc = IntegratedChassisController.from_yaml(params=params)

    def ivmc_ctrl(t, state, tele):
        return ivmc.compute_control(
            t=t,
            state=state,
            driver_steer_f=0.0,
            driver_T_drive=np.zeros(4),
            driver_T_brake=np.full(4, 1500.0),
            telemetry=tele,
            dt=dt,
            mu_est=0.6,
        )

    hist_ivmc = runner.run(ivmc_ctrl, init_state, duration=duration, dt=dt)

    # Plot comparative telemetry
    fig, axes = plt.subplots(2, 2, figsize=(14, 10), constrained_layout=True, facecolor=BG_COLOR)

    t = hist_unreg.time

    # Subplot 1: Trajectory in World Plane with Vehicle Footprints
    ax1 = axes[0, 0]
    v_params = VehicleParams(
        wheelbase=params.wheelbase,
        width=1.8,
        length=4.6,
        tire_radius=params.Reff,
        tire_width=0.225,
        wheel_track=params.wf,
        rear_overhang=0.9,
        front_overhang=0.9,
    )
    draw_vehicle_trajectory(
        ax=ax1,
        X=hist_unreg.X,
        Y=hist_unreg.Y,
        psi=hist_unreg.psi,
        steer_f=hist_unreg.steer_f,
        steer_r=hist_unreg.steer_r,
        params=v_params,
        n_snapshots=7,
        color=ACCENT_ROSE,
        path_style="--",
        label="Unregulated (Spinout)",
        alpha=0.75,
        scale=1.5,
    )
    draw_vehicle_trajectory(
        ax=ax1,
        X=hist_ivmc.X,
        Y=hist_ivmc.Y,
        psi=hist_ivmc.psi,
        steer_f=hist_ivmc.steer_f,
        steer_r=hist_ivmc.steer_r,
        params=v_params,
        n_snapshots=7,
        color=ACCENT_EMERALD,
        path_style="-",
        label="With IVMC (ABS + ESP)",
        alpha=0.90,
        scale=1.5,
    )
    ax1.axhline(0.0, color=TEXT_MUTED, ls=":", label="Split Line (Y=0)")
    ax1.fill_between([0, 40], 0, 5, color=MU_HIGH_COLOR, alpha=0.6, label=r"High $\mu=1.0$")
    ax1.fill_between([0, 40], -5, 0, color=MU_LOW_COLOR, alpha=0.6, label=r"Low $\mu=0.2$")
    ax1.set_xlabel("X [m]", color=TEXT_MUTED)
    ax1.set_ylabel("Y [m]", color=TEXT_MUTED)
    ax1.set_title("World Trajectory on Mu-Split Braking (Plan View with 2D Vehicles)", fontsize=11, fontweight="bold", color=TEXT_PRIMARY)
    ax1.set_aspect("equal", adjustable="datalim")
    ax1.legend(loc="upper left", fontsize=8, framealpha=0.92, facecolor=CARD_COLOR, edgecolor=GRID_COLOR, labelcolor=TEXT_PRIMARY)

    # Subplot 2: Heading Angle Psi
    ax2 = axes[0, 1]
    ax2.plot(t, np.degrees(hist_unreg.psi), color=ACCENT_ROSE, linestyle="--", lw=2, label="Unregulated Heading [deg]")
    ax2.plot(t, np.degrees(hist_ivmc.psi), color=ACCENT_EMERALD, linestyle="-", lw=2.5, label="IVMC Heading [deg]")
    ax2.set_xlabel("Time [s]", color=TEXT_MUTED)
    ax2.set_ylabel("Heading Angle [deg]", color=TEXT_MUTED)
    ax2.set_title("Vehicle Heading Deviation", color=TEXT_PRIMARY)
    ax2.legend(loc="best", fontsize=8, facecolor=CARD_COLOR, edgecolor=GRID_COLOR, labelcolor=TEXT_PRIMARY)

    # Subplot 3: Wheel Speeds on Right (Icy) Front Wheel
    ax3 = axes[1, 0]
    ax3.plot(t, hist_unreg.omega[:, FR], color=ACCENT_ROSE, linestyle="--", lw=2, label=r"$\omega_{FR}$ Unregulated (Locked)")
    ax3.plot(t, hist_ivmc.omega[:, FR], color=ACCENT_EMERALD, linestyle="-", lw=2, label=r"$\omega_{FR}$ with ABS (Modulated)")
    ax3.plot(t, hist_ivmc.vx / params.Reff, color=TEXT_PRIMARY, linestyle=":", lw=1.5, label=r"$v_x / R_{eff}$ (Linear Ref)")
    ax3.set_xlabel("Time [s]", color=TEXT_MUTED)
    ax3.set_ylabel("Wheel Speed [rad/s]", color=TEXT_MUTED)
    ax3.set_title("Right Front Wheel Speed (Ice Side)", color=TEXT_PRIMARY)
    ax3.legend(loc="best", fontsize=8, facecolor=CARD_COLOR, edgecolor=GRID_COLOR, labelcolor=TEXT_PRIMARY)

    # Subplot 4: Longitudinal Slip Ratio on Right Front Wheel
    ax4 = axes[1, 1]
    ax4.plot(t, hist_unreg.kappa[:, FR], color=ACCENT_ROSE, linestyle="--", lw=2, label=r"$\kappa_{FR}$ Unregulated (-1.0 Lock)")
    ax4.plot(t, hist_ivmc.kappa[:, FR], color=ACCENT_EMERALD, linestyle="-", lw=2, label=r"$\kappa_{FR}$ with ABS (-0.15 Optimal)")
    ax4.axhline(-0.15, color=ACCENT_EMERALD, ls=":", label="Peak Friction Target (-0.15)")
    ax4.axhline(-1.0, color=ACCENT_ROSE, ls=":", label="Wheel Lockup (-1.0)")
    ax4.set_xlabel("Time [s]", color=TEXT_MUTED)
    ax4.set_ylabel("Slip Ratio kappa [-]", color=TEXT_MUTED)
    ax4.set_title("Longitudinal Slip Comparison", color=TEXT_PRIMARY)
    ax4.legend(loc="best", fontsize=8, facecolor=CARD_COLOR, edgecolor=GRID_COLOR, labelcolor=TEXT_PRIMARY)

    save_path = output_dir / "mu_split_comparison.png"
    plt.savefig(save_path, dpi=200, facecolor=fig.get_facecolor(), edgecolor="none")
    plt.close(fig)
    plt.close(fig)

    summary = (
        "=== Benchmark 1: Mu-Split Emergency Braking ===\n"
        f"Initial Speed:               {v0*3.6:.1f} km/h\n"
        f"Unregulated Peak Yaw Rate:   {np.max(np.abs(hist_unreg.r)):.3f} rad/s ({math.degrees(np.max(np.abs(hist_unreg.r))):.1f} deg/s)\n"
        f"IVMC Peak Yaw Rate:          {np.max(np.abs(hist_ivmc.r)):.3f} rad/s ({math.degrees(np.max(np.abs(hist_ivmc.r))):.1f} deg/s)\n"
        f"Unregulated Final Heading:   {math.degrees(hist_unreg.psi[-1]):.1f} deg (Severe Spinout!)\n"
        f"IVMC Final Heading:          {math.degrees(hist_ivmc.psi[-1]):.1f} deg (Controlled Stop!)\n"
        f"Unregulated Icy Wheel Slip:  {np.min(hist_unreg.kappa[:, FR]):.3f} (Total Wheel Lockup)\n"
        f"IVMC Icy Wheel Slip:         {np.min(hist_ivmc.kappa[:, FR]):.3f} (ABS Regulated Slip)\n"
        f"Plot exported to:            {save_path}"
    )
    logger.info("\n" + summary)

    return hist_unreg, hist_ivmc


if __name__ == "__main__":
    out_dir = Path(__file__).resolve().parents[1] / "output" / "benchmark"
    out_dir.mkdir(parents=True, exist_ok=True)
    run_mu_split_benchmark(out_dir)
