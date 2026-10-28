"""Comparative benchmark: Passive vs 4WS vs IVMC during High-Speed Double Lane Change.

Simulates an ISO 3888-like severe obstacle-avoidance double lane change at 80 km/h (22.2 m/s)
on damp pavement (mu = 0.75) using the 7-DOF two-track vehicle dynamics model.
Evaluates standardized KPIs (Plan.md §16.1) and exports multi-panel comparative telemetry.
"""

from __future__ import annotations

import math
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np

from vehicle_model.controllers import (
    ControlTarget,
    IntegratedChassisController,
    PassiveBaselineController,
)
from vehicle_model.core.types import (
    FL,
    FR,
    RL,
    RR,
    SevenDOFControlInput,
    SevenDOFParams,
    SevenDOFState,
    WheelTelemetry,
)
from vehicle_model.logging import setup_logger
from vehicle_model.roads.surface import UniformRoad
from vehicle_model.simulation import (
    EngineeringKPIs,
    ScenarioRunner,
    SimulationHistory,
    compute_kpis,
    format_kpi_table,
)
from vehicle_model.steering.four_wheel_steering import (
    FourWheelSteering,
    SteeringConfig,
    SteeringMode,
)
from vehicle_model.vehicle_models.seven_dof import SevenDOFModel
from vehicle_model.visualization.plan_view import VehicleParams, draw_vehicle_trajectory
from vehicle_model.visualization.theme import (
    ACCENT_CYAN,
    ACCENT_EMERALD,
    ACCENT_ROSE,
    BG_COLOR,
    CARD_COLOR,
    GRID_COLOR,
    ROAD_COLOR,
    TEXT_MUTED,
    TEXT_PRIMARY,
    apply_dark_theme,
)

apply_dark_theme()

logger = setup_logger("integrated_benchmark")


def generate_double_lane_change_steer(t: float) -> float:
    """Generate smooth double-lane change steering profile (2 steer pulses)."""
    # Maneuver timing:
    # t in [0.5, 1.3] s: left lane change turn (positive pulse)
    # t in [1.3, 2.1] s: counter-turn into second lane (negative pulse)
    # t in [2.3, 3.1] s: right lane return (negative pulse)
    # t in [3.1, 3.9] s: straighten out in original lane (positive pulse)
    amplitude = math.radians(4.8)  # ~4.8 degrees at road wheel
    if 0.5 <= t < 1.3:
        return amplitude * math.sin(2.0 * math.pi * (t - 0.5) / 0.8)
    elif 2.3 <= t < 3.1:
        return -amplitude * math.sin(2.0 * math.pi * (t - 2.3) / 0.8)
    return 0.0


def run_benchmark() -> tuple[list[EngineeringKPIs], dict[str, SimulationHistory]]:
    params = SevenDOFParams.from_yaml()
    road = UniformRoad(mu=0.75)  # Damp asphalt
    duration = 4.5
    dt = 0.005

    v0 = 22.22  # 80 km/h
    omega0 = v0 / params.Reff
    init_state = SevenDOFState(
        vx=v0, vy=0.0, r=0.0, omega=np.full(4, omega0), X=0.0, Y=0.0, psi=0.0
    )

    # 1. Passive Baseline
    passive_ctrl = PassiveBaselineController(params)
    runner_passive = ScenarioRunner(model=SevenDOFModel(params=params), road=road)

    def ctrl_passive_fn(t, state, tele):
        target = ControlTarget(
            driver_steer_f=generate_double_lane_change_steer(t),
            driver_accel_pedal=0.10,  # maintain speed
            mu_estimated=0.75,
        )
        return passive_ctrl.compute_control(state, target, telemetry=tele, dt=dt)

    hist_passive = runner_passive.run(ctrl_passive_fn, init_state, duration=duration, dt=dt)

    # 2. Four-Wheel Steering Only (4WS Speed-Scheduled loaded from configs/steering/)
    fws = FourWheelSteering(SteeringConfig.from_yaml())
    runner_4ws = ScenarioRunner(model=SevenDOFModel(params=params), road=road)

    def ctrl_4ws_fn(t, state, tele):
        delta_f_driver = generate_double_lane_change_steer(t)
        delta_f, delta_r = fws.compute_steering_angles(
            cmd_steer_f=delta_f_driver,
            vx=state.vx,
            vy=state.vy,
            mode=SteeringMode.SPEED_SCHEDULED,
            wheelbase=params.wheelbase,
            lr=params.lr,
        )
        t_drive = np.full(4, 0.10 * params.max_drive_torque / 4.0)
        return SevenDOFControlInput(steer_f=delta_f, steer_r=delta_r, T_drive=t_drive)

    hist_4ws = runner_4ws.run(ctrl_4ws_fn, init_state, duration=duration, dt=dt)

    # 3. Integrated Vehicle Motion Control (IVMC: loaded from centralized configs/controllers/)
    ivmc = IntegratedChassisController.from_yaml(params=params)
    runner_ivmc = ScenarioRunner(model=SevenDOFModel(params=params), road=road)

    def ctrl_ivmc_fn(t, state, tele):
        delta_f_driver = generate_double_lane_change_steer(t)
        target = ControlTarget(
            driver_steer_f=delta_f_driver,
            driver_accel_pedal=0.10,
            mu_estimated=0.75,
        )
        return ivmc.compute_control(state, target, telemetry=tele, dt=dt)

    hist_ivmc = runner_ivmc.run(ctrl_ivmc_fn, init_state, duration=duration, dt=dt)

    # Compute KPIs
    kpi_passive = compute_kpis(hist_passive, controller_name="1. Passive Baseline")
    kpi_4ws = compute_kpis(hist_4ws, controller_name="2. Active 4WS (Speed Scheduled)")
    kpi_ivmc = compute_kpis(hist_ivmc, controller_name="3. Integrated IVMC (4WS + TV + ESC)")

    histories = {
        "Passive": hist_passive,
        "4WS": hist_4ws,
        "IVMC": hist_ivmc,
    }
    return [kpi_passive, kpi_4ws, kpi_ivmc], histories


def plot_comparison(histories: dict[str, SimulationHistory], output_path: Path) -> None:
    """Generate high-resolution 6-panel comparative telemetry plot."""
    fig, axes = plt.subplots(3, 2, figsize=(15, 11), facecolor=BG_COLOR)
    fig.suptitle(
        "ISO 3888 Double Lane Change Benchmark at 80 km/h (mu = 0.75)\n"
        "Comparative Evaluation: Passive vs. 4WS vs. Integrated Chassis Control (IVMC)",
        fontsize=13,
        fontweight="bold",
        color=TEXT_PRIMARY,
    )

    colors = {"Passive": ACCENT_ROSE, "4WS": ACCENT_CYAN, "IVMC": ACCENT_EMERALD}
    styles = {"Passive": "--", "4WS": "-.", "IVMC": "-"}

    # 1. Global Trajectory Y(X) with Plan-View Vehicle Footprints
    ax1 = axes[0, 0]
    v_params = VehicleParams(
        wheelbase=2.8,
        width=1.8,
        length=4.6,
        tire_radius=0.32,
        tire_width=0.225,
        wheel_track=1.60,
        rear_overhang=0.9,
        front_overhang=0.9,
    )
    ax1.axhline(0.0, color=TEXT_MUTED, linestyle=":", alpha=0.5)
    ax1.fill_between([-5, 115], -3.0, 5.0, color=ROAD_COLOR, zorder=0)

    for name, hist in histories.items():
        draw_vehicle_trajectory(
            ax=ax1,
            X=hist.X,
            Y=hist.Y,
            psi=hist.psi,
            steer_f=hist.steer_f,
            steer_r=hist.steer_r,
            params=v_params,
            n_snapshots=7,
            color=colors[name],
            path_style=styles[name],
            label=name,
            alpha=0.85,
            scale=1.5,
        )
    ax1.set_title("1. Planar Vehicle Trajectory $Y(X)$ (Plan View with 2D Vehicles)", fontsize=10, fontweight="bold", color=TEXT_PRIMARY)
    ax1.set_xlabel("Global X [m]", fontsize=9, color=TEXT_MUTED)
    ax1.set_ylabel("Global Y [m]", fontsize=9, color=TEXT_MUTED)
    ax1.set_aspect("equal", adjustable="datalim")
    ax1.legend(loc="upper left", fontsize=8, framealpha=0.92, facecolor=CARD_COLOR, edgecolor=GRID_COLOR, labelcolor=TEXT_PRIMARY)

    # Zoomed inset focusing on the double lane change transition
    axins = ax1.inset_axes([0.55, 0.48, 0.42, 0.48])
    axins.set_facecolor(ROAD_COLOR)
    axins.grid(True, linestyle=":", alpha=0.6, color=GRID_COLOR)
    for name, hist in histories.items():
        draw_vehicle_trajectory(
            ax=axins,
            X=hist.X,
            Y=hist.Y,
            psi=hist.psi,
            steer_f=hist.steer_f,
            steer_r=hist.steer_r,
            params=v_params,
            n_snapshots=8,
            color=colors[name],
            path_style=styles[name],
            label=name,
            alpha=0.9,
            scale=1.0,
        )
    axins.set_xlim(25, 55)
    axins.set_ylim(-0.8, 3.2)
    axins.set_aspect("equal")
    axins.set_title("Zoomed Active Lane Change (1:1 Vehicle)", fontsize=8, fontweight="bold", color=TEXT_PRIMARY)
    ax1.indicate_inset_zoom(axins, edgecolor=ACCENT_CYAN, alpha=0.8)

    # 2. Body Yaw Rate r(t)
    ax2 = axes[0, 1]
    for name, hist in histories.items():
        ax2.plot(hist.time, np.degrees(hist.r), label=name, color=colors[name], linestyle=styles[name], lw=2.0)
    ax2.set_title(r"2. Yaw Rate $r(t)$ [deg/s]", fontsize=10, fontweight="bold", color=TEXT_PRIMARY)
    ax2.set_xlabel("Time [s]", fontsize=9, color=TEXT_MUTED)
    ax2.set_ylabel("Yaw Rate [deg/s]", fontsize=9, color=TEXT_MUTED)
    ax2.legend(loc="lower left", facecolor=CARD_COLOR, edgecolor=GRID_COLOR, labelcolor=TEXT_PRIMARY)

    # 3. Vehicle Body Sideslip Angle beta(t)
    ax3 = axes[1, 0]
    for name, hist in histories.items():
        ax3.plot(hist.time, np.degrees(hist.sideslip_angle), label=name, color=colors[name], linestyle=styles[name], lw=2.0)
    ax3.set_title(r"3. Body Sideslip Angle $\beta = \arctan(v_y / v_x)$ [deg]", fontsize=10, fontweight="bold", color=TEXT_PRIMARY)
    ax3.set_xlabel("Time [s]", fontsize=9, color=TEXT_MUTED)
    ax3.set_ylabel(r"Sideslip $\beta$ [deg]", fontsize=9, color=TEXT_MUTED)
    ax3.legend(loc="lower left", facecolor=CARD_COLOR, edgecolor=GRID_COLOR, labelcolor=TEXT_PRIMARY)

    # 4. Lateral Acceleration ay(t)
    ax4 = axes[1, 1]
    dt = float(histories["IVMC"].time[1] - histories["IVMC"].time[0])
    for name, hist in histories.items():
        dvy_dt = np.gradient(hist.vy, dt)
        ay_g = (dvy_dt + hist.r * hist.vx) / 9.81
        ax4.plot(hist.time, ay_g, label=name, color=colors[name], linestyle=styles[name], lw=2.0)
    ax4.set_title(r"4. Lateral Acceleration $a_y = \dot{v}_y + r v_x$ [g]", fontsize=10, fontweight="bold", color=TEXT_PRIMARY)
    ax4.set_xlabel("Time [s]", fontsize=9, color=TEXT_MUTED)
    ax4.set_ylabel(r"Lateral Accel $a_y$ [g]", fontsize=9, color=TEXT_MUTED)
    ax4.legend(loc="lower left", facecolor=CARD_COLOR, edgecolor=GRID_COLOR, labelcolor=TEXT_PRIMARY)

    # 5. Steering Angles: Front and Rear
    ax5 = axes[2, 0]
    t_time = histories["IVMC"].time
    ax5.plot(t_time, np.degrees(histories["Passive"].steer_f), color=TEXT_PRIMARY, linestyle="--", label=r"Driver Front Steer $\delta_f$", lw=1.5)
    ax5.plot(t_time, np.degrees(histories["4WS"].steer_r), label=r"4WS Rear Steer $\delta_r$", color=ACCENT_CYAN, lw=2.0)
    ax5.plot(t_time, np.degrees(histories["IVMC"].steer_r), label=r"IVMC Rear Steer $\delta_r$", color=ACCENT_EMERALD, lw=2.0)
    ax5.set_title(r"5. Steering Actuation: $\delta_f(t)$ and $\delta_r(t)$ [deg]", fontsize=10, fontweight="bold", color=TEXT_PRIMARY)
    ax5.set_xlabel("Time [s]", fontsize=9, color=TEXT_MUTED)
    ax5.set_ylabel("Steering Angle [deg]", fontsize=9, color=TEXT_MUTED)
    ax5.legend(loc="lower left", facecolor=CARD_COLOR, edgecolor=GRID_COLOR, labelcolor=TEXT_PRIMARY)

    # 6. Peak Tyre Friction Utilization
    ax6 = axes[2, 1]
    for name, hist in histories.items():
        max_util_per_time = np.max(hist.friction_utilization, axis=1) * 100.0
        ax6.plot(hist.time, max_util_per_time, label=name, color=colors[name], linestyle=styles[name], lw=2.0)
    ax6.axhline(100.0, color=ACCENT_ROSE, linestyle=":", lw=1.5, label="Friction Limit (100%)")
    ax6.set_title("6. Peak Wheel Friction Utilization [%]", fontsize=10, fontweight="bold", color=TEXT_PRIMARY)
    ax6.set_xlabel("Time [s]", fontsize=9, color=TEXT_MUTED)
    ax6.set_ylabel("Friction Utilization [%]", fontsize=9, color=TEXT_MUTED)
    ax6.legend(loc="lower left", facecolor=CARD_COLOR, edgecolor=GRID_COLOR, labelcolor=TEXT_PRIMARY)

    plt.tight_layout(rect=[0, 0, 1, 0.95])
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=180, bbox_inches="tight", facecolor=fig.get_facecolor(), edgecolor="none")
    plt.close(fig)
    plt.close(fig)
    print(f"Benchmark plot successfully exported to: {output_path}")


def main() -> None:
    logger.info("Executing ISO 3888 Double Lane Change Benchmark (Passive vs 4WS vs IVMC)...")
    kpis, histories = run_benchmark()
    table_md = format_kpi_table(kpis)
    logger.info("\n" + "=" * 95 + "\nSTANDARDIZED ENGINEERING KPI BENCHMARK TABLE (Plan.md §16.1):\n" + "=" * 95 + "\n" + table_md)

    out_file = Path("output/benchmark/double_lane_change_comparison.png")
    plot_comparison(histories, out_file)
    logger.info(f"Benchmark plot successfully exported to: {out_file}")


if __name__ == "__main__":
    main()
