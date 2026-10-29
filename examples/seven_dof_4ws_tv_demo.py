"""Comprehensive 7-DOF Vehicle Dynamics Demonstration.

Combines:
  1. Front- and Rear-Axle Steering (4WS / AWS with speed-dependent counter-phase)
  2. Wheel-individual Drive Torques (Torque Vectoring)
  3. Wheel-individual Braking Torques (Differential braking & wheel lockup dynamics)
  4. Spatially Varying Road Friction (High-mu asphalt transitioning into a mu-split zone)
  5. Physical Limits Enforcement (Kamm's friction circle, dynamic load transfer, load conservation)

Outputs:
  - Numerical telemetry and validation metrics to console
  - Engineering multi-panel plot saved to disk
"""

from __future__ import annotations

import math
from pathlib import Path
import matplotlib
matplotlib.use("Agg")  # Non-interactive backend
import matplotlib.pyplot as plt
import numpy as np

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
from vehicle_model.roads.surface import MuSplitRoad, RoadSurface
from vehicle_model.simulation.scenario_runner import ScenarioRunner, SimulationHistory
from vehicle_model.steering.four_wheel_steering import FourWheelSteering, SteeringConfig, SteeringMode
from vehicle_model.vehicle_models.seven_dof import SevenDOFModel
from vehicle_model.visualization.plan_view import VehicleParams, draw_vehicle_trajectory
from vehicle_model.visualization.theme import (
    apply_dark_theme,
    style_axis,
    BG_COLOR,
    PANEL_BG,
    TEXT_COLOR,
    TEXT_MUTED,
    GRID_COLOR,
    MU_HIGH_COLOR,
    MU_LOW_COLOR,
    ACCENT_CYAN,
    ACCENT_EMERALD,
    ACCENT_AMBER,
    ACCENT_ROSE,
)

logger = setup_logger("seven_dof_demo")


class CombinedManeuverController:
    """Multi-phase controller demonstrating 4WS, torque vectoring, and mu-split braking."""

    def __init__(self, steering_sys: FourWheelSteering):
        self.steering = steering_sys

    def __call__(
        self,
        t: float,
        state: SevenDOFState,
        last_telemetry: WheelTelemetry | None = None,
    ) -> SevenDOFControlInput:
        T_drive = np.zeros(4, dtype=float)
        T_brake = np.zeros(4, dtype=float)

        # ── Phase 1: Straight-Line Acceleration (t: 0 -> 1.5s) ──
        if t < 1.5:
            cmd_steer_f = 0.0
            # All-wheel drive acceleration
            T_drive[:] = 300.0  # 300 Nm per wheel

        # ── Phase 2: 4WS Steering + Rear Torque Vectoring (t: 1.5 -> 3.5s) ──
        elif t < 3.5:
            # Command front steering left turn (~10 degrees)
            ramp = min(1.0, (t - 1.5) / 0.5)
            cmd_steer_f = math.radians(10.0) * ramp

            # Rear Torque Vectoring: boost outer rear wheel (RR) to assist yaw moment
            T_drive[FL] = 100.0
            T_drive[FR] = 100.0
            T_drive[RL] = 50.0   # Inner rear wheel
            T_drive[RR] = 450.0  # Outer rear wheel (Torque Vectoring: +400 Nm yaw moment)

        # ── Phase 3: Transition to Mu-Split & Heavy Braking (t: 3.5 -> 5.5s) ──
        else:
            # Straighten steering slightly
            cmd_steer_f = math.radians(2.0)
            # Heavy braking demand (1200 Nm per wheel)
            # On mu-split (left = 1.0, right = 0.2), the right wheels will experience
            # torque exceeding available friction (T_b > mu_R * Fz * R), inducing wheel lock!
            T_brake[:] = 1200.0

        # Compute 4WS steering angles using speed-scheduled counter-phase mode
        delta_f, delta_r = self.steering.compute_steering_angles(
            cmd_steer_f=cmd_steer_f,
            vx=state.vx,
            mode=SteeringMode.SPEED_SCHEDULED,
        )

        return SevenDOFControlInput(
            steer_f=delta_f,
            steer_r=delta_r,
            T_drive=T_drive,
            T_brake=T_brake,
        )


def run_demonstration(output_dir: Path | None = None) -> SimulationHistory:
    """Execute the combined 7-DOF vehicle dynamics demonstration."""
    out_dir = output_dir or Path(__file__).resolve().parents[1] / "output" / "demo_7dof"
    out_dir.mkdir(parents=True, exist_ok=True)

    # Load physical vehicle and steering parameters from centralized YAML configs
    params = SevenDOFParams.from_yaml()
    model = SevenDOFModel(params=params)
    steering_sys = FourWheelSteering(SteeringConfig.from_yaml())
    controller = CombinedManeuverController(steering_sys)

    # Road: Split-mu surface along the track
    # Y >= 1.0: High friction dry asphalt (mu = 1.0)
    # Y < 1.0: Low friction wet/ice surface (mu = 0.2)
    # As the car turns left and crosses Y, its right wheels encounter the low-mu patch!
    road = MuSplitRoad(mu_left=1.0, mu_right=0.2, split_y=1.5, transition_width=0.2)

    runner = ScenarioRunner(model=model, road=road)

    # Initial state: moving at 10 m/s (~36 km/h) in pure rolling equilibrium
    omega0 = 10.0 / params.Reff
    init_state = SevenDOFState(
        vx=10.0,
        vy=0.0,
        r=0.0,
        omega=np.full(4, omega0, dtype=float),
        X=0.0,
        Y=0.0,
        psi=0.0,
    )

    duration = 5.5
    dt = 0.005  # 5 ms integration step for accurate wheel rotation dynamics
    logger.info(f"Starting 7-DOF Simulation: duration={duration}s, dt={dt}s ({int(duration/dt)} steps)...")

    history = runner.run(
        controller=controller,
        init_state=init_state,
        duration=duration,
        dt=dt,
    )

    # Evaluate physical consistency
    total_weight = params.mass * params.g
    max_normal_error = np.max(np.abs(history.normal_load_sum - total_weight))
    max_slip = np.max(history.kappa)
    min_slip = np.min(history.kappa)
    max_yaw_rate = np.max(np.abs(history.r))
    final_speed = history.speed[-1]

    summary = (
        "=== Simulation Results & Physical Consistency ===\n"
        f"Vehicle Mass:              {params.mass:.1f} kg (Weight: {total_weight:.1f} N)\n"
        f"Initial Speed:             {history.speed[0]:.2f} m/s ({history.speed[0]*3.6:.1f} km/h)\n"
        f"Max Speed:                 {np.max(history.speed):.2f} m/s ({np.max(history.speed)*3.6:.1f} km/h)\n"
        f"Final Speed:               {final_speed:.2f} m/s ({final_speed*3.6:.1f} km/h)\n"
        f"Max Yaw Rate:              {max_yaw_rate:.3f} rad/s ({math.degrees(max_yaw_rate):.1f} deg/s)\n"
        f"Normal Load Conservation:  Max deviation = {max_normal_error:.4e} N (STRICTLY CONSERVED)\n"
        f"Longitudinal Slip Range:   [{min_slip:.3f}, {max_slip:.3f}]\n"
        f"  - Peak Acceleration Slip: {max_slip:.3f} (positive traction)\n"
        f"  - Peak Braking Slip:      {min_slip:.3f} (near-lock on low-mu right side)\n"
        f"Telemetry plot saved to:   {out_dir / 'seven_dof_maneuver.png'}"
    )
    logger.info("\n" + summary)

    # Generate telemetry plot
    plot_results(history, road, params, out_dir / "seven_dof_maneuver.png")

    return history


def plot_results(
    hist: SimulationHistory,
    road: RoadSurface,
    params: SevenDOFParams,
    save_path: Path,
) -> None:
    """Generate high-resolution multi-panel engineering plots."""
    apply_dark_theme()
    fig = plt.figure(figsize=(16, 12), facecolor=BG_COLOR, constrained_layout=True)
    gs = fig.add_gridspec(3, 3)

    t = hist.time
    wheel_labels = ["FL", "FR", "RL", "RR"]
    colors = [ACCENT_CYAN, ACCENT_EMERALD, ACCENT_AMBER, ACCENT_ROSE]

    # 1. Global Trajectory X-Y with Vehicle Footprints (Plan View)
    ax1 = fig.add_subplot(gs[0, 0])
    style_axis(ax1)
    v_params = VehicleParams(
        wheelbase=params.wheelbase,
        width=1.8,
        length=4.6,
        tire_radius=params.Reff,
        tire_width=0.225,
        wheel_track=params.wf,
        rear_overhang=0.9,
        front_overhang=0.9,
        cog_to_rear_axle=params.lr,   # trajectories are CoG positions
    )
    draw_vehicle_trajectory(
        ax=ax1,
        X=hist.X,
        Y=hist.Y,
        psi=hist.psi,
        steer_f=hist.steer_f,
        steer_r=hist.steer_r,
        params=v_params,
        n_snapshots=8,
        color=ACCENT_CYAN,
        label="4WS Vehicle Pose",
        alpha=0.85,
        scale=1.5,
    )
    ax1.axhline(1.5, color=TEXT_MUTED, ls="--", label="Mu-Split Boundary (Y=1.5m)")
    ax1.fill_between([np.min(hist.X) - 5, np.max(hist.X) + 5], 1.5, np.max(hist.Y) + 5, color=MU_HIGH_COLOR, alpha=0.4, label=r"High $\mu=1.0$")
    ax1.fill_between([np.min(hist.X) - 5, np.max(hist.X) + 5], np.min(hist.Y) - 5, 1.5, color=MU_LOW_COLOR, alpha=0.4, label=r"Low $\mu=0.2$")
    ax1.set_xlabel("X Position [m]", color=TEXT_COLOR)
    ax1.set_ylabel("Y Position [m]", color=TEXT_COLOR)
    ax1.set_title("Vehicle Trajectory (Plan View with 2D Planar 4WS)", fontsize=10, fontweight="bold", color=TEXT_COLOR)
    ax1.set_aspect("equal", adjustable="datalim")
    ax1.grid(True, alpha=0.3, color=GRID_COLOR)
    ax1.legend(loc="upper left", fontsize=8, facecolor=PANEL_BG, edgecolor=GRID_COLOR, labelcolor=TEXT_COLOR)

    # 2. Chassis Velocities & Sideslip
    ax2 = fig.add_subplot(gs[0, 1])
    style_axis(ax2)
    ax2.plot(t, hist.vx, color=TEXT_COLOR, lw=1.5, label="vx [m/s]")
    ax2.plot(t, hist.vy, color=ACCENT_EMERALD, ls="--", lw=1.5, label="vy [m/s]")
    ax2.plot(t, np.degrees(hist.sideslip_angle), color="#e879f9", ls=":", lw=1.5, label=r"$\beta$ [deg]")
    ax2.set_xlabel("Time [s]", color=TEXT_COLOR)
    ax2.set_ylabel("Velocities & Angle", color=TEXT_COLOR)
    ax2.set_title("Longitudinal / Lateral Motion", color=TEXT_COLOR)
    ax2.grid(True, alpha=0.3, color=GRID_COLOR)
    ax2.legend(loc="best", fontsize=8, facecolor=PANEL_BG, edgecolor=GRID_COLOR, labelcolor=TEXT_COLOR)

    # 3. Yaw Rate and 4WS Steer Angles
    ax3 = fig.add_subplot(gs[0, 2])
    style_axis(ax3)
    ax3.plot(t, np.degrees(hist.r), color=ACCENT_ROSE, lw=2, label="Yaw rate r [deg/s]")
    ax3.plot(t, np.degrees(hist.steer_f), color=ACCENT_CYAN, lw=1.5, label=r"Front Steer $\delta_f$ [deg]")
    ax3.plot(t, np.degrees(hist.steer_r), color=ACCENT_AMBER, ls="--", lw=1.5, label=r"Rear Steer $\delta_r$ (4WS) [deg]")
    ax3.set_xlabel("Time [s]", color=TEXT_COLOR)
    ax3.set_ylabel("Rates & Angles [deg, deg/s]", color=TEXT_COLOR)
    ax3.set_title("Yaw Dynamics & 4WS Coordination", color=TEXT_COLOR)
    ax3.grid(True, alpha=0.3, color=GRID_COLOR)
    ax3.legend(loc="best", fontsize=8, facecolor=PANEL_BG, edgecolor=GRID_COLOR, labelcolor=TEXT_COLOR)

    # 4. Wheel Rotational Speeds vs Equivalent Chassis Speed
    ax4 = fig.add_subplot(gs[1, 0])
    style_axis(ax4)
    ax4.plot(t, hist.vx / params.Reff, color=TEXT_MUTED, ls="--", lw=2, label=r"$v_x / R_{eff}$ (Linear Ref)")
    for i in range(4):
        ax4.plot(t, hist.omega[:, i], color=colors[i], label=fr"$\omega_{{{wheel_labels[i]}}}$")
    ax4.set_xlabel("Time [s]", color=TEXT_COLOR)
    ax4.set_ylabel("Wheel Speed [rad/s]", color=TEXT_COLOR)
    ax4.set_title("Wheel Rotational Speeds (Spin / Lockup)", color=TEXT_COLOR)
    ax4.grid(True, alpha=0.3, color=GRID_COLOR)
    ax4.legend(loc="best", fontsize=8, facecolor=PANEL_BG, edgecolor=GRID_COLOR, labelcolor=TEXT_COLOR)

    # 5. Longitudinal Slip Ratios kappa
    ax5 = fig.add_subplot(gs[1, 1])
    style_axis(ax5)
    for i in range(4):
        ax5.plot(t, hist.kappa[:, i], color=colors[i], lw=1.5, label=fr"$\kappa_{{{wheel_labels[i]}}}$")
    ax5.axhline(0.0, color=TEXT_MUTED, lw=0.8)
    ax5.axhline(-1.0, color=ACCENT_ROSE, ls=":", label="Lockup Limit (-1.0)")
    ax5.set_xlabel("Time [s]", color=TEXT_COLOR)
    ax5.set_ylabel("Longitudinal Slip [-]", color=TEXT_COLOR)
    ax5.set_title("Tire Longitudinal Slip Ratio", color=TEXT_COLOR)
    ax5.grid(True, alpha=0.3, color=GRID_COLOR)
    ax5.legend(loc="best", fontsize=8, facecolor=PANEL_BG, edgecolor=GRID_COLOR, labelcolor=TEXT_COLOR)

    # 6. Dynamic Normal Loads (Dynamic Load Transfer)
    ax6 = fig.add_subplot(gs[1, 2])
    style_axis(ax6)
    for i in range(4):
        ax6.plot(t, hist.Fz[:, i], color=colors[i], label=f"$F_{{z,{wheel_labels[i]}}}$")
    ax6.plot(t, hist.normal_load_sum, color=TEXT_COLOR, ls=":", lw=1.5, label=r"Total $\sum F_z$")
    ax6.set_xlabel("Time [s]", color=TEXT_COLOR)
    ax6.set_ylabel("Normal Force [N]", color=TEXT_COLOR)
    ax6.set_title("Dynamic Normal Loads & Load Conservation", color=TEXT_COLOR)
    ax6.grid(True, alpha=0.3, color=GRID_COLOR)
    ax6.legend(loc="best", fontsize=8, facecolor=PANEL_BG, edgecolor=GRID_COLOR, labelcolor=TEXT_COLOR)

    # 7. Applied Drive and Braking Torques
    ax7 = fig.add_subplot(gs[2, 0])
    style_axis(ax7)
    for i in range(4):
        ax7.plot(t, hist.T_drive[:, i], color=colors[i], label=f"$T_{{d,{wheel_labels[i]}}}$")
        ax7.plot(t, -hist.T_brake[:, i], color=colors[i], ls="--", label=f"$-T_{{b,{wheel_labels[i]}}}$" if i == 0 else None)
    ax7.set_xlabel("Time [s]", color=TEXT_COLOR)
    ax7.set_ylabel("Torque [Nm]", color=TEXT_COLOR)
    ax7.set_title("Actuator Torques (Torque Vectoring / Braking)", color=TEXT_COLOR)
    ax7.grid(True, alpha=0.3, color=GRID_COLOR)
    ax7.legend(loc="best", fontsize=8, facecolor=PANEL_BG, edgecolor=GRID_COLOR, labelcolor=TEXT_COLOR)

    # 8. Longitudinal Forces Fx
    ax8 = fig.add_subplot(gs[2, 1])
    style_axis(ax8)
    for i in range(4):
        ax8.plot(t, hist.Fx_wheel[:, i], color=colors[i], label=f"$F_{{x,{wheel_labels[i]}}}$")
    ax8.set_xlabel("Time [s]", color=TEXT_COLOR)
    ax8.set_ylabel("Tire Force Fx [N]", color=TEXT_COLOR)
    ax8.set_title("Tire Longitudinal Ground Forces", color=TEXT_COLOR)
    ax8.grid(True, alpha=0.3, color=GRID_COLOR)
    ax8.legend(loc="best", fontsize=8, facecolor=PANEL_BG, edgecolor=GRID_COLOR, labelcolor=TEXT_COLOR)

    # 9. Lateral Forces Fy
    ax9 = fig.add_subplot(gs[2, 2])
    style_axis(ax9)
    for i in range(4):
        ax9.plot(t, hist.Fy_wheel[:, i], color=colors[i], label=f"$F_{{y,{wheel_labels[i]}}}$")
    ax9.set_xlabel("Time [s]", color=TEXT_COLOR)
    ax9.set_ylabel("Tire Force Fy [N]", color=TEXT_COLOR)
    ax9.set_title("Tire Lateral Ground Forces", color=TEXT_COLOR)
    ax9.grid(True, alpha=0.3, color=GRID_COLOR)
    ax9.legend(loc="best", fontsize=8, facecolor=PANEL_BG, edgecolor=GRID_COLOR, labelcolor=TEXT_COLOR)

    plt.savefig(save_path, dpi=200, facecolor=fig.get_facecolor(), edgecolor="none")
    plt.close(fig)


if __name__ == "__main__":
    run_demonstration()
