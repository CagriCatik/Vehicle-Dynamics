"""Demonstration: Vertical Suspension & 14-DOF Full Vehicle Dynamics.

Demonstrates:
1. Quarter-car road bump traversal comparing:
   - Passive baseline damper
   - Semi-active Skyhook comfort controller
   - Semi-active Groundhook road-holding controller
2. 14-DOF full vehicle combined braking and steering on uneven road with dynamic load transfer.
3. Export of engineering diagnostic plots to output/suspension/.md estimation robustness).
"""

from __future__ import annotations

import math
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from vehicle_model.logging import setup_logger
from vehicle_model.suspension.quarter_car import (
    QuarterCarConfig,
    QuarterCarModel,
    QuarterCarState,
)
from vehicle_model.suspension.controllers import (
    GroundhookController,
    SemiActiveConfig,
    SkyhookController,
)
from vehicle_model.vehicle_models.fourteen_dof import (
    FourteenDOFConfig,
    FourteenDOFModel,
    FourteenDOFState,
)
from vehicle_model.tire_model import load_tyre
from vehicle_model.visualization.theme import (
    ACCENT_AMBER,
    ACCENT_BLUE,
    ACCENT_CYAN,
    ACCENT_EMERALD,
    ACCENT_PURPLE,
    ACCENT_ROSE,
    BG_COLOR,
    CARD_COLOR,
    GRID_COLOR,
    TEXT_MUTED,
    TEXT_PRIMARY,
    apply_dark_theme,
)


def main() -> None:
    apply_dark_theme()
    logger = setup_logger(log_file=Path("logs/suspension_demo.log"))

    logger.info("Starting Suspension Dynamics Demonstration...")

    output_dir = Path("output/suspension")
    output_dir.mkdir(parents=True, exist_ok=True)

    # =========================================================================
    # PART 1: QUARTER-CAR COMPARISON OVER ROAD BUMP
    # =========================================================================
    logger.info("Executing Part 1: Quarter-Car Bump Traversal (Passive vs Skyhook vs Groundhook)")

    qc_cfg = QuarterCarConfig.from_yaml("configs/suspension/quarter_car.yaml")
    qc_model = QuarterCarModel(qc_cfg)

    semi_cfg = SemiActiveConfig.from_yaml("configs/suspension/semi_active.yaml")
    skyhook = SkyhookController(config=semi_cfg, continuous=True)
    groundhook = GroundhookController(config=semi_cfg, continuous=True)

    dt = 0.001
    t_end = 2.5
    steps = int(t_end / dt)
    time_arr = np.linspace(0.0, t_end, steps)

    # Half-sine road bump: 0.07m height, 0.25s duration
    bump_h = 0.07
    bump_dur = 0.25
    t_start = 0.2

    def road_profile(t: float):
        if t_start <= t <= t_start + bump_dur:
            tau = (t - t_start) / bump_dur
            z = 0.5 * bump_h * (1.0 - math.cos(2.0 * math.pi * tau))
            dz = 0.5 * bump_h * (2.0 * math.pi / bump_dur) * math.sin(2.0 * math.pi * tau)
            return z, dz
        return 0.0, 0.0

    # Simulation histories
    hist_passive = {"ddzs": [], "Fz_dyn": [], "deflection": [], "zs": []}
    hist_skyhook = {"ddzs": [], "Fz_dyn": [], "deflection": [], "zs": []}
    hist_groundhook = {"ddzs": [], "Fz_dyn": [], "deflection": [], "zs": []}

    s_pass = QuarterCarState()
    s_sky = QuarterCarState()
    s_gh = QuarterCarState()

    for t in time_arr:
        rz, rdz = road_profile(t)

        # 1. Passive
        s_pass, out_p = qc_model.step(s_pass, rz, rdz, dt, damper_c=qc_cfg.suspension_damping)
        hist_passive["ddzs"].append(out_p.ddzs)
        hist_passive["Fz_dyn"].append(out_p.dynamic_tyre_load)
        hist_passive["deflection"].append(out_p.suspension_deflection)
        hist_passive["zs"].append(out_p.zs)

        # 2. Skyhook
        c_sky = skyhook.compute_damping(s_sky.dzs, s_sky.dzu, dt)
        s_sky, out_s = qc_model.step(s_sky, rz, rdz, dt, damper_c=c_sky)
        hist_skyhook["ddzs"].append(out_s.ddzs)
        hist_skyhook["Fz_dyn"].append(out_s.dynamic_tyre_load)
        hist_skyhook["deflection"].append(out_s.suspension_deflection)
        hist_skyhook["zs"].append(out_s.zs)

        # 3. Groundhook
        c_gh = groundhook.compute_damping(s_gh.dzs, s_gh.dzu, dt)
        s_gh, out_g = qc_model.step(s_gh, rz, rdz, dt, damper_c=c_gh)
        hist_groundhook["ddzs"].append(out_g.ddzs)
        hist_groundhook["Fz_dyn"].append(out_g.dynamic_tyre_load)
        hist_groundhook["deflection"].append(out_g.suspension_deflection)
        hist_groundhook["zs"].append(out_g.zs)

    rms_acc_pass = float(np.sqrt(np.mean(np.square(hist_passive["ddzs"]))))
    rms_acc_sky = float(np.sqrt(np.mean(np.square(hist_skyhook["ddzs"]))))
    rms_acc_gh = float(np.sqrt(np.mean(np.square(hist_groundhook["ddzs"]))))

    rms_fz_pass = float(np.sqrt(np.mean(np.square(hist_passive["Fz_dyn"]))))
    rms_fz_sky = float(np.sqrt(np.mean(np.square(hist_skyhook["Fz_dyn"]))))
    rms_fz_gh = float(np.sqrt(np.mean(np.square(hist_groundhook["Fz_dyn"]))))

    comfort_gain = (rms_acc_pass - rms_acc_sky) / rms_acc_pass * 100.0
    holding_gain = (rms_fz_pass - rms_fz_gh) / rms_fz_pass * 100.0

    logger.info(f"Quarter-Car Passive RMS Acceleration: {rms_acc_pass:.3f} m/s^2")
    logger.info(f"Quarter-Car Skyhook RMS Acceleration: {rms_acc_sky:.3f} m/s^2 (Comfort Gain: +{comfort_gain:.1f}%)")
    logger.info(f"Quarter-Car Groundhook RMS Dynamic Load: {rms_fz_gh:.1f} N (Road Holding Gain: +{holding_gain:.1f}%)")

    # =========================================================================
    # PART 2: 14-DOF FULL VEHICLE MANEUVER (BRAKING + STEERING + UNEVEN ROAD)
    # =========================================================================
    logger.info("Executing Part 2: 14-DOF Full Vehicle Dynamics Simulation")

    fdof_cfg = FourteenDOFConfig.from_yaml("configs/suspension/fourteen_dof.yaml")
    tyre = load_tyre("pacejka_front_synthetic")
    fdof_model = FourteenDOFModel(config=fdof_cfg, tyre_model=tyre)

    dt_14 = 0.002
    t_end_14 = 3.0
    steps_14 = int(t_end_14 / dt_14)
    time_14 = np.linspace(0.0, t_end_14, steps_14)

    state_14 = FourteenDOFState(vx=28.0)  # ~100 km/h initial speed

    hist_14 = {
        "time": [],
        "x": [],
        "y": [],
        "vx": [],
        "ax": [],
        "ay": [],
        "theta": [],  # Pitch [rad]
        "phi": [],  # Roll [rad]
        "zs": [],  # Heave [m]
        "Fz_FL": [],
        "Fz_FR": [],
        "Fz_RL": [],
        "Fz_RR": [],
    }

    for step, t in enumerate(time_14):
        # Maneuver profile:
        # t = 0.5 - 1.5s: Combined emergency brake
        T_brake = np.zeros(4)
        if 0.5 <= t <= 2.2:
            T_brake = np.full(4, 600.0)

        # t = 0.8 - 2.0s: Sine wave steering (lane change maneuver)
        delta_cmd = 0.0
        if 0.8 <= t <= 2.0:
            delta_cmd = 0.05 * math.sin(2.0 * math.pi * 0.8 * (t - 0.8))
        delta = np.array([delta_cmd, delta_cmd, 0.0, 0.0])

        T_drive = np.zeros(4)

        # Road bump under right wheels at t = 1.2s
        rz = np.zeros(4)
        rdz = np.zeros(4)
        if 1.2 <= t <= 1.45:
            tau = (t - 1.2) / 0.25
            bump_val = 0.04 * math.sin(math.pi * tau)
            bump_dot = 0.04 * (math.pi / 0.25) * math.cos(math.pi * tau)
            rz[1] = bump_val  # FR bump
            rz[3] = bump_val  # RR bump
            rdz[1] = bump_dot
            rdz[3] = bump_dot

        state_14, telem = fdof_model.step(
            state_14, delta, T_drive, T_brake, dt=dt_14, road_z=rz, road_dz=rdz
        )

        hist_14["time"].append(t)
        hist_14["x"].append(state_14.x)
        hist_14["y"].append(state_14.y)
        hist_14["vx"].append(state_14.vx)
        hist_14["ax"].append(telem.ax)
        hist_14["ay"].append(telem.ay)
        hist_14["theta"].append(state_14.theta)
        hist_14["phi"].append(state_14.phi)
        hist_14["zs"].append(state_14.zs)
        hist_14["Fz_FL"].append(telem.Fz[0])
        hist_14["Fz_FR"].append(telem.Fz[1])
        hist_14["Fz_RL"].append(telem.Fz[2])
        hist_14["Fz_RR"].append(telem.Fz[3])

    logger.info(f"14-DOF simulation complete: Stopped from 100 km/h to {state_14.vx * 3.6:.1f} km/h")
    logger.info(f"Peak Nose-Dive Pitch: {min(hist_14['theta']) * 180 / math.pi:.2f} deg")
    logger.info(f"Peak Body Roll Angle: {max(np.abs(hist_14['phi'])) * 180 / math.pi:.2f} deg")

    # =========================================================================
    # PART 3: GENERATE DIAGNOSTIC 6-PANEL FIGURE
    # =========================================================================
    fig, axes = plt.subplots(3, 2, figsize=(14, 12), facecolor=BG_COLOR)
    fig.suptitle(
        "Vertical Suspension & 14-DOF Full Vehicle Dynamics Validation",
        fontsize=14,
        fontweight="bold",
        color=TEXT_PRIMARY,
    )

    # Subplot 1: Quarter-Car Sprung Mass Acceleration
    ax1 = axes[0, 0]
    ax1.plot(time_arr, hist_passive["ddzs"], color=ACCENT_ROSE, linestyle="-", label=f"Passive (RMS: {rms_acc_pass:.2f} m/s²)")
    ax1.plot(time_arr, hist_skyhook["ddzs"], color=ACCENT_CYAN, linestyle="-", linewidth=1.8, label=f"Skyhook (RMS: {rms_acc_sky:.2f} m/s², -{comfort_gain:.0f}%)")
    ax1.plot(time_arr, hist_groundhook["ddzs"], color=ACCENT_EMERALD, linestyle="--", label=f"Groundhook (RMS: {rms_acc_gh:.2f} m/s²)")
    ax1.set_title("Quarter-Car: Body Acceleration (Comfort)", fontweight="bold", color=TEXT_PRIMARY)
    ax1.set_xlabel("Time [s]", color=TEXT_MUTED)
    ax1.set_ylabel(r"$\ddot{z}_s$ [m/s²]", color=TEXT_MUTED)
    ax1.legend(loc="upper right", fontsize=9, facecolor=CARD_COLOR, edgecolor=GRID_COLOR, labelcolor=TEXT_PRIMARY)

    # Subplot 2: Quarter-Car Dynamic Tyre Force
    ax2 = axes[0, 1]
    ax2.plot(time_arr, hist_passive["Fz_dyn"], color=ACCENT_ROSE, linestyle="-", label="Passive")
    ax2.plot(time_arr, hist_skyhook["Fz_dyn"], color=ACCENT_CYAN, linestyle="-", label="Skyhook")
    ax2.plot(time_arr, hist_groundhook["Fz_dyn"], color=ACCENT_EMERALD, linestyle="-", linewidth=1.8, label=f"Groundhook (-{holding_gain:.0f}% load var)")
    ax2.set_title("Quarter-Car: Dynamic Tyre Normal Force (Road Holding)", fontweight="bold", color=TEXT_PRIMARY)
    ax2.set_xlabel("Time [s]", color=TEXT_MUTED)
    ax2.set_ylabel(r"$\Delta F_z$ [N]", color=TEXT_MUTED)
    ax2.legend(loc="upper right", fontsize=9, facecolor=CARD_COLOR, edgecolor=GRID_COLOR, labelcolor=TEXT_PRIMARY)

    # Subplot 3: 14-DOF Pitch and Heave
    ax3 = axes[1, 0]
    ax3_twin = ax3.twinx()
    l1 = ax3.plot(hist_14["time"], np.array(hist_14["theta"]) * 180.0 / math.pi, color=ACCENT_PURPLE, label="Pitch Angle [deg]")
    l2 = ax3_twin.plot(hist_14["time"], np.array(hist_14["zs"]) * 1000.0, color=ACCENT_AMBER, linestyle="--", label="Heave [mm]")
    ax3.set_title("14-DOF: Chassis Pitch & Heave Dynamics", fontweight="bold", color=TEXT_PRIMARY)
    ax3.set_xlabel("Time [s]", color=TEXT_MUTED)
    ax3.set_ylabel("Pitch Angle [deg]", color=ACCENT_PURPLE)
    ax3_twin.set_ylabel("Heave [mm]", color=ACCENT_AMBER)
    ax3_twin.tick_params(colors=ACCENT_AMBER)
    lines = l1 + l2
    labels = [l.get_label() for l in lines]
    ax3.legend(lines, labels, loc="lower right", fontsize=9, facecolor=CARD_COLOR, edgecolor=GRID_COLOR, labelcolor=TEXT_PRIMARY)

    # Subplot 4: 14-DOF Roll Angle and Lateral Acceleration
    ax4 = axes[1, 1]
    ax4_twin = ax4.twinx()
    l3 = ax4.plot(hist_14["time"], np.array(hist_14["phi"]) * 180.0 / math.pi, color=ACCENT_CYAN, label="Roll Angle [deg]")
    l4 = ax4_twin.plot(hist_14["time"], hist_14["ay"], color=ACCENT_ROSE, linestyle="--", label=r"Lateral Acc $a_y$ [m/s²]")
    ax4.set_title("14-DOF: Chassis Roll & Lateral Acceleration", fontweight="bold", color=TEXT_PRIMARY)
    ax4.set_xlabel("Time [s]", color=TEXT_MUTED)
    ax4.set_ylabel("Roll Angle [deg]", color=ACCENT_CYAN)
    ax4_twin.set_ylabel(r"$a_y$ [m/s²]", color=ACCENT_ROSE)
    ax4_twin.tick_params(colors=ACCENT_ROSE)
    lines2 = l3 + l4
    labels2 = [l.get_label() for l in lines2]
    ax4.legend(lines2, labels2, loc="lower left", fontsize=9, facecolor=CARD_COLOR, edgecolor=GRID_COLOR, labelcolor=TEXT_PRIMARY)

    # Subplot 5: 14-DOF 4-Corner Dynamic Tyre Loads (Normal Load Transfer)
    ax5 = axes[2, 0]
    ax5.plot(hist_14["time"], hist_14["Fz_FL"], color=ACCENT_CYAN, label="FL (Front Left)")
    ax5.plot(hist_14["time"], hist_14["Fz_FR"], color=ACCENT_BLUE, label="FR (Front Right)")
    ax5.plot(hist_14["time"], hist_14["Fz_RL"], color=ACCENT_AMBER, label="RL (Rear Left)")
    ax5.plot(hist_14["time"], hist_14["Fz_RR"], color=ACCENT_ROSE, label="RR (Rear Right)")
    ax5.set_title("14-DOF: 4-Wheel Dynamic Normal Loads $F_{z,i}(t)$", fontweight="bold", color=TEXT_PRIMARY)
    ax5.set_xlabel("Time [s]", color=TEXT_MUTED)
    ax5.set_ylabel(r"$F_z$ [N]", color=TEXT_MUTED)
    ax5.legend(loc="upper right", fontsize=8, ncol=2, facecolor=CARD_COLOR, edgecolor=GRID_COLOR, labelcolor=TEXT_PRIMARY)

    # Subplot 6: 14-DOF Vehicle Path (Trajectory)
    ax6 = axes[2, 1]
    ax6.plot(hist_14["x"], hist_14["y"], color=ACCENT_CYAN, linewidth=2.0, label="Chassis Trajectory")
    ax6.set_title("14-DOF: Global XY Trajectory (Braking + Steer)", fontweight="bold", color=TEXT_PRIMARY)
    ax6.set_xlabel("Global X [m]", color=TEXT_MUTED)
    ax6.set_ylabel("Global Y [m]", color=TEXT_MUTED)
    ax6.axis("equal")
    ax6.legend(loc="lower right", fontsize=9, facecolor=CARD_COLOR, edgecolor=GRID_COLOR, labelcolor=TEXT_PRIMARY)

    plt.tight_layout()
    plot_file = output_dir / "suspension_and_14dof_validation.png"
    plt.savefig(plot_file, dpi=160, facecolor=fig.get_facecolor(), edgecolor="none")
    plt.close()

    logger.info(f"Saved diagnostic telemetry figure to {plot_file}")
    logger.info("Demonstration Successfully Completed!")


if __name__ == "__main__":
    main()
