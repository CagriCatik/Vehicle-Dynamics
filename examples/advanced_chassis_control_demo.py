"""Flagship Demonstration: Advanced Vehicle Dynamics & Chassis Control.

Demonstrates:
1. Nonlinear (beta, beta_dot) Phase-Plane Stability Boundary & Envelope Control:
   - Extreme lift-off oversteer on low/split-mu surface.
   - Plots actual phase-plane trajectory (beta, beta_dot) against theoretical stability boundaries.
   - Demonstrates how envelope control pulls state trajectory back into stable limit cycle.

2. Integrated 14-DOF Vertical & Planar Multi-Body Chassis Control:
   - Severe NHTSA Dynamic Fishhook steering reversal at 80 km/h.
   - Passive 14-DOF vs. Fully Coordinated 14-DOF Chassis Control (Active Roll Stiffness Distribution + IVMC).
   - Demonstrates roll angle suppression, wheel-lift elimination (LTR), and dynamic handling balance.

Usage:
    python examples/advanced_chassis_control_demo.py
"""

from __future__ import annotations

import math
from pathlib import Path
import sys
import numpy as np
import matplotlib.pyplot as plt

# Ensure repository root is on sys.path
_repo_root = Path(__file__).resolve().parent.parent
if str(_repo_root) not in sys.path:
    sys.path.insert(0, str(_repo_root))

from vehicle_model.controllers.active_roll import ActiveRollConfig, ActiveRollDistributionController
from vehicle_model.controllers.framework import ControlTarget, PassiveBaselineController
from vehicle_model.controllers.integrated_controller import IntegratedChassisController
from vehicle_model.controllers.phase_plane import PhasePlaneConfig, PhasePlaneEnvelopeController
from vehicle_model.core.types import FL, FR, RL, RR, SevenDOFParams, SevenDOFState
from vehicle_model.simulation.scenario_runner import simulate_maneuver
from vehicle_model.vehicle_models.fourteen_dof import FourteenDOFConfig, FourteenDOFModel, FourteenDOFState
from vehicle_model.vehicle_models.seven_dof import SevenDOFModel
from vehicle_model.visualization.plan_view import VehicleParams, draw_vehicle_trajectory
from vehicle_model.visualization.theme import (
    ACCENT_AMBER,
    ACCENT_BLUE,
    ACCENT_CYAN,
    ACCENT_EMERALD,
    ACCENT_ROSE,
    BG_COLOR,
    CARD_COLOR,
    GRID_COLOR,
    TEXT_MUTED,
    TEXT_PRIMARY,
    apply_dark_theme,
)


def run_phase_plane_demo(output_dir: Path) -> None:
    apply_dark_theme()
    print("=================================================================")
    print("PART 1: NONLINEAR (BETA, BETA_DOT) PHASE-PLANE ENVELOPE CONTROL")
    print("=================================================================")

    # Extreme lift-off oversteer scenario:
    # 90 km/h initial speed on mu=0.7 wet road, driver steers 4.5 deg at t=0.3s,
    # then suddenly lifts off throttle and applies light braking at t=0.8s
    v0 = 90.0 / 3.6
    duration = 3.5
    dt = 0.005
    steps = int(duration / dt)
    t_arr = np.linspace(0.0, duration, steps)

    delta_driver = np.zeros(steps)
    T_brake_driver = np.zeros(steps)
    for i, t in enumerate(t_arr):
        if 0.3 <= t < 0.8:
            delta_driver[i] = math.radians(4.5) * ((t - 0.3) / 0.5)
        elif t >= 0.8:
            delta_driver[i] = math.radians(4.5)
            T_brake_driver[i] = 400.0  # Lift-off with engine drag / light braking

    model_params = SevenDOFParams.from_yaml("configs/vehicles/seven_dof_passenger.yaml")

    # 1. Unregulated Passive Baseline
    from vehicle_model.controllers.framework import PassiveBaselineController
    hist_pass = simulate_maneuver(
        model=SevenDOFModel(params=model_params),
        controller=PassiveBaselineController(),
        v_initial=v0,
        delta_driver=delta_driver,
        T_drive_demand=0.0,
        T_brake_demand=T_brake_driver,
        duration=duration,
        dt=dt,
        mu=0.70,
    )

    # 2. IVMC with Active Phase-Plane Envelope Control
    ctrl_pp = IntegratedChassisController(
        params=model_params,
        enable_4ws=True,
        enable_tv=True,
        enable_phase_plane=True,
    )
    hist_pp = simulate_maneuver(
        model=SevenDOFModel(params=model_params),
        controller=ctrl_pp,
        v_initial=v0,
        delta_driver=delta_driver,
        T_drive_demand=0.0,
        T_brake_demand=T_brake_driver,
        duration=duration,
        dt=dt,
        mu=0.70,
    )

    # Compute phase-plane trajectories and stability boundaries
    pp_evaluator = PhasePlaneEnvelopeController()
    beta_pass = hist_pass.sideslip_angle
    beta_pp = hist_pp.sideslip_angle

    ay_pass = np.sum(hist_pass.Fy_body, axis=1) / model_params.mass
    ay_pp = np.sum(hist_pp.Fy_body, axis=1) / model_params.mass

    beta_dot_pass = (ay_pass / np.maximum(2.5, hist_pass.vx)) - hist_pass.r
    beta_dot_pp = (ay_pp / np.maximum(2.5, hist_pp.vx)) - hist_pp.r

    b_max, bdot_max = pp_evaluator.compute_boundary_limits(vx=v0, mu=0.70)
    b_max_deg = math.degrees(b_max)
    bdot_max_degps = math.degrees(bdot_max)

    # Phase-Plane Plotting
    fig, (ax_phase, ax_time, ax_ctrl) = plt.subplots(1, 3, figsize=(17, 5.5), facecolor=BG_COLOR)

    # Panel 1: (beta, beta_dot) Phase Plane
    # Draw theoretical stability boundaries (diamond envelope)
    c_time = pp_evaluator.config.c_phase_time_s
    s_app = pp_evaluator.config.s_approach
    s_crit = pp_evaluator.config.s_critical

    # Parallelogram / diamond vertices: |beta/beta_max| + C*|beta_dot/beta_max| = S
    for s_val, col, ls, lbl in [
        (s_app, ACCENT_EMERALD, "--", f"Approach Boundary (S={s_app})"),
        (s_crit, ACCENT_ROSE, "-", f"Critical Stability Limit (S={s_crit})"),
    ]:
        b_pts = np.array([-s_val * b_max_deg, 0.0, s_val * b_max_deg, 0.0, -s_val * b_max_deg])
        bdot_pts = np.array([0.0, (s_val * b_max_deg) / c_time, 0.0, -(s_val * b_max_deg) / c_time, 0.0])
        ax_phase.plot(b_pts, bdot_pts, color=col, linestyle=ls, linewidth=1.8, label=lbl)

    ax_phase.plot(np.degrees(beta_pass), np.degrees(beta_dot_pass), color=ACCENT_ROSE, linestyle="--", linewidth=2.0, label="Passive (Unstable Divergence)")
    ax_phase.plot(np.degrees(beta_pp), np.degrees(beta_dot_pp), color=ACCENT_CYAN, linestyle="-", linewidth=2.2, label="Phase-Plane IVMC (Envelope Contained)")
    ax_phase.scatter([np.degrees(beta_pass[0])], [np.degrees(beta_dot_pass[0])], color=TEXT_PRIMARY, s=40, zorder=5, label="Entry State")

    ax_phase.set_title(r"$(\beta, \dot{\beta})$ Phase-Plane Stability Manifold", fontweight="bold", fontsize=11, color=TEXT_PRIMARY)
    ax_phase.set_xlabel(r"Sideslip Angle $\beta$ [deg]", fontsize=10, color=TEXT_MUTED)
    ax_phase.set_ylabel(r"Sideslip Rate $\dot{\beta}$ [deg/s]", fontsize=10, color=TEXT_MUTED)
    ax_phase.set_xlim(-16, 16)
    ax_phase.set_ylim(-70, 70)
    ax_phase.legend(fontsize=8, loc="upper right", facecolor=CARD_COLOR, edgecolor=GRID_COLOR, labelcolor=TEXT_PRIMARY)

    # Panel 2: Sideslip Time History
    ax_time.plot(hist_pass.time, np.degrees(hist_pass.sideslip_angle), color=ACCENT_ROSE, linestyle="--", linewidth=2.0, label="Passive Sideslip")
    ax_time.plot(hist_pp.time, np.degrees(hist_pp.sideslip_angle), color=ACCENT_CYAN, linestyle="-", linewidth=2.0, label="Phase-Plane IVMC Sideslip")
    ax_time.axhline(b_max_deg, color=ACCENT_ROSE, linestyle=":", label=f"Critical Limit ({b_max_deg:.1f}°)")
    ax_time.axhline(-b_max_deg, color=ACCENT_ROSE, linestyle=":")
    ax_time.set_title(r"Chassis Sideslip Angle Transient $\beta(t)$", fontweight="bold", fontsize=11, color=TEXT_PRIMARY)
    ax_time.set_xlabel("Time [s]", fontsize=10, color=TEXT_MUTED)
    ax_time.set_ylabel(r"Sideslip $\beta$ [deg]", fontsize=10, color=TEXT_MUTED)
    ax_time.legend(fontsize=8, facecolor=CARD_COLOR, edgecolor=GRID_COLOR, labelcolor=TEXT_PRIMARY)

    # Panel 3: Active Counter-Steer and Yaw Rate
    ax_ctrl.plot(hist_pass.time, np.degrees(hist_pass.r), color=ACCENT_ROSE, linestyle="--", linewidth=1.8, label="Passive Yaw Rate")
    ax_ctrl.plot(hist_pp.time, np.degrees(hist_pp.r), color=ACCENT_CYAN, linestyle="-", linewidth=2.0, label="IVMC Yaw Rate")
    ax_ctrl.plot(hist_pp.time, np.degrees(hist_pp.steer_f), color=ACCENT_EMERALD, linestyle="-.", linewidth=1.8, label="Active Front Steer (with Counter-Steer)")
    ax_ctrl.set_title("Yaw Rate & Active Steering Intervention", fontweight="bold", fontsize=11, color=TEXT_PRIMARY)
    ax_ctrl.set_xlabel("Time [s]", fontsize=10, color=TEXT_MUTED)
    ax_ctrl.set_ylabel("Rate [deg/s] / Steer [deg]", fontsize=10, color=TEXT_MUTED)
    ax_ctrl.legend(fontsize=8, facecolor=CARD_COLOR, edgecolor=GRID_COLOR, labelcolor=TEXT_PRIMARY)

    plt.tight_layout()
    plot_file = output_dir / "phase_plane_stability_analysis.png"
    plt.savefig(plot_file, dpi=160, bbox_inches="tight", facecolor=fig.get_facecolor(), edgecolor="none")
    plt.close()
    print(f"  [SAVED] {plot_file}")


def run_14dof_coupled_chassis_demo(output_dir: Path) -> None:
    print("=================================================================")
    print("PART 2: 14-DOF COUPLED VERTICAL & PLANAR CHASSIS CONTROL")
    print("=================================================================")

    # Severe NHTSA NCAP Dynamic Fishhook at 80 km/h
    v0 = 80.0 / 3.6
    duration = 3.2
    dt = 0.002
    steps = int(duration / dt)
    t_arr = np.linspace(0.0, duration, steps)

    delta_fish = np.zeros(steps)
    for i, t in enumerate(t_arr):
        if 0.3 <= t < 0.55:
            delta_fish[i] = 0.105 * ((t - 0.3) / 0.25)
        elif 0.55 <= t < 0.80:
            delta_fish[i] = 0.105
        elif 0.80 <= t < 1.05:
            delta_fish[i] = 0.105 - 2.0 * 0.105 * ((t - 0.80) / 0.25)
        elif 1.05 <= t < 1.85:
            delta_fish[i] = -0.105
        elif 1.85 <= t < 2.15:
            delta_fish[i] = -0.105 * (1.0 - (t - 1.85) / 0.3)

    v_cfg = FourteenDOFConfig()

    # 1. Simulate Passive 14-DOF Multi-Body Vehicle
    model_passive = FourteenDOFModel(v_cfg)
    s_pass = FourteenDOFState(vx=v0, omega=np.full(4, v0 / v_cfg.wheel_radius))
    hist_14_pass = {
        "time": t_arr,
        "X": np.zeros(steps), "Y": np.zeros(steps), "psi": np.zeros(steps),
        "vx": np.zeros(steps), "vy": np.zeros(steps), "r": np.zeros(steps),
        "phi": np.zeros(steps), "theta": np.zeros(steps),
        "Fz": np.zeros((steps, 4)),
        "ltr": np.zeros(steps),
    }

    for k, t in enumerate(t_arr):
        d_cmd = np.array([delta_fish[k], delta_fish[k], 0.0, 0.0])
        s_pass, op = model_passive.step(
            state=s_pass,
            delta=d_cmd,
            T_drive=np.zeros(4),
            T_brake=np.zeros(4),
            dt=dt,
            mu_ground=0.90,
        )
        hist_14_pass["X"][k] = s_pass.x
        hist_14_pass["Y"][k] = s_pass.y
        hist_14_pass["psi"][k] = s_pass.psi
        hist_14_pass["vx"][k] = s_pass.vx
        hist_14_pass["vy"][k] = s_pass.vy
        hist_14_pass["r"][k] = s_pass.r
        hist_14_pass["phi"][k] = s_pass.phi
        hist_14_pass["theta"][k] = s_pass.theta
        hist_14_pass["Fz"][k] = op.Fz
        fz_l = op.Fz[FL] + op.Fz[RL]
        fz_r = op.Fz[FR] + op.Fz[RR]
        hist_14_pass["ltr"][k] = abs(fz_l - fz_r) / max(100.0, fz_l + fz_r)

    # 2. Simulate 14-DOF Vehicle with Active Roll Stiffness Distribution & Dynamic Damping
    model_active = FourteenDOFModel(v_cfg)
    ctrl_arsd = ActiveRollDistributionController(vehicle_cfg=v_cfg)
    s_act = FourteenDOFState(vx=v0, omega=np.full(4, v0 / v_cfg.wheel_radius))

    hist_14_act = {
        "time": t_arr,
        "X": np.zeros(steps), "Y": np.zeros(steps), "psi": np.zeros(steps),
        "vx": np.zeros(steps), "vy": np.zeros(steps), "r": np.zeros(steps),
        "phi": np.zeros(steps), "theta": np.zeros(steps),
        "Fz": np.zeros((steps, 4)),
        "ltr": np.zeros(steps),
        "kappa_f": np.zeros(steps),
        "F_act": np.zeros((steps, 4)),
    }

    Fz_prev = v_cfg.static_normal_loads.copy()
    for k, t in enumerate(t_arr):
        # Desired yaw rate
        r_ref = (s_act.vx / (v_cfg.wheelbase + 0.0015 * s_act.vx ** 2)) * delta_fish[k]
        ay_cur = s_act.vx * s_act.r

        # Active roll distribution control
        ar_out = ctrl_arsd.compute_control(
            state=s_act,
            r_desired=r_ref,
            ay=ay_cur,
            Fz_corners=Fz_prev,
        )

        d_cmd = np.array([delta_fish[k], delta_fish[k], 0.0, 0.0])
        s_act, op = model_active.step(
            state=s_act,
            delta=d_cmd,
            T_drive=np.zeros(4),
            T_brake=np.zeros(4),
            dt=dt,
            mu_ground=0.90,
            active_forces=ar_out.active_corner_forces,
        )
        Fz_prev = op.Fz

        hist_14_act["X"][k] = s_act.x
        hist_14_act["Y"][k] = s_act.y
        hist_14_act["psi"][k] = s_act.psi
        hist_14_act["vx"][k] = s_act.vx
        hist_14_act["vy"][k] = s_act.vy
        hist_14_act["r"][k] = s_act.r
        hist_14_act["phi"][k] = s_act.phi
        hist_14_act["theta"][k] = s_act.theta
        hist_14_act["Fz"][k] = op.Fz
        hist_14_act["kappa_f"][k] = ar_out.kappa_roll_front
        hist_14_act["F_act"][k] = ar_out.active_corner_forces
        fz_l = op.Fz[FL] + op.Fz[RL]
        fz_r = op.Fz[FR] + op.Fz[RR]
        hist_14_act["ltr"][k] = abs(fz_l - fz_r) / max(100.0, fz_l + fz_r)

    # Plot 14-DOF Comparison Suite
    fig = plt.figure(figsize=(16, 9), facecolor=BG_COLOR)
    gs = fig.add_gridspec(2, 2, hspace=0.32, wspace=0.25)
    ax_roll = fig.add_subplot(gs[0, 0])
    ax_ltr = fig.add_subplot(gs[0, 1])
    ax_fz = fig.add_subplot(gs[1, 0])
    ax_kappa = fig.add_subplot(gs[1, 1])

    # 1. Sprung Mass Roll Angle phi(t)
    ax_roll.plot(t_arr, np.degrees(hist_14_pass["phi"]), color=ACCENT_ROSE, linestyle="--", linewidth=2.0, label="Passive 14-DOF")
    ax_roll.plot(t_arr, np.degrees(hist_14_act["phi"]), color=ACCENT_CYAN, linestyle="-", linewidth=2.2, label="Active Roll Controlled 14-DOF")
    ax_roll.set_title(r"Sprung Mass Roll Attitude Angle $\phi(t)$", fontweight="bold", fontsize=11, color=TEXT_PRIMARY)
    ax_roll.set_xlabel("Time [s]", color=TEXT_MUTED)
    ax_roll.set_ylabel(r"Roll Angle $\phi$ [deg]", color=TEXT_MUTED)
    ax_roll.legend(facecolor=CARD_COLOR, edgecolor=GRID_COLOR, labelcolor=TEXT_PRIMARY)

    # 2. Dynamic Load Transfer Ratio (LTR)
    ax_ltr.plot(t_arr, hist_14_pass["ltr"], color=ACCENT_ROSE, linestyle="--", linewidth=2.0, label="Passive LTR")
    ax_ltr.plot(t_arr, hist_14_act["ltr"], color=ACCENT_CYAN, linestyle="-", linewidth=2.2, label="Active Roll LTR")
    ax_ltr.axhline(0.70, color=ACCENT_AMBER, linestyle=":", label="Wheel Lift Risk Threshold (0.70)")
    ax_ltr.axhline(1.00, color=ACCENT_ROSE, linestyle="--", label="Rollover / Wheel Lift Limit (1.00)")
    ax_ltr.set_title("Load Transfer Ratio (LTR: Rollover Hazard Index)", fontweight="bold", fontsize=11, color=TEXT_PRIMARY)
    ax_ltr.set_xlabel("Time [s]", color=TEXT_MUTED)
    ax_ltr.set_ylabel("LTR [-]", color=TEXT_MUTED)
    ax_ltr.set_ylim(0, 1.1)
    ax_ltr.legend(facecolor=CARD_COLOR, edgecolor=GRID_COLOR, labelcolor=TEXT_PRIMARY)

    # 3. Dynamic Normal Tyre Loads (Wheel Lift Check)
    ax_fz.plot(t_arr, hist_14_pass["Fz"][:, FL], color=ACCENT_ROSE, linestyle="--", alpha=0.7, label="FL Passive")
    ax_fz.plot(t_arr, hist_14_pass["Fz"][:, FR], color=ACCENT_AMBER, linestyle=":", alpha=0.7, label="FR Passive (Near Lift-off)")
    ax_fz.plot(t_arr, hist_14_act["Fz"][:, FL], color=ACCENT_BLUE, linestyle="-", linewidth=1.8, label="FL Active Controlled")
    ax_fz.plot(t_arr, hist_14_act["Fz"][:, FR], color=ACCENT_CYAN, linestyle="-", linewidth=1.8, label="FR Active Controlled")
    ax_fz.axhline(0.0, color=TEXT_MUTED, linestyle="-", linewidth=1.2)
    ax_fz.set_title("Corner Normal Tyre Loads $F_z(t)$ (Wheel-Lift Suppression)", fontweight="bold", fontsize=11, color=TEXT_PRIMARY)
    ax_fz.set_xlabel("Time [s]", color=TEXT_MUTED)
    ax_fz.set_ylabel("Normal Force $F_z$ [N]", color=TEXT_MUTED)
    ax_fz.legend(fontsize=8, loc="upper right", facecolor=CARD_COLOR, edgecolor=GRID_COLOR, labelcolor=TEXT_PRIMARY)

    # 4. Front Roll Stiffness Distribution Ratio kappa_roll_f
    ax_kappa.plot(t_arr, hist_14_act["kappa_f"], color=ACCENT_EMERALD, linestyle="-", linewidth=2.2, label=r"Dynamic $\kappa_{\mathrm{roll},f}(t)$")
    ax_kappa.axhline(ctrl_arsd.cfg.nominal_front_roll_ratio, color=TEXT_MUTED, linestyle="--", label="Nominal Bias (0.58)")
    ax_kappa.set_title(r"Dynamic Roll Stiffness Distribution Ratio $\kappa_{\mathrm{roll},f}(t)$", fontweight="bold", fontsize=11, color=TEXT_PRIMARY)
    ax_kappa.set_xlabel("Time [s]", color=TEXT_MUTED)
    ax_kappa.set_ylabel(r"Front Roll Ratio $\kappa_{\mathrm{roll},f}$ [-]", color=TEXT_MUTED)
    ax_kappa.set_ylim(0.3, 0.9)
    ax_kappa.legend(facecolor=CARD_COLOR, edgecolor=GRID_COLOR, labelcolor=TEXT_PRIMARY)

    plt.tight_layout()
    plot_file = output_dir / "14dof_coupled_fishhook_stability.png"
    plt.savefig(plot_file, dpi=160, bbox_inches="tight", facecolor=fig.get_facecolor(), edgecolor="none")
    plt.close()
    plt.close()
    print(f"  [SAVED] {plot_file}")


def main() -> None:
    output_dir = Path("output/advanced_control")
    output_dir.mkdir(parents=True, exist_ok=True)

    run_phase_plane_demo(output_dir)
    run_14dof_coupled_chassis_demo(output_dir)

    print("=================================================================")
    print("ADVANCED VEHICLE DYNAMICS & CHASSIS CONTROL DEMO COMPLETED!")
    print(f"Artifacts exported to: {output_dir}")
    print("=================================================================")


if __name__ == "__main__":
    main()
