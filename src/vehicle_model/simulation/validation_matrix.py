"""Automated Scenario Matrix Runner for Virtual Automotive Validation.

Executes a comprehensive matrix of standardized test maneuvers:
- ISO 3888-2 Double Lane Change
- Split-mu Emergency Braking
- FMVSS 126 Sine with Dwell ESC Yaw Stability
- ISO 7401 Step Steer Transient Response
- ISO 2631 Ride Comfort & Road Holding Bump Traversal

Conforms to Milestone M8 (Plan.md §16, §19 M8).
"""

from __future__ import annotations

import logging
import math
from pathlib import Path
from typing import Any, Dict, List, Tuple

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml

from vehicle_model.controllers import (
    ControlTarget,
    FourWheelSteeringController,
    IntegratedChassisController,
    PassiveBaselineController,
)

from vehicle_model.simulation.handling_limits import (
    HandlingCharacteristics,
    compute_gg_diagram,
    evaluate_understeer_gradient,
    plot_handling_and_limits,
    simulate_steady_state_circular_drive,
)
from vehicle_model.simulation.kpi import EngineeringKPIs, compute_kpis
from vehicle_model.simulation.regression_evaluator import (
    RegressionEvaluator,
    ScenarioEvaluation,
)
from vehicle_model.simulation.scenario_runner import SimulationHistory, simulate_maneuver
from vehicle_model.suspension.controllers import (
    GroundhookController,
    SemiActiveConfig,
    SkyhookController,
)
from vehicle_model.suspension.quarter_car import (
    QuarterCarConfig,
    QuarterCarModel,
    QuarterCarState,
)
from vehicle_model.vehicle_models.seven_dof import SevenDOFModel
from vehicle_model.visualization.plan_view import VehicleParams, draw_vehicle_trajectory
from vehicle_model.visualization.theme import (
    ACCENT_AMBER,
    ACCENT_BLUE,
    ACCENT_CYAN,
    ACCENT_EMERALD,
    ACCENT_ROSE,
    AXES_COLOR,
    BG_COLOR,
    CARD_COLOR,
    GRID_COLOR,
    MU_HIGH_COLOR,
    MU_LOW_COLOR,
    ROAD_COLOR,
    TEXT_MUTED,
    TEXT_PRIMARY,
    apply_dark_theme,
    style_axis,
)

apply_dark_theme()


class ValidationMatrixRunner:
    """Orchestrates multi-scenario automotive test matrix execution."""

    def __init__(
        self,
        config_path: str | Path = "configs/validation/matrix.yaml",
        figures_dir: str | Path = "output/validation_report/figures",
        logger: logging.Logger | None = None,
    ) -> None:
        self.config_path = Path(config_path)
        self.figures_dir = Path(figures_dir)
        self.figures_dir.mkdir(parents=True, exist_ok=True)
        self.logger = logger or logging.getLogger("vehicle_dynamics")

        with open(self.config_path, "r", encoding="utf-8") as f:
            self.matrix_cfg = yaml.safe_load(f)

    @staticmethod
    def _get_standard_controllers() -> Dict[str, Any]:
        return {
            "passive": PassiveBaselineController(),
            "four_wheel_steering": FourWheelSteeringController.from_yaml("configs/steering/four_wheel_steering.yaml", enable_abs=False),
            "ivmc_2ws": IntegratedChassisController.from_yaml("configs/controllers/integrated_controller.yaml", enable_4ws=False),
            "full_ivmc_4ws": IntegratedChassisController.from_yaml("configs/controllers/integrated_controller.yaml", enable_4ws=True),
        }

    @staticmethod
    def _get_standard_palette() -> Dict[str, Tuple[str, str, str]]:
        return {
            "passive": (ACCENT_ROSE, "--", "Passive Baseline"),
            "four_wheel_steering": (ACCENT_CYAN, "-.", "Active 4WS Standalone"),
            "ivmc_2ws": (ACCENT_BLUE, ":", "IVMC 2WS (TV + ESC + ABS)"),
            "full_ivmc_4ws": (ACCENT_EMERALD, "-", "Full-Stack IVMC + 4WS"),
            "integrated_ivmc": (ACCENT_EMERALD, "-", "Integrated IVMC"),
        }

    def _resolve_controllers(self, cfg: Dict[str, Any]) -> Dict[str, Any]:
        available = self._get_standard_controllers()
        c_keys = cfg.get("controllers", ["passive", "four_wheel_steering", "ivmc_2ws", "full_ivmc_4ws"])
        resolved = {}
        for k in c_keys:
            if k == "integrated_ivmc":
                if "ivmc_2ws" not in resolved:
                    resolved["ivmc_2ws"] = available["ivmc_2ws"]
                if "full_ivmc_4ws" not in resolved:
                    resolved["full_ivmc_4ws"] = available["full_ivmc_4ws"]
            elif k in available:
                resolved[k] = available[k]
        return resolved if resolved else available

    def _create_validation_figure(
        self,
        title: str = "",
        controllers: List[str] | None = None,
    ) -> Tuple[plt.Figure, plt.Axes, plt.Axes, plt.Axes, plt.Axes, plt.Axes]:
        fig = plt.figure(figsize=(16, 9.0), facecolor=BG_COLOR)
        gs = fig.add_gridspec(2, 3, height_ratios=[1.15, 1.0], hspace=0.38, wspace=0.22, top=0.83, bottom=0.06)
        ax_track = fig.add_subplot(gs[0, :2])
        ax_detail = fig.add_subplot(gs[0, 2])
        ax2 = fig.add_subplot(gs[1, 0])
        ax3 = fig.add_subplot(gs[1, 1])
        ax4 = fig.add_subplot(gs[1, 2])
        for ax in (ax_track, ax_detail, ax2, ax3, ax4):
            ax.set_facecolor(AXES_COLOR)
            for spine in ax.spines.values():
                spine.set_edgecolor(GRID_COLOR)
                spine.set_linewidth(1.0)
            ax.tick_params(colors=TEXT_MUTED, labelsize=9)
            ax.grid(True, linestyle="--", alpha=0.4, color=GRID_COLOR)

        if title:
            fig.suptitle(title, fontsize=12.5, fontweight="bold", y=0.975, color=TEXT_PRIMARY)

        if controllers:
            palette = self._get_standard_palette()
            handles = []
            labels = []
            for k in controllers:
                if k in palette:
                    col, style, lbl = palette[k]
                    handles.append(plt.Line2D([0], [0], color=col, linestyle=style, lw=2.2))
                    labels.append(lbl)
            if handles:
                fig.legend(
                    handles,
                    labels,
                    loc="center",
                    bbox_to_anchor=(0.50, 0.925),
                    ncol=min(4, len(labels)),
                    fontsize=8.5,
                    facecolor=CARD_COLOR,
                    edgecolor=GRID_COLOR,
                    labelcolor=TEXT_PRIMARY,
                    framealpha=0.95,
                )

        return fig, ax_track, ax_detail, ax2, ax3, ax4

    def _save_figure(self, fig: plt.Figure, filename: str) -> None:
        fig.savefig(
            self.figures_dir / filename,
            dpi=160,
            bbox_inches="tight",
            facecolor=fig.get_facecolor(),
            edgecolor="none",
        )
        plt.close(fig)


    def run_all(self) -> Dict[str, List[ScenarioEvaluation]]:
        """Run all test scenarios in the matrix and evaluate criteria."""
        results: Dict[str, List[ScenarioEvaluation]] = {}

        self.logger.info("=================================================================")
        self.logger.info("STARTING AUTOMOTIVE VIRTUAL VALIDATION PLATFORM MATRIX RUN")
        self.logger.info("=================================================================")

        # 1. ISO 3888-2 Double Lane Change
        results["iso_3888_dlc"] = self.run_iso_3888_dlc()

        # 2. Split-Mu Emergency Braking
        results["mu_split_braking"] = self.run_mu_split_braking()

        # 3. FMVSS 126 Sine with Dwell
        results["sine_with_dwell"] = self.run_sine_with_dwell()

        # 4. ISO 7401 Step Steer
        results["step_steer_iso_7401"] = self.run_step_steer_iso_7401()

        # 5. Ride Comfort Bump Traversal
        results["ride_comfort_bump"] = self.run_ride_comfort_bump()

        # 6. ISO 7975 Braking in a Turn
        results["braking_in_a_turn"] = self.run_braking_in_a_turn()

        # 7. NHTSA Dynamic Fishhook
        results["fishhook_ncap"] = self.run_fishhook_ncap()

        # 8. Low-Speed Slalom (4WS Agility)
        results["low_speed_slalom_4ws"] = self.run_low_speed_slalom_4ws()

        # 9. High-Speed Highway Lane Change (4WS Stability)
        results["high_speed_lane_change"] = self.run_high_speed_lane_change()

        # 10. ISO 4138 Steady-State Circular Drive (Handling Limits & Understeer Gradient)
        if "iso_4138_circle" in self.matrix_cfg.get("scenarios", {}):
            results["iso_4138_circle"] = self.run_iso_4138_circle()

        self.logger.info("=================================================================")
        self.logger.info("VALIDATION MATRIX EXECUTION COMPLETED")
        self.logger.info("=================================================================")

        return results

    # =========================================================================
    # SCENARIO 1: ISO 3888-2 DOUBLE LANE CHANGE
    # =========================================================================
    def run_iso_3888_dlc(self) -> List[ScenarioEvaluation]:
        cfg = self.matrix_cfg["scenarios"]["iso_3888_dlc"]
        self.logger.info(f"Running Scenario: {cfg['name']}")

        v0 = cfg["initial_speed_kph"] / 3.6
        duration = float(cfg["duration_s"])
        dt = 0.005
        steps = int(duration / dt)
        t_arr = np.linspace(0.0, duration, steps)

        # Steering profile for DLC (double steer input)
        delta_driver = np.zeros(steps)
        for i, t in enumerate(t_arr):
            if 0.5 <= t <= 1.5:
                delta_driver[i] = 0.115 * math.sin(2.0 * math.pi * 1.0 * (t - 0.5))
            elif 2.0 <= t <= 3.0:
                delta_driver[i] = -0.105 * math.sin(2.0 * math.pi * 1.0 * (t - 2.0))

        controllers = self._resolve_controllers(cfg)
        palette = self._get_standard_palette()

        evaluations: List[ScenarioEvaluation] = []
        histories: Dict[str, SimulationHistory] = {}

        for c_key, c_inst in controllers.items():
            model = SevenDOFModel.from_yaml("configs/vehicles/seven_dof_passenger.yaml")
            hist = simulate_maneuver(
                model=model,
                controller=c_inst,
                v_initial=v0,
                delta_driver=delta_driver,
                T_drive_demand=0.0,
                T_brake_demand=0.0,
                duration=duration,
                dt=dt,
                mu=float(cfg["road_mu"]),
            )
            histories[c_key] = hist
            c_name = palette.get(c_key, (None, None, c_key.replace("_", " ").title()))[2]
            kpis = compute_kpis(hist, controller_name=c_name)
            eval_res = RegressionEvaluator.evaluate(
                scenario_key="iso_3888_dlc",
                scenario_name=cfg["name"],
                controller_name=c_name,
                kpis=kpis,
                thresholds=cfg["thresholds"],
            )
            evaluations.append(eval_res)
            self.logger.info(
                f"  [{eval_res.status_badge}] {eval_res.controller_name}: "
                f"Sideslip={kpis.max_sideslip_deg:.2f}°, LatError={kpis.max_lateral_error_m:.2f}m"
            )

        # Plot comparison with prominent 2D planar vehicle footprints (3-Panel Non-Colliding Layout)
        fig, ax_track, ax_detail, ax2, ax3, ax4 = self._create_validation_figure(
            title=cfg["name"],
            controllers=list(histories.keys()),
        )

        v_params = VehicleParams.from_seven_dof_params(model.params)

        # Road corridor backdrop
        ax_track.axhline(0.0, color=TEXT_MUTED, linestyle=":", alpha=0.6)
        ax_track.axhline(2.0, color="#475569", linestyle="--", alpha=0.7, label="Lane 2 Center")
        ax_track.fill_between([-5, 105], -4.0, 5.0, color=ROAD_COLOR, zorder=0)

        # Draw ISO 3888-2 cone pylons (aligned with trajectory corridor)
        gate_x = [0.0, 12.0, 12.0, 0.0, 25.5, 36.5, 36.5, 25.5, 49.0, 61.0, 61.0, 49.0]
        gate_y = [-1.2, -1.2, 1.2, 1.2, 0.8, 0.8, 3.2, 3.2, -1.3, -1.3, 1.3, 1.3]
        ax_track.scatter(gate_x, gate_y, color=ACCENT_AMBER, s=32, zorder=4, label="ISO 3888 Pylons")
        ax_detail.scatter(gate_x, gate_y, color=ACCENT_AMBER, s=40, zorder=4)

        for c_key, hist in histories.items():
            col, style, name = palette.get(c_key, (ACCENT_CYAN, "-", c_key.replace("_", " ").title()))
            draw_vehicle_trajectory(
                ax=ax_track,
                X=hist.X,
                Y=hist.Y,
                psi=hist.psi,
                steer_f=hist.steer_f,
                steer_r=hist.steer_r,
                params=v_params,
                n_snapshots=6,
                color=col,
                path_style=style,
                label=name,
                alpha=0.85,
                scale=1.0,
            )
            # Detail panel: 1:1 true scale close-up of obstacle avoidance apex
            draw_vehicle_trajectory(
                ax=ax_detail,
                X=hist.X,
                Y=hist.Y,
                psi=hist.psi,
                steer_f=hist.steer_f,
                steer_r=hist.steer_r,
                params=v_params,
                n_snapshots=4,
                color=col,
                path_style=style,
                label=name,
                alpha=0.9,
                scale=1.0,
            )
            ax2.plot(hist.time, np.degrees(hist.r), label=name, color=col, linestyle=style, linewidth=2.0)
            ax3.plot(hist.time, np.degrees(np.abs(hist.sideslip_angle)), label=name, color=col, linestyle=style, linewidth=2.0)
            ay_g = np.sum(hist.Fy_body, axis=1) / model.params.mass / 9.81
            ax4.plot(hist.time, ay_g, label=name, color=col, linestyle=style, linewidth=2.0)

        ax_track.set_title("ISO 3888-2 Double Lane Change Trajectory (Full Run)", fontsize=11, fontweight="bold", color=TEXT_PRIMARY)
        ax_track.set_xlabel("X Position [m]", fontsize=9, color=TEXT_MUTED)
        ax_track.set_ylabel("Y Position [m]", fontsize=9, color=TEXT_MUTED)
        ax_track.set_xlim(-5, 105)
        ax_track.set_ylim(-4.0, 5.0)
        ax_track.set_aspect("equal", adjustable="datalim")
        ax_track.grid(True, linestyle="--", alpha=0.4, color=GRID_COLOR)
        ax_track.legend(loc="upper left", fontsize=8, facecolor=CARD_COLOR, edgecolor=GRID_COLOR, labelcolor=TEXT_PRIMARY, framealpha=0.92)

        # Detail panel configuration
        ax_detail.fill_between([20, 65], -2.0, 4.0, color=ROAD_COLOR, zorder=0)
        ax_detail.set_xlim(25, 55)
        ax_detail.set_ylim(-1.5, 3.5)
        ax_detail.set_aspect("equal")
        ax_detail.set_title("Obstacle Avoidance Apex (1:1 Scale)", fontsize=11, fontweight="bold", color=TEXT_PRIMARY)
        ax_detail.set_xlabel("X Position [m]", fontsize=9, color=TEXT_MUTED)
        ax_detail.grid(True, linestyle=":", alpha=0.5, color=GRID_COLOR)

        ax2.set_title(r"Chassis Yaw Rate $r$", fontweight="bold", fontsize=10, color=TEXT_PRIMARY)
        ax2.set_xlabel("Time [s]", color=TEXT_MUTED)
        ax2.set_ylabel("Yaw Rate [deg/s]", color=TEXT_MUTED)
        ax2.legend(fontsize=8, facecolor=CARD_COLOR, edgecolor=GRID_COLOR, labelcolor=TEXT_PRIMARY)

        ax3.set_title(r"Sideslip Angle Magnitude $|\beta|$", fontweight="bold", fontsize=10, color=TEXT_PRIMARY)
        ax3.set_xlabel("Time [s]", color=TEXT_MUTED)
        ax3.set_ylabel(r"$|\beta|$ [deg]", color=TEXT_MUTED)
        ax3.legend(fontsize=8, facecolor=CARD_COLOR, edgecolor=GRID_COLOR, labelcolor=TEXT_PRIMARY)

        ax4.set_title(r"Lateral Acceleration $a_y$", fontweight="bold", fontsize=10, color=TEXT_PRIMARY)
        ax4.set_xlabel("Time [s]", color=TEXT_MUTED)
        ax4.set_ylabel("Lateral Accel [g]", color=TEXT_MUTED)
        ax4.legend(fontsize=8, facecolor=CARD_COLOR, edgecolor=GRID_COLOR, labelcolor=TEXT_PRIMARY)

        self._save_figure(fig, "iso_3888_dlc.png")

        return evaluations

    # =========================================================================
    # SCENARIO 2: SPLIT-MU STRAIGHT-LINE EMERGENCY BRAKING
    # =========================================================================
    def run_mu_split_braking(self) -> List[ScenarioEvaluation]:
        cfg = self.matrix_cfg["scenarios"]["mu_split_braking"]
        self.logger.info(f"Running Scenario: {cfg['name']}")

        v0 = cfg["initial_speed_kph"] / 3.6
        duration = float(cfg["duration_s"])
        dt = 0.005
        steps = int(duration / dt)

        # Split road friction: Left = 1.0, Right = 0.20
        mu_split = np.array([cfg["road_mu_left"], cfg["road_mu_right"], cfg["road_mu_left"], cfg["road_mu_right"]])

        controllers = self._resolve_controllers(cfg)
        palette = self._get_standard_palette()

        evaluations: List[ScenarioEvaluation] = []
        histories: Dict[str, SimulationHistory] = {}

        for c_key, c_inst in controllers.items():
            model = SevenDOFModel.from_yaml("configs/vehicles/seven_dof_passenger.yaml")
            hist = simulate_maneuver(
                model=model,
                controller=c_inst,
                v_initial=v0,
                delta_driver=np.zeros(steps),
                T_drive_demand=0.0,
                T_brake_demand=float(cfg["brake_torque_nm"]),
                duration=duration,
                dt=dt,
                mu=mu_split,
            )
            histories[c_key] = hist
            c_name = palette.get(c_key, (None, None, c_key.replace("_", " ").title()))[2]
            kpis = compute_kpis(hist, controller_name=c_name)
            eval_res = RegressionEvaluator.evaluate(
                scenario_key="mu_split_braking",
                scenario_name=cfg["name"],
                controller_name=c_name,
                kpis=kpis,
                thresholds=cfg["thresholds"],
            )
            evaluations.append(eval_res)
            stop_str = f"{kpis.stopping_distance_m:.1f}m" if kpis.stopping_distance_m else "N/A"
            self.logger.info(
                f"  [{eval_res.status_badge}] {eval_res.controller_name}: "
                f"HeadingDev={kpis.final_heading_dev_deg:.1f}°, StopDist={stop_str}"
            )

        # Plot comparison with prominent 2D planar vehicle footprints (3-Panel Non-Colliding Layout)
        fig, ax_track, ax_detail, ax2, ax3, ax4 = self._create_validation_figure(
            title=cfg["name"],
            controllers=list(histories.keys()),
        )

        v_params = VehicleParams.from_seven_dof_params(model.params)
        x_max_val = max(np.max(h.X) for h in histories.values()) + 10.0
        ax_track.axhline(0.0, color=TEXT_MUTED, linestyle=":", lw=1.2, label="Split Line (Y=0)")
        ax_track.fill_between([0, x_max_val], 0, 8, color=MU_HIGH_COLOR, alpha=0.6, label=r"Left High $\mu=1.0$")
        ax_track.fill_between([0, x_max_val], -8, 0, color=MU_LOW_COLOR, alpha=0.6, label=r"Right Low $\mu=0.2$ (Ice)")

        for c_key, hist in histories.items():
            col, style, name = palette.get(c_key, (ACCENT_CYAN, "-", c_key.replace("_", " ").title()))
            draw_vehicle_trajectory(
                ax=ax_track,
                X=hist.X,
                Y=hist.Y,
                psi=hist.psi,
                steer_f=hist.steer_f,
                steer_r=hist.steer_r,
                params=v_params,
                n_snapshots=6,
                color=col,
                path_style=style,
                label=name,
                alpha=0.85,
                scale=1.0,
            )
            # Detail panel: 1:1 true scale close-up of spinout vs stabilized poses
            draw_vehicle_trajectory(
                ax=ax_detail,
                X=hist.X,
                Y=hist.Y,
                psi=hist.psi,
                steer_f=hist.steer_f,
                steer_r=hist.steer_r,
                params=v_params,
                n_snapshots=4,
                color=col,
                path_style=style,
                label=name,
                alpha=0.9,
                scale=1.0,
            )
            ax2.plot(hist.time, np.degrees(hist.psi), label=name, color=col, linestyle=style, linewidth=2.0)
            ax3.plot(hist.time, np.degrees(hist.r), label=name, color=col, linestyle=style, linewidth=2.0)
            ax4.plot(hist.time, hist.vx * 3.6, label=name, color=col, linestyle=style, linewidth=2.0)

        ax_track.set_title(r"Split-$\mu$ Emergency Braking Trajectory (Full Run)", fontsize=11, fontweight="bold", color=TEXT_PRIMARY)
        ax_track.set_xlabel("X Position [m]", fontsize=9, color=TEXT_MUTED)
        ax_track.set_ylabel("Y Deviation [m]", fontsize=9, color=TEXT_MUTED)
        ax_track.set_xlim(-5, x_max_val)
        ax_track.set_aspect("equal", adjustable="datalim")
        ax_track.grid(True, linestyle="--", alpha=0.4, color=GRID_COLOR)
        ax_track.legend(loc="upper left", fontsize=8, facecolor=CARD_COLOR, edgecolor=GRID_COLOR, labelcolor=TEXT_PRIMARY, framealpha=0.92)

        # Detail panel configuration
        ax_detail.axhline(0.0, color="#475569", linestyle=":", lw=1.0)
        ax_detail.fill_between([x_max_val - 35.0, x_max_val], 0, 8, color=MU_HIGH_COLOR, alpha=0.6)
        ax_detail.fill_between([x_max_val - 35.0, x_max_val], -8, 0, color=MU_LOW_COLOR, alpha=0.6)
        ax_detail.set_xlim(x_max_val - 35.0, x_max_val)
        ax_detail.set_ylim(-5.0, 7.0)
        ax_detail.set_aspect("equal")
        ax_detail.set_title("Spinout vs Stabilized Pose (1:1 Scale)", fontsize=11, fontweight="bold", color=TEXT_PRIMARY)
        ax_detail.set_xlabel("X Position [m]", fontsize=9, color=TEXT_MUTED)
        ax_detail.grid(True, linestyle=":", alpha=0.5, color=GRID_COLOR)

        ax2.set_title(r"Chassis Heading Deviation $\psi$", fontweight="bold", fontsize=10, color=TEXT_PRIMARY)
        ax2.set_xlabel("Time [s]", color=TEXT_MUTED)
        ax2.set_ylabel(r"Heading Angle $\psi$ [deg]", color=TEXT_MUTED)
        ax2.legend(fontsize=8, facecolor=CARD_COLOR, edgecolor=GRID_COLOR, labelcolor=TEXT_PRIMARY)

        ax3.set_title(r"Chassis Yaw Rate $r$", fontweight="bold", fontsize=10, color=TEXT_PRIMARY)
        ax3.set_xlabel("Time [s]", color=TEXT_MUTED)
        ax3.set_ylabel("Yaw Rate [deg/s]", color=TEXT_MUTED)
        ax3.legend(fontsize=8, facecolor=CARD_COLOR, edgecolor=GRID_COLOR, labelcolor=TEXT_PRIMARY)

        ax4.set_title(r"Vehicle Speed $v_x$ (Braking Deceleration)", fontweight="bold", fontsize=10, color=TEXT_PRIMARY)
        ax4.set_xlabel("Time [s]", color=TEXT_MUTED)
        ax4.set_ylabel("Speed $v_x$ [km/h]", color=TEXT_MUTED)
        ax4.legend(fontsize=8, facecolor=CARD_COLOR, edgecolor=GRID_COLOR, labelcolor=TEXT_PRIMARY)

        self._save_figure(fig, "mu_split_braking.png")

        return evaluations

    # =========================================================================
    # SCENARIO 3: FMVSS 126 SINE WITH DWELL
    # =========================================================================
    def run_sine_with_dwell(self) -> List[ScenarioEvaluation]:
        cfg = self.matrix_cfg["scenarios"]["sine_with_dwell"]
        self.logger.info(f"Running Scenario: {cfg['name']}")

        v0 = cfg["initial_speed_kph"] / 3.6
        duration = float(cfg["duration_s"])
        dt = 0.005
        steps = int(duration / dt)
        t_arr = np.linspace(0.0, duration, steps)

        # Build Sine with Dwell profile:
        # Sine wave at 0.7 Hz with 0.5s pause (dwell) at the peak of the second half-cycle
        f_hz = float(cfg["steering_freq_hz"])
        t_period = 1.0 / f_hz
        t_dwell = float(cfg["dwell_duration_s"])
        peak_steer = math.radians(float(cfg["peak_steer_deg"]))

        delta_driver = np.zeros(steps)
        t_steer_start = 0.5
        t_half = t_steer_start + 0.5 * t_period
        t_dwell_end = t_half + 0.25 * t_period + t_dwell
        t_steer_end = t_dwell_end + 0.25 * t_period

        for i, t in enumerate(t_arr):
            if t_steer_start <= t < t_half:
                # First half-cycle
                tau = (t - t_steer_start) * f_hz
                delta_driver[i] = peak_steer * math.sin(2.0 * math.pi * tau)
            elif t_half <= t < t_half + 0.25 * t_period:
                # Rise to opposite peak
                tau = (t - t_steer_start) * f_hz
                delta_driver[i] = peak_steer * math.sin(2.0 * math.pi * tau)
            elif t_half + 0.25 * t_period <= t < t_dwell_end:
                # Dwell at negative peak
                delta_driver[i] = -peak_steer
            elif t_dwell_end <= t < t_steer_end:
                # Return to zero
                tau_return = (t - t_dwell_end) / (0.25 * t_period)
                delta_driver[i] = -peak_steer * math.cos(0.5 * math.pi * tau_return)

        controllers = self._resolve_controllers(cfg)
        palette = self._get_standard_palette()

        evaluations: List[ScenarioEvaluation] = []
        histories: Dict[str, SimulationHistory] = {}

        for c_key, c_inst in controllers.items():
            model = SevenDOFModel.from_yaml("configs/vehicles/seven_dof_passenger.yaml")
            hist = simulate_maneuver(
                model=model,
                controller=c_inst,
                v_initial=v0,
                delta_driver=delta_driver,
                T_drive_demand=0.0,
                T_brake_demand=0.0,
                duration=duration,
                dt=dt,
                mu=float(cfg["road_mu"]),
            )
            histories[c_key] = hist
            c_name = palette.get(c_key, (None, None, c_key.replace("_", " ").title()))[2]
            kpis = compute_kpis(hist, controller_name=c_name)

            # Compute FMVSS 126 yaw rate ratio at 1.0s after completion of steer
            idx_end = np.argmin(np.abs(hist.time - t_steer_end))
            idx_1s = np.argmin(np.abs(hist.time - (t_steer_end + 1.0)))

            peak_yaw = float(np.max(np.abs(hist.r)))
            yaw_at_1s = float(abs(hist.r[idx_1s]))
            ratio_1s_pct = (yaw_at_1s / max(peak_yaw, 1e-4)) * 100.0

            custom_metrics = {"yaw_rate_ratio_1s_pct": ratio_1s_pct}

            eval_res = RegressionEvaluator.evaluate(
                scenario_key="sine_with_dwell",
                scenario_name=cfg["name"],
                controller_name=c_name,
                kpis=kpis,
                thresholds=cfg["thresholds"],
                custom_metrics=custom_metrics,
            )
            evaluations.append(eval_res)
            self.logger.info(
                f"  [{eval_res.status_badge}] {eval_res.controller_name}: "
                f"YawRateRatio@1s={ratio_1s_pct:.1f}%, Sideslip={kpis.max_sideslip_deg:.2f}°"
            )

        # Plot comparison with prominent 2D planar vehicle footprints (3-Panel Non-Colliding Layout)
        fig, ax_track, ax_detail, ax2, ax3, ax4 = self._create_validation_figure(
            title=cfg["name"],
            controllers=list(histories.keys()),
        )

        v_params = VehicleParams.from_seven_dof_params(model.params)

        # Road boundary corridor
        ax_track.axhline(0.0, color=TEXT_MUTED, linestyle=":", alpha=0.6)
        ax_track.fill_between([-5, 85], -12.0, 10.0, color=ROAD_COLOR, zorder=0)

        for c_key, hist in histories.items():
            col, style, name = palette.get(c_key, (ACCENT_CYAN, "-", c_key.replace("_", " ").title()))
            draw_vehicle_trajectory(
                ax=ax_track,
                X=hist.X,
                Y=hist.Y,
                psi=hist.psi,
                steer_f=hist.steer_f,
                steer_r=hist.steer_r,
                params=v_params,
                n_snapshots=7,
                color=col,
                path_style=style,
                label=name,
                alpha=0.85,
                scale=1.0,
            )
            # Detail panel: 1:1 true scale close-up of spinout vs stabilized poses
            draw_vehicle_trajectory(
                ax=ax_detail,
                X=hist.X,
                Y=hist.Y,
                psi=hist.psi,
                steer_f=hist.steer_f,
                steer_r=hist.steer_r,
                params=v_params,
                n_snapshots=5,
                color=col,
                path_style=style,
                label=name,
                alpha=0.9,
                scale=1.0,
            )
            ax2.plot(hist.time, np.degrees(hist.r), label=f"{name} Yaw Rate", color=col, linestyle=style, linewidth=2.0)
            ax3.plot(hist.time, np.degrees(hist.sideslip_angle), label=name, color=col, linestyle=style, linewidth=2.0)

        ax_track.set_title("FMVSS 126 Sine with Dwell Trajectory (Full Run)", fontsize=11, fontweight="bold", color=TEXT_PRIMARY)
        ax_track.set_xlabel("X Position [m]", fontsize=9, color=TEXT_MUTED)
        ax_track.set_ylabel("Y Position [m]", fontsize=9, color=TEXT_MUTED)
        ax_track.set_xlim(-5, 85)
        ax_track.set_aspect("equal", adjustable="datalim")
        ax_track.grid(True, linestyle="--", alpha=0.4, color=GRID_COLOR)
        ax_track.legend(loc="upper left", fontsize=8, facecolor=CARD_COLOR, edgecolor=GRID_COLOR, labelcolor=TEXT_PRIMARY, framealpha=0.92)

        # Detail panel configuration
        ax_detail.fill_between([45, 80], -9.0, 6.0, color=ROAD_COLOR, zorder=0)
        ax_detail.set_xlim(45, 80)
        ax_detail.set_ylim(-8.0, 6.0)
        ax_detail.set_aspect("equal")
        ax_detail.set_title("Spinout vs Stabilized Pose (1:1 Scale)", fontsize=11, fontweight="bold", color=TEXT_PRIMARY)
        ax_detail.set_xlabel("X Position [m]", fontsize=9, color=TEXT_MUTED)
        ax_detail.grid(True, linestyle=":", alpha=0.5, color=GRID_COLOR)

        ax2.axvline(t_steer_end, color=TEXT_MUTED, linestyle=":", label="Steer Completion")
        ax2.axvline(t_steer_end + 1.0, color=ACCENT_ROSE, linestyle=":", label="1.0s Evaluation Point")
        ax2.set_title("FMVSS 126 Sine with Dwell Yaw Rate", fontweight="bold", fontsize=10, color=TEXT_PRIMARY)
        ax2.set_xlabel("Time [s]", color=TEXT_MUTED)
        ax2.set_ylabel("Yaw Rate $r$ [deg/s]", color=TEXT_MUTED)
        ax2.legend(loc="upper right", fontsize=8, facecolor=CARD_COLOR, edgecolor=GRID_COLOR, labelcolor=TEXT_PRIMARY)

        ax3.set_title(r"Sideslip Angle Response $\beta$", fontweight="bold", fontsize=10, color=TEXT_PRIMARY)
        ax3.set_xlabel("Time [s]", color=TEXT_MUTED)
        ax3.set_ylabel(r"Sideslip $\beta$ [deg]", color=TEXT_MUTED)
        ax3.legend(loc="upper right", fontsize=8, facecolor=CARD_COLOR, edgecolor=GRID_COLOR, labelcolor=TEXT_PRIMARY)

        ax4.plot(t_arr, np.degrees(delta_driver), color=ACCENT_CYAN, linestyle="-", lw=1.8, label=r"Steer Demand $\delta_d$")
        ax4.axvline(t_steer_end, color=TEXT_MUTED, linestyle=":", label="Steer End")
        ax4.axvline(t_steer_end + 1.0, color=ACCENT_ROSE, linestyle=":", label="+1.0s Check")
        ax4.set_title(r"Driver Steering Input $\delta_d$", fontweight="bold", fontsize=10, color=TEXT_PRIMARY)
        ax4.set_xlabel("Time [s]", color=TEXT_MUTED)
        ax4.set_ylabel(r"Steer Angle [deg]", color=TEXT_MUTED)
        ax4.legend(loc="upper right", fontsize=8, facecolor=CARD_COLOR, edgecolor=GRID_COLOR, labelcolor=TEXT_PRIMARY)

        self._save_figure(fig, "sine_with_dwell.png")

        return evaluations

    # =========================================================================
    # SCENARIO 4: ISO 7401 STEP STEER RESPONSE
    # =========================================================================
    def run_step_steer_iso_7401(self) -> List[ScenarioEvaluation]:
        cfg = self.matrix_cfg["scenarios"]["step_steer_iso_7401"]
        self.logger.info(f"Running Scenario: {cfg['name']}")

        v0 = cfg["initial_speed_kph"] / 3.6
        duration = float(cfg["duration_s"])
        dt = 0.005
        steps = int(duration / dt)
        t_arr = np.linspace(0.0, duration, steps)

        t_step = float(cfg["step_time_s"])
        step_steer = math.radians(float(cfg["step_steer_deg"]))

        delta_driver = np.zeros(steps)
        delta_driver[t_arr >= t_step] = step_steer

        controllers = self._resolve_controllers(cfg)
        palette = self._get_standard_palette()

        evaluations: List[ScenarioEvaluation] = []
        histories: Dict[str, SimulationHistory] = {}

        for c_key, c_inst in controllers.items():
            model = SevenDOFModel.from_yaml("configs/vehicles/seven_dof_passenger.yaml")
            hist = simulate_maneuver(
                model=model,
                controller=c_inst,
                v_initial=v0,
                delta_driver=delta_driver,
                T_drive_demand=0.0,
                T_brake_demand=0.0,
                duration=duration,
                dt=dt,
                mu=float(cfg["road_mu"]),
            )
            histories[c_key] = hist
            c_name = palette.get(c_key, (None, None, c_key.replace("_", " ").title()))[2]
            kpis = compute_kpis(hist, controller_name=c_name)
            eval_res = RegressionEvaluator.evaluate(
                scenario_key="step_steer_iso_7401",
                scenario_name=cfg["name"],
                controller_name=c_name,
                kpis=kpis,
                thresholds=cfg["thresholds"],
            )
            evaluations.append(eval_res)
            self.logger.info(
                f"  [{eval_res.status_badge}] {eval_res.controller_name}: "
                f"PeakYaw={math.degrees(kpis.max_yaw_rate_radps):.1f}°/s, Sideslip={kpis.max_sideslip_deg:.2f}°"
            )

        # Plot comparison with prominent 2D planar vehicle footprints (3-Panel Non-Colliding Layout)
        fig, ax_track, ax_detail, ax2, ax3, ax4 = self._create_validation_figure(
            title=cfg["name"],
            controllers=list(histories.keys()),
        )

        v_params = VehicleParams.from_seven_dof_params(model.params)

        # Road boundary corridor
        ax_track.axhline(0.0, color=TEXT_MUTED, linestyle=":", alpha=0.6)
        ax_track.fill_between([-5, 80], -5.0, 32.0, color=ROAD_COLOR, zorder=0)

        for c_key, hist in histories.items():
            col, style, name = palette.get(c_key, (ACCENT_CYAN, "-", c_key.replace("_", " ").title()))
            draw_vehicle_trajectory(
                ax=ax_track,
                X=hist.X,
                Y=hist.Y,
                psi=hist.psi,
                steer_f=hist.steer_f,
                steer_r=hist.steer_r,
                params=v_params,
                n_snapshots=6,
                color=col,
                path_style=style,
                label=name,
                alpha=0.85,
                scale=1.0,
            )
            # Detail panel: 1:1 true scale close-up of steady-state cornering arc
            draw_vehicle_trajectory(
                ax=ax_detail,
                X=hist.X,
                Y=hist.Y,
                psi=hist.psi,
                steer_f=hist.steer_f,
                steer_r=hist.steer_r,
                params=v_params,
                n_snapshots=4,
                color=col,
                path_style=style,
                label=name,
                alpha=0.9,
                scale=1.0,
            )
            ax2.plot(hist.time, np.degrees(hist.r), label=name, color=col, linestyle=style, linewidth=2.0)
            ax3.plot(hist.time, np.degrees(np.abs(hist.sideslip_angle)), label=name, color=col, linestyle=style, linewidth=2.0)
            ay_g = np.sum(hist.Fy_body, axis=1) / model.params.mass / 9.81
            ax4.plot(hist.time, ay_g, label=name, color=col, linestyle=style, linewidth=2.0)

        ax_track.set_title("ISO 7401 Step Steer Trajectory (Full Run)", fontsize=11, fontweight="bold", color=TEXT_PRIMARY)
        ax_track.set_xlabel("X Position [m]", fontsize=9, color=TEXT_MUTED)
        ax_track.set_ylabel("Y Position [m]", fontsize=9, color=TEXT_MUTED)
        ax_track.set_xlim(-5, 80)
        ax_track.set_aspect("equal", adjustable="datalim")
        ax_track.grid(True, linestyle="--", alpha=0.4, color=GRID_COLOR)
        ax_track.legend(loc="upper left", fontsize=8, facecolor=CARD_COLOR, edgecolor=GRID_COLOR, labelcolor=TEXT_PRIMARY, framealpha=0.92)

        # Detail panel configuration
        ax_detail.fill_between([45, 76], 8.0, 28.0, color=ROAD_COLOR, zorder=0)
        ax_detail.set_xlim(45, 76)
        ax_detail.set_ylim(8.0, 28.0)
        ax_detail.set_aspect("equal")
        ax_detail.set_title("Steady-State Cornering Arc (1:1 Scale)", fontsize=11, fontweight="bold", color=TEXT_PRIMARY)
        ax_detail.set_xlabel("X Position [m]", fontsize=9, color=TEXT_MUTED)
        ax_detail.grid(True, linestyle=":", alpha=0.5, color=GRID_COLOR)

        ax2.set_title(r"Transient Yaw Rate Response $r$", fontweight="bold", fontsize=10, color=TEXT_PRIMARY)
        ax2.set_xlabel("Time [s]", color=TEXT_MUTED)
        ax2.set_ylabel("Yaw Rate [deg/s]", color=TEXT_MUTED)
        ax2.legend(fontsize=8, facecolor=CARD_COLOR, edgecolor=GRID_COLOR, labelcolor=TEXT_PRIMARY)

        ax3.set_title(r"Sideslip Angle Transient Response $|\beta|$", fontweight="bold", fontsize=10, color=TEXT_PRIMARY)
        ax3.set_xlabel("Time [s]", color=TEXT_MUTED)
        ax3.set_ylabel(r"$|\beta|$ [deg]", color=TEXT_MUTED)
        ax3.legend(fontsize=8, facecolor=CARD_COLOR, edgecolor=GRID_COLOR, labelcolor=TEXT_PRIMARY)

        ax4.set_title(r"Lateral Acceleration $a_y$", fontweight="bold", fontsize=10, color=TEXT_PRIMARY)
        ax4.set_xlabel("Time [s]", color=TEXT_MUTED)
        ax4.set_ylabel("Lateral Accel [g]", color=TEXT_MUTED)
        ax4.legend(fontsize=8, facecolor=CARD_COLOR, edgecolor=GRID_COLOR, labelcolor=TEXT_PRIMARY)

        self._save_figure(fig, "step_steer_iso_7401.png")

        return evaluations

    # =========================================================================
    # SCENARIO 5: ISO 2631 RIDE COMFORT BUMP TRAVERSAL
    # =========================================================================
    def run_ride_comfort_bump(self) -> List[ScenarioEvaluation]:
        cfg = self.matrix_cfg["scenarios"]["ride_comfort_bump"]
        self.logger.info(f"Running Scenario: {cfg['name']}")

        qc_cfg = QuarterCarConfig.from_yaml("configs/suspension/quarter_car.yaml")
        qc_model = QuarterCarModel(qc_cfg)

        semi_cfg = SemiActiveConfig.from_yaml("configs/suspension/semi_active.yaml")
        skyhook = SkyhookController(config=semi_cfg, continuous=True)
        groundhook = GroundhookController(config=semi_cfg, continuous=True)

        dt = 0.001
        t_end = 2.0
        steps = int(t_end / dt)
        t_arr = np.linspace(0.0, t_end, steps)

        bump_h = float(cfg["bump_height_m"])
        bump_dur = float(cfg["bump_duration_s"])

        def road_bump(t: float):
            if 0.2 <= t <= 0.2 + bump_dur:
                tau = (t - 0.2) / bump_dur
                z = 0.5 * bump_h * (1.0 - math.cos(2.0 * math.pi * tau))
                dz = 0.5 * bump_h * (2.0 * math.pi / bump_dur) * math.sin(2.0 * math.pi * tau)
                return z, dz
            return 0.0, 0.0

        hist_pass = {"ddzs": [], "Fz_dyn": []}
        hist_sky = {"ddzs": [], "Fz_dyn": []}
        hist_gh = {"ddzs": [], "Fz_dyn": []}

        s_pass, s_sky, s_gh = QuarterCarState(), QuarterCarState(), QuarterCarState()

        for t in t_arr:
            rz, rdz = road_bump(t)
            s_pass, op = qc_model.step(s_pass, rz, rdz, dt, damper_c=qc_cfg.suspension_damping)
            hist_pass["ddzs"].append(op.ddzs)
            hist_pass["Fz_dyn"].append(op.dynamic_tyre_load)

            c_sky = skyhook.compute_damping(s_sky.dzs, s_sky.dzu, dt)
            s_sky, os_ = qc_model.step(s_sky, rz, rdz, dt, damper_c=c_sky)
            hist_sky["ddzs"].append(os_.ddzs)
            hist_sky["Fz_dyn"].append(os_.dynamic_tyre_load)

            c_gh = groundhook.compute_damping(s_gh.dzs, s_gh.dzu, dt)
            s_gh, og = qc_model.step(s_gh, rz, rdz, dt, damper_c=c_gh)
            hist_gh["ddzs"].append(og.ddzs)
            hist_gh["Fz_dyn"].append(og.dynamic_tyre_load)

        rms_acc_pass = float(np.sqrt(np.mean(np.square(hist_pass["ddzs"]))))
        rms_acc_sky = float(np.sqrt(np.mean(np.square(hist_sky["ddzs"]))))
        sky_reduction_pct = (rms_acc_pass - rms_acc_sky) / rms_acc_pass * 100.0

        # Construct synthetic KPIs for suspension evaluation
        kpi_pass = EngineeringKPIs(
            controller_name="Passive Baseline",
            max_yaw_rate_radps=0.0,
            yaw_rate_rmse_radps=0.0,
            max_yaw_rate_error_radps=0.0,
            max_sideslip_deg=0.0,
            rms_sideslip_deg=0.0,
            final_heading_dev_deg=0.0,
            max_lateral_error_m=0.0,
            rms_lateral_error_m=0.0,
            peak_lat_accel_g=0.0,
            peak_long_decel_g=0.0,
            max_tyre_utilization=0.0,
            stopping_distance_m=None,
            stopping_time_s=None,
            mean_decel_mps2=None,
        )

        kpi_sky = EngineeringKPIs(
            controller_name="Semi-Active Skyhook",
            max_yaw_rate_radps=0.0,
            yaw_rate_rmse_radps=0.0,
            max_yaw_rate_error_radps=0.0,
            max_sideslip_deg=0.0,
            rms_sideslip_deg=0.0,
            final_heading_dev_deg=0.0,
            max_lateral_error_m=0.0,
            rms_lateral_error_m=0.0,
            peak_lat_accel_g=0.0,
            peak_long_decel_g=0.0,
            max_tyre_utilization=0.0,
            stopping_distance_m=None,
            stopping_time_s=None,
            mean_decel_mps2=None,
        )

        eval_pass = RegressionEvaluator.evaluate(
            scenario_key="ride_comfort_bump",
            scenario_name=cfg["name"],
            controller_name="Passive Baseline",
            kpis=kpi_pass,
            thresholds={},
            custom_metrics={"rms_accel_mps2": rms_acc_pass},
        )

        eval_sky = RegressionEvaluator.evaluate(
            scenario_key="ride_comfort_bump",
            scenario_name=cfg["name"],
            controller_name="Semi-Active Skyhook",
            kpis=kpi_sky,
            thresholds=cfg["thresholds"],
            custom_metrics={
                "skyhook_accel_rms_reduction_pct": sky_reduction_pct,
                "rms_accel_mps2": rms_acc_sky,
            },
        )

        self.logger.info(
            f"  [{eval_sky.status_badge}] Semi-Active Skyhook: "
            f"RMS Accel={rms_acc_sky:.2f} m/s² (Gain: +{sky_reduction_pct:.1f}%)"
        )

        # Plot comparison
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5), facecolor=BG_COLOR)
        for ax in (ax1, ax2):
            ax.set_facecolor(AXES_COLOR)
            for spine in ax.spines.values():
                spine.set_edgecolor(GRID_COLOR)
                spine.set_linewidth(1.0)
            ax.tick_params(colors=TEXT_MUTED, labelsize=9)
            ax.grid(True, linestyle="--", alpha=0.4, color=GRID_COLOR)

        ax1.plot(t_arr, hist_pass["ddzs"], color=ACCENT_ROSE, linestyle="-", label=f"Passive (RMS: {rms_acc_pass:.2f} m/s²)")
        ax1.plot(t_arr, hist_sky["ddzs"], color=ACCENT_CYAN, linestyle="-", linewidth=1.8, label=f"Skyhook (RMS: {rms_acc_sky:.2f} m/s², -{sky_reduction_pct:.0f}%)")
        ax1.plot(t_arr, hist_gh["ddzs"], color=ACCENT_EMERALD, linestyle="--", label="Groundhook")
        ax1.set_title("ISO 2631 Body Acceleration Comparison", fontweight="bold", color=TEXT_PRIMARY)
        ax1.set_xlabel("Time [s]", color=TEXT_MUTED)
        ax1.set_ylabel(r"Body Acceleration $\ddot{z}_s$ [m/s²]", color=TEXT_MUTED)
        ax1.legend(facecolor=CARD_COLOR, edgecolor=GRID_COLOR, labelcolor=TEXT_PRIMARY)

        ax2.plot(t_arr, hist_pass["Fz_dyn"], color=ACCENT_ROSE, linestyle="-", label="Passive")
        ax2.plot(t_arr, hist_sky["Fz_dyn"], color=ACCENT_CYAN, linestyle="-", label="Skyhook")
        ax2.plot(t_arr, hist_gh["Fz_dyn"], color=ACCENT_EMERALD, linestyle="-", linewidth=1.8, label="Groundhook")
        ax2.set_title("Dynamic Tyre Normal Force Variation (Road Holding)", fontweight="bold", color=TEXT_PRIMARY)
        ax2.set_xlabel("Time [s]", color=TEXT_MUTED)
        ax2.set_ylabel(r"$\Delta F_z$ [N]", color=TEXT_MUTED)
        ax2.legend(facecolor=CARD_COLOR, edgecolor=GRID_COLOR, labelcolor=TEXT_PRIMARY)

        plt.tight_layout()
        self._save_figure(fig, "ride_comfort_bump.png")

        return [eval_pass, eval_sky]

    # =========================================================================
    # SCENARIO 6: ISO 7975 BRAKING IN A TURN
    # =========================================================================
    def run_braking_in_a_turn(self) -> List[ScenarioEvaluation]:
        cfg = self.matrix_cfg["scenarios"]["braking_in_a_turn"]
        self.logger.info(f"Running Scenario: {cfg['name']}")

        v0 = cfg["initial_speed_kph"] / 3.6
        duration = float(cfg["duration_s"])
        dt = 0.005
        steps = int(duration / dt)
        t_arr = np.linspace(0.0, duration, steps)

        delta_turn = np.zeros(steps)
        for i, t in enumerate(t_arr):
            if t >= 0.2:
                delta_turn[i] = math.radians(float(cfg["steer_deg"]))

        T_brake = np.zeros(steps)
        for i, t in enumerate(t_arr):
            if t >= 1.0:
                T_brake[i] = float(cfg["brake_torque_nm"])

        controllers = self._resolve_controllers(cfg)
        palette = self._get_standard_palette()

        evaluations: List[ScenarioEvaluation] = []
        histories: Dict[str, SimulationHistory] = {}

        for c_key, c_inst in controllers.items():
            model = SevenDOFModel.from_yaml("configs/vehicles/seven_dof_passenger.yaml")
            hist = simulate_maneuver(
                model=model,
                controller=c_inst,
                v_initial=v0,
                delta_driver=delta_turn,
                T_drive_demand=0.0,
                T_brake_demand=T_brake,
                duration=duration,
                dt=dt,
                mu=float(cfg["road_mu"]),
            )
            histories[c_key] = hist
            c_name = palette.get(c_key, (None, None, c_key.replace("_", " ").title()))[2]
            kpis = compute_kpis(hist, controller_name=c_name)
            eval_res = RegressionEvaluator.evaluate(
                scenario_key="braking_in_a_turn",
                scenario_name=cfg["name"],
                controller_name=c_name,
                kpis=kpis,
                thresholds=cfg["thresholds"],
            )
            evaluations.append(eval_res)
            self.logger.info(
                f"  [{eval_res.status_badge}] {eval_res.controller_name}: "
                f"Sideslip={kpis.max_sideslip_deg:.2f}°, YawRate={math.degrees(kpis.max_yaw_rate_radps):.1f}°/s"
            )

        # Plot comparison with prominent 2D planar vehicle footprints (3-Panel Non-Colliding Layout)
        fig, ax_track, ax_detail, ax2, ax3, ax4 = self._create_validation_figure(
            title=cfg["name"],
            controllers=list(histories.keys()),
        )

        v_params = VehicleParams.from_seven_dof_params(model.params)

        # Road backdrop
        ax_track.axhline(0.0, color=TEXT_MUTED, linestyle=":", alpha=0.6)
        ax_track.fill_between([-5, 95], -5.0, 35.0, color=ROAD_COLOR, zorder=0)

        for c_key, hist in histories.items():
            col, style, name = palette.get(c_key, (ACCENT_CYAN, "-", c_key.replace("_", " ").title()))
            draw_vehicle_trajectory(
                ax=ax_track,
                X=hist.X,
                Y=hist.Y,
                psi=hist.psi,
                steer_f=hist.steer_f,
                steer_r=hist.steer_r,
                params=v_params,
                n_snapshots=6,
                color=col,
                path_style=style,
                label=name,
                alpha=0.85,
                scale=1.0,
            )
            # Detail panel: 1:1 true scale close-up of deceleration and spinout arc
            draw_vehicle_trajectory(
                ax=ax_detail,
                X=hist.X,
                Y=hist.Y,
                psi=hist.psi,
                steer_f=hist.steer_f,
                steer_r=hist.steer_r,
                params=v_params,
                n_snapshots=4,
                color=col,
                path_style=style,
                label=name,
                alpha=0.9,
                scale=1.0,
            )
            ax2.plot(hist.time, np.degrees(np.abs(hist.sideslip_angle)), label=name, color=col, linestyle=style, linewidth=2.0)
            ax3.plot(hist.time, hist.vx * 3.6, label=name, color=col, linestyle=style, linewidth=2.0)
            ax4.plot(hist.time, np.degrees(hist.r), label=name, color=col, linestyle=style, linewidth=2.0)

        ax_track.set_title("ISO 7975 Braking in a Turn Trajectory (Full Run)", fontsize=11, fontweight="bold", color=TEXT_PRIMARY)
        ax_track.set_xlabel("X Position [m]", fontsize=9, color=TEXT_MUTED)
        ax_track.set_ylabel("Y Position [m]", fontsize=9, color=TEXT_MUTED)
        ax_track.set_aspect("equal", adjustable="datalim")
        ax_track.grid(True, linestyle="--", alpha=0.4, color=GRID_COLOR)
        ax_track.legend(loc="upper left", fontsize=8, facecolor=CARD_COLOR, edgecolor=GRID_COLOR, labelcolor=TEXT_PRIMARY, framealpha=0.92)

        # Detail panel configuration
        ax_detail.fill_between([25, 65], 2.0, 20.0, color=ROAD_COLOR, zorder=0)
        ax_detail.set_xlim(25, 65)
        ax_detail.set_ylim(2.0, 20.0)
        ax_detail.set_aspect("equal")
        ax_detail.set_title("Deceleration Arc (1:1 Scale)", fontsize=11, fontweight="bold", color=TEXT_PRIMARY)
        ax_detail.set_xlabel("X Position [m]", fontsize=9, color=TEXT_MUTED)
        ax_detail.grid(True, linestyle=":", alpha=0.5, color=GRID_COLOR)

        ax2.set_title(r"Body Sideslip Angle Magnitude $|\beta|$", fontweight="bold", fontsize=10, color=TEXT_PRIMARY)
        ax2.set_xlabel("Time [s]", color=TEXT_MUTED)
        ax2.set_ylabel(r"$|\beta|$ [deg]", color=TEXT_MUTED)
        ax2.legend(fontsize=8, facecolor=CARD_COLOR, edgecolor=GRID_COLOR, labelcolor=TEXT_PRIMARY)

        ax3.set_title(r"Longitudinal Velocity $v_x$ (Braking Deceleration)", fontweight="bold", fontsize=10, color=TEXT_PRIMARY)
        ax3.set_xlabel("Time [s]", color=TEXT_MUTED)
        ax3.set_ylabel("Speed $v_x$ [km/h]", color=TEXT_MUTED)
        ax3.legend(fontsize=8, facecolor=CARD_COLOR, edgecolor=GRID_COLOR, labelcolor=TEXT_PRIMARY)

        ax4.set_title(r"Chassis Yaw Rate $r$", fontweight="bold", fontsize=10, color=TEXT_PRIMARY)
        ax4.set_xlabel("Time [s]", color=TEXT_MUTED)
        ax4.set_ylabel("Yaw Rate [deg/s]", color=TEXT_MUTED)
        ax4.legend(fontsize=8, facecolor=CARD_COLOR, edgecolor=GRID_COLOR, labelcolor=TEXT_PRIMARY)

        self._save_figure(fig, "braking_in_a_turn.png")

        return evaluations

    # =========================================================================
    # SCENARIO 7: NHTSA DYNAMIC FISHHOOK
    # =========================================================================
    def run_fishhook_ncap(self) -> List[ScenarioEvaluation]:
        cfg = self.matrix_cfg["scenarios"]["fishhook_ncap"]
        self.logger.info(f"Running Scenario: {cfg['name']}")

        v0 = cfg["initial_speed_kph"] / 3.6
        duration = float(cfg["duration_s"])
        dt = 0.005
        steps = int(duration / dt)
        t_arr = np.linspace(0.0, duration, steps)

        delta_fish = np.zeros(steps)
        for i, t in enumerate(t_arr):
            if 0.4 <= t < 0.65:
                delta_fish[i] = 0.095 * ((t - 0.4) / 0.25)
            elif 0.65 <= t < 0.90:
                delta_fish[i] = 0.095
            elif 0.90 <= t < 1.15:
                delta_fish[i] = 0.095 - 2.0 * 0.095 * ((t - 0.90) / 0.25)
            elif 1.15 <= t < 2.0:
                delta_fish[i] = -0.095
            elif 2.0 <= t < 2.3:
                delta_fish[i] = -0.095 * (1.0 - (t - 2.0) / 0.3)

        controllers = self._resolve_controllers(cfg)
        palette = self._get_standard_palette()

        evaluations: List[ScenarioEvaluation] = []
        histories: Dict[str, SimulationHistory] = {}

        for c_key, c_inst in controllers.items():
            model = SevenDOFModel.from_yaml("configs/vehicles/seven_dof_passenger.yaml")
            hist = simulate_maneuver(
                model=model,
                controller=c_inst,
                v_initial=v0,
                delta_driver=delta_fish,
                T_drive_demand=0.0,
                T_brake_demand=0.0,
                duration=duration,
                dt=dt,
                mu=float(cfg["road_mu"]),
            )
            histories[c_key] = hist
            c_name = palette.get(c_key, (None, None, c_key.replace("_", " ").title()))[2]
            kpis = compute_kpis(hist, controller_name=c_name)
            eval_res = RegressionEvaluator.evaluate(
                scenario_key="fishhook_ncap",
                scenario_name=cfg["name"],
                controller_name=c_name,
                kpis=kpis,
                thresholds=cfg["thresholds"],
            )
            evaluations.append(eval_res)
            self.logger.info(
                f"  [{eval_res.status_badge}] {eval_res.controller_name}: "
                f"Sideslip={kpis.max_sideslip_deg:.2f}°, LatAcc={kpis.peak_lat_accel_g:.2f}g"
            )

        # Plot comparison with prominent 2D planar vehicle footprints (3-Panel Non-Colliding Layout)
        fig, ax_track, ax_detail, ax2, ax3, ax4 = self._create_validation_figure(
            title=cfg["name"],
            controllers=list(histories.keys()),
        )

        v_params = VehicleParams.from_seven_dof_params(model.params)

        # Road backdrop
        ax_track.axhline(0.0, color=TEXT_MUTED, linestyle=":", alpha=0.6)
        ax_track.fill_between([-5, 85], -16.0, 8.0, color=ROAD_COLOR, zorder=0)

        for c_key, hist in histories.items():
            col, style, name = palette.get(c_key, (ACCENT_CYAN, "-", c_key.replace("_", " ").title()))
            draw_vehicle_trajectory(
                ax=ax_track,
                X=hist.X,
                Y=hist.Y,
                psi=hist.psi,
                steer_f=hist.steer_f,
                steer_r=hist.steer_r,
                params=v_params,
                n_snapshots=6,
                color=col,
                path_style=style,
                label=name,
                alpha=0.85,
                scale=1.0,
            )
            # Detail panel: 1:1 true scale close-up of dynamic reversal apex
            draw_vehicle_trajectory(
                ax=ax_detail,
                X=hist.X,
                Y=hist.Y,
                psi=hist.psi,
                steer_f=hist.steer_f,
                steer_r=hist.steer_r,
                params=v_params,
                n_snapshots=4,
                color=col,
                path_style=style,
                label=name,
                alpha=0.9,
                scale=1.0,
            )
            ax2.plot(hist.time, np.degrees(np.abs(hist.sideslip_angle)), label=name, color=col, linestyle=style, linewidth=2.0)
            ax3.plot(hist.time, np.degrees(hist.r), label=name, color=col, linestyle=style, linewidth=2.0)
            ay_g = np.sum(hist.Fy_body, axis=1) / model.params.mass / 9.81
            ax4.plot(hist.time, ay_g, label=name, color=col, linestyle=style, linewidth=2.0)

        ax_track.set_title("NHTSA Dynamic Fishhook Trajectory (Full Run)", fontsize=11, fontweight="bold", color=TEXT_PRIMARY)
        ax_track.set_xlabel("X Position [m]", fontsize=9, color=TEXT_MUTED)
        ax_track.set_ylabel("Y Position [m]", fontsize=9, color=TEXT_MUTED)
        ax_track.set_aspect("equal", adjustable="datalim")
        ax_track.grid(True, linestyle="--", alpha=0.4, color=GRID_COLOR)
        ax_track.legend(loc="lower left", fontsize=8, facecolor=CARD_COLOR, edgecolor=GRID_COLOR, labelcolor=TEXT_PRIMARY, framealpha=0.92)

        # Detail panel configuration
        ax_detail.fill_between([35, 65], -5.0, 7.0, color=ROAD_COLOR, zorder=0)
        ax_detail.set_xlim(35, 65)
        ax_detail.set_ylim(-5.0, 7.0)
        ax_detail.set_aspect("equal")
        ax_detail.set_title("Reversal Apex Pose (1:1 Scale)", fontsize=11, fontweight="bold", color=TEXT_PRIMARY)
        ax_detail.set_xlabel("X Position [m]", fontsize=9, color=TEXT_MUTED)
        ax_detail.grid(True, linestyle=":", alpha=0.5, color=GRID_COLOR)

        ax2.set_title(r"Body Sideslip Angle Magnitude $|\beta|$", fontweight="bold", fontsize=10, color=TEXT_PRIMARY)
        ax2.set_xlabel("Time [s]", color=TEXT_MUTED)
        ax2.set_ylabel(r"$|\beta|$ [deg]", color=TEXT_MUTED)
        ax2.legend(fontsize=8, facecolor=CARD_COLOR, edgecolor=GRID_COLOR, labelcolor=TEXT_PRIMARY)

        ax3.set_title(r"Chassis Yaw Rate Transient $r$", fontweight="bold", fontsize=10, color=TEXT_PRIMARY)
        ax3.set_xlabel("Time [s]", color=TEXT_MUTED)
        ax3.set_ylabel("Yaw Rate [deg/s]", color=TEXT_MUTED)
        ax3.legend(fontsize=8, facecolor=CARD_COLOR, edgecolor=GRID_COLOR, labelcolor=TEXT_PRIMARY)

        ax4.set_title(r"Lateral Acceleration $a_y$", fontweight="bold", fontsize=10, color=TEXT_PRIMARY)
        ax4.set_xlabel("Time [s]", color=TEXT_MUTED)
        ax4.set_ylabel("Lateral Accel [g]", color=TEXT_MUTED)
        ax4.legend(fontsize=8, facecolor=CARD_COLOR, edgecolor=GRID_COLOR, labelcolor=TEXT_PRIMARY)

        self._save_figure(fig, "fishhook_ncap.png")

        return evaluations

    # =========================================================================
    # SCENARIO 8: LOW-SPEED TIGHT SLALOM (4WS AGILITY)
    # =========================================================================
    def run_low_speed_slalom_4ws(self) -> List[ScenarioEvaluation]:
        cfg = self.matrix_cfg["scenarios"]["low_speed_slalom_4ws"]
        self.logger.info(f"Running Scenario: {cfg['name']}")

        v0 = cfg["initial_speed_kph"] / 3.6
        duration = float(cfg["duration_s"])
        dt = 0.005
        steps = int(duration / dt)
        t_arr = np.linspace(0.0, duration, steps)

        delta_slalom = np.zeros(steps)
        for i, t in enumerate(t_arr):
            if 0.5 <= t < 1.2:
                delta_slalom[i] = 0.09 * math.sin(math.pi * (t - 0.5) / 0.7)
            elif 1.2 <= t < 2.6:
                delta_slalom[i] = -0.12 * math.sin(math.pi * (t - 1.2) / 1.4)
            elif 2.6 <= t < 4.0:
                delta_slalom[i] = 0.12 * math.sin(math.pi * (t - 2.6) / 1.4)
            elif 4.0 <= t < 4.7:
                delta_slalom[i] = -0.06 * math.sin(math.pi * (t - 4.0) / 0.7)

        controllers = self._resolve_controllers(cfg)
        palette = self._get_standard_palette()

        evaluations: List[ScenarioEvaluation] = []
        histories: Dict[str, SimulationHistory] = {}

        for c_key, c_inst in controllers.items():
            model = SevenDOFModel.from_yaml("configs/vehicles/seven_dof_passenger.yaml")
            hist = simulate_maneuver(
                model=model,
                controller=c_inst,
                v_initial=v0,
                delta_driver=delta_slalom,
                T_drive_demand=0.0,
                T_brake_demand=0.0,
                duration=duration,
                dt=dt,
                mu=float(cfg["road_mu"]),
            )
            histories[c_key] = hist
            c_name = palette.get(c_key, (None, None, c_key.replace("_", " ").title()))[2]
            kpis = compute_kpis(hist, controller_name=c_name)
            eval_res = RegressionEvaluator.evaluate(
                scenario_key="low_speed_slalom_4ws",
                scenario_name=cfg["name"],
                controller_name=c_name,
                kpis=kpis,
                thresholds=cfg["thresholds"],
            )
            evaluations.append(eval_res)
            self.logger.info(
                f"  [{eval_res.status_badge}] {eval_res.controller_name}: "
                f"YawRate={math.degrees(kpis.max_yaw_rate_radps):.1f}°/s, MaxSideslip={kpis.max_sideslip_deg:.2f}°"
            )

        # Plot comparison with prominent 2D planar vehicle footprints (3-Panel Non-Colliding Layout)
        fig, ax_track, ax_detail, ax2, ax3, ax4 = self._create_validation_figure(
            title=cfg["name"],
            controllers=list(histories.keys()),
        )

        v_params = VehicleParams.from_seven_dof_params(model.params)

        # Draw slalom cones/pylons along centerline
        pylon_x = [10.0, 21.0, 32.0, 43.0]
        pylon_y = [0.0, 0.0, 0.0, 0.0]
        ax_track.axhline(0.0, color=TEXT_MUTED, linestyle=":", alpha=0.6)
        ax_track.fill_between([-5, 50], -3.5, 3.5, color=ROAD_COLOR, zorder=0)
        ax_track.scatter(pylon_x, pylon_y, color=ACCENT_AMBER, s=35, zorder=4, label="Slalom Pylons")
        ax_detail.scatter(pylon_x, pylon_y, color=ACCENT_AMBER, s=45, zorder=4)

        for c_key, hist in histories.items():
            col, style, name = palette.get(c_key, (ACCENT_CYAN, "-", c_key.replace("_", " ").title()))
            draw_vehicle_trajectory(
                ax=ax_track,
                X=hist.X,
                Y=hist.Y,
                psi=hist.psi,
                steer_f=hist.steer_f,
                steer_r=hist.steer_r,
                params=v_params,
                n_snapshots=6,
                color=col,
                path_style=style,
                label=name,
                alpha=0.85,
                scale=1.0,
            )
            # Detail panel: 1:1 true scale close-up of counter-phase 4WS agility
            draw_vehicle_trajectory(
                ax=ax_detail,
                X=hist.X,
                Y=hist.Y,
                psi=hist.psi,
                steer_f=hist.steer_f,
                steer_r=hist.steer_r,
                params=v_params,
                n_snapshots=4,
                color=col,
                path_style=style,
                label=name,
                alpha=0.9,
                scale=1.0,
            )
            ax2.plot(hist.time, np.degrees(hist.r), label=name, color=col, linestyle=style, linewidth=2.0)
            ax3.plot(hist.time, np.degrees(hist.steer_r), label=f"{name} Rear Steer", color=col, linestyle=style, linewidth=2.0)
            ax4.plot(hist.time, hist.Y, label=name, color=col, linestyle=style, linewidth=2.0)

        ax_track.set_title("Low-Speed Slalom Trajectory (Full Weave Run)", fontsize=11, fontweight="bold", color=TEXT_PRIMARY)
        ax_track.set_xlabel("X Position [m]", fontsize=9, color=TEXT_MUTED)
        ax_track.set_ylabel("Y Position [m]", fontsize=9, color=TEXT_MUTED)
        ax_track.set_xlim(-5, 48)
        ax_track.set_ylim(-3.0, 3.0)
        ax_track.set_aspect("equal", adjustable="datalim")
        ax_track.grid(True, linestyle="--", alpha=0.4, color=GRID_COLOR)
        ax_track.legend(loc="upper left", fontsize=8, facecolor=CARD_COLOR, edgecolor=GRID_COLOR, labelcolor=TEXT_PRIMARY, framealpha=0.92)

        # Detail panel configuration
        ax_detail.fill_between([8, 26], -2.0, 2.5, color=ROAD_COLOR, zorder=0)
        ax_detail.set_xlim(8, 26)
        ax_detail.set_ylim(-1.8, 2.2)
        ax_detail.set_aspect("equal")
        ax_detail.set_title("Counter-Phase 4WS Agility (1:1 Scale)", fontsize=11, fontweight="bold", color=TEXT_PRIMARY)
        ax_detail.set_xlabel("X Position [m]", fontsize=9, color=TEXT_MUTED)
        ax_detail.grid(True, linestyle=":", alpha=0.5, color=GRID_COLOR)

        ax2.set_title(r"Chassis Yaw Rate $r$ (Agility Comparison)", fontweight="bold", fontsize=10, color=TEXT_PRIMARY)
        ax2.set_xlabel("Time [s]", color=TEXT_MUTED)
        ax2.set_ylabel("Yaw Rate [deg/s]", color=TEXT_MUTED)
        ax2.legend(fontsize=8, facecolor=CARD_COLOR, edgecolor=GRID_COLOR, labelcolor=TEXT_PRIMARY)

        ax3.set_title(r"Rear-Axle Steering Actuation $\delta_r$", fontweight="bold", fontsize=10, color=TEXT_PRIMARY)
        ax3.set_xlabel("Time [s]", color=TEXT_MUTED)
        ax3.set_ylabel(r"Rear Steer $\delta_r$ [deg]", color=TEXT_MUTED)
        ax3.legend(fontsize=8, facecolor=CARD_COLOR, edgecolor=GRID_COLOR, labelcolor=TEXT_PRIMARY)

        ax4.set_title(r"Lateral Weave Position $Y(t)$", fontweight="bold", fontsize=10, color=TEXT_PRIMARY)
        ax4.set_xlabel("Time [s]", color=TEXT_MUTED)
        ax4.set_ylabel("Y Position [m]", color=TEXT_MUTED)
        ax4.legend(fontsize=8, facecolor=CARD_COLOR, edgecolor=GRID_COLOR, labelcolor=TEXT_PRIMARY)

        self._save_figure(fig, "low_speed_slalom_4ws.png")

        return evaluations

    # =========================================================================
    # SCENARIO 9: HIGH-SPEED HIGHWAY LANE CHANGE (4WS STABILITY)
    # =========================================================================
    def run_high_speed_lane_change(self) -> List[ScenarioEvaluation]:
        cfg = self.matrix_cfg["scenarios"]["high_speed_lane_change"]
        self.logger.info(f"Running Scenario: {cfg['name']}")

        v0 = cfg["initial_speed_kph"] / 3.6
        duration = float(cfg["duration_s"])
        dt = 0.005
        steps = int(duration / dt)
        t_arr = np.linspace(0.0, duration, steps)

        f_lane = 0.70
        T_lane = 1.0 / f_lane
        delta_overtake = np.zeros(steps)
        for i, t in enumerate(t_arr):
            if 0.5 <= t <= 0.5 + T_lane:
                delta_overtake[i] = 0.038 * math.sin(2.0 * math.pi * f_lane * (t - 0.5))

        controllers = self._resolve_controllers(cfg)
        palette = self._get_standard_palette()

        evaluations: List[ScenarioEvaluation] = []
        histories: Dict[str, SimulationHistory] = {}

        for c_key, c_inst in controllers.items():
            model = SevenDOFModel.from_yaml("configs/vehicles/seven_dof_passenger.yaml")
            hist = simulate_maneuver(
                model=model,
                controller=c_inst,
                v_initial=v0,
                delta_driver=delta_overtake,
                T_drive_demand=0.0,
                T_brake_demand=0.0,
                duration=duration,
                dt=dt,
                mu=float(cfg["road_mu"]),
            )
            histories[c_key] = hist
            c_name = palette.get(c_key, (None, None, c_key.replace("_", " ").title()))[2]
            kpis = compute_kpis(hist, controller_name=c_name)
            eval_res = RegressionEvaluator.evaluate(
                scenario_key="high_speed_lane_change",
                scenario_name=cfg["name"],
                controller_name=c_name,
                kpis=kpis,
                thresholds=cfg["thresholds"],
            )
            evaluations.append(eval_res)
            self.logger.info(
                f"  [{eval_res.status_badge}] {eval_res.controller_name}: "
                f"Sideslip={kpis.max_sideslip_deg:.2f}°, YawRate={math.degrees(kpis.max_yaw_rate_radps):.1f}°/s"
            )

        # Plot comparison with prominent 2D planar vehicle footprints (3-Panel Non-Colliding Layout)
        fig, ax_track, ax_detail, ax2, ax3, ax4 = self._create_validation_figure(
            title=cfg["name"],
            controllers=list(histories.keys()),
        )

        v_params = VehicleParams.from_seven_dof_params(model.params)

        # Highway corridor and lane markings
        ax_track.axhline(0.0, color=TEXT_MUTED, linestyle=":", alpha=0.6, label="Lane 1 Centerline")
        ax_track.axhline(3.5, color="#475569", linestyle="--", alpha=0.8, label="Lane 2 Target (3.5m)")
        ax_track.fill_between([-5, 125], -2.0, 7.0, color=ROAD_COLOR, zorder=0)

        for c_key, hist in histories.items():
            col, style, name = palette.get(c_key, (ACCENT_CYAN, "-", c_key.replace("_", " ").title()))
            draw_vehicle_trajectory(
                ax=ax_track,
                X=hist.X,
                Y=hist.Y,
                psi=hist.psi,
                steer_f=hist.steer_f,
                steer_r=hist.steer_r,
                params=v_params,
                n_snapshots=6,
                color=col,
                path_style=style,
                label=name,
                alpha=0.85,
                scale=1.0,
            )
            # Detail panel: 1:1 true scale close-up of in-phase 4WS crab translation
            draw_vehicle_trajectory(
                ax=ax_detail,
                X=hist.X,
                Y=hist.Y,
                psi=hist.psi,
                steer_f=hist.steer_f,
                steer_r=hist.steer_r,
                params=v_params,
                n_snapshots=4,
                color=col,
                path_style=style,
                label=name,
                alpha=0.9,
                scale=1.0,
            )
            ax2.plot(hist.time, np.degrees(np.abs(hist.sideslip_angle)), label=name, color=col, linestyle=style, linewidth=2.0)
            ax3.plot(hist.time, np.degrees(hist.steer_r), label=f"{name} Rear Steer", color=col, linestyle=style, linewidth=2.0)
            ax4.plot(hist.time, hist.Y, label=name, color=col, linestyle=style, linewidth=2.0)

        ax_track.set_title("120 km/h Highway Lane Change Trajectory (Full Run)", fontsize=11, fontweight="bold", color=TEXT_PRIMARY)
        ax_track.set_xlabel("X Position [m]", fontsize=9, color=TEXT_MUTED)
        ax_track.set_ylabel("Y Position [m]", fontsize=9, color=TEXT_MUTED)
        ax_track.set_xlim(-5, 125)
        ax_track.set_ylim(-2.5, 7.5)
        ax_track.set_aspect("equal", adjustable="datalim")
        ax_track.grid(True, linestyle="--", alpha=0.4, color=GRID_COLOR)
        ax_track.legend(loc="upper left", fontsize=8, facecolor=CARD_COLOR, edgecolor=GRID_COLOR, labelcolor=TEXT_PRIMARY, framealpha=0.92)

        # Detail panel configuration
        ax_detail.axhline(0.0, color=TEXT_MUTED, linestyle=":", alpha=0.5)
        ax_detail.axhline(3.5, color="#475569", linestyle="--", alpha=0.7)
        ax_detail.fill_between([25, 75], -1.0, 5.0, color=ROAD_COLOR, zorder=0)
        ax_detail.set_xlim(25, 75)
        ax_detail.set_ylim(-0.5, 4.5)
        ax_detail.set_aspect("equal")
        ax_detail.set_title("In-Phase 4WS Crab Translation (1:1 Scale)", fontsize=11, fontweight="bold", color=TEXT_PRIMARY)
        ax_detail.set_xlabel("X Position [m]", fontsize=9, color=TEXT_MUTED)
        ax_detail.grid(True, linestyle=":", alpha=0.5, color=GRID_COLOR)

        ax2.set_title(r"Body Sideslip Angle Magnitude $|\beta|$", fontweight="bold", fontsize=10, color=TEXT_PRIMARY)
        ax2.set_xlabel("Time [s]", color=TEXT_MUTED)
        ax2.set_ylabel(r"$|\beta|$ [deg]", color=TEXT_MUTED)
        ax2.legend(fontsize=8, facecolor=CARD_COLOR, edgecolor=GRID_COLOR, labelcolor=TEXT_PRIMARY)

        ax3.set_title(r"Rear-Axle Steering Actuation $\delta_r$ (In-Phase)", fontweight="bold", fontsize=10, color=TEXT_PRIMARY)
        ax3.set_xlabel("Time [s]", color=TEXT_MUTED)
        ax3.set_ylabel(r"Rear Steer $\delta_r$ [deg]", color=TEXT_MUTED)
        ax3.legend(fontsize=8, facecolor=CARD_COLOR, edgecolor=GRID_COLOR, labelcolor=TEXT_PRIMARY)

        ax4.set_title(r"Lateral Lane Position $Y(t)$", fontweight="bold", fontsize=10, color=TEXT_PRIMARY)
        ax4.axhline(3.5, color=TEXT_MUTED, linestyle="--", alpha=0.6, label="Lane 2 (3.5m)")
        ax4.set_xlabel("Time [s]", color=TEXT_MUTED)
        ax4.set_ylabel("Y Position [m]", color=TEXT_MUTED)
        ax4.legend(fontsize=8, facecolor=CARD_COLOR, edgecolor=GRID_COLOR, labelcolor=TEXT_PRIMARY)

        self._save_figure(fig, "high_speed_lane_change.png")

        return evaluations

    # =========================================================================
    # SCENARIO 10: ISO 4138 STEADY-STATE CIRCULAR DRIVING
    # =========================================================================
    def run_iso_4138_circle(self) -> List[ScenarioEvaluation]:
        """Execute ISO 4138 Constant Radius Steady-State Circular Test."""
        cfg = self.matrix_cfg["scenarios"]["iso_4138_circle"]
        self.logger.info(f"Running Scenario: {cfg['name']}")

        radius_m = float(cfg.get("radius_m", 40.0))
        v_start_mps = float(cfg.get("initial_speed_kph", 18.0)) / 3.6
        v_end_mps = float(cfg.get("final_speed_kph", 80.0)) / 3.6
        mu = float(cfg.get("road_mu", 1.0))
        duration_s = float(cfg.get("duration_s", 35.0))

        controllers = self._resolve_controllers(cfg)
        palette = self._get_standard_palette()

        evaluations: List[ScenarioEvaluation] = []
        histories: Dict[str, Any] = {}
        handling_results: Dict[str, HandlingCharacteristics] = {}

        ref_model = SevenDOFModel.from_yaml("configs/vehicles/seven_dof_passenger.yaml")

        for c_key, c_inst in controllers.items():
            model = SevenDOFModel.from_yaml("configs/vehicles/seven_dof_passenger.yaml")
            hist = simulate_steady_state_circular_drive(
                model=model,
                controller=c_inst,
                radius_m=radius_m,
                v_start_mps=v_start_mps,
                v_end_mps=v_end_mps,
                ramp_duration_s=duration_s,
                dt=0.005,
                mu=mu,
            )
            histories[c_key] = hist
            c_name = palette.get(c_key, (None, None, c_key.replace("_", " ").title()))[2]

            h_res = evaluate_understeer_gradient(
                hist=hist,
                model_params=model.params,
                radius_m=radius_m,
                controller_name=c_name,
            )
            handling_results[c_key] = h_res

            kpis = compute_kpis(
                hist,
                controller_name=c_name,
                understeer_gradient_deg_per_g=h_res.understeer_gradient_deg_per_g,
                max_steady_state_lat_accel_g=h_res.max_lat_accel_g,
            )

            custom_metrics = {
                "understeer_gradient_deg_per_g": h_res.understeer_gradient_deg_per_g,
                "max_steady_state_lat_accel_g": h_res.max_lat_accel_g,
                "characteristic_speed_kph": (
                    h_res.characteristic_speed_mps * 3.6
                    if h_res.characteristic_speed_mps is not None
                    else 0.0
                ),
            }

            eval_res = RegressionEvaluator.evaluate(
                scenario_key="iso_4138_circle",
                scenario_name=cfg["name"],
                controller_name=c_name,
                kpis=kpis,
                thresholds=cfg["thresholds"],
                custom_metrics=custom_metrics,
            )
            evaluations.append(eval_res)
            self.logger.info(
                f"  [{eval_res.status_badge}] {eval_res.controller_name}: "
                f"Kus={h_res.understeer_gradient_deg_per_g:+.2f}°/g ({h_res.classification}), "
                f"a_y,max={h_res.max_lat_accel_g:.2f}g, Sideslip={kpis.max_sideslip_deg:.2f}°"
            )

        # Compute G-G diagram for vehicle envelope
        gg_res = compute_gg_diagram(ref_model.params, mu=mu)

        # Plot 4-panel handling limits dashboard
        fig_path = self.figures_dir / "iso_4138_circle.png"
        plot_handling_and_limits(
            histories=histories,
            handling_results=handling_results,
            gg_result=gg_res,
            save_path=fig_path,
        )

        return evaluations


