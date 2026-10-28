"""Handling scenarios: ISO 3888 double lane change, ISO 7401 step steer, ISO 4138 circle, slalom, lane change, NCAP fishhook."""

from __future__ import annotations

import math
from typing import Any, Dict, List
import numpy as np
from vehicle_model.simulation.handling_limits import (
    HandlingCharacteristics,
    compute_gg_diagram,
    evaluate_understeer_gradient,
    plot_handling_and_limits,
    simulate_steady_state_circular_drive,
)
from vehicle_model.simulation.kpi import compute_kpis
from vehicle_model.simulation.regression_evaluator import RegressionEvaluator, ScenarioEvaluation
from vehicle_model.simulation.scenario_runner import SimulationHistory, simulate_maneuver
from vehicle_model.vehicle_models.seven_dof import SevenDOFModel
from vehicle_model.visualization.plan_view import VehicleParams, draw_vehicle_trajectory
from vehicle_model.visualization.theme import (
    ACCENT_AMBER,
    ACCENT_CYAN,
    CARD_COLOR,
    GRID_COLOR,
    ROAD_COLOR,
    TEXT_MUTED,
    TEXT_PRIMARY,
)


class HandlingScenariosMixin:
    """Handling scenarios: ISO 3888 double lane change, ISO 7401 step steer, ISO 4138 circle, slalom, lane change, NCAP fishhook."""

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
