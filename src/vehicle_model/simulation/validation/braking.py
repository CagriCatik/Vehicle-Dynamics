"""Braking scenarios: split-mu braking and braking in a turn."""

from __future__ import annotations

import math
from typing import Dict, List
import numpy as np
from vehicle_model.simulation.kpi import compute_kpis
from vehicle_model.simulation.regression_evaluator import RegressionEvaluator, ScenarioEvaluation
from vehicle_model.simulation.scenario_runner import SimulationHistory, simulate_maneuver
from vehicle_model.vehicle_models.seven_dof import SevenDOFModel
from vehicle_model.visualization.plan_view import VehicleParams, draw_vehicle_trajectory
from vehicle_model.visualization.theme import (
    ACCENT_CYAN,
    CARD_COLOR,
    GRID_COLOR,
    MU_HIGH_COLOR,
    MU_LOW_COLOR,
    ROAD_COLOR,
    TEXT_MUTED,
    TEXT_PRIMARY,
)


class BrakingScenariosMixin:
    """Braking scenarios: split-mu braking and braking in a turn."""

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
