"""Stability scenario: sine with dwell (ESC)."""

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
    ACCENT_ROSE,
    CARD_COLOR,
    GRID_COLOR,
    ROAD_COLOR,
    TEXT_MUTED,
    TEXT_PRIMARY,
)


class StabilityScenariosMixin:
    """Stability scenario: sine with dwell (ESC)."""

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
