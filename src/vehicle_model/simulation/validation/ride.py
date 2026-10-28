"""Ride comfort scenario: quarter car over a bump."""

from __future__ import annotations

import matplotlib
matplotlib.use("Agg")
import math
from typing import List
import matplotlib.pyplot as plt
import numpy as np
from vehicle_model.simulation.kpi import EngineeringKPIs
from vehicle_model.simulation.regression_evaluator import RegressionEvaluator, ScenarioEvaluation
from vehicle_model.suspension.controllers import GroundhookController, SemiActiveConfig, SkyhookController
from vehicle_model.suspension.quarter_car import QuarterCarConfig, QuarterCarModel, QuarterCarState
from vehicle_model.visualization.theme import (
    ACCENT_CYAN,
    ACCENT_EMERALD,
    ACCENT_ROSE,
    AXES_COLOR,
    BG_COLOR,
    CARD_COLOR,
    GRID_COLOR,
    TEXT_MUTED,
    TEXT_PRIMARY,
)


class RideScenariosMixin:
    """Ride comfort scenario: quarter car over a bump."""

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
