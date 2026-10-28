"""Configuration, controllers, figure helpers and the run_all driver of the validation matrix."""

from __future__ import annotations

import matplotlib
matplotlib.use("Agg")
import logging
from pathlib import Path
from typing import Any, Dict, List, Tuple
import matplotlib.pyplot as plt
import yaml
from vehicle_model.controllers import (
    FourWheelSteeringController,
    IntegratedChassisController,
    PassiveBaselineController,
)
from vehicle_model.simulation.regression_evaluator import ScenarioEvaluation
from vehicle_model.visualization.theme import (
    ACCENT_BLUE,
    ACCENT_CYAN,
    ACCENT_EMERALD,
    ACCENT_ROSE,
    AXES_COLOR,
    BG_COLOR,
    CARD_COLOR,
    GRID_COLOR,
    TEXT_MUTED,
    TEXT_PRIMARY,
    apply_dark_theme,
)

matplotlib.use("Agg")
apply_dark_theme()


class ValidationRunnerBase:
    """Configuration, controllers, figure helpers and the run_all driver of the validation matrix."""

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
