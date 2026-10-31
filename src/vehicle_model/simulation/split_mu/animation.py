"""Animated GIFs of the split-mu scenarios."""

from __future__ import annotations

import matplotlib
matplotlib.use("Agg")
import math
from pathlib import Path
from typing import Dict, List, Optional
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
import numpy as np
from vehicle_model.roads.surface import CurvedSplitRoad
from vehicle_model.vehicle_models.seven_dof import SevenDOFModel
from vehicle_model.visualization.plan_view import VehicleParams, draw_vehicle
from vehicle_model.visualization.theme import (
    style_axis,
    BG_COLOR,
    AXES_COLOR,
    CARD_COLOR,
    GRID_COLOR,
    TEXT_PRIMARY,
    TEXT_MUTED,
    ACCENT_CYAN,
    ACCENT_EMERALD,
    ACCENT_AMBER,
    ACCENT_ROSE,
    MU_HIGH_COLOR,
    MU_LOW_COLOR,
)

from .plots import _draw_split_mu_road_corridor
from .results import SplitMuFrictionSweepPoint, SplitMuFrictionSweepSummary, SplitMuResult


# =============================================================================
# DYNAMIC MULTI-AGENT ANIMATED GIF GENERATOR
# =============================================================================
_LABELS = {"passive": "Passive Baseline", "four_wheel_steering": "Active 4WS (Standalone)", "ivmc_2ws": "IVMC (2WS: TV + ESC)",
           "full_ivmc_4ws": "Full-Stack IVMC + 4WS", "integrated_ivmc": "Full-Stack IVMC + 4WS"}


def animate_split_mu_scenario(
    results: Dict[str, SplitMuResult],
    output_path: str | Path,
    title: str = "Split-μ Maneuver Dynamic Simulation",
    fps: int = 15,
    dpi: int = 85,
) -> Path:
    """Split-mu comparison GIF in the platform's one plan-view renderer, on the scenario's road corridor."""
    from vehicle_model.visualization.run_animation import animate_runs, history_dict
    first = next(iter(results.values()))

    def corridor(ax, xlim, ylim):
        _draw_split_mu_road_corridor(ax, first.road, xlim[0], xlim[1], ylim[0], ylim[1], show_labels=False)
    runs = [(_LABELS.get(k, k), history_dict(r.history)) for k, r in results.items()]
    return animate_runs(runs, output_path, title, fps=fps, dpi=dpi, decorate=corridor)


def animate_split_mu_friction_comparison(
    summary: SplitMuFrictionSweepSummary,
    output_path: str | Path = "output/split_mu_scenarios/split_mu_friction_comparison.gif",
    representative_mu_lows: Optional[List[float]] = None,
    fps: int = 15,
    dpi: int = 85,
) -> Path:
    """One scene with passive and full control on representative low-friction surfaces (one plan-view renderer)."""
    from vehicle_model.visualization.run_animation import animate_runs, history_dict
    if representative_mu_lows is None:
        representative_mu_lows = [0.15, 0.35, 0.60]
    runs = []
    for target in representative_mu_lows:
        pt = min(summary.sweep_points, key=lambda q: abs(q.mu_low - target))
        for key, name in (("passive", "passive"), ("full_ivmc_4ws", "full IVMC + 4WS"), ("integrated_ivmc", "full IVMC + 4WS")):
            if key in pt.results and not any(lab == f"mu {pt.mu_low:.2f}: {name}" for lab, _ in runs):
                runs.append((f"mu {pt.mu_low:.2f}: {name}", history_dict(pt.results[key].history)))
    return animate_runs(runs, output_path, "Split-mu response on three low-friction surfaces", fps=fps, dpi=dpi)
