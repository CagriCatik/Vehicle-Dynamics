"""Real-Time 2D Planar Vehicle Dynamics Animation Generator.

Generates smooth, animated GIFs and videos of multi-vehicle maneuvers
with live planar vehicle footprints (filled chassis, solid rubber wheels
steered by front/rear angles, axles, heading arrows), synchronized
dynamic telemetry cursors, and an optional 1:1 close-up tracking camera.
"""

from __future__ import annotations

from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

from vehicle_model.simulation.scenario_runner import SimulationHistory
from vehicle_model.visualization.plan_view import VehicleParams, draw_vehicle


def animate_maneuver(
    histories: Dict[str, SimulationHistory],
    output_path: str | Path,
    title: str = "Vehicle Dynamics Maneuver (Real-Time 2D Planar View)",
    v_params: Optional[VehicleParams] = None,
    fps: int = 20,
    playback_speed: float = 1.0,
    vehicle_scale: float = 1.6,
    road_y_bounds: Tuple[float, float] = (-4.0, 6.0),
    split_mu_y: Optional[float] = None,
    track_padding: float = 6.0,
    dpi: int = 100,
    fig_size: Tuple[float, float] = (15.0, 8.5),
    include_closeup: bool = True,
    reference_path: Optional[Tuple[np.ndarray, np.ndarray]] = None,
    pylons: Optional[List[Tuple[float, float]]] = None,
) -> Path:
    """Animated GIF of one or more vehicles in the platform's one plan-view renderer
    (:func:`~vehicle_model.visualization.run_animation.animate_runs`).

    ``v_params``, ``vehicle_scale``, ``road_y_bounds``, ``track_padding``, ``fig_size`` and ``include_closeup`` are
    accepted for compatibility; the layout and the true-scale vehicle are those of the renderer.
    """
    from vehicle_model.visualization.run_animation import animate_runs, history_dict
    if not histories:
        raise ValueError("At least one SimulationHistory must be provided.")
    road = None if split_mu_y is None else {"type": "split_mu", "mu_left": 1.0, "mu_right": 0.2, "split_y": split_mu_y}
    return animate_runs([(k, history_dict(h)) for k, h in histories.items()], output_path, title, road=road, fps=fps,
                        playback=playback_speed, dpi=dpi, reference=reference_path, pylons=pylons)
