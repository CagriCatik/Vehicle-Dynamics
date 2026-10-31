"""Drift animation from recorded simulation data only.

Every frame shows recorded samples, nothing else: the vehicle is drawn at the recorded
``X, Y, psi`` with the recorded front steer (``vehicle_polygons`` of :mod:`.plan_view`, the
primitive behind ``draw_vehicle``); no position or angle is corrected or interpolated. A frame
``k`` shows sample ``i = round(k * playback_speed / (fps * dt))``.

Layout: overview (whole recorded path, reference circle and centre, trails of all runs),
close-up following the first run at true scale (body axis, velocity vector, sideslip angle,
counter-steer visible), HUD (time, km/h, beta, r, delta_f, kappa RL/RR, T RL/RR, actuator
saturation, drift mode), and beta, r and steer/torque over time with a synchronised cursor.
Several runs (e.g. open loop vs drift LQR) share the axes; failed runs carry their label.
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np



def frame_indices(n_samples: int, dt: float, fps: int, playback_speed: float) -> np.ndarray:
    """Sample index per frame (recorded samples only)."""
    step = playback_speed / (fps * dt)
    k = np.arange(int(math.floor((n_samples - 1) / step)) + 1)
    return np.minimum(np.round(k * step).astype(int), n_samples - 1)


def animate_drift(runs: list, path: str | Path, fps: int = 20, playback_speed: float = 1.0, dpi: int = 90,
                  close_up_m: float = 9.0, title: str = "") -> Path:
    """GIF of one or more :class:`~vehicle_model.simulation.drift.runner.DriftRunResult` (same time base), in the
    platform's one plan-view renderer, with the reference circle and sideslip, yaw rate and steering traces."""
    from vehicle_model.visualization.run_animation import animate_runs, history_dict
    if not runs:
        raise ValueError("no runs to animate")
    labelled = [(r.variant + ("" if r.label in ("", "PASS") else f" [{r.label}]"), history_dict(r.history)) for r in runs]
    reference = None
    for c in [r.circle for r in runs if r.circle is not None][:1]:
        a = np.linspace(0.0, 2.0 * math.pi, 200)
        reference = (c.cx + c.radius * np.cos(a), c.cy + c.radius * np.sin(a))
    traces = (("sideslip", "beta", 180.0 / math.pi, "deg"), ("yaw rate", "r", 180.0 / math.pi, "deg/s"),
              ("front road-wheel angle", "steer_f", 180.0 / math.pi, "deg"))
    return animate_runs(labelled, path, title or runs[0].scenario, traces, fps=fps, playback=playback_speed, dpi=dpi,
                        close_up=close_up_m, reference=reference)


__all__ = ("animate_drift", "frame_indices")
