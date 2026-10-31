"""Detailed animated 4WS vehicle (GIF): plan view with steered wheels, HUD and steering-angle panel.

Top: plan view of the 4WS and the 2WS vehicle with their paths. Bottom: road-wheel
angles over time (front and rear of the 4WS vehicle, front of the 2WS vehicle) with a
cursor at the current frame, so the rear-axle steering can be read alongside the
motion: counter-phase (rear opposite to front) at low speed, in-phase at high speed.

The vehicle is drawn at its rear axle (``draw_vehicle`` reference point), computed
from the CoG with ``lr``; overhangs follow :func:`.parking.body_overhangs`, so the
drawn body is the outline the parking scenarios measure.
"""

from __future__ import annotations

import matplotlib
matplotlib.use("Agg")
import math
from pathlib import Path
from typing import Callable, Optional, Sequence, Tuple

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Rectangle

from vehicle_model.visualization.plan_view import VehicleParams, draw_vehicle

from .parking import BODY_LENGTH, BODY_WIDTH, body_overhangs
from .results import ScenarioRunResult

C_4WS, C_4WS_REAR, C_2WS = "#06b6d4", "#f43f5e", "#94a3b8"
BG, PANEL, TEXT, MUTED, GRID = "#0b1120", "#0f172a", "#f8fafc", "#94a3b8", "#334155"


def _style(ax: plt.Axes) -> None:
    ax.set_facecolor(PANEL)
    ax.grid(True, linestyle=":", color=GRID, alpha=0.5)
    ax.tick_params(colors=MUTED)
    for sp in ax.spines.values():
        sp.set_color(GRID)


def animate_4ws_detailed_vehicle(
    result_4ws: ScenarioRunResult,
    result_2ws: ScenarioRunResult,
    output_file: str | Path,
    title: str = "4WS Vehicle Dynamics Animation",
    fps: int = 15,
    setup_ax_fn: Optional[Callable[[plt.Axes], None]] = None,
    x_lim: Optional[Tuple[float, float]] = None,
    y_lim: Optional[Tuple[float, float]] = None,
    figsize: Tuple[float, float] = (16, 9.5),
    lf: float = 1.3,
    lr: float = 1.5,
    obstacles: Sequence[Tuple[float, float, float, float]] = (),
    max_frames: int = 70,
    footer: str | None = None,
) -> None:
    """4WS against 2WS in the platform's one plan-view renderer, with scene elements and steering traces."""
    from vehicle_model.visualization.run_animation import animate_runs, history_dict

    def scene(ax, xlim, ylim):
        if setup_ax_fn is not None:
            setup_ax_fn(ax)
        for (x, y, w, h) in obstacles:
            ax.add_patch(Rectangle((x, y), w, h, fc="#475569", ec="#94a3b8", lw=1.0, zorder=2))
    traces = (("front road-wheel angle", "steer_f", 180.0 / math.pi, "deg"), ("rear road-wheel angle", "steer_r", 180.0 / math.pi, "deg"),
              ("yaw rate", "r", 180.0 / math.pi, "deg/s"))
    animate_runs([("4WS", history_dict(result_4ws)), ("2WS", history_dict(result_2ws))], output_file, title, traces, fps=fps,
                 decorate=scene, max_frames=max_frames, footer=footer)


def plot_4ws_comparison(
    result_4ws: ScenarioRunResult,
    result_2ws: ScenarioRunResult,
    output_file: str | Path,
    title: str,
    lf: float = 1.3,
    lr: float = 1.5,
    obstacles: Sequence[Tuple[float, float, float, float]] = (),
    snapshots: int = 6,
    footer: str | None = None,
) -> Path:
    """Static PNG: paths with body outlines at a few instants (top) and road-wheel angles over time (bottom)."""
    out_path = Path(output_file)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    f_oh, r_oh = body_overhangs(lf, lr)
    v_params = VehicleParams(wheelbase=lf + lr, width=BODY_WIDTH, length=BODY_LENGTH, tire_radius=0.32,
                             tire_width=0.22, wheel_track=1.60, rear_overhang=r_oh, front_overhang=f_oh)
    fig = plt.figure(figsize=(16, 9.5), dpi=130, facecolor=BG)
    gs = fig.add_gridspec(2, 1, height_ratios=(3.2, 1.0), hspace=0.28)
    ax, ax_s = fig.add_subplot(gs[0]), fig.add_subplot(gs[1])
    _style(ax)
    _style(ax_s)
    for (x0, y0, w, h) in obstacles:
        ax.add_patch(Rectangle((x0, y0), w, h, facecolor="#475569", edgecolor="#64748b", alpha=0.85, zorder=1))
    ax.plot(result_2ws.X, result_2ws.Y, color=C_2WS, linestyle="--", linewidth=1.5, label="2WS path (CoG)")
    ax.plot(result_4ws.X, result_4ws.Y, color="#0284c7", linewidth=2.0, label="4WS path (CoG)")
    for res, color, rear in ((result_2ws, C_2WS, False), (result_4ws, C_4WS, True)):
        for i in np.linspace(0, len(res.time) - 1, snapshots).astype(int):
            draw_vehicle(x=res.X[i] - lr * math.cos(res.psi[i]), y=res.Y[i] - lr * math.sin(res.psi[i]), yaw=res.psi[i],
                         steer_front=math.radians(res.steer_f_deg[i]), params=v_params,
                         steer_rear=math.radians(res.steer_r_deg[i]) if rear else 0.0, ax=ax, color=color,
                         color_rear=C_4WS_REAR if rear and abs(res.steer_r_deg[i]) > 0.3 else color,
                         alpha=0.8 if rear else 0.5, fill_alpha=0.15)
    ax.set_aspect("equal", adjustable="datalim")
    ax.set_title(title, color=TEXT, fontsize=12, fontweight="bold", pad=10)
    ax.set_xlabel("X [m]", color=MUTED)
    ax.set_ylabel("Y [m]", color=MUTED)
    ax.legend(loc="lower right", facecolor="#1e293b", edgecolor="#475569", labelcolor=TEXT, fontsize=9)
    ax_s.axhline(0.0, color=GRID, linewidth=1.0)
    ax_s.plot(result_2ws.time, result_2ws.steer_f_deg, color=C_2WS, linestyle="--", linewidth=1.4, label="2WS front δ_f")
    ax_s.plot(result_4ws.time, result_4ws.steer_f_deg, color=C_4WS, linewidth=2.0, label="4WS front δ_f")
    ax_s.plot(result_4ws.time, result_4ws.steer_r_deg, color=C_4WS_REAR, linewidth=2.0, label="4WS rear δ_r")
    ax_s.set_xlim(0.0, max(result_4ws.time[-1], result_2ws.time[-1]))
    ax_s.set_xlabel("time [s]", color=MUTED)
    ax_s.set_ylabel("road-wheel angle [deg]", color=MUTED)
    ax_s.legend(loc="upper right", facecolor="#1e293b", edgecolor="#475569", labelcolor=TEXT, fontsize=8, ncol=3)
    if footer:
        fig.text(0.5, 0.01, footer, ha="center", color=MUTED, fontsize=9)
    fig.savefig(out_path, facecolor=fig.get_facecolor())
    plt.close(fig)
    return out_path
