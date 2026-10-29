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
    """Animated GIF: both vehicles with steered wheels, live HUD and the steering angles over time."""
    out_path = Path(output_file)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    f_oh, r_oh = body_overhangs(lf, lr)
    v_params = VehicleParams(wheelbase=lf + lr, width=BODY_WIDTH, length=BODY_LENGTH, tire_radius=0.32,
                             tire_width=0.22, wheel_track=1.60, rear_overhang=r_oh, front_overhang=f_oh)

    fig = plt.figure(figsize=figsize, dpi=120, facecolor=BG)
    gs = fig.add_gridspec(2, 1, height_ratios=(3.2, 1.0), hspace=0.28)
    ax = fig.add_subplot(gs[0])
    ax_s = fig.add_subplot(gs[1])
    _style(ax)
    _style(ax_s)

    pad_x, pad_y = (8.0, 4.0) if x_lim is None and y_lim is None else (0.0, 0.0)
    all_x = np.concatenate([result_4ws.X, result_2ws.X])
    all_y = np.concatenate([result_4ws.Y, result_2ws.Y])
    ax.set_xlim(x_lim if x_lim is not None else (all_x.min() - pad_x, all_x.max() + pad_x))
    ax.set_ylim(y_lim if y_lim is not None else (all_y.min() - pad_y, all_y.max() + pad_y))
    ax.set_aspect("equal", adjustable="box")
    ax.set_title(title, color=TEXT, fontsize=12, fontweight="bold", pad=10)
    ax.set_xlabel("X [m]", color=MUTED)
    ax.set_ylabel("Y [m]", color=MUTED)

    for (x0, y0, w, h) in obstacles:
        ax.add_patch(Rectangle((x0, y0), w, h, facecolor="#475569", edgecolor="#64748b", alpha=0.85, zorder=1))
    ax.plot(result_2ws.X, result_2ws.Y, color=C_2WS, linestyle="--", linewidth=1.5, label="2WS path (CoG, front steering only)")
    ax.plot(result_4ws.X, result_4ws.Y, color="#0284c7", linestyle="-", linewidth=2.0, label="4WS path (CoG, calibrated LUT)")
    if setup_ax_fn is not None:
        setup_ax_fn(ax)
    ax.legend(loc="lower right", facecolor="#1e293b", edgecolor="#475569", labelcolor=TEXT, fontsize=9)

    # Steering panel: one axis (deg), three series; legend and direct labels
    t4, t2 = result_4ws.time, result_2ws.time
    ax_s.axhline(0.0, color=GRID, linewidth=1.0)
    ax_s.plot(t2, result_2ws.steer_f_deg, color=C_2WS, linestyle="--", linewidth=1.4, label="2WS front δ_f")
    ax_s.plot(t4, result_4ws.steer_f_deg, color=C_4WS, linewidth=2.0, label="4WS front δ_f")
    ax_s.plot(t4, result_4ws.steer_r_deg, color=C_4WS_REAR, linewidth=2.0, label="4WS rear δ_r")
    ax_s.set_xlim(0.0, max(t4[-1], t2[-1]))
    ax_s.set_xlabel("time [s]", color=MUTED)
    ax_s.set_ylabel("road-wheel angle [deg]", color=MUTED)
    ax_s.legend(loc="upper right", facecolor="#1e293b", edgecolor="#475569", labelcolor=TEXT, fontsize=8, ncol=3)
    cursor = ax_s.axvline(0.0, color=TEXT, linewidth=1.0, alpha=0.7)
    (dot_f,) = ax_s.plot([], [], "o", color=C_4WS, markersize=8, markeredgecolor=PANEL, markeredgewidth=2)
    (dot_r,) = ax_s.plot([], [], "o", color=C_4WS_REAR, markersize=8, markeredgecolor=PANEL, markeredgewidth=2)
    if footer:
        fig.text(0.5, 0.01, footer, ha="center", color=MUTED, fontsize=9)

    hud_text = ax.text(0.02, 0.96, "", transform=ax.transAxes, color=TEXT, fontsize=9, fontfamily="monospace",
                       verticalalignment="top",
                       bbox=dict(boxstyle="round,pad=0.5", facecolor="#111827", edgecolor="#38bdf8", alpha=0.92, linewidth=1.2))

    n4, n2 = len(t4), len(t2)
    n_frames = max(n4, n2)
    step_skip = max(1, n_frames // max_frames)
    frame_indices = list(range(0, n_frames, step_skip)) + [n_frames - 1]
    initial_n_lines = len(ax.lines)
    initial_n_patches = len(ax.patches)

    def rear_axle(res: ScenarioRunResult, i: int) -> tuple[float, float]:
        return res.X[i] - lr * math.cos(res.psi[i]), res.Y[i] - lr * math.sin(res.psi[i])

    def update(frame_idx: int):
        while len(ax.lines) > initial_n_lines:
            ax.lines[-1].remove()
        while len(ax.patches) > initial_n_patches:
            ax.patches[-1].remove()
        k = frame_indices[frame_idx]
        i4, i2 = min(k, n4 - 1), min(k, n2 - 1)

        x2, y2 = rear_axle(result_2ws, i2)
        draw_vehicle(x=x2, y=y2, yaw=result_2ws.psi[i2], steer_front=math.radians(result_2ws.steer_f_deg[i2]),
                     params=v_params, steer_rear=0.0, ax=ax, color=C_2WS, alpha=0.65, fill_alpha=0.20)
        x4, y4 = rear_axle(result_4ws, i4)
        sr = result_4ws.steer_r_deg[i4]
        draw_vehicle(x=x4, y=y4, yaw=result_4ws.psi[i4], steer_front=math.radians(result_4ws.steer_f_deg[i4]),
                     params=v_params, steer_rear=math.radians(sr), ax=ax, color=C_4WS, color_front="#38bdf8",
                     color_rear=C_4WS_REAR if abs(sr) > 0.3 else C_4WS, alpha=0.95, fill_alpha=0.35)

        t = t4[i4]
        cursor.set_xdata([t, t])
        dot_f.set_data([t], [result_4ws.steer_f_deg[i4]])
        dot_r.set_data([t], [sr])

        sf = result_4ws.steer_f_deg[i4]
        if abs(sr) < 0.3:
            mode = "rear straight"
        elif sr * sf < 0:
            mode = "COUNTER-PHASE: rear opposite to front (smaller radius)"
        else:
            mode = "IN-PHASE: rear with front (less yaw, stable lane change)"
        hud_text.set_text(
            f"t = {t:5.2f} s   v = {result_4ws.vx[i4] * 3.6:+5.1f} km/h\n"
            f"4WS  δ_f = {sf:+6.2f}°   δ_r = {sr:+6.2f}°   {mode}\n"
            f"2WS  δ_f = {result_2ws.steer_f_deg[i2]:+6.2f}°   δ_r =  +0.00°\n"
            f"yaw rate 4WS {math.degrees(result_4ws.r[i4]):+6.1f}°/s | 2WS {math.degrees(result_2ws.r[i2]):+6.1f}°/s"
        )
        return (hud_text,)

    anim = animation.FuncAnimation(fig, update, frames=len(frame_indices), interval=1000 // fps, blit=False)
    anim.save(out_path, writer=animation.PillowWriter(fps=fps))
    plt.close(fig)


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
