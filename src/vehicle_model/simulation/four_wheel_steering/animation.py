"""Detailed animated 4WS vehicle (GIF)."""

from __future__ import annotations

import matplotlib
matplotlib.use("Agg")
import math
from pathlib import Path
from typing import Tuple, Optional, Callable
import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
from vehicle_model.visualization.plan_view import VehicleParams, draw_vehicle

from .results import ScenarioRunResult


# =============================================================================
# HIGH-DEFINITION ANIMATIONS WITH RENDERED 2D STEERABLE WHEELS & TELEMETRY HUD
# =============================================================================
def animate_4ws_detailed_vehicle(
    result_4ws: ScenarioRunResult,
    result_2ws: ScenarioRunResult,
    output_file: str | Path,
    title: str = "4WS Vehicle Dynamics Animation",
    fps: int = 15,
    setup_ax_fn: Optional[Callable[[plt.Axes], None]] = None,
    x_lim: Optional[Tuple[float, float]] = None,
    y_lim: Optional[Tuple[float, float]] = None,
    figsize: Tuple[float, float] = (16, 7),
) -> None:
    """Generate high-definition animated GIF rendering real 2D vehicle bodies with steered wheels and live HUD."""
    out_path = Path(output_file)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    v_params = VehicleParams(wheelbase=2.80, width=1.80, length=4.50, tire_radius=0.32, tire_width=0.22, wheel_track=1.60)

    fig, ax = plt.subplots(figsize=figsize, dpi=140, facecolor="#0b1120")
    ax.set_facecolor("#0f172a")

    if x_lim is not None:
        ax.set_xlim(x_lim)
    else:
        x_min = min(np.min(result_4ws.X), np.min(result_2ws.X)) - 8.0
        x_max = max(np.max(result_4ws.X), np.max(result_2ws.X)) + 8.0
        ax.set_xlim(x_min, x_max)

    if y_lim is not None:
        ax.set_ylim(y_lim)
    else:
        y_min = min(np.min(result_4ws.Y), np.min(result_2ws.Y)) - 4.0
        y_max = max(np.max(result_4ws.Y), np.max(result_2ws.Y)) + 4.0
        ax.set_ylim(y_min, y_max)

    ax.set_aspect("equal", adjustable="box")
    ax.set_title(title, color="#f8fafc", fontsize=12, fontweight="bold", pad=12)
    ax.set_xlabel("X Position [m]", color="#94a3b8")
    ax.set_ylabel("Y Position [m]", color="#94a3b8")
    ax.grid(True, linestyle=":", color="#334155", alpha=0.5)
    ax.tick_params(colors="#94a3b8")

    # Static trajectory guide lines
    ax.plot(result_2ws.X, result_2ws.Y, color="#64748b", linestyle="--", linewidth=1.5, label="2WS Spurverlauf (Front-Only)")
    ax.plot(result_4ws.X, result_4ws.Y, color="#0284c7", linestyle="-", linewidth=2.0, label="4WS Spurverlauf (Calibrated LUT)")

    if setup_ax_fn is not None:
        setup_ax_fn(ax)

    ax.legend(loc="lower right", facecolor="#1e293b", edgecolor="#475569", labelcolor="#f8fafc", fontsize=9)

    # Telemetry HUD text box
    hud_text = ax.text(
        0.02, 0.94, "",
        transform=ax.transAxes,
        color="#f8fafc",
        fontsize=9.5,
        fontfamily="monospace",
        verticalalignment="top",
        bbox=dict(boxstyle="round,pad=0.6", facecolor="#111827", edgecolor="#38bdf8", alpha=0.92, linewidth=1.5),
    )

    n_frames = min(len(result_4ws.time), len(result_2ws.time))
    step_skip = max(1, n_frames // 70)
    frame_indices = list(range(0, n_frames, step_skip))

    initial_n_lines = len(ax.lines)
    initial_n_patches = len(ax.patches)

    def update(frame_idx: int):
        # Clear previous frame's vehicle lines and patches
        while len(ax.lines) > initial_n_lines:
            ax.lines[-1].remove()
        while len(ax.patches) > initial_n_patches:
            ax.patches[-1].remove()

        idx = frame_indices[frame_idx]
        t = result_4ws.time[idx]

        # 1. Render 2WS Vehicle (Front-Only, Steer Rear = 0)
        draw_vehicle(
            x=result_2ws.X[idx],
            y=result_2ws.Y[idx],
            yaw=result_2ws.psi[idx],
            steer_front=math.radians(result_2ws.steer_f_deg[idx]),
            params=v_params,
            steer_rear=0.0,
            ax=ax,
            color="#94a3b8",
            alpha=0.65,
            fill_alpha=0.20,
            scale=1.1,
        )

        # 2. Render 4WS Vehicle (Steer Front & Steer Rear visibly rendered!)
        draw_vehicle(
            x=result_4ws.X[idx],
            y=result_4ws.Y[idx],
            yaw=result_4ws.psi[idx],
            steer_front=math.radians(result_4ws.steer_f_deg[idx]),
            params=v_params,
            steer_rear=math.radians(result_4ws.steer_r_deg[idx]),
            ax=ax,
            color="#06b6d4",
            color_front="#38bdf8",
            color_rear="#f43f5e" if abs(result_4ws.steer_r_deg[idx]) > 0.3 else "#06b6d4",
            alpha=0.95,
            fill_alpha=0.35,
            scale=1.1,
        )

        # Telemetry mode string
        is_split_mu = abs(result_4ws.steer_r_deg[idx]) > 0.15 and abs(result_4ws.steer_f_deg[idx]) < 0.25
        if is_split_mu:
            mode_str = "AKTIVE GIERDÄMPFUNG (SPLIT-μ)"
        elif result_4ws.steer_r_deg[idx] * result_4ws.steer_f_deg[idx] < -0.05:
            mode_str = "GEGENPHASE (WENDEKREIS & AGILITÄT)"
        elif result_4ws.steer_r_deg[idx] * result_4ws.steer_f_deg[idx] > 0.05:
            mode_str = "GLEICHPHASE (KRABBEN / ZERO-BETA)"
        else:
            mode_str = "NEUTRAL / GERADEAUS"

        hud_text.set_text(
            f"Zeit: {t:4.2f} s | Geschwindigkeit: {result_4ws.vx[idx]*3.6:5.1f} km/h\n"
            f"Lenkmodus: {mode_str}\n"
            f"┌─ Vorderachswinkel δ_f: {result_4ws.steer_f_deg[idx]:+6.2f}°\n"
            f"└─ Hinterachswinkel δ_r: {result_4ws.steer_r_deg[idx]:+6.2f}°\n"
            f"Fahrdynamik:\n"
            f"  2WS: Schwimmwinkel β = {result_2ws.beta_deg[idx]:+5.2f}° | Gierrate r = {math.degrees(result_2ws.r[idx]):+5.1f}°/s\n"
            f"  4WS: Schwimmwinkel β = {result_4ws.beta_deg[idx]:+5.2f}° | Gierrate r = {math.degrees(result_4ws.r[idx]):+5.1f}°/s"
        )
        return (hud_text,)

    anim = animation.FuncAnimation(fig, update, frames=len(frame_indices), interval=1000 // fps, blit=False)
    writer = animation.PillowWriter(fps=fps)
    anim.save(out_path, writer=writer)
    plt.close(fig)
