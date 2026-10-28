"""Real-Time 2D Planar Vehicle Dynamics Animation Generator.

Generates smooth, animated GIFs and videos of multi-vehicle maneuvers
with live planar vehicle footprints (filled chassis, solid rubber wheels
steered by front/rear angles, axles, heading arrows), synchronized
dynamic telemetry cursors, and an optional 1:1 close-up tracking camera.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Dict, List, Optional, Tuple, Any

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
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
    """Generate an animated GIF of one or more vehicles executing a maneuver.

    Args:
        histories: Dict mapping label/controller name to SimulationHistory.
        output_path: Output file path (.gif).
        title: Overall figure title.
        v_params: Vehicle dimensions for plan view drawing.
        fps: Frames per second of the resulting animation (default 20 fps).
        playback_speed: Pacing multiplier (1.0 = real-time, 0.5 = 2x slow-mo).
        vehicle_scale: Scale factor for vehicle footprint rendering.
        road_y_bounds: Lateral (Y) bounds for road corridor.
        split_mu_y: If set, renders split-mu surface (high-mu above, low-mu below).
        track_padding: Padding along longitudinal X bounds.
        dpi: Output figure DPI.
        fig_size: Matplotlib figure size (width, height) in inches.
        include_closeup: If True, adds a 1:1 scale tracking camera panel.
        reference_path: Optional (x, y) tuple representing the target reference path.
        pylons: Optional list of (x, y) coordinates for cones / obstacles.

    Returns:
        Path to the generated .gif file.
    """
    out_p = Path(output_path)
    out_p.parent.mkdir(parents=True, exist_ok=True)

    if not histories:
        raise ValueError("At least one SimulationHistory must be provided.")

    if v_params is None:
        v_params = VehicleParams()

    # Determine master time array
    t_end = min(h.time[-1] for h in histories.values())
    dt_frame = (1.0 / fps) * playback_speed
    frame_times = np.arange(0.0, t_end, dt_frame)
    if len(frame_times) < 2:
        frame_times = np.array([0.0, t_end])

    # Pre-interpolate all histories to frame_times
    interp_data: Dict[str, Dict[str, np.ndarray]] = {}
    for name, hist in histories.items():
        interp_data[name] = {
            "X": np.interp(frame_times, hist.time, hist.X),
            "Y": np.interp(frame_times, hist.time, hist.Y),
            "psi": np.interp(frame_times, hist.time, hist.psi),
            "steer_f": np.interp(frame_times, hist.time, hist.steer_f),
            "steer_r": np.interp(frame_times, hist.time, hist.steer_r),
            "vx": np.interp(frame_times, hist.time, hist.vx),
            "r": np.interp(frame_times, hist.time, hist.r),
            "beta": np.interp(frame_times, hist.time, hist.sideslip_angle),
        }

    # Primary focus vehicle (e.g. IVMC or first key)
    focus_name = "integrated_ivmc" if "integrated_ivmc" in interp_data else next(iter(interp_data.keys()))

    # Color palette
    color_cycle = [
        ("#d62728", "Passive Baseline"),     # Red
        ("#2ca02c", "Integrated IVMC"),     # Green
        ("#1f77b4", "Four Wheel Steering"), # Blue
        ("#ea580c", "Front Steer NMPC"),    # Amber / Orange
        ("#0284c7", "NMPC Controller"),     # Cyan / Deep Blue
        ("#ff7f0e", "ESC Active"),          # Orange
        ("#9467bd", "NMPC Controller"),     # Purple
    ]
    canonical_colors = {
        "passive": "#f43f5e",
        "four_wheel_steering": "#06b6d4",
        "ivmc_2ws": "#3b82f6",
        "full_ivmc_4ws": "#10b981",
        "integrated_ivmc": "#10b981",
    }
    canonical_labels = {
        "passive": "Passive Baseline",
        "four_wheel_steering": "Active 4WS Standalone",
        "ivmc_2ws": "IVMC 2WS (TV + ESC + ABS)",
        "full_ivmc_4ws": "Full-Stack IVMC + 4WS",
        "integrated_ivmc": "Integrated IVMC",
    }
    name_colors: Dict[str, str] = {}
    for i, name in enumerate(histories.keys()):
        nl = name.lower()
        if name in canonical_colors:
            name_colors[name] = canonical_colors[name]
        elif "passive" in nl or "unregulated" in nl:
            name_colors[name] = "#f43f5e"
        elif "full" in nl or "full_ivmc_4ws" in nl:
            name_colors[name] = "#10b981"
        elif "ivmc_2ws" in nl or "2ws" in nl and "ivmc" in nl:
            name_colors[name] = "#3b82f6"
        elif "four" in nl or "4ws" in nl:
            name_colors[name] = "#06b6d4"
        elif "ivmc" in nl or "integrated" in nl:
            name_colors[name] = "#10b981"
        elif "fws" in nl or ("front" in nl and "steer" in nl and "4ws" not in nl):
            name_colors[name] = "#ea580c"
        elif "nmpc" in nl:
            name_colors[name] = "#0284c7"
        else:
            name_colors[name] = color_cycle[i % len(color_cycle)][0]

    # Overall track bounds
    all_x = np.concatenate([d["X"] for d in interp_data.values()])
    all_y = np.concatenate([d["Y"] for d in interp_data.values()])
    if reference_path is not None:
        all_x = np.concatenate([all_x, reference_path[0]])
        all_y = np.concatenate([all_y, reference_path[1]])
    x_min = float(np.min(all_x)) - track_padding
    x_max = float(np.max(all_x)) + track_padding
    y_min = min(float(np.min(all_y)) - 2.5, road_y_bounds[0] - 1.0)
    y_max = max(float(np.max(all_y)) + 2.5, road_y_bounds[1] + 1.0)

    # Setup Layout:
    # Setup Layout with unified dark telemetry theme
    fig = plt.figure(figsize=fig_size, dpi=dpi, facecolor="#0b1120")
    if include_closeup:
        # Row 0: Full Track (left 65%) + 1:1 Closeup Tracking (right 35%)
        # Row 1: Yaw Rate (left 50%) + Sideslip Angle (right 50%)
        gs = fig.add_gridspec(2, 3, height_ratios=[1.3, 1.0], hspace=0.38, wspace=0.25, top=0.83, bottom=0.06)
        ax_track = fig.add_subplot(gs[0, :2], facecolor="#0f172a")
        ax_close = fig.add_subplot(gs[0, 2], facecolor="#0f172a")
        ax_yaw = fig.add_subplot(gs[1, :2], facecolor="#0f172a")
        ax_beta = fig.add_subplot(gs[1, 2], facecolor="#0f172a")
    else:
        gs = fig.add_gridspec(2, 2, height_ratios=[1.3, 1.0], hspace=0.35, wspace=0.22, top=0.83, bottom=0.06)
        ax_track = fig.add_subplot(gs[0, :], facecolor="#0f172a")
        ax_close = None
        ax_yaw = fig.add_subplot(gs[1, 0], facecolor="#0f172a")
        ax_beta = fig.add_subplot(gs[1, 1], facecolor="#0f172a")

    fig.suptitle(title, fontsize=12.5, fontweight="bold", y=0.975, color="#f8fafc")

    # Master top horizontal legend for vehicle configurations
    legend_handles = [plt.Line2D([0], [0], color=name_colors[n], lw=2.5) for n in histories]
    legend_labels = [canonical_labels.get(n, n.replace("_", " ").title()) for n in histories]
    fig.legend(
        legend_handles,
        legend_labels,
        loc="center",
        bbox_to_anchor=(0.45, 0.925),
        ncol=min(4, len(histories)),
        fontsize=8.5,
        facecolor="#1e293b",
        edgecolor="#475569",
        labelcolor="#f8fafc",
        framealpha=0.95,
    )

    # Prominent live Time Pill in top header
    time_badge = fig.text(
        0.985,
        0.925,
        "Time: 0.00 s",
        fontsize=9.5,
        fontweight="bold",
        fontfamily="monospace",
        ha="right",
        va="center",
        color="#38bdf8",
        bbox=dict(
            boxstyle="round,pad=0.35",
            facecolor="#1e293b",
            edgecolor="#38bdf8",
            lw=1.2,
        ),
    )

    for ax_item in [ax_track, ax_close, ax_yaw, ax_beta]:
        if ax_item is not None:
            ax_item.tick_params(colors="#94a3b8")

    # Static Road Elements on Track
    def draw_road(ax, x0, x1, y0, y1, is_close: bool = False):
        if split_mu_y is not None:
            ax.fill_between([x0, x1], split_mu_y, y1, color="#065f46", alpha=0.35, zorder=0, label="High-μ (1.0 Dry)" if not is_close else None)
            ax.fill_between([x0, x1], y0, split_mu_y, color="#0369a1", alpha=0.35, zorder=0, label="Low-μ (0.2 Ice)" if not is_close else None)
            ax.axhline(split_mu_y, color="#38bdf8", linestyle=":", lw=1.2, zorder=1)
        else:
            ax.fill_between([x0, x1], road_y_bounds[0], road_y_bounds[1], color="#1e293b", zorder=0)
            ax.axhline(0.0, color="#64748b", linestyle=":", lw=1.2, zorder=1)
            ax.axhline(road_y_bounds[0], color="#475569", linestyle="-", lw=1.5, zorder=1)
            ax.axhline(road_y_bounds[1], color="#475569", linestyle="-", lw=1.5, zorder=1)

    draw_road(ax_track, x_min, x_max, y_min, y_max)
    ax_track.set_xlim(x_min, x_max)
    ax_track.set_ylim(y_min, y_max)
    ax_track.set_aspect("equal", adjustable="datalim")
    ax_track.grid(True, linestyle=":", color="#334155", alpha=0.6)
    ax_track.set_xlabel("Global X Position [m]", fontsize=10, color="#94a3b8")
    ax_track.set_ylabel("Global Y Position [m]", fontsize=10, color="#94a3b8")
    ax_track.set_title("Track Overview", fontsize=10.5, fontweight="bold", color="#f8fafc")

    # Optional static reference path overlay
    if reference_path is not None:
        ref_x, ref_y = reference_path
        ax_track.plot(ref_x, ref_y, color="#38bdf8", linestyle="--", lw=1.6, alpha=0.85, zorder=1, label="Target Trajectory")

    # Optional static cones/pylons overlay
    if pylons is not None:
        px = [p[0] for p in pylons]
        py = [p[1] for p in pylons]
        ax_track.scatter(px, py, marker="^", color="#ea580c", edgecolor="#fbbf24", s=50, lw=1.2, zorder=3, label="Slalom Cones")

    # Static full telemetry background curves
    for name, hist in histories.items():
        col = name_colors[name]
        lbl = name.replace("_", " ").title()
        ax_yaw.plot(hist.time, np.degrees(hist.r), color=col, alpha=0.25, lw=1.2)
        ax_beta.plot(hist.time, np.degrees(np.abs(hist.sideslip_angle)), color=col, alpha=0.25, lw=1.2)

    # Dynamic trail lines on track
    trail_lines = {name: ax_track.plot([], [], color=name_colors[name], lw=2.0, zorder=2)[0] for name in histories}

    # Telemetry cursor lines
    cursor_yaw = ax_yaw.axvline(0.0, color="#f8fafc", lw=1.8, linestyle="--")
    cursor_beta = ax_beta.axvline(0.0, color="#f8fafc", lw=1.8, linestyle="--")

    # Animated telemetry progress lines
    active_yaw_lines = {name: ax_yaw.plot([], [], color=name_colors[name], lw=2.2)[0] for name in histories}
    active_beta_lines = {name: ax_beta.plot([], [], color=name_colors[name], lw=2.2)[0] for name in histories}

    ax_yaw.set_title("Chassis Yaw Rate r(t) [deg/s]", fontsize=11, fontweight="bold", color="#f8fafc")
    ax_yaw.set_xlabel("Time [s]", fontsize=9, color="#94a3b8")
    ax_yaw.set_ylabel("Yaw Rate [deg/s]", fontsize=9, color="#94a3b8")
    ax_yaw.set_xlim(0, t_end)
    r_all_deg = np.degrees(np.concatenate([h.r for h in histories.values()]))
    r_pad = max(6.0, (np.max(r_all_deg) - np.min(r_all_deg)) * 0.15)
    ax_yaw.set_ylim(np.min(r_all_deg) - r_pad, np.max(r_all_deg) + r_pad)
    ax_yaw.grid(True, linestyle=":", color="#334155", alpha=0.6)

    ax_beta.set_title("Body Sideslip Angle |β(t)| [deg]", fontsize=11, fontweight="bold", color="#f8fafc")
    ax_beta.set_xlabel("Time [s]", fontsize=9, color="#94a3b8")
    ax_beta.set_ylabel("|Sideslip| [deg]", fontsize=9, color="#94a3b8")
    ax_beta.set_xlim(0, t_end)
    beta_all_deg = np.degrees(np.abs(np.concatenate([h.sideslip_angle for h in histories.values()])))
    ax_beta.set_ylim(-0.5, max(4.0, np.max(beta_all_deg) * 1.25))
    ax_beta.grid(True, linestyle=":", color="#334155", alpha=0.6)

    if ax_close is not None:
        ax_close.set_aspect("equal", adjustable="box")
        ax_close.grid(True, linestyle=":", color="#334155", alpha=0.6)
        ax_close.set_title("Tracking Camera (1:1 Scale)", fontsize=10.5, fontweight="bold", color="#f8fafc")
        ax_close.set_xlabel("Local X [m]", fontsize=9, color="#94a3b8")
        ax_close.set_ylabel("Local Y [m]", fontsize=9, color="#94a3b8")
        if reference_path is not None:
            ref_x, ref_y = reference_path
            ax_close.plot(ref_x, ref_y, color="#38bdf8", linestyle="--", lw=1.8, alpha=0.85, zorder=1)
        if pylons is not None:
            px = [p[0] for p in pylons]
            py = [p[1] for p in pylons]
            ax_close.scatter(px, py, marker="^", color="#ea580c", edgecolor="#fbbf24", s=80, lw=1.4, zorder=3)

    # Telemetry HUD text box (placed in ax_close or at bottom of ax_track without overlapping road)
    if ax_close is not None:
        hud_text = ax_close.text(
            0.04, 0.04, "",
            transform=ax_close.transAxes,
            fontsize=6.8,
            fontfamily="monospace",
            fontweight="bold",
            verticalalignment="bottom",
            color="#f8fafc",
            bbox=dict(boxstyle="round,pad=0.35", facecolor="#111827", edgecolor="#38bdf8", alpha=0.90, linewidth=1.0),
            zorder=10,
        )
    else:
        hud_text = ax_track.text(
            0.02, 0.04, "",
            transform=ax_track.transAxes,
            fontsize=7.0,
            fontfamily="monospace",
            fontweight="bold",
            verticalalignment="bottom",
            color="#f8fafc",
            bbox=dict(boxstyle="round,pad=0.35", facecolor="#111827", edgecolor="#38bdf8", alpha=0.90, linewidth=1.0),
            zorder=10,
        )

    # Record baseline artist count so we can strip vehicle instances every frame
    base_track_lines = len(ax_track.lines)
    base_track_patches = len(ax_track.patches)
    base_close_lines = len(ax_close.lines) if ax_close is not None else 0
    base_close_patches = len(ax_close.patches) if ax_close is not None else 0

    def update(frame_idx: int):
        t_cur = frame_times[frame_idx]

        # 1. Clear previous vehicle artists from track and close-up
        while len(ax_track.lines) > base_track_lines:
            ax_track.lines[-1].remove()
        while len(ax_track.patches) > base_track_patches:
            ax_track.patches[-1].remove()

        if ax_close is not None:
            while len(ax_close.lines) > base_close_lines:
                ax_close.lines[-1].remove()
            while len(ax_close.patches) > base_close_patches:
                ax_close.patches[-1].remove()

        # 2. Update Time Badge and HUD digital readout
        time_badge.set_text(f"Time: {t_cur:5.2f} s")
        hud_lines = []
        for name, data in interp_data.items():
            short_name = name.replace("_", " ").title()[:11]
            vx_kph = data["vx"][frame_idx] * 3.6
            hud_lines.append(
                f"{short_name:<11}: v={vx_kph:4.1f}km/h | r={math.degrees(data['r'][frame_idx]):+5.1f}°/s | "
                f"β={math.degrees(data['beta'][frame_idx]):+4.1f}° | δf={math.degrees(data['steer_f'][frame_idx]):+4.1f}°"
            )
        hud_text.set_text("\n".join(hud_lines))

        # 3. Update telemetry cursors & progressive curves
        cursor_yaw.set_xdata([t_cur, t_cur])
        cursor_beta.set_xdata([t_cur, t_cur])

        t_hist = frame_times[:frame_idx + 1]
        for name, data in interp_data.items():
            trail_lines[name].set_data(data["X"][:frame_idx + 1], data["Y"][:frame_idx + 1])
            active_yaw_lines[name].set_data(t_hist, np.degrees(data["r"][:frame_idx + 1]))
            active_beta_lines[name].set_data(t_hist, np.degrees(np.abs(data["beta"][:frame_idx + 1])))

        # 4. Render vehicles on global track
        for name, data in interp_data.items():
            x = float(data["X"][frame_idx])
            y = float(data["Y"][frame_idx])
            yaw = float(data["psi"][frame_idx])
            steer_f = float(data["steer_f"][frame_idx])
            steer_r = float(data["steer_r"][frame_idx])
            col = name_colors[name]

            draw_vehicle(
                x=x,
                y=y,
                yaw=yaw,
                steer_front=steer_f,
                steer_rear=steer_r,
                params=v_params,
                ax=ax_track,
                color=col,
                outline_color=col,
                arrow_color=col,
                alpha=0.90,
                fill=True,
                fill_alpha=0.35,
                tire_color="#1e1e24",
                tire_alpha=0.90,
                draw_axles=True,
                scale=vehicle_scale,
            )

        # 5. Render vehicles on 1:1 close-up tracking camera
        if ax_close is not None:
            # Center close-up camera on focus vehicle
            xc = float(interp_data[focus_name]["X"][frame_idx])
            yc = float(interp_data[focus_name]["Y"][frame_idx])
            ax_close.set_xlim(xc - 7.5, xc + 10.5)
            ax_close.set_ylim(yc - 4.5, yc + 4.5)

            # Draw local road lines in close-up
            draw_road(ax_close, xc - 15.0, xc + 20.0, yc - 10.0, yc + 10.0, is_close=True)

            for name, data in interp_data.items():
                x = float(data["X"][frame_idx])
                y = float(data["Y"][frame_idx])
                yaw = float(data["psi"][frame_idx])
                steer_f = float(data["steer_f"][frame_idx])
                steer_r = float(data["steer_r"][frame_idx])
                col = name_colors[name]

                draw_vehicle(
                    x=x,
                    y=y,
                    yaw=yaw,
                    steer_front=steer_f,
                    steer_rear=steer_r,
                    params=v_params,
                    ax=ax_close,
                    color=col,
                    outline_color=col,
                    arrow_color=col,
                    alpha=0.92,
                    fill=True,
                    fill_alpha=0.35,
                    tire_color="#1e1e24",
                    tire_alpha=0.90,
                    draw_axles=True,
                    scale=1.0,  # True 1:1 physical scale
                )

        return [time_badge, hud_text, cursor_yaw, cursor_beta]

    anim = FuncAnimation(
        fig,
        update,
        frames=len(frame_times),
        interval=int(1000.0 / fps),
        blit=False,
    )

    writer = PillowWriter(fps=fps)
    anim.save(out_p, writer=writer)
    plt.close(fig)
    return out_p
