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
def animate_split_mu_scenario(
    results: Dict[str, SplitMuResult],
    output_path: str | Path,
    title: str = "Split-μ Maneuver Dynamic Simulation",
    fps: int = 15,
    dpi: int = 85,
) -> Path:
    """Generate high-definition animated GIF of multi-agent split-mu dynamics.

    Features:
    - Global overview track with moving 2D vehicle bodies, steered wheels, and dynamic breadcrumbs.
    - 1:1 scale dynamic tracking / chase camera showing tire steer angles and spinout behavior up close.
    - Real-time synchronized telemetry graphs (Yaw Rate, Lateral Deviation, Speed) with scanning cursors.
    - Live digital HUD overlay with speed, yaw rate, and heading.
    """
    out_p = Path(output_path)
    out_p.parent.mkdir(parents=True, exist_ok=True)

    palette = {
        "passive": ("#f43f5e", "Passive Baseline"),
        "four_wheel_steering": ("#f59e0b", "Active 4WS (Standalone)"),
        "ivmc_2ws": ("#06b6d4", "IVMC (2WS: TV + ESC)"),
        "full_ivmc_4ws": ("#10b981", "Full-Stack IVMC + 4WS"),
        "integrated_ivmc": ("#10b981", "Full-Stack IVMC + 4WS"),
    }

    first_res = next(iter(results.values()))
    ref_model = SevenDOFModel.from_yaml("configs/vehicles/seven_dof_passenger.yaml")
    v_params = VehicleParams.from_seven_dof_params(ref_model.params)

    t_end = max(r.history.time[-1] for r in results.values())
    frame_times = np.arange(0.0, t_end + 1e-5, 1.0 / fps)

    interp_data: Dict[str, Dict[str, np.ndarray]] = {}
    for k, r in results.items():
        interp_data[k] = {
            "X": np.interp(frame_times, r.history.time, r.history.X, right=r.history.X[-1]),
            "Y": np.interp(frame_times, r.history.time, r.history.Y, right=r.history.Y[-1]),
            "psi": np.interp(frame_times, r.history.time, r.history.psi, right=r.history.psi[-1]),
            "steer_f": np.interp(frame_times, r.history.time, r.history.steer_f, right=r.history.steer_f[-1]),
            "steer_r": np.interp(frame_times, r.history.time, r.history.steer_r, right=r.history.steer_r[-1]),
            "vx": np.interp(frame_times, r.history.time, r.history.vx, right=0.0),
            "r": np.interp(frame_times, r.history.time, r.history.r, right=0.0),
            "beta": np.interp(frame_times, r.history.time, r.history.sideslip_angle, right=r.history.sideslip_angle[-1]),
        }

    fig = plt.figure(figsize=(16, 9), dpi=dpi, facecolor="#0b1120")
    gs = fig.add_gridspec(2, 3, height_ratios=[1.2, 1.0], hspace=0.36, wspace=0.28, top=0.83, bottom=0.06)
    fig.suptitle(title, fontsize=12.5, fontweight="bold", y=0.975, color="#f8fafc")

    # Master top horizontal legend for vehicle types
    legend_handles = [
        plt.Line2D([0], [0], color=palette.get(k, ("#38bdf8", ""))[0], lw=2.5)
        for k in results
    ]
    legend_labels = [palette.get(k, ("", k))[1] for k in results]
    fig.legend(
        legend_handles,
        legend_labels,
        loc="center",
        bbox_to_anchor=(0.45, 0.925),
        ncol=min(4, len(results)),
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

    ax_track = fig.add_subplot(gs[0, :2], facecolor="#0f172a")
    ax_close = fig.add_subplot(gs[0, 2], facecolor="#0f172a")
    ax_yaw = fig.add_subplot(gs[1, 0], facecolor="#0f172a")
    ax_dev = fig.add_subplot(gs[1, 1], facecolor="#0f172a")
    ax_speed = fig.add_subplot(gs[1, 2], facecolor="#0f172a")

    for ax_item in [ax_track, ax_close, ax_yaw, ax_dev, ax_speed]:
        ax_item.tick_params(colors="#94a3b8")

    ax_track.set_aspect("equal", adjustable="box")
    ax_close.set_aspect("equal", adjustable="box")

    all_X = np.concatenate([d["X"] for d in interp_data.values()])
    all_Y = np.concatenate([d["Y"] for d in interp_data.values()])
    x_span = float(np.max(all_X)) - float(np.min(all_X))
    x_min = float(np.min(all_X)) - max(4.0, 0.05 * x_span)
    x_max = float(np.max(all_X)) + max(8.0, 0.08 * x_span)
    y_min = min(-4.0, float(np.min(all_Y)) - 2.5)
    y_max = max(4.5, float(np.max(all_Y)) + 2.5)

    if isinstance(first_res.road, CurvedSplitRoad):
        y_min = min(-3.0, float(np.min(all_Y)) - 3.0)
        y_max = max(18.0, float(np.max(all_Y)) + 4.0)

    ax_track.set_xlim(x_min, x_max)
    ax_track.set_ylim(y_min, y_max)
    ax_track.set_title("Bird-Eye Trajectory & Steered Vehicle Bodies", fontsize=10, fontweight="bold", color="#f8fafc")
    ax_track.set_xlabel("World X [m]", fontsize=9, color="#94a3b8")
    ax_track.set_ylabel("World Y [m]", fontsize=9, color="#94a3b8")
    ax_track.grid(True, linestyle=":", color="#334155", alpha=0.6)

    _draw_split_mu_road_corridor(ax_track, first_res.road, x_min, x_max, y_min, y_max, show_labels=True)
    _draw_split_mu_road_corridor(ax_close, first_res.road, x_min - 20, x_max + 20, y_min - 10, y_max + 10, show_labels=False)

    trail_lines = {
        k: ax_track.plot([], [], color=palette.get(k, ("#38bdf8", ""))[0], lw=2.0)[0]
        for k in results
    }

    cursor_yaw = ax_yaw.axvline(0.0, color="#f8fafc", linestyle="--")
    cursor_dev = ax_dev.axvline(0.0, color="#f8fafc", linestyle="--")
    cursor_speed = ax_speed.axvline(0.0, color="#f8fafc", linestyle="--")

    for k, r in results.items():
        col, disp_lbl = palette.get(k, ("#38bdf8", k.replace("_", " ").title()))
        ax_yaw.plot(r.history.time, np.degrees(r.history.r), color=col, alpha=0.25, lw=1.2)
        ax_dev.plot(r.history.time, r.history.Y, color=col, alpha=0.25, lw=1.2)
        ax_speed.plot(r.history.time, r.history.vx * 3.6, color=col, alpha=0.25, lw=1.2)

    active_yaw = {
        k: ax_yaw.plot([], [], color=palette.get(k, ("#38bdf8", ""))[0], lw=2.0)[0]
        for k in results
    }
    active_dev = {
        k: ax_dev.plot([], [], color=palette.get(k, ("#38bdf8", ""))[0], lw=2.0)[0]
        for k in results
    }
    active_speed = {
        k: ax_speed.plot([], [], color=palette.get(k, ("#38bdf8", ""))[0], lw=2.0)[0]
        for k in results
    }

    ax_yaw.set_title("Yaw Rate r(t) [deg/s]", fontsize=10, fontweight="bold", color="#f8fafc")
    ax_yaw.set_xlabel("Time [s]", fontsize=9, color="#94a3b8")
    ax_yaw.set_ylabel("r [deg/s]", fontsize=9, color="#94a3b8")
    ax_yaw.grid(True, linestyle=":", color="#334155", alpha=0.6)

    ax_dev.set_title("Lateral Deviation Y(t) [m]", fontsize=10, fontweight="bold", color="#f8fafc")
    ax_dev.set_xlabel("Time [s]", fontsize=9, color="#94a3b8")
    ax_dev.set_ylabel("Y [m]", fontsize=9, color="#94a3b8")
    ax_dev.grid(True, linestyle=":", color="#334155", alpha=0.6)

    ax_speed.set_title("Forward Speed vx(t) [km/h]", fontsize=10, fontweight="bold", color="#f8fafc")
    ax_speed.set_xlabel("Time [s]", fontsize=9, color="#94a3b8")
    ax_speed.set_ylabel("vx [km/h]", fontsize=9, color="#94a3b8")
    ax_speed.grid(True, linestyle=":", color="#334155", alpha=0.6)

    # Telemetry HUD relocated to bottom of ax_close to avoid obscuring bird-eye trajectory
    hud = ax_close.text(
        0.04,
        0.04,
        "",
        transform=ax_close.transAxes,
        fontsize=6.8,
        fontfamily="monospace",
        fontweight="bold",
        verticalalignment="bottom",
        color="#f8fafc",
        bbox=dict(boxstyle="round,pad=0.35", facecolor="#111827", alpha=0.90, edgecolor="#38bdf8", linewidth=1.0),
    )

    ax_close.set_title("Chase Camera: 1:1 Pose Tracking", fontsize=10, fontweight="bold")
    ax_close.set_xlabel("World X [m]", fontsize=9)
    ax_close.set_ylabel("World Y [m]", fontsize=9)
    ax_close.grid(True, linestyle="--", alpha=0.5)

    base_track_lines = len(ax_track.lines)
    base_track_patches = len(ax_track.patches)
    base_close_lines = len(ax_close.lines)
    base_close_patches = len(ax_close.patches)

    lead_k = "integrated_ivmc" if "integrated_ivmc" in interp_data else next(iter(interp_data.keys()))

    def update(frame_idx: int):
        t_cur = frame_times[frame_idx]

        while len(ax_track.lines) > base_track_lines:
            ax_track.lines[-1].remove()
        while len(ax_track.patches) > base_track_patches:
            ax_track.patches[-1].remove()
        while len(ax_close.lines) > base_close_lines:
            ax_close.lines[-1].remove()
        while len(ax_close.patches) > base_close_patches:
            ax_close.patches[-1].remove()

        time_badge.set_text(f"Time: {t_cur:4.2f} s")
        hud_lines = []
        for k, d in interp_data.items():
            name_short = palette.get(k, ("", k))[1][:10]
            hud_lines.append(
                f"{name_short:<10}: v={d['vx'][frame_idx]*3.6:4.1f}km/h | r={math.degrees(d['r'][frame_idx]):+5.1f}°/s | ψ={math.degrees(d['psi'][frame_idx]):+4.1f}°"
            )
        hud.set_text("\n".join(hud_lines))

        cursor_yaw.set_xdata([t_cur, t_cur])
        cursor_dev.set_xdata([t_cur, t_cur])
        cursor_speed.set_xdata([t_cur, t_cur])

        t_h = frame_times[: frame_idx + 1]
        for k, d in interp_data.items():
            col = palette.get(k, ("#333333", ""))[0]
            trail_lines[k].set_data(d["X"][: frame_idx + 1], d["Y"][: frame_idx + 1])
            active_yaw[k].set_data(t_h, np.degrees(d["r"][: frame_idx + 1]))
            active_dev[k].set_data(t_h, d["Y"][: frame_idx + 1])
            active_speed[k].set_data(t_h, d["vx"][: frame_idx + 1] * 3.6)

            # Draw vehicle footprint on track overview (scale 1.5)
            draw_vehicle(
                x=float(d["X"][frame_idx]),
                y=float(d["Y"][frame_idx]),
                yaw=float(d["psi"][frame_idx]),
                steer_front=float(d["steer_f"][frame_idx]),
                steer_rear=float(d["steer_r"][frame_idx]),
                params=v_params,
                ax=ax_track,
                color=col,
                alpha=0.90,
                scale=1.5,
                arrow_color=col,
            )

        # Dynamic chase camera tracking lead/IVMC vehicle
        xc = float(interp_data[lead_k]["X"][frame_idx])
        yc = float(interp_data[lead_k]["Y"][frame_idx])
        ax_close.set_xlim(xc - 8.0, xc + 10.0)
        ax_close.set_ylim(yc - 5.0, yc + 5.0)

        for k, d in interp_data.items():
            col = palette.get(k, ("#333333", ""))[0]
            draw_vehicle(
                x=float(d["X"][frame_idx]),
                y=float(d["Y"][frame_idx]),
                yaw=float(d["psi"][frame_idx]),
                steer_front=float(d["steer_f"][frame_idx]),
                steer_rear=float(d["steer_r"][frame_idx]),
                params=v_params,
                ax=ax_close,
                color=col,
                alpha=0.95,
                scale=1.0,
                arrow_color=col,
            )

        return [time_badge, hud, cursor_yaw, cursor_dev, cursor_speed]

    anim = FuncAnimation(fig, update, frames=len(frame_times))
    anim.save(out_p, writer=PillowWriter(fps=fps))
    plt.close(fig)
    return out_p


def animate_split_mu_friction_comparison(
    summary: SplitMuFrictionSweepSummary,
    output_path: str | Path = "output/split_mu_scenarios/split_mu_friction_comparison.gif",
    representative_mu_lows: Optional[List[float]] = None,
    fps: int = 15,
    dpi: int = 85,
) -> Path:
    """Generate high-definition animated GIF comparing vehicle trajectories across multiple mu surfaces."""
    out_p = Path(output_path)
    out_p.parent.mkdir(parents=True, exist_ok=True)

    if representative_mu_lows is None:
        representative_mu_lows = [0.15, 0.35, 0.60]

    # Find matching points
    selected_points: List[SplitMuFrictionSweepPoint] = []
    for mu_target in representative_mu_lows:
        best_pt = min(summary.sweep_points, key=lambda p: abs(p.mu_low - mu_target))
        selected_points.append(best_pt)

    ref_model = SevenDOFModel.from_yaml("configs/vehicles/seven_dof_passenger.yaml")
    v_params = VehicleParams.from_seven_dof_params(ref_model.params)

    # Frame timing
    t_end = 3.2
    frame_times = np.arange(0.0, t_end + 1e-5, 1.0 / fps)

    # Collect data for each surface
    surfaces_data = []
    surface_names = {0.15: "Glaze Ice (μ=0.15)", 0.35: "Packed Snow (μ=0.35)", 0.60: "Wet Asphalt (μ=0.60)"}

    for pt in selected_points:
        matched_name = surface_names.get(round(pt.mu_low, 2), f"Friction μ={pt.mu_low:.2f}")
        c_data = {}
        for c_key in ["passive", "four_wheel_steering", "ivmc_2ws", "full_ivmc_4ws"]:
            if c_key not in pt.results and c_key == "full_ivmc_4ws" and "integrated_ivmc" in pt.results:
                r = pt.results["integrated_ivmc"]
            elif c_key in pt.results:
                r = pt.results[c_key]
            else:
                continue
            c_data[c_key] = {
                "X": np.interp(frame_times, r.history.time, r.history.X, right=r.history.X[-1]),
                "Y": np.interp(frame_times, r.history.time, r.history.Y, right=r.history.Y[-1]),
                "psi": np.interp(frame_times, r.history.time, r.history.psi, right=r.history.psi[-1]),
                "steer_f": np.interp(frame_times, r.history.time, r.history.steer_f, right=r.history.steer_f[-1]),
                "steer_r": np.interp(frame_times, r.history.time, r.history.steer_r, right=r.history.steer_r[-1]),
                "vx": np.interp(frame_times, r.history.time, r.history.vx, right=0.0),
                "r": np.interp(frame_times, r.history.time, r.history.r, right=0.0),
            }
        surfaces_data.append((pt.mu_low, matched_name, c_data))

    fig = plt.figure(figsize=(16, 9), dpi=dpi, facecolor=BG_COLOR)
    gs = fig.add_gridspec(3, 2, width_ratios=[2.2, 1.0], hspace=0.38, wspace=0.22, top=0.83, bottom=0.06)
    fig.suptitle("Multi-Surface Split-μ Emergency Braking Dynamics Comparison (100 km/h)", fontsize=12, fontweight="bold", y=0.975, color=TEXT_PRIMARY)

    ctrl_colors = {
        "passive": ACCENT_ROSE,
        "four_wheel_steering": ACCENT_AMBER,
        "ivmc_2ws": ACCENT_CYAN,
        "full_ivmc_4ws": ACCENT_EMERALD,
        "integrated_ivmc": ACCENT_EMERALD,
    }
    ctrl_labels = {
        "passive": "Passive Baseline",
        "four_wheel_steering": "Active 4WS (Standalone)",
        "ivmc_2ws": "IVMC (2WS: TV + ESC)",
        "full_ivmc_4ws": "Full-Stack IVMC + 4WS",
        "integrated_ivmc": "Full-Stack IVMC + 4WS",
    }

    # Master top horizontal legend for the 4 vehicle configurations
    master_handles = [
        plt.Line2D([0], [0], color=ctrl_colors[k], lw=2.5)
        for k in ["passive", "four_wheel_steering", "ivmc_2ws", "full_ivmc_4ws"]
    ]
    master_labels = [
        ctrl_labels[k]
        for k in ["passive", "four_wheel_steering", "ivmc_2ws", "full_ivmc_4ws"]
    ]
    fig.legend(
        master_handles,
        master_labels,
        loc="center",
        bbox_to_anchor=(0.45, 0.925),
        ncol=4,
        fontsize=8.5,
        facecolor=CARD_COLOR,
        edgecolor=GRID_COLOR,
        labelcolor=TEXT_PRIMARY,
        framealpha=0.95,
    )

    # Prominent live Time Pill in top-right header area
    time_badge = fig.text(
        0.985,
        0.925,
        "Time: 0.00 s",
        fontsize=10.0,
        fontweight="bold",
        fontfamily="monospace",
        ha="right",
        va="center",
        color=ACCENT_CYAN,
        bbox=dict(
            boxstyle="round,pad=0.35",
            facecolor=CARD_COLOR,
            edgecolor=ACCENT_CYAN,
            lw=1.2,
        ),
    )

    axes_tracks = []
    axes_telemetry = []
    for idx, (mu_val, s_name, _) in enumerate(surfaces_data):
        ax_t = fig.add_subplot(gs[idx, 0], facecolor=AXES_COLOR)
        ax_m = fig.add_subplot(gs[idx, 1], facecolor=AXES_COLOR)
        style_axis(ax_t, title=f"Surface Track: {s_name} vs High-μ Asphalt (μ=1.0)")
        style_axis(ax_m, title=f"Yaw Rate Telemetry: {s_name}", xlabel="Time [s]" if idx == 2 else "", ylabel="Yaw Rate [°/s]")
        ax_t.set_aspect("equal", adjustable="datalim")
        ax_t.set_xlim(-3.0, 75.0)
        ax_t.set_ylim(-3.5, 3.5)
        # Corridor fill
        ax_t.fill_between([-5, 85], 0.0, 4.0, color=MU_HIGH_COLOR, alpha=0.35)
        ax_t.fill_between([-5, 85], -4.0, 0.0, color=MU_LOW_COLOR, alpha=0.35)
        ax_t.axhline(0.0, color=TEXT_MUTED, ls="--", lw=0.7, alpha=0.6)
        axes_tracks.append(ax_t)
        axes_telemetry.append(ax_m)

    def update(frame_idx: int):
        artists = []
        cur_t = frame_times[frame_idx]
        time_badge.set_text(f"Time: {cur_t:4.2f} s")

        for s_idx, (mu_val, s_name, c_dict) in enumerate(surfaces_data):
            ax_t = axes_tracks[s_idx]
            ax_m = axes_telemetry[s_idx]

            # Clear dynamic artists by redrawing static elements
            ax_t.clear()
            style_axis(ax_t, title=f"Surface Track: {s_name} vs High-μ Asphalt (μ=1.0)")
            ax_t.set_aspect("equal", adjustable="datalim")
            ax_t.set_xlim(-3.0, 75.0)
            ax_t.set_ylim(-3.5, 3.5)
            ax_t.fill_between([-5, 85], 0.0, 4.0, color=MU_HIGH_COLOR, alpha=0.35)
            ax_t.fill_between([-5, 85], -4.0, 0.0, color=MU_LOW_COLOR, alpha=0.35)
            ax_t.axhline(0.0, color=TEXT_MUTED, ls="--", lw=0.7, alpha=0.6)

            ax_m.clear()
            style_axis(ax_m, title=f"Yaw Rate Telemetry: {s_name}", xlabel="Time [s]" if s_idx == 2 else "", ylabel="Yaw Rate [°/s]")
            ax_m.set_xlim(0.0, t_end)
            ax_m.set_ylim(-20.0, 160.0)

            for c_key in [k for k in ["passive", "four_wheel_steering", "ivmc_2ws", "full_ivmc_4ws"] if k in c_dict]:
                d = c_dict[c_key]
                col = ctrl_colors[c_key]
                # Trajectory trail
                ax_t.plot(d["X"][:frame_idx + 1], d["Y"][:frame_idx + 1], color=col, lw=1.8)

                # Vehicle body snapshot
                draw_vehicle(
                    x=float(d["X"][frame_idx]),
                    y=float(d["Y"][frame_idx]),
                    yaw=float(d["psi"][frame_idx]),
                    steer_front=float(d["steer_f"][frame_idx]),
                    params=v_params,
                    steer_rear=float(d["steer_r"][frame_idx]),
                    ax=ax_t,
                    color=col,
                    alpha=0.9,
                    scale=1.4,
                )

                # Yaw rate plot with cursor
                ax_m.plot(frame_times[:frame_idx + 1], np.degrees(d["r"][:frame_idx + 1]), color=col, lw=1.8)

            ax_m.axvline(cur_t, color=TEXT_PRIMARY, ls=":", lw=1.0)

        return artists

    anim = FuncAnimation(fig, update, frames=len(frame_times), interval=1000.0 / fps, blit=False)
    writer = PillowWriter(fps=fps)
    anim.save(str(out_p), writer=writer)
    plt.close(fig)
    return out_p
