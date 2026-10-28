"""Static figures of the split-mu scenarios (road corridor, dashboards, sweep plots)."""

from __future__ import annotations

import matplotlib
matplotlib.use("Agg")
import math
from pathlib import Path
from typing import Dict, Any
import matplotlib.pyplot as plt
import numpy as np
from vehicle_model.roads.surface import CurvedSplitRoad, DiagonalSplitRoad, MuSplitRoad
from vehicle_model.vehicle_models.seven_dof import SevenDOFModel
from vehicle_model.visualization.plan_view import VehicleParams, draw_vehicle
from vehicle_model.visualization.theme import (
    apply_dark_theme,
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

from .results import SplitMuFrictionSweepSummary, SplitMuResult


# =============================================================================
# ROAD SURFACE CORRIDOR RENDERING HELPER
# =============================================================================
def _draw_split_mu_road_corridor(
    ax: Any,
    road: Any,
    x_min: float,
    x_max: float,
    y_min: float,
    y_max: float,
    show_labels: bool = True,
) -> None:
    """Draw realistic road surface corridor with dark asphalt vs low-friction ice shading."""
    if isinstance(road, MuSplitRoad):
        sy = road.split_y
        ax.axhline(sy, color="#38bdf8", linestyle=":", lw=1.5, label="Split Boundary (Y=0.0m)" if show_labels else None, zorder=1)
        ax.fill_between([x_min, x_max], sy, y_max + 10.0, color="#065f46", alpha=0.35, label=f"Left Lane (μ={road.mu_left:.2f} Asphalt)" if show_labels else None, zorder=1)
        ax.fill_between([x_min, x_max], y_min - 10.0, sy, color="#0369a1", alpha=0.35, label=f"Right Lane (μ={road.mu_right:.2f} Wet Ice)" if show_labels else None, zorder=1)
        ax.axhline(sy + 3.75, color="#475569", linestyle="-", lw=1.0, zorder=1)
        ax.axhline(sy - 3.75, color="#475569", linestyle="-", lw=1.0, zorder=1)
    elif isinstance(road, DiagonalSplitRoad):
        x_pts = np.linspace(x_min, x_max, 120)
        th = math.radians(road.theta_deg)
        y_line = road.y0 + (x_pts - road.x0) * math.tan(th)
        ax.plot(x_pts, y_line, color="#38bdf8", linestyle=":", lw=1.5, label=f"Angled Joint ({road.theta_deg:.0f}°)" if show_labels else None, zorder=1)
        ax.fill_between(x_pts, y_line, y_max + 10.0, color="#065f46", alpha=0.35, label=f"Dry Asphalt (μ={road.mu_high:.2f})" if show_labels else None, zorder=1)
        ax.fill_between(x_pts, y_min - 10.0, y_line, color="#0369a1", alpha=0.35, label=f"Ice Patch (μ={road.mu_low:.2f})" if show_labels else None, zorder=1)
        ax.axhline(3.75, color="#475569", linestyle="-", lw=1.0, zorder=1)
        ax.axhline(-3.75, color="#475569", linestyle="-", lw=1.0, zorder=1)
        ax.axhline(0.0, color="#475569", linestyle="--", lw=0.8, zorder=1)
    elif isinstance(road, CurvedSplitRoad):
        R = road.radius_split_m
        th = np.linspace(-0.15, 0.75, 120)
        arc_x = road.center_x + R * np.sin(th)
        arc_y = road.center_y - R * np.cos(th)

        inner_x = road.center_x + (R - 4.0) * np.sin(th)
        inner_y = road.center_y - (R - 4.0) * np.cos(th)
        outer_x = road.center_x + (R + 4.0) * np.sin(th)
        outer_y = road.center_y - (R + 4.0) * np.cos(th)

        poly_in_x = np.concatenate([arc_x, inner_x[::-1]])
        poly_in_y = np.concatenate([arc_y, inner_y[::-1]])
        ax.fill(poly_in_x, poly_in_y, color="#065f46", alpha=0.35, label="Inner Curve (Dry Asphalt)" if show_labels else None, zorder=1)

        poly_out_x = np.concatenate([arc_x, outer_x[::-1]])
        poly_out_y = np.concatenate([arc_y, outer_y[::-1]])
        ax.fill(poly_out_x, poly_out_y, color="#0369a1", alpha=0.35, label="Outer Curve (Slick Ice)" if show_labels else None, zorder=1)

        ax.plot(arc_x, arc_y, color="#38bdf8", linestyle=":", lw=1.5, label=f"Radial Split Arc (R={R:.0f}m)" if show_labels else None, zorder=1)
        ax.plot(inner_x, inner_y, color="#475569", linestyle="-", lw=1.0, zorder=1)
        ax.plot(outer_x, outer_y, color="#475569", linestyle="-", lw=1.0, zorder=1)
    else:
        ax.axhline(0.0, color="#38bdf8", linestyle=":", lw=1.5, label="Centerline" if show_labels else None, zorder=1)
        ax.fill_between([x_min, x_max], 0.0, y_max + 10.0, color="#065f46", alpha=0.35, zorder=1)
        ax.fill_between([x_min, x_max], y_min - 10.0, 0.0, color="#0369a1", alpha=0.35, zorder=1)
        ax.axhline(3.75, color="#475569", linestyle="-", lw=1.0, zorder=1)
        ax.axhline(-3.75, color="#475569", linestyle="-", lw=1.0, zorder=1)


# =============================================================================
# PUBLICATION-QUALITY PLOTTING DASHBOARD
# =============================================================================
def plot_split_mu_scenario_dashboard(
    results: Dict[str, SplitMuResult],
    output_path: str | Path,
    title: str = "Split-μ Scenario Benchmark Analysis",
) -> Path:
    """Generate high-resolution diagnostic dashboard with 2D planar vehicle footprints.

    Panels:
    1. Full World Trajectory with 1:1 true aspect ratio, shaded road corridor, and steered 2D vehicle bodies.
    2. 1:1 Physical Scale Close-Up Detail of vehicle stopping/spinout poses and wheel steer angles.
    3. Yaw Rate r(t) and Heading Angle psi(t) Response.
    4. Lateral Lane Departure Y(t) and Forward Velocity vx(t).
    5. Wheel Longitudinal Forces Fx and Disturbance Yaw Moment Delta Mz.
    """
    out_p = Path(output_path)
    out_p.parent.mkdir(parents=True, exist_ok=True)

    fig = plt.figure(figsize=(18, 10), dpi=140, facecolor="#0b1120")
    gs = fig.add_gridspec(2, 3, height_ratios=[0.78, 1.0], hspace=0.34, wspace=0.28, top=0.83, bottom=0.06)
    fig.suptitle(title, fontsize=12.5, fontweight="bold", y=0.975, color="#f8fafc")

    palette = {
        "passive": ("#f43f5e", "Passive Baseline"),
        "four_wheel_steering": ("#f59e0b", "Active 4WS (Standalone)"),
        "ivmc_2ws": ("#06b6d4", "IVMC (2WS: TV + ESC)"),
        "full_ivmc_4ws": ("#10b981", "Full-Stack IVMC + 4WS"),
        "integrated_ivmc": ("#10b981", "Full-Stack IVMC + 4WS"),
    }

    # Master top horizontal legend for the vehicle configurations
    master_handles = [
        plt.Line2D([0], [0], color=palette.get(k.lower(), ("#38bdf8", ""))[0], lw=2.5)
        for k in results
    ]
    master_labels = [
        f"{palette.get(k.lower(), ('', k))[1]} ({res.metrics.stability_status})"
        for k, res in results.items()
    ]
    fig.legend(
        master_handles,
        master_labels,
        loc="center",
        bbox_to_anchor=(0.50, 0.925),
        ncol=min(4, len(results)),
        fontsize=8.5,
        facecolor="#1e293b",
        edgecolor="#475569",
        labelcolor="#f8fafc",
        framealpha=0.95,
    )

    first_res = next(iter(results.values()))
    ref_model = SevenDOFModel.from_yaml("configs/vehicles/seven_dof_passenger.yaml")
    v_params = VehicleParams.from_seven_dof_params(ref_model.params)

    # -------------------------------------------------------------------------
    # Panel 1: Full World Trajectory & Dynamic 2D Vehicle Poses (Equal Aspect Ratio)
    # -------------------------------------------------------------------------
    ax1 = fig.add_subplot(gs[0, :2], facecolor="#0f172a")
    ax1.set_aspect("equal", adjustable="box")
    ax1.set_title("1. Full Vehicle Trajectories & Dynamic Poses on Split-μ Road", fontsize=11, fontweight="bold", color="#f8fafc")
    ax1.set_xlabel("World X Position [m]", fontsize=9.5, color="#94a3b8")
    ax1.set_ylabel("World Y Position [m]", fontsize=9.5, color="#94a3b8")
    ax1.grid(True, linestyle=":", color="#334155", alpha=0.6)
    ax1.tick_params(colors="#94a3b8")

    all_X = np.concatenate([r.history.X for r in results.values()])
    all_Y = np.concatenate([r.history.Y for r in results.values()])
    x_span = float(np.max(all_X)) - float(np.min(all_X))
    x_min = float(np.min(all_X)) - max(4.0, 0.05 * x_span)
    x_max = float(np.max(all_X)) + max(8.0, 0.08 * x_span)
    y_min = min(-4.5, float(np.min(all_Y)) - 2.5)
    y_max = max(4.5, float(np.max(all_Y)) + 2.5)

    if isinstance(first_res.road, CurvedSplitRoad):
        y_min = min(-3.0, float(np.min(all_Y)) - 3.0)
        y_max = max(18.0, float(np.max(all_Y)) + 4.0)

    _draw_split_mu_road_corridor(ax1, first_res.road, x_min, x_max, y_min, y_max, show_labels=True)

    for k, res in results.items():
        col, disp_lbl = palette.get(k.lower(), ("#38bdf8", k.replace("_", " ").title()))
        ax1.plot(
            res.history.X,
            res.history.Y,
            color=col,
            lw=2.2,
            zorder=3,
        )

        # Milestone 1: Start (t=0)
        draw_vehicle(
            x=res.history.X[0],
            y=res.history.Y[0],
            yaw=res.history.psi[0],
            steer_front=res.history.steer_f[0],
            steer_rear=res.history.steer_r[0],
            params=v_params,
            ax=ax1,
            color="#64748b",
            alpha=0.35,
            scale=1.5,
        )

        # Milestone 2: Midpoint
        mid_idx = len(res.history.X) // 2
        draw_vehicle(
            x=res.history.X[mid_idx],
            y=res.history.Y[mid_idx],
            yaw=res.history.psi[mid_idx],
            steer_front=res.history.steer_f[mid_idx],
            steer_rear=res.history.steer_r[mid_idx],
            params=v_params,
            ax=ax1,
            color=col,
            alpha=0.65,
            scale=1.5,
        )

        # Milestone 3: Final stop / exit
        draw_vehicle(
            x=res.history.X[-1],
            y=res.history.Y[-1],
            yaw=res.history.psi[-1],
            steer_front=res.history.steer_f[-1],
            steer_rear=res.history.steer_r[-1],
            params=v_params,
            ax=ax1,
            color=col,
            alpha=1.0,
            scale=1.5,
            arrow_color=col,
        )

    ax1.set_xlim(x_min, x_max)
    ax1.set_ylim(y_min, y_max)

    # -------------------------------------------------------------------------
    # Panel 2: 1:1 Physical Scale Close-Up Detail (Stopping & Spinout Poses)
    # -------------------------------------------------------------------------
    ax_close = fig.add_subplot(gs[0, 2], facecolor="#0f172a")
    ax_close.set_aspect("equal", adjustable="box")
    ax_close.set_title("2. 1:1 Scale Stop & Spinout Poses", fontsize=11, fontweight="bold", color="#f8fafc")
    ax_close.set_xlabel("World X Position [m]", fontsize=9.5, color="#94a3b8")
    ax_close.set_ylabel("World Y Position [m]", fontsize=9.5, color="#94a3b8")
    ax_close.grid(True, linestyle=":", color="#334155", alpha=0.6)
    ax_close.tick_params(colors="#94a3b8")

    final_xs = [float(r.history.X[-1]) for r in results.values()]
    final_ys = [float(r.history.Y[-1]) for r in results.values()]
    fx_span = max(final_xs) - min(final_xs)
    fy_span = max(final_ys) - min(final_ys)

    margin_x = max(5.0, (16.0 - fx_span) / 2.0)
    margin_y = max(6.5, (12.0 - fy_span) / 2.0)
    min_fx, max_fx = min(final_xs) - margin_x, max(final_xs) + margin_x
    min_fy, max_fy = min(final_ys) - margin_y, max(final_ys) + margin_y

    _draw_split_mu_road_corridor(ax_close, first_res.road, min_fx, max_fx, min_fy, max_fy, show_labels=False)

    callout_lines = []
    for k, res in results.items():
        col, disp_lbl = palette.get(k.lower(), ("#38bdf8", k.replace("_", " ").title()))
        draw_vehicle(
            x=res.history.X[-1],
            y=res.history.Y[-1],
            yaw=res.history.psi[-1],
            steer_front=res.history.steer_f[-1],
            steer_rear=res.history.steer_r[-1],
            params=v_params,
            ax=ax_close,
            color=col,
            alpha=1.0,
            scale=1.0,
            arrow_color=col,
        )
        psi_deg = math.degrees(res.history.psi[-1])
        y_dev = res.history.Y[-1]
        callout_lines.append(f"{disp_lbl[:10]}: ψ={psi_deg:+4.1f}°, Y={y_dev:+4.2f}m")

    callout_pairs = []
    for i in range(0, len(callout_lines), 2):
        if i + 1 < len(callout_lines):
            callout_pairs.append(f"{callout_lines[i]}  |  {callout_lines[i+1]}")
        else:
            callout_pairs.append(callout_lines[i])

    ax_close.set_xlim(min_fx, max_fx)
    ax_close.set_ylim(min_fy, max_fy)
    ax_close.text(
        0.50,
        0.03,
        "\n".join(callout_pairs),
        transform=ax_close.transAxes,
        fontsize=6.8,
        fontfamily="monospace",
        color="#f8fafc",
        horizontalalignment="center",
        verticalalignment="bottom",
        bbox=dict(boxstyle="round,pad=0.35", facecolor="#111827", alpha=0.90, edgecolor="#38bdf8", linewidth=1.0),
    )

    # -------------------------------------------------------------------------
    # Panel 3: Yaw Rate r(t) and Heading Angle psi(t)
    # -------------------------------------------------------------------------
    ax3 = fig.add_subplot(gs[1, 0], facecolor="#0f172a")
    ax3.set_title("3. Yaw Rate $r(t)$ and Heading Angle $\\psi(t)$", fontsize=11, fontweight="bold", color="#f8fafc")
    ax3.set_xlabel("Time [s]", fontsize=9.5, color="#94a3b8")
    ax3.set_ylabel("Yaw Rate $r$ [deg/s]", fontsize=9.5, color="#94a3b8")
    ax3.grid(True, linestyle=":", color="#334155", alpha=0.6)
    ax3.tick_params(colors="#94a3b8")

    ax3_h = ax3.twinx()
    ax3_h.set_ylabel("Heading Angle $\\psi$ [deg]", fontsize=9.5, color="#38bdf8")
    ax3_h.tick_params(colors="#94a3b8")

    for k, res in results.items():
        col, disp_lbl = palette.get(k.lower(), ("#38bdf8", k.replace("_", " ").title()))
        t = res.history.time
        ax3.plot(t, np.degrees(res.history.r), color=col, lw=2.0)
        ax3_h.plot(t, np.degrees(res.history.psi), color=col, linestyle="--", lw=1.5, alpha=0.7)

    ax3.axhline(0.0, color="#64748b", linestyle=":", lw=1.0)

    # -------------------------------------------------------------------------
    # Panel 4: Lateral Lane Departure Y(t) / Forward Velocity vx(t)
    # -------------------------------------------------------------------------
    ax4 = fig.add_subplot(gs[1, 1], facecolor="#0f172a")
    ax4.set_title("4. Lateral Lane Departure $Y(t)$ & Speed $v_x(t)$", fontsize=11, fontweight="bold", color="#f8fafc")
    ax4.set_xlabel("Time [s]", fontsize=9.5, color="#94a3b8")
    ax4.set_ylabel("Lateral Deviation $Y$ [m]", fontsize=9.5, color="#94a3b8")
    ax4.grid(True, linestyle=":", color="#334155", alpha=0.6)
    ax4.tick_params(colors="#94a3b8")

    ax4_v = ax4.twinx()
    ax4_v.set_ylabel("Forward Speed $v_x$ [km/h]", fontsize=9.5, color="#38bdf8")
    ax4_v.tick_params(colors="#94a3b8")

    for k, res in results.items():
        col, disp_lbl = palette.get(k.lower(), ("#38bdf8", k.replace("_", " ").title()))
        t = res.history.time
        ax4.plot(t, res.history.Y, color=col, lw=2.0)
        ax4_v.plot(t, res.history.vx * 3.6, color=col, linestyle=":", lw=1.5, alpha=0.7)

    ax4.axhline(0.0, color="#64748b", linestyle=":", lw=1.0)

    # -------------------------------------------------------------------------
    # Panel 5: Wheel Forces & Disturbance Yaw Moment
    # -------------------------------------------------------------------------
    ax5 = fig.add_subplot(gs[1, 2], facecolor="#0f172a")
    ax5.set_title("5. Wheel Forces & Disturbance Yaw Moment $\\Delta M_z$", fontsize=11, fontweight="bold", color="#f8fafc")
    ax5.set_xlabel("Time [s]", fontsize=9.5, color="#94a3b8")
    ax5.set_ylabel("Longitudinal Force $F_x$ [N]", fontsize=9.5, color="#94a3b8")
    ax5.grid(True, linestyle=":", color="#334155", alpha=0.6)
    ax5.tick_params(colors="#94a3b8")

    target_res = results.get("integrated_ivmc", first_res)
    t = target_res.history.time
    fx_fl = target_res.history.Fx_wheel[:, 0]
    fx_fr = target_res.history.Fx_wheel[:, 1]
    fx_rl = target_res.history.Fx_wheel[:, 2]
    fx_rr = target_res.history.Fx_wheel[:, 3]

    ax5.plot(t, fx_fl, color="#10b981", lw=1.8, label="Front Left (High-μ)")
    ax5.plot(t, fx_fr, color="#0284c7", linestyle="--", lw=1.8, label="Front Right (Low-μ)")

    track_w = getattr(ref_model.params, "wf", 1.60)
    delta_fx_f = fx_fl - fx_fr
    delta_fx_r = fx_rl - fx_rr
    mz_asym = (delta_fx_f + delta_fx_r) * (track_w / 2.0)

    ax5_mz = ax5.twinx()
    ax5_mz.plot(t, mz_asym / 1000.0, color="#f43f5e", linestyle=":", lw=2.0, label="Yaw Moment $\\Delta M_z$")
    ax5_mz.set_ylabel("Yaw Moment $\\Delta M_z$ [kNm]", color="#f43f5e", fontsize=9.5)
    ax5_mz.tick_params(colors="#94a3b8")

    ax5.legend(loc="upper left", fontsize=7.5, facecolor="#1e293b", edgecolor="#475569", labelcolor="#f8fafc", framealpha=0.92)
    ax5_mz.legend(loc="lower right", fontsize=7.5, facecolor="#1e293b", edgecolor="#475569", labelcolor="#f8fafc", framealpha=0.92)

    fig.subplots_adjust(top=0.83, bottom=0.06, left=0.06, right=0.94, hspace=0.34, wspace=0.28)
    fig.savefig(out_p, facecolor=fig.get_facecolor(), bbox_inches="tight")
    plt.close(fig)
    return out_p


def plot_split_mu_friction_sweep(
    summary: SplitMuFrictionSweepSummary,
    output_path: str | Path = "output/split_mu_scenarios/split_mu_friction_sweep_analysis.png",
    title: str = "Parametric Split-μ Friction Sweep & Stability Bifurcation Analysis (100 km/h ECE-R13H)",
) -> Path:
    """Generate high-resolution 6-panel parametric sweep diagram in unified dark telemetry theme."""
    apply_dark_theme()
    out_p = Path(output_path)
    out_p.parent.mkdir(parents=True, exist_ok=True)

    fig = plt.figure(figsize=(19, 11), dpi=140, facecolor=BG_COLOR)
    gs = fig.add_gridspec(2, 3, hspace=0.36, wspace=0.28, top=0.83, bottom=0.06)
    fig.suptitle(title, fontsize=13, fontweight="bold", y=0.975, color=TEXT_PRIMARY)

    ctrl_styles = {
        "passive": (ACCENT_ROSE, "o-", "Passive Baseline (Open Diff)"),
        "four_wheel_steering": (ACCENT_AMBER, "s--", "Active 4WS (Standalone)"),
        "ivmc_2ws": (ACCENT_CYAN, "^-.", "IVMC (2WS: TV + ESC)"),
        "full_ivmc_4ws": (ACCENT_EMERALD, "*-", "Full-Stack IVMC + 4WS"),
        "integrated_ivmc": (ACCENT_EMERALD, "*-", "Full-Stack IVMC + 4WS"),
    }

    # Master top horizontal legend for the 4 vehicle configurations
    master_handles = [
        plt.Line2D([0], [0], color=ctrl_styles[c_key][0], marker=ctrl_styles[c_key][1][0], lw=2.0, ms=5)
        for c_key in summary.controllers
    ]
    master_labels = [ctrl_styles[c_key][2] for c_key in summary.controllers]
    fig.legend(
        master_handles,
        master_labels,
        loc="center",
        bbox_to_anchor=(0.50, 0.925),
        ncol=min(4, len(summary.controllers)),
        fontsize=8.8,
        facecolor=CARD_COLOR,
        edgecolor=GRID_COLOR,
        labelcolor=TEXT_PRIMARY,
        framealpha=0.95,
    )

    mu_lows = np.array(summary.mu_low_values)
    delta_mus = summary.mu_high - mu_lows

    # -------------------------------------------------------------------------
    # 1. Peak Yaw Rate vs Delta Mu
    # -------------------------------------------------------------------------
    ax1 = fig.add_subplot(gs[0, 0], facecolor=AXES_COLOR)
    style_axis(ax1, title=r"1. Peak Yaw Rate $r_{max}$ vs $\Delta\mu = \mu_H - \mu_L$", xlabel=r"Friction Delta $\Delta\mu$ [-]", ylabel=r"Peak Yaw Rate [deg/s]")
    for c_key in summary.controllers:
        color, fmt, label = ctrl_styles[c_key]
        ax1.plot(delta_mus, summary.yaw_rate_max[c_key], fmt, color=color, lw=2.0, ms=5)
    th1 = ax1.axhline(15.0, color=ACCENT_AMBER, ls="--", lw=1.2)
    ax1.grid(True, alpha=0.3, color=GRID_COLOR)
    ax1.legend([th1], [r"Instability Threshold ($15^\circ/\mathrm{s}$)"], loc="upper left", fontsize=7.5, facecolor=CARD_COLOR, edgecolor=GRID_COLOR, labelcolor=TEXT_PRIMARY)

    # -------------------------------------------------------------------------
    # 2. Final Heading Deviation vs mu_low
    # -------------------------------------------------------------------------
    ax2 = fig.add_subplot(gs[0, 1], facecolor=AXES_COLOR)
    style_axis(ax2, title=r"2. Final Heading Deviation $\Delta\psi_{end}$ vs $\mu_{low}$", xlabel=r"Low-$\mu$ Friction Coefficient $\mu_{low}$ [-]", ylabel=r"Heading Deviation $\Delta\psi_{end}$ [deg]")
    sp2 = ax2.axhspan(20.0, 105.0, color=ACCENT_ROSE, alpha=0.12)
    for c_key in summary.controllers:
        color, fmt, label = ctrl_styles[c_key]
        ax2.plot(mu_lows, summary.heading_dev_end[c_key], fmt, color=color, lw=2.0, ms=5)
    ax2.set_ylim(-2.0, 105.0)
    ax2.grid(True, alpha=0.3, color=GRID_COLOR)
    ax2.legend([sp2], ["Severe Spinout Envelope (>20°)"], loc="upper right", fontsize=7.5, facecolor=CARD_COLOR, edgecolor=GRID_COLOR, labelcolor=TEXT_PRIMARY)

    # -------------------------------------------------------------------------
    # 3. Maximum Lateral Departure vs mu_low
    # -------------------------------------------------------------------------
    ax3 = fig.add_subplot(gs[0, 2], facecolor=AXES_COLOR)
    style_axis(ax3, title=r"3. Max Lateral Departure $\Delta Y_{max}$ vs $\mu_{low}$", xlabel=r"Low-$\mu$ Friction Coefficient $\mu_{low}$ [-]", ylabel=r"Max Lateral Deviation $\Delta Y_{max}$ [m]")
    th3 = ax3.axhline(0.8, color=ACCENT_AMBER, ls="--", lw=1.4)
    for c_key in summary.controllers:
        color, fmt, label = ctrl_styles[c_key]
        ax3.plot(mu_lows, summary.lateral_dev_max[c_key], fmt, color=color, lw=2.0, ms=5)
    ax3.grid(True, alpha=0.3, color=GRID_COLOR)
    ax3.legend([th3], ["ECE-R13H Corridor Limit (0.8 m)"], loc="upper right", fontsize=7.5, facecolor=CARD_COLOR, edgecolor=GRID_COLOR, labelcolor=TEXT_PRIMARY)

    # -------------------------------------------------------------------------
    # 4. Emergency Stopping Distance vs mu_low
    # -------------------------------------------------------------------------
    ax4 = fig.add_subplot(gs[1, 0], facecolor=AXES_COLOR)
    style_axis(ax4, title=r"4. Emergency Stopping Distance $s_{stop}$ vs $\mu_{low}$", xlabel=r"Low-$\mu$ Friction Coefficient $\mu_{low}$ [-]", ylabel=r"Stopping Distance $s_{stop}$ [m]")
    v0 = 100.0 / 3.6
    s_ideal = [(v0**2) / (2.0 * 9.81 * (summary.mu_high + m) / 2.0) for m in mu_lows]
    ph4 = ax4.plot(mu_lows, s_ideal, color=TEXT_MUTED, ls=":", lw=1.6)[0]
    for c_key in summary.controllers:
        color, fmt, label = ctrl_styles[c_key]
        ax4.plot(mu_lows, summary.stopping_distance[c_key], fmt, color=color, lw=2.0, ms=5)
    ax4.grid(True, alpha=0.3, color=GRID_COLOR)
    ax4.legend([ph4], [r"Physical Bound $v_0^2/(2g\mu_{avg})$"], loc="upper right", fontsize=7.5, facecolor=CARD_COLOR, edgecolor=GRID_COLOR, labelcolor=TEXT_PRIMARY)

    # -------------------------------------------------------------------------
    # 5. Peak Sideslip Angle vs mu_low
    # -------------------------------------------------------------------------
    ax5 = fig.add_subplot(gs[1, 1], facecolor=AXES_COLOR)
    style_axis(ax5, title=r"5. Peak Sideslip Angle $|\beta|_{max}$ vs $\mu_{low}$", xlabel=r"Low-$\mu$ Friction Coefficient $\mu_{low}$ [-]", ylabel=r"Sideslip Angle $|\beta|_{max}$ [deg]")
    th5 = ax5.axhline(10.0, color=ACCENT_ROSE, ls="--", lw=1.2)
    for c_key in summary.controllers:
        color, fmt, label = ctrl_styles[c_key]
        ax5.plot(mu_lows, summary.sideslip_max[c_key], fmt, color=color, lw=2.0, ms=5)
    ax5.grid(True, alpha=0.3, color=GRID_COLOR)
    ax5.legend([th5], [r"Tire Saturation Barrier ($10^\circ$)"], loc="upper right", fontsize=7.5, facecolor=CARD_COLOR, edgecolor=GRID_COLOR, labelcolor=TEXT_PRIMARY)

    # -------------------------------------------------------------------------
    # 6. Representative Plan-View Trajectories Across Friction Regimes
    # -------------------------------------------------------------------------
    ax6 = fig.add_subplot(gs[1, 2], facecolor=AXES_COLOR)
    style_axis(ax6, title=r"6. Trajectory Envelope for Selected Surfaces", xlabel=r"X Position [m]", ylabel=r"Y Position [m]")
    ax6.set_aspect("equal", adjustable="datalim")

    # Draw Road Corridor
    ax6.fill_between([-5, 80], 0.0, 4.0, color=MU_HIGH_COLOR, alpha=0.4, label=r"Left Lane ($\mu=1.00$ Dry)")
    ax6.fill_between([-5, 80], -4.0, 0.0, color=MU_LOW_COLOR, alpha=0.4, label=r"Right Lane ($\mu_{low}$ Slick)")
    ax6.axhline(0.0, color=TEXT_MUTED, ls="--", lw=0.8, alpha=0.6)

    # Select representative points
    sample_targets = [0.15, 0.35, 0.60, 1.00]
    sample_labels = {0.15: "Glaze Ice (0.15)", 0.35: "Snow (0.35)", 0.60: "Wet Asphalt (0.60)", 1.00: "Dry Asphalt (1.0)"}
    sample_colors = {0.15: "#f43f5e", 0.35: "#f59e0b", 0.60: "#38bdf8", 1.00: "#10b981"}

    for pt in summary.sweep_points:
        # Find closest sample target
        matched_target = None
        for st in sample_targets:
            if abs(pt.mu_low - st) < 0.02:
                matched_target = st
                break
        if matched_target is not None:
            c_tag = sample_colors[matched_target]
            lbl = sample_labels[matched_target]
            # Plot Passive (dashed)
            hist_pass = pt.results["passive"].history
            ax6.plot(hist_pass.X, hist_pass.Y, color=c_tag, ls="--", lw=1.3, alpha=0.85, label=f"Passive: {lbl}")
            # Plot IVMC 2WS (dash-dot) if present
            if "ivmc_2ws" in pt.results:
                hist_i2 = pt.results["ivmc_2ws"].history
                ax6.plot(hist_i2.X, hist_i2.Y, color=c_tag, ls="-.", lw=1.5, alpha=0.9, label=f"IVMC 2WS: {lbl}")
            # Plot Full-Stack IVMC + 4WS (solid)
            hist_ivmc = pt.results.get("full_ivmc_4ws", pt.results.get("integrated_ivmc")).history
            ax6.plot(hist_ivmc.X, hist_ivmc.Y, color=c_tag, ls="-", lw=2.0, label=f"IVMC 4WS: {lbl}")

    ax6.set_xlim(-2.0, 75.0)
    ax6.set_ylim(-3.5, 3.5)
    ax6.grid(True, alpha=0.3, color=GRID_COLOR)
    ax6.legend(loc="upper right", fontsize=6.8, facecolor=CARD_COLOR, edgecolor=GRID_COLOR, labelcolor=TEXT_PRIMARY, ncol=2)

    fig.savefig(out_p, bbox_inches="tight", facecolor=fig.get_facecolor(), edgecolor="none")
    plt.close(fig)
    return out_p
