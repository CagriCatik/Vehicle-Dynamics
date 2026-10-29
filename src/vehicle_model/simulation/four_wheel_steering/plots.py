"""Static 4WS dashboards (six and nine scenarios)."""

from __future__ import annotations

import matplotlib
matplotlib.use("Agg")
import math
from pathlib import Path
from typing import Dict
import matplotlib.pyplot as plt
import numpy as np
from vehicle_model.visualization.plan_view import VehicleParams, draw_vehicle

from .results import ScenarioRunResult


# =============================================================================
# MULTI-SCENARIO DASHBOARD WITH PHYSICAL 2D VEHICLE FOOTPRINTS
# =============================================================================
def plot_4ws_six_scenario_dashboard(
    results: Dict[str, Dict[str, ScenarioRunResult]],
    output_file: str | Path,
) -> None:
    """Generate a comprehensive 6-panel dashboard featuring 2D vehicle rendering on all planar views."""
    out_path = Path(output_file)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    v_params = VehicleParams(wheelbase=2.80, width=1.80, length=4.50, tire_radius=0.32, tire_width=0.22, wheel_track=1.60,
                             rear_overhang=0.91, front_overhang=0.79, cog_to_rear_axle=1.5)   # CoG positions, l_r = 1.5 m

    fig = plt.figure(figsize=(24, 18), dpi=180, facecolor="#0b1120")
    gs = fig.add_gridspec(3, 2, hspace=0.34, wspace=0.22)

    colors = {
        "2WS (Front-Only)": "#94a3b8",
        "4WS Speed-Scheduled": "#f59e0b",
        "4WS Calibrated LUT": "#06b6d4",
        "4WS Calibrated LUT + Yaw Damping": "#3b82f6",
        "Full Integrated IVMC": "#10b981",
    }

    # -------------------------------------------------------------------------
    # Panel 1: Low-Speed Turning Circle (12 km/h) with 2D Vehicle Footprints
    # -------------------------------------------------------------------------
    ax1 = fig.add_subplot(gs[0, 0], facecolor="#0f172a")
    res_s1 = results.get("turning_circle", {})

    for name, r in res_s1.items():
        c = colors.get(name, "#e2e8f0")
        ls = "--" if "2WS" in name else "-"
        lbl = f"{name} (R = {r.metrics.min_turning_radius_m:.2f} m)"
        ax1.plot(r.X, r.Y, color=c, linestyle=ls, linewidth=2.2, label=lbl)

        # Draw vehicle snapshots at 0%, 33%, 66%, 100%
        n_pts = len(r.X)
        for pct, alp in [(0.15, 0.45), (0.50, 0.65), (0.85, 0.90)]:
            idx = int(pct * (n_pts - 1))
            draw_vehicle(
                x=r.X[idx],
                y=r.Y[idx],
                yaw=r.psi[idx],
                steer_front=math.radians(r.steer_f_deg[idx]),
                params=v_params,
                steer_rear=math.radians(r.steer_r_deg[idx]),
                ax=ax1,
                color=c,
                alpha=alp,
                fill_alpha=0.20,
            )

    ax1.set_aspect("equal", adjustable="box")
    ax1.set_title("1. Low-Speed Turning Circle (12 km/h, 25° Steer) — Counter-Phase (Agilität)", color="#f8fafc", fontsize=11, fontweight="bold")
    ax1.set_xlabel("X Position [m]", color="#94a3b8")
    ax1.set_ylabel("Y Position [m]", color="#94a3b8")
    ax1.grid(True, linestyle=":", color="#334155", alpha=0.6)
    ax1.tick_params(colors="#94a3b8")
    ax1.legend(loc="upper left", fontsize=8.0, facecolor="#1e293b", edgecolor="#475569", labelcolor="#f8fafc")

    # -------------------------------------------------------------------------
    # Panel 2: Urban 90° Cornering & Curb Clearance (15 km/h)
    # -------------------------------------------------------------------------
    ax2 = fig.add_subplot(gs[0, 1], facecolor="#0f172a")
    res_s2 = results.get("urban_90deg", {})

    # Draw curb corner boundary
    curb_x = np.array([-5.0, 10.0, 10.0])
    curb_y = np.array([-2.5, -2.5, -25.0])
    ax2.plot(curb_x, curb_y, color="#ef4444", linestyle="-", linewidth=2.0, label="Bordstein / Straßenbegrenzung")

    for name, r in res_s2.items():
        c = colors.get(name, "#e2e8f0")
        ls = "--" if "2WS" in name else "-"
        ax2.plot(r.X, r.Y, color=c, linestyle=ls, linewidth=2.2, label=name)

        # Draw vehicle snapshots around corner
        n_pts = len(r.X)
        for pct in [0.15, 0.45, 0.75]:
            idx = int(pct * (n_pts - 1))
            draw_vehicle(
                x=r.X[idx],
                y=r.Y[idx],
                yaw=r.psi[idx],
                steer_front=math.radians(r.steer_f_deg[idx]),
                params=v_params,
                steer_rear=math.radians(r.steer_r_deg[idx]),
                ax=ax2,
                color=c,
                alpha=0.75,
                fill_alpha=0.25,
            )

    ax2.set_aspect("equal", adjustable="box")
    ax2.set_title("2. Städtisches 90°-Einbiegen (15 km/h) — Virtuelle Radstandsverkürzung", color="#f8fafc", fontsize=11, fontweight="bold")
    ax2.set_xlabel("X Position [m]", color="#94a3b8")
    ax2.set_ylabel("Y Position [m]", color="#94a3b8")
    ax2.grid(True, linestyle=":", color="#334155", alpha=0.6)
    ax2.tick_params(colors="#94a3b8")
    ax2.legend(loc="lower left", fontsize=8.0, facecolor="#1e293b", edgecolor="#475569", labelcolor="#f8fafc")

    # -------------------------------------------------------------------------
    # Panel 3: Evasive Double Lane Change (80 km/h ISO 3888-2)
    # -------------------------------------------------------------------------
    ax3 = fig.add_subplot(gs[1, 0], facecolor="#0f172a")
    res_s3 = results.get("double_lane_change", {})

    # Draw lane corridors
    ax3.axhline(0.0, color="#475569", linestyle=":", linewidth=1.2)
    ax3.axhline(3.50, color="#475569", linestyle=":", linewidth=1.2, label="Soll-Fahrspur (Y = 3.5 m)")

    for name, r in res_s3.items():
        c = colors.get(name, "#e2e8f0")
        ls = "--" if "2WS" in name else "-"
        lbl = f"{name} (|β|_max = {r.metrics.peak_sideslip_deg:.2f}°)"
        ax3.plot(r.X, r.Y, color=c, linestyle=ls, linewidth=2.2, label=lbl)

        # Vehicle footprints during lane reversal
        n_pts = len(r.X)
        for pct in [0.25, 0.50, 0.75]:
            idx = int(pct * (n_pts - 1))
            draw_vehicle(
                x=r.X[idx],
                y=r.Y[idx],
                yaw=r.psi[idx],
                steer_front=math.radians(r.steer_f_deg[idx]),
                params=v_params,
                steer_rear=math.radians(r.steer_r_deg[idx]),
                ax=ax3,
                color=c,
                alpha=0.70,
                fill_alpha=0.20,
            )

    ax3.set_aspect("equal", adjustable="box")
    ax3.set_title("3. Ausweichtest / Doppelspurwechsel (80 km/h ISO 3888-2) — Zero-Sideslip Krabben", color="#f8fafc", fontsize=11, fontweight="bold")
    ax3.set_xlabel("X Position [m]", color="#94a3b8")
    ax3.set_ylabel("Y Position [m]", color="#94a3b8")
    ax3.grid(True, linestyle=":", color="#334155", alpha=0.6)
    ax3.tick_params(colors="#94a3b8")
    ax3.legend(loc="lower right", fontsize=8.0, facecolor="#1e293b", edgecolor="#475569", labelcolor="#f8fafc")

    # -------------------------------------------------------------------------
    # Panel 4: High-Speed Slalom Maneuver (70 km/h, 30m Pylons)
    # -------------------------------------------------------------------------
    ax4 = fig.add_subplot(gs[1, 1], facecolor="#0f172a")
    res_s4 = results.get("slalom", {})

    # Draw slalom pylons
    pylons_x = [10.0, 40.0, 70.0, 100.0]
    pylons_y = [0.0, 0.0, 0.0, 0.0]
    ax4.scatter(pylons_x, pylons_y, color="#f59e0b", marker="^", s=90, zorder=6, label="Slalom-Pylonen (30 m Abstand)")

    for name, r in res_s4.items():
        c = colors.get(name, "#e2e8f0")
        ls = "--" if "2WS" in name else "-"
        ax4.plot(r.X, r.Y, color=c, linestyle=ls, linewidth=2.2, label=name)

        n_pts = len(r.X)
        for pct in [0.20, 0.45, 0.70]:
            idx = int(pct * (n_pts - 1))
            draw_vehicle(
                x=r.X[idx],
                y=r.Y[idx],
                yaw=r.psi[idx],
                steer_front=math.radians(r.steer_f_deg[idx]),
                params=v_params,
                steer_rear=math.radians(r.steer_r_deg[idx]),
                ax=ax4,
                color=c,
                alpha=0.70,
                fill_alpha=0.20,
            )

    ax4.set_aspect("equal", adjustable="box")
    ax4.set_title("4. Hochgeschwindigkeits-Slalom (70 km/h) — Phasenverzugselimination", color="#f8fafc", fontsize=11, fontweight="bold")
    ax4.set_xlabel("X Position [m]", color="#94a3b8")
    ax4.set_ylabel("Y Position [m]", color="#94a3b8")
    ax4.grid(True, linestyle=":", color="#334155", alpha=0.6)
    ax4.tick_params(colors="#94a3b8")
    ax4.legend(loc="lower right", fontsize=8.0, facecolor="#1e293b", edgecolor="#475569", labelcolor="#f8fafc")

    # -------------------------------------------------------------------------
    # Panel 5: Autobahn Lane Change at 130 km/h (Sideslip Beta Angle Suppression)
    # -------------------------------------------------------------------------
    ax5 = fig.add_subplot(gs[2, 0], facecolor="#0f172a")
    res_s5 = results.get("autobahn", {})

    for name, r in res_s5.items():
        c = colors.get(name, "#e2e8f0")
        ls = "--" if "2WS" in name else "-"
        lbl = f"{name} (|β|_max = {r.metrics.peak_sideslip_deg:.2f}°)"
        ax5.plot(r.time, r.beta_deg, color=c, linestyle=ls, linewidth=2.2, label=lbl)

    ax5.axhline(0.0, color="#64748b", linestyle="--", linewidth=1.0)
    ax5.set_title("5. Autobahn-Spurwechsel (130 km/h) — Schwimmwinkelvergleich β(t)", color="#f8fafc", fontsize=11, fontweight="bold")
    ax5.set_xlabel("Zeit [s]", color="#94a3b8")
    ax5.set_ylabel("Karosserie-Schwimmwinkel β [deg]", color="#94a3b8")
    ax5.grid(True, linestyle=":", color="#334155", alpha=0.6)
    ax5.tick_params(colors="#94a3b8")
    ax5.legend(loc="upper right", fontsize=8.5, facecolor="#1e293b", edgecolor="#475569", labelcolor="#f8fafc")

    # -------------------------------------------------------------------------
    # Panel 6: Step Steer Response (100 km/h ISO 7401)
    # -------------------------------------------------------------------------
    ax6 = fig.add_subplot(gs[2, 1], facecolor="#0f172a")
    res_s6 = results.get("step_steer", {})

    for name, r in res_s6.items():
        c = colors.get(name, "#e2e8f0")
        ls = "--" if "2WS" in name else "-"
        lbl = f"{name} (Verzug = {r.metrics.response_delay_ms:.1f} ms)"
        ax6.plot(r.time, r.ay_mps2, color=c, linestyle=ls, linewidth=2.2, label=lbl)

    ax6.set_title("6. Sprunglenk-Übergangsverhalten (100 km/h ISO 7401) — Querbeschleunigung a_y(t)", color="#f8fafc", fontsize=11, fontweight="bold")
    ax6.set_xlabel("Zeit [s]", color="#94a3b8")
    ax6.set_ylabel("Querbeschleunigung a_y [m/s²]", color="#94a3b8")
    ax6.grid(True, linestyle=":", color="#334155", alpha=0.6)
    ax6.tick_params(colors="#94a3b8")
    ax6.legend(loc="lower right", fontsize=8.5, facecolor="#1e293b", edgecolor="#475569", labelcolor="#f8fafc")

    fig.suptitle(
        "Vierradlenkung (4WS) Multi-Szenario Gesamtevaluation mit 2D-Fahrzeuggeometrie\n"
        "Gegenphasige Agilität beim Rangieren vs. Gleichphasige Zero-Sideslip Krabbenbewegung bei Autobahntempo",
        fontsize=16,
        fontweight="bold",
        color="#f8fafc",
        y=0.985,
    )

    plt.savefig(out_path, dpi=180, facecolor=fig.get_facecolor(), bbox_inches="tight")
    plt.close(fig)


# =============================================================================
# 9-SCENARIO COMPREHENSIVE BENCHMARK DASHBOARD WITH 2D VEHICLE FOOTPRINTS
# =============================================================================
def plot_4ws_nine_scenario_dashboard(
    results: Dict[str, Dict[str, ScenarioRunResult]],
    output_file: str | Path,
) -> None:
    """Generate a comprehensive 9-panel dashboard featuring 2D vehicle rendering across all 9 scenarios."""
    out_path = Path(output_file)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    v_params = VehicleParams(wheelbase=2.80, width=1.80, length=4.50, tire_radius=0.32, tire_width=0.22, wheel_track=1.60,
                             rear_overhang=0.91, front_overhang=0.79, cog_to_rear_axle=1.5)   # CoG positions, l_r = 1.5 m

    fig = plt.figure(figsize=(26, 22), dpi=180, facecolor="#0b1120")
    gs = fig.add_gridspec(3, 3, hspace=0.36, wspace=0.24)

    colors = {
        "2WS (Front-Only)": "#94a3b8",
        "4WS Speed-Scheduled": "#f59e0b",
        "4WS Calibrated LUT": "#06b6d4",
        "4WS Calibrated LUT + Yaw Damping": "#3b82f6",
        "Full Integrated IVMC": "#10b981",
    }

    # -------------------------------------------------------------------------
    # Panel 1: Low-Speed Turning Circle (12 km/h)
    # -------------------------------------------------------------------------
    ax1 = fig.add_subplot(gs[0, 0], facecolor="#0f172a")
    res_s1 = results.get("turning_circle", {})
    for name, r in res_s1.items():
        c = colors.get(name, "#e2e8f0")
        ls = "--" if "2WS" in name else "-"
        lbl = f"{name} (R = {r.metrics.min_turning_radius_m:.2f} m)"
        ax1.plot(r.X, r.Y, color=c, linestyle=ls, linewidth=2.0, label=lbl)
        n_pts = len(r.X)
        for pct, alp in [(0.15, 0.45), (0.50, 0.65), (0.85, 0.90)]:
            idx = int(pct * (n_pts - 1))
            draw_vehicle(
                x=r.X[idx], y=r.Y[idx], yaw=r.psi[idx],
                steer_front=math.radians(r.steer_f_deg[idx]),
                params=v_params, steer_rear=math.radians(r.steer_r_deg[idx]),
                ax=ax1, color=c, alpha=alp, fill_alpha=0.20,
            )
    ax1.set_aspect("equal", adjustable="box")
    ax1.set_title("1. Wendekreis (12 km/h, 25° Steer) — Gegenphase", color="#f8fafc", fontsize=10.5, fontweight="bold")
    ax1.set_xlabel("X [m]", color="#94a3b8")
    ax1.set_ylabel("Y [m]", color="#94a3b8")
    ax1.grid(True, linestyle=":", color="#334155", alpha=0.6)
    ax1.tick_params(colors="#94a3b8")
    ax1.legend(loc="upper left", fontsize=7.5, facecolor="#1e293b", edgecolor="#475569", labelcolor="#f8fafc")

    # -------------------------------------------------------------------------
    # Panel 2: Urban 90° Cornering (15 km/h)
    # -------------------------------------------------------------------------
    ax2 = fig.add_subplot(gs[0, 1], facecolor="#0f172a")
    res_s2 = results.get("urban_90deg", {})
    curb_x = np.array([-5.0, 10.0, 10.0])
    curb_y = np.array([-2.5, -2.5, -25.0])
    ax2.plot(curb_x, curb_y, color="#ef4444", linestyle="-", linewidth=2.0, label="Bordsteinbegrenzung")
    for name, r in res_s2.items():
        c = colors.get(name, "#e2e8f0")
        ls = "--" if "2WS" in name else "-"
        ax2.plot(r.X, r.Y, color=c, linestyle=ls, linewidth=2.0, label=name)
        n_pts = len(r.X)
        for pct in [0.15, 0.45, 0.75]:
            idx = int(pct * (n_pts - 1))
            draw_vehicle(
                x=r.X[idx], y=r.Y[idx], yaw=r.psi[idx],
                steer_front=math.radians(r.steer_f_deg[idx]),
                params=v_params, steer_rear=math.radians(r.steer_r_deg[idx]),
                ax=ax2, color=c, alpha=0.75, fill_alpha=0.25,
            )
    ax2.set_aspect("equal", adjustable="box")
    ax2.set_title("2. Städtisches 90°-Einbiegen (15 km/h) — Bordsteinabstand", color="#f8fafc", fontsize=10.5, fontweight="bold")
    ax2.set_xlabel("X [m]", color="#94a3b8")
    ax2.set_ylabel("Y [m]", color="#94a3b8")
    ax2.grid(True, linestyle=":", color="#334155", alpha=0.6)
    ax2.tick_params(colors="#94a3b8")
    ax2.legend(loc="lower left", fontsize=7.5, facecolor="#1e293b", edgecolor="#475569", labelcolor="#f8fafc")

    # -------------------------------------------------------------------------
    # Panel 3: Reverse Parallel Parking (5 km/h)
    # -------------------------------------------------------------------------
    ax3 = fig.add_subplot(gs[0, 2], facecolor="#0f172a")
    res_s8 = results.get("parallel_parking", {})
    # Draw parking bay & obstacle outlines
    ax3.axhline(0.0, color="#64748b", linestyle="-", linewidth=1.8, label="Bordstein (Y = 0m)")
    ax3.axhline(2.0, color="#475569", linestyle="--", linewidth=1.2, label="Parkstreifenlinie (Y = 2m)")
    ax3.plot([7.5, 3.0, 3.0, 7.5, 7.5], [0.1, 0.1, 1.9, 1.9, 0.1], color="#94a3b8", linestyle="--", linewidth=1.2, label="Voraus-Fahrzeug")
    ax3.plot([-4.5, -9.0, -9.0, -4.5, -4.5], [0.1, 0.1, 1.9, 1.9, 0.1], color="#94a3b8", linestyle="--", linewidth=1.2, label="Nachfolge-Fahrzeug")
    ax3.plot([3.0, -4.5, -4.5, 3.0], [0.0, 0.0, 2.0, 2.0], color="#10b981", linestyle="--", linewidth=1.5, label="Parkbucht (7.5 m)")

    for name, r in res_s8.items():
        c = colors.get(name, "#e2e8f0")
        ls = "--" if "2WS" in name else "-"
        lbl = f"{name} (Y_end = {r.Y[-1]:.2f}m, Δψ = {r.metrics.final_heading_dev_deg:.2f}°)"
        ax3.plot(r.X, r.Y, color=c, linestyle=ls, linewidth=2.0, label=lbl)
        n_pts = len(r.X)
        for pct in [0.0, 0.40, 1.0]:
            idx = int(pct * (n_pts - 1))
            draw_vehicle(
                x=r.X[idx], y=r.Y[idx], yaw=r.psi[idx],
                steer_front=math.radians(r.steer_f_deg[idx]),
                params=v_params, steer_rear=math.radians(r.steer_r_deg[idx]),
                ax=ax3, color=c, alpha=0.85, fill_alpha=0.28,
            )
    ax3.set_xlim(-10.0, 8.5)
    ax3.set_ylim(-0.8, 3.6)
    ax3.set_aspect("equal", adjustable="box")
    ax3.set_title("3. Rückwärts-Einparken (5 km/h) — Heck-Eindrehung & Hüllflächenreduktion", color="#f8fafc", fontsize=10.5, fontweight="bold")
    ax3.set_xlabel("X [m]", color="#94a3b8")
    ax3.set_ylabel("Y [m]", color="#94a3b8")
    ax3.grid(True, linestyle=":", color="#334155", alpha=0.6)
    ax3.tick_params(colors="#94a3b8")
    ax3.legend(loc="upper left", fontsize=7.2, facecolor="#1e293b", edgecolor="#475569", labelcolor="#f8fafc")

    # -------------------------------------------------------------------------
    # Panel 4: Double Lane Change (80 km/h ISO 3888-2)
    # -------------------------------------------------------------------------
    ax4 = fig.add_subplot(gs[1, 0], facecolor="#0f172a")
    res_s3 = results.get("double_lane_change", {})
    ax4.axhline(0.0, color="#475569", linestyle=":", linewidth=1.0)
    ax4.axhline(3.50, color="#475569", linestyle=":", linewidth=1.0, label="Soll-Fahrspur (Y = 3.5 m)")
    for name, r in res_s3.items():
        c = colors.get(name, "#e2e8f0")
        ls = "--" if "2WS" in name else "-"
        lbl = f"{name} (|β|_max = {r.metrics.peak_sideslip_deg:.2f}°)"
        ax4.plot(r.X, r.Y, color=c, linestyle=ls, linewidth=2.0, label=lbl)
        n_pts = len(r.X)
        for pct in [0.25, 0.50, 0.75]:
            idx = int(pct * (n_pts - 1))
            draw_vehicle(
                x=r.X[idx], y=r.Y[idx], yaw=r.psi[idx],
                steer_front=math.radians(r.steer_f_deg[idx]),
                params=v_params, steer_rear=math.radians(r.steer_r_deg[idx]),
                ax=ax4, color=c, alpha=0.70, fill_alpha=0.20,
            )
    ax4.set_aspect("equal", adjustable="box")
    ax4.set_title("4. Doppelspurwechsel (80 km/h ISO 3888-2) — Krabben", color="#f8fafc", fontsize=10.5, fontweight="bold")
    ax4.set_xlabel("X [m]", color="#94a3b8")
    ax4.set_ylabel("Y [m]", color="#94a3b8")
    ax4.grid(True, linestyle=":", color="#334155", alpha=0.6)
    ax4.tick_params(colors="#94a3b8")
    ax4.legend(loc="lower right", fontsize=7.5, facecolor="#1e293b", edgecolor="#475569", labelcolor="#f8fafc")

    # -------------------------------------------------------------------------
    # Panel 5: High-Speed Slalom (70 km/h, 30m Pylons)
    # -------------------------------------------------------------------------
    ax5 = fig.add_subplot(gs[1, 1], facecolor="#0f172a")
    res_s4 = results.get("slalom", {})
    pylons_x = [10.0, 40.0, 70.0, 100.0]
    pylons_y = [0.0, 0.0, 0.0, 0.0]
    ax5.scatter(pylons_x, pylons_y, color="#f59e0b", marker="^", s=80, zorder=6, label="Slalom-Pylonen (30 m)")
    for name, r in res_s4.items():
        c = colors.get(name, "#e2e8f0")
        ls = "--" if "2WS" in name else "-"
        ax5.plot(r.X, r.Y, color=c, linestyle=ls, linewidth=2.0, label=name)
        n_pts = len(r.X)
        for pct in [0.20, 0.45, 0.70]:
            idx = int(pct * (n_pts - 1))
            draw_vehicle(
                x=r.X[idx], y=r.Y[idx], yaw=r.psi[idx],
                steer_front=math.radians(r.steer_f_deg[idx]),
                params=v_params, steer_rear=math.radians(r.steer_r_deg[idx]),
                ax=ax5, color=c, alpha=0.70, fill_alpha=0.20,
            )
    ax5.set_aspect("equal", adjustable="box")
    ax5.set_title("5. Slalom (70 km/h, 30 m) — Phasenverzugselimination", color="#f8fafc", fontsize=10.5, fontweight="bold")
    ax5.set_xlabel("X [m]", color="#94a3b8")
    ax5.set_ylabel("Y [m]", color="#94a3b8")
    ax5.grid(True, linestyle=":", color="#334155", alpha=0.6)
    ax5.tick_params(colors="#94a3b8")
    ax5.legend(loc="lower right", fontsize=7.5, facecolor="#1e293b", edgecolor="#475569", labelcolor="#f8fafc")

    # -------------------------------------------------------------------------
    # Panel 6: Split-μ Straight-Line Emergency Braking (100 km/h ECE-R13H)
    # -------------------------------------------------------------------------
    ax6 = fig.add_subplot(gs[1, 2], facecolor="#0f172a")
    res_s7 = results.get("split_mu", {})
    # Render Split-Mu surface background
    ax6.axhspan(0.0, 5.0, color="#0284c7", alpha=0.10, label="High-μ Spur (μ = 1.0, Trockener Asphalt)")
    ax6.axhspan(-5.0, 0.0, color="#93c5fd", alpha=0.10, label="Low-μ Spur (μ = 0.2, Nasses Eis)")
    ax6.axhline(0.0, color="#f59e0b", linestyle="-", linewidth=1.5, label="Reibwertsprung-Kante (Y = 0)")

    for name, r in res_s7.items():
        c = colors.get(name, "#e2e8f0")
        ls = "--" if "2WS" in name else "-"
        lbl = f"{name} (Δψ = {r.metrics.final_heading_dev_deg:.1f}°)"
        ax6.plot(r.X, r.Y, color=c, linestyle=ls, linewidth=2.0, label=lbl)
        n_pts = len(r.X)
        for pct in [0.20, 0.55, 0.90]:
            if n_pts > 5:
                idx = int(pct * (n_pts - 1))
                draw_vehicle(
                    x=r.X[idx], y=r.Y[idx], yaw=r.psi[idx],
                    steer_front=math.radians(r.steer_f_deg[idx]),
                    params=v_params, steer_rear=math.radians(r.steer_r_deg[idx]),
                    ax=ax6, color=c, alpha=0.75, fill_alpha=0.25,
                )
    ax6.set_aspect("equal", adjustable="box")
    ax6.set_title("6. Split-μ Bremsung (100 km/h ECE-R13H) — Aktive Gierdämpfung", color="#f8fafc", fontsize=10.5, fontweight="bold")
    ax6.set_xlabel("X [m]", color="#94a3b8")
    ax6.set_ylabel("Y [m]", color="#94a3b8")
    ax6.set_ylim(-3.0, 3.0)
    ax6.grid(True, linestyle=":", color="#334155", alpha=0.6)
    ax6.tick_params(colors="#94a3b8")
    ax6.legend(loc="upper left", fontsize=7.5, facecolor="#1e293b", edgecolor="#475569", labelcolor="#f8fafc")

    # -------------------------------------------------------------------------
    # Panel 7: Autobahn Lane Change (130 km/h) — Sideslip Beta(t)
    # -------------------------------------------------------------------------
    ax7 = fig.add_subplot(gs[2, 0], facecolor="#0f172a")
    res_s5 = results.get("autobahn", {})
    for name, r in res_s5.items():
        c = colors.get(name, "#e2e8f0")
        ls = "--" if "2WS" in name else "-"
        lbl = f"{name} (|β|_max = {r.metrics.peak_sideslip_deg:.2f}°)"
        ax7.plot(r.time, r.beta_deg, color=c, linestyle=ls, linewidth=2.0, label=lbl)
    ax7.axhline(0.0, color="#64748b", linestyle="--", linewidth=1.0)
    ax7.set_title("7. Autobahn-Spurwechsel (130 km/h) — Schwimmwinkel β(t)", color="#f8fafc", fontsize=10.5, fontweight="bold")
    ax7.set_xlabel("Zeit [s]", color="#94a3b8")
    ax7.set_ylabel("Schwimmwinkel β [deg]", color="#94a3b8")
    ax7.grid(True, linestyle=":", color="#334155", alpha=0.6)
    ax7.tick_params(colors="#94a3b8")
    ax7.legend(loc="upper right", fontsize=7.5, facecolor="#1e293b", edgecolor="#475569", labelcolor="#f8fafc")

    # -------------------------------------------------------------------------
    # Panel 8: Step Steer Response (100 km/h ISO 7401) — Ay(t)
    # -------------------------------------------------------------------------
    ax8 = fig.add_subplot(gs[2, 1], facecolor="#0f172a")
    res_s6 = results.get("step_steer", {})
    for name, r in res_s6.items():
        c = colors.get(name, "#e2e8f0")
        ls = "--" if "2WS" in name else "-"
        lbl = f"{name} (Verzug = {r.metrics.response_delay_ms:.1f} ms)"
        ax8.plot(r.time, r.ay_mps2, color=c, linestyle=ls, linewidth=2.0, label=lbl)
    ax8.set_title("8. Sprunglenkung (100 km/h ISO 7401) — Querbeschleunigung a_y(t)", color="#f8fafc", fontsize=10.5, fontweight="bold")
    ax8.set_xlabel("Zeit [s]", color="#94a3b8")
    ax8.set_ylabel("a_y [m/s²]", color="#94a3b8")
    ax8.grid(True, linestyle=":", color="#334155", alpha=0.6)
    ax8.tick_params(colors="#94a3b8")
    ax8.legend(loc="lower right", fontsize=7.5, facecolor="#1e293b", edgecolor="#475569", labelcolor="#f8fafc")

    # -------------------------------------------------------------------------
    # Panel 9: NHTSA Sine with Dwell (80 km/h FMVSS 126) — Yaw Rate r(t)
    # -------------------------------------------------------------------------
    ax9 = fig.add_subplot(gs[2, 2], facecolor="#0f172a")
    res_s9 = results.get("sine_with_dwell", {})
    for name, r in res_s9.items():
        c = colors.get(name, "#e2e8f0")
        ls = "--" if "2WS" in name else "-"
        lbl = f"{name} (Status: {r.metrics.stability_status})"
        ax9.plot(r.time, [math.degrees(val) for val in r.r], color=c, linestyle=ls, linewidth=2.0, label=lbl)
    ax9.axhline(0.0, color="#64748b", linestyle="--", linewidth=1.0)
    ax9.axvspan(1.57, 2.07, color="#ef4444", alpha=0.15, label="500 ms Dwell Pause")
    ax9.set_title("9. NHTSA Sine with Dwell (80 km/h FMVSS 126) — Gierrate r(t)", color="#f8fafc", fontsize=10.5, fontweight="bold")
    ax9.set_xlabel("Zeit [s]", color="#94a3b8")
    ax9.set_ylabel("Gierrate r [deg/s]", color="#94a3b8")
    ax9.grid(True, linestyle=":", color="#334155", alpha=0.6)
    ax9.tick_params(colors="#94a3b8")
    ax9.legend(loc="upper right", fontsize=7.5, facecolor="#1e293b", edgecolor="#475569", labelcolor="#f8fafc")

    fig.suptitle(
        "Vierradlenkung (4WS) 9-Szenarien Gesamtevaluation mit 2D-Fahrzeuggeometrie\n"
        "Gegenphasige Agilität, Gleichphasiges Krabben, Aktive Split-μ Gierstabilität & NHTSA FMVSS 126 Dämpfung",
        fontsize=16,
        fontweight="bold",
        color="#f8fafc",
        y=0.985,
    )

    plt.savefig(out_path, dpi=180, facecolor=fig.get_facecolor(), bbox_inches="tight")
    plt.close(fig)
