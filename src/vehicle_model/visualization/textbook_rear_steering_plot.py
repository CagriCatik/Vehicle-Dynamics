"""Publication-quality textbook visualization of Four-Wheel Steering (4WS) characteristics.

Formats diagrams in the exact style of modern vehicle dynamics textbooks:
- Küçükay, F. (2022): Grundlagen der Fahrzeugtechnik, Springer Vieweg, Kap. 7 (Allradlenkung).
- Heißing, B.; Ersoy, M.; Gies, M. (2020): Fahrwerkhandbuch, Springer Vieweg, Kap. 8 (Vierradlenkung).
- Mitschke, M.; Wallentowitz, H. (2014): Dynamik der Kraftfahrzeuge, Springer Vieweg, Kap. 12.
- Rajamani, R. (2012): Vehicle Dynamics and Control, Springer, Sec. 7.3.

Produces:
1. Steering ratio k(v_x) = delta_r / delta_f over forward speed (0 - 180 km/h) with shaded physical regimes.
2. Family of characteristic curves delta_r(delta_f) across discrete speeds (0, 15, 36, 60, 90, 130, 180 km/h).
3. 2D Contour map & continuous surface with annotated isolines.
4. Steady-state body sideslip angle gain beta / delta_f over speed (2WS vs. 4WS).
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Sequence

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from vehicle_model.steering.lut import LookupTable, load_lookup_table
from vehicle_model.core.types import SevenDOFParams


def plot_textbook_4ws_characteristics(
    lut: LookupTable | None = None,
    output_file: str | Path = "output/four_wheel_steering/textbook_4ws_characteristics.png",
    params: SevenDOFParams | None = None,
) -> Path:
    """Generate a comprehensive 4-panel textbook reference figure for rear-axle steering."""
    if isinstance(lut, (str, Path)):
        output_file = lut
        lut = None

    out_path = Path(output_file)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    if lut is None:
        lut = load_lookup_table()
    if params is None:
        params = SevenDOFParams.from_yaml("configs/vehicles/seven_dof_passenger.yaml")

    # High-density speed axis: 0 to 180 km/h (0 to 50 m/s)
    v_kmh = np.linspace(0.0, 180.0, 200)
    v_mps = v_kmh / 3.6

    # Linear steer angle probe for small-signal steering ratio (delta_f = 1.5 deg)
    delta_f_probe_deg = 1.5

    # 1. Compute steering ratio k(v_x) = delta_r / delta_f from LUT
    ratios_lut = np.array([
        lut.desired_rear_deg(vx, delta_f_probe_deg) / delta_f_probe_deg
        for vx in v_mps
    ])

    # Theoretical Zero-Sideslip condition:
    # k_zero_beta = (-lr + (m*lf / (C_alpha_r * L)) * vx^2) / (lf + (m*lr / (C_alpha_f * L)) * vx^2)
    m = params.mass
    lf = params.lf
    lr = params.lr
    L = params.wheelbase
    C_af = 85000.0  # N/rad
    C_ar = 95000.0  # N/rad

    ratios_theory = []
    for vx in v_mps:
        num = -lr + (m * lf / (C_ar * L)) * (vx ** 2)
        den = lf + (m * lr / (C_af * L)) * (vx ** 2)
        ratios_theory.append(num / max(den, 1e-4))
    ratios_theory = np.clip(np.array(ratios_theory), -0.50, 0.35)

    # Unified Dark Telemetry Colormap: Cyan (Counter-Phase) -> Midnight Slate (Neutral) -> Crimson Rose (In-Phase)
    from matplotlib.colors import LinearSegmentedColormap
    telemetry_cmap = LinearSegmentedColormap.from_list(
        "telemetry_4ws",
        ["#06b6d4", "#0284c7", "#0f172a", "#be123c", "#f43f5e"],
        N=256,
    )

    fig = plt.figure(figsize=(18, 14), dpi=220, facecolor="#0b1120")
    gs = fig.add_gridspec(2, 2, hspace=0.30, wspace=0.25)

    # Common dark theme styling
    bg_color = "#0f172a"
    grid_color = "#334155"
    text_color = "#f8fafc"
    subtext_color = "#94a3b8"

    # =========================================================================
    # PANEL 1: Lenkübersetzungsverhältnis k(v_x) = delta_r / delta_f
    # =========================================================================
    ax1 = fig.add_subplot(gs[0, 0], facecolor=bg_color)

    # Shaded physical regimes
    v_switch_kmh = 36.0  # approx 10 m/s
    ax1.axvspan(0.0, v_switch_kmh, color="#0ea5e9", alpha=0.12, label="Gegenphase (Agilität / Wendekreisverkürzung)")
    ax1.axvspan(v_switch_kmh, 180.0, color="#10b981", alpha=0.12, label="Gleichphase (Stabilität / Schwimmwinkel β → 0)")

    # Zero crossover line
    ax1.axhline(0.0, color="#64748b", linestyle="--", linewidth=1.2)
    ax1.axvline(v_switch_kmh, color="#f59e0b", linestyle=":", linewidth=1.5, label=f"Umschaltgeschwindigkeit v_switch = {v_switch_kmh:.0f} km/h")

    # Plot LUT ratio and theoretical zero-beta target
    ax1.plot(v_kmh, ratios_lut, color="#38bdf8", linewidth=2.8, label="Kalibriertes 4WS-Kennfeld (LUT)")
    ax1.plot(v_kmh, ratios_theory, color="#a78bfa", linestyle="--", linewidth=2.0, label="Theoretisches Zero-Sideslip Ideal (β ≡ 0)")

    ax1.set_title("A: Übersetzungsverhältnis k(v_x) = δ_r / δ_f über Fahrgeschwindigkeit\n(Küçükay 2022, Kap. 7; Heißing et al. 2020, Kap. 8)", fontsize=11, fontweight="bold", color=text_color, pad=12)
    ax1.set_xlabel("Fahrzeuggeschwindigkeit v_x [km/h]", color=subtext_color, fontsize=10)
    ax1.set_ylabel("Hinterachslenk-Verhältnis k = δ_r / δ_f [-]", color=subtext_color, fontsize=10)
    ax1.set_xlim(0.0, 180.0)
    ax1.set_ylim(-0.40, 0.30)
    ax1.grid(True, linestyle=":", color=grid_color, alpha=0.6)
    ax1.tick_params(colors=subtext_color)
    ax1.legend(loc="lower right", fontsize=8.5, facecolor="#1e293b", edgecolor=grid_color, labelcolor=text_color)

    # Annotation callouts
    ax1.annotate(
        "Parken & Rangieren\nδ_r ≈ -0.35 · δ_f\n(Wendekreis -20%)",
        xy=(5.0, -0.34),
        xytext=(15.0, -0.25),
        arrowprops=dict(facecolor="#38bdf8", edgecolor="#38bdf8", shrink=0.08, width=1.2, headwidth=6),
        color="#38bdf8",
        fontsize=8.5,
        fontweight="bold",
    )
    ax1.annotate(
        "Autobahn-Spurwechsel\nδ_r ≈ +0.18 · δ_f\n(Paralleles Krabben ohne Gierverzug)",
        xy=(130.0, 0.17),
        xytext=(85.0, 0.23),
        arrowprops=dict(facecolor="#10b981", edgecolor="#10b981", shrink=0.08, width=1.2, headwidth=6),
        color="#10b981",
        fontsize=8.5,
        fontweight="bold",
    )

    # =========================================================================
    # PANEL 2: Kennlinienschar δ_r = f(δ_f) bei diskreten Geschwindigkeiten
    # =========================================================================
    ax2 = fig.add_subplot(gs[0, 1], facecolor=bg_color)

    speeds_eval_kmh = [0.0, 18.0, 36.0, 60.0, 90.0, 130.0, 180.0]
    speed_palette = ["#38bdf8", "#0284c7", "#f59e0b", "#84cc16", "#10b981", "#059669", "#047857"]
    delta_f_dense = np.linspace(-30.0, 30.0, 120)

    for spd, col in zip(speeds_eval_kmh, speed_palette):
        vx_m = spd / 3.6
        r_dense = np.array([lut.desired_rear_deg(vx_m, df) for df in delta_f_dense])
        ls = "-" if spd != 36.0 else ":"
        lw = 2.2 if spd in (0.0, 36.0, 130.0) else 1.6
        ax2.plot(delta_f_dense, r_dense, label=f"v = {spd:3.0f} km/h", color=col, linestyle=ls, linewidth=lw)

    ax2.axhline(0.0, color="#64748b", linestyle="--", linewidth=1.0)
    ax2.axvline(0.0, color="#64748b", linestyle="--", linewidth=1.0)

    # Mechanical limits
    ax2.axhline(8.0, color="#ef4444", linestyle="--", linewidth=1.2, alpha=0.7)
    ax2.axhline(-8.0, color="#ef4444", linestyle="--", linewidth=1.2, alpha=0.7, label="Mechanischer Aktor-Anschlag (±8.0°)")

    ax2.set_title("B: Hinterachswinkel δ_r über Vorderachswinkel δ_f (Kennlinienschar)\nDegressive Hoch-G Sättigung zur Sicherung der Hinterachsführung", fontsize=11, fontweight="bold", color=text_color, pad=12)
    ax2.set_xlabel("Vorderachs-Lenkwinkel δ_f [deg]", color=subtext_color, fontsize=10)
    ax2.set_ylabel("Hinterachs-Lenkwinkel δ_r [deg]", color=subtext_color, fontsize=10)
    ax2.set_xlim(-30.0, 30.0)
    ax2.set_ylim(-9.5, 9.5)
    ax2.grid(True, linestyle=":", color=grid_color, alpha=0.6)
    ax2.tick_params(colors=subtext_color)
    ax2.legend(loc="upper left", fontsize=8.0, facecolor="#1e293b", edgecolor=grid_color, labelcolor=text_color)

    # =========================================================================
    # PANEL 3: 2D Kontur-Isolinien-Diagramm δ_r(v_x, δ_f)
    # =========================================================================
    ax3 = fig.add_subplot(gs[1, 0], facecolor=bg_color)

    v_grid_kmh = np.linspace(0.0, 180.0, 100)
    f_grid_deg = np.linspace(-30.0, 30.0, 100)
    V_kmh, F_deg = np.meshgrid(v_grid_kmh, f_grid_deg, indexing="ij")
    R_grid = np.zeros_like(V_kmh)

    for i in range(V_kmh.shape[0]):
        for j in range(V_kmh.shape[1]):
            R_grid[i, j] = lut.desired_rear_deg(V_kmh[i, j] / 3.6, F_deg[i, j])

    # Filled contour
    levels = np.linspace(-8.0, 8.0, 33)
    cs = ax3.contourf(V_kmh, F_deg, R_grid, levels=levels, cmap=telemetry_cmap, extend="both")
    
    # Annotated contour isolines
    iso_levels = [-6.0, -4.0, -2.0, -0.5, 0.0, 0.5, 1.5, 3.0]
    c_lines = ax3.contour(V_kmh, F_deg, R_grid, levels=iso_levels, colors="#f8fafc", linewidths=0.9, alpha=0.7)
    ax3.clabel(c_lines, inline=True, fontsize=8.0, fmt="%1.1f°", colors="#ffffff")

    cbar = fig.colorbar(cs, ax=ax3, orientation="vertical", pad=0.03, aspect=20)
    cbar.set_label("Hinterachswinkel δ_r [deg]", color=text_color, fontsize=9.5)
    cbar.ax.tick_params(colors=subtext_color)

    ax3.axvline(v_switch_kmh, color="#f59e0b", linestyle=":", linewidth=1.5)
    ax3.set_title("C: 2D-Isolinienfeld δ_r(v_x, δ_f) mit Nulldurchgang\nIsolinien konvergieren zur neutralen Übergangsachse", fontsize=11, fontweight="bold", color=text_color, pad=12)
    ax3.set_xlabel("Fahrzeuggeschwindigkeit v_x [km/h]", color=subtext_color, fontsize=10)
    ax3.set_ylabel("Vorderachs-Lenkwinkel δ_f [deg]", color=subtext_color, fontsize=10)
    ax3.set_xlim(0.0, 180.0)
    ax3.set_ylim(-30.0, 30.0)
    ax3.grid(True, linestyle=":", color="#475569", alpha=0.4)
    ax3.tick_params(colors=subtext_color)

    # =========================================================================
    # PANEL 4: Stationärer Schwimmwinkel β/δ_f: 2WS vs. 4WS
    # =========================================================================
    ax4 = fig.add_subplot(gs[1, 1], facecolor=bg_color)

    # Linear two-track / bicycle model steady-state sideslip calculation:
    # 2WS: beta(v) = (lr/L - (m*lf / (C_ar * L^2)) * v^2) / (1 + Kus * v^2) * delta_f
    # 4WS: beta(v) -> driven to zero by dynamic same-phase rear steering
    K_us = (m / L) * (lr / C_af - lf / C_ar)

    beta_gain_2ws = []
    beta_gain_4ws = []

    for vx in v_mps:
        # 2WS sideslip gain
        num_2ws = (lr / L) - (m * lf / (C_ar * (L ** 2))) * (vx ** 2)
        den_2ws = 1.0 + K_us * (vx ** 2)
        gain_2ws = num_2ws / max(den_2ws, 1e-4)
        beta_gain_2ws.append(gain_2ws)

        # 4WS sideslip gain with calibrated LUT ratio
        k_val = lut.desired_rear_deg(vx, 5.0) / 5.0
        # Generalized bicycle model with 4WS:
        # beta = [ (lr - (m*lf/(C_ar*L))*vx^2) - k * (lf + (m*lr/(C_af*L))*vx^2) ] / [ L*(1 + Kus*vx^2) ]
        num_4ws = (lr - (m * lf / (C_ar * L)) * (vx ** 2)) - k_val * (lf + (m * lr / (C_af * L)) * (vx ** 2))
        den_4ws = L * (1.0 + K_us * (vx ** 2))
        gain_4ws = num_4ws / max(den_4ws, 1e-4)
        beta_gain_4ws.append(gain_4ws)

    ax4.axhline(0.0, color="#64748b", linestyle="--", linewidth=1.2)
    ax4.axvline(v_switch_kmh, color="#f59e0b", linestyle=":", linewidth=1.5)

    ax4.plot(v_kmh, beta_gain_2ws, color="#f43f5e", linewidth=2.4, linestyle="--", label="Konventionelle 2WS (Heck eilt bei Hochgeschwindigkeit aus)")
    ax4.plot(v_kmh, beta_gain_4ws, color="#10b981", linewidth=2.8, label="Vierradlenkung 4WS (Schwimmwinkel β ≈ 0 am Schwerpunkt)")

    ax4.fill_between(v_kmh, beta_gain_2ws, beta_gain_4ws, color="#10b981", alpha=0.15, label="Schwimmwinkel-Dämpfung (bis zu 90% Beruhigung)")

    ax4.set_title("D: Stationäre Schwimmwinkelverstärkung β / δ_f über Geschwindigkeit\nVergleich 2-Rad-Lenkung vs. 4-Rad-Lenkung (Zero-Sideslip-Effekt)", fontsize=11, fontweight="bold", color=text_color, pad=12)
    ax4.set_xlabel("Fahrzeuggeschwindigkeit v_x [km/h]", color=subtext_color, fontsize=10)
    ax4.set_ylabel("Schwimmwinkelverstärkung dβ / dδ_f [-]", color=subtext_color, fontsize=10)
    ax4.set_xlim(0.0, 180.0)
    ax4.set_ylim(-0.25, 0.60)
    ax4.grid(True, linestyle=":", color=grid_color, alpha=0.6)
    ax4.tick_params(colors=subtext_color)
    ax4.legend(loc="upper right", fontsize=8.5, facecolor="#1e293b", edgecolor=grid_color, labelcolor=text_color)

    # Super Title
    fig.suptitle(
        "Fahrwerkdynamische Kennfeld-Charakteristik der Hinterachslenkung (4WS)\n"
        "Gegenphasige Niedriggeschwindigkeitsagilität vs. Gleichphasige Hochgeschwindigkeitsstabilität",
        fontsize=15,
        fontweight="bold",
        color=text_color,
        y=0.98,
    )

    plt.savefig(out_path, dpi=220, facecolor=fig.get_facecolor(), bbox_inches="tight")
    plt.close(fig)
    return out_path


def plot_lut_3d_surface(
    lut: LookupTable | None = None,
    output_file: str | Path = "output/four_wheel_steering/lut_3d_surface.png",
) -> Path:
    """Generate high-resolution 3D surface visualization of the rear steer lookup table."""
    if isinstance(lut, (str, Path)):
        output_file = lut
        lut = None

    out_path = Path(output_file)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    if lut is None:
        lut = load_lookup_table()

    v_kmh = np.linspace(0.0, 180.0, 60)
    delta_f_deg = np.linspace(-30.0, 30.0, 60)
    V, F = np.meshgrid(v_kmh, delta_f_deg)
    R = np.zeros_like(V)

    for i in range(V.shape[0]):
        for j in range(V.shape[1]):
            R[i, j] = lut.desired_rear_deg(V[i, j] / 3.6, F[i, j])

    fig = plt.figure(figsize=(13, 8), dpi=180, facecolor="#0b1120")
    ax = fig.add_subplot(111, projection="3d", facecolor="#0f172a")

    from matplotlib.colors import LinearSegmentedColormap
    telemetry_cmap = LinearSegmentedColormap.from_list(
        "telemetry_4ws",
        ["#06b6d4", "#0284c7", "#0f172a", "#be123c", "#f43f5e"],
        N=256,
    )

    surf = ax.plot_surface(
        V,
        F,
        R,
        cmap=telemetry_cmap,
        edgecolor="#1e293b",
        linewidth=0.2,
        alpha=0.92,
        antialiased=True,
    )

    ax.set_title(
        "3D-Kennfeld Hinterachslenkung δ_r = f(v_x, δ_f)\n"
        "Gegenphasig (δ_r < 0 bei v < 36 km/h) & Gleichphasig (δ_r > 0 bei v > 36 km/h)",
        color="#f8fafc",
        fontsize=13,
        fontweight="bold",
        pad=16,
    )
    ax.set_xlabel("Geschwindigkeit v_x [km/h]", color="#94a3b8", labelpad=10)
    ax.set_ylabel("Vorderachswinkel δ_f [deg]", color="#94a3b8", labelpad=10)
    ax.set_zlabel("Hinterachswinkel δ_r [deg]", color="#94a3b8", labelpad=10)

    ax.tick_params(colors="#94a3b8")
    ax.xaxis.pane.fill = False
    ax.yaxis.pane.fill = False
    ax.zaxis.pane.fill = False
    ax.xaxis.pane.set_edgecolor("#334155")
    ax.yaxis.pane.set_edgecolor("#334155")
    ax.zaxis.pane.set_edgecolor("#334155")

    cbar = fig.colorbar(surf, ax=ax, shrink=0.55, aspect=14, pad=0.08)
    cbar.set_label("Hinterachswinkel δ_r [deg]", color="#f8fafc", fontsize=10)
    cbar.ax.tick_params(colors="#94a3b8")

    ax.view_init(elev=28, azim=-125)

    plt.savefig(out_path, dpi=180, facecolor=fig.get_facecolor(), bbox_inches="tight")
    plt.close(fig)
    return out_path


__all__ = (
    "plot_textbook_4ws_characteristics",
    "plot_lut_3d_surface",
)


if __name__ == "__main__":
    p1 = plot_textbook_4ws_characteristics()
    print(f"Textbook 4WS characteristics figure generated successfully at: {p1}")
    p2 = plot_lut_3d_surface()
    print(f"3D LUT surface figure generated successfully at: {p2}")

