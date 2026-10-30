"""Comprehensive tyre model diagnostic and characteristic plotting suite.

Implements all required visualizations provided:
- Longitudinal force vs. slip ratio Fx(kappa)
- Lateral force vs. slip angle Fy(alpha)
- Combined-slip force curves (Fy vs alpha at various kappa; Fx vs kappa at various alpha)
- Friction circle / Kamm's ellipse (Fy vs Fx)
- Normal-load sensitivity (Fz sweep)
- Friction-coefficient sensitivity (mu sweep)
- Force saturation and tyre utilization (total force vs mu*Fz)
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt

from .factory import available_tyres, load_tyre
from .framework import TyreModel
from vehicle_model.visualization.theme import (
    apply_dark_theme,
    style_axis,
    BG_COLOR,
    PANEL_BG,
    TEXT_COLOR,
    TEXT_MUTED,
    GRID_COLOR,
    ACCENT_CYAN,
    ACCENT_EMERALD,
    ACCENT_AMBER,
    ACCENT_ROSE,
)


def plot_tyre_characteristics(
    tyre: TyreModel,
    title: str | None = None,
    figsize: tuple[float, float] = (18, 9),
) -> plt.Figure:
    """Generate an 8-panel diagnostic figure characterizing a TyreModel."""
    apply_dark_theme()
    fig, axes = plt.subplots(2, 4, figsize=figsize, facecolor=BG_COLOR)
    for ax in axes.flat:
        style_axis(ax)
    prov = tyre.provenance
    fig_title = title or f"Tyre Characteristics: {prov.formulation} [{prov.data_class}] (Fz0={prov.reference_load:.0f} N)"
    fig.suptitle(fig_title, fontsize=14, fontweight="bold", y=0.98, color=TEXT_COLOR)

    fz0 = prov.reference_load
    test_loads = [0.5 * fz0, 1.0 * fz0, 1.5 * fz0, 2.0 * fz0]
    colors = [ACCENT_CYAN, ACCENT_EMERALD, ACCENT_AMBER, ACCENT_ROSE]

    # -------------------------------------------------------------
    # 1. Pure Longitudinal Force Fx(kappa) vs Fz
    # -------------------------------------------------------------
    ax1 = axes[0, 0]
    kappas = np.linspace(-0.30, 0.30, 201)
    for fz, c in zip(test_loads, colors):
        fx, _ = tyre.forces(kappas, np.zeros_like(kappas), np.full_like(kappas, fz), np.ones_like(kappas))
        ax1.plot(kappas * 100, fx, label=f"Fz = {fz:.0f} N", color=c, lw=1.8)
    ax1.set_title("1. Pure Longitudinal Slip", fontsize=10, fontweight="bold")
    ax1.set_xlabel(r"Slip Ratio $\kappa$ [%]", fontsize=9)
    ax1.set_ylabel("Longitudinal Force $F_x$ [N]", fontsize=9)
    ax1.grid(True, alpha=0.3)
    ax1.legend(fontsize=8, loc="lower right")

    # -------------------------------------------------------------
    # 2. Pure Lateral Force Fy(alpha) vs Fz
    # -------------------------------------------------------------
    ax2 = axes[0, 1]
    alphas_deg = np.linspace(-15.0, 15.0, 201)
    alphas_rad = np.radians(alphas_deg)
    for fz, c in zip(test_loads, colors):
        _, fy = tyre.forces(np.zeros_like(alphas_rad), alphas_rad, np.full_like(alphas_rad, fz), np.ones_like(alphas_rad))
        ax2.plot(alphas_deg, fy, label=f"Fz = {fz:.0f} N", color=c, lw=1.8)
    ax2.set_title("2. Pure Lateral Slip", fontsize=10, fontweight="bold")
    ax2.set_xlabel(r"Slip Angle $\alpha$ [deg]", fontsize=9)
    ax2.set_ylabel("Lateral Force $F_y$ [N]", fontsize=9)
    ax2.grid(True, alpha=0.3)
    ax2.legend(fontsize=8, loc="lower right")

    # -------------------------------------------------------------
    # 3. Combined Slip: Fy vs alpha at various kappa
    # -------------------------------------------------------------
    ax3 = axes[0, 2]
    kappa_levels = [0.0, 0.05, 0.10, 0.20, 0.40]
    cmap_comb = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd"]
    for k_val, c in zip(kappa_levels, cmap_comb):
        _, fy = tyre.forces(
            np.full_like(alphas_rad, k_val),
            alphas_rad,
            np.full_like(alphas_rad, fz0),
            np.ones_like(alphas_rad),
        )
        ax3.plot(alphas_deg, fy, label=rf"$\kappa = {k_val:.2f}$", color=c, lw=1.8)
    ax3.set_title(r"3. Combined Slip: $F_y(\alpha, \kappa)$", fontsize=10, fontweight="bold")
    ax3.set_xlabel(r"Slip Angle $\alpha$ [deg]", fontsize=9)
    ax3.set_ylabel("Lateral Force $F_y$ [N]", fontsize=9)
    ax3.grid(True, alpha=0.3)
    ax3.legend(fontsize=8, loc="lower right")

    # -------------------------------------------------------------
    # 4. Combined Slip: Fx vs kappa at various alpha
    # -------------------------------------------------------------
    ax4 = axes[0, 3]
    alpha_levels_deg = [0.0, 2.0, 5.0, 10.0]
    for a_deg, c in zip(alpha_levels_deg, cmap_comb[:len(alpha_levels_deg)]):
        a_rad = math.radians(a_deg)
        fx, _ = tyre.forces(
            kappas,
            np.full_like(kappas, a_rad),
            np.full_like(kappas, fz0),
            np.ones_like(kappas),
        )
        ax4.plot(kappas * 100, fx, label=fr"$\alpha = {a_deg:.0f}^\circ$", color=c, lw=1.8)
    ax4.set_title(r"4. Combined Slip: $F_x(\kappa, \alpha)$", fontsize=10, fontweight="bold")
    ax4.set_xlabel(r"Slip Ratio $\kappa$ [%]", fontsize=9)
    ax4.set_ylabel("Longitudinal Force $F_x$ [N]", fontsize=9)
    ax4.grid(True, alpha=0.3)
    ax4.legend(fontsize=8, loc="lower right")

    # -------------------------------------------------------------
    # 5. Friction Ellipse / Kamm's Circle (Fy vs Fx)
    # -------------------------------------------------------------
    ax5 = axes[1, 0]
    mu_nom = 1.0
    limit = mu_nom * fz0
    theta = np.linspace(0, 2 * np.pi, 200)
    ax5.plot(limit * np.cos(theta), limit * np.sin(theta), color=TEXT_MUTED, ls="--", lw=1.5, label=r"Friction Limit $\mu F_z$")

    sweep_k = np.linspace(-0.25, 0.25, 60)
    for a_deg in [0.0, 3.0, 6.0, 10.0, 14.0]:
        a_rad = math.radians(a_deg)
        fx_pts, fy_pts = tyre.forces(sweep_k, np.full_like(sweep_k, a_rad), np.full_like(sweep_k, fz0), np.full_like(sweep_k, mu_nom))
        ax5.plot(fx_pts, fy_pts, lw=1.5, label=fr"$\alpha = {a_deg:.0f}^\circ$")

    ax5.set_title("5. Friction Ellipse / Kamm Circle", fontsize=10, fontweight="bold")
    ax5.set_xlabel("Longitudinal Force $F_x$ [N]", fontsize=9)
    ax5.set_ylabel("Lateral Force $F_y$ [N]", fontsize=9)
    ax5.set_aspect("equal", adjustable="datalim")
    ax5.grid(True, alpha=0.3)
    ax5.legend(fontsize=7, loc="lower right")

    # -------------------------------------------------------------
    # 6. Normal Load Sensitivity (Stiffness & Peak mu vs Fz)
    # -------------------------------------------------------------
    ax6 = axes[1, 1]
    fz_range = np.linspace(500.0, 2.5 * fz0, 40)
    c_alpha_norm = []
    mu_eff_peak = []
    for f in fz_range:
        # Evaluate cornering stiffness around alpha=0.01 rad
        _, fy_small = tyre.forces(0.0, 0.01, f, 1.0)
        c_a = float(fy_small) / 0.01
        c_alpha_norm.append(c_a / f)

        # Evaluate peak lateral force
        _, fy_sweep = tyre.forces(np.zeros(50), np.linspace(0.0, 0.30, 50), np.full(50, f), np.ones(50))
        mu_eff_peak.append(float(np.max(fy_sweep)) / f)

    ax6.plot(fz_range, c_alpha_norm, color=ACCENT_CYAN, lw=2.0, label=r"Norm. Cornering Stiffness $C_\alpha / F_z$ [1/rad]")
    ax6_twin = ax6.twinx()
    ax6_twin.set_facecolor("none")
    ax6_twin.tick_params(colors=TEXT_COLOR)
    ax6_twin.plot(fz_range, mu_eff_peak, color=ACCENT_ROSE, ls="--", lw=2.0, label=r"Peak Grip $\mu_{peak} = F_{y,max} / F_z$ [-]")
    ax6.set_title("6. Normal Load Sensitivity", fontsize=10, fontweight="bold")
    ax6.set_xlabel("Normal Load $F_z$ [N]", fontsize=9)
    ax6.set_ylabel(r"$C_\alpha / F_z$ [1/rad]", color=ACCENT_CYAN, fontsize=9)
    ax6_twin.set_ylabel(r"$\mu_{peak}$ [-]", color=ACCENT_ROSE, fontsize=9)
    ax6.grid(True, alpha=0.3)

    # -------------------------------------------------------------
    # 7. Friction-Coefficient Sensitivity (Road Surface variations)
    # -------------------------------------------------------------
    ax7 = axes[1, 2]
    mu_surfaces = [("Dry Asphalt", 1.0, ACCENT_EMERALD), ("Wet Asphalt", 0.7, ACCENT_CYAN), ("Snow/Gravel", 0.4, ACCENT_AMBER), ("Ice", 0.2, ACCENT_ROSE)]
    for name, mu_val, c in mu_surfaces:
        _, fy_surf = tyre.forces(np.zeros_like(alphas_rad), alphas_rad, np.full_like(alphas_rad, fz0), np.full_like(alphas_rad, mu_val))
        ax7.plot(alphas_deg, fy_surf, label=rf"{name} ($\mu={mu_val}$)", color=c, lw=1.8)
    ax7.set_title("7. Road Friction Sensitivity", fontsize=10, fontweight="bold")
    ax7.set_xlabel(r"Slip Angle $\alpha$ [deg]", fontsize=9)
    ax7.set_ylabel("Lateral Force $F_y$ [N]", fontsize=9)
    ax7.grid(True, alpha=0.3)
    ax7.legend(fontsize=8, loc="lower right")

    # -------------------------------------------------------------
    # 8. Force Saturation and Tyre Utilization
    # -------------------------------------------------------------
    ax8 = axes[1, 3]
    slip_ratios = np.linspace(0.0, 0.40, 100)
    for a_deg, c in zip([0.0, 3.0, 6.0, 12.0], cmap_comb[:4]):
        a_rad = math.radians(a_deg)
        fx_pts, fy_pts = tyre.forces(slip_ratios, np.full_like(slip_ratios, a_rad), np.full_like(slip_ratios, fz0), np.ones_like(slip_ratios))
        f_total = np.hypot(fx_pts, fy_pts)
        utilization = f_total / fz0  # mu = 1.0
        ax8.plot(slip_ratios * 100, utilization, label=fr"$\alpha = {a_deg:.0f}^\circ$", color=c, lw=1.8)

    ax8.axhline(1.0, color=TEXT_MUTED, linestyle=":", lw=1.5, label="Max Friction Bound (100%)")
    ax8.set_title("8. Tyre Utilization & Saturation", fontsize=10, fontweight="bold")
    ax8.set_xlabel(r"Slip Ratio $\kappa$ [%]", fontsize=9)
    ax8.set_ylabel(r"Total Utilization $\sqrt{F_x^2+F_y^2} / (\mu F_z)$", fontsize=9)
    ax8.set_ylim(0.0, 1.1)
    ax8.grid(True, alpha=0.3)
    ax8.legend(fontsize=8, loc="lower right")

    plt.tight_layout(rect=[0, 0, 1, 0.95])
    return fig


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Plot comprehensive tyre model characteristics.")
    parser.add_argument("--tyre", type=str, default="fiala_passenger_synthetic", help="Tyre name or YAML path.")
    parser.add_argument("--output-dir", type=Path, default=Path("output/tyres"), help="Output directory for PNG.")
    parser.add_argument("--all", action="store_true", help="Generate plots for all available tyres.")
    parser.add_argument("--no-show", action="store_true", default=True, help="Don't open interactive GUI window.")
    args = parser.parse_args(argv)

    tyre_names = available_tyres() if args.all else [args.tyre]
    args.output_dir.mkdir(parents=True, exist_ok=True)

    for name in tyre_names:
        tyre = load_tyre(name)
        fig = plot_tyre_characteristics(tyre)
        out_path = args.output_dir / f"{name}_curves.png"
        fig.savefig(out_path, dpi=180, bbox_inches="tight", facecolor=fig.get_facecolor(), edgecolor="none")
        print(f"Saved tyre diagnostic plot to {out_path}")
        if not args.no_show:
            plt.show()
        plt.close(fig)


if __name__ == "__main__":
    main()
