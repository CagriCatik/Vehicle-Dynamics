"""Unified Dark Automotive Telemetry Design System for Vehicle Dynamics Visualizations.

Provides centralized color palettes and matplotlib configuration to guarantee a
consistent, professional, publication-quality aesthetic across all plots, dashboards,
and animations.
"""
from __future__ import annotations

import matplotlib as mpl
import matplotlib.pyplot as plt

# -----------------------------------------------------------------------------
# Color Palette Tokens
# -----------------------------------------------------------------------------
BG_COLOR: str = "#0b1120"        # Deep Space Slate (Figure background)
AXES_COLOR: str = "#0f172a"      # Dark Slate (Axes background)
CARD_COLOR: str = "#1e293b"      # Surface / Legend / Card background
ROAD_COLOR: str = "#1e293b"      # Dark Asphalt Road corridor
GRID_COLOR: str = "#334155"      # Subtle Slate Grid lines

TEXT_PRIMARY: str = "#f8fafc"    # Bright Off-White for titles and major text
TEXT_MUTED: str = "#94a3b8"      # Slate for labels, ticks, and secondary info
TEXT_FAINT: str = "#64748b"      # Muted slate for reference lines
TEXT_COLOR: str = TEXT_PRIMARY   # Alias for backward-compatible imports
PANEL_BG: str = CARD_COLOR       # Alias for card/panel background

ACCENT_CYAN: str = "#38bdf8"     # Cyan / Sky for 4WS / dynamic accents
ACCENT_BLUE: str = "#3b82f6"     # Vivid blue
ACCENT_EMERALD: str = "#10b981"  # Emerald green for IVMC / Pass
ACCENT_AMBER: str = "#f59e0b"    # Amber / Orange for OEM / Caution
ACCENT_ROSE: str = "#f43f5e"     # Rose / Red for Passive / Limits / Fail
ACCENT_PURPLE: str = "#a855f7"   # Violet for specialized controllers

MU_HIGH_COLOR: str = "#065f46"   # Dark Emerald for High friction road surface
MU_LOW_COLOR: str = "#0369a1"    # Dark Icy Cyan for Low friction / Split-Mu road

# -----------------------------------------------------------------------------
# Matplotlib rcParams Configuration
# -----------------------------------------------------------------------------
DARK_THEME_RC = {
    # Figure styling
    "figure.facecolor": BG_COLOR,
    "figure.edgecolor": BG_COLOR,
    "figure.titlesize": 14,
    "figure.titleweight": "bold",
    "figure.autolayout": False,

    # Axes styling
    "axes.facecolor": AXES_COLOR,
    "axes.edgecolor": GRID_COLOR,
    "axes.linewidth": 1.0,
    "axes.labelcolor": TEXT_MUTED,
    "axes.titlecolor": TEXT_PRIMARY,
    "axes.titlesize": 11,
    "axes.titleweight": "bold",
    "axes.labelsize": 10,
    "axes.labelweight": "medium",
    "axes.grid": True,

    # Ticks styling
    "xtick.color": TEXT_MUTED,
    "ytick.color": TEXT_MUTED,
    "xtick.labelsize": 9,
    "ytick.labelsize": 9,
    "xtick.direction": "out",
    "ytick.direction": "out",

    # Text styling
    "text.color": TEXT_PRIMARY,
    "font.sans-serif": ["DejaVu Sans", "Helvetica", "Arial", "sans-serif"],

    # Grid styling
    "grid.color": GRID_COLOR,
    "grid.alpha": 0.45,
    "grid.linestyle": "--",
    "grid.linewidth": 0.8,

    # Legend styling
    "legend.facecolor": CARD_COLOR,
    "legend.edgecolor": GRID_COLOR,
    "legend.labelcolor": TEXT_PRIMARY,
    "legend.fontsize": 9,
    "legend.framealpha": 0.92,
    "legend.fancybox": True,

    # Savefig styling
    "savefig.facecolor": BG_COLOR,
    "savefig.edgecolor": BG_COLOR,
    "savefig.transparent": False,
    "savefig.dpi": 150,
}


def apply_dark_theme() -> None:
    """Apply the unified dark telemetry theme to matplotlib globally."""
    mpl.rcParams.update(DARK_THEME_RC)


def style_axis(ax: plt.Axes, title: str | None = None, xlabel: str | None = None, ylabel: str | None = None) -> None:
    """Helper to ensure an individual axis strictly adheres to the dark telemetry theme."""
    ax.set_facecolor(AXES_COLOR)
    for spine in ax.spines.values():
        spine.set_edgecolor(GRID_COLOR)
        spine.set_linewidth(1.0)
    ax.tick_params(colors=TEXT_MUTED, labelsize=9)
    ax.grid(True, linestyle="--", alpha=0.45, color=GRID_COLOR, linewidth=0.8)
    if title:
        ax.set_title(title, fontsize=11, fontweight="bold", color=TEXT_PRIMARY)
    if xlabel:
        ax.set_xlabel(xlabel, fontsize=10, color=TEXT_MUTED)
    if ylabel:
        ax.set_ylabel(ylabel, fontsize=10, color=TEXT_MUTED)


# Automatically apply on import
apply_dark_theme()
