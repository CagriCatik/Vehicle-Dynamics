"""Plots for the drift work packages (Drift plan D1/D6); data in, figures out, no controller logic."""

from __future__ import annotations

from pathlib import Path

import numpy as np

_COLOURS = ("#3987e5", "#d95926", "#199e70", "#c98500", "#d55181", "#008300")  # validated categorical slots 1-6


def plot_tyre_audit(audits: list, path: str | Path) -> Path:
    """Lateral and longitudinal curves, combined-slip utilisation and check results, one row per tyre."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    from vehicle_model.visualization.theme import TEXT_MUTED, apply_dark_theme, style_axis
    apply_dark_theme()
    fig, axes = plt.subplots(len(audits), 3, figsize=(18, 4.6 * len(audits)), squeeze=False)
    for row, rep in zip(axes, audits):
        a_deg = np.degrees(rep.alpha)
        s = rep.settings
        mu = s.mus[0]
        for Fz, c in zip(s.loads, _COLOURS):
            row[0].plot(a_deg, rep.lateral[(Fz, mu, 0.0)] / Fz, color=c, lw=2, label=f"F_z {Fz / 1e3:g} kN, kappa 0")
        Fz0 = s.loads[len(s.loads) // 2]
        for k, ls in zip(s.lateral_kappas[1:], ("--", "-.", ":")):
            row[0].plot(a_deg, rep.lateral[(Fz0, mu, k)] / Fz0, color=_COLOURS[1], lw=1.4, ls=ls,
                        label=f"F_z {Fz0 / 1e3:g} kN, kappa {k:g}")
        for a, c in zip(s.longitudinal_alphas_deg, _COLOURS):
            row[1].plot(rep.kappa, rep.longitudinal[(Fz0, mu, a)] / Fz0, color=c, lw=2, label=f"alpha {a:g} deg")
        prov = rep.provenance
        for ax in row[:2]:
            ax.axhline(0.0, color=TEXT_MUTED, lw=0.8)
        va = float(np.degrees(min(abs(rep.valid_alpha[0]), abs(rep.valid_alpha[1]), np.radians(s.alpha_max_deg))))
        row[0].axvspan(-s.alpha_max_deg, -va, color=TEXT_MUTED, alpha=0.15)
        row[0].axvspan(va, s.alpha_max_deg, color=TEXT_MUTED, alpha=0.15, label="outside declared validity")
        row[0].legend(fontsize=7, loc="lower right")
        row[1].legend(fontsize=7, loc="lower right")
        style_axis(row[0], f"{rep.name}: lateral force (mu {mu:g})", "slip angle alpha [deg]", "F_y / F_z [-]")
        style_axis(row[1], f"{rep.name}: longitudinal force (F_z {Fz0 / 1e3:g} kN)", "slip kappa [-] (plant definition)",
                   "F_x / F_z [-]")
        U = rep.utilisation
        im = row[2].imshow(U, origin="lower", aspect="auto", cmap="viridis",
                           extent=(rep.kappa[0], rep.kappa[-1], 0.0, s.alpha_max_deg))
        # one contour only: at exactly 1.0 saturating tyres have a plateau, and its rounding noise draws speckles
        row[2].contour(rep.kappa, np.linspace(0.0, s.alpha_max_deg, U.shape[0]), U, levels=[0.9],
                       colors=["white"], linestyles=["--"], linewidths=1.2)
        row[2].axhline(va, color="white", lw=1, ls=":")
        fig.colorbar(im, ax=row[2], shrink=0.85, label="sqrt(Fx^2+Fy^2) / (mu F_z)")
        style_axis(row[2], f"{rep.name}: combined slip utilisation (dashed 0.9, dotted: validity limit)", "kappa [-]",
                   "alpha [deg]")
        row[2].grid(False)
        fails = [c["name"] for c in rep.checks if c["passed"] is False]
        row[0].text(0.01, 0.98, ("audit PASS" if not fails else "audit FAIL: " + ", ".join(fails)) + f"\n{prov[:70]}",
                    transform=row[0].transAxes, va="top", fontsize=7, color=TEXT_MUTED)
    fig.suptitle("Tyre audit for the drift range (synthetic tyre parameters; shaded: model extrapolates)")
    fig.tight_layout()
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=120, facecolor=fig.get_facecolor())
    plt.close(fig)
    return out


__all__ = ("plot_tyre_audit",)
