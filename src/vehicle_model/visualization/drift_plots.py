"""Plots for the drift work packages (Drift plan D1/D6); data in, figures out, no controller logic."""

from __future__ import annotations

from pathlib import Path

import numpy as np

_COLOURS = ("#3987e5", "#d95926", "#199e70", "#c98500", "#d55181", "#008300")  # validated categorical slots 1-6
_COLOURS_8 = _COLOURS + ("#8a63d2", "#7d8590")


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


_VALIDITY_MARKERS = {"valid": ("o", True), "valid_extrapolated": ("o", False), "infeasible": ("x", True)}


def plot_equilibrium_sweep(eqs: list, path: str | Path, name: str = "") -> Path:
    """Radius, counter-steer, rear torque and rear slips over |beta|; one curve per speed, one row per friction.

    Filled circles: valid; open circles: valid but a tyre extrapolates; crosses at the bottom of each panel:
    no feasible equilibrium. Left drifts only (right drifts are mirror images; the CSV has both).
    """
    import math

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D

    from vehicle_model.visualization.theme import TEXT_MUTED, apply_dark_theme, style_axis
    apply_dark_theme()
    left = [e for e in eqs if e.reference.direction == "left"] or eqs
    mus = sorted({e.reference.road_mu for e in left}, reverse=True)
    speeds = sorted({e.reference.speed_mps for e in left})
    fig, axes = plt.subplots(len(mus), 4, figsize=(20, 4.6 * len(mus)), squeeze=False)
    for row, mu in zip(axes, mus):
        for v, c in zip(speeds, _COLOURS * 3):
            pts = sorted((e for e in left if e.reference.road_mu == mu and e.reference.speed_mps == v),
                         key=lambda e: abs(e.reference.beta_rad))
            ok = [e for e in pts if e.physical_feasible]
            b = [abs(math.degrees(e.reference.beta_rad)) for e in ok]
            vals = ([e.radius for e in ok], [math.degrees(e.steer_front) for e in ok],
                    [e.wheel_drive_torque[2] for e in ok], [e.diagnostics["kappa"][2] for e in ok])
            for ax, y in zip(row, vals):
                ax.plot(b, y, color=c, lw=1.6)
                for bi, yi, e in zip(b, y, ok):
                    mk, filled = _VALIDITY_MARKERS[e.validity]
                    ax.plot(bi, yi, mk, color=c, mfc=c if filled else "none", mew=1.6, ms=6)
            row[3].plot(b, [e.diagnostics["kappa"][3] for e in ok], color=c, lw=1.2, ls=":")
            bad = [abs(math.degrees(e.reference.beta_rad)) for e in pts if not e.physical_feasible]
            for ax in row:
                lo = ax.get_ylim()[0] if ax.lines else 0.0
                ax.plot(bad, [lo] * len(bad), "x", color=c, ms=7, mew=1.8, clip_on=False)
        style_axis(row[0], f"mu {mu:g}: drift radius", "|beta| [deg]", "R = v/|r| [m]")
        style_axis(row[1], f"mu {mu:g}: front steer (left drift)", "|beta| [deg]", "delta_f [deg]")
        style_axis(row[2], f"mu {mu:g}: drive torque per rear wheel", "|beta| [deg]", "T [N m]")
        style_axis(row[3], f"mu {mu:g}: rear slip (solid inner RL, dotted outer RR)", "|beta| [deg]", "kappa [-]")
        row[1].axhline(0.0, color=TEXT_MUTED, lw=0.8)
    handles = [Line2D([], [], color=c, lw=2, label=f"v = {v:g} m/s") for v, c in zip(speeds, _COLOURS * 3)]
    handles += [Line2D([], [], ls="", marker="o", color=TEXT_MUTED, label="valid"),
                Line2D([], [], ls="", marker="o", mfc="none", color=TEXT_MUTED, mew=1.6, label="valid, tyre extrapolates"),
                Line2D([], [], ls="", marker="x", color=TEXT_MUTED, mew=1.8, label="no feasible equilibrium")]
    fig.legend(handles=handles, loc="lower center", ncol=len(handles), fontsize=8, frameon=False)
    fig.suptitle(f"{name}: stationary drift equilibria of the 7-DOF plant (RWD, synthetic parameters)")
    fig.tight_layout(rect=(0, 0.05, 1, 0.97))
    out = Path(path)
    fig.savefig(out, dpi=120, facecolor=fig.get_facecolor())
    plt.close(fig)
    return out


def plot_hold_runs(histories: dict, equilibria: dict, path: str | Path, name: str = "") -> Path:
    """Sideslip and yaw rate over time per direction: open loop (dashed) vs LQR (solid), one colour per perturbation."""
    import math

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    from vehicle_model.visualization.theme import TEXT_MUTED, apply_dark_theme, style_axis
    apply_dark_theme()
    dirs = list(equilibria)
    perts = list(dict.fromkeys(k[1] for k in histories))
    fig, axes = plt.subplots(len(dirs), 2, figsize=(16, 4.8 * len(dirs)), squeeze=False)
    for row, d in zip(axes, dirs):
        e = equilibria[d]
        for pn, c in zip(perts, _COLOURS_8 * 2):
            for variant, ls in (("open_loop", "--"), ("lqr", "-")):
                h = histories.get((d, pn, variant))
                if h is None:
                    continue
                beta = np.degrees(np.unwrap(np.arctan2(h.vy, h.vx)))   # continuous through spins (no +-180 jumps)
                lab = f"{pn} ({'LQR' if variant == 'lqr' else 'open loop'})"
                row[0].plot(h.time, beta, color=c, ls=ls, lw=1.6 if variant == "lqr" else 1.0, label=lab)
                row[1].plot(h.time, np.degrees(h.r), color=c, ls=ls, lw=1.6 if variant == "lqr" else 1.0)
        row[0].axhline(e["beta_deg"], color=TEXT_MUTED, lw=1, ls=":")
        row[1].axhline(e["yaw_rate_degps"], color=TEXT_MUTED, lw=1, ls=":")
        lim = abs(e["beta_deg"])
        row[0].set_ylim(e["beta_deg"] - 2.5 * lim, e["beta_deg"] + 2.5 * lim)
        row[1].set_ylim(e["yaw_rate_degps"] - 40, e["yaw_rate_degps"] + 40)
        style_axis(row[0], f"{d} drift: sideslip (dotted: equilibrium)", "time [s]", "beta [deg]")
        style_axis(row[1], f"{d} drift: yaw rate", "time [s]", "r [deg/s]")
        row[0].legend(fontsize=6, ncol=2, loc="best")
    fig.suptitle(f"{name}: hold and perturbation recovery, 7-DOF plant (dashed: constant u*, solid: LQR; synthetic parameters)")
    fig.tight_layout()
    out = Path(path)
    fig.savefig(out, dpi=120, facecolor=fig.get_facecolor())
    plt.close(fig)
    return out


__all__ = ("plot_equilibrium_sweep", "plot_hold_runs", "plot_tyre_audit")
