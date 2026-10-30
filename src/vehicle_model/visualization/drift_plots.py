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


def plot_initiation(h, result: dict, path: str | Path, name: str = "") -> Path:
    """Sideslip, yaw rate, speed, steer and rear torque over time with the supervisor modes as background bands."""
    import math

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    from vehicle_model.visualization.theme import TEXT_MUTED, apply_dark_theme, style_axis
    apply_dark_theme()
    fig, ax = plt.subplots(5, 1, figsize=(14, 13), sharex=True)
    t = h.time
    series = ((np.degrees(np.unwrap(np.arctan2(h.vy, h.vx))), "beta [deg]"), (np.degrees(h.r), "r [deg/s]"),
              (np.hypot(h.vx, h.vy), "v [m/s]"), (np.degrees(h.steer_f), "delta_f [deg]"), (h.T_drive[:, 2], "T rear [N m]"))
    band = {"APPROACH": _COLOURS[0], "INITIATE": _COLOURS[3], "HOLD": _COLOURS[2], "EXIT": _COLOURS[1],
            "RECOVER": _COLOURS[4], "FAILED": "#d03b3b"}
    tr = result.get("transitions", [])
    edges = [(x["t"], x["to"]) for x in tr] + [(t[-1], None)]
    for a, (y, lab) in zip(ax, series):
        a.plot(t, y, color="white", lw=1.4)
        for (t0, mode), (t1, _) in zip(edges[:-1], edges[1:]):
            if mode in band:
                a.axvspan(t0, t1, color=band[mode], alpha=0.18, lw=0)
        style_axis(a, None, None, lab)
    eq = result.get("equilibrium") or {}
    ax[1].axhline(eq.get("yaw_rate_degps", 0.0) * (1 if result["direction"] == "left" else 1), color=TEXT_MUTED, ls=":", lw=1)
    ax[0].axhline((-1 if result["direction"] == "left" else 1) * result["beta_deg"], color=TEXT_MUTED, ls=":", lw=1)
    for (t0, mode), (t1, _) in zip(edges[:-1], edges[1:]):
        if mode:
            ax[0].text(0.5 * (t0 + t1), ax[0].get_ylim()[1], mode, ha="center", va="top", fontsize=8, color=TEXT_MUTED)
    ax[-1].set_xlabel("time [s]")
    fig.suptitle(f"{name}: {result['direction']} drift, v {result['speed']:g} m/s, beta* {result['beta_deg']:g} deg, "
                 f"mu {result['mu']:g}: {'PASS' if result.get('success') else 'FAIL'} (final mode {result.get('final_mode')})")
    fig.tight_layout()
    out = Path(path)
    fig.savefig(out, dpi=110, facecolor=fig.get_facecolor())
    plt.close(fig)
    return out


def plot_tracking(traj: dict, path: str | Path, name: str = "") -> Path:
    """Driven paths (X, Y of the CoG) per case with the reference circle; one panel per case group."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    from vehicle_model.visualization.theme import TEXT_MUTED, apply_dark_theme, style_axis
    apply_dark_theme()
    groups = list(dict.fromkeys(k[0] for k in traj))
    fig, axes = plt.subplots(1, len(groups), figsize=(7 * len(groups), 7), squeeze=False)
    for ax, g in zip(axes[0], groups):
        items = [(k[1], v) for k, v in traj.items() if k[0] == g]
        drawn = False
        for (variant, (h, circ)), c in zip(items, _COLOURS * 2):
            if circ is not None and not drawn:
                cx, cy = circ.points()
                ax.plot(cx, cy, color=TEXT_MUTED, ls="--", lw=1.2, label="reference circle")
                drawn = True
            ax.plot(h.X, h.Y, color=c, lw=1.3, label=variant)
            ax.plot(h.X[0], h.Y[0], "o", color=c, ms=5)
        ax.set_aspect("equal", adjustable="datalim")
        ax.legend(fontsize=7, loc="best")
        style_axis(ax, g, "X [m]", "Y [m]")
    fig.suptitle(f"{name}: driven paths of the CoG (7-DOF plant; dot = start; synthetic parameters)")
    fig.tight_layout()
    out = Path(path)
    fig.savefig(out, dpi=110, facecolor=fig.get_facecolor())
    plt.close(fig)
    return out


def plot_run_set(res, out_dir: str | Path) -> None:
    """The five per-run figures of D6 from one recorded history (trajectory, sideslip/yaw, wheels, phase plane,
    equilibrium residuals)."""
    import math

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    from vehicle_model.visualization.theme import TEXT_MUTED, apply_dark_theme, style_axis
    apply_dark_theme()
    out = Path(out_dir)
    h = res.history
    t = h.time
    beta = np.degrees(np.unwrap(np.arctan2(h.vy, h.vx)))
    title = f"{res.scenario} / {res.variant}" + (f" [{res.label}]" if res.label else "")
    modes = [d.get("mode", "") for d in res.diagnostics]

    def bands(ax):
        start = 0
        for k in range(1, len(modes) + 1):
            if k == len(modes) or modes[k] != modes[start]:
                m = modes[start]
                if m in ("HOLD", "TRANSITION"):
                    ax.axvspan(t[start], t[min(k, len(t) - 1)], color=_COLOURS[2], alpha=0.12, lw=0)
                elif m in ("EXIT", "RECOVER", "FAILED"):
                    ax.axvspan(t[start], t[min(k, len(t) - 1)], color=_COLOURS[1], alpha=0.12, lw=0)
                start = k

    # trajectory
    fig, ax = plt.subplots(figsize=(8, 8))
    if res.circle is not None:
        cx, cy = res.circle.points()
        ax.plot(cx, cy, color=TEXT_MUTED, ls="--", lw=1.2, label="reference circle")
    ax.plot(h.X, h.Y, color=_COLOURS[0], lw=1.4, label="CoG path")
    ax.plot(h.X[0], h.Y[0], "o", color=_COLOURS[0])
    ax.set_aspect("equal", adjustable="datalim")
    ax.legend(fontsize=8)
    style_axis(ax, f"{title}: path", "X [m]", "Y [m]")
    fig.tight_layout(); fig.savefig(out / "trajectory.png", dpi=110, facecolor=fig.get_facecolor()); plt.close(fig)
    # sideslip and yaw
    fig, axs = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    axs[0].plot(t, beta, color="white", lw=1.3)
    axs[1].plot(t, np.degrees(h.r), color="white", lw=1.3)
    if res.reference_beta is not None:
        axs[0].axhline(math.degrees(res.reference_beta), color=TEXT_MUTED, ls=":")
    if res.reference_r is not None:
        axs[1].axhline(math.degrees(res.reference_r), color=TEXT_MUTED, ls=":")
    refs = [d.get("beta_ref") for d in res.diagnostics]
    if any(b is not None for b in refs):
        axs[0].plot(t[:len(refs)], [np.nan if b is None else math.degrees(b) for b in refs], color=_COLOURS[3], lw=1, ls="--",
                    label="beta_ref (schedule)")
        axs[0].legend(fontsize=8)
    for a in axs:
        bands(a)
    style_axis(axs[0], f"{title}: sideslip (green: HOLD/TRANSITION, orange: EXIT/RECOVER)", None, "beta [deg]")
    style_axis(axs[1], None, "time [s]", "r [deg/s]")
    fig.tight_layout(); fig.savefig(out / "sideslip_and_yaw.png", dpi=110, facecolor=fig.get_facecolor()); plt.close(fig)
    # wheel torques and slip
    fig, axs = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    axs[0].plot(t, h.T_drive[:, 2], color=_COLOURS[0], lw=1.2, label="T RL")
    axs[0].plot(t, h.T_drive[:, 3], color=_COLOURS[1], lw=1.2, ls="--", label="T RR")
    axs[1].plot(t, h.kappa[:, 2], color=_COLOURS[0], lw=1.2, label="kappa RL")
    axs[1].plot(t, h.kappa[:, 3], color=_COLOURS[1], lw=1.2, label="kappa RR")
    axs[2].plot(t, np.degrees(h.steer_f), color="white", lw=1.2)
    for a in axs:
        bands(a)
    axs[0].legend(fontsize=8); axs[1].legend(fontsize=8)
    style_axis(axs[0], f"{title}: rear drive torque", None, "[N m]")
    style_axis(axs[1], "rear longitudinal slip (plant definition, bounded to [-1, 1])", None, "kappa [-]")
    style_axis(axs[2], "front steer", "time [s]", "delta_f [deg]")
    fig.tight_layout(); fig.savefig(out / "wheel_torques_and_slip.png", dpi=110, facecolor=fig.get_facecolor()); plt.close(fig)
    # phase plane
    fig, ax = plt.subplots(figsize=(8, 7))
    ax.plot(beta, np.degrees(h.r), color=_COLOURS[0], lw=1.2)
    ax.plot(beta[0], math.degrees(h.r[0]), "o", color=_COLOURS[0], label="start")
    if res.reference_beta is not None and res.reference_r is not None:
        ax.plot(math.degrees(res.reference_beta), math.degrees(res.reference_r), "*", color=_COLOURS[3], ms=14,
                label="drift equilibrium")
    ax.legend(fontsize=8)
    style_axis(ax, f"{title}: phase plane", "beta [deg]", "r [deg/s]")
    fig.tight_layout(); fig.savefig(out / "phase_plane.png", dpi=110, facecolor=fig.get_facecolor()); plt.close(fig)
    # equilibrium residuals
    fig, ax = plt.subplots(figsize=(8, 5))
    if res.equilibrium:
        from vehicle_model.handling_analysis.equilibrium import RESIDUAL_TOL
        r = res.equilibrium["residual"]
        names = list(r)
        ax.bar(names, [r[n] / RESIDUAL_TOL[n] for n in names], color=_COLOURS[0])
        ax.axhline(1.0, color=_COLOURS[1], ls="--", label="tolerance")
        ax.set_yscale("log")
        ax.legend(fontsize=8)
        ex = res.equilibrium.get("extrapolating")
        style_axis(ax, f"target equilibrium residuals / tolerance ({res.equilibrium['validity']}"
                       + (f"; tyre extrapolates: {[w for w, x in zip(('FL', 'FR', 'RL', 'RR'), ex) if x]}" if ex and any(ex) else "")
                       + ")", None, "residual / tolerance")
    else:
        ax.text(0.5, 0.5, "no target equilibrium for this run", ha="center", va="center", color=TEXT_MUTED,
                transform=ax.transAxes)
        ax.axis("off")
    fig.tight_layout(); fig.savefig(out / "equilibrium_residuals.png", dpi=110, facecolor=fig.get_facecolor()); plt.close(fig)


__all__ = ("plot_equilibrium_sweep", "plot_hold_runs", "plot_initiation", "plot_run_set", "plot_tracking",
           "plot_tyre_audit")
