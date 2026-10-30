"""Drift animation from recorded simulation data only (Drift plan D6).

Every frame shows recorded samples, nothing else: the vehicle is drawn at the recorded
``X, Y, psi`` with the recorded front steer (``vehicle_polygons`` of :mod:`.plan_view`, the
primitive behind ``draw_vehicle``); no position or angle is corrected or interpolated. A frame
``k`` shows sample ``i = round(k * playback_speed / (fps * dt))``.

Layout: overview (whole recorded path, reference circle and centre, trails of all runs),
close-up following the first run at true scale (body axis, velocity vector, sideslip angle,
counter-steer visible), HUD (time, km/h, beta, r, delta_f, kappa RL/RR, T RL/RR, actuator
saturation, drift mode), and beta, r and steer/torque over time with a synchronised cursor.
Several runs (e.g. open loop vs drift LQR) share the axes; failed runs carry their label.
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np

_RUN_COLOURS = ("#3987e5", "#d95926", "#199e70", "#c98500")


def frame_indices(n_samples: int, dt: float, fps: int, playback_speed: float) -> np.ndarray:
    """Sample index per frame (recorded samples only)."""
    step = playback_speed / (fps * dt)
    k = np.arange(int(math.floor((n_samples - 1) / step)) + 1)
    return np.minimum(np.round(k * step).astype(int), n_samples - 1)


def animate_drift(runs: list, path: str | Path, fps: int = 20, playback_speed: float = 1.0, dpi: int = 90,
                  close_up_m: float = 9.0, title: str = "") -> Path:
    """GIF of one or more :class:`~vehicle_model.simulation.drift.runner.DriftRunResult` (same time base)."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter
    from matplotlib.patches import Polygon

    from vehicle_model.visualization.plan_view import VehicleParams, vehicle_polygons
    from vehicle_model.visualization.theme import TEXT_MUTED, TEXT_PRIMARY, apply_dark_theme, style_axis

    if not runs:
        raise ValueError("no runs to animate")
    apply_dark_theme()
    h0 = runs[0].history
    dt = float(h0.time[1] - h0.time[0])
    n = min(r.history.time.size for r in runs)
    idx = frame_indices(n, dt, fps, playback_speed)
    vp = VehicleParams.from_seven_dof_params(runs[0].params)

    fig = plt.figure(figsize=(16, 9))
    gs = fig.add_gridspec(3, 3, width_ratios=[1.25, 1.0, 1.15])
    ax_w = fig.add_subplot(gs[:, 0])
    ax_c = fig.add_subplot(gs[:2, 1])
    ax_h = fig.add_subplot(gs[2, 1])
    ax_b = fig.add_subplot(gs[0, 2])
    ax_r = fig.add_subplot(gs[1, 2], sharex=ax_b)
    ax_u = fig.add_subplot(gs[2, 2], sharex=ax_b)

    # overview: whole recorded path, equal scale, fixed limits
    allx = np.concatenate([r.history.X[:n] for r in runs])
    ally = np.concatenate([r.history.Y[:n] for r in runs])
    circles = [r.circle for r in runs if r.circle is not None]
    for c in circles[:1]:
        cx, cy = c.points()
        allx, ally = np.r_[allx, cx], np.r_[ally, cy]
        ax_w.plot(cx, cy, color=TEXT_MUTED, ls="--", lw=1.2, label="reference circle")
        ax_w.plot(c.cx, c.cy, "+", color=TEXT_MUTED, ms=10)
    pad = 8.0
    ax_w.set_xlim(allx.min() - pad, allx.max() + pad)
    ax_w.set_ylim(ally.min() - pad, ally.max() + pad)
    ax_w.set_aspect("equal", adjustable="datalim")
    style_axis(ax_w, "overview: recorded CoG paths", "X [m]", "Y [m]")
    ax_c.set_aspect("equal", adjustable="box")
    style_axis(ax_c, "close-up (true scale), first run", "X [m]", "Y [m]")
    ax_h.axis("off")

    t = h0.time[:n]
    trails, dots, bodies, wheels = [], [], [], []
    for k, (r, col) in enumerate(zip(runs, _RUN_COLOURS)):
        h = r.history
        lab = r.variant + ("" if r.label in ("", "PASS") else f" [{r.label}]")
        ax_w.plot(h.X[:n], h.Y[:n], color=col, lw=0.6, alpha=0.25)
        trails.append(ax_w.plot([], [], color=col, lw=1.6, label=lab)[0])
        dots.append(ax_w.plot([], [], "o", color=col, ms=5)[0])
        beta = np.degrees(np.unwrap(np.arctan2(h.vy[:n], h.vx[:n])))
        ax_b.plot(t, beta, color=col, lw=1.3)
        ax_r.plot(t, np.degrees(h.r[:n]), color=col, lw=1.3)
        ax_u.plot(t, np.degrees(h.steer_f[:n]), color=col, lw=1.3)
        if k == 0:
            ax_u2 = ax_u.twinx()
            ax_u2.plot(t, h.T_drive[:n, 2], color=col, lw=0.9, ls=":")
            ax_u2.set_ylabel("T RL [N m] (dotted)", color=TEXT_MUTED, fontsize=8)
        polys = [Polygon(np.zeros((4, 2)), closed=True, fc=col, ec=col, alpha=0.35 if k else 0.5, lw=1.2) for _ in range(1)]
        whl = [Polygon(np.zeros((4, 2)), closed=True, fc="#1e1e24", ec=col, lw=1.0) for _ in range(4)]
        for pch in polys + whl:
            ax_c.add_patch(pch)
        bodies.append(polys[0])
        wheels.append(whl)
    if runs[0].reference_beta is not None:
        ax_b.axhline(math.degrees(runs[0].reference_beta), color=TEXT_MUTED, ls=":", lw=1)
    ax_w.legend(fontsize=7, loc="upper right")
    style_axis(ax_b, "sideslip beta [deg]", None, None)
    style_axis(ax_r, "yaw rate r [deg/s]", None, None)
    style_axis(ax_u, "front steer delta_f [deg] (solid)", "time [s]", None)
    cursors = [a.axvline(t[0], color=TEXT_PRIMARY, lw=1.0) for a in (ax_b, ax_r, ax_u)]
    if circles:
        cx, cy = circles[0].points()
        ax_c.plot(cx, cy, color=TEXT_MUTED, ls="--", lw=1.0)
    ax_c.plot(h0.X[:n], h0.Y[:n], color=_RUN_COLOURS[0], lw=0.8, alpha=0.4)
    vel = ax_c.annotate("", xy=(0, 0), xytext=(0, 0), arrowprops=dict(arrowstyle="->", color="#c98500", lw=2))
    axis_line, = ax_c.plot([], [], color=TEXT_PRIMARY, lw=1.0, ls="--")
    hud = ax_h.text(0.0, 1.0, "", va="top", ha="left", family="monospace", fontsize=9, color=TEXT_PRIMARY,
                    transform=ax_h.transAxes)
    fig.suptitle(title or "Drift (recorded 7-DOF data only; synthetic parameters)")
    fig.tight_layout()

    def update(fi):
        i = int(idx[fi])
        arts = []
        for k, r in enumerate(runs):
            h = r.history
            trails[k].set_data(h.X[:i + 1], h.Y[:i + 1])
            dots[k].set_data([h.X[i]], [h.Y[i]])
            body, fr, fl, rr, rl = vehicle_polygons(float(h.X[i]), float(h.Y[i]), float(h.psi[i]), float(h.steer_f[i]), vp,
                                                    steer_rear=float(h.steer_r[i]))
            bodies[k].set_xy(body.T)
            for pch, w in zip(wheels[k], (fr, fl, rr, rl)):
                pch.set_xy(w.T)
            arts += [trails[k], dots[k], bodies[k], *wheels[k]]
        h = h0
        x, y, psi = float(h.X[i]), float(h.Y[i]), float(h.psi[i])
        ax_c.set_xlim(x - close_up_m, x + close_up_m)
        ax_c.set_ylim(y - close_up_m, y + close_up_m)
        chi = psi + math.atan2(h.vy[i], h.vx[i])
        L = 0.45 * close_up_m
        vel.xy = (x + L * math.cos(chi), y + L * math.sin(chi))
        vel.set_position((x, y))
        axis_line.set_data([x, x + L * math.cos(psi)], [y, y + L * math.sin(psi)])
        for c in cursors:
            c.set_xdata([t[i], t[i]])
        d = runs[0].diagnostics[i] if runs[0].diagnostics else {}
        v = math.hypot(h.vx[i], h.vy[i])
        lines = [f"{runs[0].variant}  [{d.get('mode', '-')}]", f"t      {t[i]:7.2f} s", f"speed  {v * 3.6:7.1f} km/h",
                 f"beta   {math.degrees(math.atan2(h.vy[i], h.vx[i])):7.1f} deg", f"r      {math.degrees(h.r[i]):7.1f} deg/s",
                 f"delta  {math.degrees(h.steer_f[i]):7.1f} deg", f"kappa  RL {h.kappa[i, 2]:5.2f}  RR {h.kappa[i, 3]:5.2f}",
                 f"T      RL {h.T_drive[i, 2]:5.0f}  RR {h.T_drive[i, 3]:5.0f} N m",
                 f"actuator saturated: {'yes' if d.get('saturated') else 'no'}"]
        for r in runs[1:]:
            hh = r.history
            lines.append(f"{r.variant}: beta {math.degrees(math.atan2(hh.vy[i], hh.vx[i])):6.1f} deg"
                         + ("" if r.label in ("", "PASS") else f"  [{r.label}]"))
        hud.set_text("\n".join(lines))
        return arts + [vel, axis_line, hud, *cursors]

    anim = FuncAnimation(fig, update, frames=len(idx), interval=1000.0 / fps, blit=False)
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    anim.save(out, writer=PillowWriter(fps=fps), dpi=dpi)
    plt.close(fig)
    return out


__all__ = ("animate_drift", "frame_indices")
