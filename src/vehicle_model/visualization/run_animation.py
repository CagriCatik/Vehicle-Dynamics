"""Comparison animations of recorded runs: one GIF per scenario, several controllers side by side.

Only recorded samples are drawn (no interpolation, no re-simulation): frame ``k`` shows sample
``round(k * playback / (fps * dt))`` of every run; a run that ended earlier (stopped vehicle) stays at its last
recorded sample and is marked "ended" in the HUD. Works with the 7-DOF histories of the experiment runner
(``X, Y, psi, steer_f, steer_r``, per-wheel ``Fx_*``, ``Fy_*``, ``Fz_*``) and the 14-DOF histories of the roll
benchmark (``X, Y, psi, steer``, ``Fz_*``, ``contact_*``, ``phi``).

Layout:
    overview   all recorded paths (thin), trails up to the frame, vehicles at true scale, friction zones shaded
    close-up   the first run followed at true scale: body, wheels with steer angle, tyre-force arrows
               (longitudinal + lateral, wheel frame) or, for the 14-DOF runs, wheels without contact in red
    traces     up to three time-trace panels (channel list per animation) with a synchronised cursor
    HUD        time, speed and the per-run values of the first trace panel
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np

COLOURS = ("#3987e5", "#d95926", "#199e70", "#c98500", "#d55181", "#8a63d2", "#f5f5f5", "#7d8590")
WHEELS = ("FL", "FR", "RL", "RR")


def frame_indices(n: int, dt: float, fps: int, playback: float) -> np.ndarray:
    step = max(playback / (fps * dt), 1e-9)
    k = np.arange(int(math.floor((n - 1) / step)) + 1)
    return np.minimum(np.round(k * step).astype(int), n - 1)


def _road_patches(ax, road: dict | None, xlim, ylim):
    from matplotlib.patches import Rectangle
    if not road:
        return
    t = road.get("type")
    low = "#5a7fa8"
    if t == "split_mu":
        y0 = float(road.get("split_y", 0.0))
        lo_side = "right" if road["mu_right"] < road["mu_left"] else "left"
        ya, yb = (ylim[0], y0) if lo_side == "right" else (y0, ylim[1])
        ax.add_patch(Rectangle((xlim[0], ya), xlim[1] - xlim[0], yb - ya, color=low, alpha=0.18, zorder=0,
                               label=f"mu {min(road['mu_left'], road['mu_right'])}"))
    elif t == "mu_step":
        x0 = float(road["x_step"])
        lo = road["mu_after"] < road["mu_before"]
        xa, xb = (x0, xlim[1]) if lo else (xlim[0], x0)
        ax.add_patch(Rectangle((xa, ylim[0]), xb - xa, ylim[1] - ylim[0], color=low, alpha=0.18, zorder=0,
                               label=f"mu {min(road['mu_before'], road['mu_after'])}"))
    elif t == "uniform" and float(road.get("mu", 1.0)) < 0.9:
        ax.add_patch(Rectangle((xlim[0], ylim[0]), xlim[1] - xlim[0], ylim[1] - ylim[0], color=low,
                               alpha=0.10 + 0.15 * (1 - float(road["mu"])), zorder=0, label=f"mu {road['mu']}"))


def animate_runs(runs: list, path: str | Path, title: str, traces: list, road: dict | None = None, fps: int = 15,
                 playback: float = 1.0, dpi: int = 75, close_up: float = 9.0, max_frames: int = 220) -> Path:
    """``runs``: ``[(label, history_dict)]`` sharing one time base; ``traces``: ``[(panel title, key, scale, unit)]``."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter
    from matplotlib.patches import Polygon

    from vehicle_model.core.types import SevenDOFParams
    from vehicle_model.visualization.plan_view import VehicleParams, vehicle_polygons
    from vehicle_model.visualization.theme import TEXT_MUTED, TEXT_PRIMARY, apply_dark_theme, style_axis

    if not runs:
        raise ValueError("no runs to animate")
    apply_dark_theme()
    vp = VehicleParams.from_seven_dof_params(SevenDOFParams())
    lens = [len(h["t"]) for _, h in runs]
    n = max(lens)
    t_long = np.asarray(runs[int(np.argmax(lens))][1]["t"], float)
    dt = float(t_long[1] - t_long[0])
    t = t_long
    at = lambda h, key, i: float(h[key][min(i, len(h[key]) - 1)])          # last recorded sample after the end
    idx = frame_indices(n, dt, fps, playback)
    if len(idx) > max_frames:                       # keep the GIF small: fewer frames, same recorded samples
        idx = idx[np.linspace(0, len(idx) - 1, max_frames).round().astype(int)]
    fig = plt.figure(figsize=(16, 8.6))
    outer = fig.add_gridspec(1, 2, width_ratios=[1.75, 1.0])
    left = outer[0].subgridspec(2, 2, height_ratios=[0.9, 1.25], width_ratios=[1.0, 0.75])
    ax_w = fig.add_subplot(left[0, :])
    ax_c = fig.add_subplot(left[1, 0])
    ax_h = fig.add_subplot(left[1, 1])
    ax_h.axis("off")
    right = outer[1].subgridspec(max(len(traces), 1), 1)
    ax_t = [fig.add_subplot(right[i, 0]) for i in range(len(traces))]
    allx = np.concatenate([np.asarray(h["X"], float) for _, h in runs])
    ally = np.concatenate([np.asarray(h["Y"], float) for _, h in runs])
    pad = 5.0
    xlim, ylim = (allx.min() - pad, allx.max() + pad), (min(ally.min(), -3.0) - pad, max(ally.max(), 3.0) + pad)
    ax_w.set_xlim(*xlim)
    ax_w.set_ylim(*ylim)
    ax_w.set_aspect("equal", adjustable="box")
    _road_patches(ax_w, road, xlim, ylim)
    _road_patches(ax_c, road, (xlim[0] - 50, xlim[1] + 50), (ylim[0] - 50, ylim[1] + 50))
    style_axis(ax_w, "recorded paths (true scale)", "X [m]", "Y [m]")
    ax_c.set_aspect("equal", adjustable="box")
    style_axis(ax_c, f"close-up: {runs[0][0]}", "X [m]", "Y [m]")
    bodies, wheels, trails, arrows = [], [], [], []
    for k, (lab, h) in enumerate(runs):
        col = COLOURS[k % len(COLOURS)]
        ax_w.plot(np.asarray(h["X"]), np.asarray(h["Y"]), color=col, lw=0.6, alpha=0.3)
        trails.append(ax_w.plot([], [], color=col, lw=1.8, label=lab)[0])
        b = Polygon(np.zeros((4, 2)), closed=True, fc=col, ec=col, alpha=0.55)
        ax_w.add_patch(b)
        bodies.append(b)
    for i, (ttl, key, scale, unit) in enumerate(traces):
        for k, (lab, h) in enumerate(runs):
            if key in h:
                ax_t[i].plot(np.asarray(h["t"], float), np.asarray(h[key], float) * scale, color=COLOURS[k % len(COLOURS)], lw=1.2)
        style_axis(ax_t[i], ttl, "t [s]" if i == len(traces) - 1 else None, unit)
    cursors = [a.axvline(t[0], color=TEXT_PRIMARY, lw=1.0) for a in ax_t]
    ax_w.legend(fontsize=7, loc="upper left")
    h0 = runs[0][1]
    cb = Polygon(np.zeros((4, 2)), closed=True, fc=COLOURS[0], ec=COLOURS[0], alpha=0.5)
    ax_c.add_patch(cb)
    cw = [Polygon(np.zeros((4, 2)), closed=True, fc="#1e1e24", ec=COLOURS[0], lw=1.0) for _ in range(4)]
    for w in cw:
        ax_c.add_patch(w)
    ax_c.plot(np.asarray(h0["X"]), np.asarray(h0["Y"]), color=COLOURS[0], lw=0.8, alpha=0.4)
    arrows = [ax_c.annotate("", xy=(0, 0), xytext=(0, 0), arrowprops=dict(arrowstyle="->", color="#f5f5f5", lw=1.6))
              for _ in range(4)]
    hud = ax_h.text(0.0, 1.0, "", transform=ax_h.transAxes, va="top", family="monospace", fontsize=8.5, color=TEXT_PRIMARY)
    fig.suptitle(title)
    fig.tight_layout()
    rear = None
    if "phi" in h0:            # 14-DOF: rear view of the first run, body tilted by the recorded roll angle
        ax_r = ax_h.inset_axes([0.05, 0.02, 0.9, 0.5])
        ax_r.set_xlim(-1.6, 1.6)
        ax_r.set_ylim(-0.2, 2.0)
        ax_r.set_aspect("equal")
        ax_r.axhline(0.0, color=TEXT_MUTED, lw=1.5)
        ax_r.set_xticks([])
        ax_r.set_yticks([])
        ax_r.set_title("rear view (roll angle x3, recorded)", fontsize=8, color=TEXT_MUTED)
        rb = Polygon(np.zeros((4, 2)), closed=True, fc=COLOURS[0], ec=COLOURS[0], alpha=0.6)
        ax_r.add_patch(rb)
        rw = [Polygon(np.zeros((4, 2)), closed=True, fc="#1e1e24", ec="#e6edf6", lw=1.0) for _ in range(2)]
        for w in rw:
            ax_r.add_patch(w)
        rear = (rb, rw)
    has_forces = all(f"Fx_{w}" in h0 for w in WHEELS)
    has_contact = all(f"contact_{w}" in h0 for w in WHEELS)
    steer_key = "steer_f" if "steer_f" in h0 else "steer"
    sp = SevenDOFParams()
    pos = np.array([[sp.lf, sp.wf / 2], [sp.lf, -sp.wf / 2], [-sp.lr, sp.wr / 2], [-sp.lr, -sp.wr / 2]])

    def update(fi):
        i = int(idx[fi])
        for k, (lab, h) in enumerate(runs):
            X, Y, psi = at(h, "X", i), at(h, "Y", i), at(h, "psi", i)
            trails[k].set_data(np.asarray(h["X"])[:i + 1], np.asarray(h["Y"])[:i + 1])
            body = vehicle_polygons(X, Y, psi, at(h, steer_key, i) if steer_key in h else 0.0, vp)[0]
            bodies[k].set_xy(body.T)
        i0 = min(i, len(h0["t"]) - 1)
        X, Y, psi = at(h0, "X", i), at(h0, "Y", i), at(h0, "psi", i)
        sf = at(h0, steer_key, i) if steer_key in h0 else 0.0
        sr = at(h0, "steer_r", i) if "steer_r" in h0 else 0.0
        polys = vehicle_polygons(X, Y, psi, sf, vp, steer_rear=sr)
        cb.set_xy(polys[0].T)
        order = (polys[2], polys[1], polys[4], polys[3])         # vehicle_polygons: body, FR, FL, RR, RL
        c, s = math.cos(psi), math.sin(psi)
        for j, w in enumerate(WHEELS):
            cw[j].set_xy(order[j].T)
            lost = has_contact and at(h0, f"contact_{w}", i) < 0.5
            cw[j].set_facecolor("#d95926" if lost else "#1e1e24")
            px, py = X + c * pos[j, 0] - s * pos[j, 1], Y + s * pos[j, 0] + c * pos[j, 1]
            if has_forces:
                d = sf if j < 2 else sr
                fx, fy = at(h0, f"Fx_{w}", i), at(h0, f"Fy_{w}", i)
                ang = psi + d
                gx, gy = fx * math.cos(ang) - fy * math.sin(ang), fx * math.sin(ang) + fy * math.cos(ang)
                k_ = 1.0 / 2500.0                                      # 1 m per 2.5 kN
                arrows[j].xy = (px + gx * k_, py + gy * k_)
                arrows[j].set_position((px, py))
            else:
                arrows[j].xy = (px, py)
                arrows[j].set_position((px, py))
        if rear is not None:
            phi = -3.0 * at(h0, "phi", i)     # x3 for visibility; phi > 0 rolls to the right: right side down (clockwise)
            c2, s2 = math.cos(phi), math.sin(phi)
            box = np.array([[-0.85, 0.45], [0.85, 0.45], [0.85, 1.35], [-0.85, 1.35]])
            rear[0].set_xy(np.c_[c2 * box[:, 0] - s2 * (box[:, 1] - 0.45), s2 * box[:, 0] + c2 * (box[:, 1] - 0.45) + 0.45])
            for j, (side, w) in enumerate(((-0.8, "RL"), (0.8, "RR"))):   # viewed from behind: left wheel on the left
                lift = 0.0 if at(h0, f"contact_{w}", i) >= 0.5 else 0.12
                rear[1][j].set_xy(np.array([[side - 0.1, lift], [side + 0.1, lift], [side + 0.1, lift + 0.6], [side - 0.1, lift + 0.6]]))
                rear[1][j].set_facecolor("#d95926" if lift else "#1e1e24")
        ax_c.set_xlim(X - close_up, X + close_up)
        ax_c.set_ylim(Y - close_up, Y + close_up)
        for cur in cursors:
            cur.set_xdata([t[i], t[i]])
        v = math.hypot(at(h0, "vx", i), at(h0, "vy", i)) if "vy" in h0 else at(h0, "vx", i)
        lines = [f"t {t[i]:6.2f} s   v {v * 3.6:6.1f} km/h"]
        if traces:
            ttl, key, scale, unit = traces[0]
            for k, (lab, h) in enumerate(runs):
                if key in h:
                    ended = "  (ended)" if i >= len(h["t"]) else ""
                    lines.append(f"{lab[:30]:30s}\n   {traces[0][0]}: {at(h, key, i) * scale:8.2f} {unit}{ended}")
        if has_forces:
            lines.append("arrows: tyre force, 1 m = 2.5 kN")
        if has_contact:
            lines.append("red wheel: no contact")
        hud.set_text("\n".join(lines))
        return []
    anim = FuncAnimation(fig, update, frames=len(idx), interval=1000.0 / fps, blit=False)
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    anim.save(out, writer=PillowWriter(fps=fps), dpi=dpi)
    plt.close(fig)
    return out


def load_history(path) -> dict:
    """A history CSV (experiment runner or roll benchmark) as a dict of arrays (text columns kept as strings)."""
    import csv
    rows = list(csv.DictReader(open(path, encoding="utf-8")))
    out = {}
    for k in rows[0]:
        try:
            out[k] = np.array([float(r[k]) for r in rows])
        except ValueError:
            out[k] = np.array([r[k] for r in rows], dtype=object)
    return out


__all__ = ("animate_runs", "animate_side_view", "animate_slope", "frame_indices", "load_history")


# one side-view vehicle for every side animation (ride, grade): outline in the body frame, x from the rear axle, z up
_SIDE_OUTLINE = ((-0.9, 0.25), (3.75, 0.25), (3.75, 0.75), (3.4, 1.25), (1.6, 1.25), (-0.9, 0.85))
_SIDE_WHEELBASE, _SIDE_WHEEL_RADIUS = 2.8, 0.33


def _side_vehicle(ax, colour):
    """Add the shared side-view vehicle to ``ax``; returns (body patch, rear wheel, front wheel)."""
    from matplotlib.patches import Circle, Polygon

    from vehicle_model.visualization.theme import TEXT_PRIMARY
    body = Polygon(np.array(_SIDE_OUTLINE), closed=True, fc=colour, ec=colour, alpha=0.85)
    ax.add_patch(body)
    wheels = [Circle((0.0, 0.0), _SIDE_WHEEL_RADIUS, fc="#1e1e24", ec=TEXT_PRIMARY, lw=1.2) for _ in range(2)]
    for w in wheels:
        ax.add_patch(w)
    return body, wheels[0], wheels[1]


def _place_side_body(body, x_rear: float, z_rear: float, angle: float) -> None:
    """Pose the outline with its rear axle at (x_rear, z_rear) and the body tilted by ``angle``."""
    c, s = math.cos(angle), math.sin(angle)
    pts = np.array(_SIDE_OUTLINE)
    body.set_xy(np.c_[x_rear + c * pts[:, 0] - s * pts[:, 1], z_rear + s * pts[:, 0] + c * pts[:, 1]])


def animate_side_view(runs: list, path: str | Path, title: str, traces: list, fps: int = 15, playback: float = 1.0,
                      dpi: int = 75, gain: float = 5.0, window: float = 8.0, max_frames: int = 220) -> Path:
    """Side view of 14-DOF ride runs (``simulation.ride`` histories): one panel per run, body heave and pitch and
    wheel travel drawn over the recorded road under the left wheels, vertical motions magnified by ``gain``
    (declared in the panel); time traces with a cursor on the right. Recorded samples only."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter

    from vehicle_model.visualization.theme import TEXT_MUTED, apply_dark_theme, style_axis

    apply_dark_theme()
    a, b, R = 1.4, 1.4, _SIDE_WHEEL_RADIUS       # drawing geometry of the shared side-view vehicle
    h0 = runs[0][1]
    t = np.asarray(h0["t"], float)
    n = min(len(h["t"]) for _, h in runs)
    idx = frame_indices(n, float(t[1] - t[0]), fps, playback)
    if len(idx) > max_frames:
        idx = idx[np.linspace(0, len(idx) - 1, max_frames).round().astype(int)]
    fig = plt.figure(figsize=(16, 2.6 * len(runs) + 1.6))
    outer = fig.add_gridspec(1, 2, width_ratios=[1.6, 1.0])
    left = outer[0].subgridspec(len(runs), 1)
    right = outer[1].subgridspec(max(len(traces), 1), 1)
    ax_s = [fig.add_subplot(left[k, 0]) for k in range(len(runs))]
    ax_t = [fig.add_subplot(right[i, 0]) for i in range(len(traces))]
    art = []
    for k, (lab, h) in enumerate(runs):
        ax = ax_s[k]
        col = COLOURS[k % len(COLOURS)]
        road_x = np.asarray(h["s_road"], float) + a
        road_z = np.asarray(h["zr_FL"], float) * gain
        line, = ax.plot([], [], color=TEXT_MUTED, lw=1.5)
        body, wr, wf = _side_vehicle(ax, col)
        sf, = ax.plot([], [], color="#c98500", lw=2)
        sr, = ax.plot([], [], color="#c98500", lw=2)
        ax.set_ylim(-0.5, 2.2)
        ax.set_aspect("equal", adjustable="box")
        style_axis(ax, f"{lab}: vertical motions x{gain:g}, recorded", None, "z [m]")
        art.append((h, road_x, road_z, line, body, wf, wr, sf, sr))
    ax_s[-1].set_xlabel("travelled distance [m]")
    for i, (ttl, key, scale, unit) in enumerate(traces):
        for k, (lab, h) in enumerate(runs):
            if key in h:
                ax_t[i].plot(np.asarray(h["t"], float), np.asarray(h[key], float) * scale, color=COLOURS[k % len(COLOURS)],
                             lw=1.1, label=lab)
        style_axis(ax_t[i], ttl, "t [s]" if i == len(traces) - 1 else None, unit)
    if ax_t:
        ax_t[0].legend(fontsize=7)
    cursors = [x.axvline(t[0], color=TEXT_PRIMARY, lw=1.0) for x in ax_t]
    fig.suptitle(title)
    fig.tight_layout()

    def update(fi):
        i = int(idx[fi])
        for h, road_x, road_z, line, body, wf, wr, sf, sr in art:
            s = float(h["s_road"][i])
            sel = (road_x > s - window) & (road_x < s + window)
            line.set_data(road_x[sel], road_z[sel])
            zs, th = float(h["zs"][i]) * gain, float(h["theta"][i]) * gain
            xc, zc = s, R + zs
            c_, s_ = math.cos(th), math.sin(th)
            _place_side_body(body, xc - c_ * b, zc - s_ * b, th)
            zf, zr = R + float(h["zu_FL"][i]) * gain, R + float(h["zu_RL"][i]) * gain
            wf.center, wr.center = (s + a, zf), (s - b, zr)
            sf.set_data([s + a, s + a], [zf, zc + 0.3 + a * s_])
            sr.set_data([s - b, s - b], [zr, zc + 0.3 - b * s_])
            ax = line.axes
            ax.set_xlim(s - window, s + window)
        for cur in cursors:
            cur.set_xdata([t[i], t[i]])
        return []
    anim = FuncAnimation(fig, update, frames=len(idx), interval=1000.0 / fps, blit=False)
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    anim.save(out, writer=PillowWriter(fps=fps), dpi=dpi)
    plt.close(fig)
    return out


def animate_slope(runs: list, path: str | Path, title: str, traces: list, v_set_kph: float | None = None, fps: int = 15,
                  playback: float = 2.0, dpi: int = 75, window: float = 14.0, max_frames: int = 220) -> Path:
    """Side view of hill-descent runs (``experiments.hill_descent`` histories, true scale): the car on the recorded
    road height ``Z(X)``, tilted with the grade, wheels red while ABS is active, HUD with speed, grade and HDC
    status; time traces with a cursor. Shorter runs hold their last sample. Recorded samples only."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter
    from matplotlib.patches import Polygon

    from vehicle_model.visualization.theme import TEXT_MUTED, TEXT_PRIMARY, apply_dark_theme, style_axis

    apply_dark_theme()
    lf, lr, R = 1.4, 1.4, _SIDE_WHEEL_RADIUS     # drawing geometry of the shared side-view vehicle
    t_all = [np.asarray(h["t"], float) for _, h in runs]
    longest = int(np.argmax([len(t) for t in t_all]))
    t = t_all[longest]
    idx = frame_indices(len(t), float(t[1] - t[0]), fps, playback)
    if len(idx) > max_frames:
        idx = idx[np.linspace(0, len(idx) - 1, max_frames).round().astype(int)]
    fig = plt.figure(figsize=(16, 2.5 * len(runs) + 1.6))
    outer = fig.add_gridspec(1, 2, width_ratios=[1.5, 1.0])
    left = outer[0].subgridspec(len(runs), 1)
    right = outer[1].subgridspec(max(len(traces), 1), 1)
    ax_s = [fig.add_subplot(left[k, 0]) for k in range(len(runs))]
    ax_t = [fig.add_subplot(right[i, 0]) for i in range(len(traces))]
    art = []
    for k, (lab, h) in enumerate(runs):
        ax = ax_s[k]
        col = COLOURS[k % len(COLOURS)]
        X, Z = np.asarray(h["X"], float), np.asarray(h["Z"], float)
        ground = Polygon(np.zeros((3, 2)), closed=True, fc="#2a2a33", ec=TEXT_MUTED, lw=1.2)
        ax.add_patch(ground)
        body, wheel_r, wheel_f = _side_vehicle(ax, col)
        wheels = [wheel_r, wheel_f]
        hud = ax.text(0.01, 0.95, "", transform=ax.transAxes, va="top", fontsize=9, color=TEXT_PRIMARY, family="monospace")
        ax.set_aspect("equal", adjustable="box")
        style_axis(ax, lab, None, "Z [m]")
        art.append((h, X, Z, ground, body, wheels, hud, ax))
    ax_s[-1].set_xlabel("X [m] (true scale)")
    for i, (ttl, key, scale, unit) in enumerate(traces):
        for k, (lab, h) in enumerate(runs):
            if key in h:
                ax_t[i].plot(np.asarray(h["t"], float), np.asarray(h[key], float) * scale, color=COLOURS[k % len(COLOURS)],
                             lw=1.2, label=lab)
        if key == "speed" and v_set_kph is not None:
            ax_t[i].axhline(v_set_kph, color=TEXT_MUTED, ls="--", lw=1.0, label="set speed")
        style_axis(ax_t[i], ttl, "t [s]" if i == len(traces) - 1 else None, unit)
    if ax_t:
        ax_t[0].legend(fontsize=7)
    cursors = [x.axvline(t[0], color=TEXT_PRIMARY, lw=1.0) for x in ax_t]
    fig.suptitle(title)
    fig.tight_layout()

    def update(fi):
        tf = float(t[int(idx[fi])])
        for h, X, Z, ground, body, wheels, hud, ax in art:
            i = min(int(np.searchsorted(h["t"], tf)), len(h["t"]) - 1)
            ended = tf > float(h["t"][-1]) + 1e-9
            x0 = float(X[i])
            gx = np.linspace(x0 - window, x0 + window, 80)
            gz = np.interp(gx, X, Z)
            ground.set_xy(np.vstack([np.c_[gx, gz], [[gx[-1], gz.min() - 3.0], [gx[0], gz.min() - 3.0]]]))
            xr, xf = x0 - lr, x0 + lf
            zr, zf = float(np.interp(xr, X, Z)), float(np.interp(xf, X, Z))
            ang = math.atan2(zf - zr, xf - xr)
            c_, s_ = math.cos(ang), math.sin(ang)
            wc = [(xr - s_ * R, zr + c_ * R), (xf - s_ * R, zf + c_ * R)]
            red = float(h["abs_active_n"][i]) > 0
            for w, cen in zip(wheels, wc):
                w.center = cen
                w.set_edgecolor("#e5484d" if red else TEXT_PRIMARY)
                w.set_linewidth(2.4 if red else 1.2)
            _place_side_body(body, xr, zr, ang)
            ax.set_xlim(x0 - window, x0 + window)
            ax.set_ylim(float(np.interp(x0, X, Z)) - 5.0, float(np.interp(x0, X, Z)) + 5.0)
            status = "INFEASIBLE" if h.get("hdc_infeasible") is not None and h["hdc_infeasible"][i] > 0 else (
                "active" if h.get("hdc_active") is not None and h["hdc_active"][i] > 0 else "off/standby")
            hud.set_text(f"v {h['speed'][i] * 3.6:5.1f} km/h  grade {h['grade_pct'][i]:+5.1f} %  HDC {status}"
                         + ("  ABS" if red else "") + ("  (ended)" if ended else ""))
        for cur in cursors:
            cur.set_xdata([tf, tf])
        return []
    anim = FuncAnimation(fig, update, frames=len(idx), interval=1000.0 / fps, blit=False)
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    anim.save(out, writer=PillowWriter(fps=fps), dpi=dpi)
    plt.close(fig)
    return out
