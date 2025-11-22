# Planar vehicle (front-steer only) with correct wheel orientation and rotation.
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple
import numpy as np


def _get_pyplot():
    import matplotlib.pyplot as plt
    return plt


# ----------------------------
# Parameters and validation
# ----------------------------

@dataclass
class VehicleParams:
    wheelbase: float = 2.9
    width: float = 2.0
    length: float = 4.9
    tire_radius: float = 0.4   # used as half wheel LENGTH proxy in plan view
    tire_width: float = 0.3    # used as wheel WIDTH in plan view
    wheel_track: float = 1.8   # full distance between wheel centers
    rear_overhang: float = 1.0
    front_overhang: float = 1.0

    @property
    def half_track(self) -> float:
        return 0.5 * self.wheel_track

    @property
    def wheel_len(self) -> float:
        # Make wheels long along +x/-x (rolling direction) so steer is visually obvious.
        return 2.0 * self.tire_radius

    @property
    def wheel_w(self) -> float:
        return self.tire_width


def _R_row(theta: float) -> np.ndarray:
    # Row-vector rotation: p' = p @ R, CCW for +theta.
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, s], [-s, c]], dtype=float)


def _apply_row_rot(poly_2xN: np.ndarray, R_row: np.ndarray) -> np.ndarray:
    return (poly_2xN.T @ R_row).T


def _validate_params(p: VehicleParams) -> None:
    expected_len = p.rear_overhang + p.wheelbase + p.front_overhang
    if not math.isclose(p.length, expected_len, rel_tol=1e-6, abs_tol=1e-9):
        p.length = expected_len


# ----------------------------
# Geometry (2xN, row-rotated)
# ----------------------------

def _body_polygon(p: VehicleParams) -> np.ndarray:
    x_front = p.wheelbase + p.front_overhang
    x_rear = -p.rear_overhang
    y_half = 0.5 * p.width
    return np.array(
        [[x_rear, x_rear, x_front, x_front, x_rear],
         [ y_half, -y_half, -y_half,  y_half,  y_half]],
        dtype=float
    )


def _wheel_polygon(p: VehicleParams) -> np.ndarray:
    # IMPORTANT: long in x (wheel_len), narrow in y (wheel_w).
    L = 0.5 * p.wheel_len
    W = 0.5 * p.wheel_w
    return np.array(
        [[ L, -L, -L,  L,  L],
         [-W, -W,  W,  W, -W]],
        dtype=float
    )


def _vehicle_parts(p: VehicleParams, yaw: float, steer_front: float) -> List[np.ndarray]:
    _validate_params(p)

    body = _body_polygon(p)
    wheel = _wheel_polygon(p)

    # Steering for front wheels only
    R_f = _R_row(steer_front)
    fr = _apply_row_rot(wheel, R_f)
    fl = _apply_row_rot(wheel, R_f)

    # Rear wheels not steered
    rr = wheel.copy()
    rl = wheel.copy()

    # Translate wheels in vehicle frame
    fr += np.array([[p.wheelbase], [-p.half_track]])
    fl += np.array([[p.wheelbase], [ p.half_track]])
    rr += np.array([[0.0],        [-p.half_track]])
    rl += np.array([[0.0],        [ p.half_track]])

    # Apply body yaw to all parts
    R_yaw = _R_row(yaw)
    parts = [body, fr, fl, rr, rl]
    parts = [_apply_row_rot(poly, R_yaw) for poly in parts]
    return parts


def _translate_all(parts: List[np.ndarray], x: float, y: float) -> List[np.ndarray]:
    off = np.array([[x], [y]], dtype=float)
    return [poly + off for poly in parts]


# ----------------------------
# Arrow primitive
# ----------------------------

def draw_arrow(x: float, y: float, theta: float, length: float, color: str) -> None:
    plt = _get_pyplot()
    ang = math.radians(30.0)
    dx = length * math.cos(theta)
    dy = length * math.sin(theta)
    x_end = x + dx
    y_end = y + dy
    plt.plot([x, x_end], [y, y_end], color=color, linewidth=2)
    hL = 0.4 * length
    lt = theta + math.pi - ang
    rt = theta + math.pi + ang
    lx = x_end + hL * math.cos(lt); ly = y_end + hL * math.sin(lt)
    rx = x_end + hL * math.cos(rt); ry = y_end + hL * math.sin(rt)
    plt.plot([x_end, lx], [y_end, ly], color=color, linewidth=2)
    plt.plot([x_end, rx], [y_end, ry], color=color, linewidth=2)


# ----------------------------
# Public API
# ----------------------------

def draw_vehicle(
    x: float,
    y: float,
    yaw: float,
    steer_front: float,
    params: VehicleParams,
    *,
    color_front: str = "tab:blue",
    color_rear: str = "tab:orange",
    outline_color: str = "black",
    arrow_color: Optional[str] = None,
    color: Optional[str] = None
) -> None:
    plt = _get_pyplot()

    if color is not None:
        outline_color = color_front = color_rear = color

    body, fr, fl, rr, rl = _translate_all(_vehicle_parts(params, yaw, steer_front), x, y)

    plt.plot(body[0, :], body[1, :], color=outline_color, linewidth=2)
    plt.plot(fr[0, :], fr[1, :], color=color_front, linewidth=2)
    plt.plot(fl[0, :], fl[1, :], color=color_front, linewidth=2)
    plt.plot(rr[0, :], rr[1, :], color=color_rear, linewidth=2)
    plt.plot(rl[0, :], rl[1, :], color=color_rear, linewidth=2)

    xb = x + params.wheelbase * math.cos(yaw)
    yb = y + params.wheelbase * math.sin(yaw)
    plt.plot([x, xb], [y, yb], "--", color="0.4", linewidth=1.2)
    plt.plot(x, y, "*k")

    if arrow_color is not None:
        draw_arrow(x, y, yaw, 0.6 * params.wheelbase, arrow_color)


# ----------------------------
# Demo UI (front-steer only)
# ----------------------------

def _fixed_axes(ax, p: VehicleParams) -> None:
    ax.set_aspect("equal", adjustable="box")
    x_min = -p.rear_overhang - 0.6
    x_max = p.wheelbase + p.front_overhang + 0.6
    y_half = 0.5 * p.width
    ax.set_xlim(x_min, x_max)
    ax.set_ylim(-y_half - 1.0, y_half + 1.4)
    ax.grid(True, linestyle="--", alpha=0.4)
    ax.set_title("Planar Vehicle (front-steer only)")


def demo_ui() -> None:
    import time
    from collections import deque
    import matplotlib.pyplot as plt
    from matplotlib.widgets import Slider
    from matplotlib.patches import Polygon

    p = VehicleParams()
    x0 = y0 = yaw0 = 0.0
    front_deg0 = 0.0
    v0 = 0.0

    sample_hz = 10.0
    dt_sec = 1.0 / sample_hz
    window_sec = 30.0
    maxlen = int(window_sec * sample_hz) + 5

    fig = plt.figure(figsize=(12, 8))
    gs = fig.add_gridspec(
        nrows=3, ncols=2,
        width_ratios=[7.0, 1.4],
        height_ratios=[2.6, 1.0, 1.0],
        wspace=0.30, hspace=0.25
    )

    ax_main = fig.add_subplot(gs[0, 0])
    ax_ts_f = fig.add_subplot(gs[1, 0])
    ax_ts_v = fig.add_subplot(gs[2, 0])

    info_ax  = fig.add_subplot(gs[0, 1])
    ax_front = fig.add_subplot(gs[1, 1])
    ax_vel   = fig.add_subplot(gs[2, 1])

    info_ax.axis("off")
    for a in (ax_front, ax_vel):
        a.set_frame_on(True)
        for s in a.spines.values():
            s.set_visible(True)

    _fixed_axes(ax_main, p)

    body, fr, fl, rr, rl = _translate_all(_vehicle_parts(p, yaw0, math.radians(front_deg0)), x0, y0)
    colors = ["k", "tab:blue", "tab:blue", "tab:orange", "tab:orange"]
    polys = [body, fr, fl, rr, rl]
    patches: List[Polygon] = []
    for poly, col in zip(polys, colors):
        patch = Polygon(poly.T, closed=True, fill=False, lw=2, ec=col)
        ax_main.add_patch(patch)
        patches.append(patch)

    arrow_color = "tab:green"
    L = 0.6 * p.wheelbase
    shaft, = ax_main.plot([], [], linewidth=2, solid_capstyle="round", color=arrow_color)
    head_l, = ax_main.plot([], [], linewidth=2, color=arrow_color)
    head_r, = ax_main.plot([], [], linewidth=2, color=arrow_color)

    def _set_arrow(x: float, y: float, theta: float) -> None:
        ang = math.radians(30.0)
        x_end = x + L * math.cos(theta)
        y_end = y + L * math.sin(theta)
        shaft.set_data([x, x_end], [y, y_end])
        hL = 0.4 * L
        lt = theta + math.pi - ang
        rt = theta + math.pi + ang
        head_l.set_data([x_end, x_end + hL * math.cos(lt)],
                        [y_end, y_end + hL * math.sin(lt)])
        head_r.set_data([x_end, x_end + hL * math.cos(rt)],
                        [y_end, y_end + hL * math.sin(rt)])

    _set_arrow(x0, y0, yaw0)

    info_txt = info_ax.text(
        0.5, 0.5, "",
        ha="center", va="center",
        fontfamily="monospace", fontsize=10,
        bbox=dict(boxstyle="round,pad=0.3", facecolor="white", edgecolor="0.6"),
        transform=info_ax.transAxes,
    )

    def _update_info(front_deg: float, vel_mps: float) -> None:
        info_txt.set_text(
            f"front: {front_deg:5.1f} deg\nvelocity: {vel_mps*3.6:6.1f} km/h"
        )

    _update_info(front_deg0, v0)

    s_front = Slider(ax=ax_front, label="Front steer (deg)",
                     valmin=-30.0, valmax=30.0, valinit=front_deg0, valstep=0.1,
                     orientation="vertical")
    s_vel = Slider(ax=ax_vel, label="Velocity (m/s)",
                   valmin=0.0, valmax=30.0, valinit=v0, valstep=0.1,
                   orientation="vertical")

    def on_change(_):
        f_deg = float(s_front.val)
        body, fr, fl, rr, rl = _translate_all(
            _vehicle_parts(p, yaw0, math.radians(f_deg)), x0, y0
        )
        for patch, poly in zip(patches, [body, fr, fl, rr, rl]):
            patch.set_xy(poly.T)
        _set_arrow(x0, y0, yaw0)
        _update_info(f_deg, float(s_vel.val))
        fig.canvas.draw_idle()

    s_front.on_changed(on_change)
    s_vel.on_changed(on_change)

    t_hist, f_hist, v_hist = (deque(maxlen=maxlen) for _ in range(3))
    t0 = time.monotonic()

    def _style_ts(ax, ylabel: str, xlabel: Optional[str] = None):
        ax.grid(True, linestyle="--", alpha=0.4)
        ax.set_ylabel(ylabel)
        if xlabel:
            ax.set_xlabel(xlabel)
        ax.set_xlim(0.0, window_sec)

    _style_ts(ax_ts_f, "Front steer (deg)")
    _style_ts(ax_ts_v, "Velocity (m/s)", "Time (s)")

    ax_ts_v_kmh = ax_ts_v.twinx()
    ax_ts_v_kmh.set_ylabel("Velocity (km/h)")
    ax_ts_v_kmh.set_ylim(ax_ts_v.get_ylim()[0] * 3.6, ax_ts_v.get_ylim()[1] * 3.6)

    ln_f, = ax_ts_f.plot([], [], lw=1.8)
    ln_v, = ax_ts_v.plot([], [], lw=1.8)

    def _auto_ylim(ax, data, pad=0.1, min_span=1.0):
        if not data:
            return
        lo = float(np.min(data)); hi = float(np.max(data))
        if hi - lo < min_span:
            mid = 0.5 * (hi + lo)
            lo = mid - 0.5 * min_span; hi = mid + 0.5 * min_span
        span = hi - lo
        ax.set_ylim(lo - pad * span, hi + pad * span)

    def _sample_and_draw():
        t = time.monotonic() - t0
        f_deg = float(s_front.val)
        v = float(s_vel.val)

        t_hist.append(t); f_hist.append(f_deg); v_hist.append(v)

        t_arr = np.asarray(t_hist, dtype=float)
        if t_arr.size:
            idx = t_arr >= max(0.0, t - window_sec)
            t_win = t_arr[idx]
            x_plot = (t_win - t_win[0]) if t_win.size else np.asarray([])
        else:
            idx = np.array([], dtype=bool)
            x_plot = np.asarray([])

        ln_f.set_data(x_plot, np.asarray(f_hist, dtype=float)[idx])
        ln_v.set_data(x_plot, np.asarray(v_hist, dtype=float)[idx])

        for ax_ts in (ax_ts_f, ax_ts_v):
            ax_ts.set_xlim(0.0, window_sec)
        _auto_ylim(ax_ts_f, list(np.asarray(f_hist)[idx]), pad=0.15, min_span=2.0)
        _auto_ylim(ax_ts_v, list(np.asarray(v_hist)[idx]), pad=0.15, min_span=0.5)

        v_lo, v_hi = ax_ts_v.get_ylim()
        ax_ts_v_kmh.set_ylim(v_lo * 3.6, v_hi * 3.6)

        fig.canvas.draw_idle()

    timer = fig.canvas.new_timer(interval=int(dt_sec * 1000))
    timer.add_callback(_sample_and_draw)
    timer.start()

    def _on_close(_evt):
        try:
            timer.stop()
        except Exception:
            pass

    fig.canvas.mpl_connect("close_event", _on_close)
    plt.show()


__all__: Tuple[str, ...] = ("VehicleParams", "draw_vehicle", "draw_arrow", "demo_ui")

if __name__ == "__main__":
    demo_ui()
