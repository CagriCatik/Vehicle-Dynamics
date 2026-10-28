"""Real-time Matplotlib dashboard for CGMRES-NMPC vehicle simulation.

Displays:
  - Left panel: bird's eye view tracking sinusoidal reference with 4WS vehicle polygons.
  - Right panel: strip charts for velocity, front steer, rear steer, and acceleration.
  - Bottom: interactive target speed slider.
"""

from __future__ import annotations

import math
from typing import Any
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Circle, Polygon
from matplotlib.widgets import Slider
import numpy as np

from vehicle_model.controllers.nmpc import (
    KinematicBicycleSystem,
    NMPCControllerCGMRES,
    get_ref_state,
)
from vehicle_model.visualization.plan_view import vehicle_polygons


def _rot(pts: np.ndarray, a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    R = np.array([[c, -s], [s, c]])
    return pts @ R.T


def get_car_polys(
    x: float,
    y: float,
    yaw: float,
    steer_f: float,
    steer_r: float,
    wheelbase: float = 0.60,
) -> list[np.ndarray]:
    """Build 5 polygon arrays (body outline + 4 wheels) in world frame."""
    btw = 0.15
    L = btw + wheelbase + 0.15
    W = 0.40
    whl = 0.10
    whw = 0.05
    tread = 0.18

    outline = np.array([
        [-btw, W / 2],
        [L - btw, W / 2],
        [L - btw, -W / 2],
        [-btw, -W / 2],
        [-btw, W / 2],
    ])

    whl_box = np.array([
        [-whl, whw],
        [whl, whw],
        [whl, -whw],
        [-whl, -whw],
        [-whl, whw],
    ])

    fr = _rot(whl_box, steer_f) + [wheelbase, -tread]
    fl = _rot(whl_box, steer_f) + [wheelbase, tread]
    rr = _rot(whl_box, steer_r) + [0.0, -tread]
    rl = _rot(whl_box, steer_r) + [0.0, tread]

    result = []
    for pts in [outline, fr, fl, rr, rl]:
        result.append(_rot(pts, yaw) + [x, y])
    return result


class RealTimeDashboard:
    """Manages the matplotlib figure, animation loop, and all artist updates."""

    def __init__(
        self,
        plant: KinematicBicycleSystem,
        controller: NMPCControllerCGMRES,
        dt: float = 0.05,
        max_iter: int = 400,
        init_x: float = 0.0,
    ):
        self.plant = plant
        self.controller = controller
        self.dt = dt
        self.max_iter = max_iter
        self.current_iter = 1
        self.init_x = init_x

        self.fig = plt.figure(figsize=(14, 8))
        self.ax_map = plt.subplot2grid((4, 2), (0, 0), rowspan=4)
        self.ax_v = plt.subplot2grid((4, 2), (0, 1))
        self.ax_steer_f = plt.subplot2grid((4, 2), (1, 1))
        self.ax_steer_r = plt.subplot2grid((4, 2), (2, 1))
        self.ax_a = plt.subplot2grid((4, 2), (3, 1))

        self.ax_map.set_aspect("equal")
        self.ax_map.set_xlim(-2, 14)
        self.ax_map.set_ylim(-3.5, 3.5)
        self.ax_map.grid(True)
        self.ax_map.set_title("Vehicle Trajectory & Slalom (Ackermann with LUT Rear-Axle Steering)")
        self.ax_map.set_xlabel("X [m]")
        self.ax_map.set_ylabel("Y [m]")

        # Reference trajectory
        cfg = controller.config
        x_hi = init_x + cfg.target_v * max_iter * dt * 1.05
        x_arr = np.linspace(init_x, x_hi, 1000)
        ref_y = [
            get_ref_state(
                0.0,
                init_x,
                vehicle_x=xi,
                target_v=cfg.target_v,
                amplitude=cfg.amplitude,
                wavelength=cfg.wavelength,
            )[1]
            for xi in x_arr
        ]
        self.ax_map.plot(x_arr, ref_y, "g--", lw=1.5, label="Reference Path")
        self.ax_map.axhline(0, color="k", lw=0.5, ls=":")

        # Obstacles
        for i, obs in enumerate(cfg.obstacles):
            ox, oy, r = obs
            label = "Obstacle" if i == 0 else None
            c = Circle((ox, oy), r, color="darkorange", alpha=0.9, label=label)
            self.ax_map.add_patch(c)
            safe = Circle(
                (ox, oy),
                r + cfg.vehicle_radius,
                color="darkorange",
                fill=False,
                ls="--",
                lw=0.8,
                alpha=0.5,
            )
            self.ax_map.add_patch(safe)

        self.traj_line, = self.ax_map.plot([], [], "b-", lw=2, label="Actual Path")

        colours = ["k", "tab:blue", "tab:blue", "tab:orange", "tab:orange"]
        self.car_patches = []
        for col in colours:
            p = Polygon(
                np.zeros((5, 2)),
                closed=True,
                fill=(col != "k"),
                facecolor=col if col != "k" else "none",
                edgecolor=col,
                lw=1.5,
            )
            self.ax_map.add_patch(p)
            self.car_patches.append(p)

        self.ax_map.legend(loc="upper left", fontsize=8)

        # Strip charts
        T = max_iter * dt

        def _chart(ax, ylabel, ylim, color="tab:blue"):
            line, = ax.plot([], [], color=color, lw=1.5)
            ax.set_ylabel(ylabel)
            ax.set_xlim(0, T)
            ax.set_ylim(*ylim)
            ax.grid(True)
            return line

        self.line_v = _chart(self.ax_v, "Velocity [m/s]", (-0.2, 5.5), "tab:green")
        self.line_sf = _chart(self.ax_steer_f, "Front Steer [deg]", (-35, 35), "tab:purple")
        self.line_sr = _chart(self.ax_steer_r, "Rear Steer [deg]", (-15, 15), "tab:cyan")
        self.line_a = _chart(self.ax_a, "Accel [m/s²]", (-3.0, 3.0), "tab:orange")
        self.ax_a.set_xlabel("Time [s]")

        self.line_vref = self.ax_v.axhline(cfg.target_v, color="gray", lw=0.8, ls=":")
        self.ax_v.legend(["v", "v_ref"], fontsize=7)

        self.time_data = [0.0]
        plt.tight_layout(rect=[0, 0.08, 1, 1])

        # Interactive slider
        self.ax_slider = plt.axes([0.15, 0.02, 0.3, 0.03])
        self.v_slider = Slider(
            ax=self.ax_slider,
            label="Target v [m/s]",
            valmin=0.5,
            valmax=5.0,
            valinit=cfg.target_v,
            color="tab:green",
        )

        def update_v(val):
            self.controller.set_target_v(val)
            self.line_vref.set_ydata([val, val])

        self.v_slider.on_changed(update_v)

    def update(self, frame: int):
        artists = self.car_patches + [
            self.traj_line,
            self.line_v,
            self.line_sf,
            self.line_sr,
            self.line_a,
        ]
        if self.current_iter >= self.max_iter:
            return artists

        time_s = float(self.current_iter) * self.dt
        u_1s, u_2s = self.controller.calc_input(
            self.plant.x, self.plant.y, self.plant.yaw, self.plant.v, time_s
        )
        u_a = float(u_1s[0])
        u_sf = float(u_2s[0])
        self.plant.update_state(u_a, u_sf, self.dt)
        self.current_iter += 1
        self.time_data.append(time_s)

        self.traj_line.set_data(self.plant.history_x, self.plant.history_y)
        px = self.plant.x
        if math.isfinite(px):
            self.ax_map.set_xlim(px - 3, px + 11)

        u_sr = self.plant.history_steer_r[-1]
        polys = get_car_polys(
            px,
            self.plant.y,
            self.plant.yaw,
            u_sf,
            u_sr,
            wheelbase=self.plant.wheelbase,
        )
        for patch, poly in zip(self.car_patches, polys):
            patch.set_xy(poly)

        T = self.time_data
        self.line_v.set_data(T, self.plant.history_v)
        self.line_sf.set_data(T, [math.degrees(s) for s in self.plant.history_steer_f])
        self.line_sr.set_data(T, [math.degrees(s) for s in self.plant.history_steer_r])
        self.line_a.set_data(T, self.plant.history_a)
        return artists

    def run(self):
        """Start animation loop."""
        ani = FuncAnimation(
            self.fig,
            self.update,
            frames=self.max_iter,
            interval=10,
            blit=False,
            repeat=False,
        )
        plt.show()


__all__ = (
    "RealTimeDashboard",
    "get_car_polys",
)
