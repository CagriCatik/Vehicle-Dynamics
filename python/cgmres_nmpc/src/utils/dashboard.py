"""
Real-time matplotlib dashboard for the CGMRES-NMPC simulation.

Layout
------
Left  (full height): Bird's-eye map — shows the reference path, actual
                     vehicle trajectory, vehicle body with 4 wheels, and
                     obstacle cones.  The view auto-scrolls to follow the
                     vehicle.
Right (4 rows):      Strip charts — velocity, front steer, rear steer,
                     and acceleration plotted against simulation time.
Bottom:              Slider to change the target vehicle speed at runtime.

The animation is driven by matplotlib's FuncAnimation; one simulation
step is computed per animation frame so the dashboard is as close to
real time as the solver allows.
"""

import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Polygon, Circle
from matplotlib.widgets import Slider

from src.config import WB, OBSTACLES, VEHICLE_RADIUS, TRAJ_V, TRAJ_F
from src.controllers.cgmres import get_ref_state, set_target_v


# ── Geometry helpers ───────────────────────────────────────────────────

def _rot(pts, a):
    """Apply a 2-D counter-clockwise rotation by angle a [rad] to Nx2 points."""
    c, s = math.cos(a), math.sin(a)
    R = np.array([[c, -s], [s, c]])
    return pts @ R.T


def get_car_polys(x, y, yaw, steer_f, steer_r):
    """
    Build 5 closed polygon arrays that represent the vehicle in world frame:
      [0] Body outline
      [1] Front-right wheel
      [2] Front-left wheel
      [3] Rear-right wheel
      [4] Rear-left wheel

    All shapes are first defined in the vehicle body frame (forward = +x,
    left = +y), then rotated by the wheel steer angles, positioned on the
    axles, rotated by the vehicle yaw, and finally translated to (x, y).

    Parameters
    ----------
    x, y    : vehicle centre position in world frame [m]
    yaw     : vehicle heading [rad]
    steer_f : front axle steer angle [rad]
    steer_r : rear axle steer angle [rad]

    Returns
    -------
    list of Nx2 numpy arrays — one per polygon, ready for Polygon.set_xy()
    """
    BTW   = 0.15          # rear overhang behind the rear axle [m]
    L     = BTW + WB + 0.15  # total body length [m]
    W     = 0.40          # body width [m]
    WHL   = 0.10          # half-length of each wheel rectangle [m]
    WHW   = 0.05          # half-width of each wheel rectangle [m]
    TREAD = 0.18          # half-distance between left and right wheels [m]

    # Body outline — a simple rectangle in body frame
    outline = np.array([[-BTW,  W/2], [L-BTW,  W/2],
                         [L-BTW, -W/2], [-BTW, -W/2], [-BTW,  W/2]])

    # Wheel template (centred at origin in body frame)
    whl = np.array([[-WHL,  WHW], [ WHL,  WHW],
                    [ WHL, -WHW], [-WHL, -WHW], [-WHL,  WHW]])

    # Rotate each wheel by its steer angle, then move to its axle position
    fr = _rot(whl, steer_f) + [WB, -TREAD]   # front-right
    fl = _rot(whl, steer_f) + [WB,  TREAD]   # front-left
    rr = _rot(whl, steer_r) + [0,  -TREAD]   # rear-right
    rl = _rot(whl, steer_r) + [0,   TREAD]   # rear-left

    # Rotate all shapes by vehicle yaw then translate to world position
    result = []
    for pts in [outline, fr, fl, rr, rl]:
        result.append(_rot(pts, yaw) + [x, y])
    return result


# ── Dashboard class ────────────────────────────────────────────────────

class RealTimeDashboard:
    """
    Manages the matplotlib figure, animation loop, and all artist updates.

    The plant and controller are passed in by reference; `update()` calls
    controller.calc_input() and plant.update_state() each frame so the
    dashboard drives the entire closed-loop simulation.
    """

    def __init__(self, plant, controller, dt, max_iter, init_x):
        self.plant        = plant
        self.controller   = controller
        self.dt           = dt
        self.max_iter     = max_iter
        self.current_iter = 1      # starts at 1 so time = iter*dt > 0 on first step
        self.init_x       = init_x

        # ── Figure and subplot layout ──────────────────────────────────
        self.fig      = plt.figure(figsize=(14, 8))
        self.ax_map     = plt.subplot2grid((4, 2), (0, 0), rowspan=4)
        self.ax_v       = plt.subplot2grid((4, 2), (0, 1))
        self.ax_steer_f = plt.subplot2grid((4, 2), (1, 1))
        self.ax_steer_r = plt.subplot2grid((4, 2), (2, 1))
        self.ax_a       = plt.subplot2grid((4, 2), (3, 1))

        # ── Map panel ─────────────────────────────────────────────────
        self.ax_map.set_aspect('equal')
        self.ax_map.set_xlim(-2, 14)
        self.ax_map.set_ylim(-3.5, 3.5)
        self.ax_map.grid(True)
        self.ax_map.set_title('Vehicle Trajectory & Slalom (Ackermann with LUT rear-axle steering)')
        self.ax_map.set_xlabel('X [m]')
        self.ax_map.set_ylabel('Y [m]')

        # Draw the reference sine path across the full simulation distance
        x_hi  = init_x + TRAJ_V * max_iter * dt * 1.05  # 5% margin beyond end
        x_arr = np.linspace(init_x, x_hi, 1000)
        ref_y = [get_ref_state(0, init_x, vehicle_x=xi)[1] for xi in x_arr]
        self.ax_map.plot(x_arr, ref_y, 'g--', lw=1.5, label='Reference path')
        self.ax_map.axhline(0, color='k', lw=0.5, ls=':')

        # Draw obstacle cones with their safety margin circles
        for i, obs in enumerate(OBSTACLES):
            ox, oy, r = obs
            label = 'Cone' if i == 0 else None
            c = Circle((ox, oy), r, color='darkorange', alpha=0.9, label=label)
            self.ax_map.add_patch(c)
            # Dashed circle shows the safety margin (obstacle radius + vehicle radius)
            safe = Circle((ox, oy), r + VEHICLE_RADIUS, color='darkorange',
                          fill=False, ls='--', lw=0.8, alpha=0.5)
            self.ax_map.add_patch(safe)

        # Live trajectory line — data is updated each frame
        self.traj_line, = self.ax_map.plot([], [], 'b-', lw=2, label='Actual path')

        # Vehicle body: 1 outline + 4 wheels, each a Polygon artist
        colours = ['k', 'tab:blue', 'tab:blue', 'tab:orange', 'tab:orange']
        self.car_patches = []
        for col in colours:
            p = Polygon(np.zeros((5, 2)), closed=True, fill=(col != 'k'),
                        facecolor=col if col != 'k' else 'none',
                        edgecolor=col, lw=1.5)
            self.ax_map.add_patch(p)
            self.car_patches.append(p)

        self.ax_map.legend(loc='upper left', fontsize=8)

        # ── Strip charts (right column) ────────────────────────────────
        T = max_iter * dt  # total simulation duration [s]

        def _chart(ax, ylabel, ylim, color='tab:blue'):
            """Helper: configure a strip chart and return its line artist."""
            line, = ax.plot([], [], color=color, lw=1.5)
            ax.set_ylabel(ylabel)
            ax.set_xlim(0, T)
            ax.set_ylim(*ylim)
            ax.grid(True)
            return line

        self.line_v       = _chart(self.ax_v,       'Velocity [m/s]',    (-0.2, 5.5),  'tab:green')
        self.line_sf      = _chart(self.ax_steer_f, 'Front Steer [deg]', (-35,  35),   'tab:purple')
        self.line_sr      = _chart(self.ax_steer_r, 'Rear Steer [deg]',  (-15,  15),   'tab:cyan')
        self.line_a       = _chart(self.ax_a,       'Accel [m/s²]',      (-3.0, 3.0),  'tab:orange')
        self.ax_a.set_xlabel('Time [s]')

        # Horizontal reference line on the velocity chart
        self.line_vref = self.ax_v.axhline(TRAJ_V, color='gray', lw=0.8, ls=':')
        self.ax_v.legend(['v', 'v_ref'], fontsize=7)

        # Shared time axis data — grows one element per frame
        self.time_data = [0.0]
        plt.tight_layout(rect=[0, 0.08, 1, 1])  # leave room for slider at bottom

        # ── Interactive target-speed slider ────────────────────────────
        self.ax_slider = plt.axes([0.15, 0.02, 0.3, 0.03])
        self.v_slider = Slider(
            ax=self.ax_slider,
            label='Target v [m/s]',
            valmin=0.5,
            valmax=5.0,
            valinit=TRAJ_V,
            color='tab:green'
        )

        def update_v(val):
            # Propagate slider value to the controller's reference speed
            # and update the dashed reference line on the velocity chart.
            set_target_v(val)
            self.line_vref.set_ydata([val, val])

        self.v_slider.on_changed(update_v)

    def update(self, frame):
        """
        FuncAnimation callback — runs once per animation frame.

        Advances the simulation by one step (solve → apply → record),
        then refreshes all artist data.  Returns a list of changed artists
        so blit mode could be enabled if needed.
        """
        artists = self.car_patches + [self.traj_line,
                  self.line_v, self.line_sf, self.line_sr, self.line_a]
        if self.current_iter >= self.max_iter:
            return artists  # simulation finished; nothing to update

        time = float(self.current_iter) * self.dt

        # ── Step the closed-loop system ────────────────────────────────
        u_1s, u_2s = self.controller.calc_input(
            self.plant.x, self.plant.y, self.plant.yaw, self.plant.v, time)
        u_a  = float(u_1s[0])  # only the first element of each sequence is applied
        u_sf = float(u_2s[0])
        self.plant.update_state(u_a, u_sf, self.dt)
        self.current_iter += 1
        self.time_data.append(time)

        # ── Update map ─────────────────────────────────────────────────
        self.traj_line.set_data(self.plant.history_x, self.plant.history_y)
        px = self.plant.x
        # Scroll the map view: keep vehicle roughly in the left third
        if math.isfinite(px):
            self.ax_map.set_xlim(px - 3, px + 11)

        # Rebuild vehicle polygons with current pose and steer angles
        u_sr   = self.plant.history_steer_r[-1]
        polys  = get_car_polys(px, self.plant.y, self.plant.yaw, u_sf, u_sr)
        for patch, poly in zip(self.car_patches, polys):
            patch.set_xy(poly)

        # ── Update strip charts ────────────────────────────────────────
        T = self.time_data
        self.line_v.set_data(T,  self.plant.history_v)
        self.line_sf.set_data(T, [math.degrees(s) for s in self.plant.history_steer_f])
        self.line_sr.set_data(T, [math.degrees(s) for s in self.plant.history_steer_r])
        self.line_a.set_data(T,  self.plant.history_a)
        return artists

    def run(self):
        """Start the animation loop. Blocks until the window is closed."""
        # ani must be assigned to prevent garbage collection stopping the animation
        ani = FuncAnimation(  # noqa: F841
            self.fig, self.update, frames=self.max_iter,
            interval=10, blit=False, repeat=False)
        plt.show()
