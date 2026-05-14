"""
NMPC steering controller for the 4WS kinematic bicycle.

Algorithm overview
------------------
At each control step the controller solves a finite-horizon optimal
control problem (OCP) of the form:

    min   sum_{i=0}^{N-1} [ Q_Y*(y_i - y_ref)^2
          over                + Q_YAW*(yaw_i - yaw_ref)^2
       u = [u_a, u_sf]       + Q_V*(v_i - v_ref)^2
                              + obstacle_barrier_i ]
                            + R_steer * ||u_sf||^2
                            + R_accel * ||u_a||^2

subject to:  -U_A_MAX   <= u_a_i   <= U_A_MAX
             -U_STEER_MAX <= u_sf_i <= U_STEER_MAX

The OCP is transcribed into a 2N-dimensional nonlinear program and
solved with scipy's SLSQP (Sequential Least Squares Programming).  This
is sometimes called the direct multiple shooting approach.

The class is named NMPCControllerCGMRES because it was designed to be a
drop-in replacement for a CGMRES-based controller (see docs/cgmres.md);
the external interface (calc_input) is identical.

Reference path
--------------
The reference is spatial, not temporal:  y_ref is expressed as a
function of the vehicle's current x-position, NOT of time.  This means
the reference path looks the same regardless of vehicle speed — the
slalom shape is fixed in space and the vehicle tracks through it.

    y_ref(x)   = A * sin(2*pi*(x - x0) / wavelength)
    yaw_ref(x) = atan2(dy/dx, 1)

Warm starting
-------------
The previous optimal solution is used as the initial guess for the next
solve ('warm start').  This dramatically reduces solver iterations because
the optimal solution changes smoothly between time steps.

Variable horizon dt
-------------------
The per-step prediction time dt grows from near-zero at startup
(exponential ramp: 1 - exp(-alpha*t)) to tf/N at steady state.  This
prevents large initial transients when the controller has not yet built
up a meaningful warm-start.
"""

import math
import numpy as np
from scipy.optimize import minimize

from src.config import (U_A_MAX, U_STEER_MAX,
                        WB, Q_Y, Q_YAW, Q_V,
                        global_config, TRAJ_V as _CONFIG_TRAJ_V, TRAJ_A, TRAJ_F,
                        OBSTACLES, VEHICLE_RADIUS)
from src.models.vehicle import bicycle_kinematic_model


# ── Reference path ─────────────────────────────────────────────────────

# TRAJ_V can be changed at runtime via set_target_v() (used by the slider)
TRAJ_V = _CONFIG_TRAJ_V

# Wavelength is kept constant so the physical slalom shape (metres) stays
# the same even when the user changes vehicle speed with the slider.
# Derived from the original config: 2.5 m/s / 0.15 Hz ≈ 16.67 m
_WAVELENGTH = 16.6666


def set_target_v(v):
    """Update the desired speed at runtime (called by the dashboard slider)."""
    global TRAJ_V
    TRAJ_V = v


def get_ref_state(t, init_x, vehicle_x=None, vehicle_v=None):
    """
    Compute the reference state at the vehicle's current x-position.

    The reference is evaluated at the vehicle's x-coordinate rather than
    at a time-derived position so the controller always tracks the correct
    spatial point, even when the vehicle is accelerating or decelerating.

    Parameters
    ----------
    t         : simulation time [s]  (only used when vehicle_x is None)
    init_x    : starting x-coordinate of the reference path [m]
    vehicle_x : current vehicle x-position [m] (preferred over t)
    vehicle_v : (unused) kept for interface compatibility

    Returns
    -------
    x_pos   : x-coordinate where reference is evaluated [m]
    y_ref   : reference lateral position [m]
    yaw_ref : reference heading angle [rad]
    v_ref   : reference speed [m/s]
    """
    x_pos = vehicle_x if vehicle_x is not None else (init_x + TRAJ_V * t)
    x_rel = x_pos - init_x  # distance travelled along the path

    y_ref   = TRAJ_A * math.sin(2.0 * math.pi * x_rel / _WAVELENGTH)
    # Derivative dy/dx gives the slope of the reference curve at this point
    dy_dx   = TRAJ_A * (2.0 * math.pi / _WAVELENGTH) * \
              math.cos(2.0 * math.pi * x_rel / _WAVELENGTH)
    yaw_ref = math.atan2(dy_dx, 1.0)  # heading tangent to the reference curve
    v_ref   = TRAJ_V
    return x_pos, y_ref, yaw_ref, v_ref


# ── MPC cost function ──────────────────────────────────────────────────

# Input regularisation weights — small values allow necessary steering
# while penalising chattering and aggressive acceleration.
_R_STEER = 0.05   # steering effort penalty
_R_ACCEL = 0.01   # acceleration effort penalty


def _mpc_cost(u_seq, x0, y0, yaw0, v0, dt, init_x, N):
    """
    Simulate the vehicle N steps into the future and accumulate cost.

    This function is the objective passed directly to scipy.minimize.
    It must be fast because it is called many times per control step.

    Parameters
    ----------
    u_seq  : flat array of length 2N — first N entries are acceleration
             commands, last N entries are front-steer commands
    x0, y0, yaw0, v0 : current vehicle state
    dt     : prediction step size [s]
    init_x : reference path origin [m]
    N      : prediction horizon length (number of steps)

    Returns
    -------
    cost : scalar total cost (lower = better)
    """
    xi, yi, yawi, vi = x0, y0, yaw0, v0
    cost = 0.0

    for i in range(N):
        # Unpack the i-th step commands from the flat optimisation vector
        u1_cmd = float(u_seq[i])       # acceleration
        usf    = float(u_seq[N + i])   # front steer

        # Evaluate reference at the predicted x-position
        _, yr, yawr, _ = get_ref_state(0, init_x, vehicle_x=xi)

        # ── Tracking cost ──────────────────────────────────────────────
        # Lateral error: penalise deviation from the sine reference
        cost += Q_Y   * (yi - yr)**2
        # Heading error: use shortest signed angle to avoid wrap-around jumps
        cost += Q_YAW * _angle_diff(yawi, yawr)**2
        # Speed error: the controller also regulates longitudinal speed
        cost += Q_V   * (vi - TRAJ_V)**2

        # ── Obstacle avoidance (log-barrier) ───────────────────────────
        # Only evaluated close to obstacles (within 4× safety radius) to
        # keep the cost smooth far away and avoid distorting normal tracking.
        for ox, oy, or_ in OBSTACLES:
            r_safe = or_ + VEHICLE_RADIUS  # combined radius of obstacle + vehicle
            d2 = (xi - ox)**2 + (yi - oy)**2 + 1e-6  # +eps avoids div-by-zero
            if d2 < (4.0 * r_safe)**2:
                cost += 80.0 * r_safe**2 / d2  # barrier grows steeply as d → 0

        # ── Propagate state one step forward ───────────────────────────
        dx, dy, dyaw, dv, _ = bicycle_kinematic_model(vi, yawi, u1_cmd, usf)
        xi   += dt * dx
        yi   += dt * dy
        yawi += dt * dyaw
        vi    = max(0.0, min(vi + dt * dv, 10.0))

    # ── Input regularisation (over full horizon) ───────────────────────
    # Penalise the Euclidean norm of each input sequence.
    # This is applied once after the loop (not per step) for efficiency.
    cost += _R_STEER * float(np.dot(u_seq[N:], u_seq[N:]))
    cost += _R_ACCEL * float(np.dot(u_seq[:N], u_seq[:N]))
    return cost


def _angle_diff(a, b):
    """
    Shortest signed angle from b to a, wrapped to (-pi, pi].

    Needed because a naive (a - b) can jump by 2*pi at the ±180° boundary,
    which creates a false large cost term and confuses the optimiser.
    """
    d = a - b
    while d >  math.pi: d -= 2 * math.pi
    while d < -math.pi: d += 2 * math.pi
    return d


# ── Controller class ───────────────────────────────────────────────────

class NMPCControllerCGMRES:
    """
    Receding-horizon NMPC controller solved with scipy SLSQP.

    The external interface (calc_input) returns the full optimal input
    sequences; the caller applies only the first element of each sequence
    and then calls calc_input again at the next time step (receding horizon).
    """

    def __init__(self, init_x):
        cfg         = global_config["controller"]
        self.N      = cfg["N"]      # prediction horizon (number of steps)
        self.tf     = cfg["tf"]     # final prediction time at steady state [s]
        self.alpha  = cfg["alpha"]  # time constant for the dt ramp-up
        self.init_x = init_x       # x-coordinate where the reference path starts

        n = self.N
        # Warm-start vectors: initialised to zero (stand still / straight)
        self.u1 = np.zeros(n)  # acceleration sequence [m/s²]
        self.u2 = np.zeros(n)  # front steer sequence [rad]

        # Cost history — used for solution acceptance check
        self.history_f = []

    def calc_input(self, x, y, yaw, v, time):
        """
        Run one NMPC solve and return the optimal input sequences.

        Only the first element of each returned array (u1[0], u2[0]) is
        applied to the plant; the rest form the warm-start for the next call.

        Parameters
        ----------
        x, y, yaw, v : current vehicle state
        time          : current simulation time [s]

        Returns
        -------
        u1 : optimal acceleration sequence, length N [m/s²]
        u2 : optimal front-steer sequence, length N [rad]
        """
        # ── Variable prediction step size ──────────────────────────────
        # dt ramps from ≈0 at t=0 to tf/N at steady state.
        # The floor at 0.05 s prevents degenerate (zero-length) predictions
        # before the exponential ramp has grown.
        dt = self.tf * (1.0 - math.exp(-self.alpha * time)) / self.N
        dt = max(dt, 0.05)

        # ── Build and solve the nonlinear program ──────────────────────
        bounds  = ([(-U_A_MAX, U_A_MAX)] * self.N +
                   [(-U_STEER_MAX, U_STEER_MAX)] * self.N)
        u_guess = np.concatenate([self.u1, self.u2])  # warm start

        res = minimize(
            _mpc_cost,
            u_guess,
            args=(x, y, yaw, v, dt, self.init_x, self.N),
            method='SLSQP',
            bounds=bounds,
            options={'maxiter': 100, 'ftol': 1e-5},
        )

        # ── Accept the new solution if it improved (or first step) ─────
        # Accepting slightly worse solutions (≤20% degradation) avoids
        # getting stuck if the solver stalls on a single difficult step.
        if res.success or (self.history_f and res.fun < self.history_f[-1] * 1.2) or not self.history_f:
            u_opt   = res.x
            self.u1 = np.clip(u_opt[:self.N],  -U_A_MAX,    U_A_MAX)
            self.u2 = np.clip(u_opt[self.N:], -U_STEER_MAX, U_STEER_MAX)

        self.history_f.append(float(res.fun))
        return self.u1, self.u2
