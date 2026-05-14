"""
Kinematic bicycle model for an Ackermann-steering vehicle with
LUT-based rear-axle steering.

The bicycle model collapses the left/right wheels on each axle into a
single central wheel, giving a compact 4-state description:

    state  = [x, y, yaw, v]
    inputs = [u_a (acceleration), u_steer_f (front steer angle)]

The rear-axle steer angle is NOT a free control input — it is derived
from a velocity-dependent lookup table (LUT) calibrated for the physical
vehicle.  The rear axle is electronically slaved to the front axle: given
a front steer angle and the current speed, the LUT returns the rear steer
angle the hardware will produce.  The MPC optimises only over front steer
and acceleration.

State update equations (continuous time):
    dx/dt   = v * cos(yaw)
    dy/dt   = v * sin(yaw)
    dyaw/dt = (v / L) * (sin(delta_f) - sin(delta_r))
    dv/dt   = u_a

where L is the wheelbase, delta_f the front steer angle (free input), and
delta_r the rear steer angle (from LUT, not a free input).  The sin
formulation of the yaw rate matches the original LUT calibration convention.
"""

import math
from src.config import WB, LUT_PATH
from src.models.lut import load_lookup_table


# ── Load rear-axle steering LUT once at module import ─────────────────
# The LUT maps (velocity, front_steer_deg) → rear_steer_deg.
# If loading fails (e.g., file missing), we fall back to zero rear steer.
try:
    steering_lut = load_lookup_table(LUT_PATH)
except Exception as e:
    print(f"Failed to load LUT: {e}")
    steering_lut = None


def bicycle_kinematic_model(v, yaw, u_a, u_steer_f):
    """
    Evaluate the continuous-time bicycle model derivatives.

    Parameters
    ----------
    v         : current speed [m/s]
    yaw       : current heading angle [rad]
    u_a       : commanded acceleration [m/s²]
    u_steer_f : commanded front steer angle [rad]

    Returns
    -------
    dx, dy, d_yaw, dv : state derivatives (for Euler integration)
    u_steer_r          : rear steer angle actually applied [rad]
    """
    # Clamp front steer to the physical actuator limit (±30° hardstop).
    # This is tighter than the MPC bound (±25°) to guard against
    # numerical edge cases from the optimiser.
    steer_f_clamped = max(-math.radians(30.0), min(math.radians(30.0), u_steer_f))

    # Look up rear steer from the 2-D table: f(velocity, front_deg).
    # Velocity is clamped to the table's valid range [0, 20] m/s.
    if steering_lut:
        steer_f_deg = math.degrees(steer_f_clamped)
        v_clamped   = max(0.0, min(v, 20.0))
        u_steer_r   = steering_lut.desired_rear_rad(v_clamped, steer_f_deg)
    else:
        u_steer_r = 0.0  # degenerate: front-steer only

    # Standard kinematic bicycle equations
    dx    = v * math.cos(yaw)
    dy    = v * math.sin(yaw)
    dv    = u_a
    # sin formula: matches the original LUT calibration and avoids the
    # small-angle approximation error at large steer angles
    d_yaw = (v / WB) * (math.sin(steer_f_clamped) - math.sin(u_steer_r))

    return dx, dy, d_yaw, dv, u_steer_r


class KinematicBicycleSystem:
    """
    Stateful wrapper around the bicycle model that acts as the 'plant'
    (the simulated physical vehicle).

    Stores the current state and maintains full history lists so the
    dashboard can plot time-series data without extra bookkeeping.
    """

    def __init__(self, init_x, init_y, init_yaw, init_v):
        # Current state
        self.x   = init_x
        self.y   = init_y
        self.yaw = init_yaw
        self.v   = init_v

        # Full history — index 0 is the initial condition
        self.history_x       = [init_x]
        self.history_y       = [init_y]
        self.history_yaw     = [init_yaw]
        self.history_v       = [init_v]
        self.history_steer_f = [0.0]
        self.history_steer_r = [0.0]
        self.history_a       = [0.0]

    def update_state(self, u_a, u_steer_f, dt=0.01):
        """
        Advance the vehicle state by one time step using forward Euler
        integration, then append the new state to all history lists.

        Parameters
        ----------
        u_a       : acceleration command applied this step [m/s²]
        u_steer_f : front steer command applied this step [rad]
        dt        : integration time step [s]
        """
        dx, dy, d_yaw, dv, u_steer_r = bicycle_kinematic_model(
            self.v, self.yaw, u_a, u_steer_f)

        # Forward Euler integration
        self.x   += dt * dx
        self.y   += dt * dy
        self.yaw += dt * d_yaw
        self.v   += dt * dv

        # Hard speed clamp — the vehicle cannot exceed 10 m/s or reverse
        self.v = max(0.0, min(self.v, 10.0))

        # Record everything for plotting and diagnostics
        self.history_x.append(self.x)
        self.history_y.append(self.y)
        self.history_yaw.append(self.yaw)
        self.history_v.append(self.v)
        self.history_steer_f.append(u_steer_f)
        self.history_steer_r.append(u_steer_r)
        self.history_a.append(u_a)
