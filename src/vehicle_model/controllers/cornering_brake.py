"""Cornering brake control, CBC: a yaw-moment *request* while braking in a turn.

Braking in a turn moves load to the front axle: the rear loses lateral capacity and the car
tends to oversteer. CBC asks for a counteracting yaw moment through the shared allocation (which
then brakes the outer wheels more than the inner ones); it does not command brakes itself:

    feedforward   Mz_ff = c_ff * m * a_x * h_cg * a_y / g           (a_y = r v_x; a_x the *achievable* braking
                  deceleration max(a_x,dem, -mu_hat g): the load transfer follows what the road delivers, not the
                  pedal; measured: with the raw demand -13 m/s^2 CBC asked for 3900 N m and cost braking)
                  (proportional to the longitudinal load transfer m a_x h / L times the lever and the
                  lateral utilisation a_y / g; sign: against the yaw increase)
    feedback      Mz_fb = k_fb * (r_ref - r)   only while the car oversteers, |r| > |r_ref| + deadband

Active only while braking (``a_x,dem <= -min_decel``) and cornering (``|a_y| >= min_lat``); otherwise it
requests nothing (``None``, not 0). The request goes to the yaw arbitration of the motion controller
(:func:`vehicle_model.controllers.motion_control.arbitrate_yaw`): CBC and the yaw law (ESC/TV) are never
added on the same brakes.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class CorneringBrakeConfig:
    c_ff: float = 0.5                 # [-] feedforward share of the load-transfer yaw moment
    k_fb: float = 6000.0              # [N m/(rad/s)] oversteer feedback gain
    deadband: float = 0.02            # [rad/s] oversteer feedback deadband
    min_decel: float = 1.0            # [m/s^2] active above this braking demand
    min_lat: float = 2.0              # [m/s^2] active above this lateral acceleration

    def __post_init__(self):
        if min(self.c_ff, self.k_fb, self.deadband, self.min_decel, self.min_lat) < 0:
            raise ValueError("cornering brake parameters must be >= 0")


class CorneringBrakeControl:
    def __init__(self, params, config: CorneringBrakeConfig | None = None):
        self.p = params
        self.config = config or CorneringBrakeConfig()

    def reset(self) -> None:
        pass

    def request(self, state, r_ref: float, Fx_des: float, mu_hat: float = 1.0):
        """Yaw moment request [N m] or ``None`` (inactive)."""
        c, p = self.config, self.p
        ax = max(Fx_des / p.mass, -max(mu_hat, 0.05) * p.g)
        ay = state.r * state.vx
        if ax > -c.min_decel or abs(ay) < c.min_lat:
            return None
        mz = c.c_ff * p.mass * ax * p.hcog * ay / p.g
        if abs(state.r) > abs(r_ref) + c.deadband and np.sign(state.r) == np.sign(r_ref or state.r):
            mz += c.k_fb * (r_ref - state.r)
        return float(mz)


__all__ = ("CorneringBrakeConfig", "CorneringBrakeControl")
