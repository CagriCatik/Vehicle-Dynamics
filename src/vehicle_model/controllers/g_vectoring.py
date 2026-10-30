"""GVC-like load-transfer control (Plan V4.0 M20): an own, simplified reference strategy.

**Not** Mazda's proprietary G-Vectoring Control. The strategy follows the publicly described principle of
coupling a small longitudinal deceleration to the lateral jerk during turn-in (Yamakado et al.'s
``G_x = -sgn(G_y dG_y/dt) C_xy |dG_y/dt| / (1 + T s)``), here in SI units and with explicit limits:

    a_y,est     = v^2 delta / (L (1 + K v^2))          lateral acceleration expected from the driver's steer
    da_y/dt     from the filtered steer rate (no measured lateral jerk needed)
    turn-in     a_y,est * da_y/dt > 0   ->  a_x,cmd = -C_xy |da_y/dt|, first-order filter T, limited to
                                            ``max_decel`` and to ``max_jerk`` (comfort)
    turn-out    -> the request is released smoothly (same jerk limit), never a positive (driving) request
    off         driver brake pedal above ``brake_override``, ABS active, below ``min_speed``, or
                ``enabled: false`` (the request is then exactly 0: the function disappears)

The request is added to the shared chain's longitudinal demand ``F_x`` (it first reduces positive drive
torque; with little or no drive it becomes a small braking request through the allocation). Mechanism in
the plants: the deceleration moves wheel load to the front axle (quasi-static in the 7-DOF model, with
pitch dynamics in the 14-DOF model; both checked in the tests); no lateral force is added by the strategy.
``brake_assist`` (the explicit minimal stabilising-brake sub-variant of the plan) is not implemented.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class GVCConfig:
    enabled: bool = True
    c_xy: float = 0.15                # [s] deceleration per lateral jerk, a_x = -c_xy |da_y/dt|
    tau: float = 0.1                  # [s] first-order filter of the request
    steer_rate_tau: float = 0.03      # [s] filter of the steer rate
    max_decel: float = 0.05 * 9.81    # [m/s^2] largest deceleration request
    max_jerk: float = 2.0             # [m/s^3] rate limit of the request (comfort)
    understeer_gradient: float = 0.0015   # [s^2/m^2] of the a_y estimate
    brake_override: float = 0.05      # [-] driver brake pedal above which GVC yields
    min_speed: float = 5.0            # [m/s]
    jerk_deadband: float = 0.05       # [m/s^3] lateral jerk below which no turn-in is detected (filter tail, noise)

    def __post_init__(self):
        if min(self.c_xy, self.tau, self.steer_rate_tau, self.max_decel, self.max_jerk, self.min_speed) < 0:
            raise ValueError("GVC parameters must be >= 0")


class GVectoringControl:
    def __init__(self, params, config: GVCConfig | None = None):
        self.p = params
        self.config = config or GVCConfig()
        self.reset()

    def reset(self) -> None:
        self._steer_prev = None
        self._rate = 0.0
        self._filt = 0.0
        self.ax_cmd = 0.0
        self.phase = "off"

    def request(self, steer: float, vx: float, brake_pedal: float, abs_active: bool, dt: float) -> float:
        """Longitudinal acceleration request [m/s^2] (<= 0)."""
        c = self.config
        if not c.enabled:
            self.phase, self.ax_cmd = "disabled", 0.0
            return 0.0
        raw = 0.0 if self._steer_prev is None else (steer - self._steer_prev) / max(dt, 1e-9)
        self._steer_prev = steer
        a = dt / (c.steer_rate_tau + dt) if c.steer_rate_tau > 0 else 1.0
        self._rate += a * (raw - self._rate)
        v = abs(vx)
        k = v * v / (self.p.wheelbase * (1.0 + c.understeer_gradient * v * v))
        ay, ay_dot = k * steer, k * self._rate
        if brake_pedal > c.brake_override or abs_active or v < c.min_speed:
            target, self.phase = 0.0, "yield"
        elif ay * ay_dot > 0.0 and abs(ay_dot) > c.jerk_deadband:
            target, self.phase = -min(c.c_xy * abs(ay_dot), c.max_decel), "turn-in"
        else:
            target, self.phase = 0.0, "release"
        b = dt / (c.tau + dt) if c.tau > 0 else 1.0
        self._filt += b * (target - self._filt)
        step = c.max_jerk * dt
        self.ax_cmd = float(np.clip(self._filt, self.ax_cmd - step, self.ax_cmd + step))
        self.ax_cmd = min(self.ax_cmd, 0.0)
        return self.ax_cmd


__all__ = ("GVCConfig", "GVectoringControl")
