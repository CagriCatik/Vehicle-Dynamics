"""Hill Descent Control: speed-holding brake request on a downhill grade.

``brake_saturated`` (from the MotionController): ABS active on a wheel, or the previous allocation cut the
braking demand at the grip limit.

Own simplified function on the shared chain: a PI on the speed error produces a **deceleration request only**
(never drive), which the MotionController combines with the driver's pedal demand by taking the stronger
braking (no summation); the allocation and ABS/EBD/IBC remain the last wheel-level instance. No road-grade
information is used: the integrator learns the downhill force (information mode A: vehicle speed and ABS flags).

    status   off          function disabled (request exactly 0)
             standby      enabled, not engaged (speed above the engagement speed, or not yet engaged)
             active       holding the set speed
             override     driver accelerates: request 0, integrator reset
             driver_brake the driver's pedal demand is the stronger braking: HDC yields, integrator held
             infeasible   engaged, speed more than ``infeasible_margin`` above the set speed for
                          ``infeasible_time`` while the request is at its limit or ABS is active: the brakes
                          cannot hold the speed (grip or authority), reported instead of claimed control

Anti-windup: conditional integration (the integrator does not grow further into braking while the request is at
``max_decel`` or ABS is active), integrator clamped to ``[-max_decel, 0]``.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class HDCConfig:
    enabled: bool = True
    v_set: float = 10.0 / 3.6          # [m/s] set speed
    v_engage: float = 35.0 / 3.6       # [m/s] engages when the speed is at or below this
    v_disengage: float = 60.0 / 3.6    # [m/s] an engaged function drops out above this
    kp: float = 1.5                    # [1/s]
    ki: float = 0.8                    # [1/s^2]
    max_decel: float = 0.5 * 9.81      # [m/s^2]
    accel_override: float = 0.05       # driver accelerator pedal above this suspends HDC
    infeasible_margin: float = 1.0     # [m/s]
    infeasible_time: float = 1.0       # [s]


class HillDescentControl:
    def __init__(self, config: HDCConfig | None = None):
        self.config = config or HDCConfig()
        self.reset()

    def reset(self) -> None:
        self.integral = 0.0
        self.engaged = False
        self.status = "off" if not self.config.enabled else "standby"
        self.cause = ""
        self._over = 0.0
        self.ax_cmd = 0.0

    def request(self, vx: float, accel_pedal: float, driver_ax: float, brake_saturated: bool, dt: float) -> float:
        """Deceleration request [m/s^2] (<= 0). ``driver_ax``: the driver's pedal demand as acceleration (<= 0)."""
        c = self.config
        self.ax_cmd = 0.0
        if not c.enabled:
            self.status = "off"
            return 0.0
        if accel_pedal > c.accel_override:
            self.integral, self.engaged, self._over, self.status = 0.0, False, 0.0, "override"
            return 0.0
        if not self.engaged:
            self.engaged = vx <= c.v_engage
        elif vx > c.v_disengage:
            self.engaged = False
        if not self.engaged:
            self.integral, self._over, self.status = 0.0, 0.0, "standby"
            return 0.0
        e = c.v_set - vx                                # < 0: too fast
        a = c.kp * e + self.integral
        at_limit = a <= -c.max_decel or brake_saturated
        if not (e < 0.0 and at_limit):                  # conditional integration (anti-windup)
            self.integral = min(0.0, max(-c.max_decel, self.integral + c.ki * e * dt))
        a = min(0.0, max(-c.max_decel, c.kp * e + self.integral))
        if driver_ax < a:                               # the driver brakes harder: HDC yields
            self.integral = min(self.integral, 0.0)
            self.status, self._over = "driver_brake", 0.0
            return 0.0
        over = -e > c.infeasible_margin and (a <= -c.max_decel + 1e-9 or brake_saturated)
        self._over = self._over + dt if over else (0.0 if -e < 0.5 * c.infeasible_margin else self._over)
        if self._over >= c.infeasible_time:
            self.status = "infeasible"
            self.cause = "grip (ABS or grip-limited allocation)" if brake_saturated else "brake authority (request at max_decel)"
        else:
            self.status, self.cause = "active", ""
        self.ax_cmd = a
        return a


__all__ = ("HDCConfig", "HillDescentControl")
