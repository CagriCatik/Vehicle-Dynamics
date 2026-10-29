"""``VehicleController`` adapter of the drift LQR (Drift plan D3): the only link to the controller port.

Each step: ``q = [vx, vy, r, omega x4]`` from the state (information mode ``ideal_full_state``:
the true simulation state). With a steer-rate design (default) the steer is a controller state:
``[delta_f_dot, dT] = -K [q - q*, delta_f - delta_f*]``, the rate is limited to
``max_steer_rate`` and integrated; otherwise ``u = u* - K (q - q*)`` with the rate limit applied
as a clip. Then the actuator limits: front steer within ``+-max_steer_f``, rear torque within
``[0, max_drive_torque]``, equal on both rear wheels; no brakes; rear steer 0. The limited
command is what reaches the plant; saturation is reported.
"""

from __future__ import annotations

import math
import time
from dataclasses import asdict

import numpy as np

from vehicle_model.controllers.framework import INFORMATION_FULL_STATE, ControlOutput

from .types import DriftControlDiagnostics


class DriftLQRController:
    information = INFORMATION_FULL_STATE

    def __init__(self, params, equilibrium, lqr, rate_limit: bool = True, equilibrium_id: str | None = None,
                 initial_steer: float | None = None):
        if not equilibrium.physical_feasible:
            raise ValueError("the drift controller needs a feasible equilibrium")
        self.params = params
        self.eq = equilibrium
        self.lqr = lqr
        self.rate_limit = bool(rate_limit)
        self.q_star = equilibrium.body_state()
        self.u_star = np.array([equilibrium.steer_front, equilibrium.wheel_drive_torque[2]])
        self.equilibrium_id = equilibrium_id
        self.initial_steer = initial_steer   # steer state at reset (default: equilibrium steer); bumpless start in D4
        self.reset()

    @property
    def name(self) -> str:
        return "Drift LQR (front steer + RWD torque)"

    def reset(self) -> None:
        self._last_steer = None

    def compute_control(self, state, target=None, telemetry=None, dt: float = 0.01) -> ControlOutput:
        t0 = time.perf_counter()
        p = self.params
        q = np.r_[state.vx, state.vy, state.r, np.asarray(state.omega, float)]
        if self._last_steer is None:
            self._last_steer = float(self.u_star[0] if self.initial_steer is None else self.initial_steer)
        if self.lqr.steer_rate:
            # steer is a controller state: integrate the commanded rate (limited) from the last command
            x = np.r_[q - self.q_star, self._last_steer - self.u_star[0]]
            v = -self.lqr.K @ x
            rate = float(np.clip(v[0], -p.max_steer_rate, p.max_steer_rate)) if self.rate_limit else float(v[0])
            u = np.array([self._last_steer + v[0] * dt, self.u_star[1] + v[1]])
            steer = float(np.clip(self._last_steer + rate * dt, -p.max_steer_f, p.max_steer_f))
        else:
            u = self.u_star - self.lqr.K @ (q - self.q_star)
            steer_lo, steer_hi = -p.max_steer_f, p.max_steer_f
            if self.rate_limit:
                step = p.max_steer_rate * dt
                steer_lo, steer_hi = max(steer_lo, self._last_steer - step), min(steer_hi, self._last_steer + step)
            steer = float(np.clip(u[0], steer_lo, steer_hi))
        torque = float(np.clip(u[1], 0.0, p.max_drive_torque))
        saturated = abs(steer - u[0]) > 1e-12 or torque != u[1]
        self._last_steer = steer
        beta = math.atan2(state.vy, state.vx)
        diag = DriftControlDiagnostics("HOLD", self.eq.beta, beta, self.eq.yaw_rate, float(state.r), self.eq.speed,
                                       math.hypot(state.vx, state.vy), self.equilibrium_id, bool(saturated),
                                       "saturated" if saturated else "nominal")
        return ControlOutput(steer_f=steer, steer_r=0.0, T_drive=np.array([0.0, 0.0, torque, torque]),
                             T_brake=np.zeros(4), status=diag.status,
                             computation_time_ms=(time.perf_counter() - t0) * 1e3,
                             diagnostics={"drift": asdict(diag), "unclipped": u.tolist(), "information": self.information})


__all__ = ("DriftLQRController",)
