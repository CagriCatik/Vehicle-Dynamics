"""Supervisory control layer: mode logic around a primary and a fallback controller.

:class:`SupervisoryController` is a ``VehicleController``. Each step it computes the
primary controller (always, also while it is not used, so its health stays observable),
judges its output against documented criteria and decides which output reaches the
plant. It claims no functional safety and does not guarantee that the vehicle is
stabilised; it makes the switching explicit, bounded and reported.

Modes (``diagnostics["supervisor_mode"]``), decided in this order:

    degraded      the fallback output is itself inadmissible or reports "degraded":
                  driver-only output (driver steer, rear steer 0, pedals as an open
                  differential and fixed brake split), held for ``degraded_hold_s``
    fallback      the primary is unhealthy: its status is "degraded", or it was "fallback"
                  / "infeasible" / inadmissible for ``n_unhealthy`` consecutive steps;
                  the fallback controller's output is used
    recovery      after ``recover_s`` of consecutive healthy primary steps in fallback:
                  the output is blended linearly from fallback to primary over ``blend_s``
    suboptimal    primary used; its solver status is "suboptimal"
    limited       primary used; its status is "saturated" (allocation at a limit or slip
                  control active)
    nominal       primary used, status "nominal"

Admissibility of an output (any mode): all values finite, steer angles within the
vehicle limits, drive torques within ``[-max_drive, max_drive]``, brake torques within
``[0, max_brake]``. A finite output outside the limits is clipped and counted as
*suppressed* (``diagnostics["suppressed"]``); a non-finite output is never passed on.

Healthy primary step: admissible and status in nominal / saturated / suboptimal.
"""

from __future__ import annotations

import time
from dataclasses import dataclass

import numpy as np

from vehicle_model.controllers.framework import ControlOutput, PassiveBaselineController

NOMINAL, LIMITED, SUBOPTIMAL, FALLBACK, DEGRADED, RECOVERY = (
    "nominal", "limited", "suboptimal", "fallback", "degraded", "recovery")
MODES = (NOMINAL, LIMITED, SUBOPTIMAL, RECOVERY, FALLBACK, DEGRADED)
_HEALTHY = ("nominal", "saturated", "suboptimal")


@dataclass(frozen=True)
class SupervisorConfig:
    n_unhealthy: int = 3          # consecutive unhealthy primary steps before fallback
    recover_s: float = 0.5        # [s] consecutive healthy primary time before recovery starts
    blend_s: float = 0.3          # [s] blend from fallback to primary
    degraded_hold_s: float = 1.0  # [s] minimum time in degraded mode
    limit_tol: float = 1e-6       # relative tolerance of the admissibility check


class SupervisoryController:
    """Primary controller with a fallback, documented switching criteria and output suppression."""

    def __init__(self, primary, fallback, params, config: SupervisorConfig | None = None):
        self.primary, self.fallback, self.params = primary, fallback, params
        self.config = config or SupervisorConfig()
        self.driver_only = PassiveBaselineController(params)
        self.information = getattr(primary, "information", None)
        self.reset()

    @property
    def name(self) -> str:
        return f"Supervised({self.primary.name} | {self.fallback.name})"

    def reset(self) -> None:
        self.primary.reset()
        self.fallback.reset()
        self.mode = NOMINAL
        self._unhealthy = 0
        self._healthy_t = 0.0
        self._blend_t = 0.0
        self._degraded_t = 0.0
        self.counts = {m: 0 for m in MODES}
        self.suppressed = 0
        self.transitions = []

    # ----------------------------------------------------------------- checks
    def _limits(self):
        p = self.params
        lo = np.r_[-p.max_steer_f, -p.max_steer_r, np.full(4, -p.max_drive_torque), np.zeros(4)]
        hi = np.r_[p.max_steer_f, p.max_steer_r, np.full(4, p.max_drive_torque), np.full(4, p.max_brake_torque)]
        return lo, hi

    @staticmethod
    def _vec(out: ControlOutput) -> np.ndarray:
        return np.r_[out.steer_f, out.steer_r, np.asarray(out.T_drive, float), np.asarray(out.T_brake, float)]

    def admissible(self, out: ControlOutput) -> tuple[bool, bool]:
        """``(finite, within_limits)`` of an output."""
        v = self._vec(out)
        if not np.all(np.isfinite(v)):
            return False, False
        lo, hi = self._limits()
        tol = self.config.limit_tol * np.maximum(np.abs(hi - lo), 1.0)
        return True, bool(np.all(v >= lo - tol) and np.all(v <= hi + tol))

    def _clip(self, out: ControlOutput) -> ControlOutput:
        lo, hi = self._limits()
        v = np.clip(self._vec(out), lo, hi)
        out.steer_f, out.steer_r, out.T_drive, out.T_brake = float(v[0]), float(v[1]), v[2:6].copy(), v[6:10].copy()
        return out

    # ----------------------------------------------------------------- step
    def _set(self, mode: str, t_reason: str) -> None:
        if mode != self.mode:
            self.transitions.append({"from": self.mode, "to": mode, "reason": t_reason})
            self.mode = mode

    def compute_control(self, state, target, telemetry=None, dt: float = 0.01) -> ControlOutput:
        t0 = time.perf_counter()
        c = self.config
        try:
            prim = self.primary.compute_control(state, target, telemetry, dt)
            p_fin, p_lim = self.admissible(prim)
            p_status = prim.status
        except Exception as ex:  # an exception is an unhealthy step, not a crash of the loop
            prim, p_fin, p_lim, p_status = None, False, False, f"exception: {type(ex).__name__}"
        healthy = p_fin and p_status in _HEALTHY
        self._unhealthy = 0 if healthy else self._unhealthy + 1
        self._healthy_t = self._healthy_t + dt if healthy else 0.0
        reason = ""

        # mode decision
        if self.mode == DEGRADED and self._degraded_t < c.degraded_hold_s:
            self._degraded_t += dt
        elif self.mode in (NOMINAL, LIMITED, SUBOPTIMAL, RECOVERY):
            if p_status == "degraded" or self._unhealthy >= c.n_unhealthy:
                reason = f"primary unhealthy ({p_status}, {self._unhealthy} steps)"
                self._set(FALLBACK, reason)
        elif self.mode in (FALLBACK, DEGRADED):
            if self._healthy_t >= c.recover_s:
                self._blend_t = 0.0
                self._set(RECOVERY, f"primary healthy for {self._healthy_t:.2f} s")

        fb = None
        if self.mode in (FALLBACK, RECOVERY, DEGRADED):
            fb = self.fallback.compute_control(state, target, telemetry, dt)
            f_fin, f_lim = self.admissible(fb)
            if self.mode == FALLBACK and (not f_fin or fb.status == "degraded"):
                self._degraded_t = 0.0
                self._set(DEGRADED, f"fallback unusable ({fb.status if f_fin else 'non-finite'})")

        if self.mode == DEGRADED:
            out = self.driver_only.compute_control(state, target, telemetry, dt)
        elif self.mode == FALLBACK:
            out = fb
        elif self.mode == RECOVERY:
            self._blend_t += dt
            w = min(1.0, self._blend_t / max(c.blend_s, dt))
            if not p_fin:                      # primary failed again during the blend
                self._set(FALLBACK, "primary failed during recovery")
                out = fb
            else:
                out = prim
                v = (1 - w) * self._vec(fb) + w * self._vec(prim)
                out.steer_f, out.steer_r, out.T_drive, out.T_brake = float(v[0]), float(v[1]), v[2:6].copy(), v[6:10].copy()
                if w >= 1.0:
                    self._set(NOMINAL, "blend complete")
        else:
            out = prim
            self._set({"suboptimal": SUBOPTIMAL, "saturated": LIMITED}.get(p_status, NOMINAL), "primary status")

        fin, lim = self.admissible(out)
        suppressed = False
        if not fin:                              # never pass a non-finite command: driver-only this step
            out = self.driver_only.compute_control(state, target, telemetry, dt)
            suppressed = True
        elif not lim:
            out = self._clip(out)
            suppressed = True
        self.suppressed += int(suppressed)
        self.counts[self.mode] += 1
        out.diagnostics = {**(out.diagnostics or {}), "supervisor_mode": self.mode, "suppressed": suppressed,
                           "primary_status": p_status, "primary_admissible": bool(p_fin and p_lim),
                           "supervisor_reason": reason}
        out.computation_time_ms = (time.perf_counter() - t0) * 1e3
        return out


__all__ = ("DEGRADED", "FALLBACK", "LIMITED", "MODES", "NOMINAL", "RECOVERY", "SUBOPTIMAL", "SupervisorConfig",
           "SupervisoryController")
