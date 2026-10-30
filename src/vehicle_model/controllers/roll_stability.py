"""Roll stability control, RSC: a supervisor that requests, it does not actuate.

``RollStabilitySupervisor`` judges the roll state of the 14-DOF vehicle and, by danger level, asks for

    drive cut        a factor (0..1) on the driver's drive demand
    braking          a deceleration force ``Fx`` (< 0) and an understeering yaw moment ``Mz`` for the
                     shared allocation, which then brakes the outer wheels (lower lateral acceleration
                     is what actually reduces the lateral load transfer ``m a_y h / s``)
    active roll      the existing :class:`~vehicle_model.controllers.active_roll.ActiveRollDistributionController`
                     (only if the active suspension is available and healthy); it reduces roll angle and
                     roll rate, hardly the load transfer itself

Danger measure (per step, from the 14-DOF telemetry: wheel loads ``Fz``, contact flags, roll rate):

    LTR      = |Fz_left - Fz_right| / (Fz_left + Fz_right)          (whole vehicle; 1 = one side unloaded)
    LTR_pred = LTR + T_pred * dLTR/dt (filtered), clipped to [0, 1]
    modes    inactive -> active      when LTR_pred >= ltr_on
             active   -> emergency   when a wheel has lost contact or LTR >= ltr_emergency
             back to inactive        when LTR_pred < ltr_off for at least t_release (hysteresis), never
                                     before t_min_active after the last activation
LTR = 1 or a lifted wheel is **not** proof of rollover (plan): the report states contact losses and the
largest roll angle separately, and roll angles beyond ``roll_validity`` are flagged as outside the model's
validity (small-angle suspension kinematics), not as a predicted rollover.

Tuning (measured, fishhook 80 km/h, 8 deg, h_cg 0.95 / 1.1 m, mu 1): braking without a yaw moment spins the
vehicle (yaw-rate error 130 deg/s), and so does strong straight braking (0.5-0.8 g emergency at h_cg 1.1 m); the
understeering yaw moment from the outer brakes is what lowers the lateral acceleration. Defaults: 0.2 g active,
0.35 g emergency, k_yaw 1.0 (no wheel lift at all at 0.95 m; no two-wheel lift and half the passive yaw-rate error
at 1.1 m).

Interventions are bounded: braking force ``brake_decel * m g`` (emergency ``emergency_decel``), yaw moment
up to ``max_yaw_moment``; the allocation's actuator and grip bounds still apply.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

MODES = ("inactive", "active", "emergency")


@dataclass(frozen=True)
class RollStabilityConfig:
    ltr_on: float = 0.75              # [-] activate when the predicted LTR reaches this
    ltr_off: float = 0.55             # [-] release below this (hysteresis)
    ltr_emergency: float = 0.95       # [-] emergency level (also: any wheel without contact)
    t_pred: float = 0.15              # [s] LTR prediction horizon
    t_min_active: float = 0.4         # [s] minimum active time after an activation
    t_release: float = 0.3            # [s] LTR_pred must stay below ltr_off this long to release
    rate_filter_tau: float = 0.03     # [s] first-order filter of dLTR/dt
    brake_decel: float = 0.2          # [g] braking request while active
    emergency_decel: float = 0.35     # [g] braking request in emergency
    k_yaw: float = 1.0                # [-] understeering yaw moment = k_yaw * m * a_y * track / 2 (signed against a_y)
    max_yaw_moment: float = 5000.0    # [N m]
    drive_cut: float = 0.0            # [-] drive factor while active/emergency (0: no drive)
    use_active_roll: bool = True      # commission the active roll controller if available and healthy
    roll_validity: float = math.radians(15.0)   # [rad] largest roll angle the 14-DOF kinematics are trusted for

    def __post_init__(self):
        if not 0.0 < self.ltr_off < self.ltr_on < self.ltr_emergency <= 1.0:
            raise ValueError("need 0 < ltr_off < ltr_on < ltr_emergency <= 1")
        if min(self.t_pred, self.t_min_active, self.t_release, self.rate_filter_tau) < 0:
            raise ValueError("times must be >= 0")


@dataclass
class RSCRequest:
    mode: str = "inactive"
    drive_factor: float = 1.0
    Fx: float | None = None           # [N] braking request (< 0) or None
    Mz: float | None = None           # [N m] yaw-moment request or None
    active_roll: bool = False
    ltr: float = 0.0
    ltr_pred: float = 0.0
    lifted: tuple = (False, False, False, False)
    reason: str = ""
    diagnostics: dict = field(default_factory=dict)


class RollStabilitySupervisor:
    def __init__(self, mass: float, track: float, config: RollStabilityConfig | None = None, g: float = 9.81):
        self.m, self.track, self.g = float(mass), float(track), float(g)
        self.config = config or RollStabilityConfig()
        self.reset()

    def reset(self) -> None:
        self.mode = "inactive"
        self._ltr_prev = None
        self._rate = 0.0
        self._t_since_on = 0.0
        self._t_below = 0.0
        self.activations = 0

    @staticmethod
    def ltr(Fz) -> float:
        Fz = np.maximum(np.asarray(Fz, float), 0.0)
        left, right = Fz[0] + Fz[2], Fz[1] + Fz[3]
        return float(abs(left - right) / max(left + right, 1.0))

    def update(self, Fz, in_contact, ay: float, dt: float, active_roll_available: bool = True) -> RSCRequest:
        c = self.config
        L = self.ltr(Fz)
        raw = 0.0 if self._ltr_prev is None else (L - self._ltr_prev) / max(dt, 1e-9)
        a = dt / (c.rate_filter_tau + dt) if c.rate_filter_tau > 0 else 1.0
        self._rate += a * (raw - self._rate)
        self._ltr_prev = L
        Lp = float(np.clip(L + c.t_pred * self._rate, 0.0, 1.0))
        lifted = tuple(bool(not x) for x in np.asarray(in_contact, bool))
        prev = self.mode
        if any(lifted) or L >= c.ltr_emergency:
            self.mode = "emergency"
        elif self.mode == "inactive" and Lp >= c.ltr_on:
            self.mode = "active"
        elif self.mode in ("active", "emergency"):
            self.mode = "active"
            self._t_below = self._t_below + dt if Lp < c.ltr_off else 0.0
            if self._t_below >= c.t_release and self._t_since_on >= c.t_min_active:
                self.mode = "inactive"
        if self.mode != "inactive" and prev == "inactive":
            self.activations += 1
            self._t_since_on = 0.0
            self._t_below = 0.0
        self._t_since_on += dt
        req = RSCRequest(mode=self.mode, ltr=L, ltr_pred=Lp, lifted=lifted)
        if self.mode == "inactive":
            req.reason = "LTR below threshold"
            return req
        decel = c.emergency_decel if self.mode == "emergency" else c.brake_decel
        req.drive_factor = c.drive_cut
        req.Fx = -decel * self.m * self.g
        req.Mz = float(np.clip(-np.sign(ay) * c.k_yaw * self.m * abs(ay) * self.track / 2.0, -c.max_yaw_moment, c.max_yaw_moment))
        req.active_roll = bool(c.use_active_roll and active_roll_available)
        req.reason = ("wheel without contact" if any(lifted) else f"LTR {L:.2f}") if self.mode == "emergency" else f"predicted LTR {Lp:.2f}"
        return req


__all__ = ("MODES", "RSCRequest", "RollStabilityConfig", "RollStabilitySupervisor")
