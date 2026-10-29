"""Drift supervisor: discrete mode decisions (Drift plan D4, section 5.3). No control law in here.

Modes and transitions (each with a logged reason):

    DISABLED   -> APPROACH   drift requested (at ``request_s``) and speed above ``min_speed``
    APPROACH   -> INITIATE   the vehicle is in steady cornering: |r - r_c| < approach_r_tol and
                             |beta_dot| small for ``approach_hold_s`` (r_c: cornering equilibrium)
    APPROACH   -> RECOVER    no steady cornering within ``approach_timeout_s``
    INITIATE   -> HOLD       inside the capture region of the drift equilibrium for ``capture_hold_s``:
                             |beta - beta*| < capture_beta, |r - r*| < capture_r, |v - v*| < capture_v
    INITIATE   -> RECOVER    divergence (|beta - beta*| > loss_beta) or ``initiate_timeout_s``
    HOLD       -> EXIT       exit requested (at ``exit_s``)
    HOLD       -> RECOVER    |beta - beta*| > loss_beta (the drift is lost)
    EXIT, RECOVER -> DISABLED back in steady cornering near the cornering equilibrium, then the
                             straight-driving ramp (|r - r_c| < approach_r_tol and |beta| < exit_beta)
    any        -> FAILED     |beta| >= spin_beta (spin) or RECOVER longer than ``recover_timeout_s``

    HOLD      <-> TRANSITION the set point moves (:meth:`retarget` with ``moving``); the reference
                             values follow it, so the loss criterion is judged against the moving set point

HOLD is never reported while the state is outside the hold tolerance; FAILED is final.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

DISABLED, APPROACH, INITIATE, HOLD, TRANSITION, EXIT, RECOVER, FAILED = (
    "DISABLED", "APPROACH", "INITIATE", "HOLD", "TRANSITION", "EXIT", "RECOVER", "FAILED")
MODES = (DISABLED, APPROACH, INITIATE, HOLD, TRANSITION, EXIT, RECOVER, FAILED)


@dataclass(frozen=True)
class SupervisorSettings:
    request_s: float = 0.5
    exit_s: float | None = None                 # None: no exit request
    min_speed: float = 5.0                      # [m/s]
    approach_r_tol: float = math.radians(2.0)   # [rad/s]
    approach_beta_rate: float = math.radians(2.0)  # [rad/s]
    approach_hold_s: float = 0.5
    approach_timeout_s: float = 8.0
    capture_beta: float = math.radians(5.0)
    capture_r: float = math.radians(8.0)
    capture_v: float = 2.5
    capture_hold_s: float = 0.5
    initiate_timeout_s: float = 10.0
    loss_beta: float = math.radians(15.0)       # |beta - beta*| beyond: drift lost
    exit_beta: float = math.radians(3.0)
    recover_timeout_s: float = 10.0
    spin_beta: float = math.radians(60.0)


@dataclass
class Transition:
    t: float
    source: str
    target: str
    reason: str


@dataclass
class DriftSupervisor:
    settings: SupervisorSettings
    beta_star: float                 # drift equilibrium
    r_star: float
    v_star: float
    r_corner: float                  # cornering equilibrium (approach / exit target)
    mode: str = DISABLED
    transitions: list = field(default_factory=list)

    def __post_init__(self):
        self.reset()

    def reset(self) -> None:
        self.mode = DISABLED
        self.transitions = []
        self._since = 0.0            # time the current condition has held
        self._entered = 0.0          # time of entering the current mode
        self._prev_beta = None
        self._exit_done_ramp = False

    def _go(self, t: float, target: str, reason: str) -> None:
        if target != self.mode:
            self.transitions.append(Transition(t, self.mode, target, reason))
            self.mode = target
            self._entered = t
            self._since = 0.0

    def retarget(self, t: float, beta_star: float, r_star: float, v_star: float, moving: bool) -> None:
        """New reference values (set-point change, D5); HOLD <-> TRANSITION while the set point moves."""
        self.beta_star, self.r_star, self.v_star = float(beta_star), float(r_star), float(v_star)
        if self.mode == HOLD and moving:
            self._go(t, TRANSITION, "set point moving")
        elif self.mode == TRANSITION and not moving:
            self._go(t, HOLD, "set point reached")

    def update(self, t: float, dt: float, beta: float, r: float, v: float, straight_reached: bool = False) -> str:
        """Advance the mode with the measured ``beta``, ``r``, ``v``; ``straight_reached`` from the exit law."""
        s = self.settings
        beta_rate = 0.0 if self._prev_beta is None else (beta - self._prev_beta) / dt
        self._prev_beta = beta
        if self.mode == FAILED:
            return self.mode
        if abs(beta) >= s.spin_beta:
            self._go(t, FAILED, f"spin: |beta| {math.degrees(abs(beta)):.0f} deg")
            return self.mode
        eb, er, ev = beta - self.beta_star, r - self.r_star, v - self.v_star
        in_corner = abs(r - self.r_corner) < s.approach_r_tol
        if self.mode == DISABLED:
            if t >= s.request_s and not self._exit_done_ramp and v >= s.min_speed:
                self._go(t, APPROACH, "drift requested")
        elif self.mode == APPROACH:
            steady = in_corner and abs(beta_rate) < s.approach_beta_rate
            self._since = self._since + dt if steady else 0.0
            if self._since >= s.approach_hold_s:
                self._go(t, INITIATE, "steady cornering reached")
            elif t - self._entered > s.approach_timeout_s:
                self._go(t, RECOVER, "approach timeout")
        elif self.mode == INITIATE:
            inside = abs(eb) < s.capture_beta and abs(er) < s.capture_r and abs(ev) < s.capture_v
            self._since = self._since + dt if inside else 0.0
            if self._since >= s.capture_hold_s:
                self._go(t, HOLD, "inside capture region")
            elif t - self._entered > s.initiate_timeout_s:
                self._go(t, RECOVER, "initiation timeout")
            elif abs(eb) > max(s.loss_beta, abs(self.beta_star) + s.loss_beta) and t - self._entered > 1.0:
                self._go(t, RECOVER, "initiation diverged")
        elif self.mode in (HOLD, TRANSITION):
            if abs(eb) > s.loss_beta:
                self._go(t, RECOVER, f"drift lost: |beta - beta*| {math.degrees(abs(eb)):.1f} deg")
            elif s.exit_s is not None and t >= s.exit_s:
                self._go(t, EXIT, "exit requested")
        elif self.mode in (EXIT, RECOVER):
            if straight_reached:
                self._exit_done_ramp = True
                self._go(t, DISABLED, "normal driving restored")
            elif self.mode == RECOVER and t - self._entered > s.recover_timeout_s:
                self._go(t, FAILED, "recovery timeout")
        return self.mode


__all__ = ("APPROACH", "DISABLED", "EXIT", "FAILED", "HOLD", "INITIATE", "MODES", "RECOVER", "TRANSITION", "DriftSupervisor",
           "SupervisorSettings", "Transition")
