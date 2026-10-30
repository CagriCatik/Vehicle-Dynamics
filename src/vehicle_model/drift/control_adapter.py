"""``VehicleController`` adapter of the drift LQR: the only link to the controller port.

Each step: ``q = [vx, vy, r, omega x4]`` from the state (information mode ``ideal_full_state``:
the true simulation state). With a steer-rate design (default) the steer is a controller state:
``[delta_f_dot, dT] = -K [q - q*, delta_f - delta_f*]``, the rate is limited to
``max_steer_rate`` and integrated; otherwise ``u = u* - K (q - q*)`` with the rate limit applied
as a clip. Then the actuator limits: front steer within ``+-max_steer_f``, rear torque within
``[0, max_drive_torque]``, equal on both rear wheels; no brakes; rear steer 0. The limited
command is what reaches the plant; saturation is reported.

With an integral design (``lqr.integral``) the controller also carries
``z = int (beta - beta*) dt`` (state ``x = [dq, d delta_f, z]``). Anti-windup by conditional
integration: ``z`` is not advanced in a step whose command is saturated (steer, steer rate or
torque), so a limit that binds does not wind up the integral.
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
        self.K = lqr.K
        self.equilibrium_id = equilibrium_id
        self.initial_steer = initial_steer   # steer state at reset (default: equilibrium steer); bumpless start in drift initiation and recovery
        self.reset()

    @property
    def name(self) -> str:
        return "Drift LQR (front steer + RWD torque)"

    def reset(self) -> None:
        self._last_steer = None
        self._z = 0.0

    def set_point(self, q_star, u_star, K) -> None:
        """Move the operating point (gain scheduling along a branch of equilibria); the steer state is kept."""
        self.q_star, self.u_star, self.K = np.asarray(q_star, float), np.asarray(u_star, float), np.asarray(K, float)

    def compute_control(self, state, target=None, telemetry=None, dt: float = 0.01) -> ControlOutput:
        t0 = time.perf_counter()
        p = self.params
        q = np.r_[state.vx, state.vy, state.r, np.asarray(state.omega, float)]
        if self._last_steer is None:
            self._last_steer = float(self.u_star[0] if self.initial_steer is None else self.initial_steer)
        if self.lqr.steer_rate:
            # steer is a controller state: integrate the commanded rate (limited) from the last command
            x = np.r_[q - self.q_star, self._last_steer - self.u_star[0]]
            if getattr(self.lqr, "integral", False):
                x = np.r_[x, self._z]
            v = -self.K @ x
            rate = float(np.clip(v[0], -p.max_steer_rate, p.max_steer_rate)) if self.rate_limit else float(v[0])
            u = np.array([self._last_steer + v[0] * dt, self.u_star[1] + v[1]])
            steer = float(np.clip(self._last_steer + rate * dt, -p.max_steer_f, p.max_steer_f))
        else:
            u = self.u_star - self.K @ (q - self.q_star)
            steer_lo, steer_hi = -p.max_steer_f, p.max_steer_f
            if self.rate_limit:
                step = p.max_steer_rate * dt
                steer_lo, steer_hi = max(steer_lo, self._last_steer - step), min(steer_hi, self._last_steer + step)
            steer = float(np.clip(u[0], steer_lo, steer_hi))
        torque = float(np.clip(u[1], 0.0, p.max_drive_torque))
        saturated = abs(steer - u[0]) > 1e-12 or torque != u[1]
        self._last_steer = steer
        beta = math.atan2(state.vy, state.vx)
        if getattr(self.lqr, "integral", False) and not saturated:
            self._z += (beta - math.atan2(self.q_star[1], self.q_star[0])) * dt   # follows set_point
        diag = DriftControlDiagnostics("HOLD", self.eq.beta, beta, self.eq.yaw_rate, float(state.r), self.eq.speed,
                                       math.hypot(state.vx, state.vy), self.equilibrium_id, bool(saturated),
                                       "saturated" if saturated else "nominal")
        return ControlOutput(steer_f=steer, steer_r=0.0, T_drive=np.array([0.0, 0.0, torque, torque]),
                             T_brake=np.zeros(4), status=diag.status,
                             computation_time_ms=(time.perf_counter() - t0) * 1e3,
                             diagnostics={"drift": asdict(diag), "unclipped": u.tolist(), "information": self.information})




class DriftSupervisedController:
    """Supervised drift: approach, initiation, hold, exit and recovery as one VehicleController.

    The supervisor (:mod:`.supervisor`) decides the mode; the laws per mode are:

        DISABLED   straight driving: steer ramped to 0 at the rate limit, torque P on the cruise speed
        APPROACH   steer ramped to the cornering steer delta_c, torque = T_c + P on the speed error
        INITIATE,  drift LQR around the drift equilibrium, entered bumplessly (its steer state starts at
        HOLD       the last commanded steer); no torque pulse: the LQR itself breaks the rear loose
        EXIT,      ``release``: torque T_c of the stable cornering equilibrium and the steer target
        RECOVER    ``delta_c + k_beta beta_d`` (rate limited), ``beta_d`` the sideslip while in drift attitude
                   (``beta r < 0``), else 0: counter-steer into the slide, unwound as it decays. With ``k_beta = 1`` the front slip angle
                   ``alpha_f ~ delta - beta - l_f r / v = delta_c - l_f r / v`` no longer depends on beta, so
                   the front axle keeps its force while the rear regains grip;
                   ``straight`` once |beta| < exit_beta and |beta_dot| < approach_beta_rate for 0.5 s:
                   steer ramp to 0 with speed control; ``straight_reached`` at |steer| < 0.5 deg
        FAILED     drive torque 0, steer ramped to 0: a defined, bounded command, not a rescue

    Design findings behind the exit law (24_drift_results.md, drift initiation and recovery): an LQR around the cornering
    equilibrium spins the vehicle from the drift (linear design far from its operating point); a
    gain-scheduled path back along the branch of drift equilibria reaches the branch end at
    <= 1 deg/s but fails at the handover. The cornering equilibrium is open-loop stable, and its
    constant inputs lead back to grip from beta = -15 ... -25 deg, but with a sideslip peak of up
    to 42 deg and a speed loss to 2.7 m/s (k_beta = 0); k_beta = 1 keeps the peak at the drift
    angle and the speed above 12 m/s. k_beta = 0.5 and 0.8 spin: partial counter-steer is worse
    than none.

    Every command passes the vehicle limits (steer, steer rate, torque); rear steer and brakes are 0.
    """

    information = INFORMATION_FULL_STATE

    def __init__(self, params, drift_eq, drift_lqr, corner_eq, supervisor, cruise_speed: float,
                 speed_gain: float = 150.0, max_accel: float = 1.5, exit_countersteer_gain: float = 1.0,
                 scheduled=None):
        self.params = params
        self.drift = DriftLQRController(params, drift_eq, drift_lqr, equilibrium_id="drift")
        self.corner_eq = corner_eq
        self.sup = supervisor
        self.cruise_speed = float(cruise_speed)
        self.kv = float(speed_gain)          # [N m per m/s]
        self.max_accel = float(max_accel)    # [m/s^2] ramp of the speed reference (no wheelspin from a speed step)
        self.k_beta = float(exit_countersteer_gain)
        self.scheduled = scheduled           # ScheduledDriftController for HOLD/TRANSITION, or None
        self.reset()

    @property
    def name(self) -> str:
        return "Supervised drift (LQR hold, feedforward exit)"

    def reset(self) -> None:
        self.sup.reset()
        self.drift.reset()
        if self.scheduled is not None:
            self.scheduled.reset()
        self.t = 0.0
        self.steer = 0.0
        self._law = None
        self._phase = None
        self._calm = 0.0
        self._prev_beta = None
        self._v_ref = None

    def _ramp(self, target: float, dt: float) -> float:
        step = self.params.max_steer_rate * dt
        self.steer = float(np.clip(self.steer + np.clip(target - self.steer, -step, step),
                                   -self.params.max_steer_f, self.params.max_steer_f))
        return self.steer

    def _speed_torque(self, base: float, v_ref: float, v: float, dt: float = 0.0) -> float:
        """Torque P on a speed reference ramped from the current speed at ``max_accel`` towards ``v_ref``.

        A speed step with full torque spins a rear-driven car at low speed (found after an exit at
        3 m/s: 1000 N m -> power oversteer -> spin); the ramp bounds the demanded acceleration.
        """
        if self._v_ref is None:
            self._v_ref = v
        self._v_ref += float(np.clip(v_ref - self._v_ref, -self.max_accel * dt, self.max_accel * dt))
        return float(np.clip(base + self.kv * (self._v_ref - v), 0.0, self.params.max_drive_torque))

    def _exit_law(self, beta: float, r: float, beta_rate: float, v: float, dt: float):
        st, c = self.sup.settings, self.corner_eq
        if self._phase is None:
            self._phase, self._calm = "release", 0.0
        if self._phase == "release":
            # drift attitude: sideslip opposite to the yaw rate. Only that part is counter-steered; ordinary
            # cornering has a sideslip of the same sign as r at low speed (11 m/s: +4 deg), and counter-steering
            # it would steer further into the corner. "Calm": drift attitude below exit_beta, beta settled.
            drift_slip = beta if beta * r < 0.0 else 0.0
            calm = abs(drift_slip) < st.exit_beta and abs(beta_rate) < st.approach_beta_rate
            self._calm = self._calm + dt if calm else 0.0
            if self._calm >= 0.5:
                self._phase = "straight"
            return self._ramp(c.steer_front + self.k_beta * drift_slip, dt), float(c.wheel_drive_torque[2])
        return self._ramp(0.0, dt), self._speed_torque(float(c.wheel_drive_torque[2] * 0.5), self.cruise_speed, v, dt)

    def compute_control(self, state, target=None, telemetry=None, dt: float = 0.01) -> ControlOutput:
        t0 = time.perf_counter()
        self.t += dt
        beta = math.atan2(state.vy, state.vx)
        beta_rate = 0.0 if self._prev_beta is None else (beta - self._prev_beta) / dt
        self._prev_beta = beta
        v = math.hypot(state.vx, state.vy)
        straight = self._phase == "straight" and abs(self.steer) < math.radians(0.5)
        mode = self.sup.update(self.t, dt, beta, float(state.r), v, straight)
        if mode not in ("EXIT", "RECOVER"):
            self._phase = None
        if mode not in ("DISABLED", "APPROACH") and not (mode in ("EXIT", "RECOVER") and self._phase == "straight"):
            self._v_ref = None                          # the speed ramp restarts from the actual speed
        sat = False
        c = self.corner_eq
        if mode == "DISABLED":
            self._law = None
            steer = self._ramp(0.0, dt)
            torque = self._speed_torque(float(c.wheel_drive_torque[2] * 0.5), self.cruise_speed, v, dt)
        elif mode == "APPROACH":
            self._law = None
            steer = self._ramp(c.steer_front, dt)
            torque = self._speed_torque(float(c.wheel_drive_torque[2]), c.speed, v, dt)
        elif mode == "INITIATE" or (mode in ("HOLD", "TRANSITION") and self.scheduled is None):
            if self._law is not self.drift:             # bumpless entry: steer state from the last command
                self.drift.initial_steer = self.steer
                self.drift.reset()
                self._law = self.drift
            out = self.drift.compute_control(state, None, None, dt)
            self.steer = out.steer_f
            steer, torque, sat = out.steer_f, out.T_drive[2], out.status == "saturated"
        elif mode in ("HOLD", "TRANSITION"):
            sc = self.scheduled
            if self._law is not sc:                     # bumpless: steer state and set point from the drift LQR
                sc.lqr.initial_steer = self.steer
                sc.beta_start = self.drift.eq.beta
                sc.reset()
                self._law = sc
            out = sc.compute_control(state, None, None, dt)
            self.steer = out.steer_f
            q, _, _ = sc.schedule.point(sc.beta_ref)
            self.sup.retarget(self.t, sc.beta_ref, q[2], math.hypot(q[0], q[1]), out.diagnostics["mode"] == "TRANSITION")
            steer, torque, sat = out.steer_f, out.T_drive[2], out.status == "saturated"
        elif mode in ("EXIT", "RECOVER"):
            self._law = None
            steer, torque = self._exit_law(beta, float(state.r), beta_rate, v, dt)
        else:  # FAILED
            self._law = None
            steer, torque = self._ramp(0.0, dt), 0.0
        eq = self.drift.eq
        diag = DriftControlDiagnostics(mode, eq.beta, beta, eq.yaw_rate, float(state.r), eq.speed, v,
                                       "drift" if mode in ("INITIATE", "HOLD", "TRANSITION") else (self._phase or "none"), bool(sat),
                                       "saturated" if sat else "nominal")
        return ControlOutput(steer_f=float(steer), steer_r=0.0, T_drive=np.array([0.0, 0.0, torque, torque]),
                             T_brake=np.zeros(4), status=diag.status,
                             computation_time_ms=(time.perf_counter() - t0) * 1e3,
                             diagnostics={"drift": asdict(diag), "mode": mode, "exit_phase": self._phase,
                                          "information": self.information})


class ScheduledDriftController:
    """Drift LQR scheduled along a branch of equilibria, with set-point ramp and optional circle outer loop.

    Inner loop: the steer-rate LQR at the operating point ``schedule.point(beta_ref)`` (gain scheduling,
    :class:`.schedule.BranchSchedule`). Set point: ``beta_ref`` moves towards ``beta_target`` at most
    ``beta_rate`` (TRANSITION when they differ). Outer loop (with a ``circle``): the radial error
    ``e = |p - c| - R`` sets ``beta_target = beta(R_ref) - sign(beta) (k_p e + k_d e_dot)`` within the branch,
    since the drift radius grows with |beta| along the branch (13 m/s, mu 0.9: 20.3 m at 6 deg, 27.5 m at
    26 deg). The outer loop only moves the set point; the LQR operating point follows it slowly.
    """

    information = INFORMATION_FULL_STATE

    def __init__(self, params, schedule, beta_target: float, beta_rate: float = math.radians(2.0), circle=None,
                 k_p: float = math.radians(2.0), k_d: float = math.radians(1.0), initial_steer: float | None = None,
                 beta_start: float | None = None):
        from .schedule import radius_to_beta
        self.params = params
        self.schedule = schedule
        self.beta_rate = float(beta_rate)
        self.circle = circle
        self.k_p, self.k_d = float(k_p), float(k_d)
        self.beta_target = float(beta_target)
        self.beta_nominal = radius_to_beta(schedule, circle.radius) if circle is not None else float(beta_target)
        self.beta_start = beta_start
        lo, hi = schedule.beta_range
        self._lim = (min(lo, hi), max(lo, hi))
        eq0 = schedule.equilibria[-1]
        self.lqr = DriftLQRController(params, eq0, schedule.lqrs[-1], equilibrium_id="scheduled",
                                      initial_steer=initial_steer)
        self.reset()

    @property
    def name(self) -> str:
        return "Scheduled drift LQR" + (" + circle outer loop" if self.circle is not None else "")

    def reset(self) -> None:
        self.lqr.reset()
        self.beta_ref = None
        self._prev_e = None

    def set_target(self, beta_target: float) -> None:
        """New set point (TRANSITION); the reference moves there at ``beta_rate``."""
        self.beta_target = float(beta_target)

    def compute_control(self, state, target=None, telemetry=None, dt: float = 0.01) -> ControlOutput:
        if self.beta_ref is None:
            b0 = math.atan2(state.vy, state.vx) if self.beta_start is None else self.beta_start
            self.beta_ref = float(np.clip(b0, *self._lim))
        goal = self.beta_target
        e = e_dot = 0.0
        if self.circle is not None:
            e = float(self.circle.radial_error(state.X, state.Y))
            e_dot = 0.0 if self._prev_e is None else (e - self._prev_e) / dt
            self._prev_e = e
            goal = self.beta_nominal - math.copysign(1.0, self.beta_nominal) * (self.k_p * e + self.k_d * e_dot)
        goal = float(np.clip(goal, *self._lim))
        step = self.beta_rate * dt
        self.beta_ref += float(np.clip(goal - self.beta_ref, -step, step))
        q, u, K = self.schedule.point(self.beta_ref)
        self.lqr.set_point(q, u, K)
        out = self.lqr.compute_control(state, None, None, dt)
        out.diagnostics.update({"beta_ref": self.beta_ref, "beta_goal": goal, "radial_error": e,
                                "mode": "TRANSITION" if abs(self.beta_ref - goal) > 1e-6 else "HOLD"})
        return out


__all__ = ("DriftLQRController", "DriftSupervisedController", "ScheduledDriftController")
