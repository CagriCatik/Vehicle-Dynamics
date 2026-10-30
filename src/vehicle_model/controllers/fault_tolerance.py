"""Fault-tolerant motion control: health monitor and degraded modes.

**Controller side** of fault tolerance. The monitor only sees what a real controller sees: the (possibly corrupted)
time-stamped input message and the actuator self-reports (see :mod:`vehicle_model.simulation.fault_injection`);
it never reads the injected fault list.

Checks per tick:

    message age      t - t_stamp > stale_limit -> comm limited; > stale_fallback -> comm degraded (yaw law kept:
                     measured, a PI on 150 ms old data still stabilises sine with dwell and braking in a turn,
                     the zero-yaw fallback spins); > stale_unusable -> comm fallback (zero-yaw law)
    yaw rate         not finite -> imu unavailable: yaw law replaced by ``ZeroYawLaw`` (yaw moment 0: the
                     allocation keeps the brake forces left/right balanced, select-low-like; brakes, ABS stay)
    wheel speeds     not finite, or frozen (bit-identical for ``frozen_time`` while the vehicle speed changes)
                     -> wheel_speed limited: speed replaced by the mean of the valid wheels, its slip by the slip
                     of the other wheel on the same axle (substitute for the slip control), that wheel's brake
                     limited to ``wheel_speed_brake_scale``
    wheel loads      a wheel deviating from the quasi-static estimate by more than ``load_tol`` m g / 4
                     -> load_sensor degraded: all loads replaced by the quasi-static estimate
    actuator reports brake/motor failed -> unavailable (health mask: no command), limited -> limited (scaled
                     bound), stuck -> degraded but kept in the allocation (measured: masking it leaves its
                     torque acting unaccounted and the vehicle drifts more, 0.44 m vs 0.009 m on split-mu);
                     steer_rear stuck -> degraded; steer_front stuck -> fallback: controlled stop at
                     ``safe_stop_decel`` (the vehicle cannot follow the driver's steering)

States: ``nominal < limited < degraded < fallback < unavailable`` per channel. The system state is the worst
channel state, capped at ``degraded`` for a lost redundant element (one brake, motor or sensor), ``fallback`` for a
controlled stop and ``unavailable`` only if no brake is left. Each
channel returns to nominal ``recovery_time`` after its cause disappeared (hysteresis). If no brake is left the
system is ``unavailable`` and the output status ``infeasible``; an allocation that cannot meet the demand
while the system is degraded is reported ``infeasible`` instead of claimed stability.

Not detectable by these plausibility checks (reported, not hidden): a friction-estimate drift and small load
biases within the tolerance.
"""

from __future__ import annotations

import copy
import dataclasses
from dataclasses import dataclass

import numpy as np

STATES = ("nominal", "limited", "degraded", "fallback", "unavailable")
WHEELS = ("FL", "FR", "RL", "RR")


@dataclass(frozen=True)
class FaultToleranceConfig:
    stale_limit: float = 0.03            # [s]
    stale_fallback: float = 0.10         # [s]
    stale_unusable: float = 0.25         # [s]
    frozen_time: float = 0.05            # [s]
    load_tol: float = 0.25               # [-] of m g / 4
    wheel_speed_brake_scale: float = 0.5
    safe_stop_decel: float = 3.0         # [m/s^2]
    recovery_time: float = 0.2           # [s]


def _rank(state: str) -> int:
    return STATES.index(state)


class ZeroYawLaw:
    """Fallback law without yaw-rate feedback: requests a zero yaw moment (balanced brake forces left/right)."""

    name: str = "fallback: zero yaw moment"
    information: str = "no yaw-rate feedback"

    def reset(self) -> None:
        pass

    def request(self, state, ctx):
        from vehicle_model.controllers.motion_control import MotionRequest
        return MotionRequest(yaw_moment=0.0)


class FaultTolerantController:
    """Health monitor + reconfiguration around a MotionController (optionally inside an ActuatorChain)."""

    def __init__(self, inner, params, config: FaultToleranceConfig | None = None):
        self.inner, self.params = inner, params
        self.config = config or FaultToleranceConfig()
        self.mc = inner if hasattr(inner, "actuation") else getattr(inner, "inner")
        self.information = getattr(inner, "information", None)
        self._law = self.mc.law
        self.reset()

    @property
    def name(self) -> str:
        return getattr(self.inner, "name", "fault tolerant")

    def reset(self) -> None:
        from vehicle_model.allocation import ActuatorHealth
        self.t, self.t_stamp, self.reports = 0.0, 0.0, []
        self.last_bad = {}                   # channel -> (time, state)
        self._omega_prev, self._omega_same = None, np.zeros(4)
        self._vx_ref = None
        if hasattr(self.inner, "reset"):
            self.inner.reset()
        self.mc.law = self._law
        self.mc.actuation.health = ActuatorHealth()
        self.channels = {}
        self.system_state = "nominal"

    def receive(self, t: float, t_stamp: float, reports) -> None:
        self.t, self.t_stamp, self.reports = float(t), float(t_stamp), list(reports)

    def _flag(self, channel: str, state: str) -> None:
        self.last_bad[channel] = (self.t, state)

    def compute_control(self, state, target, telemetry=None, dt: float = 0.01):
        from vehicle_model.allocation import ActuatorHealth
        from vehicle_model.vehicle_models.normal_loads import quasi_static_normal_loads
        c, p = self.config, self.params
        s, tel = copy.deepcopy(state), telemetry
        # communication: message age
        age = self.t - self.t_stamp
        if age > c.stale_unusable + 1e-12:
            self._flag("comm", "fallback")
        elif age > c.stale_fallback + 1e-12:
            self._flag("comm", "degraded")
        elif age > c.stale_limit + 1e-12:
            self._flag("comm", "limited")
        # yaw-rate sensor (IMU)
        if not np.isfinite(s.r):
            self._flag("imu", "unavailable")
            s.r = 0.0                                         # not used by the fallback law
        # wheel speeds: invalid or frozen
        om = np.array(s.omega, float)
        bad = ~np.isfinite(om)
        if self._omega_prev is not None:              # frozen: identical value while the vehicle speed changed
            same = (om == self._omega_prev) & np.isfinite(om)
            self._omega_same = np.where(same, self._omega_same + dt, 0.0)
            self._vx_ref = np.where(same, self._vx_ref, s.vx)
            bad |= (self._omega_same >= c.frozen_time) & (np.abs(s.vx - self._vx_ref) > 0.02)
        else:
            self._vx_ref = np.full(4, s.vx)
        self._omega_prev = om.copy()
        brake_scale = np.ones(4)
        for k in np.flatnonzero(bad):
            self._flag(f"wheel_speed {WHEELS[k]}", "limited")
        for k in range(4):
            if self._state_of(f"wheel_speed {WHEELS[k]}") != "nominal":
                brake_scale[k] = c.wheel_speed_brake_scale
        if np.any(bad):
            good = om[~bad]
            om[bad] = float(np.mean(good)) if good.size else s.vx / p.Reff
            s.omega = om
        if tel is not None:                           # the slip derived from a bad wheel speed
            kap = np.array(tel.kappa, float)
            kbad = bad | ~np.isfinite(kap) | np.array([self._state_of(f"wheel_speed {w}") != "nominal" for w in WHEELS])
            if np.any(kbad):
                partner = np.array([1, 0, 3, 2])
                kap[kbad] = np.where(np.isfinite(kap[partner[kbad]]) & ~kbad[partner[kbad]], kap[partner[kbad]], 0.0)
                tel = dataclasses.replace(tel, kappa=kap, omega=np.where(kbad, om, np.asarray(tel.omega, float)))
        # wheel-load plausibility against the quasi-static estimate
        if tel is not None:
            ax = float(np.sum(tel.Fx_body)) / p.mass
            ay = float(np.sum(tel.Fy_body)) / p.mass
            Fz_qs = quasi_static_normal_loads(p, ax, ay).Fz
            if np.any(np.abs(np.asarray(tel.Fz, float) - Fz_qs) > c.load_tol * p.mass * p.g / 4.0):
                self._flag("load_sensor", "degraded")
            if self._state_of("load_sensor") != "nominal":
                tel = dataclasses.replace(tel, Fz=Fz_qs)
        # actuator self-reports
        brake_ok, drive_ok, drive_scale = [True] * 4, [True] * 4, np.ones(4)
        for ch, w, mode, sev in self.reports:
            if ch in ("brake", "motor"):
                k = WHEELS.index(w)
                st = {"failed": "unavailable", "limited": "limited", "stuck": "degraded"}[mode]
                self._flag(f"{ch} {w}", st)
            else:
                self._flag(ch, "fallback" if ch == "steer_front" else "degraded")
        for k, w in enumerate(WHEELS):
            for ch, ok, scale in (("brake", brake_ok, brake_scale), ("motor", drive_ok, drive_scale)):
                st = self._state_of(f"{ch} {w}")
                if st == "unavailable":
                    ok[k] = False
                elif st == "limited":
                    sev = [r[3] for r in self.reports if r[0] == ch and r[1] == w]
                    scale[k] = min(scale[k], sev[0] if sev else 1.0)
        self.mc.actuation.health = ActuatorHealth(drive_ok=tuple(drive_ok), brake_ok=tuple(brake_ok),
                                                  drive_scale=tuple(float(x) for x in drive_scale),
                                                  brake_scale=tuple(float(x) for x in brake_scale))
        # reconfiguration: yaw-law fallback, controlled stop
        self.channels = {ch: self._state_of(ch) for ch in list(self.last_bad)}
        self.channels = {k: v for k, v in self.channels.items() if v != "nominal"}
        law_fallback = self.channels.get("imu") == "unavailable" or self.channels.get("comm") == "fallback"
        if law_fallback and not isinstance(self.mc.law, ZeroYawLaw):
            self.mc.law = ZeroYawLaw()
        elif not law_fallback and self.mc.law is not self._law:
            self._law.reset()
            self.mc.law = self._law
        worst = max([min(_rank(v), _rank("fallback" if ch in ("steer_front", "comm") else "degraded"))
                     for ch, v in self.channels.items()] + [0])
        if not any(brake_ok):
            worst = _rank("unavailable")
        self.system_state = STATES[worst]
        if self.channels.get("steer_front") == "fallback":
            pedal = p.mass * c.safe_stop_decel * p.Reff / (4.0 * self.mc.mc_config.actuation.allocator.max_brake_torque)
            target = dataclasses.replace(target, driver_brake_pedal=max(target.driver_brake_pedal, min(1.0, pedal)),
                                         driver_accel_pedal=0.0)
        out = self.inner.compute_control(s, target, tel, dt)
        diag = dict(out.diagnostics or {})
        if self.system_state == "unavailable" or (worst >= _rank("degraded") and diag.get("allocation_status") == "infeasible"):
            out.status = "infeasible"
        diag["health"] = {"system": self.system_state, "channels": dict(self.channels),
                          "law": "fallback (zero yaw moment)" if law_fallback else "nominal", "message_age": age,
                          "mask": {"brake_ok": brake_ok, "drive_ok": drive_ok, "brake_scale": brake_scale.tolist(),
                                   "drive_scale": drive_scale.tolist()}}
        diag["function_status"] = {**(diag.get("function_status") or {}), "health": self.system_state}
        out.diagnostics = diag
        return out

    def _state_of(self, channel: str) -> str:
        v = self.last_bad.get(channel)
        if v is None or self.t - v[0] > self.config.recovery_time + 1e-12:
            return "nominal"
        return v[1]


__all__ = ("FaultToleranceConfig", "FaultTolerantController", "STATES", "ZeroYawLaw")
