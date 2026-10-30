"""Deterministic fault injection between controller and 7-DOF plant.

The **plant/sensor side** of fault tolerance: a list of :class:`FaultEvent` (from YAML) corrupts what the
controller sees (sensor and communication faults) and what the plant receives (actuator faults). The controller
never reads this list; it gets the corrupted signals and the **actuator self-reports** (smart actuators report
their own failure on a diagnostic channel after a declared ``detection_delay``), see
:mod:`vehicle_model.controllers.fault_tolerance`.

Fault catalogue (``channel: modes``; ``wheel`` for per-wheel channels, ``severity`` as noted):

    brake          failed | limited (saturates at severity x the maximum torque) | stuck (holds the torque at onset)
    motor          failed | limited (saturates at severity x the maximum torque)
    steer_front    stuck (holds the angle at onset; steer-by-wire assumption)
    steer_rear     stuck
    imu            lost (yaw rate NaN; the 7-DOF controller input carries no separate accelerations)
    wheel_speed    invalid (NaN) | frozen (value at onset); the wheel speed and the slip derived from it (state and
                   telemetry: the slip control reads the telemetry slip)
    mu_estimate    drift (friction seen by the controller x (1 + severity))
    load_sensor    bias (wheel load seen x (1 + severity))
    delay          ticks (every controller input is ``severity`` control ticks old, time-stamped)
    stale          frozen (the input message stops updating, time stamp included)
    damper, preview  14-DOF channels: rejected here (model boundary)

Plant faults (``brake``, ``motor``, ``steer_*``) act on the plant input; the rest on the controller input.
"""

from __future__ import annotations

import copy
import dataclasses
import math
from collections import deque
from dataclasses import dataclass

import numpy as np

WHEELS = ("FL", "FR", "RL", "RR")
CATALOGUE = {
    "brake": ("failed", "limited", "stuck"), "motor": ("failed", "limited"),
    "steer_front": ("stuck",), "steer_rear": ("stuck",), "imu": ("lost",), "wheel_speed": ("invalid", "frozen"),
    "mu_estimate": ("drift",), "load_sensor": ("bias",), "delay": ("ticks",), "stale": ("frozen",),
}
PER_WHEEL = ("brake", "motor", "wheel_speed", "load_sensor")
ACTUATOR_CHANNELS = ("brake", "motor", "steer_front", "steer_rear")
FOURTEEN_DOF_ONLY = ("damper", "preview")


@dataclass(frozen=True)
class FaultEvent:
    t_start: float
    channel: str
    mode: str
    duration: float | None = None       # None: permanent
    severity: float = 1.0
    wheel: str | None = None

    def __post_init__(self):
        if self.channel in FOURTEEN_DOF_ONLY:
            raise ValueError(f"fault channel {self.channel!r} exists only on the 14-DOF model, not in the 7-DOF loop")
        if self.channel not in CATALOGUE:
            raise ValueError(f"fault channel must be one of {sorted(CATALOGUE)}")
        if self.mode not in CATALOGUE[self.channel]:
            raise ValueError(f"fault mode of {self.channel} must be one of {CATALOGUE[self.channel]}")
        if (self.channel in PER_WHEEL) != (self.wheel is not None):
            raise ValueError(f"fault channel {self.channel}: wheel {'required' if self.channel in PER_WHEEL else 'not allowed'}")
        if self.wheel is not None and self.wheel not in WHEELS:
            raise ValueError(f"fault wheel must be one of {WHEELS}")
        if not (math.isfinite(self.t_start) and self.t_start >= 0.0):
            raise ValueError("fault t_start must be finite and >= 0")
        if self.duration is not None and not (math.isfinite(self.duration) and self.duration > 0.0):
            raise ValueError("fault duration must be > 0 (or omitted: permanent)")
        if not math.isfinite(self.severity):
            raise ValueError("fault severity must be finite")
        if self.mode == "limited" and not 0.0 <= self.severity <= 1.0:
            raise ValueError("limited: severity is the remaining fraction in [0, 1]")
        if self.mode == "ticks" and not (1 <= self.severity <= 200 and float(self.severity).is_integer()):
            raise ValueError("delay: severity is the number of ticks, an integer in [1, 200]")
        if self.mode in ("drift", "bias") and not -0.9 <= self.severity <= 2.0:
            raise ValueError(f"{self.mode}: severity must lie in [-0.9, 2]")

    def active(self, t: float) -> bool:
        return t >= self.t_start - 1e-12 and (self.duration is None or t < self.t_start + self.duration - 1e-12)

    @property
    def label(self) -> str:
        w = f" {self.wheel}" if self.wheel else ""
        return f"{self.channel}{w} {self.mode}"


def parse_faults(items, where: str = "faults") -> tuple:
    """FaultEvents from a YAML list; strict (unknown keys, wrong types and values raise ValueError)."""
    if not isinstance(items, list):
        raise ValueError(f"{where} must be a list")
    out = []
    for i, f in enumerate(items):
        w = f"{where}[{i}]"
        if not isinstance(f, dict):
            raise ValueError(f"{w} must be a mapping")
        unknown = set(f) - {"t_start", "duration", "channel", "mode", "severity", "wheel"}
        if unknown:
            raise ValueError(f"{w}: unknown key(s) {sorted(unknown)}")
        for k in ("t_start", "channel", "mode"):
            if k not in f:
                raise ValueError(f"{w}: missing key {k!r}")
        for k in ("t_start", "duration", "severity"):
            if k in f and f[k] is not None and (isinstance(f[k], bool) or not isinstance(f[k], (int, float))):
                raise ValueError(f"{w}.{k} must be a number")
        for k in ("channel", "mode", "wheel"):
            if k in f and f[k] is not None and not isinstance(f[k], str):
                raise ValueError(f"{w}.{k} must be a string")
        try:
            out.append(FaultEvent(float(f["t_start"]), f["channel"], f["mode"],
                                  None if f.get("duration") is None else float(f["duration"]),
                                  float(f.get("severity", 1.0)), f.get("wheel")))
        except ValueError as exc:
            raise ValueError(f"{w}: {exc}") from None
    return tuple(out)


@dataclass(frozen=True)
class ControllerInput:
    """What the controller receives in one tick: a time-stamped message (state, telemetry)."""
    t_stamp: float
    state: object
    telemetry: object


class FaultInjectedController:
    """Wraps the controller stack: corrupts its inputs, delivers actuator self-reports, corrupts the plant input.

    ``inner.compute_control(state, target, telemetry, dt)`` is called with the corrupted message; if ``inner`` has
    ``receive(message_time, reports)``, it gets the time stamp and the actuator self-reports first.
    """

    def __init__(self, inner, faults, detection_delay: float = 0.05, Reff: float = 0.32,
                 max_brake_torque: float = 2500.0, max_drive_torque: float = 800.0):
        self.inner = inner
        self.max_torque = {"brake": float(max_brake_torque), "motor": float(max_drive_torque)}
        self.faults = tuple(faults)
        self.detection_delay = float(detection_delay)
        self.Reff = Reff
        self.information = getattr(inner, "information", None)
        self.reset()

    @property
    def name(self) -> str:
        return getattr(self.inner, "name", type(self.inner).__name__)

    def reset(self) -> None:
        self.t = 0.0
        self._buf = deque(maxlen=256)
        self._held = {}            # values held by stuck/frozen faults, keyed by fault index
        self.log = []              # per tick: t, active faults, leak flags
        if hasattr(self.inner, "reset"):
            self.inner.reset()

    def _active(self):
        return [(i, f) for i, f in enumerate(self.faults) if f.active(self.t)]

    def reports(self) -> list:
        """Actuator self-reports visible now: (channel, wheel, mode, severity) of actuator faults active for at least
        ``detection_delay``."""
        return [(f.channel, f.wheel, f.mode, f.severity) for _, f in self._active()
                if f.channel in ACTUATOR_CHANNELS and self.t >= f.t_start + self.detection_delay - 1e-12]

    def _corrupt_input(self, state, tel):
        s, tl = copy.deepcopy(state), copy.deepcopy(tel)
        for i, f in self._active():
            if f.channel == "imu":
                s.r = float("nan")
            elif f.channel == "wheel_speed":
                k = WHEELS.index(f.wheel)
                om = np.array(s.omega, float)
                held = self._held.setdefault(i, (float(om[k]), None if tl is None else float(tl.kappa[k])))
                om[k] = float("nan") if f.mode == "invalid" else held[0]
                s.omega = om
                if tl is not None:
                    kap, omt = np.array(tl.kappa, float), np.array(tl.omega, float)
                    kap[k] = float("nan") if f.mode == "invalid" else held[1]
                    omt[k] = om[k]
                    tl = dataclasses.replace(tl, kappa=kap, omega=omt)
            elif f.channel == "mu_estimate" and tl is not None:
                tl = dataclasses.replace(tl, mu=np.asarray(tl.mu, float) * (1.0 + f.severity))
            elif f.channel == "load_sensor" and tl is not None:
                Fz = np.array(tl.Fz, float)
                Fz[WHEELS.index(f.wheel)] *= 1.0 + f.severity
                tl = dataclasses.replace(tl, Fz=Fz)
        return s, tl

    def compute_control(self, state, target, telemetry=None, dt: float = 0.01):
        s, tl = self._corrupt_input(state, telemetry)
        msg = ControllerInput(self.t, s, tl)
        self._buf.append(msg)
        for i, f in self._active():
            if f.channel == "delay":
                n = int(f.severity)
                msg = self._buf[max(0, len(self._buf) - 1 - n)]
            elif f.channel == "stale":
                msg = self._held.setdefault(i, msg)
        if hasattr(self.inner, "receive"):
            self.inner.receive(self.t, msg.t_stamp, self.reports())
        out = self.inner.compute_control(msg.state, target, msg.telemetry, dt)
        commanded = (np.array(out.T_drive, float), np.array(out.T_brake, float))
        leaks = {}
        for i, f in self._active():
            if f.channel == "brake":
                k = WHEELS.index(f.wheel)
                leaks[f.label] = bool(commanded[1][k] > 1.0) and f.mode == "failed"
                T = np.array(out.T_brake, float)
                T[k] = (0.0 if f.mode == "failed" else min(T[k], f.severity * self.max_torque["brake"]) if f.mode == "limited"
                        else self._held.setdefault(i, float(T[k])))
                out.T_brake = T
            elif f.channel == "motor":
                k = WHEELS.index(f.wheel)
                leaks[f.label] = bool(commanded[0][k] > 1.0) and f.mode == "failed"
                T = np.array(out.T_drive, float)
                T[k] = 0.0 if f.mode == "failed" else min(T[k], f.severity * self.max_torque["motor"])
                out.T_drive = T
            elif f.channel == "steer_front":
                out.steer_f = self._held.setdefault(i, float(out.steer_f))
            elif f.channel == "steer_rear":
                out.steer_r = self._held.setdefault(i, float(out.steer_r))
        detected = {f"{c} {w or ''} {m}".replace("  ", " ") for c, w, m, _ in self.reports()}
        self.log.append({"t": self.t, "active": [f.label for _, f in self._active()], "leaks": leaks,
                         "reported": sorted(detected)})
        self.t += dt
        return out


__all__ = ("CATALOGUE", "ControllerInput", "FaultEvent", "FaultInjectedController", "parse_faults")
