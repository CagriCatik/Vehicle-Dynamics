"""Common uncertainty model: the simulated vehicle deviates from the nominal design (Plan V3.0 M12 7.1).

Controllers are designed once from the nominal vehicle (``nominal`` parameters and tyres)
and are not re-tuned. An :class:`UncertaintyCase` declares how the *simulated* plant and
its periphery differ from that nominal design:

    plant                mass, yaw inertia, centre-of-gravity position, axle cornering
                         stiffness (SlipStiffnessScaledTyre, friction peak kept), road friction
    actuators            transport delay and first-order lag of every actuator command
                         (front and rear steer, drive and brake torques)
    control              latency between measurement and command (an additional output delay)
    sensors              noise scale, gyro and accelerometer bias, sensor latency
                         (only with estimated or sensor-constrained feedback: with ideal
                         feedback the controller reads the true state, declared as such)

Every case is plain data and is written into the experiment results, so each deviation is
part of the documented configuration. :func:`plant_factory` builds the deviating plant,
:func:`wrap_controller` adds actuator delay/lag and control latency around any
``VehicleController``, :func:`sensor_config` the corrupted sensor suite.
"""

from __future__ import annotations

import dataclasses
from collections import deque
from dataclasses import dataclass

import numpy as np

from vehicle_model.estimation.sensors import SensorConfig

FIELDS = ("mass_factor", "Iz_factor", "cog_shift_m", "cornering_scale_front", "cornering_scale_rear", "mu",
          "actuator_delay_s", "actuator_tau_s", "control_latency_s", "sensor_noise_scale", "gyro_bias_radps",
          "accel_bias_mps2", "sensor_latency_s")


@dataclass(frozen=True)
class UncertaintyCase:
    name: str = "nominal"
    mass_factor: float = 1.0              # plant mass / nominal mass
    Iz_factor: float = 1.0                # plant yaw inertia / nominal
    cog_shift_m: float = 0.0              # [m] CoG moved forward (l_f - d, l_r + d)
    cornering_scale_front: float = 1.0    # plant front cornering stiffness / nominal
    cornering_scale_rear: float = 1.0
    mu: float | None = None               # road friction of the plant; None: the scenario's road
    actuator_delay_s: float = 0.0         # [s] transport delay of every actuator command
    actuator_tau_s: float = 0.0           # [s] first-order lag of every actuator command
    control_latency_s: float = 0.0        # [s] measurement-to-command latency
    sensor_noise_scale: float = 1.0       # factor on every sensor noise standard deviation
    gyro_bias_radps: float | None = None  # [rad/s] initial gyro bias; None: sensor default
    accel_bias_mps2: float | None = None  # [m/s^2] initial accelerometer bias; None: sensor default
    sensor_latency_s: float = 0.0         # [s] sensor transport delay

    def __post_init__(self):
        for k in ("mass_factor", "Iz_factor", "cornering_scale_front", "cornering_scale_rear"):
            if not getattr(self, k) > 0.0:
                raise ValueError(f"{k} must be positive")
        if self.mu is not None and not 0.05 <= self.mu <= 2.0:
            raise ValueError("mu must lie in [0.05, 2]")
        for k in ("actuator_delay_s", "actuator_tau_s", "control_latency_s", "sensor_latency_s", "sensor_noise_scale"):
            if getattr(self, k) < 0.0:
                raise ValueError(f"{k} must be >= 0")
        if abs(self.cog_shift_m) > 0.5:
            raise ValueError("cog_shift_m must lie in [-0.5, 0.5]")

    @property
    def plant_deviates(self) -> bool:
        return (self.mass_factor, self.Iz_factor, self.cog_shift_m, self.cornering_scale_front,
                self.cornering_scale_rear) != (1.0, 1.0, 0.0, 1.0, 1.0) or self.mu is not None

    def to_dict(self) -> dict:
        return dataclasses.asdict(self)

    @classmethod
    def from_dict(cls, d: dict) -> "UncertaintyCase":
        unknown = set(d) - set(FIELDS) - {"name"}
        if unknown:
            raise ValueError(f"unknown uncertainty keys {sorted(unknown)}; allowed: {FIELDS}")
        return cls(**d)


def plant_params(nominal, case: UncertaintyCase):
    """Vehicle parameters of the deviating plant."""
    return dataclasses.replace(nominal, mass=nominal.mass * case.mass_factor, Iz=nominal.Iz * case.Iz_factor,
                               lf=nominal.lf - case.cog_shift_m, lr=nominal.lr + case.cog_shift_m)


def plant_factory(nominal_model, case: UncertaintyCase):
    """``road -> plant`` for the deviating vehicle; the road friction is overridden if ``case.mu`` is set."""
    from vehicle_model.roads.surface import UniformRoad
    from vehicle_model.tire_model import SlipStiffnessScaledTyre
    from vehicle_model.vehicle_models import SevenDOFModel

    if nominal_model.chassis is not None and case.cog_shift_m:
        raise ValueError("cog_shift_m cannot be combined with a K&C chassis")

    def build(road):
        p = plant_params(nominal_model.params, case)
        tf, tr = nominal_model.tyre_front, nominal_model.tyre_rear
        if case.cornering_scale_front != 1.0:
            tf = SlipStiffnessScaledTyre(tf, case.cornering_scale_front)
        if case.cornering_scale_rear != 1.0:
            tr = SlipStiffnessScaledTyre(tr, case.cornering_scale_rear)
        road = UniformRoad(mu=case.mu) if case.mu is not None else road
        chassis = nominal_model.chassis
        return SevenDOFModel(params=p, tyre=tf, tyre_rear=tr, road=road, chassis=chassis)
    return build


def sensor_config(case: UncertaintyCase, base: SensorConfig | None = None) -> SensorConfig:
    b = base or SensorConfig()
    k = case.sensor_noise_scale
    return dataclasses.replace(
        b, accel_noise_std=b.accel_noise_std * k, gyro_noise_std=b.gyro_noise_std * k,
        wheel_noise_std=b.wheel_noise_std * k, steer_noise_std=b.steer_noise_std * k,
        gyro_bias_init=b.gyro_bias_init if case.gyro_bias_radps is None else case.gyro_bias_radps,
        accel_bias_init=b.accel_bias_init if case.accel_bias_mps2 is None else case.accel_bias_mps2,
        latency=b.latency + case.sensor_latency_s)


class DelayedActuation:
    """``VehicleController`` wrapper: commands reach the plant ``delay`` later and through a first-order lag.

    The delay is the sum of control latency and actuator transport delay, rounded to whole
    control periods (``n = round(delay / dt)``); before ``n`` commands exist the first command
    is held. Lag: ``y += dt/(tau + dt) (u - y)`` per channel (implicit Euler, stable for any
    ``dt``). The controller is not told about either; its own diagnostics are passed through.
    """

    def __init__(self, inner, delay_s: float = 0.0, tau_s: float = 0.0):
        self.inner = inner
        self.delay_s, self.tau_s = float(delay_s), float(tau_s)
        self.reset()

    def __getattr__(self, item):            # name, params, information, ... of the wrapped controller
        return getattr(self.inner, item)

    def reset(self) -> None:
        self.inner.reset()
        self._fifo = None
        self._y = None

    def compute_control(self, state, target, telemetry=None, dt: float = 0.01):
        out = self.inner.compute_control(state, target, telemetry, dt)
        cmd = np.r_[out.steer_f, out.steer_r, out.T_drive, out.T_brake]
        n = int(round(self.delay_s / dt))
        if self._fifo is None:
            self._fifo = deque([cmd] * (n + 1), maxlen=n + 1)
            self._y = cmd.copy()
        self._fifo.append(cmd)
        delayed = self._fifo[0]
        a = 1.0 if self.tau_s <= 0.0 else dt / (self.tau_s + dt)
        self._y = self._y + a * (delayed - self._y)
        out.diagnostics = {**(out.diagnostics or {}), "commanded": {"steer_f": out.steer_f, "steer_r": out.steer_r},
                           "actuator_delay_steps": n}
        out.steer_f, out.steer_r = float(self._y[0]), float(self._y[1])
        out.T_drive, out.T_brake = self._y[2:6].copy(), np.maximum(self._y[6:10], 0.0)
        return out


def wrap_controller(ctrl, case: UncertaintyCase):
    """Add control latency, actuator delay and lag of ``case`` (unchanged controller if all are zero)."""
    delay = case.control_latency_s + case.actuator_delay_s
    if delay == 0.0 and case.actuator_tau_s == 0.0:
        return ctrl
    return DelayedActuation(ctrl, delay, case.actuator_tau_s)


__all__ = ("FIELDS", "DelayedActuation", "UncertaintyCase", "plant_factory", "plant_params", "sensor_config",
           "wrap_controller")
