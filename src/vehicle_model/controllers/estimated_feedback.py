"""Controller fed by virtual sensors and an observer instead of the true vehicle state (Plan.md 10.2, 11; M7).

``EstimatedStateFeedback`` wraps any ``VehicleController``. Each step it

1. measures the vehicle with a :class:`~vehicle_model.estimation.sensors.SensorSuite`
   (IMU accelerations from the change of the true state over the last step, yaw
   rate, wheel speeds, steering angle),
2. updates a sensors-only observer,
3. passes the inner controller a state with ``v_x``, ``v_y`` from the observer and the
   measured yaw rate ``r`` (positions and heading are not measured and set to 0).

The telemetry (wheel loads, lateral forces, road friction) is still passed through
for the allocation and slip control layer, so only the motion feedback is
realistic; ``information`` says so.
"""

from __future__ import annotations

import numpy as np

from vehicle_model.core.types import SevenDOFState
from vehicle_model.estimation.sensors import SensorConfig, SensorSuite


class EstimatedStateFeedback:
    """VehicleController wrapper: motion feedback from sensors and an observer."""

    def __init__(self, inner, observer, sensor_config: SensorConfig | None = None, seed: int = 0):
        if getattr(observer, "information", None) != "sensors_only":
            raise ValueError("the observer must use sensor measurements only")
        self.inner = inner
        self.observer = observer
        self.sensors = SensorSuite(sensor_config, seed=seed)
        self.information = "estimated_motion_state; allocation and slip control use telemetry"
        self.reset()

    @property
    def name(self) -> str:
        return f"{self.inner.name} + {type(self.observer).__name__}"

    def reset(self) -> None:
        self.inner.reset()
        self.sensors.reset()
        self._prev = None
        self._t = 0.0
        self._initialised = False
        self.last_estimate = None

    def compute_control(self, state: SevenDOFState, target, telemetry=None, dt: float = 0.01):
        if not self._initialised:
            self.observer.reset(init_vx=state.vx)
            self._initialised = True
        if self._prev is None:
            ax = ay = 0.0
        else:
            ax = (state.vx - self._prev.vx) / dt - state.r * state.vy
            ay = (state.vy - self._prev.vy) / dt + state.r * state.vx
        self._prev = SevenDOFState.from_array(state.to_array()[:10])
        self._t += dt
        steer_r = telemetry.steer_angles[2] if telemetry is not None else 0.0
        meas = self.sensors.measure(self._t, dt, ax, ay, state.r, state.omega, target.driver_steer_f, steer_r)
        est = self.observer.update(meas, dt)
        self.last_estimate = est
        seen = SevenDOFState(vx=est.vx, vy=est.vy, r=meas.r, omega=meas.omega.copy())
        out = self.inner.compute_control(seen, target, telemetry, dt)
        out.diagnostics["estimated_sideslip"] = est.sideslip
        out.diagnostics["information"] = self.information
        return out


__all__ = ("EstimatedStateFeedback",)
