"""Controller fed by virtual sensors and observers instead of the true vehicle state (Plan.md 10.2, 11; M7; V3.0 M10 5.4).

``EstimatedStateFeedback`` wraps any ``VehicleController``. Each step it

1. measures the vehicle with a :class:`~vehicle_model.estimation.sensors.SensorSuite`
   (IMU accelerations from the change of the true state over the last step, yaw
   rate, wheel speeds, steering angles),
2. updates a sensors-only motion observer,
3. passes the inner controller a state with ``v_x``, ``v_y`` from the observer and the
   measured yaw rate ``r`` (positions and heading are not measured and set to 0).

Operating modes (Plan V3.0 M10 5.4; Mode A, ideal feedback, is the unwrapped controller):

    "B"  Estimated Motion State. The simulation's wheel telemetry (loads, lateral
         forces, slips, road friction) is still passed to allocation and slip
         control, declared in ``information``.
    "C"  Sensor-Constrained Closed Loop. The inner controller receives no simulation
         telemetry. A :class:`~vehicle_model.estimation.sensor_friction.SensorFrictionEstimator`
         (sensors and the controller's own previous torque commands only) supplies
         wheel loads, longitudinal and lateral forces, slips and friction
         (``as_telemetry``); before its speed reference exists, the inner controller
         gets ``None`` and falls back to nominal loads and ``target.mu_estimated``.

The true state enters only through the sensor model (as a real vehicle would be
measured); nothing else from the plant reaches the controller in Mode C. The
observers are initialised from the first wheel-speed measurement, not from the
true speed.
"""

from __future__ import annotations

import numpy as np

from vehicle_model.core.types import SevenDOFState
from vehicle_model.estimation.sensors import SensorConfig, SensorSuite

INFORMATION_MODE_B = "estimated_motion_state; allocation and slip control use telemetry"
INFORMATION_MODE_C = "sensor_constrained; motion, wheel loads, forces, slips and friction estimated from sensors"


class EstimatedStateFeedback:
    """VehicleController wrapper: motion feedback (B) or the complete feedback (C) from sensors and estimators."""

    def __init__(self, inner, observer, sensor_config: SensorConfig | None = None, seed: int = 0,
                 mode: str = "B", params=None, friction_estimator=None):
        if getattr(observer, "information", None) != "sensors_only":
            raise ValueError("the observer must use sensor measurements only")
        if mode not in ("B", "C"):
            raise ValueError(f"mode must be 'B' or 'C', got {mode!r}")
        self.inner = inner
        self.observer = observer
        self.mode = mode
        self.sensors = SensorSuite(sensor_config, seed=seed)
        self.friction = None
        if mode == "C":
            if friction_estimator is None:
                from vehicle_model.estimation.sensor_friction import SensorFrictionEstimator
                p = params if params is not None else getattr(inner, "params", None)
                if p is None:
                    raise ValueError("mode C needs vehicle parameters (params=...) for the friction estimator")
                friction_estimator = SensorFrictionEstimator(p)
            if getattr(friction_estimator, "information", None) != "sensors_only":
                raise ValueError("the friction estimator must use sensor measurements only")
            self.friction = friction_estimator
        self.information = INFORMATION_MODE_C if mode == "C" else INFORMATION_MODE_B
        p = params if params is not None else getattr(inner, "params", getattr(observer, "params", None))
        if p is None:
            raise ValueError("vehicle parameters (params=...) are needed to initialise the observers from wheel speeds")
        self.Reff = float(p.Reff)
        self.reset()

    @property
    def name(self) -> str:
        return f"{self.inner.name} + {type(self.observer).__name__}" + (" (mode C)" if self.mode == "C" else "")

    def reset(self) -> None:
        self.inner.reset()
        self.sensors.reset()
        self._prev = None
        self._t = 0.0
        self._initialised = False
        self._last_cmd = (np.zeros(4), np.zeros(4))
        self.last_estimate = None
        self.last_sensor_telemetry = None

    def compute_control(self, state: SevenDOFState, target, telemetry=None, dt: float = 0.01):
        if self._prev is None:
            ax = ay = 0.0
        else:
            ax = (state.vx - self._prev.vx) / dt - state.r * state.vy
            ay = (state.vy - self._prev.vy) / dt + state.r * state.vx
        self._prev = SevenDOFState.from_array(state.to_array()[:10])
        self._t += dt
        # the rear steering angle is a sensor channel; the plant reports it through telemetry
        steer_r = telemetry.steer_angles[2] if telemetry is not None else 0.0
        meas = self.sensors.measure(self._t, dt, ax, ay, state.r, state.omega, target.driver_steer_f, steer_r)
        if not self._initialised:
            # no true initial speed: the observers start from the measured wheel speeds (middle two)
            v0 = float(np.mean(np.sort(meas.omega)[1:3])) * self.Reff
            self.observer.reset(init_vx=v0)
            if self.friction is not None:
                self.friction.reset(init_vx=v0)
            self._initialised = True
        est = self.observer.update(meas, dt)
        self.last_estimate = est
        seen = SevenDOFState(vx=est.vx, vy=est.vy, r=meas.r, omega=meas.omega.copy())
        if self.mode == "C":
            self.friction.update(meas, *self._last_cmd, dt, vy=est.vy)
            inner_tel = self.friction.as_telemetry(meas)
            self.last_sensor_telemetry = inner_tel
        else:
            inner_tel = telemetry
        out = self.inner.compute_control(seen, target, inner_tel, dt)
        self._last_cmd = (np.asarray(out.T_drive, float).copy(), np.asarray(out.T_brake, float).copy())
        out.diagnostics["estimated_sideslip"] = est.sideslip
        out.diagnostics["information"] = self.information
        if self.mode == "C":
            out.diagnostics["sensor_telemetry_available"] = inner_tel is not None
            out.diagnostics["mu_estimate"] = self.friction.mu.tolist()
        return out


__all__ = ("INFORMATION_MODE_B", "INFORMATION_MODE_C", "EstimatedStateFeedback")
