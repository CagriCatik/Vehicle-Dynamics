"""Observer evaluation: plant -> virtual sensors -> observers, truth used only for scoring.

The study loop separates information strictly:

* the plant state and telemetry (ground truth) go to the sensor suite and to the
  error metric only;
* observers receive nothing but :class:`~vehicle_model.estimation.sensors.SensorMeasurements`
  and the step size, through ``observer.update(measurement, dt)``.

Observers must declare ``information = "sensors_only"``; others are rejected, so an
observer with ground-truth access cannot be scored as if it had none.

Body accelerations for the IMU are the true accelerations of the plant,
``a_x = dv_x/dt - r v_y`` and ``a_y = dv_y/dt + r v_x`` over the step.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable

import numpy as np

from vehicle_model.core.types import SevenDOFControlInput, SevenDOFState
from vehicle_model.estimation.sensors import SensorConfig, SensorSuite


@dataclass
class ObserverResult:
    name: str
    t: np.ndarray
    beta_true: np.ndarray      # [rad]
    beta_est: np.ndarray       # [rad]

    @property
    def error_deg(self) -> np.ndarray:
        return np.degrees(self.beta_est - self.beta_true)

    def metrics(self) -> dict:
        e = self.error_deg
        return {"rms_deg": float(np.sqrt(np.mean(e**2))), "max_abs_deg": float(np.max(np.abs(e))),
                "mean_deg": float(np.mean(e)), "final_deg": float(e[-1])}


def run_estimation_study(model, observers: dict, steer: Callable[[float], float], duration: float, v0: float,
                         sensor_config: SensorConfig | None = None, seed: int = 0, dt: float = 0.005) -> dict:
    """Simulate an open-loop steering manoeuvre and score each observer's sideslip estimate."""
    for name, obs in observers.items():
        if getattr(obs, "information", None) != "sensors_only":
            raise ValueError(f"observer {name!r} does not declare information = 'sensors_only'")
        obs.reset(init_vx=v0)
    p = model.params
    sensors = SensorSuite(sensor_config, seed=seed)
    s = SevenDOFState(vx=v0, omega=np.full(4, v0 / p.Reff))
    t, ts, beta = 0.0, [], []
    est = {name: [] for name in observers}
    while t < duration - 1e-12:
        delta = steer(t)
        s_new, _ = model.step(s, SevenDOFControlInput(steer_f=delta), dt)
        ax = (s_new.vx - s.vx) / dt - s.r * s.vy
        ay = (s_new.vy - s.vy) / dt + s.r * s.vx
        s, t = s_new, t + dt
        meas = sensors.measure(t, dt, ax, ay, s.r, s.omega, delta)
        for name, obs in observers.items():
            est[name].append(obs.update(meas, dt).sideslip)
        ts.append(t)
        beta.append(math.atan2(s.vy, s.vx))
    ts, beta = np.array(ts), np.array(beta)
    return {name: ObserverResult(name, ts, beta, np.array(v)) for name, v in est.items()}


__all__ = ("ObserverResult", "run_estimation_study")
