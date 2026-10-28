"""Transient response metrics for step-like manoeuvres (Plan.md, Abschnitt 6.2 and 16).

Works on any sampled signal (linear reference or nonlinear simulation output).
Definitions follow ISO 7401 terminology where one exists:

    steady_state     mean of the signal over the final ``final_window`` seconds
    peak_value       extreme value in the direction of the steady state
    overshoot        (peak - steady) / |steady|                     [-]
    peak_time        time of the peak relative to the input step   [s]
    response_time    first time the signal reaches 90 % of steady   [s]  (ISO 7401 "response time")
    rise_time        time from 10 % to 90 % of steady               [s]
    settling_time    last time the signal leaves the +-band around steady  [s]

Limits: the signal must settle within the record; metrics are not meaningful for
oscillating, diverging or zero-mean responses (a ValueError is raised for |steady| ~ 0).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class StepResponseMetrics:
    steady_state: float
    peak_value: float
    overshoot: float
    peak_time: float
    response_time: float
    rise_time: float
    settling_time: float


def _first_crossing(t: np.ndarray, y: np.ndarray, level: float) -> float:
    """Linear interpolation of the first time y reaches ``level`` (y normalised to positive steady state)."""
    idx = np.flatnonzero(y >= level)
    if idx.size == 0:
        return float("nan")
    i = int(idx[0])
    if i == 0:
        return float(t[0])
    t0, t1, y0, y1 = t[i - 1], t[i], y[i - 1], y[i]
    return float(t0 + (level - y0) * (t1 - t0) / (y1 - y0))


def step_response_metrics(
    t: np.ndarray,
    y: np.ndarray,
    t_step: float = 0.0,
    final_window: float = 0.5,
    settling_band: float = 0.02,
) -> StepResponseMetrics:
    """Evaluate step-response metrics of ``y(t)`` for an input step at ``t_step``."""
    t = np.asarray(t, dtype=float)
    y = np.asarray(y, dtype=float)
    if t.shape != y.shape or t.ndim != 1:
        raise ValueError("t and y must be 1-D arrays of equal length")
    mask = t >= t_step
    t, y = t[mask] - t_step, y[mask]
    if t[-1] < final_window:
        raise ValueError("record is shorter than final_window")

    y0 = y[0]
    steady = float(np.mean(y[t >= t[-1] - final_window])) - y0
    if abs(steady) < 1e-12:
        raise ValueError("steady-state change is ~0; step metrics undefined")

    # Normalise to a positive, unit steady state
    yn = (y - y0) / steady
    i_peak = int(np.argmax(yn))
    peak_n = float(yn[i_peak])

    t10 = _first_crossing(t, yn, 0.1)
    t90 = _first_crossing(t, yn, 0.9)

    outside = np.flatnonzero(np.abs(yn - 1.0) > settling_band)
    if outside.size == 0:
        t_settle = 0.0
    elif outside[-1] == len(t) - 1:
        t_settle = float("nan")  # never settles within the record
    else:
        t_settle = float(t[outside[-1] + 1])

    return StepResponseMetrics(
        steady_state=steady + y0,
        peak_value=peak_n * steady + y0,
        overshoot=max(peak_n - 1.0, 0.0),
        peak_time=float(t[i_peak]),
        response_time=t90,
        rise_time=t90 - t10,
        settling_time=t_settle,
    )


__all__ = ("StepResponseMetrics", "step_response_metrics")
