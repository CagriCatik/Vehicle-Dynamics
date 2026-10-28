"""Telemetry signal generation for validation plots."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any

import numpy as np

from .dynamic_single_track import DynamicSingleTrack


@dataclass(frozen=True)
class ValidationTelemetry:
    time: np.ndarray
    signals: dict[str, np.ndarray]


def _simulation_time(simulation: Any) -> np.ndarray:
    times = getattr(simulation, "times", None)
    if times is None:
        return np.arange(simulation.N_SAMPLES, dtype=float) * simulation.SAMPLING_TIME
    time = np.asarray(times, dtype=float)
    if time.size != simulation.N_SAMPLES:
        return np.arange(simulation.N_SAMPLES, dtype=float) * simulation.SAMPLING_TIME
    return time - time[0]


def _angle_error(lhs: np.ndarray, rhs: np.ndarray) -> np.ndarray:
    return np.arctan2(np.sin(lhs - rhs), np.cos(lhs - rhs))


def _gradient(values: np.ndarray, time: np.ndarray) -> np.ndarray:
    if len(values) < 2:
        return np.zeros_like(values)
    return np.gradient(values, time, edge_order=1)


def _state_channels(simulation: Any, time: np.ndarray) -> dict[str, np.ndarray]:
    true_states = np.asarray(simulation.true_states, dtype=float)
    pred_states = np.asarray(simulation.states_pred, dtype=float)

    if simulation.veh_model == "kinematic":
        x_true, y_true, yaw_true, vx_true, steer_true = true_states[:, 0], true_states[:, 1], true_states[:, 2], true_states[:, 3], true_states[:, 4]
        x_pred, y_pred, yaw_pred, vx_pred, steer_pred = pred_states[:, 0], pred_states[:, 1], pred_states[:, 2], pred_states[:, 3], pred_states[:, 4]
        vy_true = np.zeros_like(vx_true)
        yaw_rate_true = _gradient(yaw_true, time)
        yaw_rate_pred = _gradient(yaw_pred, time)
    else:
        x_true, y_true, vx_true, vy_true, yaw_true, steer_true, yaw_rate_true = (
            true_states[:, 0],
            true_states[:, 1],
            true_states[:, 2],
            true_states[:, 3],
            true_states[:, 4],
            true_states[:, 5],
            true_states[:, 6],
        )
        x_pred, y_pred, vx_pred, _, yaw_pred, steer_pred, yaw_rate_pred = (
            pred_states[:, 0],
            pred_states[:, 1],
            pred_states[:, 2],
            pred_states[:, 3],
            pred_states[:, 4],
            pred_states[:, 5],
            pred_states[:, 6],
        )

    position_error = np.hypot(x_pred - x_true, y_pred - y_true)
    yaw_error = _angle_error(yaw_pred, yaw_true)
    return {
        "vx_true": vx_true,
        "vx_pred": vx_pred,
        "vx_error": vx_pred - vx_true,
        "vy_true": vy_true,
        "steer_true_deg": np.degrees(steer_true),
        "steer_pred_deg": np.degrees(steer_pred),
        "yaw_rate_true_deg_s": np.degrees(yaw_rate_true),
        "yaw_rate_pred_deg_s": np.degrees(yaw_rate_pred),
        "position_error": position_error,
        "yaw_error_deg": np.degrees(yaw_error),
    }


def _empty_dynamic_signals(size: int) -> dict[str, np.ndarray]:
    values = np.full(size, np.nan)
    return {
        "force_front_lateral": values.copy(),
        "force_rear_lateral": values.copy(),
        "force_front_lateral_pred": values.copy(),
        "force_rear_lateral_pred": values.copy(),
        "force_longitudinal_drive": values.copy(),
        "force_longitudinal_net": values.copy(),
        "force_drag": values.copy(),
        "force_rolling": values.copy(),
        "slip_front_deg": values.copy(),
        "slip_rear_deg": values.copy(),
        "slip_balance_deg": values.copy(),
        "slip_ratio_percent": values.copy(),
        "front_tire_utilization": values.copy(),
        "rear_tire_utilization": values.copy(),
    }


def _dynamic_signals(simulation: Any) -> dict[str, np.ndarray]:
    size = simulation.N_SAMPLES
    signals = _empty_dynamic_signals(size)
    if simulation.veh_model != "dynamic":
        return signals

    model = DynamicSingleTrack(simulation.vehicle.params)
    wheelbase = model.params.wheelbase
    front_normal_load = model.mass * model.g * model.lr / wheelbase
    rear_normal_load = model.mass * model.g * model.lf / wheelbase
    front_force_capacity = max(model.params.mu * front_normal_load, np.finfo(float).eps)
    rear_force_capacity = max(model.params.mu * rear_normal_load, np.finfo(float).eps)

    for idx in range(size):
        command = simulation.inputs[min(idx, len(simulation.inputs) - 1), :]
        wheel_speed = simulation.wheel_data[min(idx, len(simulation.wheel_data) - 1), :]
        roll = simulation.roll_data[min(idx, len(simulation.roll_data) - 1)]
        true_state = simulation.true_states[idx, :]
        pred_state = simulation.states_pred[idx, :]

        forces = model.force_state(true_state, command, wheel_v=wheel_speed, roll=roll)
        pred_forces = model.force_state(pred_state, command, wheel_v=wheel_speed, roll=roll)
        kappa, alpha_front, alpha_rear = model.calc_slips(true_state, wheel_v=wheel_speed)

        signals["force_front_lateral"][idx] = forces.Fyf
        signals["force_rear_lateral"][idx] = forces.Fyr
        signals["force_front_lateral_pred"][idx] = pred_forces.Fyf
        signals["force_rear_lateral_pred"][idx] = pred_forces.Fyr
        signals["force_longitudinal_drive"][idx] = forces.Fxr
        signals["force_drag"][idx] = forces.drag
        signals["force_rolling"][idx] = forces.rolling
        signals["force_longitudinal_net"][idx] = forces.Fxr - forces.drag - forces.rolling
        signals["slip_front_deg"][idx] = math.degrees(alpha_front)
        signals["slip_rear_deg"][idx] = math.degrees(alpha_rear)
        signals["slip_balance_deg"][idx] = math.degrees(alpha_front - alpha_rear)
        signals["slip_ratio_percent"][idx] = 100.0 * kappa
        signals["front_tire_utilization"][idx] = abs(forces.Fyf) / front_force_capacity
        signals["rear_tire_utilization"][idx] = math.hypot(forces.Fyr, forces.Fxr) / rear_force_capacity
    return signals


def build_validation_telemetry(simulation: Any) -> ValidationTelemetry:
    time = _simulation_time(simulation)
    inputs = np.asarray(simulation.inputs, dtype=float)
    signals = _state_channels(simulation, time)
    signals["ax_command"] = inputs[:, 0]
    signals["steer_rate_command_deg_s"] = np.degrees(inputs[:, 1])
    signals.update(_dynamic_signals(simulation))
    return ValidationTelemetry(time=time, signals=signals)


__all__ = ("ValidationTelemetry", "build_validation_telemetry")
