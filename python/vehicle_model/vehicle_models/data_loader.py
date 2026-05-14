"""Data loading helpers for vehicle model validation scripts."""

from __future__ import annotations

from pathlib import Path
import warnings

import numpy as np

from vehicle_model.config import load_settings


def _slice_from_config(value: dict[str, int]) -> slice:
    return slice(value["start"], value["stop"])


def load_data(file_path, idx=None, models=None, config_path=None):
    """Load logged vehicle data used by the validation demo.

    Returns:
        times: sample timestamps
        states: columns ``x, y, vx, vy, yaw, steer, yaw_rate, ax,
            wheel_fl, wheel_fr, wheel_rl, wheel_rr, roll``
        inputs: ``ax, steer_rate``
        ekin_inputs: array containing acceleration and steering channels
    """
    del models
    settings = load_settings(config_path)
    schema = settings["validation"]["data_schema"]
    units = settings["units"]

    path = Path(file_path)
    idx = schema["no_limit_value"] if idx is None else idx
    max_rows = None if idx == schema["no_limit_value"] else idx
    with warnings.catch_warnings():
        warnings.filterwarnings("ignore", message=schema["missing_line_warning"], category=UserWarning)
        data = np.loadtxt(
            path,
            delimiter=schema["delimiter"],
            comments=schema["comments"],
            max_rows=max_rows,
        )

    wheel_v = data[:, _slice_from_config(schema["wheel_speed_columns"])] * units["kmh_to_mps"]
    raw_states = data[:, _slice_from_config(schema["raw_state_columns"])]
    inputs = data[:, _slice_from_config(schema["input_columns"])]
    roll = data[:, schema["roll_column"]]
    times = data[:, schema["time_column"]]

    states = np.column_stack(
        [raw_states[:, idx] for idx in schema["raw_state_output_indices"]]
        + [wheel_v[:, idx] for idx in schema["wheel_speed_output_indices"]]
        + [roll]
    )

    ekin_schema = schema["ekin_inputs"]
    ekin_inputs = np.zeros((data.shape[0], ekin_schema["size"]))
    ekin_inputs[:, ekin_schema["acceleration_column"]] = raw_states[:, ekin_schema["acceleration_raw_state_index"]]
    ekin_inputs[:, ekin_schema["steering_rate_column"]] = inputs[:, ekin_schema["steering_rate_input_index"]]
    ekin_inputs[:, ekin_schema["steering_angle_column"]] = raw_states[:, ekin_schema["steering_angle_raw_state_index"]]
    ekin_inputs[:, ekin_schema["steering_acceleration_column"]] = np.gradient(
        raw_states[:, ekin_schema["steering_angle_raw_state_index"]],
        times,
        edge_order=ekin_schema["gradient_edge_order"],
    )
    return times, states, inputs, ekin_inputs


__all__ = ("load_data",)
