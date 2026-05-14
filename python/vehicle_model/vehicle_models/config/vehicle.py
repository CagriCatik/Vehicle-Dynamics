"""Vehicle parameters for the single-track models."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from vehicle_model.config import get_settings_section


@dataclass(frozen=True)
class VehicleParams:
    g: float
    lf: float
    lr: float
    mass: float
    Iz: float
    hcog: float
    mu: float
    min_v: float
    max_v: float
    max_acc: float
    min_acc: float
    max_steer: float
    min_steer: float
    max_steer_vel: float
    min_steer_vel: float
    width: float
    length: float
    l_pressure: float
    tire_radius: float
    rolling_resistance: float
    air_density: float
    drag_coefficient: float
    frontal_area: float | None
    switch_v: float
    min_slip_speed: float
    low_speed_yaw_tau: float
    low_speed_lateral_tau: float
    MF_lat: dict[str, float]

    @property
    def wheelbase(self) -> float:
        return self.lf + self.lr

    @property
    def resolved_frontal_area(self) -> float:
        if self.frontal_area is None:
            raise ValueError("Vehicle parameter 'frontal_area' must be configured.")
        return self.frontal_area

    @property
    def max_inputs(self) -> list[float]:
        return [self.max_acc, self.max_steer]

    @property
    def min_inputs(self) -> list[float]:
        return [self.min_acc, self.min_steer]

    @property
    def max_rates(self) -> list[float | None]:
        return [None, self.max_steer_vel]

    @property
    def min_rates(self) -> list[float | None]:
        return [None, self.min_steer_vel]

def vehicle_params_from_config(config_path: str | None = None) -> VehicleParams:
    return coerce_vehicle_params(get_settings_section("vehicle", config_path))


def coerce_vehicle_params(params: VehicleParams | dict[str, Any] | None = None) -> VehicleParams:
    if params is None:
        return vehicle_params_from_config()
    if isinstance(params, VehicleParams):
        return params
    if isinstance(params, dict):
        data = dict(params.get("vehicle", params))
        allowed = set(VehicleParams.__dataclass_fields__)
        missing = sorted(allowed - set(data))
        if missing:
            raise KeyError(f"Missing vehicle parameter(s): {', '.join(missing)}")
        return VehicleParams(**{key: data[key] for key in allowed})
    raise TypeError(f"Unsupported vehicle params type: {type(params)!r}")


__all__ = ("VehicleParams", "coerce_vehicle_params", "vehicle_params_from_config")
