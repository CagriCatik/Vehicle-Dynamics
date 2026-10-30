"""Strict loading of ``configs/drift/*.yaml``: unknown keys and wrong ranges are errors."""

from __future__ import annotations

from pathlib import Path

import yaml


def require(cond: bool, msg: str) -> None:
    if not cond:
        raise ValueError(msg)


def keys(d: dict, allowed: set, where: str, required: set = frozenset()) -> None:
    require(isinstance(d, dict), f"{where} must be a mapping")
    unknown = set(d) - set(allowed)
    require(not unknown, f"{where}: unknown key(s) {sorted(unknown)}; allowed: {sorted(allowed)}")
    missing = set(required) - set(d)
    require(not missing, f"{where}: missing key(s) {sorted(missing)}")


def numbers(values, where: str, lo: float, hi: float) -> list:
    require(isinstance(values, list) and values, f"{where} must be a non-empty list")
    out = []
    for v in values:
        require(isinstance(v, (int, float)) and not isinstance(v, bool), f"{where}: {v!r} is not a number")
        require(lo <= float(v) <= hi, f"{where}: {v} outside [{lo}, {hi}]")
        out.append(float(v))
    require(len(set(out)) == len(out), f"{where}: values must be unique")
    return out


def load_yaml(path: str | Path) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    require(isinstance(data, dict), f"{path}: top level must be a mapping")
    return data


def vehicle_model(vehicle: str, tyres: dict, where: str):
    """The plant: SevenDOFModel from an existing vehicle file and existing tyre files (no new parameters)."""
    from vehicle_model.tire_model import available_tyres
    from vehicle_model.vehicle_models import SevenDOFModel
    require(Path(vehicle).is_file(), f"{where}: vehicle file not found: {vehicle}")
    keys(tyres, {"front", "rear"}, f"{where}: tyres")
    for k, n in tyres.items():
        require(n in available_tyres(), f"{where}: tyres.{k}: unknown tyre {n!r}; available: {available_tyres()}")
    return SevenDOFModel.from_yaml(vehicle, tyre_name=tyres.get("front"), tyre_rear_name=tyres.get("rear"))


__all__ = ("keys", "load_yaml", "numbers", "require", "vehicle_model")
