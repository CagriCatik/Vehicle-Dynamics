"""YAML configuration loading for the vehicle-model package with centralized configs/ support."""

from __future__ import annotations

from collections.abc import Mapping
import copy
import os
from importlib.resources import files
from pathlib import Path
from typing import Any

import yaml


CONFIG_ENV_VAR = "VEHICLE_MODEL_CONFIG"
CONFIGS_ROOT_ENV_VAR = "VEHICLE_DYNAMICS_CONFIGS_DIR"


def get_configs_root() -> Path:
    """Return the absolute path to the centralized configs/ root directory."""
    env_dir = os.getenv(CONFIGS_ROOT_ENV_VAR)
    if env_dir:
        p = Path(env_dir).expanduser().resolve()
        if p.is_dir():
            return p

    # 1. Check relative to this source file (Vehicle-Dynamics/src/vehicle_model/config/loader.py -> parents[3])
    source_root = Path(__file__).resolve().parents[3] / "configs"
    if source_root.is_dir():
        return source_root

    # 2. Check current working directory
    cwd_root = Path.cwd() / "configs"
    if cwd_root.is_dir():
        return cwd_root

    # 3. Fallback to package bundled config directory
    try:
        pkg_dir = Path(files("vehicle_model.config"))
        if pkg_dir.is_dir():
            return pkg_dir
    except Exception:
        pass

    return Path(__file__).resolve().parent


def default_config_path() -> Path:
    """Path to master default.yaml (prefers root configs/default.yaml)."""
    root_default = get_configs_root() / "default.yaml"
    if root_default.is_file():
        return root_default
    return Path(files("vehicle_model.config").joinpath("default.yaml"))


def _read_yaml(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}
    if not isinstance(data, dict):
        raise ValueError(f"Configuration file must contain a mapping: {path}")
    return data


def _deep_merge(base: Mapping[str, Any], override: Mapping[str, Any]) -> dict[str, Any]:
    merged = copy.deepcopy(dict(base))
    for key, value in override.items():
        if isinstance(value, Mapping) and isinstance(merged.get(key), Mapping):
            merged[key] = _deep_merge(merged[key], value)
        else:
            merged[key] = copy.deepcopy(value)
    return merged


def load_settings(config_path: str | os.PathLike[str] | None = None) -> dict[str, Any]:
    """Load default settings and optionally merge an override YAML file."""
    settings = _read_yaml(default_config_path())
    override_path = config_path or os.getenv(CONFIG_ENV_VAR)
    if override_path:
        path = Path(override_path).expanduser()
        if not path.is_absolute():
            path = Path.cwd() / path
        settings = _deep_merge(settings, _read_yaml(path))
    return settings


def get_settings_section(name: str, config_path: str | os.PathLike[str] | None = None) -> dict[str, Any]:
    settings = load_settings(config_path)
    section = settings.get(name)
    if not isinstance(section, dict):
        raise KeyError(f"Missing configuration section: {name}")
    return section


def load_yaml_config(subpath_or_name: str | Path) -> dict[str, Any]:
    """Load any YAML configuration file from centralized configs/ directory."""
    p = Path(subpath_or_name)
    if p.is_file():
        return _read_yaml(p)
    if p.suffix not in (".yaml", ".yml"):
        p = p.with_suffix(".yaml")
    full_path = get_configs_root() / p
    if not full_path.is_file():
        raise FileNotFoundError(f"Configuration file not found: {full_path} (searched in {get_configs_root()})")
    return _read_yaml(full_path)


def load_vehicle_config(name: str = "seven_dof_passenger") -> dict[str, Any]:
    """Load vehicle parameters from configs/vehicles/<name>.yaml."""
    return load_yaml_config(Path("vehicles") / name)


def load_steering_config(name: str = "four_wheel_steering") -> dict[str, Any]:
    """Load steering parameters from configs/steering/<name>.yaml."""
    return load_yaml_config(Path("steering") / name)


def load_powertrain_config(name: str = "electric_powertrain") -> dict[str, Any]:
    """Load powertrain parameters from configs/powertrains/<name>.yaml."""
    return load_yaml_config(Path("powertrains") / name)


def load_braking_config(name: str = "brake_by_wire") -> dict[str, Any]:
    """Load braking parameters from configs/braking/<name>.yaml."""
    return load_yaml_config(Path("braking") / name)


def load_controller_config(name: str = "integrated_controller") -> dict[str, Any]:
    """Load controller parameters from configs/controllers/<name>.yaml."""
    return load_yaml_config(Path("controllers") / name)


def load_road_config(name: str = "surfaces") -> dict[str, Any]:
    """Load road surface presets from configs/roads/<name>.yaml."""
    return load_yaml_config(Path("roads") / name)


def load_scenario_config(name: str = "double_lane_change") -> dict[str, Any]:
    """Load driving scenario configuration from configs/scenarios/<name>.yaml."""
    return load_yaml_config(Path("scenarios") / name)


__all__ = (
    "default_config_path",
    "get_configs_root",
    "get_settings_section",
    "load_settings",
    "load_yaml_config",
    "load_vehicle_config",
    "load_steering_config",
    "load_powertrain_config",
    "load_braking_config",
    "load_controller_config",
    "load_road_config",
    "load_scenario_config",
)
