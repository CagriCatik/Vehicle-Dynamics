"""YAML configuration loading for the vehicle-model package."""

from __future__ import annotations

from collections.abc import Mapping
import copy
import os
from importlib.resources import files
from pathlib import Path
from typing import Any

import yaml


CONFIG_ENV_VAR = "VEHICLE_MODEL_CONFIG"


def default_config_path() -> Path:
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
