"""Build tyre models from YAML parameter sets that carry mandatory provenance."""

from __future__ import annotations

from importlib.resources import files
from pathlib import Path
from typing import Any

import yaml

from vehicle_model.config.loader import get_configs_root
from .brush_models import FialaTyre, LinearTyre
from .combined_slip import SimplifiedCombinedSlipParams, SimplifiedCombinedSlipTyre
from .framework import TyreModel, TyreProvenance
from .magic_formula import MagicFormulaTyre, magic_formula_params_from_dict
from .pacejka import PacejkaTyre, PacejkaTyreParams

_REQUIRED_PROVENANCE = ("formulation", "version", "source", "data_class", "reference_load")


def tyre_config_dir() -> Path:
    root_tyres = get_configs_root() / "tyres"
    if root_tyres.is_dir():
        return root_tyres
    return Path(files("vehicle_model.config").joinpath("tyres"))


def available_tyres() -> list[str]:
    return sorted(p.stem for p in tyre_config_dir().glob("*.yaml"))


def _provenance(data: dict[str, Any], origin: str) -> TyreProvenance:
    prov = data.get("provenance")
    if not isinstance(prov, dict):
        raise ValueError(f"Tyre set {origin} has no 'provenance' section")
    missing = [k for k in _REQUIRED_PROVENANCE if k not in prov]
    if missing:
        raise ValueError(f"Tyre set {origin} provenance is missing: {', '.join(missing)}")
    return TyreProvenance(
        formulation=str(prov["formulation"]),
        version=str(prov["version"]),
        source=str(prov["source"]),
        data_class=prov["data_class"],
        reference_load=float(prov["reference_load"]),
        valid_kappa=tuple(prov.get("valid_kappa", (-1.0, 1.0))),
        valid_alpha=tuple(prov.get("valid_alpha", (-0.35, 0.35))),
        limitations=tuple(prov.get("limitations", ())),
    )


def tyre_from_dict(data: dict[str, Any], origin: str = "<dict>") -> TyreModel:
    model = data.get("model")
    params = dict(data.get("parameters", {}))
    provenance = _provenance(data, origin)
    if model == "linear":
        return LinearTyre(**params, provenance=provenance)
    if model == "fiala":
        return FialaTyre(**params, provenance=provenance)
    if model == "simplified_combined":
        return SimplifiedCombinedSlipTyre(SimplifiedCombinedSlipParams(**params), provenance=provenance)
    if model == "magic_formula":
        return MagicFormulaTyre(magic_formula_params_from_dict(params), provenance=provenance)
    if model == "pacejka":
        return PacejkaTyre(PacejkaTyreParams(**params), provenance=provenance)
    raise ValueError(f"Unknown tyre model {model!r} in {origin}")


def load_tyre(name_or_path: str | Path) -> TyreModel:
    """Load a tyre by bundled name (e.g. ``"fiala_passenger_synthetic"``) or YAML path."""
    path = Path(name_or_path)
    if path.suffix not in (".yaml", ".yml"):
        path = tyre_config_dir() / f"{name_or_path}.yaml"
    if not path.is_file():
        raise FileNotFoundError(f"Tyre set not found: {path}. Available: {available_tyres()}")
    with path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}
    return tyre_from_dict(data, origin=str(path))


create_tyre = load_tyre

__all__ = ("load_tyre", "create_tyre", "tyre_from_dict", "available_tyres", "tyre_config_dir")

