"""Experiment types: a small common interface for the runner (Plan V3.0 §12).

Every experiment type validates its own YAML schema and runs itself; provenance,
the output directory and ``results.json`` are shared (:func:`provenance`,
:func:`write_json`). A specification selects its type with the top-level key
``type``; without it the type is ``controller_comparison`` (the M9 format), so
existing specifications stay valid.

    type                   module                                   what it does
    controller_comparison  :mod:`.spec`, :mod:`.runner`             controllers on a manoeuvre, optional sweep
    solver_benchmark       :mod:`.solver_benchmark`                 NMPC solver variants: status, KKT, warm start,
                                                                    computation time, tracking
    equilibrium_analysis   :mod:`.equilibrium_analysis`             nonlinear equilibria, Jacobians, modes over a sweep
    stability_map          :mod:`.stability_map`                    recovery regions per controller with uncertainty

Interface of a type (:class:`ExperimentType`):

    parse(data, source) -> spec        validated, immutable specification; ``spec.kind`` is the type name
    run(spec, out_dir, verify) -> dict summary with at least ``experiment``, ``source``, ``provenance``,
                                       ``verification`` (list of checks, may be empty) and ``results``
    describe(spec) -> str              one line for ``python -m vehicle_model.experiments list``
"""

from __future__ import annotations

import json
import platform
import subprocess
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable

import numpy as np

DEFAULT_TYPE = "controller_comparison"


@dataclass(frozen=True)
class ExperimentType:
    name: str
    parse: Callable[[dict, str], Any]
    run: Callable[[Any, Path, "bool | None"], dict]
    describe: Callable[[Any], str]


def _registry() -> dict[str, ExperimentType]:
    # imported here: the type modules import the shared helpers of this module
    from . import equilibrium_analysis, runner, solver_benchmark, spec, stability_map

    types = (
        ExperimentType(DEFAULT_TYPE, spec.parse_controller_comparison, runner.run_controller_comparison,
                       runner.describe_controller_comparison),
        ExperimentType("solver_benchmark", solver_benchmark.parse, solver_benchmark.run, solver_benchmark.describe),
        ExperimentType("equilibrium_analysis", equilibrium_analysis.parse, equilibrium_analysis.run,
                       equilibrium_analysis.describe),
        ExperimentType("stability_map", stability_map.parse, stability_map.run, stability_map.describe),
    )
    return {t.name: t for t in types}


def experiment_types() -> tuple[str, ...]:
    return tuple(_registry())


def get_type(name: str) -> ExperimentType:
    reg = _registry()
    if name not in reg:
        raise ValueError(f"unknown experiment type {name!r}; known: {sorted(reg)}")
    return reg[name]


def provenance() -> dict:
    """Git commit, interpreter and library versions, creation time."""
    try:
        commit = subprocess.run(["git", "rev-parse", "--short", "HEAD"], capture_output=True, text=True,
                                timeout=5).stdout.strip()
    except (OSError, subprocess.SubprocessError):
        commit = ""
    import scipy
    return {"git_commit": commit or "unknown", "python": platform.python_version(), "numpy": np.__version__,
            "scipy": scipy.__version__, "created_utc": datetime.now(timezone.utc).isoformat(timespec="seconds")}


def json_default(o):
    if isinstance(o, (np.floating, np.integer, np.bool_)):
        return o.item()
    if isinstance(o, np.ndarray):
        return o.tolist()
    raise TypeError(type(o))


def write_json(path: Path, data: dict) -> None:
    path.write_text(json.dumps(data, indent=2, default=json_default), encoding="utf-8")


def slug(s: str) -> str:
    return "".join(ch if ch.isalnum() or ch in "-_." else "_" for ch in s).strip("_")


__all__ = ("DEFAULT_TYPE", "ExperimentType", "experiment_types", "get_type", "json_default", "provenance", "slug",
           "write_json")
