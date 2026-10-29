"""YAML-driven experiments: specification, runner, verification, report (Plan.md M9; types: Plan V3.0 §12)."""

from .runner import run_experiment
from .spec import ControllerSpec, ExperimentSpec, load_experiment, parse_experiment
from .types import experiment_types, get_type
from .verification import verify_model

__all__ = ("ControllerSpec", "ExperimentSpec", "experiment_types", "get_type", "load_experiment", "parse_experiment",
           "run_experiment", "verify_model")
