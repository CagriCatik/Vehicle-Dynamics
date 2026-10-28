"""Composed validation matrix runner."""

from __future__ import annotations

from .base import ValidationRunnerBase
from .handling import HandlingScenariosMixin
from .braking import BrakingScenariosMixin
from .stability import StabilityScenariosMixin
from .ride import RideScenariosMixin


class ValidationMatrixRunner(ValidationRunnerBase, HandlingScenariosMixin, BrakingScenariosMixin, StabilityScenariosMixin, RideScenariosMixin):
    """Orchestrates multi-scenario automotive test matrix execution.

    Scenario methods are grouped in mixins by topic (see the package modules).
    """
