"""Autonomous drift control on the existing 7-DOF plant (Drift Controller Implementierungsplan V1.0, Plan_v3.md).

Packages so far: D1 tyre audit (:mod:`.tyre_audit`), deterministic plant evaluation
(:mod:`.plant_adapter`); D2 drift equilibria (:mod:`.types`, :mod:`.equilibrium`). The plant is always :class:`~vehicle_model.vehicle_models.SevenDOFModel`
(or a subclass); this package computes no tyre or body forces of its own.
"""

from .equilibrium import DriftEquilibriumSolver, EquilibriumSettings, rwd_input
from .plant_adapter import BODY_NAMES, DeterministicPlant, LoadTransferNotConverged, PlantEvaluation
from .tyre_audit import AuditSettings, TyreAudit, audit_tyre, extrapolating
from .types import INFEASIBLE, VALID, VALID_EXTRAPOLATED, DriftControlDiagnostics, DriftEquilibrium, DriftReference

__all__ = ("INFEASIBLE", "VALID", "VALID_EXTRAPOLATED", "DriftControlDiagnostics", "DriftEquilibrium",
           "DriftEquilibriumSolver", "DriftReference", "EquilibriumSettings", "rwd_input", "AuditSettings", "BODY_NAMES", "DeterministicPlant", "LoadTransferNotConverged", "PlantEvaluation", "TyreAudit",
           "audit_tyre", "extrapolating")
