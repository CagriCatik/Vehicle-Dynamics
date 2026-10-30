"""Tyre model framework: common interface and parameter provenance.

Every tyre model used by the vehicle models implements :class:`TyreModel`.
Sign conventions (shared by all implementations, ISO 8855-like wheel frame):

* ``kappa > 0``  -> driving slip  -> ``Fx > 0`` (forward)
* ``alpha > 0``  -> ``Fy > 0`` (wheel velocity points to the right of the wheel
  plane, force acts to the left). The vehicle models define
  ``alpha = -atan2(v_wy, |v_wx|)`` so that this holds.

All methods are vectorised over numpy arrays (typically shape ``(4,)``).
Wheels with ``Fz <= 0`` or ``mu <= 0`` produce zero force.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Literal, Protocol, runtime_checkable

import numpy as np

DataClass = Literal["measured", "fitted", "synthetic"]


@dataclass(frozen=True)
class TyreProvenance:
    """Origin and validity information required for every tyre parameter set."""

    formulation: str
    version: str
    source: str
    data_class: DataClass
    reference_load: float  # [N]
    valid_kappa: tuple[float, float] = (-1.0, 1.0)
    valid_alpha: tuple[float, float] = (-0.35, 0.35)  # [rad]
    limitations: tuple[str, ...] = field(default_factory=tuple)

    def __post_init__(self) -> None:
        if self.data_class not in ("measured", "fitted", "synthetic"):
            raise ValueError(f"Invalid data_class {self.data_class!r}")
        if self.reference_load <= 0.0:
            raise ValueError("reference_load must be positive")

    def describe(self) -> str:
        return (f"{self.formulation} {self.version} [{self.data_class}] "
                f"Fz0={self.reference_load:.0f} N, source: {self.source}")


@runtime_checkable
class TyreModel(Protocol):
    """Minimal contract between tyre models and vehicle models."""

    provenance: TyreProvenance

    def forces(
        self,
        kappa: np.ndarray,
        alpha: np.ndarray,
        Fz: np.ndarray,
        mu: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Return combined-slip ``(Fx, Fy)`` in the wheel frame [N]."""
        ...

    def slip_stiffness(self, Fz: np.ndarray, mu: np.ndarray) -> np.ndarray:
        """Return ``dFx/dkappa`` at ``kappa = 0`` [N]; used for integrator step control."""
        ...


def as_wheel_arrays(*values) -> list[np.ndarray]:
    """Broadcast scalar/array inputs to float arrays of a common shape."""
    arrays = np.broadcast_arrays(*[np.asarray(v, dtype=float) for v in values])
    return [np.array(a, dtype=float) for a in arrays]


def load_sensitive(value: float, Fz: np.ndarray, Fz0: float, sensitivity: float) -> np.ndarray:
    """Linear load sensitivity ``value * (1 - sensitivity * dfz)`` with ``dfz = (Fz - Fz0)/Fz0``.

    The result is floored at 10 % of ``value`` to keep it physical at extreme loads.
    """
    dfz = (Fz - Fz0) / Fz0
    return np.maximum(value * (1.0 - sensitivity * dfz), 0.1 * value)
