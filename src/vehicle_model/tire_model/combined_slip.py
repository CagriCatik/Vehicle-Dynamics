"""Simplified combined-slip tyre based on a normalised slip vector.

This is *not* a Pacejka MF 5.2/6.x implementation. It uses a single
Magic-Formula-like envelope ``F(sigma)`` on the normalised slip magnitude

    sigma_x = kappa / kappa_peak,  sigma_y = tan(alpha) / tan(alpha_peak),
    sigma   = sqrt(sigma_x^2 + sigma_y^2)

and distributes the force along the normalised slip direction. This keeps the
resultant inside the friction circle, reduces lateral force as longitudinal
slip grows and includes a mild friction drop after the peak.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math

import numpy as np

from .framework import TyreProvenance, as_wheel_arrays, load_sensitive


@dataclass(frozen=True)
class SimplifiedCombinedSlipParams:
    kappa_peak: float = 0.12                 # Slip ratio at peak force [-]
    alpha_peak: float = math.radians(4.5)    # Slip angle at peak force at Fz0 [rad]
    sliding_ratio: float = 0.85              # Sliding / peak friction ratio [-]
    peak_load_sensitivity: float = 0.3       # alpha_peak grows with load -> C_alpha/Fz falls (understeer)
    friction_load_sensitivity: float = 0.10  # mu falls with load
    Fz0: float = 3000.0                      # Reference load [N]


class SimplifiedCombinedSlipTyre:
    """Normalised-slip combined tyre (synthetic parameters)."""

    _SHAPE_C = 1.5
    _SHAPE_B = 1.1
    _SIGMA_PEAK_END = 1.2
    _SIGMA_SLIDE_RANGE = 4.0

    def __init__(
        self,
        params: SimplifiedCombinedSlipParams | None = None,
        provenance: TyreProvenance | None = None,
    ):
        self.params = params or SimplifiedCombinedSlipParams()
        self.provenance = provenance or TyreProvenance(
            formulation="Simplified normalised combined slip", version="1.0",
            source="Synthetic passenger-car values chosen for illustration; not fitted to measurements.",
            data_class="synthetic", reference_load=self.params.Fz0,
            limitations=(
                "Not a Pacejka MF 5.2/6.x model",
                "Single envelope shape for Fx and Fy",
                "No camber, pressure, relaxation or aligning moment",
            ),
        )

    def _envelope(self, sigma: np.ndarray) -> np.ndarray:
        p = self.params
        rising = np.sin(self._SHAPE_C * np.arctan(self._SHAPE_B * sigma))
        peak_end = math.sin(self._SHAPE_C * math.atan(self._SHAPE_B * self._SIGMA_PEAK_END))
        frac = np.clip((sigma - self._SIGMA_PEAK_END) / self._SIGMA_SLIDE_RANGE, 0.0, 1.0)
        falling = peak_end + (p.sliding_ratio - peak_end) * frac
        return np.where(sigma <= self._SIGMA_PEAK_END, rising, falling)

    def forces(self, kappa, alpha, Fz, mu):
        p = self.params
        kappa, alpha, Fz, mu = as_wheel_arrays(kappa, alpha, Fz, mu)
        active = (Fz > 0.0) & (mu > 0.0)
        Fz_a = np.where(active, Fz, 1.0)
        dfz = (Fz_a - p.Fz0) / p.Fz0
        mu_eff = np.where(active, mu, 0.0) * load_sensitive(1.0, Fz_a, p.Fz0, p.friction_load_sensitivity)
        tan_ap = math.tan(p.alpha_peak) * np.maximum(1.0 + p.peak_load_sensitivity * dfz, 0.2)

        sx = kappa / p.kappa_peak
        sy = np.tan(alpha) / tan_ap
        sigma = np.hypot(sx, sy)
        F = mu_eff * Fz_a * self._envelope(sigma)
        safe = sigma > 1e-9
        inv = 1.0 / np.where(safe, sigma, 1.0)
        Fx = np.where(safe & active, F * sx * inv, 0.0)
        Fy = np.where(safe & active, F * sy * inv, 0.0)
        return Fx, Fy

    def slip_stiffness(self, Fz, mu):
        p = self.params
        Fz = np.maximum(np.asarray(Fz, dtype=float), 0.0)
        mu = np.asarray(mu, dtype=float)
        mu_eff = mu * load_sensitive(1.0, np.maximum(Fz, 1.0), p.Fz0, p.friction_load_sensitivity)
        return mu_eff * Fz * self._SHAPE_C * self._SHAPE_B / p.kappa_peak

    # Scalar convenience wrapper kept for existing callers/tests.
    def compute_forces(self, kappa: float, alpha: float, Fz: float, mu: float) -> tuple[float, float]:
        Fx, Fy = self.forces(kappa, alpha, Fz, mu)
        return float(Fx), float(Fy)


# Backwards-compatibility aliases for existing tests
CombinedSlipTireModel = SimplifiedCombinedSlipTyre
CombinedSlipPacejkaParams = SimplifiedCombinedSlipParams

__all__ = (
    "SimplifiedCombinedSlipTyre",
    "SimplifiedCombinedSlipParams",
    "CombinedSlipTireModel",
    "CombinedSlipPacejkaParams",
)
