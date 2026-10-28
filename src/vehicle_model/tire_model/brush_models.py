"""Linear and Fiala (brush) tyre models."""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .framework import TyreProvenance, as_wheel_arrays, load_sensitive

_SYNTHETIC_PASSENGER = "Synthetic passenger-car values chosen for illustration; not fitted to measurements."


@dataclass(frozen=True)
class LinearTyre:
    """Unsaturated linear tyre: ``Fx = C_kappa * kappa``, ``Fy = C_alpha * alpha``.

    Only valid for small slips. With ``saturate=True`` the force vector is
    clipped to the friction circle, which is a crude limit and not a real
    combined-slip model.
    """

    c_kappa: float = 16.0          # Longitudinal slip stiffness per unit load [-]
    c_alpha: float = 18.0          # Cornering stiffness per unit load [1/rad]
    stiffness_load_sensitivity: float = 0.3
    Fz0: float = 3000.0            # Reference load [N]
    saturate: bool = False
    provenance: TyreProvenance = field(default_factory=lambda: TyreProvenance(
        formulation="Linear", version="1.0", source=_SYNTHETIC_PASSENGER,
        data_class="synthetic", reference_load=3000.0,
        valid_kappa=(-0.03, 0.03), valid_alpha=(-0.04, 0.04),
        limitations=("No saturation unless saturate=True", "No combined-slip coupling"),
    ))

    def _stiffness(self, Fz: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        c_alpha = load_sensitive(self.c_alpha, Fz, self.Fz0, self.stiffness_load_sensitivity)
        return self.c_kappa * Fz, c_alpha * Fz

    def forces(self, kappa, alpha, Fz, mu):
        kappa, alpha, Fz, mu = as_wheel_arrays(kappa, alpha, Fz, mu)
        active = (Fz > 0.0) & (mu > 0.0)
        Fz_a = np.where(active, Fz, 0.0)
        C_k, C_a = self._stiffness(Fz_a)
        Fx = C_k * kappa
        Fy = C_a * alpha
        if self.saturate:
            limit = mu * Fz_a
            norm = np.hypot(Fx, Fy)
            scale = np.where(norm > limit, limit / np.maximum(norm, 1e-12), 1.0)
            Fx, Fy = Fx * scale, Fy * scale
        return np.where(active, Fx, 0.0), np.where(active, Fy, 0.0)

    def slip_stiffness(self, Fz, mu):
        Fz = np.maximum(np.asarray(Fz, dtype=float), 0.0)
        return self.c_kappa * Fz


@dataclass(frozen=True)
class FialaTyre:
    """Combined-slip Fiala / brush tyre with parabolic pressure distribution.

    Uses theoretical slips ``sx = kappa/(1+kappa)``, ``sy = tan(alpha)/(1+kappa)``
    and the isotropic-friction brush solution (Pacejka, *Tyre and Vehicle
    Dynamics*, ch. 3):

        psi = sqrt((C_k sx)^2 + (C_a sy)^2)
        F   = psi - psi^2/(3 mu Fz) + psi^3/(27 mu^2 Fz^2)   for psi < 3 mu Fz
        F   = mu Fz                                          otherwise
        (Fx, Fy) = F * (C_k sx, C_a sy) / psi

    The resultant force never exceeds ``mu_eff * Fz`` and the sliding force
    acts along the slip direction. There is no friction fall-off after the peak.
    """

    c_kappa: float = 16.0
    c_alpha: float = 18.0
    stiffness_load_sensitivity: float = 0.3   # reduces C_alpha/Fz at high load -> understeer
    friction_load_sensitivity: float = 0.10   # reduces mu at high load (cf. MF PDY2)
    Fz0: float = 3000.0
    provenance: TyreProvenance = field(default_factory=lambda: TyreProvenance(
        formulation="Fiala brush (combined slip)", version="1.0", source=_SYNTHETIC_PASSENGER,
        data_class="synthetic", reference_load=3000.0,
        valid_kappa=(-1.0, 1.0), valid_alpha=(-0.35, 0.35),
        limitations=(
            "No friction drop after peak (sliding force = peak force)",
            "Isotropic friction, no camber, no pressure, no relaxation length",
        ),
    ))

    def effective_mu(self, Fz: np.ndarray, mu: np.ndarray) -> np.ndarray:
        return mu * load_sensitive(1.0, Fz, self.Fz0, self.friction_load_sensitivity)

    def forces(self, kappa, alpha, Fz, mu):
        kappa, alpha, Fz, mu = as_wheel_arrays(kappa, alpha, Fz, mu)
        active = (Fz > 0.0) & (mu > 0.0)
        Fz_a = np.where(active, Fz, 1.0)
        mu_eff = self.effective_mu(Fz_a, np.where(active, mu, 1.0))
        C_k = self.c_kappa * Fz_a
        C_a = load_sensitive(self.c_alpha, Fz_a, self.Fz0, self.stiffness_load_sensitivity) * Fz_a

        denom = np.maximum(1.0 + kappa, 1e-3)   # wheel lock: direction stays defined, magnitude saturates
        sx = kappa / denom
        sy = np.tan(alpha) / denom
        gx, gy = C_k * sx, C_a * sy
        psi = np.hypot(gx, gy)
        f_max = mu_eff * Fz_a
        q = psi / (3.0 * f_max)
        F = np.where(q < 1.0, psi - psi * q + psi * q * q / 3.0, f_max)

        safe = psi > 1e-12
        Fx = np.where(safe, F * gx / np.where(safe, psi, 1.0), 0.0)
        Fy = np.where(safe, F * gy / np.where(safe, psi, 1.0), 0.0)
        return np.where(active, Fx, 0.0), np.where(active, Fy, 0.0)

    def slip_stiffness(self, Fz, mu):
        Fz = np.maximum(np.asarray(Fz, dtype=float), 0.0)
        return self.c_kappa * Fz
