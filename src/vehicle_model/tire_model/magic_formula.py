"""Parametrised Magic Formula tyre (subset of MF 5.2, Pacejka 2012, sec. 4.3.2).

Documented formulation, vectorised over wheels. Implemented effects:

* pure longitudinal and lateral slip with load sensitivity (``dfz``),
* road-friction scaling on the peak only (``lambda_mu = mu``): the slip
  stiffnesses do not depend on ``mu``,
* combined slip with the MF 5.2 cosine weighting functions ``G_xa``, ``G_yk``
  (two documented deviations, see "Combined" below),
* self-aligning moment from the pneumatic trail.

Not implemented (and therefore not claimed): camber, inflation pressure,
turn slip, conicity/ply steer (all shifts ``S_H``, ``S_V`` are zero, the tyre is
symmetric), rolling resistance moment, overturning moment, the ``s * Fx``
lever-arm term of ``Mz`` and transient behaviour (see ``relaxation.py``).

Sign convention (:mod:`.framework`): ``kappa > 0 -> Fx > 0``,
``alpha > 0 -> Fy > 0``; the aligning moment ``Mz`` acts about +z (up) and
is negative for ``alpha > 0`` below the trail sign change, i.e. it turns the
wheel towards its velocity.

Equations (``x* = tan(alpha)``, ``dfz = (Fz - Fz0)/Fz0``):

    Longitudinal    mu_x = (PDX1 + PDX2 dfz) mu,   D_x = mu_x Fz,   C_x = PCX1
                    K_xk = Fz (PKX1 + PKX2 dfz) exp(PKX3 dfz),      B_x = K_xk/(C_x D_x)
                    E_x  = min(PEX1 + PEX2 dfz + PEX3 dfz^2, 1)
                    Fx0  = D_x sin(C_x atan(B_x k - E_x (B_x k - atan(B_x k))))
    Lateral         mu_y = (PDY1 + PDY2 dfz) mu,   D_y = mu_y Fz,   C_y = PCY1
                    K_ya = PKY1 Fz0 sin(2 atan(Fz/(PKY2 Fz0))),      B_y = K_ya/(C_y D_y)
                    E_y  = min(PEY1 + PEY2 dfz, 1)
                    Fy0  = D_y sin(C_y atan(B_y x* - E_y (B_y x* - atan(B_y x*))))
    Combined        k_n = k/mu,  x_n = x*/mu      (friction-normalised slips, deviation 1)
                    G_xa = cos(RCX1 atan(RBX1 cos(atan(RBX2 k_n)) x_n)),  Fx' = G_xa Fx0
                    G_yk = cos(RCY1 atan(RBY1 cos(atan(RBY2 x_n)) k_n)),  Fy' = G_yk Fy0
                    e    = sqrt((Fx'/D_x)^2 + (Fy'/D_y)^2)
                    (Fx, Fy) = (Fx', Fy') / max(1, e)                    (ellipse limiter, deviation 2)
    Aligning        a_eq = sqrt(x*^2 + (K_xk/K_ya)^2 k^2) sgn(x*)
                    t    = D_t cos(C_t atan(B_t a_eq - E_t (B_t a_eq - atan(B_t a_eq)))) cos(alpha)
                    D_t  = Fz (R0/Fz0)(QDZ1 + QDZ2 dfz),  B_t = QBZ1 + QBZ2 dfz,  C_t = QCZ1,
                    E_t  = min(QEZ1 + QEZ2 dfz, 1)
                    Mz   = -t Fy

Deviation 1: MF 5.2 applies the weighting functions to the raw slips. Since
the peak slips scale with ``1/mu`` (``B ~ 1/D``), the coupling would then act
at a different fraction of the peak on every road, and the resultant would
exceed the friction ellipse by up to ~35 % at ``mu = 0.3``. Normalising by
``mu`` makes the combined-slip map self-similar in ``mu``, consistent with brush
theory, where the coupling depends on ``C s/(mu Fz)``.

Deviation 2: the empirical weighting still overshoots the friction ellipse by
up to ~12 % where both slips are near their peaks. The limiter scales the force
vector radially back onto the ellipse. It is continuous, leaves pure slip
untouched (``G = 1`` there) and is only active in that combined-slip region.
``RCX1, RCY1 <= 1`` keeps ``G >= 0``, so forces never reverse direction.
"""

from __future__ import annotations

from dataclasses import dataclass, fields

import numpy as np

from .framework import TyreProvenance, as_wheel_arrays

_SYNTHETIC = (
    "Synthetic MF 5.2-subset coefficients chosen to give textbook passenger-car "
    "characteristics (C_alpha/Fz ~ 18/rad, C_kappa/Fz ~ 20, peak mu ~ 1 at Fz0). "
    "Not fitted to, and not representative of, any real tyre."
)


@dataclass(frozen=True)
class MagicFormulaParams:
    """MF 5.2-subset coefficients (names as in Pacejka 2012, ISO sign convention adapted)."""

    Fz0: float = 4000.0   # [N] nominal load
    R0: float = 0.31      # [m] unloaded radius (aligning-moment scale only)
    # longitudinal
    PCX1: float = 1.65
    PDX1: float = 1.00
    PDX2: float = -0.08
    PEX1: float = -0.20
    PEX2: float = 0.0
    PEX3: float = 0.0
    PKX1: float = 20.0
    PKX2: float = 0.0
    PKX3: float = -0.20
    # lateral
    PCY1: float = 1.45
    PDY1: float = 1.00
    PDY2: float = -0.10
    PEY1: float = -0.50
    PEY2: float = 0.0
    PKY1: float = 21.2
    PKY2: float = 1.8
    # combined slip
    RBX1: float = 12.0
    RBX2: float = 10.0
    RCX1: float = 1.0
    RBY1: float = 10.0
    RBY2: float = 8.0
    RCY1: float = 1.0
    # aligning moment
    QBZ1: float = 8.0
    QBZ2: float = -1.0
    QCZ1: float = 1.2
    QDZ1: float = 0.10
    QDZ2: float = -0.01
    QEZ1: float = -1.5
    QEZ2: float = 0.0

    def __post_init__(self) -> None:
        for name in ("Fz0", "R0", "PCX1", "PDX1", "PKX1", "PCY1", "PDY1", "PKY1", "PKY2", "QCZ1"):
            if getattr(self, name) <= 0.0:
                raise ValueError(f"MagicFormulaParams.{name} must be positive")
        if not (0.0 < self.RCX1 <= 1.0 and 0.0 < self.RCY1 <= 1.0):
            raise ValueError("RCX1 and RCY1 must lie in (0, 1] so that the weighting functions stay >= 0")


class MagicFormulaTyre:
    """Vectorised MF 5.2-subset tyre implementing :class:`~.framework.TyreModel`."""

    def __init__(self, params: MagicFormulaParams | None = None, provenance: TyreProvenance | None = None):
        self.params = params or MagicFormulaParams()
        self.provenance = provenance or TyreProvenance(
            formulation="Magic Formula 5.2 subset (pure + combined slip, aligning moment)",
            version="1.0", source=_SYNTHETIC, data_class="synthetic",
            reference_load=self.params.Fz0, valid_kappa=(-1.0, 1.0), valid_alpha=(-0.35, 0.35),
            limitations=(
                "No camber, pressure, turn slip, conicity or ply steer (symmetric tyre)",
                "Mz without the s*Fx lever-arm term; no rolling-resistance or overturning moment",
                "Combined slip: friction-normalised MF 5.2 weighting plus friction-ellipse limiter",
                "Steady-state model; combine with TyreRelaxation for transient behaviour",
            ),
        )

    # ----------------------------------------------------------------- pieces
    def _dfz(self, Fz: np.ndarray) -> np.ndarray:
        return (Fz - self.params.Fz0) / self.params.Fz0

    def longitudinal_stiffness(self, Fz) -> np.ndarray:
        """``K_xk = dFx/dkappa`` at zero slip [N]; independent of road friction."""
        p = self.params
        Fz = np.maximum(np.asarray(Fz, dtype=float), 0.0)
        dfz = self._dfz(Fz)
        return Fz * (p.PKX1 + p.PKX2 * dfz) * np.exp(p.PKX3 * dfz)

    def cornering_stiffness(self, Fz) -> np.ndarray:
        """``K_ya = dFy/dalpha`` at zero slip [N/rad]; independent of road friction."""
        p = self.params
        Fz = np.maximum(np.asarray(Fz, dtype=float), 0.0)
        return p.PKY1 * p.Fz0 * np.sin(2.0 * np.arctan(Fz / (p.PKY2 * p.Fz0)))

    def peak_friction(self, Fz, mu) -> tuple[np.ndarray, np.ndarray]:
        """Load-dependent peak friction coefficients ``(mu_x, mu_y)`` on a road with friction ``mu``."""
        p = self.params
        dfz = self._dfz(np.asarray(Fz, dtype=float))
        mu = np.asarray(mu, dtype=float)
        return (p.PDX1 + p.PDX2 * dfz) * mu, (p.PDY1 + p.PDY2 * dfz) * mu

    @staticmethod
    def _sine_curve(B, C, D, E, x):
        Bx = B * x
        return D * np.sin(C * np.arctan(Bx - E * (Bx - np.arctan(Bx))))

    def _evaluate(self, kappa, alpha, Fz, mu):
        p = self.params
        kappa, alpha, Fz, mu = as_wheel_arrays(kappa, alpha, Fz, mu)
        active = (Fz > 0.0) & (mu > 0.0)
        Fz_a = np.where(active, Fz, p.Fz0)
        mu_a = np.where(active, mu, 1.0)
        dfz = self._dfz(Fz_a)
        x = np.tan(alpha)

        mu_x, mu_y = self.peak_friction(Fz_a, mu_a)
        D_x, D_y = mu_x * Fz_a, mu_y * Fz_a
        K_x = self.longitudinal_stiffness(Fz_a)
        K_y = self.cornering_stiffness(Fz_a)
        E_x = np.minimum(p.PEX1 + p.PEX2 * dfz + p.PEX3 * dfz**2, 1.0)
        E_y = np.minimum(p.PEY1 + p.PEY2 * dfz, 1.0)
        Fx0 = self._sine_curve(K_x / (p.PCX1 * D_x), p.PCX1, D_x, E_x, kappa)
        Fy0 = self._sine_curve(K_y / (p.PCY1 * D_y), p.PCY1, D_y, E_y, x)

        k_n, x_n = kappa / mu_a, x / mu_a
        G_xa = np.cos(p.RCX1 * np.arctan(p.RBX1 * np.cos(np.arctan(p.RBX2 * k_n)) * x_n))
        G_yk = np.cos(p.RCY1 * np.arctan(p.RBY1 * np.cos(np.arctan(p.RBY2 * x_n)) * k_n))
        Fx, Fy = G_xa * Fx0, G_yk * Fy0
        limiter = 1.0 / np.maximum(1.0, np.hypot(Fx / D_x, Fy / D_y))
        Fx = np.where(active, limiter * Fx, 0.0)
        Fy = np.where(active, limiter * Fy, 0.0)

        a_eq = np.sqrt(x**2 + (K_x / K_y * kappa) ** 2) * np.where(x < 0.0, -1.0, 1.0)
        D_t = Fz_a * (p.R0 / p.Fz0) * (p.QDZ1 + p.QDZ2 * dfz)
        B_t = p.QBZ1 + p.QBZ2 * dfz
        E_t = np.minimum(p.QEZ1 + p.QEZ2 * dfz, 1.0)
        Bt_a = B_t * a_eq
        trail = D_t * np.cos(p.QCZ1 * np.arctan(Bt_a - E_t * (Bt_a - np.arctan(Bt_a)))) * np.cos(alpha)
        Mz = np.where(active, -trail * Fy, 0.0)
        return Fx, Fy, Mz

    # ------------------------------------------------------------- interface
    def forces(self, kappa, alpha, Fz, mu) -> tuple[np.ndarray, np.ndarray]:
        Fx, Fy, _ = self._evaluate(kappa, alpha, Fz, mu)
        return Fx, Fy

    def forces_and_moment(self, kappa, alpha, Fz, mu) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """``(Fx, Fy, Mz)`` in the wheel frame [N, N, N m]."""
        return self._evaluate(kappa, alpha, Fz, mu)

    def aligning_moment(self, kappa, alpha, Fz, mu) -> np.ndarray:
        return self._evaluate(kappa, alpha, Fz, mu)[2]

    def slip_stiffness(self, Fz, mu) -> np.ndarray:
        return self.longitudinal_stiffness(Fz)


def magic_formula_params_from_dict(values: dict) -> MagicFormulaParams:
    known = {f.name for f in fields(MagicFormulaParams)}
    unknown = set(values) - known
    if unknown:
        raise ValueError(f"Unknown Magic Formula coefficients: {sorted(unknown)}")
    return MagicFormulaParams(**{k: float(v) for k, v in values.items()})


__all__ = ("MagicFormulaParams", "MagicFormulaTyre", "magic_formula_params_from_dict")
