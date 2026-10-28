"""Pacejka Magic Formula lateral tyre model adapter with combined slip coupling.

Adapts the repository's existing Pacejka MF implementation (from YAML coefficients)
into the unified :class:`TyreModel` protocol.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any
import numpy as np

from .framework import TyreModel, TyreProvenance, as_wheel_arrays, load_sensitive
from .magic_formula_lateral import MagicFormulaLateral


@dataclass(frozen=True)
class PacejkaTyreParams:
    """Configuration parameters for Pacejka lateral tyre with combined slip."""
    yaml_name: str = "FRONT_TIRES_EXAMPLE"
    c_kappa: float = 18.0                # Longitudinal slip stiffness per unit load [-]
    camber: float = 0.0                  # Camber angle [rad]
    pressure: float = 250000.0           # Tyre inflation pressure [Pa]
    nominal_vx: float = 20.0             # Reference speed for MF velocity scaling [m/s]
    Fz0: float = 4668.0                  # Reference vertical load [N]
    symmetric: bool = True               # Eliminate ply-steer / conicity offset for symmetric vehicle dynamics


class PacejkaTyre:
    """Pacejka Magic Formula tyre with combined slip adherence to TyreModel protocol."""

    def __init__(
        self,
        params: PacejkaTyreParams | None = None,
        provenance: TyreProvenance | None = None,
    ):
        self.params = params or PacejkaTyreParams()
        data_path = Path(__file__).resolve().parent / "tire_data" / "yamls"
        self._mf = MagicFormulaLateral(
            filename=[self.params.yaml_name],
            data_path=str(data_path),
        )
        self.provenance = provenance or TyreProvenance(
            formulation=f"Pacejka Magic Formula ({self.params.yaml_name})",
            version="6.2-lateral + friction-circle combined slip",
            source="Synthetic/illustrative MF 6.2 YAML coefficients from repository dataset.",
            data_class="synthetic",
            reference_load=self.params.Fz0,
            valid_kappa=(-1.0, 1.0),
            valid_alpha=(-0.50, 0.50),
            limitations=(
                "Pure lateral model with friction-circle combined longitudinal coupling",
                "Synthetic parameters; not validated against physical tyre test bench data",
            ),
        )

    def forces(
        self,
        kappa: np.ndarray | float,
        alpha: np.ndarray | float,
        Fz: np.ndarray | float,
        mu: np.ndarray | float,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Compute combined longitudinal and lateral tyre forces."""
        kappa_arr, alpha_arr, Fz_arr, mu_arr = as_wheel_arrays(kappa, alpha, Fz, mu)
        orig_shape = kappa_arr.shape

        k_flat = kappa_arr.ravel()
        a_flat = alpha_arr.ravel()
        fz_flat = Fz_arr.ravel()
        mu_flat = mu_arr.ravel()

        fx_flat = np.zeros_like(k_flat)
        fy_flat = np.zeros_like(a_flat)

        for i in range(len(k_flat)):
            k = float(k_flat[i])
            a = float(a_flat[i])
            fz = float(fz_flat[i])
            m = float(mu_flat[i])

            if fz <= 0.0 or m <= 0.0:
                continue

            # Evaluate lateral Magic Formula
            self._mf.online_params(
                Fz=fz,
                p=self.params.pressure,
                gamma=self.params.camber,
                kappa=k,
                alpha=a,
                Vcx=self.params.nominal_vx,
                tire_select=self.params.yaml_name,
            )
            fy_mf, _ = self._mf.calculateFy()

            if self.params.symmetric and abs(self.params.camber) < 1e-6:
                # Subtract zero-slip conicity / ply-steer offset (SVy / SHy)
                self._mf.online_params(
                    Fz=fz,
                    p=self.params.pressure,
                    gamma=0.0,
                    kappa=0.0,
                    alpha=0.0,
                    Vcx=self.params.nominal_vx,
                    tire_select=self.params.yaml_name,
                )
                fy_zero, _ = self._mf.calculateFy()
                # Pacejka MF6.2 formulation uses SAE coordinates (alpha > 0 -> Fy < 0).
                # Convert to ISO 8855 coordinates (alpha > 0 -> Fy > 0) matching vehicle models.
                fy_raw = -(float(fy_mf) - float(fy_zero)) * m
            else:
                fy_raw = -float(fy_mf) * m

            # Longitudinal force model with combined slip coupling
            # Pure longitudinal force via slip stiffness
            fx_pure = float(np.sign(k)) * min(abs(self.params.c_kappa * fz * k), m * fz)

            # Combined slip coupling via Kamm friction ellipse
            fy_limit = m * fz
            fy_norm = min(abs(fy_raw) / (fy_limit + 1e-9), 1.0)
            kamm_factor = float(np.sqrt(max(0.0, 1.0 - fy_norm**2)))

            fx_val = fx_pure * kamm_factor
            fy_val = np.clip(fy_raw, -fy_limit, fy_limit)

            # Strict verification of friction circle limit
            f_total = float(np.hypot(fx_val, fy_val))
            if f_total > fy_limit and f_total > 1e-12:
                scale = fy_limit / f_total
                fx_val *= scale
                fy_val *= scale

            fx_flat[i] = fx_val
            fy_flat[i] = fy_val

        return fx_flat.reshape(orig_shape), fy_flat.reshape(orig_shape)

    def slip_stiffness(
        self,
        Fz: np.ndarray | float,
        mu: np.ndarray | float,
    ) -> np.ndarray:
        """Return dFx/dkappa at kappa = 0 [N]."""
        fz_arr = np.maximum(np.asarray(Fz, dtype=float), 0.0)
        return self.params.c_kappa * fz_arr


__all__ = ("PacejkaTyre", "PacejkaTyreParams")
