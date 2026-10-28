"""Road surface friction coefficient (mu) estimator.

Fulfills Plan.md §11 (Milestone M6):
Real-time estimation of road friction coefficient using Recursive Least Squares (RLS)
slip-slope estimation at low-to-medium slip, combined with peak tyre force utilization
tracking during braking (ABS) and acceleration (TCS) transients.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Any
import numpy as np

from vehicle_model.core.types import FL, FR, RL, RR, SevenDOFParams, WheelTelemetry


@dataclass
class FrictionEstimatorConfig:
    """Configuration for recursive road friction estimator."""

    forgetting_factor: float = 0.985      # RLS exponential forgetting factor lambda in [0.95, 0.99]
    min_excitation_slip: float = 0.015    # Min |kappa| required to perform RLS update
    max_excitation_slip: float = 0.14     # Max |kappa| beyond which tyre saturates non-linearly
    min_wheel_speed: float = 2.0          # Min speed [m/s] to prevent division by near-zero
    initial_mu: float = 1.0               # Initial road friction assumption
    mu_min: float = 0.15                  # Physical lower bound (smooth ice)
    mu_max: float = 1.25                  # Physical upper bound (racing slick/dry asphalt)
    nominal_dry_slope: float = 30.0       # Normalized slip slope d(Fx/Fz)/d(kappa) on dry asphalt

    @classmethod
    def from_yaml(cls, path: str | Path | None = None) -> FrictionEstimatorConfig:
        """Load friction estimator parameters from centralized YAML."""
        import yaml
        if path is None:
            from vehicle_model.config.loader import get_configs_root
            p = get_configs_root() / "estimators" / "friction.yaml"
        else:
            p = Path(path)

        with open(p, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}

        algo = data.get("algorithm", {})
        ref = data.get("slope_reference", {})

        return cls(
            forgetting_factor=float(algo.get("forgetting_factor", 0.985)),
            min_excitation_slip=float(algo.get("min_excitation_slip", 0.015)),
            max_excitation_slip=float(algo.get("max_excitation_slip", 0.14)),
            min_wheel_speed=float(algo.get("min_wheel_speed", 2.0)),
            initial_mu=float(algo.get("initial_mu", 1.0)),
            mu_min=float(algo.get("mu_min", 0.15)),
            mu_max=float(algo.get("mu_max", 1.25)),
            nominal_dry_slope=float(ref.get("nominal_dry_slope", 30.0)),
        )


@dataclass
class FrictionEstimate:
    """Road friction estimate container."""

    mu_overall: float              # Fused vehicle-wide friction estimate
    mu_wheels: np.ndarray          # Corner-individual friction estimates [FL, FR, RL, RR]
    mu_left: float                 # Average left-side friction
    mu_right: float                # Average right-side friction
    is_split_mu: bool              # Flag if significant left-right asymmetry is detected
    confidence: float              # Estimation confidence in [0.0, 1.0] based on excitation


class RoadFrictionEstimator:
    """Real-time Recursive Least Squares (RLS) road friction estimator."""

    def __init__(
        self,
        params: SevenDOFParams,
        config: FrictionEstimatorConfig | None = None,
    ):
        self.params = params
        self.config = config or FrictionEstimatorConfig()

        # RLS state per wheel: slope estimate and covariance P
        self.slope_est = np.full(4, self.config.nominal_dry_slope * self.config.initial_mu, dtype=float)
        self.P_cov = np.full(4, 100.0, dtype=float)  # Initial parameter variance

        # Filtered mu estimates
        self.mu_filtered = np.full(4, self.config.initial_mu, dtype=float)
        self.confidence_filtered = 0.50

    def reset(self, initial_mu: float | None = None) -> None:
        """Reset estimator internal states."""
        mu_init = initial_mu or self.config.initial_mu
        self.slope_est = np.full(4, self.config.nominal_dry_slope * mu_init, dtype=float)
        self.P_cov = np.full(4, 100.0, dtype=float)
        self.mu_filtered = np.full(4, mu_init, dtype=float)
        self.confidence_filtered = 0.50

    def update(
        self,
        telemetry: WheelTelemetry,
        vx: float,
    ) -> FrictionEstimate:
        """Update road friction estimates using observed wheel slip and forces.

        Parameters:
            telemetry: Current WheelTelemetry containing kappa, Fx, Fy, and Fz
            vx: Vehicle longitudinal speed [m/s]

        Returns:
            FrictionEstimate with overall and corner-individual friction levels.
        """
        lam = self.config.forgetting_factor
        k_nom = self.config.nominal_dry_slope
        mu_min = self.config.mu_min
        mu_max = self.config.mu_max

        # Check vehicle operational speed
        if abs(vx) < self.config.min_wheel_speed:
            # Low speed: retain current filtered estimate with lower confidence
            self.confidence_filtered = max(0.10, self.confidence_filtered * 0.99)
            return self._build_estimate()

        has_excitation = False

        # Safely resolve Fx and Fy from telemetry
        fx_arr = getattr(telemetry, "Fx_wheel", getattr(telemetry, "Fx", None))
        fy_arr = getattr(telemetry, "Fy_wheel", getattr(telemetry, "Fy", None))
        if fx_arr is None:
            fx_arr = np.zeros(4)
        if fy_arr is None:
            fy_arr = np.zeros(4)

        for i in range(4):
            kappa_i = float(telemetry.kappa[i])
            Fz_i = max(100.0, float(telemetry.Fz[i]))
            Fx_i = float(fx_arr[i])
            Fy_i = float(fy_arr[i])

            abs_kappa = abs(kappa_i)
            utilization = math.sqrt(Fx_i ** 2 + Fy_i ** 2) / Fz_i

            # ── Branch A: Low-to-Medium Slip (RLS Linear Slope Update) ──
            if self.config.min_excitation_slip <= abs_kappa <= self.config.max_excitation_slip:
                has_excitation = True
                # Normalized force ratio
                y_i = abs(Fx_i) / Fz_i
                phi_i = abs_kappa  # Regressor

                # RLS update equation
                P_prev = self.P_cov[i]
                gain = (P_prev * phi_i) / (lam + phi_i * P_prev * phi_i)
                err = y_i - phi_i * self.slope_est[i]

                self.slope_est[i] += gain * err
                self.P_cov[i] = (1.0 / lam) * (1.0 - gain * phi_i) * P_prev

                # Convert slope to friction ratio
                raw_mu = self.slope_est[i] / k_nom
                clamped_mu = max(mu_min, min(mu_max, raw_mu))
                # Low-pass filter
                self.mu_filtered[i] = 0.90 * self.mu_filtered[i] + 0.10 * clamped_mu

            # ── Branch B: High-Slip Saturation (Peak Force Tracking) ──
            elif abs_kappa > self.config.max_excitation_slip:
                has_excitation = True
                # Direct force utilization at wheel limit
                peak_mu = max(mu_min, min(mu_max, utilization))
                # Quick tracking of lower friction limits (e.g. slipping on ice)
                if peak_mu < self.mu_filtered[i]:
                    self.mu_filtered[i] = 0.70 * self.mu_filtered[i] + 0.30 * peak_mu
                else:
                    self.mu_filtered[i] = 0.92 * self.mu_filtered[i] + 0.08 * peak_mu

        # Update confidence
        if has_excitation:
            self.confidence_filtered = min(1.0, self.confidence_filtered + 0.05)
        else:
            self.confidence_filtered = max(0.20, self.confidence_filtered * 0.995)

        return self._build_estimate()

    def _build_estimate(self) -> FrictionEstimate:
        mu_left = float((self.mu_filtered[FL] + self.mu_filtered[RL]) * 0.5)
        mu_right = float((self.mu_filtered[FR] + self.mu_filtered[RR]) * 0.5)
        mu_overall = float(np.mean(self.mu_filtered))
        is_split = abs(mu_left - mu_right) >= 0.25

        return FrictionEstimate(
            mu_overall=mu_overall,
            mu_wheels=self.mu_filtered.copy(),
            mu_left=mu_left,
            mu_right=mu_right,
            is_split_mu=is_split,
            confidence=self.confidence_filtered,
        )


__all__ = ("FrictionEstimatorConfig", "FrictionEstimate", "RoadFrictionEstimator")
