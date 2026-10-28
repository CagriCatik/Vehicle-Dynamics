"""Nonlinear (beta, beta_dot) Phase-Plane Stability Boundary & Envelope Controller.

Implements vehicle lateral stability boundary modeling in the (sideslip, sideslip-rate)
phase plane based on nonlinear tire friction saturation limits and saddle-node bifurcations.

The controller arbitrates between:
1. Inner-Envelope Agility Mode: Unhindered path tracking and 4WS yaw agility.
2. Boundary-Approach Transition Mode: Continuous sliding-mode stabilizing yaw moment.
3. Critical-Saturation Recovery Mode: Full emergency yaw containment, active counter-steer,
   and dynamic speed reduction to expand the stable phase-plane envelope.

Conforms to modern vehicle dynamics stability principles (Inagaki, Ono et al.).
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from pathlib import Path
from typing import Optional, Tuple

import numpy as np
import yaml


class PhasePlaneState(str, Enum):
    """Phase-plane stability region classification."""

    LINEAR_STABLE = "linear_stable"          # S < threshold_approach (typically < 0.65)
    APPROACHING_LIMIT = "approaching_limit"  # 0.65 <= S < 1.0 (soft boundary intervention)
    CRITICAL_SATURATION = "critical_saturation" # S >= 1.0 (hard envelope containment)


@dataclass
class PhasePlaneConfig:
    """Configuration parameters for phase-plane boundary estimation and control."""

    # Dynamic boundary parameters
    beta_sat_dry_deg: float = 9.0        # Peak sideslip saturation on dry road (mu=1.0) [deg]
    c_phase_time_s: float = 0.20         # Phase-lead time constant C_phase [s]
    k_us_factor: float = 0.0015          # Speed-dependent envelope expansion factor [s^2/m^2]
    min_speed_mps: float = 2.5           # Minimum speed to evaluate sideslip dynamics [m/s]

    # Boundary thresholds
    s_approach: float = 0.65             # Stability index threshold to begin soft intervention
    s_critical: float = 1.00             # Stability index threshold for hard emergency recovery

    # Restoring control gains
    k_mz_envelope: float = 3800.0        # Maximum stabilizing yaw moment from envelope control [N*m]
    k_steer_counter: float = 0.45        # Active counter-steer proportional gain [rad/rad]
    k_steer_derivative: float = 0.08     # Active counter-steer rate damping gain [rad/(rad/s)]
    max_counter_steer_rad: float = 0.20  # Maximum active counter-steer intervention [rad] (~11.5 deg)
    k_speed_reduction_decel: float = 2.5 # Deceleration demand gain when S > 1.0 [m/s^2 per S_excess]
    boundary_layer_epsilon: float = 0.15 # Boundary layer thickness for smooth sliding-mode transition

    @classmethod
    def from_yaml(cls, path: str | Path) -> "PhasePlaneConfig":
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        cfg = data.get("phase_plane", data)
        return cls(
            beta_sat_dry_deg=float(cfg.get("beta_sat_dry_deg", 9.0)),
            c_phase_time_s=float(cfg.get("c_phase_time_s", 0.20)),
            k_us_factor=float(cfg.get("k_us_factor", 0.0015)),
            min_speed_mps=float(cfg.get("min_speed_mps", 2.5)),
            s_approach=float(cfg.get("s_approach", 0.65)),
            s_critical=float(cfg.get("s_critical", 1.00)),
            k_mz_envelope=float(cfg.get("k_mz_envelope", 3800.0)),
            k_steer_counter=float(cfg.get("k_steer_counter", 0.45)),
            k_steer_derivative=float(cfg.get("k_steer_derivative", 0.08)),
            max_counter_steer_rad=float(cfg.get("max_counter_steer_rad", 0.20)),
            k_speed_reduction_decel=float(cfg.get("k_speed_reduction_decel", 2.5)),
            boundary_layer_epsilon=float(cfg.get("boundary_layer_epsilon", 0.15)),
        )


@dataclass
class PhasePlaneEvaluation:
    """Output evaluation from the phase-plane stability monitor."""

    beta_rad: float
    beta_dot_radps: float
    beta_max_rad: float
    beta_dot_max_radps: float
    stability_index: float               # S_phase: normalized composite distance to boundary
    phase_state: PhasePlaneState
    stabilizing_yaw_moment_nm: float     # Corrective direct yaw moment
    active_counter_steer_rad: float      # Corrective steer angle adjustment
    emergency_decel_demand_mps2: float   # Speed scrub deceleration request


class PhasePlaneEnvelopeController:
    """Nonlinear Phase-Plane (beta, beta_dot) Stability Supervisor and Envelope Controller."""

    def __init__(self, config: Optional[PhasePlaneConfig] = None):
        self.config = config or PhasePlaneConfig()

    def compute_boundary_limits(
        self,
        vx: float,
        mu: float = 1.0,
        ax: float = 0.0,
    ) -> Tuple[float, float]:
        """Compute state-dependent stability boundary limits (beta_max, beta_dot_max).

        Args:
            vx: Vehicle longitudinal speed [m/s].
            mu: Estimated or road friction coefficient [-].
            ax: Longitudinal acceleration / braking scrub [m/s^2].

        Returns:
            Tuple of (beta_max [rad], beta_dot_max [rad/s]).
        """
        mu_safe = max(0.05, min(1.2, float(mu)))
        v_safe = max(self.config.min_speed_mps, abs(float(vx)))
        g = 9.80665

        # Combined-slip friction coupling:
        # Severe braking or lift-off deceleration shrinks available lateral traction circle
        ax_ratio = min(0.95, abs(ax) / (mu_safe * g))
        mu_lat = mu_safe * math.sqrt(max(0.05, 1.0 - (ax_ratio ** 2)))

        # Peak sideslip angle scaled by available lateral grip
        base_beta_max_rad = math.radians(self.config.beta_sat_dry_deg)
        beta_max = base_beta_max_rad * mu_lat
        beta_max = max(math.radians(1.8), min(math.radians(12.0), beta_max))

        # Maximum achievable sideslip rate based on available lateral friction
        beta_dot_max = (mu_lat * g) / v_safe
        beta_dot_max = max(0.15, min(3.5, beta_dot_max))

        return beta_max, beta_dot_max

    def evaluate(
        self,
        vx: float,
        vy: float,
        r: float,
        ay: float,
        mu: float = 1.0,
        ax: float = 0.0,
    ) -> PhasePlaneEvaluation:
        """Evaluate phase-plane state and compute stabilizing intervention.

        Args:
            vx: Longitudinal velocity [m/s].
            vy: Lateral velocity [m/s].
            r: Chassis yaw rate [rad/s].
            ay: Lateral acceleration in body frame [m/s^2].
            mu: Estimated road friction coefficient [-].
            ax: Longitudinal acceleration in body frame [m/s^2].

        Returns:
            PhasePlaneEvaluation containing stability metrics and control interventions.
        """
        v_lon = max(self.config.min_speed_mps, abs(float(vx)))

        # Sideslip angle and kinematic sideslip rate: beta_dot = (ay / vx) - r
        beta = math.atan2(vy, v_lon)
        beta_dot = (ay / v_lon) - r

        beta_max, beta_dot_max = self.compute_boundary_limits(vx, mu, ax)

        # Composite phase-plane stability index S_phase:
        # Normalized distance: |beta/beta_max| + C_phase * |beta_dot/beta_dot_max|
        norm_beta = abs(beta) / beta_max
        norm_beta_dot = (self.config.c_phase_time_s * abs(beta_dot)) / beta_max
        stability_index = norm_beta + norm_beta_dot

        # Phase-plane sliding-mode surface variable:
        # sigma = beta + C_phase * beta_dot
        sigma = beta + self.config.c_phase_time_s * beta_dot

        # Determine stability state and calculate control intervention
        if stability_index < self.config.s_approach:
            phase_state = PhasePlaneState.LINEAR_STABLE
            M_z_stab = 0.0
            delta_cs = 0.0
            decel_demand = 0.0
        elif stability_index < self.config.s_critical:
            phase_state = PhasePlaneState.APPROACHING_LIMIT
            # Soft continuous boundary intervention: proportional to excess beyond s_approach
            excess = (stability_index - self.config.s_approach) / (self.config.s_critical - self.config.s_approach)
            reaching_factor = math.tanh(sigma / self.config.boundary_layer_epsilon)
            M_z_stab = self.config.k_mz_envelope * reaching_factor * (excess ** 1.5)
            # Gentle active steering stabilization (counter-steer into skid)
            steer_interv = self.config.k_steer_counter * beta + self.config.k_steer_derivative * beta_dot
            delta_cs = float(np.clip(steer_interv * excess, -self.config.max_counter_steer_rad, self.config.max_counter_steer_rad))
            decel_demand = 0.0
        else:
            phase_state = PhasePlaneState.CRITICAL_SATURATION
            # Hard Lyapunov / sliding-mode recovery intervention
            reaching_factor = math.tanh(sigma / (0.5 * self.config.boundary_layer_epsilon))
            M_z_stab = self.config.k_mz_envelope * reaching_factor
            # Full active counter-steer intervention
            steer_interv = self.config.k_steer_counter * beta + self.config.k_steer_derivative * beta_dot
            delta_cs = float(np.clip(steer_interv, -self.config.max_counter_steer_rad, self.config.max_counter_steer_rad))
            # Speed reduction to scrub kinetic energy and expand stable phase-plane envelope
            s_excess = stability_index - self.config.s_critical
            decel_demand = min(0.6 * 9.81, self.config.k_speed_reduction_decel * s_excess)

        return PhasePlaneEvaluation(
            beta_rad=beta,
            beta_dot_radps=beta_dot,
            beta_max_rad=beta_max,
            beta_dot_max_radps=beta_dot_max,
            stability_index=stability_index,
            phase_state=phase_state,
            stabilizing_yaw_moment_nm=M_z_stab,
            active_counter_steer_rad=delta_cs,
            emergency_decel_demand_mps2=decel_demand,
        )
