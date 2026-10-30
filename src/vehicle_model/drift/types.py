"""Domain types of the drift package.

Arrays inside the frozen dataclasses are copied and made read-only on construction, so a
result cannot be changed after it was reported.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

# Validity classes of an equilibrium
VALID = "valid"                          # all checks passed, tyres inside their declared validity
VALID_EXTRAPOLATED = "valid_extrapolated"  # all checks passed, but a tyre works outside its declared validity
INFEASIBLE = "infeasible"                # at least one check failed; see ``reasons``

# Reasons for infeasibility (FailureReason)
SOLVER_NOT_CONVERGED = "solver_not_converged"
RESIDUAL_TOO_LARGE = "residual_too_large"
LOAD_TRANSFER_NOT_CONVERGED = "load_transfer_not_converged"
WHEEL_LIFT = "wheel_lift"
TIP_OVER = "tip_over"
ACTUATOR_LIMIT = "actuator_limit"
TRIVIAL = "trivial_not_a_drift"
WRONG_ATTITUDE = "yaw_rate_not_opposite_to_sideslip"
NEGATIVE_WHEEL_SPEED = "negative_wheel_speed"
REAR_NOT_SATURATED = "rear_axle_below_lateral_peak"     # an equilibrium, but cornering, not a drift


def _ro(x) -> np.ndarray:
    a = np.array(x, dtype=float, copy=True)
    a.setflags(write=False)
    return a


@dataclass(frozen=True)
class DriftReference:
    """Requested drift: speed and signed sideslip; the radius follows from the equilibrium (9x9 problem)."""

    speed_mps: float
    beta_rad: float                  # < 0: left drift (yaw rate > 0), > 0: right drift
    road_mu: float                   # friction of the uniform road the equilibrium is computed for

    def __post_init__(self):
        if not self.speed_mps > 0.5:
            raise ValueError("drift speed must exceed 0.5 m/s")
        if not 0.0 < self.road_mu <= 2.0:
            raise ValueError("road_mu must lie in (0, 2]")
        if abs(self.beta_rad) >= math.pi / 2:
            raise ValueError("|beta| must be below 90 deg")

    @property
    def direction(self) -> str:
        return "left" if self.beta_rad <= 0 else "right"

    def mirrored(self) -> "DriftReference":
        return DriftReference(self.speed_mps, -self.beta_rad, self.road_mu)

    def to_dict(self) -> dict:
        return {"speed_mps": self.speed_mps, "beta_deg": math.degrees(self.beta_rad), "road_mu": self.road_mu,
                "direction": self.direction}


@dataclass(frozen=True)
class DriftEquilibrium:
    reference: DriftReference
    vx: float
    vy: float
    yaw_rate: float
    omega: np.ndarray                # (4,) wheel speeds [rad/s]
    steer_front: float               # [rad]
    wheel_drive_torque: np.ndarray   # (4,) [N m]
    wheel_brake_torque: np.ndarray   # (4,) [N m]
    residual_norm: float             # max over groups of |residual| / tolerance (<= 1: within tolerance)
    physical_feasible: bool
    validity: str                    # VALID | VALID_EXTRAPOLATED | INFEASIBLE
    reasons: tuple = ()
    diagnostics: dict = field(default_factory=dict)

    def __post_init__(self):
        for k in ("omega", "wheel_drive_torque", "wheel_brake_torque"):
            object.__setattr__(self, k, _ro(getattr(self, k)))

    @property
    def speed(self) -> float:
        return math.hypot(self.vx, self.vy)

    @property
    def beta(self) -> float:
        return math.atan2(self.vy, self.vx)

    @property
    def radius(self) -> float:
        """Path radius ``v / |r|`` (steady state only) [m]."""
        return self.speed / abs(self.yaw_rate) if self.yaw_rate else math.inf

    @property
    def direction(self) -> str:
        return "left" if self.yaw_rate > 0 else "right"

    def body_state(self) -> np.ndarray:
        return np.r_[self.vx, self.vy, self.yaw_rate, self.omega]

    def to_dict(self) -> dict:
        d = self.diagnostics
        return {"reference": self.reference.to_dict(), "validity": self.validity, "reasons": list(self.reasons),
                "physical_feasible": self.physical_feasible, "residual_norm": self.residual_norm,
                "vx": self.vx, "vy": self.vy, "beta_deg": math.degrees(self.beta), "yaw_rate_degps": math.degrees(self.yaw_rate),
                "radius_m": self.radius, "direction": self.direction, "steer_front_deg": math.degrees(self.steer_front),
                "omega": self.omega.tolist(), "wheel_drive_torque": self.wheel_drive_torque.tolist(),
                "wheel_brake_torque": self.wheel_brake_torque.tolist(), **{k: v for k, v in d.items()}}


@dataclass(frozen=True)
class DriftControlDiagnostics:
    mode: str
    beta_ref: float
    beta_actual: float
    yaw_ref: float
    yaw_actual: float
    speed_ref: float
    speed_actual: float
    equilibrium_id: str | None
    actuator_saturation: bool
    status: str


__all__ = ("ACTUATOR_LIMIT", "INFEASIBLE", "LOAD_TRANSFER_NOT_CONVERGED", "NEGATIVE_WHEEL_SPEED", "REAR_NOT_SATURATED",
           "RESIDUAL_TOO_LARGE",
           "SOLVER_NOT_CONVERGED", "TIP_OVER", "TRIVIAL", "VALID", "VALID_EXTRAPOLATED", "WHEEL_LIFT", "WRONG_ATTITUDE",
           "DriftControlDiagnostics", "DriftEquilibrium", "DriftReference")
