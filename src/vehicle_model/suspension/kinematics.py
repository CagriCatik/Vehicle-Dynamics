"""Quasi-static suspension kinematics & compliance (K&C) for planar vehicle models.

The planar 7-DOF model has no roll or pitch degree of freedom. This module
supplies what the suspension does in quasi-static equilibrium:

Roll centres and lateral load transfer
    Unsprung masses ``m_u`` per wheel at height ``h_u`` (= R_eff), sprung mass
    ``m_s`` at ``h_s = (m h - 4 m_u h_u)/m_s``, roll-centre heights ``h_rf``, ``h_rr``,
    roll axis height at the sprung CoG ``h_ra = (m_sf h_rf + m_sr h_rr)/m_s``,
    roll arm ``d = h_s - h_ra``, roll stiffness split ``k = K_f/(K_f + K_r)``.
    Lateral load transfer per axle (Milliken & Milliken 1995, ch. 18):

        w_f dF_f = a_y [ m_s d k     + m_sf h_rf + 2 m_u h_u ]    (elastic + geometric + unsprung)
        w_r dF_r = a_y [ m_s d (1-k) + m_sr h_rr + 2 m_u h_u ]

    The two sum to ``m a_y h`` exactly, so the roll moment balance is unchanged;
    only the front share changes. It enters the unilateral load distribution
    (vehicle_models/normal_loads.py) as an effective ``k_roll_f``.
    With ``h_rf = h_rr = 0`` and ``m_u = 0`` it equals ``k``.

Body attitude and wheel travel (small angles, rigid body, no gravity term)
    roll  phi   = m_s a_y d / (K_f + K_r)                    (> 0: right side down for a_y > 0)
    pitch theta = -m_s a_x h_s / K_theta,  K_theta = 2 c_f l_f^2 + 2 c_r l_r^2   (> 0: nose down)
    wheel compression  z_i = -y_i phi + x_i theta
    The pitch axis is at ground level (no anti-dive/anti-squat geometry).

Steer changes (sign: + = steer to the left, ISO)
    bump steer        toe-in(z) from a map per axle; left wheel  d_delta = -toe,  right wheel  d_delta = +toe
    compliance steer  front  d_delta = -c_cf F_y,f    rear  d_delta = +c_cr F_y,r
                      (c > 0 promotes understeer on both axles; F_y,axle: lateral axle force, wheel frame)
    roll steer        not a separate input: it follows from bump steer and roll,
                      d_delta_axle/d_phi = toe'(0) w/2 (see ``roll_steer_coefficients``).

Limits (not modelled, not claimed): camber change (the tyre models have no camber),
caster, kingpin inclination and scrub radius (no steering torque), the
destabilising gravity term ``m_s g d phi`` (~8 % of the elastic roll moment for the
reference car), CoG shift with roll, anti-dive/anti-squat, suspension dynamics
(see the 14-DOF model), longitudinal compliance.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import yaml

from vehicle_model.core.types import SevenDOFParams


@dataclass(frozen=True)
class ToeMap:
    """Toe-in [rad] over wheel compression [m]; linear interpolation, constant beyond the ends."""

    travel: tuple[float, ...] = (-0.1, 0.1)
    toe_in: tuple[float, ...] = (0.0, 0.0)

    def __post_init__(self) -> None:
        if len(self.travel) != len(self.toe_in) or len(self.travel) < 2:
            raise ValueError("ToeMap needs at least two points of equal length")
        if np.any(np.diff(self.travel) <= 0.0):
            raise ValueError("ToeMap travel must be strictly increasing")

    @classmethod
    def linear(cls, gradient: float, travel_range: float = 0.1) -> ToeMap:
        """Map with constant gradient [rad/m] over +-``travel_range``."""
        return cls((-travel_range, travel_range), (-gradient * travel_range, gradient * travel_range))

    def __call__(self, z) -> np.ndarray:
        return np.interp(np.asarray(z, dtype=float), self.travel, self.toe_in)

    def gradient_at_zero(self) -> float:
        h = 1e-6
        return float((self(h) - self(-h)) / (2 * h))


@dataclass(frozen=True)
class SuspensionKC:
    """Synthetic K&C description of one vehicle (all effects zero by default)."""

    roll_centre_height_front: float = 0.0     # [m]
    roll_centre_height_rear: float = 0.0      # [m]
    bump_steer_front: ToeMap = field(default_factory=ToeMap)
    bump_steer_rear: ToeMap = field(default_factory=ToeMap)
    compliance_steer_front: float = 0.0       # [rad/N], > 0: front steers out of the turn (understeer)
    compliance_steer_rear: float = 0.0        # [rad/N], > 0: rear steers into the turn (understeer)
    ackermann_front: float = 0.0              # [-] 0 parallel, 1 full Ackermann
    ackermann_rear: float = 0.0               # [-]


@dataclass(frozen=True)
class ChassisMasses:
    """Mass and stiffness data needed for roll centres and attitude (vehicle YAML ``chassis_extended``)."""

    sprung_mass: float               # [kg]
    unsprung_mass_per_wheel: float   # [kg]
    roll_stiffness_front: float      # [N m/rad]
    roll_stiffness_rear: float       # [N m/rad]
    spring_rate_front: float         # [N/m] wheel rate
    spring_rate_rear: float          # [N/m] wheel rate


class QuasiStaticChassis:
    """Load-transfer share, body attitude, wheel travel and steer changes for a vehicle with K&C."""

    def __init__(self, params: SevenDOFParams, masses: ChassisMasses, kc: SuspensionKC | None = None):
        self.params, self.masses, self.kc = params, masses, kc or SuspensionKC()
        p, m = params, masses
        total = m.sprung_mass + 4.0 * m.unsprung_mass_per_wheel
        if abs(total - p.mass) > 1e-6 * p.mass:
            raise ValueError(f"sprung + 4 unsprung = {total} kg differs from mass {p.mass} kg")
        L = p.wheelbase
        self.h_u = p.Reff
        self.h_s = (p.mass * p.hcog - 4.0 * m.unsprung_mass_per_wheel * self.h_u) / m.sprung_mass
        self.m_sf = p.mass * p.lr / L - 2.0 * m.unsprung_mass_per_wheel
        self.m_sr = p.mass * p.lf / L - 2.0 * m.unsprung_mass_per_wheel
        if self.m_sf <= 0.0 or self.m_sr <= 0.0:
            raise ValueError("unsprung masses exceed the static axle masses")
        kc = self.kc
        self.h_ra = (self.m_sf * kc.roll_centre_height_front + self.m_sr * kc.roll_centre_height_rear) / m.sprung_mass
        self.roll_arm = self.h_s - self.h_ra
        self.k_elastic = m.roll_stiffness_front / (m.roll_stiffness_front + m.roll_stiffness_rear)
        self.roll_gradient = m.sprung_mass * self.roll_arm / (m.roll_stiffness_front + m.roll_stiffness_rear)
        self.pitch_stiffness = 2.0 * (m.spring_rate_front * p.lf**2 + m.spring_rate_rear * p.lr**2)

    # ------------------------------------------------------------ load transfer
    def lateral_transfer_moments(self) -> tuple[float, float]:
        """Per unit a_y: ``(w_f dF_f, w_r dF_r)`` [kg m], split into elastic + geometric + unsprung parts."""
        m, kc = self.masses, self.kc
        mu_hu = 2.0 * m.unsprung_mass_per_wheel * self.h_u
        front = m.sprung_mass * self.roll_arm * self.k_elastic + self.m_sf * kc.roll_centre_height_front + mu_hu
        rear = m.sprung_mass * self.roll_arm * (1.0 - self.k_elastic) + self.m_sr * kc.roll_centre_height_rear + mu_hu
        return front, rear

    @property
    def front_lateral_transfer_share(self) -> float:
        """Effective ``k_roll_f`` for vehicle_models/normal_loads.py."""
        front, rear = self.lateral_transfer_moments()
        return front / (front + rear)

    # ------------------------------------------------------------ attitude
    def roll_angle(self, ay) -> np.ndarray:
        return self.roll_gradient * np.asarray(ay, dtype=float)

    def pitch_angle(self, ax) -> np.ndarray:
        return -self.masses.sprung_mass * np.asarray(ax, dtype=float) * self.h_s / self.pitch_stiffness

    def wheel_travel(self, ax: float, ay: float) -> np.ndarray:
        """Wheel compression [m], rows FL, FR, RL, RR."""
        p = self.params
        x = np.array([p.lf, p.lf, -p.lr, -p.lr])
        y = np.array([p.wf, -p.wf, p.wr, -p.wr]) / 2.0
        return -y * self.roll_angle(ay) + x * self.pitch_angle(ax)

    # ------------------------------------------------------------ steer
    def steer_change(self, ax: float, ay: float, Fy_front: float, Fy_rear: float) -> np.ndarray:
        """K&C steer change [rad] per wheel (FL, FR, RL, RR) for accelerations and axle lateral forces."""
        kc = self.kc
        z = self.wheel_travel(ax, ay)
        toe = np.concatenate([kc.bump_steer_front(z[:2]), kc.bump_steer_rear(z[2:])])
        side = np.array([-1.0, 1.0, -1.0, 1.0])  # toe-in turns a left wheel to the right
        compliance = np.array([-kc.compliance_steer_front * Fy_front] * 2 + [kc.compliance_steer_rear * Fy_rear] * 2)
        return side * toe + compliance

    def roll_steer_coefficients(self) -> tuple[float, float]:
        """Axle steer per body roll ``d delta/d phi`` [rad/rad] (front, rear); > 0: towards the turn."""
        p, kc = self.params, self.kc
        return kc.bump_steer_front.gradient_at_zero() * p.wf / 2.0, kc.bump_steer_rear.gradient_at_zero() * p.wr / 2.0

    # ------------------------------------------------------------ construction
    @classmethod
    def from_yaml(cls, params: SevenDOFParams, vehicle_yaml: str | Path, kc_yaml: str | Path | None) -> QuasiStaticChassis:
        """Masses from the vehicle YAML's ``chassis_extended`` section, K&C from ``kc_yaml`` (None: all zero)."""
        with open(vehicle_yaml, "r", encoding="utf-8") as f:
            ext = (yaml.safe_load(f) or {}).get("chassis_extended")
        if not ext:
            raise ValueError(f"{vehicle_yaml} has no chassis_extended section")
        masses = ChassisMasses(**{k: float(ext[k]) for k in ChassisMasses.__dataclass_fields__})
        return cls(params, masses, load_kc(kc_yaml) if kc_yaml else None)


def _toe_map(spec) -> ToeMap:
    if spec is None:
        return ToeMap()
    if isinstance(spec, (int, float)):
        return ToeMap.linear(float(spec))
    return ToeMap(tuple(float(v) for v in spec["travel"]), tuple(float(v) for v in spec["toe_in"]))


def load_kc(path: str | Path) -> SuspensionKC:
    """Read a K&C set: scalar bump steer = gradient [rad/m], or a map ``{travel: [...], toe_in: [...]}``."""
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    if "provenance" not in data:
        raise ValueError(f"K&C set {path} has no provenance section")
    kc = data.get("kinematics_compliance", {})
    return SuspensionKC(
        roll_centre_height_front=float(kc.get("roll_centre_height_front", 0.0)),
        roll_centre_height_rear=float(kc.get("roll_centre_height_rear", 0.0)),
        bump_steer_front=_toe_map(kc.get("bump_steer_front")),
        bump_steer_rear=_toe_map(kc.get("bump_steer_rear")),
        compliance_steer_front=float(kc.get("compliance_steer_front", 0.0)),
        compliance_steer_rear=float(kc.get("compliance_steer_rear", 0.0)),
        ackermann_front=float(kc.get("ackermann_front", 0.0)),
        ackermann_rear=float(kc.get("ackermann_rear", 0.0)),
    )


__all__ = ("ChassisMasses", "QuasiStaticChassis", "SuspensionKC", "ToeMap", "load_kc")
