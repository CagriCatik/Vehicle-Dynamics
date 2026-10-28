"""Plausibility and consistency checks for vehicle parameter sets (Plan.md, Abschnitt 4.2).

``check_vehicle_consistency`` returns a list of human-readable violations; an empty
list means the set passed. The plausibility bands are general passenger-car ranges
from the literature, not hard physical limits, and are listed in ``BANDS``.
"""

from __future__ import annotations

from typing import Any, Mapping

from vehicle_model.core.types import SevenDOFParams

# name: (lower, upper, explanation)
BANDS: dict[str, tuple[float, float, str]] = {
    "dynamic_index": (0.7, 1.3, "Iz/(m lf lr), passenger cars ~0.8-1.1"),
    "static_stability_factor": (1.0, 1.8, "(track/2)/hcog, passenger cars ~1.2-1.5"),
    "front_axle_load_share": (0.40, 0.65, "lr/L"),
    "Reff": (0.20, 0.45, "effective rolling radius [m]"),
}


def derived_quantities(p: SevenDOFParams) -> dict[str, float]:
    """Quantities derived from the base parameters (computed, never stored in YAML)."""
    L = p.wheelbase
    return {
        "wheelbase": L,
        "front_axle_load": p.mass * p.g * p.lr / L,
        "rear_axle_load": p.mass * p.g * p.lf / L,
        "front_axle_load_share": p.lr / L,
        "dynamic_index": p.Iz / (p.mass * p.lf * p.lr),
        "static_stability_factor": min(p.wf, p.wr) / (2.0 * p.hcog),
    }


def check_vehicle_consistency(p: SevenDOFParams, extended: Mapping[str, Any] | None = None) -> list[str]:
    """Return all violated consistency rules for ``p`` (and optional ``chassis_extended`` data)."""
    issues: list[str] = []
    for name in ("mass", "Iz", "lf", "lr", "wf", "wr", "hcog", "Iw", "Reff", "g", "rho"):
        if not getattr(p, name) > 0.0:
            issues.append(f"{name} must be positive (got {getattr(p, name)})")
    if issues:
        return issues

    if not 0.0 < p.k_roll_f < 1.0:
        issues.append(f"k_roll_f must lie in (0, 1) (got {p.k_roll_f})")
    if p.max_steer_r > p.max_steer_f:
        issues.append("max_steer_r exceeds max_steer_f")

    d = derived_quantities(p)
    if abs(d["front_axle_load"] + d["rear_axle_load"] - p.mass * p.g) > 1e-9 * p.mass * p.g:
        issues.append("static axle loads do not sum to m g")
    for key, (lo, hi, why) in BANDS.items():
        value = d[key] if key in d else getattr(p, key)
        if not lo <= value <= hi:
            issues.append(f"{key} = {value:.3f} outside plausible band [{lo}, {hi}] ({why})")

    if extended:
        m_s = extended.get("sprung_mass")
        m_u = extended.get("unsprung_mass_per_wheel")
        if m_s is not None and m_u is not None and abs(m_s + 4.0 * m_u - p.mass) > 1e-6 * p.mass:
            issues.append(f"sprung + 4*unsprung mass = {m_s + 4 * m_u} != mass {p.mass}")
        kf = extended.get("roll_stiffness_front")
        kr = extended.get("roll_stiffness_rear")
        if kf is not None and kr is not None:
            share = kf / (kf + kr)
            if abs(share - p.k_roll_f) > 1e-3:
                issues.append(f"k_roll_f = {p.k_roll_f} inconsistent with roll stiffness split {share:.4f}")
    return issues


__all__ = ("BANDS", "derived_quantities", "check_vehicle_consistency")
