"""Electronic brakeforce distribution, EBD (Plan V4.0 M17.1): a strategy for the shared allocation, not a brake.

EBD decides how a total braking demand is *shared* between the wheels and how much the rear
axle may take; the bounded allocation (``WheelForceAllocator``) and the slip control after it
stay the only owners of the wheel torques. The strategy returns, per control step,

    share        (4,) distribution weights for the allocation's secondary objective
                 (among equally good solutions ``F_i`` is proportional to ``share_i``)
    brake_limit  (4,) [N] additional upper bound on the braking force per wheel

computed from the wheel loads ``Fz_hat`` and friction ``mu_hat`` the *controller* receives (plant
telemetry in information modes A/B, the sensor-based estimates in mode C: no ground truth in C).

Modes:

    grip      share = None, no limit: the allocation's own grip-proportional distribution
              (the behaviour before M17)
    static    fixed front share (default 0.6, the passive car's 60:40 bias), left/right equal;
              a reference, blind to load transfer and friction
    ebd       share proportional to the estimated grip ``mu_hat Fz_hat`` (dynamic load transfer
              included in ``Fz_hat``), and two rear-axle protections:
                - the rear braking force is limited to ``rear_margin`` times the rear wheel's
                  estimated grip, so the rear does not reach its limit before the front
                  (rear lock-up costs directional stability);
                - pressure hold: when ABS starts working on a rear wheel, its limit is frozen at the
                  braking force the allocation gave it in the step before (before slip control) and
                  released when ABS stops; frozen, not lowered step by step (no ratchet)

Failed brakes are handled by the allocation (``ActuatorHealth``): the strategy does not shift
demand by itself. ``reset()`` clears the hold memory.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

MODES = ("grip", "static", "ebd")


@dataclass(frozen=True)
class BrakeDistributionConfig:
    mode: str = "grip"
    static_front_share: float = 0.6       # [-] static mode: front axle share of the braking force
    rear_margin: float = 0.85             # [-] ebd: rear braking force <= rear_margin * mu_hat * Fz_hat
    hold_on_abs: bool = True              # ebd: no rise of a rear limit while ABS works on that wheel

    def __post_init__(self):
        if self.mode not in MODES:
            raise ValueError(f"brake distribution mode must be one of {MODES}, got {self.mode!r}")
        if not 0.0 < self.static_front_share < 1.0:
            raise ValueError("static_front_share must lie in (0, 1)")
        if not 0.0 < self.rear_margin <= 1.0:
            raise ValueError("rear_margin must lie in (0, 1]")


class BrakeDistributionStrategy:
    def __init__(self, config: BrakeDistributionConfig | None = None):
        self.config = config or BrakeDistributionConfig()
        self.reset()

    def reset(self) -> None:
        self._hold = np.full(4, np.nan)

    def distribute(self, Fz_hat, mu_hat, abs_active=None, last_brake_force=None):
        """``last_brake_force`` (4) [N]: braking force the allocation gave each wheel in the previous step."""
        """``(share, brake_limit)`` for the allocation (``None`` where the strategy imposes nothing)."""
        c = self.config
        if c.mode == "grip":
            return None, None
        Fz = np.maximum(np.asarray(Fz_hat, float).reshape(4), 1.0)
        mu = np.maximum(np.asarray(mu_hat, float).reshape(4), 0.01)
        if c.mode == "static":
            f, r = c.static_front_share / 2.0, (1.0 - c.static_front_share) / 2.0
            return np.array([f, f, r, r]), None
        grip = mu * Fz
        share = grip / np.sum(grip)
        limit = np.full(4, np.inf)
        limit[2:] = c.rear_margin * grip[2:]
        if c.hold_on_abs and abs_active is not None:
            act = np.asarray(abs_active, bool)
            prev = np.full(4, np.nan) if last_brake_force is None else np.asarray(last_brake_force, float)
            start = act & np.isnan(self._hold)
            self._hold = np.where(start, prev, self._hold)
            self._hold = np.where(act, self._hold, np.nan)
            hold = self._hold[2:]
            limit[2:] = np.where(np.isfinite(hold), np.minimum(limit[2:], hold), limit[2:])
        return share, np.where(np.isfinite(limit), limit, 1e9)




def brake_functions_config(path=None) -> dict:
    """The four brake-function configurations from ``configs/braking/brake_functions.yaml`` (Plan V4.0 M17):
    ``{"ebd": dict, "cbc": CorneringBrakeConfig, "ibc": dict, "regenerative_slip": dict}``; strict keys."""
    from pathlib import Path

    import yaml

    from vehicle_model.controllers.cornering_brake import CorneringBrakeConfig
    if path is None:
        from vehicle_model.config.loader import get_configs_root
        path = get_configs_root() / "braking" / "brake_functions.yaml"
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    allowed = {"ebd": {"static_front_share", "rear_margin", "hold_on_abs"},
               "cbc": {"c_ff", "k_fb", "deadband", "min_decel", "min_lat"},
               "ibc": {"tyre", "Fz_nominal", "mu_grid", "peak_fraction"},
               "regenerative_slip": {"target_slip", "kp", "ki", "min_speed"}}
    unknown = set(data) - set(allowed)
    if unknown:
        raise ValueError(f"{path}: unknown sections {sorted(unknown)}")
    for sec, keys in allowed.items():
        bad = set(data.get(sec) or {}) - keys
        if bad:
            raise ValueError(f"{path}: {sec}: unknown keys {sorted(bad)}")
    ebd = dict(data.get("ebd") or {})
    BrakeDistributionConfig(mode="ebd", **ebd)                    # validates the values
    ibc = dict(data.get("ibc") or {})
    if "mu_grid" in ibc:
        ibc["mu_grid"] = tuple(float(x) for x in ibc["mu_grid"])
    return {"ebd": ebd, "cbc": CorneringBrakeConfig(**(data.get("cbc") or {})), "ibc": ibc,
            "regenerative_slip": dict(data.get("regenerative_slip") or {})}


__all__ = ("BrakeDistributionConfig", "BrakeDistributionStrategy", "MODES", "brake_functions_config")
