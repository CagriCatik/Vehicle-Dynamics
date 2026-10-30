"""Integrated brake slip control, IBC (Plan V4.0 M17.3): physical slip targets for the existing slip controller.

The existing ``WheelSlipController`` (ABS mode) tracks a target slip ``kappa*(mu_hat)`` taken from a
calibration table. IBC does not replace it; it *derives* that table from the tyre model the
controller is designed with (``Fx(kappa)`` at zero slip angle and nominal load, one row per friction
value) instead of a hand calibration, so a change of tyre set changes the target consistently:

    kappa*(mu) = argmax_kappa |Fx(kappa, alpha = 0, Fz_nom, mu)|        (brake side, kappa < 0)

optionally shifted below the peak by ``peak_fraction`` (a target on the stable side of the peak:
``|Fx(kappa*)| = peak_fraction * max |Fx|``, the lateral-force reserve under combined slip). The
slip controller's ``utilisation_at_target`` is set to the matching force fraction, so its upward
friction adaptation stays consistent with the target.

IBC also owns the **regeneration policy under ABS** that the actuator chain applies
(``regen_policy``):

    cut_on_abs     the existing brake-by-wire behaviour: regen torque to 0 on a wheel while ABS works
    modulated      the regen torque on that wheel is limited by the slip loop (see
                   :mod:`.regenerative_slip`) instead of being cut: experimental, compared, not assumed better

The tyre model here is the controller's *design* model (bundled tyre set), not the plant object.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from vehicle_model.controllers.slip_control import SlipControlConfig

REGEN_POLICIES = ("cut_on_abs", "modulated")


@dataclass(frozen=True)
class IntegratedBrakeConfig:
    tyre: str = "magic_formula_passenger_synthetic"   # design tyre set (bundled name or YAML path)
    Fz_nominal: float = 3500.0                         # [N] load at which the slip curve is evaluated
    mu_grid: tuple = (0.1, 0.2, 0.3, 0.5, 0.7, 1.0)    # [-]
    peak_fraction: float = 1.0                         # [-] 1: target at the force peak; < 1: on the stable side
    regen_policy: str = "cut_on_abs"

    def __post_init__(self):
        if not 0.5 <= self.peak_fraction <= 1.0:
            raise ValueError("peak_fraction must lie in [0.5, 1]")
        if self.regen_policy not in REGEN_POLICIES:
            raise ValueError(f"regen_policy must be one of {REGEN_POLICIES}")
        if self.Fz_nominal <= 0 or len(self.mu_grid) < 2 or np.any(np.diff(self.mu_grid) <= 0):
            raise ValueError("Fz_nominal must be > 0 and mu_grid increasing")


def slip_targets(tyre, Fz_nominal: float, mu_grid, peak_fraction: float = 1.0) -> tuple[np.ndarray, np.ndarray]:
    """``(|kappa*|, force fraction at kappa*)`` per friction value from the tyre's pure longitudinal curve."""
    kap = -np.linspace(1e-4, 0.6, 3000)
    out_k, out_f = [], []
    for mu in mu_grid:
        n = kap.size
        Fx, _ = tyre.forces(kap, np.zeros(n), np.full(n, Fz_nominal), np.full(n, mu))
        a = np.abs(np.asarray(Fx, float))
        i = int(np.argmax(a))
        if peak_fraction < 1.0:
            ok = np.flatnonzero(a[: i + 1] >= peak_fraction * a[i])
            i = int(ok[0]) if ok.size else i
        out_k.append(abs(kap[i]))
        out_f.append(a[i] / (mu * Fz_nominal))
    return np.array(out_k), np.array(out_f)


def ibc_slip_config(config: IntegratedBrakeConfig | None = None, base: SlipControlConfig | None = None) -> SlipControlConfig:
    """The ABS slip-controller configuration with tyre-derived targets (gains etc. from ``base``)."""
    from vehicle_model.tire_model import load_tyre
    c = config or IntegratedBrakeConfig()
    b = base or SlipControlConfig(mode="brake")
    k, frac = slip_targets(load_tyre(c.tyre), c.Fz_nominal, c.mu_grid, c.peak_fraction)
    k = np.maximum.accumulate(k)                       # monotone over mu (interpolation table)
    return SlipControlConfig(mode="brake", kp=b.kp, ki=b.ki, mu_table=tuple(float(x) for x in c.mu_grid),
                             slip_table=tuple(float(x) for x in k),
                             utilisation_at_target=float(np.clip(np.min(frac) * 0.98, 0.5, 1.2)),
                             min_speed=b.min_speed, initial_mu=b.initial_mu)


__all__ = ("IntegratedBrakeConfig", "REGEN_POLICIES", "ibc_slip_config", "slip_targets")
