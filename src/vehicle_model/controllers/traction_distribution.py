"""Drive-torque distribution for four independent motors: open, virtual eLSD, free; motor envelope (Plan V4.0 M18).

Like EBD for braking (:mod:`.brake_distribution`), this is a *strategy for the shared allocation*, not a
second allocator: it returns per-wheel upper bounds on the drive force (``drive_limit`` [N]) while the
allocation demand is positive. There is no mechanical differential in the model; the "differential"
behaviour is a constraint on the electronically commanded wheel forces of each axle:

    free     no coupling: each motor up to its own grip and torque bound (the M5 allocation)
    open     open-differential equivalent: both wheels of an axle at most what the weaker wheel can
             transmit, ``F_L, F_R <= min(U_L, U_R)``; the axle delivers at most twice the weaker side
    elsd     virtual electronic limited-slip: the stronger wheel may exceed the weaker by at most the
             locking torque, ``F_strong <= U_weak + T_lock / R`` (``T_lock = 0`` is ``open``, a large
             ``T_lock`` is ``free``)

``U_i`` are the allocation's own upper bounds (grip from the loads, friction and lateral forces the
controller receives, and the drive torque limit). The constraint only lowers bounds: transferring torque
between wheels never creates force that the sum of the wheel demands did not contain, and the allocation
reports any unmet demand as residual.

Motor envelope (``envelope: true``, part of ITC): the drive bound of each wheel is additionally limited to
the torque the electric machine can deliver at the measured wheel speed, ``min(T_max, P_max / omega)``
(``ElectricDriveConfig``), so the allocation does not plan torque the motors cannot produce.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

MODES = ("free", "open", "elsd")


@dataclass(frozen=True)
class TractionDistributionConfig:
    mode: str = "free"
    lock_torque: float = 300.0        # [N m] eLSD: largest left/right torque difference per axle
    envelope: bool = False            # limit the drive bound to the motor's speed-dependent torque

    def __post_init__(self):
        if self.mode not in MODES:
            raise ValueError(f"traction distribution mode must be one of {MODES}, got {self.mode!r}")
        if self.lock_torque < 0.0:
            raise ValueError("lock_torque must be >= 0")


class TractionDistributionStrategy:
    def __init__(self, config: TractionDistributionConfig | None = None, Reff: float = 0.32, drive_config=None):
        self.config = config or TractionDistributionConfig()
        self.Reff = float(Reff)
        self.drive_config = drive_config
        if self.config.envelope and drive_config is None:
            from vehicle_model.actuators.powertrain_actuator import ElectricDriveConfig
            self.drive_config = ElectricDriveConfig.from_yaml()

    def reset(self) -> None:
        pass

    def motor_limit(self, omega) -> np.ndarray:
        """[N] drive force each motor can deliver at the wheel speeds ``omega`` [rad/s]."""
        c = self.drive_config
        w = np.maximum(np.abs(np.asarray(omega, float)), c.base_wheel_speed)
        return np.minimum(c.max_torque, c.max_power / w) / self.Reff

    def drive_limit(self, upper, omega=None):
        """Upper drive-force bounds [N] for the allocation (``None``: no additional bound)."""
        c = self.config
        U = np.asarray(upper, float).reshape(4).copy()
        if c.envelope and omega is not None:
            U = np.minimum(U, self.motor_limit(omega))
        if c.mode == "free":
            return U if c.envelope else None
        out = U.copy()
        for a, b in ((0, 1), (2, 3)):
            weak = min(U[a], U[b])
            extra = 0.0 if c.mode == "open" else c.lock_torque / self.Reff
            out[a], out[b] = min(U[a], weak + extra), min(U[b], weak + extra)
        return out


__all__ = ("MODES", "TractionDistributionConfig", "TractionDistributionStrategy")
