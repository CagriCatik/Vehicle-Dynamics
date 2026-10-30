"""Active front steering, AFS: variable ratio and a bounded overlay, finalised in one place.

The front road-wheel angle is decided only in :class:`~vehicle_model.controllers.motion_control.MotionController`:

    delta_f = saturate( delta_driver * g(v)  +  overlay ,  +-max_steer_f )

    g(v)      variable steering ratio as a gain on the driver's road-wheel angle (``ratio_speeds``,
              ``ratio_gains``, linear in between; 1 = the fixed ratio of the reference car): more direct
              at low speed, less direct at high speed
    overlay   the yaw law's ``front_steer`` request (``LQRYawLaw`` with inputs ``afs`` / ``afs_tv``),
              limited to ``max_overlay`` and to ``max_overlay_rate`` per second; 0 if the AFS has failed

The physical steering actuator (lag, rate, delay) is the existing ``SteeringActuator`` in the actuator
chain (``actuators: dynamic``); AFS, 4WS and TV can serve the same target yaw rate, but only this
function sets the front angle.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class SteeringOverlayConfig:
    ratio_speeds: tuple = (0.0, 15.0, 35.0)          # [m/s]
    ratio_gains: tuple = (1.0, 1.0, 1.0)              # [-] fixed ratio by default
    max_overlay: float = math.radians(5.0)            # [rad] road-wheel overlay authority
    max_overlay_rate: float = math.radians(40.0)      # [rad/s]
    failed: bool = False                              # AFS failed: overlay 0, driver steering through the fixed ratio

    def __post_init__(self):
        if len(self.ratio_speeds) != len(self.ratio_gains) or np.any(np.diff(self.ratio_speeds) <= 0):
            raise ValueError("ratio_speeds must be increasing and match ratio_gains")
        if min(self.ratio_gains) <= 0 or self.max_overlay < 0 or self.max_overlay_rate <= 0:
            raise ValueError("ratio gains must be > 0, overlay limits >= 0")

    def gain(self, vx: float) -> float:
        if self.failed:
            return 1.0
        return float(np.interp(abs(vx), self.ratio_speeds, self.ratio_gains))


class SteeringOverlay:
    """Rate-limited overlay state and the one place where the front angle is finalised."""

    def __init__(self, config: SteeringOverlayConfig | None = None):
        self.config = config or SteeringOverlayConfig()
        self.reset()

    def reset(self) -> None:
        self.overlay = 0.0

    @property
    def authority(self) -> float:
        return 0.0 if self.config.failed else self.config.max_overlay

    def finalise(self, driver_steer: float, vx: float, overlay_request: float, max_steer: float, dt: float):
        """``(delta_f, driver part, overlay)``."""
        c = self.config
        drv = driver_steer * c.gain(vx)
        want = 0.0 if c.failed else float(np.clip(overlay_request, -c.max_overlay, c.max_overlay))
        step = c.max_overlay_rate * dt
        self.overlay = float(np.clip(want, self.overlay - step, self.overlay + step))
        return float(np.clip(drv + self.overlay, -max_steer, max_steer)), drv, self.overlay


__all__ = ("SteeringOverlay", "SteeringOverlayConfig")
