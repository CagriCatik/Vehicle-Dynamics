"""Slip-stiffness scaling for any :class:`~vehicle_model.tire_model.framework.TyreModel`.

:class:`SlipStiffnessScaledTyre` stretches the slip axes of a base tyre:

    F(kappa, alpha) = F_base(s_x kappa, atan(s_y tan alpha))

so the cornering stiffness is multiplied by ``s_y`` and the longitudinal slip
stiffness by ``s_x``, while the friction peak and the force at full sliding stay
those of the base tyre (only the slip at which they are reached moves). This is
the same idea as the stiffness scaling factors of the Magic Formula, but it works
for every tyre model behind the protocol. It is a sensitivity tool, not a tyre
fitted to measurements.
"""

from __future__ import annotations

import numpy as np

from .framework import TyreModel


class SlipStiffnessScaledTyre:
    """Base tyre with cornering stiffness x ``cornering`` and slip stiffness x ``longitudinal``."""

    def __init__(self, base: TyreModel, cornering: float = 1.0, longitudinal: float = 1.0):
        if not isinstance(base, TyreModel):
            raise TypeError(f"{type(base).__name__} does not implement the TyreModel protocol")
        if cornering <= 0.0 or longitudinal <= 0.0:
            raise ValueError("stiffness scale factors must be positive")
        self.base = base
        self.cornering = float(cornering)
        self.longitudinal = float(longitudinal)
        self.provenance = base.provenance

    def forces(self, kappa, alpha, Fz, mu):
        alpha_s = np.arctan(self.cornering * np.tan(np.asarray(alpha, dtype=float)))
        return self.base.forces(self.longitudinal * np.asarray(kappa, dtype=float), alpha_s, Fz, mu)

    def slip_stiffness(self, Fz, mu):
        return self.longitudinal * np.asarray(self.base.slip_stiffness(Fz, mu), dtype=float)


__all__ = ("SlipStiffnessScaledTyre",)
