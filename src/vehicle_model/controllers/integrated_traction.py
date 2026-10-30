"""Integrated traction control, ITC (Plan V4.0 M18.2): coordination of TCS, eLSD, TV and the motor envelope.

ITC is not a new actuator path. It configures the shared chain so that its parts do not work against each
other:

    TCS         the drive slip controller of the ActuationLayer (unchanged, ``enable_tcs``)
    eLSD        the traction distribution's coupling bounds (:mod:`.traction_distribution`)
    envelope    drive bounds from the motor's speed-dependent torque (same module)
    TV          the yaw law, wrapped by :class:`YawToleranceLaw`

Why the wrapper (measured, M18 split-mu launch): a yaw-rate law with yaw priority asks for ``M_z`` that
cancels every yaw-rate error, i.e. ``M_z ~ 0`` on a straight launch; the allocation then cannot drive the
high-friction side harder than the low-friction side, and the launch is exactly as slow as with an open
differential (34 km/h in 6 s against 48 km/h with the eLSD alone). ITC lets the traction asymmetry through
while the yaw-rate error stays inside a tolerance, and hands the yaw to the TV law outside it:

    |r_ref - r| <  tolerance   -> no yaw request (``None``: M_z free within the eLSD/grip bounds)
    |r_ref - r| >= tolerance   -> the wrapped law's yaw request

The wrapped law keeps running every step (its integrator and state stay consistent); the band only decides
whether its request is passed on.
"""

from __future__ import annotations

import math
from dataclasses import replace


class YawToleranceLaw:
    def __init__(self, inner, tolerance: float):
        if tolerance < 0.0:
            raise ValueError("yaw tolerance must be >= 0")
        self.inner, self.tolerance = inner, float(tolerance)
        self.name = f"{inner.name} + ITC band {math.degrees(self.tolerance):.1f} deg/s"
        self.information = getattr(inner, "information", None)
        self.passed = False

    def reset(self) -> None:
        self.inner.reset()
        self.passed = False

    def request(self, state, ctx):
        req = self.inner.request(state, ctx)
        self.passed = abs(ctx.r_ref - state.r) >= self.tolerance
        if self.passed:
            return req
        return replace(req, yaw_moment=None, diagnostics={**(req.diagnostics or {}), "itc": "yaw inside band"})


__all__ = ("YawToleranceLaw",)
