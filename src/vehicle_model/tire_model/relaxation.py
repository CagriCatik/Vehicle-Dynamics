"""First-order tyre relaxation (transient slip), deflection form (Pacejka 2012, sec. 7.2).

The contact patch needs a rolled distance ``sigma`` (relaxation length) to build
up force after a slip change. With the transient slips as states,

    sigma_k  d(kappa')/dt = (omega R - v_x) - V_ref kappa'
    sigma_a  d(s')/dt     = -v_y            - V_x   s'          (s' = tan(alpha'))

    V_ref = max(|v_x|, |omega R|, v_eps),   V_x = max(|v_x|, v_eps)

(``v_x``, ``v_y``: wheel-centre velocity in the wheel frame). The steady states are
exactly the quasi-static slips of :class:`~vehicle_model.vehicle_models.SevenDOFModel`,
``kappa = (omega R - v_x)/V_ref`` and ``alpha = atan(-v_y/V_x)``, so the steady-state
behaviour of the vehicle is unchanged. At constant speed the response to a slip step
is ``1 - exp(-x/sigma)`` over the rolled distance ``x``: a low-pass filter with
time constant ``sigma/V``. Unlike the division form ``d(alpha)/dt = V/sigma (alpha_ss - alpha)``,
the deflection form stays well posed at standstill (the tyre then behaves as a
spring, decaying slowly with ``v_eps/sigma``).

Limits: constant relaxation lengths (no load or slip dependence), no carcass mass,
no lag of the aligning moment beyond the slip lag, not valid for frequencies
above ~10 Hz or wavelengths below ~``2 pi sigma`` (Pacejka 2012, ch. 8).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class TyreRelaxation:
    """Relaxation lengths [m] and regularisation speed [m/s] (synthetic defaults)."""

    sigma_kappa: float = 0.10   # longitudinal (stiff carcass, ~0.1-0.2 m for passenger tyres)
    sigma_alpha: float = 0.40   # lateral (~0.3-0.6 m, roughly a tyre radius or more)
    v_eps: float = 0.20         # [m/s] same regularisation as the quasi-static slips

    def __post_init__(self) -> None:
        if self.sigma_kappa <= 0.0 or self.sigma_alpha <= 0.0 or self.v_eps <= 0.0:
            raise ValueError("relaxation lengths and v_eps must be positive")

    def reference_speeds(self, v_wx, v_wheel) -> tuple[np.ndarray, np.ndarray]:
        """``(V_ref, V_x)`` used by the longitudinal and lateral equations."""
        v_wx = np.asarray(v_wx, dtype=float)
        V_x = np.maximum(np.abs(v_wx), self.v_eps)
        V_ref = np.maximum(V_x, np.abs(np.asarray(v_wheel, dtype=float)))
        return V_ref, V_x

    def derivatives(self, kappa_t, tan_alpha_t, v_wx, v_wy, v_wheel) -> tuple[np.ndarray, np.ndarray]:
        """``(d kappa'/dt, d tan(alpha')/dt)`` for wheel-frame velocities and ``v_wheel = omega R``."""
        V_ref, V_x = self.reference_speeds(v_wx, v_wheel)
        dk = ((np.asarray(v_wheel) - v_wx) - V_ref * np.asarray(kappa_t)) / self.sigma_kappa
        ds = (-np.asarray(v_wy) - V_x * np.asarray(tan_alpha_t)) / self.sigma_alpha
        return dk, ds

    def steady_state(self, v_wx, v_wy, v_wheel) -> tuple[np.ndarray, np.ndarray]:
        """Steady transient slips ``(kappa', tan(alpha'))`` = quasi-static slips."""
        V_ref, V_x = self.reference_speeds(v_wx, v_wheel)
        return (np.asarray(v_wheel) - v_wx) / V_ref, -np.asarray(v_wy) / V_x


__all__ = ("TyreRelaxation",)
