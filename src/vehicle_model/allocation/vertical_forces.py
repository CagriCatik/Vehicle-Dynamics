"""Vertical force allocation: body demand ``(F_heave, M_pitch, M_roll)`` -> four active strut forces.

Active strut forces act between body and wheel at the corners ``(x_i, y_i)`` of the 14-DOF model (positive:
pushing the body up). With the model's corner kinematics ``z_s,i = z_s + x_i theta + y_i phi``:

    F_heave = sum_i F_i,     M_pitch = sum_i x_i F_i  (nose-up positive),     M_roll = sum_i y_i F_i

The bounded weighted least-squares solver of the longitudinal allocation (``wls.solve_box_wls``) is reused;
priorities by the weights ``W_v`` (pitch first by default), forces within ``+-f_max``. No force is applied at
the centre of gravity directly: only these four strut forces enter the plant (its ``active_forces`` input).
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .wls import solve_box_wls


@dataclass(frozen=True)
class VerticalAllocationResult:
    forces: np.ndarray        # [N] FL, FR, RL, RR
    demand: np.ndarray        # (F_heave, M_pitch, M_roll)
    achieved: np.ndarray
    residual: np.ndarray
    saturated: bool
    diagnostics: dict = field(default_factory=dict)


class VerticalForceAllocator:
    def __init__(self, corner_x, corner_y, f_max: float = 3000.0, weights=(1.0 / 2000.0, 1.0 / 500.0, 1.0 / 1000.0)):
        self.B = np.vstack([np.ones(4), np.asarray(corner_x, float), np.asarray(corner_y, float)])
        self.f_max = float(f_max)
        self.W_v = np.asarray(weights, float)

    def allocate(self, F_heave: float, M_pitch: float, M_roll: float) -> VerticalAllocationResult:
        v = np.array([F_heave, M_pitch, M_roll], float)
        if not np.all(np.isfinite(v)):
            raise ValueError("vertical demand must be finite")
        lo, hi = np.full(4, -self.f_max), np.full(4, self.f_max)
        sol = solve_box_wls(self.B, v, lo, hi, W_v=self.W_v, W_u=np.full(4, 1.0 / self.f_max), eps=1e-9)
        return VerticalAllocationResult(sol.u, v, sol.achieved, sol.residual, bool(np.any(sol.at_lower | sol.at_upper)))


__all__ = ("VerticalAllocationResult", "VerticalForceAllocator")
