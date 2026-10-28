"""Steering geometry: individual wheel angles with Ackermann share (Plan.md, Abschnitt 8; M4).

The axle (bicycle) angles ``delta_f``, ``delta_r`` define a rigid-body motion of
the vehicle. Its lateral velocity grows linearly along x, so the ideal
(slip-free, low-speed) angle of a wheel at body position ``(x_i, y_i)`` is

    tan(delta_ideal_i) = tan(delta(x_i)) / (1 - kappa y_i)
    tan(delta(x))      = tan(delta_r) + (x + l_r)/L (tan(delta_f) - tan(delta_r))
    kappa              = (tan(delta_f) - tan(delta_r)) / L        (path curvature per unit speed)

For front steer only this is the classic Ackermann condition
``cot(delta_o) - cot(delta_i) = w/L``. The form is not singular for parallel
4WS (``delta_f = delta_r``: ``kappa = 0``, every wheel takes the axle angle),
unlike the construction via the instantaneous centre.

A per-axle Ackermann share ``A`` blends between parallel steering (``A = 0``,
both wheels at the axle angle: the base model) and full Ackermann (``A = 1``):

    delta_i = delta_axle + A (delta_ideal_i - delta_axle)

``A > 1`` (over-Ackermann) and ``A < 0`` (anti-Ackermann) are allowed.

Limits: ideal geometry of a rigid linkage; no steering-arm kinematics, no
caster/kingpin effects, no steering compliance (see suspension/kinematics.py).
"""

from __future__ import annotations

import numpy as np


def ideal_wheel_angles(delta_f: float, delta_r: float, positions: np.ndarray, lf: float, lr: float) -> np.ndarray:
    """Slip-free wheel angles [rad] for wheels at ``positions`` ((n, 2) body x, y) [m]."""
    L = lf + lr
    tf, tr = np.tan(delta_f), np.tan(delta_r)
    kappa = (tf - tr) / L
    x, y = positions[:, 0], positions[:, 1]
    tan_axis = tr + (x + lr) / L * (tf - tr)
    return np.arctan2(tan_axis, 1.0 - kappa * y)  # branch-safe if the pole lies between the wheels


def wheel_steer_angles(
    delta_f: float,
    delta_r: float,
    positions: np.ndarray,
    lf: float,
    lr: float,
    ackermann_front: float = 0.0,
    ackermann_rear: float = 0.0,
) -> np.ndarray:
    """Wheel angles [rad], rows FL, FR, RL, RR, for axle angles and Ackermann shares."""
    ideal = ideal_wheel_angles(delta_f, delta_r, positions, lf, lr)
    axle = np.array([delta_f, delta_f, delta_r, delta_r], dtype=float)
    share = np.array([ackermann_front, ackermann_front, ackermann_rear, ackermann_rear], dtype=float)
    return axle + share * (ideal - axle)


__all__ = ("ideal_wheel_angles", "wheel_steer_angles")
