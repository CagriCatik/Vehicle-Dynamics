"""Path references for drift tracking: circles and the figure-eight.

Kinematics used throughout (plan 4.1): the direction of travel is ``chi = psi + beta``; only in
a stationary drift is ``chi_dot = r``. The path error of a circle is radial: ``e = |p - c| - R``
(positive outside), so it does not depend on where on the circle the vehicle is.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class CircleReference:
    cx: float
    cy: float
    radius: float
    direction: str                 # "left": counter-clockwise (yaw rate > 0), "right": clockwise

    def __post_init__(self):
        if self.radius <= 0.0 or self.direction not in ("left", "right"):
            raise ValueError("radius must be positive and direction 'left' or 'right'")

    @property
    def curvature(self) -> float:
        """Signed path curvature [1/m] (> 0 left)."""
        return (1.0 if self.direction == "left" else -1.0) / self.radius

    def radial_error(self, X, Y):
        """``|p - c| - R`` [m]: > 0 outside the circle."""
        return np.hypot(np.asarray(X, float) - self.cx, np.asarray(Y, float) - self.cy) - self.radius

    @classmethod
    def through_state(cls, X: float, Y: float, chi: float, radius: float, direction: str, offset: float = 0.0):
        """Circle tangent to the direction of travel at ``(X, Y)``; ``offset`` puts the start outside (> 0)."""
        s = 1.0 if direction == "left" else -1.0
        n = np.array([-math.sin(chi), math.cos(chi)]) * s          # towards the centre
        c = np.array([X, Y]) + (radius + offset) * n
        return cls(float(c[0]), float(c[1]), float(radius), direction)

    def points(self, n: int = 361):
        a = np.linspace(0.0, 2.0 * math.pi, n)
        return self.cx + self.radius * np.cos(a), self.cy + self.radius * np.sin(a)


@dataclass(frozen=True)
class FigureEightReference:
    """Two tangent circles of equal radius; the left one first, crossing point at the origin of the eight."""

    left: CircleReference
    right: CircleReference

    @classmethod
    def through_state(cls, X: float, Y: float, chi: float, radius: float):
        left = CircleReference.through_state(X, Y, chi, radius, "left")
        right = CircleReference.through_state(X, Y, chi, radius, "right")
        return cls(left, right)

    def radial_error(self, X, Y, section: str):
        return (self.left if section == "left" else self.right).radial_error(X, Y)


def chi(psi, vx, vy):
    """Direction of travel ``psi + atan2(vy, vx)`` [rad]."""
    return np.asarray(psi, float) + np.arctan2(np.asarray(vy, float), np.asarray(vx, float))


__all__ = ("CircleReference", "FigureEightReference", "chi")
