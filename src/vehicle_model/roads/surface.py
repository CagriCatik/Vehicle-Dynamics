"""Road surface friction models with spatial dependency."""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
import numpy as np


class RoadSurface(ABC):
    """Base abstract road surface delivering local friction coefficients mu(X, Y)."""

    @abstractmethod
    def get_friction(self, X: float, Y: float) -> float:
        """Return the local road friction coefficient at world coordinates (X, Y)."""
        pass

    def get_friction_batch(self, X: np.ndarray, Y: np.ndarray) -> np.ndarray:
        """Vectorized evaluation over multiple wheel contact points."""
        X_arr = np.atleast_1d(X)
        Y_arr = np.atleast_1d(Y)
        out = np.zeros_like(X_arr, dtype=float)
        for i in range(X_arr.size):
            out.flat[i] = self.get_friction(float(X_arr.flat[i]), float(Y_arr.flat[i]))
        return out


@dataclass
class UniformRoad(RoadSurface):
    """Uniform road with identical friction coefficient everywhere."""

    mu: float = 1.0

    def get_friction(self, X: float, Y: float) -> float:
        return float(self.mu)


@dataclass
class MuSplitRoad(RoadSurface):
    """Split-mu road dividing the surface along a dividing line (default Y = 0).

    Typically:
      Y >= split_y: Left side (e.g., dry asphalt mu = 1.0)
      Y <  split_y: Right side (e.g., ice/snow mu = 0.2)
    """

    mu_left: float = 1.0
    mu_right: float = 0.2
    split_y: float = 0.0
    transition_width: float = 0.1  # Smooth transition to avoid numerical step discontinuity

    def get_friction(self, X: float, Y: float) -> float:
        if self.transition_width <= 1e-4:
            return self.mu_left if Y >= self.split_y else self.mu_right
        # Smooth logistic transition across split boundary
        s = 1.0 / (1.0 + np.exp(-(Y - self.split_y) / self.transition_width))
        return float(self.mu_right + (self.mu_left - self.mu_right) * s)


@dataclass
class TransitionRoad(RoadSurface):
    """Road exhibiting a longitudinal friction transition (e.g. asphalt onto icy bridge)."""

    mu_before: float = 1.0
    mu_after: float = 0.2
    x_start: float = 30.0
    transition_length: float = 5.0

    def get_friction(self, X: float, Y: float) -> float:
        if X < self.x_start:
            return float(self.mu_before)
        if X > self.x_start + self.transition_length:
            return float(self.mu_after)
        # Linear or smooth transition
        progress = (X - self.x_start) / self.transition_length
        return float(self.mu_before + (self.mu_after - self.mu_before) * progress)


@dataclass
class FrictionPatchRoad(RoadSurface):
    """Road with a default background friction and rectangular localized friction patches."""

    default_mu: float = 1.0
    # List of tuples: (x_min, x_max, y_min, y_max, patch_mu)
    patches: list[tuple[float, float, float, float, float]] = field(default_factory=list)

    def get_friction(self, X: float, Y: float) -> float:
        for x_min, x_max, y_min, y_max, patch_mu in self.patches:
            if x_min <= X <= x_max and y_min <= Y <= y_max:
                return float(patch_mu)
        return float(self.default_mu)


@dataclass
class DiagonalSplitRoad(RoadSurface):
    """Split-mu road with an arbitrary angled boundary line.

    The dividing boundary passes through anchor point (x0, y0) at an angle
    theta_deg relative to the world X-axis.
    - One side has mu_high (e.g. dry asphalt 1.0)
    - The other side has mu_low (e.g. ice 0.20)
    """

    mu_high: float = 1.0
    mu_low: float = 0.20
    x0: float = 30.0
    y0: float = 0.0
    theta_deg: float = 30.0  # Angle of dividing boundary line [deg]
    transition_width: float = 0.15

    def get_friction(self, X: float, Y: float) -> float:
        th = np.radians(self.theta_deg)
        # Normal vector to line pointing towards the mu_high side: (-sin(th), cos(th))
        # Signed perpendicular distance from (X, Y) to the dividing line:
        d = -(X - self.x0) * np.sin(th) + (Y - self.y0) * np.cos(th)

        if self.transition_width <= 1e-4:
            return self.mu_high if d >= 0.0 else self.mu_low
        s = 1.0 / (1.0 + np.exp(-d / self.transition_width))
        return float(self.mu_low + (self.mu_high - self.mu_low) * s)


@dataclass
class CurvedSplitRoad(RoadSurface):
    """Circular split-mu road dividing friction radially (inner lane vs outer lane).

    Used for cornering on split-mu (ISO 7975 combined slip in a turn):
    - Radius < R_split: mu_inner (e.g. inside ice 0.20 or inside asphalt 1.0)
    - Radius >= R_split: mu_outer (e.g. outside asphalt 1.0 or outside ice 0.20)
    """

    mu_inner: float = 0.20
    mu_outer: float = 1.0
    center_x: float = 0.0
    center_y: float = 0.0
    radius_split_m: float = 80.0
    transition_width: float = 0.20

    def get_friction(self, X: float, Y: float) -> float:
        r = np.hypot(X - self.center_x, Y - self.center_y)
        dr = r - self.radius_split_m
        if self.transition_width <= 1e-4:
            return self.mu_outer if dr >= 0.0 else self.mu_inner
        s = 1.0 / (1.0 + np.exp(-dr / self.transition_width))
        return float(self.mu_inner + (self.mu_outer - self.mu_inner) * s)


@dataclass
class CheckerboardRoad(RoadSurface):
    """Checkerboard road with alternating periodic high-mu and low-mu rectangular patches.

    Simulates uneven, heavily degraded road surfaces, potholes, or alternating ice patches.
    """

    mu_high: float = 1.0
    mu_low: float = 0.25
    patch_length_x: float = 15.0  # Length along X [m]
    patch_width_y: float = 1.8    # Width along Y [m]

    def get_friction(self, X: float, Y: float) -> float:
        idx_x = int(np.floor(X / self.patch_length_x))
        idx_y = int(np.floor(Y / self.patch_width_y))
        is_high = ((idx_x + idx_y) % 2 == 0)
        return float(self.mu_high if is_high else self.mu_low)



@dataclass
class GradedRoad(RoadSurface):
    """Friction surface ``base`` with a longitudinal road grade along the world X axis (Plan V4.0 M22).

    Convention: ``segments`` are ``(x_start, grade)`` pairs sorted by ``x_start``; ``grade`` is the rise per metre
    in the +X direction (``-0.10`` = 10 % downhill when driving towards +X); the road is level before the first
    segment. Each grade change is a smooth (half-cosine) transition of ``transition`` metres, so slope and height
    ``Z(X)`` are continuous. No cross slope: ``dZ/dY = 0``. The vehicle model reads ``slope`` only from roads with
    ``has_grade``; every other road is level and its results are unchanged.
    """

    base: RoadSurface = field(default_factory=UniformRoad)
    segments: tuple = ()
    transition: float = 10.0
    has_grade = True

    def __post_init__(self):
        xs = [float(x) for x, _ in self.segments]
        if xs != sorted(xs):
            raise ValueError("grade segments must be sorted by x_start")
        if self.transition <= 0.0:
            raise ValueError("grade transition must be > 0")
        g_prev, steps = 0.0, []
        for x0, g in self.segments:
            steps.append((float(x0), float(g) - g_prev))
            g_prev = float(g)
        self._steps = tuple(steps)

    def get_friction(self, X: float, Y: float) -> float:
        return self.base.get_friction(X, Y)

    def get_friction_batch(self, X: np.ndarray, Y: np.ndarray) -> np.ndarray:
        return self.base.get_friction_batch(X, Y)

    def grade(self, X: float) -> float:
        """Rise per metre in +X at ``X``."""
        T, s = self.transition, 0.0
        for x0, dg in self._steps:
            u = min(max((X - x0) / T, 0.0), 1.0)
            s += dg * (0.5 - 0.5 * np.cos(np.pi * u))
        return float(s)

    def slope(self, X: float, Y: float) -> tuple[float, float]:
        """``(dZ/dX, dZ/dY)`` at the point."""
        return self.grade(X), 0.0

    def height(self, X: float) -> float:
        """Road height ``Z(X)`` [m], 0 on the level start."""
        T, z = self.transition, 0.0
        for x0, dg in self._steps:
            d = X - x0
            if d <= 0.0:
                continue
            z += dg * (T * (0.5 * min(d / T, 1.0) - np.sin(np.pi * min(d / T, 1.0)) / (2.0 * np.pi)) + max(d - T, 0.0))
        return float(z)
