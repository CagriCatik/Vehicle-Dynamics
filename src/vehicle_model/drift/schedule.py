"""Gain schedule along a branch of drift equilibria (Drift plan D4 exit/recovery, D5 set-point changes).

At fixed speed and friction the drift equilibria form a branch over the sideslip angle. For
13 m/s, mu 0.9 it is continuous from beta = -1 deg (near-limit cornering with spinning inner
rear wheel) to -25 deg, open-loop unstable and stabilisable everywhere (DRIFT_RESULTS.md, D4).
Straight driving lies on another branch (stable cornering over the steer angle); the two come
close at the small-|beta| end, where the handover to the cornering LQR takes place.

:class:`BranchSchedule` precomputes equilibrium and LQR at a grid of beta values (continuation,
drift criterion off, since the small-|beta| end is cornering) and interpolates ``q*``, ``u*`` and
``K`` linearly in beta. Moving the reference slowly along the branch keeps the state near the
current operating point; the rate is a parameter, the result is checked in simulation.
"""

from __future__ import annotations

import math

import numpy as np

from .equilibrium import DriftEquilibriumSolver, EquilibriumSettings, cornering_equilibrium
from .linearization import linearise
from .lqr import design_lqr
from .types import DriftReference


class BranchSchedule:
    def __init__(self, model, speed: float, mu: float, betas_deg, weights=None):
        betas = sorted((float(b) for b in betas_deg), key=abs)
        if len(betas) < 2 or len({math.copysign(1.0, b) for b in betas}) != 1:
            raise ValueError("the schedule needs at least two beta values of one sign")
        self.speed, self.mu = float(speed), float(mu)
        solver = DriftEquilibriumSolver(model, EquilibriumSettings(require_drift=False))
        eqs = solver.sweep([DriftReference(self.speed, math.radians(b), self.mu) for b in betas])
        bad = [(b, e.reasons) for b, e in zip(betas, eqs) if not e.physical_feasible]
        if bad:
            raise ValueError(f"branch interrupted (no equilibrium) at {bad}")
        self.betas = np.radians(betas)                  # increasing |beta|
        self.equilibria = eqs
        self.lqrs = [design_lqr(linearise(solver.plant(self.mu), e), model.params, weights) for e in eqs]
        self.q = np.array([e.body_state() for e in eqs])
        self.u = np.array([[e.steer_front, e.wheel_drive_torque[2]] for e in eqs])
        self.K = np.array([l.K for l in self.lqrs])
        self.model = model

    @property
    def beta_range(self) -> tuple[float, float]:
        return float(self.betas[0]), float(self.betas[-1])

    def point(self, beta: float):
        """``(q*, u*, K)`` interpolated at ``beta`` (clamped to the branch)."""
        x = np.abs(self.betas)
        b = float(np.clip(abs(beta), x[0], x[-1]))
        i = int(np.clip(np.searchsorted(x, b) - 1, 0, len(x) - 2))
        w = (b - x[i]) / (x[i + 1] - x[i])
        return ((1 - w) * self.q[i] + w * self.q[i + 1], (1 - w) * self.u[i] + w * self.u[i + 1],
                (1 - w) * self.K[i] + w * self.K[i + 1])

    def end_cornering(self):
        """Stable cornering equilibrium at the steer angle of the small-|beta| end of the branch (handover target)."""
        return cornering_equilibrium(self.model, self.speed, float(self.u[0, 0]), self.mu)


__all__ = ("BranchSchedule",)
