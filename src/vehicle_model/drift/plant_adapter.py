"""Deterministic evaluation of the existing 7-DOF plant for root finding and linearisation (Drift plan D1, 1.2).

:meth:`SevenDOFModel.evaluate_dynamics` starts its load-transfer fixed point from
``model._accel_guess`` and overwrites it with the result: the derivative at a state then
depends (within the iteration tolerance) on which state was evaluated before. For a root
solver or a finite-difference Jacobian that makes results order dependent.

:class:`DeterministicPlant` evaluates the *same* model class and parameters without that
side effect:

* it works on its own shallow copy of the model; the caller's model is never touched;
* before every evaluation the start value of the fixed point is reset to a fixed value
  (``accel_guess``), so equal inputs give bit-identical outputs in any call order;
* the fixed-point tolerance is tightened (default 1e-12 m/s^2, max 500 iterations), so the
  result is also independent of the start value to rounding level;
* a non-converged load transfer is reported in every result (``load_transfer_converged``)
  and counted; with ``strict=True`` it raises :class:`LoadTransferNotConverged`.

No forces are computed here: everything comes from ``evaluate_dynamics`` of the plant.
"""

from __future__ import annotations

import copy
from dataclasses import dataclass

import numpy as np

from vehicle_model.core.types import SevenDOFControlInput

NAV = (7, 8, 9)          # X, Y, psi in the state array
BODY_NAMES = ("vx", "vy", "r", "omega_FL", "omega_FR", "omega_RL", "omega_RR")


class LoadTransferNotConverged(RuntimeError):
    """The quasi-static load-transfer fixed point did not converge (strict mode)."""


@dataclass(frozen=True)
class PlantEvaluation:
    derivative: np.ndarray        # full state derivative (read-only)
    telemetry: object             # WheelTelemetry of the plant
    ax: float                     # [m/s^2] body-frame accelerations from the tyre forces
    ay: float
    load_transfer_converged: bool

    @property
    def body_derivative(self) -> np.ndarray:
        """Derivative of the body states (vx, vy, r, omega x4 and any appended states)."""
        keep = [i for i in range(self.derivative.size) if i not in NAV]
        return self.derivative[keep]


class DeterministicPlant:
    """Order-independent, side-effect-free evaluation of a SevenDOFModel-family plant."""

    def __init__(self, model, load_transfer_tol: float = 1e-12, max_iter: int = 500,
                 accel_guess: tuple[float, float] = (0.0, 0.0), strict: bool = False):
        self.model = copy.copy(model)
        self.model.load_transfer_tol = float(load_transfer_tol)
        self.model.load_transfer_max_iter = int(max_iter)
        self.accel_guess = (float(accel_guess[0]), float(accel_guess[1]))
        self.strict = bool(strict)
        self.evaluations = 0
        self.not_converged = 0

    @property
    def params(self):
        return self.model.params

    @property
    def n_states(self) -> int:
        return self.model.n_states

    def state(self, y):
        """State object of the plant from a full state array."""
        return self.model.state_type.from_array(np.asarray(y, dtype=float))

    def evaluate(self, y, u: SevenDOFControlInput) -> PlantEvaluation:
        """Derivative and telemetry at the full state array ``y`` with input ``u`` (not clamped)."""
        self.model._accel_guess = self.accel_guess
        d, tel, ax, ay = self.model.evaluate_dynamics(self.state(y), u)
        self.evaluations += 1
        conv = bool(tel.load_transfer_converged)
        if not conv:
            self.not_converged += 1
            if self.strict:
                raise LoadTransferNotConverged(f"load transfer not converged at state {np.asarray(y).tolist()}")
        d = np.array(d, dtype=float)
        d.setflags(write=False)
        return PlantEvaluation(d, tel, float(ax), float(ay), conv)

    def body_state(self, y) -> np.ndarray:
        return np.array([v for i, v in enumerate(np.asarray(y, float)) if i not in NAV])

    def full_state(self, q, nav=(0.0, 0.0, 0.0)) -> np.ndarray:
        """Full state array from body states ``q`` (7 or more) and ``(X, Y, psi)``."""
        q = np.asarray(q, dtype=float)
        y = np.zeros(self.n_states)
        body = [i for i in range(self.n_states) if i not in NAV]
        y[body] = q
        y[list(NAV)] = nav
        return y


__all__ = ("BODY_NAMES", "DeterministicPlant", "LoadTransferNotConverged", "PlantEvaluation")
