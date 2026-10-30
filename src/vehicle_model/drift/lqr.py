"""LQR around a drift equilibrium: ``u = u* - K (q - q*)``.

Two variants:

* ``steer_rate=False``: inputs ``[delta_f, T]`` directly (the plan's basic form).
* ``steer_rate=True`` (default): the front steer becomes a state and its rate the input,
  ``x_a = [dq, d delta_f]``, ``u_a = [delta_f_dot, dT]`` with
  ``A_a = [[A, b_delta], [0, 0]]``, ``B_a = [[0, b_T], [1, 0]]``. The steering rate is then
  weighted against the vehicle's rate limit inside the design. Reason (measured): the basic
  design asks for up to 4200 deg/s road-wheel steer rate after a 0.1 rad/s yaw disturbance
  (95th percentile ~750 deg/s), the vehicle allows 50 deg/s; clipped afterwards, the
  limit binds almost all the time and the drift is lost.

Design in scaled coordinates ``x = D_q^-1 dq`` (typical magnitudes, :data:`.linearization.STATE_SCALE`)
and ``v = D_u^-1 du`` (actuator limits), so the weights are dimensionless:

    min int x'Q x + v'R v dt,   Q = diag(q_scaled) (7), R = diag(r_scaled) (2),
    K_s = R^-1 B_s' P with P the stabilising solution of the continuous algebraic Riccati
    equation for (A_s, B_s) (``scipy.linalg.solve_continuous_are``),
    K = D_u K_s D_q^-1 in physical units.

Integral action (``integral=True``, requires the steer-rate form): one more state
``z = int (beta - beta*) dt`` with ``dz/dt ~ c' dq``, ``c = [-vy*, vx*, 0, ...] / v*^2`` (the
linearised sideslip), scaled by ``Z_SCALE`` and weighted with ``q_integral``. It removes the
steady sideslip offset that a plant deviating from the design model leaves (up to 4.8 deg);
the speed is then free to settle elsewhere (two inputs, one integrated output).

A design is refused (``NotStabilisable``) if the linearisation fails the PBH test for an
unstable mode; weight vectors of the wrong length fail hard. The closed-loop eigenvalues of
the linear model are reported; they say nothing about the region in which the nonlinear
plant is held (that is measured in the closed-loop simulations).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.linalg import solve_continuous_are

from .linearization import STATE_SCALE, DriftLinearisation, input_scale


class NotStabilisable(ValueError):
    """An unstable mode of the linearisation is not reachable by the inputs (PBH test failed)."""


@dataclass(frozen=True)
class LQRWeights:
    q_scaled: tuple = (8.0, 10.0, 25.0, 0.2, 0.2, 1.0, 1.0)     # vx, vy, r, omega FL FR RL RR
    r_scaled: tuple = (5.0, 1.0)                               # delta_f (or its rate), rear torque
    q_steer: float = 1.0                                       # steer deviation (steer-rate design only)
    q_integral: float = 10.0                                   # integral of the sideslip error (integral design only)

    def __post_init__(self):
        if len(self.q_scaled) != 7 or len(self.r_scaled) != 2:
            raise ValueError(f"LQR weights need 7 state and 2 input weights, got {len(self.q_scaled)} and "
                             f"{len(self.r_scaled)}")
        if min(self.q_scaled) < 0.0 or min(self.r_scaled) <= 0.0 or self.q_steer < 0.0 or self.q_integral < 0.0:
            raise ValueError("state weights must be >= 0 and input weights > 0")


@dataclass(frozen=True)
class DriftLQR:
    K: np.ndarray                    # (2, 7), with steer_rate (2, 8), with integral (2, 9): rows [delta_f (rate), T]
    closed_loop_eigenvalues: np.ndarray
    weights: LQRWeights
    steer_rate: bool = False
    integral: bool = False

    @property
    def stable(self) -> bool:
        return bool(np.all(self.closed_loop_eigenvalues.real < 0.0))


def augmented(lin: DriftLinearisation):
    """``(A_a, B_a)`` with states ``[dq, d delta_f]`` and inputs ``[delta_f_dot, dT]``."""
    A_a = np.zeros((8, 8))
    A_a[:7, :7] = lin.A
    A_a[:7, 7] = lin.B[:, 0]
    B_a = np.zeros((8, 2))
    B_a[:7, 1] = lin.B[:, 1]
    B_a[7, 0] = 1.0
    return A_a, B_a


Z_SCALE = 0.05   # [rad s] typical magnitude of the sideslip-error integral


def sideslip_row(lin: DriftLinearisation) -> np.ndarray:
    """``c`` with ``d beta ~ c' dq`` at the operating point (``q = [vx, vy, r, omega x4]``)."""
    vx, vy = float(lin.q_star[0]), float(lin.q_star[1])
    c = np.zeros(7)
    c[0], c[1] = -vy / (vx * vx + vy * vy), vx / (vx * vx + vy * vy)
    return c


def design_lqr(lin: DriftLinearisation, params, weights: LQRWeights | None = None, steer_rate: bool = True,
               integral: bool = False) -> DriftLQR:
    w = weights or LQRWeights()
    if integral and not steer_rate:
        raise ValueError("integral action is implemented for the steer-rate design only")
    if not lin.stabilisable:
        raise NotStabilisable(f"unstable modes {lin.unstable} with PBH ranks {lin.pbh_ranks} (< 7)")
    us = input_scale(params)
    if steer_rate:
        A, B = augmented(lin)
        Ds = np.diag(np.r_[STATE_SCALE, us[0]])
        Du = np.diag([params.max_steer_rate, us[1]])
        Q = np.diag(np.r_[w.q_scaled, w.q_steer])
        if integral:
            A = np.block([[A, np.zeros((8, 1))], [np.r_[sideslip_row(lin), 0.0][None, :], np.zeros((1, 1))]])
            B = np.vstack([B, np.zeros((1, 2))])
            Ds = np.diag(np.r_[np.diag(Ds), Z_SCALE])
            Q = np.diag(np.r_[np.diag(Q), w.q_integral])
    else:
        A, B = lin.A, lin.B
        Ds, Du = np.diag(STATE_SCALE), np.diag(us)
        Q = np.diag(w.q_scaled)
    As, Bs = np.linalg.solve(Ds, A @ Ds), np.linalg.solve(Ds, B @ Du)
    R = np.diag(w.r_scaled)
    P = solve_continuous_are(As, Bs, Q, R)
    Ks = np.linalg.solve(R, Bs.T @ P)
    K = Du @ Ks @ np.linalg.inv(Ds)
    K.setflags(write=False)
    cl = np.linalg.eigvals(A - B @ K)
    return DriftLQR(K, cl[np.argsort(-cl.real)], w, steer_rate, integral)


__all__ = ("Z_SCALE", "DriftLQR", "LQRWeights", "NotStabilisable", "augmented", "design_lqr", "sideslip_row")
