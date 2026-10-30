"""Linear 2-DOF single-track (bicycle) model as analytical handling reference.

Linearised single-track model (platform plan, sections 5 and 6).

States, inputs, outputs (ISO 8855: x forward, y left, z up; angles positive counter-clockwise):
    x = [v_y, r]                lateral velocity [m/s], yaw rate [rad/s]
    u = [delta_f, delta_r]      front / rear axle steer angle [rad], same sign = in-phase
    y = [r, a_y, beta]          yaw rate [rad/s], lateral acceleration [m/s^2], sideslip v_y/v_x [rad]

Equations (constant v_x, small angles, linear axle forces F_y = C * alpha):
    alpha_f = delta_f - (v_y + l_f r) / v_x
    alpha_r = delta_r - (v_y - l_r r) / v_x
    m (v_y_dot + v_x r) = C_f alpha_f + C_r alpha_r
    I_z r_dot           = l_f C_f alpha_f - l_r C_r alpha_r

Validity: |alpha| within the linear tyre range (typically a_y < ~0.4 g), constant speed,
no load transfer, no longitudinal forces, level road, rigid steering.

Closed-form results (Mitschke/Wallentowitz 2014, Kap. 12; Gillespie 1992, Ch. 6):
    understeer gradient     EG = m/L * (l_r/C_f - l_f/C_r)            [rad/(m/s^2)]
    steady yaw gain         r/delta_f = v_x / (L + EG v_x^2)
    characteristic speed    v_char = sqrt(L/EG)      for EG > 0 (maximum of r/delta_f)
    critical speed          v_crit = sqrt(-L/EG)     for EG < 0 (det A = 0, divergent)
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np
from scipy.linalg import expm


@dataclass(frozen=True)
class LinearSingleTrackParams:
    """Parameters of the linear single-track model. All SI units."""

    mass: float          # [kg]
    Iz: float            # [kg m^2] yaw moment of inertia
    lf: float            # [m] CoG to front axle
    lr: float            # [m] CoG to rear axle
    C_f: float           # [N/rad] front AXLE cornering stiffness (both wheels)
    C_r: float           # [N/rad] rear AXLE cornering stiffness (both wheels)

    def __post_init__(self) -> None:
        for name in ("mass", "Iz", "lf", "lr", "C_f", "C_r"):
            if not getattr(self, name) > 0.0:
                raise ValueError(f"{name} must be positive, got {getattr(self, name)!r}")

    @property
    def wheelbase(self) -> float:
        return self.lf + self.lr


@dataclass(frozen=True)
class StateSpace:
    """Continuous LTI system x' = A x + B u, y = C x + D u."""

    A: np.ndarray
    B: np.ndarray
    C: np.ndarray
    D: np.ndarray

    def dc_gain(self) -> np.ndarray:
        """Steady-state gain matrix -C A^-1 B + D (requires an invertible A)."""
        return -self.C @ np.linalg.solve(self.A, self.B) + self.D

    def frequency_response(self, omega: np.ndarray) -> np.ndarray:
        """Complex response H(j omega), shape (len(omega), n_outputs, n_inputs)."""
        n = self.A.shape[0]
        eye = np.eye(n)
        return np.array([self.C @ np.linalg.solve(1j * w * eye - self.A, self.B) + self.D for w in np.asarray(omega, float)])

    def step_response(self, t: np.ndarray, u: np.ndarray) -> np.ndarray:
        """Exact response to a constant input ``u`` applied at t = 0 from x = 0, shape (len(t), n_outputs).

        Uses x(t) = A^-1 (e^{At} - I) B u, which requires an invertible A.
        """
        Bu = self.B @ np.asarray(u, dtype=float)
        n = self.A.shape[0]
        eye = np.eye(n)
        out = []
        for ti in np.asarray(t, dtype=float):
            x = np.linalg.solve(self.A, (expm(self.A * ti) - eye) @ Bu)
            out.append(self.C @ x + self.D @ u)
        return np.array(out)


@dataclass(frozen=True)
class ModalProperties:
    """Eigen-analysis of the linear lateral dynamics at one speed."""

    eigenvalues: np.ndarray         # complex, shape (2,)
    eigenvectors: np.ndarray        # columns, shape (2, 2)
    natural_frequency: float        # [rad/s] sqrt(det A); nan if det A <= 0
    damping_ratio: float            # [-] -tr A / (2 sqrt(det A)); nan if det A <= 0
    stable: bool                    # all Re(lambda) < 0


OUTPUTS = ("r", "a_y", "beta")
INPUTS = ("delta_f", "delta_r")


class LinearSingleTrack:
    """Linear 2-DOF single-track model with front and rear axle steering."""

    def __init__(self, params: LinearSingleTrackParams):
        self.params = params

    # ------------------------------------------------------------ closed form
    @property
    def understeer_gradient(self) -> float:
        """EG = m/L (l_r/C_f - l_f/C_r) [rad/(m/s^2)]; > 0 understeer, < 0 oversteer."""
        p = self.params
        return p.mass / p.wheelbase * (p.lr / p.C_f - p.lf / p.C_r)

    def understeer_gradient_deg_per_g(self, g: float) -> float:
        """Understeer gradient in [deg/g] for the given gravitational acceleration g [m/s^2]."""
        return math.degrees(self.understeer_gradient) * g

    @property
    def characteristic_speed(self) -> float | None:
        """V_char = sqrt(L/EG) [m/s] for an understeering vehicle, else None."""
        eg = self.understeer_gradient
        return math.sqrt(self.params.wheelbase / eg) if eg > 0.0 else None

    @property
    def critical_speed(self) -> float | None:
        """V_crit = sqrt(-L/EG) [m/s] for an oversteering vehicle, else None."""
        eg = self.understeer_gradient
        return math.sqrt(-self.params.wheelbase / eg) if eg < 0.0 else None

    def yaw_rate_gain(self, vx: float) -> float:
        """Steady-state (r/delta_f) [1/s] for front steer only."""
        return vx / (self.params.wheelbase + self.understeer_gradient * vx**2)

    def lateral_accel_gain(self, vx: float) -> float:
        """Steady-state (a_y/delta_f) [m/s^2/rad] for front steer only."""
        return vx * self.yaw_rate_gain(vx)

    def sideslip_gain(self, vx: float) -> float:
        """Steady-state (beta/delta_f) [-] for front steer only.

        beta = (l_r - m l_f v_x^2 / (L C_r)) / (L + EG v_x^2)
        """
        p = self.params
        return (p.lr - p.mass * p.lf * vx**2 / (p.wheelbase * p.C_r)) / (p.wheelbase + self.understeer_gradient * vx**2)

    def steady_state_steer_angle(self, vx: float, radius: float) -> float:
        """Front steer angle [rad] for steady-state circular driving: L/R + EG v_x^2/R."""
        return self.params.wheelbase / radius + self.understeer_gradient * vx**2 / radius

    # ------------------------------------------------------------ state space
    def state_space(self, vx: float) -> StateSpace:
        if vx <= 0.0:
            raise ValueError("The linear single-track model is only defined for v_x > 0 (forward driving)")
        p = self.params
        Cf, Cr, lf, lr, m, Iz = p.C_f, p.C_r, p.lf, p.lr, p.mass, p.Iz
        a11 = -(Cf + Cr) / (m * vx)
        a12 = -(Cf * lf - Cr * lr) / (m * vx) - vx
        a21 = -(Cf * lf - Cr * lr) / (Iz * vx)
        a22 = -(Cf * lf**2 + Cr * lr**2) / (Iz * vx)
        A = np.array([[a11, a12], [a21, a22]])
        B = np.array([[Cf / m, Cr / m], [Cf * lf / Iz, -Cr * lr / Iz]])
        # a_y = v_y_dot + v_x r = (F_yf + F_yr) / m
        C = np.array([
            [0.0, 1.0],
            [a11, a12 + vx],
            [1.0 / vx, 0.0],
        ])
        D = np.array([
            [0.0, 0.0],
            [Cf / m, Cr / m],
            [0.0, 0.0],
        ])
        return StateSpace(A, B, C, D)

    def modal_properties(self, vx: float) -> ModalProperties:
        A = self.state_space(vx).A
        eigvals, eigvecs = np.linalg.eig(A)
        det, tr = float(np.linalg.det(A)), float(np.trace(A))
        if det > 0.0:
            wn = math.sqrt(det)
            zeta = -tr / (2.0 * wn)
        else:
            wn = zeta = float("nan")
        return ModalProperties(
            eigenvalues=eigvals,
            eigenvectors=eigvecs,
            natural_frequency=wn,
            damping_ratio=zeta,
            stable=bool(np.all(eigvals.real < 0.0)),
        )


__all__ = (
    "LinearSingleTrackParams",
    "LinearSingleTrack",
    "StateSpace",
    "ModalProperties",
    "OUTPUTS",
    "INPUTS",
)
