"""Bounded allocation of longitudinal wheel forces to a total force and yaw moment.

Actuators: the longitudinal tyre force ``F_i`` of each wheel (wheel frame, rows
FL, FR, RL, RR). Demand ``v = (F_x, M_z)`` in the body frame:

    F_x = sum_i cos(d_i) F_i,     M_z = sum_i (x_i sin(d_i) - y_i cos(d_i)) F_i

Bounds per wheel (all enforced, none silently):

    available grip   F_avail = margin * sqrt(max((mu Fz)^2 - Fy^2, 0))   (friction circle minus the
                                                                          lateral force in use)
    upper bound      min(T_drive_max / R, F_avail)  if the wheel is driven and its drive is healthy, else 0
    lower bound     -min(T_brake_max / R, F_avail)  if its brake is healthy, else 0

Priority: yaw moment before longitudinal force (stability before the driver's
acceleration request), set by the weights ``W_v = (1/F_ref, k_prio/M_ref)``;
among equally good solutions ``sum F_i^2 / F_avail_i`` is minimised, which
distributes a force demand in proportion to the available grip (equal
utilisation of all wheels, i.e. they reach their limit together). Wheel torque: ``T_i = F_i R`` (quasi-static: wheel inertia and slip
dynamics are left to the slip controllers).

Status of every allocation:
    nominal     demand met, no bound active
    saturated   demand met, at least one bound active
    infeasible  demand not achievable within the bounds; ``residual`` says by how much
Invalid requests (non-finite values) raise ``ValueError``.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .wls import solve_box_wls

STATUS_NOMINAL = "nominal"
STATUS_SATURATED = "saturated"
STATUS_INFEASIBLE = "infeasible"


@dataclass(frozen=True)
class AllocatorConfig:
    driven: tuple[bool, bool, bool, bool] = (True, True, True, True)  # which wheels have a drive
    max_drive_torque: float = 800.0     # [N m] per wheel
    max_brake_torque: float = 2500.0    # [N m] per wheel
    grip_margin: float = 0.95           # [-] usable share of the remaining friction circle
    yaw_priority: float = 100.0         # [-] weight ratio M_z : F_x after normalisation
    force_ref: float = 1000.0           # [N]    normalisation of F_x errors
    moment_ref: float = 1000.0          # [N m]  normalisation of M_z errors
    rel_tol: float = 0.01               # [-] demand met if |residual| <= rel_tol |demand| + abs_tol
    abs_tol: tuple[float, float] = (10.0, 10.0)  # [N, N m]


@dataclass(frozen=True)
class ActuatorHealth:
    """Per-wheel actuator availability (False = failed, contributes no force).

    ``drive_scale`` / ``brake_scale`` (compatible extension): remaining fraction of the torque
    limit of a healthy but limited actuator (1.0 = full; the defaults leave every bound unchanged)."""

    drive_ok: tuple[bool, bool, bool, bool] = (True, True, True, True)
    brake_ok: tuple[bool, bool, bool, bool] = (True, True, True, True)
    drive_scale: tuple[float, float, float, float] = (1.0, 1.0, 1.0, 1.0)
    brake_scale: tuple[float, float, float, float] = (1.0, 1.0, 1.0, 1.0)


@dataclass(frozen=True)
class AllocationResult:
    F: np.ndarray               # [N] longitudinal wheel forces
    T_drive: np.ndarray         # [N m] >= 0
    T_brake: np.ndarray         # [N m] >= 0
    demand: np.ndarray          # (F_x, M_z)
    achieved: np.ndarray        # (F_x, M_z)
    residual: np.ndarray        # achieved - demand
    lower: np.ndarray           # [N] bounds used
    upper: np.ndarray
    at_bound: np.ndarray        # bool per wheel
    grip_limited: np.ndarray    # bool per wheel: active bound is the friction limit
    status: str
    diagnostics: dict = field(default_factory=dict)


class WheelForceAllocator:
    """Allocates ``(F_x, M_z)`` to four longitudinal wheel forces within actuator and grip limits."""

    def __init__(self, positions: np.ndarray, Reff: float, config: AllocatorConfig | None = None):
        self.positions = np.asarray(positions, dtype=float).reshape(4, 2)
        self.Reff = float(Reff)
        self.config = config or AllocatorConfig()

    def effectiveness(self, steer: np.ndarray) -> np.ndarray:
        """Matrix B (2x4): generalised force per unit longitudinal wheel force."""
        c, s = np.cos(steer), np.sin(steer)
        x, y = self.positions[:, 0], self.positions[:, 1]
        return np.vstack([c, x * s - y * c])

    def bounds(self, Fz, mu, Fy, health: ActuatorHealth | None = None):
        cfg = self.config
        health = health or ActuatorHealth()
        avail = cfg.grip_margin * np.sqrt(np.maximum((np.asarray(mu) * np.asarray(Fz)) ** 2 - np.asarray(Fy) ** 2, 0.0))
        drive = np.where(np.array(cfg.driven) & np.array(health.drive_ok), cfg.max_drive_torque / self.Reff, 0.0)
        brake = np.where(np.array(health.brake_ok), cfg.max_brake_torque / self.Reff, 0.0)
        if health.drive_scale != (1.0, 1.0, 1.0, 1.0) or health.brake_scale != (1.0, 1.0, 1.0, 1.0):
            drive = drive * np.clip(np.asarray(health.drive_scale, float), 0.0, 1.0)
            brake = brake * np.clip(np.asarray(health.brake_scale, float), 0.0, 1.0)
        upper = np.minimum(drive, avail)
        lower = -np.minimum(brake, avail)
        return lower, upper, avail, drive, brake

    def allocate(self, Fx_demand: float, Mz_demand: float | None, steer, Fz, mu, Fy=None,
                 health: ActuatorHealth | None = None, share=None, brake_limit=None,
                 drive_limit=None) -> AllocationResult:
        """``share`` (4, > 0): optional distribution weights replacing the available grip in the secondary
        objective (``F_i`` proportional to ``share_i`` among equally good solutions); ``brake_limit`` (4, >= 0) [N]:
        optional additional bound on the braking force per wheel. Both come from a brake-distribution
        strategy (EBD); ``drive_limit`` (4, >= 0) [N]: optional additional bound on the drive
        force per wheel (traction distribution / eLSD / motor envelope, traction control). Without them the allocation is
        unchanged."""
        cfg = self.config
        free_yaw = Mz_demand is None          # no yaw request: M_z unweighted (reported as achieved, residual 0)
        demand = np.array([Fx_demand, 0.0 if free_yaw else Mz_demand], dtype=float)
        steer = np.asarray(steer, dtype=float).reshape(4)
        Fz, mu = np.asarray(Fz, dtype=float).reshape(4), np.asarray(mu, dtype=float).reshape(4)
        Fy = np.zeros(4) if Fy is None else np.asarray(Fy, dtype=float).reshape(4)
        if not (np.all(np.isfinite(demand)) and np.all(np.isfinite(steer)) and np.all(np.isfinite(Fz))
                and np.all(np.isfinite(mu)) and np.all(np.isfinite(Fy))):
            raise ValueError("allocation request contains non-finite values")

        lower, upper, avail, drive, brake = self.bounds(Fz, mu, Fy, health)
        if brake_limit is not None:
            bl = np.asarray(brake_limit, dtype=float).reshape(4)
            if not (np.all(np.isfinite(bl)) and np.all(bl >= 0.0)):
                raise ValueError("brake_limit must be finite and >= 0")
            lower = np.maximum(lower, -bl)
        if drive_limit is not None:
            dl = np.asarray(drive_limit, dtype=float).reshape(4)
            if not (np.all(np.isfinite(dl)) and np.all(dl >= 0.0)):
                raise ValueError("drive_limit must be finite and >= 0")
            upper = np.minimum(upper, dl)
        B = self.effectiveness(steer)
        W_v = np.array([1.0 / cfg.force_ref, 0.0 if free_yaw else cfg.yaw_priority / cfg.moment_ref])
        weight = avail if share is None else np.asarray(share, dtype=float).reshape(4) * np.mean(np.maximum(avail, 1.0))
        if share is not None and not (np.all(np.isfinite(weight)) and np.all(weight > 0.0)):
            raise ValueError("share must be finite and > 0")
        W_u = 1.0 / np.sqrt(np.maximum(weight, 1.0))  # sum F^2/w -> F proportional to w (default w = F_avail)
        sol = solve_box_wls(B, demand, lower, upper, W_v=W_v, W_u=W_u, eps=1e-9)  # bias on demands ~1e-7 rel.

        F = sol.u
        residual = sol.residual.copy()
        if free_yaw:
            demand[1] = sol.achieved[1]
            residual[1] = 0.0
        at_bound = (sol.at_lower | sol.at_upper) & (upper > lower)
        grip_limited = (sol.at_upper & (avail <= drive)) | (sol.at_lower & (avail <= brake))
        tol = cfg.rel_tol * np.abs(demand) + np.array(cfg.abs_tol)
        if np.any(np.abs(residual) > tol):
            status = STATUS_INFEASIBLE
        elif np.any(at_bound) or np.any(upper == lower):
            status = STATUS_SATURATED
        else:
            status = STATUS_NOMINAL
        T = F * self.Reff
        return AllocationResult(
            F=F, T_drive=np.maximum(T, 0.0), T_brake=np.maximum(-T, 0.0), demand=demand,
            achieved=sol.achieved, residual=residual, lower=lower, upper=upper,
            at_bound=at_bound, grip_limited=grip_limited & at_bound, status=status,
            diagnostics={"available_grip": avail, "utilisation": np.abs(F) / np.maximum(avail, 1e-9)},
        )


__all__ = (
    "ActuatorHealth",
    "AllocationResult",
    "AllocatorConfig",
    "STATUS_INFEASIBLE",
    "STATUS_NOMINAL",
    "STATUS_SATURATED",
    "WheelForceAllocator",
)
