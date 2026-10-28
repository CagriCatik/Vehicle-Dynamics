"""Quarter-car vertical suspension dynamics model.

Implements the standard 2-DOF quarter-car system:
- Sprung mass (chassis 1/4) vertical bounce: z_s
- Unsprung mass (wheel/hub) vertical hop: z_u
- Road profile excitation: z_r(t)

Equations of motion:
    m_s * ddz_s = -k_s * (z_s - z_u) - F_damper + F_act
    m_u * ddz_u =  k_s * (z_s - z_u) + F_damper - k_t * (z_u - z_r) - c_t * (dzu - dzr) - F_act

All coordinates are deviations from static equilibrium. Tyre contact is
unilateral: the contact force F_c = max(0, F_static - k_t (z_u - z_r) - c_t (dzu - dzr))
enters the unsprung equation, so a lifted wheel feels no tyre force.

Conforms to Milestone M7 (Plan.md §9, §19 M7).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Tuple

import numpy as np
import yaml


@dataclass
class QuarterCarConfig:
    """Quarter-car physical parameters."""

    sprung_mass: float = 375.0  # kg
    unsprung_mass: float = 45.0  # kg
    suspension_stiffness: float = 35000.0  # N/m
    suspension_damping: float = 2200.0  # Ns/m
    tyre_stiffness: float = 210000.0  # N/m
    tyre_damping: float = 250.0  # Ns/m
    rattle_space_max: float = 0.08  # m
    rattle_space_min: float = -0.08  # m
    bump_stop_stiffness: float = 150000.0  # N/m

    @classmethod
    def from_yaml(cls, path: str | Path) -> "QuarterCarConfig":
        """Load configuration from a YAML file."""
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
        return cls(
            sprung_mass=float(data.get("sprung_mass", 375.0)),
            unsprung_mass=float(data.get("unsprung_mass", 45.0)),
            suspension_stiffness=float(data.get("suspension_stiffness", 35000.0)),
            suspension_damping=float(data.get("suspension_damping", 2200.0)),
            tyre_stiffness=float(data.get("tyre_stiffness", 210000.0)),
            tyre_damping=float(data.get("tyre_damping", 250.0)),
            rattle_space_max=float(data.get("rattle_space_max", 0.08)),
            rattle_space_min=float(data.get("rattle_space_min", -0.08)),
            bump_stop_stiffness=float(data.get("bump_stop_stiffness", 150000.0)),
        )


@dataclass
class QuarterCarState:
    """State vector of the 2-DOF quarter-car model."""

    zs: float = 0.0  # Sprung mass vertical displacement [m]
    dzs: float = 0.0  # Sprung mass vertical velocity [m/s]
    zu: float = 0.0  # Unsprung mass vertical displacement [m]
    dzu: float = 0.0  # Unsprung mass vertical velocity [m/s]

    def as_array(self) -> np.ndarray:
        return np.array([self.zs, self.dzs, self.zu, self.dzu], dtype=np.float64)

    @classmethod
    def from_array(cls, arr: np.ndarray) -> "QuarterCarState":
        return cls(zs=float(arr[0]), dzs=float(arr[1]), zu=float(arr[2]), dzu=float(arr[3]))


@dataclass
class QuarterCarOutputs:
    """Telemetry outputs from quarter car evaluation."""

    zs: float
    dzs: float
    ddzs: float  # Sprung mass acceleration [m/s^2] (ride comfort)
    zu: float
    dzu: float
    ddzu: float  # Unsprung mass acceleration [m/s^2]
    suspension_deflection: float  # zs - zu [m] (rattle space)
    dynamic_tyre_load: float  # Contact force variation total_tyre_load - static load [N] (>= -static)
    total_tyre_load: float  # Total contact normal load [N], >= 0
    in_contact: bool  # False while the wheel is lifted off the road
    spring_force: float  # [N]
    damping_force: float  # [N]
    active_force: float  # [N]
    road_z: float  # [m]
    road_dz: float  # [m/s]


class QuarterCarModel:
    """Quarter-car suspension simulation model with RK4 numerical integration."""

    def __init__(self, config: QuarterCarConfig | None = None) -> None:
        self.cfg = config or QuarterCarConfig()

    @property
    def static_tyre_load(self) -> float:
        """Total static normal load on tyre at rest [N]."""
        g = 9.80665
        return (self.cfg.sprung_mass + self.cfg.unsprung_mass) * g

    def modal_analysis(self) -> Tuple[float, float, float, float]:
        """Compute natural frequencies and damping ratios of the unforced system.

        Returns:
            f_bounce (Hz): Sprung mass bounce frequency (~1.0 - 1.5 Hz)
            f_hop (Hz): Unsprung mass wheel hop frequency (~10.0 - 14.0 Hz)
            zeta_bounce: Damping ratio of bounce mode
            zeta_hop: Damping ratio of hop mode
        """
        ms = self.cfg.sprung_mass
        mu = self.cfg.unsprung_mass
        ks = self.cfg.suspension_stiffness
        cs = self.cfg.suspension_damping
        kt = self.cfg.tyre_stiffness
        ct = self.cfg.tyre_damping

        # Linear state matrix A (4x4)
        # States: x = [zs, dzs, zu, dzu]^T
        A = np.array([
            [0.0, 1.0, 0.0, 0.0],
            [-ks / ms, -cs / ms, ks / ms, cs / ms],
            [0.0, 0.0, 0.0, 1.0],
            [ks / mu, cs / mu, -(ks + kt) / mu, -(cs + ct) / mu],
        ], dtype=np.float64)

        eigvals = np.linalg.eigvals(A)
        # Group conjugate pairs
        modes = []
        for ev in eigvals:
            if ev.imag >= 0:
                wn = abs(ev)
                fn = wn / (2.0 * math.pi)
                zeta = -ev.real / wn if wn > 1e-9 else 0.0
                modes.append((fn, zeta))

        # Sort by frequency
        modes.sort(key=lambda m: m[0])
        f_bounce, zeta_bounce = modes[0]
        f_hop, zeta_hop = modes[1]
        return f_bounce, f_hop, zeta_bounce, zeta_hop

    def evaluate_derivatives(
        self,
        state: QuarterCarState,
        road_z: float,
        road_dz: float,
        active_force: float = 0.0,
        damper_c: float | None = None,
    ) -> Tuple[np.ndarray, QuarterCarOutputs]:
        """Compute state derivatives and diagnostic telemetry.

        Args:
            state: Current quarter car state
            road_z: Road vertical elevation [m]
            road_dz: Road vertical velocity [m/s]
            active_force: Controllable actuator force between unsprung and sprung mass [N]
            damper_c: Optional override for suspension damping coefficient (e.g. from semi-active control)
        """
        cfg = self.cfg
        cs = damper_c if damper_c is not None else cfg.suspension_damping

        deflection = state.zs - state.zu  # > 0 means suspension is extended (rebound)
        rel_vel = state.dzs - state.dzu

        # Spring force (including bump stops on extreme deflection)
        spring_force = cfg.suspension_stiffness * deflection
        if deflection > cfg.rattle_space_max:
            spring_force += cfg.bump_stop_stiffness * (deflection - cfg.rattle_space_max)
        elif deflection < cfg.rattle_space_min:
            spring_force += cfg.bump_stop_stiffness * (deflection - cfg.rattle_space_min)

        damping_force = cs * rel_vel
        suspension_force = spring_force + damping_force - active_force

        # Tyre vertical force
        tyre_deflection = state.zu - road_z
        tyre_rel_vel = state.dzu - road_dz
        # Unilateral contact: upward force on the wheel, never negative
        total_tyre_load = max(0.0, self.static_tyre_load - cfg.tyre_stiffness * tyre_deflection
                              - cfg.tyre_damping * tyre_rel_vel)
        dyn_tyre_load = total_tyre_load - self.static_tyre_load

        # Equations of motion
        ddzs = -suspension_force / cfg.sprung_mass
        ddzu = (suspension_force + dyn_tyre_load) / cfg.unsprung_mass


        derivatives = np.array([state.dzs, ddzs, state.dzu, ddzu], dtype=np.float64)

        outputs = QuarterCarOutputs(
            zs=state.zs,
            dzs=state.dzs,
            ddzs=ddzs,
            zu=state.zu,
            dzu=state.dzu,
            ddzu=ddzu,
            suspension_deflection=deflection,
            dynamic_tyre_load=dyn_tyre_load,
            total_tyre_load=total_tyre_load,
            in_contact=total_tyre_load > 0.0,
            spring_force=spring_force,
            damping_force=damping_force,
            active_force=active_force,
            road_z=road_z,
            road_dz=road_dz,
        )
        return derivatives, outputs

    def step(
        self,
        state: QuarterCarState,
        road_z: float,
        road_dz: float,
        dt: float,
        active_force: float = 0.0,
        damper_c: float | None = None,
    ) -> Tuple[QuarterCarState, QuarterCarOutputs]:
        """Advance the system state by dt using classic Runge-Kutta 4th order.

        The road input is held constant over the step (zero-order hold), which delays
        the excitation by dt/2 and makes energy bookkeeping first-order accurate; use
        :meth:`step_with_road` for a road given as a function of time.
        """
        x0 = state.as_array()

        k1, out = self.evaluate_derivatives(state, road_z, road_dz, active_force, damper_c)

        s2 = QuarterCarState.from_array(x0 + 0.5 * dt * k1)
        k2, _ = self.evaluate_derivatives(s2, road_z, road_dz, active_force, damper_c)

        s3 = QuarterCarState.from_array(x0 + 0.5 * dt * k2)
        k3, _ = self.evaluate_derivatives(s3, road_z, road_dz, active_force, damper_c)

        s4 = QuarterCarState.from_array(x0 + dt * k3)
        k4, _ = self.evaluate_derivatives(s4, road_z, road_dz, active_force, damper_c)

        x_next = x0 + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
        next_state = QuarterCarState.from_array(x_next)

        return next_state, out

    def step_with_road(
        self,
        state: QuarterCarState,
        t: float,
        dt: float,
        road,
        active_force: float = 0.0,
        damper_c: float | None = None,
    ) -> Tuple[QuarterCarState, QuarterCarOutputs]:
        """RK4 step with the road ``road(t) -> (z_r, z_r')`` evaluated at the stage times t, t+dt/2, t+dt."""
        x0 = state.as_array()
        z0, z_half, z1 = road(t), road(t + 0.5 * dt), road(t + dt)
        k1, out = self.evaluate_derivatives(state, *z0, active_force, damper_c)
        k2, _ = self.evaluate_derivatives(QuarterCarState.from_array(x0 + 0.5 * dt * k1), *z_half, active_force, damper_c)
        k3, _ = self.evaluate_derivatives(QuarterCarState.from_array(x0 + 0.5 * dt * k2), *z_half, active_force, damper_c)
        k4, _ = self.evaluate_derivatives(QuarterCarState.from_array(x0 + dt * k3), *z1, active_force, damper_c)
        return QuarterCarState.from_array(x0 + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)), out
