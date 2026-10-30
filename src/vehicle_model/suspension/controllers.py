"""Suspension damping and active suspension controllers.

Includes:
- Passive baseline
- Semi-active Skyhook controller (Karnopp continuous & 2-state)
- Semi-active Groundhook controller
- Hybrid Skyhook/Groundhook controller
- Active suspension force actuatormd estimation robustness).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Tuple

import numpy as np
import yaml

from vehicle_model.suspension.quarter_car import QuarterCarState


@dataclass
class SemiActiveConfig:
    """Semi-active controller tuning parameters."""

    c_sky: float = 3200.0  # Ns/m (Skyhook damping coefficient)
    c_ground: float = 2400.0  # Ns/m (Groundhook damping coefficient)
    c_min: float = 800.0  # Ns/m (Softest damper state)
    c_max: float = 4800.0  # Ns/m (Firmest damper state)
    alpha_hybrid: float = 0.65  # Comfort vs road holding weighting [0..1]
    time_constant: float = 0.015  # s (solenoid response delay)
    active_force_max: float = 3500.0  # N (for full active suspension)

    @classmethod
    def from_yaml(cls, path: str | Path) -> "SemiActiveConfig":
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)

        sky = data.get("skyhook", {})
        ground = data.get("groundhook", {})
        hybrid = data.get("hybrid", {})
        active = data.get("active_suspension", {})

        return cls(
            c_sky=float(sky.get("skyhook_damping", 3200.0)),
            c_ground=float(ground.get("groundhook_damping", 2400.0)),
            c_min=float(sky.get("damper_min", 800.0)),
            c_max=float(sky.get("damper_max", 4800.0)),
            alpha_hybrid=float(hybrid.get("skyhook_weight", 0.65)),
            time_constant=float(sky.get("actuator_time_constant", 0.015)),
            active_force_max=float(active.get("force_max", 3500.0)),
        )


class SkyhookController:
    """Karnopp semi-active Skyhook damper controller.

    Targets minimization of sprung mass vertical acceleration (ride comfort).
    """

    def __init__(self, config: SemiActiveConfig | None = None, continuous: bool = True) -> None:
        self.cfg = config or SemiActiveConfig()
        self.continuous = continuous
        self._current_c = self.cfg.c_min

    def compute_damping(self, dzs: float, dzu: float, dt: float = 0.001) -> float:
        """Compute the demanded damper coefficient c [Ns/m].

        Args:
            dzs: Sprung mass velocity [m/s]
            dzu: Unsprung mass velocity [m/s]
            dt: Time step [s] for lag dynamics
        """
        v_rel = dzs - dzu  # Positive in extension (rebound)
        product = dzs * v_rel

        if product >= 1e-6:
            if self.continuous:
                c_target = self.cfg.c_sky * abs(dzs) / max(abs(v_rel), 1e-4)
                c_demanded = float(np.clip(c_target, self.cfg.c_min, self.cfg.c_max))
            else:
                c_demanded = self.cfg.c_max
        else:
            c_demanded = self.cfg.c_min

        # 1st-order solenoid actuator lag: c_dot = (c_demanded - c) / tau
        tau = max(self.cfg.time_constant, 1e-4)
        alpha = dt / (tau + dt)
        self._current_c += alpha * (c_demanded - self._current_c)
        return float(np.clip(self._current_c, self.cfg.c_min, self.cfg.c_max))


class GroundhookController:
    """Semi-active Groundhook damper controller.

    Targets minimization of dynamic tyre normal load variations (road holding & safety).
    """

    def __init__(self, config: SemiActiveConfig | None = None, continuous: bool = True) -> None:
        self.cfg = config or SemiActiveConfig()
        self.continuous = continuous
        self._current_c = self.cfg.c_min

    def compute_damping(self, dzs: float, dzu: float, dt: float = 0.001) -> float:
        """Compute the demanded damper coefficient c [Ns/m]."""
        v_rel = dzs - dzu
        product = -dzu * v_rel

        if product >= 1e-6:
            if self.continuous:
                c_target = self.cfg.c_ground * abs(dzu) / max(abs(v_rel), 1e-4)
                c_demanded = float(np.clip(c_target, self.cfg.c_min, self.cfg.c_max))
            else:
                c_demanded = self.cfg.c_max
        else:
            c_demanded = self.cfg.c_min

        tau = max(self.cfg.time_constant, 1e-4)
        alpha = dt / (tau + dt)
        self._current_c += alpha * (c_demanded - self._current_c)
        return float(np.clip(self._current_c, self.cfg.c_min, self.cfg.c_max))


class HybridSuspensionController:
    """Blends Skyhook comfort control with Groundhook road holding."""

    def __init__(self, config: SemiActiveConfig | None = None) -> None:
        self.cfg = config or SemiActiveConfig()
        self.skyhook = SkyhookController(config=self.cfg, continuous=True)
        self.groundhook = GroundhookController(config=self.cfg, continuous=True)

    def compute_damping(self, dzs: float, dzu: float, dt: float = 0.001) -> float:
        c_sky = self.skyhook.compute_damping(dzs, dzu, dt)
        c_ground = self.groundhook.compute_damping(dzs, dzu, dt)
        c_blended = self.cfg.alpha_hybrid * c_sky + (1.0 - self.cfg.alpha_hybrid) * c_ground
        return float(np.clip(c_blended, self.cfg.c_min, self.cfg.c_max))


class ActiveSuspensionController:
    """Active force generator (e.g. electro-hydraulic or electromagnetic strut).

    Generates commanded control force F_act [N] between chassis and unsprung hub.
    Positive F_act pushes sprung mass upwards and unsprung mass downwards.
    """

    def __init__(
        self,
        config: SemiActiveConfig | None = None,
        k_zs: float = 4000.0,
        k_dzs: float = 2800.0,
        k_deflection: float = 12000.0,
    ) -> None:
        self.cfg = config or SemiActiveConfig()
        self.k_zs = k_zs
        self.k_dzs = k_dzs
        self.k_deflection = k_deflection
        self._current_force = 0.0

    def compute_force(
        self,
        state: QuarterCarState,
        zs_target: float = 0.0,
        dt: float = 0.001,
    ) -> float:
        """Compute active force output [N]."""
        # State feedback control
        # Penalize sprung mass displacement from target, velocity, and suspension deflection
        error_zs = state.zs - zs_target
        deflection = state.zs - state.zu

        f_raw = -(self.k_zs * error_zs + self.k_dzs * state.dzs + self.k_deflection * deflection)

        # Clamping
        f_clamped = float(np.clip(f_raw, -self.cfg.active_force_max, self.cfg.active_force_max))

        # Actuator lag
        tau = max(self.cfg.time_constant, 1e-4)
        alpha = dt / (tau + dt)
        self._current_force += alpha * (f_clamped - self._current_force)
        return float(self._current_force)
