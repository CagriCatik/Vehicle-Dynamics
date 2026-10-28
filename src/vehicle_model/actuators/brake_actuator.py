"""Brake-by-wire actuator models with hydraulic dynamics, ABS valve modulation, and regenerative blending.

Implements electro-hydraulic friction brakes and regenerative brake blending
as required by Plan.md §8 and §19 (Milestone M4).
"""

from __future__ import annotations

from dataclasses import dataclass, field
import numpy as np


@dataclass
class BrakeActuatorConfig:
    """Parameters for electro-hydraulic friction brake calipers and regenerative blending."""

    tau_apply: float = 0.035              # Hydraulic pressure build-up time constant [s] (~35 ms)
    tau_release: float = 0.025            # Rapid dump valve time constant [s] (~25 ms)
    max_brake_torque: float = 3000.0      # Caliper torque limit per wheel [Nm]
    max_build_rate: float = 22000.0       # Max hydraulic pressure build slew rate [Nm/s]
    max_dump_rate: float = 36000.0        # Max ABS dump valve slew rate [Nm/s]
    ideal: bool = False                   # True: torques follow the command instantly (limits still apply)

    # Regenerative braking blending parameters
    enable_regen_blending: bool = True    # Blend electric motor regen with hydraulic friction
    max_regen_torque: float = 500.0       # Max regenerative braking torque per wheel [Nm]
    regen_cutoff_speed: float = 2.0       # Speed [m/s] where regen blends down to 0 for smooth stop
    regen_blend_bandwidth: float = 1.0    # Transition band width [m/s]
    tau_regen: float = 0.020              # Regenerative torque time constant [s] (electric drive)

    @classmethod
    def from_yaml(cls, path: str | None = None) -> BrakeActuatorConfig:
        """Load brake actuator parameters from centralized YAML configuration."""
        from pathlib import Path
        import yaml

        if path is None:
            from vehicle_model.config.loader import get_configs_root
            p = get_configs_root() / "braking" / "brake_by_wire.yaml"
        else:
            p = Path(path)

        with open(p, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}

        calipers = data.get("hydraulic_calipers", {})
        regen = data.get("regenerative_blending", {})

        return cls(
            tau_apply=float(calipers.get("tau_apply", 0.035)),
            tau_release=float(calipers.get("tau_release", 0.025)),
            max_brake_torque=float(calipers.get("max_brake_torque", 3000.0)),
            max_build_rate=float(calipers.get("max_build_rate", 22000.0)),
            max_dump_rate=float(calipers.get("max_dump_rate", 36000.0)),
            enable_regen_blending=bool(regen.get("enable_regen_blending", True)),
            max_regen_torque=float(regen.get("max_regen_torque", 500.0)),
            regen_cutoff_speed=float(regen.get("regen_cutoff_speed", 2.0)),
            regen_blend_bandwidth=float(regen.get("regen_blend_bandwidth", 1.0)),
            tau_regen=float(regen.get("tau_regen", 0.020)),
            ideal=bool(data.get("ideal", False)),
        )


@dataclass
class BrakeTelemetry:
    """Operational diagnostics for friction and regenerative braking split."""

    demanded_torque: np.ndarray      # Commanded total braking torque [Nm] (shape 4)
    friction_torque: np.ndarray      # Hydraulic caliper braking torque [Nm] (shape 4)
    regen_torque: np.ndarray         # Electric machine regen braking torque [Nm] (shape 4)
    total_delivered: np.ndarray      # Total braking torque delivered [Nm] (shape 4)
    regen_active: np.ndarray         # Boolean flags for regen availability per wheel
    abs_intervening: np.ndarray      # Boolean flags if ABS cut regen on that wheel


class BrakeByWireActuator:
    """4-channel electro-hydraulic brake actuator with regenerative braking blending."""

    def __init__(self, config: BrakeActuatorConfig | None = None):
        self.config = config or BrakeActuatorConfig()
        self.friction_torque = np.zeros(4, dtype=float)
        self.regen_torque = np.zeros(4, dtype=float)
        self.telemetry: BrakeTelemetry | None = None

    def reset(self) -> None:
        """Reset friction and regen brake states to zero."""
        self.friction_torque = np.zeros(4, dtype=float)
        self.regen_torque = np.zeros(4, dtype=float)
        self.telemetry = None

    def step(
        self,
        cmd_brake_torque: np.ndarray | list[float],
        vx: float,
        abs_active: np.ndarray | list[bool] | None = None,
        dt: float = 0.005,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Update brake actuator states forward by dt.

        Parameters
        ----------
        cmd_brake_torque : array_like of shape (4,)
            Total commanded braking torque per wheel [Nm] >= 0.
        vx : float
            Vehicle longitudinal speed [m/s].
        abs_active : array_like of bool, optional
            Flags indicating whether ABS is actively modulating wheel pressure.
        dt : float
            Time step [s].

        Returns
        -------
        tuple[np.ndarray, np.ndarray]
            (friction_torque, regen_torque) delivered at all 4 wheels [Nm].
        """
        if dt <= 0.0:
            return self.friction_torque.copy(), self.regen_torque.copy()

        cmd = np.clip(np.asarray(cmd_brake_torque, dtype=float), 0.0, self.config.max_brake_torque)
        abs_flag = np.zeros(4, dtype=bool) if abs_active is None else np.asarray(abs_active, dtype=bool)

        # 1. Regenerative Braking Coordination & Blending
        # Speed-dependent blend factor: 0 at standstill, 1 at speed
        v_abs = abs(vx)
        if v_abs <= (self.config.regen_cutoff_speed - self.config.regen_blend_bandwidth):
            speed_blend = 0.0
        elif v_abs >= (self.config.regen_cutoff_speed + self.config.regen_blend_bandwidth):
            speed_blend = 1.0
        else:
            speed_blend = (v_abs - (self.config.regen_cutoff_speed - self.config.regen_blend_bandwidth)) / (
                2.0 * self.config.regen_blend_bandwidth
            )

        target_regen = np.zeros(4, dtype=float)
        target_friction = cmd.copy()

        if self.config.enable_regen_blending and speed_blend > 1e-3:
            for i in range(4):
                if not abs_flag[i]:
                    # Wheel has grip, no ABS: utilize regenerative braking up to limit
                    r_des = min(cmd[i], self.config.max_regen_torque) * speed_blend
                    target_regen[i] = r_des
                    target_friction[i] = max(0.0, cmd[i] - r_des)
                else:
                    # ABS active: dump regen immediately to 0 for rapid hydraulic anti-lock control
                    target_regen[i] = 0.0
                    target_friction[i] = cmd[i]

        # 2. Electric Motor Regen Dynamics (Fast electrical torque control)
        if self.config.ideal:
            self.regen_torque = np.clip(target_regen, 0.0, self.config.max_regen_torque)
            self.friction_torque = np.clip(target_friction, 0.0, self.config.max_brake_torque)
        else:
            rate_regen = (target_regen - self.regen_torque) / max(self.config.tau_regen, 1e-4)
            self.regen_torque = np.clip(self.regen_torque + rate_regen * dt, 0.0, self.config.max_regen_torque)

        # 3. Electro-Hydraulic Caliper Dynamics with Asymmetric Apply / Release
        if not self.config.ideal:
            for i in range(4):
                t_curr = self.friction_torque[i]
                t_des = target_friction[i]

                if t_des >= t_curr:
                    # Pressure build-up
                    tau = max(self.config.tau_apply, 1e-4)
                    rate = min((t_des - t_curr) / tau, self.config.max_build_rate)
                else:
                    # Dump valve release (faster pressure relief)
                    tau = max(self.config.tau_release, 1e-4)
                    rate = max((t_des - t_curr) / tau, -self.config.max_dump_rate)

                self.friction_torque[i] = float(np.clip(t_curr + rate * dt, 0.0, self.config.max_brake_torque))

        # 4. Diagnostics & Telemetry
        total_delivered = self.friction_torque + self.regen_torque
        self.telemetry = BrakeTelemetry(
            demanded_torque=cmd.copy(),
            friction_torque=self.friction_torque.copy(),
            regen_torque=self.regen_torque.copy(),
            total_delivered=total_delivered.copy(),
            regen_active=(self.regen_torque > 1.0),
            abs_intervening=abs_flag.copy(),
        )

        return self.friction_torque.copy(), self.regen_torque.copy()


__all__ = ("BrakeByWireActuator", "BrakeActuatorConfig", "BrakeTelemetry")
