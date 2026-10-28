"""Electric powertrain actuator models with torque envelopes, response lag, and power limits.

Implements electric drive dynamics for 4 independently driven wheels (or e-axles)
as required by Plan.md §7.3 and §19 (Milestone M4).
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
import numpy as np


@dataclass
class ElectricDriveConfig:
    """Parameters for electric motor drives and power inverter."""

    tau: float = 0.020                    # Inverter + PMSM electric time constant [s] (~20 ms)
    max_torque: float = 1000.0            # Max peak torque per wheel [Nm]
    max_power: float = 125000.0           # Max power per wheel [W] (125 kW per corner = 500 kW total)
    max_torque_slew_rate: float = 8000.0  # Max torque rate of change [Nm/s]
    base_wheel_speed: float = 35.0        # Wheel speed where field weakening begins [rad/s] (~40 km/h)
    efficiency: float = 0.94              # Inverter & motor round-trip efficiency [-]
    total_battery_power_limit: float = 450000.0  # Total DC battery pack discharge limit [W]
    ideal: bool = False                   # True: no lag or slew limit (torque and power limits still apply)

    @classmethod
    def from_yaml(cls, path: str | None = None) -> ElectricDriveConfig:
        """Load electric powertrain parameters from centralized YAML configuration."""
        from pathlib import Path
        import yaml

        if path is None:
            from vehicle_model.config.loader import get_configs_root
            p = get_configs_root() / "powertrains" / "electric_powertrain.yaml"
        else:
            p = Path(path)

        with open(p, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}

        ed = data.get("electric_drive", data)
        return cls(
            tau=float(ed.get("tau", 0.020)),
            max_torque=float(ed.get("max_torque", 1000.0)),
            max_power=float(ed.get("max_power", 125000.0)),
            max_torque_slew_rate=float(ed.get("max_torque_slew_rate", 8000.0)),
            base_wheel_speed=float(ed.get("base_wheel_speed", 35.0)),
            efficiency=float(ed.get("efficiency", 0.94)),
            total_battery_power_limit=float(ed.get("total_battery_power_limit", 450000.0)),
            ideal=bool(ed.get("ideal", False)),
        )


@dataclass
class PowertrainTelemetry:
    """Instantaneous operational diagnostics for the powertrain actuators."""

    actual_torque: np.ndarray       # Delivered wheel torques [Nm] (shape 4)
    demanded_torque: np.ndarray     # Commanded wheel torques [Nm] (shape 4)
    max_available_torque: np.ndarray# Current speed/power envelope limits [Nm] (shape 4)
    mechanical_power_w: np.ndarray  # Mechanical shaft power per wheel [W] (shape 4)
    electrical_power_w: float       # Total battery electrical power draw [W]
    power_limited: bool             # Flag if battery/inverter power limit was active
    slew_rate_limited: bool         # Flag if slew rate clipping occurred


class ElectricPowertrainActuator:
    """4-channel independent electric powertrain actuator."""

    def __init__(self, config: ElectricDriveConfig | None = None):
        self.config = config or ElectricDriveConfig()
        self.actual_torque = np.zeros(4, dtype=float)
        self.telemetry: PowertrainTelemetry | None = None

    def reset(self) -> None:
        """Reset actuator state to zero torque."""
        self.actual_torque = np.zeros(4, dtype=float)
        self.telemetry = None

    def get_max_torque(self, wheel_speeds: np.ndarray | list[float]) -> np.ndarray:
        """Calculate maximum available motor torque envelope given wheel angular speeds.

        In the constant torque region (omega <= base_wheel_speed): T_max = max_torque.
        In the field weakening region (omega > base_wheel_speed): T_max = max_power / omega.
        """
        w = np.abs(np.asarray(wheel_speeds, dtype=float))
        w_eff = np.maximum(w, self.config.base_wheel_speed)
        t_power = self.config.max_power / w_eff
        return np.minimum(self.config.max_torque, t_power)

    def step(
        self,
        cmd_torque: np.ndarray | list[float],
        wheel_speeds: np.ndarray | list[float],
        dt: float,
    ) -> np.ndarray:
        """Update powertrain actuator states forward by dt and return delivered torque [Nm]."""
        if dt <= 0.0:
            return self.actual_torque.copy()

        cmd = np.asarray(cmd_torque, dtype=float)
        omega = np.asarray(wheel_speeds, dtype=float)
        t_avail = self.get_max_torque(omega)

        # 1. Individual motor capability envelope limit
        t_target = np.clip(cmd, 0.0, t_avail)

        # 2. Total battery pack power limit enforcement
        # Mechanical power = sum(T_i * omega_i). Electrical power = P_mech / efficiency.
        est_mech_power = np.sum(t_target * np.maximum(np.abs(omega), 0.1))
        est_elec_power = est_mech_power / self.config.efficiency
        power_limited = False
        if est_elec_power > self.config.total_battery_power_limit and est_mech_power > 1e-3:
            scale = (self.config.total_battery_power_limit * self.config.efficiency) / est_mech_power
            t_target = t_target * scale
            power_limited = True

        if self.config.ideal:
            self.actual_torque = np.clip(t_target, 0.0, t_avail)
            slew_limited = False
        else:
            # 3. First-order actuator response: dT/dt_raw = (target - actual) / tau
            tau = max(self.config.tau, 1e-4)
            desired_rate = (t_target - self.actual_torque) / tau

            # 4. Slew rate limiting: |dT/dt| <= max_torque_slew_rate
            max_slew = self.config.max_torque_slew_rate
            actual_rate = np.clip(desired_rate, -max_slew, max_slew)
            slew_limited = bool(np.any(np.abs(desired_rate) > max_slew + 1e-6))

            # 5. Integration
            new_torque = self.actual_torque + actual_rate * dt
            self.actual_torque = np.clip(new_torque, 0.0, t_avail)

        # 6. Operational telemetry
        mech_p = self.actual_torque * np.abs(omega)
        elec_p = float(np.sum(mech_p) / self.config.efficiency)
        self.telemetry = PowertrainTelemetry(
            actual_torque=self.actual_torque.copy(),
            demanded_torque=cmd.copy(),
            max_available_torque=t_avail.copy(),
            mechanical_power_w=mech_p,
            electrical_power_w=elec_p,
            power_limited=power_limited,
            slew_rate_limited=slew_limited,
        )

        return self.actual_torque.copy()


__all__ = ("ElectricPowertrainActuator", "ElectricDriveConfig", "PowertrainTelemetry")
