"""Torque Vectoring and Electronic Stability Control (ESC) direct yaw moment controller.

Computes desired vehicle yaw rate based on driver steering and speed, calculates yaw error,
and allocates differential drive torques and stabilizing corrective braking forces.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import numpy as np

from vehicle_model.controllers.framework import INFORMATION_FULL_STATE
from vehicle_model.core.types import (
    FL,
    FR,
    RL,
    RR,
    SevenDOFParams,
    SevenDOFState,
    WheelTelemetry,
)


@dataclass
class TorqueVectoringConfig:
    """Parameters for Torque Vectoring and ESC."""

    kp_yaw: float = 3500.0            # Proportional gain on yaw rate error [N*m/(rad/s)]
    kd_yaw: float = 250.0             # Derivative gain on yaw rate error [N*m/(rad/s^2)]
    max_yaw_moment: float = 4000.0    # Maximum corrective yaw moment [N*m]
    understeer_gradient: float = 0.0015  # Vehicle understeer gradient [s^2/m^2]
    max_torque_diff: float = 600.0    # Max differential drive torque per axle [Nm]
    esc_brake_gain: float = 1.0       # Gain translating excess yaw moment to stabilizing brake torque

    @classmethod
    def from_yaml(cls, path: str | None = None) -> TorqueVectoringConfig:
        """Load TV/ESC parameters from YAML configuration file."""
        from pathlib import Path
        import yaml

        if path is None:
            from vehicle_model.config.loader import get_configs_root
            p = get_configs_root() / "controllers" / "integrated_controller.yaml"
        else:
            p = Path(path)

        with open(p, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}

        tv_data = data.get("torque_vectoring", data)
        return cls(
            kp_yaw=float(tv_data.get("kp_yaw", 3500.0)),
            kd_yaw=float(tv_data.get("kd_yaw", 250.0)),
            max_yaw_moment=float(tv_data.get("max_yaw_moment", 4000.0)),
            max_torque_diff=float(tv_data.get("max_torque_diff", 600.0)),
            understeer_gradient=float(tv_data.get("understeer_gradient", 0.0015)),
            esc_brake_gain=float(tv_data.get("esc_brake_gain", 1.0)),
        )


class TorqueVectoringController:
    """Direct Yaw Moment Controller for agile torque distribution and ESC stability intervention."""

    information = INFORMATION_FULL_STATE

    def __init__(
        self,
        params: SevenDOFParams,
        config: TorqueVectoringConfig | None = None,
    ):
        self.params = params
        self.config = config or TorqueVectoringConfig()
        self._prev_error = 0.0

    def reset(self) -> None:
        """Reset internal controller state."""
        self._prev_error = 0.0

    def compute_desired_yaw_rate(
        self,
        steer_f: float,
        vx: float,
        mu_est: float = 1.0,
    ) -> float:
        """Compute the reference linear single-track steady-state yaw rate r_des [rad/s]."""
        v = max(0.5, abs(vx))
        L = self.params.wheelbase
        K_us = self.config.understeer_gradient

        # Neutral/understeer steady-state yaw rate
        r_linear = (v / (L + K_us * (v ** 2))) * steer_f

        # Road friction physical saturation limit: |r| <= mu * g / v
        r_friction_limit = (mu_est * self.params.g) / v
        return float(np.clip(r_linear, -r_friction_limit, r_friction_limit))

    def compute_corrective_moment(
        self,
        r_actual: float,
        r_des: float,
        dt: float = 0.01,
    ) -> float:
        """PD controller calculating desired corrective yaw moment Delta Mz [Nm].

        Positive Delta Mz turns vehicle counter-clockwise (left).
        """
        error = r_des - r_actual
        d_error = (error - self._prev_error) / max(dt, 1e-4)
        self._prev_error = error

        Mz_cmd = self.config.kp_yaw * error + self.config.kd_yaw * d_error
        return float(np.clip(Mz_cmd, -self.config.max_yaw_moment, self.config.max_yaw_moment))

    def allocate_torques(
        self,
        driver_T_drive: np.ndarray,
        driver_T_brake: np.ndarray,
        Mz_corr: float,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Allocate corrective yaw moment via differential drive and stabilizing braking.

        Positive Mz_corr requires counter-clockwise moment:
          - Drive: Boost RR (+thrust), cut RL (-thrust)
          - Brake (ESC oversteer/understeer): Apply outer/inner wheel differential braking

        Returns:
            (T_drive, T_brake) shaped (4,)
        """
        p = self.params
        cfg = self.config

        T_drive = np.asarray(driver_T_drive, dtype=float).copy()
        T_brake = np.asarray(driver_T_brake, dtype=float).copy()

        # 1. Torque Vectoring via Rear Drive Axle:
        # Delta Mz = (T_drive_RR - T_drive_RL) / Reff * (wr / 2)
        # => Delta T = Mz_corr * Reff / (wr / 2)
        if np.any(T_drive > 10.0):
            delta_T = (Mz_corr * p.Reff) / (p.wr / 2.0)
            delta_T = float(np.clip(delta_T, -cfg.max_torque_diff, cfg.max_torque_diff))

            # Apply to rear axle: RR gets +delta_T/2, RL gets -delta_T/2
            T_drive[RL] = max(0.0, T_drive[RL] - 0.5 * delta_T)
            T_drive[RR] = max(0.0, T_drive[RR] + 0.5 * delta_T)

        # 2. ESC Stabilizing Differential Braking:
        # If yaw error remains significant or Mz_corr demands more authority:
        # In a left turn (steer > 0):
        #   - Understeer (Mz_corr > 0): vehicle won't turn in => brake inner rear wheel (RL)
        #   - Oversteer (Mz_corr < 0): vehicle spinning out => brake outer front wheel (FR)
        if abs(Mz_corr) > 500.0:
            brake_mag = (abs(Mz_corr) * p.Reff / (p.wf / 2.0)) * cfg.esc_brake_gain
            brake_mag = min(brake_mag, 1500.0)

            if Mz_corr > 0:
                # Understeer intervention: brake inner rear wheel
                T_brake[RL] += brake_mag
            else:
                # Oversteer intervention: brake outer front wheel
                T_brake[FR] += brake_mag

        return T_drive, T_brake
