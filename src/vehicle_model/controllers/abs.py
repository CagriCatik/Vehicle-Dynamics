"""Antilock Braking System (ABS) slip-based controller.

Regulates individual wheel brake torques to prevent wheel lockup (kappa -> -1.0),
maximizing longitudinal braking force while preserving lateral steering grip.
"""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from vehicle_model.core.types import SevenDOFParams, WheelTelemetry


@dataclass
class ABSConfig:
    """Parameters for individual wheel ABS slip controller."""

    target_slip: float = -0.15       # Optimal longitudinal slip ratio at peak braking grip
    slip_threshold: float = -0.20    # Slip threshold below which ABS intervenes
    release_slip: float = -0.10      # Slip threshold above which full braking is restored
    kp: float = 8.0                  # Proportional slip gain
    min_speed: float = 1.5           # Speed below which ABS deactivates to allow full vehicle halt [m/s]

    @classmethod
    def from_yaml(cls, path: str | None = None) -> ABSConfig:
        """Load ABS parameters from YAML configuration file."""
        from pathlib import Path
        import yaml

        if path is None:
            from vehicle_model.config.loader import get_configs_root
            p = get_configs_root() / "braking" / "abs_tcs.yaml"
        else:
            p = Path(path)

        with open(p, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}

        abs_data = data.get("abs", data)
        return cls(
            target_slip=float(abs_data.get("target_slip", -0.15)),
            slip_threshold=float(abs_data.get("slip_threshold", -0.20)),
            release_slip=float(abs_data.get("release_slip", -0.10)),
            kp=float(abs_data.get("kp", 8.0)),
            min_speed=float(abs_data.get("min_speed", 1.5)),
        )


class AntilockBrakingSystem:
    """Four-channel independent wheel ABS controller."""

    def __init__(self, config: ABSConfig | None = None):
        self.config = config or ABSConfig()

    def modulate_brake_torques(
        self,
        driver_brake_torques: np.ndarray,
        telemetry: WheelTelemetry | None,
        vehicle_speed: float,
    ) -> np.ndarray:
        """Modulate requested brake torques based on real-time wheel slip.

        Parameters:
            driver_brake_torques: Demanded brake torque per wheel [Nm] [FL, FR, RL, RR]
            telemetry: Last wheel telemetry containing kappa [-]
            vehicle_speed: Current vehicle longitudinal speed [m/s]

        Returns:
            Modulated brake torques [Nm] [FL, FR, RL, RR]
        """
        torques = np.asarray(driver_brake_torques, dtype=float).copy()

        # Deactivate ABS below minimum speed threshold to allow full stop
        if vehicle_speed < self.config.min_speed or telemetry is None:
            return torques

        for i in range(4):
            kappa_i = float(telemetry.kappa[i])
            T_req = torques[i]

            if T_req <= 1e-3:
                continue

            # If slip is worse than threshold (e.g. kappa < -0.20), reduce brake torque
            if kappa_i < self.config.slip_threshold:
                # Proportional slip reduction factor
                # When kappa_i -> -1.0, factor -> 0.0 (dump brake pressure)
                error = kappa_i - self.config.target_slip  # Negative when locked
                factor = max(0.05, min(1.0, 1.0 + self.config.kp * error))
                torques[i] = T_req * factor
            elif kappa_i > self.config.release_slip:
                # Normal braking zone, allow full requested torque
                torques[i] = T_req
            else:
                # Smooth transition zone
                ratio = (kappa_i - self.config.slip_threshold) / (self.config.release_slip - self.config.slip_threshold)
                torques[i] = T_req * (0.6 + 0.4 * ratio)

        return torques
