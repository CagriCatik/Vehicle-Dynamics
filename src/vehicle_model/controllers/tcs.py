"""Traction Control System (TCS) slip-based drive torque controller.

Limits excessive drive torque on low-friction surfaces to prevent wheel spin (kappa >> 0),
maintaining optimal acceleration traction and lateral stability.
"""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from vehicle_model.core.types import WheelTelemetry


@dataclass
class TCSConfig:
    """Parameters for individual wheel TCS controller."""

    target_slip: float = 0.12        # Target longitudinal slip ratio for peak acceleration
    slip_threshold: float = 0.18     # Slip threshold above which TCS cuts torque
    kp: float = 6.0                  # Proportional torque reduction gain

    @classmethod
    def from_yaml(cls, path: str | None = None) -> TCSConfig:
        """Load TCS parameters from YAML configuration file."""
        from pathlib import Path
        import yaml

        if path is None:
            from vehicle_model.config.loader import get_configs_root
            p = get_configs_root() / "braking" / "abs_tcs.yaml"
        else:
            p = Path(path)

        with open(p, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}

        tcs_data = data.get("tcs", data)
        return cls(
            target_slip=float(tcs_data.get("target_slip", 0.12)),
            slip_threshold=float(tcs_data.get("slip_threshold", 0.18)),
            kp=float(tcs_data.get("kp", 6.0)),
        )


class TractionControlSystem:
    """Four-wheel independent traction control system."""

    def __init__(self, config: TCSConfig | None = None):
        self.config = config or TCSConfig()

    def modulate_drive_torques(
        self,
        driver_drive_torques: np.ndarray,
        telemetry: WheelTelemetry | None,
    ) -> np.ndarray:
        """Modulate requested drive torques based on observed positive wheel slip.

        Parameters:
            driver_drive_torques: Demanded drive torque per wheel [Nm] [FL, FR, RL, RR]
            telemetry: Last wheel telemetry containing kappa [-]

        Returns:
            Modulated drive torques [Nm] [FL, FR, RL, RR]
        """
        torques = np.asarray(driver_drive_torques, dtype=float).copy()
        if telemetry is None:
            return torques

        for i in range(4):
            kappa_i = float(telemetry.kappa[i])
            T_req = torques[i]

            if T_req <= 1e-3:
                continue

            # If wheel is spinning beyond threshold, cut drive torque
            if kappa_i > self.config.slip_threshold:
                error = kappa_i - self.config.target_slip  # Positive when spinning
                reduction_factor = max(0.10, min(1.0, 1.0 - self.config.kp * error))
                torques[i] = T_req * reduction_factor

        return torques
