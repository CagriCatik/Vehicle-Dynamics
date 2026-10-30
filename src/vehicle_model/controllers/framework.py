"""Unified vehicle motion controller interfaces and data contracts.

providing a clean interface
for passive baselines, PID/feedback controllers, IVMC, and NMPC path trackers.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import time
from typing import Any, Protocol, runtime_checkable
import numpy as np

from vehicle_model.core.types import (
    FL,
    FR,
    RL,
    RR,
    SevenDOFControlInput,
    SevenDOFParams,
    SevenDOFState,
    WheelTelemetry,
)


# Declared information access of a controller: what it reads besides the driver inputs.
INFORMATION_FULL_STATE = "full_state_ideal"  # state and telemetry incl. road friction, no sensor model
INFORMATION_DRIVER_ONLY = "driver_inputs_only"  # no feedback from the vehicle

@dataclass
class ControlTarget:
    """Standardized high-level motion demand and driver intent."""

    target_vx: float = 0.0               # Desired longitudinal velocity [m/s]
    target_yaw_rate: float = 0.0         # Desired yaw rate [rad/s] (0.0 if left to controller)
    target_y: float = 0.0                # Desired lateral path coordinate [m]
    target_psi: float = 0.0              # Desired heading [rad]

    # Driver raw pedal / handwheel inputs
    driver_steer_f: float = 0.0          # Driver handwheel steering angle at road wheel [rad]
    driver_accel_pedal: float = 0.0      # Throttle command [0.0 to 1.0]
    driver_brake_pedal: float = 0.0      # Brake command [0.0 to 1.0]

    # Estimated conditions
    mu_estimated: float = 1.0            # Estimated road friction coefficient [-]


@dataclass
class ControlOutput(SevenDOFControlInput):
    """Actuator command demands and control diagnostics produced by a controller."""

    steer_f: float = 0.0                 # Front steer angle demand [rad]
    steer_r: float = 0.0                 # Rear steer angle demand [rad]
    T_drive: np.ndarray = field(default_factory=lambda: np.zeros(4, dtype=float))
    T_brake: np.ndarray = field(default_factory=lambda: np.zeros(4, dtype=float))
    desired_yaw_moment: float = 0.0      # Controller desired net yaw moment [Nm]
    allocated_yaw_moment: float = 0.0    # Achieved yaw moment from allocated torques [Nm]
    status: str = "nominal"              # "nominal", "saturated", "infeasible", "fallback"
    computation_time_ms: float = 0.0     # Execution duration in milliseconds
    diagnostics: dict[str, Any] = field(default_factory=dict)


@runtime_checkable
class VehicleController(Protocol):
    """Common interface for all vehicle motion controllers."""

    @property
    def name(self) -> str:
        """Human-readable name of the controller."""
        ...

    def reset(self) -> None:
        """Reset internal states, integrators, or warm-start buffers."""
        ...

    def compute_control(
        self,
        state: SevenDOFState,
        target: ControlTarget,
        telemetry: WheelTelemetry | None = None,
        dt: float = 0.01,
    ) -> ControlOutput:
        """Compute control commands given current state and target demand."""
        ...


class PassiveBaselineController:
    """Passive vehicle baseline: driver-only front steer, open differential, fixed 60/40 brake bias."""

    information = INFORMATION_DRIVER_ONLY

    def __init__(self, params: SevenDOFParams | None = None):
        self.params = params or SevenDOFParams()

    @property
    def name(self) -> str:
        return "Passive Baseline (No Active Chassis Control)"

    def reset(self) -> None:
        pass

    def compute_control(
        self,
        state: SevenDOFState,
        target: ControlTarget,
        telemetry: WheelTelemetry | None = None,
        dt: float = 0.01,
    ) -> ControlOutput:
        t0 = time.perf_counter()

        # Front steering directly follows driver; rear is 0
        steer_f = float(np.clip(target.driver_steer_f, -self.params.max_steer_f, self.params.max_steer_f))
        steer_r = 0.0

        # Open differential drive torque distribution
        t_req_drive = target.driver_accel_pedal * (4.0 * self.params.max_drive_torque)
        t_per_wheel = t_req_drive / 4.0
        T_drive = np.full(4, t_per_wheel, dtype=float)

        # Fixed 60/40 front/rear brake bias with symmetric left/right distribution
        t_req_brake = target.driver_brake_pedal * (4.0 * self.params.max_brake_torque)
        t_front = 0.60 * t_req_brake / 2.0
        t_rear = 0.40 * t_req_brake / 2.0
        T_brake = np.array([t_front, t_front, t_rear, t_rear], dtype=float)

        t_elapsed = (time.perf_counter() - t0) * 1000.0
        return ControlOutput(
            steer_f=steer_f,
            steer_r=steer_r,
            T_drive=T_drive,
            T_brake=T_brake,
            desired_yaw_moment=0.0,
            allocated_yaw_moment=0.0,
            status="nominal",
            computation_time_ms=t_elapsed,
        )


class FourWheelSteeringController:
    """Four-wheel steering coordinator conforming to VehicleController protocol."""

    information = INFORMATION_FULL_STATE

    def __init__(
        self,
        steering: Any | None = None,
        params: SevenDOFParams | None = None,
        mode: Any | None = None,
        enable_yaw_damping: bool | None = None,
        k_yaw_damping: float | None = None,
        enable_abs: bool = False,
    ):
        from vehicle_model.steering import FourWheelSteering, SteeringConfig, SteeringMode
        self.steering = steering or FourWheelSteering(SteeringConfig.from_yaml())
        self.params = params or SevenDOFParams()
        self.mode = mode if mode is not None else getattr(self.steering.config, "default_mode", SteeringMode.LOOKUP_TABLE)
        self.enable_yaw_damping = (
            enable_yaw_damping if enable_yaw_damping is not None
            else getattr(self.steering.config, "enable_yaw_damping", True)
        )
        self.k_yaw_damping = (
            k_yaw_damping if k_yaw_damping is not None
            else getattr(self.steering.config, "k_yaw_damping", 0.08)
        )
        self.enable_abs = enable_abs

    @classmethod
    def from_yaml(
        cls,
        path: str | Path = "configs/steering/four_wheel_steering.yaml",
        enable_abs: bool = False,
    ) -> "FourWheelSteeringController":
        from vehicle_model.steering import FourWheelSteering, SteeringConfig
        cfg = SteeringConfig.from_yaml(path)
        return cls(steering=FourWheelSteering(cfg), enable_abs=enable_abs)

    @property
    def name(self) -> str:
        suffix = " + Yaw Damping" if self.enable_yaw_damping else ""
        return f"Four-Wheel Steering (4WS LUT{suffix})"

    def reset(self) -> None:
        pass

    def compute_control(
        self,
        state: SevenDOFState,
        target: ControlTarget,
        telemetry: WheelTelemetry | None = None,
        dt: float = 0.01,
    ) -> ControlOutput:
        t0 = time.perf_counter()

        delta_f, delta_r = self.steering.compute_steering_angles(
            cmd_steer_f=target.driver_steer_f,
            vx=state.vx,
            vy=state.vy,
            mode=self.mode,
            wheelbase=self.params.wheelbase,
            lr=self.params.lr,
        )

        if self.enable_yaw_damping and abs(state.vx) > 0.5:
            # Active yaw disturbance rejection:
            # Calculate desired reference yaw rate from driver input
            v = max(abs(state.vx), 1.0)
            L = self.params.wheelbase
            r_des = target.target_yaw_rate if abs(target.target_yaw_rate) > 1e-4 else (v / L) * delta_f
            mu_est = target.mu_estimated if target.mu_estimated > 0.05 else 0.85
            r_max = (0.85 * mu_est * 9.81) / v
            r_des = float(np.clip(r_des, -r_max, r_max))

            r_error = state.r - r_des
            delta_r_damping = self.k_yaw_damping * r_error
            delta_r = float(np.clip(delta_r + delta_r_damping, -self.steering.config.max_steer_r, self.steering.config.max_steer_r))

        t_req_drive = target.driver_accel_pedal * (4.0 * self.params.max_drive_torque)
        T_drive = np.full(4, t_req_drive / 4.0, dtype=float)

        t_req_brake = target.driver_brake_pedal * (4.0 * self.params.max_brake_torque)
        t_front = 0.60 * t_req_brake / 2.0
        t_rear = 0.40 * t_req_brake / 2.0
        T_brake = np.array([t_front, t_front, t_rear, t_rear], dtype=float)

        # Standalone 4WS has no integrated IVMC/ABS unless explicitly configured
        if self.enable_abs and telemetry is not None and hasattr(telemetry, "kappa"):
            for w in range(4):
                if telemetry.kappa[w] < -0.15:
                    slip_excess = abs(telemetry.kappa[w]) - 0.15
                    attenuation = max(0.15, 1.0 - 5.0 * slip_excess)
                    T_brake[w] *= attenuation

        t_elapsed = (time.perf_counter() - t0) * 1000.0
        return ControlOutput(
            steer_f=delta_f,
            steer_r=delta_r,
            T_drive=T_drive,
            T_brake=T_brake,
            desired_yaw_moment=0.0,
            allocated_yaw_moment=0.0,
            status="nominal",
            computation_time_ms=t_elapsed,
        )


__all__ = (
    "INFORMATION_DRIVER_ONLY",
    "INFORMATION_FULL_STATE",
    "ControlTarget",
    "ControlOutput",
    "VehicleController",
    "PassiveBaselineController",
    "FourWheelSteeringController",
)

