"""Core data types, indices, and structures for vehicle dynamics modeling."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import IntEnum
import math
from typing import Any

import numpy as np


class WheelIndex(IntEnum):
    """Standard 4-wheel vehicle indexing."""
    FL = 0  # Front Left
    FR = 1  # Front Right
    RL = 2  # Rear Left
    RR = 3  # Rear Right


FL = WheelIndex.FL
FR = WheelIndex.FR
RL = WheelIndex.RL
RR = WheelIndex.RR


@dataclass(frozen=True)
class SevenDOFParams:
    """Physical parameters for a 7-DOF two-track passenger vehicle."""

    # Vehicle body inertial properties
    mass: float = 1200.0          # Total vehicle mass [kg]
    Iz: float = 1800.0            # Yaw moment of inertia [kg*m^2]
    lf: float = 1.30              # CoG to front axle distance [m]
    lr: float = 1.50              # CoG to rear axle distance [m]
    wf: float = 1.60              # Front track width [m]
    wr: float = 1.60              # Rear track width [m]
    hcog: float = 0.50            # Height of center of gravity [m]
    k_roll_f: float = 0.55        # Fraction of lateral roll stiffness on front axle [-]

    # Wheel and tire mechanical properties
    Iw: float = 1.5               # Rotational inertia of each wheel assembly [kg*m^2]
    Reff: float = 0.32            # Effective tire rolling radius [m]
    c_rr: float = 0.015           # Rolling resistance coefficient [-]
    wheel_damping: float = 0.10   # Wheel rotational viscous damping [N*m*s/rad]

    # Aerodynamic drag properties
    Cd: float = 0.30              # Aerodynamic drag coefficient [-]
    Af: float = 2.20              # Frontal projected area [m^2]
    rho: float = 1.225            # Air density [kg/m^3]

    # Environment
    g: float = 9.81               # Gravitational acceleration [m/s^2]

    # Tyre parameters are owned by the TyreModel (see vehicle_model.tire_model / config/tyres).

    # Actuator physical limits
    max_steer_f: float = math.radians(35.0)  # [rad] Max front steer angle (~35 deg)
    max_steer_r: float = math.radians(10.0)  # [rad] Max rear steer angle (~10 deg)
    max_steer_rate: float = math.radians(50.0)  # [rad/s]
    max_drive_torque: float = 1000.0        # [Nm] per wheel
    max_brake_torque: float = 3000.0        # [Nm] per wheel

    @property
    def wheelbase(self) -> float:
        return self.lf + self.lr

    @property
    def static_normal_load(self) -> float:
        return self.mass * self.g

    @property
    def static_front_axle_load(self) -> float:
        return self.mass * self.g * (self.lr / self.wheelbase)

    @property
    def static_rear_axle_load(self) -> float:
        return self.mass * self.g * (self.lf / self.wheelbase)

    @classmethod
    def from_yaml(cls, path: str | Path | None = None) -> SevenDOFParams:
        """Load vehicle parameters from a YAML configuration file.

        If path is None, loads from centralized configs/vehicles/seven_dof_passenger.yaml.
        """
        import dataclasses
        from pathlib import Path
        import yaml

        if path is None:
            from vehicle_model.config.loader import get_configs_root
            p = get_configs_root() / "vehicles" / "seven_dof_passenger.yaml"
        else:
            p = Path(path)

        with open(p, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}

        valid_fields = {f.name for f in dataclasses.fields(cls)}
        filtered = {k: float(v) if isinstance(v, (int, float)) else v for k, v in data.items() if k in valid_fields}
        return cls(**filtered)


@dataclass
class SevenDOFControlInput:
    """Control inputs commanding steering, drive, and braking."""

    steer_f: float = 0.0
    steer_r: float = 0.0
    T_drive: np.ndarray = field(default_factory=lambda: np.zeros(4, dtype=float))
    T_brake: np.ndarray = field(default_factory=lambda: np.zeros(4, dtype=float))

    def __post_init__(self):
        self.T_drive = np.asarray(self.T_drive, dtype=float)
        self.T_brake = np.asarray(self.T_brake, dtype=float)
        if self.T_drive.shape != (4,):
            raise ValueError(f"T_drive must have shape (4,), got {self.T_drive.shape}")
        if self.T_brake.shape != (4,):
            raise ValueError(f"T_brake must have shape (4,), got {self.T_brake.shape}")

    def clamped(self, params: SevenDOFParams) -> SevenDOFControlInput:
        """Return a copy of inputs clamped to actuator physical limits."""
        clamped_sf = float(np.clip(self.steer_f, -params.max_steer_f, params.max_steer_f))
        clamped_sr = float(np.clip(self.steer_r, -params.max_steer_r, params.max_steer_r))
        clamped_td = np.clip(self.T_drive, -params.max_drive_torque, params.max_drive_torque)
        clamped_tb = np.clip(self.T_brake, 0.0, params.max_brake_torque)
        return SevenDOFControlInput(
            steer_f=clamped_sf,
            steer_r=clamped_sr,
            T_drive=clamped_td,
            T_brake=clamped_tb,
        )


@dataclass
class SevenDOFState:
    """Complete dynamic and navigation state of the 7-DOF vehicle.

    Dynamic DOFs (7):
      - vx: Longitudinal chassis velocity [m/s]
      - vy: Lateral chassis velocity [m/s]
      - r: Yaw rate [rad/s]
      - omega: 4 wheel angular rotational speeds [rad/s] (FL, FR, RL, RR)

    Navigation kinematic states (3):
      - X: Global longitudinal coordinate [m]
      - Y: Global lateral coordinate [m]
      - psi: Heading angle [rad]
    """

    vx: float = 0.0
    vy: float = 0.0
    r: float = 0.0
    omega: np.ndarray = field(default_factory=lambda: np.zeros(4, dtype=float))
    X: float = 0.0
    Y: float = 0.0
    psi: float = 0.0

    def __post_init__(self):
        self.omega = np.asarray(self.omega, dtype=float)
        if self.omega.shape != (4,):
            raise ValueError(f"omega must have shape (4,), got {self.omega.shape}")

    @property
    def speed(self) -> float:
        """Total chassis planar velocity magnitude [m/s]."""
        return float(math.hypot(self.vx, self.vy))

    @property
    def sideslip_angle(self) -> float:
        """Vehicle body sideslip angle beta = arctan(vy / vx) [rad]."""
        if abs(self.vx) < 0.1:
            return 0.0
        return float(math.atan2(self.vy, self.vx))

    def to_array(self) -> np.ndarray:
        """Serialize state to 1D float array of length 10."""
        return np.array([
            self.vx,
            self.vy,
            self.r,
            self.omega[0],
            self.omega[1],
            self.omega[2],
            self.omega[3],
            self.X,
            self.Y,
            self.psi,
        ], dtype=float)

    @classmethod
    def from_array(cls, arr: np.ndarray | list[float]) -> SevenDOFState:
        """Deserialize state from 1D array of length 10."""
        a = np.asarray(arr, dtype=float)
        if a.size != 10:
            raise ValueError(f"Expected array of size 10, got {a.size}")
        return cls(
            vx=float(a[0]),
            vy=float(a[1]),
            r=float(a[2]),
            omega=np.array([a[3], a[4], a[5], a[6]], dtype=float),
            X=float(a[7]),
            Y=float(a[8]),
            psi=float(a[9]),
        )


@dataclass
class WheelTelemetry:
    """Instantaneous physical quantities at all 4 wheels."""

    omega: np.ndarray          # Wheel angular speeds [rad/s]
    v_wx: np.ndarray           # Wheel longitudinal velocity in wheel frame [m/s]
    v_wy: np.ndarray           # Wheel lateral velocity in wheel frame [m/s]
    kappa: np.ndarray          # Longitudinal slip ratio [-]
    alpha: np.ndarray          # Lateral slip angle [rad]
    Fz: np.ndarray             # Normal load [N]
    mu: np.ndarray             # Friction coefficient [-]
    Fx_wheel: np.ndarray       # Longitudinal ground force in wheel frame [N]
    Fy_wheel: np.ndarray       # Lateral ground force in wheel frame [N]
    Fx_body: np.ndarray        # Longitudinal ground force in vehicle body frame [N]
    Fy_body: np.ndarray        # Lateral ground force in vehicle body frame [N]
    steer_angles: np.ndarray   # Steer angle of each wheel [rad]


def seven_dof_params_from_config(config_path: str | None = None) -> SevenDOFParams:
    """Load SevenDOFParams from default YAML or custom config file."""
    from vehicle_model.config import get_settings_section
    sec = get_settings_section("seven_dof", config_path)
    allowed = set(SevenDOFParams.__dataclass_fields__)
    return SevenDOFParams(**{k: float(sec[k]) for k in allowed if k in sec})

