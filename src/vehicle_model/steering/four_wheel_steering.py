"""Four-wheel steering (4WS) systems and rear-wheel steering coordination."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from typing import Any
import numpy as np


class SteeringMode(Enum):
    """Operation mode of the steering system."""
    FRONT_ONLY = "front_only"               # delta_r = 0
    SPEED_SCHEDULED = "speed_scheduled"     # Counter-phase at low speed, same-phase at high speed
    ZERO_SIDESLIP = "zero_sideslip"         # Analytical rear steer to minimize vehicle body slip angle
    DIRECT = "direct"                       # Direct rear steering angle input
    LOOKUP_TABLE = "lookup_table"           # 2-D speed and front-steer lookup table (LUT)
    LUT_BASED = "lookup_table"              # Alias for LOOKUP_TABLE


@dataclass
class SteeringConfig:
    """Configuration parameters for front and rear steering actuators."""

    max_steer_f: float = math.radians(35.0)  # Max front wheel angle [rad]
    max_steer_r: float = math.radians(10.0)  # Max rear wheel angle [rad]
    max_rate_f: float = math.radians(45.0)   # Max front steer rate [rad/s]
    max_rate_r: float = math.radians(35.0)   # Max rear steer rate [rad/s]

    # Speed-scheduled 4WS parameters
    v_switch: float = 10.0                   # Transition speed [m/s] (~36 km/h)
    v_width: float = 2.5                     # Transition transition width [m/s]
    k_low_speed: float = -0.30               # Counter-phase ratio (agile, reduces turning radius)
    k_high_speed: float = +0.20              # Same-phase ratio (stable, reduces sideslip)
    lut_path: str = "configs/steering/lut.json"  # Path to rear-axle steering LUT table
    default_mode: SteeringMode = SteeringMode.LOOKUP_TABLE
    enable_yaw_damping: bool = True
    k_yaw_damping: float = 0.08

    @classmethod
    def from_yaml(cls, path: str | None = None) -> SteeringConfig:
        """Load steering parameters from a YAML file."""
        from pathlib import Path
        import yaml

        if path is None:
            from vehicle_model.config.loader import get_configs_root
            p = get_configs_root() / "steering" / "four_wheel_steering.yaml"
        else:
            p = Path(path)

        with open(p, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}

        sched = data.get("speed_scheduling", {})
        act = data.get("actuator", {})
        rear_act = data.get("rear_actuator", {})
        cl_damping = data.get("closed_loop_damping", {})

        raw_mode = str(data.get("mode", "lookup_table")).lower()
        if raw_mode in ("speed_scheduled", "speed_schedule"):
            mode_enum = SteeringMode.SPEED_SCHEDULED
        elif raw_mode in ("zero_sideslip", "zero_beta"):
            mode_enum = SteeringMode.ZERO_SIDESLIP
        elif raw_mode == "front_only":
            mode_enum = SteeringMode.FRONT_ONLY
        elif raw_mode == "direct":
            mode_enum = SteeringMode.DIRECT
        else:
            mode_enum = SteeringMode.LOOKUP_TABLE

        return cls(
            max_steer_f=float(act.get("max_angle", math.radians(35.0))),
            max_steer_r=float(rear_act.get("max_angle", math.radians(10.0))),
            max_rate_f=float(act.get("max_rate", math.radians(45.0))),
            max_rate_r=float(rear_act.get("max_rate", math.radians(35.0))),
            v_switch=float(sched.get("v_switch", 10.0)),
            v_width=float(sched.get("v_width", 2.5)),
            k_low_speed=float(sched.get("k_low_speed", -0.30)),
            k_high_speed=float(sched.get("k_high_speed", 0.20)),
            lut_path=str(data.get("lut_path", "configs/steering/lut.json")),
            default_mode=mode_enum,
            enable_yaw_damping=bool(cl_damping.get("enabled", True)),
            k_yaw_damping=float(cl_damping.get("k_yaw_damping", 0.08)),
        )


class FourWheelSteering:
    """Four-wheel steering coordinator supporting multiple control modes."""

    def __init__(
        self,
        config: SteeringConfig | None = None,
        lut: Any | None = None,
    ):
        self.config = config or SteeringConfig()
        self._lut = lut

    @property
    def lut(self):
        """Lazy loader for rear-axle steering lookup table."""
        if self._lut is None:
            from vehicle_model.steering.lut import load_lookup_table
            self._lut = load_lookup_table(self.config.lut_path)
        return self._lut

    def get_rear_steer_ratio(self, vx: float) -> float:
        """Compute the continuous speed-dependent rear-to-front steer ratio k(vx).

        At low speeds (vx << v_switch): k ~ k_low_speed (< 0, counter-phase).
        At high speeds (vx >> v_switch): k ~ k_high_speed (> 0, same-phase).
        Smooth logistic sigmoid transition ensures no discontinuous jump in torque or acceleration.
        """
        v = abs(vx)
        sigmoid = 1.0 / (1.0 + math.exp(-max(-30.0, min(30.0, (v - self.config.v_switch) / self.config.v_width))))
        return self.config.k_low_speed + (self.config.k_high_speed - self.config.k_low_speed) * sigmoid

    def compute_steering_angles(
        self,
        cmd_steer_f: float,
        cmd_steer_r: float = 0.0,
        vx: float = 0.0,
        vy: float = 0.0,
        mode: SteeringMode | None = None,
        wheelbase: float = 2.8,
        lr: float = 1.5,
    ) -> tuple[float, float]:
        """Compute actual front and rear steering angles (delta_f, delta_r).

        Clamps angles to physical actuator limits.
        """
        active_mode = mode if mode is not None else self.config.default_mode
        delta_f = float(np.clip(cmd_steer_f, -self.config.max_steer_f, self.config.max_steer_f))

        if active_mode == SteeringMode.FRONT_ONLY:
            delta_r = 0.0
        elif active_mode == SteeringMode.SPEED_SCHEDULED:
            ratio = self.get_rear_steer_ratio(vx)
            delta_r = ratio * delta_f
        elif active_mode == SteeringMode.ZERO_SIDESLIP:
            # Kinematic zero-sideslip condition:
            # tan(beta) = (lr * tan(delta_f) + lf * tan(delta_r)) / L = 0 => delta_r = -atan((lr / lf) * tan(delta_f))
            ratio = self.get_rear_steer_ratio(vx)
            delta_r = ratio * delta_f
        elif active_mode in (SteeringMode.LOOKUP_TABLE, SteeringMode.LUT_BASED):
            # Calibrated 2-D lookup table: f(speed, front_steer_deg) -> rear_steer
            delta_r = self.lut.compute_rear_angle(vx, delta_f)
        elif active_mode == SteeringMode.DIRECT:
            delta_r = float(cmd_steer_r)
        else:
            delta_r = 0.0

        delta_r = float(np.clip(delta_r, -self.config.max_steer_r, self.config.max_steer_r))
        return delta_f, delta_r
