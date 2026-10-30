"""Closed-Loop Neuromuscular Preview Driver Model (Driver-in-the-Loop Dynamics).

Implements realistic human driver steering behavior combining:
1. Two-Point Visual Preview Control (Salvucci & Gray / MacAdam formulation):
   - Near-point guidance for lane keeping: e_near(t) at preview horizon T_near (~0.35 s).
   - Far-point anticipation for road curvature & heading: theta_far(t) at preview horizon T_far (~1.2-1.5 s).
2. Pure Human Neural Transport Delay (tau_d ~ 150-200 ms):
   - Discrete ring-buffer pipeline modeling cognitive perception and neural transmission latency.
3. Neuromuscular Arm Dynamics (tau_n ~ 80-120 ms):
   - First-order lag with realistic human physical slew rate limits (dot_delta_max ~ 500-600 deg/s)
     and hand-wheel stroke saturation (delta_max ~ 35 deg).
4. Driver Workload & Effort Diagnostics:
   - Evaluates steering effort (integral of delta^2 dt), control activity (integral of dot_delta^2 dt),
     and high-frequency Steering Reversal Rate (SRR).

"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
import math
from pathlib import Path
from typing import Callable, Optional, Sequence, Tuple

import numpy as np
import yaml


@dataclass
class DriverModelConfig:
    """Configuration parameters for the Neuromuscular Preview Driver Model."""

    # Visual preview horizons
    t_near_preview_s: float = 0.35       # Near-point lookahead time for lateral lane keeping [s]
    t_far_preview_s: float = 1.35        # Far-point lookahead time for road curvature anticipation [s]
    min_lookahead_dist_m: float = 3.5    # Minimum lookahead distance at low speeds [m]

    # Human neural & neuromuscular dynamics
    tau_delay_s: float = 0.18            # Pure human cognitive/neural transport delay [s] (180 ms)
    tau_neuromuscular_s: float = 0.10    # Arm/muscle neuromuscular first-order lag [s] (100 ms)
    max_steer_rad: float = math.radians(35.0)       # Handwheel/road-wheel stroke limit [rad]
    max_steer_rate_radps: float = math.radians(600.0) # Maximum human arm slew rate [rad/s]

    # Preview steering gains
    k_near_lat_error: float = 0.05       # Near error proportional gain [rad/m]
    k_far_heading: float = 0.55          # Far preview heading angle gain [-]
    k_yaw_rate_damping: float = 0.12     # Inner-loop yaw rate damping gain [rad / (rad/s)]

    # Workload threshold for steering reversal count
    reversal_rate_deadband_deg: float = 1.5 # Steering angle change required to count as deliberate reversal

    # Adaptive & Predictive Driver Intelligence (Human Cerebellar Internal Model)
    enable_speed_adaptive_gains: bool = True  # Scales k_near inversely with forward speed
    v_base_mps: float = 20.0                  # Reference speed for nominal preview gain calibration [m/s]
    enable_cerebellar_prediction: bool = True # Internal forward predictive compensation for human delay
    tau_predictive_horizon_s: float = 0.18    # Internal forward projection horizon [s]

    @classmethod
    def from_yaml(cls, path: str | Path) -> "DriverModelConfig":
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        cfg = data.get("driver_model", data)
        return cls(
            t_near_preview_s=float(cfg.get("t_near_preview_s", 0.35)),
            t_far_preview_s=float(cfg.get("t_far_preview_s", 1.35)),
            min_lookahead_dist_m=float(cfg.get("min_lookahead_dist_m", 3.5)),
            tau_delay_s=float(cfg.get("tau_delay_s", 0.18)),
            tau_neuromuscular_s=float(cfg.get("tau_neuromuscular_s", 0.10)),
            max_steer_rad=math.radians(float(cfg.get("max_steer_deg", 35.0))),
            max_steer_rate_radps=math.radians(float(cfg.get("max_steer_rate_degps", 600.0))),
            k_near_lat_error=float(cfg.get("k_near_lat_error", 0.05)),
            k_far_heading=float(cfg.get("k_far_heading", 0.55)),
            k_yaw_rate_damping=float(cfg.get("k_yaw_rate_damping", 0.12)),
            reversal_rate_deadband_deg=float(cfg.get("reversal_rate_deadband_deg", 1.5)),
            enable_speed_adaptive_gains=bool(cfg.get("enable_speed_adaptive_gains", True)),
            v_base_mps=float(cfg.get("v_base_mps", 20.0)),
            enable_cerebellar_prediction=bool(cfg.get("enable_cerebellar_prediction", True)),
            tau_predictive_horizon_s=float(cfg.get("tau_predictive_horizon_s", 0.18)),
        )


@dataclass
class DriverTelemetry:
    """Continuous telemetry output from the driver model."""

    steer_actual_rad: float              # Current actual steering angle applied to steering rack [rad]
    steer_desired_mental_rad: float      # Mental desired steering angle before neural delay [rad]
    steer_delayed_rad: float             # Delayed command reaching the neuromuscular system [rad]
    steer_rate_radps: float              # Current steering angular velocity [rad/s]
    near_lateral_error_m: float          # Lane center error at near lookahead point [m]
    far_heading_error_rad: float         # Preview heading error at far lookahead point [rad]
    driver_workload_energy: float        # Integrated steering energy: sum(delta^2 * dt) [rad^2*s]
    reversal_count: int                  # Deliberate steering reversal count (panic / workload metric)


class NeuromuscularDriverModel:
    """Human Driver Preview Steering Model with Neural Delay and Neuromuscular Lag."""

    def __init__(
        self,
        config: Optional[DriverModelConfig] = None,
        path_function: Optional[Callable[[float], Tuple[float, float]]] = None,
        dt: float = 0.005,
    ):
        """Initialize driver model.

        Args:
            config: Driver model configuration parameters.
            path_function: Callable path_function(x) -> (y_ref, psi_ref).
                           Defaults to straight centerline y=0, psi=0 if None.
            dt: Discrete simulation step size [s].
        """
        self.config = config or DriverModelConfig()
        self.path_function = path_function or (lambda x: (0.0, 0.0))
        self.dt = dt

        # State variables
        self.steer_actual = 0.0
        self.steer_rate = 0.0
        self.driver_workload_energy = 0.0
        self.reversal_count = 0
        self._last_peak_steer = 0.0
        self._last_direction = 0  # +1 for increasing, -1 for decreasing

        # Delay buffer: ring buffer holding delay steps
        delay_steps = max(1, int(round(self.config.tau_delay_s / self.dt)))
        self._delay_buffer: deque[float] = deque([0.0] * delay_steps, maxlen=delay_steps)

    def reset(self, initial_steer: float = 0.0) -> None:
        """Reset internal driver delay buffer and states."""
        self.steer_actual = initial_steer
        self.steer_rate = 0.0
        self.driver_workload_energy = 0.0
        self.reversal_count = 0
        self._last_peak_steer = initial_steer
        self._last_direction = 0
        for _ in range(self._delay_buffer.maxlen or 1):
            self._delay_buffer.append(initial_steer)

    def set_path_function(self, path_function: Callable[[float], Tuple[float, float]]) -> None:
        """Update reference target path function: path_fun(x) -> (y, psi)."""
        self.path_function = path_function

    def step(
        self,
        x: float,
        y: float,
        psi: float,
        vx: float,
        vy: float,
        r: float,
        dt: Optional[float] = None,
    ) -> DriverTelemetry:
        """Execute one simulation step of closed-loop driver preview steering.

        Args:
            x: Vehicle longitudinal position in world frame [m].
            y: Vehicle lateral position in world frame [m].
            psi: Vehicle yaw heading angle in world frame [rad].
            vx: Vehicle longitudinal speed [m/s].
            vy: Vehicle lateral speed [m/s].
            r: Vehicle yaw rate [rad/s].
            dt: Optional time step override [s].

        Returns:
            DriverTelemetry containing commanded steer angle and workload diagnostics.
        """
        step_dt = dt or self.dt
        v_lon = max(1.5, abs(vx))

        # 1. Human Cerebellar Forward Internal Model (Predictive Pose Compensation):
        # Experienced drivers predict where the vehicle will be after cognitive latency (tau_d),
        # preventing delay-induced pilot-induced oscillations (PIO) without artificial gain reduction.
        if self.config.enable_cerebellar_prediction:
            tau_pred = self.config.tau_predictive_horizon_s
            # Planar kinematics projection of vehicle CG position over cognitive delay horizon
            c_p = math.cos(psi)
            s_p = math.sin(psi)
            x_origin = x + (vx * c_p - vy * s_p) * tau_pred
            y_origin = y + (vx * s_p + vy * c_p) * tau_pred
            # Damped yaw heading projection to prevent high-frequency derivative peaking
            psi_origin = psi + 0.25 * r * tau_pred
        else:
            x_origin = x
            y_origin = y
            psi_origin = psi

        # 2. Visual Preview Lookahead Coordinates:
        # Near lookahead distance (lane center alignment)
        d_near = max(self.config.min_lookahead_dist_m, v_lon * self.config.t_near_preview_s)
        x_near = x_origin + d_near * math.cos(psi_origin)
        y_near_veh = y_origin + d_near * math.sin(psi_origin)
        y_near_target, psi_near_target = self.path_function(x_near)

        # Near lateral error in vehicle-aligned transverse frame
        e_near = (y_near_target - y_near_veh) * math.cos(psi_origin)

        # Far lookahead distance (curvature anticipation):
        d_far = max(self.config.min_lookahead_dist_m * 2.0, v_lon * self.config.t_far_preview_s)
        x_far = x_origin + d_far * math.cos(psi_origin)
        y_far_target, psi_far_target = self.path_function(x_far)

        # Far heading vector error
        dx_far = x_far - x_origin
        dy_far = y_far_target - y_origin
        psi_target_line = math.atan2(dy_far, dx_far)
        e_far_heading = (psi_target_line - psi_origin)
        # Normalize angle to [-pi, pi]
        e_far_heading = math.atan2(math.sin(e_far_heading), math.cos(e_far_heading))

        # 3. Speed-Adaptive Preview Gains:
        # Human steering sensitivity scales inversely with forward speed because
        # lateral acceleration response per steer angle increases as v_x^2.
        if self.config.enable_speed_adaptive_gains:
            k_near_eff = self.config.k_near_lat_error * (self.config.v_base_mps / max(5.0, v_lon))
        else:
            k_near_eff = self.config.k_near_lat_error

        # 4. Desired Mental Steering Angle (Two-Point Model + Yaw Rate Damping):
        steer_desired = (
            k_near_eff * e_near
            + self.config.k_far_heading * e_far_heading
            - self.config.k_yaw_rate_damping * r
        )
        steer_desired = float(np.clip(steer_desired, -self.config.max_steer_rad, self.config.max_steer_rad))

        # 3. Pure Neural Transport Delay (tau_d):
        # Push desired mental command into delay ring-buffer and pop delayed command
        self._delay_buffer.append(steer_desired)
        steer_delayed = self._delay_buffer[0]

        # 4. Neuromuscular Arm Dynamics (tau_n & rate limits):
        # First-order muscle lag: tau_n * dot_delta + delta = delta_delayed
        raw_dot_steer = (steer_delayed - self.steer_actual) / max(0.01, self.config.tau_neuromuscular_s)
        self.steer_rate = float(np.clip(
            raw_dot_steer,
            -self.config.max_steer_rate_radps,
            self.config.max_steer_rate_radps,
        ))

        # Integrate steering angle
        self.steer_actual += self.steer_rate * step_dt
        self.steer_actual = float(np.clip(
            self.steer_actual,
            -self.config.max_steer_rad,
            self.config.max_steer_rad,
        ))

        # 5. Driver Workload & Reversal Metrics:
        self.driver_workload_energy += (self.steer_actual ** 2) * step_dt

        # Steering reversal detection
        current_dir = 1 if self.steer_rate > 0.05 else (-1 if self.steer_rate < -0.05 else 0)
        deadband_rad = math.radians(self.config.reversal_rate_deadband_deg)
        if current_dir != 0 and current_dir != self._last_direction:
            if abs(self.steer_actual - self._last_peak_steer) >= deadband_rad:
                self.reversal_count += 1
                self._last_peak_steer = self.steer_actual
                self._last_direction = current_dir

        return DriverTelemetry(
            steer_actual_rad=self.steer_actual,
            steer_desired_mental_rad=steer_desired,
            steer_delayed_rad=steer_delayed,
            steer_rate_radps=self.steer_rate,
            near_lateral_error_m=e_near,
            far_heading_error_rad=e_far_heading,
            driver_workload_energy=self.driver_workload_energy,
            reversal_count=self.reversal_count,
        )
