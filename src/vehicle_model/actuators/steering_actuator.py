"""Steering actuator models with first-order dynamics, rate limiting, and failure modes.

Implements EPS (Electric Power Steering) and Steer-by-Wire actuator physical response.1 ().
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
import numpy as np


@dataclass
class SteeringActuatorConfig:
    """Parameters defining steering actuator dynamics and physical limits."""

    tau: float = 0.045                    # Actuator first-order time constant [s] (~45 ms for modern EPS)
    max_angle: float = math.radians(35.0) # Maximum steering angle limit [rad]
    max_rate: float = math.radians(60.0)  # Maximum road-wheel slew rate [rad/s] (60 deg/s)
    delay_s: float = 0.0                  # Pure transport delay [s]
    steering_ratio: float = 1.0           # Handwheel / road-wheel angle [-]; 1.0: commands are road-wheel angles
    ideal: bool = False                   # True: no lag, rate limit or delay (angle limit and faults still apply)
    fault_mode: str = "none"              # "none", "stuck", "free", "offset"
    fault_val: float = 0.0                # Stuck angle or additive offset [rad]


class SteeringActuator:
    """Front or rear steering actuator with lag, rate limit, transport delay, and fault modes."""

    def __init__(self, config: SteeringActuatorConfig | None = None, initial_angle: float = 0.0):
        self.config = config or SteeringActuatorConfig()
        self.angle: float = float(np.clip(initial_angle, -self.config.max_angle, self.config.max_angle))
        self.rate: float = 0.0
        self.target: float = self.angle

        # Pure transport delay buffer: deque of (time, commanded_angle)
        self._delay_queue: deque[tuple[float, float]] = deque([(0.0, self.angle)])
        self._current_time: float = 0.0

        # Diagnostics / saturation flags
        self.saturated_angle: bool = False
        self.saturated_rate: bool = False

    def reset(self, initial_angle: float = 0.0) -> None:
        """Reset actuator internal state and delay history."""
        self.angle = float(np.clip(initial_angle, -self.config.max_angle, self.config.max_angle))
        self.rate = 0.0
        self.target = self.angle
        self._delay_queue = deque([(0.0, self.angle)])
        self._current_time = 0.0
        self.saturated_angle = False
        self.saturated_rate = False

    def set_fault(self, mode: str, value: float = 0.0) -> None:
        """Inject actuator fault mode ("none", "stuck", "free", "offset")."""
        if mode not in ("none", "stuck", "free", "offset"):
            raise ValueError(f"Unknown fault mode {mode!r}")
        self.config.fault_mode = mode
        self.config.fault_val = value

    def road_wheel_command(self, handwheel_angle: float) -> float:
        """Road-wheel angle commanded by a handwheel angle [rad] (constant ratio)."""
        if self.config.steering_ratio <= 0.0:
            raise ValueError("steering_ratio must be positive")
        return handwheel_angle / self.config.steering_ratio

    def step_handwheel(self, handwheel_angle: float, dt: float) -> float:
        """Like :meth:`step`, commanded by a handwheel angle; returns the road-wheel angle [rad]."""
        return self.step(self.road_wheel_command(handwheel_angle), dt)

    def step(self, cmd_angle: float, dt: float) -> float:
        """Step actuator dynamics forward by dt seconds and return actual wheel angle [rad]."""
        if dt <= 0.0:
            return self.angle

        self._current_time += dt

        # 1. Handle transport delay
        if self.config.delay_s > 1e-6 and not self.config.ideal:
            self._delay_queue.append((self._current_time, cmd_angle))
            delayed_time = self._current_time - self.config.delay_s
            while len(self._delay_queue) > 1 and self._delay_queue[1][0] <= delayed_time:
                self._delay_queue.popleft()
            delayed_cmd = self._delay_queue[0][1]
        else:
            delayed_cmd = cmd_angle

        # 2. Handle fault modes
        if self.config.fault_mode == "stuck":
            self.angle = float(np.clip(self.config.fault_val, -self.config.max_angle, self.config.max_angle))
            self.rate = 0.0
            return self.angle
        elif self.config.fault_mode == "offset":
            effective_cmd = delayed_cmd + self.config.fault_val
        elif self.config.fault_mode == "free":
            # Self-centering / passive wheel trailing to zero angle
            effective_cmd = 0.0
        else:
            effective_cmd = delayed_cmd

        # 3. Position limit clamping of target
        target = float(np.clip(effective_cmd, -self.config.max_angle, self.config.max_angle))
        self.target = target
        if self.config.ideal:
            self.saturated_rate = False
            self.saturated_angle = abs(target) >= (self.config.max_angle - 1e-9)
            self.rate = (target - self.angle) / dt
            self.angle = target
            return self.angle

        # 4. First-order response: d(angle)/dt = (target - angle) / tau
        tau = max(self.config.tau, 1e-4)
        desired_rate = (target - self.angle) / tau

        # 5. Rate limiting: |rate| <= max_rate
        max_rate = self.config.max_rate
        actual_rate = float(np.clip(desired_rate, -max_rate, max_rate))
        self.saturated_rate = abs(actual_rate) >= (max_rate - 1e-9) and abs(desired_rate) > abs(actual_rate)

        # 6. Integration and physical stroke clamping
        new_angle = self.angle + actual_rate * dt
        clamped_angle = float(np.clip(new_angle, -self.config.max_angle, self.config.max_angle))
        self.saturated_angle = abs(clamped_angle) >= (self.config.max_angle - 1e-9)

        # Update state
        self.rate = (clamped_angle - self.angle) / dt
        self.angle = clamped_angle
        return self.angle


__all__ = ("SteeringActuator", "SteeringActuatorConfig")
