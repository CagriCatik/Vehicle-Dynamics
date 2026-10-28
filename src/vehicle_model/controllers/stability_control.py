"""Yaw stability and torque vectoring through bounded control allocation (Plan.md, Abschnitte 9-10; M5).

:class:`AllocatedStabilityController` is the motion-control chain of
:mod:`.motion_control` with the yaw-rate PI law of :mod:`.yaw_laws`:

1. Reference yaw rate ``r_ref = v delta/(L + EG_ref v^2)``, limited to ``k_mu mu_hat g/v``.
2. Yaw-moment demand, PI with anti-windup; with ``deadband > 0`` it acts like an ESC
   (only for large errors), with ``deadband = 0`` it vectors torque continuously.
3. Longitudinal demand from the pedals.
4. Allocation (:class:`~vehicle_model.allocation.WheelForceAllocator`): yaw moment
   before longitudinal force, within actuator limits, actuator health and each
   wheel's remaining friction circle.
5. Inner loop: ABS and TCS (:class:`~vehicle_model.controllers.slip_control.WheelSlipController`).

Actuator architecture assumed: by-wire, i.e. drive and brake torque of every wheel
can be set individually. Steering stays with the driver. Information used: the
vehicle state and the telemetry including road friction (ideal full-state feedback).
"""

from __future__ import annotations

from dataclasses import dataclass, field

from vehicle_model.allocation import ActuatorHealth, AllocatorConfig
from vehicle_model.controllers.motion_control import (
    INFORMATION_FULL_STATE,
    ActuationConfig,
    MotionController,
    MotionControllerConfig,
    MotionRequest,
)
from vehicle_model.controllers.slip_control import SlipControlConfig
from vehicle_model.controllers.yaw_laws import YawRatePI
from vehicle_model.core.types import SevenDOFParams


@dataclass(frozen=True)
class StabilityControlConfig:
    understeer_gradient_ref: float = 0.0010   # [rad/(m/s^2)] reference EG (~0.56 deg/g)
    kp_yaw: float = 15000.0                   # [N m/(rad/s)]  95 % of the TV reference in 0.17 s, no overshoot
    ki_yaw: float = 100000.0                  # [N m/rad]
    max_yaw_moment: float = 4000.0            # [N m]
    deadband: float = 0.0                     # [rad/s] 0: continuous TV; > 0: ESC-like intervention
    mu_utilisation: float = 0.85              # [-] k_mu in the yaw-rate limit
    min_speed: float = 3.0                    # [m/s] no yaw control below
    allocator: AllocatorConfig = field(default_factory=AllocatorConfig)
    abs_config: SlipControlConfig = field(default_factory=lambda: SlipControlConfig(mode="brake"))
    tcs_config: SlipControlConfig = field(default_factory=lambda: SlipControlConfig(mode="drive"))
    enable_yaw_control: bool = True
    enable_slip_control: bool = True


class _NoYawControl:
    name = "no yaw control"
    information = INFORMATION_FULL_STATE

    def reset(self) -> None:
        pass

    def request(self, state, ctx) -> MotionRequest:
        return MotionRequest()


class AllocatedStabilityController(MotionController):
    """ESC / torque vectoring with allocation feedback; implements the VehicleController protocol."""

    def __init__(self, params: SevenDOFParams, config: StabilityControlConfig | None = None,
                 health: ActuatorHealth | None = None):
        self.config = c = config or StabilityControlConfig()
        law = (YawRatePI(kp=c.kp_yaw, ki=c.ki_yaw, deadband=c.deadband,
                         name=f"Allocated stability control ({'ESC' if c.deadband > 0 else 'TV'})")
               if c.enable_yaw_control else _NoYawControl())
        mc = MotionControllerConfig(
            understeer_gradient_ref=c.understeer_gradient_ref, mu_utilisation=c.mu_utilisation,
            max_yaw_moment=c.max_yaw_moment, min_speed=c.min_speed,
            actuation=ActuationConfig(c.allocator, c.abs_config, c.tcs_config, c.enable_slip_control),
        )
        super().__init__(params, law, mc, health)

    @property
    def abs(self):
        return self.actuation.abs

    @property
    def tcs(self):
        return self.actuation.tcs


__all__ = ("AllocatedStabilityController", "StabilityControlConfig")
