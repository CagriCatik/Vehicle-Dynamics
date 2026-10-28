"""Yaw stability and torque vectoring through bounded control allocation (Plan.md, Abschnitte 9-10; M5).

Actuator architecture assumed: by-wire, i.e. drive and brake torque of every
wheel can be set individually (in-wheel or per-wheel motors plus brake-by-wire).
Steering stays with the driver.

Control chain per step:

1. Reference yaw rate from the driver's steer angle (linear single track with a
   reference understeer gradient ``EG_ref``), limited to ``r_max = k_mu mu_hat g / v``:
       r_ref = v delta / (L + EG_ref v^2)
2. Yaw-moment demand, PI on the yaw-rate error with anti-windup:
       M_z = K_p (r_ref - r) + K_i int(r_ref - r) dt,   |M_z| <= M_max
   With ``deadband > 0`` the controller acts like an ESC (only for large errors);
   with ``deadband = 0`` it vectors torque continuously (TV).
3. Longitudinal demand from the pedals: ``F_x = (a_pedal 4 T_drive,max - b_pedal 4 T_brake,max)/R``.
4. Allocation (:class:`~vehicle_model.allocation.WheelForceAllocator`): yaw moment
   before longitudinal force, within actuator limits, actuator health and each
   wheel's remaining friction circle. Without yaw demand this yields a brake
   distribution proportional to the available grip.
5. Inner loop: ABS and TCS (:class:`~vehicle_model.controllers.slip_control.WheelSlipController`).

Information used: yaw rate and speed (state), and from the telemetry the wheel
loads, lateral forces, slips and road friction ``mu``. The road friction is
therefore known ideally. A real system needs the estimators of
:mod:`vehicle_model.estimation`.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field

import numpy as np

from vehicle_model.allocation import (
    STATUS_INFEASIBLE,
    ActuatorHealth,
    AllocatorConfig,
    WheelForceAllocator,
)
from vehicle_model.controllers.framework import ControlOutput, ControlTarget
from vehicle_model.controllers.slip_control import SlipControlConfig, WheelSlipController
from vehicle_model.core.types import SevenDOFParams, SevenDOFState, WheelTelemetry
from vehicle_model.vehicle_models.normal_loads import quasi_static_normal_loads, wheel_contact_positions


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


class AllocatedStabilityController:
    """ESC / torque vectoring with allocation feedback; implements the VehicleController protocol."""

    def __init__(self, params: SevenDOFParams, config: StabilityControlConfig | None = None,
                 health: ActuatorHealth | None = None):
        self.params = params
        self.config = config or StabilityControlConfig()
        alloc_cfg = self.config.allocator
        if alloc_cfg.max_drive_torque > params.max_drive_torque or alloc_cfg.max_brake_torque > params.max_brake_torque:
            raise ValueError("allocator limits exceed the vehicle's actuator limits")
        self.allocator = WheelForceAllocator(wheel_contact_positions(params), params.Reff, alloc_cfg)
        self.health = health or ActuatorHealth()
        self.abs = WheelSlipController(self.config.abs_config)
        self.tcs = WheelSlipController(self.config.tcs_config)
        self.reset()

    @property
    def name(self) -> str:
        mode = "ESC" if self.config.deadband > 0 else "TV"
        return f"Allocated stability control ({mode})"

    def reset(self) -> None:
        self._integral = 0.0
        self.abs.reset()
        self.tcs.reset()
        self.last_allocation = None

    def reference_yaw_rate(self, steer: float, vx: float, mu_hat: float) -> float:
        p, c = self.params, self.config
        v = max(abs(vx), 0.1)
        r = vx * steer / (p.wheelbase + c.understeer_gradient_ref * vx**2)
        r_max = c.mu_utilisation * mu_hat * p.g / v
        return float(np.clip(r, -r_max, r_max))

    def compute_control(self, state: SevenDOFState, target: ControlTarget,
                        telemetry: WheelTelemetry | None = None, dt: float = 0.01) -> ControlOutput:
        t0 = time.perf_counter()
        p, c = self.params, self.config
        steer = float(np.clip(target.driver_steer_f, -p.max_steer_f, p.max_steer_f))

        if telemetry is not None:
            Fz, mu, Fy, wheel_steer = telemetry.Fz, telemetry.mu, telemetry.Fy_wheel, telemetry.steer_angles
        else:
            Fz = quasi_static_normal_loads(p, 0.0, 0.0).Fz
            mu = np.full(4, max(target.mu_estimated, 0.05))
            Fy = np.zeros(4)
            wheel_steer = np.array([steer, steer, 0.0, 0.0])
        mu_hat = float(np.mean(mu))

        # 1-2 yaw-moment demand
        Mz_des = 0.0
        r_ref = self.reference_yaw_rate(steer, state.vx, mu_hat)
        if c.enable_yaw_control and abs(state.vx) >= c.min_speed:
            e = r_ref - state.r
            if abs(e) > c.deadband:
                e_eff = e - np.sign(e) * c.deadband
                Mz_unsat = c.kp_yaw * e_eff + c.ki_yaw * self._integral
                if abs(Mz_unsat) < c.max_yaw_moment:  # anti-windup: integrate only when not saturated
                    self._integral += e_eff * dt
                Mz_des = float(np.clip(c.kp_yaw * e_eff + c.ki_yaw * self._integral, -c.max_yaw_moment, c.max_yaw_moment))
            else:
                self._integral = 0.0
        else:
            self._integral = 0.0

        # 3 longitudinal demand
        acfg = c.allocator
        n_driven = sum(acfg.driven)
        Fx_des = (target.driver_accel_pedal * n_driven * acfg.max_drive_torque
                  - target.driver_brake_pedal * 4.0 * acfg.max_brake_torque) / p.Reff

        # 4 allocation
        alloc = self.allocator.allocate(Fx_des, Mz_des, wheel_steer, Fz, mu, Fy, self.health)
        self.last_allocation = alloc
        T_drive, T_brake = alloc.T_drive, alloc.T_brake

        # 5 slip control
        if c.enable_slip_control:
            T_brake = self.abs.update(T_brake, telemetry, state.vx, dt)
            T_drive = self.tcs.update(T_drive, telemetry, state.vx, dt)

        F_final = (T_drive - T_brake) / p.Reff
        achieved_Mz = float(self.allocator.effectiveness(wheel_steer)[1] @ F_final)
        status = alloc.status
        if status != STATUS_INFEASIBLE and (np.any(self.abs.active) or np.any(self.tcs.active)):
            status = "saturated"
        return ControlOutput(
            steer_f=steer, steer_r=0.0, T_drive=T_drive, T_brake=T_brake,
            desired_yaw_moment=Mz_des, allocated_yaw_moment=achieved_Mz, status=status,
            computation_time_ms=(time.perf_counter() - t0) * 1e3,
            diagnostics={
                "r_ref": r_ref, "Fx_demand": Fx_des, "allocation_status": alloc.status,
                "allocation_residual": alloc.residual, "at_bound": alloc.at_bound,
                "grip_limited": alloc.grip_limited, "abs_active": self.abs.active.copy(),
                "tcs_active": self.tcs.active.copy(),
            },
        )


__all__ = ("AllocatedStabilityController", "StabilityControlConfig")
