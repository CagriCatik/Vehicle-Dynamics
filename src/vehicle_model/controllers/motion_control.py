"""Vehicle motion control chain with exchangeable control laws (Plan.md, Abschnitt 10; M6).

The chain separates (Plan.md 10):

    driver request (steer, pedals)
      -> target motion            YawReference: r_ref from a linear single track with EG_ref
      -> motion control law       MotionLaw: yaw moment and/or rear steer (PI, LQR, MPC, ...)
      -> control allocation       ActuationLayer: WheelForceAllocator (M5)
      -> actuator-level control   ActuationLayer: ABS/TCS slip control (M5)
      -> vehicle physics          plant, outside this module

Every law runs on the same reference and the same actuation layer, so controllers
are compared on equal terms. Laws see the vehicle state and the telemetry
(wheel loads, lateral forces, slips, road friction): this is an idealised
full-state-feedback setting and is declared as such in ``information``.

Output status: the law's solver status if it is not nominal (see
solver_diagnostics), otherwise the allocation status (nominal / saturated /
infeasible); both are in the diagnostics.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Protocol, runtime_checkable

import numpy as np

from vehicle_model.allocation import ActuatorHealth, AllocatorConfig, WheelForceAllocator
from vehicle_model.controllers.framework import ControlOutput, ControlTarget
from vehicle_model.controllers.slip_control import SlipControlConfig, WheelSlipController
from vehicle_model.core.types import SevenDOFParams, SevenDOFState, WheelTelemetry
from vehicle_model.vehicle_models.normal_loads import quasi_static_normal_loads, wheel_contact_positions

INFORMATION_FULL_STATE = "full_state_ideal"  # state and telemetry incl. road friction, no sensor model


@dataclass(frozen=True)
class YawReference:
    """Target yaw rate ``r_ref = v delta/(L + EG_ref v^2)``, limited to ``k_mu mu_hat g / v``; target sideslip 0."""

    wheelbase: float
    g: float = 9.81
    understeer_gradient: float = 0.0010   # [rad/(m/s^2)] ~0.56 deg/g
    mu_utilisation: float = 0.85

    def yaw_rate(self, steer: float, vx: float, mu_hat: float) -> float:
        v = max(abs(vx), 0.1)
        r = vx * steer / (self.wheelbase + self.understeer_gradient * vx**2)
        r_max = self.mu_utilisation * mu_hat * self.g / v
        return float(np.clip(r, -r_max, r_max))


@dataclass(frozen=True)
class MotionContext:
    """What a law gets each step (besides the state)."""

    steer_f: float          # [rad] driver front steer
    r_ref: float            # [rad/s]
    mu_hat: float           # [-]
    dt: float               # [s]
    max_yaw_moment: float   # [N m]
    max_rear_steer: float   # [rad]


@dataclass(frozen=True)
class MotionRequest:
    yaw_moment: float = 0.0       # [N m] demand to the allocation
    rear_steer: float = 0.0       # [rad] rear road-wheel angle command
    status: str = "nominal"       # law/solver status
    diagnostics: dict = field(default_factory=dict)


@runtime_checkable
class MotionLaw(Protocol):
    name: str
    information: str

    def reset(self) -> None: ...

    def request(self, state: SevenDOFState, ctx: MotionContext) -> MotionRequest: ...


@dataclass(frozen=True)
class ActuationConfig:
    allocator: AllocatorConfig = field(default_factory=AllocatorConfig)
    abs_config: SlipControlConfig = field(default_factory=lambda: SlipControlConfig(mode="brake"))
    tcs_config: SlipControlConfig = field(default_factory=lambda: SlipControlConfig(mode="drive"))
    enable_slip_control: bool = True


class ActuationLayer:
    """Allocation of (F_x, M_z) to wheel torques plus ABS/TCS (shared by all laws)."""

    def __init__(self, params: SevenDOFParams, config: ActuationConfig | None = None,
                 health: ActuatorHealth | None = None):
        self.params = params
        self.config = config or ActuationConfig()
        a = self.config.allocator
        if a.max_drive_torque > params.max_drive_torque or a.max_brake_torque > params.max_brake_torque:
            raise ValueError("allocator limits exceed the vehicle's actuator limits")
        self.allocator = WheelForceAllocator(wheel_contact_positions(params), params.Reff, a)
        self.health = health or ActuatorHealth()
        self.abs = WheelSlipController(self.config.abs_config)
        self.tcs = WheelSlipController(self.config.tcs_config)
        self.last_allocation = None

    def reset(self) -> None:
        self.abs.reset()
        self.tcs.reset()
        self.last_allocation = None

    def actuate(self, Fx_des: float, Mz_des: float, wheel_steer, Fz, mu, Fy, telemetry, vx, dt):
        alloc = self.allocator.allocate(Fx_des, Mz_des, wheel_steer, Fz, mu, Fy, self.health)
        self.last_allocation = alloc
        T_drive, T_brake = alloc.T_drive, alloc.T_brake
        if self.config.enable_slip_control:
            T_brake = self.abs.update(T_brake, telemetry, vx, dt)
            T_drive = self.tcs.update(T_drive, telemetry, vx, dt)
        F = (T_drive - T_brake) / self.params.Reff
        achieved_Mz = float(self.allocator.effectiveness(wheel_steer)[1] @ F)
        slip_active = bool(np.any(self.abs.active) or np.any(self.tcs.active))
        return T_drive, T_brake, alloc, achieved_Mz, slip_active


@dataclass(frozen=True)
class MotionControllerConfig:
    understeer_gradient_ref: float = 0.0010   # [rad/(m/s^2)]
    mu_utilisation: float = 0.85
    max_yaw_moment: float = 4000.0            # [N m]
    min_speed: float = 3.0                    # [m/s] law inactive below
    actuation: ActuationConfig = field(default_factory=ActuationConfig)


class MotionController:
    """Driver request -> reference -> law -> allocation -> slip control; a VehicleController."""

    def __init__(self, params: SevenDOFParams, law: MotionLaw, config: MotionControllerConfig | None = None,
                 health: ActuatorHealth | None = None):
        self.params = params
        self.law = law
        self.mc_config = config or MotionControllerConfig()
        c = self.mc_config
        self.reference = YawReference(params.wheelbase, params.g, c.understeer_gradient_ref, c.mu_utilisation)
        self.actuation = ActuationLayer(params, c.actuation, health)

    @property
    def name(self) -> str:
        return self.law.name

    @property
    def information(self) -> str:
        return self.law.information

    @property
    def health(self) -> ActuatorHealth:
        return self.actuation.health

    @property
    def last_allocation(self):
        return self.actuation.last_allocation

    def reset(self) -> None:
        self.law.reset()
        self.actuation.reset()

    def reference_yaw_rate(self, steer: float, vx: float, mu_hat: float) -> float:
        return self.reference.yaw_rate(steer, vx, mu_hat)

    def compute_control(self, state: SevenDOFState, target: ControlTarget,
                        telemetry: WheelTelemetry | None = None, dt: float = 0.01) -> ControlOutput:
        t0 = time.perf_counter()
        p, c = self.params, self.mc_config
        steer = float(np.clip(target.driver_steer_f, -p.max_steer_f, p.max_steer_f))
        if telemetry is not None:
            Fz, mu, Fy, wheel_steer = telemetry.Fz, telemetry.mu, telemetry.Fy_wheel, telemetry.steer_angles
        else:
            Fz = quasi_static_normal_loads(p, 0.0, 0.0).Fz
            mu = np.full(4, max(target.mu_estimated, 0.05))
            Fy, wheel_steer = np.zeros(4), np.array([steer, steer, 0.0, 0.0])
        mu_hat = float(np.mean(mu))
        r_ref = self.reference.yaw_rate(steer, state.vx, mu_hat)

        if abs(state.vx) >= c.min_speed:
            ctx = MotionContext(steer, r_ref, mu_hat, dt, c.max_yaw_moment, p.max_steer_r)
            req = self.law.request(state, ctx)
        else:
            self.law.reset()
            req = MotionRequest()
        Mz_des = float(np.clip(req.yaw_moment, -c.max_yaw_moment, c.max_yaw_moment))
        steer_r = float(np.clip(req.rear_steer, -p.max_steer_r, p.max_steer_r))

        acfg = c.actuation.allocator
        Fx_des = (target.driver_accel_pedal * sum(acfg.driven) * acfg.max_drive_torque
                  - target.driver_brake_pedal * 4.0 * acfg.max_brake_torque) / p.Reff
        T_drive, T_brake, alloc, achieved_Mz, slip_active = self.actuation.actuate(
            Fx_des, Mz_des, wheel_steer, Fz, mu, Fy, telemetry, state.vx, dt)

        status = alloc.status
        if status != "infeasible" and slip_active:
            status = "saturated"
        if req.status != "nominal":
            status = req.status
        return ControlOutput(
            steer_f=steer, steer_r=steer_r, T_drive=T_drive, T_brake=T_brake,
            desired_yaw_moment=Mz_des, allocated_yaw_moment=achieved_Mz, status=status,
            computation_time_ms=(time.perf_counter() - t0) * 1e3,
            diagnostics={
                "r_ref": r_ref, "Fx_demand": Fx_des, "law_status": req.status, "law": req.diagnostics,
                "allocation_status": alloc.status, "allocation_residual": alloc.residual,
                "at_bound": alloc.at_bound, "grip_limited": alloc.grip_limited,
                "abs_active": self.actuation.abs.active.copy(), "tcs_active": self.actuation.tcs.active.copy(),
                "information": self.law.information,
            },
        )


__all__ = (
    "INFORMATION_FULL_STATE",
    "ActuationConfig",
    "ActuationLayer",
    "MotionContext",
    "MotionController",
    "MotionControllerConfig",
    "MotionLaw",
    "MotionRequest",
    "YawReference",
)
