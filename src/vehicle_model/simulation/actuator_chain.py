"""Actuator dynamics between a controller and the plant (Plan V4.0 M16, work package 3 and 6).

``ActuatorChain`` wraps any ``VehicleController``. The controller's output is the *issued*
command; the chain passes it through the existing actuator models and hands the *realised*
command to the plant:

    steer_f, steer_r   SteeringActuator (lag, rate limit, transport delay), one per axle
    T_drive            ElectricPowertrainActuator (lag, slew rate, torque/power envelope, battery limit)
    T_brake            BrakeByWireActuator (asymmetric hydraulic build/dump, regenerative blending;
                       the controller's ABS flags make it drop regen on that wheel)

**Regenerative torque enters exactly once** (Plan V4.0 2.2 F). Both the friction and the regen
torque of the brake actuator are braking torques (>= 0, against the wheel rotation); they are
passed to the plant as ``T_brake = friction + regen``. The powertrain actuator delivers only
drive torque (>= 0) into ``T_drive`` and has no regen path of its own, so the same negative
machine torque cannot appear twice. Per wheel the machine torque is ``T_drive - T_regen``; its
mechanical power and the recovered energy are reported (``regen_power_w``), but no battery or
thermal model is claimed.

With ``ideal=True`` the chain is a pass-through (bit-identical to the unwrapped controller,
tested); the existing ideal behaviour therefore stays the comparison baseline.

Diagnostics (``diagnostics["trace"]``, extended): ``realised`` = ``(F_x, M_z)`` of the realised
wheel torques through the same effectiveness matrix as the allocation, and the torques at both
stages (``issued_torque``, ``realised_torque``), the friction/regen split and actuator flags.
"""

from __future__ import annotations

import copy
from dataclasses import dataclass, field

import numpy as np

from vehicle_model.actuators.brake_actuator import BrakeActuatorConfig, BrakeByWireActuator
from vehicle_model.actuators.powertrain_actuator import ElectricDriveConfig, ElectricPowertrainActuator
from vehicle_model.actuators.steering_actuator import SteeringActuator, SteeringActuatorConfig


@dataclass
class ActuatorChainConfig:
    brake: BrakeActuatorConfig = field(default_factory=BrakeActuatorConfig)
    drive: ElectricDriveConfig = field(default_factory=ElectricDriveConfig)
    steer_front: SteeringActuatorConfig = field(default_factory=SteeringActuatorConfig)
    steer_rear: SteeringActuatorConfig = field(
        default_factory=lambda: SteeringActuatorConfig(tau=0.035, max_angle=np.radians(10.0), max_rate=np.radians(35.0)))
    ideal: bool = False

    @classmethod
    def from_yaml(cls, ideal: bool = False) -> "ActuatorChainConfig":
        """Bundled actuator configurations (configs/braking, configs/powertrains, configs/steering)."""
        return cls(brake=BrakeActuatorConfig.from_yaml(), drive=ElectricDriveConfig.from_yaml(), ideal=ideal)


def generalised_forces(params, T_drive, T_brake, steer_angles) -> np.ndarray:
    """``(F_x, M_z)`` of wheel torques through the allocation's effectiveness matrix (longitudinal forces only)."""
    from vehicle_model.allocation import WheelForceAllocator
    from vehicle_model.vehicle_models.normal_loads import wheel_contact_positions
    F = (np.asarray(T_drive, float) - np.asarray(T_brake, float)) / params.Reff
    return WheelForceAllocator(wheel_contact_positions(params), params.Reff).effectiveness(np.asarray(steer_angles, float)) @ F


class ActuatorChain:
    """VehicleController wrapper: issued command -> actuator dynamics -> realised command."""

    def __init__(self, inner, params, config: ActuatorChainConfig | None = None):
        self.inner, self.params = inner, params
        self.config = config or ActuatorChainConfig()
        self.information = getattr(inner, "information", None)
        c = self.config
        if c.ideal:
            self.brake = self.drive = self.front = self.rear = None
        else:
            self.brake = BrakeByWireActuator(copy.deepcopy(c.brake))
            self.drive = ElectricPowertrainActuator(copy.deepcopy(c.drive))
            self.front = SteeringActuator(copy.deepcopy(c.steer_front))
            self.rear = SteeringActuator(copy.deepcopy(c.steer_rear))
        self.reset()

    @property
    def name(self) -> str:
        n = getattr(self.inner, "name", type(self.inner).__name__)
        return n if self.config.ideal else f"{n} [actuator dynamics]"

    def reset(self) -> None:
        self.inner.reset()
        for a in (self.brake, self.drive):
            if a is not None:
                a.reset()
        for a in (self.front, self.rear):
            if a is not None:
                a.reset(0.0)

    def compute_control(self, state, target, telemetry=None, dt: float = 0.01):
        out = self.inner.compute_control(state, target, telemetry, dt)
        if self.config.ideal:
            return out
        issued_drive = np.asarray(out.T_drive, float).copy()
        issued_brake = np.asarray(out.T_brake, float).copy()
        issued_steer = (float(out.steer_f), float(out.steer_r))
        abs_active = (out.diagnostics or {}).get("abs_active")
        friction, regen = self.brake.step(issued_brake, float(state.vx), abs_active, dt)
        drive = self.drive.step(issued_drive, np.asarray(state.omega, float), dt)
        sf, sr = self.front.step(issued_steer[0], dt), self.rear.step(issued_steer[1], dt)
        realised_brake = friction + regen                      # regen enters here and nowhere else
        res = copy.copy(out)
        res.steer_f, res.steer_r = float(sf), float(sr)
        res.T_drive, res.T_brake = drive, realised_brake
        steer_w = np.array([sf, sf, sr, sr])
        realised = generalised_forces(self.params, drive, realised_brake, steer_w)
        res.diagnostics = dict(out.diagnostics or {})
        trace = dict(res.diagnostics.get("trace") or {})
        if "issued" not in trace:                             # controllers without their own trace (e.g. passive)
            trace["issued"] = generalised_forces(self.params, issued_drive, issued_brake,
                                                 np.array([issued_steer[0]] * 2 + [issued_steer[1]] * 2)).tolist()
        omega = np.abs(np.asarray(state.omega, float))
        trace.update({"realised": realised.tolist(), "issued_torque": (issued_drive - issued_brake).tolist(),
                      "realised_torque": (drive - realised_brake).tolist(), "issued_steer": list(issued_steer),
                      "realised_steer": [float(sf), float(sr)], "friction_torque": friction.tolist(),
                      "regen_torque": regen.tolist(), "regen_power_w": float(np.sum(regen * omega)),
                      "drive_power_limited": bool(self.drive.telemetry.power_limited),
                      "steer_rate_limited": bool(self.front.saturated_rate or self.rear.saturated_rate)})
        res.diagnostics["trace"] = trace
        res.diagnostics["actuators"] = "dynamic"
        return res


__all__ = ("ActuatorChain", "ActuatorChainConfig", "generalised_forces")
