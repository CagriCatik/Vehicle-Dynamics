"""Regenerative slip control and drag-torque control, MSR (Plan V4.0 M17.4).

Negative machine torque (regenerative braking, or the coast "drag" torque of one-pedal driving) can
lock a wheel on a slippery road. This controller limits the regenerative torque per wheel; it is
applied by the actuator chain (:mod:`vehicle_model.simulation.actuator_chain`) through the brake-by-wire
actuator's ``regen_limit`` and never adds a torque path of its own.

Policies (``IntegratedBrakeConfig.regen_policy``):

    cut_on_abs   baseline, the existing brake-by-wire behaviour: regen to 0 on a wheel while ABS works on it;
                 the coast drag torque is not slip-controlled (only cut under ABS)
    modulated    a slip loop (the existing ``WheelSlipController`` in brake mode, reused) reduces the regen
                 torque of a wheel whose slip exceeds the target; under a driver brake demand the reduced
                 regen is handed over to the friction brake (the actuator blends ``friction = demand - regen``),
                 the coast drag torque is simply reduced (no hydraulic replacement: the driver did not brake)

Information: the slip loop reads the wheel slip ``kappa`` of the telemetry it is given (an idealised
wheel-slip measurement). It is therefore declared ``wheel_slip_ideal`` and must not be combined with the
sensor-constrained information mode C (rejected in the experiment specification).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from vehicle_model.controllers.slip_control import SlipControlConfig, WheelSlipController

INFORMATION = "wheel_slip_ideal"


@dataclass(frozen=True)
class RegenSlipConfig:
    policy: str = "cut_on_abs"            # "cut_on_abs" | "modulated"
    coast_torque: float = 0.0             # [N m] per driven wheel: drag torque with both pedals released (0: off)
    driven: tuple = (True, True, True, True)     # wheels with an electric machine (the reference car: four motors)
    target_slip: float = 0.04             # [-] |kappa| target of the regen slip loop (modulated)
    kp: float = 4.0e3                     # [N m] per unit slip
    ki: float = 8.0e4                     # [N m/s] per unit slip
    min_speed: float = 1.5                # [m/s] below: no regen, no loop

    def __post_init__(self):
        if self.policy not in ("cut_on_abs", "modulated"):
            raise ValueError("policy must be cut_on_abs or modulated")
        if self.coast_torque < 0 or not 0.0 < self.target_slip < 0.5:
            raise ValueError("coast_torque must be >= 0 and target_slip in (0, 0.5)")


class RegenSlipController:
    information = INFORMATION

    def __init__(self, config: RegenSlipConfig | None = None, max_regen_torque: float = 500.0):
        self.config = config or RegenSlipConfig()
        self.max_regen = float(max_regen_torque)
        c = self.config
        self.loop = WheelSlipController(SlipControlConfig(mode="brake", kp=c.kp, ki=c.ki, mu_table=(0.0, 1.0),
                                                          slip_table=(c.target_slip, c.target_slip),
                                                          min_speed=c.min_speed))
        self.reset()

    def reset(self) -> None:
        self.loop.reset()
        self.active = np.zeros(4, dtype=bool)

    def coast_request(self, target, vx: float) -> np.ndarray:
        """Drag torque per wheel [N m] (>= 0, braking) while both pedals are released."""
        c = self.config
        released = target is not None and target.driver_accel_pedal < 0.01 and target.driver_brake_pedal < 0.01
        if c.coast_torque <= 0.0 or not released or abs(vx) < c.min_speed:
            return np.zeros(4)
        return np.where(np.array(c.driven), c.coast_torque, 0.0)

    def regen_limit(self, abs_active, telemetry, vx: float, dt: float) -> np.ndarray:
        """Upper limit of the regenerative torque per wheel [N m] for this step."""
        c = self.config
        cap = np.where(np.array(c.driven), self.max_regen, 0.0)
        if c.policy == "cut_on_abs":
            if abs_active is not None:
                cap = np.where(np.asarray(abs_active, bool), 0.0, cap)
            self.active = np.zeros(4, dtype=bool) if abs_active is None else np.asarray(abs_active, bool) & (cap == 0.0)
            return cap
        cap = self.loop.update(cap, telemetry, vx, dt)             # reduces the cap while |kappa| > target
        self.active = self.loop.active.copy()
        return cap


__all__ = ("INFORMATION", "RegenSlipConfig", "RegenSlipController")
