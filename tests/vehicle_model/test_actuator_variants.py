"""M5: ideal actuator variants (Plan.md, Abschnitt 9) and the steering ratio."""

from __future__ import annotations

import math

import numpy as np
import pytest

from vehicle_model.actuators.brake_actuator import BrakeActuatorConfig, BrakeByWireActuator
from vehicle_model.actuators.powertrain_actuator import ElectricDriveConfig, ElectricPowertrainActuator
from vehicle_model.actuators.steering_actuator import SteeringActuator, SteeringActuatorConfig


def test_ideal_steering_follows_instantly_but_keeps_angle_limit_and_faults():
    a = SteeringActuator(SteeringActuatorConfig(ideal=True, delay_s=0.1))
    assert a.step(0.2, 0.005) == 0.2                                     # no lag, rate limit or delay
    assert a.step(1.0, 0.005) == pytest.approx(a.config.max_angle)       # angle limit
    a.set_fault("offset", 0.01)
    assert a.step(0.1, 0.005) == pytest.approx(0.11)


def test_dynamic_steering_is_slower_than_ideal():
    a = SteeringActuator(SteeringActuatorConfig())
    assert 0.0 < a.step(0.2, 0.005) < 0.2


def test_steering_ratio_maps_handwheel_to_road_wheel():
    a = SteeringActuator(SteeringActuatorConfig(ideal=True, steering_ratio=15.0))
    assert a.step_handwheel(math.radians(90.0), 0.005) == pytest.approx(math.radians(6.0))
    with pytest.raises(ValueError):
        SteeringActuator(SteeringActuatorConfig(steering_ratio=0.0)).road_wheel_command(0.1)


def test_ideal_brake_delivers_the_command_including_regen_split():
    b = BrakeByWireActuator(BrakeActuatorConfig(ideal=True))
    friction, regen = b.step(np.array([1000.0, 200.0, 0.0, 5000.0]), vx=20.0, dt=0.005)
    assert np.allclose(friction + regen, [1000.0, 200.0, 0.0, b.config.max_brake_torque])
    dyn = BrakeByWireActuator(BrakeActuatorConfig())
    f2, r2 = dyn.step(np.full(4, 1000.0), vx=20.0, dt=0.005)
    assert np.all(f2 + r2 < 1000.0)


def test_regen_time_constant_is_a_parameter():
    slow = BrakeByWireActuator(BrakeActuatorConfig(tau_regen=0.2))
    fast = BrakeByWireActuator(BrakeActuatorConfig(tau_regen=0.01))
    _, r_slow = slow.step(np.full(4, 400.0), vx=20.0, dt=0.005)
    _, r_fast = fast.step(np.full(4, 400.0), vx=20.0, dt=0.005)
    assert np.all(r_slow < r_fast)


def test_ideal_drive_keeps_torque_and_power_limits():
    d = ElectricPowertrainActuator(ElectricDriveConfig(ideal=True))
    omega = np.full(4, 10.0)
    assert np.allclose(d.step(np.full(4, 300.0), omega, 0.005), 300.0)
    T = d.step(np.full(4, 5000.0), np.full(4, 100.0), 0.005)
    assert np.allclose(T, d.get_max_torque(np.full(4, 100.0)))      # field-weakening limit
