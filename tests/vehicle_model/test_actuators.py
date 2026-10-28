"""Tests for steering, powertrain, and brake actuator dynamics and fault modes."""

from __future__ import annotations

import math
import numpy as np
import pytest

from vehicle_model.actuators import (
    BrakeActuatorConfig,
    BrakeByWireActuator,
    ElectricDriveConfig,
    ElectricPowertrainActuator,
    SteeringActuator,
    SteeringActuatorConfig,
)


def test_steering_actuator_rate_limiting():
    """Rapid steering step must be rate-limited by max_rate."""
    config = SteeringActuatorConfig(
        tau=0.01,
        max_angle=math.radians(35.0),
        max_rate=math.radians(50.0),  # 50 deg/s
    )
    actuator = SteeringActuator(config, initial_angle=0.0)

    # Step command to 30 deg
    cmd = math.radians(30.0)
    dt = 0.1  # In 0.1 s at max 50 deg/s, angle change cannot exceed 5.0 deg
    actual = actuator.step(cmd, dt=dt)

    assert actual > 0.0
    assert math.degrees(actual) <= 5.001
    assert actuator.saturated_rate


def test_steering_actuator_angle_clamping():
    """Actuator angle must not exceed max_angle mechanical limit."""
    config = SteeringActuatorConfig(max_angle=math.radians(35.0))
    actuator = SteeringActuator(config, initial_angle=0.0)

    # Command 60 degrees (exceeds 35 deg rack limit)
    cmd = math.radians(60.0)
    for _ in range(50):
        actuator.step(cmd, dt=0.05)

    assert math.isclose(actuator.angle, math.radians(35.0), rel_tol=1e-5)
    assert actuator.saturated_angle


def test_steering_actuator_fault_modes():
    """Test stuck fault and offset injection."""
    config = SteeringActuatorConfig(max_angle=math.radians(35.0))
    actuator = SteeringActuator(config, initial_angle=0.0)

    # Stuck fault at 10 deg
    stuck_angle = math.radians(10.0)
    actuator.set_fault("stuck", stuck_angle)
    actual = actuator.step(math.radians(-25.0), dt=0.1)
    assert math.isclose(actual, stuck_angle)

    # Offset fault (+5 deg)
    actuator.set_fault("offset", math.radians(5.0))
    for _ in range(30):
        actuator.step(0.0, dt=0.05)
    assert math.isclose(actuator.angle, math.radians(5.0), rel_tol=1e-3)


def test_powertrain_field_weakening_envelope():
    """Above base speed, available drive torque must decrease inversely with speed (constant power)."""
    config = ElectricDriveConfig(
        max_torque=1000.0,
        max_power=100000.0,  # 100 kW per wheel
        base_wheel_speed=30.0,
    )
    actuator = ElectricPowertrainActuator(config)

    # At low speed (20 rad/s < base 30 rad/s), full 1000 Nm is available
    t_low = actuator.get_max_torque([20.0, 20.0, 20.0, 20.0])
    assert np.allclose(t_low, 1000.0)

    # At high speed (100 rad/s), torque is limited by power: 100000 / 100 = 1000 Nm
    # At 200 rad/s: 100000 / 200 = 500 Nm
    t_high = actuator.get_max_torque([200.0, 200.0, 200.0, 200.0])
    assert np.allclose(t_high, 500.0)


def test_powertrain_battery_pack_power_limit():
    """If total commanded power exceeds battery DC limit, torques are scaled down proportionally."""
    config = ElectricDriveConfig(
        max_torque=1000.0,
        total_battery_power_limit=200000.0,  # 200 kW total battery limit
        efficiency=1.0,
    )
    actuator = ElectricPowertrainActuator(config)

    # Command 1000 Nm at 100 rad/s on all 4 wheels
    # Total mechanical power would be 4 * (1000 * 100) = 400 kW, which exceeds 200 kW
    wheel_speeds = np.array([100.0, 100.0, 100.0, 100.0])
    cmd_torques = np.array([1000.0, 1000.0, 1000.0, 1000.0])

    for _ in range(50):
        actuator.step(cmd_torques, wheel_speeds, dt=0.01)

    tel = actuator.telemetry
    assert tel.power_limited
    assert tel.electrical_power_w <= 200000.0 * 1.001


def test_brake_actuator_asymmetric_dump_rate():
    """Pressure dump rate (release) must be faster than build rate for ABS reactivity."""
    config = BrakeActuatorConfig(
        max_build_rate=10000.0,
        max_dump_rate=40000.0,
        enable_regen_blending=False,
    )
    actuator = BrakeByWireActuator(config)

    # Build up to 2000 Nm
    for _ in range(30):
        actuator.step([2000.0, 2000.0, 2000.0, 2000.0], vx=20.0, dt=0.01)

    t_built = actuator.friction_torque[0]
    assert t_built > 1500.0

    # Dump command to 0 Nm for 0.02s
    dt_dump = 0.02
    actuator.step([0.0, 0.0, 0.0, 0.0], vx=20.0, dt=dt_dump)
    t_dumped = actuator.friction_torque[0]

    # In 0.02s at 40000 Nm/s, torque drop can be up to 800 Nm
    drop = t_built - t_dumped
    assert drop > 600.0


def test_brake_actuator_regen_blending_and_abs_cutoff():
    """Brake-by-wire blends regen with friction; cuts regen immediately when ABS intervenes."""
    config = BrakeActuatorConfig(
        enable_regen_blending=True,
        max_regen_torque=500.0,
        regen_cutoff_speed=2.0,
    )
    actuator = BrakeByWireActuator(config)

    # Request 800 Nm braking at 20 m/s without ABS
    # Wheel should blend: ~500 Nm regen + ~300 Nm friction
    for _ in range(40):
        actuator.step([800.0, 800.0, 800.0, 800.0], vx=20.0, dt=0.01)

    tel = actuator.telemetry
    assert np.allclose(tel.regen_torque, 500.0, atol=20.0)
    assert np.allclose(tel.friction_torque, 300.0, atol=20.0)
    assert np.allclose(tel.total_delivered, 800.0, atol=20.0)

    # Now simulate ABS triggering on front-left wheel (FL = index 0)
    abs_active = [True, False, False, False]
    for _ in range(40):
        actuator.step([800.0, 800.0, 800.0, 800.0], vx=20.0, abs_active=abs_active, dt=0.01)

    tel_abs = actuator.telemetry
    # On FL: regen must be cut to 0, friction handles the full 800 Nm
    assert math.isclose(tel_abs.regen_torque[0], 0.0, abs_tol=10.0)
    assert math.isclose(tel_abs.friction_torque[0], 800.0, abs_tol=20.0)

    # Other wheels without ABS still maintain regen
    assert tel_abs.regen_torque[1] > 400.0
