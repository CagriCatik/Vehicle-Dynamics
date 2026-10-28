"""Unit and physics tests for Closed-Loop Neuromuscular Preview Driver Model."""

import math
import numpy as np
import pytest

from vehicle_model.controllers.driver_model import (
    DriverModelConfig,
    DriverTelemetry,
    NeuromuscularDriverModel,
)


def test_driver_model_config_defaults():
    cfg = DriverModelConfig()
    assert 0.20 <= cfg.t_near_preview_s <= 0.60
    assert 1.0 <= cfg.t_far_preview_s <= 2.0
    assert 0.12 <= cfg.tau_delay_s <= 0.25
    assert 0.05 <= cfg.tau_neuromuscular_s <= 0.15
    assert cfg.max_steer_rad > math.radians(25.0)


def test_driver_model_straight_line_neutral_tracking():
    # Driving straight down centerline Y=0
    driver = NeuromuscularDriverModel(dt=0.01)
    driver.set_path_function(lambda x: (0.0, 0.0))

    # Vehicle on centerline with 0 error
    for _ in range(30):
        telem = driver.step(x=10.0, y=0.0, psi=0.0, vx=20.0, vy=0.0, r=0.0)

    assert abs(telem.steer_actual_rad) < 1e-4
    assert abs(telem.near_lateral_error_m) < 1e-4
    assert abs(telem.far_heading_error_rad) < 1e-4


def test_driver_model_corrects_lateral_offset():
    driver = NeuromuscularDriverModel(dt=0.01)
    driver.set_path_function(lambda x: (0.0, 0.0))

    # Vehicle is displaced to Y = +2.0m (left of centerline)
    # The driver must steer to the right (negative delta_f) to return to Y=0
    telem = None
    for _ in range(50):
        telem = driver.step(x=10.0, y=2.0, psi=0.0, vx=20.0, vy=0.0, r=0.0)

    assert telem is not None
    assert telem.steer_actual_rad < 0.0  # Steers right (negative angle in ISO 8855)
    assert telem.near_lateral_error_m < 0.0  # Path is to the right


def test_driver_model_neural_transport_delay_latency():
    # Test that a sudden path step is NOT instantly applied to the handwheel
    # due to human transport delay tau_d = 180 ms
    cfg = DriverModelConfig(tau_delay_s=0.20, tau_neuromuscular_s=0.05)
    driver = NeuromuscularDriverModel(config=cfg, dt=0.01)
    driver.reset(initial_steer=0.0)

    # Initially on path
    driver.set_path_function(lambda x: (0.0, 0.0))
    for _ in range(10):
        driver.step(x=0.0, y=0.0, psi=0.0, vx=20.0, vy=0.0, r=0.0)

    # Sudden step: road shifts to Y = 3.5m
    driver.set_path_function(lambda x: (3.5, 0.0))

    # Step for 50 ms (5 steps): human brain has NOT finished cognitive latency (200 ms)
    for _ in range(5):
        telem = driver.step(x=0.0, y=0.0, psi=0.0, vx=20.0, vy=0.0, r=0.0)

    # Handwheel should remain near 0 because delayed command has not emerged from delay buffer
    assert abs(telem.steer_actual_rad) < 0.02

    # Step forward beyond delay horizon (total 350 ms = 35 steps)
    for _ in range(30):
        telem = driver.step(x=0.0, y=0.0, psi=0.0, vx=20.0, vy=0.0, r=0.0)

    # Now handwheel must be actively turning towards positive Y (steer left > 0)
    assert telem.steer_actual_rad > 0.05


def test_driver_model_slew_rate_limitation():
    cfg = DriverModelConfig(max_steer_rate_radps=math.radians(400.0))
    driver = NeuromuscularDriverModel(config=cfg, dt=0.005)

    # Command huge step
    driver.set_path_function(lambda x: (10.0, 0.0))
    rates = []
    for _ in range(60):
        telem = driver.step(x=0.0, y=0.0, psi=0.0, vx=20.0, vy=0.0, r=0.0)
        rates.append(abs(telem.steer_rate_radps))

    # Arm slew rate must never exceed physical human capability
    assert max(rates) <= math.radians(400.0) + 1e-4


def test_driver_workload_and_reversal_metrics():
    driver = NeuromuscularDriverModel(dt=0.01)

    # Weave left and right
    for i in range(100):
        y_tgt = 2.0 * math.sin(2.0 * math.pi * i / 50.0)
        driver.set_path_function(lambda x, y_val=y_tgt: (y_val, 0.0))
        telem = driver.step(x=0.0, y=0.0, psi=0.0, vx=15.0, vy=0.0, r=0.0)

    # Workload energy and reversals must be accumulated
    assert telem.driver_workload_energy > 0.0
    assert telem.reversal_count >= 1
