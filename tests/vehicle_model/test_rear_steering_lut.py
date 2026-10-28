"""Unit tests for rear-axle steering lookup table (LUT) model and 4WS integration."""

import math
from pathlib import Path
import numpy as np
import pytest

from vehicle_model.steering.lut import LookupTable, load_lookup_table
from vehicle_model.steering.four_wheel_steering import (
    FourWheelSteering,
    SteeringConfig,
    SteeringMode,
)


def test_load_lookup_table_default():
    """Verify loading from centralized configs/steering/lut.json."""
    lut = load_lookup_table()
    assert isinstance(lut, LookupTable)
    assert len(lut.velocity_knots) == 13
    assert len(lut.front_knots) == 13
    assert lut.values_deg.shape == (13, 13)


def test_lookup_table_interpolation_knots():
    """Verify interpolation exactly matches known knot values."""
    lut = load_lookup_table()

    # At v=0 m/s, front=15 deg -> rear should be -5.25 deg
    r_deg = lut.desired_rear_deg(0.0, 15.0)
    assert math.isclose(r_deg, -5.25, rel_tol=1e-3)

    # At v=20 m/s, front=15 deg -> rear should be +1.2 deg (in-phase / stability)
    r_deg_high = lut.desired_rear_deg(20.0, 15.0)
    assert math.isclose(r_deg_high, 1.2, rel_tol=1e-3)

    # In radians:
    r_rad = lut.desired_rear_rad(20.0, 15.0)
    assert math.isclose(r_rad, math.radians(1.2), rel_tol=1e-3)


def test_lookup_table_bilinear_interpolation_midpoint():
    """Verify smooth bilinear interpolation between breakpoints."""
    lut = load_lookup_table()

    # Midpoint between v=0 and v=2 at front=0 -> 0.0
    r_mid_0 = lut.desired_rear_deg(1.0, 0.0)
    assert math.isclose(r_mid_0, 0.0, abs_tol=1e-4)

    # Exact knot checks between v=2.0 and v=5.0:
    # At v=2.0, front=15 -> -5.044; at v=5.0, front=15 -> -4.688
    # Midpoint v=3.5 should smoothly interpolate
    r_mid = lut.desired_rear_deg(3.5, 15.0)
    assert -5.10 < r_mid < -4.60


def test_lookup_table_flat_extrapolation_out_of_bounds():
    """Verify flat extrapolation prevents crashes when speed or angle exceeds bounds."""
    lut = load_lookup_table()

    # v = 65.0 m/s (above 50 m/s max) -> clamps to 50 m/s knot
    r_at_max_v = lut.desired_rear_deg(50.0, 15.0)
    r_extrap_v = lut.desired_rear_deg(65.0, 15.0)
    assert math.isclose(r_extrap_v, r_at_max_v, rel_tol=1e-3)

    # front = 45.0 deg (above 30 deg max) at v=20 m/s -> clamps to 30 deg knot
    r_at_max_f = lut.desired_rear_deg(20.0, 30.0)
    r_extrap_f = lut.desired_rear_deg(20.0, 45.0)
    assert math.isclose(r_extrap_f, r_at_max_f, rel_tol=1e-3)


def test_four_wheel_steering_lookup_table_mode():
    """Verify FourWheelSteering integration with SteeringMode.LOOKUP_TABLE."""
    fws = FourWheelSteering()
    cmd_f = math.radians(15.0)

    # Low speed (0 m/s): rear should turn counter-phase (opposite sign / negative rear angle in table for positive front)
    delta_f, delta_r = fws.compute_steering_angles(
        cmd_steer_f=cmd_f,
        vx=0.0,
        mode=SteeringMode.LOOKUP_TABLE,
    )
    assert math.isclose(delta_f, cmd_f, rel_tol=1e-3)
    assert math.isclose(delta_r, math.radians(-5.25), rel_tol=1e-3)

    # High speed (20 m/s): rear should turn in-phase (same sign / +1.2 deg in table for positive front)
    _, delta_r_high = fws.compute_steering_angles(
        cmd_steer_f=cmd_f,
        vx=20.0,
        mode=SteeringMode.LOOKUP_TABLE,
    )
    assert math.isclose(delta_r_high, math.radians(1.2), rel_tol=1e-3)

    # Actuator limit clamping: rear angle cannot exceed max_steer_r
    fws.config.max_steer_r = math.radians(2.0)
    _, delta_r_clamped = fws.compute_steering_angles(
        cmd_steer_f=math.radians(30.0),
        vx=0.0,
        mode=SteeringMode.LOOKUP_TABLE,
    )
    assert abs(delta_r_clamped) <= math.radians(2.0) + 1e-6
