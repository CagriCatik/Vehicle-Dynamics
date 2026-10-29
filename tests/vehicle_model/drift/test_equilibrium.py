"""Drift plan D2: stationary drift equilibria (residuals, torque and force balance, symmetry, honest infeasibility)."""

from __future__ import annotations

import dataclasses
import math

import numpy as np
import pytest

from vehicle_model.drift import (INFEASIBLE, VALID, VALID_EXTRAPOLATED, DriftEquilibriumSolver, DriftReference,
                                 rwd_input)
from vehicle_model.drift.types import ACTUATOR_LIMIT, TRIVIAL
from vehicle_model.vehicle_models import SevenDOFModel

REF = DriftReference(13.0, math.radians(-20.0), 0.9)


@pytest.fixture(scope="module")
def solver():
    return DriftEquilibriumSolver(SevenDOFModel.from_yaml())


@pytest.fixture(scope="module")
def eq(solver):
    return solver.solve(REF)


def test_a_nontrivial_left_drift_exists_with_documented_extrapolation(eq):
    assert eq.physical_feasible and eq.validity in (VALID, VALID_EXTRAPOLATED)
    assert eq.yaw_rate > 0.05 and eq.beta == pytest.approx(REF.beta_rad, abs=1e-12)    # r opposite to beta
    assert eq.speed == pytest.approx(13.0, abs=1e-12)
    assert eq.steer_front < 0.0                      # result for this point (counter-steer), not a hard-coded rule
    assert eq.residual_norm <= 1.0
    if eq.validity == VALID_EXTRAPOLATED:
        assert any(eq.diagnostics["extrapolating"])


def test_wheel_torque_and_body_force_balance_from_plant_telemetry(solver, eq):
    """Independent of the solver residual: I_w omega_dot = T - F_x R - d omega = 0 and the body balances."""
    P = solver.plant(REF.road_mu)
    p = P.params
    ev = P.evaluate(P.full_state(eq.body_state()), rwd_input(eq.steer_front, eq.wheel_drive_torque[2]))
    t = ev.telemetry
    torque = eq.wheel_drive_torque - t.Fx_wheel * p.Reff - p.wheel_damping * eq.omega
    assert np.max(np.abs(torque)) < 1e-4 * p.max_drive_torque
    drag = 0.5 * p.rho * p.Cd * p.Af * eq.vx * abs(eq.vx) + p.c_rr * p.mass * p.g * math.tanh(eq.vx / 0.5)
    assert np.sum(t.Fx_body) - drag == pytest.approx(-p.mass * eq.yaw_rate * eq.vy, abs=1e-3)
    assert np.sum(t.Fy_body) == pytest.approx(p.mass * eq.yaw_rate * eq.vx, abs=1e-3)
    assert eq.wheel_drive_torque[0] == eq.wheel_drive_torque[1] == 0.0 and np.all(eq.wheel_brake_torque == 0.0)


def test_right_drift_is_the_mirror_image(solver, eq):
    m = solver.solve(REF.mirrored())
    assert m.physical_feasible
    assert m.yaw_rate == pytest.approx(-eq.yaw_rate, rel=1e-8) and m.steer_front == pytest.approx(-eq.steer_front, rel=1e-8)
    assert m.wheel_drive_torque[2] == pytest.approx(eq.wheel_drive_torque[2], rel=1e-8)
    assert m.omega[2] == pytest.approx(eq.omega[3], rel=1e-8)     # inner and outer rear wheel swap


def test_results_are_read_only(eq):
    with pytest.raises(ValueError):
        eq.omega[0] = 0.0


def test_an_unreachable_request_is_reported_not_faked():
    m = SevenDOFModel.from_yaml()
    weak = SevenDOFModel(params=dataclasses.replace(m.params, max_drive_torque=20.0), tyre=m.tyre_front,
                         tyre_rear=m.tyre_rear, road=m.road)
    e = DriftEquilibriumSolver(weak).solve(REF)
    assert e.validity == INFEASIBLE and not e.physical_feasible
    assert ACTUATOR_LIMIT in e.reasons or "residual_too_large" in e.reasons


def test_a_tiny_sideslip_is_not_accepted_as_drift(solver):
    e = solver.evaluate(DriftReference(13.0, math.radians(-1.0), 0.9), np.r_[0.3, np.full(4, 40.0), -0.05, 300.0])
    assert TRIVIAL in e.reasons and e.validity == INFEASIBLE


def test_multistart_lists_branches_feasible_first(solver):
    branches = solver.solve(REF, multistart="all")
    assert branches[0].physical_feasible
    assert all(b.physical_feasible or b.reasons for b in branches)


def test_continuation_finds_the_same_point_as_a_cold_start(solver):
    refs = [DriftReference(13.0, math.radians(-b), 0.9) for b in (15.0, 20.0)]
    cont = solver.sweep(refs)[1]
    cold = solver.solve(refs[1])
    assert cont.physical_feasible and cont.yaw_rate == pytest.approx(cold.yaw_rate, rel=1e-6)


def test_sweep_config_is_strict():
    from vehicle_model.simulation.drift.equilibrium_sweep import parse
    base = {"name": "x", "vehicle": "configs/vehicles/seven_dof_passenger.yaml",
            "sweep": {"speed_mps": [13], "beta_deg": [20], "mu": [0.9]}}
    assert parse(base).directions == ("left", "right")
    for bad, msg in (({"sweep": {"speed_mps": [13], "beta_deg": [20], "mu": [0.9], "radius": [20]}}, "unknown key"),
                     ({"powertrain": {"layout": "awd"}}, "only 'rwd'"),
                     ({"sweep": {"speed_mps": [13], "beta_deg": [95], "mu": [0.9]}}, "outside"),
                     ({"tyres": {"rear": "slick_9000"}}, "unknown tyre"),
                     ({"directions": ["up"]}, "directions")):
        with pytest.raises(ValueError, match=msg):
            parse({**base, **bad})


def test_fast_cornering_is_not_accepted_as_a_drift(solver):
    """beta < 0 at r > 0 alone also holds for fast ordinary cornering: the rear axle must be past its lateral peak."""
    from vehicle_model.drift.types import REAR_NOT_SATURATED
    e = solver.solve(DriftReference(20.0, math.radians(-5.0), 0.9))
    assert REAR_NOT_SATURATED in e.reasons and e.validity == INFEASIBLE
    alpha, peak = e.diagnostics["alpha_deg"][2:], e.diagnostics["rear_peak_slip_angle_deg"]
    assert min(abs(a) - p for a, p in zip(alpha, peak)) < 0.0
