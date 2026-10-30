"""Bounded control allocation (exact box-constrained WLS) and wheel-force allocation."""

from __future__ import annotations

import numpy as np
import pytest
from scipy.optimize import lsq_linear

from vehicle_model.allocation import (
    STATUS_INFEASIBLE,
    STATUS_NOMINAL,
    STATUS_SATURATED,
    ActuatorHealth,
    AllocatorConfig,
    WheelForceAllocator,
    solve_box_wls,
)
from vehicle_model.core.types import SevenDOFParams
from vehicle_model.vehicle_models.normal_loads import quasi_static_normal_loads, wheel_contact_positions

VEHICLE = "configs/vehicles/reference_passenger_car.yaml"


# ----------------------------------------------------------------------------- generic solver
def test_matches_scipy_bvls_on_random_problems():
    rng = np.random.default_rng(0)
    eps = 1e-6
    for _ in range(200):
        n = rng.integers(2, 6)
        B = rng.normal(size=(2, n))
        v = rng.normal(size=2) * 3
        lo, hi = -rng.uniform(0.1, 2, n), rng.uniform(0.1, 2, n)
        W_v, W_u = rng.uniform(0.5, 2, 2), rng.uniform(0.5, 2, n)
        r = solve_box_wls(B, v, lo, hi, W_v=W_v, W_u=W_u, eps=eps)
        A = np.vstack([W_v[:, None] * B, np.sqrt(eps) * np.diag(W_u)])
        b = np.concatenate([W_v * v, np.zeros(n)])
        ref = lsq_linear(A, b, bounds=(lo, hi), tol=1e-14, method="bvls")
        assert np.allclose(r.u, ref.x, atol=1e-7)
        assert np.all(r.u >= lo) and np.all(r.u <= hi)


def test_unconstrained_solution_is_the_weighted_pseudo_inverse():
    B = np.array([[1.0, 1.0, 1.0, 1.0], [-1.0, 1.0, -1.0, 1.0]])
    v = np.array([2.0, 0.4])
    eps = 1e-9
    r = solve_box_wls(B, v, np.full(4, -10.0), np.full(4, 10.0), eps=eps)
    assert np.allclose(r.u, np.linalg.pinv(B) @ v, atol=1e-6)
    assert np.allclose(r.residual, 0.0, atol=1e-8) and not r.at_lower.any() and not r.at_upper.any()


def test_priority_weights_decide_which_demand_is_met():
    B = np.array([[1.0, 1.0], [1.0, -1.0]])
    lo, hi = np.zeros(2), np.ones(2)
    v = np.array([2.0, 1.0])  # exact solution u = (1.5, 0.5) violates u1 <= 1: infeasible
    first = solve_box_wls(B, v, lo, hi, W_v=[1.0, 100.0])
    second = solve_box_wls(B, v, lo, hi, W_v=[100.0, 1.0])
    assert abs(first.residual[1]) < abs(second.residual[1])
    assert abs(second.residual[0]) < abs(first.residual[0])


def test_invalid_inputs_rejected():
    with pytest.raises(ValueError):
        solve_box_wls(np.eye(2), [0, 0], [1, 0], [0, 1])          # lo > hi
    with pytest.raises(ValueError):
        solve_box_wls(np.ones((2, 7)), [0, 0], -np.ones(7), np.ones(7))  # too many actuators
    with pytest.raises(ValueError):
        solve_box_wls(np.eye(2), [0, 0], -np.ones(2), np.ones(2), eps=0.0)


# ----------------------------------------------------------------------------- wheel forces
@pytest.fixture(scope="module")
def setup():
    p = SevenDOFParams.from_yaml(VEHICLE)
    alloc = WheelForceAllocator(wheel_contact_positions(p), p.Reff, AllocatorConfig())
    return p, alloc, quasi_static_normal_loads(p, 0.0, 0.0).Fz


def test_effectiveness_matrix(setup):
    p, alloc, _ = setup
    B = alloc.effectiveness(np.zeros(4))
    assert np.allclose(B[0], 1.0)
    assert np.allclose(B[1], [-p.wf / 2, p.wf / 2, -p.wr / 2, p.wr / 2])  # right wheels drive -> +Mz


def test_nominal_request_is_met_and_reported(setup):
    p, alloc, Fz = setup
    r = alloc.allocate(2000.0, 500.0, np.zeros(4), Fz, np.ones(4))
    assert r.status == STATUS_NOMINAL
    assert np.allclose(r.achieved, [2000.0, 500.0], atol=1e-3)
    assert np.allclose(r.T_drive - r.T_brake, r.F * p.Reff)


def test_pure_braking_is_distributed_with_the_available_grip(setup):
    p, alloc, _ = setup
    Fz = quasi_static_normal_loads(p, -6.0, 0.0).Fz  # loads while braking
    r = alloc.allocate(-8000.0, 0.0, np.zeros(4), Fz, np.ones(4))
    assert np.allclose(r.F / Fz, r.F[0] / Fz[0], rtol=1e-5)  # equal utilisation on all wheels


def test_grip_bound_uses_the_remaining_friction_circle(setup):
    p, alloc, Fz = setup
    Fy = np.array([0.9, 0.9, 0.5, 0.5]) * Fz
    lower, upper, avail, _, _ = alloc.bounds(Fz, np.ones(4), Fy)
    assert np.allclose(avail, 0.95 * np.sqrt(Fz**2 - Fy**2))
    assert np.all(upper <= avail + 1e-9) and np.all(-lower <= avail + 1e-9)


def test_infeasible_demand_is_detected_and_yaw_has_priority(setup):
    p, alloc, Fz = setup
    r = alloc.allocate(-40000.0, 1500.0, np.zeros(4), Fz, np.full(4, 0.3))
    assert r.status == STATUS_INFEASIBLE
    assert abs(r.residual[1]) < 10.0           # yaw moment still delivered
    assert r.residual[0] > 20000.0             # longitudinal demand reported as not met
    assert r.grip_limited.any()


def test_saturation_without_loss_of_demand(setup):
    p, alloc, Fz = setup
    cfg = AllocatorConfig(driven=(False, False, True, True))
    rear = WheelForceAllocator(alloc.positions, p.Reff, cfg)
    r = rear.allocate(1000.0, 0.0, np.zeros(4), Fz, np.ones(4))
    assert r.status == STATUS_SATURATED          # front wheels cannot drive: upper bound 0 active
    assert np.allclose(r.F[:2], 0.0, atol=1e-6) and np.allclose(r.achieved, [1000.0, 0.0], atol=1e-3)


def test_failed_brake_is_compensated_by_the_others(setup):
    p, alloc, Fz = setup
    failed = ActuatorHealth(brake_ok=(False, True, True, True))
    r = alloc.allocate(-4000.0, 0.0, np.zeros(4), Fz, np.ones(4), health=failed)
    assert r.F[0] == pytest.approx(0.0, abs=1e-6)
    assert r.status == STATUS_SATURATED
    assert np.allclose(r.achieved, [-4000.0, 0.0], atol=1.0)
    # Without yaw moment the left side has only RL: at most twice its grip can be braked
    r = alloc.allocate(-6000.0, 0.0, np.zeros(4), Fz, np.ones(4), health=failed)
    assert r.status == STATUS_INFEASIBLE and abs(r.achieved[1]) < 10.0
    assert -r.achieved[0] == pytest.approx(2 * r.diagnostics["available_grip"][2], rel=0.01)


def test_mirror_symmetry(setup):
    p, alloc, Fz = setup
    a = alloc.allocate(-3000.0, 800.0, np.array([0.05, 0.05, 0, 0]), Fz, np.array([1.0, 0.4, 1.0, 0.4]))
    b = alloc.allocate(-3000.0, -800.0, np.array([-0.05, -0.05, 0, 0]), Fz, np.array([0.4, 1.0, 0.4, 1.0]))
    assert np.allclose(a.F, b.F[[1, 0, 3, 2]], atol=1e-6)


def test_non_finite_request_rejected(setup):
    p, alloc, Fz = setup
    with pytest.raises(ValueError):
        alloc.allocate(np.nan, 0.0, np.zeros(4), Fz, np.ones(4))
