"""Verification of the kinematic single track with front/rear steering.

Criteria: straight driving at zero steer, analytical radius, consistent signs (incl. reverse),
limit case delta_f = delta_r, numerical convergence.
"""

import math

import numpy as np
import pytest

from vehicle_model.vehicle_models import KinematicSingleTrack

V = 10.0


def _run(model, steer_f, delta_r=None, v=V, T=2.0, dt=0.01):
    t = np.arange(0.0, T + 1e-12, dt)
    u = [0.0, 0.0] if delta_r is None else [0.0, 0.0, delta_r]
    x, _ = model.sim_continuous(np.array([0.0, 0.0, 0.0, v, steer_f]), np.array(u), t)
    return t, x


def _circle(model, steer_f, delta_r, v, t):
    """Analytical CoG path for constant steer: arc with radius R about the instantaneous centre."""
    L = model.lf + model.lr
    tf, tr = math.tan(steer_f), math.tan(delta_r)
    beta = math.atan((model.lf * tr + model.lr * tf) / L)
    w = v * math.cos(beta) * (tf - tr) / L
    R = v / w
    phi = beta + w * t
    return np.column_stack([R * (np.sin(phi) - math.sin(beta)), R * (math.cos(beta) - np.cos(phi)), w * t])


def test_zero_steer_drives_straight():
    _, x = _run(KinematicSingleTrack(), 0.0)
    np.testing.assert_allclose(x[-1, :3], [V * 2.0, 0.0, 0.0], atol=1e-12)


@pytest.mark.parametrize("rear", [False, True])
def test_rear_steer_disabled_or_zero_equals_front_steer_model(rear):
    model = KinematicSingleTrack(rear_steer=rear)
    _, x = _run(model, 0.1, 0.0 if rear else None)
    _, x_ref = _run(KinematicSingleTrack(), 0.1)
    np.testing.assert_allclose(x, x_ref, atol=1e-12)


@pytest.mark.parametrize("steer_f, delta_r", [(0.1, 0.0), (0.2, -0.1), (0.15, 0.05), (-0.2, 0.0)])
def test_constant_steer_follows_analytical_circle(steer_f, delta_r):
    model = KinematicSingleTrack(rear_steer=True)
    t, x = _run(model, steer_f, delta_r, T=3.0, dt=0.001)
    np.testing.assert_allclose(x[:, :3], _circle(model, steer_f, delta_r, V, t), atol=1e-9)
    R = model.turning_radius(steer_f, delta_r)
    yaw_rate = (x[-1, 2] - x[0, 2]) / t[-1]
    assert abs(yaw_rate) == pytest.approx(V / R, rel=1e-9)


def test_signs_forward_and_reverse():
    model = KinematicSingleTrack()
    _, fwd = _run(model, 0.1, v=5.0)
    _, rev = _run(model, 0.1, v=-5.0)
    assert fwd[-1, 2] > 0.0 and fwd[-1, 1] > 0.0      # left steer, forward: turns left (counter-clockwise)
    assert rev[-1, 2] < 0.0 and rev[-1, 0] < 0.0      # reverse: yaw sign flips, drives backwards
    np.testing.assert_allclose(rev[-1, 2], -fwd[-1, 2], rtol=1e-12)


@pytest.mark.parametrize("delta", [0.05, -0.12])
def test_equal_front_and_rear_steer_gives_pure_translation(delta):
    model = KinematicSingleTrack(rear_steer=True)
    _, x = _run(model, delta, delta)
    assert x[-1, 2] == pytest.approx(0.0, abs=1e-14)
    assert model.turning_radius(delta, delta) == math.inf
    assert x[-1, 1] == pytest.approx(x[-1, 0] * math.tan(delta), rel=1e-12)  # moves along delta


def test_counter_phase_rear_steer_reduces_turning_radius():
    model = KinematicSingleTrack(rear_steer=True)
    r_2ws = model.turning_radius(0.3, 0.0)
    r_4ws = model.turning_radius(0.3, -0.1)
    assert r_4ws < r_2ws
    # symmetric counter-phase steer on a symmetric car: no sideslip, R = L / (2 tan(delta))
    sym = KinematicSingleTrack({**_params_dict(model), "lf": 1.4, "lr": 1.4}, rear_steer=True)
    assert sym.turning_radius(0.2, -0.2) == pytest.approx(2.8 / (2 * math.tan(0.2)), rel=1e-12)


def _params_dict(model):
    import dataclasses
    return dataclasses.asdict(model.params)


def test_rear_steer_input_is_clipped_to_steer_limits():
    model = KinematicSingleTrack(rear_steer=True)
    _, x_big = _run(model, 0.0, 10.0)
    _, x_lim = _run(model, 0.0, model.max_steer)
    np.testing.assert_allclose(x_big, x_lim, atol=1e-12)


def test_rk4_converges_with_fourth_order():
    model = KinematicSingleTrack(rear_steer=True)
    errors = []
    for dt in (0.2, 0.1, 0.05):
        t, x = _run(model, 0.25, -0.1, T=4.0, dt=dt)
        errors.append(np.max(np.abs(x[:, :3] - _circle(model, 0.25, -0.1, V, t))))
    orders = [math.log2(a / b) for a, b in zip(errors, errors[1:])]
    assert all(3.7 < q < 4.3 for q in orders), orders
