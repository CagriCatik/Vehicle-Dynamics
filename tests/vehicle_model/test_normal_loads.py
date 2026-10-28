"""Quasi-static wheel loads with unilateral contact (Plan.md M2; Audit 4.3).

Mathematical verification of vehicle_models/normal_loads.py: equilibrium,
non-negativity, analytical lift-off and tip-over thresholds, continuity and
mirror symmetry.
"""

from __future__ import annotations

import dataclasses

import numpy as np
import pytest

from vehicle_model.core.types import FL, FR, RL, RR, SevenDOFParams
from vehicle_model.vehicle_models.normal_loads import (
    SUPPORT_FOUR_WHEEL,
    SUPPORT_THREE_WHEEL,
    SUPPORT_TIP_OVER,
    quasi_static_normal_loads,
    wheel_contact_positions,
)

REFERENCE = "configs/vehicles/reference_passenger_car.yaml"


@pytest.fixture(scope="module")
def ref() -> SevenDOFParams:
    return SevenDOFParams.from_yaml(REFERENCE)


@pytest.fixture(scope="module")
def tall() -> SevenDOFParams:
    """High CoG and front-biased roll stiffness: the inner front wheel lifts before tip-over."""
    return dataclasses.replace(SevenDOFParams.from_yaml(REFERENCE), hcog=0.9, k_roll_f=0.9)


def _balance_residual(p: SevenDOFParams, Fz: np.ndarray, ax: float, ay: float) -> np.ndarray:
    pos = wheel_contact_positions(p)
    mg = p.mass * p.g
    return np.array([
        Fz.sum() - mg,
        pos[:, 0] @ Fz + p.mass * ax * p.hcog,
        pos[:, 1] @ Fz + p.mass * ay * p.hcog,
    ])


def _lift_threshold_ay(p: SevenDOFParams) -> float:
    """a_y at which the inner front wheel unloads (a_x = 0), classic four-wheel formula."""
    return 0.5 * p.g * p.lr / p.wheelbase * p.wf / (p.hcog * p.k_roll_f)


def _tip_threshold_ay(p: SevenDOFParams) -> float:
    """a_y at which the centre of pressure reaches the outer side edge at x = 0 (a_x = 0)."""
    y_edge = (p.wf * p.lr + p.wr * p.lf) / (2.0 * p.wheelbase)
    return p.g * y_edge / p.hcog


@pytest.mark.parametrize("which", ["ref", "tall"])
def test_equilibrium_and_non_negativity_on_grid(which, request):
    p = request.getfixturevalue(which)
    mg = p.mass * p.g
    seen = set()
    for ax in np.linspace(-12.0, 12.0, 25):
        for ay in np.linspace(-14.0, 14.0, 29):
            loads = quasi_static_normal_loads(p, ax, ay)
            seen.add(loads.support)
            assert np.all(loads.Fz >= 0.0)
            assert loads.Fz.sum() == pytest.approx(mg, rel=1e-12)
            res = _balance_residual(p, loads.Fz, ax, ay)
            if loads.support == SUPPORT_TIP_OVER:
                assert np.allclose(loads.moment_residual, res[1:], atol=1e-6 * mg)
                assert np.linalg.norm(loads.moment_residual) > 0.0
                assert loads.in_contact.sum() <= 2
            else:
                assert np.allclose(res, 0.0, atol=1e-9 * mg)
                assert np.all(loads.moment_residual == 0.0)
    assert seen == {SUPPORT_FOUR_WHEEL, SUPPORT_THREE_WHEEL, SUPPORT_TIP_OVER}


def test_four_wheel_region_keeps_roll_stiffness_split(ref):
    ay = 0.5 * _lift_threshold_ay(ref)
    loads = quasi_static_normal_loads(ref, 1.0, ay)
    assert loads.support == SUPPORT_FOUR_WHEEL and loads.in_contact.all()
    roll_f = 0.5 * ref.wf * (loads.Fz[FR] - loads.Fz[FL])
    roll_r = 0.5 * ref.wr * (loads.Fz[RR] - loads.Fz[RL])
    assert roll_f / (roll_f + roll_r) == pytest.approx(ref.k_roll_f, rel=1e-12)


def test_static_loads(ref):
    loads = quasi_static_normal_loads(ref, 0.0, 0.0)
    mg = ref.mass * ref.g
    assert loads.Fz[FL] == pytest.approx(0.5 * mg * ref.lr / ref.wheelbase, rel=1e-12)
    assert loads.Fz[RR] == pytest.approx(0.5 * mg * ref.lf / ref.wheelbase, rel=1e-12)


def test_inner_front_wheel_lifts_at_analytical_threshold(tall):
    ay_lift = _lift_threshold_ay(tall)
    assert ay_lift < _tip_threshold_ay(tall)
    below = quasi_static_normal_loads(tall, 0.0, ay_lift * (1 - 1e-6))
    above = quasi_static_normal_loads(tall, 0.0, ay_lift * (1 + 1e-3))
    assert below.support == SUPPORT_FOUR_WHEEL
    assert above.support == SUPPORT_THREE_WHEEL
    assert list(above.in_contact) == [False, True, True, True]  # left turn: FL is the inner front wheel
    assert above.Fz[FL] == 0.0


def test_lift_off_is_continuous(tall):
    ay_lift = _lift_threshold_ay(tall)
    eps = 1e-6
    a = quasi_static_normal_loads(tall, 0.0, ay_lift - eps).Fz
    b = quasi_static_normal_loads(tall, 0.0, ay_lift + eps).Fz
    # Lipschitz bound: load transfer gradient is at most m h / min(track) per wheel
    lipschitz = tall.mass * tall.hcog / min(tall.wf, tall.wr)
    assert np.max(np.abs(a - b)) <= 2 * eps * lipschitz


def test_after_lift_off_additional_roll_moment_goes_to_rear(tall):
    """With the inner front wheel lifted the front axle cannot take more roll moment."""
    ay1 = _lift_threshold_ay(tall) * 1.02
    ay2 = ay1 + 0.2
    l1 = quasi_static_normal_loads(tall, 0.0, ay1)
    l2 = quasi_static_normal_loads(tall, 0.0, ay2)
    assert l1.support == l2.support == SUPPORT_THREE_WHEEL
    d_roll_f = 0.5 * tall.wf * ((l2.Fz[FR] - l2.Fz[FL]) - (l1.Fz[FR] - l1.Fz[FL]))
    d_roll_r = 0.5 * tall.wr * ((l2.Fz[RR] - l2.Fz[RL]) - (l1.Fz[RR] - l1.Fz[RL]))
    total = tall.mass * (ay2 - ay1) * tall.hcog
    assert d_roll_f + d_roll_r == pytest.approx(total, rel=1e-9)
    assert d_roll_r / total > 1.0 - tall.k_roll_f  # rear share above its stiffness share


def test_tip_over_beyond_support_polygon(tall):
    ay_tip = _tip_threshold_ay(tall)
    inside = quasi_static_normal_loads(tall, 0.0, ay_tip * (1 - 1e-6))
    outside = quasi_static_normal_loads(tall, 0.0, ay_tip * 1.05)
    assert inside.support != SUPPORT_TIP_OVER
    assert outside.support == SUPPORT_TIP_OVER
    assert list(outside.in_contact) == [False, True, False, True]  # right-hand side only
    # The unbalanced roll moment is that of the CoP offset beyond the edge: m g dy cos(edge angle)
    dy = (ay_tip * 1.05 - ay_tip) * tall.hcog / tall.g
    slope = (tall.wf - tall.wr) / (2.0 * tall.wheelbase)
    expected = tall.mass * tall.g * dy / (1.0 + slope**2)
    assert abs(outside.moment_residual[1]) == pytest.approx(expected, rel=1e-6)


def test_tip_over_residual_vanishes_at_the_boundary(tall):
    ay_tip = _tip_threshold_ay(tall)
    near = quasi_static_normal_loads(tall, 0.0, ay_tip * (1 + 1e-9))
    assert np.linalg.norm(near.moment_residual) < 1e-3


@pytest.mark.parametrize("ax", [-9.0, 0.0, 4.0])
@pytest.mark.parametrize("ay", [2.0, 6.0, 11.0])
def test_mirror_symmetry(tall, ax, ay):
    left = quasi_static_normal_loads(tall, ax, ay)
    right = quasi_static_normal_loads(tall, ax, -ay)
    mirror = [FR, FL, RR, RL]
    assert left.support == right.support
    assert np.allclose(left.Fz, right.Fz[mirror], rtol=0, atol=1e-9)


def test_braking_lifts_rear_wheels_before_front(ref):
    """Strong braking moves the CoP towards the front axle; beyond it the vehicle pitches over."""
    ax_tip = -ref.g * ref.lf / ref.hcog
    near = quasi_static_normal_loads(ref, ax_tip * (1 - 1e-6), 0.0)
    over = quasi_static_normal_loads(ref, ax_tip * 1.1, 0.0)
    assert near.support == SUPPORT_FOUR_WHEEL
    assert near.Fz[RL] == pytest.approx(0.0, abs=1e-5 * ref.mass * ref.g)
    assert over.support == SUPPORT_TIP_OVER
    assert list(over.in_contact) == [True, True, False, False]
