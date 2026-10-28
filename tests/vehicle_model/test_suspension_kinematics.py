"""M4: steering geometry, quasi-static K&C and roll-centre load transfer.

Geometric checks (Ackermann), physical checks (load-transfer and moment
balance, signs of roll and compliance steer), the analytical understeer budget
against the 7-DOF model, and: disabled extra effects reproduce the base model.
"""

from __future__ import annotations

import dataclasses
import math

import numpy as np
import pytest

from vehicle_model.core.types import SevenDOFControlInput, SevenDOFState
from vehicle_model.handling_analysis import understeer_budget
from vehicle_model.steering.geometry import ideal_wheel_angles, wheel_steer_angles
from vehicle_model.suspension.kinematics import (
    ChassisMasses,
    QuasiStaticChassis,
    SuspensionKC,
    ToeMap,
    load_kc,
)
from vehicle_model.vehicle_models import SevenDOFModel, TransientSevenDOFModel

VEHICLE = "configs/vehicles/reference_passenger_car.yaml"
KC = "configs/suspension/kc_reference_passenger_car.yaml"


@pytest.fixture(scope="module")
def base() -> SevenDOFModel:
    return SevenDOFModel.from_yaml(VEHICLE)


@pytest.fixture(scope="module")
def kc_model() -> SevenDOFModel:
    return SevenDOFModel.from_yaml(VEHICLE, kc_config=KC)


# ----------------------------------------------------------------------------- steering geometry
@pytest.mark.parametrize("delta_f, delta_r", [(0.3, 0.0), (-0.2, 0.0), (0.3, -0.08), (0.1, 0.04), (0.0, 0.05)])
def test_ideal_angles_are_slip_free_for_the_rigid_body_motion(base, delta_f, delta_r):
    p = base.params
    pos = base.wheel_positions
    V = 1.0
    kappa = (math.tan(delta_f) - math.tan(delta_r)) / p.wheelbase
    r = V * kappa
    vy = V * (math.tan(delta_r) + p.lr * kappa)  # lateral velocity at the CoG
    v_x = V - r * pos[:, 1]
    v_y = vy + r * pos[:, 0]
    expected = np.arctan2(v_y, v_x)
    assert np.allclose(ideal_wheel_angles(delta_f, delta_r, pos, p.lf, p.lr), expected, atol=1e-13)


def test_classic_ackermann_condition(base):
    p = base.params
    d = ideal_wheel_angles(0.35, 0.0, base.wheel_positions, p.lf, p.lr)
    inner, outer = d[0], d[1]  # left turn: FL inner
    assert inner > 0.35 > outer
    assert 1 / math.tan(outer) - 1 / math.tan(inner) == pytest.approx(p.wf / p.wheelbase, rel=1e-12)
    assert d[2] == d[3] == 0.0


def test_parallel_four_wheel_steer_has_no_singularity(base):
    p = base.params
    d = ideal_wheel_angles(0.1, 0.1, base.wheel_positions, p.lf, p.lr)
    assert np.allclose(d, 0.1, atol=1e-15)


def test_ackermann_share_blends_between_parallel_and_ideal(base):
    p, pos = base.params, base.wheel_positions
    ideal = ideal_wheel_angles(0.3, -0.05, pos, p.lf, p.lr)
    axle = np.array([0.3, 0.3, -0.05, -0.05])
    assert np.array_equal(wheel_steer_angles(0.3, -0.05, pos, p.lf, p.lr), axle)
    assert np.allclose(wheel_steer_angles(0.3, -0.05, pos, p.lf, p.lr, 1.0, 1.0), ideal, atol=1e-15)
    half = wheel_steer_angles(0.3, -0.05, pos, p.lf, p.lr, 0.5, 0.0)
    assert np.allclose(half[:2], 0.5 * (axle[:2] + ideal[:2])) and np.array_equal(half[2:], axle[2:])
    mirrored = wheel_steer_angles(-0.3, 0.05, pos, p.lf, p.lr, 0.6, 0.6)
    full = wheel_steer_angles(0.3, -0.05, pos, p.lf, p.lr, 0.6, 0.6)
    assert np.allclose(mirrored, -full[[1, 0, 3, 2]], atol=1e-15)


# ----------------------------------------------------------------------------- load transfer
def _chassis(base, **kc):
    return QuasiStaticChassis.from_yaml(base.params, VEHICLE, None) if not kc else \
        QuasiStaticChassis(base.params, QuasiStaticChassis.from_yaml(base.params, VEHICLE, None).masses, SuspensionKC(**kc))


@pytest.mark.parametrize("h_rf, h_rr", [(0.0, 0.0), (0.05, 0.10), (0.15, 0.0), (0.0, 0.2)])
def test_lateral_transfer_sums_to_m_h(base, h_rf, h_rr):
    c = _chassis(base, roll_centre_height_front=h_rf, roll_centre_height_rear=h_rr)
    p = base.params
    front, rear = c.lateral_transfer_moments()
    assert front + rear == pytest.approx(p.mass * p.hcog, rel=1e-12)


def test_roll_centres_shift_load_transfer_towards_their_axle(base):
    low = _chassis(base, roll_centre_height_front=0.0, roll_centre_height_rear=0.0).front_lateral_transfer_share
    front_high = _chassis(base, roll_centre_height_front=0.15).front_lateral_transfer_share
    rear_high = _chassis(base, roll_centre_height_rear=0.15).front_lateral_transfer_share
    assert front_high > low > rear_high


def test_without_roll_centres_and_unsprung_mass_share_equals_roll_stiffness_split(base):
    m = _chassis(base).masses
    rigid = dataclasses.replace(m, sprung_mass=base.params.mass, unsprung_mass_per_wheel=0.0)
    c = QuasiStaticChassis(base.params, rigid)
    assert c.front_lateral_transfer_share == pytest.approx(base.params.k_roll_f, rel=1e-12)


def test_mass_inconsistency_rejected(base):
    m = _chassis(base).masses
    with pytest.raises(ValueError):
        QuasiStaticChassis(base.params, dataclasses.replace(m, sprung_mass=m.sprung_mass + 10.0))


# ----------------------------------------------------------------------------- attitude and steer
def test_roll_pitch_and_wheel_travel_signs(kc_model):
    c = kc_model.chassis
    z = c.wheel_travel(0.0, 5.0)                      # left turn
    assert z[1] > 0 > z[0] and z[3] > 0 > z[2]        # right (outer) wheels compress
    assert c.roll_angle(5.0) == pytest.approx(c.masses.sprung_mass * 5.0 * c.roll_arm /
                                              (c.masses.roll_stiffness_front + c.masses.roll_stiffness_rear))
    z = c.wheel_travel(-8.0, 0.0)                     # braking: nose dives
    assert z[0] == z[1] > 0 > z[2] == z[3]


def test_roll_and_compliance_steer_promote_understeer_with_the_reference_set(kc_model):
    c = kc_model.chassis
    p = kc_model.params
    ay = 4.0
    roll_only = c.steer_change(0.0, ay, 0.0, 0.0)
    assert np.all(roll_only[:2] < 0)                  # front steers out of the (left) turn
    assert np.all(roll_only[2:] > 0)                  # rear steers into it
    eps_f, eps_r = c.roll_steer_coefficients()
    assert np.mean(roll_only[:2]) == pytest.approx(eps_f * c.roll_angle(ay), rel=0.02)  # progressive rear map
    Fy_f, Fy_r = p.mass * p.lr / p.wheelbase * ay, p.mass * p.lf / p.wheelbase * ay
    compliance = c.steer_change(0.0, 0.0, Fy_f, Fy_r)
    assert np.allclose(compliance[:2], -c.kc.compliance_steer_front * Fy_f)
    assert np.allclose(compliance[2:], c.kc.compliance_steer_rear * Fy_r)


def test_toe_map():
    lin = ToeMap.linear(0.05)
    tab = ToeMap((-0.1, 0.0, 0.1), (-0.005, 0.0, 0.005))
    z = np.linspace(-0.1, 0.1, 11)
    assert np.allclose(lin(z), tab(z)) and lin.gradient_at_zero() == pytest.approx(0.05)
    assert tab(0.5) == pytest.approx(0.005)            # constant beyond the map
    with pytest.raises(ValueError):
        ToeMap((0.1, -0.1), (0.0, 0.0))


def test_kc_yaml_requires_provenance(tmp_path):
    f = tmp_path / "kc.yaml"
    f.write_text("kinematics_compliance:\n  roll_centre_height_front: 0.1\n")
    with pytest.raises(ValueError):
        load_kc(f)
    kc = load_kc(KC)
    assert kc.bump_steer_rear.gradient_at_zero() == pytest.approx(0.05, rel=1e-6)


# ----------------------------------------------------------------------------- vehicle model
def _random_cases(n, seed=2):
    rng = np.random.default_rng(seed)
    for _ in range(n):
        vx = rng.uniform(5, 30)
        yield (SevenDOFState(vx=vx, vy=rng.uniform(-0.5, 0.5), r=rng.uniform(-0.3, 0.3),
                             omega=vx / 0.31 * (1 + rng.uniform(-0.05, 0.05, 4))),
               SevenDOFControlInput(steer_f=rng.uniform(-0.08, 0.08), steer_r=rng.uniform(-0.02, 0.02),
                                    T_drive=rng.uniform(-100, 300, 4), T_brake=rng.uniform(0, 300, 4)))


def test_disabled_kc_reproduces_the_base_model(base):
    """Rigid masses, no roll centres, no steer effects: identical derivative and telemetry."""
    m = _chassis(base).masses
    rigid = QuasiStaticChassis(base.params, dataclasses.replace(m, sprung_mass=base.params.mass, unsprung_mass_per_wheel=0.0))
    off = SevenDOFModel(base.params, base.tyre_front, base.road, base.tyre_rear, chassis=rigid)
    ref = SevenDOFModel(base.params, base.tyre_front, base.road, base.tyre_rear)
    for s, u in _random_cases(20):
        d0, t0, _, _ = ref.evaluate_dynamics(s, u)
        d1, t1, _, _ = off.evaluate_dynamics(s, u)
        assert np.allclose(d1, d0, rtol=1e-6, atol=1e-6)
        assert np.allclose(t1.steer_angles, t0.steer_angles, atol=0)


def test_kc_model_balances_and_converges(kc_model):
    p = kc_model.params
    pos = kc_model.wheel_positions
    for s, u in _random_cases(30, seed=8):
        d, tel, ax, ay = kc_model.evaluate_dynamics(s, u.clamped(p))
        assert tel.load_transfer_converged
        Mz = np.sum(pos[:, 0] * tel.Fy_body - pos[:, 1] * tel.Fx_body)
        assert p.Iz * d[2] == pytest.approx(Mz, rel=1e-12, abs=1e-9)
        assert tel.Fz.sum() == pytest.approx(p.mass * p.g, rel=1e-12)
        # the steer angles used are geometry + K&C at the converged point
        c = kc_model.chassis
        geo = wheel_steer_angles(u.clamped(p).steer_f, u.clamped(p).steer_r, pos, p.lf, p.lr,
                                 c.kc.ackermann_front, c.kc.ackermann_rear)
        expected = geo + c.steer_change(ax, ay, tel.Fy_wheel[:2].sum(), tel.Fy_wheel[2:].sum())
        assert np.allclose(tel.steer_angles, expected, atol=1e-6)


def test_budget_components(kc_model, base):
    b0, b = understeer_budget(base), understeer_budget(kc_model)
    assert b0.roll_steer == b0.compliance_steer == 0.0 and b.tyres == pytest.approx(b0.tyres, rel=1e-12)
    eps_f, eps_r = kc_model.chassis.roll_steer_coefficients()
    assert b.roll_steer == pytest.approx((eps_r - eps_f) * kc_model.chassis.roll_gradient, rel=1e-12)
    assert b.roll_steer > 0 and b.compliance_steer > 0


def _steady_yaw_rate(model, vx, delta, T=4.0, dt=0.005):
    p = model.params
    s = SevenDOFState(vx=vx, omega=np.full(4, vx / p.Reff))
    for _ in range(int(round(T / dt))):
        f = 0.5 * p.rho * p.Cd * p.Af * s.vx**2 + p.c_rr * p.mass * p.g + 2.0 * p.mass * (vx - s.vx)
        s, _ = model.step(s, SevenDOFControlInput(steer_f=delta, T_drive=np.full(4, f * p.Reff / 4)), dt)
    return s.r


@pytest.mark.parametrize("vx", [pytest.param(10.0, marks=pytest.mark.slow), 20.0, pytest.param(30.0, marks=pytest.mark.slow)])
def test_steady_yaw_rate_matches_linear_understeer_budget(kc_model, vx):
    """a_y = 0.3 m/s^2: 7-DOF with K&C vs v delta/(L + EG v^2), EG = tyres + roll steer + compliance."""
    L = kc_model.params.wheelbase
    EG = understeer_budget(kc_model).total
    delta = 0.3 * (L + EG * vx**2) / vx**2
    assert _steady_yaw_rate(kc_model, vx, delta) == pytest.approx(vx * delta / (L + EG * vx**2), rel=0.01)


def test_transient_variant_accepts_kc():
    tr = TransientSevenDOFModel.from_yaml(VEHICLE, kc_config=KC)
    s = SevenDOFState(vx=20.0, vy=0.1, r=0.1, omega=np.full(4, 20.0 / tr.params.Reff))
    u = SevenDOFControlInput(steer_f=0.03)
    d, tel, _, _ = tr.evaluate_dynamics(tr.steady_slip_state(s, u), u)
    assert np.max(np.abs(d[10:])) < 1e-6
    assert tel.steer_angles[0] != tel.steer_angles[1]  # Ackermann + K&C act per wheel
