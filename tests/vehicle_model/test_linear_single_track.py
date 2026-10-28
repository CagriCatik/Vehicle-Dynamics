"""Mathematical verification of the linear 2-DOF single-track reference (Plan.md, Abschnitt 6 and 15.2)."""

import math

import numpy as np
import pytest

from vehicle_model.handling_analysis import (
    LinearSingleTrack,
    LinearSingleTrackParams,
    step_response_metrics,
)

US = LinearSingleTrackParams(mass=1500.0, Iz=2500.0, lf=1.2, lr=1.6, C_f=100_000.0, C_r=120_000.0)
SPEEDS = [5.0, 15.0, 30.0, 45.0]


def _with(**kw) -> LinearSingleTrack:
    d = dict(mass=US.mass, Iz=US.Iz, lf=US.lf, lr=US.lr, C_f=US.C_f, C_r=US.C_r)
    d.update(kw)
    return LinearSingleTrack(LinearSingleTrackParams(**d))


def test_parameters_must_be_positive():
    with pytest.raises(ValueError):
        LinearSingleTrackParams(mass=1500.0, Iz=2500.0, lf=1.2, lr=1.6, C_f=-1.0, C_r=1.0)


def test_rejects_non_positive_speed():
    with pytest.raises(ValueError):
        LinearSingleTrack(US).state_space(0.0)


def test_understeer_gradient_closed_form():
    m = LinearSingleTrack(US)
    eg = US.mass / US.wheelbase * (US.lr / US.C_f - US.lf / US.C_r)
    assert m.understeer_gradient == pytest.approx(eg)
    assert m.understeer_gradient > 0.0
    assert m.understeer_gradient_deg_per_g(9.81) == pytest.approx(math.degrees(eg) * 9.81)


@pytest.mark.parametrize("vx", SPEEDS)
def test_state_space_dc_gain_matches_closed_form(vx):
    m = LinearSingleTrack(US)
    dc = m.state_space(vx).dc_gain()  # rows r, a_y, beta; columns delta_f, delta_r
    assert dc[0, 0] == pytest.approx(m.yaw_rate_gain(vx), rel=1e-12)
    assert dc[1, 0] == pytest.approx(m.lateral_accel_gain(vx), rel=1e-12)
    assert dc[2, 0] == pytest.approx(m.sideslip_gain(vx), rel=1e-12, abs=1e-15)


def test_neutral_steer_has_kinematic_yaw_gain():
    # C_f l_f = C_r l_r  ->  EG = 0  ->  r/delta = v/L at every speed
    m = _with(C_r=US.C_f * US.lf / US.lr)
    assert m.understeer_gradient == pytest.approx(0.0, abs=1e-15)
    assert m.characteristic_speed is None and m.critical_speed is None
    for vx in SPEEDS:
        assert m.yaw_rate_gain(vx) == pytest.approx(vx / US.wheelbase)


def test_characteristic_speed_maximises_yaw_gain():
    m = LinearSingleTrack(US)
    v = np.linspace(1.0, 120.0, 20001)
    v_peak = v[np.argmax([m.yaw_rate_gain(x) for x in v])]
    assert m.characteristic_speed == pytest.approx(v_peak, rel=1e-3)
    assert m.yaw_rate_gain(m.characteristic_speed) == pytest.approx(m.characteristic_speed / (2 * US.wheelbase))


def test_critical_speed_is_stability_boundary_for_oversteer():
    m = _with(C_f=140_000.0, C_r=80_000.0)
    assert m.understeer_gradient < 0.0
    v_crit = m.critical_speed
    assert np.linalg.det(m.state_space(v_crit).A) == pytest.approx(0.0, abs=1e-6)
    assert m.modal_properties(0.95 * v_crit).stable
    assert not m.modal_properties(1.05 * v_crit).stable


def test_understeering_vehicle_is_stable_at_all_speeds():
    m = LinearSingleTrack(US)
    assert all(m.modal_properties(v).stable for v in np.linspace(1.0, 80.0, 50))


@pytest.mark.parametrize("vx", SPEEDS)
def test_modal_properties_match_eigenvalues(vx):
    modal = LinearSingleTrack(US).modal_properties(vx)
    lam = modal.eigenvalues
    assert np.prod(lam).real == pytest.approx(modal.natural_frequency**2, rel=1e-9)
    assert -np.sum(lam).real == pytest.approx(2 * modal.damping_ratio * modal.natural_frequency, rel=1e-9)
    if np.iscomplexobj(lam) and abs(lam[0].imag) > 0:
        assert modal.damping_ratio < 1.0
        assert abs(lam[0].imag) == pytest.approx(modal.natural_frequency * math.sqrt(1 - modal.damping_ratio**2))


def test_damping_decreases_with_speed():
    m = LinearSingleTrack(US)
    zetas = [m.modal_properties(v).damping_ratio for v in (10.0, 20.0, 30.0, 40.0)]
    assert all(a > b for a, b in zip(zetas, zetas[1:]))


@pytest.mark.parametrize("vx", [10.0, 30.0])
def test_step_response_converges_to_dc_gain_and_has_direct_feedthrough(vx):
    ss = LinearSingleTrack(US).state_space(vx)
    u = np.array([0.01, 0.0])
    y = ss.step_response(np.array([0.0, 20.0]), u)
    np.testing.assert_allclose(y[0], [0.0, US.C_f / US.mass * 0.01, 0.0], atol=1e-12)  # a_y jumps at t=0+
    np.testing.assert_allclose(y[1], ss.dc_gain() @ u, rtol=1e-9)


@pytest.mark.parametrize("vx", SPEEDS)
def test_frequency_response_limits(vx):
    ss = LinearSingleTrack(US).state_space(vx)
    H = ss.frequency_response(np.array([1e-6, 1e8]))  # error terms scale with omega and ||A||/omega
    np.testing.assert_allclose(H[0].real, ss.dc_gain(), rtol=1e-6, atol=1e-9)
    np.testing.assert_allclose(H[1], ss.D, rtol=1e-6, atol=1e-6)


@pytest.mark.parametrize("vx", SPEEDS)
def test_in_phase_equal_rear_steer_gives_pure_translation(vx):
    # delta_f = delta_r: both slip angles vanish at r = 0, beta = delta -> no yaw, no lateral force
    dc = LinearSingleTrack(US).state_space(vx).dc_gain()
    r, ay, beta = dc @ np.array([0.02, 0.02])
    assert r == pytest.approx(0.0, abs=1e-12)
    assert ay == pytest.approx(0.0, abs=1e-10)
    assert beta == pytest.approx(0.02, rel=1e-9)


def test_counter_phase_rear_steer_increases_low_speed_yaw_gain():
    ss = LinearSingleTrack(US).state_space(5.0)
    r_front = (ss.dc_gain() @ np.array([0.02, 0.0]))[0]
    r_4ws = (ss.dc_gain() @ np.array([0.02, -0.01]))[0]
    assert r_4ws > r_front > 0.0


def test_sideslip_sign_change_speed():
    m = LinearSingleTrack(US)
    v0 = math.sqrt(US.lr * US.wheelbase * US.C_r / (US.mass * US.lf))
    assert m.sideslip_gain(v0) == pytest.approx(0.0, abs=1e-12)
    assert m.sideslip_gain(0.5 * v0) > 0.0 > m.sideslip_gain(1.5 * v0)


def test_steady_state_steer_angle():
    m = LinearSingleTrack(US)
    vx, R = 20.0, 100.0
    delta = m.steady_state_steer_angle(vx, R)
    assert m.yaw_rate_gain(vx) * delta == pytest.approx(vx / R)


# ---------------------------------------------------------------- response metrics
def test_metrics_first_order_closed_form():
    tau = 0.3
    t = np.linspace(0.0, 5.0, 50001)
    m = step_response_metrics(t, 2.0 * (1.0 - np.exp(-t / tau)))
    assert m.steady_state == pytest.approx(2.0, rel=1e-6)
    assert m.overshoot == pytest.approx(0.0, abs=1e-6)  # residual exp(-5/tau) in the final window
    assert m.response_time == pytest.approx(tau * math.log(10.0), rel=1e-4)
    assert m.rise_time == pytest.approx(tau * math.log(9.0), rel=1e-4)
    assert m.settling_time == pytest.approx(tau * math.log(50.0), rel=1e-3)


def test_metrics_second_order_overshoot_closed_form():
    zeta, wn = 0.4, 8.0
    wd = wn * math.sqrt(1 - zeta**2)
    t = np.linspace(0.0, 6.0, 60001)
    y = 1 - np.exp(-zeta * wn * t) * (np.cos(wd * t) + zeta / math.sqrt(1 - zeta**2) * np.sin(wd * t))
    m = step_response_metrics(t, y)
    assert m.overshoot == pytest.approx(math.exp(-zeta * math.pi / math.sqrt(1 - zeta**2)), rel=1e-4)
    assert m.peak_time == pytest.approx(math.pi / wd, rel=1e-3)


def test_metrics_handle_negative_steps_and_offsets():
    t = np.linspace(0.0, 4.0, 4001)
    y = 3.0 - (1.0 - np.exp(-t / 0.2))
    m = step_response_metrics(t, y)
    assert m.steady_state == pytest.approx(2.0, rel=1e-6)
    assert m.response_time == pytest.approx(0.2 * math.log(10.0), rel=1e-3)


def test_metrics_reject_zero_step():
    t = np.linspace(0.0, 2.0, 201)
    with pytest.raises(ValueError):
        step_response_metrics(t, np.zeros_like(t))
