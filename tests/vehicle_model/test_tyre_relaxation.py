"""Transient tyre behaviour: first-order relaxation.

Component: analytical step response over rolled distance, steady states equal
to the quasi-static slips, standstill. Vehicle: the transient 7-DOF variant
reproduces the base model in steady state and converges to it as the
relaxation lengths go to zero.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from vehicle_model.core.types import SevenDOFControlInput, SevenDOFState
from vehicle_model.tire_model import TyreRelaxation
from vehicle_model.vehicle_models import SevenDOFModel, TransientSevenDOFModel, TransientSevenDOFState

REFERENCE = "configs/vehicles/reference_passenger_car.yaml"


def _integrate(rel: TyreRelaxation, v_wx, v_wy, v_wheel, T, dt=1e-4, k0=0.0, s0=0.0):
    """RK4 of the relaxation ODE at constant wheel velocities; returns t, kappa', tan(alpha')."""
    y = np.array([k0, s0], dtype=float)
    out = [y.copy()]

    def f(y):
        dk, ds = rel.derivatives(y[0], y[1], v_wx, v_wy, v_wheel)
        return np.array([float(dk), float(ds)])

    n = int(round(T / dt))
    for _ in range(n):
        k1 = f(y); k2 = f(y + 0.5 * dt * k1); k3 = f(y + 0.5 * dt * k2); k4 = f(y + dt * k3)
        y = y + dt / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)
        out.append(y.copy())
    return np.arange(n + 1) * dt, np.array(out)


# ----------------------------------------------------------------------------- component
@pytest.mark.parametrize("V", [5.0, 30.0])
def test_lateral_step_response_is_exponential_in_rolled_distance(V):
    rel = TyreRelaxation(sigma_kappa=0.1, sigma_alpha=0.4)
    s_ss = math.tan(math.radians(3.0))
    T = 5 * rel.sigma_alpha / V
    t, y = _integrate(rel, V, -s_ss * V, V, T)
    x = V * t
    assert np.allclose(y[:, 1], s_ss * (1.0 - np.exp(-x / rel.sigma_alpha)), rtol=0, atol=1e-10)
    i = np.argmin(np.abs(x - rel.sigma_alpha))
    assert y[i, 1] / s_ss == pytest.approx(1.0 - math.exp(-1.0), abs=1e-3)  # 63 % after one sigma


def test_longitudinal_step_response():
    rel = TyreRelaxation(sigma_kappa=0.15, sigma_alpha=0.4)
    V, kappa = 20.0, -0.05  # braking: wheel slower than the road
    v_wheel = (1 + kappa) * V
    t, y = _integrate(rel, V, 0.0, v_wheel, 5 * rel.sigma_kappa / V)
    assert np.allclose(y[:, 0], kappa * (1.0 - np.exp(-V * t / rel.sigma_kappa)), rtol=0, atol=1e-10)


def test_steady_state_equals_quasi_static_slips():
    rng = np.random.default_rng(1)
    rel = TyreRelaxation()
    base = SevenDOFModel.from_yaml(REFERENCE)
    v_wx = rng.uniform(-30, 30, 200)
    v_wy = rng.uniform(-3, 3, 200)
    v_wheel = v_wx * (1 + rng.uniform(-0.3, 0.3, 200)) + rng.uniform(-0.5, 0.5, 200)
    k_ss, s_ss = rel.steady_state(v_wx, v_wy, v_wheel)
    kappa, alpha = base._slips(None, v_wx, v_wy, v_wheel)
    assert np.allclose(k_ss, kappa, rtol=1e-14, atol=0)
    assert np.allclose(np.arctan(s_ss), alpha, rtol=1e-14, atol=1e-15)
    dk, ds = rel.derivatives(k_ss, s_ss, v_wx, v_wy, v_wheel)
    assert np.max(np.abs(dk)) < 1e-12 and np.max(np.abs(ds)) < 1e-12


def test_well_posed_at_standstill():
    rel = TyreRelaxation()
    dk, ds = rel.derivatives(0.0, 0.0, 0.0, -0.05, 0.0)  # lateral creep at standstill: deflection grows
    assert np.isfinite(dk) and ds == pytest.approx(0.05 / rel.sigma_alpha)


def test_invalid_parameters_rejected():
    with pytest.raises(ValueError):
        TyreRelaxation(sigma_alpha=0.0)
    with pytest.raises(ValueError):
        TransientSevenDOFModel.from_yaml(REFERENCE, relaxation=TyreRelaxation(v_eps=0.5))


def test_transient_state_round_trip():
    s = TransientSevenDOFState(vx=3.0, omega=np.arange(4.0), kappa_t=np.full(4, 0.1), tan_alpha_t=np.arange(4) * 0.01)
    t = TransientSevenDOFState.from_array(s.to_array())
    assert np.array_equal(t.to_array(), s.to_array()) and s.to_array().size == 18


# ----------------------------------------------------------------------------- vehicle
@pytest.fixture(scope="module")
def models():
    return SevenDOFModel.from_yaml(REFERENCE), TransientSevenDOFModel.from_yaml(REFERENCE)


def test_steady_slip_state_reproduces_base_derivative(models):
    base, tr = models
    p = base.params
    rng = np.random.default_rng(4)
    for _ in range(20):
        vx = rng.uniform(5, 30)
        s = SevenDOFState(vx=vx, vy=rng.uniform(-0.5, 0.5), r=rng.uniform(-0.3, 0.3),
                          omega=vx / p.Reff * (1 + rng.uniform(-0.05, 0.05, 4)))
        u = SevenDOFControlInput(steer_f=rng.uniform(-0.05, 0.05), T_drive=rng.uniform(0, 200, 4))
        d_base, tel_b, _, _ = base.evaluate_dynamics(s, u)
        d_tr, tel_t, _, _ = tr.evaluate_dynamics(tr.steady_slip_state(s, u), u)
        assert np.allclose(d_tr[:10], d_base, rtol=1e-6, atol=1e-6)
        assert np.max(np.abs(d_tr[10:])) < 1e-9
        assert np.allclose(tel_t.alpha, tel_b.alpha, atol=1e-14)


def _step_steer(model, s0, T=0.6, dt=0.005):
    u = SevenDOFControlInput(steer_f=0.03, T_drive=np.full(4, 60.0))
    r, alpha = [], []
    s = s0
    for _ in range(int(round(T / dt))):
        s, tel = model.step(s, u, dt)
        r.append(s.r)
        alpha.append(tel.alpha[0])
    return np.array(r), np.array(alpha)


def test_relaxation_delays_slip_build_up(models):
    base, tr = models
    p = base.params
    s0 = SevenDOFState(vx=20.0, omega=np.full(4, 20.0 / p.Reff))
    r_b, a_b = _step_steer(base, s0)
    r_t, a_t = _step_steer(tr, tr.steady_slip_state(s0, SevenDOFControlInput()))
    # first 5 ms ~ 0.25 sigma_alpha/V: transient slip well below the quasi-static one
    assert a_t[0] < 0.6 * a_b[0]
    assert r_t[0] < 0.5 * r_b[0]
    # same steady state, slightly larger overshoot (less yaw damping)
    assert r_t[-1] == pytest.approx(r_b[-1], rel=5e-3)
    assert r_t.max() >= r_b.max()


def test_converges_to_base_model_as_relaxation_length_vanishes(models):
    base, _ = models
    p = base.params
    s0 = SevenDOFState(vx=20.0, omega=np.full(4, 20.0 / p.Reff))
    r_b, _ = _step_steer(base, s0)
    dev = []
    for sigma in (0.4, 0.2, 0.1):
        tr = TransientSevenDOFModel.from_yaml(REFERENCE, relaxation=TyreRelaxation(sigma_kappa=sigma / 4, sigma_alpha=sigma))
        r_t, _ = _step_steer(tr, tr.steady_slip_state(s0, SevenDOFControlInput()))
        dev.append(np.max(np.abs(r_t - r_b)))
    ratios = np.array(dev[:-1]) / np.array(dev[1:])
    assert np.all((ratios > 1.6) & (ratios < 2.5)), ratios  # first order in sigma


def test_low_speed_launch_is_stable(models):
    _, tr = models
    s = tr.steady_slip_state(SevenDOFState(vx=0.0), SevenDOFControlInput())
    u = SevenDOFControlInput(steer_f=0.2, T_drive=np.full(4, 150.0))
    for _ in range(400):
        s, tel = tr.step(s, u, 0.005)
    assert np.all(np.isfinite(s.to_array()))
    assert 1.0 < s.vx < 8.0 and np.all(np.abs(tel.kappa) < 0.3)
