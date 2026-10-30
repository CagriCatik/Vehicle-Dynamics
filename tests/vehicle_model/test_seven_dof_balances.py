"""Acceptance for the 7-DOF two-track model.

Force, moment and wheel-torque balances of ``evaluate_dynamics``, symmetric
limit cases, temporal convergence of the integrator and explicit contact-loss
reporting in a simulated manoeuvre.
"""

from __future__ import annotations

import dataclasses
import math

import numpy as np
import pytest

import vehicle_model.vehicle_models.seven_dof as seven_dof_module
from vehicle_model.core.types import FL, FR, RL, RR, SevenDOFControlInput, SevenDOFState
from vehicle_model.roads.surface import UniformRoad
from vehicle_model.simulation.scenario_runner import ScenarioRunner
from vehicle_model.vehicle_models import SevenDOFModel
from vehicle_model.vehicle_models.normal_loads import (
    SUPPORT_THREE_WHEEL,
    SUPPORT_TIP_OVER,
    quasi_static_normal_loads,
)

REFERENCE = "configs/vehicles/reference_passenger_car.yaml"
MIRROR = [FR, FL, RR, RL]


@pytest.fixture(scope="module", params=["reference", "default"])
def model(request) -> SevenDOFModel:
    """Reference car (Fiala tyres) and the legacy default car (Pacejka tyres)."""
    return SevenDOFModel.from_yaml(REFERENCE if request.param == "reference" else None)


@pytest.fixture(scope="module")
def ref_model() -> SevenDOFModel:
    return SevenDOFModel.from_yaml(REFERENCE)


def _random_cases(n: int, seed: int = 7):
    rng = np.random.default_rng(seed)
    for _ in range(n):
        vx = rng.uniform(3.0, 35.0)
        state = SevenDOFState(
            vx=vx,
            vy=rng.uniform(-1.0, 1.0),
            r=rng.uniform(-0.4, 0.4),
            omega=vx / 0.31 * (1.0 + rng.uniform(-0.08, 0.08, 4)),
            X=rng.uniform(-50, 50),
            Y=rng.uniform(-50, 50),
            psi=rng.uniform(-math.pi, math.pi),
        )
        u = SevenDOFControlInput(
            steer_f=rng.uniform(-0.1, 0.1),
            steer_r=rng.uniform(-0.03, 0.03),
            T_drive=rng.uniform(-200.0, 400.0, 4),
            T_brake=rng.uniform(0.0, 600.0, 4),
        )
        yield state, u


# --------------------------------------------------------------------------- balances
def test_chassis_force_and_yaw_moment_balance(model):
    p = model.params
    pos = model.wheel_positions
    for state, u in _random_cases(40):
        u = u.clamped(p)
        d, tel, ax, ay = model.evaluate_dynamics(state, u)
        F_aero = 0.5 * p.rho * p.Cd * p.Af * state.vx * abs(state.vx)
        F_roll = p.c_rr * p.mass * p.g * math.tanh(state.vx / 0.5)
        # Newton in the rotating body frame
        assert p.mass * (d[0] - state.r * state.vy) == pytest.approx(tel.Fx_body.sum() - F_aero - F_roll, rel=1e-12, abs=1e-9)
        assert p.mass * (d[1] + state.r * state.vx) == pytest.approx(tel.Fy_body.sum(), rel=1e-12, abs=1e-9)
        # Euler about z through the CoG
        Mz = np.sum(pos[:, 0] * tel.Fy_body - pos[:, 1] * tel.Fx_body)
        assert p.Iz * d[2] == pytest.approx(Mz, rel=1e-12, abs=1e-9)
        # Wheel-frame to body-frame force rotation preserves magnitudes
        assert np.allclose(np.hypot(tel.Fx_body, tel.Fy_body), np.hypot(tel.Fx_wheel, tel.Fy_wheel), rtol=1e-12)


def test_wheel_spin_torque_balance(model):
    p = model.params
    for state, u in _random_cases(40, seed=11):
        u = u.clamped(p)
        d, tel, _, _ = model.evaluate_dynamics(state, u)
        T_brake = u.T_brake * np.tanh(state.omega / seven_dof_module._OMEGA_EPS)
        net = u.T_drive - T_brake - tel.Fx_wheel * p.Reff - p.wheel_damping * state.omega
        assert np.allclose(p.Iw * d[3:7], net, rtol=1e-12, atol=1e-9)


def test_navigation_kinematics_and_contact_points(model):
    for state, u in _random_cases(20, seed=3):
        d, tel, _, _ = model.evaluate_dynamics(state, u.clamped(model.params))
        c, s = math.cos(state.psi), math.sin(state.psi)
        assert d[7] == pytest.approx(state.vx * c - state.vy * s, rel=1e-12, abs=1e-12)
        assert d[8] == pytest.approx(state.vx * s + state.vy * c, rel=1e-12, abs=1e-12)
        assert d[9] == state.r
        pos = model.wheel_positions
        expected = np.column_stack([
            state.X + pos[:, 0] * c - pos[:, 1] * s,
            state.Y + pos[:, 0] * s + pos[:, 1] * c,
        ])
        assert np.allclose(tel.contact_xy, expected, rtol=0, atol=1e-12)


def test_normal_loads_consistent_with_accelerations(model):
    """The algebraic loop ax, ay -> Fz is solved: Fz equals the quasi-static loads of the returned ax, ay."""
    p = model.params
    tol = seven_dof_module._LT_TOL
    dF_da = p.mass * p.hcog / min(p.wf, p.wr, p.wheelbase)  # upper bound on |dFz/da|
    for state, u in _random_cases(40, seed=5):
        _, tel, ax, ay = model.evaluate_dynamics(state, u.clamped(p))
        assert tel.load_transfer_converged
        assert tel.support == "four_wheel" and tel.in_contact.all()
        assert np.allclose(tel.Fz, quasi_static_normal_loads(p, ax, ay).Fz, rtol=0, atol=10 * tol * dF_da)


# --------------------------------------------------------------------------- symmetry
def _simulate(model, state, u_of_t, duration=1.5, dt=0.005, road=None):
    traj = []
    for k in range(int(round(duration / dt))):
        state, tel = model.step(state, u_of_t(k * dt), dt, road=road)
        traj.append(state.to_array())
    return np.array(traj)


@pytest.mark.parametrize("T_drive, T_brake", [(150.0, 0.0), (0.0, 900.0)])
def test_straight_line_stays_straight_with_symmetric_torques(ref_model, T_drive, T_brake):
    p = ref_model.params
    state = SevenDOFState(vx=20.0, omega=np.full(4, 20.0 / p.Reff))
    u = SevenDOFControlInput(T_drive=np.full(4, T_drive), T_brake=np.full(4, T_brake))
    traj = _simulate(ref_model, state, lambda t: u)
    assert np.max(np.abs(traj[:, [1, 2, 8, 9]])) < 1e-12  # vy, r, Y, psi
    assert np.allclose(traj[:, 3], traj[:, 4], rtol=0, atol=1e-12)  # FL == FR
    assert np.allclose(traj[:, 5], traj[:, 6], rtol=0, atol=1e-12)  # RL == RR


def test_mirrored_inputs_give_mirrored_motion(ref_model):
    """Steer +delta with a left-rear brake vs. steer -delta with a right-rear brake."""
    p = ref_model.params
    state = SevenDOFState(vx=20.0, omega=np.full(4, 20.0 / p.Reff))

    def left(t):
        return SevenDOFControlInput(steer_f=0.04 * min(t / 0.2, 1.0), steer_r=0.005,
                                    T_drive=np.array([0.0, 0.0, 80.0, 40.0]), T_brake=np.array([0.0, 0.0, 300.0, 0.0]))

    def right(t):
        u = left(t)
        return SevenDOFControlInput(steer_f=-u.steer_f, steer_r=-u.steer_r,
                                    T_drive=u.T_drive[MIRROR], T_brake=u.T_brake[MIRROR])

    a = _simulate(ref_model, state, left)
    b = _simulate(ref_model, state, right)
    assert np.max(np.abs(a[:, 2])) > 0.1  # the manoeuvre actually turns
    sign = np.array([1, -1, -1, 1, 1, 1, 1, 1, -1, -1], dtype=float)  # vx, vy, r, omega x4, X, Y, psi
    b_mirrored = b * sign
    b_mirrored[:, 3:7] = b[:, 3:7][:, MIRROR]
    assert np.allclose(a, b_mirrored, rtol=1e-9, atol=1e-9)


# --------------------------------------------------------------------------- convergence
@pytest.mark.slow
def test_rk4_temporal_convergence_order(ref_model, monkeypatch):
    """Self-convergence of the fixed-step integrator (4th order once the wheel mode is resolved).

    The load-transfer tolerance is tightened so that the algebraic-loop residual
    (production tolerance: noise floor ~1e-8) does not mask the truncation error.
    """
    monkeypatch.setattr(ref_model, "load_transfer_tol", 1e-12)
    monkeypatch.setattr(ref_model, "load_transfer_max_iter", 60)
    p = ref_model.params
    u = SevenDOFControlInput(steer_f=0.03, T_drive=np.full(4, 60.0), T_brake=np.array([0.0, 0.0, 80.0, 0.0]))

    def final(dt, T=0.4):
        s = SevenDOFState(vx=20.0, vy=0.2, r=0.1, omega=np.array([66.0, 64.0, 65.0, 63.0]))
        for _ in range(int(round(T / dt))):
            s, _ = ref_model.step(s, u, dt)
        return s.to_array()

    reference = final(1e-4)
    errors = [np.max(np.abs(final(dt) - reference)) for dt in (0.002, 0.001, 0.0005)]
    orders = np.log2(np.array(errors[:-1]) / np.array(errors[1:]))
    assert np.all(orders > 3.5), orders
    assert np.all(orders < 5.5), orders


def test_default_step_is_accurate(ref_model):
    """At dt = 5 ms (stability-based sub-stepping) the state agrees with a 0.5 ms solution to < 1e-6."""
    p = ref_model.params
    u = SevenDOFControlInput(steer_f=0.03, T_drive=np.full(4, 60.0), T_brake=np.array([0.0, 0.0, 80.0, 0.0]))

    def final(dt, T=0.5):
        s = SevenDOFState(vx=20.0, omega=np.full(4, 20.0 / p.Reff))
        for _ in range(int(round(T / dt))):
            s, _ = ref_model.step(s, u, dt)
        return s.to_array()

    assert np.max(np.abs(final(0.005) - final(0.0005))) < 1e-6


# --------------------------------------------------------------------------- contact loss
def test_ramp_steer_reports_lift_off_and_keeps_moment_balance(ref_model):
    """High-CoG variant: the inner front wheel lifts; the loads stay in equilibrium (Audit 4.3)."""
    p = dataclasses.replace(ref_model.params, hcog=0.9, k_roll_f=0.9)
    model = SevenDOFModel(p, ref_model.tyre_front, tyre_rear=ref_model.tyre_rear)
    V = 20.0

    def controller(t, s, tel):
        f_req = 0.5 * p.rho * p.Cd * p.Af * s.vx**2 + p.c_rr * p.mass * p.g + 3.0 * p.mass * (V - s.vx)
        return SevenDOFControlInput(steer_f=min(math.radians(4.0) * t, math.radians(8.0)),
                                    T_drive=np.full(4, max(f_req, 0.0) * p.Reff / 4.0))

    hist = ScenarioRunner(model, UniformRoad(mu=0.75)).run(
        controller, SevenDOFState(vx=V, omega=np.full(4, V / p.Reff)), duration=3.0, dt=0.005)

    events = hist.contact_events()
    assert events and events[0][1] == FL and events[0][2] == "lift_off"
    assert SUPPORT_THREE_WHEEL in set(hist.support)
    assert SUPPORT_TIP_OVER not in set(hist.support)
    lifted = ~hist.in_contact[:, FL]
    assert np.all(hist.Fz[lifted, FL] == 0.0)

    # Equilibrium of the recorded loads with the recorded accelerations
    pos = model.wheel_positions
    mg = p.mass * p.g
    F_aero = 0.5 * p.rho * p.Cd * p.Af * hist.vx * np.abs(hist.vx)
    ax = (hist.Fx_body.sum(axis=1) - F_aero - p.c_rr * mg * np.tanh(hist.vx / 0.5)) / p.mass
    ay = hist.Fy_body.sum(axis=1) / p.mass
    tol = 10 * seven_dof_module._LT_TOL * p.mass * p.hcog
    assert np.allclose(hist.Fz.sum(axis=1), mg, rtol=1e-12)
    # vx in the record lags the telemetry by one step; the drag term is negligible at this tolerance
    assert np.max(np.abs(hist.Fz @ pos[:, 1] + p.mass * ay * p.hcog)) < max(tol, 1e-6 * mg)
    assert np.max(np.abs(hist.Fz @ pos[:, 0] + p.mass * ax * p.hcog)) < 1e-3 * mg
