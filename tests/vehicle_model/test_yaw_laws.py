"""LQR, gain-scheduled LQR and LTV-MPC yaw laws, and the shared motion-control chain.

Analytical checks run the laws against their own linear design model as the
plant (Riccati equation, closed-loop stability, equilibrium, exact tracking,
constraints, solver failure). The nonlinear plant is used in the benchmark tests.
"""

from __future__ import annotations

import types

import numpy as np
import pytest
from scipy.linalg import solve_continuous_are

import vehicle_model.controllers.yaw_laws as yl
from vehicle_model.controllers.motion_control import MotionContext, MotionController, MotionLaw
from vehicle_model.controllers.solver_diagnostics import DEGRADED, FALLBACK, NOMINAL
from vehicle_model.controllers.stability_control import AllocatedStabilityController
from vehicle_model.controllers.yaw_laws import (
    INPUTS_4WS_TV,
    INPUTS_TV,
    LQRWeights,
    LQRYawLaw,
    LTVMPCYawLaw,
    MPCSettings,
    YawDesignModel,
    YawRatePI,
    lqr_gain,
)
from vehicle_model.core.types import SevenDOFState
from vehicle_model.handling_analysis import linear_params_from_seven_dof
from vehicle_model.vehicle_models import SevenDOFModel

VEHICLE = "configs/vehicles/reference_passenger_car.yaml"
MF = "magic_formula_passenger_synthetic"
SPEEDS = (5.0, 10.0, 15.0, 20.0, 30.0, 40.0)


@pytest.fixture(scope="module")
def design():
    m = SevenDOFModel.from_yaml(VEHICLE, tyre_name=MF, tyre_rear_name=MF)
    return linear_params_from_seven_dof(m.params, m.tyre_front, m.tyre_rear)


def _ctx(steer, r_ref, dt=0.005, max_mz=4000.0, max_dr=0.087):
    return MotionContext(steer, r_ref, 1.0, dt, max_mz, max_dr)


def _simulate_linear(design, law, vx=20.0, steer=0.03, T=4.0, dt=0.005, r_ref=None, ctx_kw=None):
    """Closed loop on the linear design model itself: x = [vy, r]."""
    model = YawDesignModel(design, INPUTS_4WS_TV)
    A, B_all = model.matrices(vx)
    r_ref = vx * steer / (design.lf + design.lr + 0.001 * vx**2) if r_ref is None else r_ref
    x = np.zeros(2)
    for _ in range(int(round(T / dt))):
        req = law.request(SevenDOFState(vx=vx, vy=x[0], r=x[1]), _ctx(steer, r_ref, dt, **(ctx_kw or {})))
        u = np.array([steer, req.rear_steer, req.yaw_moment])
        k1 = A @ x + B_all @ u
        k2 = A @ (x + 0.5 * dt * k1) + B_all @ u
        k3 = A @ (x + 0.5 * dt * k2) + B_all @ u
        k4 = A @ (x + dt * k3) + B_all @ u
        x = x + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
    return x, r_ref, req


# ----------------------------------------------------------------------------- design model
@pytest.mark.parametrize("inputs", [INPUTS_TV, INPUTS_4WS_TV])
def test_equilibrium_is_a_steady_state(design, inputs):
    m = YawDesignModel(design, inputs)
    A, B_all = m.matrices(25.0)
    x_ss, u_ss = m.equilibrium(25.0, 0.02, 0.15)
    u_all = np.array([0.02, 0.0, 0.0])
    u_all[m.cols] = u_ss
    assert np.allclose(A @ x_ss + B_all @ u_all, 0.0, atol=1e-10)
    assert x_ss[1] == 0.15
    if inputs == INPUTS_4WS_TV:
        assert x_ss[0] == 0.0


def test_invalid_input_set_rejected(design):
    with pytest.raises(ValueError):
        YawDesignModel(design, ("rear_steer",))
    with pytest.raises(ValueError):
        LQRYawLaw(design, design_speed=None)


# ----------------------------------------------------------------------------- LQR
@pytest.mark.parametrize("inputs", [INPUTS_TV, INPUTS_4WS_TV])
def test_lqr_solves_the_riccati_equation_and_stabilises(design, inputs):
    m = YawDesignModel(design, inputs)
    Q, R = LQRWeights(q_vy=1.0).matrices(m.inputs)
    for v in SPEEDS:
        A, B = m.augmented(v)
        K = lqr_gain(A, B, Q, R)
        P = solve_continuous_are(A, B, Q, R)
        assert np.allclose(A.T @ P + P @ A - P @ B @ np.linalg.solve(R, B.T @ P) + Q, 0.0, atol=1e-6 * np.abs(P).max())
        assert np.all(np.linalg.eigvals(A - B @ K).real < 0)


def test_gain_schedule_interpolates_and_clamps(design):
    gs = LQRYawLaw(design, INPUTS_TV, design_speed=None, schedule_speeds=SPEEDS)
    assert np.allclose(gs.gain(20.0), gs.gains[3])
    assert np.allclose(gs.gain(17.5), 0.5 * (gs.gains[2] + gs.gains[3]))
    assert np.allclose(gs.gain(100.0), gs.gains[-1]) and np.allclose(gs.gain(1.0), gs.gains[0])
    assert gs.name == "GS-LQR (TV)" and LQRYawLaw(design, INPUTS_4WS_TV).name == "LQR (4WS+TV)"


@pytest.mark.parametrize("inputs", [INPUTS_TV, INPUTS_4WS_TV])
@pytest.mark.parametrize("vx", [12.0, 30.0])
def test_lqr_tracks_the_reference_on_its_design_model(design, inputs, vx):
    law = LQRYawLaw(design, inputs, design_speed=None, schedule_speeds=SPEEDS)
    steer = 0.36 / vx  # same moderate lateral acceleration at both speeds (inputs not saturated)
    x, r_ref, req = _simulate_linear(design, law, vx=vx, steer=steer)
    assert abs(req.yaw_moment) < 4000.0 and abs(req.rear_steer) < 0.087
    assert x[1] == pytest.approx(r_ref, rel=1e-3)
    if inputs == INPUTS_4WS_TV:
        assert abs(x[0]) < 1e-3 * vx  # zero sideslip target reached


def test_lqr_respects_input_limits(design):
    law = LQRYawLaw(design, INPUTS_TV, design_speed=None, schedule_speeds=SPEEDS)
    _, _, req = _simulate_linear(design, law, steer=0.05, r_ref=0.6, ctx_kw={"max_mz": 500.0})
    assert abs(req.yaw_moment) <= 500.0 + 1e-9


# ----------------------------------------------------------------------------- MPC
@pytest.mark.parametrize("inputs", [INPUTS_TV, INPUTS_4WS_TV])
def test_mpc_tracks_the_reference_on_its_design_model(design, inputs):
    law = LTVMPCYawLaw(design, inputs)
    x, r_ref, req = _simulate_linear(design, law, vx=20.0, T=3.0)
    assert x[1] == pytest.approx(r_ref, rel=1e-3)
    assert req.status == NOMINAL and law.monitor.counts[NOMINAL] > 0


def test_mpc_respects_input_limits_in_the_whole_plan(design):
    law = LTVMPCYawLaw(design, INPUTS_4WS_TV)
    _, _, req = _simulate_linear(design, law, steer=0.05, r_ref=0.6, T=0.5, ctx_kw={"max_mz": 500.0, "max_dr": 0.01})
    assert np.all(np.abs(law._plan[:, 1] + req.diagnostics["u_ss"][1]) <= 500.0 + 1e-6)
    assert abs(req.rear_steer) <= 0.01 + 1e-12 and abs(req.yaw_moment) <= 500.0 + 1e-9


def test_mpc_solution_is_a_kkt_point_also_with_active_limits(design):
    """BVLS solves the box-constrained least squares exactly: the KKT residuals must vanish."""
    law = LTVMPCYawLaw(design, INPUTS_4WS_TV)
    _, _, req = _simulate_linear(design, law, steer=0.05, r_ref=0.6, T=0.5, ctx_kw={"max_mz": 500.0, "max_dr": 0.01})
    rep = req.diagnostics["solver"]
    assert rep["kkt_stationarity_relative"] < 1e-6 and rep["kkt_primal"] < 1e-9
    assert law.monitor.summary()["kkt_stationarity_relative_max"] < 1e-6


def test_mpc_solver_failure_falls_back_and_is_reported(design, monkeypatch):
    law = LTVMPCYawLaw(design, INPUTS_TV, MPCSettings(max_consecutive_failures=2))
    ctx = _ctx(0.03, 0.2)
    state = SevenDOFState(vx=20.0, r=0.1)
    law.request(state, ctx)
    plan = law._plan.copy()

    def boom(*a, **k):
        raise np.linalg.LinAlgError("injected")

    monkeypatch.setattr(yl, "lsq_linear", boom)
    first, second = law.request(state, ctx), law.request(state, ctx)
    assert (first.status, second.status) == (FALLBACK, DEGRADED)
    assert "injected" in second.diagnostics["solver"]["message"]
    assert np.all(np.isfinite(law._plan)) and law._plan.shape == plan.shape


# ----------------------------------------------------------------------------- chain
def test_all_laws_implement_the_motion_law_protocol(design):
    for law in (YawRatePI(), LQRYawLaw(design), LTVMPCYawLaw(design)):
        assert isinstance(law, MotionLaw) and law.information == "full_state_ideal"


def test_pi_chain_equals_the_m5_stability_controller():
    from vehicle_model.controllers.framework import ControlTarget
    m = SevenDOFModel.from_yaml(VEHICLE)
    a = AllocatedStabilityController(m.params)
    b = MotionController(m.params, YawRatePI())
    s = SevenDOFState(vx=20.0, r=0.05, omega=np.full(4, 20.0 / m.params.Reff))
    tgt = ControlTarget(driver_steer_f=0.03, driver_accel_pedal=0.1)
    for _ in range(5):
        oa, ob = a.compute_control(s, tgt, None, 0.005), b.compute_control(s, tgt, None, 0.005)
        assert np.allclose(oa.T_drive, ob.T_drive) and np.allclose(oa.T_brake, ob.T_brake)
        assert oa.desired_yaw_moment == ob.desired_yaw_moment
