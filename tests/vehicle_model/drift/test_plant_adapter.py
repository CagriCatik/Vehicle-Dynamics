"""Deterministic, side-effect-free evaluation of the 7-DOF plant."""

from __future__ import annotations

import numpy as np
import pytest

from vehicle_model.core.types import SevenDOFControlInput, SevenDOFState
from vehicle_model.drift import DeterministicPlant, LoadTransferNotConverged
from vehicle_model.vehicle_models import SevenDOFModel, TransientSevenDOFModel

DRIFT_STATE = SevenDOFState(vx=15.0, vy=-4.0, r=0.6, omega=np.array([48.0, 50.0, 70.0, 72.0]))
DRIFT_INPUT = SevenDOFControlInput(steer_f=-0.15, T_drive=np.array([0.0, 0.0, 600.0, 600.0]))
STRAIGHT = SevenDOFState(vx=20.0, omega=np.full(4, 62.5))


def test_raw_model_is_order_dependent_the_adapter_is_not():
    m = SevenDOFModel.from_yaml()
    d1 = m.evaluate_dynamics(DRIFT_STATE, DRIFT_INPUT)[0]
    m.evaluate_dynamics(STRAIGHT, SevenDOFControlInput())
    d2 = m.evaluate_dynamics(DRIFT_STATE, DRIFT_INPUT)[0]
    assert np.max(np.abs(d1 - d2)) > 0.0                       # the documented side effect of _accel_guess

    p = DeterministicPlant(m)
    a = p.evaluate(DRIFT_STATE.to_array(), DRIFT_INPUT).derivative
    for other in (STRAIGHT, DRIFT_STATE, SevenDOFState(vx=5.0, vy=2.0, r=-1.0, omega=np.full(4, 10.0))):
        p.evaluate(other.to_array(), DRIFT_INPUT)
        assert np.array_equal(p.evaluate(DRIFT_STATE.to_array(), DRIFT_INPUT).derivative, a)


def test_adapter_does_not_touch_the_callers_model_and_agrees_with_it():
    m = SevenDOFModel.from_yaml()
    m.evaluate_dynamics(STRAIGHT, SevenDOFControlInput())
    guess, tol = m._accel_guess, m.load_transfer_tol
    p = DeterministicPlant(m)
    ev = p.evaluate(DRIFT_STATE.to_array(), DRIFT_INPUT)
    assert m._accel_guess == guess and m.load_transfer_tol == tol
    ref = m.evaluate_dynamics(DRIFT_STATE, DRIFT_INPUT)[0]
    assert np.allclose(ev.derivative, ref, atol=1e-4)           # production tolerance 1e-6 m/s^2 of the raw model
    assert not ev.derivative.flags.writeable


def test_non_convergence_is_reported_and_raised_in_strict_mode():
    m = SevenDOFModel.from_yaml()
    loose = DeterministicPlant(m, max_iter=1)
    ev = loose.evaluate(DRIFT_STATE.to_array(), DRIFT_INPUT)
    assert not ev.load_transfer_converged and loose.not_converged == 1
    with pytest.raises(LoadTransferNotConverged):
        DeterministicPlant(m, max_iter=1, strict=True).evaluate(DRIFT_STATE.to_array(), DRIFT_INPUT)


def test_low_speed_and_large_slip_states_stay_finite():
    p = DeterministicPlant(SevenDOFModel.from_yaml())
    for s in (SevenDOFState(vx=0.05, vy=0.02), SevenDOFState(vx=10.0, vy=-8.0, r=1.5, omega=np.array([30, 30, 90, 90.0]))):
        ev = p.evaluate(s.to_array(), DRIFT_INPUT)
        assert np.all(np.isfinite(ev.derivative)) and ev.load_transfer_converged


def test_body_state_round_trip_and_transient_model():
    p = DeterministicPlant(TransientSevenDOFModel.from_yaml())
    y = p.full_state(np.arange(15, dtype=float) + 1.0, nav=(5.0, 6.0, 0.3))
    assert y.size == 18 and np.array_equal(p.body_state(y), np.arange(15, dtype=float) + 1.0)
    assert p.evaluate(y, DRIFT_INPUT).body_derivative.size == 15
