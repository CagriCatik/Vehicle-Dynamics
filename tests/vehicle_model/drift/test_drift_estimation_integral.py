"""Drift D7 follow-up: drift-capable velocity observer and integral action in the drift LQR."""

from __future__ import annotations

import math

import numpy as np
import pytest

from vehicle_model.controllers.estimated_feedback import EstimatedStateFeedback
from vehicle_model.core.types import SevenDOFState
from vehicle_model.drift import DriftEquilibriumSolver, DriftReference
from vehicle_model.drift.control_adapter import DriftLQRController
from vehicle_model.drift.estimator import DriftVelocityObserver
from vehicle_model.drift.linearization import linearise
from vehicle_model.drift.lqr import LQRWeights, design_lqr
from vehicle_model.estimation.blended_sideslip import BlendedSideslipObserver
from vehicle_model.estimation.sensors import SensorMeasurements
from vehicle_model.roads.surface import UniformRoad
from vehicle_model.simulation.drift.hold import simulate
from vehicle_model.simulation.drift.robustness import _SteerSensor
from vehicle_model.vehicle_models import SevenDOFModel


@pytest.fixture(scope="module")
def drift():
    base = SevenDOFModel.from_yaml()
    sol = DriftEquilibriumSolver(base)
    eq = sol.solve(DriftReference(13.0, math.radians(-20.0), 0.9))
    lin = linearise(sol.plant(0.9), eq)
    return base.params, SevenDOFModel.from_yaml(road=UniformRoad(mu=0.9)), eq, lin


def _x0(eq, dbeta_deg=-3.0):
    b = eq.beta + math.radians(dbeta_deg)
    return SevenDOFState(vx=eq.speed * math.cos(b), vy=eq.speed * math.sin(b), r=eq.yaw_rate, omega=eq.omega.copy())


class _Log:
    """Records (true beta, estimated beta, true vx, estimated vx) per step."""

    def __init__(self, inner, est):
        self.inner, self.est, self.rows = inner, est, []

    def reset(self):
        self.inner.reset()

    def compute_control(self, state, target, telemetry, dt):
        out = self.inner.compute_control(state, target, telemetry, dt)
        e = self.est.last_estimate
        self.rows.append((math.atan2(state.vy, state.vx), e.sideslip, state.vx, e.vx))
        return out


def _closed_loop(drift, observer, duration=3.0):
    params, plant, eq, lin = drift
    est = EstimatedStateFeedback(DriftLQRController(params, eq, design_lqr(lin, params)), observer, seed=0, params=params)
    log = _Log(_SteerSensor(est), est)
    h, _ = simulate(plant, log, _x0(eq), duration, 0.005)
    return np.array(log.rows), h, eq


def test_observer_reconstructs_the_drift_without_a_sideslip_prior(drift):
    rows, h, eq = _closed_loop(drift, DriftVelocityObserver(drift[0]))
    assert abs(rows[0, 1]) < 1e-9                         # starts at beta = 0: no knowledge of the drift
    late = rows[len(rows) // 2:]
    assert np.max(np.abs(np.degrees(late[:, 0] - late[:, 1]))) < 2.0
    assert np.max(np.abs(late[:, 2] - late[:, 3])) < 0.3
    beta = np.degrees(np.arctan2(h.vy, h.vx))
    assert abs(beta[-1] - math.degrees(eq.beta)) < 3.0     # the drift is held on estimated states


def test_general_blended_observer_is_not_drift_capable(drift):
    """Expected limit (D7): speed from the spinning rear wheel, start at beta = 0 -> drift lost."""
    rows, h, eq = _closed_loop(drift, BlendedSideslipObserver(drift[0]))
    beta = np.arctan2(h.vy, h.vx)
    assert np.max(np.abs(beta - eq.beta)) > math.radians(30.0)
    assert np.max(rows[:, 3] - rows[:, 2]) > 3.0           # speed estimate far above the true speed


def test_observer_measurement_model_is_exact_for_free_rolling_front_wheels(drift):
    params = drift[0]
    vx, vy, r, d = 12.0, -4.5, 0.65, -0.15
    c, s = math.cos(d), math.sin(d)
    w = [((vx - s_ * r * params.wf / 2) * c + (vy + params.lf * r) * s) / params.Reff for s_ in (1.0, -1.0)]
    obs = DriftVelocityObserver(params)
    meas = SensorMeasurements(t=0.0, ax=0.0, ay=0.0, r=r, omega=np.array([w[0], w[1], 80.0, 80.0]), steer_f=d, steer_r=0.0)
    y, H = obs._front(meas)
    assert np.allclose(y, H @ np.array([vx, vy]))
    assert obs.information == "sensors_only"


def test_integral_design_shape_and_validation(drift):
    params, _, _, lin = drift
    k = design_lqr(lin, params, integral=True)
    assert k.K.shape == (2, 9) and k.integral and k.stable
    with pytest.raises(ValueError, match="steer-rate"):
        design_lqr(lin, params, integral=True, steer_rate=False)
    with pytest.raises(ValueError):
        LQRWeights(q_integral=-1.0)
    assert design_lqr(lin, params).K.shape == (2, 8)       # default unchanged


def test_integral_does_not_wind_up_while_saturated(drift):
    params, _, eq, lin = drift
    c = DriftLQRController(params, eq, design_lqr(lin, params, integral=True))
    far = SevenDOFState(vx=13.0, vy=0.0, r=0.0, omega=eq.omega.copy())      # beta 20 deg off: commands saturate
    out = c.compute_control(far, dt=0.005)
    assert out.status == "saturated" and c._z == 0.0
    c.reset()
    near = _x0(eq, -0.02)
    out = c.compute_control(near, dt=0.005)
    assert out.status != "saturated" and c._z == pytest.approx(math.radians(-0.02) * 0.005, rel=1e-6)


@pytest.mark.slow
def test_integral_removes_the_steady_offset_of_a_deviating_plant(drift):
    """mu 0.8 instead of the design mu 0.9: D7 offset -3.4 deg without integral action."""
    params, _, eq, lin = drift
    plant = SevenDOFModel.from_yaml(road=UniformRoad(mu=0.8))
    final = {}
    for integral in (False, True):
        h, _ = simulate(plant, DriftLQRController(params, eq, design_lqr(lin, params, integral=integral)), _x0(eq), 10.0, 0.005)
        final[integral] = math.degrees(math.atan2(h.vy[-1], h.vx[-1]) - eq.beta)
    assert abs(final[False]) > 2.0 and abs(final[True]) < 0.5
