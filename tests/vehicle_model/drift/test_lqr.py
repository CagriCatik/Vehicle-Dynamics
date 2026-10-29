"""Drift plan D3: LQR design and closed-loop hold on the 7-DOF plant (scenarios S01-S03, short)."""

from __future__ import annotations

import dataclasses
import math

import numpy as np
import pytest

from vehicle_model.core.types import SevenDOFState
from vehicle_model.drift import DriftEquilibriumSolver, DriftReference
from vehicle_model.drift.control_adapter import DriftLQRController
from vehicle_model.drift.linearization import linearise
from vehicle_model.drift.lqr import LQRWeights, NotStabilisable, design_lqr
from vehicle_model.drift.metrics import hold_kpis
from vehicle_model.roads.surface import UniformRoad
from vehicle_model.simulation.drift.hold import _OpenLoop, initial_state, simulate
from vehicle_model.vehicle_models import SevenDOFModel


@pytest.fixture(scope="module")
def drift():
    base = SevenDOFModel.from_yaml()
    sol = DriftEquilibriumSolver(base)
    out = {}
    for d, b in (("left", -20.0), ("right", 20.0)):
        eq = sol.solve(DriftReference(13.0, math.radians(b), 0.9))
        lin = linearise(sol.plant(0.9), eq)
        out[d] = (eq, lin, design_lqr(lin, base.params))
    plant = SevenDOFModel.from_yaml(road=UniformRoad(mu=0.9))
    return base.params, plant, out


def test_weights_with_wrong_dimensions_fail_hard():
    with pytest.raises(ValueError, match="7 state and 2 input"):
        LQRWeights(q_scaled=(1.0,) * 6)
    with pytest.raises(ValueError):
        LQRWeights(r_scaled=(0.0, 1.0))


def test_design_refuses_a_non_stabilisable_linearisation(drift):
    params, _, out = drift
    lin = out["left"][1]
    blind = dataclasses.replace(lin, stabilisable=False, pbh_ranks=(5, 7))
    with pytest.raises(NotStabilisable):
        design_lqr(blind, params)


def test_linear_closed_loop_is_stable_in_both_designs(drift):
    params, _, out = drift
    eq, lin, lqr = out["left"]
    assert lqr.stable and lqr.steer_rate and lqr.K.shape == (2, 8)
    assert design_lqr(lin, params, steer_rate=False).stable


def _run(params, plant, eq, lqr, pert, duration=3.0, controller=None):
    ctrl = controller or DriftLQRController(params, eq, lqr)
    h, sat = simulate(plant, ctrl, initial_state(eq, pert), duration, 0.005)
    return h, hold_kpis(h.time, h.vx, h.vy, h.r, eq.beta, eq.yaw_rate, eq.speed, 1.5, sat)


@pytest.mark.parametrize("direction", ["left", "right"])
def test_s01_s02_hold_from_the_equilibrium(drift, direction):
    params, plant, out = drift
    eq, _, lqr = out[direction]
    h, k = _run(params, plant, eq, lqr, {"name": "hold"})
    assert k["max_beta_error_deg"] < 0.1 and k["yaw_rmse_degps"] < 0.1 and not k["stability_loss"]
    assert np.sign(h.r[-1]) == (1 if direction == "left" else -1)


@pytest.mark.parametrize("pert", [{"name": "b+", "beta_deg": 3.0}, {"name": "b-", "beta_deg": -3.0},
                                  {"name": "r+", "r_radps": 0.1}, {"name": "r-", "r_radps": -0.1}])
def test_s03_perturbations_decay_with_lqr_and_diverge_open_loop(drift, pert):
    params, plant, out = drift
    eq, _, lqr = out["left"]
    _, k = _run(params, plant, eq, lqr, pert)
    assert not k["stability_loss"] and k["error_norm_final"] < 0.5 * k["error_norm_initial"]
    assert k["saturation_fraction"] < 0.05
    _, k_open = _run(params, plant, eq, lqr, pert, controller=_OpenLoop(eq))
    assert k_open["error_norm_final"] > k_open["error_norm_initial"]          # the equilibrium is unstable


def test_actuator_limits_act_on_the_command(drift):
    params, _, out = drift
    eq, _, lqr = out["left"]
    c = DriftLQRController(params, eq, lqr)
    far = SevenDOFState(vx=13.0, vy=-8.0, r=1.5, omega=eq.omega.copy())
    o1 = c.compute_control(far, None, None, 0.005)
    o2 = c.compute_control(far, None, None, 0.005)
    assert abs(o2.steer_f - o1.steer_f) <= params.max_steer_rate * 0.005 + 1e-12
    assert 0.0 <= o1.T_drive[2] <= params.max_drive_torque and o1.T_drive[0] == 0.0 and np.all(o1.T_brake == 0.0)
    c.reset()
    assert c.compute_control(far, None, None, 0.005).steer_f == pytest.approx(o1.steer_f)    # reset restores the start
