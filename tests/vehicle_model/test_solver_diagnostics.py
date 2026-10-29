"""M6: solver diagnostics and degraded mode of the optimisation-based controllers."""

from __future__ import annotations

import types

import numpy as np
import pytest

import vehicle_model.controllers.dynamic_two_track_nmpc as dyn
from vehicle_model.controllers.dynamic_two_track_nmpc import DynamicNMPCConfig, DynamicTwoTrackNMPC
from vehicle_model.controllers.framework import ControlTarget
from vehicle_model.controllers.solver_diagnostics import (
    DEGRADED,
    FALLBACK,
    NOMINAL,
    SUBOPTIMAL,
    SolverMonitor,
)
from vehicle_model.core.types import SevenDOFState
from vehicle_model.vehicle_models.normal_loads import wheel_contact_positions


def test_classification():
    m = SolverMonitor(max_consecutive_failures=3, time_budget_ms=5.0)
    assert m.classify(True, np.zeros(2), 1.0, 2.0).status == NOMINAL
    r = m.classify(False, np.zeros(2), 1.0, 2.0, solve_time_ms=9.0)
    assert r.status == SUBOPTIMAL and r.accepted and r.over_budget
    r = m.classify(False, np.zeros(2), 3.0, 2.0)
    assert r.status == FALLBACK and not r.accepted
    assert m.classify(True, np.array([np.nan]), 1.0, 2.0).status == DEGRADED  # third failure in a row
    assert m.classify(True, np.zeros(2), 1.0, 2.0).status == NOMINAL          # recovers
    assert m.counts == {NOMINAL: 2, SUBOPTIMAL: 1, FALLBACK: 1, DEGRADED: 1}
    with pytest.raises(ValueError):
        SolverMonitor(max_consecutive_failures=0)


def _failing_minimize(*args, **kwargs):
    x0 = kwargs.get("x0", args[1] if len(args) > 1 else None)
    return types.SimpleNamespace(success=False, x=np.full_like(x0, np.nan), message="injected failure", nit=0, fun=np.nan)


def test_dynamic_nmpc_reports_fallback_then_degraded_and_keeps_the_warm_start(monkeypatch):
    ctrl = DynamicTwoTrackNMPC(config=DynamicNMPCConfig(N=4, max_consecutive_failures=3))
    state = SevenDOFState(vx=15.0, Y=0.5, omega=np.full(4, 15.0 / ctrl.params.Reff))
    target = ControlTarget(target_vx=15.0, target_y=0.0)
    first = ctrl.compute_control(state, target)          # real solve
    assert first.status in (NOMINAL, SUBOPTIMAL)
    plan = ctrl.u_prev.copy()
    monkeypatch.setattr(dyn, "minimize", _failing_minimize)
    statuses = [ctrl.compute_control(state, target).status for _ in range(3)]
    assert statuses == [FALLBACK, FALLBACK, DEGRADED]
    shifted = np.concatenate([plan[4:], plan[-4:]])
    shifted2 = np.concatenate([shifted[4:], shifted[-4:]])
    shifted3 = np.concatenate([shifted2[4:], shifted2[-4:]])
    assert np.allclose(ctrl.u_prev, shifted3)              # fell back to the shifted plan each time
    assert ctrl.monitor.counts[DEGRADED] == 1


def test_dynamic_nmpc_reports_the_yaw_moment_actually_produced():
    ctrl = DynamicTwoTrackNMPC(config=DynamicNMPCConfig(N=4))
    state = SevenDOFState(vx=15.0, r=0.2, omega=np.full(4, 15.0 / ctrl.params.Reff))
    out = ctrl.compute_control(state, ControlTarget(target_vx=15.0))
    p = ctrl.params
    pos = wheel_contact_positions(p)
    steer = np.array([out.steer_f, out.steer_f, out.steer_r, out.steer_r])
    F = (out.T_drive - out.T_brake) / p.Reff
    assert out.allocated_yaw_moment == pytest.approx(np.sum((pos[:, 0] * np.sin(steer) - pos[:, 1] * np.cos(steer)) * F))
    assert out.diagnostics["solver"]["status"] == out.status


def test_dynamic_nmpc_acts_through_the_shared_actuation_layer():
    """Plan V3.0 M10 5.3: demand, achievable, issued and realised are reported separately."""
    from vehicle_model.allocation import ActuatorHealth
    from vehicle_model.vehicle_models import SevenDOFModel
    from vehicle_model.core.types import SevenDOFControlInput

    model = SevenDOFModel.from_yaml()
    ctrl = DynamicTwoTrackNMPC(model.params, DynamicNMPCConfig(N=4))
    state = SevenDOFState(vx=20.0, omega=np.full(4, 20.0 / model.params.Reff))
    target = ControlTarget(target_vx=12.0)                                  # demands braking
    out = ctrl.compute_control(state, target, dt=0.02)
    a = out.diagnostics["actuation"]
    assert set(a) >= {"demand", "allocation_achievable", "issued", "realised_previous_step", "allocation_status"}
    assert a["demand"]["Fx"] < 0 and a["realised_previous_step"] is None   # no telemetry yet
    steer = np.array([out.steer_f, out.steer_f, out.steer_r, out.steer_r])
    F = (out.T_drive - out.T_brake) / model.params.Reff
    assert a["issued"]["Fx"] == pytest.approx(np.sum(np.cos(steer) * F))     # body frame, like the demand
    assert out.allocated_yaw_moment == pytest.approx(a["issued"]["Mz"])
    assert out.diagnostics["information"] == ctrl.information

    _, tel = model.step(state, SevenDOFControlInput(steer_f=out.steer_f, steer_r=out.steer_r,
                                                    T_drive=out.T_drive, T_brake=out.T_brake), 0.02)
    a2 = ctrl.compute_control(state, target, tel, dt=0.02).diagnostics["actuation"]
    assert a2["realised_previous_step"]["Fx"] < 0                          # the plant braked

    # failed brakes on the left: the allocation reports it, no torque at the failed wheels
    failed = DynamicTwoTrackNMPC(model.params, DynamicNMPCConfig(N=4),
                                 health=ActuatorHealth(brake_ok=(False, True, False, True)))
    out_f = failed.compute_control(state, target, dt=0.02)
    assert out_f.T_brake[0] == 0.0 and out_f.T_brake[2] == 0.0
    assert out_f.diagnostics["actuation"]["allocation_status"] in ("saturated", "infeasible")


def test_kinematic_path_nmpc_acts_through_the_shared_actuation_layer():
    from vehicle_model.controllers.nmpc import NMPCPathTrackingController

    ctrl = NMPCPathTrackingController()
    out = ctrl.compute_control(SevenDOFState(vx=2.0), ControlTarget(target_vx=2.5), dt=0.05)
    a = out.diagnostics["actuation"]
    assert out.diagnostics["feasibility"]["feasible"]                      # SLSQP bounds hold on the plan
    assert a["allocation_status"] == "nominal" and a["demand"]["Mz"] == 0.0
    assert a["issued"]["Fx"] == pytest.approx(a["demand"]["Fx"], rel=1e-3)  # accelerating, within all limits
    assert abs(out.allocated_yaw_moment) < 1.0
    assert out.status == NOMINAL or out.status == out.diagnostics["solver"]["status"]


# ----------------------------------------------------------------------------- solution quality (V3.0 M10 5.2)
def test_kkt_residuals_on_known_problems():
    from vehicle_model.controllers.solver_diagnostics import kkt_residuals

    # min (x-2)^2 s.t. x <= 1: optimum at the bound, multiplier 2
    k = kkt_residuals([-2.0], [1.0], upper=1.0)
    assert k.stationarity < 1e-12 and k.primal == 0.0 and k.n_active == 1
    k = kkt_residuals([-3.0], [0.5], upper=1.0, cost=6.0)                             # interior, not stationary
    assert (k.stationarity, k.stationarity_relative) == (3.0, 0.5)
    assert kkt_residuals([2.0], [1.0], upper=1.0).stationarity == pytest.approx(2.0)    # wrong-sign multiplier
    # min -(x1 + x2) s.t. 1 - x1 - x2 >= 0
    J = np.array([[-1.0, -1.0]])
    assert kkt_residuals([-1.0, -1.0], [0.5, 0.5], ineq_value=[0.0], ineq_jacobian=J).stationarity < 1e-12
    k = kkt_residuals([-1.0, -1.0], [1.0, 1.0], ineq_value=[-1.0], ineq_jacobian=J)
    assert k.primal == pytest.approx(1.0)
    assert np.isnan(kkt_residuals(None, [1.0]).stationarity)
    assert np.isnan(kkt_residuals([1.0], [np.nan]).stationarity)


def test_monitor_reports_cost_improvement_warm_start_step_and_a_summary():
    from vehicle_model.controllers.solver_diagnostics import KKTResiduals

    m = SolverMonitor(max_consecutive_failures=5, time_budget_ms=5.0)
    r = m.classify(True, np.array([1.0, 0.0]), 1.0, 4.0, x_warm_start=np.zeros(2),
                   kkt=KKTResiduals(1e-8, 0.0, 1, 1e-8))
    assert (r.cost_improvement, r.relative_cost_improvement, r.warm_start_step) == (3.0, 0.75, 1.0)
    assert r.kkt_stationarity == 1e-8
    m.classify(False, np.zeros(2), 5.0, 4.0, solve_time_ms=10.0)                      # fallback, over budget
    s = m.summary()
    assert s["solves"] == 2 and s["fraction_nominal"] == 0.5 and s["fraction_fallback"] == 0.5
    assert s["over_budget"] == 1 and s["kkt_stationarity_relative_max"] == 1e-8 and s["solve_ms_max"] == 10.0
    assert s["relative_cost_improvement_median"] == pytest.approx((0.75 - 0.25) / 2)
    m.reset()
    assert m.summary() == {"solves": 0}


def test_dynamic_nmpc_reports_kkt_and_warm_start_quality():
    ctrl = DynamicTwoTrackNMPC(config=DynamicNMPCConfig(N=4))
    state = SevenDOFState(vx=15.0, Y=0.5, omega=np.full(4, 15.0 / ctrl.params.Reff))
    out = ctrl.compute_control(state, ControlTarget(target_vx=15.0, target_y=0.0))
    rep = out.diagnostics["solver"]
    assert np.isfinite(rep["kkt_stationarity_relative"]) and rep["kkt_primal"] < 1e-6
    assert rep["cost_improvement"] >= 0.0 and rep["warm_start_step"] > 0.0
    assert out.diagnostics["solver_summary"]["solves"] == 1
