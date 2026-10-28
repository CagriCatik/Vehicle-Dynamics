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
