"""Constraint semantics and feasibility separate from the solver status."""

from __future__ import annotations

import math

import numpy as np
import pytest

from vehicle_model.controllers.constraints import (
    ADVISORY,
    HARD,
    PENALTY,
    PLANT,
    SOFT,
    Constraint,
    ConstraintSet,
    FeasibilityMonitor,
    stability_limits,
)
from vehicle_model.controllers.dynamic_two_track_nmpc import DynamicNMPCConfig, DynamicTwoTrackNMPC
from vehicle_model.controllers.framework import ControlTarget
from vehicle_model.controllers.motion_control import MotionContext
from vehicle_model.controllers.yaw_laws import INPUTS_4WS_TV, LTVMPCYawLaw
from vehicle_model.core.types import SevenDOFParams, SevenDOFState
from vehicle_model.handling_analysis import linear_params_from_seven_dof
from vehicle_model.vehicle_models import SevenDOFModel


def test_only_hard_excess_makes_a_plan_infeasible_and_every_excess_is_reported():
    cs = ConstraintSet([Constraint("u", HARD, 1.0), Constraint("s", SOFT, 1.0), Constraint("p", PENALTY, 0.9, lower=0.0),
                        Constraint("a", ADVISORY, 1.0), Constraint("f", PLANT, 1.0, lower=0.0)])
    ok = cs.check({"u": [0.5, -1.0], "s": 2.0, "p": 0.95, "a": -3.0, "f": 0.99})
    assert ok.feasible and ok.realisable
    assert set(ok.exceeded()) == {"s", "p", "a"}
    assert ok.max_excess(ADVISORY) == pytest.approx(2.0) and ok.max_excess(PENALTY) == pytest.approx(0.05)

    bad = cs.check({"u": [[0.0, 1.2]], "s": 0.0, "p": 0.0, "a": 0.0, "f": 1.3})
    assert not bad.feasible and not bad.realisable
    assert bad.exceeded(HARD) == ("u",) and bad.exceeded(PLANT) == ("f",)
    assert bad.checks[0].peak == pytest.approx(1.2) and bad.checks[0].excess == pytest.approx(0.2)
    assert not cs.check({"u": 0.0, "s": 0.0, "p": 0.0, "a": 0.0, "f": 1.01}).realisable   # plant only: feasible, not realisable
    assert not cs.check({"u": np.nan, "s": 0.0, "p": 0.0, "a": 0.0, "f": 0.0}).feasible  # non-finite plan is not feasible


def test_missing_unknown_or_ill_defined_values_are_errors_not_silent_passes():
    cs = ConstraintSet([Constraint("u", HARD, 1.0), Constraint("a", ADVISORY, 1.0)])
    with pytest.raises(KeyError):
        cs.check({"u": 0.0})
    with pytest.raises(KeyError):
        cs.check({"u": 0.0, "a": 0.0, "typo": 0.0})
    with pytest.raises(ValueError):
        Constraint("x", "strict", 1.0)
    with pytest.raises(ValueError):
        Constraint("x", HARD, 1.0, lower=2.0)
    with pytest.raises(ValueError):
        ConstraintSet([Constraint("x", HARD, 1.0), Constraint("x", SOFT, 1.0)])


def test_tolerance_and_monitor_bookkeeping():
    c = Constraint("rate", HARD, 1.0, rel_tol=1e-3)
    assert not c.check(1.0005).exceeded and c.check(1.002).exceeded
    mon = FeasibilityMonitor()
    cs = ConstraintSet([c, Constraint("f", PLANT, 1.0)])
    for v in (0.5, 1.5, 1.2):
        mon.record(cs.check({"rate": v, "f": 0.0}))
    s = mon.summary()
    assert (s["steps"], s["infeasible_steps"], s["unrealisable_steps"]) == (3, 2, 2)
    assert s["excess_counts"] == {"hard:rate": 2} and s["max_excess"]["hard:rate"] == pytest.approx(0.5)


def test_stability_limits_scale_with_friction_and_speed():
    beta1, r1 = stability_limits(1.0, 20.0)
    beta02, r02 = stability_limits(0.2, 20.0)
    assert math.degrees(beta1) == pytest.approx(11.1, abs=0.1) and beta02 < beta1
    assert r1 == pytest.approx(9.81 / 20.0) and r02 == pytest.approx(0.2 * r1)


def test_ltv_mpc_reports_plan_feasibility_next_to_the_solver_status():
    src = SevenDOFModel.from_yaml()
    law = LTVMPCYawLaw(linear_params_from_seven_dof(src.params, src.tyre_front, src.tyre_rear), INPUTS_4WS_TV)
    ctx = MotionContext(steer_f=0.05, r_ref=0.3, mu_hat=1.0, dt=0.005, max_yaw_moment=4000.0, max_rear_steer=0.05)
    req = law.request(SevenDOFState(vx=25.0), ctx)
    assert req.status == "nominal" and "solver" in req.diagnostics
    feas = req.diagnostics["feasibility"]
    assert feas["feasible"]                                              # BVLS bounds hold on the whole plan
    kinds = {c["name"]: c["kind"] for c in feas["checks"]}
    assert kinds == {"rear_steer": HARD, "yaw_moment": HARD, "predicted_sideslip": ADVISORY, "predicted_yaw_rate": ADVISORY}
    assert law.feasibility.summary()["steps"] == 1


def _nmpc(enforce: bool, mu: float = 1.0):
    p = SevenDOFParams.from_yaml()
    cfg = DynamicNMPCConfig(N=8, max_iter=10, enforce_steer_rate=enforce)
    return DynamicTwoTrackNMPC(p, cfg), ControlTarget(target_y=3.5, target_vx=25.0, mu_estimated=mu), SevenDOFState(vx=25.0)


def test_dynamic_nmpc_steering_rate_is_a_hard_limit_on_the_applied_command():
    """A 3.5 m lateral step demands fast steering; the first applied step is limited to rate * dt."""
    dt = 0.02
    for enforce in (True, False):
        nmpc, target, state = _nmpc(enforce)
        out = nmpc.compute_control(state, target, dt=dt)
        rate = abs(out.steer_f) / dt
        kinds = {c["name"]: c["kind"] for c in out.diagnostics["feasibility"]["checks"]}
        if enforce:
            assert rate <= nmpc.config.max_steer_rate_radps * (1 + nmpc.config.tolerance)
            assert kinds["steer_rate_front"] == HARD and out.diagnostics["feasibility"]["feasible"]
        else:  # former behaviour: penalised only, the configured limit is exceeded and reported as advisory
            assert rate > nmpc.config.max_steer_rate_radps
            assert kinds["steer_rate_front"] == ADVISORY
            assert "steer_rate_front" in out.diagnostics["feasibility"]["exceeded"][ADVISORY]


def test_dynamic_nmpc_flags_plans_that_need_more_friction_than_the_tyres_have():
    """The prediction does not cap F_x at mu F_z: a hard braking plan on ice is reported as not realisable."""
    nmpc, _, state = _nmpc(True, mu=0.1)
    U = np.tile([0.0, 0.0, 0.0, -nmpc.config.max_decel_mps2], nmpc.config.N)   # full braking on mu = 0.1
    x_pred, kamm = nmpc.predict(np.array([0, 0, 0, 25.0, 0, 0]), U, 0.1)
    assert np.max(kamm) > 1.0
    feas = nmpc.constraints(0.1, 25.0).check({
        "steer_front": 0.0, "steer_rear": 0.0, "yaw_moment": 0.0, "accel": U[3::4], "steer_rate_front": 0.0,
        "steer_rate_rear": 0.0, "friction_penalty": kamm, "friction_utilisation": kamm,
        "predicted_sideslip": 0.0, "predicted_yaw_rate": x_pred[:, 5]})
    assert feas.feasible and not feas.realisable and feas.exceeded(PLANT) == ("friction_utilisation",)
