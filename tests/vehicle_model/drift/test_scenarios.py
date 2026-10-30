"""End-to-end initiation from straight driving, hold and exit on the 7-DOF plant; branch schedule."""

from __future__ import annotations

import math

import numpy as np
import pytest

from vehicle_model.vehicle_models import SevenDOFModel


def test_branch_schedule_interpolates_between_equilibria():
    from vehicle_model.drift.schedule import BranchSchedule
    sch = BranchSchedule(SevenDOFModel.from_yaml(), 13.0, 0.9, [-10.0, -12.0])
    q0, u0, K0 = sch.point(math.radians(-10.0))
    q1, u1, _ = sch.point(math.radians(-12.0))
    qm, um, _ = sch.point(math.radians(-11.0))
    assert np.allclose(qm, 0.5 * (q0 + q1)) and np.allclose(um, 0.5 * (u0 + u1))
    assert np.allclose(sch.point(math.radians(-30.0))[0], q1)                     # clamped to the branch
    with pytest.raises(ValueError):
        BranchSchedule(SevenDOFModel.from_yaml(), 13.0, 0.9, [-10.0, 10.0])


def test_approach_equilibrium_matches_the_requested_lateral_acceleration():
    from vehicle_model.drift.equilibrium import approach_equilibrium
    m = SevenDOFModel.from_yaml()
    for mu, d in ((0.9, "left"), (0.6, "right")):
        e = approach_equilibrium(m, 13.0, mu, 0.6, d)
        assert abs(e.yaw_rate) * e.speed == pytest.approx(0.6 * mu * 9.81, rel=0.02)
        assert np.sign(e.yaw_rate) == (1 if d == "left" else -1)


@pytest.mark.slow
def test_s04_s05_initiation_hold_exit_from_straight_driving():
    from vehicle_model.simulation.drift.config import load_yaml
    from vehicle_model.simulation.drift.scenarios import parse, run_case
    d = load_yaml("configs/drift/initiation.yaml")
    d["supervisor"]["exit_s"] = 22.0
    d["simulation"]["duration_s"] = 30.0
    spec = parse(d, "test")
    r = run_case((spec, {"mu": 0.9, "direction": "left", "speed": 13.0, "beta_deg": 20.0}, True))
    h = r["history"]
    assert h.vx[0] == pytest.approx(13.0) and h.vy[0] == 0.0 and h.r[0] == 0.0      # straight start, no teleport
    assert r["success"] and r["final_mode"] == "DISABLED"
    assert [t["to"] for t in r["transitions"]] == ["APPROACH", "INITIATE", "HOLD", "EXIT", "DISABLED"]
    assert r["hold_beta_rmse_deg"] < 3.0 and r["hold_yaw_rmse_degps"] < 5.0
    assert r["exit_peak_beta_deg"] < r["shutdown"]["peak_beta_deg"]                  # supervised exit beats shutdown
    assert r["exit_min_speed_mps"] > 10.0
