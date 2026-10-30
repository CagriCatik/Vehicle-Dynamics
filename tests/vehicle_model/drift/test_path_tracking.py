"""Path references, circle fit, set-point transition and the radial outer loop."""

from __future__ import annotations

import math

import numpy as np
import pytest

from vehicle_model.drift.path_reference import CircleReference, FigureEightReference, chi
from vehicle_model.simulation.drift.tracking import fit_circle


def test_circle_through_state_is_tangent_and_offset_puts_the_start_outside():
    c = CircleReference.through_state(0.0, 0.0, math.radians(-20.0), 25.0, "left", offset=2.0)
    assert c.radial_error(0.0, 0.0) == pytest.approx(2.0)
    d = np.array([c.cx, c.cy])
    v = np.array([math.cos(math.radians(-20.0)), math.sin(math.radians(-20.0))])
    assert abs(d @ v) < 1e-9                                      # centre perpendicular to the direction of travel
    assert v[0] * d[1] - v[1] * d[0] > 0.0 and c.curvature == pytest.approx(1 / 25.0)   # left: centre on the left
    r = CircleReference.through_state(0.0, 0.0, 0.0, 25.0, "right")
    assert r.cy == pytest.approx(-25.0) and r.curvature < 0.0


def test_figure_eight_circles_touch_at_the_start():
    f = FigureEightReference.through_state(0.0, 0.0, 0.0, 20.0)
    assert f.left.radial_error(0.0, 0.0) == pytest.approx(0.0) and f.right.radial_error(0.0, 0.0) == pytest.approx(0.0)
    assert math.hypot(f.left.cx - f.right.cx, f.left.cy - f.right.cy) == pytest.approx(40.0)


def test_direction_of_travel_includes_sideslip():
    assert chi(0.5, 10.0, -10.0) == pytest.approx(0.5 - math.pi / 4)


def test_circle_fit_recovers_centre_and_radius():
    a = np.linspace(0.3, 2.5, 200)
    cx, cy, R = fit_circle(3.0 + 17.0 * np.cos(a), -4.0 + 17.0 * np.sin(a))
    assert (cx, cy, R) == pytest.approx((3.0, -4.0, 17.0), abs=1e-9)


@pytest.fixture(scope="module")
def schedule():
    from vehicle_model.drift.schedule import BranchSchedule
    from vehicle_model.vehicle_models import SevenDOFModel
    return BranchSchedule(SevenDOFModel.from_yaml(), 13.0, 0.9, [-14.0, -16.0, -18.0, -20.0, -22.0])


def test_radius_grows_with_sideslip_along_the_branch(schedule):
    from vehicle_model.drift.schedule import radius_to_beta
    R = [e.radius for e in schedule.equilibria]
    assert all(np.diff(R) > 0.0)
    assert radius_to_beta(schedule, R[2]) == pytest.approx(schedule.betas[2])


def test_set_point_ramps_at_the_configured_rate(schedule):
    from vehicle_model.core.types import SevenDOFState
    from vehicle_model.drift.control_adapter import ScheduledDriftController
    e = schedule.equilibria[1]
    c = ScheduledDriftController(schedule.model.params, schedule, math.radians(-22.0), beta_rate=math.radians(2.0))
    s = SevenDOFState(vx=e.vx, vy=e.vy, r=e.yaw_rate, omega=e.omega.copy())
    outs = [c.compute_control(s, None, None, 0.01) for _ in range(50)]
    assert c.beta_ref == pytest.approx(e.beta - math.radians(2.0) * 0.5, abs=1e-9)   # 0.5 s at 2 deg/s
    assert outs[0].diagnostics["mode"] == "TRANSITION"


@pytest.mark.slow
def test_outer_loop_removes_a_radial_offset_that_the_plain_lqr_keeps(schedule):
    from vehicle_model.core.types import SevenDOFState
    from vehicle_model.drift.control_adapter import ScheduledDriftController
    from vehicle_model.roads.surface import UniformRoad
    from vehicle_model.simulation.drift.hold import simulate
    from vehicle_model.vehicle_models import SevenDOFModel
    plant = SevenDOFModel.from_yaml(road=UniformRoad(mu=0.9))
    e = schedule.equilibria[3]
    circ = CircleReference.through_state(0.0, 0.0, e.beta, e.radius, "left", offset=2.0)
    x0 = SevenDOFState(vx=e.vx, vy=e.vy, r=e.yaw_rate, omega=e.omega.copy())
    res = {}
    for name, loop in (("plain", None), ("outer", circ)):
        c = ScheduledDriftController(schedule.model.params, schedule, e.beta, circle=loop, k_p=math.radians(1.0),
                                     k_d=math.radians(0.5))
        h, _ = simulate(plant, c, x0, 20.0, 0.005)
        err = circ.radial_error(h.X, h.Y)
        res[name] = float(np.sqrt(np.mean(err[-1000:] ** 2)))
    assert res["outer"] < 0.3 and res["plain"] > 1.0
