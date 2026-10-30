"""Road grade in the 7-DOF plant and Hill Descent Control."""

from __future__ import annotations

import math

import numpy as np
import pytest

from vehicle_model.controllers.hill_descent import HDCConfig, HillDescentControl
from vehicle_model.core.types import SevenDOFControlInput, SevenDOFState
from vehicle_model.roads.surface import GradedRoad, MuSplitRoad, UniformRoad
from vehicle_model.vehicle_models import SevenDOFModel

VEH = "configs/vehicles/seven_dof_passenger.yaml"


def test_graded_road_is_continuous_and_height_integrates_the_slope():
    road = GradedRoad(UniformRoad(0.7), ((10.0, -0.10), (50.0, 0.05)), 8.0)
    assert road.grade(0.0) == 0.0 and road.height(5.0) == 0.0 and road.get_friction(3.0, 1.0) == 0.7
    assert road.grade(30.0) == pytest.approx(-0.10) and road.grade(100.0) == pytest.approx(0.05)
    x = np.linspace(0.0, 120.0, 24001)
    g = np.array([road.grade(v) for v in x])
    assert np.max(np.abs(np.diff(g))) < 1e-3                                   # no slope jump
    z_num = np.concatenate([[0.0], np.cumsum(0.5 * (g[1:] + g[:-1]) * np.diff(x))])
    assert np.array([road.height(v) for v in x[::500]]) == pytest.approx(z_num[::500], abs=1e-6)
    with pytest.raises(ValueError):
        GradedRoad(UniformRoad(), ((20.0, -0.1), (10.0, 0.0)))


def test_level_grade_is_bit_identical_to_the_base_road():
    for base in (UniformRoad(0.8), MuSplitRoad(mu_left=1.0, mu_right=0.2, split_y=0.0, transition_width=0.05)):
        m = SevenDOFModel.from_yaml(VEH)
        s = SevenDOFState(vx=15.0, vy=0.3, r=0.1, omega=np.full(4, 15.0 / m.params.Reff) * 1.02)
        u = SevenDOFControlInput(steer_f=0.03, T_brake=np.full(4, 200.0))
        d0, t0, *_ = SevenDOFModel.from_yaml(VEH).evaluate_dynamics(s, u, base)
        d1, t1, *_ = SevenDOFModel.from_yaml(VEH).evaluate_dynamics(s, u, GradedRoad(base, ((0.0, 0.0),), 5.0))
        assert np.array_equal(d0, d1) and np.array_equal(t0.Fz, t1.Fz)


@pytest.mark.parametrize("grade", [-0.10, -0.20, 0.10])
def test_gravity_reproduces_the_analytic_low_speed_case(grade):
    """Free rolling at low speed: a = (m g sin - c_rr m g cos - drag) / (m + 4 I_w / R^2), loads sum to m g cos."""
    road = GradedRoad(UniformRoad(1.0), ((-1e6, grade),), 1.0)
    m = SevenDOFModel.from_yaml(VEH, road=road)
    p = m.params
    v0 = 4.0
    s = SevenDOFState(vx=v0, omega=np.full(4, v0 / p.Reff))
    th = -math.atan(grade)                                         # downhill angle (> 0 when going down)
    T, dt = 1.0, 0.005
    for _ in range(int(T / dt)):
        s, tel = m.step(s, SevenDOFControlInput(), dt)
    v_mid = 0.5 * (v0 + s.vx)
    drag = 0.5 * p.rho * p.Cd * p.Af * v_mid ** 2
    a_ana = (p.mass * p.g * math.sin(th) - p.c_rr * p.mass * p.g * math.cos(th) - drag) / (p.mass + 4 * p.Iw / p.Reff ** 2)
    assert (s.vx - v0) / T == pytest.approx(a_ana, rel=0.03)
    assert float(np.sum(tel.Fz)) == pytest.approx(p.mass * p.g * math.cos(th), rel=1e-3)
    # free rolling: gravity and inertia both act at the CoG, the tyres carry ~no force -> static axle split
    assert (tel.Fz[0] + tel.Fz[1]) / np.sum(tel.Fz) == pytest.approx(p.lr / p.wheelbase, abs=0.01)


def test_hdc_disabled_and_override_request_nothing():
    off = HillDescentControl(HDCConfig(enabled=False))
    assert off.request(8.0, 0.0, 0.0, False, 0.01) == 0.0 and off.status == "off"
    h = HillDescentControl()
    assert h.request(5.0, 0.0, 0.0, False, 0.01) < 0.0 and h.status == "active"
    assert h.request(5.0, 0.3, 0.0, False, 0.01) == 0.0 and h.status == "override" and h.integral == 0.0
    fast = HillDescentControl()
    assert fast.request(20.0, 0.0, 0.0, False, 0.01) == 0.0 and fast.status == "standby"   # above engagement speed


def test_hdc_pi_bounds_antiwindup_and_infeasible():
    c = HDCConfig(max_decel=3.0)
    h = HillDescentControl(c)
    reqs = [h.request(3.0 + 0.5, 0.0, 0.0, False, 0.01) for _ in range(500)]
    assert all(-3.0 - 1e-12 <= a <= 0.0 for a in reqs) and -3.0 <= h.integral <= 0.0
    slow = HillDescentControl(c)
    assert all(slow.request(1.0, 0.0, 0.0, False, 0.01) == 0.0 for _ in range(50))         # below set speed: no drive
    sat = HillDescentControl(c)
    sat.request(c.v_set + 0.5, 0.0, 0.0, False, 0.01)
    i0 = sat.integral
    for _ in range(100):
        sat.request(c.v_set + 0.5, 0.0, 0.0, True, 0.01)                                     # ABS active: hold
    assert sat.integral == i0
    inf = HillDescentControl(c)
    for _ in range(int(c.infeasible_time / 0.01) + 5):
        inf.request(c.v_set + 3.0, 0.0, 0.0, True, 0.01)
    assert inf.status == "infeasible" and inf.cause.startswith("grip")
    yld = HillDescentControl(c)
    assert yld.request(c.v_set + 0.5, 0.0, -5.0, False, 0.01) == 0.0 and yld.status == "driver_brake"


def _run(grade_pct, mu, hdc, duration=16.0, **kw):
    from vehicle_model.experiments.hill_descent import _task, parse
    spec = parse({"type": "hill_descent_benchmark", "name": "t", "vehicle": VEH, "workers": 1,
                  "scenarios": [{"name": "s", "profile": [[15, grade_pct]], "surface": {"mu": mu}, "duration": duration, **kw}],
                  "controllers": [{"name": "c", "hdc": hdc}]})
    return _task((spec, spec.scenarios[0], spec.controllers[0]))


@pytest.mark.slow
def test_hdc_holds_the_set_speed_and_reports_infeasible_grip():
    on = _run(-10, 1.0, {"v_set_kph": 10}, duration=22.0)
    off = _run(-10, 1.0, {"enabled": False})
    assert not on["error"] and on["kpis"]["speed_rms_error_kph"] < 1.0 and on["kpis"]["v_max_kph"] < 13.0
    assert off["kpis"]["v_end_kph"] > 20.0 and off["kpis"]["status_counts"] == {"off": len(off["history"]["t"])}
    assert on["kpis"]["infeasible_s"] == 0.0
    # holding on the slope: the brake forces (at the ground) move m g sin(th) h / L to the front axle
    from vehicle_model.vehicle_models import SevenDOFModel
    p = SevenDOFModel.from_yaml(VEH).params
    h = on["history"]
    late = h["t"] > 18.0                                            # settled on the slope (reached at ~9 s)
    th = math.atan(0.10)
    fz_front = np.mean(h["Fz_FL"][late] + h["Fz_FR"][late])
    fx_total = -np.mean(h["ax"][late]) * p.mass                     # brake force sum (tyre forces)
    assert fz_front == pytest.approx(p.mass * p.g * math.cos(th) * p.lr / p.wheelbase + fx_total * p.hcog / p.wheelbase, rel=0.01)
    assert fx_total == pytest.approx(p.mass * p.g * (math.sin(th) - p.c_rr * math.cos(th)), rel=0.05)   # force balance
    ice = _run(-20, 0.12, {"v_set_kph": 10})                     # mu 0.12 < tan(11.3 deg) = 0.2: cannot hold
    assert ice["kpis"]["infeasible_s"] > 1.0 and ice["kpis"]["v_end_kph"] > 20.0


def test_hill_descent_spec_is_strict():
    from vehicle_model.experiments.hill_descent import parse
    base = {"type": "hill_descent_benchmark", "name": "x", "vehicle": VEH, "matrix": {"grades_percent": [5], "surfaces": {"dry": 1.0}},
            "controllers": [{"name": "c", "hdc": {}}]}
    assert [s["name"] for s in parse(base).scenarios] == ["dry_5pct"]
    bad = [({"controllers": [{"name": "c", "hdc": {"gain": 1}}]}, "unknown key"),
           ({"controllers": [{"name": "c", "hdc": {"v_set_kph": 100}}]}, "v_set_kph"),
           ({"controllers": [{"name": "c", "hdc": {}, "brake_failed": ["XX"]}]}, "brake_failed"),
           ({"scenarios": [{"name": "s", "profile": [[20, -5], [10, -5]], "surface": {"mu": 1}}]}, "sorted"),
           ({"scenarios": [{"name": "s", "profile": [[20, -5]], "surface": {"mu": 1, "mu_left": 1}}]}, "surface"),
           ({"scenarios": [{"name": "s", "profile": [[20, -5]], "surface": {"mu": 1}, "driver": [{"t0": 5, "t1": 2}]}]}, "t0 < t1")]
    for patch, msg in bad:
        with pytest.raises(ValueError, match=msg):
            parse({**base, **patch})
