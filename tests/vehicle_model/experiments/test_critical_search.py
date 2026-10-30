"""Critical scenario search (spec, constraints, search, verification, minimisation, replay)."""

from __future__ import annotations

import copy
import math

import numpy as np
import pytest

from vehicle_model.experiments import critical_search as cs
from vehicle_model.experiments.types import get_type

BASE = {
    "type": "critical_scenario_search", "name": "t", "vehicle": "configs/vehicles/seven_dof_passenger.yaml",
    "manoeuvre": "sine_with_dwell", "controller": {"name": "GS-LQR", "law": "gs_lqr", "inputs": "4ws_tv"},
    "parameters": [{"name": "amplitude_deg", "low": 3, "high": 10, "nominal": 5},
                   {"name": "mu", "low": 0.3, "high": 1.0, "nominal": 1.0}],
    "objective": {"kpi": "max_sideslip_deg"}, "criterion": {"kpi": "max_sideslip_deg", "threshold": 15.0},
    "search": [{"method": "lhs", "samples": 12}, {"method": "differential_evolution", "maxiter": 3, "popsize": 5}],
    "minimise": ["amplitude_deg", "mu"], "seed": 3, "workers": 1,
}


def spec(**over):
    d = copy.deepcopy(BASE)
    d.update(over)
    return cs.parse(d, "<test>")


def test_registered_and_described():
    assert get_type("critical_scenario_search").parse is cs.parse
    assert "lhs -> differential_evolution" in cs.describe(spec())


@pytest.mark.parametrize("bad, msg", [
    ({"parameters": [{"name": "warp_factor", "low": 0, "high": 1, "nominal": 0}]}, "not a parameter"),
    ({"parameters": [{"name": "mu", "low": 1.0, "high": 0.3, "nominal": 0.5}]}, "low < high"),
    ({"parameters": [{"name": "steer_deg", "low": 1, "high": 3, "nominal": 2}]}, "not a parameter of sine_with_dwell"),
    ({"objective": {"kpi": "fun"}}, "objective.kpi"),
    ({"criterion": {"kpi": "max_sideslip_deg", "threshold": 1, "direction": "sideways"}}, "direction"),
    ({"search": [{"method": "simulated_annealing"}]}, "method"),
    ({"search": [{"method": "lhs", "samples": 0}]}, "samples"),
    ({"minimise": ["speed_kph"]}, "not a searched parameter"),
    ({"fixed": {"actuator_failure": "everything"}}, "actuator_failure"),
    ({"constraints": [{"type": "linear", "coefficients": {"nope": 1}, "max": 1}]}, "unknown parameter"),
    ({"replay": {"values": {"amplitude_deg": 5, "mu": 1.0}}}, "exactly one of search"),
])
def test_spec_is_strict(bad, msg):
    with pytest.raises(ValueError, match=msg):
        spec(**bad)


def test_constraints_reject_unphysical_combinations_without_simulating():
    s = spec(constraints=[{"type": "lateral_demand", "max_factor": 2.0},
                          {"type": "linear", "coefficients": {"amplitude_deg": 1.0, "mu": -10.0}, "max": 0.0,
                           "reason": "no large steer on ice"}])
    assert cs.constraint_violations(s, {"amplitude_deg": 3.0, "mu": 1.0}) == []
    v = cs.constraint_violations(s, {"amplitude_deg": 10.0, "mu": 0.3})
    assert len(v) == 2 and "no large steer on ice" in v
    r = cs.evaluate(s, {"amplitude_deg": 10.0, "mu": 0.3})
    assert r["class"] == "rejected" and not r["kpis"]


def test_scenarios_and_road_zone():
    from vehicle_model.roads.surface import FrictionPatchRoad
    s = spec(manoeuvre="brake_in_turn",
             parameters=[{"name": "zone_length_m", "low": 0, "high": 40, "nominal": 0},
                         {"name": "brake_pedal", "low": 0.1, "high": 1.0, "nominal": 0.5}],
             fixed={"zone_start_m": 20.0, "zone_mu": 0.2, "steer_deg": 2.0, "steer_rate_degps": 10.0},
             minimise=[])
    sc = cs.scenario(s, {"zone_length_m": 15.0, "brake_pedal": 0.8})
    road = sc.road()
    assert isinstance(road, FrictionPatchRoad)
    assert road.get_friction(25.0, 3.0) == pytest.approx(0.2) and road.get_friction(40.0, 0.0) == pytest.approx(1.0)
    from vehicle_model.core.types import SevenDOFState
    st = SevenDOFState(vx=20.0)
    assert sc.driver(0.6, st).driver_steer_f == pytest.approx(math.radians(1.0))       # steer gradient 10 deg/s
    assert sc.driver(10.0, st).driver_brake_pedal == pytest.approx(0.8) and sc.stop_speed == 1.0


def _analytic(monkeypatch, boom=False):
    """Replace the simulation: max sideslip = 4 * amplitude * (1.2 - mu); amplitude > 9.5 raises (numerics)."""
    calls = []

    def fake(spec_, values, controller=None, dt=None, load_transfer_tol=None, keep_history=False):
        calls.append((dict(values), dt, load_transfer_tol))
        if cs.constraint_violations(spec_, values):
            return {"values": values, "class": "rejected", "objective": float("nan"), "kpis": {}, "error": ""}
        a, mu = values["amplitude_deg"], values["mu"]
        if boom and a > 9.5:
            return {"values": values, "class": cs.NUMERICAL, "objective": float("nan"), "kpis": {}, "error": "boom"}
        x = 4.0 * a * (1.2 - mu) * (1.0 + (0.3 if dt and dt < spec_.dt and a > 9.0 else 0.0))
        k = {"max_sideslip_deg": x}
        return {"values": dict(values), "class": "critical" if cs.is_critical(spec_, k) else "nominal", "objective": x,
                "kpis": k, "error": ""}
    monkeypatch.setattr(cs, "evaluate", fake)
    return calls


def test_search_is_reproducible_and_never_counts_numerical_failures(monkeypatch):
    _analytic(monkeypatch, boom=True)
    s = spec()
    a, b = cs.search(s), cs.search(s)
    assert [e["values"] for e in a] == [e["values"] for e in b]
    assert any(e["stage"].endswith("differential_evolution") for e in a)
    num = [e for e in a if e["class"] == cs.NUMERICAL]
    crit = [e for e in a if e["class"] == "critical"]
    assert crit and all(e["values"]["amplitude_deg"] <= 9.5 for e in crit)
    assert all(not np.isfinite(e["objective"]) for e in num)
    best = max(e["objective"] for e in a if np.isfinite(e["objective"]))
    assert best > max(e["objective"] for e in a[:12] if np.isfinite(e["objective"])) - 1e-9   # DE does not lose ground


def test_verification_separates_numerical_artefacts(monkeypatch):
    _analytic(monkeypatch)
    s = spec(verification_tolerance=0.1)
    ok = cs.verify(s, {"amplitude_deg": 8.0, "mu": 0.4})
    assert ok["verdict"] == "confirmed" and ok["reproducible"]
    art = cs.verify(s, {"amplitude_deg": 9.2, "mu": 0.4})            # 30 % change at half step
    assert art["verdict"] == "numerical_artefact"
    assert cs.verify(s, {"amplitude_deg": 3.0, "mu": 1.0})["verdict"] == "not_critical"


def test_minimisation_finds_the_boundary_closest_to_nominal(monkeypatch):
    _analytic(monkeypatch)
    s = spec()
    minimal, trace = cs.minimise(s, {"amplitude_deg": 9.0, "mu": 0.4}, iterations=10)
    # amplitude first: critical down to 15 / (4 * 0.8) = 4.69 > nominal 5? no: 5 is still critical -> nominal 5
    assert minimal["amplitude_deg"] == 5.0
    # then mu towards 1.0: critical while 4 * 5 * (1.2 - mu) >= 15, i.e. mu <= 0.45
    assert minimal["mu"] == pytest.approx(0.45, abs=0.6 / 2**10 + 1e-12)
    assert cs.is_critical(s, {"max_sideslip_deg": 4 * 5 * (1.2 - minimal["mu"])})
    assert trace[0] == {"parameter": "amplitude_deg", "value": 5.0, "class": "critical"}


def test_minimisation_stops_before_step_dependent_points(monkeypatch):
    """Above amplitude 9 the fake KPI changes by 30 % at half the step: such points are not accepted."""
    _analytic(monkeypatch)
    s = spec(minimise=["mu"], verification_tolerance=0.1)
    assert cs.robustly_critical(s, {"amplitude_deg": 8.0, "mu": 0.4})
    assert not cs.robustly_critical(s, {"amplitude_deg": 9.5, "mu": 0.4})
    minimal, trace = cs.minimise(s, {"amplitude_deg": 9.5, "mu": 0.4})
    assert minimal == {"amplitude_deg": 9.5, "mu": 0.4}          # nothing robust to move to: unchanged
    assert all(t["class"] == "not_robustly_critical" for t in trace)


def test_actuator_failure_wrappers():
    from vehicle_model.controllers.framework import ControlOutput

    class Inner:
        name = "x"

        def reset(self):
            pass

        def compute_control(self, *a, **k):
            return ControlOutput(steer_f=0.1, steer_r=0.05, T_drive=np.zeros(4), T_brake=np.array([100.0, 300, 0, 400]))
    o = cs._ActuatorFailure(Inner(), "rear_steer_stuck").compute_control(None, None)
    assert o.steer_r == 0.0
    o = cs._ActuatorFailure(Inner(), "brake_vectoring_off").compute_control(None, None)
    assert np.allclose(o.T_brake, [200, 200, 200, 200]) and o.steer_r == 0.05


def test_a_crashing_run_is_numerical_not_critical(monkeypatch):
    import vehicle_model.simulation.controller_benchmark as cb
    monkeypatch.setattr(cb, "run_scenario", lambda *a, **k: (_ for _ in ()).throw(FloatingPointError("solver diverged")))
    r = cs.evaluate(spec(), {"amplitude_deg": 5.0, "mu": 1.0})
    assert r["class"] == cs.NUMERICAL and "solver diverged" in r["error"] and not math.isfinite(r["objective"])


# ----------------------------------------------------------------------------- regression cases
CRITICAL = sorted(__import__("pathlib").Path("configs/critical").glob("*.yaml"))


def test_regression_cases_exist():
    assert CRITICAL, "at least one automatically found critical case is kept as a regression test"


@pytest.mark.parametrize("path", CRITICAL, ids=lambda p: p.stem)
def test_regression_case_is_still_critical(path):
    import yaml
    s = cs.parse(yaml.safe_load(path.read_text()), str(path))
    r = cs.evaluate(s, s.replay["values"])
    assert r["class"] == "critical", r["kpis"].get(s.criterion["kpi"])


@pytest.mark.slow
@pytest.mark.parametrize("path", CRITICAL, ids=lambda p: p.stem)
def test_regression_case_is_confirmed_and_the_baseline_holds(path, tmp_path):
    import yaml
    s = cs.parse(yaml.safe_load(path.read_text()), str(path))
    out = cs.run(s, tmp_path)
    assert out["replay"]["verdict"] == "confirmed" and out["replay"]["passed"]
    assert out["replay"]["checks"]["same_seed"]["relative_change"] == 0.0
    base = cs.evaluate(s, s.replay["values"], controller=s.baseline)
    assert base["class"] == "nominal"            # the baseline PI keeps the same case under control
