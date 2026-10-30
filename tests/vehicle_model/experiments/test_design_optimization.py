"""Plan V3.0 M14: design optimisation (Pareto dominance, NSGA-II, validity, coupling, re-check)."""

from __future__ import annotations

import copy
import itertools
import math

import numpy as np
import pytest
import yaml

from vehicle_model.experiments import design_optimization as do
from vehicle_model.experiments.types import get_type

CFG = "configs/experiments/design_chassis_tv.yaml"


def spec(**over):
    d = yaml.safe_load(open(CFG))
    d.update(over)
    return do.parse(d, "<test>")


# ----------------------------------------------------------------------------- Pareto dominance
def test_dominance_is_strict_and_minimising():
    assert do.dominates([1, 2], [2, 2]) and do.dominates([1, 1], [2, 3])
    assert not do.dominates([1, 2], [1, 2])            # equal: no domination
    assert not do.dominates([1, 3], [2, 2]) and not do.dominates([2, 2], [1, 3])


def test_non_dominated_sort_against_brute_force():
    rng = np.random.default_rng(0)
    F = rng.integers(0, 6, size=(60, 3)).astype(float)    # many ties and duplicates
    fronts = do.non_dominated_sort(F)
    assert sorted(itertools.chain(*fronts)) == list(range(60))
    rank = {i: r for r, f in enumerate(fronts) for i in f}
    for i in range(60):
        dominators = [j for j in range(60) if do.dominates(F[j], F[i])]
        # rank = 1 + highest rank of any dominator (0 if none): the definition of the fronts
        assert rank[i] == (1 + max(rank[j] for j in dominators) if dominators else 0)
    for f in fronts:
        assert not any(do.dominates(F[a], F[b]) for a in f for b in f)


def test_crowding_distance_keeps_the_extremes():
    F = np.array([[0.0, 3.0], [1.0, 2.0], [2.0, 1.0], [3.0, 0.0]])
    d = do.crowding_distance(F)
    assert math.isinf(d[0]) and math.isinf(d[3]) and d[1] == pytest.approx(d[2]) and np.isfinite(d[1])


# ----------------------------------------------------------------------------- spec and validity
def test_registered():
    assert get_type("design_optimization").parse is do.parse


@pytest.mark.parametrize("bad, msg", [
    ({"design": [{"name": "warp", "low": 0, "high": 1}]}, "unknown"),
    ({"design": [{"name": "arb_front", "low": 5, "high": 1}]}, "low < high"),
    ({"objectives": ["handling_yaw_error"]}, "2-4"),
    ({"objectives": ["handling_yaw_error", "happiness"]}, "2-4"),
    ({"validity": {"bounce_hz": [2.0, 1.0]}}, "validity"),
    ({"controller": {"law": "pi"}, "design": [{"name": "lqr_q_r", "low": 1, "high": 9}]}, "LQR weights need"),
    ({"search": {"method": "grid"}}, "nsga2"),
])
def test_spec_is_strict(bad, msg):
    with pytest.raises(ValueError, match=msg):
        spec(**bad)


def test_optimisation_and_evaluation_sets_must_differ():
    d = yaml.safe_load(open(CFG))
    d["evaluation"] = copy.deepcopy(d["optimisation"])
    with pytest.raises(ValueError, match="must differ"):
        do.parse(d, "<test>")


def test_roll_stiffness_coupling_matches_the_14dof_model():
    """K_phi from the 14-DOF force law: springs k t^2/2, bar k_arb [N m/deg] * 180/pi; checked by a roll probe."""
    from vehicle_model.vehicle_models.fourteen_dof import FourteenDOFModel, FourteenDOFState
    s = spec()
    cfg = do.ride_config(s)
    kf, kr = do.roll_stiffness(cfg)
    m = FourteenDOFModel(cfg)
    phi = 0.01
    st = FourteenDOFState(vx=0.0, phi=phi)
    # deflections from a pure body roll; the model's roll acceleration gives the restoring moment
    _, tel = m.evaluate_dynamics(st, np.zeros(4), np.zeros(4), np.zeros(4))
    moment = -tel.ddphi * cfg.Ixx
    assert moment == pytest.approx((kf + kr) * phi, rel=0.03)
    ref = {p["name"]: do.reference_values(s)[p["name"]] for p in s.design}
    assert do.derived(s, ref)["roll_share_front"] == pytest.approx(kf / (kf + kr))


def test_invalid_designs_are_excluded_with_reasons():
    s = spec()
    ref = {p["name"]: do.reference_values(s)[p["name"]] for p in s.design}
    assert do.validity_violations(s, ref) == []
    soft = {**ref, "damper_front": 1200.0, "damper_rear": 1200.0}
    s2 = spec(validity={"damping_ratio": [0.2, 0.7]})
    assert any("damping ratio" in r for r in do.validity_violations(s2, soft))
    r = do.evaluate(s2, soft)
    assert not r["valid"] and r["reasons"] and not r["objectives"]
    assert any("outside" in x for x in do.validity_violations(s, {**ref, "arb_front": 5000.0}))
    s3 = spec(fixed={"rear_ratio_low": 0.3})
    assert any("rear_ratio_low" in x for x in do.validity_violations(s3, {**ref, "rear_ratio_high": 0.1}))


# ----------------------------------------------------------------------------- NSGA-II and selection
def _fake(monkeypatch):
    """Analytic objectives: f1 = x0, f2 = (1 - x0)^2 + x1 (front: x1 = low), designs with x2 > 0.9 invalid."""
    def fake(spec_, values, which="optimisation", objectives=None):
        names = [p["name"] for p in spec_.design]
        u = [(values[n] - p["low"]) / (p["high"] - p["low"]) for n, p in zip(names, spec_.design)]
        if u[2] > 0.9:
            return {"values": values, "valid": False, "reasons": ["invalid region"], "objectives": {}, "error": ""}
        shift = 0.05 if which == "evaluation" else 0.0
        f = {spec_.objectives[0]: u[0] + shift, spec_.objectives[1]: (1 - u[0]) ** 2 + u[1] + shift}
        for o in spec_.objectives[2:] + spec_.report:
            f[o] = 1.0
        return {"values": dict(values), "valid": True, "reasons": [], "objectives": f, "error": "",
                "derived": {"roll_share_front": 0.5}}
    monkeypatch.setattr(do, "evaluate", fake)


def test_nsga2_is_reproducible_and_approaches_the_true_front(monkeypatch):
    _fake(monkeypatch)
    s = spec(objectives=["handling_yaw_error", "comfort_body_accel"], report=[],
             search={"method": "nsga2", "population": 16, "generations": 20, "seed": 5})
    a, b = do.nsga2(s), do.nsga2(s)
    assert [e["values"] for e in a] == [e["values"] for e in b]
    assert len(a) == 16 * 21
    front = do.pareto_set(s, a)
    assert front and all(a[i]["valid"] for i in front)
    gap = lambda ev, idx: np.median([(ev[i]["values"]["arb_rear"] - 50.0) / 750.0 for i in idx])   # true front: 0
    rnd = do.nsga2(spec(objectives=["handling_yaw_error", "comfort_body_accel"], report=[],
                        search={"method": "nsga2", "population": 16 * 21, "generations": 0, "seed": 5}))
    rfront = do.pareto_set(s, rnd)
    assert gap(a, front) < 0.01 and gap(a, front) < gap(rnd, rfront)      # same budget, random sampling
    assert len(front) > len(rfront)
    last = [e for e in a if e["generation"] == 20 and e["valid"]]
    first = [e for e in a if e["generation"] == 0 and e["valid"]]
    gap = lambda es: np.mean([(es_["values"]["arb_rear"] - 50.0) / 750.0 for es_ in es])
    assert gap(last) < gap(first)


def test_selection_and_recheck(monkeypatch):
    _fake(monkeypatch)
    s = spec(objectives=["handling_yaw_error", "comfort_body_accel"], report=[],
             search={"method": "nsga2", "population": 12, "generations": 2, "seed": 1}, selection={"count": 3})
    ev = do.nsga2(s)
    front = do.pareto_set(s, ev)
    chosen = do.select(s, ev, front)
    labels = [c[0] for c in chosen]
    assert labels[0] == "knee" and len(chosen) <= 3 and all(i in front for _, i in chosen)
    o0 = s.objectives[0]
    best = min(front, key=lambda i: ev[i]["objectives"][o0])
    assert ("best_" + o0, best) in chosen or any(i == best for _, i in chosen)
    ref = {"valid": True, "objectives": {o: 0.6 for o in s.objectives}}
    opt = {"values": ev[best]["values"], "objectives": {o0: 0.1, s.objectives[1]: 0.9}}
    good = do.recheck(s, opt, ref, {"valid": True, "objectives": {o0: 0.2, s.objectives[1]: 0.7, "stability_sideslip": 2.0}},
                      {"valid": True, "objectives": {o0: 0.5, s.objectives[1]: 0.6}})
    assert good["passed"] and good["claims"][o0]["kept"] and s.objectives[1] not in good["claims"]
    bad = do.recheck(s, opt, ref, {"valid": True, "objectives": {o0: 0.6, s.objectives[1]: 0.7, "stability_sideslip": 2.0}},
                     {"valid": True, "objectives": {o0: 0.5, s.objectives[1]: 0.6}})
    assert not bad["passed"] and "not confirmed" in bad["reasons"][0]
    lost = do.recheck(s, opt, ref, {"valid": True, "objectives": {o0: 0.2, s.objectives[1]: 0.7, "stability_sideslip": 40.0}},
                      {"valid": True, "objectives": {o0: 0.5, s.objectives[1]: 0.6}})
    assert not lost["passed"] and "loss of control" in lost["reasons"][0]


def test_sensitivities_recover_a_linear_dependence(monkeypatch):
    _fake(monkeypatch)
    s = spec(objectives=["handling_yaw_error", "comfort_body_accel"], report=[],
             search={"method": "nsga2", "population": 30, "generations": 0, "seed": 2})
    sens = do.sensitivities(s, do.nsga2(s))
    src = sens["handling_yaw_error"]["src"]
    assert sens["handling_yaw_error"]["r2"] > 0.99
    assert max(src, key=lambda k: abs(src[k])) == "arb_front" and src["arb_front"] > 0.9


# ----------------------------------------------------------------------------- one real evaluation
@pytest.mark.slow
def test_reference_design_evaluates_on_the_existing_models():
    s = spec(objectives=["handling_yaw_error", "comfort_body_accel"], report=["roll_gradient"])
    ref = {p["name"]: do.reference_values(s)[p["name"]] for p in s.design}
    r = do.evaluate(s, ref)
    assert r["valid"], r["reasons"]
    assert 0.2 < r["objectives"]["comfort_body_accel"] < 3.0
    assert 1.0 < r["objectives"]["roll_gradient"] < 8.0           # deg/g, passenger car range
    assert r["objectives"] == do.evaluate(s, ref)["objectives"]   # deterministic
