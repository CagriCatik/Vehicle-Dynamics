"""Drift plan D7: robustness study plumbing (plant variants, strict config, classification)."""

from __future__ import annotations

import copy

import pytest

from vehicle_model.simulation.drift.robustness import _design, _plant, load, run_case

CFG = "configs/drift/robustness.yaml"


@pytest.fixture(scope="module")
def cfg():
    c = load(CFG)
    c["scenario"]["duration_s"] = 1.0
    return c


def test_config_is_strict(tmp_path):
    import yaml
    base = yaml.safe_load(open(CFG))
    for bad, msg in (({"cases": [{"name": "x", "masss": 1.1}]}, "unknown key"),
                     ({"cases": [{"name": "x", "feedback": "telepathy"}]}, "feedback"),
                     ({"cases": [{"name": "x"}, {"name": "x"}]}, "unique"),
                     ({"cases": [{"name": "x", "tyre_model": "slicks"}]}, "tyre_model")):
        p = tmp_path / "c.yaml"
        p.write_text(yaml.safe_dump({**base, **bad}))
        with pytest.raises(ValueError, match=msg):
            load(p)


def test_plant_variants_are_built_from_the_existing_models(cfg):
    from vehicle_model.roads.surface import MuSplitRoad
    from vehicle_model.vehicle_models import TransientSevenDOFModel
    m, _, _ = _design(cfg)
    p, _ = _plant(m, {"name": "h", "hcog_m": 0.6}, 0.9)
    assert p.params.hcog == 0.6 and p._load_params.hcog == 0.6 and m.params.hcog != 0.6
    t, _ = _plant(m, {"name": "t", "tyre_model": "transient"}, 0.9)
    assert isinstance(t, TransientSevenDOFModel)
    s, _ = _plant(m, {"name": "s", "split_road": {"mu_low": 0.5, "split_y": 24.0}}, 0.9)
    assert isinstance(s.road, MuSplitRoad) and s.road.get_friction(0.0, 40.0) == pytest.approx(0.5)
    assert s.road.get_friction(0.0, 0.0) == pytest.approx(0.9)


@pytest.mark.parametrize("name", ["nominal", "transient_tyres", "actuator_lag"])
def test_short_cases_run_and_are_classified(cfg, name):
    case = next(c for c in cfg["cases"] if c["name"] == name)
    r = run_case((cfg, case, 0))
    assert r["class"] in ("held", "degraded", "lost") and "error" not in r
    assert r["load_balance_max_rel_error"] < 1e-6


def test_an_exception_is_a_classified_result(cfg):
    bad = copy.deepcopy(cfg)
    bad["scenario"]["dt_s"] = -1.0          # makes the run fail inside the case
    r = run_case((bad, {"name": "nominal"}, 0))
    assert r["class"] == "failed" and r["error"]
