"""Function registry, deterministic arbitration, supervisor record and the passive to fault_tolerant benchmark matrix."""

from __future__ import annotations

import copy
import math

import numpy as np
import pytest
import yaml

from vehicle_model.controllers.framework import ControlTarget
from vehicle_model.controllers.motion_control import (LongitudinalRequest, MotionContext, MotionController,
                                                      arbitrate_longitudinal)
from vehicle_model.controllers.motion_supervisor import FUNCTIONS, PRIORITIES, VehicleMotionSupervisor
from vehicle_model.controllers.yaw_laws import YawRatePI
from vehicle_model.core.types import SevenDOFState
from vehicle_model.vehicle_models import SevenDOFModel

VEH = "configs/vehicles/seven_dof_passenger.yaml"
MATRIX = "configs/experiments/motion_management_matrix.yaml"


def test_longitudinal_arbitration_takes_the_strongest_braking_and_never_sums():
    drv = LongitudinalRequest("driver", "driver", "base", -2000.0)
    fx, log = arbitrate_longitudinal([drv])
    assert fx == -2000.0 and log["winner"] == "driver"
    fx, log = arbitrate_longitudinal([drv, LongitudinalRequest("hdc", "safety", "brake_min", -3000.0)])
    assert fx == -3000.0 and log["winner"] == "hdc"                         # stronger floor wins, not -5000
    fx, log = arbitrate_longitudinal([drv, LongitudinalRequest("hdc", "safety", "brake_min", -1000.0)])
    assert fx == -2000.0 and log["winner"] == "driver"                      # weaker floor: the driver brakes harder
    gvc = LongitudinalRequest("gvc", "agility", "add", -300.0)
    fx, log = arbitrate_longitudinal([LongitudinalRequest("driver", "driver", "base", 500.0), gvc])
    assert fx == 200.0 and log["winner"] == "driver+gvc" and not log["suppressed"]
    fx, log = arbitrate_longitudinal([LongitudinalRequest("driver", "driver", "base", 500.0), gvc,
                                      LongitudinalRequest("hdc", "safety", "brake_min", -1500.0)])
    assert fx == -1500.0 and log["suppressed"] == ["gvc"]                    # agility yields to a safety floor
    fx, _ = arbitrate_longitudinal([LongitudinalRequest("driver", "driver", "base", 0.0),
                                    LongitudinalRequest("hdc", "safety", "brake_min", 0.0)])
    assert fx == 0.0                                                          # an inactive floor changes nothing


def test_registry_declares_one_owner_and_valid_priorities():
    names = [f.name for f in FUNCTIONS]
    assert len(set(names)) == len(names)
    assert all(f.priority in PRIORITIES and f.model in ("7dof", "14dof", "both") and f.owner for f in FUNCTIONS)
    assert {"abs", "tcs", "yaw_law", "cbc", "hdc", "gvc", "afs", "health", "rsc", "active_pitch"} <= set(names)
    from pathlib import Path
    assert all((Path("src/vehicle_model") / f.module).is_file() for f in FUNCTIONS)


def test_empty_function_block_means_enabled_not_absent():
    """Regression (motion management review): ``hdc: {}`` / ``gvc: {}`` were stored as 'no block' and silently dropped."""
    from vehicle_model.experiments.spec import parse_controller_comparison
    base = {"name": "x", "vehicle": VEH, "road": {"type": "uniform", "mu": 1.0}, "manoeuvre": {"type": "coast", "speed_kph": 10}}
    c = parse_controller_comparison({**base, "controllers": [{"law": "no_yaw", "hdc": {}, "gvc": {}}]}).controllers[0]
    assert dict(c.hdc) == {"enabled": True} and dict(c.gvc) == {"enabled": True}
    assert "HDC" in c.label and "GVC" in c.label


def test_itc_traction_scope_leaves_braking_and_handling_untouched():
    from vehicle_model.controllers.integrated_traction import YawToleranceLaw
    s = SevenDOFState(vx=20.0, r=0.10)
    ctx = lambda fx: MotionContext(0.0, 0.11, 1.0, 0.01, 4000.0, 0.1, 0.0, fx)        # error 0.01 rad/s, inside 2 deg/s
    inner = YawRatePI()
    always, scoped = YawToleranceLaw(YawRatePI(), math.radians(2.0)), YawToleranceLaw(YawRatePI(), math.radians(2.0), "traction")
    assert always.request(s, ctx(-500.0)).yaw_moment is None                 # traction control behaviour: band always active
    assert scoped.request(s, ctx(-500.0)).yaw_moment == pytest.approx(inner.request(s, ctx(-500.0)).yaw_moment)
    assert YawToleranceLaw(YawRatePI(), math.radians(2.0), "traction").request(s, ctx(800.0)).yaw_moment is None
    with pytest.raises(ValueError):
        YawToleranceLaw(YawRatePI(), 0.1, "sometimes")


def test_supervisor_only_observes():
    from vehicle_model.roads.surface import UniformRoad
    m = SevenDOFModel.from_yaml(VEH, road=UniformRoad(0.4))
    p = m.params
    a, b = MotionController(p, YawRatePI()), VehicleMotionSupervisor(MotionController(p, YawRatePI()))
    s = SevenDOFState(vx=20.0, omega=np.full(4, 20.0 / p.Reff))
    tel = None
    for k in range(60):
        tgt = ControlTarget(driver_steer_f=0.03, driver_brake_pedal=1.0 if k > 20 else 0.0)
        oa, ob = a.compute_control(s, tgt, tel, 0.01), b.compute_control(s, tgt, tel, 0.01)
        assert np.array_equal(oa.T_brake, ob.T_brake) and np.array_equal(oa.T_drive, ob.T_drive) and oa.steer_f == ob.steer_f
        s, tel = m.step(s, _input(ob), 0.01)
    summ = b.summary()
    assert summ["transitions"] >= 2 and "yaw_law" in summ["active_time_s"]
    assert any(e[1] == "grip_limit" and e[2] == "on" and e[0] > 0.2 for e in b.events)   # full braking on mu 0.4
    assert 1.0 <= summ["condition_median"] < 1e3


def _input(out):
    from vehicle_model.core.types import SevenDOFControlInput
    return SevenDOFControlInput(steer_f=out.steer_f, steer_r=out.steer_r, T_drive=out.T_drive, T_brake=out.T_brake)


def test_matrix_spec_draws_and_identical_by_construction():
    from vehicle_model.experiments.motion_management_benchmark import _effective_level, draws, parse
    data = yaml.safe_load(open(MATRIX))
    spec = parse(data)
    assert [lv["name"] for lv in spec.levels] == "passive esc tv_4ws brake_functions traction afs_gvc_rsc ride_control fault_tolerant".split() and len(spec.manoeuvres) == 15
    d1, d2 = draws(spec), draws(parse(copy.deepcopy(data)))
    assert d1 == d2 and d1[0]["speed"] == 1.0 and d1[0]["mass"] == 1.0 and len(d1) == data["variations"]["draws"]
    lv = {x["name"]: x for x in spec.levels}
    m7 = next(m for m in spec.manoeuvres if m["model"] == "7dof")
    ride = next(m for m in spec.manoeuvres if m["model"] == "14dof_ride")
    roll = next(m for m in spec.manoeuvres if m["model"] == "14dof_rollover")
    assert _effective_level(spec, m7, lv["ride_control"]) == "afs_gvc_rsc" and _effective_level(spec, m7, lv["fault_tolerant"]) == "fault_tolerant"
    assert len({_effective_level(spec, ride, lv[n]) for n in ("passive", "esc", "tv_4ws", "brake_functions", "traction", "afs_gvc_rsc")}) == 1   # passive damping
    assert _effective_level(spec, ride, lv["ride_control"]) == _effective_level(spec, ride, lv["fault_tolerant"])
    assert _effective_level(spec, roll, lv["traction"]) != _effective_level(spec, roll, lv["afs_gvc_rsc"])  # RSC from afs_gvc_rsc
    for patch, msg in (({"levels": data["levels"][:1] + [dict(data["levels"][1], **{"same_as_7dof": "P9"})]}, "earlier level"),
                       ({"variations": {"draws": 0}}, "draws"),
                       ({"manoeuvres": [dict(data["manoeuvres"][0], **{"model": "9dof"})]}, "model"),
                       ({"levels": [dict(data["levels"][0], fault_tolerant=True)]}, "passive")):
        with pytest.raises(ValueError, match=msg):
            parse({**data, **patch})



@pytest.mark.slow
def test_small_matrix_end_to_end(tmp_path):
    from vehicle_model.experiments.motion_management_benchmark import parse, run
    data = yaml.safe_load(open(MATRIX))
    keep = {"passive", "tv_4ws", "afs_gvc_rsc", "ride_control"}
    data["levels"] = [lv for lv in data["levels"] if lv["name"] in keep]
    data["manoeuvres"] = [m for m in data["manoeuvres"] if m["name"].startswith(("06_", "12_"))]
    data["variations"] = {"draws": 2, "seed": 3, "speed_pct": 5, "mu_pct": 5, "mass_pct": 5, "hcog_pct": 5}
    data["workers"] = 1
    data["outputs"] = {"directory": str(tmp_path), "formats": ["csv", "json", "md", "png"]}
    res = run(parse(data), tmp_path)
    assert not any(r["error"] for r in res["results"])
    st = {(s["manoeuvre"], s["kpi"], s["level"]): s for s in res["statistics"]}
    k = "yaw_error_rms_degps"
    assert st[("06_step_steer", k, "ride_control")]["median"] == st[("06_step_steer", k, "afs_gvc_rsc")]["median"]        # shared by construction
    assert st[("06_step_steer", k, "passive")]["paired_median_diff_vs_baseline"] == 0.0
    assert st[("06_step_steer", k, "tv_4ws")]["median"] < st[("06_step_steer", k, "passive")]["median"]
    assert st[("12_uneven_road", "body_accel_rms", "ride_control")]["median"] < st[("12_uneven_road", "body_accel_rms", "afs_gvc_rsc")]["median"]
    assert (tmp_path / "stats.csv").is_file() and (tmp_path / "matrix.png").is_file() and (tmp_path / "report.md").is_file()
