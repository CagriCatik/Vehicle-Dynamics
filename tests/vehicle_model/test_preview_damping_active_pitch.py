"""Predictive damping, active pitch and vertical force allocation (14-DOF only)."""

from __future__ import annotations

import numpy as np
import pytest

from vehicle_model.allocation.vertical_forces import VerticalForceAllocator
from vehicle_model.simulation.ride import build_road, run
from vehicle_model.suspension.preview_control import (ActivePitchController, CornerDamping, DampingConfig, PitchConfig,
                                                      PreviewConfig, RoadPreview)
from vehicle_model.vehicle_models.fourteen_dof import FourteenDOFConfig, FourteenDOFModel, FourteenDOFState

VEH = "configs/suspension/fourteen_dof.yaml"


@pytest.fixture(scope="module")
def model():
    return FourteenDOFModel(FourteenDOFConfig.from_yaml(VEH))


def test_vertical_allocation_reproduces_feasible_demand_and_respects_bounds(model):
    al = VerticalForceAllocator(model.corner_x, model.corner_y, f_max=3000.0)
    r = al.allocate(500.0, -2000.0, 300.0)
    assert r.achieved == pytest.approx(r.demand, rel=1e-3, abs=1.0) and not r.saturated
    assert al.B @ r.forces == pytest.approx(r.achieved, abs=1e-6)
    big = al.allocate(0.0, -50000.0, 0.0)                        # infeasible pitch moment: saturates, does not exceed
    assert big.saturated and np.all(np.abs(big.forces) <= 3000.0 + 1e-9) and abs(big.achieved[1]) < 50000.0
    assert big.achieved[1] < 0.0                                  # still pushes in the demanded direction
    with pytest.raises(ValueError):
        al.allocate(float("nan"), 0.0, 0.0)


def test_preview_is_causal_latency_bound_and_deterministic():
    left, right = build_road({"bumps": [{"x": 20.0, "length": 0.5, "height": 0.05}]})
    pv = RoadPreview((left, right), PreviewConfig(distance=8.0, latency=0.02, noise_std=0.0, seed=1))
    pv.measure(0.0, 12.0)                                         # looks at x = 20 m: the bump crest region
    assert pv.heights_ahead(0.01, 11.0, 22.0) == []               # younger than the latency: not usable
    rows = pv.heights_ahead(0.03, 11.0, 22.0)
    assert len(rows) == 1 and rows[0][0] == pytest.approx(20.0) and rows[0][1] == pytest.approx(float(left.height(20.0)[0]))
    a = RoadPreview((left, right), PreviewConfig(noise_std=0.002, seed=4))
    b = RoadPreview((left, right), PreviewConfig(noise_std=0.002, seed=4))
    for k in range(5):
        a.measure(0.002 * k, 1.0 * k)
        b.measure(0.002 * k, 1.0 * k)
    assert a.heights_ahead(1.0, 8.0, 19.0) == b.heights_ahead(1.0, 8.0, 19.0)
    failed = RoadPreview((left, right), PreviewConfig(failed=True))
    failed.measure(0.0, 12.0)
    assert failed.heights_ahead(1.0, 11.0, 22.0) is None


def test_failed_preview_falls_back_to_skyhook():
    base = dict(road={"bumps": [{"x": 25.0, "length": 0.6, "height": 0.06}]}, manoeuvre={"type": "cruise", "speed_kph": 60},
                duration=2.5)
    sky = run(VEH, damping={"mode": "skyhook"}, **base)
    fail = run(VEH, damping={"mode": "preview_skyhook"}, preview={"failed": True}, **base)
    assert fail["kpis"] == pytest.approx(sky["kpis"])
    live = run(VEH, damping={"mode": "preview_skyhook"}, preview={"latency": 0.02, "noise_std": 0.0}, **base)
    assert np.min(live["history"]["c_FL"]) == pytest.approx(800.0)   # the preview did soften the corner at the bump


@pytest.mark.parametrize("mode", ["passive", "skyhook", "groundhook", "hybrid", "preview_skyhook"])
def test_dampers_stay_dissipative_and_bounded(mode, model):
    cfg = FourteenDOFConfig.from_yaml(VEH)
    r = run(VEH, {"iso_class": "C", "seed": 2}, {"type": "cruise", "speed_kph": 60}, {"mode": mode}, duration=1.5)
    c = np.column_stack([r["history"][f"c_{w}"] for w in ("FL", "FR", "RL", "RR")])
    upper = max(4800.0, cfg.c_damper_front, cfg.c_damper_rear)
    assert np.all(c >= 800.0 - 1e-9) and np.all(c <= upper + 1e-9)
    assert np.all(np.isfinite(r["history"]["zs"])) and r["kpis"]["active_energy_kj"] == 0.0


def test_failed_damper_is_stuck_soft():
    d = CornerDamping(DampingConfig(mode="skyhook", damper_failed=("FL",)), np.full(4, 3000.0), 2.7)
    for k in range(20):
        c = d.command(np.array([0.3, 0.3, -0.2, -0.2]), np.zeros(4), 0.002)
        assert c[0] == pytest.approx(800.0)


def test_active_pitch_is_bounded_and_power_limited(model):
    ctl = ActivePitchController(model.corner_x, model.corner_y, PitchConfig(f_max=3000.0, p_max=4000.0))
    s = FourteenDOFState(vx=25.0, theta=-0.03, dtheta=-0.2, dzs=0.1)
    F = ctl.forces(s, np.full(4, 0.5))
    assert np.all(np.abs(F) <= 3000.0 + 1e-9) and ctl.power <= 4000.0 + 1e-9
    assert float(model.corner_x @ F) > 0.0                        # nose-down pitch is answered with a nose-up moment
    assert np.allclose(ctl.forces(FourteenDOFState(vx=25.0), np.zeros(4)), 0.0, atol=1e-6)


def test_ride_run_is_deterministic():
    kw = dict(road={"iso_class": "B", "seed": 3}, manoeuvre={"type": "brake", "speed_kph": 80, "decel_g": 0.5},
              damping={"mode": "preview_skyhook"}, active_pitch=True, preview={"noise_std": 0.001, "seed": 2}, duration=1.5)
    assert run(VEH, **kw)["kpis"] == run(VEH, **kw)["kpis"]


def test_ride_spec_is_strict(tmp_path):
    from vehicle_model.experiments.ride import parse
    base = {"type": "ride_benchmark", "name": "x", "vehicle": VEH,
            "scenarios": [{"name": "s", "road": {"iso_class": "B"}, "manoeuvre": {"type": "cruise"}}],
            "controllers": [{"name": "c", "damping": {"mode": "skyhook"}}]}
    assert parse(base).name == "x" and parse(parse(base).to_dict()).scenarios == parse(base).scenarios
    bad = [({"controllers": [{"name": "c", "damping": {"mode": "magic"}}]}, "damping.mode"),
           ({"controllers": [{"name": "c", "damping": {"mode": "skyhook"}, "preview": {}}]}, "preview_skyhook"),
           ({"controllers": [{"name": "c", "damping": {"mode": "skyhook", "damper_failed": ["XX"]}}]}, "wheel names"),
           ({"scenarios": [{"name": "s", "road": {"iso_class": "Z"}, "manoeuvre": {"type": "cruise"}}]}, "iso_class"),
           ({"scenarios": [{"name": "s", "road": {}, "manoeuvre": {"type": "drift"}}]}, "manoeuvre.type"),
           ({"scenarios": [{"name": "s", "road": {}, "manoeuvre": {"type": "cruise"}, "duration": 99}]}, "duration")]
    for patch, msg in bad:
        with pytest.raises(ValueError, match=msg):
            parse({**base, **patch})


@pytest.mark.slow
def test_benchmark_effects(tmp_path):
    """Skyhook lowers body acceleration on ISO C; active pitch lowers braking pitch; neither leaves the travel limit."""
    from vehicle_model.experiments.ride import parse
    from vehicle_model.experiments.ride import run as run_exp
    spec = parse({"type": "ride_benchmark", "name": "t", "vehicle": VEH, "workers": 1,
                  "scenarios": [{"name": "iso_C", "road": {"iso_class": "C", "seed": 3}, "manoeuvre": {"type": "cruise", "speed_kph": 60}},
                                {"name": "brake", "road": {"iso_class": "B", "seed": 5},
                                 "manoeuvre": {"type": "brake", "speed_kph": 100, "decel_g": 0.6, "start_s": 0.5}}],
                  "controllers": [{"name": "passive", "damping": {"mode": "passive"}},
                                  {"name": "skyhook", "damping": {"mode": "skyhook"}},
                                  {"name": "pitch", "damping": {"mode": "skyhook"}, "active_pitch": True}],
                  "outputs": {"directory": str(tmp_path), "formats": ["csv", "json", "md"]}})
    res = run_exp(spec, tmp_path)
    k = {(r["scenario"], r["controller"]): r["kpis"] for r in res["results"]}
    assert all(not r["error"] for r in res["results"])
    assert k[("iso_C", "skyhook")]["body_accel_rms"] < 0.9 * k[("iso_C", "passive")]["body_accel_rms"]
    assert k[("iso_C", "skyhook")]["tyre_load_variation"] > k[("iso_C", "passive")]["tyre_load_variation"]   # the trade-off
    assert k[("brake", "pitch")]["pitch_rms_deg"] < 0.9 * k[("brake", "skyhook")]["pitch_rms_deg"]
    assert all(v["travel_over_limit_s"] == 0.0 for v in k.values())
    assert (tmp_path / "kpis.csv").is_file() and (tmp_path / "report.md").is_file()


def test_preview_noise_is_not_read_as_road_input_and_rear_wheels_get_preview():
    """Regression: a finite difference of adjacent noisy samples read 0.5 mm sensor noise as > v_soft road velocity
    and kept every damper soft all the time; the rear wheels had no samples at all."""
    flat = run(VEH, {"bumps": [{"x": 200.0, "length": 0.5, "height": 0.05}]}, {"type": "cruise", "speed_kph": 60},
               {"mode": "preview_skyhook"}, preview={"noise_std": 0.002, "seed": 1}, duration=1.5)
    sky = run(VEH, {"bumps": [{"x": 200.0, "length": 0.5, "height": 0.05}]}, {"type": "cruise", "speed_kph": 60},
              {"mode": "skyhook"}, duration=1.5)
    assert flat["kpis"] == pytest.approx(sky["kpis"])                 # flat road: never softened by noise
    bump = run(VEH, {"bumps": [{"x": 22.0, "length": 0.6, "height": 0.06}]}, {"type": "cruise", "speed_kph": 60},
               {"mode": "preview_skyhook"}, preview={"noise_std": 0.0005}, duration=3.0)
    h = bump["history"]
    cfg = FourteenDOFConfig.from_yaml(VEH)
    for w, x_wheel in (("FL", cfg.a), ("RL", -cfg.b)):                  # each axle soft from shortly before its bump
        t_arrive = float(np.interp(22.0 - x_wheel, h["s_road"], h["t"]))
        t_soft = h["t"][h[f"soft_{w}"] > 0]
        assert t_soft.size and t_arrive - 0.25 < t_soft.min() < t_arrive and t_soft.max() < t_arrive + 0.15
