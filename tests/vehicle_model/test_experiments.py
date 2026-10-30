"""Acceptance: YAML selects vehicle, tyres, manoeuvre, controllers and road; the run is reproducible
and yields a comparison and a verification result."""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from vehicle_model.experiments import experiment_types, load_experiment, parse_experiment, run_experiment
from vehicle_model.experiments.__main__ import main as cli

CONFIGS = Path("configs/experiments")


def _spec(**over):
    d = {
        "name": "tiny",
        "vehicle": "configs/vehicles/reference_passenger_car.yaml",
        "tyres": {"front": "magic_formula_passenger_synthetic", "rear": "magic_formula_passenger_synthetic"},
        "road": {"type": "uniform", "mu": 1.0},
        "manoeuvre": {"type": "step_steer", "speed_kph": 72, "steer_deg": 1.5, "duration_s": 0.3},
        "controllers": [{"law": "passive"}, {"law": "pi"}],
    }
    d.update(over)
    return d


@pytest.mark.parametrize("path", sorted(CONFIGS.glob("*.yaml")), ids=lambda p: p.name)
def test_bundled_experiments_are_valid(path):
    spec = load_experiment(path)
    assert spec.kind in experiment_types() and spec.outputs["directory"]


@pytest.mark.parametrize("override, message", [
    ({"road": {"type": "ice"}}, "road.type"),
    ({"road": {"type": "uniform", "mu": 1.0, "grip": 2}}, "unknown key"),
    ({"manoeuvre": {"type": "step_steer", "speed_kph": 72, "steer": 1}}, "unknown key"),
    ({"controllers": [{"law": "fuzzy"}]}, "law"),
    ({"controllers": [{"law": "pi", "inputs": "4ws_tv"}]}, "yaw-moment input only"),
    ({"controllers": [{"law": "passive", "estimated_state": True}]}, "passive"),
    ({"vehicle": "configs/vehicles/missing.yaml"}, "not found"),
    ({"sweep": {"parameter": "manoeuvre.speed_kph", "values": [72, 900]}}, "speed_kph"),
    ({"dt": 0.5}, "dt"),
    ({"colour": "red"}, "unknown key"),
])
def test_invalid_specifications_are_rejected_with_the_offending_key(override, message):
    with pytest.raises(ValueError, match=message):
        parse_experiment(_spec(**override))


def test_run_is_reproducible_and_complete(tmp_path):
    spec = parse_experiment(_spec(sweep={"parameter": "manoeuvre.speed_kph", "values": [50, 72]}), "tiny.yaml")
    a = run_experiment(spec, tmp_path / "a", verify=False)
    b = run_experiment(spec, tmp_path / "b", verify=False)
    strip = lambda res: [{**r, "kpis": {k: v for k, v in r["kpis"].items() if not k.startswith("computation")}}
                         for r in res["results"]]
    assert json.dumps(strip(a), default=str) == json.dumps(strip(b), default=str)
    assert len(a["results"]) == 4                                     # 2 sweep points x 2 controllers
    for name in ("results.json", "kpis.csv", "report.md", "manoeuvre.speed_kph_50.png"):
        assert (tmp_path / "a" / name).is_file(), name
    assert len(list((tmp_path / "a" / "histories").glob("*.csv"))) == 4
    report = (tmp_path / "a" / "report.md").read_text()
    assert "## Results" in report and "PI (TV)" in report and "synthetic" in report


@pytest.mark.slow
def test_cli_run_includes_a_passing_model_verification(tmp_path):
    spec_file = tmp_path / "spec.yaml"
    import yaml
    spec_file.write_text(yaml.safe_dump(_spec(outputs={"directory": str(tmp_path / "out"), "formats": ["json", "md"]})))
    assert cli(["run", str(spec_file)]) == 0
    res = json.loads((tmp_path / "out" / "results.json").read_text())
    names = {c["name"] for c in res["verification"]}
    assert names == {"parameter consistency", "static wheel loads", "tyre symmetry", "linear reference",
                     "mirror symmetry", "determinism"}
    assert all(c["passed"] for c in res["verification"])
    assert res["provenance"]["git_commit"]


def test_cli_validate_reports_invalid_files(tmp_path, capsys):
    bad = tmp_path / "bad.yaml"
    bad.write_text("name: x\nvehicle: nowhere.yaml\nroad: {type: uniform, mu: 1}\nmanoeuvre: {type: step_steer, speed_kph: 50}\ncontrollers: [{law: pi}]\n")
    assert cli(["validate", str(bad)]) == 1
    assert "INVALID" in capsys.readouterr().out


def test_gif_output_is_opt_in(tmp_path):
    spec = parse_experiment(_spec(outputs={"directory": str(tmp_path), "formats": ["gif"]}))
    run_experiment(spec, verify=False)
    gif = tmp_path / "base.gif"
    assert gif.is_file() and gif.read_bytes()[:6] in (b"GIF89a", b"GIF87a")
    assert "gif" not in parse_experiment(_spec()).outputs["formats"]


# ----------------------------------------------------------------------------- experiment types
def _bench(**over):
    d = {"type": "solver_benchmark", "name": "tiny_bench", "vehicle": "configs/vehicles/seven_dof_passenger.yaml",
         "road": {"type": "uniform", "mu": 0.75},
         "manoeuvre": {"type": "lane_change", "speed_kph": 72, "offset_m": 1.0, "start_s": 0.0, "duration_s": 0.15},
         "control_dt": 0.05, "base": {"N": 3, "max_iter": 10, "max_steer_rear_deg": 8},
         "variants": [{"name": "a"}, {"name": "b", "max_iter": 3}], "verification": False}
    d.update(over)
    return d


def test_type_dispatch_keeps_the_m9_format_and_rejects_unknown_types():
    assert parse_experiment(_spec()).kind == "controller_comparison"            # no type key: experiment engine format
    assert parse_experiment(_spec(type="controller_comparison")).to_dict()["type"] == "controller_comparison"
    with pytest.raises(ValueError, match="unknown experiment type"):
        parse_experiment(_spec(type="monte_carlo"))
    with pytest.raises(ValueError, match="unknown key"):                          # each type has its own schema
        parse_experiment(_spec(type="solver_benchmark"))
    assert {"controller_comparison", "solver_benchmark"} <= set(experiment_types())


@pytest.mark.parametrize("override, message", [
    ({"base": {"N": 3, "horizon": 4}}, "unknown key"),
    ({"base": {"N": 0}}, "integer"),
    ({"base": {"max_steer_rear_deg": 8, "max_steer_rear_rad": 0.1}}, "degrees and radians"),
    ({"variants": [{"name": "a"}, {"name": "a"}]}, "unique"),
    ({"variants": [{"max_iter": 3}]}, "name"),
    ({"manoeuvre": {"type": "sine_with_dwell", "speed_kph": 72}}, "lane_change"),
    ({"road": {"type": "split_mu", "mu": 1.0}}, "uniform"),
    ({"control_dt": 0.5}, "control_dt"),
    ({"controllers": [{"law": "pi"}]}, "unknown key"),
])
def test_solver_benchmark_schema(override, message):
    with pytest.raises(ValueError, match=message):
        parse_experiment(_bench(**override))


def test_solver_benchmark_runs_and_reports_solution_quality(tmp_path):
    import math
    spec = parse_experiment(_bench(), "tiny_bench.yaml")
    assert math.isclose(spec.config({}).max_steer_rear_rad, math.radians(8))      # degree alias
    a = run_experiment(spec, tmp_path / "a")
    b = run_experiment(spec, tmp_path / "b")
    strip = lambda s: [{k: v for k, v in r["kpis"].items() if "_ms_" not in k} for r in s["results"]]
    assert strip(a) == strip(b)                                                   # reproducible except wall clock
    k = a["results"][0]["kpis"]
    assert k["solver_solves"] == 3 and "solver_kkt_stationarity_relative_max" in k and k["plan_infeasible_steps"] == 0
    assert a["results"][1]["config"]["max_iter"] == 3
    for f in ("results.json", "kpis.csv", "report.md", "solver_benchmark.png", "histories/a.csv"):
        assert (tmp_path / "a" / f).is_file(), f
    assert json.loads((tmp_path / "a" / "results.json").read_text())["experiment"]["type"] == "solver_benchmark"
    assert "KKT" in (tmp_path / "a" / "report.md").read_text()


def test_cli_lists_the_type(capsys):
    assert cli(["list"]) == 0
    out = capsys.readouterr().out
    assert "[solver_benchmark]" in out and "[controller_comparison]" in out


def _eq(**over):
    d = {"type": "equilibrium_analysis", "name": "tiny_eq", "vehicle": "configs/vehicles/seven_dof_passenger.yaml",
         "road": {"type": "uniform", "mu": 0.6}, "operating_point": {"speed_kph": 90},
         "sweep": {"parameter": "steer_deg", "values": [0.0, 1.0]}, "linear_check": "main", "verification": False}
    d.update(over)
    return d


@pytest.mark.parametrize("override, message", [
    ({"sweep": {"parameter": "camber", "values": [1]}}, "parameter must be one of"),
    ({"sweep": {"parameter": "steer_deg", "values": []}}, "non-empty"),
    ({"sweep": {"parameter": "steer_deg", "values": [90]}}, "must lie in"),
    ({"series": {"parameter": "steer_deg", "values": [1]}}, "differ from the sweep"),
    ({"road": {"type": "split_mu", "mu": 1.0}}, "uniform"),
    ({"operating_point": {"drive_split": [1, 1, 0, 0]}}, "sum to 1"),
    ({"operating_point": {"camber_deg": 1}}, "unknown key"),
    ({"model": "fourteen_dof"}, "model must be"),
    ({"linear_check": "some"}, "linear_check"),
    ({"kc": "configs/suspension/kc_reference_passenger_car.yaml", "operating_point": {"cog_shift_m": 0.1}}, "cog_shift_m"),
])
def test_equilibrium_analysis_schema(override, message):
    with pytest.raises(ValueError, match=message):
        parse_experiment(_eq(**override))


def test_equilibrium_analysis_runs_with_checks_and_outputs(tmp_path):
    spec = parse_experiment(_eq(series={"parameter": "cornering_scale_rear", "values": [1.0, 0.8]}), "tiny_eq.yaml")
    s = run_experiment(spec, tmp_path)
    assert len(s["results"]) == 4 and all(r["main_accepted"] for r in s["results"])
    assert all(c["passed"] in (True, None) for c in s["checks"])
    e = s["results"][1]["equilibria"][0]
    assert e["branch"] == "main" and e["linear_check"] and e["linearisation"]["stability"] == "stable"
    # softer rear tyres: slower decay of the lateral/yaw modes (the damping ratio alone is no margin: the
    # modes can turn real, zeta = 1, and still decay more slowly)
    decay = lambda r: max(m["re"] for m in r["equilibria"][0]["linearisation"]["modes"] if m["group"] == "lateral_yaw")
    assert decay(s["results"][1]) < decay(s["results"][3]) < 0.0
    for f in ("results.json", "equilibria.csv", "equilibria.png", "eigenvalues.png", "report.md"):
        assert (tmp_path / f).is_file(), f
    assert "Checks of the analysis" in (tmp_path / "report.md").read_text()


def _map(**over):
    d = {"type": "stability_map", "name": "tiny_map", "vehicle": "configs/vehicles/seven_dof_passenger.yaml",
         "road": {"type": "uniform", "mu": 0.6}, "operating_point": {"speed_kph": 80, "steer_deg": 1.0},
         "grid": {"beta_deg": [-20, 20, 2], "r_degps": [-40, 40, 2]}, "simulation": {"duration_s": 1.0},
         "controllers": ["passive"], "uncertainty": {"refine": False}, "portraits": {"points": 2}, "workers": 1,
         "verification": False}
    d.update(over)
    return d


@pytest.mark.parametrize("override, message", [
    ({"controllers": ["passive", "magic"]}, "unknown"),
    ({"controllers": ["passive", "passive"]}, "unique"),
    ({"grid": {"beta_deg": [10, -10, 5], "r_degps": [-40, 40, 5]}}, "min must be below max"),
    ({"grid": {"beta_deg": [-10, 10, 1], "r_degps": [-40, 40, 5]}}, r"integer in \[2, 101\]"),
    ({"simulation": {"dt": 0.1}}, "simulation.dt"),
    ({"criteria": {"loss_beta_deg": -5}}, "positive"),
    ({"uncertainty": {"dt_factor": 2.0}}, "dt_factor"),
    ({"portraits": {"controllers": ["tv"]}}, "portraits.controllers"),
    ({"workers": 0}, "workers"),
])
def test_stability_map_schema(override, message):
    with pytest.raises(ValueError, match=message):
        parse_experiment(_map(**override))


@pytest.mark.slow
def test_stability_map_runs_and_writes_every_output(tmp_path):
    s = run_experiment(parse_experiment(_map(), "tiny_map.yaml"), tmp_path)
    r = s["results"][0]
    assert r["points"] == 4 and sum(r["counts"].values()) == 4
    assert set(r["heuristic_comparison"]) >= {"agreement", "inside_not_returned", "outside_and_returned"}
    assert s["open_loop_equilibria"] and s["open_loop_equilibria"][0]["stability"] == "stable"
    for f in ("results.json", "map.csv", "stability_maps.png", "region_comparison.png", "phase_portraits.png", "report.md"):
        assert (tmp_path / f).is_file(), f
    assert "not a proven region of attraction" in (tmp_path / "report.md").read_text()


def test_cli_lists_the_m11_types(capsys):
    assert cli(["list"]) == 0
    out = capsys.readouterr().out
    assert "[equilibrium_analysis]" in out and "[stability_map]" in out


def _rb(**over):
    d = {"type": "robust_benchmark", "name": "tiny_rb", "vehicle": "configs/vehicles/seven_dof_passenger.yaml",
         "road": {"type": "uniform", "mu": 1.0},
         "manoeuvre": {"type": "step_steer", "speed_kph": 72, "steer_deg": 1.5, "duration_s": 0.4},
         "controllers": [{"name": "base", "law": "pi", "baseline": True},
                         {"name": "rlqr", "law": "robust_lqr", "inputs": "4ws_tv", "alpha": 3,
                          "supervisor": {"fallback": "pi"}}],
         "feedback": ["ideal", "sensor_constrained"],
         "uncertainty": [{"name": "nominal"}, {"name": "soft", "cornering_scale_rear": 0.8, "actuator_delay_s": 0.02}],
         "workers": 1, "verification": False}
    d.update(over)
    return d


@pytest.mark.parametrize("override, message", [
    ({"controllers": [{"name": "a", "law": "h_infinity"}]}, "law must be one of"),
    ({"controllers": [{"name": "a", "law": "pi", "inputs": "4ws_tv"}]}, "yaw-moment input only"),
    ({"controllers": [{"name": "a", "law": "pi"}, {"name": "a", "law": "lqr"}]}, "unique"),
    ({"controllers": [{"name": "a", "law": "pi", "supervisor": {"fallback": "nmpc"}}]}, "fallback must be"),
    ({"feedback": ["ideal", "telepathic"]}, "feedback must be"),
    ({"uncertainty": [{"name": "x", "mass": 1.2}]}, "unknown uncertainty keys"),
    ({"uncertainty": [{"name": "x", "mass_factor": -1}]}, "positive"),
    ({"manoeuvre": {"type": "split_mu_braking", "speed_kph": 80}}, "manoeuvre.type"),
])
def test_robust_benchmark_schema(override, message):
    with pytest.raises(ValueError, match=message):
        parse_experiment(_rb(**override))


@pytest.mark.slow
def test_robust_benchmark_runs_matrix_with_baseline_comparison(tmp_path):
    s = run_experiment(parse_experiment(_rb(), "tiny_rb.yaml"), tmp_path)
    assert len(s["results"]) == 2 * 2 * 2 and not any(r["error"] for r in s["results"])
    assert {r["feedback"] for r in s["results"]} == {"ideal", "sensor_constrained"}
    assert len(s["comparison"]) == 4 and all(c["baseline"] == "base" for c in s["comparison"])
    assert "rlqr" in s["linear_robustness"] and s["parameter_box"] == {"C_r": (0.8, 1.0)}
    k = s["results"][0]["kpis"]
    assert {"yaw_rate_rms_error_degps", "max_tyre_utilisation", "fallback_rate", "degraded_share"} <= set(k)
    for f in ("results.json", "kpis.csv", "report.md", "matrix_max_sideslip_deg.png", "yaw_rate_histories.png"):
        assert (tmp_path / f).is_file(), f
    assert "Relative to the baseline" in (tmp_path / "report.md").read_text()
