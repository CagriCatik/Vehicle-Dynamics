"""M9 acceptance: YAML selects vehicle, tyres, manoeuvre, controllers and road; the run is reproducible
and yields a comparison and a verification result."""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from vehicle_model.experiments import load_experiment, parse_experiment, run_experiment
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
    assert spec.controllers and spec.outputs["directory"]


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
