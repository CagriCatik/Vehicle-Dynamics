"""Workbench over the experiment engine (configure, run, cancel, results, compare, export, CLI).

Acceptance (Plan 10.6): an experiment is configured through the HTTP interface, executed by the existing
experiment engine in a separate process, compared and exported as a report, and the same experiment is
reproduced by the CLI with identical results.
"""

from __future__ import annotations

import csv
import io
import shutil
import subprocess
import sys
import uuid
import zipfile
from pathlib import Path

import pytest

pytest.importorskip("fastapi")
pytest.importorskip("httpx")
from fastapi.testclient import TestClient  # noqa: E402

from vehicle_model.workbench.app import create_app  # noqa: E402

REPO = Path(__file__).resolve().parents[2]


@pytest.fixture(scope="module")
def client():
    root = f"output/workbench_test_{uuid.uuid4().hex[:8]}"          # inside the repository, removed afterwards
    app = create_app(root=root, repo=str(REPO))
    with TestClient(app) as c:
        c.root = REPO / root
        yield c
    shutil.rmtree(REPO / root, ignore_errors=True)


def experiment(name="wb_step", controllers=({"law": "passive"}, {"law": "pi"}), duration=1.0):
    return {"type": "controller_comparison", "name": name, "vehicle": "configs/vehicles/seven_dof_passenger.yaml",
            "tyres": {"front": "magic_formula_passenger_synthetic", "rear": "magic_formula_passenger_synthetic"},
            "road": {"type": "uniform", "mu": 1.0},
            "manoeuvre": {"type": "step_steer", "speed_kph": 72, "steer_deg": 2, "duration_s": duration},
            "controllers": list(controllers), "dt": 0.005, "seed": 0, "verification": False,
            "outputs": {"formats": ["csv", "json", "png", "md"]}}


def test_options_come_from_the_experiment_spec(client):
    from vehicle_model.experiments import spec as S
    o = client.get("/api/options").json()
    assert o["laws"] == list(S.LAWS) and set(o["manoeuvres"]) == set(S.MANOEUVRES)
    assert "configs/vehicles/seven_dof_passenger.yaml" in o["vehicles"] and "magic_formula_passenger_synthetic" in o["tyres"]
    assert "manoeuvre.speed_kph" in o["sweep_parameters"] and "design_optimization" in o["experiment_types"]


def test_validation_uses_the_engine_parser(client):
    ok = client.post("/api/validate", json=experiment()).json()
    assert ok["valid"] and ok["kind"] == "controller_comparison" and "step_steer" in ok["description"]
    bad = client.post("/api/validate", json={**experiment(), "road": {"type": "ice"}}).json()
    assert not bad["valid"] and "road.type" in bad["error"]
    assert client.post("/api/configs", json={**experiment(), "dt": 5.0}).status_code == 400
    raw = client.post("/api/validate_yaml", json={"text": "type: design_optimization\nname: x\n"}).json()
    assert not raw["valid"]                                            # other types: their own parser


def test_paths_outside_the_workbench_are_refused(client):
    assert client.get("/files/../../etc/passwd").status_code in (400, 404)
    assert client.get("/api/config", params={"path": "pyproject.toml"}).status_code == 400
    assert client.get("/api/history", params={"path": "src/vehicle_model/__init__.py"}).status_code == 400


def test_configure_run_compare_export_and_reproduce_with_the_cli(client, tmp_path):
    saved = client.post("/api/configs", json=experiment()).json()
    assert saved["path"].endswith("wb_step.yaml") and (REPO / saved["path"]).is_file()
    assert any(c["path"] == saved["path"] for c in client.get("/api/configs").json())
    job = client.post("/api/jobs", json={"config": saved["path"]}).json()
    assert job["status"] == "running" and job["command_line"].startswith("python -m vehicle_model.experiments run")
    done = client.app.state.jobs.wait(job["id"], timeout=300)
    assert done["status"] == "done", done["log_tail"]
    assert done["progress"] == {"done": 2, "total": 2, "label": "writing outputs"}
    res = client.get("/api/result", params={"id": done["out"]}).json()
    assert len(res["kpis"]) == 2 and len(res["histories"]) == 2 and res["figures"]
    assert res["reproduce"] == done["command_line"]
    h = client.get("/api/history", params={"path": res["histories"][0]}).json()["columns"]
    for col in ("t", "X", "Y", "r", "beta", "ay", "Fz_FL", "Fy_RR", "Mz_allocated"):
        assert col in h
    assert abs(sum(h[f"Fz_{w}"][5] for w in ("FL", "FR", "RL", "RR")) - 1200 * 9.81) < 1.0   # wheel loads carry the weight
    assert any(r["id"] == done["out"] for r in client.get("/api/results").json())

    # a second run of the same configuration, compared and exported
    job2 = client.post("/api/jobs", json={"config": saved["path"]}).json()
    done2 = client.app.state.jobs.wait(job2["id"], timeout=300)
    cmp_ = client.post("/api/compare", json={"ids": [done["out"], done2["out"]]}).json()
    assert len(cmp_["rows"]) == 4 and "max_sideslip_deg" in cmp_["kpis"]
    exp = client.post("/api/export", json={"ids": [done["out"], done2["out"]], "title": "acceptance"}).json()
    z = zipfile.ZipFile(io.BytesIO(client.get(f"/files/{exp['path']}").content))
    names = z.namelist()
    assert any(n.endswith("report.md") for n in names) and any(n.endswith(".png") for n in names)
    md = z.read(next(n for n in names if n.endswith("report.md"))).decode()
    assert "acceptance" in md and "KPI comparison" in md and "python -m vehicle_model.experiments run" in md

    # the same experiment through the CLI: identical KPIs
    cli_out = tmp_path / "cli"
    subprocess.run([sys.executable, "-m", "vehicle_model.experiments", "run", saved["path"], "--out", str(cli_out)],
                   cwd=REPO, check=True, capture_output=True)
    read = lambda p: list(csv.DictReader(open(p, encoding="utf-8")))
    a, b = read(REPO / done["out"] / "kpis.csv"), read(cli_out / "kpis.csv")
    drop = lambda rows: [{k: v for k, v in r.items() if not k.startswith("computation")} for r in rows]
    assert drop(a) == drop(b)


def test_long_run_does_not_block_and_can_be_cancelled(client):
    cfg = experiment("wb_long", controllers=[{"law": "ltv_mpc", "inputs": "4ws_tv"}] * 1, duration=30.0)
    saved = client.post("/api/configs", json=cfg).json()
    job = client.post("/api/jobs", json={"config": saved["path"]}).json()
    # the server answers while the run is going on
    assert client.get("/api/jobs").status_code == 200 and client.get(f"/api/jobs/{job['id']}").json()["status"] == "running"
    c = client.post(f"/api/jobs/{job['id']}/cancel").json()
    assert c["status"] == "cancelled"
    end = client.app.state.jobs.wait(job["id"], timeout=60)
    assert end["status"] == "cancelled" and end["returncode"] != 0 and not end["has_results"]


def test_static_front_end_is_served(client):
    r = client.get("/ui/index.html")
    assert r.status_code == 200 and "Vehicle-Dynamics workbench" in r.text
    assert client.get("/ui/app.js").status_code == 200
