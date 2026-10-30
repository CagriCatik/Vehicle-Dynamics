"""Parking scenarios with rear-axle steering and the scenario book (catalogue + CLI)."""

from __future__ import annotations

import json
import math

import numpy as np
import pytest

from vehicle_model.scenario_book.__main__ import main as book_main
from vehicle_model.scenario_book.catalog import CATEGORIES, book_markdown, catalog, run_entry
from vehicle_model.simulation.four_wheel_steering.parking import (
    PARKING_SCENARIOS,
    body_corners,
    clearance,
    required_gap_ahead,
    run_parking_comparison,
)
from vehicle_model.simulation.four_wheel_steering.results import ScenarioMetrics, ScenarioRunResult
from vehicle_model.vehicle_models import SevenDOFModel

LF, LR = 1.3, 1.5


def _result(X, Y, psi):
    z = np.zeros_like(np.asarray(X, dtype=float))
    return ScenarioRunResult("x", ScenarioMetrics("s", "x"), z, np.asarray(X, float), np.asarray(Y, float),
                             np.asarray(psi, float), z, z, z, z, z, z, z)


def test_body_outline_is_4_5_by_1_8_m_and_rotates_with_the_heading():
    c = body_corners(0.0, 0.0, 0.0, LF, LR)[0]
    assert np.ptp(c[:, 0]) == pytest.approx(4.5) and np.ptp(c[:, 1]) == pytest.approx(1.8)
    c90 = body_corners(0.0, 0.0, math.pi / 2, LF, LR)[0]
    assert np.ptp(c90[:, 1]) == pytest.approx(4.5) and np.ptp(c90[:, 0]) == pytest.approx(1.8)
    assert c[:, 0].max() - LF == pytest.approx(LF * (4.5 / (1.5 * 2.8) * 1.5 - 1.0))   # front overhang


def test_clearance_and_required_gap_on_a_straight_drive():
    r = _result(np.linspace(0.0, 3.0, 31), np.zeros(31), np.zeros(31))
    front0 = body_corners(0.0, 0.0, 0.0, LF, LR)[0][:, 0].max()
    assert clearance(r, LF, LR, (front0 + 3.0 + 0.5, -1.0, 4.0, 2.0)) == pytest.approx(0.5, abs=1e-9)
    assert clearance(r, LF, LR, (front0 + 2.0, -1.0, 4.0, 2.0)) == 0.0      # drives into it
    assert required_gap_ahead(r, LF, LR) == pytest.approx(3.0)              # never leaves the lane: the whole path


def test_catalogue_is_complete_and_consistent():
    cat = catalog()
    assert {e.category for e in cat.values()} == set(CATEGORIES)
    assert set(PARKING_SCENARIOS) <= set(cat)
    from vehicle_model.simulation.validation.runner import ValidationMatrixRunner
    import vehicle_model.simulation.four_wheel_steering_scenarios as fws
    from vehicle_model.scenario_book.catalog import ENTRY_TITLES
    assert all(hasattr(fws, f) for f in ENTRY_TITLES)
    assert all(hasattr(ValidationMatrixRunner, "run_" + n.removeprefix("val_")) for n in cat if n.startswith("val_"))
    md = book_markdown()
    assert all(f"`{n}`" in md for n in cat)


def test_parameters_are_validated():
    e = catalog()["split_mu_matrix"]
    assert e.resolve({"pedal": 0.3, "v_kph": 80})["v_kph"] == 80.0
    with pytest.raises(KeyError):
        e.resolve({"speed": 3})
    with pytest.raises(TypeError):
        e.resolve({"pedal": "full"})


def test_cli_list_info_and_errors(capsys):
    assert book_main(["list", "--category", "parking"]) == 0
    assert "u_turn" in capsys.readouterr().out
    assert book_main(["info", "u_turn"]) == 0
    assert book_main(["info", "nope"]) == 1
    assert book_main(["run", "u_turn", "--set", "speed=3"]) == 1
    assert book_main(["run"]) == 1


@pytest.mark.slow
def test_rear_axle_steering_needs_less_space_at_walking_pace():
    for name in ("u_turn", "perpendicular_parking", "tight_alley"):
        r4, r2, ext = run_parking_comparison(SevenDOFModel.from_yaml, name)
        assert r4.metrics.stability_status == r2.metrics.stability_status == "PASS"
        assert np.min(r4.steer_r_deg * np.sign(r4.steer_f_deg + 1e-12)) < -7.0   # counter-phase, about -8 deg
        assert np.all(r2.steer_r_deg == 0.0)
        assert ext["4WS"]["extent_y"] < ext["2WS"]["extent_y"] - 0.2, name
    r4, r2, _ = run_parking_comparison(SevenDOFModel.from_yaml, "u_turn")
    assert abs(r4.metrics.final_heading_dev_deg - r2.metrics.final_heading_dev_deg) < 5.0   # same task done


@pytest.mark.slow
def test_scenario_book_runs_an_entry_into_its_directory(tmp_path):
    s = run_entry("tight_alley", tmp_path, gif=False)
    assert (tmp_path / "tight_alley.png").exists()
    saved = json.loads((tmp_path / "summary.json").read_text())
    assert saved["results"]["4WS"]["extent_y"] == pytest.approx(s["results"]["4WS"]["extent_y"])


def test_generated_scenario_book_document_is_up_to_date():
    from pathlib import Path
    doc = Path(__file__).resolve().parents[2] / "docs" / "35_scenario_book.md"
    assert doc.read_text(encoding="utf-8") == book_markdown() + "\n", \
        "regenerate: python -m vehicle_model.scenario_book book --write docs/35_scenario_book.md"


def test_all_image_links_in_the_documentation_resolve():
    import re
    from pathlib import Path
    root = Path(__file__).resolve().parents[2]
    broken = []
    for md in [root / "README.md", *sorted((root / "docs").glob("*.md"))]:
        for link in re.findall(r"!\[[^\]]*\]\(([^)\s]+)\)", md.read_text(encoding="utf-8")):
            if not link.startswith("http") and not (md.parent / link).exists():
                broken.append(f"{md.name}: {link}")
    assert not broken, broken
