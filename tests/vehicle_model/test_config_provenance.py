"""Configuration provenance checks."""

from pathlib import Path

import pytest
import yaml

from vehicle_model.config.loader import get_configs_root
from vehicle_model.core.types import SevenDOFParams
from vehicle_model.tire_model import available_tyres, load_tyre

CONFIGS = get_configs_root()
BUNDLED_TYRES = Path(__file__).resolve().parents[2] / "src" / "vehicle_model" / "config" / "tyres"
VEHICLE_DATA_CLASSES = {"analytical", "synthetic", "derived_from_assumptions", "optimised_virtual"}


@pytest.mark.parametrize("path", sorted((CONFIGS / "vehicles").glob("*.yaml")), ids=lambda p: p.stem)
def test_vehicle_config_has_id_version_provenance(path):
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    assert data["id"] == path.stem
    assert str(data["version"])
    prov = data["provenance"]
    assert prov["data_class"] in VEHICLE_DATA_CLASSES
    assert prov["source"].strip()


@pytest.mark.parametrize("path", sorted((CONFIGS / "vehicles").glob("*.yaml")), ids=lambda p: p.stem)
def test_vehicle_metadata_does_not_leak_into_params(path):
    params = SevenDOFParams.from_yaml(path)
    assert params.mass > 0.0
    assert params.wheelbase == pytest.approx(params.lf + params.lr)


@pytest.mark.parametrize("name", available_tyres())
def test_synthetic_tyres_do_not_claim_calibration(name):
    prov = load_tyre(name).provenance
    if prov.data_class == "synthetic":
        assert "calibrated" not in prov.source.lower()


def test_bundled_default_config_matches_configs():
    """The package fallback default.yaml must not drift from configs/default.yaml."""
    bundled = BUNDLED_TYRES.parent / "default.yaml"
    assert bundled.read_text(encoding="utf-8") == (CONFIGS / "default.yaml").read_text(encoding="utf-8")


@pytest.mark.parametrize("path", sorted(BUNDLED_TYRES.glob("*.yaml")), ids=lambda p: p.stem)
def test_bundled_tyre_copies_match_configs(path):
    """The package fallback copies must not drift from configs/tyres (single source of truth)."""
    master = CONFIGS / "tyres" / path.name
    assert master.is_file(), f"{path.name} exists only as bundled copy"
    assert path.read_text(encoding="utf-8") == master.read_text(encoding="utf-8")


@pytest.mark.parametrize("path", sorted((CONFIGS / "vehicles").glob("*.yaml")), ids=lambda p: p.stem)
def test_vehicle_configs_pass_consistency_checks(path):
    from vehicle_model.vehicle_models.consistency import check_vehicle_consistency

    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    assert check_vehicle_consistency(SevenDOFParams.from_yaml(path), data.get("chassis_extended")) == []


def test_consistency_check_reports_violations():
    import dataclasses

    from vehicle_model.vehicle_models.consistency import check_vehicle_consistency

    p = SevenDOFParams.from_yaml(CONFIGS / "vehicles" / "reference_passenger_car.yaml")
    assert any("dynamic_index" in i for i in check_vehicle_consistency(dataclasses.replace(p, Iz=5000.0)))
    assert any("k_roll_f" in i for i in check_vehicle_consistency(p, {"roll_stiffness_front": 1.0, "roll_stiffness_rear": 1.0}))
    assert any("unsprung" in i for i in check_vehicle_consistency(p, {"sprung_mass": 1000.0, "unsprung_mass_per_wheel": 10.0}))
    assert check_vehicle_consistency(dataclasses.replace(p, mass=-1.0)) == ["mass must be positive (got -1.0)"]


def test_seven_dof_tyre_selection_order():
    from vehicle_model.vehicle_models import SevenDOFModel

    ref = SevenDOFModel.from_yaml(str(CONFIGS / "vehicles" / "reference_passenger_car.yaml"))
    assert ref.tyre_front.provenance.formulation.startswith("Fiala")  # from the YAML 'tyres' section
    old = SevenDOFModel.from_yaml(str(CONFIGS / "vehicles" / "seven_dof_passenger.yaml"))
    assert old.tyre_front.provenance.formulation.startswith("Magic Formula 5.2")  # YAML section (since controller library)
    assert SevenDOFModel().tyre_rear.provenance.formulation.startswith("Magic Formula 5.2")  # code default
    explicit = SevenDOFModel.from_yaml(
        str(CONFIGS / "vehicles" / "reference_passenger_car.yaml"),
        tyre_name="linear_passenger_synthetic",
        tyre_rear_name="linear_passenger_synthetic",
    )
    assert explicit.tyre_rear.provenance.formulation == "Linear"
