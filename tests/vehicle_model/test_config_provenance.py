"""Configuration provenance checks (Plan.md, Abschnitt 4.2 and 7)."""

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


@pytest.mark.parametrize("path", sorted(BUNDLED_TYRES.glob("*.yaml")), ids=lambda p: p.stem)
def test_bundled_tyre_copies_match_configs(path):
    """The package fallback copies must not drift from configs/tyres (single source of truth)."""
    master = CONFIGS / "tyres" / path.name
    assert master.is_file(), f"{path.name} exists only as bundled copy"
    assert path.read_text(encoding="utf-8") == master.read_text(encoding="utf-8")
