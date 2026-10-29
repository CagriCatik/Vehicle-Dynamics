"""Split-mu braking across actuator configurations (Plan.md V3.0 M10 section 5.5, G05).

Expected physical failures are passing tests: a configuration that cannot reduce the
brake asymmetry must spin, and the model must show that.
"""

from __future__ import annotations

import pytest

from vehicle_model.simulation.four_wheel_steering.split_mu_matrix import CONFIGURATIONS, matrix_table, run_split_mu_matrix
from vehicle_model.vehicle_models import SevenDOFModel


def plant(road):
    return SevenDOFModel.from_yaml(road=road)


@pytest.fixture(scope="module")
def split_full():
    return {r.configuration: r for r in run_split_mu_matrix(plant, mu_low=0.2, pedal=1.0)}


@pytest.mark.slow
def test_all_configurations_brake_straight_on_a_uniform_road():
    for r in run_split_mu_matrix(plant, mu_low=1.0, pedal=1.0, duration=1.5):
        assert r.stable and abs(r.kpis["final_heading_deg"]) < 0.1, r.configuration
        assert r.peak_brake_yaw_moment < 1.0                    # symmetric plant: no brake yaw moment


@pytest.mark.slow
def test_configurations_without_brake_reduction_spin_as_physically_expected(split_full):
    for name, (_, yaw_by) in CONFIGURATIONS.items():
        r = split_full[name]
        if yaw_by in ("none", "addition"):
            assert not r.stable and abs(r.kpis["final_heading_deg"]) > 45.0, name   # correct limit, not a defect
        else:
            assert r.stable, name


@pytest.mark.slow
def test_rear_steering_lacks_the_authority_against_the_brake_yaw_moment(split_full):
    for name in ("4WS", "4WS + ABS"):
        r = split_full[name]
        assert r.peak_steer_yaw_moment < 0.3 * r.peak_brake_yaw_moment, name


@pytest.mark.slow
def test_allocation_stabilises_by_reducing_the_brake_asymmetry(split_full):
    alloc, passive = split_full["TV (allocation)"], split_full["passive"]
    assert alloc.peak_brake_yaw_moment < 0.4 * passive.peak_brake_yaw_moment
    assert alloc.kpis["max_abs_slip"] < 0.1                   # slip control keeps the wheels rolling
    assert 1.5 < alloc.kpis["mean_deceleration"] < passive.kpis["mean_deceleration"]   # the price: deceleration
    assert "| TV (allocation) | 1 | allocation | yes |" in matrix_table(list(split_full.values()))
