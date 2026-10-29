"""Split-mu braking across actuator configurations (Plan.md V3.0 M10 section 5.5, G05).

Expected physical failures are passing tests: a configuration that cannot reduce the
brake asymmetry must spin, and the model must show that.
"""

from __future__ import annotations

import numpy as np
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


@pytest.mark.slow
def test_additive_esc_holds_only_at_the_moderate_brake_level_of_the_validation_matrix():
    """The validation's split-mu case brakes with 1450 N m in total (about 12 % pedal); legacy IVMC passes there.

    The same controller on the same road spins from about 20 % pedal on: validation and
    matrix agree, they test different brake levels.
    """
    from vehicle_model.simulation.four_wheel_steering.split_mu_matrix import CONFIGURATIONS as C
    cfg = {"IVMC (legacy)": C["IVMC (legacy)"]}
    pedal_validation = 1450.0 / (4 * SevenDOFModel.from_yaml().params.max_brake_torque)
    moderate = run_split_mu_matrix(plant, pedal=pedal_validation, duration=3.5, configurations=cfg)[0]
    firm = run_split_mu_matrix(plant, pedal=0.2, duration=3.5, configurations=cfg)[0]
    assert pedal_validation == pytest.approx(0.121, abs=1e-3)
    assert abs(moderate.kpis["final_heading_deg"]) < 15.0            # the validation threshold
    assert abs(firm.kpis["final_heading_deg"]) > 45.0


def test_ivmc_uses_the_shared_allocation_by_default():
    from vehicle_model.controllers.framework import ControlTarget
    from vehicle_model.controllers.integrated_controller import IntegratedChassisController
    from vehicle_model.core.types import SevenDOFState
    p = SevenDOFModel.from_yaml().params
    assert IntegratedChassisController(p).actuation == "allocation"
    assert IntegratedChassisController.from_yaml(params=p).actuation == "allocation"
    assert IntegratedChassisController.from_yaml(params=p, actuation="legacy").actuation == "legacy"
    with pytest.raises(ValueError):
        IntegratedChassisController(p, actuation="additive")
    out = IntegratedChassisController(p).compute_control(SevenDOFState(vx=25.0), ControlTarget(driver_brake_pedal=0.5),
                                                         dt=0.005)
    assert out.diagnostics["actuation"] == "allocation" and out.diagnostics["Fx_demand"] < 0
    assert np.all(out.T_brake <= p.max_brake_torque + 1e-9)


def test_ivmc_yaw_priority_is_scheduled_over_speed():
    from vehicle_model.controllers.integrated_controller import IntegratedChassisController
    c = IntegratedChassisController(SevenDOFModel.from_yaml().params)
    assert c.yaw_priority(3.0) == pytest.approx(1.0) and c.yaw_priority(25.0) == pytest.approx(100.0)
    assert c.yaw_priority(11.5) == pytest.approx(10.0)          # log-linear: geometric mean at mid-speed
    with pytest.raises(ValueError):
        IntegratedChassisController(SevenDOFModel.from_yaml().params, yaw_priority_speeds=(15.0, 8.0))
