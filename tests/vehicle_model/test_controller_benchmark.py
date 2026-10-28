"""M6 acceptance: controllers compared on the same plant, identical scenarios and defined KPIs."""

from __future__ import annotations

import math

import numpy as np
import pytest

from vehicle_model.controllers.motion_control import MotionController
from vehicle_model.controllers.yaw_laws import INPUTS_4WS_TV, LQRYawLaw, YawRatePI
from vehicle_model.handling_analysis import linear_params_from_seven_dof
from vehicle_model.simulation.controller_benchmark import (
    Scenario,
    compute_kpis,
    kpi_table,
    run_benchmark,
    run_scenario,
    sine_with_dwell,
    sine_with_dwell_scenario,
    step_steer_scenario,
)
from vehicle_model.vehicle_models import SevenDOFModel

VEHICLE = "configs/vehicles/reference_passenger_car.yaml"
KC = "configs/suspension/kc_reference_passenger_car.yaml"
MF = "magic_formula_passenger_synthetic"


def plant(road):
    return SevenDOFModel.from_yaml(VEHICLE, tyre_name=MF, tyre_rear_name=MF, kc_config=KC, road=road)


def test_sine_with_dwell_signal():
    A = math.radians(8.0)
    steer, t_end = sine_with_dwell(A, frequency=0.7, dwell=0.5, t_start=0.5)
    assert steer(0.4) == 0.0
    assert steer(0.5 + 0.25 / 0.7) == pytest.approx(A)          # first peak
    assert steer(0.5 + 0.75 / 0.7 + 0.25) == -A                  # dwell
    assert t_end == pytest.approx(0.5 + 1.0 / 0.7 + 0.5)
    assert steer(t_end + 0.1) == 0.0
    t = np.linspace(0, t_end + 1, 5001)
    assert np.max(np.abs(np.diff([steer(x) for x in t]))) < 0.01  # continuous


def test_kpis_on_a_synthetic_history():
    sc = Scenario("x", None, 20.0, 3.0, None,
                  kpis=("yaw_rate_rms_error", "yaw_rate_ratio_1s", "yaw_rate_ratio_1_75s", "lateral_displacement",
                        "stopping_distance", "max_sideslip_deg"),
                  params={"t_end": 1.0, "t_start": 0.0})
    t = np.linspace(0.005, 3.0, 600)
    r = np.where(t <= 1.0, 0.4 * np.sin(np.pi * t), 0.1 * np.exp(-(t - 1.0)))
    h = {"t": t, "r": r, "r_ref": r + 0.01, "beta": np.full_like(t, math.radians(2.0)), "psi": t * 0,
         "Y": 2.0 * t, "X": 10 * t, "Mz_allocated": t * 0, "computation_ms": np.ones_like(t),
         "status": np.array(["nominal"] * len(t), dtype=object)}
    k = compute_kpis(h, sc)
    assert k["yaw_rate_rms_error"] == pytest.approx(0.01)
    assert k["yaw_rate_ratio_1s"] == pytest.approx(0.1 * math.exp(-1.0) / 0.4, rel=1e-3)
    assert k["yaw_rate_ratio_1_75s"] == pytest.approx(0.1 * math.exp(-1.75) / 0.4, rel=1e-3)
    assert k["lateral_displacement"] == pytest.approx(2.14, rel=1e-3)
    assert k["stopping_distance"] == pytest.approx(30.0) and k["max_sideslip_deg"] == pytest.approx(2.0)
    assert k["status_counts"] == {"nominal": 600}


def test_runs_are_reproducible_and_independent():
    """Same controller before and after another one on fresh plants: identical results."""
    sc = step_steer_scenario(duration=0.4)
    ctrls = {"pi": lambda p: MotionController(p, YawRatePI()), "passive": None,
             "pi again": lambda p: MotionController(p, YawRatePI())}
    a, passive, b = run_benchmark(plant, ctrls, [sc])
    assert a.controller == b.controller == "PI (TV)" and passive.controller == "passive"
    for key in ("r", "Y", "Mz_allocated"):
        assert np.array_equal(a.history[key], b.history[key])
    assert a.kpis["yaw_rate_rms_error"] < passive.kpis["yaw_rate_rms_error"]
    assert "| PI (TV) |" in kpi_table([a, passive])


@pytest.mark.slow
def test_sine_with_dwell_passive_spins_controllers_meet_the_fmvss_like_limits():
    src = SevenDOFModel.from_yaml(VEHICLE, tyre_name=MF, tyre_rear_name=MF)
    design = linear_params_from_seven_dof(src.params, src.tyre_front, src.tyre_rear)
    ctrls = {
        "passive": None,
        "pi": lambda p: MotionController(p, YawRatePI()),
        "gs-lqr": lambda p: MotionController(p, LQRYawLaw(design, INPUTS_4WS_TV, design_speed=None,
                                                          schedule_speeds=(5, 10, 15, 20, 25, 30, 40))),
    }
    passive, *controlled = run_benchmark(plant, ctrls, [sine_with_dwell_scenario()])
    assert passive.kpis["yaw_rate_ratio_1s"] > 0.35                 # unstable without control
    for r in controlled:
        assert r.kpis["yaw_rate_ratio_1s"] <= 0.35 and r.kpis["yaw_rate_ratio_1_75s"] <= 0.20
        assert r.kpis["lateral_displacement"] >= 1.83
        assert r.kpis["max_sideslip_deg"] < 10.0
