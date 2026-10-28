"""M7: observers without ground-truth access, sideslip estimation quality, Monte Carlo robustness."""

from __future__ import annotations

import inspect
import math

import numpy as np
import pytest

from vehicle_model.estimation.blended_sideslip import BlendedSideslipObserver, KinematicSideslipObserver
from vehicle_model.estimation.ekf_sideslip import EKFSideslipObserver
from vehicle_model.estimation.friction_estimator import RoadFrictionEstimator
from vehicle_model.roads.surface import UniformRoad
from vehicle_model.simulation.controller_benchmark import Scenario, sine_with_dwell, step_steer_scenario
from vehicle_model.simulation.estimation_study import run_estimation_study
from vehicle_model.simulation.robustness import Limit, Variation, run_monte_carlo
from vehicle_model.vehicle_models import SevenDOFModel

VEHICLE = "configs/vehicles/reference_passenger_car.yaml"
MF = "magic_formula_passenger_synthetic"


def _model(mu=1.0):
    return SevenDOFModel.from_yaml(VEHICLE, tyre_name=MF, tyre_rear_name=MF, road=UniformRoad(mu=mu))


# ----------------------------------------------------------------------------- information access
def test_observers_declare_their_information_access():
    p = _model().params
    for obs in (EKFSideslipObserver(p), KinematicSideslipObserver(p), BlendedSideslipObserver(p)):
        assert obs.information == "sensors_only"
        assert list(inspect.signature(obs.update).parameters) == ["meas", "dt"]
    assert RoadFrictionEstimator.information == "ground_truth_tyre_forces"


def test_study_rejects_observers_with_ground_truth_access():
    with pytest.raises(ValueError):
        run_estimation_study(_model(), {"friction": RoadFrictionEstimator(_model().params)}, lambda t: 0.0, 0.1, 20.0)


# ----------------------------------------------------------------------------- estimation quality
def _study(amp_deg, mu, duration=6.0):
    m = _model(mu)
    steer, _ = sine_with_dwell(math.radians(amp_deg))
    obs = {"ekf": EKFSideslipObserver(m.params), "kinematic": KinematicSideslipObserver(m.params),
           "blended": BlendedSideslipObserver(m.params)}
    return {k: r.metrics() for k, r in run_estimation_study(m, obs, steer, duration, 22.0, seed=3).items()}


@pytest.mark.slow
def test_model_based_observer_wins_in_the_linear_range_and_fails_at_the_limit():
    linear = _study(4.0, 1.0)
    assert linear["ekf"]["rms_deg"] < linear["kinematic"]["rms_deg"]
    limit = _study(4.0, 0.5)  # sliding on mu 0.5: linear tyre model invalid
    assert limit["ekf"]["rms_deg"] > 10.0
    assert limit["kinematic"]["rms_deg"] < 2.0


@pytest.mark.slow
@pytest.mark.parametrize("amp, mu", [(4.0, 1.0), (6.0, 1.0), (4.0, 0.5), (8.0, 1.0)])
def test_blended_observer_is_robust_in_and_beyond_the_linear_range(amp, mu):
    m = _study(amp, mu)
    assert m["blended"]["rms_deg"] < 1.3
    assert m["blended"]["rms_deg"] <= 1.1 * min(m["ekf"]["rms_deg"], m["kinematic"]["rms_deg"]) + 0.3


def test_estimation_study_is_repeatable():
    m1 = _study(4.0, 1.0, duration=1.0)
    m2 = _study(4.0, 1.0, duration=1.0)
    assert m1 == m2


# ----------------------------------------------------------------------------- Monte Carlo
def _plant_builder(sample):
    return lambda road: SevenDOFModel.from_yaml(VEHICLE, tyre_name=MF, tyre_rear_name=MF, road=UniformRoad(mu=sample["mu"]))


def test_variation_sampling_and_validation():
    rng = np.random.default_rng(0)
    v = Variation("normal", 1.0, 0.5, 0.9, 1.1)
    x = [v.sample(rng) for _ in range(200)]
    assert min(x) >= 0.9 and max(x) <= 1.1
    with pytest.raises(ValueError):
        Variation("uniform", 1.0, 0.5)
    with pytest.raises(ValueError):
        Limit("x", "==", 1.0)


def test_monte_carlo_is_repeatable_and_counts_violations():
    sc = step_steer_scenario(duration=0.3)
    variations = {"mu": Variation("uniform", 0.3, 1.0)}
    limit = Limit("max_sideslip_deg", "<=", 0.2)
    a = run_monte_carlo(_plant_builder, None, sc, variations, n=4, seed=7, limits=[limit])
    b = run_monte_carlo(_plant_builder, None, sc, variations, n=4, seed=7, limits=[limit])
    c = run_monte_carlo(_plant_builder, None, sc, variations, n=4, seed=8, limits=[limit])
    def physical(kpis):  # computation times are wall clock and not repeatable
        return [{k: v for k, v in kp.items() if not k.startswith("computation_ms")} for kp in kpis]

    assert a.samples == b.samples and a.samples != c.samples
    for x, y in zip(physical(a.kpis), physical(b.kpis)):
        assert x.keys() == y.keys()
        for k in x:
            assert x[k] == y[k] or (isinstance(x[k], float) and np.isnan(x[k]) and np.isnan(y[k])), k
    bad = [i for i, k in enumerate(a.kpis) if k["max_sideslip_deg"] > 0.2]
    v = a.violations["max_sideslip_deg <= 0.2"]
    assert v["samples"] == bad and v["rate"] == pytest.approx(len(bad) / 4)
    st = a.statistics["max_sideslip_deg"]
    assert st["min"] <= st["p5"] <= st["p50"] <= st["p95"] <= st["max"]
    assert len(a.table()) == 4 and "mu" in a.table()[0]


# ----------------------------------------------------------------------------- closed loop with an observer
def test_estimated_feedback_rejects_observers_with_ground_truth():
    from vehicle_model.controllers.estimated_feedback import EstimatedStateFeedback
    from vehicle_model.controllers.motion_control import MotionController
    from vehicle_model.controllers.yaw_laws import YawRatePI
    p = _model().params
    with pytest.raises(ValueError):
        EstimatedStateFeedback(MotionController(p, YawRatePI()), RoadFrictionEstimator(p))


@pytest.mark.slow
def test_yaw_control_with_sensors_and_observer_still_meets_the_limits():
    from vehicle_model.controllers.estimated_feedback import EstimatedStateFeedback
    from vehicle_model.controllers.motion_control import MotionController
    from vehicle_model.controllers.yaw_laws import YawRatePI
    from vehicle_model.simulation.controller_benchmark import run_scenario, sine_with_dwell_scenario
    plant = lambda road: SevenDOFModel.from_yaml(VEHICLE, tyre_name=MF, tyre_rear_name=MF,
                                                 kc_config="configs/suspension/kc_reference_passenger_car.yaml", road=road)
    ctrl = lambda p: EstimatedStateFeedback(MotionController(p, YawRatePI()), BlendedSideslipObserver(p), seed=5)
    k = run_scenario(plant, ctrl, sine_with_dwell_scenario()).kpis
    assert k["yaw_rate_ratio_1s"] <= 0.35 and k["yaw_rate_ratio_1_75s"] <= 0.20 and k["lateral_displacement"] >= 1.83
