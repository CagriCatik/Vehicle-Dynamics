"""Mode C (sensor-constrained closed loop): no simulation telemetry reaches the controller (Plan V3.0 M10 5.4)."""

from __future__ import annotations

import dataclasses

import numpy as np
import pytest

from vehicle_model.controllers.estimated_feedback import INFORMATION_MODE_C, EstimatedStateFeedback
from vehicle_model.controllers.framework import ControlOutput, ControlTarget
from vehicle_model.controllers.motion_control import MotionController
from vehicle_model.controllers.yaw_laws import YawRatePI
from vehicle_model.core.types import SevenDOFControlInput, SevenDOFState
from vehicle_model.estimation.blended_sideslip import BlendedSideslipObserver
from vehicle_model.roads.surface import UniformRoad
from vehicle_model.vehicle_models import SevenDOFModel


class Spy:
    """Inner controller that records what it is given and brakes moderately."""

    name = "spy"

    def __init__(self, params):
        self.params = params
        self.seen = []

    def reset(self):
        self.seen.clear()

    def compute_control(self, state, target, telemetry=None, dt=0.01):
        self.seen.append(telemetry)
        return ControlOutput(steer_f=0.02, T_brake=np.full(4, 400.0))


def test_plant_telemetry_never_reaches_the_controller_in_mode_c():
    model = SevenDOFModel.from_yaml(road=UniformRoad(mu=0.6))
    p = model.params
    spy = Spy(p)
    ctrl = EstimatedStateFeedback(spy, BlendedSideslipObserver(p), seed=1, mode="C")
    s, tel = SevenDOFState(vx=20.0, omega=np.full(4, 20.0 / p.Reff)), None
    plant_tels = []
    for _ in range(200):
        out = ctrl.compute_control(s, ControlTarget(driver_brake_pedal=0.2), tel, 0.005)
        s, tel = model.step(s, SevenDOFControlInput(steer_f=out.steer_f, T_brake=out.T_brake), 0.005)
        plant_tels.append(tel)
    assert all(t is None or all(t is not pt for pt in plant_tels) for t in spy.seen)
    given = [t for t in spy.seen if t is not None]
    assert given, "the estimator should provide sensor telemetry after initialisation"
    last = given[-1]
    np.testing.assert_allclose(last.mu, ctrl.friction.mu)                         # the estimate, not the road
    assert not np.allclose(last.Fz, plant_tels[-2].Fz, rtol=1e-6)                  # quasi-static estimate, not the plant
    assert np.all(np.abs(last.kappa - plant_tels[-2].kappa) < 0.1)                 # but close to it
    assert out.diagnostics["information"] == INFORMATION_MODE_C


def test_mode_b_still_passes_the_telemetry_and_says_so():
    model = SevenDOFModel.from_yaml()
    p = model.params
    spy = Spy(p)
    ctrl = EstimatedStateFeedback(spy, BlendedSideslipObserver(p), seed=1, mode="B")
    s = SevenDOFState(vx=20.0, omega=np.full(4, 20.0 / p.Reff))
    _, tel = model.step(s, SevenDOFControlInput(), 0.005)
    out = ctrl.compute_control(s, ControlTarget(), tel, 0.005)
    assert spy.seen[-1] is tel and "telemetry" in out.diagnostics["information"]


def test_invalid_configurations_are_rejected():
    p = SevenDOFModel.from_yaml().params
    with pytest.raises(ValueError):
        EstimatedStateFeedback(Spy(p), BlendedSideslipObserver(p), mode="D")

    class NoParams:
        name = "x"

        def reset(self):
            pass

    with pytest.raises(ValueError):
        EstimatedStateFeedback(NoParams(), BlendedSideslipObserver(p), mode="C")


def test_experiment_spec_feedback_modes(tmp_path):
    from vehicle_model.experiments.spec import load_experiment
    import yaml

    base = yaml.safe_load(open("configs/experiments/split_mu_braking.yaml"))
    for ctrls, expect in (([{"law": "pi", "feedback": "sensor_constrained"}], "sensor_constrained"),
                          ([{"law": "pi", "estimated_state": True}], "estimated")):
        f = tmp_path / "x.yaml"
        f.write_text(yaml.safe_dump({**base, "controllers": ctrls}))
        spec = load_experiment(f)
        assert spec.controllers[0].feedback == expect
    for bad in ([{"law": "pi", "feedback": "magic"}], [{"law": "pi", "estimated_state": True, "feedback": "ideal"}],
                [{"law": "pi", "estimated_state": "C"}], [{"law": "passive", "feedback": "sensor_constrained"}]):
        f = tmp_path / "bad.yaml"
        f.write_text(yaml.safe_dump({**base, "controllers": bad}))
        with pytest.raises(ValueError):
            load_experiment(f)


@pytest.mark.slow
def test_mode_c_closed_loop_brakes_on_ice_and_holds_the_sine_with_dwell_limits():
    """Measured cost of Mode C (see docs/ESTIMATION_ROBUSTNESS.md): lower deceleration, but stable."""
    from vehicle_model.simulation.controller_benchmark import Scenario, run_scenario, sine_with_dwell_scenario

    plant = lambda road: SevenDOFModel.from_yaml(road=road)
    mode_c = lambda p: EstimatedStateFeedback(MotionController(p, YawRatePI()), BlendedSideslipObserver(p), seed=3,
                                              mode="C")
    ice = Scenario("ice", lambda: UniformRoad(mu=0.2), 25.0, 15.0, lambda t, s: ControlTarget(driver_brake_pedal=1.0),
                   kpis=("final_heading_deg", "mean_deceleration"), stop_speed=1.0)
    r = run_scenario(plant, mode_c, ice)
    assert abs(r.kpis["final_heading_deg"]) < 10.0 and r.kpis["mean_deceleration"] > 1.0
    sd = run_scenario(plant, mode_c, dataclasses.replace(sine_with_dwell_scenario(),
                                                         kpis=("yaw_rate_ratio_1s", "lateral_displacement")))
    assert sd.kpis["yaw_rate_ratio_1s"] < 0.35 and sd.kpis["lateral_displacement"] > 1.83
