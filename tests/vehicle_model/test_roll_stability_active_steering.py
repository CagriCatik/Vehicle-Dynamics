"""Roll stability control on the 14-DOF model and active front steering on the 7-DOF chain."""

from __future__ import annotations

import math

import numpy as np
import pytest

from vehicle_model.controllers.active_front_steering import SteeringOverlay, SteeringOverlayConfig
from vehicle_model.controllers.roll_stability import RollStabilityConfig, RollStabilitySupervisor

V14 = "configs/suspension/fourteen_dof.yaml"
VEH = "configs/vehicles/seven_dof_passenger.yaml"


# ----------------------------------------------------------------------------- RSC supervisor
def test_ltr_definition_and_contact():
    assert RollStabilitySupervisor.ltr([4000, 4000, 3000, 3000]) == 0.0
    assert RollStabilitySupervisor.ltr([8000, 0, 6000, 0]) == pytest.approx(1.0)
    assert RollStabilitySupervisor.ltr([0, 0, 0, 0]) == 0.0                      # no division by zero


def test_supervisor_modes_hysteresis_and_requests():
    c = RollStabilityConfig(ltr_on=0.7, ltr_off=0.5, t_min_active=0.1, t_release=0.05, t_pred=0.0)
    s = RollStabilitySupervisor(1500.0, 1.6, c)
    contact = [True] * 4
    left_turn = 6.0                                                                # a_y > 0
    r = s.update([3000, 3000, 2500, 2500], contact, left_turn, 0.005)
    assert r.mode == "inactive" and r.Fx is None and r.Mz is None
    r = s.update([500, 5500, 400, 4600], contact, left_turn, 0.005)            # LTR 0.82
    assert r.mode == "active" and r.Fx < 0 and r.Mz < 0 and s.activations == 1   # brake, understeering yaw moment
    assert s.update([500, 5500, 400, 4600], contact, -left_turn, 0.005).Mz > 0  # mirrored a_y: mirrored yaw moment
    r = s.update([500, 5500, 0, 5000], [True, True, False, True], left_turn, 0.005)
    assert r.mode == "emergency" and r.Fx < -c.brake_decel * 1500 * 9.81 + 1e-6 and "contact" in r.reason
    for _ in range(4):                                                         # back below ltr_off, too short
        r = s.update([2400, 3600, 2000, 3000], contact, left_turn, 0.005)
    assert r.mode == "active"                                                  # hysteresis and minimum time hold
    for _ in range(40):
        r = s.update([2900, 3100, 2400, 2600], contact, left_turn, 0.005)
    assert r.mode == "inactive" and s.activations == 1
    with pytest.raises(ValueError):
        RollStabilityConfig(ltr_on=0.4, ltr_off=0.5)


@pytest.mark.slow
def test_rsc_removes_two_wheel_lift_on_the_14dof_fishhook():
    from vehicle_model.simulation.roll_stability import run
    m = {"type": "fishhook", "speed_kph": 80, "amplitude_deg": 8}
    var = {"cg_height": 0.95, "sprung_mass": 1520}
    passive = run(V14, var, m, 1.0, "passive")["kpis"]
    rsc = run(V14, var, m, 1.0, "rsc")["kpis"]
    failed = run(V14, var, m, 1.0, "rsc_active_roll", "failed")["kpis"]
    # measured (benchmark roll_stability_fishhook_evasive): passive lifts two wheels, RSC keeps all four on the ground
    assert passive["two_wheel_lift_time_s"] > 0.05 and rsc["two_wheel_lift_time_s"] == 0.0
    assert rsc["ltr_max"] < passive["ltr_max"] and rsc["yaw_error_max_degps"] < passive["yaw_error_max_degps"]
    assert rsc["within_model_validity"] and passive["within_model_validity"]
    assert failed == rsc                                                       # failed active suspension: RSC through the brakes


# ----------------------------------------------------------------------------- AFS
def test_overlay_finalises_the_front_angle_once_with_limits():
    o = SteeringOverlay(SteeringOverlayConfig(max_overlay=math.radians(5.0), max_overlay_rate=math.radians(40.0)))
    delta, drv, ov = o.finalise(math.radians(2.0), 20.0, math.radians(10.0), math.radians(35.0), 0.01)
    assert ov == pytest.approx(math.radians(0.4))                              # rate-limited (40 deg/s * 10 ms)
    for _ in range(100):
        delta, drv, ov = o.finalise(math.radians(2.0), 20.0, math.radians(10.0), math.radians(35.0), 0.01)
    assert ov == pytest.approx(math.radians(5.0)) and delta == pytest.approx(math.radians(7.0))   # authority limit
    _, _, ov = o.finalise(math.radians(2.0), 20.0, math.radians(-10.0), math.radians(35.0), 0.01)
    assert ov == pytest.approx(math.radians(4.6))                              # sign follows the request, rate-limited
    big, _, _ = o.finalise(math.radians(34.0), 20.0, math.radians(5.0), math.radians(35.0), 1.0)
    assert big == pytest.approx(math.radians(35.0))                            # saturated at the steering limit
    failed = SteeringOverlay(SteeringOverlayConfig(failed=True, ratio_gains=(1.3, 1.0, 0.8)))
    d, _, ov = failed.finalise(math.radians(2.0), 30.0, math.radians(3.0), math.radians(35.0), 0.01)
    assert ov == 0.0 and d == pytest.approx(math.radians(2.0))                 # failed: fixed ratio, no overlay
    var = SteeringOverlayConfig(ratio_gains=(1.25, 1.0, 0.85))
    assert var.gain(0.0) == 1.25 and var.gain(35.0) == 0.85 and var.gain(25.0) == pytest.approx(0.925)


def test_afs_design_model_and_sign():
    from vehicle_model.controllers.yaw_laws import INPUTS_AFS, INPUTS_AFS_TV, LQRYawLaw, YawDesignModel
    from vehicle_model.handling_analysis import linear_params_from_seven_dof
    from vehicle_model.vehicle_models import SevenDOFModel
    m = SevenDOFModel.from_yaml(VEH)
    design = linear_params_from_seven_dof(m.params, m.tyre_front, m.tyre_rear)
    x, u = YawDesignModel(design, INPUTS_AFS).equilibrium(20.0, math.radians(2.0), 0.15)
    assert x[1] == pytest.approx(0.15) and u.shape == (1,)
    law = LQRYawLaw(design, INPUTS_AFS, design_speed=20.0)
    assert law.name == "LQR (AFS)" and LQRYawLaw(design, INPUTS_AFS_TV, design_speed=20.0).name == "LQR (AFS+TV)"
    from vehicle_model.controllers.motion_control import MotionContext
    from vehicle_model.core.types import SevenDOFState
    ctx = MotionContext(0.0, 0.2, 1.0, 0.01, 4000.0, 0.17, math.radians(5.0))
    req = law.request(SevenDOFState(vx=20.0, r=0.1), ctx)                     # too little yaw: steer further in
    assert req.front_steer > 0.0 and req.yaw_moment == 0.0


def test_spec_afs_rules():
    from vehicle_model.experiments.spec import parse_controller_comparison
    base = {"name": "x", "vehicle": VEH, "road": {"type": "uniform", "mu": 1.0},
            "manoeuvre": {"type": "step_steer", "speed_kph": 80}}
    ok = parse_controller_comparison({**base, "controllers": [
        {"law": "gs_lqr", "inputs": "afs", "steering": {"ratio": "variable"}},
        {"law": "gs_lqr", "inputs": "afs", "steering": {"afs_failed": True}}]})
    assert ok.controllers[0].label == "GS-LQR (AFS) [variable ratio]"
    assert parse_controller_comparison(ok.to_dict()).controllers == ok.controllers
    for ctrl, msg in (({"law": "pi", "inputs": "afs"}, "PI law|LQR"),
                      ({"law": "gs_lqr", "inputs": "tv", "steering": {"afs_failed": True}}, "needs inputs afs"),
                      ({"law": "passive", "steering": {"ratio": "variable"}}, "fixed ratio")):
        with pytest.raises(ValueError, match=msg):
            parse_controller_comparison({**base, "controllers": [ctrl]})


@pytest.mark.slow
def test_afs_against_2ws_and_failure(tmp_path):
    import yaml

    from vehicle_model.experiments.runner import run_experiment
    from vehicle_model.experiments.spec import parse_experiment
    cfg = yaml.safe_load(open("configs/experiments/active_front_steering_sine_with_dwell.yaml"))
    cfg["controllers"] = [{"law": "passive"}, {"law": "gs_lqr", "inputs": "afs"},
                          {"law": "gs_lqr", "inputs": "afs", "steering": {"afs_failed": True}}]
    res = {r["controller"]: r["kpis"] for r in run_experiment(parse_experiment(cfg, "<t>"), tmp_path, verify=False)["results"]}
    p, afs, failed = res["passive"], res["GS-LQR (AFS)"], res["GS-LQR (AFS) [AFS failed]"]
    # measured (benchmark active_front_steering_sine_with_dwell): the passive car spins in the sine with dwell, AFS alone holds it
    assert p["max_sideslip_deg"] > 30.0 and afs["max_sideslip_deg"] < 5.0
    assert afs["yaw_rate_ratio_1s"] < 0.35 and afs["yaw_rate_ratio_1_75s"] < 0.2
    assert afs["max_front_steer_deg"] < 6.0 + 1e-6                               # it steers against the driver (6 deg)
    assert failed["max_sideslip_deg"] == pytest.approx(p["max_sideslip_deg"], rel=1e-6)   # failed AFS = 2WS
