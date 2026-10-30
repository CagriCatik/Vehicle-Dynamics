"""Virtual eLSD, traction distribution, motor envelope and ITC on the shared chain."""

from __future__ import annotations

import math

import numpy as np
import pytest

from vehicle_model.allocation import ActuatorHealth, WheelForceAllocator
from vehicle_model.controllers.framework import ControlOutput, ControlTarget
from vehicle_model.controllers.integrated_traction import YawToleranceLaw
from vehicle_model.controllers.motion_control import MotionContext, MotionController, MotionRequest
from vehicle_model.controllers.traction_distribution import TractionDistributionConfig, TractionDistributionStrategy
from vehicle_model.controllers.yaw_laws import NoYawLaw, YawRatePI
from vehicle_model.core.types import SevenDOFState
from vehicle_model.vehicle_models import SevenDOFModel
from vehicle_model.vehicle_models.normal_loads import wheel_contact_positions

VEH = "configs/vehicles/seven_dof_passenger.yaml"
U = np.array([4000.0, 800.0, 3000.0, 600.0])          # [N] upper drive bounds: left high grip, right low


@pytest.fixture(scope="module")
def p():
    return SevenDOFModel.from_yaml(VEH).params


def test_coupling_modes(p):
    R = p.Reff
    free = TractionDistributionStrategy(TractionDistributionConfig("free"), R)
    assert free.drive_limit(U) is None
    op = TractionDistributionStrategy(TractionDistributionConfig("open"), R).drive_limit(U)
    assert op == pytest.approx([800, 800, 600, 600])                          # both at the weaker wheel
    el = TractionDistributionStrategy(TractionDistributionConfig("elsd", lock_torque=320.0), R).drive_limit(U)
    assert el == pytest.approx([800 + 320 / R, 800, 600 + 320 / R, 600])
    zero = TractionDistributionStrategy(TractionDistributionConfig("elsd", lock_torque=0.0), R).drive_limit(U)
    assert zero == pytest.approx(op)                                          # T_lock = 0 is the open coupling
    big = TractionDistributionStrategy(TractionDistributionConfig("elsd", lock_torque=1e6), R).drive_limit(U)
    assert big == pytest.approx(U)                                            # large T_lock is free
    assert np.all(el <= U + 1e-9)                                             # bounds are only ever lowered
    with pytest.raises(ValueError):
        TractionDistributionConfig("spool")


def test_motor_envelope(p):
    s = TractionDistributionStrategy(TractionDistributionConfig("free", envelope=True), p.Reff)
    c = s.drive_config
    slow, fast = s.drive_limit(np.full(4, 1e5), np.full(4, 10.0)), s.drive_limit(np.full(4, 1e5), np.full(4, 300.0))
    assert slow == pytest.approx(np.full(4, c.max_torque / p.Reff))
    assert fast == pytest.approx(np.full(4, c.max_power / 300.0 / p.Reff))     # power-limited at high wheel speed


def test_allocation_respects_the_drive_limit_and_creates_no_force(p):
    a = WheelForceAllocator(wheel_contact_positions(p), p.Reff)
    Fz, mu = np.array([3500, 3500, 2500, 2500.0]), np.array([1.0, 0.2, 1.0, 0.2])
    lim = TractionDistributionStrategy(TractionDistributionConfig("open"), p.Reff).drive_limit(a.bounds(Fz, mu, np.zeros(4))[1])
    r = a.allocate(8000.0, None, np.zeros(4), Fz, mu, drive_limit=lim)
    assert np.all(r.F <= lim + 1e-6) and r.F.sum() <= 8000.0 + 1e-6
    assert r.F[0] == pytest.approx(r.F[1], abs=1e-3) and r.status == "infeasible" and r.residual[0] < 0.0
    with pytest.raises(ValueError):
        a.allocate(8000.0, None, np.zeros(4), Fz, mu, drive_limit=np.array([1, 1, -1, 1.0]))


def test_yaw_tolerance_band(p):
    inner = YawRatePI()
    law = YawToleranceLaw(inner, math.radians(2.0))
    ctx = MotionContext(0.0, 0.0, 1.0, 0.01, 4000.0, p.max_steer_r)
    inside = law.request(SevenDOFState(vx=10.0, r=math.radians(1.0)), ctx)
    assert inside.yaw_moment is None and not law.passed and inner._integral != 0.0   # inner law kept running
    outside = law.request(SevenDOFState(vx=10.0, r=math.radians(3.0)), ctx)
    assert outside.yaw_moment is not None and outside.yaw_moment < 0.0 and law.passed
    law.reset()
    assert inner._integral == 0.0
    with pytest.raises(ValueError):
        YawToleranceLaw(YawRatePI(), -1.0)


def test_law_inactive_at_low_speed_is_no_yaw_request(p):
    """Below min_speed the chain has no yaw request (None): with one side's motors failed it can still drive."""
    m = SevenDOFModel.from_yaml(VEH)
    ctrl = MotionController(p, YawRatePI(), health=ActuatorHealth(drive_ok=(False, True, False, True)))
    s = SevenDOFState(vx=1.0, omega=np.full(4, 1.0 / p.Reff))
    _, tel = m.step(s, ControlOutput(), 0.005)
    out = ctrl.compute_control(s, ControlTarget(driver_accel_pedal=1.0), tel, 0.005)
    assert math.isnan(out.diagnostics["trace"]["requested"][1])
    assert out.T_drive[0] == 0.0 and out.T_drive[2] == 0.0 and out.T_drive[1] > 100.0 and out.T_drive[3] > 100.0


def test_spec_traction_block():
    from vehicle_model.experiments.spec import parse_controller_comparison
    base = {"name": "x", "vehicle": VEH, "road": {"type": "split_mu", "mu_left": 1.0, "mu_right": 0.2},
            "manoeuvre": {"type": "launch", "speed_kph": 5, "accel_pedal": 1.0}}
    ok = parse_controller_comparison({**base, "controllers": [
        {"law": "no_yaw", "traction": {"distribution": "elsd", "lock_torque": 250, "failed": ["FL"]}},
        {"law": "pi", "traction": {"distribution": "elsd", "envelope": True, "yaw_tolerance": 2.0, "driven": "rwd"}}]})
    assert parse_controller_comparison(ok.to_dict()).controllers == ok.controllers
    assert "ITC band 2.0 deg/s" in ok.controllers[1].label and "RWD" in ok.controllers[1].label
    for ctrl, msg in (({"law": "passive", "traction": {"distribution": "open"}}, "equal torques"),
                      ({"law": "no_yaw", "traction": {"yaw_tolerance": 2.0}}, "needs a yaw law"),
                      ({"law": "no_yaw", "traction": {"failed": ["XX"]}}, "failed"),
                      ({"law": "no_yaw", "traction": {"distribution": "spool"}}, "distribution")):
        with pytest.raises(ValueError, match=msg):
            parse_controller_comparison({**base, "controllers": [ctrl]})


def _run(text, tmp_path):
    import yaml

    from vehicle_model.experiments.runner import run_experiment
    from vehicle_model.experiments.spec import parse_experiment
    res = run_experiment(parse_experiment(yaml.safe_load(text), "<t>"), tmp_path, verify=False)["results"]
    return {r["controller"]: r["kpis"] for r in res}


SPLIT = """
name: t
vehicle: configs/vehicles/seven_dof_passenger.yaml
road: {{type: split_mu, mu_left: {l}, mu_right: {r}}}
manoeuvre: {{type: launch, speed_kph: 5, accel_pedal: 1.0, duration_s: 5.0}}
controllers:
  - {{law: no_yaw, actuators: dynamic, traction: {{distribution: open}}}}
  - {{law: pi, actuators: dynamic, traction: {{distribution: elsd, lock_torque: 300, envelope: true, yaw_tolerance: 2.0}}}}
outputs: {{formats: [json]}}
"""


@pytest.mark.slow
def test_itc_split_mu_launch_is_better_than_open_mirrored_and_without_extra_energy(tmp_path, p):
    a = _run(SPLIT.format(l=1.0, r=0.2), tmp_path / "a")
    b = _run(SPLIT.format(l=0.2, r=1.0), tmp_path / "b")
    op, itc = a["no yaw control [actuator dynamics] [open]"], next(v for k, v in a.items() if "ITC" in k)
    itc_m = next(v for k, v in b.items() if "ITC" in k)
    # measured (benchmark traction_split_mu_launch): ITC gains clearly more speed than the open coupling ...
    assert itc["speed_gain_kph"] > 1.2 * op["speed_gain_kph"]
    # ... and holds the yaw-rate error at its band (2 deg/s; small overshoot from the actuator lag)
    assert itc["max_yaw_rate_error_degps"] < 2.3
    # left/right mirror: same speed gain, opposite heading
    assert itc_m["speed_gain_kph"] == pytest.approx(itc["speed_gain_kph"], rel=0.01)
    assert np.sign(itc_m["final_heading_deg"]) == -np.sign(itc["final_heading_deg"]) != 0
    # no extra energy: mechanical drive work at the wheels >= kinetic energy gained
    for k in (op, itc):
        v0, v1 = 5 / 3.6, (5 + k["speed_gain_kph"]) / 3.6
        assert k["drive_energy_kj"] * 1e3 >= 0.5 * p.mass * (v1**2 - v0**2)
