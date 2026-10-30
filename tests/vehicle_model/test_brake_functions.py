"""EBD, CBC, IBC and regenerative slip control (MSR) on the shared chain."""

from __future__ import annotations

import math

import numpy as np
import pytest

from vehicle_model.actuators.brake_actuator import BrakeActuatorConfig, BrakeByWireActuator
from vehicle_model.allocation import WheelForceAllocator
from vehicle_model.controllers.brake_distribution import (BrakeDistributionConfig, BrakeDistributionStrategy,
                                                          brake_functions_config)
from vehicle_model.controllers.cornering_brake import CorneringBrakeConfig, CorneringBrakeControl
from vehicle_model.controllers.framework import ControlTarget
from vehicle_model.controllers.integrated_brake import IntegratedBrakeConfig, ibc_slip_config, slip_targets
from vehicle_model.controllers.motion_control import arbitrate_yaw
from vehicle_model.controllers.regenerative_slip import RegenSlipConfig, RegenSlipController
from vehicle_model.core.types import SevenDOFState
from vehicle_model.vehicle_models import SevenDOFModel
from vehicle_model.vehicle_models.normal_loads import wheel_contact_positions

VEH = "configs/vehicles/seven_dof_passenger.yaml"


@pytest.fixture(scope="module")
def p():
    return SevenDOFModel.from_yaml(VEH).params


@pytest.fixture(scope="module")
def alloc(p):
    return WheelForceAllocator(wheel_contact_positions(p), p.Reff)


FZ, MU = np.array([3600.0, 3600.0, 2400.0, 2400.0]), np.full(4, 1.0)


# ----------------------------------------------------------------------------- allocation extensions
def test_allocation_without_strategy_is_unchanged(alloc):
    a = alloc.allocate(-6000.0, 300.0, np.zeros(4), FZ, MU)
    b = alloc.allocate(-6000.0, 300.0, np.zeros(4), FZ, MU, share=None, brake_limit=None)
    assert np.array_equal(a.F, b.F) and a.status == b.status


def test_share_and_brake_limit_are_respected(alloc):
    share = np.array([0.3, 0.3, 0.2, 0.2])
    r = alloc.allocate(-4000.0, 0.0, np.zeros(4), FZ, MU, share=share)
    assert r.F / r.F.sum() == pytest.approx(share, rel=1e-3)               # demand met, split by the share
    lim = np.array([1e9, 1e9, 300.0, 300.0])
    r2 = alloc.allocate(-4000.0, 0.0, np.zeros(4), FZ, MU, brake_limit=lim)
    assert np.all(-r2.F[2:] <= 300.0 + 1e-6) and r2.achieved[0] == pytest.approx(-4000.0, rel=1e-3)
    for bad in (dict(share=np.array([1, 1, 0, 1.0])), dict(brake_limit=np.array([1, 1, -1, 1.0]))):
        with pytest.raises(ValueError):
            alloc.allocate(-4000.0, 0.0, np.zeros(4), FZ, MU, **bad)


def test_no_yaw_request_leaves_the_yaw_moment_free(alloc):
    steer = np.radians([3.0, 3.0, 0.0, 0.0])
    free = alloc.allocate(-30000.0, None, steer, FZ, MU)
    zero = alloc.allocate(-30000.0, 0.0, steer, FZ, MU)
    assert free.residual[1] == 0.0 and free.demand[1] == pytest.approx(free.achieved[1])
    assert free.achieved[0] <= zero.achieved[0] + 1e-6                     # no yaw constraint: at least as much braking


def test_yaw_arbitration_never_sums():
    assert arbitrate_yaw(None, None) == (None, "none")
    assert arbitrate_yaw(None, -500.0) == (-500.0, "cbc")
    assert arbitrate_yaw(-200.0, -500.0) == (-500.0, "cbc")
    assert arbitrate_yaw(-800.0, -500.0) == (-800.0, "law")
    assert arbitrate_yaw(300.0, -500.0) == (300.0, "law, cbc suppressed")
    assert arbitrate_yaw(300.0, None) == (300.0, "law")


# ----------------------------------------------------------------------------- EBD
def test_ebd_modes():
    assert BrakeDistributionStrategy(BrakeDistributionConfig("grip")).distribute(FZ, MU) == (None, None)
    share, lim = BrakeDistributionStrategy(BrakeDistributionConfig("static")).distribute(FZ, MU)
    assert share == pytest.approx([0.3, 0.3, 0.2, 0.2]) and lim is None
    mu = np.array([1.0, 0.2, 1.0, 0.2])
    share, lim = BrakeDistributionStrategy(BrakeDistributionConfig("ebd", rear_margin=0.8)).distribute(FZ, mu)
    assert share == pytest.approx(mu * FZ / np.sum(mu * FZ))
    assert lim[2:] == pytest.approx(0.8 * mu[2:] * FZ[2:]) and np.all(lim[:2] >= 1e8)
    with pytest.raises(ValueError):
        BrakeDistributionConfig("magic")


def test_ebd_pressure_hold_freezes_and_releases():
    s = BrakeDistributionStrategy(BrakeDistributionConfig("ebd", rear_margin=1.0))
    off, rear_abs = [False] * 4, [False, False, True, False]
    s.distribute(FZ, MU, off, np.array([1000.0, 1000.0, 900.0, 900.0]))
    _, l1 = s.distribute(FZ, MU, rear_abs, np.array([1000.0, 1000.0, 800.0, 900.0]))
    _, l2 = s.distribute(FZ, MU, rear_abs, np.array([1000.0, 1000.0, 500.0, 900.0]))     # ABS lowered the force
    assert l1[2] == 800.0 and l2[2] == 800.0                                           # frozen at onset, no ratchet
    _, l3 = s.distribute(FZ, MU, off, np.array([1000.0, 1000.0, 500.0, 900.0]))
    assert l3[2] == pytest.approx(2400.0)                                              # released


# ----------------------------------------------------------------------------- IBC
def test_ibc_slip_targets_come_from_the_tyre():
    from vehicle_model.tire_model import load_tyre
    tyre = load_tyre("magic_formula_passenger_synthetic")
    k, frac = slip_targets(tyre, 3500.0, (0.2, 0.5, 1.0))
    assert np.all(np.diff(k) > 0) and k == pytest.approx(0.107 * np.array([0.2, 0.5, 1.0]), rel=0.05)
    n = 5
    Fx, _ = tyre.forces(-np.full(n, k[2]), np.zeros(n), np.full(n, 3500.0), np.ones(n))
    Fx2, _ = tyre.forces(-np.full(n, k[2] * 1.2), np.zeros(n), np.full(n, 3500.0), np.ones(n))
    assert abs(Fx[0]) >= abs(Fx2[0])                                                   # the target is the peak
    k95, f95 = slip_targets(tyre, 3500.0, (1.0,), 0.95)
    assert k95[0] < k[2] and f95[0] == pytest.approx(0.95 * frac[2], rel=0.01)
    cfg = ibc_slip_config(IntegratedBrakeConfig())
    assert cfg.mode == "brake" and list(cfg.slip_table) == sorted(cfg.slip_table)


# ----------------------------------------------------------------------------- CBC
def test_cbc_requests_only_while_braking_in_a_turn(p):
    cbc = CorneringBrakeControl(p, CorneringBrakeConfig())
    turning = SevenDOFState(vx=22.0, r=0.35)                       # left turn, a_y = 7.7 m/s^2
    assert cbc.request(SevenDOFState(vx=22.0), 0.0, -8000.0) is None         # straight
    assert cbc.request(turning, 0.35, 0.0) is None                           # not braking
    mz = cbc.request(turning, 0.35, -8000.0, mu_hat=1.0)
    assert mz < 0.0                                                          # against the yaw increase
    clamp = cbc.request(turning, 0.35, -60000.0, mu_hat=0.5)                 # demand far above the road's capacity
    assert clamp == pytest.approx(0.5 * p.mass * (-0.5 * p.g) * p.hcog * (0.35 * 22.0) / p.g)
    over = cbc.request(SevenDOFState(vx=22.0, r=0.45), 0.35, -8000.0)        # oversteer adds feedback
    assert over < cbc.request(SevenDOFState(vx=22.0, r=0.45), 0.45, -8000.0)


# ----------------------------------------------------------------------------- MSR and the brake actuator
def test_brake_actuator_regen_extensions(p):
    cfg = BrakeActuatorConfig(ideal=True)
    a, b = BrakeByWireActuator(cfg), BrakeByWireActuator(cfg)
    cmd = np.array([300.0, 300.0, 300.0, 300.0])
    assert np.array_equal(a.step(cmd, 20.0, None, 0.005)[1], b.step(cmd, 20.0, None, 0.005, regen_limit=None, regen_only=None)[1])
    fr, rg = BrakeByWireActuator(cfg).step(np.zeros(4), 20.0, None, 0.005, regen_only=np.full(4, 200.0))
    assert np.allclose(fr, 0.0) and np.allclose(rg, 200.0)                   # coast drag never becomes friction
    fr, rg = BrakeByWireActuator(cfg).step(cmd, 20.0, None, 0.005, regen_limit=np.array([0, 100, 500, 500.0]))
    assert rg == pytest.approx([0, 100, 300, 300]) and fr + rg == pytest.approx(cmd)   # hand-over to friction


def test_regen_slip_controller(p):
    target = ControlTarget()
    cut = RegenSlipController(RegenSlipConfig(policy="cut_on_abs", coast_torque=250.0))
    assert cut.regen_limit([True, False, False, False], None, 20.0, 0.005) == pytest.approx([0, 500, 500, 500])
    assert cut.coast_request(target, 20.0) == pytest.approx([250.0] * 4)
    assert np.all(cut.coast_request(ControlTarget(driver_brake_pedal=0.2), 20.0) == 0.0)
    assert np.all(cut.coast_request(target, 1.0) == 0.0)
    mod = RegenSlipController(RegenSlipConfig(policy="modulated"))

    class Tel:
        Fz = np.full(4, 3000.0)
        Fx_wheel = Fy_wheel = np.zeros(4)
        kappa = np.array([-0.30, -0.01, -0.01, -0.01])               # FL far beyond the target slip
    caps = [mod.regen_limit(None, Tel, 20.0, 0.005) for _ in range(20)][-1]
    assert caps[0] < 100.0 and caps[1] == pytest.approx(500.0) and mod.active[0]


def test_brake_functions_config_is_strict(tmp_path):
    cfg = brake_functions_config()
    assert cfg["ebd"]["rear_margin"] == 0.85 and isinstance(cfg["cbc"], CorneringBrakeConfig)
    bad = tmp_path / "b.yaml"
    bad.write_text("ebd: {rear_margins: 0.9}\n")
    with pytest.raises(ValueError, match="unknown keys"):
        brake_functions_config(bad)


def test_spec_brake_block_rules():
    from vehicle_model.experiments.spec import parse_controller_comparison
    base = {"name": "x", "vehicle": VEH, "road": {"type": "uniform", "mu": 0.5},
            "manoeuvre": {"type": "straight_braking", "speed_kph": 80, "brake_pedal": 1.0}}
    ok = parse_controller_comparison({**base, "controllers": [
        {"law": "brakes", "brake": {"distribution": "ebd", "ibc": True}},
        {"law": "brakes", "actuators": "dynamic", "brake": {"regen": "modulated", "coast_torque": 300}}]})
    assert "[EBD, IBC]" in ok.controllers[0].label and parse_controller_comparison(ok.to_dict()).controllers == ok.controllers
    for ctrl, msg in (({"law": "passive", "brake": {"distribution": "ebd"}}, "60:40"),
                      ({"law": "brakes", "brake": {"regen": "modulated"}}, "actuators: dynamic"),
                      ({"law": "brakes", "actuators": "dynamic", "feedback": "sensor_constrained",
                        "brake": {"regen": "modulated"}}, "ideal wheel slip"),
                      ({"law": "brakes", "brake": {"distribution": "ebd", "abs": True}}, "unknown key")):
        with pytest.raises(ValueError, match=msg):
            parse_controller_comparison({**base, "controllers": [ctrl]})


# ----------------------------------------------------------------------------- closed loop
def _run(cfg_text, tmp_path):
    import yaml

    from vehicle_model.experiments.runner import run_experiment
    from vehicle_model.experiments.spec import parse_experiment
    spec = parse_experiment(yaml.safe_load(cfg_text), "<test>")
    res = run_experiment(spec, tmp_path, verify=False)["results"]
    if spec.sweep:
        return {(r["point"], r["controller"]): r["kpis"] for r in res}
    return {r["controller"]: r["kpis"] for r in res}


@pytest.mark.slow
def test_msr_prevents_wheel_lock_under_drag_torque(tmp_path):
    k = _run(f"""
name: msr
vehicle: {VEH}
road: {{type: uniform, mu: 0.2}}
manoeuvre: {{type: coast, speed_kph: 80, duration_s: 5.0}}
controllers:
  - {{law: brakes, actuators: dynamic, brake: {{regen: cut_on_abs, coast_torque: 300}}}}
  - {{law: brakes, actuators: dynamic, brake: {{regen: modulated, coast_torque: 300}}}}
outputs: {{formats: [json]}}
""", tmp_path)
    cut, mod = list(k.values())
    assert cut["wheel_lock_time_s"] > 2.0 and mod["wheel_lock_time_s"] == 0.0
    assert mod["regen_energy_kj"] > 3.0 * cut["regen_energy_kj"] and mod["mean_deceleration"] > cut["mean_deceleration"]
    assert mod["hydraulic_share"] == 0.0                          # drag torque is never replaced by friction


@pytest.mark.slow
def test_cbc_and_the_yaw_controller_in_braking_in_a_turn(tmp_path):
    k = _run(f"""
name: cbc
vehicle: {VEH}
road: {{type: uniform, mu: 1.0}}
manoeuvre: {{type: brake_in_turn, speed_kph: 80, steer_deg: 3.0, brake_pedal: 0.5, brake_start_s: 2.0, duration_s: 6.0}}
controllers:
  - {{law: brakes}}
  - {{law: brakes, brake: {{cbc: true}}}}
  - {{law: pi}}
  - {{law: pi, brake: {{cbc: true}}}}
outputs: {{formats: [json]}}
""", tmp_path)
    # measured (benchmark brake_functions_brake_in_turn, about 0.8 mu g before braking); not every case improves
    beta = lambda c: k[c]["max_sideslip_braking_deg"]
    stop = lambda c: k[c]["stopping_distance"]
    assert beta("brake functions") < 2.0                       # the reference car stays stable without yaw control
    assert beta("PI (TV) [CBC]") < beta("PI (TV)") and stop("PI (TV) [CBC]") < stop("PI (TV)")
    assert stop("brake functions [CBC]") > stop("brake functions")          # CBC alone only costs stopping distance
    assert abs(beta("brake functions [CBC]") - beta("brake functions")) < 0.1
