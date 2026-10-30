"""One actuation path, the trace requested -> allocated -> issued -> realised, actuator dynamics.

Characterisation of the shared chain (MotionController / ActuationLayer / WheelForceAllocator) and of the
actuator chain that sits between controller and plant. Mode C (no plant telemetry) is covered by
``test_sensor_constrained_feedback.py``; the IVMC actuation paths by ``test_split_mu_matrix.py``.
"""

from __future__ import annotations

import dataclasses
import math

import numpy as np
import pytest

from vehicle_model.allocation import ActuatorHealth
from vehicle_model.controllers.framework import ControlOutput, ControlTarget, PassiveBaselineController
from vehicle_model.controllers.motion_control import ActuationLayer, MotionController
from vehicle_model.controllers.yaw_laws import YawRatePI
from vehicle_model.core.types import SevenDOFState
from vehicle_model.simulation.actuator_chain import ActuatorChain, ActuatorChainConfig, generalised_forces
from vehicle_model.simulation.controller_benchmark import (actuation_lag, run_scenario, sine_with_dwell_scenario,
                                                           split_mu_braking_scenario)
from vehicle_model.vehicle_models import SevenDOFModel

VEH = "configs/vehicles/seven_dof_passenger.yaml"


@pytest.fixture(scope="module")
def p():
    return SevenDOFModel.from_yaml(VEH).params


def plant(road):
    return SevenDOFModel.from_yaml(VEH, road=road)


class Fixed:
    """Controller that issues a fixed command (for actuator-chain unit tests)."""

    name = "fixed"

    def __init__(self, T_drive=(0, 0, 0, 0), T_brake=(0, 0, 0, 0), steer_f=0.0, steer_r=0.0, abs_active=None):
        self.cmd = (np.array(T_drive, float), np.array(T_brake, float), steer_f, steer_r)
        self.abs_active = abs_active

    def reset(self):
        pass

    def compute_control(self, state, target, telemetry=None, dt=0.01):
        d, b, sf, sr = self.cmd
        diag = {} if self.abs_active is None else {"abs_active": np.array(self.abs_active)}
        return ControlOutput(steer_f=sf, steer_r=sr, T_drive=d.copy(), T_brake=b.copy(), diagnostics=diag)


def state(v=20.0, p=None):
    return SevenDOFState(vx=v, omega=np.full(4, v / (p.Reff if p else 0.32)))


# ----------------------------------------------------------------------------- trace of the shared chain
def test_motion_controller_trace_stages_are_consistent(p):
    ctrl = MotionController(p, YawRatePI())
    tel_plant = plant(None)
    s = state(25.0, p)
    _, tel = tel_plant.step(s, ControlOutput(), 0.005)
    out = ctrl.compute_control(s, ControlTarget(driver_steer_f=0.05, driver_brake_pedal=0.3), tel, 0.005)
    tr = out.diagnostics["trace"]
    assert tr["requested"][1] == pytest.approx(out.desired_yaw_moment)
    assert tr["allocated"] == pytest.approx(ctrl.last_allocation.achieved.tolist())
    assert tr["issued"][1] == pytest.approx(out.allocated_yaw_moment)       # legacy name = issued yaw moment
    gf = generalised_forces(p, out.T_drive, out.T_brake, tel.steer_angles)
    assert tr["issued"] == pytest.approx(gf.tolist(), rel=1e-9, abs=1e-6)


def test_single_owner_of_the_wheel_torques(p):
    """The controller's output torques are exactly those of its ActuationLayer: no second, additive path."""
    ctrl = MotionController(p, YawRatePI())
    s = state(25.0, p)
    _, tel = plant(None).step(s, ControlOutput(), 0.005)
    out = ctrl.compute_control(s, ControlTarget(driver_steer_f=0.05, driver_brake_pedal=0.5), tel, 0.005)
    ref = ActuationLayer(p)
    T_d, T_b, *_ = ref.actuate(out.diagnostics["Fx_demand"], out.desired_yaw_moment, tel.steer_angles, tel.Fz, tel.mu,
                               tel.Fy_wheel, tel, s.vx, 0.005)
    assert np.allclose(out.T_drive, T_d) and np.allclose(out.T_brake, T_b)
    assert np.all(out.T_brake >= -1e-9) and np.all(out.T_drive >= -1e-9)
    from vehicle_model.controllers.integrated_controller import IntegratedChassisController
    assert IntegratedChassisController.from_yaml(params=p).actuation == "allocation"   # default


def test_failed_brake_issues_no_torque_and_the_others_take_over(p):
    s = state(25.0, p)
    _, tel = plant(None).step(s, ControlOutput(), 0.005)
    ok = ActuationLayer(p).actuate(-6000.0, 0.0, tel.steer_angles, tel.Fz, tel.mu, tel.Fy_wheel, tel, s.vx, 0.005)
    bad = ActuationLayer(p, health=ActuatorHealth(brake_ok=(True, False, True, True))).actuate(
        -6000.0, 0.0, tel.steer_angles, tel.Fz, tel.mu, tel.Fy_wheel, tel, s.vx, 0.005)
    T_b, alloc = bad[1], bad[2]
    # measured: FR 0, RR takes over up to its grip limit, FL/RL reduced so that M_z stays 0
    assert T_b[1] == 0.0 and T_b[3] > 1.5 * ok[1][3] and bad[2].grip_limited[3]
    assert T_b[0] < ok[1][0] and T_b[2] < ok[1][2]
    assert abs(alloc.achieved[1]) < 50.0                              # yaw moment kept (yaw has priority) ...
    assert alloc.residual[0] > 100.0 and alloc.status == "infeasible"  # ... the braking shortfall is reported, not hidden


# ----------------------------------------------------------------------------- actuator chain
def test_ideal_chain_is_a_bit_identical_pass_through(p):
    sc = sine_with_dwell_scenario(80 / 3.6)
    a = run_scenario(plant, lambda q: MotionController(q, YawRatePI()), sc)
    b = run_scenario(plant, lambda q: ActuatorChain(MotionController(q, YawRatePI()), q, ActuatorChainConfig(ideal=True)), sc)
    for k in ("X", "Y", "r", "beta", "steer_r", "Mz_allocated", "Mz_realised"):
        assert np.array_equal(a.history[k], b.history[k]), k
    # no actuators: issued = realised up to the steer angle used (controller: plant telemetry of the last step,
    # realised: the commanded steer of this step), measured <= 0.25 % of the peak
    scale = np.max(np.abs(a.history["Mz_realised"]))
    assert np.max(np.abs(a.history["Mz_issued"] - a.history["Mz_realised"])) < 0.005 * scale
    assert a.kpis["issued_to_realised_lag_ms"] == 0.0


def test_regen_enters_once_as_braking_torque(p):
    chain = ActuatorChain(Fixed(T_brake=(400, 400, 400, 400)), p, ActuatorChainConfig())
    s = state(20.0, p)
    for _ in range(200):
        out = chain.compute_control(s, ControlTarget(), None, 0.005)
    tr = out.diagnostics["trace"]
    fr, rg = np.array(tr["friction_torque"]), np.array(tr["regen_torque"])
    assert np.allclose(out.T_brake, fr + rg) and np.all(out.T_drive == 0.0)      # nothing in the drive channel
    assert rg.sum() > 1000.0 and np.allclose(fr + rg, 400.0, atol=1.0)           # blended, total as commanded
    assert tr["regen_power_w"] == pytest.approx(float(np.sum(rg * s.omega)))
    no_regen = ActuatorChainConfig(brake=dataclasses.replace(ActuatorChainConfig().brake, enable_regen_blending=False))
    chain2 = ActuatorChain(Fixed(T_brake=(400, 400, 400, 400)), p, no_regen)
    for _ in range(200):
        out2 = chain2.compute_control(s, ControlTarget(), None, 0.005)
    assert np.allclose(out2.diagnostics["trace"]["regen_torque"], 0.0) and np.allclose(out2.T_brake, 400.0, atol=1.0)


def test_abs_flag_drops_regen_on_that_wheel(p):
    chain = ActuatorChain(Fixed(T_brake=(400, 400, 400, 400), abs_active=(True, False, False, False)), p)
    for _ in range(100):
        out = chain.compute_control(state(20.0, p), ControlTarget(), None, 0.005)
    rg = out.diagnostics["trace"]["regen_torque"]
    assert rg[0] == pytest.approx(0.0, abs=1e-6) and rg[1] > 100.0


def test_dynamic_chain_lags_and_converges(p):
    chain = ActuatorChain(Fixed(T_drive=(0, 0, 600, 600), steer_f=0.1), p, ActuatorChainConfig())
    s = state(10.0, p)
    first = chain.compute_control(s, ControlTarget(), None, 0.005)
    assert 0.0 < first.T_drive[2] < 600.0 and 0.0 < first.steer_f < 0.1          # lag, not a jump
    for _ in range(400):
        out = chain.compute_control(s, ControlTarget(), None, 0.005)
    assert out.T_drive[2] == pytest.approx(600.0, rel=1e-3) and out.steer_f == pytest.approx(0.1, rel=1e-3)
    assert np.all(out.T_drive >= 0.0) and np.all(out.T_brake >= 0.0)


def test_chain_is_mirror_symmetric_and_replays_identically(p):
    def run(cmd_b, steer):
        c = ActuatorChain(Fixed(T_brake=cmd_b, steer_f=steer), p)
        return [c.compute_control(state(15.0, p), ControlTarget(), None, 0.005) for _ in range(50)][-1]
    a, m = run((300, 900, 100, 500), 0.05), run((900, 300, 500, 100), -0.05)
    assert np.allclose(a.T_brake, m.T_brake[[1, 0, 3, 2]]) and a.steer_f == pytest.approx(-m.steer_f)
    b = run((300, 900, 100, 500), 0.05)
    assert np.array_equal(a.T_brake, b.T_brake) and a.steer_f == b.steer_f


def test_actuation_lag_estimate():
    t = np.arange(0.0, 2.0, 0.005)
    issued = (np.sin(2 * np.pi * 1.5 * t) > 0).astype(float)
    realised = np.r_[np.zeros(6), issued[:-6]]                          # 30 ms transport delay
    assert actuation_lag(t, issued, realised) == pytest.approx(0.030)
    assert actuation_lag(t, issued, issued) == 0.0


# ----------------------------------------------------------------------------- experiment DSL
def test_spec_actuators_field_is_backward_compatible_and_strict():
    from vehicle_model.experiments.spec import parse_controller_comparison
    base = {"name": "x", "vehicle": VEH, "road": {"type": "uniform", "mu": 1.0},
            "manoeuvre": {"type": "step_steer", "speed_kph": 72}, "controllers": [{"law": "pi"}]}
    s = parse_controller_comparison(base)
    assert s.controllers[0].actuators == "ideal" and "actuators" not in s.to_dict()["controllers"][0]
    s2 = parse_controller_comparison({**base, "controllers": [{"law": "pi"}, {"law": "pi", "actuators": "dynamic"}]})
    assert s2.controllers[1].label.endswith("[actuator dynamics]")
    assert parse_controller_comparison(s2.to_dict()).controllers == s2.controllers
    with pytest.raises(ValueError, match="actuators"):
        parse_controller_comparison({**base, "controllers": [{"law": "pi", "actuators": "magic"}]})
    with pytest.raises(ValueError, match="duplicate"):
        parse_controller_comparison({**base, "controllers": [{"law": "pi"}, {"law": "pi"}]})


# ----------------------------------------------------------------------------- closed loop
@pytest.mark.slow
def test_split_mu_braking_with_dynamic_actuators(p):
    """Split-mu full braking, 90 km/h: PI keeps the heading with ideal and dynamic actuators, passive spins."""
    sc = split_mu_braking_scenario(25.0)
    dyn = ActuatorChainConfig.from_yaml()
    pi = run_scenario(plant, lambda q: ActuatorChain(MotionController(q, YawRatePI()), q, dyn), sc)
    pas = run_scenario(plant, lambda q: ActuatorChain(PassiveBaselineController(q), q, dyn), sc)
    assert abs(pi.kpis["final_heading_deg"]) < 1.0 and abs(pas.kpis["final_heading_deg"]) > 90.0
    assert 5.0 <= pi.kpis["issued_to_realised_lag_ms"] <= 60.0
    h = pi.history
    assert np.all(np.isfinite(h["Mz_realised"])) and h["T_realised_abs"].min() >= 0.0
