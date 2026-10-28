"""M5 acceptance: slip control (ABS/TCS), split-mu braking, torque vectoring, actuator faults.

All scenarios use the reference car with the Magic Formula tyre set (MF) so that
the peak slip depends on the road friction as for a physical tyre.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from vehicle_model.allocation import STATUS_INFEASIBLE, ActuatorHealth
from vehicle_model.controllers.framework import ControlTarget, VehicleController
from vehicle_model.controllers.slip_control import SlipControlConfig, WheelSlipController
from vehicle_model.controllers.stability_control import AllocatedStabilityController
from vehicle_model.core.types import SevenDOFControlInput, SevenDOFState
from vehicle_model.roads.surface import MuSplitRoad, UniformRoad
from vehicle_model.vehicle_models import SevenDOFModel

VEHICLE = "configs/vehicles/reference_passenger_car.yaml"
KC = "configs/suspension/kc_reference_passenger_car.yaml"
MF = "magic_formula_passenger_synthetic"
DT = 0.005


def _model(road, kc=None):
    return SevenDOFModel.from_yaml(VEHICLE, tyre_name=MF, tyre_rear_name=MF, road=road, kc_config=kc)


def _straight_braking(mu, controller=None, v0=25.0):
    """Full brake torque on all wheels; returns (braking efficiency 20->5 m/s, slips)."""
    m = _model(UniformRoad(mu=mu))
    p = m.params
    s = SevenDOFState(vx=v0, omega=np.full(4, v0 / p.Reff))
    tel, v, k = None, [], []
    while s.vx > 3.0:
        Tb = np.full(4, p.max_brake_torque)
        if controller is not None:
            Tb = controller.update(Tb, tel, s.vx, DT)
        s, tel = m.step(s, SevenDOFControlInput(T_brake=Tb), DT)
        v.append(s.vx)
        k.append(tel.kappa.copy())
    v, k = np.array(v), np.array(k)
    i0, i1 = np.argmax(v < 20.0), np.argmax(v < 5.0)
    return (v[i0] - v[i1]) / ((i1 - i0) * DT) / (mu * p.g), k[i0:i1]


# ----------------------------------------------------------------------------- slip control
@pytest.mark.parametrize("mu", [1.0, pytest.param(0.5, marks=pytest.mark.slow), pytest.param(0.2, marks=pytest.mark.slow)])
def test_abs_tracks_target_slip_and_brakes_near_the_limit(mu):
    locked, k_locked = _straight_braking(mu)
    ctrl = WheelSlipController(SlipControlConfig(mode="brake"))
    eff, k = _straight_braking(mu, ctrl)
    assert np.median(k_locked) < -0.9                         # without ABS the wheels lock
    target = -np.interp(mu, ctrl.config.mu_table, ctrl.config.slip_table)
    assert np.median(k) == pytest.approx(target, abs=0.15 * abs(target) + 0.002)
    assert np.std(k[:, 0]) < 0.01                              # no limit cycle
    assert eff > 0.95 and eff > locked + 0.25
    assert np.allclose(ctrl.mu_hat[:2], mu, rtol=0.05)          # estimate adapted up from 0.1


@pytest.mark.slow
def test_overestimated_friction_is_not_corrected_downwards_but_still_beats_locking():
    ctrl = WheelSlipController(SlipControlConfig(mode="brake", initial_mu=1.0))
    eff, _ = _straight_braking(0.2, ctrl)
    locked, _ = _straight_braking(0.2)
    assert locked + 0.1 < eff < 0.9


def test_tcs_limits_spin_on_low_friction():
    m = _model(UniformRoad(mu=0.2))
    p = m.params
    for use_tcs in (False, True):
        ctrl = WheelSlipController(SlipControlConfig(mode="drive"))
        s = SevenDOFState(vx=5.0, omega=np.full(4, 5.0 / p.Reff))
        tel, kappa = None, []
        for _ in range(400):
            Td = np.full(4, p.max_drive_torque)
            if use_tcs:
                Td = ctrl.update(Td, tel, s.vx, DT)
            s, tel = m.step(s, SevenDOFControlInput(T_drive=Td), DT)
            kappa.append(tel.kappa.copy())
        if not use_tcs:
            v_free, k_free = s.vx, np.median(kappa[-100:])
    assert k_free > 0.5                                    # spinning without TCS
    assert np.median(kappa[-100:]) == pytest.approx(np.interp(0.2, ctrl.config.mu_table, ctrl.config.slip_table), abs=0.01)
    assert s.vx > v_free                                   # and accelerates better


def test_slip_controller_passes_through_without_telemetry_or_at_low_speed():
    c = WheelSlipController()
    T = np.array([100.0, 200.0, 300.0, 400.0])
    assert np.array_equal(c.update(T, None, 20.0, DT), T)
    with pytest.raises(ValueError):
        c.update(T, None, 20.0, 0.0)


# ----------------------------------------------------------------------------- stability control
def test_controller_implements_the_vehicle_controller_protocol():
    assert isinstance(AllocatedStabilityController(_model(UniformRoad()).params), VehicleController)


def _run(model, controller, target_fn, duration, v0, stop_speed=None):
    p = model.params
    s = SevenDOFState(vx=v0, omega=np.full(4, v0 / p.Reff))
    tel, t, outs = None, 0.0, []
    while t < duration and (stop_speed is None or s.vx > stop_speed):
        tgt = target_fn(t, s)
        if controller is None:
            u = SevenDOFControlInput(steer_f=tgt.driver_steer_f,
                                     T_drive=np.full(4, tgt.driver_accel_pedal * p.max_drive_torque),
                                     T_brake=np.array([0.3, 0.3, 0.2, 0.2]) * 4 * p.max_brake_torque * tgt.driver_brake_pedal)
        else:
            out = controller.compute_control(s, tgt, tel, DT)
            outs.append(out)
            u = SevenDOFControlInput(steer_f=out.steer_f, T_drive=out.T_drive, T_brake=out.T_brake)
        s, tel = model.step(s, u, DT)
        t += DT
    return s, t, outs


@pytest.mark.slow
def test_split_mu_braking_stays_stable_where_the_passive_car_spins():
    road = MuSplitRoad(mu_left=1.0, mu_right=0.2, split_y=0.0, transition_width=0.05)
    brake = lambda t, s: ControlTarget(driver_brake_pedal=1.0)
    s_p, _, _ = _run(_model(road), None, brake, 15.0, 25.0, stop_speed=1.0)
    m = _model(road)
    s_c, t_c, outs = _run(m, AllocatedStabilityController(m.params), brake, 15.0, 25.0, stop_speed=1.0)
    assert abs(math.degrees(s_p.psi)) > 45.0                  # passive: spins
    assert abs(math.degrees(s_c.psi)) < 1.0 and abs(s_c.Y) < 0.5
    select_low = 25.0**2 / (2 * 0.2 * m.params.g)           # both sides at mu = 0.2
    assert s_c.X < select_low                                 # at least as good as select-low
    assert all(o.status == STATUS_INFEASIBLE for o in outs[10:])  # full pedal exceeds the grip: reported


@pytest.mark.slow
def test_torque_vectoring_reduces_understeer_towards_the_reference():
    delta, v0 = 0.04, 20.0

    def target(t, s):
        return ControlTarget(driver_steer_f=delta, driver_accel_pedal=max(0.08 + 0.05 * (v0 - s.vx), 0.0))

    s_p, _, _ = _run(_model(UniformRoad(), KC), None, target, 3.0, v0)
    m = _model(UniformRoad(), KC)
    ctrl = AllocatedStabilityController(m.params)
    s_c, _, outs = _run(m, ctrl, target, 3.0, v0)
    r_ref = ctrl.reference_yaw_rate(delta, s_c.vx, 1.0)
    assert s_p.r < 0.75 * r_ref                               # K&C car understeers strongly
    assert s_c.r == pytest.approx(r_ref, rel=0.05)
    assert abs(s_c.vx - v0) < 1.0 and outs[-1].desired_yaw_moment > 0


def test_failed_brake_is_compensated_and_reported():
    failed = ActuatorHealth(brake_ok=(False, True, True, True))
    brake = lambda t, s: ControlTarget(driver_brake_pedal=0.5)
    m = _model(UniformRoad())
    p = m.params
    s0 = SevenDOFState(vx=25.0, omega=np.full(4, 25.0 / p.Reff))
    # passive car with the FL brake failed
    s, tel = s0, None
    for _ in range(300):
        s, tel = m.step(s, SevenDOFControlInput(T_brake=np.array([0.0, 0.3, 0.2, 0.2]) * 4 * p.max_brake_torque * 0.5), DT)
    assert abs(math.degrees(s.psi)) > 20.0
    m = _model(UniformRoad())
    s_c, _, outs = _run(m, AllocatedStabilityController(p, health=failed), brake, 1.5, 25.0)
    assert abs(math.degrees(s_c.psi)) < 1.0
    assert all(o.T_brake[0] == 0.0 for o in outs)
    assert outs[-1].status == STATUS_INFEASIBLE              # 0.5 pedal is not achievable yaw-neutrally


def test_allocator_limits_must_not_exceed_vehicle_limits():
    from dataclasses import replace

    from vehicle_model.allocation import AllocatorConfig
    from vehicle_model.controllers.stability_control import StabilityControlConfig
    p = _model(UniformRoad()).params
    with pytest.raises(ValueError):
        AllocatedStabilityController(p, StabilityControlConfig(allocator=AllocatorConfig(max_brake_torque=p.max_brake_torque + 1)))
