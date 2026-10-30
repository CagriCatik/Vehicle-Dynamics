"""Road-friction estimation from virtual sensors only (G02 / state estimation Mode C)."""

from __future__ import annotations

import inspect

import numpy as np
import pytest

from vehicle_model.controllers.framework import ControlTarget
from vehicle_model.controllers.slip_control import WheelSlipController
from vehicle_model.controllers.stability_control import AllocatedStabilityController
from vehicle_model.core.types import SevenDOFControlInput, SevenDOFState
from vehicle_model.estimation.sensor_friction import SensorFrictionEstimator, brush_force
from vehicle_model.estimation.sensors import SensorSuite
from vehicle_model.roads.surface import MuSplitRoad, UniformRoad
from vehicle_model.tire_model import FialaTyre
from vehicle_model.vehicle_models import SevenDOFModel

VEHICLE = "configs/vehicles/reference_passenger_car.yaml"
MF = "magic_formula_passenger_synthetic"
DT = 0.005


def _model(road):
    return SevenDOFModel.from_yaml(VEHICLE, tyre_name=MF, tyre_rear_name=MF, road=road)


def _run(road, controller, T, v0=25.0, seed=1):
    """Plant with MF tyres, default sensors; the estimator sees sensors and commanded torques only."""
    m = _model(road)
    p = m.params
    s = SevenDOFState(vx=v0, omega=np.full(4, v0 / p.Reff))
    sens = SensorSuite(seed=seed)
    est = SensorFrictionEstimator(p)
    est.reset(init_vx=v0)                    # wheels roll freely before the manoeuvre
    tel, t, e = None, 0.0, None
    while t < T and s.vx > 4.0:
        Td, Tb = controller(s, tel, p)
        s_new, tel = m.step(s, SevenDOFControlInput(T_drive=Td, T_brake=Tb), DT)
        ax = (s_new.vx - s.vx) / DT - s.r * s.vy
        ay = (s_new.vy - s.vy) / DT + s.r * s.vx
        s, t = s_new, t + DT
        e = est.update(sens.measure(t, DT, ax, ay, s.r, s.omega, 0.0), Td, Tb, DT)
    return e, est


def _abs_braking():
    abs_ = WheelSlipController()
    return lambda s, tel, p: (np.zeros(4), abs_.update(np.full(4, p.max_brake_torque), tel, s.vx, DT))


# ----------------------------------------------------------------------------- model and interface
def test_declares_sensor_only_information_and_its_inputs():
    est = SensorFrictionEstimator(_model(UniformRoad()).params)
    assert est.information == "sensors_only"
    assert list(inspect.signature(est.update).parameters) == ["meas", "T_drive", "T_brake", "dt", "vy"]


def test_brush_force_equals_the_fiala_tyre_of_the_library():
    tyre = FialaTyre(c_kappa=18.0, c_alpha=16.0, stiffness_load_sensitivity=0.0, friction_load_sensitivity=0.0)
    Fz, mu = 4000.0, 0.7
    kappa = np.linspace(-0.4, 0.4, 41)
    Fx, _ = tyre.forces(kappa, 0.0 * kappa, Fz, mu)
    sx = kappa / np.maximum(1 + kappa, 1e-3)
    assert np.allclose(np.abs(Fx), brush_force(18.0 * Fz * np.abs(sx), mu * Fz), rtol=1e-12, atol=1e-9)


def test_inversion_recovers_mu_where_identifiable_and_reports_no_sensitivity_where_not():
    est = SensorFrictionEstimator(_model(UniformRoad()).params)
    Fz = np.full(4, 4000.0)
    for mu in (0.3, 0.8, 1.2):
        psi = np.array([1.0, 1.5, 2.0, 2.5]) * mu * Fz          # nonlinear range
        F = brush_force(psi, mu * Fz)
        mu_hat, sens = est._invert(F, psi, Fz)
        assert np.allclose(mu_hat, mu, atol=1e-6) and np.all(sens > 0.05)
        assert np.all(np.diff(sens) > 0)                        # information grows towards saturation
    psi = np.full(4, 0.02 * 4000.0)                              # deep in the linear range
    _, sens = est._invert(brush_force(psi, 0.8 * Fz), psi, Fz)
    assert np.all(sens < 0.01)


# ----------------------------------------------------------------------------- closed loop with the plant
def test_no_excitation_no_claim():
    cruise = lambda s, tel, p: (np.full(4, (0.5 * p.rho * p.Cd * p.Af * s.vx**2 + p.c_rr * p.mass * p.g) * p.Reff / 4), np.zeros(4))
    e, est = _run(UniformRoad(mu=0.3), cruise, T=1.5)
    assert np.allclose(e.mu_wheels, est.config.initial_mu) and e.confidence < 0.05


@pytest.mark.parametrize("mu", [pytest.param(1.0, marks=pytest.mark.slow), 0.6, pytest.param(0.3, marks=pytest.mark.slow)])
def test_abs_braking_identifies_the_road_friction(mu):
    e, est = _run(UniformRoad(mu=mu), _abs_braking(), T=2.5)
    assert np.all(est.confidence_per_wheel() > 0.9)
    assert np.allclose(e.mu_wheels[:2], mu, atol=0.05)          # front wheels
    # rear wheels: lightly loaded under braking, the MF tyre's peak friction exceeds mu there (load sensitivity)
    assert np.all((e.mu_wheels[2:] >= mu - 0.05) & (e.mu_wheels[2:] <= 1.25 * mu))


@pytest.mark.slow
@pytest.mark.parametrize("mu_left, mu_right", [(1.0, 0.3), (0.6, 0.2)])
def test_split_mu_is_detected_with_confidence_weighted_sides(mu_left, mu_right):
    road = MuSplitRoad(mu_left=mu_left, mu_right=mu_right, split_y=0.0, transition_width=0.05)
    ctrl = AllocatedStabilityController(_model(road).params)

    def controller(s, tel, p):
        out = ctrl.compute_control(s, ControlTarget(driver_brake_pedal=1.0), tel, DT)
        return out.T_drive, out.T_brake

    e, est = _run(road, controller, T=3.0, seed=2)
    assert e.is_split_mu
    assert e.mu_left == pytest.approx(mu_left, abs=0.15) and e.mu_right == pytest.approx(mu_right, abs=0.15)


def test_speed_reference_is_not_started_from_slipping_wheels():
    est = SensorFrictionEstimator(_model(UniformRoad()).params)
    from vehicle_model.estimation.sensors import SensorMeasurements
    z = SensorMeasurements(t=0.005, ax=-8.0, ay=0.0, r=0.0, omega=np.full(4, 50.0), steer_f=0.0, steer_r=0.0)
    est.update(z, np.zeros(4), np.full(4, 2000.0), DT)            # braking from the first sample, no init_vx
    assert est.v_ref is None and est.confidence_per_wheel().max() == 0.0
