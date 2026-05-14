from __future__ import annotations

from types import SimpleNamespace

import numpy as np

from vehicle_model.vehicle_models.config import vehicle_params_from_config
from vehicle_model.vehicle_models.data_loader import load_data
from vehicle_model.vehicle_models.dynamic_single_track import DynamicSingleTrack
from vehicle_model.vehicle_models.kinematic_single_track import KinematicSingleTrack
from vehicle_model.vehicle_models.validation_telemetry import build_validation_telemetry


def test_dynamic_model_uses_input_order_and_is_finite_at_standstill():
    model = DynamicSingleTrack()
    state = np.zeros(model.n_states)
    command = np.array([1.2, 0.1])

    dxdt = model.derivative_eqs(None, state, command, wheel_v=np.zeros(4), roll=0.0)

    assert np.isfinite(dxdt).all()
    assert dxdt[2] == command[0]
    assert dxdt[5] == command[1]


def test_dynamic_model_does_not_mutate_wheel_speed_input():
    model = DynamicSingleTrack()
    state = np.array([0.0, 0.0, 12.0, 0.2, 0.0, 0.04, 0.1])
    command = np.array([0.0, 0.0])
    wheel_v = np.array([12.5, 12.4, 12.3, 12.2])
    original = wheel_v.copy()

    _ = model.calc_slips(state, command, wheel_v)

    np.testing.assert_allclose(wheel_v, original)


def test_dynamic_model_treats_roll_as_radians_consistently():
    model = DynamicSingleTrack()
    state = np.array([0.0, 0.0, 20.0, 0.0, 0.0, 0.02, 0.0])
    command = np.array([0.0, 0.0])

    flat = model.derivative_eqs(None, state, command, roll=0.0)
    banked = model.derivative_eqs(None, state, command, roll=0.1)

    assert banked[3] < flat[3]


def test_kinematic_model_simulates_multistep_from_repo_root():
    model = KinematicSingleTrack()
    x, dxdt = model.sim_continuous(
        np.zeros(model.n_states),
        np.array([[1.0, 0.0], [1.0, 0.0]]),
        np.array([0.0, 0.1, 0.2]),
    )

    assert x.shape == (3, model.n_states)
    assert dxdt.shape == (3, model.n_states)
    assert np.isfinite(x).all()


def test_vehicle_params_can_be_overridden_by_yaml(tmp_path):
    override = tmp_path / "vehicle_override.yaml"
    override.write_text("vehicle:\n  mass: 900.0\n  max_acc: 8.0\n", encoding="utf-8")

    params = vehicle_params_from_config(str(override))

    assert params.mass == 900.0
    assert params.max_acc == 8.0


def test_load_data_uses_configured_schema_and_units(tmp_path):
    csv_path = tmp_path / "run.csv"
    data = np.array(
        [
            [0.0, 10.0, 20.0, 30.0, 40.0, 0.50, 0.06, 0.70, 1.20, 0.02, 100.0, 101.0, 102.0, 103.0, 0.03],
            [0.1, 11.0, 21.0, 31.0, 41.0, 0.55, 0.07, 0.75, 1.25, 0.03, 104.0, 105.0, 106.0, 107.0, 0.04],
        ]
    )
    np.savetxt(csv_path, data, delimiter=",")

    override = tmp_path / "schema_override.yaml"
    override.write_text("units:\n  kmh_to_mps: 0.5\n", encoding="utf-8")

    times, states, inputs, ekin_inputs = load_data(csv_path, config_path=override)

    np.testing.assert_allclose(times, [0.0, 0.1])
    np.testing.assert_allclose(states[:, 8], [50.0, 52.0])
    np.testing.assert_allclose(inputs[:, 0], [1.20, 1.25])
    np.testing.assert_allclose(ekin_inputs[:, 0], [1.20, 1.25])


def test_validation_telemetry_contains_dynamic_force_channels():
    params = vehicle_params_from_config()
    true_states = np.array(
        [
            [0.0, 0.0, 12.0, 0.2, 0.0, 0.04, 0.10],
            [0.5, 0.1, 12.2, 0.2, 0.1, 0.05, 0.11],
        ]
    )
    simulation = SimpleNamespace(
        veh_model="dynamic",
        N_SAMPLES=2,
        SAMPLING_TIME=0.04,
        times=np.array([0.0, 0.04]),
        true_states=true_states,
        states_pred=true_states.copy(),
        inputs=np.array([[0.2, 0.01], [0.1, 0.02]]),
        wheel_data=np.array([[12.0], [12.2]]),
        roll_data=np.array([0.0, 0.01]),
        vehicle=SimpleNamespace(params=params),
    )

    telemetry = build_validation_telemetry(simulation)

    assert telemetry.time.shape == (2,)
    assert np.isfinite(telemetry.signals["force_front_lateral"]).all()
    assert np.isfinite(telemetry.signals["force_longitudinal_net"]).all()
    assert np.isfinite(telemetry.signals["front_tire_utilization"]).all()
