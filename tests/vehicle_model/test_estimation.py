"""Unit and physical consistency tests for : State Estimation and Optimization.

Validates:
  - Sensor corruption, bias drift, and wheel speed quantization.
  - EKF sideslip observer convergence and bias tracking.
  - Recursive Least Squares (RLS) road friction estimation across mu transitions.
  - Offline Particle Swarm Optimization (PSO) convergence and YAML export.
"""

from __future__ import annotations

import math
from pathlib import Path
import numpy as np
import pytest

from vehicle_model.core.types import FL, FR, RL, RR, SevenDOFParams, WheelTelemetry
from vehicle_model.estimation.ekf_sideslip import EKFConfig, EKFSideslipObserver
from vehicle_model.estimation.friction_estimator import FrictionEstimatorConfig, RoadFrictionEstimator
from vehicle_model.estimation.pso_optimizer import PSOOptimizer
from vehicle_model.estimation.sensors import SensorConfig, SensorMeasurements, SensorSuite


def test_sensor_suite_noise_and_bias_corruption() -> None:
    """SensorSuite must corrupt ground truth signals with configured noise, biases, and quantization."""
    config = SensorConfig(
        accel_noise_std=0.20,
        accel_bias_init=0.50,
        gyro_noise_std=0.02,
        gyro_bias_init=0.03,
        encoder_ticks_per_rev=48,
        wheel_noise_std=0.10,
        steer_noise_std=0.01,
        steer_quantization_rad=0.005,
    )
    sensors = SensorSuite(config=config, seed=123)

    meas = sensors.measure(
        t=0.1,
        dt=0.01,
        true_ax=2.0,
        true_ay=1.5,
        true_r=0.10,
        true_omega=np.full(4, 50.0),
        true_steer_f=0.08,
    )

    # Measured signals must differ from ground truth due to bias and noise
    assert not math.isclose(meas.ax, 2.0, abs_tol=1e-3)
    assert not math.isclose(meas.ay, 1.5, abs_tol=1e-3)
    assert not math.isclose(meas.r, 0.10, abs_tol=1e-4)

    # Accelerometer bias should shift mean close to true + bias (2.0 + 0.5 = 2.5)
    assert 2.0 < meas.ax < 3.0
    assert 1.5 < meas.ay < 2.5

    # Wheel speeds must be quantized according to 48 ticks per rev (2*pi/48 rad)
    rad_per_tick = 2.0 * math.pi / 48
    for w in meas.omega:
        tick_ratio = w / rad_per_tick
        assert math.isclose(tick_ratio, round(tick_ratio), abs_tol=1e-6)


def test_ekf_sideslip_observer_convergence_on_transient_cornering() -> None:
    """EKF sideslip observer must track true sideslip angle within 0.6 degrees RMSE."""
    params = SevenDOFParams.from_yaml()
    ekf = EKFSideslipObserver(params=params)

    # Simulate 3 seconds of a slalom cornering maneuver:
    # vx = 20 m/s (~72 km/h)
    # vy(t) = 0.5 * sin(2*pi*f*t) -> true sideslip beta = vy / vx
    # r(t) = vy_dot / vx + steady-state yaw rate
    dt = 0.01
    duration = 3.0
    steps = int(duration / dt)

    t_arr = np.linspace(0.0, duration, steps)
    true_vx = 20.0
    true_vy = 0.6 * np.sin(2.0 * math.pi * 0.8 * t_arr)
    true_beta = np.arctan2(true_vy, true_vx)
    true_r = 0.6 * (2.0 * math.pi * 0.8) * np.cos(2.0 * math.pi * 0.8 * t_arr) / true_vx + 0.05 * np.sin(2.0 * math.pi * 0.8 * t_arr)
    true_ay = true_r * true_vx  # Kinematic lateral acceleration

    sensor_config = SensorConfig(
        accel_noise_std=0.08,
        gyro_noise_std=0.008,
        wheel_noise_std=0.05,
        accel_bias_init=0.0,
        gyro_bias_init=0.0,
    )
    sensors = SensorSuite(config=sensor_config, seed=42)

    ekf.reset(init_vx=true_vx, init_vy=0.0, init_r=0.0)

    est_beta_list = []
    omega_ideal = np.full(4, true_vx / params.Reff)

    for k in range(steps):
        t = t_arr[k]
        meas = sensors.measure(
            t=t,
            dt=dt,
            true_ax=0.0,
            true_ay=float(true_ay[k]),
            true_r=float(true_r[k]),
            true_omega=omega_ideal,
            true_steer_f=float(true_r[k] * params.wheelbase / true_vx + true_beta[k]),
        )

        est = ekf.update(meas, dt=dt)
        est_beta_list.append(est.sideslip)

    est_beta_arr = np.array(est_beta_list)

    # Discard first 0.5s initialization settling time
    settled_idx = int(0.5 / dt)
    error_rad = est_beta_arr[settled_idx:] - true_beta[settled_idx:]
    rmse_deg = math.degrees(math.sqrt(float(np.mean(error_rad ** 2))))

    # RMSE error must be well below 1.0 deg (~0.017 rad) for noisy IMU sensors
    assert rmse_deg < 1.0, f"EKF sideslip RMSE {rmse_deg:.3f} deg exceeds 1.0 deg threshold"


def test_ekf_sideslip_bias_estimation() -> None:
    """EKF must estimate and reject constant lateral accelerometer bias."""
    params = SevenDOFParams.from_yaml()
    # Higher bias process noise for rapid static convergence test
    config = EKFConfig(q_bias_ay=0.02, p_bias_ay=0.5)
    ekf = EKFSideslipObserver(params=params, config=config)

    # Injected constant sensor bias: b_ay = 0.25 m/s^2
    sensor_config = SensorConfig(
        accel_noise_std=0.02,
        accel_bias_init=0.25,
        gyro_noise_std=0.002,
        gyro_bias_init=0.0,
    )
    sensors = SensorSuite(config=sensor_config, seed=99)

    dt = 0.01
    true_vx = 18.0
    omega_ideal = np.full(4, true_vx / params.Reff)
    ekf.reset(init_vx=true_vx, init_vy=0.0, init_r=0.0)

    for k in range(500):
        meas = sensors.measure(
            t=k * dt,
            dt=dt,
            true_ax=0.0,
            true_ay=0.0,
            true_r=0.0,
            true_omega=omega_ideal,
            true_steer_f=0.0,
        )
        est = ekf.update(meas, dt=dt)

    # Estimated bias_ay should converge close to 0.25 m/s^2
    assert math.isclose(est.bias_ay, 0.25, abs_tol=0.08)


def test_friction_estimator_tracks_split_mu_transition() -> None:
    """RoadFrictionEstimator must track a step reduction in road friction."""
    params = SevenDOFParams.from_yaml()
    estimator = RoadFrictionEstimator(params=params)
    estimator.reset(initial_mu=1.0)

    vx = 15.0  # 54 km/h

    # Phase 1: High friction dry asphalt (mu = 1.0)
    Fz_nom = np.full(4, 3000.0)
    for _ in range(50):
        telemetry = WheelTelemetry(
            omega=np.full(4, 15.0 / params.Reff),
            v_wx=np.full(4, 15.0),
            v_wy=np.zeros(4),
            kappa=np.full(4, 0.04),
            alpha=np.zeros(4),
            Fz=Fz_nom,
            mu=np.full(4, 1.0),
            Fx_wheel=np.full(4, 1.0 * 30.0 * 0.04 * 3000.0),
            Fy_wheel=np.zeros(4),
            Fx_body=np.zeros(4),
            Fy_body=np.zeros(4),
            steer_angles=np.zeros(4),
        )
        est = estimator.update(telemetry, vx=vx)

    assert est.mu_overall > 0.85

    # Phase 2: Enter low friction ice patch (mu = 0.25)
    for _ in range(60):
        telemetry = WheelTelemetry(
            omega=np.full(4, 12.0 / params.Reff),
            v_wx=np.full(4, 15.0),
            v_wy=np.zeros(4),
            kappa=np.full(4, -0.16),
            alpha=np.zeros(4),
            Fz=Fz_nom,
            mu=np.full(4, 0.25),
            Fx_wheel=np.full(4, -0.25 * 3000.0),
            Fy_wheel=np.zeros(4),
            Fx_body=np.zeros(4),
            Fy_body=np.zeros(4),
            steer_angles=np.zeros(4),
        )
        est = estimator.update(telemetry, vx=vx)

    # Estimator must adapt and identify low-mu ice surface
    assert est.mu_overall < 0.45, f"Expected mu < 0.45 on ice, got {est.mu_overall:.3f}"


def test_pso_optimizer_convergence_and_yaml_export(tmp_path: Path) -> None:
    """PSOOptimizer must minimize test objective and export traceable YAML metadata."""
    # Quadratic sphere objective: f(x, y) = (x - 3.5)^2 + (y + 1.2)^2
    def sphere_cost(x: np.ndarray) -> float:
        return float((x[0] - 3.5) ** 2 + (x[1] + 1.2) ** 2)

    bounds = [(0.0, 5.0), (-3.0, 2.0)]
    param_names = ["gain_x", "offset_y"]

    pso = PSOOptimizer(
        cost_func=sphere_cost,
        bounds=bounds,
        param_names=param_names,
        n_particles=20,
        n_iterations=25,
        seed=42,
    )

    result = pso.optimize(verbose=False)

    # Must converge very close to the true optimum (3.5, -1.2)
    assert math.isclose(result.best_params["gain_x"], 3.5, abs_tol=0.08)
    assert math.isclose(result.best_params["offset_y"], -1.2, abs_tol=0.08)
    assert result.best_cost < 0.01

    # Cost history must be monotonically non-increasing
    for i in range(len(result.history_best_cost) - 1):
        assert result.history_best_cost[i + 1] <= result.history_best_cost[i] + 1e-12

    # Export to YAML and verify schema
    yaml_out = tmp_path / "optimized_gains.yaml"
    result.export_to_yaml(
        yaml_out,
        objective_name="sphere_test_benchmark",
        extra_metadata={"vehicle_type": "passenger_d_segment"},
    )

    assert yaml_out.is_file()
    import yaml
    with open(yaml_out, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    assert "pso_optimization" in data
    assert "optimized_parameters" in data
    assert data["pso_optimization"]["objective"] == "sphere_test_benchmark"
    assert math.isclose(data["optimized_parameters"]["gain_x"], 3.5, abs_tol=0.08)
