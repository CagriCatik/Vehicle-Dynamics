"""Virtual sensor sampling, latency, lag and dropout."""

from __future__ import annotations

import dataclasses
import math

import numpy as np
import pytest

from vehicle_model.estimation.sensors import CHANNELS, SensorConfig, SensorMeasurements, SensorSuite

NOISELESS = SensorConfig(accel_noise_std=0.0, accel_bias_init=0.0, accel_bias_drift_std=0.0, gyro_noise_std=0.0,
                         gyro_bias_init=0.0, gyro_bias_drift_std=0.0, wheel_noise_std=0.0, quantize_wheel_speed=False,
                         steer_noise_std=0.0, steer_quantization_rad=0.0)
DT = 0.001


def _ramp(cfg, T=0.2):
    """Feed r(t) = t (and the same ramp on all channels); return (t, measured r, valid flags)."""
    s = SensorSuite(cfg, seed=1)
    out = []
    for k in range(int(round(T / DT))):
        t = k * DT
        m = s.measure(t, DT, t, t, t, np.full(4, t), t, t)
        out.append((t, m.r, m.valid))
    return out


def test_defaults_pass_the_truth_through_unchanged():
    for t, r, valid in _ramp(NOISELESS):
        assert r == pytest.approx(t, abs=1e-15) and all(valid.values())


def test_latency_delays_every_channel():
    cfg = dataclasses.replace(NOISELESS, latency=0.02)
    for t, r, _ in _ramp(cfg):
        if t >= 0.02:
            assert r == pytest.approx(t - 0.02, abs=1e-9)


def test_sample_and_hold_at_the_sensor_rate():
    cfg = dataclasses.replace(NOISELESS, sample_period=0.01)
    data = _ramp(cfg)
    for t, r, _ in data:
        assert r == pytest.approx(math.floor(t / 0.01 + 1e-9) * 0.01, abs=1e-9)
    assert len({r for _, r, _ in data}) == 20


def test_steering_lag_is_first_order():
    cfg = dataclasses.replace(NOISELESS, steer_lag_tau=0.05)
    s = SensorSuite(cfg)
    s.measure(0.0, DT, 0, 0, 0, np.zeros(4), 0.0)
    for k in range(1, 51):  # step to 0.1 rad for one time constant
        m = s.measure(k * DT, DT, 0, 0, 0, np.zeros(4), 0.1)
    assert m.steer_f == pytest.approx(0.1 * (1 - math.exp(-1.0)), rel=0.02)


def test_scheduled_outage_holds_the_last_value_and_flags_it():
    cfg = dataclasses.replace(NOISELESS, outages=(("r", 0.05, 0.10),))
    data = _ramp(cfg)
    held = [r for t, r, v in data if 0.05 <= t < 0.10]
    assert all(r == pytest.approx(0.049, abs=1e-12) for r in held)
    assert all(not v["r"] and v["ay"] for t, _, v in data if 0.05 <= t < 0.10)
    assert all(v["r"] for t, _, v in data if t >= 0.10)
    assert data[-1][1] == pytest.approx(data[-1][0])


def test_random_dropout_is_repeatable_and_has_the_configured_rate():
    cfg = dataclasses.replace(NOISELESS, dropout_probability=0.2)
    a = [v["omega"] for _, _, v in _ramp(cfg, T=2.0)]
    b = [v["omega"] for _, _, v in _ramp(cfg, T=2.0)]
    assert a == b
    assert 1 - np.mean(a) == pytest.approx(0.2, abs=0.03)


def test_invalid_configuration_rejected():
    with pytest.raises(ValueError):
        SensorSuite(dataclasses.replace(NOISELESS, latency=-0.01))
    with pytest.raises(ValueError):
        SensorSuite(dataclasses.replace(NOISELESS, outages=(("gps", 0.0, 1.0),)))


def test_measurement_container_holds_no_ground_truth_quantities():
    fields = {f.name for f in dataclasses.fields(SensorMeasurements)}
    assert fields == {"t", "ax", "ay", "r", "omega", "steer_f", "steer_r", "valid"}
    assert set(CHANNELS) == {"ax", "ay", "r", "omega", "steer"}
