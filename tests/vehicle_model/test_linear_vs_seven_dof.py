"""Cross-model consistency: linear single track vs nonlinear 7-DOF.

In the small-signal range (a_y = 0.3 m/s^2, constant speed, symmetric vehicle) the
7-DOF model must reproduce the analytical linear reference. Remaining deviations
come from the documented reduction assumptions (track width, load transfer,
aerodynamics, speed-holding drive torque) and must stay small.
"""

import math

import numpy as np
import pytest

from vehicle_model.core.types import SevenDOFControlInput, SevenDOFState
from vehicle_model.handling_analysis import LinearSingleTrack, linear_params_from_seven_dof
from vehicle_model.vehicle_models import SevenDOFModel

REFERENCE = "configs/vehicles/reference_passenger_car.yaml"
AY_TARGET = 0.3  # [m/s^2]
DT = 0.005
DURATION = 4.0


def _simulate_step_steer(model: SevenDOFModel, vx0: float, delta: float):
    """Front steer step at t = 0; drive torque holds the speed (feed-forward + P)."""
    p = model.params
    state = SevenDOFState(vx=vx0, omega=np.full(4, vx0 / p.Reff))
    t, r, ay, beta = [], [], [], []
    for k in range(int(round(DURATION / DT))):
        f_res = 0.5 * p.rho * p.Cd * p.Af * state.vx**2 + p.c_rr * p.mass * p.g
        f_req = f_res + 2.0 * p.mass * (vx0 - state.vx)
        u = SevenDOFControlInput(steer_f=delta, T_drive=np.full(4, f_req * p.Reff / 4.0))
        state, tel = model.step(state, u, DT)
        t.append((k + 1) * DT)
        r.append(state.r)
        ay.append(tel.Fy_body.sum() / p.mass)
        beta.append(math.atan2(state.vy, state.vx))
    return np.array(t), np.array(r), np.array(ay), np.array(beta)


@pytest.fixture(scope="module")
def models():
    nl = SevenDOFModel.from_yaml(REFERENCE)
    lin = LinearSingleTrack(linear_params_from_seven_dof(nl.params, nl.tyre_front, nl.tyre_rear))
    return nl, lin


@pytest.mark.parametrize("vx", [10.0, 20.0, 30.0])
def test_steady_state_gains_match_linear_reference(models, vx):
    nl, lin = models
    delta = AY_TARGET / lin.lateral_accel_gain(vx)
    t, r, ay, beta = _simulate_step_steer(nl, vx, delta)
    final = t > DURATION - 0.5
    assert np.mean(r[final]) == pytest.approx(lin.yaw_rate_gain(vx) * delta, rel=0.01)
    assert np.mean(ay[final]) == pytest.approx(AY_TARGET, rel=0.01)
    # sideslip passes through zero near 12 m/s, so compare against the a_y-scaled magnitude
    beta_ref = lin.sideslip_gain(vx) * delta
    assert abs(np.mean(beta[final]) - beta_ref) < 0.02 * abs(lin.lateral_accel_gain(vx) * delta) / vx


@pytest.mark.parametrize("vx", [10.0, 20.0, 30.0])
def test_yaw_rate_transient_matches_linear_step_response(models, vx):
    nl, lin = models
    delta = AY_TARGET / lin.lateral_accel_gain(vx)
    t, r, _, _ = _simulate_step_steer(nl, vx, delta)
    r_lin = lin.state_space(vx).step_response(t, np.array([delta, 0.0]))[:, 0]
    r_ss = lin.yaw_rate_gain(vx) * delta
    assert np.max(np.abs(r - r_lin)) < 0.03 * r_ss


@pytest.mark.slow
def test_steady_state_circle_matches_linear_reference_at_small_ay(models):
    """ISO 4138 quasi-steady circle on the 7-DOF model vs the analytical steer demand.

    The linear reference is the a_y -> 0 limit; load transfer with load-sensitive tyres
    makes the 7-DOF understeer gradient grow with a_y (expected physical direction).
    """
    from vehicle_model.simulation.handling_limits import simulate_steady_state_circular_drive

    nl, lin = models
    R, g, L = 40.0, nl.params.g, nl.params.wheelbase
    h = simulate_steady_state_circular_drive(
        model=nl, radius_m=R, v_start_kph=20.0, v_end_kph=60.0, speed_ramp_rate_kph_per_s=1.5, dt=0.01
    )
    ay = h.vx * h.r
    r_actual = h.vx / np.maximum(h.r, 1e-6)

    small = (ay > 0.5) & (ay < 1.0)
    delta_lin = L / r_actual[small] + lin.understeer_gradient * ay[small]
    assert np.mean(h.steer_f[small] / delta_lin) == pytest.approx(1.0, abs=0.02)

    def k_us(lo, hi):
        s = (ay / g >= lo) & (ay / g <= hi)
        return np.polyfit(ay[s], h.steer_f[s] - L / r_actual[s], 1)[0]

    assert k_us(0.15, 0.3) > k_us(0.05, 0.15) > 0.0
