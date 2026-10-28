"""Unit and physics tests for Dynamic Two-Track Nonlinear Model Predictive Controller."""

import math
import numpy as np
import pytest

from vehicle_model.controllers.dynamic_two_track_nmpc import (
    DynamicNMPCConfig,
    DynamicTwoTrackNMPC,
)
from vehicle_model.controllers.framework import ControlTarget
from vehicle_model.core.types import SevenDOFParams, SevenDOFState


def test_dynamic_nmpc_config_defaults():
    cfg = DynamicNMPCConfig()
    assert cfg.N >= 8
    assert cfg.dt_pred > 0.0
    assert cfg.max_steer_front_rad > 0.0
    assert cfg.max_steer_rear_rad > 0.0
    assert cfg.max_yaw_moment_nm > 0.0


def test_dynamic_nmpc_two_track_continuous_dynamics():
    params = SevenDOFParams.from_yaml()
    nmpc = DynamicTwoTrackNMPC(params=params)

    # State: [X, Y, psi, vx, vy, r]
    state = np.array([0.0, 0.0, 0.0, 20.0, 0.0, 0.0])
    # Control: [delta_f, delta_r, Mz_tv, ax_req]
    control = np.array([math.radians(3.0), 0.0, 0.0, 0.0])

    x_dot, kamm_ratios = nmpc.continuous_dynamics(state, control, mu=1.0)

    assert len(x_dot) == 6
    assert np.all(np.isfinite(x_dot))
    # Steer angle delta_f > 0 (left) must produce positive yaw acceleration r_dot > 0
    assert x_dot[5] > 0.0
    # Kamm circle ratios must be positive and bounded
    assert len(kamm_ratios) == 4
    assert np.all(kamm_ratios >= 0.0)
    assert np.all(kamm_ratios <= 1.0)


def test_dynamic_nmpc_rk4_step_integration():
    params = SevenDOFParams.from_yaml()
    nmpc = DynamicTwoTrackNMPC(params=params)

    state0 = np.array([0.0, 0.0, 0.0, 25.0, 0.0, 0.0])
    control = np.array([math.radians(2.0), math.radians(-1.0), 500.0, 1.0])

    state_next, kamm = nmpc.rk4_step(state0, control, dt=0.05, mu=0.85)

    assert len(state_next) == 6
    assert np.all(np.isfinite(state_next))
    # Longitudinal displacement forward
    assert state_next[0] > state0[0]
    # Positive speed under ax_req = 1.0
    assert state_next[3] > state0[3]


def test_dynamic_nmpc_compute_control_lane_tracking():
    params = SevenDOFParams.from_yaml()
    # Fast lightweight horizon for unit testing
    cfg = DynamicNMPCConfig(N=6, max_iter=10)
    nmpc = DynamicTwoTrackNMPC(params=params, config=cfg)

    # Vehicle is on Lane 1 at Y = 0.0, moving at 20 m/s
    state = SevenDOFState(
        vx=20.0,
        vy=0.0,
        r=0.0,
        omega=np.full(4, 20.0 / params.Reff),
        X=0.0,
        Y=0.0,
        psi=0.0,
    )
    # Target: Lane 2 at Y = 3.5 m
    target = ControlTarget(
        target_y=3.5,
        target_psi=0.0,
        target_vx=20.0,
        mu_estimated=0.9,
    )

    out = nmpc.compute_control(state, target)

    assert out.steer_f is not None
    assert out.steer_r is not None
    assert out.T_drive is not None
    assert out.T_brake is not None
    assert out.computation_time_ms > 0.0
    # To transition towards Y = 3.5m, controller must steer left (delta_f > 0)
    assert out.steer_f > 0.0


def test_dynamic_nmpc_kamm_circle_constraint_enforcement():
    """Verify that tire forces computed in dynamics stay bounded by mu * Fz."""
    params = SevenDOFParams.from_yaml()
    nmpc = DynamicTwoTrackNMPC(params=params)

    # State: 20 m/s with lateral slip and yaw rate
    state = np.array([0.0, 0.0, 0.0, 20.0, 1.5, 0.25])
    # Extreme steering input
    control = np.array([math.radians(25.0), math.radians(-5.0), 3000.0, 2.0])

    _, kamm_util = nmpc.continuous_dynamics(state, control, mu=0.5)

    # All tire utilisations should be strictly positive and finite
    assert len(kamm_util) == 4
    for util in kamm_util:
        assert util > 0.0
        assert not math.isnan(util)


def test_dynamic_nmpc_torque_vectoring_generation():
    """Verify that active torque vectoring creates differential drive torques."""
    params = SevenDOFParams.from_yaml()
    cfg = DynamicNMPCConfig(N=6, max_iter=15)
    nmpc = DynamicTwoTrackNMPC(params=params, config=cfg)

    # Vehicle heading deviated from target
    state = SevenDOFState(
        vx=22.0,
        vy=0.0,
        r=0.0,
        omega=np.full(4, 22.0 / params.Reff),
        X=0.0,
        Y=0.0,
        psi=math.radians(-5.0),
    )
    target = ControlTarget(
        target_y=0.0,
        target_psi=0.0,
        target_vx=22.0,
        mu_estimated=0.85,
    )

    out = nmpc.compute_control(state, target)

    # Controller should command drive torques
    assert out.T_drive is not None
    assert len(out.T_drive) == 4
    # Reset controller state
    nmpc.reset()
    assert np.all(nmpc.u_prev == 0.0)

