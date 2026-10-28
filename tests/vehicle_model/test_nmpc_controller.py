"""Unit tests for NMPC controller, kinematic bicycle plant, and VehicleController protocol."""

import math
import numpy as np
import pytest

from vehicle_model.controllers.framework import (
    ControlOutput,
    ControlTarget,
    VehicleController,
)
from vehicle_model.controllers.nmpc import (
    KinematicBicycleSystem,
    NMPCConfig,
    NMPCControllerCGMRES,
    NMPCPathTrackingController,
    bicycle_kinematic_model,
    get_ref_state,
)
from vehicle_model.core.types import SevenDOFParams, SevenDOFState


def test_nmpc_config_defaults_and_yaml():
    """Verify NMPCConfig defaults and YAML loader."""
    cfg = NMPCConfig.from_yaml()
    assert cfg.wheelbase == 0.60
    assert cfg.max_accel == 4.5
    assert cfg.target_v == 2.5
    assert cfg.N == 8
    assert cfg.wavelength > 0


def test_get_ref_state_evaluation():
    """Verify spatial sinusoidal reference path calculation."""
    x0, y0, yaw0, v0 = get_ref_state(0.0, init_x=0.0, vehicle_x=0.0, amplitude=1.5, wavelength=16.0)
    assert x0 == 0.0
    assert y0 == 0.0
    assert v0 == 2.5
    # At x=0, dy/dx = A * (2pi/W) * cos(0) > 0 => yaw0 > 0
    assert yaw0 > 0.0

    # At quarter wavelength, y should be maximum (+amplitude) and slope 0 => yaw=0
    x_q, y_q, yaw_q, _ = get_ref_state(0.0, init_x=0.0, vehicle_x=4.0, amplitude=1.5, wavelength=16.0)
    assert math.isclose(y_q, 1.5, rel_tol=1e-4)
    assert math.isclose(yaw_q, 0.0, abs_tol=1e-4)


def test_kinematic_bicycle_plant_step():
    """Verify kinematic bicycle plant state propagation and history."""
    plant = KinematicBicycleSystem(init_x=0.0, init_y=0.0, init_yaw=0.0, init_v=1.0)
    plant.update_state(u_a=1.0, u_steer_f=math.radians(10.0), dt=0.05)

    assert plant.x > 0.0
    assert plant.v > 1.0
    assert len(plant.history_x) == 2
    assert len(plant.history_steer_r) == 2


def test_nmpc_cgmres_controller_steps():
    """Verify NMPCControllerCGMRES closed-loop steps converge and return finite inputs."""
    dt = 0.05
    plant = KinematicBicycleSystem(0.0, 0.0, 0.0, 0.5)
    ctrl = NMPCControllerCGMRES(init_x=0.0)

    for i in range(5):
        t = (i + 1) * dt
        u1s, u2s = ctrl.calc_input(plant.x, plant.y, plant.yaw, plant.v, t)
        assert len(u1s) == ctrl.N
        assert len(u2s) == ctrl.N
        assert np.isfinite(u1s).all()
        assert np.isfinite(u2s).all()
        plant.update_state(float(u1s[0]), float(u2s[0]), dt)

    assert plant.x > 0.0
    assert np.isfinite([plant.x, plant.y, plant.yaw, plant.v]).all()


def test_nmpc_path_tracking_controller_protocol():
    """Verify NMPCPathTrackingController adheres to VehicleController protocol."""
    ctrl = NMPCPathTrackingController.from_yaml()
    assert isinstance(ctrl, VehicleController)
    assert "NMPC" in ctrl.name

    state = SevenDOFState(X=0.0, Y=0.0, psi=0.0, vx=2.0)
    target = ControlTarget(target_vx=2.5)

    out = ctrl.compute_control(state, target, dt=0.05)
    assert isinstance(out, ControlOutput)
    assert np.isfinite(out.steer_f)
    assert np.isfinite(out.steer_r)
    assert len(out.T_drive) == 4
    assert len(out.T_brake) == 4
    assert out.status == "nominal"
    assert out.computation_time_ms > 0.0
