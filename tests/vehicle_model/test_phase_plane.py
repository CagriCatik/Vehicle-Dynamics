"""Unit and physics tests for nonlinear (beta, beta_dot) phase-plane stability controller."""

import math
import numpy as np
import pytest

from vehicle_model.controllers.phase_plane import (
    PhasePlaneConfig,
    PhasePlaneEnvelopeController,
    PhasePlaneState,
)
from vehicle_model.controllers.integrated_controller import IntegratedChassisController
from vehicle_model.controllers.framework import ControlTarget
from vehicle_model.core.types import SevenDOFParams, SevenDOFState


def test_phase_plane_config_defaults():
    cfg = PhasePlaneConfig()
    assert cfg.beta_sat_dry_deg == 9.0
    assert cfg.c_phase_time_s == 0.20
    assert cfg.s_approach == 0.65
    assert cfg.s_critical == 1.00
    assert cfg.k_mz_envelope > 0.0


def test_phase_plane_boundary_limits_scale_with_friction():
    ctrl = PhasePlaneEnvelopeController()

    # Dry asphalt (mu=1.0) vs Ice (mu=0.2)
    b_dry, bdot_dry = ctrl.compute_boundary_limits(vx=25.0, mu=1.0)
    b_ice, bdot_ice = ctrl.compute_boundary_limits(vx=25.0, mu=0.2)

    # Ice stability envelope must be significantly smaller than dry road
    assert b_ice < b_dry
    assert bdot_ice < bdot_dry
    assert math.degrees(b_dry) >= 8.0
    assert math.degrees(b_ice) <= 4.0


def test_phase_plane_linear_stable_region():
    ctrl = PhasePlaneEnvelopeController()
    eval_res = ctrl.evaluate(vx=20.0, vy=0.05, r=0.02, ay=0.4, mu=1.0)

    assert eval_res.phase_state == PhasePlaneState.LINEAR_STABLE
    assert eval_res.stability_index < 0.65
    assert eval_res.stabilizing_yaw_moment_nm == 0.0
    assert eval_res.active_counter_steer_rad == 0.0
    assert eval_res.emergency_decel_demand_mps2 == 0.0


def test_phase_plane_approaching_limit_soft_intervention():
    ctrl = PhasePlaneEnvelopeController()
    # Moderate sideslip: beta ~ 6 deg on dry asphalt
    vy = 20.0 * math.tan(math.radians(6.0))
    eval_res = ctrl.evaluate(vx=20.0, vy=vy, r=0.1, ay=2.5, mu=1.0)

    assert eval_res.phase_state in (PhasePlaneState.APPROACHING_LIMIT, PhasePlaneState.CRITICAL_SATURATION)
    assert eval_res.stability_index >= 0.65
    # Positive beta (right turn oversteer tail-slide) must command positive (CCW) yaw moment and counter-steer
    assert eval_res.stabilizing_yaw_moment_nm > 0.0
    assert eval_res.active_counter_steer_rad > 0.0


def test_phase_plane_critical_saturation_emergency_response():
    ctrl = PhasePlaneEnvelopeController()
    # Extreme sideslip: beta = 15 deg on dry road (well outside 9 deg envelope)
    vy = 20.0 * math.tan(math.radians(15.0))
    eval_res = ctrl.evaluate(vx=20.0, vy=vy, r=0.3, ay=1.0, mu=1.0)

    assert eval_res.phase_state == PhasePlaneState.CRITICAL_SATURATION
    assert eval_res.stability_index >= 1.0
    # Full sliding-mode restoring moment
    assert eval_res.stabilizing_yaw_moment_nm >= 0.9 * ctrl.config.k_mz_envelope
    assert eval_res.emergency_decel_demand_mps2 > 0.0


def test_phase_plane_sign_symmetry():
    ctrl = PhasePlaneEnvelopeController()
    vy_pos = 20.0 * math.tan(math.radians(8.0))
    vy_neg = -vy_pos

    eval_pos = ctrl.evaluate(vx=20.0, vy=vy_pos, r=0.15, ay=2.0, mu=0.8)
    eval_neg = ctrl.evaluate(vx=20.0, vy=vy_neg, r=-0.15, ay=-2.0, mu=0.8)

    assert math.isclose(eval_pos.stability_index, eval_neg.stability_index, rel_tol=1e-3)
    assert math.isclose(eval_pos.stabilizing_yaw_moment_nm, -eval_neg.stabilizing_yaw_moment_nm, rel_tol=1e-3)
    assert math.isclose(eval_pos.active_counter_steer_rad, -eval_neg.active_counter_steer_rad, rel_tol=1e-3)


def test_integrated_ivmc_with_phase_plane_controller():
    params = SevenDOFParams.from_yaml()
    ivmc = IntegratedChassisController(params=params, enable_phase_plane=True)

    state = SevenDOFState(
        vx=25.0,
        vy=25.0 * math.tan(math.radians(10.0)),  # Large sideslip
        r=0.25,
        omega=np.full(4, 25.0 / params.Reff),
        X=0.0,
        Y=0.0,
        psi=0.0,
    )
    target = ControlTarget(driver_steer_f=0.0, driver_accel_pedal=0.0, driver_brake_pedal=0.0)

    out = ivmc.compute_control(state, target)
    assert out.T_brake is not None
    # Corrective action active
    assert out.steer_f != 0.0
