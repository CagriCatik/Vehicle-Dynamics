"""Unit and physics tests for 14-DOF Active Roll Distribution Controller."""

import math
import numpy as np
import pytest

from vehicle_model.controllers.active_roll import (
    ActiveRollConfig,
    ActiveRollDistributionController,
    ActiveRollOutput,
)
from vehicle_model.vehicle_models.fourteen_dof import FourteenDOFConfig, FourteenDOFState
from vehicle_model.core.types import FL, FR, RL, RR


def test_active_roll_config_defaults():
    cfg = ActiveRollConfig()
    assert 0.50 <= cfg.nominal_front_roll_ratio <= 0.65
    assert cfg.min_front_roll_ratio < cfg.nominal_front_roll_ratio < cfg.max_front_roll_ratio
    assert cfg.ltr_critical_threshold > 0.60


def test_active_roll_distribution_understeer_vs_oversteer():
    v_cfg = FourteenDOFConfig()
    ctrl = ActiveRollDistributionController(vehicle_cfg=v_cfg)

    # 1. Nominal straight driving
    s_nom = FourteenDOFState(vx=25.0)
    out_nom = ctrl.compute_control(s_nom, r_desired=0.0, ay=0.0, Fz_corners=v_cfg.static_normal_loads)
    assert math.isclose(out_nom.kappa_roll_front, ctrl.cfg.nominal_front_roll_ratio, abs_tol=1e-3)

    # 2. Understeer condition (actual yaw rate r is less than desired r_des)
    # r = 0.10, r_des = 0.25 -> r_error = -0.15 (understeer)
    s_under = FourteenDOFState(vx=25.0, r=0.10)
    out_under = ctrl.compute_control(s_under, r_desired=0.25, ay=2.5, Fz_corners=v_cfg.static_normal_loads)
    # Front roll ratio must DECREASE (shift rearward) to induce yaw moment
    assert out_under.kappa_roll_front < ctrl.cfg.nominal_front_roll_ratio

    # 3. Oversteer condition (actual yaw rate r is higher than desired r_des, tail sliding)
    # r = 0.35, r_des = 0.15 -> r_error = +0.20 (oversteer)
    s_over = FourteenDOFState(vx=25.0, r=0.35, vy=2.0)
    out_over = ctrl.compute_control(s_over, r_desired=0.15, ay=4.0, Fz_corners=v_cfg.static_normal_loads)
    # Front roll ratio must INCREASE (shift forward) to stabilize rear axle
    assert out_over.kappa_roll_front > ctrl.cfg.nominal_front_roll_ratio


def test_active_roll_force_balance_conservation():
    v_cfg = FourteenDOFConfig()
    ctrl = ActiveRollDistributionController(vehicle_cfg=v_cfg)

    # Cornering with roll angle phi = +3 deg (rolling to the right)
    s_roll = FourteenDOFState(vx=25.0, phi=math.radians(3.0), dphi=math.radians(5.0))
    out = ctrl.compute_control(s_roll, r_desired=0.20, ay=4.0, Fz_corners=v_cfg.static_normal_loads)

    f_act = out.active_corner_forces
    assert len(f_act) == 4

    # Net vertical force must sum to 0 (pure roll couple, no net heave force)
    assert math.isclose(np.sum(f_act), 0.0, abs_tol=1e-6)

    # Left corners (FL, RL) must have opposite sign to right corners (FR, RR)
    assert math.isclose(f_act[FL] + f_act[FR], 0.0, abs_tol=1e-6)
    assert math.isclose(f_act[RL] + f_act[RR], 0.0, abs_tol=1e-6)


def test_active_roll_load_transfer_ratio_and_wheel_lift_mitigation():
    v_cfg = FourteenDOFConfig()
    ctrl = ActiveRollDistributionController(vehicle_cfg=v_cfg)

    # Near wheel lift: right side heavily unloaded
    Fz_near_lift = np.array([4800.0, 300.0, 4200.0, 200.0])  # Left ~ 9000N, Right ~ 500N
    s_severe = FourteenDOFState(vx=22.0, phi=math.radians(5.5), dphi=math.radians(8.0))

    out = ctrl.compute_control(s_severe, r_desired=0.30, ay=7.5, Fz_corners=Fz_near_lift)
    assert out.load_transfer_ratio > 0.70
    assert out.wheel_lift_risk is True
    # Active restoring forces must engage strongly
    assert np.any(np.abs(out.active_corner_forces) > 200.0)
