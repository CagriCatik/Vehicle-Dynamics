"""Unilateral tyre contact in the vertical models (Plan.md, Abschnitt 8; Audit 4.3).

A lifted wheel carries no load and the tyre exerts no force on it; the
reported contact force falls when the wheel moves up.
"""

from __future__ import annotations

import dataclasses

import numpy as np
import pytest

from vehicle_model.suspension.quarter_car import QuarterCarModel, QuarterCarState
from vehicle_model.vehicle_models.fourteen_dof import (
    FourteenDOFConfig,
    FourteenDOFModel,
    FourteenDOFState,
)

G = 9.80665  # value used by both vertical models


# ----------------------------------------------------------------------------- quarter car
def test_quarter_car_contact_force_falls_when_wheel_rises():
    m = QuarterCarModel()
    k_t = m.cfg.tyre_stiffness
    for dz in (-0.004, 0.004):
        _, out = m.evaluate_derivatives(QuarterCarState(zs=dz, zu=dz), road_z=0.0, road_dz=0.0)
        assert out.total_tyre_load == pytest.approx(m.static_tyre_load - k_t * dz, rel=1e-12)
        assert out.dynamic_tyre_load == pytest.approx(-k_t * dz, rel=1e-9)
        assert out.in_contact


def test_quarter_car_lifted_wheel_feels_no_tyre_force():
    """Wheel 3 cm above equilibrium (static tyre deflection ~2 cm): free flight of the unsprung mass."""
    m = QuarterCarModel()
    cfg = m.cfg
    dz = 2.0 * m.static_tyre_load / cfg.tyre_stiffness
    _, out = m.evaluate_derivatives(QuarterCarState(zs=dz, zu=dz), road_z=0.0, road_dz=0.0)
    assert out.total_tyre_load == 0.0
    assert not out.in_contact
    # Deviation coordinates: the spring preload still pushes the wheel down by m_s g
    assert out.ddzu == pytest.approx(-(cfg.sprung_mass + cfg.unsprung_mass) * G / cfg.unsprung_mass, rel=1e-9)
    assert out.ddzs == pytest.approx(0.0, abs=1e-9)


def test_quarter_car_tyre_force_is_continuous_at_lift_off():
    m = QuarterCarModel()
    dz_lift = m.static_tyre_load / m.cfg.tyre_stiffness
    eps = 1e-9
    a = m.evaluate_derivatives(QuarterCarState(zs=dz_lift - eps, zu=dz_lift - eps), 0.0, 0.0)[1]
    b = m.evaluate_derivatives(QuarterCarState(zs=dz_lift + eps, zu=dz_lift + eps), 0.0, 0.0)[1]
    assert a.in_contact and not b.in_contact
    assert abs(a.ddzu - b.ddzu) <= 2 * eps * m.cfg.tyre_stiffness / m.cfg.unsprung_mass * (1 + 1e-6)


# ----------------------------------------------------------------------------- 14-DOF
def _derivative(cfg: FourteenDOFConfig, state: FourteenDOFState):
    model = FourteenDOFModel(cfg)
    zeros = np.zeros(4)
    return model.evaluate_dynamics(state, zeros, zeros, zeros)


def test_fourteen_dof_lifted_wheel_is_independent_of_tyre_stiffness():
    """With the FL wheel off the ground its hop acceleration cannot depend on the tyre spring."""
    cfg = FourteenDOFConfig()
    stiff = dataclasses.replace(cfg, k_tyre_front=2.0 * cfg.k_tyre_front)
    lifted = FourteenDOFState(vx=20.0, zu=np.array([0.1, 0.0, 0.0, 0.0]))
    d1, t1 = _derivative(cfg, lifted)
    d2, t2 = _derivative(stiff, lifted)
    assert list(t1.in_contact) == [False, True, True, True]
    assert t1.Fz[0] == 0.0
    assert d1[16] == pytest.approx(d2[16], rel=1e-12)  # ddzu_FL

    # Control: in contact the tyre spring does matter
    pressed = FourteenDOFState(vx=20.0, zu=np.array([-0.005, 0.0, 0.0, 0.0]))
    assert _derivative(cfg, pressed)[0][16] != pytest.approx(_derivative(stiff, pressed)[0][16], rel=1e-3)


def test_fourteen_dof_static_state_is_in_contact():
    _, tel = _derivative(FourteenDOFConfig(), FourteenDOFState(vx=20.0))
    assert tel.in_contact.all()
    assert np.allclose(tel.Fz, FourteenDOFConfig().static_normal_loads, rtol=1e-12)
