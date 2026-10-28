"""Magic Formula 5.2 subset (Plan.md, Abschnitt 7; M3).

Low slip, saturation, combined slip, friction scaling, signs, continuity and
contact loss, each checked against the documented formulation.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from vehicle_model.core.types import SevenDOFControlInput, SevenDOFState
from vehicle_model.tire_model import MagicFormulaParams, MagicFormulaTyre, load_tyre
from vehicle_model.vehicle_models import SevenDOFModel


@pytest.fixture(scope="module")
def tyre() -> MagicFormulaTyre:
    return load_tyre("magic_formula_passenger_synthetic")


LOADS = (1500.0, 4000.0, 7000.0)
MUS = (1.0, 0.6, 0.2)


# ----------------------------------------------------------------------------- low slip
@pytest.mark.parametrize("Fz", LOADS)
@pytest.mark.parametrize("mu", MUS)
def test_slip_stiffness_matches_analytical_and_is_independent_of_mu(tyre, Fz, mu):
    h = 1e-7
    Fx, _ = tyre.forces(np.array([-h, h]), 0.0, Fz, mu)
    _, Fy = tyre.forces(0.0, np.array([-h, h]), Fz, mu)
    assert (Fx[1] - Fx[0]) / (2 * h) == pytest.approx(float(tyre.longitudinal_stiffness(Fz)), rel=1e-6)
    assert (Fy[1] - Fy[0]) / (2 * h) == pytest.approx(float(tyre.cornering_stiffness(Fz)), rel=1e-6)


def test_reference_characteristics_at_nominal_load(tyre):
    Fz0 = tyre.params.Fz0
    assert tyre.cornering_stiffness(Fz0) / Fz0 == pytest.approx(18.0, rel=0.01)
    assert tyre.longitudinal_stiffness(Fz0) / Fz0 == pytest.approx(20.0, rel=1e-12)


def test_load_sensitivity_is_degressive(tyre):
    Fz = np.array(LOADS)
    assert np.all(np.diff(tyre.cornering_stiffness(Fz) / Fz) < 0)
    assert np.all(np.diff(tyre.longitudinal_stiffness(Fz) / Fz) < 0)
    mu_x, mu_y = tyre.peak_friction(Fz, 1.0)
    assert np.all(np.diff(mu_x) < 0) and np.all(np.diff(mu_y) < 0)


# ----------------------------------------------------------------------------- saturation
@pytest.mark.parametrize("Fz", LOADS)
@pytest.mark.parametrize("mu", MUS)
def test_pure_slip_peak_equals_D(tyre, Fz, mu):
    mu_x, mu_y = tyre.peak_friction(Fz, mu)
    k = np.linspace(0.0, 1.0, 20001)
    a = np.linspace(0.0, 0.6, 20001)
    Fx, _ = tyre.forces(k, 0.0, Fz, mu)
    _, Fy = tyre.forces(0.0, a, Fz, mu)
    assert Fx.max() == pytest.approx(float(mu_x * Fz), rel=1e-6)
    assert Fy.max() == pytest.approx(float(mu_y * Fz), rel=1e-6)
    # Peak followed by a friction drop, force never reverses
    assert Fx[-1] < Fx.max() and Fx[-1] > 0.0
    assert Fy[-1] < Fy.max() and Fy[-1] > 0.0


def test_peak_slip_scales_with_friction(tyre):
    k = np.linspace(0.0, 0.5, 50001)
    peaks = [k[np.argmax(tyre.forces(k, 0.0, 4000.0, mu)[0])] for mu in (1.0, 0.5)]
    assert peaks[1] / peaks[0] == pytest.approx(0.5, rel=1e-3)


# ----------------------------------------------------------------------------- combined slip
def _grid():
    return np.meshgrid(np.linspace(-1.0, 1.0, 201), np.radians(np.linspace(-30.0, 30.0, 201)))


@pytest.mark.parametrize("Fz", LOADS)
@pytest.mark.parametrize("mu", MUS)
def test_resultant_stays_inside_friction_ellipse(tyre, Fz, mu):
    K, A = _grid()
    Fx, Fy = tyre.forces(K, A, Fz, mu)
    mu_x, mu_y = tyre.peak_friction(Fz, mu)
    assert np.max(np.hypot(Fx / (mu_x * Fz), Fy / (mu_y * Fz))) <= 1.0 + 1e-12


def test_combined_slip_map_is_self_similar_in_mu(tyre):
    """F(mu k, atan(mu tan a); mu) = mu F(k, a; 1): the coupling acts at the same fraction of the peak."""
    K, A = _grid()
    mu = 0.35
    Fx1, Fy1 = tyre.forces(K, A, 4000.0, 1.0)
    Fxm, Fym = tyre.forces(mu * K, np.arctan(mu * np.tan(A)), 4000.0, mu)
    assert np.allclose(Fxm, mu * Fx1, rtol=1e-9, atol=1e-6)
    assert np.allclose(Fym, mu * Fy1, rtol=1e-9, atol=1e-6)


def test_longitudinal_slip_reduces_lateral_force_and_vice_versa(tyre):
    k = np.linspace(0.0, 0.3, 31)
    _, Fy = tyre.forces(k, math.radians(4.0), 4000.0, 1.0)
    assert np.all(np.diff(Fy) < 0)
    a = np.radians(np.linspace(0.0, 10.0, 31))
    Fx, _ = tyre.forces(0.05, a, 4000.0, 1.0)
    assert np.all(np.diff(Fx) < 0)


def test_limiter_leaves_pure_slip_untouched_and_is_continuous(tyre):
    p = tyre.params
    k = np.linspace(-1, 1, 101)
    Fx, Fy = tyre.forces(k, 0.0, 4000.0, 1.0)
    D = p.PDX1 * 4000.0
    expected = tyre._sine_curve(tyre.longitudinal_stiffness(4000.0) / (p.PCX1 * D), p.PCX1, D, p.PEX1, k)
    assert np.allclose(Fx, expected, rtol=1e-12) and np.all(Fy == 0.0)
    # Continuity along a path crossing the limited region: steps shrink with the sampling
    t = np.linspace(0.0, 1.0, 4001)
    Fx, Fy = tyre.forces(0.3 * t, np.radians(15.0) * t, 4000.0, 1.0)
    assert np.max(np.hypot(np.diff(Fx), np.diff(Fy))) < 10.0  # [N] per 1/4000 of the path


# ----------------------------------------------------------------------------- signs
def test_signs_and_symmetry(tyre):
    K, A = _grid()
    Fx, Fy, Mz = tyre.forces_and_moment(K, A, 4000.0, 0.9)
    Fx_m, Fy_m, Mz_m = tyre.forces_and_moment(-K, -A, 4000.0, 0.9)
    assert np.allclose(Fx_m, -Fx, atol=1e-9) and np.allclose(Fy_m, -Fy, atol=1e-9)
    assert np.allclose(Mz_m, -Mz, atol=1e-9)
    _, Fy_k = tyre.forces(-K, A, 4000.0, 0.9)  # lateral force even in kappa
    assert np.allclose(Fy_k, Fy, atol=1e-9)
    assert np.all(np.sign(Fx[K != 0]) == np.sign(K[K != 0]))
    assert np.all(np.sign(Fy[A != 0]) == np.sign(A[A != 0]))


def test_aligning_moment(tyre):
    p = tyre.params
    Fz = p.Fz0
    a = np.radians(np.linspace(0.01, 25.0, 2500))
    _, Fy, Mz = tyre.forces_and_moment(0.0, a, Fz, 1.0)
    trail = -Mz / Fy
    # small slip: trail -> D_t = R0 QDZ1 (at Fz0), moment opposes the slip angle
    assert trail[0] == pytest.approx(p.R0 * p.QDZ1, rel=1e-3)
    assert Mz[0] < 0.0
    # moment peaks before the lateral force and changes sign at large slip (trail < 0)
    assert a[np.argmin(Mz)] < a[np.argmax(Fy)]
    assert Mz[-1] > 0.0


# ----------------------------------------------------------------------------- contact loss
def test_zero_force_without_load_or_friction(tyre):
    Fx, Fy, Mz = tyre.forces_and_moment([0.1, 0.1, 0.1], [0.1, 0.1, 0.1], [0.0, -50.0, 4000.0], [1.0, 1.0, 0.0])
    assert np.all(Fx == 0.0) and np.all(Fy == 0.0) and np.all(Mz == 0.0)


def test_force_vanishes_continuously_as_load_goes_to_zero(tyre):
    Fz = np.array([1e-3, 1.0, 10.0])
    Fx, Fy = tyre.forces(0.05, 0.05, Fz, 1.0)
    assert np.all(np.hypot(Fx, Fy) <= 1.2 * Fz)


def test_invalid_weighting_shape_rejected():
    with pytest.raises(ValueError):
        MagicFormulaParams(RCY1=1.3)


# ----------------------------------------------------------------------------- exchangeability
def test_vehicle_model_runs_with_magic_formula_tyres_unchanged():
    model = SevenDOFModel.from_yaml("configs/vehicles/reference_passenger_car.yaml",
                                    tyre_name="magic_formula_passenger_synthetic",
                                    tyre_rear_name="magic_formula_passenger_synthetic")
    p = model.params
    s = SevenDOFState(vx=20.0, omega=np.full(4, 20.0 / p.Reff))
    for _ in range(400):
        s, tel = model.step(s, SevenDOFControlInput(steer_f=0.04, T_drive=np.full(4, 40.0)), 0.005)
    ay = tel.Fy_body.sum() / p.mass
    assert 5.0 < ay < 9.5 and abs(s.vy) < 2.0 and np.all(np.isfinite(s.to_array()))
