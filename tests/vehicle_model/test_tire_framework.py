"""Unit and physics validation tests for the tyre model framework.

Verifies:
- LinearTyre, FialaTyre, SimplifiedCombinedSlipTyre, and PacejkaTyre
- Friction ellipse and Kamm circle bounds (sqrt(Fx^2 + Fy^2) <= mu * Fz)
- Combined slip lateral reduction under traction / braking
- Degenerate boundary conditions (Fz <= 0, mu <= 0)
- Provenance constraints and factory loading
"""

from __future__ import annotations

import math
import numpy as np
import pytest

from vehicle_model.tire_model import (
    FialaTyre,
    LinearTyre,
    PacejkaTyre,
    SimplifiedCombinedSlipTyre,
    available_tyres,
    load_tyre,
)
from vehicle_model.tire_model.framework import TyreProvenance


@pytest.fixture(params=["linear_passenger_synthetic", "fiala_passenger_synthetic", "simplified_combined_synthetic", "pacejka_front_synthetic", "magic_formula_passenger_synthetic"])
def tyre_instance(request):
    return load_tyre(request.param)


def test_factory_available_tyres():
    available = available_tyres()
    assert "fiala_passenger_synthetic" in available
    assert "linear_passenger_synthetic" in available
    assert "simplified_combined_synthetic" in available
    assert "pacejka_front_synthetic" in available
    assert "pacejka_rear_synthetic" in available


def test_tyre_provenance_integrity(tyre_instance):
    prov = tyre_instance.provenance
    assert isinstance(prov, TyreProvenance)
    assert prov.data_class in ("synthetic", "fitted", "measured")
    assert prov.reference_load > 0.0
    assert len(prov.formulation) > 0
    assert len(prov.source) > 0


def test_tyre_friction_bound_enforcement(tyre_instance):
    """Resultant ground force must not exceed mu * Fz under any slip condition."""
    if isinstance(tyre_instance, LinearTyre) and not tyre_instance.saturate:
        pytest.skip("Unsaturated LinearTyre does not enforce friction limit by definition")

    Fz = 4000.0
    mu = 0.85
    limit = mu * Fz * 1.001  # small floating-point tolerance

    kappas = np.linspace(-0.5, 0.5, 21)
    alphas = np.radians(np.linspace(-20.0, 20.0, 21))

    for k in kappas:
        for a in alphas:
            fx, fy = tyre_instance.forces(k, a, Fz, mu)
            f_total = math.hypot(float(fx), float(fy))
            assert f_total <= limit, f"Friction circle exceeded: {f_total} > {limit} for k={k}, a={a}"


def test_tyre_zero_force_at_zero_slip(tyre_instance):
    """At zero slip, forces should be zero."""
    fx, fy = tyre_instance.forces(0.0, 0.0, 3000.0, 1.0)
    assert abs(float(fx)) < 1e-6
    assert abs(float(fy)) < 1e-6


def test_tyre_zero_force_at_zero_load_or_friction(tyre_instance):
    """Tyre on zero load or zero friction produces no force."""
    fx_z, fy_z = tyre_instance.forces(0.1, 0.1, 0.0, 1.0)
    assert abs(float(fx_z)) == 0.0
    assert abs(float(fy_z)) == 0.0

    fx_m, fy_m = tyre_instance.forces(0.1, 0.1, 3000.0, 0.0)
    assert abs(float(fx_m)) == 0.0
    assert abs(float(fy_m)) == 0.0


def test_tyre_combined_slip_degradation(tyre_instance):
    """Large longitudinal slip kappa must reduce available lateral cornering force Fy."""
    if isinstance(tyre_instance, LinearTyre) and not tyre_instance.saturate:
        pytest.skip("Unsaturated LinearTyre has no combined slip coupling")

    alpha = math.radians(5.0)
    Fz = 3500.0
    mu = 1.0

    _, fy_pure = tyre_instance.forces(0.0, alpha, Fz, mu)
    _, fy_combined = tyre_instance.forces(0.30, alpha, Fz, mu)

    # Lateral force with 30% spin/lock must be lower than pure cornering force
    assert abs(float(fy_combined)) < abs(float(fy_pure)), (
        f"Combined slip did not degrade lateral force: pure={fy_pure}, combined={fy_combined}"
    )


def test_tyre_slip_stiffness_positive(tyre_instance):
    """Initial slip stiffness dFx/dkappa must be positive for positive load."""
    stiffness = tyre_instance.slip_stiffness(3000.0, 1.0)
    assert float(stiffness) > 0.0


def test_tyre_lateral_force_is_odd_in_slip_angle_at_any_longitudinal_slip(tyre_instance):
    """Symmetric tyres: Fy(kappa, -alpha) = -Fy(kappa, alpha), so Fy(kappa, 0) = 0 (no pull when braking).

    The legacy Pacejka adapter only removed the offset at kappa = 0 and produced up to
    ~320 N lateral force per tyre at alpha = 0 under braking (Audit 4.6).
    """
    kappas = np.array([-0.3, -0.1, -0.03, 0.0, 0.03, 0.1, 0.3])
    for a in np.radians([0.0, 0.5, 2.0, 6.0]):
        _, fy_pos = tyre_instance.forces(kappas, np.full(7, a), np.full(7, 3500.0), np.ones(7))
        _, fy_neg = tyre_instance.forces(kappas, np.full(7, -a), np.full(7, 3500.0), np.ones(7))
        assert np.allclose(fy_pos, -fy_neg, atol=1e-6), f"alpha={np.degrees(a):.1f} deg"
