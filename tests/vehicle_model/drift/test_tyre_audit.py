"""Tyre audit over the large-slip range (zero/low friction, large slip angles, load lift, scaling)."""

from __future__ import annotations

import numpy as np
import pytest

from vehicle_model.drift import audit_tyre, extrapolating
from vehicle_model.tire_model import SlipStiffnessScaledTyre, load_tyre
from vehicle_model.tire_model.framework import TyreProvenance

TYRES = ("magic_formula_passenger_synthetic", "simplified_combined_synthetic", "fiala_passenger_synthetic")


@pytest.mark.parametrize("name", TYRES)
def test_bundled_tyres_pass_the_audit(name):
    rep = audit_tyre(load_tyre(name), name)
    assert rep.passed, [c for c in rep.checks if c["passed"] is False]
    assert rep.check("validity_range")["value"] > 0.5        # +-45 deg audit grid exceeds the declared +-20 deg
    assert rep.utilisation.shape == (91, rep.kappa.size)


def test_large_slip_angles_keep_a_falling_but_finite_lateral_force():
    rep = audit_tyre(load_tyre("magic_formula_passenger_synthetic"))
    drop = rep.check("post_peak")["value"]
    assert 0.5 < drop < 1.0                                    # MF: force falls after the peak, not to zero
    fy = rep.lateral[(4000.0, 1.0, 0.0)]
    assert np.all(np.isfinite(fy)) and abs(fy[-1]) > 0.5 * np.max(np.abs(fy))


def test_zero_low_friction_and_load_lift():
    t = load_tyre("magic_formula_passenger_synthetic")
    rep = audit_tyre(t)
    assert rep.check("zero_friction")["passed"] and rep.check("zero_load")["passed"]
    Fx, Fy = t.forces(np.array([0.2]), np.array([0.2]), np.array([4000.0]), np.array([0.05]))
    assert np.hypot(Fx, Fy)[0] <= 0.05 * 4000.0 * 1.1          # low friction bounds the force


def test_scaled_tyre_passes_with_scaled_stiffness():
    base = load_tyre("magic_formula_passenger_synthetic")
    rep = audit_tyre(SlipStiffnessScaledTyre(base, cornering=0.6), "scaled")
    assert rep.passed
    a = rep.alpha
    i = int(np.argmin(np.abs(a - np.radians(0.5))))
    ref = audit_tyre(base)
    ratio = rep.lateral[(4000.0, 1.0, 0.0)][i] / ref.lateral[(4000.0, 1.0, 0.0)][i]
    assert ratio == pytest.approx(0.6, rel=0.02)


class _Jumpy:
    """A tyre with a force jump at alpha = 10 deg: the continuity check must catch it."""
    provenance = TyreProvenance("jumpy", "1", "test", "synthetic", 4000.0)

    def forces(self, kappa, alpha, Fz, mu):
        kappa, alpha, Fz, mu = (np.asarray(x, float) for x in (kappa, alpha, Fz, mu))
        Fy = mu * Fz * np.tanh(10 * alpha) * np.where(np.abs(alpha) > np.radians(10.0), 0.5, 1.0)
        return mu * Fz * np.tanh(10 * kappa), Fy

    def slip_stiffness(self, Fz, mu):
        return 10 * np.asarray(Fz, float)


def test_the_audit_detects_a_discontinuity():
    rep = audit_tyre(_Jumpy(), "jumpy")
    assert rep.check("continuity")["passed"] is False and not rep.passed


def test_extrapolation_flag_per_wheel():
    t = load_tyre("magic_formula_passenger_synthetic")
    flags = extrapolating(t, np.radians([5.0, 25.0, -30.0, 0.0]), np.array([0.1, 0.0, 0.0, 0.5]))
    assert flags.tolist() == [False, True, True, False]
