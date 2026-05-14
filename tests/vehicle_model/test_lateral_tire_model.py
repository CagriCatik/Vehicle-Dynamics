from __future__ import annotations

import numpy as np

from vehicle_model.tire_model.lateral_demo import calculate_lateral_curves


def test_lateral_magic_formula_returns_finite_forces():
    curves = calculate_lateral_curves(n_points=20)
    for values in curves.values():
        assert np.isfinite(values).all()
