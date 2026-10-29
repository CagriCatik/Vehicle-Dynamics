"""Drift plan D3: Jacobians at a drift equilibrium (step study, reproducibility, stabilisability)."""

from __future__ import annotations

import dataclasses
import math

import numpy as np
import pytest

from vehicle_model.drift import DriftEquilibriumSolver, DriftReference
from vehicle_model.drift.linearization import jacobians, linearise
from vehicle_model.vehicle_models import SevenDOFModel


@pytest.fixture(scope="module")
def setup():
    sol = DriftEquilibriumSolver(SevenDOFModel.from_yaml())
    eq = sol.solve(DriftReference(13.0, math.radians(-20.0), 0.9))
    return sol.plant(0.9), eq


def test_drift_equilibrium_is_open_loop_unstable_but_stabilisable(setup):
    P, eq = setup
    lin = linearise(P, eq)
    assert lin.converged and np.all(np.isfinite(lin.A)) and np.all(np.isfinite(lin.B))
    assert len(lin.unstable) >= 1 and max(l.real for l in lin.unstable) > 1.0       # the drift mode
    assert lin.stabilisable and all(r == 7 for r in lin.pbh_ranks)
    assert lin.controllability_rank < 7          # stable fast wheel modes are not reachable: reported, not hidden


def test_jacobians_are_reproducible_in_any_order(setup):
    P, eq = setup
    q, u = eq.body_state(), np.array([eq.steer_front, eq.wheel_drive_torque[2]])
    A1, B1 = jacobians(P, q, u, 1e-6)
    jacobians(P, q + 0.3, u * 1.1, 1e-4)              # evaluate elsewhere in between
    A2, B2 = jacobians(P, q, u, 1e-6)
    assert np.array_equal(A1, A2) and np.array_equal(B1, B2)


def test_torque_acts_only_on_the_rear_wheel_speeds(setup):
    P, eq = setup
    B = linearise(P, eq).B
    assert np.allclose(B[:5, 1], 0.0, atol=1e-9) and B[5, 1] > 0.0 and B[5, 1] == pytest.approx(B[6, 1])


def test_infeasible_equilibria_are_not_linearised(setup):
    P, eq = setup
    bad = dataclasses.replace(eq, physical_feasible=False, reasons=("residual_too_large",))
    with pytest.raises(ValueError, match="infeasible"):
        linearise(P, bad)
