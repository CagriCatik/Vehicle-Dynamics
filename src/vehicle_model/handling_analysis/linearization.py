"""Derive linear single-track parameters from a two-track parameter set and its tyre models.

Assumptions of the reduction (each one is a source of deviation from the 7-DOF model):
  * static axle loads, split equally left/right (no load transfer)
  * axle cornering stiffness = 2 x wheel stiffness dF_y/d(alpha) at alpha = 0, kappa = 0
  * track width, aerodynamics, rolling resistance and wheel dynamics are neglected
"""

from __future__ import annotations

import numpy as np

from vehicle_model.core.types import SevenDOFParams
from vehicle_model.tire_model.framework import TyreModel

from .linear_single_track import LinearSingleTrackParams

_D_ALPHA = 1e-5  # [rad] central-difference step, well inside the linear range


def cornering_stiffness(tyre: TyreModel, Fz: float, mu: float = 1.0) -> float:
    """Wheel cornering stiffness dF_y/d(alpha) at alpha = kappa = 0 [N/rad] by central difference."""
    alpha = np.array([-_D_ALPHA, _D_ALPHA])
    _, Fy = tyre.forces(np.zeros(2), alpha, np.full(2, float(Fz)), np.full(2, float(mu)))
    return float((Fy[1] - Fy[0]) / (2.0 * _D_ALPHA))


def linear_params_from_seven_dof(
    params: SevenDOFParams,
    tyre_front: TyreModel,
    tyre_rear: TyreModel,
    mu: float = 1.0,
) -> LinearSingleTrackParams:
    L = params.wheelbase
    Fz_f = params.mass * params.g * params.lr / L / 2.0
    Fz_r = params.mass * params.g * params.lf / L / 2.0
    return LinearSingleTrackParams(
        mass=params.mass,
        Iz=params.Iz,
        lf=params.lf,
        lr=params.lr,
        C_f=2.0 * cornering_stiffness(tyre_front, Fz_f, mu),
        C_r=2.0 * cornering_stiffness(tyre_rear, Fz_r, mu),
    )


__all__ = ("cornering_stiffness", "linear_params_from_seven_dof")
