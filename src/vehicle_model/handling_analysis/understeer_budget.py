"""Linear understeer budget: tyres, roll steer and compliance steer.

In steady cornering at small lateral acceleration the front steer angle is
``delta = L/R + EG a_y``. Quasi-static K&C steer changes add to the tyre part
(Gillespie 1992, ch. 6; Milliken & Milliken 1995, ch. 5):

    EG = EG_tyres + EG_roll_steer + EG_compliance

    EG_tyres       = m/L (l_r/C_f - l_f/C_r)                       (linearised tyres, static loads)
    EG_roll_steer  = (eps_r - eps_f) dphi/da_y                     (eps: axle steer per roll, > 0 towards the turn)
    EG_compliance  = c_f m l_r/L + c_r m l_f/L                     (c: understeer-positive compliance [rad/N])

all in rad/(m/s^2). A positive contribution adds understeer. The axle lateral
forces are the steady-state values ``m l_r/L a_y`` and ``m l_f/L a_y``.

Not in the linear budget: lateral load transfer with load-sensitive tyres (zero
slope at ``a_y -> 0``, grows with ``a_y``; this is where roll centres and the roll
stiffness split act), Ackermann geometry (second order in the steer angle) and
aligning-moment steer.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from .linearization import linear_params_from_seven_dof
from .linear_single_track import LinearSingleTrack


@dataclass(frozen=True)
class UndersteerBudget:
    """Contributions to the understeer gradient [rad/(m/s^2)]."""

    tyres: float
    roll_steer: float
    compliance_steer: float

    @property
    def total(self) -> float:
        return self.tyres + self.roll_steer + self.compliance_steer

    def deg_per_g(self, g: float = 9.81) -> dict[str, float]:
        f = math.degrees(1.0) * g
        return {"tyres": self.tyres * f, "roll_steer": self.roll_steer * f,
                "compliance_steer": self.compliance_steer * f, "total": self.total * f}


def understeer_budget(model, mu: float = 1.0) -> UndersteerBudget:
    """Budget of a :class:`~vehicle_model.vehicle_models.SevenDOFModel` (K&C terms zero without a chassis)."""
    p = model.params
    tyres = LinearSingleTrack(linear_params_from_seven_dof(p, model.tyre_front, model.tyre_rear, mu)).understeer_gradient
    chassis = model.chassis
    if chassis is None:
        return UndersteerBudget(tyres, 0.0, 0.0)
    eps_f, eps_r = chassis.roll_steer_coefficients()
    roll = (eps_r - eps_f) * chassis.roll_gradient
    kc = chassis.kc
    L = p.wheelbase
    compliance = kc.compliance_steer_front * p.mass * p.lr / L + kc.compliance_steer_rear * p.mass * p.lf / L
    return UndersteerBudget(tyres, roll, compliance)


__all__ = ("UndersteerBudget", "understeer_budget")
