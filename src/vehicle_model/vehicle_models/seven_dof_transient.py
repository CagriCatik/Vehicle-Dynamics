"""7-DOF two-track model with first-order tyre relaxation.

Opt-in variant of :class:`SevenDOFModel`: the slips fed to the tyre model are
states (``kappa'``, ``tan(alpha')`` per wheel) governed by
:class:`~vehicle_model.tire_model.relaxation.TyreRelaxation`. Everything else
(tyre models, loads, body and wheel equations) is the base model, unchanged.

State vector (18): the 10 base states, then ``kappa'`` (4) and ``tan(alpha')`` (4).
In steady state the transient slips equal the quasi-static ones, so steady
manoeuvres reproduce the base model; as the relaxation lengths go to zero,
transient manoeuvres converge to it as well.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from vehicle_model.core.types import SevenDOFControlInput, SevenDOFState, WheelTelemetry
from vehicle_model.tire_model.relaxation import TyreRelaxation
from vehicle_model.vehicle_models.seven_dof import _V_EPS, SevenDOFModel


@dataclass
class TransientSevenDOFState(SevenDOFState):
    """Base state plus transient slips ``kappa_t`` and ``tan_alpha_t`` (FL, FR, RL, RR)."""

    kappa_t: np.ndarray = field(default_factory=lambda: np.zeros(4, dtype=float))
    tan_alpha_t: np.ndarray = field(default_factory=lambda: np.zeros(4, dtype=float))

    def __post_init__(self):
        super().__post_init__()
        self.kappa_t = np.asarray(self.kappa_t, dtype=float)
        self.tan_alpha_t = np.asarray(self.tan_alpha_t, dtype=float)
        if self.kappa_t.shape != (4,) or self.tan_alpha_t.shape != (4,):
            raise ValueError("kappa_t and tan_alpha_t must have shape (4,)")

    def to_array(self) -> np.ndarray:
        return np.concatenate([super().to_array(), self.kappa_t, self.tan_alpha_t])

    @classmethod
    def from_array(cls, arr) -> TransientSevenDOFState:
        a = np.asarray(arr, dtype=float)
        if a.size != 18:
            raise ValueError(f"Expected array of size 18, got {a.size}")
        base = SevenDOFState.from_array(a[:10])
        return cls(vx=base.vx, vy=base.vy, r=base.r, omega=base.omega, X=base.X, Y=base.Y, psi=base.psi,
                   kappa_t=a[10:14].copy(), tan_alpha_t=a[14:18].copy())


class TransientSevenDOFModel(SevenDOFModel):
    """:class:`SevenDOFModel` with relaxation-length tyre dynamics."""

    n_states = 18
    state_type = TransientSevenDOFState

    def __init__(self, *args, relaxation: TyreRelaxation | None = None, **kwargs):
        super().__init__(*args, **kwargs)
        self.relaxation = relaxation or TyreRelaxation()
        if self.relaxation.v_eps != _V_EPS:
            raise ValueError(f"relaxation.v_eps must equal the slip regularisation speed {_V_EPS} m/s")

    @classmethod
    def from_yaml(cls, *args, relaxation: TyreRelaxation | None = None, **kwargs) -> TransientSevenDOFModel:
        base = SevenDOFModel.from_yaml(*args, **kwargs)
        return cls(base.params, base.tyre_front, base.road, base.tyre_rear, chassis=base.chassis, relaxation=relaxation)

    # ----------------------------------------------------------------- hooks
    def _slips(self, state, v_wx, v_wy, v_wheel):
        return state.kappa_t.copy(), np.arctan(state.tan_alpha_t)

    def _extra_derivatives(self, state, v_wx, v_wy, v_wheel):
        dk, ds = self.relaxation.derivatives(state.kappa_t, state.tan_alpha_t, v_wx, v_wy, v_wheel)
        return np.concatenate([dk, ds])

    # ------------------------------------------------------------ utilities
    def steady_slip_state(self, state: SevenDOFState, u: SevenDOFControlInput) -> TransientSevenDOFState:
        """``state`` with the transient slips set to their steady (quasi-static) values.

        With K&C the wheel angles depend on the forces, so the quasi-static model
        is evaluated once to obtain the wheel velocities.
        """
        quasi_static = SevenDOFModel(self.params, self.tyre_front, self.road, self.tyre_rear, chassis=self.chassis)
        _, tel, _, _ = quasi_static.evaluate_dynamics(SevenDOFState.from_array(state.to_array()[:10]), u.clamped(self.params))
        k, s = self.relaxation.steady_state(tel.v_wx, tel.v_wy, state.omega * self.params.Reff)
        return TransientSevenDOFState(vx=state.vx, vy=state.vy, r=state.r, omega=state.omega.copy(),
                                      X=state.X, Y=state.Y, psi=state.psi, kappa_t=k, tan_alpha_t=s)

    def wheel_eigenvalue(self, state: SevenDOFState, telemetry: WheelTelemetry) -> float:
        """Bound on the fastest eigenvalue of wheel spin + slip relaxation [1/s].

        Linearised per wheel: ``lambda^2 + (V/sigma) lambda + C_k R^2/(Iw sigma) = 0``,
        hence ``|lambda| <= V/sigma + sqrt(C_k R^2/(Iw sigma))``; the lateral slip pole is ``V_x/sigma_a``.
        """
        p, rel = self.params, self.relaxation
        C_k = np.concatenate([
            self.tyre_front.slip_stiffness(telemetry.Fz[:2], telemetry.mu[:2]),
            self.tyre_rear.slip_stiffness(telemetry.Fz[2:], telemetry.mu[2:]),
        ])
        V_ref, V_x = rel.reference_speeds(telemetry.v_wx, state.omega * p.Reff)
        lam_k = V_ref / rel.sigma_kappa + np.sqrt(C_k * p.Reff**2 / (p.Iw * rel.sigma_kappa))
        return float(max(np.max(lam_k), np.max(V_x) / rel.sigma_alpha))


__all__ = ("TransientSevenDOFModel", "TransientSevenDOFState")
