"""Tyre characteristic audit over the large-slip range a drift needs (Drift plan D1, section 4.4).

Evaluates any :class:`~vehicle_model.tire_model.TyreModel` far beyond the linear range and
checks it for properties a drift equilibrium relies on, without changing the tyre:

    finite          no NaN/Inf, also at zero/low friction, zero load, |alpha| up to 89 deg, |kappa| = 1
    sign            F_y has the sign of alpha (kappa = 0), F_x the sign of kappa (alpha = 0)
    symmetry        F_y(-alpha) = -F_y(alpha) (required); F_x(-kappa) = -F_x(kappa) reported (brush models
                    use an asymmetric theoretical slip by design)
    continuity      the largest step between grid points must shrink on a 10x finer grid (a jump does not)
    zero friction   forces vanish for mu -> 0; zero load gives zero force
    friction        largest sqrt(F_x^2 + F_y^2) / (mu F_z) over the combined-slip map (reported; the
                    synthetic MF tyre can exceed 1 at light loads by its load sensitivity)
    post-peak       slip angle of the lateral peak and the remaining force at the largest angle
    validity        share of the audited grid outside the tyre's declared validity range
                    (``provenance.valid_alpha`` / ``valid_kappa``): there the model extrapolates

Slip definitions are the plant's: ``alpha`` in rad, ``kappa`` the regularised longitudinal slip
of :class:`~vehicle_model.vehicle_models.SevenDOFModel`, ``(v_wheel - v_x) / max(|v_x|, |v_wheel|)``,
which is bounded to [-1, 1] (``kappa = 1``: wheel spinning, vehicle at rest relative to it).

The audit returns data only; plots are in :mod:`vehicle_model.visualization.drift_plots`.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


@dataclass(frozen=True)
class AuditSettings:
    loads: tuple = (2000.0, 4000.0, 6000.0)              # [N]
    mus: tuple = (1.0, 0.6)
    lateral_kappas: tuple = (0.0, 0.05, 0.1, 0.3)        # combined slip on the F_y(alpha) curves
    longitudinal_alphas_deg: tuple = (0.0, 5.0, 15.0, 30.0)
    alpha_max_deg: float = 45.0
    alpha_points: int = 181                              # 0.5 deg resolution over +-45 deg
    kappa_points: int = 201                              # 0.01 resolution over [-1, 1]
    continuity_tol: float = 0.3                          # max ratio of the largest step, 10x finer / audit grid
    friction_tol: float = 0.25                           # reported excess allowed over mu F_z


@dataclass
class TyreAudit:
    name: str
    provenance: str
    settings: AuditSettings
    alpha: np.ndarray                                    # [rad] grid of the lateral curves
    kappa: np.ndarray                                    # grid of the longitudinal curves
    lateral: dict = field(default_factory=dict)          # (Fz, mu, kappa) -> Fy(alpha)
    longitudinal: dict = field(default_factory=dict)     # (Fz, mu, alpha_deg) -> Fx(kappa)
    utilisation: np.ndarray | None = None                # (alpha 0..max, kappa) at the reference load, mu = 1
    checks: list = field(default_factory=list)           # {name, passed (True/False/None = info), value, detail}
    valid_alpha: tuple = (-np.inf, np.inf)               # [rad] declared validity of the tyre
    valid_kappa: tuple = (-np.inf, np.inf)

    @property
    def passed(self) -> bool:
        return all(c["passed"] is not False for c in self.checks)

    def check(self, name: str) -> dict:
        return next(c for c in self.checks if c["name"] == name)

    def summary(self) -> dict:
        return {"name": self.name, "provenance": self.provenance, "passed": self.passed, "checks": self.checks}


def _forces(tyre, kappa, alpha, Fz, mu):
    kappa, alpha, Fz, mu = np.broadcast_arrays(*(np.asarray(x, dtype=float) for x in (kappa, alpha, Fz, mu)))
    Fx, Fy = tyre.forces(kappa.ravel(), alpha.ravel(), Fz.ravel(), mu.ravel())
    return np.asarray(Fx, float).reshape(kappa.shape), np.asarray(Fy, float).reshape(kappa.shape)


def _check(name, passed, value, detail):
    return {"name": name, "passed": passed, "value": float(value) if value is not None else None, "detail": detail}


def audit_tyre(tyre, name: str = "", settings: AuditSettings | None = None) -> TyreAudit:
    """Curves, combined-slip map and checks of ``tyre`` (see module docstring)."""
    s = settings or AuditSettings()
    a_max = np.radians(s.alpha_max_deg)
    alpha = np.linspace(-a_max, a_max, s.alpha_points)
    kappa = np.linspace(-1.0, 1.0, s.kappa_points)
    prov = getattr(tyre, "provenance", None)
    rep = TyreAudit(name or type(tyre).__name__, prov.describe() if prov else "unknown", s, alpha, kappa,
                    valid_alpha=tuple(getattr(prov, "valid_alpha", (-np.inf, np.inf))),
                    valid_kappa=tuple(getattr(prov, "valid_kappa", (-np.inf, np.inf))))

    for Fz in s.loads:
        for mu in s.mus:
            for k in s.lateral_kappas:
                rep.lateral[(Fz, mu, k)] = _forces(tyre, k, alpha, Fz, mu)[1]
            for a in s.longitudinal_alphas_deg:
                rep.longitudinal[(Fz, mu, a)] = _forces(tyre, kappa, np.radians(a), Fz, mu)[0]
    Fz0 = float(getattr(prov, "reference_load", 4000.0))
    A, K = np.meshgrid(np.linspace(0.0, a_max, 91), kappa, indexing="ij")
    Fx, Fy = _forces(tyre, K, A, Fz0, 1.0)
    rep.utilisation = np.hypot(Fx, Fy) / Fz0

    # finiteness, including the edges of the domain
    edge = [_forces(tyre, k, a, Fz, mu) for k in (-1.0, 0.0, 1.0) for a in np.radians([-89.0, 0.0, 45.0, 89.0])
            for Fz in (0.0, 10.0, 4000.0, 20000.0) for mu in (0.0, 1e-3, 1.0, 1.5)]
    all_curves = [*rep.lateral.values(), *rep.longitudinal.values(), rep.utilisation, *(x for e in edge for x in e)]
    finite = all(np.all(np.isfinite(c)) for c in all_curves)
    rep.checks.append(_check("finite", finite, None, "no NaN/Inf on the grids and at mu = 0, F_z = 0, |alpha| = 89 deg"))

    # sign and symmetry at kappa = 0 / alpha = 0
    sign_y, sign_x, sym_y, sym_x = [], [], [], []
    for Fz in s.loads:
        for mu in s.mus:
            fy = rep.lateral[(Fz, mu, 0.0)]
            nz = np.abs(alpha) > 1e-9
            sign_y.append(np.all(np.sign(fy[nz]) == np.sign(alpha[nz])))
            sym_y.append(float(np.max(np.abs(fy + fy[::-1]))) / (mu * Fz))
            fx = rep.longitudinal[(Fz, mu, 0.0)]
            nzk = np.abs(kappa) > 1e-9
            sign_x.append(np.all(np.sign(fx[nzk]) == np.sign(kappa[nzk])))
            sym_x.append(float(np.max(np.abs(fx + fx[::-1]))) / (mu * Fz))
    rep.checks.append(_check("sign", all(sign_y) and all(sign_x), None,
                             "F_y has the sign of alpha (kappa = 0), F_x the sign of kappa (alpha = 0)"))
    rep.checks.append(_check("symmetry_lateral", max(sym_y) < 1e-9, max(sym_y),
                             "max |F_y(-alpha) + F_y(alpha)| / (mu F_z), kappa = 0"))
    rep.checks.append(_check("symmetry_longitudinal", None, max(sym_x),
                             "max |F_x(-kappa) + F_x(kappa)| / (mu F_z), alpha = 0; reported only: brush models use the "
                             "theoretical slip kappa/(1+kappa), asymmetric between driving and braking by design"))

    # continuity: a continuous curve's largest step shrinks with the grid, a jump does not
    ratios = []
    for Fz in s.loads:
        for mu in s.mus:
            for grid, f in ((alpha, lambda x: _forces(tyre, 0.0, x, Fz, mu)[1]),
                            (kappa, lambda x: _forces(tyre, x, 0.0, Fz, mu)[0])):
                fine = np.linspace(grid[0], grid[-1], 10 * (grid.size - 1) + 1)
                coarse_step = float(np.max(np.abs(np.diff(f(grid)))))
                fine_step = float(np.max(np.abs(np.diff(f(fine)))))
                ratios.append(fine_step / max(coarse_step, 1e-12))
    rep.checks.append(_check("continuity", max(ratios) <= s.continuity_tol, max(ratios),
                             f"largest step on a 10x finer grid / largest step on the audit grid (continuous: ~0.1, "
                             f"jump: ~1); limit {s.continuity_tol}"))

    # zero friction and zero load
    fx0, fy0 = _forces(tyre, 0.3, np.radians(10.0), 4000.0, 0.0)
    fxl, fyl = _forces(tyre, 0.3, np.radians(10.0), 0.0, 1.0)
    rep.checks.append(_check("zero_friction", max(abs(float(fx0)), abs(float(fy0))) < 1e-6, max(abs(float(fx0)), abs(float(fy0))),
                             "|F| at mu = 0 (kappa 0.3, alpha 10 deg, 4 kN)"))
    rep.checks.append(_check("zero_load", max(abs(float(fxl)), abs(float(fyl))) < 1e-6, max(abs(float(fxl)), abs(float(fyl))),
                             "|F| at F_z = 0"))

    # friction utilisation over all loads (combined map at every load, mu = 1)
    util = []
    for Fz in s.loads:
        fx, fy = _forces(tyre, K, A, Fz, 1.0)
        util.append(float(np.max(np.hypot(fx, fy))) / Fz)
    worst = max(util)
    rep.checks.append(_check("friction", worst <= 1.0 + s.friction_tol, worst,
                             f"max sqrt(Fx^2+Fy^2)/(mu Fz) over the combined map, per load {[round(u, 3) for u in util]}; "
                             f"> 1 is the load sensitivity of the synthetic tyre, reported not hidden"))

    # post-peak behaviour of the lateral force (kappa = 0, mu = 1)
    pos = alpha > 0
    peaks, drops = [], []
    for Fz in s.loads:
        fy = rep.lateral[(Fz, 1.0, 0.0)][pos]
        i = int(np.argmax(fy))
        peaks.append(float(np.degrees(alpha[pos][i])))
        drops.append(float(fy[-1] / fy[i]))
    rep.checks.append(_check("post_peak", None, min(drops),
                             f"lateral peak at {[round(p, 1) for p in peaks]} deg (loads {list(s.loads)}); force at "
                             f"{s.alpha_max_deg:g} deg = {[round(d, 3) for d in drops]} x peak"))

    # declared validity range against the audited grid
    va = getattr(prov, "valid_alpha", (-np.inf, np.inf))
    vk = getattr(prov, "valid_kappa", (-np.inf, np.inf))
    out_a = float(np.mean((alpha < va[0]) | (alpha > va[1])))
    out_k = float(np.mean((kappa < vk[0]) | (kappa > vk[1])))
    rep.checks.append(_check("validity_range", None, out_a,
                             f"declared validity alpha {np.degrees(va[0]):.0f}..{np.degrees(va[1]):.0f} deg, kappa "
                             f"{vk[0]:g}..{vk[1]:g}; {out_a:.0%} of the audited alpha grid and {out_k:.0%} of the kappa grid "
                             f"lie outside: there the model extrapolates (warning, not a failure)"))
    return rep


def extrapolating(tyre, alpha, kappa) -> np.ndarray:
    """Per wheel: True where ``(alpha, kappa)`` is outside the tyre's declared validity range."""
    prov = getattr(tyre, "provenance", None)
    va = getattr(prov, "valid_alpha", (-np.inf, np.inf))
    vk = getattr(prov, "valid_kappa", (-np.inf, np.inf))
    a, k = np.asarray(alpha, float), np.asarray(kappa, float)
    return (a < va[0]) | (a > va[1]) | (k < vk[0]) | (k > vk[1])


__all__ = ("AuditSettings", "TyreAudit", "audit_tyre", "extrapolating")
