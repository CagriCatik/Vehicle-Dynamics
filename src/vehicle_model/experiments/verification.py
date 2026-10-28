"""Physics and numerics checks on the configured vehicle (Plan.md M9).

Run for every experiment (unless disabled) and reported next to the results, so
a comparison is only read together with evidence that the configured model is
consistent. Each check returns ``{name, passed, value, tolerance, description}``.

1. parameter consistency      vehicle_models/consistency.py (bands and mass/roll splits)
2. static wheel loads         sum = m g, front share = l_r / L (exact)
3. tyre symmetry              F_y(kappa, alpha = 0) = 0 for the configured tyres
4. linear reference           7-DOF steady yaw rate vs v delta/(L + EG v^2) at a_y = 0.3 m/s^2, 20 m/s,
                              EG = linear understeer budget (tyres, roll steer, compliance steer)
5. mirror symmetry            steer step +delta vs -delta: yaw rates opposite
6. determinism                the same short run twice gives the identical state
"""

from __future__ import annotations

import numpy as np

from vehicle_model.core.types import SevenDOFControlInput, SevenDOFState
from vehicle_model.handling_analysis import understeer_budget
from vehicle_model.vehicle_models.consistency import check_vehicle_consistency
from vehicle_model.vehicle_models.normal_loads import quasi_static_normal_loads


def _check(name, passed, value, tolerance, description):
    return {"name": name, "passed": bool(passed), "value": value, "tolerance": tolerance, "description": description}


def _run(model, steer, T, v0=20.0, dt=0.005):
    p = model.params
    s = SevenDOFState(vx=v0, omega=np.full(4, v0 / p.Reff))
    for _ in range(int(round(T / dt))):
        f = 0.5 * p.rho * p.Cd * p.Af * s.vx**2 + p.c_rr * p.mass * p.g + 2.0 * p.mass * (v0 - s.vx)
        s, _ = model.step(s, SevenDOFControlInput(steer_f=steer, T_drive=np.full(4, f * p.Reff / 4)), dt)
    return s


def verify_model(model_factory) -> list[dict]:
    """``model_factory()`` returns a fresh SevenDOFModel on a uniform mu = 1 road."""
    out = []
    m = model_factory()
    p = m.params

    issues = check_vehicle_consistency(p)
    out.append(_check("parameter consistency", not issues, len(issues), 0,
                      "plausibility bands and mass/roll-stiffness splits; " + ("; ".join(issues) or "no findings")))

    Fz = quasi_static_normal_loads(p, 0.0, 0.0).Fz
    err = max(abs(Fz.sum() - p.mass * p.g) / (p.mass * p.g), abs((Fz[0] + Fz[1]) / Fz.sum() - p.lr / p.wheelbase))
    out.append(_check("static wheel loads", err < 1e-12, err, 1e-12, "sum = m g and front share = l_r/L"))

    kap = np.array([-0.2, -0.05, 0.0, 0.05, 0.2])
    pull = max(float(np.max(np.abs(t.forces(kap, 0 * kap, 3500 + 0 * kap, 1 + 0 * kap)[1]))) for t in (m.tyre_front, m.tyre_rear))
    out.append(_check("tyre symmetry", pull < 1e-6, pull, 1e-6, "lateral force at zero slip angle under longitudinal slip [N]"))

    EG = understeer_budget(m).total
    v, L = 20.0, p.wheelbase
    delta = 0.3 * (L + EG * v**2) / v**2
    r_sim = _run(m, delta, 4.0, v).r
    dev = abs(r_sim / (v * delta / (L + EG * v**2)) - 1.0)
    out.append(_check("linear reference", dev < 0.01, dev, 0.01,
                      "steady yaw rate vs linear single track with the understeer budget, a_y = 0.3 m/s^2, 20 m/s"))

    a, b = _run(model_factory(), 0.03, 0.5), _run(model_factory(), -0.03, 0.5)
    sym = abs(a.r + b.r) / max(abs(a.r), 1e-9)
    out.append(_check("mirror symmetry", sym < 1e-9, sym, 1e-9, "yaw rate after +/- steer step"))

    c, d = _run(model_factory(), 0.03, 0.3), _run(model_factory(), 0.03, 0.3)
    same = bool(np.array_equal(c.to_array(), d.to_array()))
    out.append(_check("determinism", same, 0.0 if same else 1.0, 0.0, "identical state after two identical runs"))
    return out


__all__ = ("verify_model",)
