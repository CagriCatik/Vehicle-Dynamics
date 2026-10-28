"""Quasi-static wheel loads with unilateral contact (Plan.md, Abschnitt 8; Audit 4.3).

Rigid chassis on level ground, four contact points ``(x_i, y_i)`` (body frame,
rows FL, FR, RL, RR). The loads must satisfy three equilibrium conditions

    sum Fz_i         = m g
    sum x_i Fz_i     = -m a_x h          (pitch: braking moves load forward)
    sum y_i Fz_i     = -m a_y h          (roll: left turn moves load to the right)

i.e. the centre of pressure lies at ``(-a_x h/g, -a_y h/g)`` relative to the CoG.
Four unknowns, three equations: the remaining freedom is the diagonal *warp*
mode ``q = (1/w_f, -1/w_f, -1/w_r, 1/w_r)``, which changes no resultant.
The roll-stiffness split ``k_roll_f`` fixes the warp coordinate
(``F* = F0 + w* q``, the classic four-wheel formula).

Contact is unilateral (``Fz_i >= 0``). Treating the warp as an elastic mode
(energy ~ (w - w*)^2) the consistent loads are ``F = F* + clip(0, lo, hi) q``,
where ``[lo, hi]`` is the interval of warp offsets keeping every load
non-negative:

* ``lo <= 0 <= hi``  -> all four wheels carry load (``support = "four_wheel"``),
  identical to the classic formula.
* ``0`` outside      -> the bound wheel lifts; the remaining three loads are the
  statically determinate three-point support (``"three_wheel"``). The roll
  stiffness split is then no longer enforced; this is physical (an anti-roll bar
  cannot pull a wheel down). Force and both moment balances stay exact, and the
  transition is continuous.
* ``lo > hi``        -> the centre of pressure lies outside the support polygon:
  the vehicle would tip over (``"tip_over"``). No admissible load set exists.
  The loads are those of the nearest boundary point of the support polygon,
  shared between the two wheels of that edge; the moment the contact cannot
  carry is reported in ``moment_residual``. Results in this state are outside
  the model's validity.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from vehicle_model.core.types import SevenDOFParams

SUPPORT_FOUR_WHEEL = "four_wheel"
SUPPORT_THREE_WHEEL = "three_wheel"
SUPPORT_TIP_OVER = "tip_over"

_ZERO_TOL = 1e-9  # relative to m g: loads below this count as "not in contact"


@dataclass(frozen=True)
class NormalLoads:
    """Wheel loads and contact state, rows FL, FR, RL, RR."""

    Fz: np.ndarray               # [N], >= 0
    in_contact: np.ndarray       # bool per wheel
    support: str                 # SUPPORT_FOUR_WHEEL | SUPPORT_THREE_WHEEL | SUPPORT_TIP_OVER
    moment_residual: np.ndarray  # [N m] (sum x Fz, sum y Fz) minus required; zero unless tip_over


def wheel_contact_positions(p: SevenDOFParams) -> np.ndarray:
    """Contact-point positions [x, y] in the body frame, rows FL, FR, RL, RR."""
    return np.array([
        [p.lf, p.wf / 2.0],
        [p.lf, -p.wf / 2.0],
        [-p.lr, p.wr / 2.0],
        [-p.lr, -p.wr / 2.0],
    ], dtype=float)


def _closest_point_on_polygon(c: np.ndarray, poly: np.ndarray, order: tuple[int, ...]):
    """Nearest point to ``c`` on the polygon boundary; returns (i, j, t) with point = (1-t) P_i + t P_j."""
    best = None
    for k in range(len(order)):
        i, j = order[k], order[(k + 1) % len(order)]
        a, b = poly[i], poly[j]
        ab = b - a
        t = float(np.clip(np.dot(c - a, ab) / np.dot(ab, ab), 0.0, 1.0))
        d = float(np.linalg.norm(a + t * ab - c))
        if best is None or d < best[0]:
            best = (d, i, j, t)
    return best[1], best[2], best[3]


def quasi_static_normal_loads(p: SevenDOFParams, ax: float, ay: float) -> NormalLoads:
    """Wheel loads for body accelerations ``ax, ay`` [m/s^2] (see module docstring)."""
    L = p.wheelbase
    mg = p.mass * p.g
    pos = wheel_contact_positions(p)

    # Classic four-wheel distribution (roll stiffness split k_roll_f)
    d_long = p.mass * ax * p.hcog / L                      # ax > 0 -> load to rear
    d_lat_f = p.mass * ay * p.hcog / p.wf * p.k_roll_f     # ay > 0 (left turn) -> load to right
    d_lat_r = p.mass * ay * p.hcog / p.wr * (1.0 - p.k_roll_f)
    Fz_f = 0.5 * (mg * p.lr / L - d_long)
    Fz_r = 0.5 * (mg * p.lf / L + d_long)
    F_star = np.array([Fz_f - d_lat_f, Fz_f + d_lat_f, Fz_r - d_lat_r, Fz_r + d_lat_r])

    if np.all(F_star >= 0.0):
        return NormalLoads(F_star, F_star > _ZERO_TOL * mg, SUPPORT_FOUR_WHEEL, np.zeros(2))

    # Admissible warp offsets: F_star + w q >= 0
    q = np.array([1.0 / p.wf, -1.0 / p.wf, -1.0 / p.wr, 1.0 / p.wr])
    bound = -F_star / q
    up, down = np.flatnonzero(q > 0), np.flatnonzero(q < 0)
    i_lo, i_hi = up[np.argmax(bound[up])], down[np.argmin(bound[down])]
    lo, hi = float(bound[i_lo]), float(bound[i_hi])
    if lo <= hi:
        lifted = i_lo if lo > 0.0 else i_hi  # F_star has a negative entry, so 0 lies outside [lo, hi]
        Fz = F_star + bound[lifted] * q
        Fz[lifted] = 0.0  # exact zero instead of round-off
        return NormalLoads(Fz, Fz > _ZERO_TOL * mg, SUPPORT_THREE_WHEEL, np.zeros(2))

    # Tip-over: centre of pressure outside the support polygon (FL, FR, RR, RL)
    c = np.array([-ax * p.hcog / p.g, -ay * p.hcog / p.g])
    i, j, t = _closest_point_on_polygon(c, pos, (0, 1, 3, 2))
    Fz = np.zeros(4)
    Fz[i] += mg * (1.0 - t)
    Fz[j] += mg * t
    residual = pos.T @ Fz - mg * c
    return NormalLoads(Fz, Fz > _ZERO_TOL * mg, SUPPORT_TIP_OVER, residual)


__all__ = (
    "NormalLoads",
    "SUPPORT_FOUR_WHEEL",
    "SUPPORT_THREE_WHEEL",
    "SUPPORT_TIP_OVER",
    "quasi_static_normal_loads",
    "wheel_contact_positions",
)
