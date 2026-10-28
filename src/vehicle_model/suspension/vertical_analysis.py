"""Analytical references for the quarter car: modes, frequency responses, energy (Plan.md, Abschnitt 9; M8).

Linear quarter car (bump stops inactive), coordinates measured from static
equilibrium, road displacement ``z_r`` as input:

    m_s z_s'' = -k_s (z_s - z_u) - c_s (z_s' - z_u')
    m_u z_u'' =  k_s (z_s - z_u) + c_s (z_s' - z_u') - k_t (z_u - z_r) - c_t (z_u' - z_r')

State ``x = [z_s, z_s', z_u, z_u']``, input ``u = [z_r, z_r']``. For a harmonic road
``z_r = Z e^{j w t}`` (so ``z_r' = j w z_r``) the responses per unit road amplitude are

    H_x(jw)      = (jw I - A)^{-1} (B_z + jw B_v)
    body acceleration   H_a = -w^2 H_zs
    suspension travel   H_d = H_zs - H_zu
    dynamic tyre load   H_F = -k_t (H_zu - 1) - c_t jw (H_zu - 1)     (upward contact-force change)

Energy (spring and kinetic, about equilibrium; gravity is balanced by the static
deflections and drops out):

    E = 1/2 m_s z_s'^2 + 1/2 m_u z_u'^2 + 1/2 k_s (z_s - z_u)^2 + 1/2 k_t (z_u - z_r)^2

    dE/dt = -c_s (z_s' - z_u')^2 - c_t (z_u' - z_r')^2 + P_road,
    P_road = -[k_t (z_u - z_r) + c_t (z_u' - z_r')] z_r'               (power fed in by the road)

So without damping and with a fixed road, E is conserved. Valid only while the
tyre stays in contact (the nonlinear model releases it, see quarter_car.py) and the
bump stops are not reached.
"""

from __future__ import annotations

import math

import numpy as np

from .quarter_car import QuarterCarConfig, QuarterCarState


def state_space(cfg: QuarterCarConfig):
    """``A`` (4x4), ``B_z`` and ``B_v`` (4,) for road displacement and road velocity."""
    ms, mu, ks, cs, kt, ct = (cfg.sprung_mass, cfg.unsprung_mass, cfg.suspension_stiffness,
                              cfg.suspension_damping, cfg.tyre_stiffness, cfg.tyre_damping)
    A = np.array([
        [0.0, 1.0, 0.0, 0.0],
        [-ks / ms, -cs / ms, ks / ms, cs / ms],
        [0.0, 0.0, 0.0, 1.0],
        [ks / mu, cs / mu, -(ks + kt) / mu, -(cs + ct) / mu],
    ])
    B_z = np.array([0.0, 0.0, 0.0, kt / mu])
    B_v = np.array([0.0, 0.0, 0.0, ct / mu])
    return A, B_z, B_v


def frequency_response(cfg: QuarterCarConfig, frequency_hz) -> dict:
    """Complex responses per unit road displacement at the given frequencies [Hz]."""
    A, B_z, B_v = state_space(cfg)
    f = np.atleast_1d(np.asarray(frequency_hz, dtype=float))
    H_zs, H_zu = np.empty(len(f), complex), np.empty(len(f), complex)
    for i, fi in enumerate(f):
        jw = 2j * math.pi * fi
        x = np.linalg.solve(jw * np.eye(4) - A, B_z + jw * B_v)
        H_zs[i], H_zu[i] = x[0], x[2]
    jw = 2j * math.pi * f
    return {
        "body_displacement": H_zs,
        "body_acceleration": jw**2 * H_zs,
        "suspension_travel": H_zs - H_zu,
        "dynamic_tyre_load": -(cfg.tyre_stiffness + cfg.tyre_damping * jw) * (H_zu - 1.0),
    }


def modes(cfg: QuarterCarConfig) -> list[tuple[float, float]]:
    """``(undamped natural frequency [Hz], damping ratio)`` of both modes, ascending."""
    A, _, _ = state_space(cfg)
    out = []
    for ev in np.linalg.eigvals(A):
        if ev.imag > 0:
            wn = abs(ev)
            out.append((wn / (2 * math.pi), -ev.real / wn))
    return sorted(out)


def energy(cfg: QuarterCarConfig, s: QuarterCarState, road_z: float = 0.0) -> float:
    return (0.5 * cfg.sprung_mass * s.dzs**2 + 0.5 * cfg.unsprung_mass * s.dzu**2
            + 0.5 * cfg.suspension_stiffness * (s.zs - s.zu) ** 2 + 0.5 * cfg.tyre_stiffness * (s.zu - road_z) ** 2)


def power_balance(cfg: QuarterCarConfig, s: QuarterCarState, road_z: float = 0.0, road_dz: float = 0.0,
                  damper_c: float | None = None) -> tuple[float, float]:
    """``(dissipated power >= 0, power fed in by the road)`` [W]."""
    cs = cfg.suspension_damping if damper_c is None else damper_c
    dissipation = cs * (s.dzs - s.dzu) ** 2 + cfg.tyre_damping * (s.dzu - road_dz) ** 2
    road = -(cfg.tyre_stiffness * (s.zu - road_z) + cfg.tyre_damping * (s.dzu - road_dz)) * road_dz
    return dissipation, road


__all__ = ("energy", "frequency_response", "modes", "power_balance", "state_space")
