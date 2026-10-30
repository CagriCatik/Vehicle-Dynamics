"""Road preview, four-channel (predictive) semi-active damping and active pitch (14-DOF only).

**Preview is an explicitly modelled external sensor**, never a future plant state: ``RoadPreview`` samples the
road height profiles (the same objects that excite the plant, i.e. the "real road") at a look-ahead distance in
front of each front wheel, with declared latency, Gaussian noise (seeded) and an optional failure. The rear
wheels use the front measurement of their track, delayed by ``wheelbase / v`` (each wheel has its own contact
position and preview time).

Damping controllers per corner (the 14-DOF ``damper_c_override`` input; passive dissipative dampers:
``c_min <= c <= c_max``, never negative):

    passive          the configuration's damping
    skyhook          the existing SkyhookController (continuous) per corner
    groundhook       the existing GroundhookController per corner
    hybrid           the existing HybridSuspensionController per corner
    preview_skyhook  skyhook, but a corner whose previewed road velocity within ``window`` exceeds ``v_soft`` and
                     ``event_factor`` times its running RMS (an event standing out of the road roughness) is set
                     soft (``c_min``) just before and while the input passes, so it is not transmitted into the
                     body; with the preview failed it falls back to plain skyhook. The road velocity is the slope
                     of the samples smoothed over ``baseline`` (adjacent noisy samples would read noise as input)

Active pitch (``ActivePitchController``): PD on pitch angle/rate and heave, demand ``(F_heave, M_pitch, M_roll)``
allocated to the four active strut forces (``allocation.vertical_forces``), bounded per strut and by a total
power limit ``sum |F_i v_rel,i| <= P_max`` (forces scaled down when exceeded).
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass

import numpy as np

DAMPING_MODES = ("passive", "skyhook", "groundhook", "hybrid", "preview_skyhook")


@dataclass(frozen=True)
class PreviewConfig:
    distance: float = 8.0           # [m] look-ahead in front of the front axle
    latency: float = 0.02           # [s] measurement-to-use delay
    noise_std: float = 0.0005       # [m] height noise
    seed: int = 0
    failed: bool = False


class RoadPreview:
    """External road-height preview per front track; rear wheels reuse it delayed by the wheelbase."""

    def __init__(self, profiles, config: PreviewConfig | None = None):
        self.left, self.right = profiles
        self.config = config or PreviewConfig()
        self.rng = np.random.default_rng(self.config.seed)
        self._buf = deque()

    def measure(self, t: float, x_front: float):
        """Called every control step: stores the noisy look-ahead sample (time-stamped)."""
        if self.config.failed:
            return
        x = x_front + self.config.distance
        n = self.rng.normal(0.0, self.config.noise_std, 2)
        self._buf.append((t, x, float(self.left.height(x)[0] + n[0]), float(self.right.height(x)[0] + n[1])))

    def heights_ahead(self, t: float, x_from: float, x_to: float):
        """Usable samples (older than the latency) with ``x_from <= x <= x_to``: (x, zL, zR)."""
        if self.config.failed:
            return None
        avail = [s[1:] for s in self._buf if s[0] <= t - self.config.latency and x_from <= s[1] <= x_to]
        while self._buf and self._buf[0][1] < x_from - 30.0:        # forget what lies far behind
            self._buf.popleft()
        return avail


@dataclass(frozen=True)
class DampingConfig:
    mode: str = "passive"
    c_min: float = 800.0            # [N s/m]
    c_max: float = 4800.0
    c_sky: float = 3200.0
    v_soft: float = 0.15            # [m/s] previewed road velocity that triggers the soft setting
    window: float = 0.12            # [s] preview window ahead of each wheel
    baseline: float = 0.3           # [m] smoothing length / difference baseline of the previewed slope
    event_factor: float = 3.0       # soften only if the previewed road velocity also exceeds this multiple of its
                                    # running RMS (an event standing out of the road roughness); 0: fixed threshold
    rms_time: float = 1.0           # [s] time constant of that running RMS
    damper_failed: tuple = ()       # corners whose damper is stuck soft (c_min)


class CornerDamping:
    def __init__(self, config: DampingConfig, nominal_c, wheelbase: float):
        from vehicle_model.suspension.controllers import (GroundhookController, HybridSuspensionController,
                                                          SemiActiveConfig, SkyhookController)
        if config.mode not in DAMPING_MODES:
            raise ValueError(f"damping mode must be one of {DAMPING_MODES}")
        self.c = config
        self.nominal = np.asarray(nominal_c, float)
        self.L = float(wheelbase)
        sa = SemiActiveConfig(c_sky=config.c_sky, c_min=config.c_min, c_max=config.c_max)
        kind = {"skyhook": SkyhookController, "preview_skyhook": SkyhookController, "groundhook": GroundhookController,
                "hybrid": HybridSuspensionController}.get(config.mode)
        self.corner = [kind(sa) for _ in range(4)] if kind else None
        self.soft = np.zeros(4, dtype=bool)
        self.rate_ms = None                                  # running mean square of the previewed road velocity

    def command(self, dzs_c, dzu, dt: float, preview_rows=None, x_front: float = 0.0, vx: float = 1.0):
        c = self.c
        if self.corner is None:
            cmd = self.nominal.copy()
        else:
            cmd = np.array([ctl.compute_damping(float(dzs_c[i]), float(dzu[i]), dt) for i, ctl in enumerate(self.corner)])
        self.soft[:] = False
        if c.mode == "preview_skyhook" and preview_rows:
            rows = np.array(preview_rows)
            if len(rows) >= 3:
                order = np.argsort(rows[:, 0])
                xs, zl, zr = rows[order, 0], rows[order, 1], rows[order, 2]
                v = max(abs(vx), 1.0)
                B = c.baseline
                for i, (off, z) in enumerate(((0.0, zl), (0.0, zr), (-self.L, zl), (-self.L, zr))):
                    xw = x_front + off                               # wheel position along the track
                    if xs[0] > xw or xs[-1] < xw + v * c.window + B:
                        continue
                    # smoothed slope over a baseline B (moving average of the samples within B, then a B-difference):
                    # a finite difference of adjacent noisy samples (3 cm apart) would read sensor noise as road
                    # velocity (0.5 mm noise -> ~0.25 m/s at 60 km/h)
                    grid = np.arange(xw - B, xw + v * c.window + B + 1e-9, B / 4.0)
                    cs = np.concatenate([[0.0], np.cumsum(z)])
                    lo, hi = np.searchsorted(xs, grid - B / 2.0), np.searchsorted(xs, grid + B / 2.0)
                    n = np.maximum(hi - lo, 1)
                    zs = (cs[hi] - cs[lo]) / n
                    rates = np.abs(zs[4:] - zs[:-4]) / B * v
                    rate = float(np.max(rates))
                    if i == 0:                                   # roughness reference from the front-left track
                        ms = float(np.mean(rates ** 2))
                        a = min(1.0, dt / c.rms_time)
                        self.rate_ms = ms if self.rate_ms is None else (1.0 - a) * self.rate_ms + a * ms
                    threshold = max(c.v_soft, c.event_factor * np.sqrt(self.rate_ms or 0.0))
                    if rate > threshold:
                        cmd[i] = c.c_min
                        self.soft[i] = True
        for name in c.damper_failed:
            cmd["FL FR RL RR".split().index(name)] = c.c_min
        return np.clip(cmd, c.c_min, max(c.c_max, float(np.max(self.nominal))))


@dataclass(frozen=True)
class PitchConfig:
    k_pitch: float = 60000.0        # [N m/rad]
    c_pitch: float = 9000.0         # [N m s/rad]
    k_heave: float = 0.0            # [N/m]
    c_heave: float = 1500.0         # [N s/m]
    c_roll: float = 2000.0          # [N m s/rad] roll damping (no roll stiffness: that is RSC/active roll)
    f_max: float = 3000.0           # [N] per strut
    p_max: float = 4000.0           # [W] total active power


class ActivePitchController:
    def __init__(self, corner_x, corner_y, config: PitchConfig | None = None):
        from vehicle_model.allocation.vertical_forces import VerticalForceAllocator
        self.c = config or PitchConfig()
        self.alloc = VerticalForceAllocator(corner_x, corner_y, self.c.f_max)
        self.power = 0.0

    def forces(self, state, rel_vel) -> np.ndarray:
        c = self.c
        M_pitch = -c.k_pitch * state.theta - c.c_pitch * state.dtheta
        F_heave = -c.k_heave * state.zs - c.c_heave * state.dzs
        M_roll = -c.c_roll * state.dphi
        F = self.alloc.allocate(F_heave, M_pitch, M_roll).forces
        p = float(np.sum(np.abs(F * np.asarray(rel_vel, float))))
        if p > c.p_max:
            F = F * (c.p_max / p)
            p = c.p_max
        self.power = p
        return F


__all__ = ("ActivePitchController", "CornerDamping", "DAMPING_MODES", "DampingConfig", "PitchConfig", "PreviewConfig",
           "RoadPreview")
