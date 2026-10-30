"""Ride and pitch closed loop on the 14-DOF model (Plan V4.0 M21; the 7-DOF model has no vertical dynamics).

Road: two height tracks (left/right) along the travelled distance ``s`` (so the profile follows the path,
also in a corner): an ISO 8608 class (own seed per track) plus discrete events (half-sine bumps on one or both
tracks, a series of transverse grooves). Front and rear wheels see the track at ``s + x_i``.

Manoeuvres (longitudinal driver, open loop):

    cruise    drive torque holding the start speed (P on the speed error)
    brake     constant braking deceleration ``decel_g`` from ``start_s`` (equal brake torques)
    accel     constant total drive torque ``drive_torque`` from ``start_s`` (launch squat)
    corner    cruise with a constant front steer ``steer_deg`` (cornering on an uneven road)

Controllers: a damping mode (:mod:`vehicle_model.suspension.preview_control`), optional active pitch
(active strut forces through the vertical allocation), optional road preview (external sensor with latency,
noise, failure) and a failed damper (stuck soft). Plant step 1 ms, control every 2 ms, information mode A for the
body/wheel velocities; the preview is the only look-ahead information and comes from the road, not the plant.
"""

from __future__ import annotations

import dataclasses
import math

import numpy as np


class TrackProfile:
    """Sum of height components along the travelled distance (``height``, ``slope``)."""

    def __init__(self, parts):
        self.parts = list(parts)

    def height(self, x):
        x = np.atleast_1d(np.asarray(x, float))
        return sum((p.height(x) for p in self.parts), np.zeros_like(x))

    def slope(self, x):
        x = np.atleast_1d(np.asarray(x, float))
        return sum((p.slope(x) for p in self.parts), np.zeros_like(x))


def build_road(road: dict):
    """``(left, right)`` TrackProfiles from a road description."""
    from vehicle_model.roads.road_profile import HalfSineBump, ISO8608Profile
    left, right = [], []
    if road.get("iso_class"):
        seed = int(road.get("seed", 1))
        left.append(ISO8608Profile(road["iso_class"], seed=seed))
        right.append(ISO8608Profile(road["iso_class"], seed=seed + 1000))
    for b in road.get("bumps", []):
        bump = HalfSineBump(x0=float(b["x"]), length=float(b.get("length", 0.5)), peak=float(b.get("height", 0.05)))
        side = b.get("side", "both")
        if side in ("both", "left"):
            left.append(bump)
        if side in ("both", "right"):
            right.append(bump)
    g = road.get("grooves")
    if g:
        for k in range(int(g.get("count", 5))):
            groove = HalfSineBump(x0=float(g["x0"]) + k * float(g.get("spacing", 3.0)), length=float(g.get("length", 0.3)),
                                  peak=-abs(float(g.get("depth", 0.02))))
            left.append(groove)
            right.append(groove)
    return TrackProfile(left), TrackProfile(right)


def run(vehicle_yaml: str, road: dict, manoeuvre: dict, damping: dict, active_pitch: bool = False, preview: dict | None = None,
        duration: float = 4.0, dt: float = 0.001, control_period: float = 0.002, travel_limit: float = 0.08) -> dict:
    from vehicle_model.suspension.preview_control import (ActivePitchController, CornerDamping, DampingConfig, PreviewConfig,
                                                          RoadPreview)
    from vehicle_model.vehicle_models.fourteen_dof import FourteenDOFConfig, FourteenDOFModel, FourteenDOFState

    cfg = FourteenDOFConfig.from_yaml(vehicle_yaml)
    model = FourteenDOFModel(cfg)
    left, right = build_road(road)
    v0 = float(manoeuvre.get("speed_kph", 60.0)) / 3.6
    s = FourteenDOFState(vx=v0)
    nominal = np.array([cfg.c_damper_front, cfg.c_damper_front, cfg.c_damper_rear, cfg.c_damper_rear])
    dc = DampingConfig(**damping)
    damp = CornerDamping(dc, nominal, cfg.wheelbase)
    pv = RoadPreview((left, right), PreviewConfig(**(preview or {}))) if dc.mode == "preview_skyhook" else None
    pitch = ActivePitchController(model.corner_x, model.corner_y) if active_pitch else None
    kind = manoeuvre.get("type", "cruise")
    t0 = float(manoeuvre.get("start_s", 0.5))
    m_tot = cfg.total_mass
    steer = math.radians(float(manoeuvre.get("steer_deg", 0.0))) if kind == "corner" else 0.0
    x_rel = np.array([cfg.a, cfg.a, -cfg.b, -cfg.b])
    n_ctrl = max(1, int(round(control_period / dt)))
    c_cmd, F_act = nominal.copy(), np.zeros(4)
    dist = 0.0
    keys = ("t", "X", "Y", "psi", "vx", "vy", "r", "zs", "theta", "phi", "dzs", "dtheta", "ddzs", "steer",
            *(f"{q}_{w}" for q in ("Fz", "zu", "zr", "defl", "c", "F_act", "soft") for w in ("FL", "FR", "RL", "RR")), "P_act",
            "s_road")
    h = {k: [] for k in keys}
    energy = 0.0
    t, k = 0.0, 0
    while t < duration - 1e-12:
        xw = dist + x_rel
        z = np.array([left.height(xw[0])[0], right.height(xw[1])[0], left.height(xw[2])[0], right.height(xw[3])[0]])
        dzr = s.vx * np.array([left.slope(xw[0])[0], right.slope(xw[1])[0], left.slope(xw[2])[0], right.slope(xw[3])[0]])
        zs_c = s.zs + model.corner_x * s.theta + model.corner_y * s.phi
        dzs_c = s.dzs + model.corner_x * s.dtheta + model.corner_y * s.dphi
        if k % n_ctrl == 0:
            rows = None
            if pv is not None:
                pv.measure(t, dist + cfg.a)
                x_f = dist + cfg.a                    # rear wheels use the same track, a wheelbase behind
                rows = pv.heights_ahead(t, x_f - cfg.wheelbase - 2.0 * dc.baseline, x_f + s.vx * dc.window + 2.0 * dc.baseline)
            c_cmd = damp.command(dzs_c, s.dzu, dt * n_ctrl, rows, dist + cfg.a, s.vx)
            F_act = pitch.forces(s, dzs_c - s.dzu) if pitch is not None else np.zeros(4)
        T_drive, T_brake = np.zeros(4), np.zeros(4)
        if kind == "brake" and t >= t0 and s.vx > 1.0:
            T_brake[:] = float(manoeuvre.get("decel_g", 0.6)) * 9.81 * m_tot * cfg.wheel_radius / 4.0
        elif kind == "accel" and t >= t0:
            T_drive[:] = float(manoeuvre.get("drive_torque", 2000.0)) / 4.0
        else:
            T_drive[:] = max(0.0, 60.0 * (v0 - s.vx) + 40.0)
        s, tel = model.step(s, np.array([steer, steer, 0.0, 0.0]), T_drive, T_brake, dt, 1.0, z, dzr, F_act, c_cmd)
        if not np.all(np.isfinite(s.as_array())):
            raise FloatingPointError("non-finite 14-DOF state")
        p_act = float(np.sum(np.abs(F_act * (dzs_c - s.dzu))))
        energy += p_act * dt
        dist += s.vx * dt
        t += dt
        k += 1
        if k % n_ctrl == 0:
            vals = {"t": t, "X": s.x, "Y": s.y, "psi": s.psi, "vx": s.vx, "vy": s.vy, "r": s.r, "zs": s.zs, "theta": s.theta,
                    "phi": s.phi, "dzs": s.dzs, "dtheta": s.dtheta, "ddzs": tel.ddzs, "steer": steer, "P_act": p_act,
                    "s_road": dist}
            defl = zs_c - s.zu
            for i, w in enumerate(("FL", "FR", "RL", "RR")):
                vals[f"Fz_{w}"], vals[f"zu_{w}"], vals[f"zr_{w}"] = float(tel.Fz[i]), float(s.zu[i]), float(z[i])
                vals[f"defl_{w}"], vals[f"c_{w}"], vals[f"F_act_{w}"] = float(defl[i]), float(c_cmd[i]), float(F_act[i])
                vals[f"soft_{w}"] = float(damp.soft[i])     # preview set this corner soft
            for key, v in vals.items():
                h[key].append(v)
    h = {key: np.array(v, float) for key, v in h.items()}
    return {"history": h, "kpis": kpis(h, cfg, energy, travel_limit, t0 if kind in ("brake", "accel") else 0.5)}


def kpis(h: dict, cfg, energy: float, travel_limit: float, t_eval: float) -> dict:
    t = h["t"]
    dt = t[1] - t[0]
    sel = t >= 0.5
    Fz = np.column_stack([h[f"Fz_{w}"] for w in ("FL", "FR", "RL", "RR")])[sel]
    static = np.mean(Fz, axis=0)
    defl = np.column_stack([h[f"defl_{w}"] for w in ("FL", "FR", "RL", "RR")])
    soft = np.column_stack([h[f"c_{w}"] for w in ("FL", "FR", "RL", "RR")])
    return {
        "body_accel_rms": float(np.sqrt(np.mean(h["ddzs"][sel] ** 2))),
        "body_accel_peak": float(np.max(np.abs(h["ddzs"][sel]))),
        "pitch_rms_deg": math.degrees(float(np.sqrt(np.mean(h["theta"][t >= t_eval] ** 2)))),
        "pitch_peak_deg": math.degrees(float(np.max(np.abs(h["theta"])))),
        "pitch_rate_peak_degps": math.degrees(float(np.max(np.abs(h["dtheta"])))),
        "travel_max_m": float(np.max(np.abs(defl))),
        "travel_over_limit_s": float(np.sum(np.any(np.abs(defl) > travel_limit, axis=1)) * dt),
        "tyre_load_variation": float(np.mean(np.std(Fz, axis=0) / np.maximum(static, 1.0))),
        "contact_loss_s": float(np.sum(np.any(Fz <= 0.0, axis=1)) * dt),
        "active_energy_kj": energy / 1e3,
        "active_force_max_n": float(np.max(np.abs(np.column_stack([h[f"F_act_{w}"] for w in ("FL", "FR", "RL", "RR")])))),
        "damper_min_ns_m": float(np.min(soft)),
    }


__all__ = ("TrackProfile", "build_road", "kpis", "run")
