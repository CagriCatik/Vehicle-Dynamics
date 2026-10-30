"""Closed loop on the 14-DOF model for roll stability control (Plan V4.0 M19.1).

Roll stability with wheel loads and contact needs the vertical/roll degrees of freedom of the 14-DOF
model (plan blocker E); the 7-DOF model has no suspension and is not used here.

One run: a manoeuvre (driver steering, throttle released: the vehicle coasts), one vehicle variant
(``cg_height``, ``sprung_mass`` on top of ``configs/suspension/fourteen_dof.yaml``), one friction value and
one controller:

    passive            driver steering only
    active_roll        the existing ActiveRollDistributionController (active corner forces), nothing else
    rsc                RollStabilitySupervisor -> braking request (F_x, M_z) -> WheelForceAllocator (the
                       shared allocation; wheel positions, loads, lateral forces from the 14-DOF telemetry)
    rsc_active_roll    rsc and, while RSC is active, the active roll controller
    ``active_suspension: failed`` sets the active corner forces to zero (RSC must work through the brakes)

The plant integrates with 1 ms (stiff tyre/suspension modes); the controller is sampled every 5 ms and
holds its output. Information mode A (plant telemetry), declared in the results.

Manoeuvres (road-wheel angles; ``-inspired``, not the normative procedures):

    fishhook     NHTSA-fishhook-inspired: ramp to +A at ``rate``, hold, ramp to -A, hold
    evasive      one full sine period of amplitude A and frequency f (quick lane change at high speed)
"""

from __future__ import annotations

import dataclasses
import math

import numpy as np

CONTROLLERS = ("passive", "active_roll", "rsc", "rsc_active_roll")


def steer_profile(m: dict):
    A = math.radians(float(m.get("amplitude_deg", 8.0)))
    if m["type"] == "fishhook":
        rate, t0, hold = math.radians(float(m.get("rate_degps", 45.0))), 0.5, float(m.get("hold_s", 0.25))
        t1 = t0 + A / rate
        t2 = t1 + hold
        t3 = t2 + 2 * A / rate

        def f(t):
            if t < t0:
                return 0.0
            if t < t1:
                return rate * (t - t0)
            if t < t2:
                return A
            if t < t3:
                return A - rate * (t - t2)
            return -A
        return f
    freq, t0 = float(m.get("frequency_hz", 0.5)), 0.5

    def g(t):
        return A * math.sin(2 * math.pi * freq * (t - t0)) if t0 <= t < t0 + 1.0 / freq else 0.0
    return g


def run(vehicle_yaml: str, variant: dict, manoeuvre: dict, mu: float, controller: str, active_suspension: str = "healthy",
        rsc_config=None, duration: float = 5.0, dt: float = 0.001, control_period: float = 0.005) -> dict:
    from vehicle_model.allocation import AllocatorConfig, WheelForceAllocator
    from vehicle_model.controllers.active_roll import ActiveRollConfig, ActiveRollDistributionController
    from vehicle_model.controllers.roll_stability import RollStabilitySupervisor
    from vehicle_model.vehicle_models.fourteen_dof import FourteenDOFConfig, FourteenDOFModel, FourteenDOFState

    if controller not in CONTROLLERS:
        raise ValueError(f"controller must be one of {CONTROLLERS}")
    cfg = dataclasses.replace(FourteenDOFConfig.from_yaml(vehicle_yaml), **variant)
    model = FourteenDOFModel(cfg)
    pos = np.array([[cfg.a, cfg.track_front / 2], [cfg.a, -cfg.track_front / 2],
                    [-cfg.b, cfg.track_rear / 2], [-cfg.b, -cfg.track_rear / 2]])
    alloc = WheelForceAllocator(pos, cfg.wheel_radius, AllocatorConfig(max_brake_torque=3000.0, max_drive_torque=800.0))
    rsc = RollStabilitySupervisor(cfg.total_mass, (cfg.track_front + cfg.track_rear) / 2, rsc_config) \
        if controller.startswith("rsc") else None
    roll = ActiveRollDistributionController(cfg, ActiveRollConfig()) if controller in ("active_roll", "rsc_active_roll") else None
    healthy = active_suspension == "healthy"
    steer = steer_profile(manoeuvre)
    v0 = float(manoeuvre.get("speed_kph", 80.0)) / 3.6
    s = FourteenDOFState(vx=v0)
    n_ctrl = max(1, int(round(control_period / dt)))
    T_brake, F_act = np.zeros(4), np.zeros(4)
    mode, req = "inactive", None
    tel = None
    keys = ("t", "X", "Y", "psi", "vx", "vy", "r", "r_ref", "ay", "phi", "dphi", "ltr", "ltr_pred", "steer", "mode",
            *(f"Fz_{w}" for w in "FL FR RL RR".split()), *(f"contact_{w}" for w in "FL FR RL RR".split()),
            *(f"T_brake_{w}" for w in "FL FR RL RR".split()), *(f"F_act_{w}" for w in "FL FR RL RR".split()))
    h = {k: [] for k in keys}
    L = cfg.wheelbase
    k = 0
    t = 0.0
    while t < duration - 1e-12:
        d = steer(t)
        r_ref = s.vx * d / (L * (1.0 + 0.0015 * s.vx ** 2))
        if tel is not None and k % n_ctrl == 0:
            T_brake[:] = 0.0
            F_act[:] = 0.0
            if rsc is not None:
                req = rsc.update(tel.Fz, tel.in_contact, tel.ay, dt * n_ctrl, active_roll_available=healthy)
                mode = req.mode
                if req.Fx is not None:
                    a = alloc.allocate(req.Fx, req.Mz, np.array([d, d, 0.0, 0.0]), tel.Fz, np.full(4, mu), tel.Fy)
                    T_brake[:] = a.T_brake
            use_roll = roll is not None and healthy and (controller == "active_roll" or (req is not None and req.active_roll))
            if use_roll:
                F_act[:] = roll.compute_control(s, r_ref, tel.ay, tel.Fz).active_corner_forces
        s, tel = model.step(s, np.array([d, d, 0.0, 0.0]), np.zeros(4), T_brake, dt, mu, None, None, F_act)
        k += 1
        t += dt
        if not np.all(np.isfinite(s.as_array())):
            raise FloatingPointError("non-finite 14-DOF state")
        if k % n_ctrl == 0:
            Fz = tel.Fz
            vals = {"t": t, "X": s.x, "Y": s.y, "psi": s.psi, "vx": s.vx, "vy": s.vy, "r": s.r, "r_ref": r_ref, "ay": tel.ay,
                    "phi": s.phi, "dphi": s.dphi, "ltr": RollStabilitySupervisor.ltr(Fz),
                    "ltr_pred": req.ltr_pred if req is not None else float("nan"), "steer": d, "mode": mode}
            for i, w in enumerate("FL FR RL RR".split()):
                vals[f"Fz_{w}"], vals[f"contact_{w}"] = float(Fz[i]), int(tel.in_contact[i])
                vals[f"T_brake_{w}"], vals[f"F_act_{w}"] = float(T_brake[i]), float(F_act[i])
            for key, v in vals.items():
                h[key].append(v)
    h = {key: (np.array(v, dtype=object) if key == "mode" else np.array(v, float)) for key, v in h.items()}
    return {"history": h, "kpis": kpis(h, rsc, rsc_config), "activations": rsc.activations if rsc else 0}


def kpis(h: dict, rsc=None, rsc_config=None) -> dict:
    from vehicle_model.controllers.roll_stability import RollStabilityConfig
    c = rsc_config or RollStabilityConfig()
    t = h["t"]
    dt = t[1] - t[0]
    contact = np.column_stack([h[f"contact_{w}"] for w in "FL FR RL RR".split()])
    lifted = (contact < 0.5).sum(axis=1)
    Fz = np.column_stack([h[f"Fz_{w}"] for w in "FL FR RL RR".split()])
    roll_max = float(np.max(np.abs(h["phi"])))
    mode = h["mode"]
    out = {
        "ltr_max": float(np.max(h["ltr"])), "min_wheel_load_n": float(np.min(Fz)),
        "lift_time_s": float(np.sum(lifted > 0) * dt), "two_wheel_lift_time_s": float(np.sum(lifted >= 2) * dt),
        "roll_max_deg": math.degrees(roll_max), "roll_rate_max_degps": math.degrees(float(np.max(np.abs(h["dphi"])))),
        "yaw_error_max_degps": math.degrees(float(np.max(np.abs(h["r"] - h["r_ref"])))),
        "speed_loss_kph": float((h["vx"][0] - h["vx"][-1]) * 3.6),
        "rsc_active_time_s": float(np.sum(mode != "inactive") * dt), "rsc_emergency_time_s": float(np.sum(mode == "emergency") * dt),
        "rsc_activations": int(rsc.activations) if rsc is not None else 0,
        "within_model_validity": bool(roll_max <= c.roll_validity),
    }
    out["outcome"] = ("outside model validity (roll > %.0f deg)" % math.degrees(c.roll_validity) if not out["within_model_validity"]
                      else "no wheel lift" if out["lift_time_s"] == 0.0
                      else "wheel lift, recovered")
    return out


__all__ = ("CONTROLLERS", "kpis", "run", "steer_profile")
