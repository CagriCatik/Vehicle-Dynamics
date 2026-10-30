"""Drift plan D5: circle tracking, set-point transition and figure-eight (scenarios S06, S07).

S06 circle tracking: the vehicle starts on the drift equilibrium; the reference circle has the
equilibrium radius and is shifted radially by ``offset`` (start outside for > 0). Compared
with the same plant, tyres and road:

    open_loop        constant equilibrium inputs u*
    esc              the existing allocated ESC (yaw-rate PI, deadband 2 deg/s, RWD allocation) with the
                     equilibrium inputs as driver commands
    drift_lqr        drift LQR at the fixed equilibrium (D3), no path loop
    drift_tracking   LQR scheduled along the branch + radial outer loop (:class:`ScheduledDriftController`)

Transition: staged initiation from straight driving, first to ``initial_beta_deg`` (supervised, D4),
then TRANSITION along the branch to ``final_beta_deg``.

S07 figure-eight: left drift for ``left_hold_s``, supervised exit, right drift initiated from the
resulting normal driving, hold, exit. Evaluated per section (left, transition, right): sideslip,
radius from a geometric circle fit (plan 7.1) and the radial error to the fitted / reference circle.
No path geometry is forced: the eight follows from the supervised sequence; a section that is not
reached is reported as such.

Outputs: ``kpi.json``, ``report.md``, ``trajectory.png``, ``sideslip_and_yaw.png``, state histories,
``run_config_resolved.yaml``, ``metadata.json``.
"""

from __future__ import annotations

import copy
import dataclasses
import json
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml

from .config import keys, load_yaml, numbers, require, vehicle_model

_TOP = {"name", "description", "vehicle", "tyres", "powertrain", "reference", "circle", "transition", "figure_eight",
        "simulation", "outputs"}


@dataclass(frozen=True)
class TrackingSpec:
    name: str
    vehicle: str
    tyres: tuple
    speed: float
    beta_deg: float
    mu: float
    branch_deg: tuple
    offsets: tuple
    k_p_deg_per_m: float
    k_d_deg_per_mps: float
    circle_duration: float
    transition: dict | None
    figure_eight: dict | None
    dt: float
    output_dir: str
    source: str = ""
    raw: dict | None = None


def parse(data: dict, source: str = "<dict>") -> TrackingSpec:
    keys(data, _TOP, source, {"name", "vehicle", "reference"})
    tyres = dict(data.get("tyres") or {})
    vehicle_model(data["vehicle"], tyres, source)
    pt = dict(data.get("powertrain") or {"layout": "rwd"})
    keys(pt, {"layout"}, f"{source}: powertrain")
    require(pt.get("layout", "rwd") == "rwd", f"{source}: powertrain.layout: only 'rwd' is implemented (V1)")
    ref = dict(data["reference"])
    keys(ref, {"speed_mps", "beta_deg", "mu", "branch_beta_deg"}, f"{source}: reference",
         {"speed_mps", "beta_deg", "mu", "branch_beta_deg"})
    branch = numbers(ref["branch_beta_deg"], f"{source}: reference.branch_beta_deg", 1.0, 45.0)
    require(len(branch) >= 2, f"{source}: reference.branch_beta_deg needs at least two values")
    beta = float(ref["beta_deg"])
    require(min(branch) <= beta <= max(branch), f"{source}: reference.beta_deg must lie inside the branch")
    circ = dict(data.get("circle") or {})
    keys(circ, {"offsets_m", "k_p_deg_per_m", "k_d_deg_per_mps", "duration_s"}, f"{source}: circle")
    offsets = numbers(circ.get("offsets_m", [0.0, 2.0, -2.0]), f"{source}: circle.offsets_m", -10.0, 10.0)
    tr = data.get("transition")
    if tr is not None:
        tr = dict(tr)
        keys(tr, {"initial_beta_deg", "final_beta_deg", "rate_degps", "mu", "duration_s", "exit_s"},
             f"{source}: transition", {"initial_beta_deg", "final_beta_deg"})
        for k in ("initial_beta_deg", "final_beta_deg"):
            require(min(branch) <= float(tr[k]) <= max(branch), f"{source}: transition.{k} must lie inside the branch")
    fe = data.get("figure_eight")
    if fe is not None:
        fe = dict(fe)
        keys(fe, {"beta_deg", "left_hold_s", "right_hold_s", "duration_s", "geometry_tolerance"}, f"{source}: figure_eight")
    sim = dict(data.get("simulation") or {})
    keys(sim, {"dt_s"}, f"{source}: simulation")
    out = dict(data.get("outputs") or {})
    keys(out, {"directory"}, f"{source}: outputs")
    return TrackingSpec(str(data["name"]), str(data["vehicle"]), tuple(sorted(tyres.items())), float(ref["speed_mps"]),
                        beta, float(ref["mu"]), tuple(branch), tuple(offsets), float(circ.get("k_p_deg_per_m", 1.0)),
                        float(circ.get("k_d_deg_per_mps", 0.5)), float(circ.get("duration_s", 25.0)), tr, fe,
                        float(sim.get("dt_s", 0.005)), str(out.get("directory", f"output/drift/{data['name']}")), source,
                        data)


def load(path):
    return parse(load_yaml(path), str(path))


def fit_circle(X, Y):
    """Least-squares (Kasa) circle fit: centre and radius [m]."""
    X, Y = np.asarray(X, float), np.asarray(Y, float)
    A = np.column_stack([X, Y, np.ones_like(X)])
    b = X ** 2 + Y ** 2
    (a0, a1, a2), *_ = np.linalg.lstsq(A, b, rcond=None)
    cx, cy = a0 / 2.0, a1 / 2.0
    return float(cx), float(cy), float(math.sqrt(a2 + cx ** 2 + cy ** 2))


class _ESCDriver:
    """The existing allocated ESC with the equilibrium inputs as driver commands (RWD allocation)."""

    def __init__(self, params, eq, mu):
        from vehicle_model.allocation import AllocatorConfig
        from vehicle_model.controllers.stability_control import AllocatedStabilityController, StabilityControlConfig
        from vehicle_model.simulation.stability_region import ESC_DEADBAND
        cfg = StabilityControlConfig(deadband=ESC_DEADBAND, allocator=AllocatorConfig(
            driven=(False, False, True, True), max_drive_torque=params.max_drive_torque,
            max_brake_torque=params.max_brake_torque))
        self.esc = AllocatedStabilityController(params, cfg)
        self.steer, self.pedal = eq.steer_front, eq.wheel_drive_torque[2] / params.max_drive_torque
        self.mu = mu

    def compute_control(self, state, target, telemetry, dt):
        from vehicle_model.controllers.framework import ControlTarget
        return self.esc.compute_control(state, ControlTarget(driver_steer_f=self.steer, driver_accel_pedal=self.pedal,
                                                             mu_estimated=self.mu), telemetry, dt)


def run(spec: TrackingSpec, out_dir=None) -> dict:
    from vehicle_model.core.types import SevenDOFState
    from vehicle_model.drift import DriftEquilibriumSolver, DriftReference
    from vehicle_model.drift.control_adapter import DriftLQRController, ScheduledDriftController
    from vehicle_model.drift.linearization import linearise
    from vehicle_model.drift.lqr import design_lqr
    from vehicle_model.drift.metrics import hold_kpis
    from vehicle_model.drift.path_reference import CircleReference
    from vehicle_model.drift.schedule import BranchSchedule
    from vehicle_model.experiments.types import provenance
    from vehicle_model.roads.surface import UniformRoad
    from vehicle_model.visualization.drift_plots import plot_tracking

    from .hold import _OpenLoop, _write_history, simulate

    out = Path(out_dir or spec.output_dir)
    out.mkdir(parents=True, exist_ok=True)
    m = vehicle_model(spec.vehicle, dict(spec.tyres), spec.source)
    p = m.params
    plant = copy.copy(m)
    plant.road = UniformRoad(mu=spec.mu)
    sch = BranchSchedule(m, spec.speed, spec.mu, [-b for b in spec.branch_deg])
    sol = DriftEquilibriumSolver(m)
    eq = sol.solve(DriftReference(spec.speed, -math.radians(spec.beta_deg), spec.mu))
    lqr = design_lqr(linearise(sol.plant(spec.mu), eq), p)
    x0 = SevenDOFState(vx=eq.vx, vy=eq.vy, r=eq.yaw_rate, omega=eq.omega.copy())
    results = {"circle": [], "transition": None, "figure_eight": None}
    traj = {}
    k_p, k_d = math.radians(spec.k_p_deg_per_m), math.radians(spec.k_d_deg_per_mps)
    for off in spec.offsets:
        circ = CircleReference.through_state(0.0, 0.0, eq.beta, eq.radius, "left", offset=off)
        for variant in ("open_loop", "esc", "drift_lqr", "drift_tracking"):
            ctrl = {"open_loop": lambda: _OpenLoop(eq), "esc": lambda: _ESCDriver(p, eq, spec.mu),
                    "drift_lqr": lambda: DriftLQRController(p, eq, lqr),
                    "drift_tracking": lambda: ScheduledDriftController(p, sch, eq.beta, circle=circ, k_p=k_p, k_d=k_d)}[variant]()
            h, _ = simulate(plant, ctrl, x0, spec.circle_duration, spec.dt)
            err = circ.radial_error(h.X, h.Y)
            last = h.time >= spec.circle_duration - 10.0
            k = hold_kpis(h.time, h.vx, h.vy, h.r, eq.beta, eq.yaw_rate, eq.speed, spec.circle_duration - 10.0)
            results["circle"].append({"offset_m": off, "variant": variant, "radial_rmse_last10s_m":
                                      float(np.sqrt(np.mean(err[last] ** 2))), "radial_max_m": float(np.max(np.abs(err))),
                                      "radial_final_m": float(err[-1]), **{n: k[n] for n in ("beta_rmse_deg",
                                                                                              "yaw_rmse_degps",
                                                                                              "stability_loss")}})
            if off == spec.offsets[-1] or off == max(spec.offsets):
                traj[(f"offset {off:+g} m", variant)] = (h, circ)
            if off == max(spec.offsets):
                _write_history(out / f"state_history_circle_{variant}.csv", h)
    if spec.transition:
        results["transition"] = _transition(spec, m, plant, sch, out)
    if spec.figure_eight:
        results["figure_eight"], fe_h, _ = _figure_eight(spec, m, out)
        if fe_h is not None:
            traj[("figure-eight", "supervised")] = (fe_h, None)
    (out / "run_config_resolved.yaml").write_text(yaml.safe_dump(spec.raw, sort_keys=False), encoding="utf-8")
    (out / "metadata.json").write_text(json.dumps({"source": spec.source, "provenance": provenance(),
                                                   "equilibrium": eq.to_dict()}, indent=2, default=float), encoding="utf-8")
    (out / "kpi.json").write_text(json.dumps(results, indent=2, default=float), encoding="utf-8")
    plot_tracking(traj, out / "trajectory.png", spec.name)
    (out / "report.md").write_text(_report(spec, results, eq), encoding="utf-8")
    return {"results": results, "directory": str(out)}


def _supervised(m, speed, beta_deg, mu, direction, exit_s, scheduled_final_deg=None, sch=None, rate_degps=1.0):
    from vehicle_model.drift import DriftEquilibriumSolver, DriftReference
    from vehicle_model.drift.control_adapter import DriftSupervisedController, ScheduledDriftController
    from vehicle_model.drift.equilibrium import approach_equilibrium
    from vehicle_model.drift.linearization import linearise
    from vehicle_model.drift.lqr import design_lqr
    from vehicle_model.drift.supervisor import DriftSupervisor, SupervisorSettings
    sign = -1.0 if direction == "left" else 1.0
    sol = DriftEquilibriumSolver(m)
    de = sol.solve(DriftReference(speed, sign * math.radians(beta_deg), mu))
    dl = design_lqr(linearise(sol.plant(mu), de), m.params)
    ae = approach_equilibrium(m, speed, mu, 0.6, direction)
    sup = DriftSupervisor(SupervisorSettings(exit_s=exit_s), de.beta, de.yaw_rate, de.speed, ae.yaw_rate)
    sc = None
    if scheduled_final_deg is not None:
        sc = ScheduledDriftController(m.params, sch, sign * math.radians(scheduled_final_deg),
                                      beta_rate=math.radians(rate_degps))
    return DriftSupervisedController(m.params, de, dl, ae, sup, speed, scheduled=sc), sup


def _modes(ctrl):
    log = []
    orig = ctrl.compute_control

    def logged(*a, **k):
        o = orig(*a, **k)
        log.append(o.diagnostics["mode"])
        return o
    ctrl.compute_control = logged
    return log


def _transition(spec, m, plant_base, sch, out) -> dict:
    from vehicle_model.core.types import SevenDOFState
    from vehicle_model.roads.surface import UniformRoad

    from .hold import _write_history, simulate
    tr = spec.transition
    mu = float(tr.get("mu", spec.mu))
    plant = copy.copy(m)
    plant.road = UniformRoad(mu=mu)
    from vehicle_model.drift.schedule import BranchSchedule
    sch_t = sch if mu == spec.mu else BranchSchedule(m, spec.speed, mu, [-b for b in spec.branch_deg])
    dur, exit_s = float(tr.get("duration_s", 60.0)), float(tr.get("exit_s", 50.0))
    ctrl, sup = _supervised(m, spec.speed, float(tr["initial_beta_deg"]), mu, "left", exit_s,
                            float(tr["final_beta_deg"]), sch_t, float(tr.get("rate_degps", 1.0)))
    modes = _modes(ctrl)
    h, _ = simulate(plant, ctrl, SevenDOFState(vx=spec.speed, omega=np.full(4, spec.speed / m.params.Reff)), dur, spec.dt)
    _write_history(out / "state_history_transition.csv", h)
    modes = np.array(modes, dtype=object)
    beta = np.degrees(np.arctan2(h.vy, h.vx))
    final_hold = (modes == "HOLD") & (h.time > h.time[np.argmax(modes == "TRANSITION")] if np.any(modes == "TRANSITION") else False)
    fb = -float(tr["final_beta_deg"])
    return {"mu": mu, "initial_beta_deg": tr["initial_beta_deg"], "final_beta_deg": tr["final_beta_deg"],
            "transitions": [{"t": t.t, "to": t.target, "reason": t.reason} for t in sup.transitions],
            "reached_final_hold": bool(np.any(final_hold)), "final_mode": str(modes[-1]),
            "peak_beta_deg": float(np.max(np.abs(beta))), "overshoot_beyond_final_deg": float(max(0.0, -np.min(beta) + fb)),
            "final_hold_beta_rmse_deg": float(np.sqrt(np.mean((beta[final_hold] - fb) ** 2))) if np.any(final_hold) else None,
            "success": bool(np.any(final_hold)) and str(modes[-1]) == "DISABLED" and "FAILED" not in modes}


def _figure_eight(spec, m, out):
    from vehicle_model.core.types import SevenDOFState
    from vehicle_model.roads.surface import UniformRoad

    from .hold import _write_history
    fe = spec.figure_eight
    b = float(fe.get("beta_deg", spec.beta_deg))
    t_left, t_right = float(fe.get("left_hold_s", 12.0)), float(fe.get("right_hold_s", 12.0))
    fe_tol = float(fe.get("geometry_tolerance", 0.15))   # |centre distance / (R_l + R_r) - 1|
    dur = float(fe.get("duration_s", 80.0))
    plant = copy.copy(m)
    plant.road = UniformRoad(mu=spec.mu)
    left, sup_l = _supervised(m, spec.speed, b, spec.mu, "left", None)
    right, sup_r = _supervised(m, spec.speed, b, spec.mu, "right", None)

    class Sequence:
        """Left drift, exit after ``t_left`` s in HOLD, then the right drift requested from normal driving."""

        def __init__(self):
            self.active, self.t, self.t_hold, self.section = left, 0.0, None, "left"
            self.log = []

        def compute_control(self, state, target, telemetry, dt):
            self.t += dt
            if self.active is left:
                if sup_l.mode == "HOLD" and self.t_hold is None:
                    self.t_hold = self.t
                if self.t_hold is not None and self.t - self.t_hold >= t_left and sup_l.settings.exit_s is None:
                    sup_l.settings = dataclasses.replace(sup_l.settings, exit_s=self.t)
                    self.section = "transition"
                if sup_l.mode == "DISABLED" and self.section == "transition":
                    self.active = right
                    right.steer = left.steer
                    sup_r.settings = dataclasses.replace(sup_r.settings, request_s=0.0)
                    self.t_right0 = self.t
            elif sup_r.mode in ("INITIATE", "HOLD") and self.section == "transition":
                self.section = "right"
            if self.active is right and getattr(self, "t_rhold", None) is None and sup_r.mode == "HOLD":
                self.t_rhold = self.t
            if self.active is right and getattr(self, "t_rhold", None) is not None and \
                    self.t - self.t_rhold >= t_right and sup_r.settings.exit_s is None:
                sup_r.settings = dataclasses.replace(sup_r.settings, exit_s=right.t + dt)
            o = self.active.compute_control(state, target, telemetry, dt)
            self.log.append((self.section, o.diagnostics["mode"]))
            return o

    seq = Sequence()
    from .hold import simulate
    h, _ = simulate(plant, seq, SevenDOFState(vx=spec.speed, omega=np.full(4, spec.speed / m.params.Reff)), dur, spec.dt)
    _write_history(out / "state_history_figure_eight.csv", h)
    sec = np.array([s for s, _ in seq.log], dtype=object)
    mode = np.array([md for _, md in seq.log], dtype=object)
    beta = np.degrees(np.arctan2(h.vy, h.vx))
    res = {"transitions_left": [{"t": t.t, "to": t.target, "reason": t.reason} for t in sup_l.transitions],
           "transitions_right": [{"t": t.t + getattr(seq, "t_right0", 0.0), "to": t.target, "reason": t.reason}
                                 for t in sup_r.transitions], "sections": {}}
    for name, md_ok in (("left", "HOLD"), ("right", "HOLD")):
        w = (sec == name) & (mode == md_ok)
        if np.sum(w) < 200:
            res["sections"][name] = {"reached": False}
            continue
        cx, cy, R = fit_circle(h.X[w], h.Y[w])
        rad = np.hypot(h.X[w] - cx, h.Y[w] - cy) - R
        res["sections"][name] = {"reached": True, "duration_s": float(np.sum(w) * spec.dt), "fit_radius_m": R,
                                 "fit_centre": [cx, cy], "radial_rms_about_fit_m": float(np.sqrt(np.mean(rad ** 2))),
                                 "beta_mean_deg": float(np.mean(beta[w])), "beta_std_deg": float(np.std(beta[w]))}
    tw = sec == "transition"
    res["sections"]["transition"] = {"duration_s": float(np.sum(tw) * spec.dt),
                                     "peak_abs_beta_deg": float(np.max(np.abs(beta[tw]))) if np.any(tw) else None,
                                     "min_speed_mps": float(np.min(np.hypot(h.vx[tw], h.vy[tw]))) if np.any(tw) else None}
    res["failed"] = bool(np.any(mode == "FAILED"))
    res["sequence_success"] = all(res["sections"][n].get("reached") for n in ("left", "right")) and not res["failed"]
    # geometry of an eight: the two circles touch, centre distance = R_left + R_right (ratio 1)
    L, R = res["sections"].get("left", {}), res["sections"].get("right", {})
    if L.get("reached") and R.get("reached"):
        d = math.dist(L["fit_centre"], R["fit_centre"])
        res["centre_distance_m"] = d
        res["centre_distance_ratio"] = d / (L["fit_radius_m"] + R["fit_radius_m"])
        res["geometry_success"] = abs(res["centre_distance_ratio"] - 1.0) <= fe_tol
    else:
        res["geometry_success"] = False
    res["success"] = res["sequence_success"] and res["geometry_success"]
    return res, h, [f"{sc}:{md}" for sc, md in seq.log]


def _report(spec, res, eq) -> str:
    L = [f"# Drift tracking, transition and figure-eight: {spec.name}", "",
         f"Equilibrium: {spec.speed:g} m/s, beta {math.degrees(eq.beta):.1f} deg, R {eq.radius:.1f} m, mu {spec.mu:g}; "
         f"branch {spec.branch_deg} deg. Synthetic parameters, virtual model only.", "",
         "## S06 circle tracking (radial error to the reference circle)", "",
         "| start offset [m] | variant | radial RMSE last 10 s [m] | max radial error [m] | final [m] | beta RMSE [deg] | "
         "drift lost |", "|---|---|---|---|---|---|---|"]
    for r in res["circle"]:
        L.append(f"| {r['offset_m']:+g} | {r['variant']} | {r['radial_rmse_last10s_m']:.2f} | {r['radial_max_m']:.2f} | "
                 f"{r['radial_final_m']:+.2f} | {r['beta_rmse_deg']:.2f} | {'yes' if r['stability_loss'] else 'no'} |")
    t = res.get("transition")
    if t:
        L += ["", f"## Staged initiation with TRANSITION ({t['initial_beta_deg']:g} -> {t['final_beta_deg']:g} deg, mu "
              f"{t['mu']:g})", "", f"Result: {'PASS' if t['success'] else 'FAIL'}; final mode {t['final_mode']}; peak "
              f"|beta| {t['peak_beta_deg']:.1f} deg (overshoot beyond the final set point {t['overshoot_beyond_final_deg']:.1f} "
              f"deg); beta RMSE in the final HOLD {t['final_hold_beta_rmse_deg'] if t['final_hold_beta_rmse_deg'] is None else round(t['final_hold_beta_rmse_deg'], 2)} deg.",
              "", "Mode sequence: " + " -> ".join(f"{x['to']} ({x['t']:.1f} s)" for x in t["transitions"])]
    f = res.get("figure_eight")
    if f:
        ratio = f.get("centre_distance_ratio")
        L += ["", "## S07 figure-eight (supervised: left drift, exit, right drift)", "",
              f"Sequence (both drift sections reached, no FAILED): {'PASS' if f['sequence_success'] else 'FAIL'}. "
              f"Geometry (circles touch, centre distance / (R_l + R_r) = 1 +- tolerance): "
              f"{'PASS' if f['geometry_success'] else 'FAIL'}"
              + (f" (ratio {ratio:.2f}, centre distance {f['centre_distance_m']:.1f} m)" if ratio is not None else "")
              + (". FAILED mode reached" if f["failed"] else "") + ".", "",
              "| section | reached | duration [s] | fitted radius [m] | radial RMS about the fit [m] | beta mean / std [deg] |",
              "|---|---|---|---|---|---|"]
        for n in ("left", "right"):
            s = f["sections"][n]
            if not s.get("reached"):
                L.append(f"| {n} | no | | | | |")
            else:
                L.append(f"| {n} | yes | {s['duration_s']:.1f} | {s['fit_radius_m']:.1f} | {s['radial_rms_about_fit_m']:.2f} | "
                         f"{s['beta_mean_deg']:.1f} / {s['beta_std_deg']:.2f} |")
        tr = f["sections"]["transition"]
        L.append(f"| transition | | {tr['duration_s']:.1f} | | | peak {tr['peak_abs_beta_deg']} deg, min v {tr['min_speed_mps']} m/s |")
    L += ["", "Radial error: |p - c| - R, > 0 outside. The figure-eight is not a forced geometry: left and right circles",
          "follow from the supervised sequence (exit, normal driving, new initiation); their centres and radii come from a",
          "geometric least-squares circle fit. The radial outer loop can move the radius only within the branch",
          "(about 20-27 m), not the centre, so the sequence cannot place the second circle tangent to the first.", ""]
    return "\n".join(L)


def main(argv=None) -> int:
    import argparse
    ap = argparse.ArgumentParser(description="Drift circle tracking, transition and figure-eight (Drift plan D5)")
    ap.add_argument("config", nargs="?", default="configs/drift/circle_hold.yaml")
    ap.add_argument("--out", default=None)
    a = ap.parse_args(argv)
    res = run(load(a.config), a.out)
    print(f"-> {res['directory']}")
    return 0


__all__ = ("TrackingSpec", "fit_circle", "load", "main", "parse", "run")
