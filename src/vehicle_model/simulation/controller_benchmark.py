"""Controller benchmark: same plant, identical scenarios, defined KPIs (Plan.md, M6 acceptance).

* A **plant factory** builds a fresh vehicle model for every run, so no state is
  carried between controllers and every controller sees the same plant.
* A **scenario** is data: road, initial speed, duration, the driver's request over
  time and the KPIs to report. Controllers act on top of the driver request.
* A **controller factory** builds a fresh controller (any ``VehicleController``)
  for the plant's parameters; ``None`` means the passive baseline.
* **KPIs** are computed here from the recorded history, against one common yaw
  reference (``YawReference``), not against each controller's own reference.

Scenarios provided:
    step_steer          step steer at a speed-holding pedal
    sine_with_dwell     0.7 Hz sine with 500 ms dwell, modelled on FMVSS 126
                        (yaw-rate ratios 1.0 s / 1.75 s after the end of steer,
                        lateral displacement 1.07 s after its start); a simulation
                        modelled on the test, not the regulatory test itself
    split_mu_braking    full braking on split friction, steering held straight

Determinism: the plant, the scenarios and all bundled controllers are
deterministic; running the same benchmark twice gives identical histories and
KPIs, except the measured computation times (wall clock).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Callable

import numpy as np

from vehicle_model.controllers.framework import ControlTarget, PassiveBaselineController
from vehicle_model.controllers.motion_control import YawReference
from vehicle_model.core.types import SevenDOFControlInput, SevenDOFState
from vehicle_model.roads.surface import MuSplitRoad, RoadSurface, UniformRoad
from vehicle_model.vehicle_models import SevenDOFModel

PlantFactory = Callable[[RoadSurface], SevenDOFModel]
ControllerFactory = Callable[[object], object]  # params -> VehicleController
WHEELS = ("FL", "FR", "RL", "RR")


@dataclass(frozen=True)
class Scenario:
    name: str
    road: Callable[[], RoadSurface]
    v0: float                                      # [m/s]
    duration: float                                # [s]
    driver: Callable[[float, SevenDOFState], ControlTarget]
    kpis: tuple[str, ...] = ("yaw_rate_rms_error", "max_sideslip_deg", "final_heading_deg")
    stop_speed: float | None = None                # [m/s] end the run below this speed
    params: dict = field(default_factory=dict)     # scenario constants used by its KPIs


@dataclass
class RunResult:
    scenario: str
    controller: str
    history: dict
    kpis: dict


# ----------------------------------------------------------------------------- scenarios
def sine_with_dwell(amplitude: float, frequency: float = 0.7, dwell: float = 0.5, t_start: float = 0.5):
    """Road-wheel steer angle over time and the time the steer input ends [s]."""
    T1 = t_start + 0.75 / frequency
    T2 = T1 + dwell
    t_end = T2 + 0.25 / frequency

    def steer(t: float) -> float:
        if t < t_start:
            return 0.0
        if t < T1:
            return amplitude * math.sin(2 * math.pi * frequency * (t - t_start))
        if t < T2:
            return -amplitude
        if t < t_end:
            return amplitude * math.sin(2 * math.pi * frequency * (t - T2 + 0.75 / frequency))
        return 0.0

    return steer, t_end


def step_steer_scenario(v0: float = 20.0, delta: float = 0.04, duration: float = 3.0) -> Scenario:
    return Scenario(
        "step_steer", lambda: UniformRoad(mu=1.0), v0, duration,
        lambda t, s: ControlTarget(driver_steer_f=delta, driver_accel_pedal=max(0.08 + 0.05 * (v0 - s.vx), 0.0)),
        kpis=("yaw_rate_rms_error", "yaw_rate_t95", "yaw_rate_overshoot", "max_sideslip_deg", "max_yaw_moment"),
        params={"delta": delta},
    )


def sine_with_dwell_scenario(v0: float = 80 / 3.6, amplitude: float = math.radians(8.0), mu: float = 1.0) -> Scenario:
    steer, t_end = sine_with_dwell(amplitude)
    return Scenario(
        "sine_with_dwell", lambda: UniformRoad(mu=mu), v0, t_end + 2.5,
        lambda t, s: ControlTarget(driver_steer_f=steer(t)),
        kpis=("yaw_rate_ratio_1s", "yaw_rate_ratio_1_75s", "lateral_displacement", "max_sideslip_deg",
              "final_heading_deg", "max_yaw_moment"),
        params={"t_end": t_end, "t_start": 0.5},
    )


def split_mu_braking_scenario(v0: float = 25.0, mu_high: float = 1.0, mu_low: float = 0.2) -> Scenario:
    return Scenario(
        "split_mu_braking", lambda: MuSplitRoad(mu_left=mu_high, mu_right=mu_low, split_y=0.0, transition_width=0.05),
        v0, 15.0, lambda t, s: ControlTarget(driver_brake_pedal=1.0),
        kpis=("stopping_distance", "final_heading_deg", "max_lateral_offset", "max_yaw_moment"),
        stop_speed=1.0,
    )


BRAKE_KPIS = ("stopping_distance", "mean_deceleration", "max_sideslip_deg", "max_sideslip_braking_deg",
              "max_yaw_rate_error_degps",
              "final_heading_deg", "max_lateral_offset", "abs_active_time_s", "abs_switches", "hydraulic_share",
              "torque_slew_max_knmps", "alloc_residual_fx_max_n", "wheel_lock_time_s")


def straight_braking_scenario(v0: float = 25.0, brake_pedal: float = 1.0) -> Scenario:
    """Straight braking to standstill with a constant pedal (Plan V4.0 M17)."""
    return Scenario("straight_braking", lambda: UniformRoad(mu=1.0), v0, 20.0,
                    lambda t, s: ControlTarget(driver_brake_pedal=brake_pedal), kpis=BRAKE_KPIS, stop_speed=1.0,
                    params={"brake_start": 0.0})


def brake_in_turn_scenario(v0: float = 22.0, steer: float = math.radians(2.0), brake_pedal: float = 0.4,
                           brake_start: float = 2.0, duration: float = 6.0) -> Scenario:
    """Steady cornering, then braking with the steering held (ISO 7975-inspired, open loop; Plan V4.0 M17)."""
    return Scenario(
        "brake_in_turn", lambda: UniformRoad(mu=1.0), v0, duration,
        lambda t, s: ControlTarget(driver_steer_f=steer,
                                   driver_accel_pedal=0.0 if t >= brake_start else max(0.08 + 0.05 * (v0 - s.vx), 0.0),
                                   driver_brake_pedal=brake_pedal if t >= brake_start else 0.0),
        kpis=BRAKE_KPIS, stop_speed=1.0, params={"brake_start": brake_start})


def coast_scenario(v0: float = 22.0, duration: float = 8.0) -> Scenario:
    """Both pedals released, steering straight: coast (drag-torque) deceleration (Plan V4.0 M17.4, MSR)."""
    return Scenario("coast", lambda: UniformRoad(mu=1.0), v0, duration, lambda t, s: ControlTarget(),
                    kpis=BRAKE_KPIS, stop_speed=1.0, params={"brake_start": 0.0})


# ----------------------------------------------------------------------------- KPIs
def _first_crossing(t, y, level):
    idx = np.flatnonzero(y >= level)
    return float(t[idx[0]]) if idx.size else float("nan")


def compute_kpis(h: dict, sc: Scenario) -> dict:
    t, r, r_ref = h["t"], h["r"], h["r_ref"]
    out = {}
    for k in sc.kpis:
        if k == "yaw_rate_rms_error":
            out[k] = float(np.sqrt(np.mean((r - r_ref) ** 2)))
        elif k == "yaw_rate_t95":
            out[k] = _first_crossing(t, r / r_ref[-1], 0.95)
        elif k == "yaw_rate_overshoot":
            out[k] = float(max(np.max(r) / r_ref[-1] - 1.0, 0.0))
        elif k == "max_sideslip_deg":
            out[k] = float(np.degrees(np.max(np.abs(h["beta"]))))
        elif k == "final_heading_deg":
            out[k] = float(np.degrees(h["psi"][-1]))
        elif k == "max_yaw_moment":
            out[k] = float(np.max(np.abs(h["Mz_allocated"])))
        elif k in ("yaw_rate_ratio_1s", "yaw_rate_ratio_1_75s"):
            t_end = sc.params["t_end"]
            r_peak = np.max(np.abs(r[t <= t_end]))
            dt_after = 1.0 if k == "yaw_rate_ratio_1s" else 1.75
            out[k] = float(abs(np.interp(t_end + dt_after, t, r)) / r_peak)
        elif k == "lateral_displacement":  # 1.07 s after the start of steer, from the initial path
            out[k] = float(abs(np.interp(sc.params["t_start"] + 1.07, t, h["Y"])))
        elif k == "stopping_distance":  # path length: equals X when braking straight, also valid when spinning
            out[k] = float(h["X"][0] + np.sum(np.hypot(np.diff(h["X"]), np.diff(h["Y"]))))
        elif k == "max_lateral_offset":
            out[k] = float(np.max(np.abs(h["Y"])))
        elif k == "peak_yaw_rate_deg":
            out[k] = float(np.degrees(np.max(np.abs(r))))
        elif k == "max_abs_slip":
            out[k] = float(np.max(h["max_abs_slip"]))
        elif k == "mean_deceleration":  # from the speed magnitude: v_x alone drops when the car turns sideways
            t0 = sc.params.get("brake_start", 0.0)
            i0 = int(np.searchsorted(t, t0))
            v_start = sc.v0 if i0 == 0 else h["speed"][max(i0 - 1, 0)]
            out[k] = float((v_start - h["speed"][-1]) / max(t[-1] - t0, 1e-9))
        elif k == "max_sideslip_braking_deg":    # from the start of braking (excludes steady cornering before)
            sel = t >= sc.params.get("brake_start", 0.0)
            out[k] = float(np.degrees(np.max(np.abs(h["beta"][sel])))) if np.any(sel) else 0.0
        elif k == "max_yaw_rate_error_degps":
            out[k] = float(np.degrees(np.max(np.abs(r - r_ref))))
        elif k == "abs_active_time_s":
            out[k] = float(np.sum(np.asarray(h["abs_active_n"]) > 0) * (t[1] - t[0])) if len(t) > 1 else 0.0
        elif k == "abs_switches":          # on/off transitions summed over the wheels
            a = np.asarray(h["abs_active_mask"], int)
            out[k] = int(np.sum(np.abs(np.diff(a, axis=0)))) if len(a) > 1 else 0
        elif k == "hydraulic_share":       # friction share of the braking energy at the wheels
            fr, rg = float(np.sum(h["friction_power_w"])), float(np.sum(h["regen_power_w"]))
            out[k] = fr / (fr + rg) if fr + rg > 0 else float("nan")
        elif k == "torque_slew_max_knmps":
            out[k] = float(np.max(np.abs(np.diff(h["T_realised_abs"]))) / (t[1] - t[0]) / 1e3) if len(t) > 1 else 0.0
        elif k == "wheel_lock_time_s":     # time with at least one wheel at |kappa| > 0.5 (locked or nearly)
            out[k] = float(np.sum(np.asarray(h["max_abs_slip"]) > 0.5) * (t[1] - t[0])) if len(t) > 1 else 0.0
        elif k == "alloc_residual_fx_max_n":
            out[k] = float(np.nanmax(np.abs(h["alloc_residual_fx"]))) if np.any(np.isfinite(h["alloc_residual_fx"])) else 0.0
        else:
            raise ValueError(f"unknown KPI {k!r}")
    if "T_issued_abs" in h and len(h["t"]) > 2:   # actuation trace (Plan V4.0 M16)
        out["issued_to_realised_lag_ms"] = actuation_lag(h["t"], h["T_issued_abs"], h["T_realised_abs"]) * 1e3
        out["regen_energy_kj"] = float(np.sum(h["regen_power_w"]) * (h["t"][1] - h["t"][0]) / 1e3)
    out["computation_ms_mean"] = float(np.mean(h["computation_ms"]))
    out["computation_ms_max"] = float(np.max(h["computation_ms"]))
    statuses, counts = np.unique(h["status"], return_counts=True)
    out["status_counts"] = {str(s): int(c) for s, c in zip(statuses, counts)}
    checked = [f for f in h.get("feasibility", ()) if f is not None]
    if checked:  # constraint feasibility of the plan, counted separately from the solver status (Plan V3.0 M10 5.1)
        out["plan_infeasible_steps"] = sum(not f["feasible"] for f in checked)
        out["plan_unrealisable_steps"] = sum(not f["realisable"] for f in checked)
        out["plan_advisory_steps"] = sum(bool(f["exceeded"].get("advisory")) for f in checked)
    solves = [r for r in h.get("solver", ()) if r is not None]
    if solves:  # solution quality of optimisation-based controllers (Plan V3.0 M10 5.2)
        fin = lambda k: np.array([r.get(k, np.nan) for r in solves], float)
        kkt, rel = fin("kkt_stationarity_relative"), fin("relative_cost_improvement")
        out["solver_over_budget_steps"] = sum(bool(r.get("over_budget")) for r in solves)
        if np.isfinite(kkt).any():
            out["solver_kkt_stationarity_relative_max"] = float(np.nanmax(kkt))
        if np.isfinite(rel).any():
            out["solver_relative_improvement_median"] = float(np.nanmedian(rel))
    return out


def actuation_lag(t, issued, realised, max_lag: float = 0.3) -> float:
    """Delay [s] between the issued and the realised actuation (shift of maximal cross-correlation, 0..max_lag).

    0 if both are identical (ideal actuators) or the issued signal does not vary.
    """
    t, a, b = np.asarray(t, float), np.asarray(issued, float), np.asarray(realised, float)
    if np.allclose(a, b) or np.std(a) < 1e-9:
        return 0.0
    dt = t[1] - t[0]
    a, b = a - a.mean(), b - b.mean()
    best, lag = -np.inf, 0
    for k in range(0, min(int(max_lag / dt), len(a) - 2) + 1):
        c = float(np.dot(a[:len(a) - k], b[k:])) / (len(a) - k)
        if c > best:
            best, lag = c, k
    return lag * dt


# ----------------------------------------------------------------------------- runner
def _solver_report(diag: dict) -> dict | None:
    """The SolverReport of an optimisation-based controller, directly or from its yaw law."""
    return diag.get("solver") or (diag.get("law") or {}).get("solver")


def run_scenario(plant_factory: PlantFactory, controller_factory: ControllerFactory | None, sc: Scenario,
                 dt: float = 0.005, reference: YawReference | None = None) -> RunResult:
    model = plant_factory(sc.road())
    p = model.params
    ctrl = PassiveBaselineController(p) if controller_factory is None else controller_factory(p)
    ref = reference or YawReference(p.wheelbase, p.g)
    s = SevenDOFState(vx=sc.v0, omega=np.full(4, sc.v0 / p.Reff))
    tel = None
    keys = ("t", "X", "Y", "psi", "vx", "r", "beta", "r_ref", "steer_f", "steer_r", "Mz_demand",
            "Mz_allocated", "computation_ms", "status", "T_brake_total", "max_abs_slip", "Mz_tyre_longitudinal",
            "Mz_tyre_lateral", "speed", "feasibility", "solver", "tyre_utilisation", "supervisor_mode", "vy", "ax", "ay",
            *(f"{q}_{w}" for q in ("Fz", "Fx", "Fy") for w in WHEELS),
            "Mz_requested", "Mz_alloc_pre_slip", "Mz_issued", "Mz_realised", "T_issued_abs", "T_realised_abs",
            "regen_power_w", "friction_power_w", "abs_active_n", "abs_active_mask", "alloc_residual_fx")
    pos = model.wheel_positions
    h = {k: [] for k in keys}
    t = 0.0
    # speed magnitude, not v_x: a spinning vehicle with small v_x is still moving
    while t < sc.duration - 1e-9 and (sc.stop_speed is None or math.hypot(s.vx, s.vy) > sc.stop_speed):
        target = sc.driver(t, s)
        out = ctrl.compute_control(s, target, tel, dt)
        mu_hat = float(np.mean(tel.mu)) if tel is not None else 1.0
        r_ref = ref.yaw_rate(target.driver_steer_f, s.vx, mu_hat)
        # actuation trace (Plan V4.0 M16): requested -> allocated -> issued -> realised (what the plant gets now)
        trace = (out.diagnostics or {}).get("trace") or {}
        steer_w = np.array([out.steer_f, out.steer_f, out.steer_r, out.steer_r])
        F_real = (np.asarray(out.T_drive, float) - np.asarray(out.T_brake, float)) / p.Reff
        mz_real = float(np.sum((pos[:, 0] * np.sin(steer_w) - pos[:, 1] * np.cos(steer_w)) * F_real))
        nan = float("nan")
        h["Mz_requested"].append(float(trace["requested"][1]) if "requested" in trace else float(out.desired_yaw_moment))
        h["Mz_alloc_pre_slip"].append(float(trace["allocated"][1]) if "allocated" in trace else nan)
        h["Mz_issued"].append(float(trace["issued"][1]) if "issued" in trace else mz_real)
        h["Mz_realised"].append(mz_real)
        T_iss = np.asarray(trace["issued_torque"], float) if "issued_torque" in trace else F_real * p.Reff
        h["T_issued_abs"].append(float(np.sum(np.abs(T_iss))))
        h["T_realised_abs"].append(float(np.sum(np.abs(F_real * p.Reff))))
        h["regen_power_w"].append(float(trace.get("regen_power_w", 0.0)))
        omega_abs = np.abs(np.asarray(s.omega, float))
        fr_t = np.asarray(trace["friction_torque"], float) if "friction_torque" in trace else np.asarray(out.T_brake, float)
        h["friction_power_w"].append(float(np.sum(fr_t * omega_abs)))
        am = np.asarray((out.diagnostics or {}).get("abs_active", np.zeros(4)), bool).reshape(4)
        h["abs_active_mask"].append(am.astype(int))
        h["abs_active_n"].append(int(am.sum()))
        res = (out.diagnostics or {}).get("allocation_residual")
        h["alloc_residual_fx"].append(float(res[0]) if res is not None else float("nan"))
        s, tel = model.step(s, SevenDOFControlInput(steer_f=out.steer_f, steer_r=out.steer_r,
                                                    T_drive=out.T_drive, T_brake=out.T_brake), dt)
        t += dt
        # Yaw moment of the tyre forces about the CoG, split by force direction in the wheel frame:
        # longitudinal (brake/drive) and lateral (steering) contributions; together sum (x F_y,b - y F_x,b).
        c, sn = np.cos(tel.steer_angles), np.sin(tel.steer_angles)
        mz_long = float(np.sum((pos[:, 0] * sn - pos[:, 1] * c) * tel.Fx_wheel))
        mz_lat = float(np.sum((pos[:, 0] * c + pos[:, 1] * sn) * tel.Fy_wheel))
        loaded = tel.Fz > 1.0
        util = np.hypot(tel.Fx_wheel, tel.Fy_wheel)[loaded] / np.maximum(tel.mu * tel.Fz, 1e-9)[loaded]
        h["tyre_utilisation"].append(float(np.max(util)) if util.size else float("nan"))
        h["supervisor_mode"].append((out.diagnostics or {}).get("supervisor_mode"))
        # body accelerations from the tyre forces (drag and rolling resistance not included) and per-wheel forces
        h["vy"].append(s.vy)
        h["ax"].append(float(np.sum(tel.Fx_body)) / p.mass)
        h["ay"].append(float(np.sum(tel.Fy_body)) / p.mass)
        for q, arr in (("Fz", tel.Fz), ("Fx", tel.Fx_wheel), ("Fy", tel.Fy_wheel)):
            for i, w in enumerate(WHEELS):
                h[f"{q}_{w}"].append(float(arr[i]))
        for k, v in (("T_brake_total", float(np.sum(out.T_brake))), ("max_abs_slip", float(np.max(np.abs(tel.kappa)))),
                     ("Mz_tyre_longitudinal", mz_long), ("Mz_tyre_lateral", mz_lat), ("speed", math.hypot(s.vx, s.vy))):
            h[k].append(v)
        for k, v in (("t", t), ("X", s.X), ("Y", s.Y), ("psi", s.psi), ("vx", s.vx), ("r", s.r),
                     ("beta", math.atan2(s.vy, max(abs(s.vx), 0.1))), ("r_ref", r_ref),
                     ("steer_f", out.steer_f), ("steer_r", out.steer_r), ("Mz_demand", out.desired_yaw_moment),
                     ("Mz_allocated", out.allocated_yaw_moment), ("computation_ms", out.computation_time_ms),
                     ("status", out.status), ("feasibility", (out.diagnostics or {}).get("feasibility")),
                     ("solver", _solver_report(out.diagnostics or {}))):
            h[k].append(v)
    h = {k: (np.array(v, dtype=object) if k in ("status", "feasibility", "solver", "supervisor_mode") else np.array(v))
         for k, v in h.items()}
    if not len(h["abs_active_mask"]):
        h["abs_active_mask"] = np.zeros((0, 4), int)
    name = getattr(ctrl, "name", type(ctrl).__name__)
    return RunResult(sc.name, "passive" if controller_factory is None else name, h, compute_kpis(h, sc))


def run_benchmark(plant_factory: PlantFactory, controllers: dict, scenarios, dt: float = 0.005) -> list[RunResult]:
    """Every controller on every scenario, each run on a fresh plant and controller."""
    return [run_scenario(plant_factory, factory, sc, dt) for sc in scenarios for factory in controllers.values()]


def kpi_table(results: list[RunResult]) -> str:
    """Markdown table: one block per scenario, one row per controller."""
    lines = []
    for sc_name in dict.fromkeys(r.scenario for r in results):
        rows = [r for r in results if r.scenario == sc_name]
        keys = [k for k in dict.fromkeys(k for r in rows for k in r.kpis) if k != "status_counts"]
        lines.append(f"\n**{sc_name}**\n")
        lines.append("| controller | " + " | ".join(keys) + " | statuses |")
        lines.append("|---" * (len(keys) + 2) + "|")
        for r in rows:
            vals = " | ".join(f"{r.kpis[k]:.3g}" if k in r.kpis else "–" for k in keys)
            st = ", ".join(f"{k}: {v}" for k, v in r.kpis["status_counts"].items())
            lines.append(f"| {r.controller} | {vals} | {st} |")
    return "\n".join(lines)


__all__ = (
    "RunResult",
    "Scenario",
    "compute_kpis",
    "kpi_table",
    "run_benchmark",
    "run_scenario",
    "sine_with_dwell",
    "sine_with_dwell_scenario",
    "split_mu_braking_scenario",
    "step_steer_scenario",
)
