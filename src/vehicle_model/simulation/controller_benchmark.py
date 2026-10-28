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
deterministic; running the same benchmark twice gives identical KPIs.
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
        elif k == "stopping_distance":
            out[k] = float(h["X"][-1])
        elif k == "max_lateral_offset":
            out[k] = float(np.max(np.abs(h["Y"])))
        else:
            raise ValueError(f"unknown KPI {k!r}")
    out["computation_ms_mean"] = float(np.mean(h["computation_ms"]))
    out["computation_ms_max"] = float(np.max(h["computation_ms"]))
    statuses, counts = np.unique(h["status"], return_counts=True)
    out["status_counts"] = {str(s): int(c) for s, c in zip(statuses, counts)}
    return out


# ----------------------------------------------------------------------------- runner
def run_scenario(plant_factory: PlantFactory, controller_factory: ControllerFactory | None, sc: Scenario,
                 dt: float = 0.005, reference: YawReference | None = None) -> RunResult:
    model = plant_factory(sc.road())
    p = model.params
    ctrl = PassiveBaselineController(p) if controller_factory is None else controller_factory(p)
    ref = reference or YawReference(p.wheelbase, p.g)
    s = SevenDOFState(vx=sc.v0, omega=np.full(4, sc.v0 / p.Reff))
    tel = None
    keys = ("t", "X", "Y", "psi", "vx", "r", "beta", "r_ref", "steer_f", "steer_r", "Mz_demand",
            "Mz_allocated", "computation_ms", "status")
    h = {k: [] for k in keys}
    t = 0.0
    while t < sc.duration - 1e-9 and (sc.stop_speed is None or s.vx > sc.stop_speed):
        target = sc.driver(t, s)
        out = ctrl.compute_control(s, target, tel, dt)
        mu_hat = float(np.mean(tel.mu)) if tel is not None else 1.0
        r_ref = ref.yaw_rate(target.driver_steer_f, s.vx, mu_hat)
        s, tel = model.step(s, SevenDOFControlInput(steer_f=out.steer_f, steer_r=out.steer_r,
                                                    T_drive=out.T_drive, T_brake=out.T_brake), dt)
        t += dt
        for k, v in (("t", t), ("X", s.X), ("Y", s.Y), ("psi", s.psi), ("vx", s.vx), ("r", s.r),
                     ("beta", math.atan2(s.vy, max(abs(s.vx), 0.1))), ("r_ref", r_ref),
                     ("steer_f", out.steer_f), ("steer_r", out.steer_r), ("Mz_demand", out.desired_yaw_moment),
                     ("Mz_allocated", out.allocated_yaw_moment), ("computation_ms", out.computation_time_ms),
                     ("status", out.status)):
            h[k].append(v)
    h = {k: (np.array(v) if k != "status" else np.array(v, dtype=object)) for k, v in h.items()}
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
        keys = [k for k in rows[0].kpis if k not in ("status_counts",)]
        lines.append(f"\n**{sc_name}**\n")
        lines.append("| controller | " + " | ".join(keys) + " | statuses |")
        lines.append("|---" * (len(keys) + 2) + "|")
        for r in rows:
            vals = " | ".join(f"{r.kpis[k]:.3g}" for k in keys)
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
