"""Split-mu braking across actuator configurations (Plan.md V3.0 M10 section 5.5, G05).

Question: which actuator configuration can keep the vehicle straight during full
braking on split friction, and why do the others fail?

All configurations run on a fresh plant through the M6 benchmark
(:func:`~vehicle_model.simulation.controller_benchmark.run_scenario`) with the same
driver request (brake pedal, steering held straight), so they face the same brake
demand. Two pedal levels: partial braking (headroom on the high-friction side) and
full braking (no headroom).

    passive              60/40 brake distribution, no control
    4WS                  rear-axle steering with yaw damping (LUT mode)
    4WS + ABS            the same with the controller's wheel-slip limitation
    4WS + ABS + ESC/TV   integrated controller without phase-plane supervision, legacy
                         additive actuation (ESC brake intervention and torque vectoring
                         sit in one subsystem there and cannot be switched separately)
    TV (allocation)      yaw-moment control through the bounded wheel-force allocation (M5)
    IVMC (legacy)        full integrated controller, legacy additive actuation
    IVMC (allocation)    the same controller on the shared allocation (default since V3.0)

Authority: the yaw moment of the tyre forces about the CoG is split into the part of
the longitudinal (brake/drive) forces and the part of the lateral (steering) forces.
Asymmetric braking creates a longitudinal yaw moment; rear-axle steering can only
counter it through lateral forces, which the braked tyres can hardly provide
(friction circle). Configurations that cannot reduce the brake asymmetry itself
(no brake yaw moment control) are expected to spin: that is the physical limit the
model has to show, not a software defect.

How a configuration produces yaw moment decides the outcome (``yaw_moment_by``):

    none        no brake yaw moment: rear steering only. Its lateral-force yaw moment stays far
                below the brake yaw moment (measured 260-1300 N m against 3700-4000 N m at
                100 km/h on 1.0/0.2): the vehicle spins.
    addition    ESC adds brake torque at single wheels. The vehicle yaws towards the high-friction
                side, so the corrective brake acts on the low-friction side, where added torque
                yields at most mu_low F_z; the brake yaw moment stays almost unchanged (3500 vs
                3700 N m) and the vehicle spins, at partial and full pedal alike. With a driver
                who counter-steers (scenario simulate_split_mu_braking_4ws) partial braking stays
                stable; with the steering held straight it does not.
    allocation  the wheel-force allocation redistributes and *reduces* brake torque on the
                high-friction side (select-low-like): brake yaw moment 950-1300 N m, the vehicle
                stays straight at any pedal, at the price of deceleration (2.2 m/s^2).

The failing configurations are expected physical failures; the tests check them as such.

Brake level matters for "addition": at about 12 % pedal (the validation matrix's
"mu_split_braking", 1450 N m total) ESC/TV and IVMC end at about 10 deg and pass the
validation's 15 deg limit; from about 20 % pedal on they spin (61 deg at 0.2, 97 deg at
0.3). The validation and the matrix therefore agree; they test different brake levels.
"""

from __future__ import annotations

import dataclasses
from dataclasses import dataclass

import numpy as np

from vehicle_model.controllers.framework import ControlTarget, FourWheelSteeringController
from vehicle_model.controllers.integrated_controller import IntegratedChassisController
from vehicle_model.controllers.stability_control import AllocatedStabilityController
from vehicle_model.simulation.controller_benchmark import RunResult, run_scenario, split_mu_braking_scenario
from vehicle_model.steering.four_wheel_steering import SteeringMode

# name -> (factory(params) or None for passive, how the configuration produces yaw moment)
CONFIGURATIONS = {
    "passive": (None, "none"),
    "4WS": (lambda p: FourWheelSteeringController(params=p, mode=SteeringMode.LOOKUP_TABLE, enable_yaw_damping=True), "none"),
    "4WS + ABS": (lambda p: FourWheelSteeringController(params=p, mode=SteeringMode.LOOKUP_TABLE,
                                                        enable_yaw_damping=True, enable_abs=True), "none"),
    "4WS + ABS + ESC/TV": (lambda p: IntegratedChassisController(p, enable_phase_plane=False, enable_tcs=False,
                                                                 actuation="legacy"), "addition"),
    "TV (allocation)": (lambda p: AllocatedStabilityController(p), "allocation"),
    "IVMC (legacy)": (lambda p: IntegratedChassisController(p, actuation="legacy"), "addition"),
    "IVMC (allocation)": (lambda p: IntegratedChassisController(p), "allocation"),
}
KPIS = ("final_heading_deg", "peak_yaw_rate_deg", "max_lateral_offset", "max_sideslip_deg",
        "mean_deceleration", "max_abs_slip", "max_yaw_moment")
STABLE_HEADING_DEG = 10.0     # [deg] |final heading| below and
STABLE_LATERAL_M = 0.5        # [m]   lateral offset below: kept straight
AUTHORITY_WINDOW_S = 0.3      # [s] early phase in which the yaw moments are compared (before any spin)


@dataclass(frozen=True)
class MatrixRow:
    configuration: str
    pedal: float
    yaw_moment_by: str
    stable: bool
    kpis: dict
    peak_brake_yaw_moment: float      # [N m] max |yaw moment of the longitudinal tyre forces| in the window
    peak_steer_yaw_moment: float      # [N m] max |yaw moment of the lateral tyre forces| in the window
    result: RunResult


def run_split_mu_matrix(plant_factory, v_kph: float = 100.0, mu_high: float = 1.0, mu_low: float = 0.2,
                        pedal: float = 1.0, duration: float = 3.0, dt: float = 0.005, configurations=None) -> list[MatrixRow]:
    sc = dataclasses.replace(split_mu_braking_scenario(v_kph / 3.6, mu_high, mu_low), duration=duration, kpis=KPIS,
                             driver=lambda t, s: ControlTarget(driver_brake_pedal=pedal))
    rows = []
    for name, (factory, yaw_by) in (configurations or CONFIGURATIONS).items():
        r = run_scenario(plant_factory, factory, sc, dt)
        h = r.history
        early = h["t"] <= AUTHORITY_WINDOW_S
        stable = abs(r.kpis["final_heading_deg"]) < STABLE_HEADING_DEG and r.kpis["max_lateral_offset"] < STABLE_LATERAL_M
        rows.append(MatrixRow(name, pedal, yaw_by, stable, r.kpis,
                              float(np.max(np.abs(h["Mz_tyre_longitudinal"][early]))),
                              float(np.max(np.abs(h["Mz_tyre_lateral"][early]))), r))
    return rows


def matrix_table(rows: list[MatrixRow]) -> str:
    lines = ["| configuration | pedal | yaw moment by | stable | final heading [deg] | peak yaw rate [deg/s] | max lateral [m] "
             "| mean decel. [m/s^2] | max slip | brake yaw moment [N m] | steering yaw moment [N m] |",
             "|---|---|---|---|---|---|---|---|---|---|---|"]
    for r in rows:
        k = r.kpis
        lines.append(f"| {r.configuration} | {r.pedal:g} | {r.yaw_moment_by} | {'yes' if r.stable else '**no**'} "
                     f"| {k['final_heading_deg']:.1f} | {k['peak_yaw_rate_deg']:.1f} | {k['max_lateral_offset']:.2f} "
                     f"| {k['mean_deceleration']:.2f} | {k['max_abs_slip']:.2f} | {r.peak_brake_yaw_moment:.0f} "
                     f"| {r.peak_steer_yaw_moment:.0f} |")
    return "\n".join(lines)


__all__ = ("CONFIGURATIONS", "MatrixRow", "matrix_table", "run_split_mu_matrix")
