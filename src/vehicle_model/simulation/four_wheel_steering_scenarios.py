"""Four-Wheel Steering (4WS) Multi-Scenario Dynamics & Benchmark Suite.

Simulates, validates, and visualizes 6 comprehensive scenarios:
1. Low-Speed Turning Circle (ISO 7401 / SAE J266, 12 km/h) - Counter-phase agility.
2. Low-Speed 90° Urban Cornering & Curb Clearance (15 km/h) - Virtual wheelbase shortening.
3. High-Speed Evasive Double Lane Change (ISO 3888-2, 80 km/h) - Zero-sideslip crab translation.
4. High-Speed Slalom Maneuver (70 km/h, 30m pylons) - Phase-lag elimination.
5. High-Speed Autobahn Lane Change (130 km/h) - High-speed parallel crabbing.
6. Transient Step Steer Response (ISO 7401, 100 km/h) - Yaw damping & lateral latency.

Renders physical 2D vehicle bodies (draw_vehicle) with steerable front and rear wheels,
and comprehensive telemetry overlays (speed, front steer, rear steer, sideslip beta, yaw rate).

Conforms to:
- Küçükay, F. (2022): Grundlagen der Fahrzeugtechnik, Springer Vieweg.
- Heißing, B.; Ersoy, M.; Gies, M. (2020): Fahrwerkhandbuch, Springer Vieweg.
- Rajamani, R. (2012): Vehicle Dynamics and Control, Springer.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from pathlib import Path
from typing import Dict, List, Tuple, Any, Optional, Callable

import matplotlib
matplotlib.use("Agg")
import matplotlib.animation as animation
import matplotlib.patches as patches
import matplotlib.pyplot as plt
import numpy as np

from vehicle_model.controllers.driver_model import DriverModelConfig, NeuromuscularDriverModel
from vehicle_model.controllers.framework import (
    ControlOutput,
    ControlTarget,
    FourWheelSteeringController,
    PassiveBaselineController,
)
from vehicle_model.controllers.integrated_controller import IntegratedChassisController
from vehicle_model.core.types import SevenDOFControlInput, SevenDOFParams, SevenDOFState
from vehicle_model.roads.surface import MuSplitRoad, UniformRoad
from vehicle_model.steering.four_wheel_steering import (
    FourWheelSteering,
    SteeringConfig,
    SteeringMode,
)
from vehicle_model.steering.lut import LookupTable, load_lookup_table
from vehicle_model.vehicle_models.seven_dof import SevenDOFModel
from vehicle_model.visualization.plan_view import VehicleParams, draw_vehicle


@dataclass
class ScenarioMetrics:
    scenario_name: str
    controller_name: str
    min_turning_radius_m: float | None = None
    peak_sideslip_deg: float | None = None
    peak_yaw_rate_deg_s: float | None = None
    lateral_tracking_rms_m: float | None = None
    response_delay_ms: float | None = None
    final_heading_dev_deg: float | None = None
    max_lateral_dev_m: float | None = None
    stability_status: str = "PASS"


@dataclass
class ScenarioRunResult:
    controller_name: str
    metrics: ScenarioMetrics
    time: np.ndarray
    X: np.ndarray
    Y: np.ndarray
    psi: np.ndarray
    vx: np.ndarray
    vy: np.ndarray
    r: np.ndarray
    beta_deg: np.ndarray
    steer_f_deg: np.ndarray
    steer_r_deg: np.ndarray
    ay_mps2: np.ndarray


# =============================================================================
# SCENARIO 1: LOW-SPEED TURNING CIRCLE (ISO 7401 / SAE J266)
# =============================================================================
def simulate_low_speed_turning_circle(
    model: SevenDOFModel,
    steer_mode: SteeringMode = SteeringMode.LOOKUP_TABLE,
    enable_4ws: bool = True,
    v_kph: float = 12.0,
    front_steer_deg: float = 25.0,
    duration_s: float = 7.0,
    dt: float = 0.01,
    controller_name: str = "4WS Calibrated LUT",
) -> ScenarioRunResult:
    """Simulate low-speed turning circle to verify Ackermann counter-phase radius reduction."""
    params = model.params
    v0_mps = v_kph / 3.6
    state = SevenDOFState(
        vx=v0_mps,
        vy=0.0,
        r=0.0,
        omega=np.full(4, v0_mps / params.Reff),
        X=0.0,
        Y=0.0,
        psi=0.0,
    )
    road = UniformRoad(mu=1.0)
    steering_sys = FourWheelSteering(SteeringConfig.from_yaml())

    n_steps = int(duration_s / dt)
    time_arr = np.linspace(0.0, duration_s, n_steps)

    hist_X = np.zeros(n_steps)
    hist_Y = np.zeros(n_steps)
    hist_psi = np.zeros(n_steps)
    hist_vx = np.zeros(n_steps)
    hist_vy = np.zeros(n_steps)
    hist_r = np.zeros(n_steps)
    hist_beta = np.zeros(n_steps)
    hist_steer_f = np.zeros(n_steps)
    hist_steer_r = np.zeros(n_steps)
    hist_ay = np.zeros(n_steps)

    cmd_f_rad = math.radians(front_steer_deg)

    for i, t in enumerate(time_arr):
        steer_factor = min(1.0, t / 0.50)
        curr_steer_f = cmd_f_rad * steer_factor

        if enable_4ws:
            delta_f, delta_r = steering_sys.compute_steering_angles(
                cmd_steer_f=curr_steer_f,
                vx=state.vx,
                vy=state.vy,
                mode=steer_mode,
            )
        else:
            delta_f = curr_steer_f
            delta_r = 0.0

        t_drive = np.full(4, 50.0) if state.vx < v0_mps else np.zeros(4)
        t_brake = np.full(4, 20.0) if state.vx > v0_mps + 0.1 else np.zeros(4)

        u = SevenDOFControlInput(
            steer_f=delta_f,
            steer_r=delta_r,
            T_drive=t_drive,
            T_brake=t_brake,
        ).clamped(params)

        state, telem = model.step(state, u, dt=dt, road=road)

        hist_X[i] = state.X
        hist_Y[i] = state.Y
        hist_psi[i] = state.psi
        hist_vx[i] = state.vx
        hist_vy[i] = state.vy
        hist_r[i] = state.r
        beta = math.atan2(state.vy, max(abs(state.vx), 0.5))
        hist_beta[i] = math.degrees(beta)
        hist_steer_f[i] = math.degrees(delta_f)
        hist_steer_r[i] = math.degrees(delta_r)
        hist_ay[i] = float(np.sum(telem.Fy_body) / params.mass)

    steady_mask = time_arr > 3.0
    r_mean = np.mean(np.abs(hist_r[steady_mask]))
    v_mean = np.mean(hist_vx[steady_mask])
    radius = (v_mean / r_mean) if r_mean > 0.01 else 999.0

    metrics = ScenarioMetrics(
        scenario_name="Scenario 1: Low-Speed Turning Circle (12 km/h)",
        controller_name=controller_name,
        min_turning_radius_m=float(radius),
        peak_sideslip_deg=float(np.max(np.abs(hist_beta))),
        peak_yaw_rate_deg_s=float(math.degrees(np.max(np.abs(hist_r)))),
    )

    return ScenarioRunResult(
        controller_name=controller_name,
        metrics=metrics,
        time=time_arr,
        X=hist_X,
        Y=hist_Y,
        psi=hist_psi,
        vx=hist_vx,
        vy=hist_vy,
        r=hist_r,
        beta_deg=hist_beta,
        steer_f_deg=hist_steer_f,
        steer_r_deg=hist_steer_r,
        ay_mps2=hist_ay,
    )


# =============================================================================
# SCENARIO 2: LOW-SPEED 90° URBAN CORNERING & CURB CLEARANCE (15 km/h)
# =============================================================================
def simulate_urban_90deg_cornering(
    model: SevenDOFModel,
    steer_mode: SteeringMode = SteeringMode.LOOKUP_TABLE,
    enable_4ws: bool = True,
    v_kph: float = 15.0,
    duration_s: float = 5.0,
    dt: float = 0.01,
    controller_name: str = "4WS Calibrated LUT",
) -> ScenarioRunResult:
    """Simulate a tight 90-degree right turn into a narrow street corridor."""
    params = model.params
    v0_mps = v_kph / 3.6
    state = SevenDOFState(
        vx=v0_mps,
        vy=0.0,
        r=0.0,
        omega=np.full(4, v0_mps / params.Reff),
        X=0.0,
        Y=0.0,
        psi=0.0,
    )
    road = UniformRoad(mu=1.0)
    steering_sys = FourWheelSteering(SteeringConfig.from_yaml())

    # Path geometry: straight from X=0 to 12m, circular 90 deg right turn R=7.0m, straight along Y=-7 to -25m
    R_corner = 7.0
    def corner_path_fn(x: float) -> Tuple[float, float]:
        if x < 10.0:
            return 0.0, 0.0
        elif x < 10.0 + R_corner:
            dx = x - 10.0
            # circle equation: (x - 10)^2 + (y - (-R))^2 = R^2 => y = -R + sqrt(R^2 - dx^2)
            y = -R_corner + math.sqrt(max(0.0, R_corner**2 - dx**2))
            dy = -dx / math.sqrt(max(1e-4, R_corner**2 - dx**2))
            return y, math.atan(dy)
        else:
            return -R_corner - (x - (10.0 + R_corner)), -math.pi / 2.0

    driver = NeuromuscularDriverModel(
        config=DriverModelConfig(
            t_near_preview_s=0.30,
            t_far_preview_s=0.90,
            tau_delay_s=0.10,
            tau_neuromuscular_s=0.08,
            k_near_lat_error=0.12,
            k_far_heading=0.75,
        ),
        dt=dt,
    )
    driver.set_path_function(corner_path_fn)

    n_steps = int(duration_s / dt)
    time_arr = np.linspace(0.0, duration_s, n_steps)

    hist_X = np.zeros(n_steps)
    hist_Y = np.zeros(n_steps)
    hist_psi = np.zeros(n_steps)
    hist_vx = np.zeros(n_steps)
    hist_vy = np.zeros(n_steps)
    hist_r = np.zeros(n_steps)
    hist_beta = np.zeros(n_steps)
    hist_steer_f = np.zeros(n_steps)
    hist_steer_r = np.zeros(n_steps)
    hist_ay = np.zeros(n_steps)

    for i, t in enumerate(time_arr):
        drv_tel = driver.step(
            x=state.X,
            y=state.Y,
            psi=state.psi,
            vx=state.vx,
            vy=state.vy,
            r=state.r,
            dt=dt,
        )
        drv_steer = drv_tel.steer_actual_rad

        if enable_4ws:
            delta_f, delta_r = steering_sys.compute_steering_angles(
                cmd_steer_f=drv_steer,
                vx=state.vx,
                vy=state.vy,
                mode=steer_mode,
            )
        else:
            delta_f = drv_steer
            delta_r = 0.0

        t_drive = np.full(4, 45.0) if state.vx < v0_mps else np.zeros(4)
        t_brake = np.full(4, 25.0) if state.vx > v0_mps + 0.1 else np.zeros(4)

        u = SevenDOFControlInput(
            steer_f=delta_f,
            steer_r=delta_r,
            T_drive=t_drive,
            T_brake=t_brake,
        ).clamped(params)

        state, telem = model.step(state, u, dt=dt, road=road)

        hist_X[i] = state.X
        hist_Y[i] = state.Y
        hist_psi[i] = state.psi
        hist_vx[i] = state.vx
        hist_vy[i] = state.vy
        hist_r[i] = state.r
        beta = math.atan2(state.vy, max(abs(state.vx), 0.5))
        hist_beta[i] = math.degrees(beta)
        hist_steer_f[i] = math.degrees(delta_f)
        hist_steer_r[i] = math.degrees(delta_r)
        hist_ay[i] = float(np.sum(telem.Fy_body) / params.mass)

    metrics = ScenarioMetrics(
        scenario_name="Scenario 2: Urban 90° Cornering (15 km/h)",
        controller_name=controller_name,
        peak_sideslip_deg=float(np.max(np.abs(hist_beta))),
        peak_yaw_rate_deg_s=float(math.degrees(np.max(np.abs(hist_r)))),
        max_lateral_dev_m=float(np.min(hist_Y)),
    )

    return ScenarioRunResult(
        controller_name=controller_name,
        metrics=metrics,
        time=time_arr,
        X=hist_X,
        Y=hist_Y,
        psi=hist_psi,
        vx=hist_vx,
        vy=hist_vy,
        r=hist_r,
        beta_deg=hist_beta,
        steer_f_deg=hist_steer_f,
        steer_r_deg=hist_steer_r,
        ay_mps2=hist_ay,
    )


# =============================================================================
# SCENARIO 3: HIGH-SPEED EVASIVE DOUBLE LANE CHANGE (ISO 3888-2, 80 km/h)
# =============================================================================
def simulate_high_speed_double_lane_change(
    model: SevenDOFModel,
    controller: Any | None = None,
    v_kph: float = 80.0,
    duration_s: float = 3.5,
    dt: float = 0.01,
    controller_name: str = "4WS Calibrated LUT + Yaw Damping",
) -> ScenarioRunResult:
    """Simulate severe double lane change (ISO 3888-2) at 80 km/h."""
    params = model.params
    v0_mps = v_kph / 3.6
    state = SevenDOFState(
        vx=v0_mps,
        vy=0.0,
        r=0.0,
        omega=np.full(4, v0_mps / params.Reff),
        X=0.0,
        Y=0.0,
        psi=0.0,
    )
    road = UniformRoad(mu=1.0)

    def path_fn(x: float) -> Tuple[float, float]:
        if x < 15.0:
            return 0.0, 0.0
        elif x < 40.0:
            s = (x - 15.0) / 25.0
            y = 3.50 * (3.0 * s**2 - 2.0 * s**3)
            dy = (3.50 / 25.0) * (6.0 * s - 6.0 * s**2)
            return y, math.atan(dy)
        elif x < 55.0:
            return 3.50, 0.0
        elif x < 80.0:
            s = (x - 55.0) / 25.0
            y = 3.50 * (1.0 - (3.0 * s**2 - 2.0 * s**3))
            dy = -(3.50 / 25.0) * (6.0 * s - 6.0 * s**2)
            return y, math.atan(dy)
        else:
            return 0.0, 0.0

    driver = NeuromuscularDriverModel(
        config=DriverModelConfig(
            t_near_preview_s=0.25,
            t_far_preview_s=0.90,
            tau_delay_s=0.12,
            tau_neuromuscular_s=0.08,
            k_near_lat_error=0.08,
            k_far_heading=0.65,
        ),
        dt=dt,
    )
    driver.set_path_function(path_fn)

    n_steps = int(duration_s / dt)
    time_arr = np.linspace(0.0, duration_s, n_steps)

    hist_X = np.zeros(n_steps)
    hist_Y = np.zeros(n_steps)
    hist_psi = np.zeros(n_steps)
    hist_vx = np.zeros(n_steps)
    hist_vy = np.zeros(n_steps)
    hist_r = np.zeros(n_steps)
    hist_beta = np.zeros(n_steps)
    hist_steer_f = np.zeros(n_steps)
    hist_steer_r = np.zeros(n_steps)
    hist_ay = np.zeros(n_steps)

    last_telem = None

    for i, t in enumerate(time_arr):
        drv_tel = driver.step(
            x=state.X,
            y=state.Y,
            psi=state.psi,
            vx=state.vx,
            vy=state.vy,
            r=state.r,
            dt=dt,
        )
        drv_steer = drv_tel.steer_actual_rad

        if controller is not None:
            target = ControlTarget(
                target_vx=v0_mps,
                target_yaw_rate=0.0,
                driver_steer_f=drv_steer,
                driver_accel_pedal=0.10,
                driver_brake_pedal=0.0,
                mu_estimated=1.0,
            )
            ctrl_out = controller.compute_control(state, target, telemetry=last_telem, dt=dt)
            delta_f = ctrl_out.steer_f
            delta_r = ctrl_out.steer_r
            t_drive = ctrl_out.T_drive
            t_brake = ctrl_out.T_brake
        else:
            delta_f = drv_steer
            delta_r = 0.0
            t_drive = np.full(4, 50.0)
            t_brake = np.zeros(4)

        u = SevenDOFControlInput(
            steer_f=delta_f,
            steer_r=delta_r,
            T_drive=t_drive,
            T_brake=t_brake,
        ).clamped(params)

        state, telem = model.step(state, u, dt=dt, road=road)
        last_telem = telem

        hist_X[i] = state.X
        hist_Y[i] = state.Y
        hist_psi[i] = state.psi
        hist_vx[i] = state.vx
        hist_vy[i] = state.vy
        hist_r[i] = state.r
        beta = math.atan2(state.vy, max(abs(state.vx), 1.0))
        hist_beta[i] = math.degrees(beta)
        hist_steer_f[i] = math.degrees(delta_f)
        hist_steer_r[i] = math.degrees(delta_r)
        hist_ay[i] = float(np.sum(telem.Fy_body) / params.mass)

    target_y_arr = np.array([path_fn(x)[0] for x in hist_X])
    tracking_err = hist_Y - target_y_arr
    rms_err = math.sqrt(np.mean(tracking_err ** 2))

    metrics = ScenarioMetrics(
        scenario_name="Scenario 3: Evasive Double Lane Change (80 km/h ISO 3888-2)",
        controller_name=controller_name,
        peak_sideslip_deg=float(np.max(np.abs(hist_beta))),
        peak_yaw_rate_deg_s=float(math.degrees(np.max(np.abs(hist_r)))),
        lateral_tracking_rms_m=float(rms_err),
    )

    return ScenarioRunResult(
        controller_name=controller_name,
        metrics=metrics,
        time=time_arr,
        X=hist_X,
        Y=hist_Y,
        psi=hist_psi,
        vx=hist_vx,
        vy=hist_vy,
        r=hist_r,
        beta_deg=hist_beta,
        steer_f_deg=hist_steer_f,
        steer_r_deg=hist_steer_r,
        ay_mps2=hist_ay,
    )


# =============================================================================
# SCENARIO 4: HIGH-SPEED CONTINUOUS SLALOM (70 km/h, 30m Pylons)
# =============================================================================
def simulate_high_speed_slalom(
    model: SevenDOFModel,
    controller: Any | None = None,
    v_kph: float = 70.0,
    duration_s: float = 4.0,
    dt: float = 0.01,
    controller_name: str = "4WS Calibrated LUT",
) -> ScenarioRunResult:
    """Simulate continuous high-speed slalom weaving through pylons spaced 30m apart."""
    params = model.params
    v0_mps = v_kph / 3.6
    state = SevenDOFState(
        vx=v0_mps,
        vy=0.0,
        r=0.0,
        omega=np.full(4, v0_mps / params.Reff),
        X=0.0,
        Y=0.0,
        psi=0.0,
    )
    road = UniformRoad(mu=1.0)

    # Slalom path: sinusoid with wavelength lambda = 60m (30m between gates), amplitude 1.8m
    wavelength_m = 60.0
    amp_m = 1.80
    def slalom_path(x: float) -> Tuple[float, float]:
        if x < 10.0:
            return 0.0, 0.0
        s = (x - 10.0)
        y = amp_m * math.sin(2.0 * math.pi * s / wavelength_m)
        dy = amp_m * (2.0 * math.pi / wavelength_m) * math.cos(2.0 * math.pi * s / wavelength_m)
        return y, math.atan(dy)

    driver = NeuromuscularDriverModel(
        config=DriverModelConfig(
            t_near_preview_s=0.20,
            t_far_preview_s=0.80,
            tau_delay_s=0.10,
            tau_neuromuscular_s=0.08,
            k_near_lat_error=0.10,
            k_far_heading=0.70,
        ),
        dt=dt,
    )
    driver.set_path_function(slalom_path)

    n_steps = int(duration_s / dt)
    time_arr = np.linspace(0.0, duration_s, n_steps)

    hist_X = np.zeros(n_steps)
    hist_Y = np.zeros(n_steps)
    hist_psi = np.zeros(n_steps)
    hist_vx = np.zeros(n_steps)
    hist_vy = np.zeros(n_steps)
    hist_r = np.zeros(n_steps)
    hist_beta = np.zeros(n_steps)
    hist_steer_f = np.zeros(n_steps)
    hist_steer_r = np.zeros(n_steps)
    hist_ay = np.zeros(n_steps)

    last_telem = None

    for i, t in enumerate(time_arr):
        drv_tel = driver.step(
            x=state.X,
            y=state.Y,
            psi=state.psi,
            vx=state.vx,
            vy=state.vy,
            r=state.r,
            dt=dt,
        )
        drv_steer = drv_tel.steer_actual_rad

        if controller is not None:
            target = ControlTarget(
                target_vx=v0_mps,
                target_yaw_rate=0.0,
                driver_steer_f=drv_steer,
                driver_accel_pedal=0.05,
                driver_brake_pedal=0.0,
                mu_estimated=1.0,
            )
            ctrl_out = controller.compute_control(state, target, telemetry=last_telem, dt=dt)
            delta_f = ctrl_out.steer_f
            delta_r = ctrl_out.steer_r
        else:
            delta_f = drv_steer
            delta_r = 0.0

        u = SevenDOFControlInput(
            steer_f=delta_f,
            steer_r=delta_r,
            T_drive=np.zeros(4),
            T_brake=np.zeros(4),
        ).clamped(params)

        state, telem = model.step(state, u, dt=dt, road=road)
        last_telem = telem

        hist_X[i] = state.X
        hist_Y[i] = state.Y
        hist_psi[i] = state.psi
        hist_vx[i] = state.vx
        hist_vy[i] = state.vy
        hist_r[i] = state.r
        beta = math.atan2(state.vy, max(abs(state.vx), 1.0))
        hist_beta[i] = math.degrees(beta)
        hist_steer_f[i] = math.degrees(delta_f)
        hist_steer_r[i] = math.degrees(delta_r)
        hist_ay[i] = float(np.sum(telem.Fy_body) / params.mass)

    target_y = np.array([slalom_path(x)[0] for x in hist_X])
    tracking_err = hist_Y - target_y
    rms_err = math.sqrt(np.mean(tracking_err ** 2))

    metrics = ScenarioMetrics(
        scenario_name="Scenario 4: High-Speed Slalom (70 km/h, 30m Pylons)",
        controller_name=controller_name,
        peak_sideslip_deg=float(np.max(np.abs(hist_beta))),
        peak_yaw_rate_deg_s=float(math.degrees(np.max(np.abs(hist_r)))),
        lateral_tracking_rms_m=float(rms_err),
    )

    return ScenarioRunResult(
        controller_name=controller_name,
        metrics=metrics,
        time=time_arr,
        X=hist_X,
        Y=hist_Y,
        psi=hist_psi,
        vx=hist_vx,
        vy=hist_vy,
        r=hist_r,
        beta_deg=hist_beta,
        steer_f_deg=hist_steer_f,
        steer_r_deg=hist_steer_r,
        ay_mps2=hist_ay,
    )


# =============================================================================
# SCENARIO 5: HIGH-SPEED AUTOBAHN LANE CHANGE (130 km/h)
# =============================================================================
def simulate_autobahn_lane_change(
    model: SevenDOFModel,
    controller: Any | None = None,
    v_kph: float = 130.0,
    duration_s: float = 3.5,
    dt: float = 0.01,
    controller_name: str = "4WS Calibrated LUT",
) -> ScenarioRunResult:
    """Simulate a realistic high-speed highway lane change at 130 km/h."""
    params = model.params
    v0_mps = v_kph / 3.6
    state = SevenDOFState(
        vx=v0_mps,
        vy=0.0,
        r=0.0,
        omega=np.full(4, v0_mps / params.Reff),
        X=0.0,
        Y=0.0,
        psi=0.0,
    )
    road = UniformRoad(mu=1.0)

    # Gentle lane change: 3.5m over 80 meters (approx 2.2 seconds)
    def hwy_path(x: float) -> Tuple[float, float]:
        if x < 20.0:
            return 0.0, 0.0
        elif x < 100.0:
            s = (x - 20.0) / 80.0
            y = 3.50 * (3.0 * s**2 - 2.0 * s**3)
            dy = (3.50 / 80.0) * (6.0 * s - 6.0 * s**2)
            return y, math.atan(dy)
        else:
            return 3.50, 0.0

    driver = NeuromuscularDriverModel(
        config=DriverModelConfig(
            t_near_preview_s=0.35,
            t_far_preview_s=1.20,
            tau_delay_s=0.10,
            tau_neuromuscular_s=0.08,
            k_near_lat_error=0.04,
            k_far_heading=0.60,
        ),
        dt=dt,
    )
    driver.set_path_function(hwy_path)

    n_steps = int(duration_s / dt)
    time_arr = np.linspace(0.0, duration_s, n_steps)

    hist_X = np.zeros(n_steps)
    hist_Y = np.zeros(n_steps)
    hist_psi = np.zeros(n_steps)
    hist_vx = np.zeros(n_steps)
    hist_vy = np.zeros(n_steps)
    hist_r = np.zeros(n_steps)
    hist_beta = np.zeros(n_steps)
    hist_steer_f = np.zeros(n_steps)
    hist_steer_r = np.zeros(n_steps)
    hist_ay = np.zeros(n_steps)

    last_telem = None

    for i, t in enumerate(time_arr):
        drv_tel = driver.step(
            x=state.X,
            y=state.Y,
            psi=state.psi,
            vx=state.vx,
            vy=state.vy,
            r=state.r,
            dt=dt,
        )
        drv_steer = drv_tel.steer_actual_rad

        if controller is not None:
            target = ControlTarget(
                target_vx=v0_mps,
                target_yaw_rate=0.0,
                driver_steer_f=drv_steer,
                driver_accel_pedal=0.08,
                driver_brake_pedal=0.0,
                mu_estimated=1.0,
            )
            ctrl_out = controller.compute_control(state, target, telemetry=last_telem, dt=dt)
            delta_f = ctrl_out.steer_f
            delta_r = ctrl_out.steer_r
        else:
            delta_f = drv_steer
            delta_r = 0.0

        u = SevenDOFControlInput(
            steer_f=delta_f,
            steer_r=delta_r,
            T_drive=np.zeros(4),
            T_brake=np.zeros(4),
        ).clamped(params)

        state, telem = model.step(state, u, dt=dt, road=road)
        last_telem = telem

        hist_X[i] = state.X
        hist_Y[i] = state.Y
        hist_psi[i] = state.psi
        hist_vx[i] = state.vx
        hist_vy[i] = state.vy
        hist_r[i] = state.r
        beta = math.atan2(state.vy, max(abs(state.vx), 1.0))
        hist_beta[i] = math.degrees(beta)
        hist_steer_f[i] = math.degrees(delta_f)
        hist_steer_r[i] = math.degrees(delta_r)
        hist_ay[i] = float(np.sum(telem.Fy_body) / params.mass)

    target_y = np.array([hwy_path(x)[0] for x in hist_X])
    tracking_err = hist_Y - target_y
    rms_err = math.sqrt(np.mean(tracking_err ** 2))

    metrics = ScenarioMetrics(
        scenario_name="Scenario 5: Autobahn Lane Change (130 km/h)",
        controller_name=controller_name,
        peak_sideslip_deg=float(np.max(np.abs(hist_beta))),
        peak_yaw_rate_deg_s=float(math.degrees(np.max(np.abs(hist_r)))),
        lateral_tracking_rms_m=float(rms_err),
    )

    return ScenarioRunResult(
        controller_name=controller_name,
        metrics=metrics,
        time=time_arr,
        X=hist_X,
        Y=hist_Y,
        psi=hist_psi,
        vx=hist_vx,
        vy=hist_vy,
        r=hist_r,
        beta_deg=hist_beta,
        steer_f_deg=hist_steer_f,
        steer_r_deg=hist_steer_r,
        ay_mps2=hist_ay,
    )


# =============================================================================
# SCENARIO 6: TRANSIENT STEP STEER (ISO 7401, 100 km/h)
# =============================================================================
def simulate_high_speed_step_steer(
    model: SevenDOFModel,
    controller: Any | None = None,
    v_kph: float = 100.0,
    step_steer_deg: float = 3.0,
    duration_s: float = 2.5,
    dt: float = 0.01,
    controller_name: str = "4WS Calibrated LUT",
) -> ScenarioRunResult:
    """Simulate ISO 7401 high-speed transient step steer response at 100 km/h."""
    params = model.params
    v0_mps = v_kph / 3.6
    state = SevenDOFState(
        vx=v0_mps,
        vy=0.0,
        r=0.0,
        omega=np.full(4, v0_mps / params.Reff),
        X=0.0,
        Y=0.0,
        psi=0.0,
    )
    road = UniformRoad(mu=1.0)

    n_steps = int(duration_s / dt)
    time_arr = np.linspace(0.0, duration_s, n_steps)

    hist_X = np.zeros(n_steps)
    hist_Y = np.zeros(n_steps)
    hist_psi = np.zeros(n_steps)
    hist_vx = np.zeros(n_steps)
    hist_vy = np.zeros(n_steps)
    hist_r = np.zeros(n_steps)
    hist_beta = np.zeros(n_steps)
    hist_steer_f = np.zeros(n_steps)
    hist_steer_r = np.zeros(n_steps)
    hist_ay = np.zeros(n_steps)

    cmd_f_rad = math.radians(step_steer_deg)
    t_step_start = 0.50

    last_telem = None

    for i, t in enumerate(time_arr):
        if t < t_step_start:
            steer_cmd = 0.0
        elif t < t_step_start + 0.15:
            steer_cmd = cmd_f_rad * ((t - t_step_start) / 0.15)
        else:
            steer_cmd = cmd_f_rad

        if controller is not None:
            target = ControlTarget(
                target_vx=v0_mps,
                target_yaw_rate=0.0,
                driver_steer_f=steer_cmd,
                driver_accel_pedal=0.05,
                driver_brake_pedal=0.0,
                mu_estimated=1.0,
            )
            ctrl_out = controller.compute_control(state, target, telemetry=last_telem, dt=dt)
            delta_f = ctrl_out.steer_f
            delta_r = ctrl_out.steer_r
        else:
            delta_f = steer_cmd
            delta_r = 0.0

        u = SevenDOFControlInput(
            steer_f=delta_f,
            steer_r=delta_r,
            T_drive=np.zeros(4),
            T_brake=np.zeros(4),
        ).clamped(params)

        state, telem = model.step(state, u, dt=dt, road=road)
        last_telem = telem

        hist_X[i] = state.X
        hist_Y[i] = state.Y
        hist_psi[i] = state.psi
        hist_vx[i] = state.vx
        hist_vy[i] = state.vy
        hist_r[i] = state.r
        beta = math.atan2(state.vy, max(abs(state.vx), 1.0))
        hist_beta[i] = math.degrees(beta)
        hist_steer_f[i] = math.degrees(delta_f)
        hist_steer_r[i] = math.degrees(delta_r)
        hist_ay[i] = float(np.sum(telem.Fy_body) / params.mass)

    ay_50 = 0.50 * np.max(np.abs(hist_ay))
    idx_steer_50 = np.where(hist_steer_f >= 0.50 * step_steer_deg)[0]
    idx_ay_50 = np.where(np.abs(hist_ay) >= ay_50)[0]
    delay_ms = 0.0
    if len(idx_steer_50) > 0 and len(idx_ay_50) > 0:
        delay_ms = max(0.0, (time_arr[idx_ay_50[0]] - time_arr[idx_steer_50[0]]) * 1000.0)

    metrics = ScenarioMetrics(
        scenario_name="Scenario 6: Step Steer Response (100 km/h ISO 7401)",
        controller_name=controller_name,
        peak_sideslip_deg=float(np.max(np.abs(hist_beta))),
        peak_yaw_rate_deg_s=float(math.degrees(np.max(np.abs(hist_r)))),
        response_delay_ms=float(delay_ms),
    )

    return ScenarioRunResult(
        controller_name=controller_name,
        metrics=metrics,
        time=time_arr,
        X=hist_X,
        Y=hist_Y,
        psi=hist_psi,
        vx=hist_vx,
        vy=hist_vy,
        r=hist_r,
        beta_deg=hist_beta,
        steer_f_deg=hist_steer_f,
        steer_r_deg=hist_steer_r,
        ay_mps2=hist_ay,
    )


# =============================================================================
# SCENARIO 7: SPLIT-MU STRAIGHT-LINE EMERGENCY BRAKING (ECE-R13H, 100 km/h)
# =============================================================================
def simulate_split_mu_braking_4ws(
    model: SevenDOFModel,
    controller: Any | None = None,
    v_kph: float = 100.0,
    mu_left: float = 1.0,
    mu_right: float = 0.20,
    duration_s: float = 3.0,
    dt: float = 0.01,
    controller_name: str = "4WS Calibrated LUT + Yaw Damping",
) -> ScenarioRunResult:
    """Simulate ECE-R13H emergency braking on split-mu surface with active rear steering."""
    params = model.params
    v0_mps = v_kph / 3.6
    state = SevenDOFState(
        vx=v0_mps,
        vy=0.0,
        r=0.0,
        omega=np.full(4, v0_mps / params.Reff),
        X=0.0,
        Y=0.0,
        psi=0.0,
    )
    road = MuSplitRoad(mu_left=mu_left, mu_right=mu_right, split_y=0.0, transition_width=0.40)

    driver = NeuromuscularDriverModel(
        config=DriverModelConfig(
            t_near_preview_s=0.35,
            t_far_preview_s=1.35,
            tau_delay_s=0.18,
            tau_neuromuscular_s=0.10,
        ),
        dt=dt,
    )
    driver.set_path_function(lambda x: (0.0, 0.0))

    n_steps = int(duration_s / dt)
    time_arr = np.linspace(0.0, duration_s, n_steps)

    hist_X = np.zeros(n_steps)
    hist_Y = np.zeros(n_steps)
    hist_psi = np.zeros(n_steps)
    hist_vx = np.zeros(n_steps)
    hist_vy = np.zeros(n_steps)
    hist_r = np.zeros(n_steps)
    hist_beta = np.zeros(n_steps)
    hist_steer_f = np.zeros(n_steps)
    hist_steer_r = np.zeros(n_steps)
    hist_ay = np.zeros(n_steps)

    last_telem = None
    stopped_step = n_steps
    base_t_brake = np.array([1800.0, 1800.0, 1200.0, 1200.0], dtype=float)

    for i, t in enumerate(time_arr):
        if state.vx < 0.5:
            stopped_step = i
            break

        drv_tel = driver.step(
            x=state.X,
            y=state.Y,
            psi=state.psi,
            vx=state.vx,
            vy=state.vy,
            r=state.r,
            dt=dt,
        )
        drv_steer = drv_tel.steer_actual_rad

        if controller is not None:
            target = ControlTarget(
                target_vx=0.0,
                target_yaw_rate=0.0,
                driver_steer_f=drv_steer,
                driver_accel_pedal=0.0,
                driver_brake_pedal=0.30,
                mu_estimated=0.60,
            )
            ctrl_out = controller.compute_control(state, target, telemetry=last_telem, dt=dt)
            delta_f = ctrl_out.steer_f if abs(ctrl_out.steer_f) > 1e-4 else drv_steer
            delta_r = ctrl_out.steer_r
            t_drive = ctrl_out.T_drive
            t_brake = ctrl_out.T_brake if np.max(ctrl_out.T_brake) > 10.0 else base_t_brake
        else:
            delta_f = drv_steer
            delta_r = 0.0
            t_drive = np.zeros(4)
            t_brake = base_t_brake

        u = SevenDOFControlInput(
            steer_f=delta_f,
            steer_r=delta_r,
            T_drive=t_drive,
            T_brake=t_brake,
        ).clamped(params)

        state, telem = model.step(state, u, dt=dt, road=road)
        last_telem = telem

        hist_X[i] = state.X
        hist_Y[i] = state.Y
        hist_psi[i] = state.psi
        hist_vx[i] = state.vx
        hist_vy[i] = state.vy
        hist_r[i] = state.r
        beta = math.atan2(state.vy, max(abs(state.vx), 0.5))
        hist_beta[i] = math.degrees(beta)
        hist_steer_f[i] = math.degrees(delta_f)
        hist_steer_r[i] = math.degrees(delta_r)
        hist_ay[i] = float(np.sum(telem.Fy_body) / params.mass)

    hist_X = hist_X[:stopped_step]
    hist_Y = hist_Y[:stopped_step]
    hist_psi = hist_psi[:stopped_step]
    hist_vx = hist_vx[:stopped_step]
    hist_vy = hist_vy[:stopped_step]
    hist_r = hist_r[:stopped_step]
    hist_beta = hist_beta[:stopped_step]
    hist_steer_f = hist_steer_f[:stopped_step]
    hist_steer_r = hist_steer_r[:stopped_step]
    hist_ay = hist_ay[:stopped_step]
    time_arr = time_arr[:stopped_step]

    final_heading = float(math.degrees(abs(hist_psi[-1]))) if len(hist_psi) > 0 else 0.0
    max_lat_dev = float(np.max(np.abs(hist_Y))) if len(hist_Y) > 0 else 0.0
    status = "PASS" if (final_heading < 15.0 and max_lat_dev < 1.5) else "SPINOUT"

    metrics = ScenarioMetrics(
        scenario_name="Scenario 7: Split-μ Emergency Braking (100 km/h ECE-R13H)",
        controller_name=controller_name,
        peak_sideslip_deg=float(np.max(np.abs(hist_beta))),
        peak_yaw_rate_deg_s=float(math.degrees(np.max(np.abs(hist_r)))),
        final_heading_dev_deg=final_heading,
        max_lateral_dev_m=max_lat_dev,
        stability_status=status,
    )

    return ScenarioRunResult(
        controller_name=controller_name,
        metrics=metrics,
        time=time_arr,
        X=hist_X,
        Y=hist_Y,
        psi=hist_psi,
        vx=hist_vx,
        vy=hist_vy,
        r=hist_r,
        beta_deg=hist_beta,
        steer_f_deg=hist_steer_f,
        steer_r_deg=hist_steer_r,
        ay_mps2=hist_ay,
    )


# =============================================================================
# SCENARIO 8: REVERSE PARALLEL PARKING AGILITY (5 km/h, Counter-Phase)
# =============================================================================
def simulate_parallel_parking(
    model: SevenDOFModel,
    enable_4ws: bool = True,
    duration_s: float = 7.2,
    dt: float = 0.01,
    controller_name: str = "4WS Calibrated LUT",
) -> ScenarioRunResult:
    """Simulate low-speed reverse parallel parking into a tight roadside bay.

    Demonstrates virtual wheelbase shortening and rear-end sweep reduction:
    In counter-phase, steering the rear axle opposite to the front steers the rear
    directly toward the curb, cutting vehicle swept envelope width and achieving
    rapid alignment parallel to the curb with zero heading error.
    """
    params = model.params
    v0_mps = -1.20  # reverse at approx 4.3 km/h
    state = SevenDOFState(
        vx=v0_mps,
        vy=0.0,
        r=0.0,
        omega=np.full(4, v0_mps / params.Reff),
        X=6.0,
        Y=2.4,
        psi=0.0,
    )
    road = UniformRoad(mu=1.0)
    steering_sys = FourWheelSteering(SteeringConfig.from_yaml())

    n_steps = int(duration_s / dt)
    time_arr = np.linspace(0.0, duration_s, n_steps)

    hist_X = np.zeros(n_steps)
    hist_Y = np.zeros(n_steps)
    hist_psi = np.zeros(n_steps)
    hist_vx = np.zeros(n_steps)
    hist_vy = np.zeros(n_steps)
    hist_r = np.zeros(n_steps)
    hist_beta = np.zeros(n_steps)
    hist_steer_f = np.zeros(n_steps)
    hist_steer_r = np.zeros(n_steps)
    hist_ay = np.zeros(n_steps)

    steer_rad = math.radians(20.5)

    for i, t in enumerate(time_arr):
        # Calibrated S-curve reverse parallel parking trajectory:
        # Phase 1: 0.0 - 0.5s: straight reverse to clear front car's rear axle
        # Phase 2: 0.5 - 1.1s: ramp steer right to -20.5 deg (angle into bay)
        # Phase 3: 1.1 - 2.5s: hold -20.5 deg (rear axle tucks toward curb)
        # Phase 4: 2.5 - 3.7s: transition counter-steer to +20.5 deg (1.2s ramp)
        # Phase 5: 3.7 - 5.1s: hold +20.5 deg (pull front bumper into bay and align)
        # Phase 6: 5.1 - 5.7s: return steer to 0 deg
        # Phase 7: 5.7 - 7.2s: progressive brake to stop inside bay
        if t < 0.5:
            drv_steer = 0.0
        elif t < 1.1:
            drv_steer = -steer_rad * ((t - 0.5) / 0.6)
        elif t < 2.5:
            drv_steer = -steer_rad
        elif t < 3.7:
            prog = (t - 2.5) / 1.2
            drv_steer = -steer_rad + 2.0 * steer_rad * prog
        elif t < 5.1:
            drv_steer = steer_rad
        elif t < 5.7:
            prog = (t - 5.1) / 0.6
            drv_steer = steer_rad * (1.0 - prog)
        else:
            drv_steer = 0.0

        if enable_4ws:
            # Reversing at low speed -> Ackermann counter-phase
            delta_f, delta_r = steering_sys.compute_steering_angles(
                cmd_steer_f=drv_steer,
                vx=abs(state.vx),  # positive magnitude for LUT knot evaluation
                vy=state.vy,
                mode=SteeringMode.LOOKUP_TABLE,
            )
        else:
            delta_f = drv_steer
            delta_r = 0.0

        # Closed-loop velocity control for reverse parking
        if t < 5.7:
            err_v = v0_mps - state.vx  # negative if moving too fast backwards
            torque = err_v * 280.0 - 38.0
            t_drive = np.full(4, np.clip(torque, -160.0, 80.0))
            t_brake = np.zeros(4)
        else:
            t_drive = np.zeros(4)
            t_brake = np.full(4, 280.0)  # smooth brake to standstill

        u = SevenDOFControlInput(
            steer_f=delta_f,
            steer_r=delta_r,
            T_drive=t_drive,
            T_brake=t_brake,
        ).clamped(params)

        state, telem = model.step(state, u, dt=dt, road=road)

        hist_X[i] = state.X
        hist_Y[i] = state.Y
        hist_psi[i] = state.psi
        hist_vx[i] = state.vx
        hist_vy[i] = state.vy
        hist_r[i] = state.r
        beta = math.atan2(state.vy, max(abs(state.vx), 0.2))
        hist_beta[i] = math.degrees(beta)
        hist_steer_f[i] = math.degrees(delta_f)
        hist_steer_r[i] = math.degrees(delta_r)
        hist_ay[i] = float(np.sum(telem.Fy_body) / params.mass)

    sweep_width = float(np.max(hist_Y) - np.min(hist_Y))
    final_heading = float(math.degrees(abs(hist_psi[-1])))

    metrics = ScenarioMetrics(
        scenario_name="Scenario 3: Reverse Parallel Parking (5 km/h)",
        controller_name=controller_name,
        peak_sideslip_deg=float(np.max(np.abs(hist_beta))),
        peak_yaw_rate_deg_s=float(math.degrees(np.max(np.abs(hist_r)))),
        final_heading_dev_deg=final_heading,
        max_lateral_dev_m=sweep_width,
    )

    return ScenarioRunResult(
        controller_name=controller_name,
        metrics=metrics,
        time=time_arr,
        X=hist_X,
        Y=hist_Y,
        psi=hist_psi,
        vx=hist_vx,
        vy=hist_vy,
        r=hist_r,
        beta_deg=hist_beta,
        steer_f_deg=hist_steer_f,
        steer_r_deg=hist_steer_r,
        ay_mps2=hist_ay,
    )


# =============================================================================
# SCENARIO 9: NHTSA SINE WITH DWELL STABILITY MANEUVER (FMVSS 126, 80 km/h)
# =============================================================================
def simulate_sine_with_dwell(
    model: SevenDOFModel,
    controller: Any | None = None,
    v_kph: float = 80.0,
    steer_amp_deg: float = 5.0,
    duration_s: float = 4.5,
    dt: float = 0.01,
    controller_name: str = "4WS Calibrated LUT + Yaw Damping",
) -> ScenarioRunResult:
    """Simulate NHTSA FMVSS 126 / ISO 19365 Sine with Dwell evasive stability maneuver.

    Standard regulatory test for rollover and yaw stability:
    0.7 Hz sinusoidal steering input with a 500 ms pause/dwell at second peak steer.
    Tests transient damping and verifies the elimination of trailing yaw oscillations.
    """
    params = model.params
    v0_mps = v_kph / 3.6
    state = SevenDOFState(
        vx=v0_mps,
        vy=0.0,
        r=0.0,
        omega=np.full(4, v0_mps / params.Reff),
        X=0.0,
        Y=0.0,
        psi=0.0,
    )
    road = UniformRoad(mu=1.0)

    n_steps = int(duration_s / dt)
    time_arr = np.linspace(0.0, duration_s, n_steps)

    hist_X = np.zeros(n_steps)
    hist_Y = np.zeros(n_steps)
    hist_psi = np.zeros(n_steps)
    hist_vx = np.zeros(n_steps)
    hist_vy = np.zeros(n_steps)
    hist_r = np.zeros(n_steps)
    hist_beta = np.zeros(n_steps)
    hist_steer_f = np.zeros(n_steps)
    hist_steer_r = np.zeros(n_steps)
    hist_ay = np.zeros(n_steps)

    freq = 0.70  # Hz
    period = 1.0 / freq  # ~1.428s
    t_start = 0.50
    t_half = t_start + 0.5 * period  # ~1.214s
    t_three_quarter = t_start + 0.75 * period  # ~1.571s
    t_dwell_end = t_three_quarter + 0.50  # ~2.071s (500 ms dwell)
    t_return = t_dwell_end + 0.25 * period  # ~2.428s

    last_telem = None

    for i, t in enumerate(time_arr):
        if t < t_start:
            steer_deg = 0.0
        elif t < t_half:
            steer_deg = steer_amp_deg * math.sin(2.0 * math.pi * freq * (t - t_start))
        elif t < t_three_quarter:
            steer_deg = steer_amp_deg * math.sin(2.0 * math.pi * freq * (t - t_start))
        elif t < t_dwell_end:
            # 500 ms dwell at peak negative steer!
            steer_deg = -steer_amp_deg
        elif t < t_return:
            prog = (t - t_dwell_end) / (t_return - t_dwell_end)
            steer_deg = -steer_amp_deg * (1.0 - prog)
        else:
            steer_deg = 0.0

        drv_steer = math.radians(steer_deg)

        if controller is not None:
            target = ControlTarget(
                target_vx=v0_mps,
                target_yaw_rate=0.0,
                driver_steer_f=drv_steer,
                driver_accel_pedal=0.04,
                driver_brake_pedal=0.0,
                mu_estimated=1.0,
            )
            ctrl_out = controller.compute_control(state, target, telemetry=last_telem, dt=dt)
            delta_f = ctrl_out.steer_f
            delta_r = ctrl_out.steer_r
        else:
            delta_f = drv_steer
            delta_r = 0.0

        u = SevenDOFControlInput(
            steer_f=delta_f,
            steer_r=delta_r,
            T_drive=np.zeros(4),
            T_brake=np.zeros(4),
        ).clamped(params)

        state, telem = model.step(state, u, dt=dt, road=road)
        last_telem = telem

        hist_X[i] = state.X
        hist_Y[i] = state.Y
        hist_psi[i] = state.psi
        hist_vx[i] = state.vx
        hist_vy[i] = state.vy
        hist_r[i] = state.r
        beta = math.atan2(state.vy, max(abs(state.vx), 1.0))
        hist_beta[i] = math.degrees(beta)
        hist_steer_f[i] = math.degrees(delta_f)
        hist_steer_r[i] = math.degrees(delta_r)
        hist_ay[i] = float(np.sum(telem.Fy_body) / params.mass)

    # Post-dwell stability metric: yaw rate 1.0s after dwell completion
    idx_1s_post = np.where(time_arr >= t_dwell_end + 1.0)[0]
    r_1s_post = float(math.degrees(abs(hist_r[idx_1s_post[0]]))) if len(idx_1s_post) > 0 else 0.0

    metrics = ScenarioMetrics(
        scenario_name="Scenario 9: NHTSA Sine with Dwell (80 km/h FMVSS 126)",
        controller_name=controller_name,
        peak_sideslip_deg=float(np.max(np.abs(hist_beta))),
        peak_yaw_rate_deg_s=float(math.degrees(np.max(np.abs(hist_r)))),
        final_heading_dev_deg=r_1s_post,
        stability_status="STABLE" if r_1s_post < 5.0 else "UNSTABLE",
    )

    return ScenarioRunResult(
        controller_name=controller_name,
        metrics=metrics,
        time=time_arr,
        X=hist_X,
        Y=hist_Y,
        psi=hist_psi,
        vx=hist_vx,
        vy=hist_vy,
        r=hist_r,
        beta_deg=hist_beta,
        steer_f_deg=hist_steer_f,
        steer_r_deg=hist_steer_r,
        ay_mps2=hist_ay,
    )


# =============================================================================
# MULTI-SCENARIO DASHBOARD WITH PHYSICAL 2D VEHICLE FOOTPRINTS
# =============================================================================
def plot_4ws_six_scenario_dashboard(
    results: Dict[str, Dict[str, ScenarioRunResult]],
    output_file: str | Path,
) -> None:
    """Generate a comprehensive 6-panel dashboard featuring 2D vehicle rendering on all planar views."""
    out_path = Path(output_file)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    v_params = VehicleParams(wheelbase=2.80, width=1.80, length=4.50, tire_radius=0.32, tire_width=0.22, wheel_track=1.60)

    fig = plt.figure(figsize=(24, 18), dpi=180, facecolor="#0b1120")
    gs = fig.add_gridspec(3, 2, hspace=0.34, wspace=0.22)

    colors = {
        "2WS (Front-Only)": "#94a3b8",
        "4WS Speed-Scheduled": "#f59e0b",
        "4WS Calibrated LUT": "#06b6d4",
        "4WS Calibrated LUT + Yaw Damping": "#3b82f6",
        "Full Integrated IVMC": "#10b981",
    }

    # -------------------------------------------------------------------------
    # Panel 1: Low-Speed Turning Circle (12 km/h) with 2D Vehicle Footprints
    # -------------------------------------------------------------------------
    ax1 = fig.add_subplot(gs[0, 0], facecolor="#0f172a")
    res_s1 = results.get("turning_circle", {})

    for name, r in res_s1.items():
        c = colors.get(name, "#e2e8f0")
        ls = "--" if "2WS" in name else "-"
        lbl = f"{name} (R = {r.metrics.min_turning_radius_m:.2f} m)"
        ax1.plot(r.X, r.Y, color=c, linestyle=ls, linewidth=2.2, label=lbl)

        # Draw vehicle snapshots at 0%, 33%, 66%, 100%
        n_pts = len(r.X)
        for pct, alp in [(0.15, 0.45), (0.50, 0.65), (0.85, 0.90)]:
            idx = int(pct * (n_pts - 1))
            draw_vehicle(
                x=r.X[idx],
                y=r.Y[idx],
                yaw=r.psi[idx],
                steer_front=math.radians(r.steer_f_deg[idx]),
                params=v_params,
                steer_rear=math.radians(r.steer_r_deg[idx]),
                ax=ax1,
                color=c,
                alpha=alp,
                fill_alpha=0.20,
            )

    ax1.set_aspect("equal", adjustable="box")
    ax1.set_title("1. Low-Speed Turning Circle (12 km/h, 25° Steer) — Counter-Phase (Agilität)", color="#f8fafc", fontsize=11, fontweight="bold")
    ax1.set_xlabel("X Position [m]", color="#94a3b8")
    ax1.set_ylabel("Y Position [m]", color="#94a3b8")
    ax1.grid(True, linestyle=":", color="#334155", alpha=0.6)
    ax1.tick_params(colors="#94a3b8")
    ax1.legend(loc="upper left", fontsize=8.0, facecolor="#1e293b", edgecolor="#475569", labelcolor="#f8fafc")

    # -------------------------------------------------------------------------
    # Panel 2: Urban 90° Cornering & Curb Clearance (15 km/h)
    # -------------------------------------------------------------------------
    ax2 = fig.add_subplot(gs[0, 1], facecolor="#0f172a")
    res_s2 = results.get("urban_90deg", {})

    # Draw curb corner boundary
    curb_x = np.array([-5.0, 10.0, 10.0])
    curb_y = np.array([-2.5, -2.5, -25.0])
    ax2.plot(curb_x, curb_y, color="#ef4444", linestyle="-", linewidth=2.0, label="Bordstein / Straßenbegrenzung")

    for name, r in res_s2.items():
        c = colors.get(name, "#e2e8f0")
        ls = "--" if "2WS" in name else "-"
        ax2.plot(r.X, r.Y, color=c, linestyle=ls, linewidth=2.2, label=name)

        # Draw vehicle snapshots around corner
        n_pts = len(r.X)
        for pct in [0.15, 0.45, 0.75]:
            idx = int(pct * (n_pts - 1))
            draw_vehicle(
                x=r.X[idx],
                y=r.Y[idx],
                yaw=r.psi[idx],
                steer_front=math.radians(r.steer_f_deg[idx]),
                params=v_params,
                steer_rear=math.radians(r.steer_r_deg[idx]),
                ax=ax2,
                color=c,
                alpha=0.75,
                fill_alpha=0.25,
            )

    ax2.set_aspect("equal", adjustable="box")
    ax2.set_title("2. Städtisches 90°-Einbiegen (15 km/h) — Virtuelle Radstandsverkürzung", color="#f8fafc", fontsize=11, fontweight="bold")
    ax2.set_xlabel("X Position [m]", color="#94a3b8")
    ax2.set_ylabel("Y Position [m]", color="#94a3b8")
    ax2.grid(True, linestyle=":", color="#334155", alpha=0.6)
    ax2.tick_params(colors="#94a3b8")
    ax2.legend(loc="lower left", fontsize=8.0, facecolor="#1e293b", edgecolor="#475569", labelcolor="#f8fafc")

    # -------------------------------------------------------------------------
    # Panel 3: Evasive Double Lane Change (80 km/h ISO 3888-2)
    # -------------------------------------------------------------------------
    ax3 = fig.add_subplot(gs[1, 0], facecolor="#0f172a")
    res_s3 = results.get("double_lane_change", {})

    # Draw lane corridors
    ax3.axhline(0.0, color="#475569", linestyle=":", linewidth=1.2)
    ax3.axhline(3.50, color="#475569", linestyle=":", linewidth=1.2, label="Soll-Fahrspur (Y = 3.5 m)")

    for name, r in res_s3.items():
        c = colors.get(name, "#e2e8f0")
        ls = "--" if "2WS" in name else "-"
        lbl = f"{name} (|β|_max = {r.metrics.peak_sideslip_deg:.2f}°)"
        ax3.plot(r.X, r.Y, color=c, linestyle=ls, linewidth=2.2, label=lbl)

        # Vehicle footprints during lane reversal
        n_pts = len(r.X)
        for pct in [0.25, 0.50, 0.75]:
            idx = int(pct * (n_pts - 1))
            draw_vehicle(
                x=r.X[idx],
                y=r.Y[idx],
                yaw=r.psi[idx],
                steer_front=math.radians(r.steer_f_deg[idx]),
                params=v_params,
                steer_rear=math.radians(r.steer_r_deg[idx]),
                ax=ax3,
                color=c,
                alpha=0.70,
                fill_alpha=0.20,
            )

    ax3.set_aspect("equal", adjustable="box")
    ax3.set_title("3. Ausweichtest / Doppelspurwechsel (80 km/h ISO 3888-2) — Zero-Sideslip Krabben", color="#f8fafc", fontsize=11, fontweight="bold")
    ax3.set_xlabel("X Position [m]", color="#94a3b8")
    ax3.set_ylabel("Y Position [m]", color="#94a3b8")
    ax3.grid(True, linestyle=":", color="#334155", alpha=0.6)
    ax3.tick_params(colors="#94a3b8")
    ax3.legend(loc="lower right", fontsize=8.0, facecolor="#1e293b", edgecolor="#475569", labelcolor="#f8fafc")

    # -------------------------------------------------------------------------
    # Panel 4: High-Speed Slalom Maneuver (70 km/h, 30m Pylons)
    # -------------------------------------------------------------------------
    ax4 = fig.add_subplot(gs[1, 1], facecolor="#0f172a")
    res_s4 = results.get("slalom", {})

    # Draw slalom pylons
    pylons_x = [10.0, 40.0, 70.0, 100.0]
    pylons_y = [0.0, 0.0, 0.0, 0.0]
    ax4.scatter(pylons_x, pylons_y, color="#f59e0b", marker="^", s=90, zorder=6, label="Slalom-Pylonen (30 m Abstand)")

    for name, r in res_s4.items():
        c = colors.get(name, "#e2e8f0")
        ls = "--" if "2WS" in name else "-"
        ax4.plot(r.X, r.Y, color=c, linestyle=ls, linewidth=2.2, label=name)

        n_pts = len(r.X)
        for pct in [0.20, 0.45, 0.70]:
            idx = int(pct * (n_pts - 1))
            draw_vehicle(
                x=r.X[idx],
                y=r.Y[idx],
                yaw=r.psi[idx],
                steer_front=math.radians(r.steer_f_deg[idx]),
                params=v_params,
                steer_rear=math.radians(r.steer_r_deg[idx]),
                ax=ax4,
                color=c,
                alpha=0.70,
                fill_alpha=0.20,
            )

    ax4.set_aspect("equal", adjustable="box")
    ax4.set_title("4. Hochgeschwindigkeits-Slalom (70 km/h) — Phasenverzugselimination", color="#f8fafc", fontsize=11, fontweight="bold")
    ax4.set_xlabel("X Position [m]", color="#94a3b8")
    ax4.set_ylabel("Y Position [m]", color="#94a3b8")
    ax4.grid(True, linestyle=":", color="#334155", alpha=0.6)
    ax4.tick_params(colors="#94a3b8")
    ax4.legend(loc="lower right", fontsize=8.0, facecolor="#1e293b", edgecolor="#475569", labelcolor="#f8fafc")

    # -------------------------------------------------------------------------
    # Panel 5: Autobahn Lane Change at 130 km/h (Sideslip Beta Angle Suppression)
    # -------------------------------------------------------------------------
    ax5 = fig.add_subplot(gs[2, 0], facecolor="#0f172a")
    res_s5 = results.get("autobahn", {})

    for name, r in res_s5.items():
        c = colors.get(name, "#e2e8f0")
        ls = "--" if "2WS" in name else "-"
        lbl = f"{name} (|β|_max = {r.metrics.peak_sideslip_deg:.2f}°)"
        ax5.plot(r.time, r.beta_deg, color=c, linestyle=ls, linewidth=2.2, label=lbl)

    ax5.axhline(0.0, color="#64748b", linestyle="--", linewidth=1.0)
    ax5.set_title("5. Autobahn-Spurwechsel (130 km/h) — Schwimmwinkelvergleich β(t)", color="#f8fafc", fontsize=11, fontweight="bold")
    ax5.set_xlabel("Zeit [s]", color="#94a3b8")
    ax5.set_ylabel("Karosserie-Schwimmwinkel β [deg]", color="#94a3b8")
    ax5.grid(True, linestyle=":", color="#334155", alpha=0.6)
    ax5.tick_params(colors="#94a3b8")
    ax5.legend(loc="upper right", fontsize=8.5, facecolor="#1e293b", edgecolor="#475569", labelcolor="#f8fafc")

    # -------------------------------------------------------------------------
    # Panel 6: Step Steer Response (100 km/h ISO 7401)
    # -------------------------------------------------------------------------
    ax6 = fig.add_subplot(gs[2, 1], facecolor="#0f172a")
    res_s6 = results.get("step_steer", {})

    for name, r in res_s6.items():
        c = colors.get(name, "#e2e8f0")
        ls = "--" if "2WS" in name else "-"
        lbl = f"{name} (Verzug = {r.metrics.response_delay_ms:.1f} ms)"
        ax6.plot(r.time, r.ay_mps2, color=c, linestyle=ls, linewidth=2.2, label=lbl)

    ax6.set_title("6. Sprunglenk-Übergangsverhalten (100 km/h ISO 7401) — Querbeschleunigung a_y(t)", color="#f8fafc", fontsize=11, fontweight="bold")
    ax6.set_xlabel("Zeit [s]", color="#94a3b8")
    ax6.set_ylabel("Querbeschleunigung a_y [m/s²]", color="#94a3b8")
    ax6.grid(True, linestyle=":", color="#334155", alpha=0.6)
    ax6.tick_params(colors="#94a3b8")
    ax6.legend(loc="lower right", fontsize=8.5, facecolor="#1e293b", edgecolor="#475569", labelcolor="#f8fafc")

    fig.suptitle(
        "Vierradlenkung (4WS) Multi-Szenario Gesamtevaluation mit 2D-Fahrzeuggeometrie\n"
        "Gegenphasige Agilität beim Rangieren vs. Gleichphasige Zero-Sideslip Krabbenbewegung bei Autobahntempo",
        fontsize=16,
        fontweight="bold",
        color="#f8fafc",
        y=0.985,
    )

    plt.savefig(out_path, dpi=180, facecolor=fig.get_facecolor(), bbox_inches="tight")
    plt.close(fig)


# =============================================================================
# 9-SCENARIO COMPREHENSIVE BENCHMARK DASHBOARD WITH 2D VEHICLE FOOTPRINTS
# =============================================================================
def plot_4ws_nine_scenario_dashboard(
    results: Dict[str, Dict[str, ScenarioRunResult]],
    output_file: str | Path,
) -> None:
    """Generate a comprehensive 9-panel dashboard featuring 2D vehicle rendering across all 9 scenarios."""
    out_path = Path(output_file)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    v_params = VehicleParams(wheelbase=2.80, width=1.80, length=4.50, tire_radius=0.32, tire_width=0.22, wheel_track=1.60)

    fig = plt.figure(figsize=(26, 22), dpi=180, facecolor="#0b1120")
    gs = fig.add_gridspec(3, 3, hspace=0.36, wspace=0.24)

    colors = {
        "2WS (Front-Only)": "#94a3b8",
        "4WS Speed-Scheduled": "#f59e0b",
        "4WS Calibrated LUT": "#06b6d4",
        "4WS Calibrated LUT + Yaw Damping": "#3b82f6",
        "Full Integrated IVMC": "#10b981",
    }

    # -------------------------------------------------------------------------
    # Panel 1: Low-Speed Turning Circle (12 km/h)
    # -------------------------------------------------------------------------
    ax1 = fig.add_subplot(gs[0, 0], facecolor="#0f172a")
    res_s1 = results.get("turning_circle", {})
    for name, r in res_s1.items():
        c = colors.get(name, "#e2e8f0")
        ls = "--" if "2WS" in name else "-"
        lbl = f"{name} (R = {r.metrics.min_turning_radius_m:.2f} m)"
        ax1.plot(r.X, r.Y, color=c, linestyle=ls, linewidth=2.0, label=lbl)
        n_pts = len(r.X)
        for pct, alp in [(0.15, 0.45), (0.50, 0.65), (0.85, 0.90)]:
            idx = int(pct * (n_pts - 1))
            draw_vehicle(
                x=r.X[idx], y=r.Y[idx], yaw=r.psi[idx],
                steer_front=math.radians(r.steer_f_deg[idx]),
                params=v_params, steer_rear=math.radians(r.steer_r_deg[idx]),
                ax=ax1, color=c, alpha=alp, fill_alpha=0.20,
            )
    ax1.set_aspect("equal", adjustable="box")
    ax1.set_title("1. Wendekreis (12 km/h, 25° Steer) — Gegenphase", color="#f8fafc", fontsize=10.5, fontweight="bold")
    ax1.set_xlabel("X [m]", color="#94a3b8")
    ax1.set_ylabel("Y [m]", color="#94a3b8")
    ax1.grid(True, linestyle=":", color="#334155", alpha=0.6)
    ax1.tick_params(colors="#94a3b8")
    ax1.legend(loc="upper left", fontsize=7.5, facecolor="#1e293b", edgecolor="#475569", labelcolor="#f8fafc")

    # -------------------------------------------------------------------------
    # Panel 2: Urban 90° Cornering (15 km/h)
    # -------------------------------------------------------------------------
    ax2 = fig.add_subplot(gs[0, 1], facecolor="#0f172a")
    res_s2 = results.get("urban_90deg", {})
    curb_x = np.array([-5.0, 10.0, 10.0])
    curb_y = np.array([-2.5, -2.5, -25.0])
    ax2.plot(curb_x, curb_y, color="#ef4444", linestyle="-", linewidth=2.0, label="Bordsteinbegrenzung")
    for name, r in res_s2.items():
        c = colors.get(name, "#e2e8f0")
        ls = "--" if "2WS" in name else "-"
        ax2.plot(r.X, r.Y, color=c, linestyle=ls, linewidth=2.0, label=name)
        n_pts = len(r.X)
        for pct in [0.15, 0.45, 0.75]:
            idx = int(pct * (n_pts - 1))
            draw_vehicle(
                x=r.X[idx], y=r.Y[idx], yaw=r.psi[idx],
                steer_front=math.radians(r.steer_f_deg[idx]),
                params=v_params, steer_rear=math.radians(r.steer_r_deg[idx]),
                ax=ax2, color=c, alpha=0.75, fill_alpha=0.25,
            )
    ax2.set_aspect("equal", adjustable="box")
    ax2.set_title("2. Städtisches 90°-Einbiegen (15 km/h) — Bordsteinabstand", color="#f8fafc", fontsize=10.5, fontweight="bold")
    ax2.set_xlabel("X [m]", color="#94a3b8")
    ax2.set_ylabel("Y [m]", color="#94a3b8")
    ax2.grid(True, linestyle=":", color="#334155", alpha=0.6)
    ax2.tick_params(colors="#94a3b8")
    ax2.legend(loc="lower left", fontsize=7.5, facecolor="#1e293b", edgecolor="#475569", labelcolor="#f8fafc")

    # -------------------------------------------------------------------------
    # Panel 3: Reverse Parallel Parking (5 km/h)
    # -------------------------------------------------------------------------
    ax3 = fig.add_subplot(gs[0, 2], facecolor="#0f172a")
    res_s8 = results.get("parallel_parking", {})
    # Draw parking bay & obstacle outlines
    ax3.axhline(0.0, color="#64748b", linestyle="-", linewidth=1.8, label="Bordstein (Y = 0m)")
    ax3.axhline(2.0, color="#475569", linestyle="--", linewidth=1.2, label="Parkstreifenlinie (Y = 2m)")
    ax3.plot([7.5, 3.0, 3.0, 7.5, 7.5], [0.1, 0.1, 1.9, 1.9, 0.1], color="#94a3b8", linestyle="--", linewidth=1.2, label="Voraus-Fahrzeug")
    ax3.plot([-4.5, -9.0, -9.0, -4.5, -4.5], [0.1, 0.1, 1.9, 1.9, 0.1], color="#94a3b8", linestyle="--", linewidth=1.2, label="Nachfolge-Fahrzeug")
    ax3.plot([3.0, -4.5, -4.5, 3.0], [0.0, 0.0, 2.0, 2.0], color="#10b981", linestyle="--", linewidth=1.5, label="Parkbucht (7.5 m)")

    for name, r in res_s8.items():
        c = colors.get(name, "#e2e8f0")
        ls = "--" if "2WS" in name else "-"
        lbl = f"{name} (Y_end = {r.Y[-1]:.2f}m, Δψ = {r.metrics.final_heading_dev_deg:.2f}°)"
        ax3.plot(r.X, r.Y, color=c, linestyle=ls, linewidth=2.0, label=lbl)
        n_pts = len(r.X)
        for pct in [0.0, 0.40, 1.0]:
            idx = int(pct * (n_pts - 1))
            draw_vehicle(
                x=r.X[idx], y=r.Y[idx], yaw=r.psi[idx],
                steer_front=math.radians(r.steer_f_deg[idx]),
                params=v_params, steer_rear=math.radians(r.steer_r_deg[idx]),
                ax=ax3, color=c, alpha=0.85, fill_alpha=0.28,
            )
    ax3.set_xlim(-10.0, 8.5)
    ax3.set_ylim(-0.8, 3.6)
    ax3.set_aspect("equal", adjustable="box")
    ax3.set_title("3. Rückwärts-Einparken (5 km/h) — Heck-Eindrehung & Hüllflächenreduktion", color="#f8fafc", fontsize=10.5, fontweight="bold")
    ax3.set_xlabel("X [m]", color="#94a3b8")
    ax3.set_ylabel("Y [m]", color="#94a3b8")
    ax3.grid(True, linestyle=":", color="#334155", alpha=0.6)
    ax3.tick_params(colors="#94a3b8")
    ax3.legend(loc="upper left", fontsize=7.2, facecolor="#1e293b", edgecolor="#475569", labelcolor="#f8fafc")

    # -------------------------------------------------------------------------
    # Panel 4: Double Lane Change (80 km/h ISO 3888-2)
    # -------------------------------------------------------------------------
    ax4 = fig.add_subplot(gs[1, 0], facecolor="#0f172a")
    res_s3 = results.get("double_lane_change", {})
    ax4.axhline(0.0, color="#475569", linestyle=":", linewidth=1.0)
    ax4.axhline(3.50, color="#475569", linestyle=":", linewidth=1.0, label="Soll-Fahrspur (Y = 3.5 m)")
    for name, r in res_s3.items():
        c = colors.get(name, "#e2e8f0")
        ls = "--" if "2WS" in name else "-"
        lbl = f"{name} (|β|_max = {r.metrics.peak_sideslip_deg:.2f}°)"
        ax4.plot(r.X, r.Y, color=c, linestyle=ls, linewidth=2.0, label=lbl)
        n_pts = len(r.X)
        for pct in [0.25, 0.50, 0.75]:
            idx = int(pct * (n_pts - 1))
            draw_vehicle(
                x=r.X[idx], y=r.Y[idx], yaw=r.psi[idx],
                steer_front=math.radians(r.steer_f_deg[idx]),
                params=v_params, steer_rear=math.radians(r.steer_r_deg[idx]),
                ax=ax4, color=c, alpha=0.70, fill_alpha=0.20,
            )
    ax4.set_aspect("equal", adjustable="box")
    ax4.set_title("4. Doppelspurwechsel (80 km/h ISO 3888-2) — Krabben", color="#f8fafc", fontsize=10.5, fontweight="bold")
    ax4.set_xlabel("X [m]", color="#94a3b8")
    ax4.set_ylabel("Y [m]", color="#94a3b8")
    ax4.grid(True, linestyle=":", color="#334155", alpha=0.6)
    ax4.tick_params(colors="#94a3b8")
    ax4.legend(loc="lower right", fontsize=7.5, facecolor="#1e293b", edgecolor="#475569", labelcolor="#f8fafc")

    # -------------------------------------------------------------------------
    # Panel 5: High-Speed Slalom (70 km/h, 30m Pylons)
    # -------------------------------------------------------------------------
    ax5 = fig.add_subplot(gs[1, 1], facecolor="#0f172a")
    res_s4 = results.get("slalom", {})
    pylons_x = [10.0, 40.0, 70.0, 100.0]
    pylons_y = [0.0, 0.0, 0.0, 0.0]
    ax5.scatter(pylons_x, pylons_y, color="#f59e0b", marker="^", s=80, zorder=6, label="Slalom-Pylonen (30 m)")
    for name, r in res_s4.items():
        c = colors.get(name, "#e2e8f0")
        ls = "--" if "2WS" in name else "-"
        ax5.plot(r.X, r.Y, color=c, linestyle=ls, linewidth=2.0, label=name)
        n_pts = len(r.X)
        for pct in [0.20, 0.45, 0.70]:
            idx = int(pct * (n_pts - 1))
            draw_vehicle(
                x=r.X[idx], y=r.Y[idx], yaw=r.psi[idx],
                steer_front=math.radians(r.steer_f_deg[idx]),
                params=v_params, steer_rear=math.radians(r.steer_r_deg[idx]),
                ax=ax5, color=c, alpha=0.70, fill_alpha=0.20,
            )
    ax5.set_aspect("equal", adjustable="box")
    ax5.set_title("5. Slalom (70 km/h, 30 m) — Phasenverzugselimination", color="#f8fafc", fontsize=10.5, fontweight="bold")
    ax5.set_xlabel("X [m]", color="#94a3b8")
    ax5.set_ylabel("Y [m]", color="#94a3b8")
    ax5.grid(True, linestyle=":", color="#334155", alpha=0.6)
    ax5.tick_params(colors="#94a3b8")
    ax5.legend(loc="lower right", fontsize=7.5, facecolor="#1e293b", edgecolor="#475569", labelcolor="#f8fafc")

    # -------------------------------------------------------------------------
    # Panel 6: Split-μ Straight-Line Emergency Braking (100 km/h ECE-R13H)
    # -------------------------------------------------------------------------
    ax6 = fig.add_subplot(gs[1, 2], facecolor="#0f172a")
    res_s7 = results.get("split_mu", {})
    # Render Split-Mu surface background
    ax6.axhspan(0.0, 5.0, color="#0284c7", alpha=0.10, label="High-μ Spur (μ = 1.0, Trockener Asphalt)")
    ax6.axhspan(-5.0, 0.0, color="#93c5fd", alpha=0.10, label="Low-μ Spur (μ = 0.2, Nasses Eis)")
    ax6.axhline(0.0, color="#f59e0b", linestyle="-", linewidth=1.5, label="Reibwertsprung-Kante (Y = 0)")

    for name, r in res_s7.items():
        c = colors.get(name, "#e2e8f0")
        ls = "--" if "2WS" in name else "-"
        lbl = f"{name} (Δψ = {r.metrics.final_heading_dev_deg:.1f}°)"
        ax6.plot(r.X, r.Y, color=c, linestyle=ls, linewidth=2.0, label=lbl)
        n_pts = len(r.X)
        for pct in [0.20, 0.55, 0.90]:
            if n_pts > 5:
                idx = int(pct * (n_pts - 1))
                draw_vehicle(
                    x=r.X[idx], y=r.Y[idx], yaw=r.psi[idx],
                    steer_front=math.radians(r.steer_f_deg[idx]),
                    params=v_params, steer_rear=math.radians(r.steer_r_deg[idx]),
                    ax=ax6, color=c, alpha=0.75, fill_alpha=0.25,
                )
    ax6.set_aspect("equal", adjustable="box")
    ax6.set_title("6. Split-μ Bremsung (100 km/h ECE-R13H) — Aktive Gierdämpfung", color="#f8fafc", fontsize=10.5, fontweight="bold")
    ax6.set_xlabel("X [m]", color="#94a3b8")
    ax6.set_ylabel("Y [m]", color="#94a3b8")
    ax6.set_ylim(-3.0, 3.0)
    ax6.grid(True, linestyle=":", color="#334155", alpha=0.6)
    ax6.tick_params(colors="#94a3b8")
    ax6.legend(loc="upper left", fontsize=7.5, facecolor="#1e293b", edgecolor="#475569", labelcolor="#f8fafc")

    # -------------------------------------------------------------------------
    # Panel 7: Autobahn Lane Change (130 km/h) — Sideslip Beta(t)
    # -------------------------------------------------------------------------
    ax7 = fig.add_subplot(gs[2, 0], facecolor="#0f172a")
    res_s5 = results.get("autobahn", {})
    for name, r in res_s5.items():
        c = colors.get(name, "#e2e8f0")
        ls = "--" if "2WS" in name else "-"
        lbl = f"{name} (|β|_max = {r.metrics.peak_sideslip_deg:.2f}°)"
        ax7.plot(r.time, r.beta_deg, color=c, linestyle=ls, linewidth=2.0, label=lbl)
    ax7.axhline(0.0, color="#64748b", linestyle="--", linewidth=1.0)
    ax7.set_title("7. Autobahn-Spurwechsel (130 km/h) — Schwimmwinkel β(t)", color="#f8fafc", fontsize=10.5, fontweight="bold")
    ax7.set_xlabel("Zeit [s]", color="#94a3b8")
    ax7.set_ylabel("Schwimmwinkel β [deg]", color="#94a3b8")
    ax7.grid(True, linestyle=":", color="#334155", alpha=0.6)
    ax7.tick_params(colors="#94a3b8")
    ax7.legend(loc="upper right", fontsize=7.5, facecolor="#1e293b", edgecolor="#475569", labelcolor="#f8fafc")

    # -------------------------------------------------------------------------
    # Panel 8: Step Steer Response (100 km/h ISO 7401) — Ay(t)
    # -------------------------------------------------------------------------
    ax8 = fig.add_subplot(gs[2, 1], facecolor="#0f172a")
    res_s6 = results.get("step_steer", {})
    for name, r in res_s6.items():
        c = colors.get(name, "#e2e8f0")
        ls = "--" if "2WS" in name else "-"
        lbl = f"{name} (Verzug = {r.metrics.response_delay_ms:.1f} ms)"
        ax8.plot(r.time, r.ay_mps2, color=c, linestyle=ls, linewidth=2.0, label=lbl)
    ax8.set_title("8. Sprunglenkung (100 km/h ISO 7401) — Querbeschleunigung a_y(t)", color="#f8fafc", fontsize=10.5, fontweight="bold")
    ax8.set_xlabel("Zeit [s]", color="#94a3b8")
    ax8.set_ylabel("a_y [m/s²]", color="#94a3b8")
    ax8.grid(True, linestyle=":", color="#334155", alpha=0.6)
    ax8.tick_params(colors="#94a3b8")
    ax8.legend(loc="lower right", fontsize=7.5, facecolor="#1e293b", edgecolor="#475569", labelcolor="#f8fafc")

    # -------------------------------------------------------------------------
    # Panel 9: NHTSA Sine with Dwell (80 km/h FMVSS 126) — Yaw Rate r(t)
    # -------------------------------------------------------------------------
    ax9 = fig.add_subplot(gs[2, 2], facecolor="#0f172a")
    res_s9 = results.get("sine_with_dwell", {})
    for name, r in res_s9.items():
        c = colors.get(name, "#e2e8f0")
        ls = "--" if "2WS" in name else "-"
        lbl = f"{name} (Status: {r.metrics.stability_status})"
        ax9.plot(r.time, [math.degrees(val) for val in r.r], color=c, linestyle=ls, linewidth=2.0, label=lbl)
    ax9.axhline(0.0, color="#64748b", linestyle="--", linewidth=1.0)
    ax9.axvspan(1.57, 2.07, color="#ef4444", alpha=0.15, label="500 ms Dwell Pause")
    ax9.set_title("9. NHTSA Sine with Dwell (80 km/h FMVSS 126) — Gierrate r(t)", color="#f8fafc", fontsize=10.5, fontweight="bold")
    ax9.set_xlabel("Zeit [s]", color="#94a3b8")
    ax9.set_ylabel("Gierrate r [deg/s]", color="#94a3b8")
    ax9.grid(True, linestyle=":", color="#334155", alpha=0.6)
    ax9.tick_params(colors="#94a3b8")
    ax9.legend(loc="upper right", fontsize=7.5, facecolor="#1e293b", edgecolor="#475569", labelcolor="#f8fafc")

    fig.suptitle(
        "Vierradlenkung (4WS) 9-Szenarien Gesamtevaluation mit 2D-Fahrzeuggeometrie\n"
        "Gegenphasige Agilität, Gleichphasiges Krabben, Aktive Split-μ Gierstabilität & NHTSA FMVSS 126 Dämpfung",
        fontsize=16,
        fontweight="bold",
        color="#f8fafc",
        y=0.985,
    )

    plt.savefig(out_path, dpi=180, facecolor=fig.get_facecolor(), bbox_inches="tight")
    plt.close(fig)


# =============================================================================
# HIGH-DEFINITION ANIMATIONS WITH RENDERED 2D STEERABLE WHEELS & TELEMETRY HUD
# =============================================================================
def animate_4ws_detailed_vehicle(
    result_4ws: ScenarioRunResult,
    result_2ws: ScenarioRunResult,
    output_file: str | Path,
    title: str = "4WS Vehicle Dynamics Animation",
    fps: int = 15,
    setup_ax_fn: Optional[Callable[[plt.Axes], None]] = None,
    x_lim: Optional[Tuple[float, float]] = None,
    y_lim: Optional[Tuple[float, float]] = None,
    figsize: Tuple[float, float] = (16, 7),
) -> None:
    """Generate high-definition animated GIF rendering real 2D vehicle bodies with steered wheels and live HUD."""
    out_path = Path(output_file)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    v_params = VehicleParams(wheelbase=2.80, width=1.80, length=4.50, tire_radius=0.32, tire_width=0.22, wheel_track=1.60)

    fig, ax = plt.subplots(figsize=figsize, dpi=140, facecolor="#0b1120")
    ax.set_facecolor("#0f172a")

    if x_lim is not None:
        ax.set_xlim(x_lim)
    else:
        x_min = min(np.min(result_4ws.X), np.min(result_2ws.X)) - 8.0
        x_max = max(np.max(result_4ws.X), np.max(result_2ws.X)) + 8.0
        ax.set_xlim(x_min, x_max)

    if y_lim is not None:
        ax.set_ylim(y_lim)
    else:
        y_min = min(np.min(result_4ws.Y), np.min(result_2ws.Y)) - 4.0
        y_max = max(np.max(result_4ws.Y), np.max(result_2ws.Y)) + 4.0
        ax.set_ylim(y_min, y_max)

    ax.set_aspect("equal", adjustable="box")
    ax.set_title(title, color="#f8fafc", fontsize=12, fontweight="bold", pad=12)
    ax.set_xlabel("X Position [m]", color="#94a3b8")
    ax.set_ylabel("Y Position [m]", color="#94a3b8")
    ax.grid(True, linestyle=":", color="#334155", alpha=0.5)
    ax.tick_params(colors="#94a3b8")

    # Static trajectory guide lines
    ax.plot(result_2ws.X, result_2ws.Y, color="#64748b", linestyle="--", linewidth=1.5, label="2WS Spurverlauf (Front-Only)")
    ax.plot(result_4ws.X, result_4ws.Y, color="#0284c7", linestyle="-", linewidth=2.0, label="4WS Spurverlauf (Calibrated LUT)")

    if setup_ax_fn is not None:
        setup_ax_fn(ax)

    ax.legend(loc="lower right", facecolor="#1e293b", edgecolor="#475569", labelcolor="#f8fafc", fontsize=9)

    # Telemetry HUD text box
    hud_text = ax.text(
        0.02, 0.94, "",
        transform=ax.transAxes,
        color="#f8fafc",
        fontsize=9.5,
        fontfamily="monospace",
        verticalalignment="top",
        bbox=dict(boxstyle="round,pad=0.6", facecolor="#111827", edgecolor="#38bdf8", alpha=0.92, linewidth=1.5),
    )

    n_frames = min(len(result_4ws.time), len(result_2ws.time))
    step_skip = max(1, n_frames // 70)
    frame_indices = list(range(0, n_frames, step_skip))

    initial_n_lines = len(ax.lines)
    initial_n_patches = len(ax.patches)

    def update(frame_idx: int):
        # Clear previous frame's vehicle lines and patches
        while len(ax.lines) > initial_n_lines:
            ax.lines[-1].remove()
        while len(ax.patches) > initial_n_patches:
            ax.patches[-1].remove()

        idx = frame_indices[frame_idx]
        t = result_4ws.time[idx]

        # 1. Render 2WS Vehicle (Front-Only, Steer Rear = 0)
        draw_vehicle(
            x=result_2ws.X[idx],
            y=result_2ws.Y[idx],
            yaw=result_2ws.psi[idx],
            steer_front=math.radians(result_2ws.steer_f_deg[idx]),
            params=v_params,
            steer_rear=0.0,
            ax=ax,
            color="#94a3b8",
            alpha=0.65,
            fill_alpha=0.20,
            scale=1.1,
        )

        # 2. Render 4WS Vehicle (Steer Front & Steer Rear visibly rendered!)
        draw_vehicle(
            x=result_4ws.X[idx],
            y=result_4ws.Y[idx],
            yaw=result_4ws.psi[idx],
            steer_front=math.radians(result_4ws.steer_f_deg[idx]),
            params=v_params,
            steer_rear=math.radians(result_4ws.steer_r_deg[idx]),
            ax=ax,
            color="#06b6d4",
            color_front="#38bdf8",
            color_rear="#f43f5e" if abs(result_4ws.steer_r_deg[idx]) > 0.3 else "#06b6d4",
            alpha=0.95,
            fill_alpha=0.35,
            scale=1.1,
        )

        # Telemetry mode string
        is_split_mu = abs(result_4ws.steer_r_deg[idx]) > 0.15 and abs(result_4ws.steer_f_deg[idx]) < 0.25
        if is_split_mu:
            mode_str = "AKTIVE GIERDÄMPFUNG (SPLIT-μ)"
        elif result_4ws.steer_r_deg[idx] * result_4ws.steer_f_deg[idx] < -0.05:
            mode_str = "GEGENPHASE (WENDEKREIS & AGILITÄT)"
        elif result_4ws.steer_r_deg[idx] * result_4ws.steer_f_deg[idx] > 0.05:
            mode_str = "GLEICHPHASE (KRABBEN / ZERO-BETA)"
        else:
            mode_str = "NEUTRAL / GERADEAUS"

        hud_text.set_text(
            f"Zeit: {t:4.2f} s | Geschwindigkeit: {result_4ws.vx[idx]*3.6:5.1f} km/h\n"
            f"Lenkmodus: {mode_str}\n"
            f"┌─ Vorderachswinkel δ_f: {result_4ws.steer_f_deg[idx]:+6.2f}°\n"
            f"└─ Hinterachswinkel δ_r: {result_4ws.steer_r_deg[idx]:+6.2f}°\n"
            f"Fahrdynamik:\n"
            f"  2WS: Schwimmwinkel β = {result_2ws.beta_deg[idx]:+5.2f}° | Gierrate r = {math.degrees(result_2ws.r[idx]):+5.1f}°/s\n"
            f"  4WS: Schwimmwinkel β = {result_4ws.beta_deg[idx]:+5.2f}° | Gierrate r = {math.degrees(result_4ws.r[idx]):+5.1f}°/s"
        )
        return (hud_text,)

    anim = animation.FuncAnimation(fig, update, frames=len(frame_indices), interval=1000 // fps, blit=False)
    writer = animation.PillowWriter(fps=fps)
    anim.save(out_path, writer=writer)
    plt.close(fig)


__all__ = (
    "ScenarioMetrics",
    "ScenarioRunResult",
    "simulate_low_speed_turning_circle",
    "simulate_urban_90deg_cornering",
    "simulate_parallel_parking",
    "simulate_high_speed_double_lane_change",
    "simulate_high_speed_slalom",
    "simulate_autobahn_lane_change",
    "simulate_high_speed_step_steer",
    "simulate_split_mu_braking_4ws",
    "simulate_sine_with_dwell",
    "plot_4ws_six_scenario_dashboard",
    "plot_4ws_nine_scenario_dashboard",
    "animate_4ws_detailed_vehicle",
)

