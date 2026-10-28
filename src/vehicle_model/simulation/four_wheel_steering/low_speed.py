"""Low-speed 4WS scenarios: turning circle, urban 90-degree cornering, parallel parking."""

from __future__ import annotations

import math
from typing import Tuple
import numpy as np
from vehicle_model.controllers.driver_model import DriverModelConfig, NeuromuscularDriverModel
from vehicle_model.core.types import SevenDOFControlInput, SevenDOFState
from vehicle_model.roads.surface import UniformRoad
from vehicle_model.steering.four_wheel_steering import FourWheelSteering, SteeringConfig, SteeringMode
from vehicle_model.vehicle_models.seven_dof import SevenDOFModel

from .results import ScenarioMetrics, ScenarioRunResult


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
