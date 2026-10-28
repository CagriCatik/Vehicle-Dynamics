"""High-speed 4WS scenarios: double lane change, slalom, Autobahn lane change, step steer, sine with dwell."""

from __future__ import annotations

import math
from typing import Tuple, Any
import numpy as np
from vehicle_model.controllers.driver_model import DriverModelConfig, NeuromuscularDriverModel
from vehicle_model.controllers.framework import ControlTarget
from vehicle_model.core.types import SevenDOFControlInput, SevenDOFState
from vehicle_model.roads.surface import UniformRoad
from vehicle_model.vehicle_models.seven_dof import SevenDOFModel

from .results import ScenarioMetrics, ScenarioRunResult


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
