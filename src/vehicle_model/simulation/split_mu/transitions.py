"""Split-mu scenarios with changing friction along the path: diagonal bridge transition and evasive double lane change (ISO 3888-2)."""

from __future__ import annotations

import math
from typing import Optional, Tuple
import numpy as np
from vehicle_model.controllers.driver_model import DriverModelConfig, NeuromuscularDriverModel
from vehicle_model.controllers import ControlTarget, VehicleController
from vehicle_model.core.types import SevenDOFControlInput, SevenDOFState
from vehicle_model.roads.surface import DiagonalSplitRoad, MuSplitRoad
from vehicle_model.simulation.scenario_runner import SimulationHistory
from vehicle_model.vehicle_models.seven_dof import SevenDOFModel

from .results import SplitMuMetrics, SplitMuResult


# =============================================================================
# SCENARIO 4: Diagonal Split-μ Bridge Transition (Staggered Wheel Entry)
# =============================================================================
def simulate_split_mu_diagonal_transition(
    model: SevenDOFModel,
    controller: Optional[VehicleController] = None,
    v_initial_kph: float = 80.0,
    angle_deg: float = 35.0,
    transition_x: float = 25.0,
    mu_high: float = 1.0,
    mu_low: float = 0.20,
    duration_s: float = 3.5,
    dt: float = 0.005,
    controller_name: str = "Passive",
) -> SplitMuResult:
    """Execute diagonal split-mu road transition at highway cruising speed.

    Physical significance:
    The vehicle encounters an angled friction transition boundary (e.g. icy expansion joint).
    Front-right wheel hits ice first, then rear-right wheel.
    The staggered tire entry produces sharp transient yaw impulses.
    A realistic closed-loop Neuromuscular Preview Driver model attempts to keep
    the center line Y = 0. We measure driver workload, steering reversals, and yaw excursion.
    """
    v0 = v_initial_kph / 3.6
    road = DiagonalSplitRoad(
        x0=transition_x,
        y0=0.0,
        theta_deg=angle_deg,
        mu_high=mu_high,
        mu_low=mu_low,
    )

    n_steps = int(duration_s / dt)
    time_arr = np.linspace(0.0, duration_s, n_steps)

    params = model.params
    state = SevenDOFState(
        vx=v0,
        vy=0.0,
        r=0.0,
        omega=np.full(4, v0 / params.Reff),
        X=0.0,
        Y=0.0,
        psi=0.0,
    )

    driver = NeuromuscularDriverModel(
        config=DriverModelConfig(
            t_near_preview_s=0.35,
            t_far_preview_s=1.35,
            tau_delay_s=0.18,          # 180 ms human cognitive transport lag
            tau_neuromuscular_s=0.10,  # 100 ms neuromuscular arm lag
            enable_speed_adaptive_gains=True,
            enable_cerebellar_prediction=True,
        ),
        dt=dt,
    )
    driver.set_path_function(lambda x: (0.0, 0.0))

    if controller is not None and hasattr(controller, "reset"):
        controller.reset()

    # Data loggers
    hist_vx = np.zeros(n_steps)
    hist_vy = np.zeros(n_steps)
    hist_r = np.zeros(n_steps)
    hist_omega = np.zeros((n_steps, 4))
    hist_X = np.zeros(n_steps)
    hist_Y = np.zeros(n_steps)
    hist_psi = np.zeros(n_steps)
    hist_steer_f = np.zeros(n_steps)
    hist_steer_r = np.zeros(n_steps)
    hist_T_drive = np.zeros((n_steps, 4))
    hist_T_brake = np.zeros((n_steps, 4))
    hist_kappa = np.zeros((n_steps, 4))
    hist_alpha = np.zeros((n_steps, 4))
    hist_Fz = np.zeros((n_steps, 4))
    hist_mu = np.zeros((n_steps, 4))
    hist_Fx_w = np.zeros((n_steps, 4))
    hist_Fy_w = np.zeros((n_steps, 4))
    hist_Fx_b = np.zeros((n_steps, 4))
    hist_Fy_b = np.zeros((n_steps, 4))

    c_rr = getattr(params, "c_rr", 0.015)
    f_resist = 0.5 * 1.225 * params.Cd * params.Af * (v0**2) + params.mass * 9.81 * c_rr
    cruise_torque_wheel = (f_resist * params.Reff) / 4.0
    last_telem = None

    for step in range(n_steps):
        t = time_arr[step]

        drv_tel = driver.step(
            x=state.X,
            y=state.Y,
            psi=state.psi,
            vx=state.vx,
            vy=state.vy,
            r=state.r,
            dt=dt,
        )
        delta_driver = drv_tel.steer_actual_rad

        if controller is not None:
            target = ControlTarget(
                target_vx=v0,
                target_yaw_rate=0.0,
                driver_steer_f=delta_driver,
                driver_accel_pedal=0.15,
                driver_brake_pedal=0.0,
                mu_estimated=0.6,
            )
            ctrl_out = controller.compute_control(state, target, telemetry=last_telem, dt=dt)
            effective_steer_f = ctrl_out.steer_f if abs(ctrl_out.steer_f) > 1e-4 else delta_driver
            steer_r = ctrl_out.steer_r
            t_drive = ctrl_out.T_drive
            t_brake = ctrl_out.T_brake
        else:
            effective_steer_f = delta_driver
            steer_r = 0.0
            t_drive = np.full(4, cruise_torque_wheel)
            t_brake = np.zeros(4)

        # Log
        hist_vx[step] = state.vx
        hist_vy[step] = state.vy
        hist_r[step] = state.r
        hist_omega[step] = state.omega
        hist_X[step] = state.X
        hist_Y[step] = state.Y
        hist_psi[step] = state.psi
        hist_steer_f[step] = effective_steer_f
        hist_steer_r[step] = steer_r
        hist_T_drive[step] = t_drive
        hist_T_brake[step] = t_brake

        u = SevenDOFControlInput(
            steer_f=effective_steer_f,
            steer_r=steer_r,
            T_drive=t_drive,
            T_brake=t_brake,
        ).clamped(params)
        state, telem = model.step(state, u, dt=dt, road=road)
        last_telem = telem

        hist_kappa[step] = telem.kappa
        hist_alpha[step] = telem.alpha
        hist_Fz[step] = telem.Fz
        hist_mu[step] = telem.mu
        hist_Fx_w[step] = telem.Fx_wheel
        hist_Fy_w[step] = telem.Fy_wheel
        hist_Fx_b[step] = telem.Fx_body
        hist_Fy_b[step] = telem.Fy_body

    history = SimulationHistory(
        time=time_arr,
        vx=hist_vx,
        vy=hist_vy,
        r=hist_r,
        omega=hist_omega,
        X=hist_X,
        Y=hist_Y,
        psi=hist_psi,
        steer_f=hist_steer_f,
        steer_r=hist_steer_r,
        T_drive=hist_T_drive,
        T_brake=hist_T_brake,
        kappa=hist_kappa,
        alpha=hist_alpha,
        Fz=hist_Fz,
        mu=hist_mu,
        Fx_wheel=hist_Fx_w,
        Fy_wheel=hist_Fy_w,
        Fx_body=hist_Fx_b,
        Fy_body=hist_Fy_b,
    )

    max_r_deg = float(np.max(np.abs(np.degrees(hist_r))))
    max_y_dev = float(np.max(np.abs(hist_Y)))
    max_beta = float(np.max(np.abs(np.degrees(history.sideslip_angle))))
    final_psi = float(abs(np.degrees(hist_psi[-1])))
    max_steer = float(np.max(np.abs(np.degrees(hist_steer_f))))

    if max_y_dev > 1.8 or max_r_deg > 20.0:
        verdict = "UNSTABLE"
    elif max_y_dev > 0.8 or max_r_deg > 10.0:
        verdict = "DEGRADED"
    else:
        verdict = "PASS"

    metrics = SplitMuMetrics(
        scenario_name="Diagonal Split-μ Bridge Transition (80 km/h)",
        controller_name=controller_name,
        initial_speed_kph=v_initial_kph,
        final_speed_kph=float(hist_vx[-1] * 3.6),
        max_yaw_rate_deg_s=max_r_deg,
        final_heading_dev_deg=final_psi,
        max_lateral_dev_m=max_y_dev,
        max_sideslip_deg=max_beta,
        max_driver_steer_deg=max_steer,
        stability_status=verdict,
    )

    return SplitMuResult(
        scenario_name="Diagonal Split-μ Bridge Transition (80 km/h)",
        scenario_id="mu_split_diagonal_transition",
        controller_name=controller_name,
        history=history,
        road=road,
        metrics=metrics,
    )


# =============================================================================
# SCENARIO 5: Split-μ Severe Double Lane Change (ISO 3888-2 Obstacle Avoidance)
# =============================================================================
def simulate_split_mu_evasive_dlc(
    model: SevenDOFModel,
    controller: Optional[VehicleController] = None,
    v_initial_kph: float = 75.0,
    mu_lane1: float = 1.0,
    mu_lane2: float = 0.35,
    split_y: float = 1.6,
    duration_s: float = 4.5,
    dt: float = 0.005,
    controller_name: str = "Passive",
) -> SplitMuResult:
    """Execute ISO 3888-2 Severe Double Lane Change across a split-mu surface.

    Physical significance:
    The vehicle initiates obstacle evasion on high-mu asphalt (Lane 1, Y ~ 0),
    enters the evasion lane (Lane 2, Y ~ 3.5m) which has low friction (wet/ice),
    and must execute steering reversal to return to Lane 1.
    Tests transient limit handling under rapid friction jump.
    """
    v0 = v_initial_kph / 3.6
    road = MuSplitRoad(mu_left=mu_lane2, mu_right=mu_lane1, split_y=split_y)

    n_steps = int(duration_s / dt)
    time_arr = np.linspace(0.0, duration_s, n_steps)

    params = model.params
    state = SevenDOFState(
        vx=v0,
        vy=0.0,
        r=0.0,
        omega=np.full(4, v0 / params.Reff),
        X=0.0,
        Y=0.0,
        psi=0.0,
    )

    def dlc_target_path(x: float) -> Tuple[float, float]:
        if x < 15.0:
            return 0.0, 0.0
        elif x < 45.0:
            s = (x - 15.0) / 30.0
            y = 3.5 * (10.0 * (s**3) - 15.0 * (s**4) + 6.0 * (s**5))
            dy = 3.5 * (30.0 * (s**2) - 60.0 * (s**3) + 30.0 * (s**4)) / 30.0
            return y, math.atan(dy)
        elif x < 65.0:
            return 3.5, 0.0
        elif x < 95.0:
            s = (x - 65.0) / 30.0
            y = 3.5 - 3.5 * (10.0 * (s**3) - 15.0 * (s**4) + 6.0 * (s**5))
            dy = -3.5 * (30.0 * (s**2) - 60.0 * (s**3) + 30.0 * (s**4)) / 30.0
            return y, math.atan(dy)
        else:
            return 0.0, 0.0

    driver = NeuromuscularDriverModel(
        config=DriverModelConfig(
            t_near_preview_s=0.35,
            t_far_preview_s=1.35,
            tau_delay_s=0.18,
            tau_neuromuscular_s=0.10,
            enable_speed_adaptive_gains=True,
            enable_cerebellar_prediction=True,
        ),
        dt=dt,
    )
    driver.set_path_function(dlc_target_path)

    if controller is not None and hasattr(controller, "reset"):
        controller.reset()

    # Data loggers
    hist_vx = np.zeros(n_steps)
    hist_vy = np.zeros(n_steps)
    hist_r = np.zeros(n_steps)
    hist_omega = np.zeros((n_steps, 4))
    hist_X = np.zeros(n_steps)
    hist_Y = np.zeros(n_steps)
    hist_psi = np.zeros(n_steps)
    hist_steer_f = np.zeros(n_steps)
    hist_steer_r = np.zeros(n_steps)
    hist_T_drive = np.zeros((n_steps, 4))
    hist_T_brake = np.zeros((n_steps, 4))
    hist_kappa = np.zeros((n_steps, 4))
    hist_alpha = np.zeros((n_steps, 4))
    hist_Fz = np.zeros((n_steps, 4))
    hist_mu = np.zeros((n_steps, 4))
    hist_Fx_w = np.zeros((n_steps, 4))
    hist_Fy_w = np.zeros((n_steps, 4))
    hist_Fx_b = np.zeros((n_steps, 4))
    hist_Fy_b = np.zeros((n_steps, 4))

    c_rr = getattr(params, "c_rr", 0.015)
    f_resist = 0.5 * 1.225 * params.Cd * params.Af * (v0**2) + params.mass * 9.81 * c_rr
    cruise_torque_wheel = (f_resist * params.Reff) / 4.0
    last_telem = None

    for step in range(n_steps):
        t = time_arr[step]

        drv_tel = driver.step(
            x=state.X,
            y=state.Y,
            psi=state.psi,
            vx=state.vx,
            vy=state.vy,
            r=state.r,
            dt=dt,
        )
        delta_driver = drv_tel.steer_actual_rad
        target_y, target_heading = dlc_target_path(state.X)

        if controller is not None:
            target = ControlTarget(
                target_vx=v0,
                target_yaw_rate=state.vx * math.sin(target_heading) / max(1.0, params.lf + params.lr),
                driver_steer_f=delta_driver,
                driver_accel_pedal=0.15,
                driver_brake_pedal=0.0,
                mu_estimated=0.6,
            )
            ctrl_out = controller.compute_control(state, target, telemetry=last_telem, dt=dt)
            effective_steer_f = ctrl_out.steer_f if abs(ctrl_out.steer_f) > 1e-4 else delta_driver
            steer_r = ctrl_out.steer_r
            t_drive = ctrl_out.T_drive
            t_brake = ctrl_out.T_brake
        else:
            effective_steer_f = delta_driver
            steer_r = 0.0
            t_drive = np.full(4, cruise_torque_wheel)
            t_brake = np.zeros(4)

        hist_vx[step] = state.vx
        hist_vy[step] = state.vy
        hist_r[step] = state.r
        hist_omega[step] = state.omega
        hist_X[step] = state.X
        hist_Y[step] = state.Y
        hist_psi[step] = state.psi
        hist_steer_f[step] = effective_steer_f
        hist_steer_r[step] = steer_r
        hist_T_drive[step] = t_drive
        hist_T_brake[step] = t_brake

        u = SevenDOFControlInput(
            steer_f=effective_steer_f,
            steer_r=steer_r,
            T_drive=t_drive,
            T_brake=t_brake,
        ).clamped(params)
        state, telem = model.step(state, u, dt=dt, road=road)
        last_telem = telem

        hist_kappa[step] = telem.kappa
        hist_alpha[step] = telem.alpha
        hist_Fz[step] = telem.Fz
        hist_mu[step] = telem.mu
        hist_Fx_w[step] = telem.Fx_wheel
        hist_Fy_w[step] = telem.Fy_wheel
        hist_Fx_b[step] = telem.Fx_body
        hist_Fy_b[step] = telem.Fy_body

    history = SimulationHistory(
        time=time_arr,
        vx=hist_vx,
        vy=hist_vy,
        r=hist_r,
        omega=hist_omega,
        X=hist_X,
        Y=hist_Y,
        psi=hist_psi,
        steer_f=hist_steer_f,
        steer_r=hist_steer_r,
        T_drive=hist_T_drive,
        T_brake=hist_T_brake,
        kappa=hist_kappa,
        alpha=hist_alpha,
        Fz=hist_Fz,
        mu=hist_mu,
        Fx_wheel=hist_Fx_w,
        Fy_wheel=hist_Fy_w,
        Fx_body=hist_Fx_b,
        Fy_body=hist_Fy_b,
    )

    max_r_deg = float(np.max(np.abs(np.degrees(hist_r))))
    max_beta = float(np.max(np.abs(np.degrees(history.sideslip_angle))))
    final_psi = float(abs(np.degrees(hist_psi[-1])))
    max_steer = float(np.max(np.abs(np.degrees(hist_steer_f))))

    target_ys = np.array([dlc_target_path(x)[0] for x in hist_X])
    max_track_error = float(np.max(np.abs(hist_Y - target_ys)))

    if max_beta > 10.0 or max_track_error > 2.5:
        verdict = "UNSTABLE"
    elif max_beta > 6.0 or max_track_error > 1.2:
        verdict = "DEGRADED"
    else:
        verdict = "PASS"

    metrics = SplitMuMetrics(
        scenario_name="Split-μ Severe Double Lane Change (ISO 3888-2)",
        controller_name=controller_name,
        initial_speed_kph=v_initial_kph,
        final_speed_kph=float(hist_vx[-1] * 3.6),
        max_yaw_rate_deg_s=max_r_deg,
        final_heading_dev_deg=final_psi,
        max_lateral_dev_m=max_track_error,
        max_sideslip_deg=max_beta,
        max_driver_steer_deg=max_steer,
        stability_status=verdict,
    )

    return SplitMuResult(
        scenario_name="Split-μ Severe Double Lane Change (ISO 3888-2)",
        scenario_id="mu_split_evasive_dlc",
        controller_name=controller_name,
        history=history,
        road=road,
        metrics=metrics,
    )
