"""Split-mu braking scenarios: straight-line emergency braking and braking in a turn (ISO 7975)."""

from __future__ import annotations

import math
from typing import Optional, Tuple
import numpy as np
from vehicle_model.controllers.driver_model import DriverModelConfig, NeuromuscularDriverModel
from vehicle_model.controllers import ControlTarget, VehicleController
from vehicle_model.core.types import SevenDOFControlInput, SevenDOFState
from vehicle_model.roads.surface import CurvedSplitRoad, MuSplitRoad
from vehicle_model.simulation.scenario_runner import SimulationHistory
from vehicle_model.vehicle_models.seven_dof import SevenDOFModel

from .results import SplitMuMetrics, SplitMuResult


# =============================================================================
# SCENARIO 1: Split-μ Straight-Line Emergency Braking (100 km/h)
# =============================================================================
def simulate_split_mu_braking(
    model: SevenDOFModel,
    controller: Optional[VehicleController] = None,
    v_initial_kph: float = 100.0,
    mu_left: float = 1.0,
    mu_right: float = 0.20,
    brake_torque_nm: float = 1600.0,
    duration_s: float = 3.5,
    dt: float = 0.005,
    controller_name: str = "Passive",
) -> SplitMuResult:
    """Execute high-speed straight-line emergency braking on asymmetrical split-mu.

    Physical significance:
    Massive asymmetric braking force (left high-mu vs right slick ice) generates
    a powerful yaw disturbance moment M_z = (Fx_L - Fx_R) * track/2.
    Unassisted vehicles suffer violent yaw spinout; active chassis control
    stabilizes heading and preserves directional control.
    """
    v0 = v_initial_kph / 3.6
    road = MuSplitRoad(mu_left=mu_left, mu_right=mu_right, split_y=0.0)

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

    # Standard front/rear brake distribution (60% front, 40% rear)
    t_brake_f = brake_torque_nm * 0.60
    t_brake_r = brake_torque_nm * 0.40
    base_t_brake = np.array([t_brake_f, t_brake_f, t_brake_r, t_brake_r])

    driver = NeuromuscularDriverModel(
        config=DriverModelConfig(
            t_near_preview_s=0.35,
            t_far_preview_s=1.35,
            tau_delay_s=0.18,          # 180 ms human cognitive transport lag
            tau_neuromuscular_s=0.10,  # 100 ms neuromuscular arm lag
        ),
        dt=dt,
    )
    driver.set_path_function(lambda x: (0.0, 0.0))

    stopped_step = n_steps
    last_telem = None

    for step in range(n_steps):
        t = time_arr[step]

        if state.vx < 0.20:
            stopped_step = max(1, step)
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
        driver_delta = drv_tel.steer_actual_rad

        if controller is not None:
            target = ControlTarget(
                target_vx=0.0,
                target_yaw_rate=0.0,
                driver_steer_f=driver_delta,
                driver_accel_pedal=0.0,
                driver_brake_pedal=1.0,
                mu_estimated=0.6,
            )
            ctrl_out = controller.compute_control(state, target, telemetry=last_telem, dt=dt)
            steer_f = ctrl_out.steer_f if abs(ctrl_out.steer_f) > 1e-4 else driver_delta
            steer_r = ctrl_out.steer_r
            t_drive = ctrl_out.T_drive
            t_brake = ctrl_out.T_brake if np.max(ctrl_out.T_brake) > 10.0 else base_t_brake
        else:
            steer_f = driver_delta
            steer_r = 0.0
            t_drive = np.zeros(4)
            t_brake = base_t_brake

        # Log state
        hist_vx[step] = state.vx
        hist_vy[step] = state.vy
        hist_r[step] = state.r
        hist_omega[step] = state.omega
        hist_X[step] = state.X
        hist_Y[step] = state.Y
        hist_psi[step] = state.psi
        hist_steer_f[step] = steer_f
        hist_steer_r[step] = steer_r
        hist_T_drive[step] = t_drive
        hist_T_brake[step] = t_brake

        # Dynamics step
        u = SevenDOFControlInput(
            steer_f=steer_f,
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

    if stopped_step < n_steps:
        hist_vx = hist_vx[:stopped_step]
        hist_vy = hist_vy[:stopped_step]
        hist_r = hist_r[:stopped_step]
        hist_omega = hist_omega[:stopped_step]
        hist_X = hist_X[:stopped_step]
        hist_Y = hist_Y[:stopped_step]
        hist_psi = hist_psi[:stopped_step]
        hist_steer_f = hist_steer_f[:stopped_step]
        hist_steer_r = hist_steer_r[:stopped_step]
        hist_T_drive = hist_T_drive[:stopped_step]
        hist_T_brake = hist_T_brake[:stopped_step]
        hist_kappa = hist_kappa[:stopped_step]
        hist_alpha = hist_alpha[:stopped_step]
        hist_Fz = hist_Fz[:stopped_step]
        hist_mu = hist_mu[:stopped_step]
        hist_Fx_w = hist_Fx_w[:stopped_step]
        hist_Fy_w = hist_Fy_w[:stopped_step]
        hist_Fx_b = hist_Fx_b[:stopped_step]
        hist_Fy_b = hist_Fy_b[:stopped_step]
        time_arr = time_arr[:stopped_step]

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

    stop_dist = float(hist_X[-1] - hist_X[0])
    max_r_deg = float(np.max(np.abs(np.degrees(hist_r))))
    final_psi_deg = float(abs(np.degrees(hist_psi[-1])))
    max_y_dev = float(np.max(np.abs(hist_Y)))
    max_beta = float(np.max(np.abs(np.degrees(history.sideslip_angle))))

    # Stability classification:
    if final_psi_deg > 18.0 or max_y_dev > 2.5:
        verdict = "UNSTABLE"
    elif final_psi_deg > 6.0 or max_y_dev > 1.0:
        verdict = "DEGRADED"
    else:
        verdict = "PASS"

    metrics = SplitMuMetrics(
        scenario_name="Split-μ Emergency Braking (100 km/h)",
        controller_name=controller_name,
        initial_speed_kph=v_initial_kph,
        final_speed_kph=float(hist_vx[-1] * 3.6),
        max_yaw_rate_deg_s=max_r_deg,
        final_heading_dev_deg=final_psi_deg,
        max_lateral_dev_m=max_y_dev,
        max_sideslip_deg=max_beta,
        stopping_distance_m=stop_dist,
        stability_status=verdict,
    )

    return SplitMuResult(
        scenario_name="Split-μ Emergency Braking (100 km/h)",
        scenario_id="mu_split_braking",
        controller_name=controller_name,
        history=history,
        road=road,
        metrics=metrics,
    )


# =============================================================================
# SCENARIO 3: Split-μ Braking in a Turn (ISO 7975 Combined Slip)
# =============================================================================
def simulate_split_mu_braking_in_turn(
    model: SevenDOFModel,
    controller: Optional[VehicleController] = None,
    radius_m: float = 80.0,
    v_initial_kph: float = 48.0,
    low_mu_on_outer: bool = True,
    mu_high: float = 1.0,
    mu_low: float = 0.20,
    brake_torque_nm: float = 550.0,
    duration_s: float = 3.5,
    dt: float = 0.005,
    controller_name: str = "Passive",
) -> SplitMuResult:
    """Execute ISO 7975 Braking in a Turn on circular split-mu road.

    Physical significance:
    During cornering, centrifugal force transfers vertical wheel load from
    inner to outer wheels (Delta Fz_lat = m * ay * h / track).
    - If Low-μ is on outer wheels: Heavily-loaded tires lose friction capacity,
      causing severe sudden limit understeer or spinning out!
    - If Low-μ is on inner wheels: Unladen wheels lock early, but outer tires
      sustain the cornering force.
    """
    v0 = v_initial_kph / 3.6
    mu_inner = mu_high if low_mu_on_outer else mu_low
    mu_outer = mu_low if low_mu_on_outer else mu_high
    case_label = "Low-μ Outer Wheels (Critical)" if low_mu_on_outer else "Low-μ Inner Wheels"

    # Road center at (0, R) so vehicle starts at (0, 0) heading +X
    road = CurvedSplitRoad(
        center_x=0.0,
        center_y=radius_m,
        radius_split_m=radius_m,
        mu_inner=mu_inner,
        mu_outer=mu_outer,
    )

    n_steps = int(duration_s / dt)
    time_arr = np.linspace(0.0, duration_s, n_steps)

    params = model.params
    init_yaw_rate = v0 / radius_m
    wheelbase = params.lf + params.lr
    ackermann_steer = math.atan(wheelbase / radius_m)

    state = SevenDOFState(
        vx=v0,
        vy=0.0,
        r=init_yaw_rate,
        omega=np.full(4, v0 / params.Reff),
        X=0.0,
        Y=0.0,
        psi=0.0,
    )

    def turn_path(x: float) -> Tuple[float, float]:
        clipped_x = min(x, radius_m - 1e-3)
        y = radius_m - math.sqrt(max(0.0, radius_m**2 - clipped_x**2))
        psi = math.asin(clipped_x / radius_m)
        return y, psi

    driver = NeuromuscularDriverModel(
        config=DriverModelConfig(
            t_near_preview_s=0.35,
            t_far_preview_s=1.35,
            tau_delay_s=0.18,          # 180 ms human cognitive transport lag
            tau_neuromuscular_s=0.10,  # 100 ms neuromuscular arm lag
        ),
        path_function=turn_path,
        dt=dt,
    )
    last_telem = None

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

    t_brake_f = brake_torque_nm * 0.60
    t_brake_r = brake_torque_nm * 0.40
    base_t_brake = np.array([t_brake_f, t_brake_f, t_brake_r, t_brake_r])

    stopped_step = n_steps
    for step in range(n_steps):
        t = time_arr[step]

        if state.vx < 0.20:
            stopped_step = max(1, step)
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
        driver_delta = drv_tel.steer_actual_rad
        target_r = state.vx / radius_m

        if controller is not None:
            target = ControlTarget(
                target_vx=0.0,
                target_yaw_rate=target_r,
                driver_steer_f=driver_delta,
                driver_accel_pedal=0.0,
                driver_brake_pedal=0.8,
                mu_estimated=0.6,
            )
            ctrl_out = controller.compute_control(state, target, telemetry=last_telem, dt=dt)
            steer_f = ctrl_out.steer_f if abs(ctrl_out.steer_f) > 1e-4 else driver_delta
            steer_r = ctrl_out.steer_r
            t_drive = ctrl_out.T_drive
            t_brake = ctrl_out.T_brake if np.max(ctrl_out.T_brake) > 10.0 else base_t_brake
        else:
            steer_f = driver_delta
            steer_r = 0.0
            t_drive = np.zeros(4)
            t_brake = base_t_brake

        # Log
        hist_vx[step] = state.vx
        hist_vy[step] = state.vy
        hist_r[step] = state.r
        hist_omega[step] = state.omega
        hist_X[step] = state.X
        hist_Y[step] = state.Y
        hist_psi[step] = state.psi
        hist_steer_f[step] = steer_f
        hist_steer_r[step] = steer_r
        hist_T_drive[step] = t_drive
        hist_T_brake[step] = t_brake

        u = SevenDOFControlInput(
            steer_f=steer_f,
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

    if stopped_step < n_steps:
        hist_vx = hist_vx[:stopped_step]
        hist_vy = hist_vy[:stopped_step]
        hist_r = hist_r[:stopped_step]
        hist_omega = hist_omega[:stopped_step]
        hist_X = hist_X[:stopped_step]
        hist_Y = hist_Y[:stopped_step]
        hist_psi = hist_psi[:stopped_step]
        hist_steer_f = hist_steer_f[:stopped_step]
        hist_steer_r = hist_steer_r[:stopped_step]
        hist_T_drive = hist_T_drive[:stopped_step]
        hist_T_brake = hist_T_brake[:stopped_step]
        hist_kappa = hist_kappa[:stopped_step]
        hist_alpha = hist_alpha[:stopped_step]
        hist_Fz = hist_Fz[:stopped_step]
        hist_mu = hist_mu[:stopped_step]
        hist_Fx_w = hist_Fx_w[:stopped_step]
        hist_Fy_w = hist_Fy_w[:stopped_step]
        hist_Fx_b = hist_Fx_b[:stopped_step]
        hist_Fy_b = hist_Fy_b[:stopped_step]
        time_arr = time_arr[:stopped_step]

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

    rad_dist = np.hypot(hist_X, hist_Y - radius_m)
    max_path_dev = float(np.max(np.abs(rad_dist - radius_m)))
    max_r_deg = float(np.max(np.abs(np.degrees(hist_r))))
    max_beta = float(np.max(np.abs(np.degrees(history.sideslip_angle))))

    if max_beta > 12.0 or max_path_dev > 3.0:
        verdict = "UNSTABLE"
    elif max_beta > 7.0 or max_path_dev > 1.5:
        verdict = "DEGRADED"
    else:
        verdict = "PASS"

    scenario_label = f"Split-μ Braking in Turn ({case_label})"
    metrics = SplitMuMetrics(
        scenario_name=scenario_label,
        controller_name=controller_name,
        initial_speed_kph=v_initial_kph,
        final_speed_kph=float(hist_vx[-1] * 3.6),
        max_yaw_rate_deg_s=max_r_deg,
        final_heading_dev_deg=float(abs(np.degrees(hist_psi[-1]))),
        max_lateral_dev_m=max_path_dev,
        max_sideslip_deg=max_beta,
        stopping_distance_m=float(hist_X[-1] - hist_X[0]),
        stability_status=verdict,
    )

    return SplitMuResult(
        scenario_name=scenario_label,
        scenario_id="mu_split_braking_in_turn",
        controller_name=controller_name,
        history=history,
        road=road,
        metrics=metrics,
    )
