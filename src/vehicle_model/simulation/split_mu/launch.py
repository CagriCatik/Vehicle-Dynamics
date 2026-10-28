"""Split-mu full-throttle launch (traction control and torque vectoring)."""

from __future__ import annotations

from typing import Optional
import numpy as np
from vehicle_model.controllers.driver_model import DriverModelConfig, NeuromuscularDriverModel
from vehicle_model.controllers import ControlTarget, VehicleController
from vehicle_model.core.types import SevenDOFControlInput, SevenDOFState
from vehicle_model.roads.surface import MuSplitRoad
from vehicle_model.simulation.scenario_runner import SimulationHistory
from vehicle_model.vehicle_models.seven_dof import SevenDOFModel

from .results import SplitMuMetrics, SplitMuResult


# =============================================================================
# SCENARIO 2: Split-μ Full-Throttle Launch / Acceleration (0 -> 60 km/h)
# =============================================================================
def simulate_split_mu_launch(
    model: SevenDOFModel,
    controller: Optional[VehicleController] = None,
    v_initial_kph: float = 5.0,
    target_speed_kph: float = 60.0,
    mu_left: float = 1.0,
    mu_right: float = 0.20,
    drive_torque_per_wheel_nm: float = 500.0,
    duration_s: float = 4.5,
    dt: float = 0.005,
    controller_name: str = "Passive",
) -> SplitMuResult:
    """Execute aggressive full-throttle launch on split-mu.

    Physical significance:
    With an open differential, the slick right wheel spins up (excessive slip kappa -> 1.0),
    collapsing drive torque on both wheels to the low-mu limit (poor launch acceleration).
    With active torque vectoring / e-LSD, torque is directed to the high-mu left wheels,
    producing immense forward drive, but generating a drive-yaw moment that tries to pull
    the vehicle toward the right. Active yaw stabilization must prevent road departure!
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

    target_v_mps = target_speed_kph / 3.6
    time_to_target = None

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
    last_telem = None

    for step in range(n_steps):
        t = time_arr[step]

        if state.vx >= target_v_mps and time_to_target is None:
            time_to_target = t

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
                target_vx=target_v_mps,
                target_yaw_rate=0.0,
                driver_steer_f=driver_delta,
                driver_accel_pedal=1.0,
                driver_brake_pedal=0.0,
                mu_estimated=0.6,
            )
            ctrl_out = controller.compute_control(state, target, telemetry=last_telem, dt=dt)
            steer_f = ctrl_out.steer_f if abs(ctrl_out.steer_f) > 1e-4 else driver_delta
            steer_r = ctrl_out.steer_r
            t_drive = ctrl_out.T_drive
            t_brake = ctrl_out.T_brake
        else:
            steer_f = driver_delta
            steer_r = 0.0
            t_drive = np.full(4, drive_torque_per_wheel_nm)
            t_brake = np.zeros(4)

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
    final_psi_deg = float(abs(np.degrees(hist_psi[-1])))
    max_y_dev = float(np.max(np.abs(hist_Y)))
    max_beta = float(np.max(np.abs(np.degrees(history.sideslip_angle))))

    if final_psi_deg > 15.0 or max_y_dev > 2.0:
        verdict = "UNSTABLE"
    elif final_psi_deg > 5.0 or max_y_dev > 0.8:
        verdict = "DEGRADED"
    else:
        verdict = "PASS"

    metrics = SplitMuMetrics(
        scenario_name="Split-μ Full-Throttle Launch (0-60 km/h)",
        controller_name=controller_name,
        initial_speed_kph=v_initial_kph,
        final_speed_kph=float(hist_vx[-1] * 3.6),
        max_yaw_rate_deg_s=max_r_deg,
        final_heading_dev_deg=final_psi_deg,
        max_lateral_dev_m=max_y_dev,
        max_sideslip_deg=max_beta,
        acceleration_time_s=time_to_target,
        stability_status=verdict,
    )

    return SplitMuResult(
        scenario_name="Split-μ Full-Throttle Launch (0-60 km/h)",
        scenario_id="mu_split_launch",
        controller_name=controller_name,
        history=history,
        road=road,
        metrics=metrics,
    )
