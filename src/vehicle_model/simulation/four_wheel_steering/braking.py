"""Split-mu emergency braking with active rear steering."""

from __future__ import annotations

import math
from typing import Any
import numpy as np
from vehicle_model.controllers.driver_model import DriverModelConfig, NeuromuscularDriverModel
from vehicle_model.controllers.framework import ControlTarget
from vehicle_model.core.types import SevenDOFControlInput, SevenDOFState
from vehicle_model.roads.surface import MuSplitRoad
from vehicle_model.vehicle_models.seven_dof import SevenDOFModel

from .results import ScenarioMetrics, ScenarioRunResult


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
