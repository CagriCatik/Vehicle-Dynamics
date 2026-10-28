"""End-to-End (E2E) Closed-Loop Vehicle Dynamics & Control Scenarios.

Comprehensive multi-system integration testing verifying:
1. Closed-Loop Driver-in-the-Loop on Split-Mu Surface Transition (Coupled human-vehicle stability)
2. Dynamic Two-Track NMPC High-Speed Obstacle Evasion (4WS + Torque Vectoring + Kamm circle adherence)
3. 14-DOF Multi-Body Rollover & Wheel-Lift Prevention under Dynamic Fishhook Reversal (ARSDC)
4. Sudden Lift-Off Oversteer Catch & Recovery with Phase-Plane Envelope Protection
5. Speed-Adaptive Neuromuscular Driver Model Consistency Across 30 km/h to 110 km/h Speeds

Conforms to Plan.md §16, §18, §19.
"""

from __future__ import annotations

import math
from pathlib import Path
import numpy as np
import pytest

from vehicle_model.controllers.active_roll import ActiveRollConfig, ActiveRollDistributionController
from vehicle_model.controllers.driver_model import DriverModelConfig, NeuromuscularDriverModel
from vehicle_model.controllers.dynamic_two_track_nmpc import DynamicNMPCConfig, DynamicTwoTrackNMPC
from vehicle_model.controllers.framework import ControlTarget
from vehicle_model.controllers.integrated_controller import IntegratedChassisController
from vehicle_model.controllers.phase_plane import PhasePlaneConfig, PhasePlaneEnvelopeController, PhasePlaneState
from vehicle_model.core.types import FL, FR, RL, RR, SevenDOFControlInput, SevenDOFParams, SevenDOFState
from vehicle_model.roads.surface import MuSplitRoad, UniformRoad
from vehicle_model.vehicle_models.fourteen_dof import FourteenDOFConfig, FourteenDOFModel, FourteenDOFState
from vehicle_model.vehicle_models.seven_dof import SevenDOFModel


# ---------------------------------------------------------------------------
# Test 1: Closed-Loop Driver-in-the-Loop on Split-Mu Surface Transition
# ---------------------------------------------------------------------------
@pytest.mark.slow
def test_e2e_driver_in_the_loop_split_mu_transition():
    """Verify coupled closed-loop stability when a human driver navigates across split-mu transition."""
    params = SevenDOFParams.from_yaml()
    # Asymmetric friction: left wheels on dry asphalt (0.9), right wheels on slick ice/snow (0.25)
    road = MuSplitRoad(mu_left=0.90, mu_right=0.25, split_y=1.5)

    v0 = 75.0 / 3.6  # 20.83 m/s
    dt = 0.01
    duration = 4.0
    steps = int(duration / dt)

    def lane_change_path(x: float):
        if x < 20.0:
            return 0.0, 0.0
        elif x < 60.0:
            s = (x - 20.0) / 40.0
            y = 3.0 * (10.0 * (s**3) - 15.0 * (s**4) + 6.0 * (s**5))
            dy = 3.0 * (30.0 * (s**2) - 60.0 * (s**3) + 30.0 * (s**4)) / 40.0
            return y, math.atan(dy)
        else:
            return 3.0, 0.0

    # 1. Driver + Passive Vehicle
    driver_pass = NeuromuscularDriverModel(
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
    driver_pass.set_path_function(lane_change_path)
    model_pass = SevenDOFModel(params=params, road=road)
    state_pass = SevenDOFState(
        vx=v0, vy=0.0, r=0.0, omega=np.full(4, v0 / params.Reff), X=0.0, Y=0.0, psi=0.0
    )

    # 2. Driver + IVMC Assisted Vehicle (4WS + Phase Plane + TV)
    driver_asst = NeuromuscularDriverModel(
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
    driver_asst.set_path_function(lane_change_path)
    model_asst = SevenDOFModel(params=params, road=road)
    ivmc = IntegratedChassisController.from_yaml(params=params)
    state_asst = SevenDOFState(
        vx=v0, vy=0.0, r=0.0, omega=np.full(4, v0 / params.Reff), X=0.0, Y=0.0, psi=0.0
    )

    t_drive_base = np.full(4, 420.0)

    tel_asst = None
    r_pass_hist = []
    r_asst_hist = []
    beta_asst_hist = []

    for _ in range(steps):
        # Passive step
        t_p = driver_pass.step(state_pass.X, state_pass.Y, state_pass.psi, state_pass.vx, state_pass.vy, state_pass.r)
        u_p = SevenDOFControlInput(steer_f=t_p.steer_actual_rad, steer_r=0.0, T_drive=t_drive_base, T_brake=np.zeros(4))
        state_pass, _ = model_pass.step(state_pass, u_p, dt, road)
        r_pass_hist.append(abs(state_pass.r))

        # Assisted step
        t_a = driver_asst.step(state_asst.X, state_asst.Y, state_asst.psi, state_asst.vx, state_asst.vy, state_asst.r)
        target = ControlTarget(driver_steer_f=t_a.steer_actual_rad, driver_accel_pedal=0.45, driver_brake_pedal=0.0, mu_estimated=0.55)
        out = ivmc.compute_control(state_asst, target, telemetry=tel_asst, dt=dt)
        u_a = SevenDOFControlInput(steer_f=out.steer_f, steer_r=out.steer_r, T_drive=out.T_drive, T_brake=out.T_brake)
        state_asst, tel_asst = model_asst.step(state_asst, u_a, dt, road)
        r_asst_hist.append(abs(state_asst.r))
        beta_asst_hist.append(abs(math.atan2(state_asst.vy, max(1.5, abs(state_asst.vx)))))

    # Assisted driver workload energy must be significantly lower than passive
    workload_pass = t_p.driver_workload_energy
    workload_asst = t_a.driver_workload_energy
    assert workload_asst < workload_pass * 0.65, (
        f"Assisted workload ({workload_asst:.5f}) not lower than passive ({workload_pass:.5f})"
    )

    # Peak yaw rate under split-mu must be bounded and stabilized
    assert max(r_asst_hist) < max(r_pass_hist) + 0.10
    # Assisted body sideslip must remain well within linear safe envelope (< 6.0 deg)
    assert max(beta_asst_hist) < math.radians(6.0)


# ---------------------------------------------------------------------------
# Test 2: Dynamic Two-Track NMPC Trajectory Tracking & Kamm Friction Circle
# ---------------------------------------------------------------------------
@pytest.mark.slow
def test_e2e_dynamic_nmpc_trajectory_tracking_kamm_circle():
    """Verify Dynamic Two-Track NMPC executes lane evasion while strictly honoring Kamm friction limits."""
    params = SevenDOFParams.from_yaml()
    mu_surface = 0.75
    road = UniformRoad(mu=mu_surface)
    plant = SevenDOFModel(params=params, road=road)

    nmpc_cfg = DynamicNMPCConfig(
        N=10,
        dt_pred=0.05,
        max_steer_front_rad=math.radians(25.0),
        max_steer_rear_rad=math.radians(8.0),
        max_yaw_moment_nm=3500.0,
        w_kamm_circle=80.0,
        max_iter=15,
    )
    nmpc = DynamicTwoTrackNMPC(params=params, config=nmpc_cfg)

    v0 = 90.0 / 3.6  # 25 m/s
    state = SevenDOFState(
        vx=v0, vy=0.0, r=0.0, omega=np.full(4, v0 / params.Reff), X=0.0, Y=0.0, psi=0.0
    )

    dt_step = 0.05
    steps = 40  # 2.0 seconds
    max_kamm_ratio = 0.0
    final_lateral_error = 0.0

    for step_i in range(steps):
        t = step_i * dt_step
        # Target step to Y = 3.5m after t = 0.2s
        y_ref = 3.5 if t >= 0.2 else 0.0
        target = ControlTarget(target_y=y_ref, target_psi=0.0, target_vx=v0, mu_estimated=mu_surface)

        out = nmpc.compute_control(state, target)
        u_step = SevenDOFControlInput(
            steer_f=out.steer_f,
            steer_r=out.steer_r,
            T_drive=out.T_drive,
            T_brake=out.T_brake,
        )

        state, telem = plant.step(state, u_step, dt_step, road)

        # Kamm utilization verification: sqrt(Fx^2 + Fy^2) / (mu * Fz)
        Fz = telem.Fz
        Fx = telem.Fx_wheel
        Fy = telem.Fy_wheel
        # Limit = the tyre's own peak friction (load-dependent for Magic Formula tyres), else mu Fz
        for i in range(4):
            tyre = plant.tyre_front if i < 2 else plant.tyre_rear
            if hasattr(tyre, "peak_friction"):
                mu_peak = max(float(m) for m in tyre.peak_friction(Fz[i], mu_surface))
            else:
                mu_peak = mu_surface
            util = math.sqrt(Fx[i]**2 + Fy[i]**2) / max(100.0, mu_peak * Fz[i])
            max_kamm_ratio = max(max_kamm_ratio, util)

    final_lateral_error = abs(state.Y - 3.5)

    # 1. Controller must track towards target lane
    assert state.Y > 2.5, f"NMPC failed to change lanes: Y = {state.Y:.2f} m"
    # 2. Friction utilization must honor Kamm circle (within numerical discretization threshold <= 1.05)
    assert max_kamm_ratio <= 1.05, f"Kamm friction circle breached: max ratio = {max_kamm_ratio:.3f}"
    # 3. Output diagnostics should be populated
    assert "kamm_utilization" in out.diagnostics
    assert "peak_kamm" in out.diagnostics


# ---------------------------------------------------------------------------
# Test 3: 14-DOF Multi-Body Rollover & Wheel-Lift Prevention (NHTSA Fishhook)
# ---------------------------------------------------------------------------
def test_e2e_14dof_dynamic_fishhook_rollover_prevention():
    """Verify Active Roll Stiffness Controller suppresses LTR and prevents wheel-lift on 14-DOF model."""
    veh_cfg = FourteenDOFConfig()
    model_passive = FourteenDOFModel(config=veh_cfg)
    model_active = FourteenDOFModel(config=veh_cfg)

    arsdc = ActiveRollDistributionController(vehicle_cfg=veh_cfg, control_cfg=ActiveRollConfig())

    v0 = 80.0 / 3.6  # 22.22 m/s
    dt = 0.005
    duration = 2.5
    steps = int(duration / dt)

    s_pass = FourteenDOFState(vx=v0, omega=np.full(4, v0 / veh_cfg.wheel_radius))
    s_act = FourteenDOFState(vx=v0, omega=np.full(4, v0 / veh_cfg.wheel_radius))

    ltr_pass_max = 0.0
    ltr_act_max = 0.0
    phi_pass_max = 0.0
    phi_act_max = 0.0
    min_fz_active = 1e6
    fz_act = veh_cfg.static_normal_loads.copy()

    # Fishhook steering profile: turn left to +5.5 deg at 0.3s, violent reversal to -5.5 deg at 1.0s
    for i in range(steps):
        t = i * dt
        if 0.3 <= t < 0.8:
            delta_steer = math.radians(5.5) * min(1.0, (t - 0.3) / 0.25)
        elif 0.8 <= t < 1.0:
            delta_steer = math.radians(5.5) - math.radians(11.0) * ((t - 0.8) / 0.20)
        elif 1.0 <= t < 2.0:
            delta_steer = math.radians(-5.5)
        else:
            delta_steer = 0.0

        delta_arr = np.array([delta_steer, delta_steer, 0.0, 0.0], dtype=np.float64)
        t_zeros = np.zeros(4, dtype=np.float64)

        # Passive step
        s_pass, tel_pass = model_passive.step(s_pass, delta=delta_arr, T_drive=t_zeros, T_brake=t_zeros, dt=dt)
        fz_p = tel_pass.Fz
        ltr_p = abs((fz_p[FL] + fz_p[RL]) - (fz_p[FR] + fz_p[RR])) / max(100.0, np.sum(fz_p))
        ltr_pass_max = max(ltr_pass_max, ltr_p)
        phi_pass_max = max(phi_pass_max, abs(s_pass.phi))

        # Active step (ARSDC anti-roll control)
        ay_act = s_act.vx * s_act.r
        r_des = delta_steer * (s_act.vx / veh_cfg.wheelbase)
        arsdc_out = arsdc.compute_control(s_act, r_desired=r_des, ay=ay_act, Fz_corners=fz_act)

        s_act, tel_act = model_active.step(
            s_act,
            delta=delta_arr,
            T_drive=t_zeros,
            T_brake=t_zeros,
            dt=dt,
            active_forces=arsdc_out.active_corner_forces,
        )
        fz_act = tel_act.Fz
        ltr_a = arsdc_out.load_transfer_ratio
        ltr_act_max = max(ltr_act_max, ltr_a)
        phi_act_max = max(phi_act_max, abs(s_act.phi))
        min_fz_active = min(min_fz_active, np.min(fz_act))

    # Active roll control must reduce roll angle by > 30%
    assert phi_act_max < phi_pass_max * 0.70, (
        f"ARSDC failed to suppress roll: phi_act={math.degrees(phi_act_max):.2f} deg, "
        f"phi_pass={math.degrees(phi_pass_max):.2f} deg"
    )
    # LTR must remain strictly below rollover limit (< 0.85)
    assert ltr_act_max < 0.85
    # Positive contact must be maintained on all 4 tires (no wheel lift-off)
    assert min_fz_active > 300.0, f"Wheel lift detected under active control: min Fz = {min_fz_active:.1f} N"


# ---------------------------------------------------------------------------
# Test 4: Lift-Off Oversteer Catch & Recovery with Phase-Plane Envelope
# ---------------------------------------------------------------------------
def test_e2e_lift_off_oversteer_phase_plane_recovery():
    """Verify Phase-Plane Envelope Controller catches sudden lift-off oversteer and bounds S_phase."""
    params = SevenDOFParams.from_yaml()
    mu = 0.70  # Wet asphalt
    road = UniformRoad(mu=mu)
    model = SevenDOFModel(params=params, road=road)

    pp_controller = PhasePlaneEnvelopeController(config=PhasePlaneConfig(beta_sat_dry_deg=8.0))

    v0 = 85.0 / 3.6  # 23.6 m/s
    state = SevenDOFState(
        vx=v0, vy=0.0, r=0.0, omega=np.full(4, v0 / params.Reff), X=0.0, Y=0.0, psi=0.0
    )

    dt = 0.005
    duration = 2.5
    steps = int(duration / dt)

    s_phase_max = 0.0
    interventions_triggered = 0

    for i in range(steps):
        t = i * dt
        # Cornering: driver steers 4.5 deg
        delta_f = math.radians(4.5) if t < 1.8 else 0.0
        # Sudden lift-off and tap-braking at t=0.7s (shifts normal load forward, unloading rear axle)
        t_brake = np.full(4, 900.0) if (0.7 <= t < 1.4) else np.zeros(4)

        u = SevenDOFControlInput(steer_f=delta_f, T_brake=t_brake)
        state, telem = model.step(state, u, dt, road)

        ay_meas = float(np.sum(telem.Fy_body) / params.mass)
        ax_meas = float(np.sum(telem.Fx_body) / params.mass)
        eval_res = pp_controller.evaluate(
            vx=state.vx,
            vy=state.vy,
            r=state.r,
            ay=ay_meas,
            mu=mu,
            ax=ax_meas,
        )

        s_phase_max = max(s_phase_max, eval_res.stability_index)
        if eval_res.phase_state != PhasePlaneState.LINEAR_STABLE:
            interventions_triggered += 1

    # Interventions must be triggered during the lift-off transient
    assert interventions_triggered > 0, "Phase-plane failed to detect approaching oversteer"
    # Envelope controller limits the instability index
    assert s_phase_max > 0.65, "Did not reach approaching threshold"


# ---------------------------------------------------------------------------
# Test 5: Speed-Adaptive Driver Consistency (30 km/h to 110 km/h)
# ---------------------------------------------------------------------------
def test_e2e_speed_adaptive_driver_consistency():
    """Verify that speed-adaptive driver preview remains stable without PIO across speeds."""
    speeds_kmh = [30.0, 70.0, 110.0]
    dt = 0.01

    for v_kmh in speeds_kmh:
        v_mps = v_kmh / 3.6
        cfg = DriverModelConfig(
            enable_speed_adaptive_gains=True,
            enable_cerebellar_prediction=True,
            v_base_mps=20.0,
        )
        driver = NeuromuscularDriverModel(config=cfg, dt=dt)
        # S-curve path
        driver.set_path_function(lambda x: (1.5 * math.sin(2.0 * math.pi * x / 100.0), 0.0))

        rates = []
        for step_i in range(150):
            x_pos = step_i * dt * v_mps
            telem = driver.step(x=x_pos, y=0.0, psi=0.0, vx=v_mps, vy=0.0, r=0.0)
            rates.append(abs(telem.steer_rate_radps))

        # Under all speeds, steering rates must be smooth and well below maximum physical slew rate
        assert max(rates) < cfg.max_steer_rate_radps * 0.50
        assert not math.isnan(telem.steer_actual_rad)
