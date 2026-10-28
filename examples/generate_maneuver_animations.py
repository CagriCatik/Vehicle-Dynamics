"""Script to generate real-time 2D planar vehicle dynamics animations (GIFs).

Generates high-definition, synchronized animated GIFs for standard automotive maneuvers:
1. ISO 3888-2 Double Lane Change (DLC at 80 km/h: Passive vs 4WS vs IVMC)
2. Split-Mu Emergency Braking (100 km/h: Passive vs IVMC with ABS/ESC)
3. FMVSS 126 Sine with Dwell (80 km/h: Passive Spinout vs IVMC Stabilization)
4. ISO 7401 Step Steer Response (100 km/h: Passive vs 4WS vs IVMC)
5. ISO 7975 Braking in a Turn (80 km/h: Combined-Slip Oversteer vs CBC/IVMC)
6. NHTSA Dynamic Fishhook (80 km/h: Rollover / Spinout Mitigation & Yaw Stabilization)
7. Low-Speed Slalom (30 km/h: Virtual Wheelbase Reduction via Counter-Phase 4WS)
8. High-Speed Highway Lane Change (120 km/h: In-Phase 4WS Crab Translation)
9. C/GMRES NMPC Slalom Path Tracking (Kinematic Bicycle with Rear-Axle Steering LUT)

Run:
    python examples/generate_maneuver_animations.py
"""

from __future__ import annotations

import math
import sys
from pathlib import Path
import shutil
import time
import numpy as np

# Ensure repository root is on sys.path
_repo_root = Path(__file__).resolve().parent.parent
if str(_repo_root) not in sys.path:
    sys.path.insert(0, str(_repo_root))

from vehicle_model.controllers import (
    FourWheelSteeringController,
    IntegratedChassisController,
    PassiveBaselineController,
)
from vehicle_model.controllers.nmpc import (
    KinematicBicycleSystem,
    NMPCConfig,
    NMPCControllerCGMRES,
    NMPCPathTrackingController,
    get_ref_state,
)
from vehicle_model.simulation.scenario_runner import SimulationHistory, simulate_maneuver
from vehicle_model.vehicle_models.seven_dof import SevenDOFModel
from vehicle_model.visualization.animation import animate_maneuver
from vehicle_model.visualization.plan_view import VehicleParams


def get_standard_maneuver_controllers() -> dict:
    """Return dictionary of the standard 4-vehicle quartet for benchmark comparisons."""
    return {
        "passive": PassiveBaselineController(),
        "four_wheel_steering": FourWheelSteeringController.from_yaml("configs/steering/four_wheel_steering.yaml", enable_abs=False),
        "ivmc_2ws": IntegratedChassisController.from_yaml("configs/controllers/integrated_controller.yaml", enable_4ws=False),
        "full_ivmc_4ws": IntegratedChassisController.from_yaml("configs/controllers/integrated_controller.yaml", enable_4ws=True),
    }


def generate_all_animations(
    output_dirs: list[str | Path] = ["output/validation_report/animations", "output/animations"],
    fps: int = 20,
    force_targets: list[str] | None = None,
) -> None:
    for d in output_dirs:
        Path(d).mkdir(parents=True, exist_ok=True)

    print("=================================================================")
    print("STARTING VEHICLE DYNAMICS REAL-TIME ANIMATION GENERATION (20 FPS)")
    print("=================================================================")

    # Shared vehicle model and params
    model = SevenDOFModel.from_yaml("configs/vehicles/seven_dof_passenger.yaml")
    v_params = VehicleParams.from_seven_dof_params(model.params)

    # Generate once into primary dir, then copy to secondary dirs
    primary_dir = Path(output_dirs[0])
    primary_dir.mkdir(parents=True, exist_ok=True)
    other_dirs = [Path(d) for d in output_dirs[1:]]
    for d in other_dirs:
        d.mkdir(parents=True, exist_ok=True)

    def save_and_replicate(name: str, histories: dict, **kwargs) -> None:
        p_main = primary_dir / f"{name}.gif"
        animate_maneuver(histories=histories, output_path=p_main, **kwargs)
        for d in other_dirs:
            shutil.copy(p_main, d / f"{name}.gif")

    def should_skip(name: str) -> bool:
        if force_targets is not None and ("all" in force_targets or "--force" in force_targets or name in force_targets):
            return False
        p_main = primary_dir / f"{name}.gif"
        return p_main.exists() and p_main.stat().st_size > 1000

    # -------------------------------------------------------------------------
    # 1. ISO 3888-2 Double Lane Change
    # -------------------------------------------------------------------------
    if not should_skip("iso_3888_dlc"):
        print("\n[1/9] Simulating and Animating: ISO 3888-2 Double Lane Change...")
        t0 = time.time()
        duration = 4.5
        dt = 0.005
        steps = int(duration / dt)
        t_arr = np.linspace(0.0, duration, steps)
        delta_dlc = np.zeros(steps)
        for i, t in enumerate(t_arr):
            if 0.5 <= t <= 1.5:
                delta_dlc[i] = 0.115 * math.sin(2.0 * math.pi * 1.0 * (t - 0.5))
            elif 2.0 <= t <= 3.0:
                delta_dlc[i] = -0.105 * math.sin(2.0 * math.pi * 1.0 * (t - 2.0))

        ctrls_dlc = get_standard_maneuver_controllers()
        hists_dlc = {}
        for c_key, c_inst in ctrls_dlc.items():
            m = SevenDOFModel.from_yaml("configs/vehicles/seven_dof_passenger.yaml")
            hists_dlc[c_key] = simulate_maneuver(
                model=m,
                controller=c_inst,
                v_initial=80.0 / 3.6,
                delta_driver=delta_dlc,
                T_drive_demand=0.0,
                T_brake_demand=0.0,
                duration=duration,
                dt=dt,
                mu=1.0,
            )

        save_and_replicate(
            name="iso_3888_dlc",
            histories=hists_dlc,
            title="ISO 3888-2 Double Lane Change (Real-Time 2D Planar Vehicles)",
            v_params=v_params,
            fps=fps,
            road_y_bounds=(-3.0, 5.0),
            vehicle_scale=1.5,
        )
        print(f"      Saved iso_3888_dlc.gif in {time.time() - t0:.1f}s")
    else:
        print("\n[1/9] Skipping ISO 3888-2 Double Lane Change (already exists)")

    # -------------------------------------------------------------------------
    # 2. Split-Mu Emergency Braking
    # -------------------------------------------------------------------------
    if not should_skip("mu_split_braking"):
        print("\n[2/9] Simulating and Animating: Split-Mu Emergency Braking...")
        t0 = time.time()
        dur_mu = 3.5
        dt = 0.005
        steps_mu = int(dur_mu / dt)
        t_mu = np.linspace(0.0, dur_mu, steps_mu)
        delta_mu = np.zeros(steps_mu)
        mu_split = np.array([1.0, 0.20, 1.0, 0.20])

        ctrls_mu = get_standard_maneuver_controllers()
        hists_mu = {}
        for c_key, c_inst in ctrls_mu.items():
            m = SevenDOFModel.from_yaml("configs/vehicles/seven_dof_passenger.yaml")
            hists_mu[c_key] = simulate_maneuver(
                model=m,
                controller=c_inst,
                v_initial=100.0 / 3.6,
                delta_driver=delta_mu,
                T_drive_demand=0.0,
                T_brake_demand=1450.0,
                duration=dur_mu,
                dt=dt,
                mu=mu_split,
            )

        save_and_replicate(
            name="mu_split_braking",
            histories=hists_mu,
            title="Split-Mu Emergency Braking (100 km/h: Ice vs Dry Surface)",
            v_params=v_params,
            fps=fps,
            road_y_bounds=(-6.0, 6.0),
            split_mu_y=0.0,
            vehicle_scale=1.5,
        )
        print(f"      Saved mu_split_braking.gif in {time.time() - t0:.1f}s")
    else:
        print("\n[2/9] Skipping Split-Mu Emergency Braking (already exists)")

    # -------------------------------------------------------------------------
    # 3. FMVSS 126 Sine with Dwell
    # -------------------------------------------------------------------------
    if not should_skip("sine_with_dwell"):
        print("\n[3/9] Simulating and Animating: FMVSS 126 Sine with Dwell...")
        t0 = time.time()
        dur_swd = 3.5
        dt = 0.005
        steps_swd = int(dur_swd / dt)
        t_swd = np.linspace(0.0, dur_swd, steps_swd)
        delta_swd = np.zeros(steps_swd)
        f_sine = 0.7
        for i, t in enumerate(t_swd):
            if 0.5 <= t <= 1.5:
                delta_swd[i] = 0.087 * math.sin(2.0 * math.pi * f_sine * (t - 0.5))
            elif 1.5 < t <= 2.5:
                delta_swd[i] = -0.087

        ctrls_swd = get_standard_maneuver_controllers()
        hists_swd = {}
        for c_key, c_inst in ctrls_swd.items():
            m = SevenDOFModel.from_yaml("configs/vehicles/seven_dof_passenger.yaml")
            hists_swd[c_key] = simulate_maneuver(
                model=m,
                controller=c_inst,
                v_initial=80.0 / 3.6,
                delta_driver=delta_swd,
                T_drive_demand=0.0,
                T_brake_demand=0.0,
                duration=dur_swd,
                dt=dt,
                mu=0.85,
            )

        save_and_replicate(
            name="sine_with_dwell",
            histories=hists_swd,
            title="FMVSS 126 Sine with Dwell (Passive Spinout vs IVMC Stabilization)",
            v_params=v_params,
            fps=fps,
            road_y_bounds=(-6.0, 6.0),
            vehicle_scale=1.5,
        )
        print(f"      Saved sine_with_dwell.gif in {time.time() - t0:.1f}s")
    else:
        print("\n[3/9] Skipping FMVSS 126 Sine with Dwell (already exists)")

    # -------------------------------------------------------------------------
    # 4. ISO 7401 Step Steer Response
    # -------------------------------------------------------------------------
    if not should_skip("step_steer_iso_7401"):
        print("\n[4/9] Simulating and Animating: ISO 7401 Transient Step Steer...")
        t0 = time.time()
        dur_step = 3.0
        dt = 0.005
        steps_step = int(dur_step / dt)
        t_step = np.linspace(0.0, dur_step, steps_step)
        delta_step = np.zeros(steps_step)
        for i, t in enumerate(t_step):
            if t >= 0.5:
                delta_step[i] = math.radians(3.0)

        ctrls_step = get_standard_maneuver_controllers()
        hists_step = {}
        for c_key, c_inst in ctrls_step.items():
            m = SevenDOFModel.from_yaml("configs/vehicles/seven_dof_passenger.yaml")
            hists_step[c_key] = simulate_maneuver(
                model=m,
                controller=c_inst,
                v_initial=100.0 / 3.6,
                delta_driver=delta_step,
                T_drive_demand=0.0,
                T_brake_demand=0.0,
                duration=dur_step,
                dt=dt,
                mu=0.85,
            )

        save_and_replicate(
            name="step_steer_iso_7401",
            histories=hists_step,
            title="ISO 7401 Transient Step Steer at 100 km/h (Planar Vehicles & 4WS)",
            v_params=v_params,
            fps=fps,
            road_y_bounds=(-5.0, 32.0),
            vehicle_scale=1.5,
        )
        print(f"      Saved step_steer_iso_7401.gif in {time.time() - t0:.1f}s")
    else:
        print("\n[4/9] Skipping ISO 7401 Step Steer (already exists)")

    # -------------------------------------------------------------------------
    # 5. ISO 7975 Braking in a Turn
    # -------------------------------------------------------------------------
    if not should_skip("braking_in_a_turn"):
        print("\n[5/9] Simulating and Animating: ISO 7975 Braking in a Turn...")
        t0 = time.time()
        dur_bit = 3.5
        dt = 0.005
        steps_bit = int(dur_bit / dt)
        t_bit = np.linspace(0.0, dur_bit, steps_bit)
        delta_bit = np.zeros(steps_bit)
        for i, t in enumerate(t_bit):
            if t >= 0.2:
                delta_bit[i] = math.radians(3.0)

        T_brake_bit = np.zeros(steps_bit)
        for i, t in enumerate(t_bit):
            if t >= 1.0:
                T_brake_bit[i] = 900.0

        ctrls_bit = get_standard_maneuver_controllers()
        hists_bit = {}
        for c_key, c_inst in ctrls_bit.items():
            m = SevenDOFModel.from_yaml("configs/vehicles/seven_dof_passenger.yaml")
            hists_bit[c_key] = simulate_maneuver(
                model=m,
                controller=c_inst,
                v_initial=80.0 / 3.6,
                delta_driver=delta_bit,
                T_drive_demand=0.0,
                T_brake_demand=T_brake_bit,
                duration=dur_bit,
                dt=dt,
                mu=0.85,
            )

        save_and_replicate(
            name="braking_in_a_turn",
            histories=hists_bit,
            title="ISO 7975 Braking in a Turn (80 km/h: Oversteer vs CBC/IVMC Stabilization)",
            v_params=v_params,
            fps=fps,
            road_y_bounds=(-3.0, 18.0),
            vehicle_scale=1.5,
        )
        print(f"      Saved braking_in_a_turn.gif in {time.time() - t0:.1f}s")
    else:
        print("\n[5/9] Skipping ISO 7975 Braking in a Turn (already exists)")

    # -------------------------------------------------------------------------
    # 6. NHTSA Dynamic Fishhook
    # -------------------------------------------------------------------------
    if not should_skip("fishhook_ncap"):
        print("\n[6/9] Simulating and Animating: NHTSA Dynamic Fishhook...")
        t0 = time.time()
        dur_fish = 3.5
        dt = 0.005
        steps_fish = int(dur_fish / dt)
        t_fish = np.linspace(0.0, dur_fish, steps_fish)
        delta_fish = np.zeros(steps_fish)
        for i, t in enumerate(t_fish):
            if 0.4 <= t < 0.65:
                delta_fish[i] = 0.095 * ((t - 0.4) / 0.25)
            elif 0.65 <= t < 0.90:
                delta_fish[i] = 0.095
            elif 0.90 <= t < 1.15:
                delta_fish[i] = 0.095 - 2.0 * 0.095 * ((t - 0.90) / 0.25)
            elif 1.15 <= t < 2.0:
                delta_fish[i] = -0.095
            elif 2.0 <= t < 2.3:
                delta_fish[i] = -0.095 * (1.0 - (t - 2.0) / 0.3)

        ctrls_fish = get_standard_maneuver_controllers()
        hists_fish = {}
        for c_key, c_inst in ctrls_fish.items():
            m = SevenDOFModel.from_yaml("configs/vehicles/seven_dof_passenger.yaml")
            hists_fish[c_key] = simulate_maneuver(
                model=m,
                controller=c_inst,
                v_initial=80.0 / 3.6,
                delta_driver=delta_fish,
                T_drive_demand=0.0,
                T_brake_demand=0.0,
                duration=dur_fish,
                dt=dt,
                mu=0.85,
            )

        save_and_replicate(
            name="fishhook_ncap",
            histories=hists_fish,
            title="NHTSA Dynamic Fishhook (80 km/h: Spinout Mitigation & Yaw Stabilization)",
            v_params=v_params,
            fps=fps,
            road_y_bounds=(-6.0, 10.0),
            vehicle_scale=1.5,
        )
        print(f"      Saved fishhook_ncap.gif in {time.time() - t0:.1f}s")
    else:
        print("\n[6/9] Skipping NHTSA Dynamic Fishhook (already exists)")

    # -------------------------------------------------------------------------
    # 7. Low-Speed Slalom (4WS Counter-Phase Agility)
    # -------------------------------------------------------------------------
    if not should_skip("low_speed_slalom_4ws"):
        print("\n[7/9] Simulating and Animating: Low-Speed Slalom (4WS Agility)...")
        t0 = time.time()
        dur_slalom = 4.8
        dt = 0.005
        steps_slalom = int(dur_slalom / dt)
        t_slalom = np.linspace(0.0, dur_slalom, steps_slalom)
        delta_slalom = np.zeros(steps_slalom)
        for i, t in enumerate(t_slalom):
            if 0.5 <= t < 1.2:
                delta_slalom[i] = 0.09 * math.sin(math.pi * (t - 0.5) / 0.7)
            elif 1.2 <= t < 2.6:
                delta_slalom[i] = -0.12 * math.sin(math.pi * (t - 1.2) / 1.4)
            elif 2.6 <= t < 4.0:
                delta_slalom[i] = 0.12 * math.sin(math.pi * (t - 2.6) / 1.4)
            elif 4.0 <= t < 4.7:
                delta_slalom[i] = -0.06 * math.sin(math.pi * (t - 4.0) / 0.7)

        ctrls_slalom = get_standard_maneuver_controllers()
        hists_slalom = {}
        for c_key, c_inst in ctrls_slalom.items():
            m = SevenDOFModel.from_yaml("configs/vehicles/seven_dof_passenger.yaml")
            hists_slalom[c_key] = simulate_maneuver(
                model=m,
                controller=c_inst,
                v_initial=30.0 / 3.6,
                delta_driver=delta_slalom,
                T_drive_demand=0.0,
                T_brake_demand=0.0,
                duration=dur_slalom,
                dt=dt,
                mu=0.90,
            )

        save_and_replicate(
            name="low_speed_slalom_4ws",
            histories=hists_slalom,
            title="Low-Speed Slalom at 30 km/h (Counter-Phase 4WS Turning Radius Shortening)",
            v_params=v_params,
            fps=fps,
            road_y_bounds=(-4.0, 4.0),
            vehicle_scale=1.5,
        )
        print(f"      Saved low_speed_slalom_4ws.gif in {time.time() - t0:.1f}s")
    else:
        print("\n[7/9] Skipping Low-Speed Slalom (already exists)")

    # -------------------------------------------------------------------------
    # 8. High-Speed 120 km/h Highway Lane Change (4WS In-Phase Crab Stability)
    # -------------------------------------------------------------------------
    if not should_skip("high_speed_lane_change"):
        print("\n[8/9] Simulating and Animating: High-Speed 120 km/h Highway Lane Change...")
        t0 = time.time()
        dur_hslc = 3.5
        dt = 0.005
        steps_hslc = int(dur_hslc / dt)
        t_hslc = np.linspace(0.0, dur_hslc, steps_hslc)
        f_lane = 0.70
        T_lane = 1.0 / f_lane
        delta_hslc = np.zeros(steps_hslc)
        for i, t in enumerate(t_hslc):
            if 0.5 <= t <= 0.5 + T_lane:
                delta_hslc[i] = 0.038 * math.sin(2.0 * math.pi * f_lane * (t - 0.5))

        ctrls_hslc = get_standard_maneuver_controllers()
        hists_hslc = {}
        for c_key, c_inst in ctrls_hslc.items():
            m = SevenDOFModel.from_yaml("configs/vehicles/seven_dof_passenger.yaml")
            hists_hslc[c_key] = simulate_maneuver(
                model=m,
                controller=c_inst,
                v_initial=120.0 / 3.6,
                delta_driver=delta_hslc,
                T_drive_demand=0.0,
                T_brake_demand=0.0,
                duration=dur_hslc,
                dt=dt,
                mu=0.90,
            )

        save_and_replicate(
            name="high_speed_lane_change",
            histories=hists_hslc,
            title="High-Speed 120 km/h Lane Change (In-Phase 4WS Crab Translation & Sideslip Reduction)",
            v_params=v_params,
            fps=fps,
            road_y_bounds=(-1.5, 4.5),
            vehicle_scale=1.5,
        )
        print(f"      Saved high_speed_lane_change.gif in {time.time() - t0:.1f}s")
    else:
        print("\n[8/9] Skipping High-Speed Lane Change (already exists)")

    # -------------------------------------------------------------------------
    # 9. C/GMRES NMPC Slalom Path Tracking (Full-Scale 7-DOF Vehicle Dynamics)
    # -------------------------------------------------------------------------
    if not should_skip("cgmres_nmpc_slalom"):
        print("\n[9/9] Simulating and Animating: C/GMRES NMPC Slalom Path Tracking...")
        t0 = time.time()
        m_sample = SevenDOFModel.from_yaml("configs/vehicles/seven_dof_passenger.yaml")
        v_params_nmpc = VehicleParams.from_seven_dof_params(m_sample.params)
        wheelbase = m_sample.params.lf + m_sample.params.lr

        # Slalom course setup: 10 m/s (36 km/h), wavelength 35m, amplitude 1.6m
        dur_nmpc = 4.2
        dt_nmpc = 0.02
        steps_nmpc = int(dur_nmpc / dt_nmpc)
        v_slalom = 10.0  # 36 km/h
        wavelength_slalom = 35.0
        amplitude_slalom = 1.6

        # Reference trajectory line and slalom cones
        x_ref = np.linspace(0.0, 48.0, 500)
        k_slalom = 2.0 * math.pi / wavelength_slalom
        y_ref = amplitude_slalom * np.sin(k_slalom * x_ref)
        # Slalom cones placed along centerline at half-wavelength increments (apex points)
        cone_positions = [(8.75 + i * 17.5, 0.0) for i in range(3)]

        cfg_4ws = NMPCConfig(
            wheelbase=wheelbase,
            target_v=v_slalom,
            amplitude=amplitude_slalom,
            wavelength=wavelength_slalom,
            frequency=v_slalom / wavelength_slalom,
            max_accel=3.5,
            max_steer_front_rad=math.radians(25.0),
            N=8,
            tf=1.2,
            q_y=35.0,
            q_yaw=5.0,
            q_v=15.0,
            r_steer=0.1,
            r_accel=0.02,
        )
        cfg_fws = NMPCConfig(
            wheelbase=wheelbase,
            target_v=v_slalom,
            amplitude=amplitude_slalom,
            wavelength=wavelength_slalom,
            frequency=v_slalom / wavelength_slalom,
            max_accel=3.5,
            max_steer_front_rad=math.radians(25.0),
            N=8,
            tf=1.2,
            q_y=35.0,
            q_yaw=5.0,
            q_v=15.0,
            r_steer=0.1,
            r_accel=0.02,
            lut_path="",
        )

        # Open-loop driver steer for passive baseline chassis
        t_arr = np.linspace(0.0, dur_nmpc, steps_nmpc)
        delta_passive = 0.14 * np.sin(2.0 * math.pi * (v_slalom / wavelength_slalom) * t_arr)

        ctrls_nmpc = {
            "passive": (PassiveBaselineController(), delta_passive),
            "nmpc_front_steer": (NMPCPathTrackingController(config=cfg_fws, params=m_sample.params), np.zeros(steps_nmpc)),
            "cgmres_nmpc_4ws": (NMPCPathTrackingController(config=cfg_4ws, params=m_sample.params), np.zeros(steps_nmpc)),
        }

        hists_nmpc = {}
        for c_key, (c_inst, delta_cmd) in ctrls_nmpc.items():
            m = SevenDOFModel.from_yaml("configs/vehicles/seven_dof_passenger.yaml")
            hists_nmpc[c_key] = simulate_maneuver(
                model=m,
                controller=c_inst,
                v_initial=v_slalom,
                delta_driver=delta_cmd,
                T_drive_demand=0.0,
                T_brake_demand=0.0,
                duration=dur_nmpc,
                dt=dt_nmpc,
                mu=1.0,
            )

        save_and_replicate(
            name="cgmres_nmpc_slalom",
            histories=hists_nmpc,
            title="C/GMRES NMPC Slalom Path Tracking (4WS LUT vs Front Steer vs Passive)",
            v_params=v_params_nmpc,
            fps=fps,
            road_y_bounds=(-4.0, 4.0),
            vehicle_scale=1.5,
            reference_path=(x_ref, y_ref),
            pylons=cone_positions,
        )
        print(f"      Saved cgmres_nmpc_slalom.gif in {time.time() - t0:.1f}s")
    else:
        print("\n[9/9] Skipping C/GMRES NMPC Slalom (already exists)")

    print("\n=================================================================")
    print("ALL 9 VEHICLE DYNAMICS ANIMATIONS SUCCESSFULLY GENERATED!")
    print("=================================================================")


if __name__ == "__main__":
    targets = sys.argv[1:] if len(sys.argv) > 1 else None
    generate_all_animations(force_targets=targets)
