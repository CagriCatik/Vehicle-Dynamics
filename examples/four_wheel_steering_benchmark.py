"""Executable Four-Wheel Steering (4WS) Benchmark & Multi-Scenario Evaluation.

Executes 9 comprehensive scenarios:
1. Low-Speed Turning Circle (ISO 7401 / SAE J266, 12 km/h) - Counter-Phase.
2. Urban 90° Cornering & Curb Clearance (15 km/h) - Virtual Wheelbase Shortening.
3. Reverse Parallel Parking Agility (5 km/h) - Rear-End Sweep Reduction.
4. High-Speed Evasive Double Lane Change (ISO 3888-2, 80 km/h) - In-Phase Crabbing.
5. High-Speed Slalom (70 km/h, 30m Pylons) - Phase-Lag Elimination.
6. Split-μ Emergency Braking (ECE-R13H, 100 km/h) - Active Yaw Disturbance Rejection.
7. Autobahn Cruising Lane Change (130 km/h) - Sideslip Suppression.
8. Transient Step Steer Response (ISO 7401, 100 km/h) - Latency & Damping.
9. NHTSA Sine with Dwell (FMVSS 126, 80 km/h) - Rollover & Transient Yaw Stabilization.

Renders:
- Textbook 4WS characteristics plot (Küçükay / Heißing).
- 3D surface visualization of the calibrated 13x13 LUT map.
- Full 9-panel PNG dashboard with 2D vehicle bodies & steerable wheels.
- 7 high-definition animated GIFs with 2D body, front/rear wheels, and live HUD telemetry.
"""

from __future__ import annotations

import math
from pathlib import Path
import time
import matplotlib.pyplot as plt
import numpy as np

from vehicle_model.controllers.framework import FourWheelSteeringController, PassiveBaselineController
from vehicle_model.simulation.four_wheel_steering_scenarios import (
    animate_4ws_detailed_vehicle,
    plot_4ws_nine_scenario_dashboard,
    simulate_autobahn_lane_change,
    simulate_high_speed_double_lane_change,
    simulate_high_speed_slalom,
    simulate_high_speed_step_steer,
    simulate_low_speed_turning_circle,
    simulate_parallel_parking,
    simulate_sine_with_dwell,
    simulate_split_mu_braking_4ws,
    simulate_urban_90deg_cornering,
)
from vehicle_model.steering.four_wheel_steering import SteeringMode
from vehicle_model.steering.lut import load_lookup_table
from vehicle_model.vehicle_models.seven_dof import SevenDOFModel
from vehicle_model.visualization.textbook_rear_steering_plot import (
    plot_lut_3d_surface,
    plot_textbook_4ws_characteristics,
)


def main():
    print("=" * 85)
    print("FOUR-WHEEL STEERING (4WS) MULTI-SCENARIO BENCHMARK & ANIMATION SUITE")
    print("=" * 85)

    out_dir = Path("output/four_wheel_steering")
    out_dir.mkdir(parents=True, exist_ok=True)

    # 1. Textbook 4WS Characteristics Plot (Küçükay / Heißing style)
    print("\n[1/11] Generating Textbook 4WS Characteristics Plot (Küçükay / Heißing)...")
    textbook_png = out_dir / "textbook_4ws_characteristics.png"
    plot_textbook_4ws_characteristics(lut=None, output_file=textbook_png)
    print(f"       Saved: {textbook_png}")

    # 2. 3D LUT Visualization
    print("\n[2/11] Generating 3D Surface Visualization of Calibrated 13x13 LUT...")
    lut = load_lookup_table()
    surf_path = out_dir / "lut_3d_surface.png"
    plot_lut_3d_surface(lut, surf_path)
    print(f"       Saved: {surf_path}")

    # Controllers
    ctrl_2ws = PassiveBaselineController()
    ctrl_4ws_sched = FourWheelSteeringController(mode=SteeringMode.SPEED_SCHEDULED, enable_yaw_damping=False)
    ctrl_4ws_lut = FourWheelSteeringController(mode=SteeringMode.LOOKUP_TABLE, enable_yaw_damping=False)
    ctrl_4ws_damping = FourWheelSteeringController(mode=SteeringMode.LOOKUP_TABLE, enable_yaw_damping=True, k_yaw_damping=0.08)

    # 3. Scenario 1: Low-Speed Turning Circle (12 km/h, 25 deg steer)
    print("\n[3/11] Scenario 1: Low-Speed Turning Circle (12 km/h, Counter-Phase)...")
    res_s1 = {
        "2WS (Front-Only)": simulate_low_speed_turning_circle(
            model=SevenDOFModel.from_yaml("configs/vehicles/seven_dof_passenger.yaml"),
            enable_4ws=False,
            controller_name="2WS (Front-Only)",
        ),
        "4WS Speed-Scheduled": simulate_low_speed_turning_circle(
            model=SevenDOFModel.from_yaml("configs/vehicles/seven_dof_passenger.yaml"),
            steer_mode=SteeringMode.SPEED_SCHEDULED,
            enable_4ws=True,
            controller_name="4WS Speed-Scheduled",
        ),
        "4WS Calibrated LUT": simulate_low_speed_turning_circle(
            model=SevenDOFModel.from_yaml("configs/vehicles/seven_dof_passenger.yaml"),
            steer_mode=SteeringMode.LOOKUP_TABLE,
            enable_4ws=True,
            controller_name="4WS Calibrated LUT",
        ),
    }
    for name, r in res_s1.items():
        print(f"       {name:25s} -> Turning Radius: {r.metrics.min_turning_radius_m:.2f} m")

    # 4. Scenario 2: Urban 90° Cornering & Curb Clearance (15 km/h)
    print("\n[4/11] Scenario 2: Urban 90° Cornering & Curb Clearance (15 km/h)...")
    res_s2 = {
        "2WS (Front-Only)": simulate_urban_90deg_cornering(
            model=SevenDOFModel.from_yaml("configs/vehicles/seven_dof_passenger.yaml"),
            enable_4ws=False,
            controller_name="2WS (Front-Only)",
        ),
        "4WS Speed-Scheduled": simulate_urban_90deg_cornering(
            model=SevenDOFModel.from_yaml("configs/vehicles/seven_dof_passenger.yaml"),
            steer_mode=SteeringMode.SPEED_SCHEDULED,
            enable_4ws=True,
            controller_name="4WS Speed-Scheduled",
        ),
        "4WS Calibrated LUT": simulate_urban_90deg_cornering(
            model=SevenDOFModel.from_yaml("configs/vehicles/seven_dof_passenger.yaml"),
            steer_mode=SteeringMode.LOOKUP_TABLE,
            enable_4ws=True,
            controller_name="4WS Calibrated LUT",
        ),
    }
    for name, r in res_s2.items():
        print(f"       {name:25s} -> Peak Yaw Rate: {r.metrics.peak_yaw_rate_deg_s:5.1f}°/s | Peak Sideslip: {r.metrics.peak_sideslip_deg:4.2f}°")

    # 5. Scenario 3: Reverse Parallel Parking Agility (5 km/h)
    print("\n[5/11] Scenario 3: Reverse Parallel Parking Agility (5 km/h)...")
    res_s3 = {
        "2WS (Front-Only)": simulate_parallel_parking(
            model=SevenDOFModel.from_yaml("configs/vehicles/seven_dof_passenger.yaml"),
            enable_4ws=False,
            controller_name="2WS (Front-Only)",
        ),
        "4WS Calibrated LUT": simulate_parallel_parking(
            model=SevenDOFModel.from_yaml("configs/vehicles/seven_dof_passenger.yaml"),
            enable_4ws=True,
            controller_name="4WS Calibrated LUT",
        ),
    }
    for name, r in res_s3.items():
        print(f"       {name:25s} -> Envelope Width: {r.metrics.max_lateral_dev_m:4.2f} m | Final Heading: {r.metrics.final_heading_dev_deg:4.1f}°")

    # 6. Scenario 4: High-Speed Double Lane Change (80 km/h ISO 3888-2)
    print("\n[6/11] Scenario 4: High-Speed Evasive Double Lane Change (80 km/h, In-Phase)...")
    res_s4 = {
        "2WS (Front-Only)": simulate_high_speed_double_lane_change(
            model=SevenDOFModel.from_yaml("configs/vehicles/seven_dof_passenger.yaml"),
            controller=ctrl_2ws,
            controller_name="2WS (Front-Only)",
        ),
        "4WS Speed-Scheduled": simulate_high_speed_double_lane_change(
            model=SevenDOFModel.from_yaml("configs/vehicles/seven_dof_passenger.yaml"),
            controller=ctrl_4ws_sched,
            controller_name="4WS Speed-Scheduled",
        ),
        "4WS Calibrated LUT": simulate_high_speed_double_lane_change(
            model=SevenDOFModel.from_yaml("configs/vehicles/seven_dof_passenger.yaml"),
            controller=ctrl_4ws_lut,
            controller_name="4WS Calibrated LUT",
        ),
        "4WS Calibrated LUT + Yaw Damping": simulate_high_speed_double_lane_change(
            model=SevenDOFModel.from_yaml("configs/vehicles/seven_dof_passenger.yaml"),
            controller=ctrl_4ws_damping,
            controller_name="4WS Calibrated LUT + Yaw Damping",
        ),
    }
    for name, r in res_s4.items():
        print(f"       {name:32s} -> Peak Sideslip |β|: {r.metrics.peak_sideslip_deg:5.2f}° | RMS Tracking: {r.metrics.lateral_tracking_rms_m:.3f} m")

    # 7. Scenario 5: High-Speed Slalom (70 km/h, 30m pylons)
    print("\n[7/11] Scenario 5: High-Speed Slalom (70 km/h, 30m Pylons)...")
    res_s5 = {
        "2WS (Front-Only)": simulate_high_speed_slalom(
            model=SevenDOFModel.from_yaml("configs/vehicles/seven_dof_passenger.yaml"),
            controller=ctrl_2ws,
            controller_name="2WS (Front-Only)",
        ),
        "4WS Calibrated LUT": simulate_high_speed_slalom(
            model=SevenDOFModel.from_yaml("configs/vehicles/seven_dof_passenger.yaml"),
            controller=ctrl_4ws_lut,
            controller_name="4WS Calibrated LUT",
        ),
        "4WS Calibrated LUT + Yaw Damping": simulate_high_speed_slalom(
            model=SevenDOFModel.from_yaml("configs/vehicles/seven_dof_passenger.yaml"),
            controller=ctrl_4ws_damping,
            controller_name="4WS Calibrated LUT + Yaw Damping",
        ),
    }
    for name, r in res_s5.items():
        print(f"       {name:32s} -> Peak Sideslip |β|: {r.metrics.peak_sideslip_deg:5.2f}° | RMS Tracking: {r.metrics.lateral_tracking_rms_m:.3f} m")

    # 8. Scenario 6: Split-μ Straight-Line Emergency Braking (100 km/h ECE-R13H)
    print("\n[8/11] Scenario 6: Split-μ Emergency Braking (100 km/h ECE-R13H)...")
    res_s6 = {
        "2WS (Front-Only)": simulate_split_mu_braking_4ws(
            model=SevenDOFModel.from_yaml("configs/vehicles/seven_dof_passenger.yaml"),
            controller=ctrl_2ws,
            controller_name="2WS (Front-Only)",
        ),
        "4WS Calibrated LUT": simulate_split_mu_braking_4ws(
            model=SevenDOFModel.from_yaml("configs/vehicles/seven_dof_passenger.yaml"),
            controller=ctrl_4ws_lut,
            controller_name="4WS Calibrated LUT",
        ),
        "4WS Calibrated LUT + Yaw Damping": simulate_split_mu_braking_4ws(
            model=SevenDOFModel.from_yaml("configs/vehicles/seven_dof_passenger.yaml"),
            controller=ctrl_4ws_damping,
            controller_name="4WS Calibrated LUT + Yaw Damping",
        ),
    }
    for name, r in res_s6.items():
        print(f"       {name:32s} -> Status: {r.metrics.stability_status:7s} | Δψ: {r.metrics.final_heading_dev_deg:5.1f}° | Max Lat: {r.metrics.max_lateral_dev_m:.2f} m")

    # 9. Scenario 7: Autobahn Lane Change (130 km/h)
    print("\n[9/11] Scenario 7: Autobahn Cruising Lane Change (130 km/h)...")
    res_s7 = {
        "2WS (Front-Only)": simulate_autobahn_lane_change(
            model=SevenDOFModel.from_yaml("configs/vehicles/seven_dof_passenger.yaml"),
            controller=ctrl_2ws,
            controller_name="2WS (Front-Only)",
        ),
        "4WS Calibrated LUT": simulate_autobahn_lane_change(
            model=SevenDOFModel.from_yaml("configs/vehicles/seven_dof_passenger.yaml"),
            controller=ctrl_4ws_lut,
            controller_name="4WS Calibrated LUT",
        ),
        "4WS Calibrated LUT + Yaw Damping": simulate_autobahn_lane_change(
            model=SevenDOFModel.from_yaml("configs/vehicles/seven_dof_passenger.yaml"),
            controller=ctrl_4ws_damping,
            controller_name="4WS Calibrated LUT + Yaw Damping",
        ),
    }
    for name, r in res_s7.items():
        print(f"       {name:32s} -> Peak Sideslip |β|: {r.metrics.peak_sideslip_deg:5.2f}° | RMS Tracking: {r.metrics.lateral_tracking_rms_m:.3f} m")

    # 10. Scenario 8: Transient Step Steer (100 km/h ISO 7401)
    print("\n[10/11] Scenario 8: Transient Step Steer Response (100 km/h)...")
    res_s8 = {
        "2WS (Front-Only)": simulate_high_speed_step_steer(
            model=SevenDOFModel.from_yaml("configs/vehicles/seven_dof_passenger.yaml"),
            controller=ctrl_2ws,
            controller_name="2WS (Front-Only)",
        ),
        "4WS Speed-Scheduled": simulate_high_speed_step_steer(
            model=SevenDOFModel.from_yaml("configs/vehicles/seven_dof_passenger.yaml"),
            controller=ctrl_4ws_sched,
            controller_name="4WS Speed-Scheduled",
        ),
        "4WS Calibrated LUT": simulate_high_speed_step_steer(
            model=SevenDOFModel.from_yaml("configs/vehicles/seven_dof_passenger.yaml"),
            controller=ctrl_4ws_lut,
            controller_name="4WS Calibrated LUT",
        ),
        "4WS Calibrated LUT + Yaw Damping": simulate_high_speed_step_steer(
            model=SevenDOFModel.from_yaml("configs/vehicles/seven_dof_passenger.yaml"),
            controller=ctrl_4ws_damping,
            controller_name="4WS Calibrated LUT + Yaw Damping",
        ),
    }
    for name, r in res_s8.items():
        print(f"       {name:32s} -> Ay Response Delay: {r.metrics.response_delay_ms:5.1f} ms | Peak Sideslip: {r.metrics.peak_sideslip_deg:4.2f}°")

    # 11. Scenario 9: NHTSA Sine with Dwell (80 km/h FMVSS 126)
    print("\n[11/11] Scenario 9: NHTSA Sine with Dwell Stability Test (80 km/h)...")
    res_s9 = {
        "2WS (Front-Only)": simulate_sine_with_dwell(
            model=SevenDOFModel.from_yaml("configs/vehicles/seven_dof_passenger.yaml"),
            controller=ctrl_2ws,
            controller_name="2WS (Front-Only)",
        ),
        "4WS Calibrated LUT": simulate_sine_with_dwell(
            model=SevenDOFModel.from_yaml("configs/vehicles/seven_dof_passenger.yaml"),
            controller=ctrl_4ws_lut,
            controller_name="4WS Calibrated LUT",
        ),
        "4WS Calibrated LUT + Yaw Damping": simulate_sine_with_dwell(
            model=SevenDOFModel.from_yaml("configs/vehicles/seven_dof_passenger.yaml"),
            controller=ctrl_4ws_damping,
            controller_name="4WS Calibrated LUT + Yaw Damping",
        ),
    }
    for name, r in res_s9.items():
        print(f"       {name:32s} -> Status: {r.metrics.stability_status:7s} | Peak |β|: {r.metrics.peak_sideslip_deg:4.2f}° | Post-Dwell r: {r.metrics.final_heading_dev_deg:4.1f}°/s")

    # Combine into dict for 9-scenario dashboard
    all_results = {
        "turning_circle": res_s1,
        "urban_90deg": res_s2,
        "parallel_parking": res_s3,
        "double_lane_change": res_s4,
        "slalom": res_s5,
        "split_mu": res_s6,
        "autobahn": res_s7,
        "step_steer": res_s8,
        "sine_with_dwell": res_s9,
    }

    # Generate Dashboards
    print("\n[+] Generating 9-Scenario Master Dashboard...")
    dash_path = out_dir / "four_wheel_steering_dashboard.png"
    plot_4ws_nine_scenario_dashboard(all_results, dash_path)
    print(f"    Saved 9-Scenario Dashboard: {dash_path}")

    # =========================================================================
    # GENERATING COMPREHENSIVE SUITE OF 7 HIGH-DEFINITION ANIMATED GIFS
    # =========================================================================
    print("\n[+] Generating High-Definition Animated GIFs with 2D Steered Wheels & Telemetry HUD...")

    # GIF 1: Low-Speed Turning Circle (12 km/h)
    gif1_path = out_dir / "scenario_1_low_speed_turning_circle_4ws.gif"
    print(f"    -> [1/7] Scenario 1: Low-Speed Turning Circle (12 km/h)...")
    animate_4ws_detailed_vehicle(
        result_4ws=res_s1["4WS Calibrated LUT"],
        result_2ws=res_s1["2WS (Front-Only)"],
        output_file=gif1_path,
        title="1. Rangieren / Wendekreis (12 km/h) — Gegenphasige Hinterachslenkung",
        fps=15,
    )
    print(f"       Saved: {gif1_path}")

    # GIF 2: Urban 90° Cornering & Curb Clearance (15 km/h)
    gif2_path = out_dir / "scenario_2_urban_90deg_cornering_4ws.gif"
    print(f"    -> [2/7] Scenario 2: Urban 90° Cornering & Curb Clearance (15 km/h)...")
    def setup_curb(ax: plt.Axes):
        curb_x = np.array([-5.0, 10.0, 10.0])
        curb_y = np.array([-2.5, -2.5, -25.0])
        ax.plot(curb_x, curb_y, color="#ef4444", linestyle="-", linewidth=2.5, label="Bordstein / Begrenzung")
    animate_4ws_detailed_vehicle(
        result_4ws=res_s2["4WS Calibrated LUT"],
        result_2ws=res_s2["2WS (Front-Only)"],
        output_file=gif2_path,
        title="2. Städtisches 90°-Einbiegen (15 km/h) — Virtuelle Radstandsverkürzung & Bordsteinabstand",
        fps=15,
        setup_ax_fn=setup_curb,
        x_lim=(-7.0, 20.0),
        y_lim=(-27.0, 4.0),
    )
    print(f"       Saved: {gif2_path}")

    # GIF 3: High-Speed Evasive Double Lane Change (80 km/h ISO 3888-2)
    gif3_path = out_dir / "scenario_3_double_lane_change_4ws.gif"
    print(f"    -> [3/7] Scenario 3: Double Lane Change (80 km/h ISO 3888-2)...")
    def setup_lanes(ax: plt.Axes):
        ax.axhline(0.0, color="#475569", linestyle=":", linewidth=1.2)
        ax.axhline(3.50, color="#475569", linestyle=":", linewidth=1.2, label="Soll-Fahrspur (Y = 3.5 m)")
    animate_4ws_detailed_vehicle(
        result_4ws=res_s4["4WS Calibrated LUT + Yaw Damping"],
        result_2ws=res_s4["2WS (Front-Only)"],
        output_file=gif3_path,
        title="3. Doppelspurwechsel (80 km/h ISO 3888-2) — Gleichphasige Krabbenbewegung",
        fps=15,
        setup_ax_fn=setup_lanes,
    )
    print(f"       Saved: {gif3_path}")

    # GIF 4: High-Speed Slalom (70 km/h, 30m pylons)
    gif4_path = out_dir / "scenario_4_high_speed_slalom_4ws.gif"
    print(f"    -> [4/7] Scenario 4: High-Speed Slalom (70 km/h, 30m Pylons)...")
    def setup_pylons(ax: plt.Axes):
        px = [10.0, 40.0, 70.0, 100.0]
        py = [0.0, 0.0, 0.0, 0.0]
        ax.scatter(px, py, color="#f59e0b", marker="^", s=120, zorder=6, label="Slalom-Pylonen (30 m Abstand)")
    animate_4ws_detailed_vehicle(
        result_4ws=res_s5["4WS Calibrated LUT + Yaw Damping"],
        result_2ws=res_s5["2WS (Front-Only)"],
        output_file=gif4_path,
        title="4. Hochgeschwindigkeits-Slalom (70 km/h) — Phasenverzugselimination & Pylonenkurs",
        fps=15,
        setup_ax_fn=setup_pylons,
    )
    print(f"       Saved: {gif4_path}")

    # GIF 5: Split-μ Straight-Line Emergency Braking (100 km/h ECE-R13H)
    gif5_path = out_dir / "scenario_6_split_mu_braking_4ws.gif"
    print(f"    -> [5/7] Scenario 6: Split-μ Emergency Braking (100 km/h ECE-R13H)...")
    def setup_split_mu(ax: plt.Axes):
        ax.axhspan(0.0, 4.0, color="#0284c7", alpha=0.12, label="High-μ Spur (μ = 1.0, Trockener Asphalt)")
        ax.axhspan(-4.0, 0.0, color="#93c5fd", alpha=0.12, label="Low-μ Spur (μ = 0.2, Nasses Eis)")
        ax.axhline(0.0, color="#f59e0b", linestyle="-", linewidth=1.8, label="Reibwertsprung-Kante (Y = 0)")
    animate_4ws_detailed_vehicle(
        result_4ws=res_s6["4WS Calibrated LUT + Yaw Damping"],
        result_2ws=res_s6["2WS (Front-Only)"],
        output_file=gif5_path,
        title="5. Split-μ Notbremsung (100 km/h ECE-R13H) — Aktive Gierdämpfung auf der Hinterachse",
        fps=15,
        setup_ax_fn=setup_split_mu,
        x_lim=(-5.0, 70.0),
        y_lim=(-3.0, 3.0),
    )
    print(f"       Saved: {gif5_path}")

    # GIF 6: Reverse Parallel Parking Agility (5 km/h)
    gif6_path = out_dir / "scenario_8_parallel_parking_4ws.gif"
    print(f"    -> [6/7] Scenario 8: Reverse Parallel Parking (5 km/h)...")
    def setup_parking_bay(ax: plt.Axes):
        ax.axhline(0.0, color="#64748b", linestyle="-", linewidth=2.0, label="Bordstein (Y = 0m)")
        ax.axhline(2.0, color="#475569", linestyle="--", linewidth=1.2, label="Parkstreifenlinie (Y = 2m)")
        ax.plot([7.5, 3.0, 3.0, 7.5, 7.5], [0.1, 0.1, 1.9, 1.9, 0.1], color="#94a3b8", linestyle="--", linewidth=1.5, label="Voraus-Fahrzeug")
        ax.plot([-4.5, -9.0, -9.0, -4.5, -4.5], [0.1, 0.1, 1.9, 1.9, 0.1], color="#94a3b8", linestyle="--", linewidth=1.5, label="Nachfolge-Fahrzeug")
        ax.plot([3.0, -4.5, -4.5, 3.0], [0.0, 0.0, 2.0, 2.0], color="#10b981", linestyle="--", linewidth=1.8, label="Parkbucht (7.5 m)")
    animate_4ws_detailed_vehicle(
        result_4ws=res_s3["4WS Calibrated LUT"],
        result_2ws=res_s3["2WS (Front-Only)"],
        output_file=gif6_path,
        title="6. Rückwärts-Längseinparken (5 km/h) — Gegenphasige Heck-Eindrehung & Hüllflächenreduktion",
        fps=15,
        setup_ax_fn=setup_parking_bay,
        x_lim=(-10.0, 8.5),
        y_lim=(-0.8, 3.8),
    )
    print(f"       Saved: {gif6_path}")

    # GIF 7: NHTSA Sine with Dwell Stability Test (80 km/h FMVSS 126)
    gif7_path = out_dir / "scenario_9_sine_with_dwell_4ws.gif"
    print(f"    -> [7/7] Scenario 9: NHTSA Sine with Dwell (80 km/h FMVSS 126)...")
    animate_4ws_detailed_vehicle(
        result_4ws=res_s9["4WS Calibrated LUT + Yaw Damping"],
        result_2ws=res_s9["2WS (Front-Only)"],
        output_file=gif7_path,
        title="7. NHTSA Sine with Dwell (80 km/h FMVSS 126) — Transienter Gierratendämpfungs-Nachweis",
        fps=15,
        x_lim=(-5.0, 110.0),
        y_lim=(-5.0, 4.0),
    )
    print(f"       Saved: {gif7_path}")

    print("\n" + "=" * 85)
    print("FOUR-WHEEL STEERING BENCHMARK & ALL 7 ANIMATIONS COMPLETED SUCCESSFULLY!")
    print(f"Artifacts saved in: {out_dir.resolve()}")
    print("=" * 85)


if __name__ == "__main__":
    main()


