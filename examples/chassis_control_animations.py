"""Comparison GIFs of the chassis-control benchmarks, drawn from the recorded histories (run the experiments first).

    python examples/chassis_control_animations.py                                  # all topics
    python examples/chassis_control_animations.py brake_functions hill_descent     # selected topics
Out: output/<topic>/<scenario>.gif
"""

from __future__ import annotations

import sys
from pathlib import Path

from vehicle_model.visualization.run_animation import animate_runs, animate_side_view, animate_slope, load_history

D = 57.29577951308232
OUT = Path("output")
SPLIT = {"type": "split_mu", "mu_left": 1.0, "mu_right": 0.2}


def gif(topic, name, folder, runs, title, traces, road=None, playback=1.0):
    H = OUT / topic / folder / "histories"
    loaded = [(lab, load_history(H / f)) for f, lab in runs if (H / f).is_file()]
    missing = [f for f, _ in runs if not (H / f).is_file()]
    if missing:
        print(f"  skipped {name}: missing {missing} (run the experiment first)")
        return None
    out = animate_runs(loaded, OUT / topic / f"{name}.gif", title, traces, road=road, playback=playback)
    print(f"  {out} ({out.stat().st_size / 1e6:.1f} MB)")
    return out


def actuation_chain():
    gif("actuation_chain", "split_mu_braking", "split_mu_braking",
        [("base__passive__actuator_dynamics.csv", "passive, actuator dynamics"),
         ("base__PI__TV___actuator_dynamics.csv", "PI (TV), actuator dynamics")],
        "split-mu full braking 90 km/h (left mu 1.0, right 0.2), recorded 7-DOF runs",
        [("yaw rate", "r", D, "deg/s"), ("speed", "speed", 3.6, "km/h"), ("yaw moment issued -> realised", "Mz_realised", 1e-3, "kN m")],
        road=SPLIT, playback=2.0)


def brake_functions():
    gif("brake_functions", "split_mu", "split_mu",
        [("base__brake_functions.csv", "no yaw control"), ("base__brake_functions__EBD__IBC.csv", "EBD + IBC, no yaw control"),
         ("base__PI__TV.csv", "PI (TV)")],
        "split-mu full braking 90 km/h: stability against stopping distance",
        [("yaw rate", "r", D, "deg/s"), ("speed", "speed", 3.6, "km/h"), ("sideslip", "beta", D, "deg")], road=SPLIT, playback=2.0)
    gif("brake_functions", "brake_in_turn", "brake_in_turn",
        [("base__brake_functions.csv", "no yaw control"), ("base__PI__TV.csv", "PI (TV)"), ("base__PI__TV___CBC.csv", "PI + CBC")],
        "braking in a turn, 80 km/h, 3 deg, pedal 0.5 from 2 s (mu 1.0)",
        [("sideslip", "beta", D, "deg"), ("yaw rate", "r", D, "deg/s"), ("speed", "speed", 3.6, "km/h")])
    gif("brake_functions", "regen_coast_low_mu", "regen_coast_low_mu",
        [("manoeuvre.speed_kph_80__brake_functions__actuator_dynamics___regen_cut_on_abs__coast_300_N_m.csv", "regen cut on ABS"),
         ("manoeuvre.speed_kph_80__brake_functions__actuator_dynamics___regen_modulated__coast_300_N_m.csv", "regen slip control (MSR)")],
        "coast with 300 N m drag torque per wheel on mu 0.2 from 80 km/h",
        [("max |wheel slip|", "max_abs_slip", 1.0, "-"), ("regen power", "regen_power_w", 1e-3, "kW"), ("speed", "speed", 3.6, "km/h")],
        road={"type": "uniform", "mu": 0.2})


def traction_control():
    gif("traction_control", "split_mu_launch", "split_mu_launch",
        [("base__passive__actuator_dynamics.csv", "passive (no TCS)"), ("base__no_yaw_control__actuator_dynamics___open.csv", "open coupling + TCS"),
         ("base__no_yaw_control__actuator_dynamics___elsd_300_N_m.csv", "eLSD 300 N m"),
         ("base__PI__TV___actuator_dynamics___elsd_300_N_m__envelope__ITC_band_2.0_deg_s.csv", "ITC (band 2 deg/s)")],
        "split-mu launch from 5 km/h, full pedal (left mu 1.0, right 0.2)",
        [("speed", "speed", 3.6, "km/h"), ("yaw rate", "r", D, "deg/s"), ("max |wheel slip|", "max_abs_slip", 1.0, "-")], road=SPLIT)


def roll_stability_active_steering():
    gif("roll_stability", "fishhook_rsc", "fishhook_evasive",
        [("extreme_cog__fishhook_80__mu1.0__passive.csv", "passive"), ("extreme_cog__fishhook_80__mu1.0__RSC.csv", "RSC (brakes)"),
         ("extreme_cog__fishhook_80__mu1.0__active_roll.csv", "active roll only")],
        "14-DOF fishhook 80 km/h, 8 deg, h_cg 1.1 m: wheel lift (red wheels) and RSC",
        [("load transfer ratio", "ltr", 1.0, "-"), ("roll angle", "phi", D, "deg"), ("yaw rate", "r", D, "deg/s")])
    gif("active_front_steering", "sine_with_dwell", "sine_with_dwell",
        [("base__passive.csv", "2WS passive"), ("base__GS-LQR__TV.csv", "TV"), ("base__GS-LQR__AFS.csv", "AFS")],
        "sine with dwell 80 km/h, 6 deg: active front steering against 2WS and TV",
        [("sideslip", "beta", D, "deg"), ("front road-wheel angle", "steer_f", D, "deg"), ("yaw rate", "r", D, "deg/s")])


def g_vectoring():
    gif("g_vectoring", "lane_change", "lane_change",
        [("base__no_yaw_control.csv", "GVC off"), ("base__no_yaw_control__GVC.csv", "GVC on"),
         ("base__no_yaw_control__GVC_C_xy_0.3.csv", "GVC, C_xy 0.3")],
        "lane change 80 km/h, 2.5 deg, 0.5 Hz, constant pedal: GVC-like load transfer",
        [("longitudinal acceleration", "ax", 1.0, "m/s^2"), ("front axle load FL", "Fz_FL", 1e-3, "kN"), ("yaw rate", "r", D, "deg/s")])


def side(name, scenario, controllers, title, traces):
    H = OUT / "ride_control" / "benchmark" / "histories"
    files = [(H / f"{scenario}__{c}.csv".replace(" ", "_").replace("/", "_"), c) for c in controllers]
    missing = [str(f) for f, _ in files if not f.is_file()]
    if missing:
        print(f"  skipped {name}: missing {missing} (run configs/experiments/ride_preview_damping_active_pitch.yaml first)")
        return None
    out = animate_side_view([(c, load_history(f)) for f, c in files], OUT / "ride_control" / f"{name}.gif", title, traces)
    print(f"  {out} ({out.stat().st_size / 1e6:.1f} MB)")
    return out


def ride_control():
    side("brake_pitch", "brake_100_wavy", ["passive", "skyhook", "active pitch + skyhook"],
         "braking 0.6 g from 100 km/h on a wavy road (14-DOF): pitch with and without active pitch",
         [("pitch", "theta", D, "deg"), ("body vertical acceleration", "ddzs", 1.0, "m/s^2"), ("active strut force FL", "F_act_FL", 1e-3, "kN")])
    side("bump_preview", "bump_60", ["passive", "skyhook", "preview skyhook"],
         "5 cm bump at 60 km/h: skyhook against preview skyhook (external road sensor, 20 ms latency)",
         [("body vertical acceleration", "ddzs", 1.0, "m/s^2"), ("pitch", "theta", D, "deg"), ("damping FL", "c_FL", 1e-3, "kN s/m")])
    side("iso_C_comfort", "iso_C_60", ["passive", "skyhook", "groundhook"],
         "ISO 8608 class C road at 60 km/h: comfort (skyhook) against wheel load (groundhook)",
         [("body vertical acceleration", "ddzs", 1.0, "m/s^2"), ("tyre load FL", "Fz_FL", 1e-3, "kN"), ("damping FL", "c_FL", 1e-3, "kN s/m")])


def slope(name, scenario, controllers, title, playback=2.0):
    H = OUT / "hill_descent" / "benchmark" / "histories"
    files = [(H / f"{scenario}__{c}.csv".replace(" ", "_").replace("/", "_"), c) for c in controllers]
    missing = [str(f) for f, _ in files if not f.is_file()]
    if missing:
        print(f"  skipped {name}: missing {missing} (run configs/experiments/hill_descent_control.yaml first)")
        return None
    out = animate_slope([(c, load_history(f)) for f, c in files], OUT / "hill_descent" / f"{name}.gif", title,
                        [("speed", "speed", 3.6, "km/h"), ("road grade", "grade_pct", 1.0, "%"),
                         ("brake torque (sum)", "T_brake_total", 1e-3, "kN m")], v_set_kph=10.0, playback=playback)
    print(f"  {out} ({out.stat().st_size / 1e6:.1f} MB)")
    return out


def hill_descent():
    slope("hdc_20pct_dry", "dry_20pct", ["HDC off", "HDC", "HDC + actuator dynamics"],
          "20 % downhill, dry: hill descent control holds 10 km/h (7-DOF, gravity in the plant)")
    slope("hdc_ice_infeasible", "ice_20pct", ["HDC off", "HDC"],
          "20 % downhill on ice (mu 0.12 < tan of the grade): physically infeasible, reported as such")
    slope("hdc_grade_change", "grade_change_dry", ["HDC", "HDC, RR brake failed"],
          "grade change 5 -> 20 -> 5 %, dry, with and without a failed rear-right brake")


def fault_tolerance():
    H = "catalogue"
    health = ("health (-1 not monitored, 0 nominal, 1 limited, 2 degraded, 3 fallback, 4 unavail.)", "health_rank", 1.0, "-")
    gif("fault_tolerance", "split_mu_three_brakes_failed", H,
        [("split_mu_braking_90__brakes_RR_FL_FR_failed__PI_unaware.csv", "unaware: 3 brakes failed"),
         ("split_mu_braking_90__brakes_RR_FL_FR_failed__PI_fault_tolerant.csv", "fault tolerant: 3 brakes failed"),
         ("split_mu_braking_90__none__PI_fault_tolerant.csv", "no fault")],
        "split-mu braking 90 km/h, brakes RR, FL, FR fail at 0.5 s (self-report after 50 ms)",
        [("yaw rate", "r", D, "deg/s"), ("speed", "speed", 3.6, "km/h"), health], road=SPLIT, playback=2.0)
    gif("fault_tolerance", "imu_lost_split_mu", H,
        [("split_mu_braking_90__none__PI_fault_tolerant.csv", "no fault (PI yaw control)"),
         ("split_mu_braking_90__imu_lost__PI_fault_tolerant.csv", "IMU lost at 0.5 s: zero-yaw fallback")],
        "split-mu braking, yaw-rate sensor lost: fallback to balanced braking (unaware run: NaN, aborted)",
        [("yaw rate", "r", D, "deg/s"), ("speed", "speed", 3.6, "km/h"), health], road=SPLIT, playback=2.0)
    gif("fault_tolerance", "sequence_recovery", H,
        [("brake_in_turn_80__none__PI_fault_tolerant.csv", "no fault"),
         ("brake_in_turn_80__sequence_recovery__PI_fault_tolerant.csv", "RR brake 0.5-1.5 s, IMU 2.0-2.5 s")],
        "braking in a turn 80 km/h with a fault sequence: degraded modes and recovery to nominal",
        [health, ("yaw rate", "r", D, "deg/s"), ("speed", "speed", 3.6, "km/h")])


def motion_management():
    H = "matrix"
    gif("motion_management", "levels_sine_with_dwell", H,
        [("05_sine_with_dwell__passive.csv", "passive"), ("05_sine_with_dwell__tv_4ws.csv", "tv_4ws: 4WS+TV, ideal actuators"),
         ("05_sine_with_dwell__brake_functions.csv", "brake_functions: + actuator dynamics"), ("05_sine_with_dwell__afs_gvc_rsc.csv", "afs_gvc_rsc: + AFS, GVC")],
        "sine with dwell 80 km/h, 6 deg across configuration levels (nominal draw)",
        [("sideslip", "beta", D, "deg"), ("yaw rate", "r", D, "deg/s"), ("rear road-wheel angle", "steer_r", D, "deg")])
    gif("motion_management", "levels_actuator_fault", H,
        [("14_actuator_fault__passive.csv", "passive"), ("14_actuator_fault__afs_gvc_rsc.csv", "afs_gvc_rsc (no health monitor)"),
         ("14_actuator_fault__fault_tolerant.csv", "fault_tolerant (health monitor)")],
        "split-mu braking 90 km/h, brakes RR and FL fail at 0.5 s, across levels",
        [("yaw rate", "r", D, "deg/s"), ("speed", "speed", 3.6, "km/h"), ("sideslip", "beta", D, "deg")], road=SPLIT, playback=2.0)
    gif("motion_management", "levels_corner_exit", H,
        [("10_corner_exit__esc.csv", "esc: ESC-like"), ("10_corner_exit__brake_functions.csv", "brake_functions: 4WS+TV + brake functions"),
         ("10_corner_exit__afs_gvc_rsc.csv", "afs_gvc_rsc: + eLSD, ITC, AFS, GVC")],
        "corner exit on mu 0.6, full throttle from 2 s, across levels",
        [("speed", "speed", 3.6, "km/h"), ("sideslip", "beta", D, "deg"), ("yaw rate", "r", D, "deg/s")],
        road={"type": "uniform", "mu": 0.6})


TOPICS = {f.__name__: f for f in (actuation_chain, brake_functions, traction_control, roll_stability_active_steering, g_vectoring,
                                  ride_control, hill_descent, fault_tolerance, motion_management)}


def main(argv=None) -> int:
    names = (argv if argv is not None else sys.argv[1:]) or list(TOPICS)
    for n in names:
        print(n)
        TOPICS[n]()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
