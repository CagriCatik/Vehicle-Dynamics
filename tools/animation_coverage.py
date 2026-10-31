"""Which item of output/run_all_log.csv has an animation? Writes output/animation_coverage.csv, exit 1 if one is missing."""

from __future__ import annotations

import csv
import re
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
# items without a folder of their own, or sharing one: where their GIF lies
EXPLICIT = {
    "abs_tcs_esc_benchmark": "output/benchmark/abs_tcs_esc.gif", "integrated_control_benchmark": "output/benchmark/integrated_lane_change.gif",
    "multi_mu_scenarios_benchmark": "output/animations/multi_mu_lane_change_comparison.gif", "advanced_chassis_control_demo": "output/advanced_control/lift_off.gif",
    "driver_and_nmpc_dynamics_demo": "output/advanced_control/driver_in_the_loop.gif", "drift_circle_demo": "output/drift/runs/circle_hold/comparison.gif",
    "drift_figure_eight_demo": "output/drift/runs/figure_eight/supervised/drift_animation.gif", "drift_hold_demo": "output/drift/lqr_hold/hold_recovery.gif",
    "drift_equilibria_demo": "output/drift/equilibrium_sweep/held_equilibria.gif", "drift_initiation_demo": "output/drift/initiation/initiation.gif",
    "drift_robustness_demo": "output/drift/robustness/disturbances.gif", "drift_tyre_audit": "output/drift/tyre_audit.gif",
    "seven_dof_4ws_tv_demo": "output/demo_7dof/split_mu_4ws_tv.gif", "generate_maneuver_animations": "output/animations/sine_with_dwell.gif",
    "run_validation_platform": "output/validation_report/animations/sine_with_dwell.gif", "parking_4ws_demo": "output/four_wheel_steering/parking/u_turn.gif",
    "four_wheel_steering_benchmark": "output/four_wheel_steering/scenario_3_double_lane_change_4ws.gif",
    "split_mu_scenarios_demo": "output/split_mu_scenarios/scenario_1_split_mu_braking.gif", "nmpc_path_tracking_simulation": "output/nmpc_path_tracking/nmpc_path_tracking.gif",
    "actuation_trace": "output/actuation_chain/split_mu_braking.gif", "brake_functions_figures": "output/brake_functions/split_mu.gif",
    "traction_figures": "output/traction_control/split_mu_launch.gif", "roll_stability_steering_figures": "output/roll_stability/fishhook_rsc.gif",
    "g_vectoring_figures": "output/g_vectoring/lane_change.gif", "ride_control_figures": "output/ride_control/brake_pitch.gif",
    "hill_descent_figures": "output/hill_descent/hdc_20pct_dry.gif", "chassis_control_animations": "output/motion_management/levels_sine_with_dwell.gif",
    "stage_animations": "output/experiments/stability_map_cornering/comparison.gif",
    "ride_preview_damping_active_pitch": "output/ride_control/brake_pitch.gif", "hill_descent_control": "output/hill_descent/hdc_20pct_dry.gif",
    "fault_tolerance_catalogue": "output/fault_tolerance/sequence_recovery.gif", "motion_management_matrix": "output/motion_management/levels_sine_with_dwell.gif",
    "roll_stability_fishhook_evasive": "output/roll_stability/fishhook_rsc.gif",
}


def experiment_dir(name: str) -> Path:
    cfg = yaml.safe_load((ROOT / "configs/experiments" / f"{name}.yaml").read_text(encoding="utf-8"))
    return ROOT / (cfg.get("outputs") or {}).get("directory", f"output/experiments/{name}")


def example_dirs(name: str) -> list:
    return [ROOT / d for d in sorted(set(re.findall(r"output/[A-Za-z0-9_/-]+", (ROOT / "examples" / f"{name}.py").read_text(encoding="utf-8"))))]


def main() -> int:
    rows, missing = [], []
    for r in csv.DictReader(open(ROOT / "output/run_all_log.csv", encoding="utf-8")):
        item = r["item"]
        if item in EXPLICIT:
            gifs = [ROOT / EXPLICIT[item]] if (ROOT / EXPLICIT[item]).is_file() else []
        else:
            dirs = [experiment_dir(item)] if r["stage"] == "experiment" else example_dirs(item)
            gifs = sorted(g for d in dirs if d.is_dir() for g in d.glob("**/*.gif"))
        rows.append({"stage": r["stage"], "item": item, "status": r["status"], "animations": len(gifs),
                     "example": gifs[0].relative_to(ROOT).as_posix() if gifs else ""})
        if not gifs:
            missing.append(item)
    with open(ROOT / "output/animation_coverage.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    print(f"{len(rows) - len(missing)} of {len(rows)} items with an animation; missing: {missing}")
    return 1 if missing else 0


if __name__ == "__main__":
    sys.exit(main())
