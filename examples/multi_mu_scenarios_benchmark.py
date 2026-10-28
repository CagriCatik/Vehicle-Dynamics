"""Executive Multi-Friction (Multi-μ) Vehicle Dynamics & Control Benchmark Suite.

Executes standardized dynamic scenarios across a broad spectrum of surface friction
coefficients (μ in [0.20, 0.40, 0.70, 1.00]), characterizing:
1. ISO 3888-2 Severe Double Lane Change (DLC) Obstacle Avoidance (80 km/h)
2. FMVSS 126 / ECE-R13H Sine with Dwell ESC Stability (80 km/h)
3. NHTSA Dynamic Fishhook Steering Reversal (80 km/h)
4. ISO 7401 Transient Step Steer Response (100 km/h)
5. ISO 7975 Braking in a Turn (Combined-Slip Limit, 80 km/h)
6. ISO 4138 Circular Driving (Handling limits & adhesion boundary a_y,max ≈ μ · g)

Compares:
- Passive Baseline (Open Differential, No Active Control)
- Active 4WS (Zero Sideslip Strategy)
- Integrated IVMC (Active 4WS + Direct Yaw Torque Vectoring)

Outputs:
- output/benchmark/multi_mu_scenarios_dashboard.png
- output/animations/multi_mu_lane_change_comparison.gif
- output/benchmark/multi_mu_scenarios_report.md

Usage:
    python examples/multi_mu_scenarios_benchmark.py
    python examples/multi_mu_scenarios_benchmark.py --mu-levels 1.0,0.7,0.4,0.2 --skip-gif
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path
from typing import List

# Ensure repository root is on sys.path
_repo_root = Path(__file__).resolve().parent.parent
if str(_repo_root) not in sys.path:
    sys.path.insert(0, str(_repo_root))

from vehicle_model.logging import setup_logger
from vehicle_model.simulation.multi_mu_matrix import (
    MultiMuMatrixSummary,
    animate_multi_mu_lane_change,
    generate_multi_mu_report_md,
    plot_multi_mu_matrix_dashboard,
    run_comprehensive_multi_mu_benchmark,
)


def print_matrix_summary_table(summary: MultiMuMatrixSummary) -> None:
    """Print an executive console telemetry table across all scenarios and friction levels."""
    print("\n" + "=" * 148)
    print("EXECUTIVE MULTI-FRICTION (MULTI-μ) VEHICLE DYNAMICS BENCHMARK RESULTS")
    print("=" * 148)
    print(
        f"{'SCENARIO':<22} | "
        f"{'μ':<5} | "
        f"{'SURFACE':<20} | "
        f"{'PASSIVE (|β|, r)':<18} | "
        f"{'ACTIVE 4WS':<18} | "
        f"{'IVMC 2WS':<18} | "
        f"{'FULL IVMC+4WS':<18} | "
        f"{'STABILITY'}"
    )
    print("-" * 148)

    for sc_id in summary.scenarios:
        sc_name = summary.scenario_names.get(sc_id, sc_id)
        sc_dict = summary.results[sc_id]
        print(f"\n>> {sc_name.upper()}")

        for mu in summary.friction_levels:
            res = sc_dict[mu]
            kp = res.kpis["passive"]
            kf = res.kpis["four_wheel_steering"]
            ki2 = res.kpis.get("ivmc_2ws")
            ki = res.kpis.get("full_ivmc_4ws", res.kpis.get("integrated_ivmc"))

            p_str = f"{kp.max_sideslip_deg:4.1f}°, {math.degrees(kp.max_yaw_rate_radps):4.1f}°/s"
            f_str = f"{kf.max_sideslip_deg:4.1f}°, {math.degrees(kf.max_yaw_rate_radps):4.1f}°/s"
            i2_str = f"{ki2.max_sideslip_deg:4.1f}°, {math.degrees(ki2.max_yaw_rate_radps):4.1f}°/s" if ki2 else "N/A"
            i_str = f"{ki.max_sideslip_deg:4.1f}°, {math.degrees(ki.max_yaw_rate_radps):4.1f}°/s"

            if kp.max_sideslip_deg < 6.0 and math.degrees(kp.max_yaw_rate_radps) < 45.0:
                note = "✅ ALL STABLE"
            elif ki.max_sideslip_deg < 5.0 and kp.max_sideslip_deg >= 10.0:
                note = "❌ PASSIVE UNSTABLE / IVMC SECURED"
            elif ki.max_sideslip_deg < 5.0 and kf.max_sideslip_deg >= 8.0:
                note = "⚠️ 4WS SATURATED / IVMC SECURED"
            else:
                note = "⚠️ ELEVATED SIDESLIP"

            print(
                f"  {sc_name[:20]:<20} | "
                f"{mu:4.2f} | "
                f"{res.surface_name[:20]:<20} | "
                f"{p_str:<18} | "
                f"{f_str:<18} | "
                f"{i2_str:<18} | "
                f"{i_str:<18} | "
                f"{note}"
            )

    print("=" * 148 + "\n")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Comprehensive Multi-Mu Vehicle Dynamics Benchmark Across Proving Ground Friction Surfaces."
    )
    parser.add_argument(
        "--mu-levels",
        type=str,
        default="1.00,0.70,0.40,0.20",
        help="Comma-separated friction coefficients (default: 1.00,0.70,0.40,0.20)",
    )
    parser.add_argument(
        "--output-dir",
        type=str,
        default="output/benchmark",
        help="Directory to save telemetry dashboard and markdown reports (default: output/benchmark)",
    )
    parser.add_argument(
        "--anim-dir",
        type=str,
        default="output/animations",
        help="Directory to save animated GIFs (default: output/animations)",
    )
    parser.add_argument(
        "--skip-gif",
        action="store_true",
        help="Skip animated GIF generation to accelerate benchmark execution",
    )
    parser.add_argument(
        "--fps",
        type=int,
        default=15,
        help="Frames per second for generated animations (default: 15)",
    )
    parser.add_argument(
        "--dpi",
        type=int,
        default=85,
        help="DPI for animated GIFs (default: 85)",
    )

    args = parser.parse_args()
    logger = setup_logger("multi_mu_benchmark")

    # Parse friction levels
    mu_list: List[float] = [float(x.strip()) for x in args.mu_levels.split(",") if x.strip()]
    logger.info("Executing Multi-Friction Matrix Benchmark for μ = %s", mu_list)

    # Output paths
    out_dir = Path(args.output_dir)
    anim_dir = Path(args.anim_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    anim_dir.mkdir(parents=True, exist_ok=True)

    # 1. Run multi-scenario, multi-mu benchmark
    summary = run_comprehensive_multi_mu_benchmark(mu_levels=mu_list, output_dir=out_dir)

    # 2. Print terminal summary table
    print_matrix_summary_table(summary)

    # 3. Generate high-resolution telemetry dashboard
    dashboard_path = out_dir / "multi_mu_scenarios_dashboard.png"
    logger.info("Generating Telemetry Dashboard -> %s", dashboard_path)
    plot_multi_mu_matrix_dashboard(summary, dashboard_path)
    print(f"✅ Dashboard generated: file://{dashboard_path.resolve()}")

    # 4. Generate Markdown Engineering Report
    report_path = out_dir / "multi_mu_scenarios_report.md"
    logger.info("Generating Engineering Report -> %s", report_path)
    generate_multi_mu_report_md(summary, report_path)
    print(f"✅ Report generated:    file://{report_path.resolve()}")

    # 5. Generate Multi-Surface Synchronized Animation
    if not args.skip_gif:
        gif_path = anim_dir / "multi_mu_lane_change_comparison.gif"
        logger.info("Generating Synchronized Double Lane Change GIF -> %s", gif_path)
        animate_multi_mu_lane_change(summary, gif_path, fps=args.fps, dpi=args.dpi)
        print(f"✅ Animation generated: file://{gif_path.resolve()}")
    else:
        logger.info("Skipping animated GIF generation (--skip-gif flag set).")

    print("\n🎉 Multi-Friction Benchmark Execution Completed Successfully!\n")


if __name__ == "__main__":
    main()
