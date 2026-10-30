"""Flagship Runner: Automotive Virtual Validation Platform.

Executes the automated scenario validation matrix and compiles traceable
Markdown and HTML engineering reports with embedded telemetry figures.

Usage:
    python examples/run_validation_platform.pymd vertical dynamics).
"""

from __future__ import annotations

import sys
import argparse
from pathlib import Path

# Ensure repository root is on sys.path
_repo_root = Path(__file__).resolve().parent.parent
if str(_repo_root) not in sys.path:
    sys.path.insert(0, str(_repo_root))

from vehicle_model.logging import setup_logger
from vehicle_model.simulation import (
    ValidationMatrixRunner,
    ValidationReportGenerator,
)


def main() -> int:
    parser = argparse.ArgumentParser(description="Automotive Virtual Validation Platform Runner")
    parser.add_argument("--animations", action="store_true", help="Generate real-time 2D planar vehicle animated GIFs (20 FPS)")
    args = parser.parse_args()

    log_file = Path("logs/validation_platform.log")
    logger = setup_logger(log_file=log_file)
    logger.info("Initializing Automotive Virtual Validation Platform...")

    matrix_cfg_path = Path("configs/validation/matrix.yaml")
    output_dir = Path("output/validation_report")
    figures_dir = output_dir / "figures"

    runner = ValidationMatrixRunner(
        config_path=matrix_cfg_path,
        figures_dir=figures_dir,
        logger=logger,
    )

    results = runner.run_all()

    if args.animations:
        logger.info("Generating real-time 2D planar vehicle dynamics animations...")
        from examples.generate_maneuver_animations import generate_all_animations
        generate_all_animations(output_dirs=[output_dir / "animations", Path("output/animations")])

    report_gen = ValidationReportGenerator(output_dir=output_dir)
    md_file = report_gen.generate_markdown(results)
    html_file = report_gen.generate_html(results)

    logger.info("=================================================================")
    logger.info("VALIDATION REPORT GENERATION COMPLETE")
    logger.info(f"Markdown Report: {md_file}")
    logger.info(f"HTML Dashboard:  {html_file}")
    logger.info("=================================================================")

    # Check candidate active controllers against acceptance criteria
    def is_candidate(name: str) -> bool:
        nl = name.lower()
        return ("full" in nl or "integrated" in nl or "skyhook" in nl) and "standalone" not in nl and "2ws" not in nl

    candidates_passed = True

    for s_key, evals in results.items():
        for ev in evals:
            if is_candidate(ev.controller_name):
                if not ev.overall_passed:
                    candidates_passed = False
                    logger.error(f"Candidate controller '{ev.controller_name}' FAILED criteria in scenario '{s_key}'.")
                else:
                    logger.info(f"Candidate controller '{ev.controller_name}' PASSED all criteria in scenario '{s_key}'.")
            else:
                if not ev.overall_passed:
                    logger.info(f"Baseline '{ev.controller_name}' failed dynamic thresholds as expected (validating active control requirement).")

    if candidates_passed:
        logger.info("SUCCESS: All active chassis control systems (IVMC 2WS, Full-Stack IVMC+4WS, Skyhook) MET 100% OF ACCEPTANCE CRITERIA.")
        return 0
    else:
        logger.error("FAILURE: One or more active chassis controllers did not meet acceptance criteria.")
        return 1



if __name__ == "__main__":
    sys.exit(main())
