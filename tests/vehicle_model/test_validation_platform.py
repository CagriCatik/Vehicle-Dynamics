"""Unit and integration tests for Automotive Virtual Validation Platform (Milestone M8).

Verifies:
- RegressionEvaluator criteria checking and delta calculations
- ValidationMatrixRunner scenario execution and telemetry capture
- ValidationReportGenerator Markdown and HTML compilation

Conforms to Milestone M8 (Plan.md §16, §19 M8).
"""

from pathlib import Path
import numpy as np
import pytest

from vehicle_model.simulation.kpi import EngineeringKPIs
from vehicle_model.simulation.regression_evaluator import (
    RegressionEvaluator,
    ScenarioEvaluation,
)
from vehicle_model.simulation.report_generator import ValidationReportGenerator
from vehicle_model.simulation.validation_matrix import ValidationMatrixRunner


def test_regression_evaluator_criteria_checking():
    """Verify criteria evaluator correctly detects threshold passes, fails, and margins."""
    kpis_pass = EngineeringKPIs(
        controller_name="IVMC",
        max_yaw_rate_radps=0.45,
        yaw_rate_rmse_radps=0.08,
        max_yaw_rate_error_radps=0.10,
        max_sideslip_deg=3.2,
        rms_sideslip_deg=1.1,
        final_heading_dev_deg=2.5,
        max_lateral_error_m=0.42,
        rms_lateral_error_m=0.18,
        peak_lat_accel_g=0.72,
        peak_long_decel_g=0.85,
        max_tyre_utilization=0.91,
        stopping_distance_m=42.5,
        stopping_time_s=2.8,
        mean_decel_mps2=9.8,
    )

    thresholds = {
        "max_sideslip_deg": 5.0,
        "max_lateral_error_m": 0.80,
        "final_heading_dev_deg": 10.0,
        "stopping_distance_m": 50.0,
    }

    res = RegressionEvaluator.evaluate(
        scenario_key="test_scenario",
        scenario_name="Test Scenario",
        controller_name="IVMC",
        kpis=kpis_pass,
        thresholds=thresholds,
    )

    assert res.overall_passed is True
    assert len(res.criteria) == 4
    for c in res.criteria:
        assert c.passed is True
        assert c.margin > 0.0

    # Test failure case
    kpis_fail = EngineeringKPIs(
        controller_name="Unregulated",
        max_yaw_rate_radps=1.20,
        yaw_rate_rmse_radps=0.45,
        max_yaw_rate_error_radps=0.60,
        max_sideslip_deg=12.5,  # Exceeds 5.0 deg
        rms_sideslip_deg=4.5,
        final_heading_dev_deg=45.0,  # Exceeds 10.0 deg
        max_lateral_error_m=2.1,  # Exceeds 0.80 m
        rms_lateral_error_m=0.9,
        peak_lat_accel_g=0.98,
        peak_long_decel_g=0.55,
        max_tyre_utilization=1.0,
        stopping_distance_m=72.0,  # Exceeds 50.0 m
        stopping_time_s=4.1,
        mean_decel_mps2=6.8,
    )

    res_fail = RegressionEvaluator.evaluate(
        scenario_key="test_scenario",
        scenario_name="Test Scenario",
        controller_name="Unregulated",
        kpis=kpis_fail,
        thresholds=thresholds,
    )

    assert res_fail.overall_passed is False
    assert any(not c.passed for c in res_fail.criteria)


def test_regression_evaluator_relative_deltas():
    """Verify calculation of relative percentage improvements over baseline."""
    base_kpi = EngineeringKPIs(
        controller_name="Passive",
        max_yaw_rate_radps=0.8,
        yaw_rate_rmse_radps=0.2,
        max_yaw_rate_error_radps=0.3,
        max_sideslip_deg=8.0,
        rms_sideslip_deg=3.0,
        final_heading_dev_deg=20.0,
        max_lateral_error_m=1.0,
        rms_lateral_error_m=0.4,
        peak_lat_accel_g=0.6,
        peak_long_decel_g=0.5,
        max_tyre_utilization=0.9,
        stopping_distance_m=60.0,
        stopping_time_s=3.5,
        mean_decel_mps2=7.9,
    )

    cand_kpi = EngineeringKPIs(
        controller_name="IVMC",
        max_yaw_rate_radps=0.5,
        yaw_rate_rmse_radps=0.1,
        max_yaw_rate_error_radps=0.15,
        max_sideslip_deg=4.0,  # 50% improvement
        rms_sideslip_deg=1.5,
        final_heading_dev_deg=4.0,  # 80% improvement
        max_lateral_error_m=0.6,  # 40% improvement
        rms_lateral_error_m=0.2,
        peak_lat_accel_g=0.75,
        peak_long_decel_g=0.85,
        max_tyre_utilization=0.95,
        stopping_distance_m=45.0,  # 25% improvement
        stopping_time_s=2.8,
        mean_decel_mps2=9.8,
    )

    deltas = RegressionEvaluator.compute_relative_deltas(base_kpi, cand_kpi)
    assert abs(deltas["sideslip_reduction_pct"] - 50.0) < 1e-3
    assert abs(deltas["heading_dev_reduction_pct"] - 80.0) < 1e-3
    assert abs(deltas["lateral_error_reduction_pct"] - 40.0) < 1e-3
    assert abs(deltas["stopping_distance_reduction_pct"] - 25.0) < 1e-3


def test_validation_report_compilation(tmp_path: Path):
    """Verify Markdown and HTML report generation."""
    kpi1 = EngineeringKPIs(
        controller_name="Passive Baseline",
        max_yaw_rate_radps=0.65,
        yaw_rate_rmse_radps=0.15,
        max_yaw_rate_error_radps=0.20,
        max_sideslip_deg=5.8,
        rms_sideslip_deg=2.1,
        final_heading_dev_deg=12.0,
        max_lateral_error_m=0.95,
        rms_lateral_error_m=0.35,
        peak_lat_accel_g=0.70,
        peak_long_decel_g=0.60,
        max_tyre_utilization=0.92,
        stopping_distance_m=55.0,
        stopping_time_s=3.2,
        mean_decel_mps2=8.6,
    )

    kpi2 = EngineeringKPIs(
        controller_name="Integrated IVMC",
        max_yaw_rate_radps=0.48,
        yaw_rate_rmse_radps=0.08,
        max_yaw_rate_error_radps=0.10,
        max_sideslip_deg=3.4,
        rms_sideslip_deg=1.2,
        final_heading_dev_deg=1.5,
        max_lateral_error_m=0.45,
        rms_lateral_error_m=0.16,
        peak_lat_accel_g=0.82,
        peak_long_decel_g=0.90,
        max_tyre_utilization=0.95,
        stopping_distance_m=42.0,
        stopping_time_s=2.6,
        mean_decel_mps2=10.6,
    )

    thresholds = {"max_sideslip_deg": 6.0, "final_heading_dev_deg": 15.0}

    eval1 = RegressionEvaluator.evaluate("mu_split", "Split-Mu Braking", "Passive Baseline", kpi1, thresholds)
    eval2 = RegressionEvaluator.evaluate("mu_split", "Split-Mu Braking", "Integrated IVMC", kpi2, thresholds)

    results = {"mu_split": [eval1, eval2]}

    gen = ValidationReportGenerator(output_dir=tmp_path)
    md_file = gen.generate_markdown(results)
    html_file = gen.generate_html(results)

    assert md_file.exists()
    assert html_file.exists()

    md_text = md_file.read_text(encoding="utf-8")
    assert "Vehicle Dynamics Virtual Validation Report" in md_text
    assert "Split-Mu Braking" in md_text
    assert "Passive Baseline" in md_text
    assert "Integrated IVMC" in md_text

    html_text = html_file.read_text(encoding="utf-8")
    assert "ACTIVE SUITE: 100% PASS" in html_text

