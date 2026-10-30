"""Automated regression evaluation and pass/fail criterion checking.

Evaluates simulation KPIs against engineering thresholds and computes
performance deltas between controller configurations.md vertical dynamics).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List

from vehicle_model.simulation.kpi import EngineeringKPIs


@dataclass
class CriterionResult:
    """Individual pass/fail check for a specific metric."""

    metric_name: str
    value: float
    threshold: float
    unit: str
    comparator: str  # '<=', '<', '>=', '>'
    passed: bool
    margin: float  # Difference to threshold

    @property
    def status_str(self) -> str:
        return "PASS" if self.passed else "FAIL"


@dataclass
class ScenarioEvaluation:
    """Consolidated validation results for a controller under a specific scenario."""

    scenario_key: str
    scenario_name: str
    controller_name: str
    kpis: EngineeringKPIs
    criteria: List[CriterionResult] = field(default_factory=list)
    custom_metrics: Dict[str, float] = field(default_factory=dict)

    @property
    def overall_passed(self) -> bool:
        return all(c.passed for c in self.criteria) if self.criteria else True

    @property
    def status_badge(self) -> str:
        return "✅ PASS" if self.overall_passed else "❌ FAIL"


class RegressionEvaluator:
    """Evaluates automotive engineering acceptance criteria."""

    @staticmethod
    def evaluate(
        scenario_key: str,
        scenario_name: str,
        controller_name: str,
        kpis: EngineeringKPIs,
        thresholds: Dict[str, Any],
        custom_metrics: Dict[str, float] | None = None,
    ) -> ScenarioEvaluation:
        """Evaluate KPIs against the configured threshold rules."""
        criteria: List[CriterionResult] = []
        c_metrics = custom_metrics or {}

        # 1. Sideslip angle check
        if "max_sideslip_deg" in thresholds:
            limit = float(thresholds["max_sideslip_deg"])
            val = float(kpis.max_sideslip_deg)
            passed = val <= limit
            criteria.append(
                CriterionResult(
                    metric_name="Peak Sideslip Angle",
                    value=val,
                    threshold=limit,
                    unit="deg",
                    comparator="<=",
                    passed=passed,
                    margin=limit - val,
                )
            )

        # 2. Lateral error check
        if "max_lateral_error_m" in thresholds:
            limit = float(thresholds["max_lateral_error_m"])
            val = float(kpis.max_lateral_error_m)
            passed = val <= limit
            criteria.append(
                CriterionResult(
                    metric_name="Max Lateral Error",
                    value=val,
                    threshold=limit,
                    unit="m",
                    comparator="<=",
                    passed=passed,
                    margin=limit - val,
                )
            )

        # 3. Final heading deviation (spinout check)
        if "final_heading_dev_deg" in thresholds:
            limit = float(thresholds["final_heading_dev_deg"])
            val = float(kpis.final_heading_dev_deg)
            passed = val <= limit
            criteria.append(
                CriterionResult(
                    metric_name="Final Heading Deviation",
                    value=val,
                    threshold=limit,
                    unit="deg",
                    comparator="<=",
                    passed=passed,
                    margin=limit - val,
                )
            )

        # 4. Stopping distance check
        if "stopping_distance_m" in thresholds and kpis.stopping_distance_m is not None:
            limit = float(thresholds["stopping_distance_m"])
            val = float(kpis.stopping_distance_m)
            passed = val <= limit
            criteria.append(
                CriterionResult(
                    metric_name="Stopping Distance",
                    value=val,
                    threshold=limit,
                    unit="m",
                    comparator="<=",
                    passed=passed,
                    margin=limit - val,
                )
            )

        # 5. Yaw rate check
        if "max_yaw_rate_deg_s" in thresholds:
            import math
            limit = float(thresholds["max_yaw_rate_deg_s"])
            val = float(math.degrees(kpis.max_yaw_rate_radps))
            passed = val <= limit
            criteria.append(
                CriterionResult(
                    metric_name="Peak Yaw Rate",
                    value=val,
                    threshold=limit,
                    unit="deg/s",
                    comparator="<=",
                    passed=passed,
                    margin=limit - val,
                )
            )

        # 6. Custom scenario-specific metrics (e.g. FMVSS 126 yaw rate ratio, Ride comfort RMS)
        if "yaw_rate_ratio_1s_pct" in thresholds and "yaw_rate_ratio_1s_pct" in c_metrics:
            limit = float(thresholds["yaw_rate_ratio_1s_pct"])
            val = float(c_metrics["yaw_rate_ratio_1s_pct"])
            passed = val <= limit
            criteria.append(
                CriterionResult(
                    metric_name="FMVSS 126 Yaw Damping Ratio at 1.0s",
                    value=val,
                    threshold=limit,
                    unit="%",
                    comparator="<=",
                    passed=passed,
                    margin=limit - val,
                )
            )

        if "skyhook_accel_rms_reduction_pct" in thresholds and "skyhook_accel_rms_reduction_pct" in c_metrics:
            limit = float(thresholds["skyhook_accel_rms_reduction_pct"])
            val = float(c_metrics["skyhook_accel_rms_reduction_pct"])
            passed = val >= limit
            criteria.append(
                CriterionResult(
                    metric_name="Skyhook Body Acceleration RMS Reduction",
                    value=val,
                    threshold=limit,
                    unit="%",
                    comparator=">=",
                    passed=passed,
                    margin=val - limit,
                )
            )

        # 7. ISO 4138 Handling & Understeer checks
        if "understeer_gradient_min_deg_per_g" in thresholds:
            limit = float(thresholds["understeer_gradient_min_deg_per_g"])
            val = (
                float(kpis.understeer_gradient_deg_per_g)
                if kpis.understeer_gradient_deg_per_g is not None
                else float(c_metrics.get("understeer_gradient_deg_per_g", 0.0))
            )
            passed = val >= limit
            criteria.append(
                CriterionResult(
                    metric_name="Linear Understeer Gradient Kus",
                    value=val,
                    threshold=limit,
                    unit="deg/g",
                    comparator=">=",
                    passed=passed,
                    margin=val - limit,
                )
            )

        if "max_steady_state_lat_accel_g" in thresholds:
            limit = float(thresholds["max_steady_state_lat_accel_g"])
            val = (
                float(kpis.max_steady_state_lat_accel_g)
                if kpis.max_steady_state_lat_accel_g is not None
                else float(c_metrics.get("max_steady_state_lat_accel_g", 0.0))
            )
            passed = val >= limit
            criteria.append(
                CriterionResult(
                    metric_name="Max Steady-State Lat Accel",
                    value=val,
                    threshold=limit,
                    unit="g",
                    comparator=">=",
                    passed=passed,
                    margin=val - limit,
                )
            )

        return ScenarioEvaluation(
            scenario_key=scenario_key,
            scenario_name=scenario_name,
            controller_name=controller_name,
            kpis=kpis,
            criteria=criteria,
            custom_metrics=c_metrics,
        )

    @staticmethod
    def compute_relative_deltas(
        baseline: EngineeringKPIs,
        candidate: EngineeringKPIs,
    ) -> Dict[str, float]:
        """Compute relative percentage improvements: positive means candidate improved over baseline."""
        deltas: Dict[str, float] = {}

        if baseline.max_sideslip_deg > 1e-4:
            deltas["sideslip_reduction_pct"] = (
                (baseline.max_sideslip_deg - candidate.max_sideslip_deg)
                / baseline.max_sideslip_deg
                * 100.0
            )

        if baseline.max_lateral_error_m > 1e-4:
            deltas["lateral_error_reduction_pct"] = (
                (baseline.max_lateral_error_m - candidate.max_lateral_error_m)
                / baseline.max_lateral_error_m
                * 100.0
            )

        if baseline.final_heading_dev_deg > 1e-4:
            deltas["heading_dev_reduction_pct"] = (
                (baseline.final_heading_dev_deg - candidate.final_heading_dev_deg)
                / baseline.final_heading_dev_deg
                * 100.0
            )

        if (
            baseline.stopping_distance_m is not None
            and candidate.stopping_distance_m is not None
            and baseline.stopping_distance_m > 1e-4
        ):
            deltas["stopping_distance_reduction_pct"] = (
                (baseline.stopping_distance_m - candidate.stopping_distance_m)
                / baseline.stopping_distance_m
                * 100.0
            )

        return deltas
