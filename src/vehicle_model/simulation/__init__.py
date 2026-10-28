"""Scenario simulation and execution utilities."""

from .kpi import EngineeringKPIs, compute_kpis, format_kpi_table
from .scenario_runner import ScenarioRunner, SimulationHistory, simulate_maneuver
from .regression_evaluator import CriterionResult, RegressionEvaluator, ScenarioEvaluation
from .validation_matrix import ValidationMatrixRunner
from .report_generator import ValidationReportGenerator
from .handling_limits import (
    HandlingCharacteristics,
    GGDiagramResult,
    simulate_steady_state_circular_drive,
    evaluate_understeer_gradient,
    compute_gg_diagram,
    plot_handling_and_limits,
)
from .split_mu_scenarios import (
    SplitMuMetrics,
    SplitMuResult,
    simulate_split_mu_braking,
    simulate_split_mu_launch,
    simulate_split_mu_braking_in_turn,
    simulate_split_mu_diagonal_transition,
    simulate_split_mu_evasive_dlc,
    plot_split_mu_scenario_dashboard,
    animate_split_mu_scenario,
    run_all_split_mu_scenarios,
    SplitMuFrictionSweepPoint,
    SplitMuFrictionSweepSummary,
    simulate_split_mu_friction_sweep,
    plot_split_mu_friction_sweep,
    animate_split_mu_friction_comparison,
    generate_split_mu_friction_report_md,
)
from .multi_mu_matrix import (
    MultiMuScenarioResult,
    MultiMuMatrixSummary,
    simulate_dlc_multi_mu,
    simulate_sine_with_dwell_multi_mu,
    simulate_fishhook_multi_mu,
    simulate_step_steer_multi_mu,
    simulate_braking_in_turn_multi_mu,
    simulate_circle_handling_multi_mu,
    run_comprehensive_multi_mu_benchmark,
    plot_multi_mu_matrix_dashboard,
    animate_multi_mu_lane_change,
    generate_multi_mu_report_md,
)

__all__ = [
    "ScenarioRunner",
    "SimulationHistory",
    "simulate_maneuver",
    "EngineeringKPIs",
    "compute_kpis",
    "format_kpi_table",
    "CriterionResult",
    "RegressionEvaluator",
    "ScenarioEvaluation",
    "ValidationMatrixRunner",
    "ValidationReportGenerator",
    "HandlingCharacteristics",
    "GGDiagramResult",
    "simulate_steady_state_circular_drive",
    "evaluate_understeer_gradient",
    "compute_gg_diagram",
    "plot_handling_and_limits",
    "SplitMuMetrics",
    "SplitMuResult",
    "simulate_split_mu_braking",
    "simulate_split_mu_launch",
    "simulate_split_mu_braking_in_turn",
    "simulate_split_mu_diagonal_transition",
    "simulate_split_mu_evasive_dlc",
    "plot_split_mu_scenario_dashboard",
    "animate_split_mu_scenario",
    "run_all_split_mu_scenarios",
    "SplitMuFrictionSweepPoint",
    "SplitMuFrictionSweepSummary",
    "simulate_split_mu_friction_sweep",
    "plot_split_mu_friction_sweep",
    "animate_split_mu_friction_comparison",
    "generate_split_mu_friction_report_md",
    "MultiMuScenarioResult",
    "MultiMuMatrixSummary",
    "simulate_dlc_multi_mu",
    "simulate_sine_with_dwell_multi_mu",
    "simulate_fishhook_multi_mu",
    "simulate_step_steer_multi_mu",
    "simulate_braking_in_turn_multi_mu",
    "simulate_circle_handling_multi_mu",
    "run_comprehensive_multi_mu_benchmark",
    "plot_multi_mu_matrix_dashboard",
    "animate_multi_mu_lane_change",
    "generate_multi_mu_report_md",
]




