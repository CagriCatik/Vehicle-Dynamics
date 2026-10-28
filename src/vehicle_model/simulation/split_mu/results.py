"""Result and metric containers of the split-mu scenarios."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional
from vehicle_model.roads.surface import RoadSurface
from vehicle_model.simulation.scenario_runner import SimulationHistory


@dataclass
class SplitMuMetrics:
    """Quantitative performance and stability metrics for a split-mu scenario."""

    scenario_name: str
    controller_name: str
    initial_speed_kph: float
    final_speed_kph: float
    max_yaw_rate_deg_s: float
    final_heading_dev_deg: float
    max_lateral_dev_m: float
    max_sideslip_deg: float
    stopping_distance_m: Optional[float] = None
    acceleration_time_s: Optional[float] = None
    max_driver_steer_deg: float = 0.0
    stability_status: str = "PASS"  # "PASS", "DEGRADED", "UNSTABLE"


@dataclass
class SplitMuResult:
    """Detailed simulation history, road surface, and metrics for a split-mu run."""

    scenario_name: str
    scenario_id: str
    controller_name: str
    history: SimulationHistory
    road: RoadSurface
    metrics: SplitMuMetrics


# =============================================================================
# PARAMETRIC SPLIT-MU FRICTION SWEEP & MULTI-SURFACE SENSITIVITY
# =============================================================================
@dataclass
class SplitMuFrictionSweepPoint:
    """Simulation results for a single (mu_high, mu_low) friction pair."""

    mu_high: float
    mu_low: float
    delta_mu: float
    results: Dict[str, SplitMuResult]


@dataclass
class SplitMuFrictionSweepSummary:
    """Aggregate multi-surface parametric sweep across varying mu_low levels."""

    mu_high: float
    mu_low_values: List[float]
    controllers: List[str]
    controller_names: Dict[str, str]
    sweep_points: List[SplitMuFrictionSweepPoint]
    yaw_rate_max: Dict[str, List[float]]
    heading_dev_end: Dict[str, List[float]]
    lateral_dev_max: Dict[str, List[float]]
    stopping_distance: Dict[str, List[float]]
    sideslip_max: Dict[str, List[float]]
    bifurcation_mu_low: Dict[str, Optional[float]]
