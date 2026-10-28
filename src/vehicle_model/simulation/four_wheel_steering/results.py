"""Result and metric containers of the 4WS scenarios."""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np


@dataclass
class ScenarioMetrics:
    scenario_name: str
    controller_name: str
    min_turning_radius_m: float | None = None
    peak_sideslip_deg: float | None = None
    peak_yaw_rate_deg_s: float | None = None
    lateral_tracking_rms_m: float | None = None
    response_delay_ms: float | None = None
    final_heading_dev_deg: float | None = None
    max_lateral_dev_m: float | None = None
    stability_status: str = "PASS"


@dataclass
class ScenarioRunResult:
    controller_name: str
    metrics: ScenarioMetrics
    time: np.ndarray
    X: np.ndarray
    Y: np.ndarray
    psi: np.ndarray
    vx: np.ndarray
    vy: np.ndarray
    r: np.ndarray
    beta_deg: np.ndarray
    steer_f_deg: np.ndarray
    steer_r_deg: np.ndarray
    ay_mps2: np.ndarray
