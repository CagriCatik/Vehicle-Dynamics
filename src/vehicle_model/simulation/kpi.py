"""Standardized Engineering Key Performance Indicators (KPIs) for vehicle dynamics evaluation.

Implements all engineering metrics specified in Plan.md §16.1 and §19 (Milestone M5)
with documented mathematical definitions and units.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import numpy as np

from vehicle_model.simulation.scenario_runner import SimulationHistory


@dataclass(frozen=True)
class EngineeringKPIs:
    """Documented vehicle dynamics and control metrics (Plan.md §16.1)."""

    controller_name: str

    # 1. Yaw & Lateral Stability KPIs
    max_yaw_rate_radps: float           # Max absolute body yaw rate [rad/s]
    yaw_rate_rmse_radps: float          # Root-Mean-Square yaw rate tracking error [rad/s]
    max_yaw_rate_error_radps: float     # Peak yaw rate tracking error [rad/s]
    max_sideslip_deg: float             # Peak body sideslip angle beta = arctan(vy/vx) [deg]
    rms_sideslip_deg: float             # RMS body sideslip angle [deg]
    final_heading_dev_deg: float        # Final vehicle heading deviation from target [deg]

    # 2. Path Tracking KPIs
    max_lateral_error_m: float          # Peak lateral displacement error |Y - Y_ref| [m]
    rms_lateral_error_m: float          # RMS lateral displacement error [m]

    # 3. Acceleration & Dynamic Limits
    peak_lat_accel_g: float             # Peak lateral acceleration ay = v_dot_y + r*vx [g]
    peak_long_decel_g: float            # Peak longitudinal deceleration -ax [g]
    max_tyre_utilization: float         # Peak friction utilization sqrt(Fx^2+Fy^2)/(mu*Fz) [-]

    # 4. Stopping & Longitudinal Performance
    stopping_distance_m: float | None   # Total distance to reach standstill (vx < 0.2 m/s) [m]
    stopping_time_s: float | None       # Time to reach standstill [s]
    mean_decel_mps2: float | None       # Average deceleration over stopping distance [m/s^2]

    # 5. Handling Limits & Understeer Metrics (Optional / ISO 4138)
    understeer_gradient_deg_per_g: float | None = None  # Steady-state understeer gradient K_us [deg/g]
    max_steady_state_lat_accel_g: float | None = None   # Peak sustainable lateral acceleration [g]

    def to_markdown_row(self) -> str:
        """Format KPIs as a row in a GitHub-flavored markdown table."""
        stop_dist_str = f"{self.stopping_distance_m:.2f} m" if self.stopping_distance_m is not None else "N/A"
        stop_time_str = f"{self.stopping_time_s:.2f} s" if self.stopping_time_s is not None else "N/A"
        return (
            f"| **{self.controller_name}** "
            f"| {math.degrees(self.max_yaw_rate_radps):.1f}°/s "
            f"| {self.max_sideslip_deg:.2f}° "
            f"| {self.final_heading_dev_deg:.2f}° "
            f"| {self.max_lateral_error_m:.2f} m "
            f"| {self.peak_lat_accel_g:.2f} g "
            f"| {stop_dist_str} "
            f"| {self.max_tyre_utilization * 100:.1f} % |"
        )


def compute_kpis(
    hist: SimulationHistory,
    controller_name: str = "Controller",
    r_des: np.ndarray | None = None,
    Y_ref: np.ndarray | None = None,
    psi_ref: float = 0.0,
    understeer_gradient_deg_per_g: float | None = None,
    max_steady_state_lat_accel_g: float | None = None,
) -> EngineeringKPIs:
    """Compute all standard engineering KPIs from a recorded SimulationHistory."""
    # Yaw rate analysis
    r_abs = np.abs(hist.r)
    max_r = float(np.max(r_abs))

    if r_des is not None and len(r_des) == len(hist.r):
        r_err = hist.r - r_des
        rmse_r = float(np.sqrt(np.mean(r_err**2)))
        max_r_err = float(np.max(np.abs(r_err)))
    else:
        rmse_r = float(np.sqrt(np.mean(hist.r**2)))
        max_r_err = max_r

    # Sideslip analysis (mask standstill speeds < 2.5 m/s to prevent kinematic singularity)
    moving_mask = hist.vx >= 2.5
    if np.any(moving_mask):
        beta_deg = np.degrees(np.abs(hist.sideslip_angle[moving_mask]))
    else:
        beta_deg = np.degrees(np.abs(hist.sideslip_angle))
    max_beta = float(np.max(beta_deg))
    rms_beta = float(np.sqrt(np.mean(beta_deg**2)))

    # Heading deviation
    final_heading_dev = float(abs(np.degrees(hist.psi[-1] - psi_ref)))

    # Lateral path deviation
    if Y_ref is not None and len(Y_ref) == len(hist.Y):
        y_err = hist.Y - Y_ref
    else:
        y_err = hist.Y
    max_lat_err = float(np.max(np.abs(y_err)))
    rms_lat_err = float(np.sqrt(np.mean(y_err**2)))

    # Accelerations
    # ay = d(vy)/dt + r * vx
    dt = float(hist.time[1] - hist.time[0]) if len(hist.time) > 1 else 0.01
    dvy_dt = np.gradient(hist.vy, dt)
    ay = dvy_dt + hist.r * hist.vx
    peak_lat_g = float(np.max(np.abs(ay))) / 9.81

    # ax = d(vx)/dt - r * vy
    dvx_dt = np.gradient(hist.vx, dt)
    ax = dvx_dt - hist.r * hist.vy
    decel_indices = np.where(ax < 0.0)[0]
    peak_long_decel_g = float(np.max(-ax[decel_indices])) / 9.81 if len(decel_indices) > 0 else 0.0

    # Tyre friction utilization
    max_util = float(np.max(hist.friction_utilization))

    # Stopping metrics
    stopped_mask = np.where(hist.vx < 0.35)[0]
    if len(stopped_mask) > 0:
        stop_idx = stopped_mask[0]
        stop_dist = float(hist.X[stop_idx] - hist.X[0])
        stop_time = float(hist.time[stop_idx] - hist.time[0])
        mean_decel = float(hist.vx[0] / max(stop_time, 1e-3))
    else:
        stop_dist = None
        stop_time = None
        mean_decel = None

    return EngineeringKPIs(
        controller_name=controller_name,
        max_yaw_rate_radps=max_r,
        yaw_rate_rmse_radps=rmse_r,
        max_yaw_rate_error_radps=max_r_err,
        max_sideslip_deg=max_beta,
        rms_sideslip_deg=rms_beta,
        final_heading_dev_deg=final_heading_dev,
        max_lateral_error_m=max_lat_err,
        rms_lateral_error_m=rms_lat_err,
        peak_lat_accel_g=peak_lat_g,
        peak_long_decel_g=peak_long_decel_g,
        max_tyre_utilization=max_util,
        stopping_distance_m=stop_dist,
        stopping_time_s=stop_time,
        mean_decel_mps2=mean_decel,
        understeer_gradient_deg_per_g=understeer_gradient_deg_per_g,
        max_steady_state_lat_accel_g=max_steady_state_lat_accel_g,
    )



def format_kpi_table(kpis: list[EngineeringKPIs]) -> str:
    """Generate a clean markdown comparison table for multiple controllers."""
    header = (
        "| Controller Configuration | Peak Yaw Rate | Max Sideslip | Heading Dev | Max Path Dev | Peak Lat Accel | Stopping Dist | Max Tyre Util |\n"
        "| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: |\n"
    )
    rows = [k.to_markdown_row() for k in kpis]
    return header + "\n".join(rows) + "\n"


__all__ = ("EngineeringKPIs", "compute_kpis", "format_kpi_table")
