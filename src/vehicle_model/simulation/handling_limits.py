"""Handling Limits and Understeer/Oversteer Analysis Module (ISO 4138 & ISO 8855).

Provides tools for evaluating:
1. Steady-state circular driving behavior conforming to ISO 4138.
2. Understeer gradient K_us(a_y) across linear and non-linear limit regimes.
3. Characteristic and critical vehicle speeds (v_char, v_crit).
4. Sideslip angle gradient d(beta)/d(a_y) and yaw velocity gain.
5. G-G acceleration envelopes and handling limits at the tyre-road friction boundary.

References:
- Küçükay, F. (2022): Grundlagen der Fahrzeugtechnik, Kap. 6.
- Mitschke, M.; Wallentowitz, H. (2014): Dynamik der Kraftfahrzeuge, Kap. 12.
- Gillespie, T. D. (1992): Fundamentals of Vehicle Dynamics, Ch. 6.
- ISO 4138:2012: Passenger cars — Steady-state circular driving behaviour.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from pathlib import Path
from typing import Dict, List, Optional, Tuple, Any

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from vehicle_model.controllers.framework import ControlTarget, VehicleController
from vehicle_model.core.types import SevenDOFControlInput, SevenDOFParams, SevenDOFState
from vehicle_model.roads.surface import UniformRoad
from vehicle_model.simulation.scenario_runner import SimulationHistory
from vehicle_model.vehicle_models.seven_dof import SevenDOFModel


@dataclass
class HandlingCharacteristics:
    """Quantitative results of steady-state handling and understeer analysis."""

    understeer_gradient_rad_per_ms2: float
    understeer_gradient_deg_per_g: float
    classification: str  # "understeer", "neutral", "oversteer"
    characteristic_speed_mps: Optional[float]
    critical_speed_mps: Optional[float]
    max_lateral_accel_ms2: float
    max_lateral_accel_g: float
    sideslip_gradient_deg_per_g: float
    linear_range_ay_max_g: float

    # Raw / smoothed response arrays across the acceleration sweep
    ay_array_g: np.ndarray
    delta_f_array_deg: np.ndarray
    delta_ackermann_deg: float
    beta_array_deg: np.ndarray
    yaw_rate_array_deg_s: np.ndarray
    speed_array_kph: np.ndarray
    local_understeer_gradient_deg_per_g: np.ndarray
    controller_name: str = ""

    # Effective 4WS and dynamic sideslip metrics (compensating ISO 4138 artifacts)
    understeer_gradient_eff_deg_per_g: float = 0.0
    delta_eff_array_deg: np.ndarray = field(default_factory=lambda: np.array([]))
    beta_dynamic_deg: np.ndarray = field(default_factory=lambda: np.array([]))
    beta_kinematic_deg: float = 0.0

    @property
    def max_lat_accel_g(self) -> float:
        return self.max_lateral_accel_g

    @property
    def max_sideslip_deg(self) -> float:
        if len(self.beta_array_deg) == 0:
            return 0.0
        return float(np.max(np.abs(self.beta_array_deg)))


@dataclass
class GGDiagramResult:
    """G-G acceleration boundary representation and friction circle envelope."""

    mu: float
    ay_limit_positive_g: float
    ay_limit_negative_g: float
    ax_accel_limit_g: float
    ax_brake_limit_g: float
    boundary_ay_g: np.ndarray
    boundary_ax_g: np.ndarray
    simulated_points: Optional[Tuple[np.ndarray, np.ndarray]] = None


def simulate_steady_state_circular_drive(
    model: SevenDOFModel,
    controller: Optional[VehicleController] = None,
    radius_m: float = 40.0,
    v_start_kph: float = 18.0,
    v_end_kph: float = 85.0,
    speed_ramp_rate_kph_per_s: float = 1.5,
    ramp_duration_s: Optional[float] = None,
    v_start_mps: Optional[float] = None,
    v_end_mps: Optional[float] = None,
    mu: float = 1.0,
    dt: float = 0.01,
) -> SimulationHistory:
    """Execute a quasi-steady-state constant-radius circular driving test (ISO 4138).

    In this test, the vehicle tracks a circle of constant radius R while speed
    is ramped up slowly (quasi-stationarily) until the adhesion limit is reached.
    A low speed ramp rate ensures dynamic transient inertial forces remain negligible.

    Args:
        model: SevenDOFModel instance with non-linear tyre and load transfer mechanics.
        controller: Optional active chassis controller (if None, neutral steering is commanded).
        radius_m: Radius of the circular trajectory track [m] (default 40.0 m).
        v_start_kph: Initial vehicle speed [km/h] (default 18 km/h).
        v_end_kph: Maximum test speed [km/h] (default 85 km/h).
        speed_ramp_rate_kph_per_s: Rate of longitudinal speed increase [km/h per second].
        ramp_duration_s: Optional explicit ramp duration [s].
        v_start_mps: Optional initial speed in m/s (overrides v_start_kph).
        v_end_mps: Optional final speed in m/s (overrides v_end_kph).
        mu: Road surface friction coefficient (default 1.0).
        dt: Integration time step [s].

    Returns:
        SimulationHistory containing full kinematics, tire forces, and chassis state.
    """
    if v_start_mps is not None:
        v_start_kph = v_start_mps * 3.6
    if v_end_mps is not None:
        v_end_kph = v_end_mps * 3.6

    if ramp_duration_s is not None and ramp_duration_s > 0:
        total_duration_s = float(ramp_duration_s)
        speed_ramp_rate_kph_per_s = (v_end_kph - v_start_kph) / total_duration_s
    else:
        total_duration_s = max(5.0, (v_end_kph - v_start_kph) / speed_ramp_rate_kph_per_s)

    n_steps = int(total_duration_s / dt)
    time_arr = np.linspace(0.0, total_duration_s, n_steps)

    params = model.params
    wheelbase = params.lf + params.lr
    ackermann_steer_rad = math.atan(wheelbase / radius_m)

    # Initial state initialization on the circular arc
    v_start_mps = v_start_kph / 3.6
    init_yaw_rate = v_start_mps / radius_m
    r_eff = params.Reff

    state = SevenDOFState(
        vx=v_start_mps,
        vy=0.0,
        r=init_yaw_rate,
        omega=np.full(4, v_start_mps / r_eff),
        X=0.0,
        Y=0.0,
        psi=0.0,
    )

    if controller is not None:
        controller.reset()

    # Data logging structures
    hist_vx = np.zeros(n_steps)
    hist_vy = np.zeros(n_steps)
    hist_r = np.zeros(n_steps)
    hist_omega = np.zeros((n_steps, 4))
    hist_X = np.zeros(n_steps)
    hist_Y = np.zeros(n_steps)
    hist_psi = np.zeros(n_steps)
    hist_steer_f = np.zeros(n_steps)
    hist_steer_r = np.zeros(n_steps)
    hist_T_drive = np.zeros((n_steps, 4))
    hist_T_brake = np.zeros((n_steps, 4))
    hist_kappa = np.zeros((n_steps, 4))
    hist_alpha = np.zeros((n_steps, 4))
    hist_Fz = np.zeros((n_steps, 4))
    hist_mu = np.zeros((n_steps, 4))
    hist_Fx_w = np.zeros((n_steps, 4))
    hist_Fy_w = np.zeros((n_steps, 4))
    hist_Fx_b = np.zeros((n_steps, 4))
    hist_Fy_b = np.zeros((n_steps, 4))

    # Steer angle and drive tracking PI variables
    steer_f_cmd = ackermann_steer_rad
    integral_steer_error = 0.0
    integral_speed_error = 0.0

    for step in range(n_steps):
        t = time_arr[step]

        # Target longitudinal speed ramp
        target_v_mps = (v_start_kph + speed_ramp_rate_kph_per_s * t) / 3.6
        # Target yaw rate for constant radius R: r_target = v / R
        target_r = state.vx / radius_m

        # Closed-loop kinematic guidance: track curvature 1/R
        # Curvature error = actual_curvature - target_curvature = (r / vx) - (1 / R)
        actual_curv = state.r / max(0.5, state.vx)
        target_curv = 1.0 / radius_m
        curv_error = target_curv - actual_curv

        integral_steer_error += curv_error * dt
        # Adaptive steer angle correction to maintain the circular arc
        k_p_steer = 1.5 * wheelbase
        k_i_steer = 0.4 * wheelbase
        delta_f_driver = ackermann_steer_rad + k_p_steer * curv_error + k_i_steer * integral_steer_error
        max_steer = getattr(params, "max_steer_f", math.radians(35.0))
        delta_f_driver = float(np.clip(delta_f_driver, -max_steer, max_steer))

        # Drive force calculation with PI speed regulation
        a_x_ramp = (speed_ramp_rate_kph_per_s / 3.6)
        c_rr = getattr(params, "c_rr", 0.015)
        rho = getattr(params, "rho", 1.225)
        f_resist = 0.5 * rho * params.Cd * params.Af * (state.vx ** 2) + params.mass * 9.81 * c_rr
        speed_err = target_v_mps - state.vx
        integral_speed_error = float(np.clip(integral_speed_error + speed_err * dt, -50.0, 50.0))
        f_long_total = params.mass * a_x_ramp + f_resist + 1500.0 * speed_err + 300.0 * integral_speed_error
        t_drive_wheel = max(0.0, f_long_total * r_eff / 4.0)
        accel_pedal = float(np.clip(t_drive_wheel / max(1.0, params.max_drive_torque), 0.0, 1.0))

        if controller is not None:
            target = ControlTarget(
                target_vx=target_v_mps,
                target_yaw_rate=target_r,
                driver_steer_f=delta_f_driver,
                driver_accel_pedal=accel_pedal,
                driver_brake_pedal=0.0,
                mu_estimated=mu,
            )
            ctrl_out = controller.compute_control(state, target, dt=dt)
            effective_steer_f = ctrl_out.steer_f if abs(ctrl_out.steer_f) > 1e-4 else delta_f_driver
            effective_steer_r = ctrl_out.steer_r
            t_drive = ctrl_out.T_drive
            t_brake = ctrl_out.T_brake
        else:
            effective_steer_f = delta_f_driver
            effective_steer_r = 0.0
            t_drive = np.full(4, t_drive_wheel)
            t_brake = np.zeros(4)

        # Log state before step
        hist_vx[step] = state.vx
        hist_vy[step] = state.vy
        hist_r[step] = state.r
        hist_omega[step] = state.omega
        hist_X[step] = state.X
        hist_Y[step] = state.Y
        hist_psi[step] = state.psi
        hist_steer_f[step] = effective_steer_f
        hist_steer_r[step] = effective_steer_r
        hist_T_drive[step] = t_drive
        hist_T_brake[step] = t_brake

        # Step non-linear 7-DOF dynamics
        u_input = SevenDOFControlInput(
            steer_f=effective_steer_f,
            steer_r=effective_steer_r,
            T_drive=t_drive,
            T_brake=t_brake,
        )
        u_clamped = u_input.clamped(model.params)
        road_surface = UniformRoad(mu=mu)
        state, telem = model.step(
            state=state,
            u=u_clamped,
            dt=dt,
            road=road_surface,
        )

        hist_kappa[step] = telem.kappa
        hist_alpha[step] = telem.alpha
        hist_Fz[step] = telem.Fz
        hist_mu[step] = telem.mu
        hist_Fx_w[step] = telem.Fx_wheel
        hist_Fy_w[step] = telem.Fy_wheel
        hist_Fx_b[step] = telem.Fx_body
        hist_Fy_b[step] = telem.Fy_body

        # Break early if vehicle spins out or leaves trajectory completely
        if abs(state.vy / max(0.1, state.vx)) > math.radians(35.0):
            # Spinout / complete breakaway detected
            n_steps = step + 1
            hist_vx = hist_vx[:n_steps]
            hist_vy = hist_vy[:n_steps]
            hist_r = hist_r[:n_steps]
            hist_omega = hist_omega[:n_steps]
            hist_X = hist_X[:n_steps]
            hist_Y = hist_Y[:n_steps]
            hist_psi = hist_psi[:n_steps]
            hist_steer_f = hist_steer_f[:n_steps]
            hist_steer_r = hist_steer_r[:n_steps]
            hist_T_drive = hist_T_drive[:n_steps]
            hist_T_brake = hist_T_brake[:n_steps]
            hist_kappa = hist_kappa[:n_steps]
            hist_alpha = hist_alpha[:n_steps]
            hist_Fz = hist_Fz[:n_steps]
            hist_mu = hist_mu[:n_steps]
            hist_Fx_w = hist_Fx_w[:n_steps]
            hist_Fy_w = hist_Fy_w[:n_steps]
            hist_Fx_b = hist_Fx_b[:n_steps]
            hist_Fy_b = hist_Fy_b[:n_steps]
            time_arr = time_arr[:n_steps]
            break

    return SimulationHistory(
        time=time_arr,
        vx=hist_vx,
        vy=hist_vy,
        r=hist_r,
        omega=hist_omega,
        X=hist_X,
        Y=hist_Y,
        psi=hist_psi,
        steer_f=hist_steer_f,
        steer_r=hist_steer_r,
        T_drive=hist_T_drive,
        T_brake=hist_T_brake,
        kappa=hist_kappa,
        alpha=hist_alpha,
        Fz=hist_Fz,
        mu=hist_mu,
        Fx_wheel=hist_Fx_w,
        Fy_wheel=hist_Fy_w,
        Fx_body=hist_Fx_b,
        Fy_body=hist_Fy_b,
    )


def evaluate_understeer_gradient(
    history: Optional[SimulationHistory] = None,
    wheelbase_m: float = 2.70,
    radius_m: float = 40.0,
    linear_fit_ay_max_g: float = 0.40,
    hist: Optional[SimulationHistory] = None,
    model_params: Optional[Any] = None,
    controller_name: str = "",
) -> HandlingCharacteristics:
    """Analyze steady-state circular test data and evaluate ISO 4138 understeer metrics.

    Evaluates:
    - Overall understeer gradient K_us [deg/g] and [rad/(m/s^2)] in the linear region.
    - Local non-linear understeer gradient d(delta_f)/d(a_y) up to limit handling.
    - ISO 8855 steering classification (Understeer, Neutral, Oversteer).
    - Characteristic speed v_char or critical speed v_crit.
    - Peak steady-state lateral acceleration a_y,max [g].
    - Body sideslip angle gradient d(beta)/d(a_y) [deg/g].

    Args:
        history: SimulationHistory from steady-state circular driving.
        wheelbase_m: Wheelbase L [m].
        radius_m: Circular trajectory radius [m].
        linear_fit_ay_max_g: Lateral acceleration threshold for linear region fit [g].
        hist: Alias for history.
        model_params: Optional SevenDOFParams to automatically extract wheelbase.
        controller_name: Optional controller identifier string.

    Returns:
        HandlingCharacteristics dataclass with quantitative handling metrics.
    """
    if history is None and hist is not None:
        history = hist
    if history is None:
        raise ValueError("history (or hist) argument must be provided to evaluate_understeer_gradient.")

    if model_params is not None:
        wheelbase_m = float(getattr(model_params, "wheelbase", getattr(model_params, "lf", 1.3) + getattr(model_params, "lr", 1.5)))

    g = 9.81
    # Lateral acceleration a_y = v_x * r [m/s^2]
    ay_ms2 = history.vx * history.r
    ay_g = ay_ms2 / g
    delta_f_deg = np.degrees(history.steer_f)
    delta_r_deg = np.degrees(history.steer_r)
    beta_deg = np.degrees(history.sideslip_angle)
    r_deg_s = np.degrees(history.r)
    speed_kph = history.vx * 3.6
    ackermann_steer_deg = math.degrees(math.atan(wheelbase_m / radius_m))

    # Kinematic sideslip offset on circular path of radius R: beta_kin = +atan(lr / R) for left turn (r > 0)
    lr_val = float(getattr(model_params, "lr", wheelbase_m * 0.536))
    beta_kin_rad = math.atan(lr_val / radius_m)
    beta_kin_deg = math.degrees(beta_kin_rad)

    # Filter steady-state evaluation window (a_y >= 0.05g) to eliminate entry transients
    valid_mask = ay_g >= 0.05
    if np.sum(valid_mask) > 10:
        ay_g = ay_g[valid_mask]
        ay_ms2 = ay_ms2[valid_mask]
        delta_f_deg = delta_f_deg[valid_mask]
        delta_r_deg = delta_r_deg[valid_mask]
        beta_deg = beta_deg[valid_mask]
        r_deg_s = r_deg_s[valid_mask]
        speed_kph = speed_kph[valid_mask]
        steer_f_rad = history.steer_f[valid_mask]
    else:
        steer_f_rad = history.steer_f

    # Sort data by lateral acceleration to handle non-monotonic points
    sort_idx = np.argsort(ay_g)
    ay_g_sorted = ay_g[sort_idx]
    delta_f_sorted = delta_f_deg[sort_idx]
    delta_r_sorted = delta_r_deg[sort_idx]
    delta_eff_sorted = delta_f_sorted - delta_r_sorted
    beta_sorted = beta_deg[sort_idx]
    beta_dyn_sorted = beta_kin_deg - beta_sorted
    ay_ms2_sorted = ay_ms2[sort_idx]
    delta_f_rad_sorted = steer_f_rad[sort_idx]

    # Filter linear region (0.05g <= a_y <= linear_fit_ay_max_g)
    linear_mask = (ay_g_sorted >= 0.05) & (ay_g_sorted <= linear_fit_ay_max_g)
    if np.sum(linear_mask) > 5:
        ay_linear_g = ay_g_sorted[linear_mask]
        delta_linear_deg = delta_f_sorted[linear_mask]
        delta_eff_linear_deg = delta_eff_sorted[linear_mask]
        beta_linear_deg = beta_sorted[linear_mask]
        ay_linear_ms2 = ay_ms2_sorted[linear_mask]
        delta_linear_rad = delta_f_rad_sorted[linear_mask]

        # Linear regression for front steer understeer gradient: delta = delta_0 + K_us * a_y
        poly_deg = np.polyfit(ay_linear_g, delta_linear_deg, deg=1)
        k_us_deg_per_g = float(poly_deg[0])

        # Linear regression for effective 4WS understeer gradient: delta_eff = delta_f - delta_r
        poly_eff = np.polyfit(ay_linear_g, delta_eff_linear_deg, deg=1)
        k_us_eff_deg_per_g = float(poly_eff[0])

        poly_rad = np.polyfit(ay_linear_ms2, delta_linear_rad, deg=1)
        k_us_rad_per_ms2 = float(poly_rad[0])

        # Dynamic sideslip gradient (ignoring kinematic offset)
        poly_beta = np.polyfit(ay_linear_g, beta_dyn_sorted[linear_mask], deg=1)
        d_beta_deg_per_g = float(poly_beta[0])
    else:
        # Fallback to simple finite difference if points are sparse
        d_ay = max(0.1, float(ay_g_sorted[-1] - ay_g_sorted[0]))
        d_delta = float(delta_f_sorted[-1] - delta_f_sorted[0])
        k_us_deg_per_g = d_delta / d_ay
        k_us_eff_deg_per_g = k_us_deg_per_g
        k_us_rad_per_ms2 = math.radians(k_us_deg_per_g) / g
        d_beta_deg_per_g = 0.0

    # Understeer / Oversteer classification according to ISO 8855
    if k_us_deg_per_g > 0.15:
        classification = "understeer"
        # Characteristic speed: v_char = sqrt(L / K_us)
        if k_us_rad_per_ms2 > 1e-6:
            v_char_mps = math.sqrt(wheelbase_m / k_us_rad_per_ms2)
        else:
            v_char_mps = None
        v_crit_mps = None
    elif k_us_deg_per_g < -0.15:
        classification = "oversteer"
        # Critical speed: v_crit = sqrt(-L / K_us)
        if k_us_rad_per_ms2 < -1e-6:
            v_crit_mps = math.sqrt(-wheelbase_m / k_us_rad_per_ms2)
        else:
            v_crit_mps = None
        v_char_mps = None
    else:
        classification = "neutral"
        v_char_mps = None
        v_crit_mps = None

    # Peak lateral acceleration (handling limit)
    max_ay_ms2 = float(np.max(ay_ms2))
    max_ay_g = float(np.max(ay_g))

    # Local differential understeer gradient d(delta_f) / d(a_y) smoothed across range
    window = max(5, len(ay_g_sorted) // 20)
    # Moving average smoothing
    kernel = np.ones(window) / window
    delta_smoothed = np.convolve(delta_f_sorted, kernel, mode="same")
    ay_smoothed = np.convolve(ay_g_sorted, kernel, mode="same")
    d_delta_local = np.gradient(delta_smoothed, np.maximum(1e-4, ay_smoothed))

    return HandlingCharacteristics(
        understeer_gradient_rad_per_ms2=k_us_rad_per_ms2,
        understeer_gradient_deg_per_g=k_us_deg_per_g,
        classification=classification,
        characteristic_speed_mps=v_char_mps,
        critical_speed_mps=v_crit_mps,
        max_lateral_accel_ms2=max_ay_ms2,
        max_lateral_accel_g=max_ay_g,
        sideslip_gradient_deg_per_g=d_beta_deg_per_g,
        linear_range_ay_max_g=linear_fit_ay_max_g,
        ay_array_g=ay_g_sorted,
        delta_f_array_deg=delta_f_sorted,
        delta_ackermann_deg=ackermann_steer_deg,
        beta_array_deg=beta_sorted,
        yaw_rate_array_deg_s=r_deg_s[sort_idx],
        speed_array_kph=speed_kph[sort_idx],
        local_understeer_gradient_deg_per_g=d_delta_local,
        controller_name=controller_name,
        understeer_gradient_eff_deg_per_g=k_us_eff_deg_per_g,
        delta_eff_array_deg=delta_eff_sorted,
        beta_dynamic_deg=beta_dyn_sorted,
        beta_kinematic_deg=beta_kin_deg,
    )


def compute_gg_diagram(
    params: SevenDOFParams,
    mu: float = 1.0,
    n_points: int = 100,
    operating_history: Optional[SimulationHistory] = None,
) -> GGDiagramResult:
    """Compute theoretical and empirical G-G acceleration diagram (friction envelope).

    Models combined acceleration limits taking into account:
    - Tyre-road friction coefficient mu.
    - Powertrain acceleration capability a_x,max.
    - Brake-by-wire deceleration capability a_x,brake.
    - Lateral grip limit a_y,max.

    Args:
        params: SevenDOFParams of the vehicle.
        mu: Road friction coefficient (default 1.0).
        n_points: Number of perimeter evaluation points.
        operating_history: Optional history of driving points to plot inside the envelope.

    Returns:
        GGDiagramResult containing the boundary contour and operating points.
    """
    g = 9.81
    # Lateral grip limit: a_y,max = mu * g (scaled slightly by load transfer degradation)
    ay_max_g = mu * 0.95
    # Braking limit: a_x,brake = mu * g
    ax_brake_g = -mu * 0.98

    # Powertrain acceleration limit: bounded by max electric drive torque
    total_max_drive_force = 4.0 * params.max_drive_torque / params.Reff
    ax_power_limit_g = min(mu * 0.95, total_max_drive_force / (params.mass * g))

    # Parametric angle theta from 0 to 2*pi
    theta = np.linspace(0.0, 2.0 * math.pi, n_points)
    ay_boundary = ay_max_g * np.cos(theta)
    ax_boundary = np.zeros(n_points)

    for i, th in enumerate(theta):
        sin_th = np.sin(th)
        if sin_th >= 0:
            # Acceleration upper half
            ax_boundary[i] = ax_power_limit_g * sin_th
        else:
            # Braking lower half
            ax_boundary[i] = abs(ax_brake_g) * sin_th

    op_points = None
    if operating_history is not None:
        ay_op_g = (operating_history.vx * operating_history.r) / g
        # Longitudinal acceleration a_x = d(v_x)/dt - r * v_y
        dvx_dt = np.gradient(operating_history.vx, operating_history.time)
        ax_op_g = (dvx_dt - operating_history.r * operating_history.vy) / g
        op_points = (ay_op_g, ax_op_g)

    return GGDiagramResult(
        mu=mu,
        ay_limit_positive_g=ay_max_g,
        ay_limit_negative_g=-ay_max_g,
        ax_accel_limit_g=ax_power_limit_g,
        ax_brake_limit_g=ax_brake_g,
        boundary_ay_g=ay_boundary,
        boundary_ax_g=ax_boundary,
        simulated_points=op_points,
    )


def plot_handling_and_limits(
    results: Optional[Dict[str, HandlingCharacteristics]] = None,
    gg_result: Optional[GGDiagramResult] = None,
    output_path: str | Path = "output/handling_limits/steady_state_handling_characteristics.png",
    title: str = "ISO 4138 Steady-State Circular Driving & Handling Limits",
    handling_results: Optional[Dict[str, HandlingCharacteristics]] = None,
    histories: Optional[Dict[str, Any]] = None,
    save_path: Optional[str | Path] = None,
) -> Path:
    """Generate publication-quality 4-panel handling limits diagnostic dashboard.

    Panels:
    1. Steer Angle vs Lateral Acceleration (delta_f vs a_y with Ackermann line).
    2. Local Understeer Gradient d(delta_f)/d(a_y) vs Lateral Acceleration.
    3. Body Sideslip Angle |beta| vs Lateral Acceleration.
    4. G-G Acceleration Diagram (Friction Circle Envelope & Operating Bounds).

    Args:
        results: Dict mapping controller/chassis label to HandlingCharacteristics.
        gg_result: Optional GGDiagramResult for Panel 4.
        output_path: Destination file path for PNG image.
        title: Overall dashboard title.
        handling_results: Alias for results.
        histories: Optional dictionary of SimulationHistory.
        save_path: Alias for output_path.

    Returns:
        Path to saved figure.
    """
    if results is None and handling_results is not None:
        results = handling_results
    if results is None:
        raise ValueError("results or handling_results dictionary must be provided to plot_handling_and_limits.")

    if save_path is not None:
        output_path = save_path

    out_p = Path(output_path)
    out_p.parent.mkdir(parents=True, exist_ok=True)

    from vehicle_model.visualization.theme import (
        AXES_COLOR,
        BG_COLOR,
        CARD_COLOR,
        GRID_COLOR,
        TEXT_MUTED,
        TEXT_PRIMARY,
        apply_dark_theme,
    )
    apply_dark_theme()

    fig, axs = plt.subplots(2, 2, figsize=(14, 10), dpi=140, facecolor=BG_COLOR)
    fig.suptitle(title, fontsize=14, fontweight="bold", y=0.98, color=TEXT_PRIMARY)

    for ax in axs.flat:
        ax.set_facecolor(AXES_COLOR)
        for spine in ax.spines.values():
            spine.set_edgecolor(GRID_COLOR)
            spine.set_linewidth(1.0)
        ax.tick_params(colors=TEXT_MUTED, labelsize=9)
        ax.grid(True, linestyle="--", alpha=0.4, color=GRID_COLOR)

    colors = {
        "passive": "#f43f5e",                  # Rose Red
        "passive_baseline": "#f43f5e",
        "passive_staggered": "#f59e0b",        # Amber Orange
        "four_wheel_steering": "#38bdf8",      # Cyan
        "4ws": "#38bdf8",
        "ivmc_2ws": "#3b82f6",                 # Blue
        "full_ivmc_4ws": "#10b981",            # Emerald Green
        "integrated_ivmc": "#10b981",          # Emerald Green
        "ivmc": "#10b981",
    }

    # -------------------------------------------------------------------------
    # Panel 1: Steer Angle delta_f vs Lateral Acceleration a_y
    # -------------------------------------------------------------------------
    ax1 = axs[0, 0]
    ax1.set_title("1. Steer Angle vs. Lateral Accel (ISO 4138)", fontsize=11, fontweight="bold", color=TEXT_PRIMARY)
    ax1.set_xlabel("Lateral Acceleration $a_y$ [g]", fontsize=10, color=TEXT_MUTED)
    ax1.set_ylabel("Steer Angle $\\delta_f$ [deg]", fontsize=10, color=TEXT_MUTED)

    # Plot Ackermann neutral steering reference
    first_res = next(iter(results.values()))
    ax1.axhline(
        first_res.delta_ackermann_deg,
        color="#94a3b8",
        linestyle=":",
        lw=1.8,
        label=f"Ackermann (Neutral: {first_res.delta_ackermann_deg:.2f}°)",
    )

    for name, res in results.items():
        col = colors.get(name.lower(), "#38bdf8")
        lbl = name.replace("_", " ").title()
        ax1.plot(
            res.ay_array_g,
            res.delta_f_array_deg,
            color=col,
            lw=2.2,
            label=f"{lbl} Front ($K_{{us}}={res.understeer_gradient_deg_per_g:+.2f}^\\circ/g$)",
        )
        # If vehicle has active rear steering, also plot effective steering angle delta_eff = delta_f - delta_r
        if (
            len(res.delta_eff_array_deg) == len(res.delta_f_array_deg)
            and np.max(np.abs(res.delta_f_array_deg - res.delta_eff_array_deg)) > 0.05
        ):
            ax1.plot(
                res.ay_array_g,
                res.delta_eff_array_deg,
                color=col,
                linestyle="--",
                lw=1.8,
                alpha=0.85,
                label=f"{lbl} Effective 4WS ($K_{{us,eff}}={res.understeer_gradient_eff_deg_per_g:+.2f}^\\circ/g$)",
            )

        # Mark limit handling point a_y,max
        ax1.scatter(
            [res.max_lateral_accel_g],
            [res.delta_f_array_deg[-1]],
            color=col,
            s=50,
            edgecolor=TEXT_PRIMARY,
            zorder=4,
        )

    ax1.legend(loc="upper left", fontsize=7.5, facecolor=CARD_COLOR, edgecolor=GRID_COLOR, labelcolor=TEXT_PRIMARY, framealpha=0.92)

    # -------------------------------------------------------------------------
    # Panel 2: Local Understeer Gradient d(delta_f)/d(a_y) vs a_y
    # -------------------------------------------------------------------------
    ax2 = axs[0, 1]
    ax2.set_title("2. Local Understeer Gradient $K_{us}(a_y)$", fontsize=11, fontweight="bold", color=TEXT_PRIMARY)
    ax2.set_xlabel("Lateral Acceleration $a_y$ [g]", fontsize=10, color=TEXT_MUTED)
    ax2.set_ylabel("Gradient $d\\delta_f / da_y$ [deg/g]", fontsize=10, color=TEXT_MUTED)
    ax2.axhline(0.0, color="#94a3b8", linestyle=":", lw=1.5, label="Neutral Line ($K_{us} = 0$)")

    for name, res in results.items():
        col = colors.get(name.lower(), "#38bdf8")
        lbl = name.replace("_", " ").title()
        ax2.plot(
            res.ay_array_g,
            res.local_understeer_gradient_deg_per_g,
            color=col,
            lw=2.0,
            label=f"{lbl} (Linear: {res.understeer_gradient_deg_per_g:+.2f}°/g)",
        )

    ax2.set_ylim(-2.0, 15.0)
    ax2.legend(loc="upper left", fontsize=8.5, facecolor=CARD_COLOR, edgecolor=GRID_COLOR, labelcolor=TEXT_PRIMARY, framealpha=0.92)

    # -------------------------------------------------------------------------
    # Panel 3: Dynamic Body Sideslip Delta beta vs Lateral Acceleration
    # -------------------------------------------------------------------------
    ax3 = axs[1, 0]
    ax3.set_title(r"3. Dynamic Body Sideslip $\Delta\beta = \beta_{\mathrm{kin}} - \beta$", fontsize=11, fontweight="bold", color=TEXT_PRIMARY)
    ax3.set_xlabel("Lateral Acceleration $a_y$ [g]", fontsize=10, color=TEXT_MUTED)
    ax3.set_ylabel(r"Dynamic Sideslip $\Delta\beta$ [deg]", fontsize=10, color=TEXT_MUTED)
    ax3.axhline(0.0, color="#94a3b8", linestyle=":", lw=1.2, alpha=0.7)

    for name, res in results.items():
        col = colors.get(name.lower(), "#38bdf8")
        lbl = name.replace("_", " ").title()
        dyn_beta = res.beta_dynamic_deg if len(res.beta_dynamic_deg) == len(res.ay_array_g) else res.beta_kinematic_deg - res.beta_array_deg
        ax3.plot(
            res.ay_array_g,
            dyn_beta,
            color=col,
            lw=2.2,
            label=f"{lbl} ($d\\Delta\\beta/da_y = {res.sideslip_gradient_deg_per_g:+.2f}^\\circ/g$)",
        )

    first_r = next(iter(results.values()))
    if hasattr(first_r, "beta_kinematic_deg") and abs(first_r.beta_kinematic_deg) > 0.01:
        ax3.text(
            0.03,
            0.05,
            f"Offset $\\beta_{{kin}} = +l_r/R = {first_r.beta_kinematic_deg:+.2f}^\\circ$ removed to show tire slip",
            transform=ax3.transAxes,
            fontsize=8,
            color=TEXT_MUTED,
            style="italic",
            bbox=dict(boxstyle="round,pad=0.25", facecolor=CARD_COLOR, edgecolor=GRID_COLOR, alpha=0.92),
        )

    ax3.legend(loc="upper left", fontsize=8.5, facecolor=CARD_COLOR, edgecolor=GRID_COLOR, labelcolor=TEXT_PRIMARY, framealpha=0.92)

    # -------------------------------------------------------------------------
    # Panel 4: G-G Diagram / Acceleration Envelope
    # -------------------------------------------------------------------------
    ax4 = axs[1, 1]
    ax4.set_title("4. G-G Acceleration Diagram (Friction Limits)", fontsize=11, fontweight="bold", color=TEXT_PRIMARY)
    ax4.set_xlabel("Lateral Acceleration $a_y$ [g]", fontsize=10, color=TEXT_MUTED)
    ax4.set_ylabel("Longitudinal Acceleration $a_x$ [g]", fontsize=10, color=TEXT_MUTED)

    if gg_result is not None:
        ax4.plot(
            gg_result.boundary_ay_g,
            gg_result.boundary_ax_g,
            color=TEXT_PRIMARY,
            linestyle="--",
            lw=2.0,
            label=f"Kamm Friction Boundary ($\\mu={gg_result.mu:.2f}$)",
        )
        ax4.fill(
            gg_result.boundary_ay_g,
            gg_result.boundary_ax_g,
            color=GRID_COLOR,
            alpha=0.35,
            zorder=0,
        )

        if gg_result.simulated_points is not None:
            ay_pts, ax_pts = gg_result.simulated_points
            ax4.scatter(
                ay_pts,
                ax_pts,
                c=np.sqrt(ay_pts**2 + ax_pts**2),
                cmap="plasma",
                s=12,
                alpha=0.75,
                zorder=2,
                label="Operating Trajectory Points",
            )
    else:
        # Generic circular boundary
        theta = np.linspace(0, 2 * math.pi, 100)
        ax4.plot(np.cos(theta), np.sin(theta), color="#94a3b8", linestyle="--", lw=1.5, label="Friction Circle (1.0g)")

    ax4.set_xlim(-1.25, 1.25)
    ax4.set_ylim(-1.25, 0.75)
    ax4.set_aspect("equal", adjustable="box")
    ax4.legend(loc="lower right", fontsize=8.5, facecolor=CARD_COLOR, edgecolor=GRID_COLOR, labelcolor=TEXT_PRIMARY, framealpha=0.92)

    plt.tight_layout(rect=[0, 0, 1, 0.95])
    fig.savefig(out_p, bbox_inches="tight", facecolor=fig.get_facecolor(), edgecolor="none")
    plt.close(fig)
    return out_p


__all__ = (
    "HandlingCharacteristics",
    "GGDiagramResult",
    "simulate_steady_state_circular_drive",
    "evaluate_understeer_gradient",
    "compute_gg_diagram",
    "plot_handling_and_limits",
)
