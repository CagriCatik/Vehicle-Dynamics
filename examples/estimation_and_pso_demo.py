"""Demonstration: State Estimation (EKF, RLS Friction) and Offline PSO Tuning.

Covers:
1. Simulates dynamic slalom maneuver on a varying-friction road using the 7-DOF two-track model.
2. Ingests realistic noisy/biased sensor measurements via SensorSuite.
3. Observers estimate sideslip angle beta, body velocities (vx, vy), and sensor biases in real-time.
4. Estimates road friction coefficient mu using RLS slip-slope & peak utilization.
5. Performs an offline PSO parameter tuning run to optimize controller gains and exports results to YAML.
6. Exports high-resolution diagnostic plots to output/estimation/ and logs to logs/.
"""

from __future__ import annotations

import math
from pathlib import Path
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from vehicle_model.controllers.torque_vectoring import TorqueVectoringConfig, TorqueVectoringController
from vehicle_model.core.types import (
    FL,
    FR,
    RL,
    RR,
    SevenDOFControlInput,
    SevenDOFParams,
    SevenDOFState,
)
from vehicle_model.estimation.ekf_sideslip import EKFConfig, EKFSideslipObserver
from vehicle_model.estimation.friction_estimator import FrictionEstimatorConfig, RoadFrictionEstimator
from vehicle_model.estimation.pso_optimizer import PSOOptimizer
from vehicle_model.estimation.sensors import SensorConfig, SensorSuite
from vehicle_model.logging import setup_logger
from vehicle_model.roads.surface import MuSplitRoad
from vehicle_model.vehicle_models.seven_dof import SevenDOFModel
from vehicle_model.visualization.plan_view import VehicleParams, draw_vehicle_trajectory
from vehicle_model.visualization.theme import (
    ACCENT_AMBER,
    ACCENT_CYAN,
    ACCENT_EMERALD,
    ACCENT_ROSE,
    BG_COLOR,
    CARD_COLOR,
    GRID_COLOR,
    TEXT_MUTED,
    TEXT_PRIMARY,
    apply_dark_theme,
)

apply_dark_theme()

logger = setup_logger("estimation_pso_demo")


def run_estimation_simulation(
    params: SevenDOFParams,
    output_dir: Path,
) -> dict[str, np.ndarray]:
    """Execute forward simulation comparing ground truth to EKF observer & friction estimates."""
    logger.info("Executing 7-DOF dynamic maneuver with simulated noisy IMU and WSS sensors...")
    model = SevenDOFModel(params=params)
    # Split-mu surface: high grip dry asphalt (mu=1.0) on left, lower friction wet (mu=0.35) on right
    road = MuSplitRoad(mu_left=1.0, mu_right=0.35, split_y=1.2, transition_width=0.3)

    # Initialize sensors and estimators
    sensor_cfg = SensorConfig.from_yaml()
    sensors = SensorSuite(config=sensor_cfg, seed=42)

    ekf = EKFSideslipObserver(params=params)
    friction_est = RoadFrictionEstimator(params=params)

    dt = 0.005
    duration = 4.0
    steps = int(duration / dt)

    v0 = 18.0  # ~65 km/h
    omega0 = v0 / params.Reff
    state = SevenDOFState(
        vx=v0, vy=0.0, r=0.0, omega=np.full(4, omega0), X=0.0, Y=0.0, psi=0.0
    )
    ekf.reset(init_vx=v0, init_vy=0.0, init_r=0.0)
    friction_est.reset(initial_mu=1.0)

    # Data collection arrays
    t_hist = np.zeros(steps)
    true_beta = np.zeros(steps)
    est_beta = np.zeros(steps)
    true_vx = np.zeros(steps)
    est_vx = np.zeros(steps)
    true_r = np.zeros(steps)
    est_r = np.zeros(steps)
    true_ay = np.zeros(steps)
    meas_ay = np.zeros(steps)
    est_bias_ay = np.zeros(steps)
    true_mu = np.zeros(steps)
    est_mu = np.zeros(steps)
    X_hist = np.zeros(steps)
    Y_hist = np.zeros(steps)
    psi_hist = np.zeros(steps)
    steer_f_hist = np.zeros(steps)

    last_telemetry = None

    for k in range(steps):
        t = k * dt
        t_hist[k] = t

        # Driver steering input: Slalom sine steer (frequency 0.8 Hz, amplitude 5.5 deg)
        cmd_steer_f = math.radians(5.5) * math.sin(2.0 * math.pi * 0.8 * t) if t > 0.4 else 0.0
        steer_f_hist[k] = cmd_steer_f

        # Driver drive torque (maintain speed)
        T_drive = np.full(4, 80.0)
        u = SevenDOFControlInput(steer_f=cmd_steer_f, steer_r=0.0, T_drive=T_drive)

        # Simulation step
        state, last_telemetry = model.step(state, u, dt, road=road)
        _, _, ax_true, ay_true = model.evaluate_dynamics(state, u, road=road)

        # Synthesize noisy onboard measurements
        meas = sensors.measure(
            t=t,
            dt=dt,
            true_ax=ax_true,
            true_ay=ay_true,
            true_r=state.r,
            true_omega=state.omega,
            true_steer_f=cmd_steer_f,
        )

        # Update EKF sideslip observer
        est_state = ekf.update(meas, dt=dt)

        # Update RLS road friction estimator
        mu_res = friction_est.update(last_telemetry, vx=state.vx)

        # Record history
        true_beta[k] = state.sideslip_angle
        est_beta[k] = est_state.sideslip
        true_vx[k] = state.vx
        est_vx[k] = est_state.vx
        true_r[k] = state.r
        est_r[k] = est_state.r
        true_ay[k] = ay_true
        meas_ay[k] = meas.ay
        est_bias_ay[k] = est_state.bias_ay
        true_mu[k] = float(np.mean(last_telemetry.mu))
        est_mu[k] = mu_res.mu_overall
        X_hist[k] = state.X
        Y_hist[k] = state.Y
        psi_hist[k] = state.psi

    # Calculate observer RMSE metrics
    settled = int(0.5 / dt)
    beta_error_deg = math.degrees(math.sqrt(float(np.mean((est_beta[settled:] - true_beta[settled:]) ** 2))))
    vx_error_mps = math.sqrt(float(np.mean((est_vx[settled:] - true_vx[settled:]) ** 2)))
    mu_error = math.sqrt(float(np.mean((est_mu[settled:] - true_mu[settled:]) ** 2)))

    logger.info(
        f"Observer Accuracy (post-settling):\n"
        f"  - Sideslip Angle (beta) RMSE: {beta_error_deg:.3f} deg\n"
        f"  - Longitudinal Speed (vx) RMSE: {vx_error_mps:.3f} m/s\n"
        f"  - Road Friction (mu) RMSE:     {mu_error:.3f}"
    )

    return {
        "t": t_hist,
        "true_beta": true_beta,
        "est_beta": est_beta,
        "true_vx": true_vx,
        "est_vx": est_vx,
        "true_r": true_r,
        "est_r": est_r,
        "true_ay": true_ay,
        "meas_ay": meas_ay,
        "est_bias_ay": est_bias_ay,
        "true_mu": true_mu,
        "est_mu": est_mu,
        "X": X_hist,
        "Y": Y_hist,
        "psi": psi_hist,
        "steer_f": steer_f_hist,
    }


def run_pso_tuning_pass(
    params: SevenDOFParams,
    output_dir: Path,
) -> tuple[dict[str, float], list[float]]:
    """Execute offline PSO tuning pass optimizing Torque Vectoring gains."""
    logger.info("Executing offline Particle Swarm Optimization (PSO) for TV controller gains...")

    # Tuning parameters: [kp_yaw, kd_yaw]
    # Parameter bounds: kp in [1500, 6000] Nm/(rad/s), kd in [100, 600] Nm/(rad/s^2)
    bounds = [(1500.0, 6000.0), (100.0, 600.0)]
    param_names = ["kp_yaw", "kd_yaw"]

    # Target objective: Minimize integrated yaw rate tracking error + penalty on sideslip
    def evaluate_controller_cost(gains: np.ndarray) -> float:
        kp, kd = gains[0], gains[1]
        model = SevenDOFModel(params=params)
        road = MuSplitRoad(mu_left=1.0, mu_right=0.5, split_y=0.0)
        tv_cfg = TorqueVectoringConfig(kp_yaw=kp, kd_yaw=kd, max_yaw_moment=4000.0)
        tv = TorqueVectoringController(params=params, config=tv_cfg)

        state = SevenDOFState(
            vx=20.0, vy=0.0, r=0.0, omega=np.full(4, 20.0 / params.Reff)
        )
        dt = 0.01
        steps = 150  # 1.5s rapid evaluation
        cost = 0.0

        for k in range(steps):
            t = k * dt
            cmd_steer = math.radians(6.0) * math.sin(2.0 * math.pi * 0.8 * t)
            # Reference yaw rate & corrective moment
            r_des = tv.compute_desired_yaw_rate(cmd_steer, state.vx)
            Mz_corr = tv.compute_corrective_moment(state.r, r_des, dt=dt)
            # Allocation across 4 wheels
            t_drv, t_brk = tv.allocate_torques(np.full(4, 100.0), np.zeros(4), Mz_corr)
            u = SevenDOFControlInput(steer_f=cmd_steer, T_drive=t_drv, T_brake=t_brk)
            state, tele = model.step(state, u, dt, road=road)

            yaw_err = state.r - r_des
            sideslip = state.sideslip_angle
            cost += float(yaw_err ** 2 + 2.5 * sideslip ** 2) * dt

        return float(cost)

    pso = PSOOptimizer(
        cost_func=evaluate_controller_cost,
        bounds=bounds,
        param_names=param_names,
        n_particles=15,
        n_iterations=18,
        seed=42,
    )

    result = pso.optimize(verbose=False)

    yaml_target = Path("output/estimation/pso_optimized_tv.yaml")   # generated result, not a tracked config
    result.export_to_yaml(
        yaml_target,
        objective_name="yaw_rate_and_sideslip_minimization",
        extra_metadata={"vehicle_mass": params.mass, "target_maneuver": "slalom_split_mu"},
    )
    logger.info(f"PSO Optimization converged in {result.convergence_time_s:.2f}s with best cost: {result.best_cost:.4f}")
    logger.info(f"Optimized Parameters exported to: {yaml_target}")

    return result.best_params, result.history_best_cost


def plot_results(
    sim_data: dict[str, np.ndarray],
    pso_cost_hist: list[float],
    save_path: Path,
) -> None:
    """Generate high-resolution 5-panel validation telemetry plot."""
    t = sim_data["t"]
    fig = plt.figure(figsize=(15, 12), facecolor=BG_COLOR)
    gs = fig.add_gridspec(3, 2, hspace=0.35, wspace=0.25)

    # 1. Sideslip Angle beta(t): True vs. EKF Estimate
    ax1 = fig.add_subplot(gs[0, 0])
    ax1.plot(t, np.degrees(sim_data["true_beta"]), color=TEXT_PRIMARY, lw=2.0, label=r"Ground Truth $\beta_{\mathrm{true}}$")
    ax1.plot(t, np.degrees(sim_data["est_beta"]), color=ACCENT_CYAN, linestyle="--", lw=1.8, label=r"EKF Estimated $\hat{\beta}$")
    ax1.set_ylabel("Sideslip Angle [deg]", color=TEXT_MUTED)
    ax1.set_title(r"1. Vehicle Sideslip Angle $\beta(t)$: Ground Truth vs. EKF Observer", color=TEXT_PRIMARY)
    ax1.legend(loc="upper right", fontsize=9, facecolor=CARD_COLOR, edgecolor=GRID_COLOR, labelcolor=TEXT_PRIMARY)

    # 2. Road Friction Coefficient mu(t): True vs. RLS Estimate
    ax2 = fig.add_subplot(gs[0, 1])
    ax2.plot(t, sim_data["true_mu"], color=ACCENT_EMERALD, lw=2.0, label=r"True Surface $\mu$")
    ax2.plot(t, sim_data["est_mu"], color=ACCENT_AMBER, linestyle="-.", lw=1.8, label=r"RLS Estimated $\hat{\mu}$")
    ax2.set_ylabel("Friction Coefficient $\\mu$ [-]", color=TEXT_MUTED)
    ax2.set_title("2. Road Friction Coefficient: Ground Truth vs. RLS Estimator", color=TEXT_PRIMARY)
    ax2.set_ylim([0.0, 1.3])
    ax2.legend(loc="lower left", fontsize=9, facecolor=CARD_COLOR, edgecolor=GRID_COLOR, labelcolor=TEXT_PRIMARY)

    # 3. Lateral Acceleration ay & Estimated Bias
    ax3 = fig.add_subplot(gs[1, 0])
    ax3.plot(t, sim_data["true_ay"], color=TEXT_PRIMARY, lw=1.5, label=r"True $a_y$")
    ax3.plot(t, sim_data["meas_ay"], color=TEXT_MUTED, alpha=0.5, lw=0.8, label=r"Noisy Sensor $a_{y,\mathrm{meas}}$")
    ax3.plot(t, sim_data["est_bias_ay"], color=ACCENT_EMERALD, lw=1.8, label=r"Estimated Bias $\hat{b}_{ay}$")
    ax3.set_ylabel("Acceleration [m/s$^2$]", color=TEXT_MUTED)
    ax3.set_title("3. Lateral Acceleration & IMU Bias Identification", color=TEXT_PRIMARY)
    ax3.legend(loc="upper right", fontsize=9, facecolor=CARD_COLOR, edgecolor=GRID_COLOR, labelcolor=TEXT_PRIMARY)

    # 4. Trajectory & Yaw Dynamics
    ax4 = fig.add_subplot(gs[1, 1])
    ax4.plot(sim_data["X"], sim_data["Y"], color=ACCENT_CYAN, lw=2.0, label="Vehicle Trajectory")
    ax4.axhline(1.2, color=ACCENT_ROSE, ls=":", lw=1.5, label="Split-$\\mu$ Boundary ($Y=1.2$m)")
    ax4.set_xlabel("Global X [m]", color=TEXT_MUTED)
    ax4.set_ylabel("Global Y [m]", color=TEXT_MUTED)
    ax4.set_title("4. Vehicle Path Crossing Split-$\\mu$ Transition", color=TEXT_PRIMARY)
    ax4.legend(loc="upper left", fontsize=9, facecolor=CARD_COLOR, edgecolor=GRID_COLOR, labelcolor=TEXT_PRIMARY)

    # 5. Offline PSO Convergence History
    ax5 = fig.add_subplot(gs[2, :])
    iters = list(range(len(pso_cost_hist)))
    ax5.plot(iters, pso_cost_hist, color=ACCENT_ROSE, marker="o", lw=2.0, markersize=5, label="Best Swarm Fitness")
    ax5.set_xlabel("PSO Iteration", color=TEXT_MUTED)
    ax5.set_ylabel("Cost Function Value $J(\\mathbf{\\theta})$", color=TEXT_MUTED)
    ax5.set_title("5. Offline Particle Swarm Optimization (PSO) Convergence Curve", color=TEXT_PRIMARY)
    ax5.legend(loc="upper right", fontsize=9, facecolor=CARD_COLOR, edgecolor=GRID_COLOR, labelcolor=TEXT_PRIMARY)

    save_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(save_path, dpi=200, bbox_inches="tight", facecolor=fig.get_facecolor(), edgecolor="none")
    plt.close(fig)
    logger.info(f"Diagnostic telemetry plot exported to: {save_path}")


def main() -> None:
    params = SevenDOFParams.from_yaml()
    out_dir = Path("output/estimation")
    out_dir.mkdir(parents=True, exist_ok=True)

    # 1. Forward observer simulation
    sim_data = run_estimation_simulation(params, out_dir)

    # 2. Offline PSO parameter tuning
    best_gains, cost_hist = run_pso_tuning_pass(params, out_dir)

    # 3. Telemetry plot
    plot_file = out_dir / "estimation_and_pso_validation.png"
    plot_results(sim_data, cost_hist, plot_file)


if __name__ == "__main__":
    main()
