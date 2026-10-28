"""Scenario simulation engine for the 7-DOF vehicle dynamics model."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, field
import math
import numpy as np

from vehicle_model.core.types import (
    SevenDOFControlInput,
    SevenDOFParams,
    SevenDOFState,
    WheelTelemetry,
)
from vehicle_model.roads.surface import RoadSurface, UniformRoad
from vehicle_model.vehicle_models.seven_dof import SevenDOFModel


@dataclass
class SimulationHistory:
    """Recorded time-series history from a scenario run."""

    time: np.ndarray
    vx: np.ndarray
    vy: np.ndarray
    r: np.ndarray
    omega: np.ndarray        # Shape (N, 4)
    X: np.ndarray
    Y: np.ndarray
    psi: np.ndarray
    steer_f: np.ndarray
    steer_r: np.ndarray
    T_drive: np.ndarray      # Shape (N, 4)
    T_brake: np.ndarray      # Shape (N, 4)
    kappa: np.ndarray        # Shape (N, 4)
    alpha: np.ndarray        # Shape (N, 4)
    Fz: np.ndarray           # Shape (N, 4)
    mu: np.ndarray           # Shape (N, 4)
    Fx_wheel: np.ndarray     # Shape (N, 4)
    Fy_wheel: np.ndarray     # Shape (N, 4)
    Fx_body: np.ndarray      # Shape (N, 4)
    Fy_body: np.ndarray      # Shape (N, 4)

    @property
    def sideslip_angle(self) -> np.ndarray:
        """Vehicle body slip angle beta(t) [rad]."""
        return np.arctan2(self.vy, np.maximum(np.abs(self.vx), 0.1))

    @property
    def speed(self) -> np.ndarray:
        """Total planar speed magnitude [m/s]."""
        return np.hypot(self.vx, self.vy)

    @property
    def normal_load_sum(self) -> np.ndarray:
        """Total vertical force across all 4 wheels [N]."""
        return np.sum(self.Fz, axis=1)

    @property
    def friction_utilization(self) -> np.ndarray:
        """Normalized friction utilization ratio sqrt(Fx^2 + Fy^2) / (mu * Fz). Shape (N, 4)."""
        resultant = np.hypot(self.Fx_wheel, self.Fy_wheel)
        limit = np.maximum(self.mu * self.Fz, 1e-3)
        return resultant / limit


class ScenarioRunner:
    """Executes closed-loop or open-loop driving scenarios with the 7-DOF model."""

    def __init__(
        self,
        model: SevenDOFModel | None = None,
        road: RoadSurface | None = None,
    ):
        self.model = model or SevenDOFModel()
        self.road = road or UniformRoad(mu=1.0)

    def run(
        self,
        controller: Callable[[float, SevenDOFState, WheelTelemetry | None], SevenDOFControlInput],
        init_state: SevenDOFState,
        duration: float,
        dt: float = 0.005,
    ) -> SimulationHistory:
        """Simulate the vehicle scenario from t=0 to t=duration with time step dt."""
        n_steps = int(math.ceil(duration / dt))
        time_arr = np.arange(n_steps) * dt

        # Preallocate history buffers
        vx = np.zeros(n_steps)
        vy = np.zeros(n_steps)
        r = np.zeros(n_steps)
        omega = np.zeros((n_steps, 4))
        X = np.zeros(n_steps)
        Y = np.zeros(n_steps)
        psi = np.zeros(n_steps)
        steer_f = np.zeros(n_steps)
        steer_r = np.zeros(n_steps)
        T_drive = np.zeros((n_steps, 4))
        T_brake = np.zeros((n_steps, 4))
        kappa = np.zeros((n_steps, 4))
        alpha = np.zeros((n_steps, 4))
        Fz = np.zeros((n_steps, 4))
        mu = np.zeros((n_steps, 4))
        Fx_wheel = np.zeros((n_steps, 4))
        Fy_wheel = np.zeros((n_steps, 4))
        Fx_body = np.zeros((n_steps, 4))
        Fy_body = np.zeros((n_steps, 4))

        current_state = init_state
        last_telemetry: WheelTelemetry | None = None

        for k in range(n_steps):
            t = time_arr[k]

            # Query controller
            u = controller(t, current_state, last_telemetry)
            u_clamped = u.clamped(self.model.params)

            # Advance vehicle state
            next_state, telemetry = self.model.step(current_state, u_clamped, dt, road=self.road)
            last_telemetry = telemetry

            # Record state and telemetry
            vx[k] = current_state.vx
            vy[k] = current_state.vy
            r[k] = current_state.r
            omega[k, :] = current_state.omega
            X[k] = current_state.X
            Y[k] = current_state.Y
            psi[k] = current_state.psi

            steer_f[k] = u_clamped.steer_f
            steer_r[k] = u_clamped.steer_r
            T_drive[k, :] = u_clamped.T_drive
            T_brake[k, :] = u_clamped.T_brake

            kappa[k, :] = telemetry.kappa
            alpha[k, :] = telemetry.alpha
            Fz[k, :] = telemetry.Fz
            mu[k, :] = telemetry.mu
            Fx_wheel[k, :] = telemetry.Fx_wheel
            Fy_wheel[k, :] = telemetry.Fy_wheel
            Fx_body[k, :] = telemetry.Fx_body
            Fy_body[k, :] = telemetry.Fy_body

            current_state = next_state

        return SimulationHistory(
            time=time_arr,
            vx=vx,
            vy=vy,
            r=r,
            omega=omega,
            X=X,
            Y=Y,
            psi=psi,
            steer_f=steer_f,
            steer_r=steer_r,
            T_drive=T_drive,
            T_brake=T_brake,
            kappa=kappa,
            alpha=alpha,
            Fz=Fz,
            mu=mu,
            Fx_wheel=Fx_wheel,
            Fy_wheel=Fy_wheel,
            Fx_body=Fx_body,
            Fy_body=Fy_body,
        )


def simulate_maneuver(
    model: SevenDOFModel,
    controller: Any,
    v_initial: float,
    delta_driver: np.ndarray,
    T_drive_demand: float | np.ndarray = 0.0,
    T_brake_demand: float | np.ndarray = 0.0,
    duration: float = 4.0,
    dt: float = 0.005,
    mu: float | np.ndarray | RoadSurface = 1.0,
) -> SimulationHistory:
    """Run an automated maneuver with the 7-DOF model and any VehicleController or callable."""
    from vehicle_model.controllers import ControlOutput, ControlTarget
    from vehicle_model.roads import MuSplitRoad, UniformRoad

    if isinstance(mu, RoadSurface):
        road = mu
    elif isinstance(mu, (list, tuple, np.ndarray)) and len(mu) >= 2:
        mu_left = float(mu[0])
        mu_right = float(mu[1])
        road = MuSplitRoad(mu_left=mu_left, mu_right=mu_right, split_y=0.0)
    else:
        road = UniformRoad(mu=float(mu))

    omega_init = np.full(4, v_initial / model.params.Reff)
    init_state = SevenDOFState(vx=v_initial, vy=0.0, r=0.0, omega=omega_init)


    n_steps = int(math.ceil(duration / dt))
    delta_arr = np.asarray(delta_driver, dtype=float)
    if delta_arr.size < n_steps:
        padded = np.zeros(n_steps)
        padded[:delta_arr.size] = delta_arr
        delta_arr = padded

    if isinstance(T_drive_demand, np.ndarray):
        t_drive_arr = np.asarray(T_drive_demand, dtype=float)
        if t_drive_arr.size < n_steps:
            pad = np.zeros(n_steps)
            pad[:t_drive_arr.size] = t_drive_arr
            t_drive_arr = pad
    else:
        t_drive_arr = np.full(n_steps, float(T_drive_demand))

    if isinstance(T_brake_demand, np.ndarray):
        t_brake_arr = np.asarray(T_brake_demand, dtype=float)
        if t_brake_arr.size < n_steps:
            pad = np.zeros(n_steps)
            pad[:t_brake_arr.size] = t_brake_arr
            t_brake_arr = pad
    else:
        t_brake_arr = np.full(n_steps, float(T_brake_demand))

    max_tot_drive = max(1.0, 4.0 * model.params.max_drive_torque)
    max_tot_brake = max(1.0, 4.0 * model.params.max_brake_torque)

    def ctrl_callback(t: float, state: SevenDOFState, last_telem: WheelTelemetry | None) -> SevenDOFControlInput:
        k = min(int(t / dt), n_steps - 1)
        steer_cmd = float(delta_arr[k])
        p_accel = float(np.clip(t_drive_arr[k] / max_tot_drive, 0.0, 1.0))
        p_brake = float(np.clip(t_brake_arr[k] / max_tot_brake, 0.0, 1.0))

        if hasattr(controller, "compute_control"):
            target = ControlTarget(
                driver_steer_f=steer_cmd,
                driver_accel_pedal=p_accel,
                driver_brake_pedal=p_brake,
            )
            out: ControlOutput = controller.compute_control(state, target, telemetry=last_telem, dt=dt)
            return SevenDOFControlInput(
                steer_f=out.steer_f,
                steer_r=out.steer_r,
                T_drive=out.T_drive,
                T_brake=out.T_brake,
            )
        else:
            return controller(t, state, last_telem)

    runner = ScenarioRunner(model=model, road=road)
    return runner.run(ctrl_callback, init_state, duration=duration, dt=dt)

