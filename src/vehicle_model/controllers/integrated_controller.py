"""Integrated Vehicle Motion Controller (IVMC) coordinating 4WS, TV, ESC, ABS, and TCS."""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np

import time
from vehicle_model.controllers.abs import AntilockBrakingSystem
from vehicle_model.controllers.framework import ControlOutput, ControlTarget
from vehicle_model.controllers.phase_plane import (
    PhasePlaneConfig,
    PhasePlaneEnvelopeController,
    PhasePlaneEvaluation,
)
from vehicle_model.controllers.tcs import TractionControlSystem
from vehicle_model.controllers.torque_vectoring import TorqueVectoringController
from vehicle_model.core.types import (
    FL,
    FR,
    RL,
    RR,
    SevenDOFControlInput,
    SevenDOFParams,
    SevenDOFState,
    WheelTelemetry,
)
from vehicle_model.steering.four_wheel_steering import FourWheelSteering, SteeringMode


class IntegratedChassisController:
    """Master chassis control coordinator combining all active subsystems (IVMC)."""

    def __init__(
        self,
        params: SevenDOFParams,
        steering: FourWheelSteering | None = None,
        tv: TorqueVectoringController | None = None,
        abs_sys: AntilockBrakingSystem | None = None,
        tcs_sys: TractionControlSystem | None = None,
        phase_plane: PhasePlaneEnvelopeController | None = None,
        enable_4ws: bool = True,
        enable_tv: bool = True,
        enable_abs: bool = True,
        enable_tcs: bool = True,
        enable_phase_plane: bool = True,
        k_yaw_counter_steer: float = 0.12,  # Active steering counter-steer gain [rad / (rad/s)]
    ):
        self.params = params
        self.steering = steering or FourWheelSteering()
        self.tv = tv or TorqueVectoringController(params=params)
        self.abs = abs_sys or AntilockBrakingSystem()
        self.tcs = tcs_sys or TractionControlSystem()
        self.phase_plane = phase_plane or PhasePlaneEnvelopeController()

        self.enable_4ws = enable_4ws
        self.enable_tv = enable_tv
        self.enable_abs = enable_abs
        self.enable_tcs = enable_tcs
        self.enable_phase_plane = enable_phase_plane
        self.k_yaw_counter_steer = k_yaw_counter_steer

    @classmethod
    def from_yaml(
        cls,
        path: str | None = None,
        params: SevenDOFParams | None = None,
        enable_4ws: bool | None = None,
        enable_tv: bool | None = None,
        enable_abs: bool | None = None,
        enable_tcs: bool | None = None,
        enable_phase_plane: bool | None = None,
    ) -> IntegratedChassisController:
        """Instantiate an IntegratedChassisController configured from YAML."""
        from pathlib import Path
        import yaml
        from vehicle_model.controllers.abs import ABSConfig
        from vehicle_model.controllers.tcs import TCSConfig
        from vehicle_model.controllers.torque_vectoring import TorqueVectoringConfig
        from vehicle_model.steering.four_wheel_steering import SteeringConfig

        if path is None:
            from vehicle_model.config.loader import get_configs_root
            p = get_configs_root() / "controllers" / "integrated_controller.yaml"
        else:
            p = Path(path)

        with open(p, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}

        subs = data.get("subsystems_enabled", {})
        tv_cfg = data.get("torque_vectoring", {})
        stab_cfg = data.get("stability_control", {})

        veh_params = params or SevenDOFParams.from_yaml()
        steering = FourWheelSteering(SteeringConfig.from_yaml())
        tv = TorqueVectoringController(
            params=veh_params,
            config=TorqueVectoringConfig.from_yaml(p),
        )
        abs_sys = AntilockBrakingSystem(ABSConfig.from_yaml())
        tcs_sys = TractionControlSystem(TCSConfig.from_yaml())
        phase_plane = PhasePlaneEnvelopeController(config=PhasePlaneConfig.from_yaml(p))

        en_4ws = bool(subs.get("enable_4ws", True)) if enable_4ws is None else bool(enable_4ws)
        en_tv = bool(subs.get("enable_tv", True)) if enable_tv is None else bool(enable_tv)
        en_abs = bool(subs.get("enable_abs", True)) if enable_abs is None else bool(enable_abs)
        en_tcs = bool(subs.get("enable_tcs", True)) if enable_tcs is None else bool(enable_tcs)
        en_pp = bool(subs.get("enable_phase_plane", True)) if enable_phase_plane is None else bool(enable_phase_plane)

        return cls(
            params=veh_params,
            steering=steering,
            tv=tv,
            abs_sys=abs_sys,
            tcs_sys=tcs_sys,
            phase_plane=phase_plane,
            enable_4ws=en_4ws,
            enable_tv=en_tv,
            enable_abs=en_abs,
            enable_tcs=en_tcs,
            enable_phase_plane=en_pp,
            k_yaw_counter_steer=float(stab_cfg.get("k_yaw_counter_steer", 0.12)),
        )

    @property
    def name(self) -> str:
        rws_tag = "4WS + " if self.enable_4ws else "2WS + "
        return f"Integrated Vehicle Motion Controller (IVMC: {rws_tag}TV + ABS + TCS)"

    def reset(self) -> None:
        """Reset internal controller and subsystem states."""
        self.tv.reset()

    def compute_control(
        self,
        *args,
        **kwargs,
    ) -> ControlOutput:
        """Execute full integrated chassis control arbitration and allocation."""
        t0 = time.perf_counter()

        # Handle unified VehicleController signature: (state, target, telemetry=..., dt=...)
        if (
            len(args) >= 2
            and isinstance(args[0], SevenDOFState)
            and isinstance(args[1], ControlTarget)
        ) or ("target" in kwargs and isinstance(kwargs["target"], ControlTarget)):
            state = args[0] if len(args) > 0 else kwargs["state"]
            target = args[1] if len(args) > 1 else kwargs["target"]
            telemetry = args[2] if len(args) > 2 else kwargs.get("telemetry", None)
            dt = args[3] if len(args) > 3 else kwargs.get("dt", 0.01)

            driver_steer_f = target.driver_steer_f
            driver_T_drive = np.full(4, target.driver_accel_pedal * self.params.max_drive_torque, dtype=float)
            driver_T_brake = np.full(4, target.driver_brake_pedal * self.params.max_brake_torque, dtype=float)
            mu_est = target.mu_estimated
        else:
            # Handle direct/legacy signature: (t, state, driver_steer_f, driver_T_drive, driver_T_brake, ...)
            if "state" in kwargs:
                state = kwargs["state"]
            elif len(args) > 1 and isinstance(args[1], SevenDOFState):
                state = args[1]
            elif len(args) > 0 and isinstance(args[0], SevenDOFState):
                state = args[0]
            else:
                raise ValueError("No SevenDOFState provided to compute_control")

            driver_steer_f = kwargs.get("driver_steer_f", args[2] if len(args) > 2 else 0.0)
            driver_T_drive = kwargs.get("driver_T_drive", args[3] if len(args) > 3 else np.zeros(4))
            driver_T_brake = kwargs.get("driver_T_brake", args[4] if len(args) > 4 else np.zeros(4))
            telemetry = kwargs.get("telemetry", args[5] if len(args) > 5 else None)
            dt = kwargs.get("dt", args[6] if len(args) > 6 else 0.01)
            mu_est = kwargs.get("mu_est", args[7] if len(args) > 7 else 1.0)

        # Compute reference yaw rate from driver intent
        r_des = self.tv.compute_desired_yaw_rate(driver_steer_f, state.vx, mu_est=mu_est)

        # Phase-Plane (beta, beta_dot) Envelope Stability Supervision
        pp_eval: PhasePlaneEvaluation | None = None
        if self.enable_phase_plane:
            ay_body = float(telemetry.ay) if (telemetry is not None and hasattr(telemetry, "ay")) else float(state.vx * state.r)
            ax_body = float(telemetry.ax) if (telemetry is not None and hasattr(telemetry, "ax")) else 0.0
            pp_eval = self.phase_plane.evaluate(
                vx=state.vx,
                vy=state.vy,
                r=state.r,
                ay=ay_body,
                mu=mu_est,
                ax=ax_body,
            )

        # 1. Active Steering Coordination with closed-loop yaw stabilization
        steer_f_cmd = driver_steer_f
        if self.enable_tv or self.enable_4ws:
            # Active counter-steer intervention to damp involuntary yaw perturbations
            r_error = state.r - r_des
            counter_steer = -self.k_yaw_counter_steer * r_error
            if pp_eval is not None:
                counter_steer += pp_eval.active_counter_steer_rad
            steer_f_cmd += float(np.clip(counter_steer, -0.30, 0.30))

        steer_mode = SteeringMode.SPEED_SCHEDULED if self.enable_4ws else SteeringMode.FRONT_ONLY
        delta_f, delta_r = self.steering.compute_steering_angles(
            cmd_steer_f=steer_f_cmd,
            vx=state.vx,
            vy=state.vy,
            mode=steer_mode,
            wheelbase=self.params.wheelbase,
            lr=self.params.lr,
        )

        T_drive = np.asarray(driver_T_drive, dtype=float).copy()
        T_brake = np.asarray(driver_T_brake, dtype=float).copy()

        # 2. Torque Vectoring & ESC Direct Yaw Moment Control
        if self.enable_tv:
            Mz_corr = self.tv.compute_corrective_moment(state.r, r_des, dt=dt)
            if pp_eval is not None:
                Mz_corr += pp_eval.stabilizing_yaw_moment_nm
            T_drive, T_brake = self.tv.allocate_torques(T_drive, T_brake, Mz_corr)

        # Emergency speed scrub if phase-plane envelope is critically breached
        if pp_eval is not None and pp_eval.emergency_decel_demand_mps2 > 0.0:
            decel_tq = (self.params.mass * pp_eval.emergency_decel_demand_mps2 * self.params.Reff) / 4.0
            T_brake += decel_tq

        # 3. Traction Control System (TCS)
        if self.enable_tcs:
            T_drive = self.tcs.modulate_drive_torques(T_drive, telemetry)

        # 4. Antilock Braking System (ABS)
        if self.enable_abs:
            T_brake = self.abs.modulate_brake_torques(T_brake, telemetry, state.vx)

            # 5. Yaw Moment Build-up Delay (GMR) on mu-split braking:
            # If large asymmetric yaw perturbation persists during braking, limit brake torque difference
            if telemetry is not None and np.any(T_brake > 100.0):
                mu_left = 0.5 * (telemetry.mu[FL] + telemetry.mu[RL])
                mu_right = 0.5 * (telemetry.mu[FR] + telemetry.mu[RR])
                if abs(mu_left - mu_right) > 0.3:
                    # Limit brake torque on high-mu side to prevent overpowering stabilizing forces
                    if mu_left > mu_right:
                        max_left_T = 1.6 * max(T_brake[FR], T_brake[RR]) + 200.0
                        T_brake[FL] = min(T_brake[FL], max_left_T)
                        T_brake[RL] = min(T_brake[RL], max_left_T)
                    else:
                        max_right_T = 1.6 * max(T_brake[FL], T_brake[RL]) + 200.0
                        T_brake[FR] = min(T_brake[FR], max_right_T)
                        T_brake[RR] = min(T_brake[RR], max_right_T)

        t_elapsed = (time.perf_counter() - t0) * 1000.0
        return ControlOutput(
            steer_f=delta_f,
            steer_r=delta_r,
            T_drive=T_drive,
            T_brake=T_brake,
            computation_time_ms=t_elapsed,
        )
