"""Integrated Vehicle Motion Controller (IVMC) coordinating 4WS, TV, ESC, ABS, and TCS.

Two actuation paths (``actuation``, default "allocation" since V3.0):

    "legacy"      the original additive chain: driver torques, TV adds a differential
                  torque (``TorqueVectoringController.allocate_torques``), emergency
                  deceleration, legacy TCS/ABS, heuristic split-mu brake limiter (GMR).
    "allocation"  the shared chain of Plan V3.0 M10 5.3: the longitudinal force demand
                  (pedals, minus the phase-plane emergency deceleration) and the yaw
                  moment demand (TV + phase plane) go to
                  :class:`~vehicle_model.controllers.motion_control.ActuationLayer`
                  (bounded wheel-force allocation, then ABS/TCS slip control). The
                  allocation can *reduce* brake torque on one side; the additive chain
                  cannot. The weight of the yaw moment against the longitudinal force
                  (``yaw_priority``) is scheduled over speed: 100 above
                  ``yaw_priority_speeds[1]`` (select-low-like, the car stays straight under
                  split-mu braking), 1 below ``yaw_priority_speeds[0]`` (traction and braking
                  force first; the asymmetric yaw moment is small in consequence at low speed
                  and the steering can balance it), log-linear in between. Fixed priorities
                  fail one case: 100 gives 19.5 km/h after a 2.5 s split-mu launch (passive
                  31), 1 lets the car yaw 19 deg under split-mu braking from 100 km/h.

Steering (4WS look-up/schedule, counter-steer, phase-plane supervision) is the same in
both paths.
"""

from __future__ import annotations

import dataclasses
import math
from dataclasses import dataclass
import numpy as np

import time
from vehicle_model.controllers.abs import AntilockBrakingSystem
from vehicle_model.controllers.framework import INFORMATION_FULL_STATE, ControlOutput, ControlTarget
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

    information = INFORMATION_FULL_STATE

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
        actuation: str = "allocation",
        yaw_priority_speeds: tuple[float, float] = (8.0, 15.0),   # [m/s] low / high end of the schedule
        yaw_priority_range: tuple[float, float] = (1.0, 100.0),   # [-] priority at low / high speed
    ):
        if actuation not in ("legacy", "allocation"):
            raise ValueError(f"actuation must be 'legacy' or 'allocation', got {actuation!r}")
        self.actuation = actuation
        self.yaw_priority_speeds = tuple(map(float, yaw_priority_speeds))
        self.yaw_priority_range = tuple(map(float, yaw_priority_range))
        if not (0.0 <= self.yaw_priority_speeds[0] < self.yaw_priority_speeds[1]) or min(self.yaw_priority_range) <= 0.0:
            raise ValueError("yaw_priority_speeds must be 0 <= low < high and yaw_priority_range positive")
        self.actuation_layer = None
        if actuation == "allocation":
            from vehicle_model.allocation import AllocatorConfig
            from vehicle_model.controllers.motion_control import ActuationConfig, ActuationLayer
            # ABS and TCS are one slip controller in the shared layer: on if either is enabled
            self.actuation_layer = ActuationLayer(params, ActuationConfig(
                allocator=AllocatorConfig(max_drive_torque=params.max_drive_torque,
                                          max_brake_torque=params.max_brake_torque),
                enable_slip_control=enable_abs or enable_tcs))
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
        actuation: str | None = None,
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
            actuation=str(data.get("actuation", "allocation")) if actuation is None else actuation,
        )

    @property
    def name(self) -> str:
        rws_tag = "4WS + " if self.enable_4ws else "2WS + "
        alloc = ", allocation" if self.actuation == "allocation" else ""
        return f"Integrated Vehicle Motion Controller (IVMC: {rws_tag}TV + ABS + TCS{alloc})"

    def reset(self) -> None:
        """Reset internal controller and subsystem states."""
        self.tv.reset()
        if self.actuation_layer is not None:
            self.actuation_layer.reset()

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

        if self.actuation == "allocation":
            return self._actuate_allocated(state, telemetry, dt, mu_est, r_des, pp_eval, delta_f, delta_r,
                                           T_drive, T_brake, t0)

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

    def yaw_priority(self, vx: float) -> float:
        """Allocator weight of M_z against F_x at speed ``vx`` (log-linear between the schedule ends)."""
        lo, hi = self.yaw_priority_speeds
        p_lo, p_hi = self.yaw_priority_range
        f = min(max((abs(vx) - lo) / (hi - lo), 0.0), 1.0)
        return float(math.exp(math.log(p_lo) + f * (math.log(p_hi) - math.log(p_lo))))

    def _actuate_allocated(self, state, telemetry, dt, mu_est, r_des, pp_eval, delta_f, delta_r,
                           T_drive_driver, T_brake_driver, t0) -> ControlOutput:
        """Shared actuation chain: (F_x, M_z) demand -> bounded allocation -> ABS/TCS."""
        from vehicle_model.vehicle_models.normal_loads import quasi_static_normal_loads

        p = self.params
        Fx_des = float(np.sum(T_drive_driver) - np.sum(T_brake_driver)) / p.Reff
        if pp_eval is not None and pp_eval.emergency_decel_demand_mps2 > 0.0:
            Fx_des -= p.mass * pp_eval.emergency_decel_demand_mps2
        Mz_des = 0.0
        if self.enable_tv:
            Mz_des = self.tv.compute_corrective_moment(state.r, r_des, dt=dt)
            if pp_eval is not None:
                Mz_des += pp_eval.stabilizing_yaw_moment_nm
        if telemetry is not None:
            Fz, mu, Fy, wheel_steer = telemetry.Fz, telemetry.mu, telemetry.Fy_wheel, telemetry.steer_angles
        else:
            Fz = quasi_static_normal_loads(p, 0.0, 0.0).Fz
            mu, Fy = np.full(4, max(mu_est, 0.05)), np.zeros(4)
            wheel_steer = np.array([delta_f, delta_f, delta_r, delta_r])
        alloc_cfg = self.actuation_layer.allocator.config
        self.actuation_layer.allocator.config = dataclasses.replace(alloc_cfg, yaw_priority=self.yaw_priority(state.vx))
        T_drive, T_brake, alloc, Mz_achieved, slip_active = self.actuation_layer.actuate(
            Fx_des, Mz_des, wheel_steer, Fz, mu, Fy, telemetry, state.vx, dt)
        status = "saturated" if (alloc.status != "infeasible" and slip_active) else alloc.status
        return ControlOutput(
            steer_f=delta_f,
            steer_r=delta_r,
            T_drive=T_drive,
            T_brake=T_brake,
            desired_yaw_moment=Mz_des,
            allocated_yaw_moment=Mz_achieved,
            status=status,
            computation_time_ms=(time.perf_counter() - t0) * 1000.0,
            diagnostics={"actuation": "allocation", "Fx_demand": Fx_des, "allocation_status": alloc.status,
                         "allocation_achieved": alloc.achieved.tolist(), "at_bound": alloc.at_bound.tolist(),
                         "grip_limited": alloc.grip_limited.tolist(),
                         "yaw_priority": self.actuation_layer.allocator.config.yaw_priority},
        )
