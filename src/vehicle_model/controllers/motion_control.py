"""Vehicle motion control chain with exchangeable control laws (Plan.md, Abschnitt 10; M6).

The chain separates (Plan.md 10):

    driver request (steer, pedals)
      -> target motion            YawReference: r_ref from a linear single track with EG_ref
      -> motion control law       MotionLaw: yaw moment and/or rear steer (PI, LQR, MPC, ...)
      -> control allocation       ActuationLayer: WheelForceAllocator (M5)
      -> actuator-level control   ActuationLayer: ABS/TCS slip control (M5)
      -> vehicle physics          plant, outside this module

Every law runs on the same reference and the same actuation layer, so controllers
are compared on equal terms. Laws see the vehicle state and the telemetry
(wheel loads, lateral forces, slips, road friction): this is an idealised
full-state-feedback setting and is declared as such in ``information``.

Actuation trace (Plan V4.0 M16): ``diagnostics["trace"]`` holds the generalised forces
``(F_x [N], M_z [N m])`` at each stage of one control step,

    requested   the law's demand and the pedal force demand (before any limit)
    allocated   the bounded allocation (``AllocationResult.achieved``), before ABS/TCS
    issued      the wheel torques this controller outputs (after ABS/TCS), through the same
                effectiveness matrix

``ControlOutput.allocated_yaw_moment`` is the *issued* yaw moment (name kept for compatibility).
The controller evaluates the effectiveness with the wheel steer angles it knows (plant
telemetry of the previous step); the simulation evaluates *realised* with the steer of the
current command, so without actuator dynamics the two agree up to that one-step steer change.
What the plant receives after actuator dynamics (``realised``) is added outside the
controller (:mod:`vehicle_model.simulation.actuator_chain`).

Brake functions (Plan V4.0 M17, all optional, default off = unchanged behaviour):

    ActuationConfig.distribution   EBD strategy: shares and rear limits for the allocation while braking
    ActuationConfig.ibc            IBC: tyre-derived slip targets for the ABS slip controller
    MotionControllerConfig.cbc     CBC: yaw-moment request while braking in a turn, arbitrated with the
                                   law's request by :func:`arbitrate_yaw` (never summed)

Output status: the law's solver status if it is not nominal (see
solver_diagnostics), otherwise the allocation status (nominal / saturated /
infeasible); both are in the diagnostics. A law that checks its plan against declared
limits (:mod:`.constraints`) reports it as ``diagnostics["feasibility"]``, kept
separate from both statuses.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Protocol, runtime_checkable

import numpy as np

from vehicle_model.allocation import ActuatorHealth, AllocatorConfig, WheelForceAllocator
from vehicle_model.controllers.framework import INFORMATION_FULL_STATE, ControlOutput, ControlTarget
from vehicle_model.controllers.slip_control import SlipControlConfig, WheelSlipController
from vehicle_model.core.types import SevenDOFParams, SevenDOFState, WheelTelemetry
from vehicle_model.vehicle_models.normal_loads import quasi_static_normal_loads, wheel_contact_positions



@dataclass(frozen=True)
class YawReference:
    """Target yaw rate ``r_ref = v delta/(L + EG_ref v^2)``, limited to ``k_mu mu_hat g / v``; target sideslip 0."""

    wheelbase: float
    g: float = 9.81
    understeer_gradient: float = 0.0010   # [rad/(m/s^2)] ~0.56 deg/g
    mu_utilisation: float = 0.85

    def yaw_rate(self, steer: float, vx: float, mu_hat: float) -> float:
        v = max(abs(vx), 0.1)
        r = vx * steer / (self.wheelbase + self.understeer_gradient * vx**2)
        r_max = self.mu_utilisation * mu_hat * self.g / v
        return float(np.clip(r, -r_max, r_max))


@dataclass(frozen=True)
class MotionContext:
    """What a law gets each step (besides the state)."""

    steer_f: float          # [rad] driver front steer
    r_ref: float            # [rad/s]
    mu_hat: float           # [-]
    dt: float               # [s]
    max_yaw_moment: float   # [N m]
    max_rear_steer: float   # [rad]
    max_front_overlay: float = 0.0   # [rad] AFS: largest front-steer overlay (0: no AFS)
    fx_demand: float = 0.0           # [N] the driver's longitudinal demand this step (> 0: traction phase; M24)


@dataclass(frozen=True)
class MotionRequest:
    yaw_moment: float | None = 0.0  # [N m] demand to the allocation; None: no yaw request (yaw left free), not 0
    rear_steer: float = 0.0       # [rad] rear road-wheel angle command
    front_steer: float = 0.0      # [rad] AFS overlay on the driver's front road-wheel angle (Plan V4.0 M19.2)
    status: str = "nominal"       # law/solver status
    diagnostics: dict = field(default_factory=dict)


@runtime_checkable
class MotionLaw(Protocol):
    name: str
    information: str

    def reset(self) -> None: ...

    def request(self, state: SevenDOFState, ctx: MotionContext) -> MotionRequest: ...


@dataclass(frozen=True)
class ActuationConfig:
    allocator: AllocatorConfig = field(default_factory=AllocatorConfig)
    abs_config: SlipControlConfig = field(default_factory=lambda: SlipControlConfig(mode="brake"))
    tcs_config: SlipControlConfig = field(default_factory=lambda: SlipControlConfig(mode="drive"))
    enable_slip_control: bool = True
    distribution: object = None     # BrakeDistributionConfig (EBD) or None: allocation's own grip distribution
    ibc: object = None              # IntegratedBrakeConfig or None: ABS targets from the calibration table
    traction: object = None         # TractionDistributionConfig (open / eLSD / free, motor envelope; M18) or None
    enable_tcs: bool = True         # drive slip control (TCS) on/off independently of ABS (M18 comparisons)


class ActuationLayer:
    """Allocation of (F_x, M_z) to wheel torques plus ABS/TCS (shared by all laws)."""

    def __init__(self, params: SevenDOFParams, config: ActuationConfig | None = None,
                 health: ActuatorHealth | None = None):
        self.params = params
        self.config = config or ActuationConfig()
        a = self.config.allocator
        if a.max_drive_torque > params.max_drive_torque or a.max_brake_torque > params.max_brake_torque:
            raise ValueError("allocator limits exceed the vehicle's actuator limits")
        self.allocator = WheelForceAllocator(wheel_contact_positions(params), params.Reff, a)
        self.health = health or ActuatorHealth()
        abs_cfg = self.config.abs_config
        if self.config.ibc is not None:
            from vehicle_model.controllers.integrated_brake import ibc_slip_config
            abs_cfg = ibc_slip_config(self.config.ibc, abs_cfg)
        self.abs = WheelSlipController(abs_cfg)
        self.traction = None
        if self.config.traction is not None:
            from vehicle_model.controllers.traction_distribution import TractionDistributionStrategy
            self.traction = TractionDistributionStrategy(self.config.traction, params.Reff)
        self.distribution = None
        if self.config.distribution is not None:
            from vehicle_model.controllers.brake_distribution import BrakeDistributionStrategy
            self.distribution = BrakeDistributionStrategy(self.config.distribution)
        self.tcs = WheelSlipController(self.config.tcs_config)
        self.last_allocation = None
        self.last_issued = np.zeros(2)                # (F_x, M_z) of the last issued wheel torques

    def reset(self) -> None:
        self.abs.reset()
        self.tcs.reset()
        if self.distribution is not None:
            self.distribution.reset()
        self.last_allocation = None
        self.last_issued = np.zeros(2)

    def actuate(self, Fx_des: float, Mz_des: float, wheel_steer, Fz, mu, Fy, telemetry, vx, dt, omega=None):
        share = limit = None
        if self.distribution is not None and Fx_des < 0.0:       # EBD acts on braking demands only
            prev = None if self.last_allocation is None else np.maximum(-self.last_allocation.F, 0.0)
            share, limit = self.distribution.distribute(Fz, mu, self.abs.active, prev)
        if Mz_des is None and Fx_des >= 0.0 and share is None:
            # no yaw request while driving: left/right equal per axle (open-differential behaviour); a grip-
            # proportional split would vector torque to the loaded outer wheels (measured: -6 deg sideslip in a
            # steady turn at 80 km/h with the brake-functions-only law)
            g = np.asarray(mu, float) * np.maximum(np.asarray(Fz, float), 1.0)
            share = np.repeat([(g[0] + g[1]) / 2.0, (g[2] + g[3]) / 2.0], 2) / np.sum(g)
        dlim = None
        if self.traction is not None and Fx_des > 0.0:           # drive coupling acts on drive demands only
            upper = self.allocator.bounds(Fz, mu, Fy if Fy is not None else np.zeros(4), self.health)[1]
            dlim = self.traction.drive_limit(upper, omega)
        alloc = self.allocator.allocate(Fx_des, Mz_des, wheel_steer, Fz, mu, Fy, self.health, share=share,
                                        brake_limit=limit, drive_limit=dlim)
        self.last_allocation = alloc
        T_drive, T_brake = alloc.T_drive, alloc.T_brake
        if self.config.enable_slip_control:
            T_brake = self.abs.update(T_brake, telemetry, vx, dt)
            if self.config.enable_tcs:
                T_drive = self.tcs.update(T_drive, telemetry, vx, dt)
        F = (T_drive - T_brake) / self.params.Reff
        self.last_issued = self.allocator.effectiveness(wheel_steer) @ F      # (F_x, M_z) of the issued torques
        achieved_Mz = float(self.last_issued[1])
        slip_active = bool(np.any(self.abs.active) or np.any(self.tcs.active))
        return T_drive, T_brake, alloc, achieved_Mz, slip_active


@dataclass(frozen=True)
class MotionControllerConfig:
    understeer_gradient_ref: float = 0.0010   # [rad/(m/s^2)]
    mu_utilisation: float = 0.85
    max_yaw_moment: float = 4000.0            # [N m]
    min_speed: float = 3.0                    # [m/s] law inactive below
    actuation: ActuationConfig = field(default_factory=ActuationConfig)
    cbc: object = None                        # CorneringBrakeConfig (Plan V4.0 M17.2) or None
    steering: object = None                   # SteeringOverlayConfig (AFS: variable ratio, overlay limits; M19.2) or None
    gvc: object = None                        # GVCConfig (GVC-like load-transfer control, M20) or None
    hdc: object = None                        # HDCConfig (hill descent control, M22) or None


class MotionController:
    """Driver request -> reference -> law -> allocation -> slip control; a VehicleController."""

    def __init__(self, params: SevenDOFParams, law: MotionLaw, config: MotionControllerConfig | None = None,
                 health: ActuatorHealth | None = None):
        self.params = params
        self.law = law
        self.mc_config = config or MotionControllerConfig()
        c = self.mc_config
        self.reference = YawReference(params.wheelbase, params.g, c.understeer_gradient_ref, c.mu_utilisation)
        self.actuation = ActuationLayer(params, c.actuation, health)
        self.steering = None
        if c.steering is not None:
            from vehicle_model.controllers.active_front_steering import SteeringOverlay
            self.steering = SteeringOverlay(c.steering)
        self.gvc = None
        if c.gvc is not None:
            from vehicle_model.controllers.g_vectoring import GVectoringControl
            self.gvc = GVectoringControl(params, c.gvc)
        self.hdc = None
        if c.hdc is not None:
            from vehicle_model.controllers.hill_descent import HillDescentControl
            self.hdc = HillDescentControl(c.hdc)
        self.cbc = None
        if c.cbc is not None:
            from vehicle_model.controllers.cornering_brake import CorneringBrakeControl
            self.cbc = CorneringBrakeControl(params, c.cbc)

    @property
    def name(self) -> str:
        return self.law.name

    @property
    def information(self) -> str:
        return self.law.information

    @property
    def health(self) -> ActuatorHealth:
        return self.actuation.health

    @property
    def last_allocation(self):
        return self.actuation.last_allocation

    def reset(self) -> None:
        self.law.reset()
        self.actuation.reset()
        if self.steering is not None:
            self.steering.reset()
        if self.gvc is not None:
            self.gvc.reset()
        if self.hdc is not None:
            self.hdc.reset()

    def reference_yaw_rate(self, steer: float, vx: float, mu_hat: float) -> float:
        return self.reference.yaw_rate(steer, vx, mu_hat)

    def compute_control(self, state: SevenDOFState, target: ControlTarget,
                        telemetry: WheelTelemetry | None = None, dt: float = 0.01) -> ControlOutput:
        t0 = time.perf_counter()
        p, c = self.params, self.mc_config
        steer = float(np.clip(target.driver_steer_f, -p.max_steer_f, p.max_steer_f))
        if self.steering is not None:          # variable ratio: the driver's effective road-wheel angle (M19.2)
            steer = float(np.clip(steer * self.steering.config.gain(state.vx), -p.max_steer_f, p.max_steer_f))
        if telemetry is not None:
            Fz, mu, Fy, wheel_steer = telemetry.Fz, telemetry.mu, telemetry.Fy_wheel, telemetry.steer_angles
        else:
            Fz = quasi_static_normal_loads(p, 0.0, 0.0).Fz
            mu = np.full(4, max(target.mu_estimated, 0.05))
            Fy, wheel_steer = np.zeros(4), np.array([steer, steer, 0.0, 0.0])
        mu_hat = float(np.mean(mu))
        r_ref = self.reference.yaw_rate(steer, state.vx, mu_hat)

        acfg = c.actuation.allocator
        Fx_des = (target.driver_accel_pedal * sum(acfg.driven) * acfg.max_drive_torque
                  - target.driver_brake_pedal * 4.0 * acfg.max_brake_torque) / p.Reff
        if abs(state.vx) >= c.min_speed:
            ctx = MotionContext(steer, r_ref, mu_hat, dt, c.max_yaw_moment, p.max_steer_r,
                                0.0 if self.steering is None else self.steering.authority, Fx_des)
            req = self.law.request(state, ctx)
        else:
            # law inactive at low speed. Braking to a stop: M_z = 0 (symmetric, no yaw jerk before standstill).
            # Driving off: no yaw request (None). Measured (M18): with M_z = 0 and both motors of one side failed
            # the allocation could not drive at all below min_speed (deadlock); with None while braking, split-mu
            # stops turned by 0.6 deg in the last metres (asymmetric grip-proportional braking).
            self.law.reset()
            req = MotionRequest(yaw_moment=None if Fx_des >= 0.0 else 0.0)
        steer_r = float(np.clip(req.rear_steer, -p.max_steer_r, p.max_steer_r))
        steer_drv = steer
        if self.steering is not None:          # the one place where the front angle is finalised (M19.2)
            g = self.steering.config.gain(state.vx)
            steer, _, overlay = self.steering.finalise(steer / g, state.vx, req.front_steer, p.max_steer_f, dt)
        else:
            overlay = 0.0
        gvc_ax = 0.0
        long_requests = [LongitudinalRequest("driver", "driver", "base", Fx_des)]
        if self.gvc is not None:   # small deceleration on turn-in, added to the longitudinal demand (M20)
            gvc_ax = self.gvc.request(steer, state.vx, target.driver_brake_pedal,
                                      bool(np.any(self.actuation.abs.active)), dt)
            long_requests.append(LongitudinalRequest("gvc", "agility", "add", p.mass * gvc_ax))
        if self.hdc is not None:   # hill descent (M22): the stronger braking of driver and HDC, never a sum
            last = self.actuation.last_allocation     # previous step: braking demand cut at the grip limit?
            grip_cut = last is not None and bool(np.any(last.grip_limited)) and float(last.residual[0]) > 1.0
            hdc_ax = self.hdc.request(state.vx, target.driver_accel_pedal, min(0.0, Fx_des / p.mass),
                                      bool(np.any(self.actuation.abs.active)) or grip_cut, dt)
            long_requests.append(LongitudinalRequest("hdc", "safety", "brake_min", p.mass * hdc_ax))
        Fx_des, long_arb = arbitrate_longitudinal(long_requests)
        law_mz = None if req.yaw_moment is None else float(req.yaw_moment)
        cbc_mz = self.cbc.request(state, r_ref, Fx_des, mu_hat) if (self.cbc is not None and abs(state.vx) >= c.min_speed) else None
        Mz_raw, winner = arbitrate_yaw(law_mz, cbc_mz)
        Mz_des = None if Mz_raw is None else float(np.clip(Mz_raw, -c.max_yaw_moment, c.max_yaw_moment))
        T_drive, T_brake, alloc, achieved_Mz, slip_active = self.actuation.actuate(
            Fx_des, Mz_des, wheel_steer, Fz, mu, Fy, telemetry, state.vx, dt, omega=state.omega)

        status = alloc.status
        if status != "infeasible" and slip_active:
            status = "saturated"
        if req.status != "nominal":
            status = req.status
        return ControlOutput(
            steer_f=steer, steer_r=steer_r, T_drive=T_drive, T_brake=T_brake,
            desired_yaw_moment=0.0 if Mz_des is None else Mz_des, allocated_yaw_moment=achieved_Mz, status=status,
            computation_time_ms=(time.perf_counter() - t0) * 1e3,
            diagnostics={
                "r_ref": r_ref, "Fx_demand": Fx_des, "law_status": req.status, "law": req.diagnostics,
                "allocation_status": alloc.status, "allocation_residual": alloc.residual,
                "at_bound": alloc.at_bound, "grip_limited": alloc.grip_limited,
                "abs_active": self.actuation.abs.active.copy(), "tcs_active": self.actuation.tcs.active.copy(),
                "information": self.law.information,
                "feasibility": req.diagnostics.get("feasibility"),
                "arbitration": {"law": law_mz, "cbc": cbc_mz, "winner": winner},
                "longitudinal_arbitration": long_arb,
                "steering": {"driver": steer_drv, "overlay": overlay, "final": steer},
                "gvc": None if self.gvc is None else {"ax_cmd": gvc_ax, "phase": self.gvc.phase},
                "hdc": None if self.hdc is None else {"ax_cmd": self.hdc.ax_cmd, "status": self.hdc.status,
                                                      "cause": self.hdc.cause, "integral": self.hdc.integral},
                "function_status": None if self.hdc is None else {"hdc": self.hdc.status},
                "trace": {"requested": [float(Fx_des), float("nan") if Mz_des is None else Mz_des], "allocated": [float(x) for x in alloc.achieved],
                          "issued": [float(x) for x in self.actuation.last_issued]},
            },
        )


@dataclass(frozen=True)
class LongitudinalRequest:
    """One function's longitudinal force request (Plan V4.0 M24.1)."""
    source: str           # driver, hdc, gvc, ...
    priority: str         # safety | driver | stability | agility | comfort
    kind: str             # base (the driver's pedal demand) | brake_min (a braking floor) | add (small additive)
    force: float          # [N]


def arbitrate_longitudinal(requests) -> tuple[float, dict]:
    """Deterministic longitudinal arbitration (Plan V4.0 M24.1): no unbounded summation of interventions.

    The driver's pedal demand is the base. Active braking floors (``brake_min`` < 0, e.g. HDC) take the strongest
    braking of base and floors (``min``), never a sum. Additive agility/comfort requests (``add``, e.g. GVC) are
    granted only while no braking floor is active; otherwise they are suppressed and logged as a conflict.
    """
    base = next((r for r in requests if r.kind == "base"), None)
    fx = float(base.force) if base is not None else 0.0
    winner = "driver" if base is not None else "none"
    floors = [r for r in requests if r.kind == "brake_min" and r.force < 0.0]
    adds = [r for r in requests if r.kind == "add" and r.force != 0.0]
    suppressed = []
    if floors:
        f = min(floors, key=lambda r: r.force)
        if f.force < fx:
            fx, winner = float(f.force), f.source
        suppressed = [r.source for r in adds]
    else:
        for r in adds:
            fx += float(r.force)
        if adds:
            winner = "+".join([winner] + [r.source for r in adds])
    return fx, {"requests": {r.source: float(r.force) for r in requests}, "winner": winner, "suppressed": suppressed}


def arbitrate_yaw(law_mz: float | None, cbc_mz: float | None) -> tuple[float | None, str]:
    """Deterministic yaw-moment arbitration of the yaw law (ESC/TV) and CBC (Plan V4.0 M17.2, 3.4).

    No summation of two interventions on the same brakes:

        CBC inactive (None)              -> the law's request                         ("law")
        same sign (or one of them 0)     -> the request of larger magnitude           ("law" / "cbc")
        opposite signs                   -> the law's request (stability priority)    ("law, cbc suppressed")
    """
    if cbc_mz is None:
        return law_mz, ("law" if law_mz is not None else "none")
    if law_mz is None:
        return cbc_mz, "cbc"
    if law_mz * cbc_mz >= 0.0:
        return (cbc_mz, "cbc") if abs(cbc_mz) > abs(law_mz) else (law_mz, "law")
    return law_mz, "law, cbc suppressed"


__all__ = (
    "arbitrate_longitudinal",
    "arbitrate_yaw",
    "LongitudinalRequest",
    "INFORMATION_FULL_STATE",
    "ActuationConfig",
    "ActuationLayer",
    "MotionContext",
    "MotionController",
    "MotionControllerConfig",
    "MotionLaw",
    "MotionRequest",
    "YawReference",
)
