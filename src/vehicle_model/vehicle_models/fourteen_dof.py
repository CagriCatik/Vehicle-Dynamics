"""Fourteen Degree of Freedom (14-DOF) Full Vehicle Dynamics Model.

Includes:
- Chassis planar motion: Longitudinal (vx), Lateral (vy), Yaw (r) [3 DOFs]
- Chassis vertical & attitude motion: Heave (zs), Pitch (theta), Roll (phi) [3 DOFs]
- Four unsprung wheel hop motions: zu_FL, zu_FR, zu_RL, zu_RR [4 DOFs]
- Four wheel rotational dynamics: omega_FL, omega_FR, omega_RL, omega_RR [4 DOFs]
Total: 14 dynamic degrees of freedom (24 continuous state variables).

Integrates dynamic normal load variations Fz_i(t) directly into the
interchangeable TyreModel framework (Linear, Fiala, Magic Formula, Pacejka).

Conforms to Milestone M7 (Plan.md §4.5, §9, §19 M7).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
import yaml

from vehicle_model.tire_model import TyreModel, load_tyre



@dataclass
class FourteenDOFConfig:
    """14-DOF vehicle and suspension configuration."""

    sprung_mass: float = 1320.0  # kg
    unsprung_mass_front: float = 45.0  # kg (per wheel)
    unsprung_mass_rear: float = 45.0  # kg (per wheel)
    Ixx: float = 480.0  # kg*m^2 (roll inertia)
    Iyy: float = 1650.0  # kg*m^2 (pitch inertia)
    Izz: float = 2200.0  # kg*m^2 (yaw inertia)
    wheelbase: float = 2.70  # m
    a: float = 1.25  # m (CG to front axle)
    b: float = 1.45  # m (CG to rear axle)
    track_front: float = 1.60  # m
    track_rear: float = 1.58  # m
    cg_height: float = 0.52  # m
    roll_center_front: float = 0.12  # m
    roll_center_rear: float = 0.14  # m

    # Suspension parameters
    k_spring_front: float = 36000.0  # N/m
    c_damper_front: float = 2300.0  # Ns/m
    k_arb_front: float = 450.0  # N*m/deg
    k_spring_rear: float = 34000.0  # N/m
    c_damper_rear: float = 2100.0  # Ns/m
    k_arb_rear: float = 220.0  # N*m/deg

    # Tyre radial compliance
    k_tyre_front: float = 210000.0  # N/m
    c_tyre_front: float = 250.0  # Ns/m
    k_tyre_rear: float = 210000.0  # N/m
    c_tyre_rear: float = 250.0  # Ns/m

    wheel_radius: float = 0.33  # m
    wheel_inertia: float = 1.2  # kg*m^2
    aerodynamic_drag_coeff: float = 0.30
    frontal_area: float = 2.2  # m^2
    air_density: float = 1.225  # kg/m^3

    @property
    def total_mass(self) -> float:
        return self.sprung_mass + 2.0 * self.unsprung_mass_front + 2.0 * self.unsprung_mass_rear

    @property
    def static_normal_loads(self) -> np.ndarray:
        """Static normal loads [FL, FR, RL, RR] in Newtons."""
        g = 9.80665
        m_total = self.total_mass
        w_f = (self.b / self.wheelbase) * m_total * g * 0.5
        w_r = (self.a / self.wheelbase) * m_total * g * 0.5
        return np.array([w_f, w_f, w_r, w_r], dtype=np.float64)

    @classmethod
    def from_yaml(cls, path: str | Path) -> "FourteenDOFConfig":
        with open(path, "r", encoding="utf-8") as f:
            d = yaml.safe_load(f)
        return cls(
            sprung_mass=float(d.get("sprung_mass", 1320.0)),
            unsprung_mass_front=float(d.get("unsprung_mass_front", 45.0)),
            unsprung_mass_rear=float(d.get("unsprung_mass_rear", 45.0)),
            Ixx=float(d.get("Ixx", 480.0)),
            Iyy=float(d.get("Iyy", 1650.0)),
            Izz=float(d.get("Izz", 2200.0)),
            wheelbase=float(d.get("wheelbase", 2.70)),
            a=float(d.get("a", 1.25)),
            b=float(d.get("b", 1.45)),
            track_front=float(d.get("track_front", 1.60)),
            track_rear=float(d.get("track_rear", 1.58)),
            cg_height=float(d.get("cg_height", 0.52)),
            roll_center_front=float(d.get("roll_center_height_front", 0.12)),
            roll_center_rear=float(d.get("roll_center_height_rear", 0.14)),
            k_spring_front=float(d.get("spring_stiffness_front", 36000.0)),
            c_damper_front=float(d.get("damping_front", 2300.0)),
            k_arb_front=float(d.get("anti_roll_bar_front", 450.0)),
            k_spring_rear=float(d.get("spring_stiffness_rear", 34000.0)),
            c_damper_rear=float(d.get("damping_rear", 2100.0)),
            k_arb_rear=float(d.get("anti_roll_bar_rear", 220.0)),
            k_tyre_front=float(d.get("tyre_radial_stiffness_front", 210000.0)),
            c_tyre_front=float(d.get("tyre_radial_damping_front", 250.0)),
            k_tyre_rear=float(d.get("tyre_radial_stiffness_rear", 210000.0)),
            c_tyre_rear=float(d.get("tyre_radial_damping_rear", 250.0)),
            wheel_radius=float(d.get("wheel_radius", 0.33)),
            wheel_inertia=float(d.get("wheel_inertia", 1.2)),
        )


@dataclass
class FourteenDOFState:
    """State vector of the 14-DOF model (24 variables)."""

    # Global position & heading
    x: float = 0.0
    y: float = 0.0
    psi: float = 0.0

    # Chassis planar velocities
    vx: float = 20.0  # Longitudinal velocity [m/s]
    vy: float = 0.0  # Lateral velocity [m/s]
    r: float = 0.0  # Yaw rate [rad/s]

    # Chassis vertical & attitude
    zs: float = 0.0  # Heave displacement [m]
    dzs: float = 0.0  # Heave velocity [m/s]
    theta: float = 0.0  # Pitch angle [rad] (positive nose-up)
    dtheta: float = 0.0  # Pitch rate [rad/s]
    phi: float = 0.0  # Roll angle [rad] (positive roll-left)
    dphi: float = 0.0  # Roll rate [rad/s]

    # Unsprung mass hops [m] and velocities [m/s] for FL, FR, RL, RR
    zu: np.ndarray = None  # shape (4,)
    dzu: np.ndarray = None  # shape (4,)

    # Wheel rotational speeds [rad/s] for FL, FR, RL, RR
    omega: np.ndarray = None  # shape (4,)

    def __post_init__(self):
        if self.zu is None:
            self.zu = np.zeros(4, dtype=np.float64)
        if self.dzu is None:
            self.dzu = np.zeros(4, dtype=np.float64)
        if self.omega is None:
            self.omega = np.full(4, self.vx / 0.33, dtype=np.float64)

    def as_array(self) -> np.ndarray:
        return np.concatenate([
            np.array([self.x, self.y, self.psi, self.vx, self.vy, self.r,
                      self.zs, self.dzs, self.theta, self.dtheta, self.phi, self.dphi]),
            self.zu,
            self.dzu,
            self.omega,
        ])

    @classmethod
    def from_array(cls, arr: np.ndarray) -> "FourteenDOFState":
        return cls(
            x=float(arr[0]),
            y=float(arr[1]),
            psi=float(arr[2]),
            vx=float(arr[3]),
            vy=float(arr[4]),
            r=float(arr[5]),
            zs=float(arr[6]),
            dzs=float(arr[7]),
            theta=float(arr[8]),
            dtheta=float(arr[9]),
            phi=float(arr[10]),
            dphi=float(arr[11]),
            zu=np.array(arr[12:16], dtype=np.float64),
            dzu=np.array(arr[16:20], dtype=np.float64),
            omega=np.array(arr[20:24], dtype=np.float64),
        )


@dataclass
class FourteenDOFTelemetry:
    """Detailed diagnostic telemetry from 14-DOF simulation step."""

    Fz: np.ndarray  # Normal tyre forces [FL, FR, RL, RR] (N), >= 0
    in_contact: np.ndarray  # bool per wheel; False while lifted off the road
    Fx: np.ndarray  # Longitudinal tyre forces (N)
    Fy: np.ndarray  # Lateral tyre forces (N)
    alpha: np.ndarray  # Slip angles (rad)
    kappa: np.ndarray  # Longitudinal slips (-)
    suspension_deflection: np.ndarray  # Corner deflections (m)
    ax: float  # Longitudinal acceleration (m/s^2)
    ay: float  # Lateral acceleration (m/s^2)
    ddzs: float  # Heave acceleration (m/s^2)
    ddtheta: float  # Pitch acceleration (rad/s^2)
    ddphi: float  # Roll acceleration (rad/s^2)


class FourteenDOFModel:
    """14-DOF Full Vehicle Dynamics Simulation Plant."""

    def __init__(
        self,
        config: FourteenDOFConfig | None = None,
        tyre_model: TyreModel | None = None,
    ) -> None:
        self.cfg = config or FourteenDOFConfig()
        self.tyre_model = tyre_model or load_tyre("pacejka_front_synthetic")

        # Corner coordinates (x_i, y_i)
        # FL: (+a, +wf/2), FR: (+a, -wf/2), RL: (-b, +wr/2), RR: (-b, -wr/2)
        self.corner_x = np.array([self.cfg.a, self.cfg.a, -self.cfg.b, -self.cfg.b], dtype=np.float64)
        self.corner_y = np.array([
            self.cfg.track_front / 2.0,
            -self.cfg.track_front / 2.0,
            self.cfg.track_rear / 2.0,
            -self.cfg.track_rear / 2.0,
        ], dtype=np.float64)

    def evaluate_dynamics(
        self,
        state: FourteenDOFState,
        delta: np.ndarray,  # Steering angles [FL, FR, RL, RR] (rad)
        T_drive: np.ndarray,  # Drive torques (Nm)
        T_brake: np.ndarray,  # Brake torques (Nm)
        mu_ground: np.ndarray | float = 1.0,  # Road friction
        road_z: np.ndarray | None = None,  # Road heights (m)
        road_dz: np.ndarray | None = None,  # Road vertical velocities (m/s)
        active_forces: np.ndarray | None = None,  # Active suspension strut forces (N)
        damper_c_override: np.ndarray | None = None,  # Semi-active damper overrides (Ns/m)
    ) -> Tuple[np.ndarray, FourteenDOFTelemetry]:
        """Compute state derivatives and telemetry."""
        cfg = self.cfg
        if np.isscalar(mu_ground):
            mu_arr = np.full(4, float(mu_ground), dtype=np.float64)
        else:
            mu_arr = np.asarray(mu_ground, dtype=np.float64)

        if road_z is None:
            road_z = np.zeros(4, dtype=np.float64)
        if road_dz is None:
            road_dz = np.zeros(4, dtype=np.float64)
        if active_forces is None:
            active_forces = np.zeros(4, dtype=np.float64)

        # 1. Corner kinematics of sprung mass
        # zs_i = zs + x_i * theta + y_i * phi
        # dzs_i = dzs + x_i * dtheta + y_i * dphi
        zs_corners = state.zs + self.corner_x * state.theta + self.corner_y * state.phi
        dzs_corners = state.dzs + self.corner_x * state.dtheta + self.corner_y * state.dphi

        # Suspension deflections (zs_i - zu_i)
        deflections = zs_corners - state.zu
        rel_vels = dzs_corners - state.dzu

        # Suspension stiffness & damping rates
        k_s = np.array([cfg.k_spring_front, cfg.k_spring_front, cfg.k_spring_rear, cfg.k_spring_rear])
        if damper_c_override is not None:
            c_s = np.asarray(damper_c_override, dtype=np.float64)
        else:
            c_s = np.array([cfg.c_damper_front, cfg.c_damper_front, cfg.c_damper_rear, cfg.c_damper_rear])

        # Anti-roll bar contributions (N/m conversion from N*m/deg)
        deg_to_rad = 180.0 / math.pi
        k_arb_front_eff = (cfg.k_arb_front * deg_to_rad) / (cfg.track_front ** 2)
        k_arb_rear_eff = (cfg.k_arb_rear * deg_to_rad) / (cfg.track_rear ** 2)

        delta_roll_front = deflections[0] - deflections[1]
        delta_roll_rear = deflections[2] - deflections[3]

        F_arb = np.array([
            -k_arb_front_eff * delta_roll_front,
            +k_arb_front_eff * delta_roll_front,
            -k_arb_rear_eff * delta_roll_rear,
            +k_arb_rear_eff * delta_roll_rear,
        ], dtype=np.float64)

        # Total suspension force on sprung mass (upward positive)
        # F_susp = -k_s * deflection - c_s * rel_vel + F_arb + F_act
        F_susp = -(k_s * deflections + c_s * rel_vels) + F_arb + active_forces

        # 2. Tyre radial forces and dynamic normal loads
        k_t = np.array([cfg.k_tyre_front, cfg.k_tyre_front, cfg.k_tyre_rear, cfg.k_tyre_rear])
        c_t = np.array([cfg.c_tyre_front, cfg.c_tyre_front, cfg.c_tyre_rear, cfg.c_tyre_rear])

        tyre_deflections = state.zu - road_z
        tyre_rel_vels = state.dzu - road_dz

        # Dynamic tyre normal variation (positive when tyre is compressed into ground)
        # Unilateral contact: a lifted wheel carries no load and feels no tyre force
        Fz = np.maximum(0.0, cfg.static_normal_loads - (k_t * tyre_deflections + c_t * tyre_rel_vels))
        F_tyre_dyn = Fz - cfg.static_normal_loads

        # 3. Planar tyre forces from TyreModel
        # Wheel velocities in wheel heading frames
        vx_wheel = state.vx - state.r * self.corner_y
        vy_wheel = state.vy + state.r * self.corner_x

        # Transform to wheel-oriented coordinates
        cos_d = np.cos(delta)
        sin_d = np.sin(delta)
        vx_w = vx_wheel * cos_d + vy_wheel * sin_d
        vy_w = -vx_wheel * sin_d + vy_wheel * cos_d

        # Slip angles and slip ratios
        v_lon = np.maximum(0.5, np.abs(vx_w))
        alpha = -np.arctan2(vy_w, v_lon)

        v_wheel = state.omega * cfg.wheel_radius
        kappa = np.clip((v_wheel - vx_w) / v_lon, -1.0, 1.0)

        Fx_w, Fy_w = self.tyre_model.forces(kappa, alpha, Fz, mu_arr)

        # Transform tyre forces to vehicle chassis coordinates
        Fx_veh = Fx_w * cos_d - Fy_w * sin_d
        Fy_veh = Fx_w * sin_d + Fy_w * cos_d

        # Aerodynamic drag
        F_aero = 0.5 * cfg.air_density * cfg.aerodynamic_drag_coeff * cfg.frontal_area * (state.vx ** 2)

        # 4. Planar equations of motion
        m_tot = cfg.total_mass
        ax = (np.sum(Fx_veh) - F_aero) / m_tot
        ay = np.sum(Fy_veh) / m_tot

        # State derivatives in rotating body frame
        dvx = ax + state.vy * state.r
        dvy = ay - state.vx * state.r

        # Yaw acceleration
        Mz = np.sum(self.corner_x * Fy_veh - self.corner_y * Fx_veh)
        r_dot = Mz / cfg.Izz

        # Pose derivatives
        dx = state.vx * math.cos(state.psi) - state.vy * math.sin(state.psi)
        dy = state.vx * math.sin(state.psi) + state.vy * math.cos(state.psi)
        dpsi = state.r

        # 5. Sprung mass vertical & attitude accelerations
        # Heave
        ddzs = np.sum(F_susp) / cfg.sprung_mass

        # Pitch (positive nose up)
        # Pitch moment from suspension forces: F_susp pushes up
        M_pitch_susp = np.sum(self.corner_x * F_susp)
        # Inertial dive moment under braking (ax < 0 causes nose-down pitch: ddtheta < 0)
        h_pitch = cfg.cg_height - 0.5 * (cfg.roll_center_front + cfg.roll_center_rear)
        M_pitch_inertial = cfg.sprung_mass * ax * h_pitch
        ddtheta = (M_pitch_susp + M_pitch_inertial) / cfg.Iyy

        # Roll (positive roll right: left side rises, right side lowers)
        # Susp moment: right compression produces negative restoring moment
        M_roll_susp = np.sum(self.corner_y * F_susp)
        # Centrifugal roll moment during cornering (ay > 0 rolls vehicle to right: ddphi > 0)
        h_roll = cfg.cg_height - 0.5 * (cfg.roll_center_front + cfg.roll_center_rear)
        M_roll_inertial = cfg.sprung_mass * ay * h_roll
        ddphi = (M_roll_susp + M_roll_inertial) / cfg.Ixx

        # 6. Unsprung mass hops
        # m_u * ddzu_i = -F_susp_i + F_tyre_dyn_i
        m_u = np.array([
            cfg.unsprung_mass_front,
            cfg.unsprung_mass_front,
            cfg.unsprung_mass_rear,
            cfg.unsprung_mass_rear,
        ])
        ddzu = (-F_susp + F_tyre_dyn) / m_u

        # 7. Wheel rotational accelerations
        domega = np.zeros(4, dtype=np.float64)
        for i in range(4):
            sign_omega = math.copysign(1.0, state.omega[i]) if abs(state.omega[i]) > 0.05 else 0.0
            torque_net = T_drive[i] - T_brake[i] * sign_omega - Fx_w[i] * cfg.wheel_radius
            domega[i] = torque_net / cfg.wheel_inertia

        derivatives = np.concatenate([
            np.array([dx, dy, dpsi, dvx, dvy, r_dot, state.dzs, ddzs, state.dtheta, ddtheta, state.dphi, ddphi]),
            state.dzu,
            ddzu,
            domega,
        ])

        telemetry = FourteenDOFTelemetry(
            Fz=Fz,
            in_contact=Fz > 0.0,
            Fx=Fx_veh,
            Fy=Fy_veh,
            alpha=alpha,
            kappa=kappa,
            suspension_deflection=deflections,
            ax=ax,
            ay=ay,
            ddzs=ddzs,
            ddtheta=ddtheta,
            ddphi=ddphi,
        )

        return derivatives, telemetry

    def step(
        self,
        state: FourteenDOFState,
        delta: np.ndarray,
        T_drive: np.ndarray,
        T_brake: np.ndarray,
        dt: float,
        mu_ground: np.ndarray | float = 1.0,
        road_z: np.ndarray | None = None,
        road_dz: np.ndarray | None = None,
        active_forces: np.ndarray | None = None,
        damper_c_override: np.ndarray | None = None,
    ) -> Tuple[FourteenDOFState, FourteenDOFTelemetry]:
        """RK4 numerical integration step."""
        x0 = state.as_array()

        k1, telem = self.evaluate_dynamics(
            state, delta, T_drive, T_brake, mu_ground, road_z, road_dz, active_forces, damper_c_override
        )

        s2 = FourteenDOFState.from_array(x0 + 0.5 * dt * k1)
        k2, _ = self.evaluate_dynamics(
            s2, delta, T_drive, T_brake, mu_ground, road_z, road_dz, active_forces, damper_c_override
        )

        s3 = FourteenDOFState.from_array(x0 + 0.5 * dt * k2)
        k3, _ = self.evaluate_dynamics(
            s3, delta, T_drive, T_brake, mu_ground, road_z, road_dz, active_forces, damper_c_override
        )

        s4 = FourteenDOFState.from_array(x0 + dt * k3)
        k4, _ = self.evaluate_dynamics(
            s4, delta, T_drive, T_brake, mu_ground, road_z, road_dz, active_forces, damper_c_override
        )

        x_next = x0 + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
        next_state = FourteenDOFState.from_array(x_next)

        return next_state, telem
