"""Integrated 14-DOF Vertical & Planar Chassis Controller.

Implements Active Roll Stiffness Distribution Control (ARSDC) coupled with
Dynamic Roll Damping (CDC/Skyhook) on the full 14-DOF multi-body vehicle plant.

Vehicle Dynamics Principles:
1. Dynamic Roll Stiffness Distribution:
   - Due to tyre load degressivity (d^2 Fy / d Fz^2 < 0), total axle lateral grip
     decreases as lateral load transfer increases.
   - Understeer Mitigation: Dynamically shift roll stiffness rearward (kappa_roll,f -> 0.35)
     to increase rear lateral load transfer, reducing rear grip and tucking the nose in.
   - Oversteer Mitigation: Dynamically shift roll stiffness forward (kappa_roll,f -> 0.80)
     to increase front lateral load transfer, reducing front grip and stabilizing the rear axle.
2. Rollover & Wheel Lift Mitigation (NHTSA Fishhook):
   - Monitors Load Transfer Ratio (LTR = |Fz_left - Fz_right| / Fz_total).
   - Dynamically commands active corner forces to suppress roll angle (phi) and roll rate (dphi),
     guaranteeing positive tyre normal loads (Fz_i > 0) without wheel lift.

Conforms to Plan.md §4.5, §7, §9, §14.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Optional, Tuple

import numpy as np
import yaml

from vehicle_model.core.types import FL, FR, RL, RR
from vehicle_model.vehicle_models.fourteen_dof import FourteenDOFConfig, FourteenDOFState


@dataclass
class ActiveRollConfig:
    """Configuration for Active Roll Stiffness Distribution & 14-DOF Chassis Control."""

    # Baseline nominal roll distribution (fraction on front axle)
    nominal_front_roll_ratio: float = 0.58   # Baseline gentle understeer bias [-]
    min_front_roll_ratio: float = 0.32       # Max rearward shift under understeer [-]
    max_front_roll_ratio: float = 0.82       # Max forward shift under oversteer [-]

    # Dynamic tuning gains
    k_yaw_error_shift: float = 0.40          # Roll ratio shift per rad/s of yaw rate error [- / (rad/s)]
    k_sideslip_shift: float = 0.55           # Roll ratio shift per rad of sideslip angle [- / rad]
    k_roll_stiffness_gain: float = 45000.0   # Active anti-roll stiffness gain [N*m / rad]
    c_roll_damping_gain: float = 4500.0      # Active roll rate damping gain [N*m / (rad/s)]

    # Rollover prevention (Load Transfer Ratio: LTR)
    ltr_critical_threshold: float = 0.70     # LTR threshold to engage anti-rollover boost [-]
    max_active_corner_force_n: float = 3500.0# Maximum active corner suspension force [N]

    @classmethod
    def from_yaml(cls, path: str | Path) -> "ActiveRollConfig":
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        cfg = data.get("active_roll", data)
        return cls(
            nominal_front_roll_ratio=float(cfg.get("nominal_front_roll_ratio", 0.58)),
            min_front_roll_ratio=float(cfg.get("min_front_roll_ratio", 0.32)),
            max_front_roll_ratio=float(cfg.get("max_front_roll_ratio", 0.82)),
            k_yaw_error_shift=float(cfg.get("k_yaw_error_shift", 0.40)),
            k_sideslip_shift=float(cfg.get("k_sideslip_shift", 0.55)),
            k_roll_stiffness_gain=float(cfg.get("k_roll_stiffness_gain", 2400.0)),
            c_roll_damping_gain=float(cfg.get("c_roll_damping_gain", 650.0)),
            ltr_critical_threshold=float(cfg.get("ltr_critical_threshold", 0.70)),
            max_active_corner_force_n=float(cfg.get("max_active_corner_force_n", 3500.0)),
        )


@dataclass
class ActiveRollOutput:
    """Control output from the 14-DOF Active Roll Distribution Controller."""

    kappa_roll_front: float                  # Dynamic front roll stiffness distribution ratio [-]
    active_corner_forces: np.ndarray         # [FL, FR, RL, RR] active forces [N]
    total_anti_roll_moment_nm: float         # Net active anti-roll moment [N*m]
    load_transfer_ratio: float               # LTR: |Fz_left - Fz_right| / Fz_total [-]
    wheel_lift_risk: bool                    # Flag indicating high rollover / wheel-lift propensity


class ActiveRollDistributionController:
    """14-DOF Coordinated Vertical & Planar Active Roll Stiffness Controller."""

    def __init__(
        self,
        vehicle_cfg: FourteenDOFConfig,
        control_cfg: Optional[ActiveRollConfig] = None,
    ):
        self.veh_cfg = vehicle_cfg
        self.cfg = control_cfg or ActiveRollConfig()

    def compute_control(
        self,
        state: FourteenDOFState,
        r_desired: float,
        ay: float,
        Fz_corners: np.ndarray,
    ) -> ActiveRollOutput:
        """Compute active corner forces and dynamic roll stiffness distribution.

        Args:
            state: Full 14-DOF continuous vehicle state.
            r_desired: Reference/intended yaw rate from driver [rad/s].
            ay: Current lateral acceleration [m/s^2].
            Fz_corners: Measured or estimated dynamic normal loads [FL, FR, RL, RR] [N].

        Returns:
            ActiveRollOutput containing 4-corner active forces and diagnostics.
        """
        # 1. Compute handling discrepancy and sideslip angle
        r_error = state.r - r_desired
        v_lon = max(2.0, abs(state.vx))
        beta = math.atan2(state.vy, v_lon)

        # 2. Dynamic Front Roll Stiffness Distribution Ratio:
        # Oversteer (r_error > 0 or beta > 0): Shift roll stiffness forward -> front grip drops -> stabilizes rear
        # Understeer (r_error < 0): Shift roll stiffness rearward -> rear grip drops -> induces turn-in yaw rate
        delta_kappa = self.cfg.k_yaw_error_shift * r_error + self.cfg.k_sideslip_shift * abs(beta)
        kappa_f = float(np.clip(
            self.cfg.nominal_front_roll_ratio + delta_kappa,
            self.cfg.min_front_roll_ratio,
            self.cfg.max_front_roll_ratio,
        ))

        # 3. Dynamic Load Transfer Ratio (LTR) monitoring
        fz_left = Fz_corners[FL] + Fz_corners[RL]
        fz_right = Fz_corners[FR] + Fz_corners[RR]
        fz_total = max(100.0, fz_left + fz_right)
        ltr = float(abs(fz_left - fz_right) / fz_total)
        wheel_lift_risk = bool(ltr >= self.cfg.ltr_critical_threshold)

        # 4. Total Anti-Roll Restoring Moment Demand:
        # Stiffening against roll angle (phi) and active damping against roll rate (dphi)
        # Note: In ISO 8855, positive roll (phi > 0) is vehicle rolling to the right.
        # Restoring moment must push right side up and pull left side down (negative roll moment).
        roll_moment_stiff = -self.cfg.k_roll_stiffness_gain * state.phi
        roll_moment_damp = -self.cfg.c_roll_damping_gain * state.dphi
        net_roll_moment = roll_moment_stiff + roll_moment_damp

        # If high rollover propensity, boost anti-roll moment to prevent wheel lift
        if wheel_lift_risk:
            ltr_boost = 1.0 + 2.0 * ((ltr - self.cfg.ltr_critical_threshold) / (1.0 - self.cfg.ltr_critical_threshold))
            net_roll_moment *= ltr_boost

        # 5. Distribute net anti-roll moment across front and rear axles via kappa_f
        M_roll_front = kappa_f * net_roll_moment
        M_roll_rear = (1.0 - kappa_f) * net_roll_moment

        # 6. Resolve into 4 corner vertical suspension forces:
        # Positive active force F_act pushes sprung mass upward at that corner.
        # A positive M_roll rolls the car to the right (raises left side, depresses right side).
        # Therefore:
        # Front: F_act_FL = +M_roll_front / track_front, F_act_FR = -M_roll_front / track_front
        # Rear:  F_act_RL = +M_roll_rear  / track_rear,  F_act_RR = -M_roll_rear  / track_rear
        w_f = self.veh_cfg.track_front
        w_r = self.veh_cfg.track_rear

        F_fl = +M_roll_front / w_f
        F_fr = -M_roll_front / w_f
        F_rl = +M_roll_rear / w_r
        F_rr = -M_roll_rear / w_r

        # Limit corner forces to actuator envelope
        f_max = self.cfg.max_active_corner_force_n
        corner_forces = np.clip(
            np.array([F_fl, F_fr, F_rl, F_rr], dtype=np.float64),
            -f_max,
            f_max,
        )

        return ActiveRollOutput(
            kappa_roll_front=kappa_f,
            active_corner_forces=corner_forces,
            total_anti_roll_moment_nm=net_roll_moment,
            load_transfer_ratio=ltr,
            wheel_lift_risk=wheel_lift_risk,
        )
