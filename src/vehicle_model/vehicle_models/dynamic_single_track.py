"""Dynamic single-track bicycle model.

State: ``[x, y, vx, vy, yaw, steer, yaw_rate]``
Input: ``[ax, steer_rate]``

The implementation is explicit about units, input order, low-speed behavior,
and package imports. Roll/bank is treated as radians everywhere.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Any

import numpy as np

from vehicle_model.tire_model import MagicFormulaLateral
from .config import VehicleParams, coerce_vehicle_params
from .integration import input_at, rk4_step, scalar_at

_TIRE_DATA_DIR = Path(__file__).resolve().parents[1] / "tire_model" / "tire_data" / "yamls"
# Keys must match filenames of calibrated tire YAMLs (e.g. "VEHICLE_FRONT_TIRE_xxx.yaml").
# The shipped EXAMPLE files are for a different vehicle and will not match these keys.
_FRONT_TIRE_KEY = "VEHICLE_FRONT_TIRE"
_REAR_TIRE_KEY = "VEHICLE_REAR_TIRE"


@dataclass(frozen=True)
class SlipState:
    kappa: float
    alpha_f: float
    alpha_r: float


@dataclass(frozen=True)
class ForceState:
    Fyr: float
    Fyf: float
    Fxr: float
    drag: float
    rolling: float
    bank_lateral: float


def _magic_formula(slip: float, *, B: float, C: float, D: float, E: float, Sv: float = 0.0) -> float:
    bx = B * slip
    return Sv + D * math.sin(C * math.atan(bx - E * (bx - math.atan(bx))))


class DynamicSingleTrack:
    n_states = 7
    n_inputs = 2

    def __init__(self, params: VehicleParams | dict[str, Any] | None = None):
        self.params = coerce_vehicle_params(params)
        self._sync_model_attrs()
        self._mf62 = self._load_mf62()
        if self._mf62 is not None:
            mu = self.params.mu
            self._mf62_scale_f = mu / max(self._mf62.param_tires[_FRONT_TIRE_KEY].get("PDY1", mu), 1e-6)
            self._mf62_scale_r = mu / max(self._mf62.param_tires[_REAR_TIRE_KEY].get("PDY1", mu), 1e-6)

    def _sync_model_attrs(self) -> None:
        self.g = self.params.g
        self.lf = self.params.lf
        self.lr = self.params.lr
        self.mass = self.params.mass
        self.Iz = self.params.Iz
        self.min_v = self.params.min_v
        self.max_v = self.params.max_v
        self.max_acc = self.params.max_acc
        self.min_acc = self.params.min_acc
        self.max_steer = self.params.max_steer
        self.min_steer = self.params.min_steer
        self.max_steer_vel = self.params.max_steer_vel
        self.min_steer_vel = self.params.min_steer_vel
        self.MF_lat = self.params.MF_lat
        self.switch_v = self.params.switch_v
        self.rho = self.params.air_density
        self.Af = self.params.resolved_frontal_area
        self.Cd = self.params.drag_coefficient
        self.tire_p = self.params.l_pressure
        self.tire_prs = self.params.l_pressure

    def _load_mf62(self) -> MagicFormulaLateral | None:
        try:
            return MagicFormulaLateral([_FRONT_TIRE_KEY, _REAR_TIRE_KEY], _TIRE_DATA_DIR)
        except Exception:
            return None

    def _fz_axles(self, ax: float, bank: float) -> tuple[float, float]:
        wheelbase = self.lf + self.lr
        Fz_normal = self.mass * self.g * max(math.cos(bank), 0.0)
        delta_Fz = self.mass * ax * self.params.hcog / wheelbase
        return (
            max(Fz_normal * self.lr / wheelbase - delta_Fz, 0.0),
            max(Fz_normal * self.lf / wheelbase + delta_Fz, 0.0),
        )

    def _bounded_input(self, u) -> np.ndarray:
        cmd = np.asarray(u, dtype=float).copy()
        if cmd.shape != (self.n_inputs,):
            raise ValueError(f"Dynamic input must have shape ({self.n_inputs},).")
        cmd[0] = np.clip(cmd[0], self.min_acc, self.max_acc)
        cmd[1] = np.clip(cmd[1], self.min_steer_vel, self.max_steer_vel)
        return cmd

    def _bounded_state(self, x) -> np.ndarray:
        state = np.asarray(x, dtype=float).copy()
        if state.shape != (self.n_states,):
            raise ValueError(f"Dynamic state must have shape ({self.n_states},).")
        state[2] = np.clip(state[2], self.min_v, self.max_v)
        state[5] = np.clip(state[5], self.min_steer, self.max_steer)
        return state

    def _wheel_speed_scalar(self, wheel_v) -> float | None:
        if wheel_v is None:
            return None
        arr = np.asarray(wheel_v, dtype=float)
        if arr.size == 0:
            return None
        return float(np.nanmean(arr))

    def _wheel_at(self, wheel_v, index: int):
        if wheel_v is None:
            return None
        arr = np.asarray(wheel_v, dtype=float)
        if arr.ndim == 0:
            return float(arr)
        return arr[min(index, arr.shape[0] - 1)]

    def calc_slips(self, x, u=None, wheel_v=None) -> tuple[float, float, float]:
        del u
        state = self._bounded_state(x)
        vx = state[2]
        vy = state[3]
        steer = state[5]
        yaw_rate = state[6]

        speed_for_slip = max(abs(vx), self.params.min_slip_speed)
        alpha_f = steer - math.atan2(vy + self.lf * yaw_rate, speed_for_slip)
        alpha_r = -math.atan2(vy - self.lr * yaw_rate, speed_for_slip)

        wheel_speed = self._wheel_speed_scalar(wheel_v)
        if wheel_speed is None or abs(vx) < self.params.min_slip_speed:
            kappa = 0.0
        else:
            denom = max(abs(wheel_speed), abs(vx), self.params.min_slip_speed)
            kappa = (wheel_speed - vx) / denom

        if abs(vx) < self.params.min_slip_speed:
            alpha_f = 0.0
            alpha_r = 0.0

        return float(kappa), float(alpha_f), float(alpha_r)

    def calc_forces(self, x, u, wheel_v=None, roll: float | None = None) -> tuple[float, float, float, float, float]:
        force_state = self.force_state(x, u, wheel_v=wheel_v, roll=roll)
        return (
            force_state.Fyr,
            force_state.Fyf,
            force_state.bank_lateral * self.mass,
            0.0,
            force_state.Fxr,
        )

    def force_state(self, x, u, wheel_v=None, roll: float | None = None) -> ForceState:
        state = self._bounded_state(x)
        ax, _ = self._bounded_input(u)
        vx = state[2]
        steer = state[5]
        bank = 0.0 if roll is None else float(roll)
        kappa, alpha_f, alpha_r = self.calc_slips(state, wheel_v=wheel_v)

        Fz_front, Fz_rear = self._fz_axles(ax, bank)
        if self._mf62 is not None:
            self._mf62.online_params(Fz_front, self.tire_prs, 0.0, kappa, alpha_f, abs(vx), _FRONT_TIRE_KEY)
            Fyf = float(self._mf62.calculateFy()[0]) * self._mf62_scale_f
            self._mf62.online_params(Fz_rear, self.tire_prs, 0.0, kappa, alpha_r, abs(vx), _REAR_TIRE_KEY)
            Fyr = float(self._mf62.calculateFy()[0]) * self._mf62_scale_r
        else:
            lat = self.MF_lat
            mu = self.params.mu
            Fyf = _magic_formula(
                alpha_f + lat["Shfy"],
                B=lat["Bf"], C=lat["Cf"], D=mu * Fz_front, E=lat["Ef"], Sv=lat["Svfy"],
            )
            Fyr = _magic_formula(
                alpha_r + lat["Shry"],
                B=lat["Br"], C=lat["Cr"], D=mu * Fz_rear, E=lat["Er"], Sv=lat["Svry"],
            )

        # The measured/control input is longitudinal acceleration. Use it as
        # the traction/brake request and subtract passive losses explicitly.
        Fxr = self.mass * ax
        drag = 0.5 * self.rho * self.Cd * self.Af * vx * abs(vx)
        normal_force = self.mass * self.g * max(math.cos(bank), 0.0)
        if abs(vx) < self.params.min_slip_speed:
            rolling = 0.0
        else:
            rolling = self.params.rolling_resistance * normal_force * math.copysign(1.0, vx)
        bank_lateral = self.g * math.sin(bank)

        # Preserve the slip-ratio calculation as a diagnostic path. The current
        # data interface drives longitudinal motion through ax rather than
        # wheel torque, so kappa is not injected into Fxr.
        _ = kappa
        _ = steer
        return ForceState(Fyr=Fyr, Fyf=Fyf, Fxr=Fxr, drag=drag, rolling=rolling, bank_lateral=bank_lateral)

    def derivative_eqs(self, t, x, u, wheel_v=None, roll=None):
        del t
        state = self._bounded_state(x)
        ax, steer_rate = self._bounded_input(u)
        vx = state[2]
        vy = state[3]
        yaw = state[4]
        steer = state[5]
        yaw_rate = state[6]

        if (steer >= self.max_steer and steer_rate > 0.0) or (
            steer <= self.min_steer and steer_rate < 0.0
        ):
            steer_rate = 0.0

        dxdt = np.zeros(self.n_states)
        dxdt[0] = vx * math.cos(yaw) - vy * math.sin(yaw)
        dxdt[1] = vx * math.sin(yaw) + vy * math.cos(yaw)
        dxdt[4] = yaw_rate
        dxdt[5] = steer_rate

        if abs(vx) < self.switch_v:
            wheelbase = self.lf + self.lr
            yaw_rate_target = vx * math.tan(steer) / max(wheelbase, 1e-9)
            dxdt[2] = ax
            dxdt[3] = -vy / self.params.low_speed_lateral_tau
            dxdt[6] = (yaw_rate_target - yaw_rate) / self.params.low_speed_yaw_tau
            return dxdt

        forces = self.force_state(state, [ax, steer_rate], wheel_v=wheel_v, roll=roll)
        dxdt[2] = (forces.Fxr - forces.drag - forces.rolling - forces.Fyf * math.sin(steer)) / self.mass
        dxdt[2] += vy * yaw_rate
        dxdt[3] = (forces.Fyr + forces.Fyf * math.cos(steer)) / self.mass
        dxdt[3] -= vx * yaw_rate + forces.bank_lateral
        dxdt[6] = (self.lf * forces.Fyf * math.cos(steer) - self.lr * forces.Fyr) / self.Iz
        return dxdt

    def odeintRK4(self, y0, t, u, wheel_v=None, roll=None):
        dt = float(t[-1] - t[0])
        cmd = self._bounded_input(u)
        return rk4_step(self.derivative_eqs, np.asarray(y0, dtype=float), dt, cmd, wheel_v, roll)

    def sim_continuous(self, x0, u, t, wheel_v=None, roll=None):
        times = np.asarray(t, dtype=float)
        n_steps = len(times) - 1
        x = np.zeros((n_steps + 1, self.n_states))
        dxdt = np.zeros_like(x)
        x[0, :] = np.asarray(x0, dtype=float)
        first_u = input_at(u, 0, self.n_inputs)
        dxdt[0, :] = self.derivative_eqs(None, x[0, :], first_u, wheel_v, scalar_at(roll, 0))

        for idx in range(1, n_steps + 1):
            cmd = input_at(u, idx - 1, self.n_inputs)
            roll_i = scalar_at(roll, idx - 1)
            wheel_i = self._wheel_at(wheel_v, idx - 1)
            x[idx, :] = self.odeintRK4(x[idx - 1, :], times[idx - 1 : idx + 1], cmd, wheel_i, roll_i)
            dxdt[idx, :] = self.derivative_eqs(None, x[idx, :], cmd, wheel_i, roll_i)
        return x, dxdt

    def sim_continuous_multistep(self, x0, u, t, step, step_ls, wheel_v=None, roll=None):
        x = np.zeros((step + 1, self.n_states))
        dxdt = np.zeros_like(x)
        x[0, :] = np.asarray(x0, dtype=float)
        for idx in range(1, step + 1):
            input_idx = idx - 1
            cmd = input_at(u, input_idx, self.n_inputs)
            wheel_i = self._wheel_at(wheel_v, input_idx)
            roll_i = scalar_at(roll, input_idx)
            x[idx, :] = self.odeintRK4(x[idx - 1, :], t, cmd, wheel_i, roll_i)
            dxdt[idx, :] = self.derivative_eqs(None, x[idx - 1, :], cmd, wheel_i, roll_i)
            step_ls.append(step)
        return x, dxdt, step_ls


__all__ = ("DynamicSingleTrack", "ForceState", "SlipState")
