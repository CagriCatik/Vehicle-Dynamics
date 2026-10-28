"""Kinematic single-track bicycle model.

State: ``[x, y, yaw, v, steer]``
Input: ``[ax, steer_rate]``
All units are SI and angles are radians.
"""

from __future__ import annotations

import numpy as np

from .config import VehicleParams, coerce_vehicle_params
from .integration import input_at, rk4_step


class KinematicSingleTrack:
    n_states = 5
    n_inputs = 2

    def __init__(self, params: VehicleParams | dict | None = None):
        self.params = coerce_vehicle_params(params)
        self._sync_model_attrs()

    def _sync_model_attrs(self) -> None:
        self.lf = self.params.lf
        self.lr = self.params.lr
        self.min_v = self.params.min_v
        self.max_v = self.params.max_v
        self.max_acc = self.params.max_acc
        self.min_acc = self.params.min_acc
        self.max_steer = self.params.max_steer
        self.min_steer = self.params.min_steer
        self.max_steer_vel = self.params.max_steer_vel
        self.min_steer_vel = self.params.min_steer_vel
        self.tire_p = self.params.l_pressure

    def _bounded_input(self, u) -> np.ndarray:
        cmd = np.asarray(u, dtype=float).copy()
        if cmd.shape != (self.n_inputs,):
            raise ValueError(f"Kinematic input must have shape ({self.n_inputs},).")
        cmd[0] = np.clip(cmd[0], self.min_acc, self.max_acc)
        cmd[1] = np.clip(cmd[1], self.min_steer_vel, self.max_steer_vel)
        return cmd

    def derivative_eqs(self, t, x, u):
        del t
        state = np.asarray(x, dtype=float)
        ax, steer_rate = self._bounded_input(u)
        yaw = state[2]
        speed = np.clip(state[3], self.min_v, self.max_v)
        steer = np.clip(state[4], self.min_steer, self.max_steer)

        if (steer >= self.max_steer and steer_rate > 0.0) or (
            steer <= self.min_steer and steer_rate < 0.0
        ):
            steer_rate = 0.0

        beta = np.arctan2(self.lr * np.tan(steer), self.lf + self.lr)
        dxdt = np.zeros(self.n_states)
        dxdt[0] = speed * np.cos(yaw + beta)
        dxdt[1] = speed * np.sin(yaw + beta)
        dxdt[2] = speed * np.sin(beta) / max(self.lr, 1e-9)
        dxdt[3] = ax
        dxdt[4] = steer_rate
        return dxdt

    def odeintRK4(self, y0, t, u):
        dt = float(t[-1] - t[0])
        return rk4_step(self.derivative_eqs, np.asarray(y0, dtype=float), dt, self._bounded_input(u))

    def sim_continuous(self, x0, u, t, wheel_v=None, roll=None):
        del wheel_v, roll
        times = np.asarray(t, dtype=float)
        n_steps = len(times) - 1
        x = np.zeros((n_steps + 1, self.n_states))
        dxdt = np.zeros_like(x)
        x[0, :] = np.asarray(x0, dtype=float)
        dxdt[0, :] = self.derivative_eqs(None, x[0, :], input_at(u, 0, self.n_inputs))

        for idx in range(1, n_steps + 1):
            cmd = input_at(u, idx - 1, self.n_inputs)
            x[idx, :] = self.odeintRK4(x[idx - 1, :], times[idx - 1 : idx + 1], cmd)
            dxdt[idx, :] = self.derivative_eqs(None, x[idx, :], cmd)
        return x, dxdt

    def sim_continuous_multistep(self, x0, u, t, step, step_ls, wheel_v=None, roll=None):
        del wheel_v, roll
        x = np.zeros((step + 1, self.n_states))
        dxdt = np.zeros_like(x)
        x[0, :] = np.asarray(x0, dtype=float)
        for idx in range(1, step + 1):
            cmd = input_at(u, idx - 1, self.n_inputs)
            x[idx, :] = self.odeintRK4(x[idx - 1, :], t, cmd)
            dxdt[idx, :] = self.derivative_eqs(None, x[idx - 1, :], cmd)
            step_ls.append(step)
        return x, dxdt, step_ls


__all__ = ("KinematicSingleTrack",)
