"""Wheel-slip control for ABS and TCS: PI with anti-windup and friction-adaptive target slip.

Per wheel, the controller reduces the requested torque so that the longitudinal
slip tracks a target ``kappa*``:

    e        = kappa - kappa*                     (brake mode: kappa* < 0;  drive mode: kappa* > 0)
    I       <- clip(I + s K_i e dt, 0, T_req)     (integrated torque reduction, anti-windup by clipping)
    T        = T_req - clip(I + s K_p e, 0, T_req)
    s        = -1 (brake) / +1 (drive)

The reduction is zero while the wheel is below the target slip, so the
controller only ever takes torque away; the integrator releases it again when
the slip recovers. Below ``min_speed`` (brake mode) the request passes through
so the vehicle can stop.

Target slip: the slip of peak force falls with road friction (for the bundled MF
tyre roughly ``0.11 mu``). ``kappa*`` is interpolated from a calibration table over
a friction estimate ``mu_hat`` per wheel. Because an underestimated ``mu_hat`` would
hold the wheel far below its peak (measured: 39 % braking efficiency on dry road
with ``mu_hat = 0.2``), the estimate is raised from the observed utilisation:

    mu_hat_i <- max(mu_hat_i, |F_i| / (F_z,i * utilisation_at_target))

so it climbs to the true friction within a few cycles. It is never lowered.
The estimate therefore starts low (``initial_mu = 0.1``): straight braking from
90 km/h then reaches 98 / 102 / 108 % of ``mu g`` on ``mu`` = 1.0 / 0.5 / 0.2, and
``mu_hat`` ends at the true friction. An external ``mu_estimate`` that is too high
cannot be corrected downwards and holds the wheel beyond its peak (81 % on
``mu = 0.2`` with ``mu_hat = 1.0``; locked wheels: 62 %). Values above 100 % are possible
because the tyre's peak friction exceeds ``mu`` at light loads. ``|F_i|`` is the tyre
force from the telemetry, i.e. an ideal force estimate.

Limits: the calibration table belongs to a tyre set (default: magic_formula_passenger_synthetic);
no wheel-acceleration thresholds, no pressure modulation phases, no hydraulics
(see actuators/brake_actuator.py for actuator dynamics).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from vehicle_model.core.types import WheelTelemetry


@dataclass(frozen=True)
class SlipControlConfig:
    mode: str = "brake"                                   # "brake" (ABS) or "drive" (TCS)
    kp: float = 5.0e3                                     # [N m] per unit slip
    ki: float = 1.0e5                                     # [N m/s] per unit slip
    mu_table: tuple[float, ...] = (0.1, 0.3, 0.6, 1.0)    # [-] friction grid
    slip_table: tuple[float, ...] = (0.010, 0.028, 0.055, 0.090)  # [-] |kappa*| over mu_table (MF tyre set)
    utilisation_at_target: float = 0.98                   # [-] |F|/(mu Fz) reached at kappa*
    min_speed: float = 1.5                                # [m/s] brake mode: pass-through below
    initial_mu: float = 0.1                               # [-] mu_hat before any observation (adapted upwards)

    def __post_init__(self) -> None:
        if self.mode not in ("brake", "drive"):
            raise ValueError("mode must be 'brake' or 'drive'")
        if len(self.mu_table) != len(self.slip_table) or np.any(np.diff(self.mu_table) <= 0):
            raise ValueError("mu_table must be increasing and match slip_table")
        if self.kp < 0 or self.ki <= 0:
            raise ValueError("gains must be positive")


class WheelSlipController:
    """Four-channel slip controller (ABS in brake mode, TCS in drive mode)."""

    def __init__(self, config: SlipControlConfig | None = None):
        self.config = config or SlipControlConfig()
        self.reset()

    def reset(self) -> None:
        self.integrator = np.zeros(4)
        self.mu_hat = np.full(4, self.config.initial_mu)
        self.active = np.zeros(4, dtype=bool)

    def target_slip(self) -> np.ndarray:
        c = self.config
        k = np.interp(self.mu_hat, c.mu_table, c.slip_table)
        return -k if c.mode == "brake" else k

    def update(self, T_request, telemetry: WheelTelemetry | None, vehicle_speed: float, dt: float,
               mu_estimate=None) -> np.ndarray:
        """Torque per wheel [N m] after slip control (same sign convention as the request, >= 0)."""
        c = self.config
        T_req = np.maximum(np.asarray(T_request, dtype=float).reshape(4), 0.0)
        if dt <= 0.0:
            raise ValueError("dt must be positive")
        if mu_estimate is not None:
            self.mu_hat = np.maximum(self.mu_hat, np.broadcast_to(np.asarray(mu_estimate, dtype=float), (4,)))
        if telemetry is None or (c.mode == "brake" and abs(vehicle_speed) < c.min_speed):
            self.integrator[:] = 0.0
            self.active[:] = False
            return T_req

        Fz = np.maximum(telemetry.Fz, 1.0)
        observed = np.hypot(telemetry.Fx_wheel, telemetry.Fy_wheel) / (Fz * c.utilisation_at_target)
        self.mu_hat = np.where(telemetry.Fz > 1.0, np.maximum(self.mu_hat, observed), self.mu_hat)

        s = -1.0 if c.mode == "brake" else 1.0
        e = telemetry.kappa - self.target_slip()
        self.integrator = np.clip(self.integrator + s * c.ki * e * dt, 0.0, T_req)
        reduction = np.clip(self.integrator + s * c.kp * e, 0.0, T_req)
        self.active = reduction > 1e-6
        return T_req - reduction


__all__ = ("SlipControlConfig", "WheelSlipController")
