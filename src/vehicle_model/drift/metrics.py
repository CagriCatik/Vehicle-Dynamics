"""Drift KPIs from recorded histories ; no visualisation, no controller logic."""

from __future__ import annotations

import math

import numpy as np

LOSS_BETA_ERROR = math.radians(30.0)   # |beta - beta_ref| above: the drift is lost (spin or exit)


def hold_kpis(t, vx, vy, r, beta_ref: float, r_ref: float, v_ref: float, settle_s: float, saturated=None) -> dict:
    """RMS errors over the hold window ``t >= settle_s``, peak errors over the whole run, loss of the drift."""
    t, vx, vy, r = (np.asarray(x, float) for x in (t, vx, vy, r))
    beta = np.arctan2(vy, vx)
    v = np.hypot(vx, vy)
    eb, er, ev = beta - beta_ref, r - r_ref, v - v_ref
    w = t >= settle_s
    if not np.any(w):
        w = np.ones_like(t, dtype=bool)
    lost = bool(np.max(np.abs(eb)) >= LOSS_BETA_ERROR)
    scaled0 = math.sqrt((eb[0] / 0.05) ** 2 + (er[0] / 0.1) ** 2 + (ev[0] / 1.0) ** 2)
    scaled1 = math.sqrt((eb[-1] / 0.05) ** 2 + (er[-1] / 0.1) ** 2 + (ev[-1] / 1.0) ** 2)
    return {
        "beta_rmse_deg": float(np.degrees(np.sqrt(np.mean(eb[w] ** 2)))),
        "yaw_rmse_degps": float(np.degrees(np.sqrt(np.mean(er[w] ** 2)))),
        "speed_rmse_mps": float(np.sqrt(np.mean(ev[w] ** 2))),
        "max_beta_error_deg": float(np.degrees(np.max(np.abs(eb)))),
        "final_beta_error_deg": float(np.degrees(eb[-1])),
        "error_norm_initial": scaled0, "error_norm_final": scaled1,     # scales: 0.05 rad, 0.1 rad/s, 1 m/s
        "stability_loss": lost,
        "saturation_fraction": float(np.mean(saturated)) if saturated is not None else float("nan"),
    }


__all__ = ("LOSS_BETA_ERROR", "hold_kpis")
