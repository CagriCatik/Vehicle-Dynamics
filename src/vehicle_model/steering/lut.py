"""Rear-axle steering lookup table (LUT) model and interpolation.

The LUT encodes the relationship between front-steer angle and rear-steer
angle as a function of vehicle speed. This reflects calibrated Ackermann
rear-axle steering behaviour:
  - Low speed: rear steer is counter-phase to front steer for tighter turning radius.
  - High speed: rear steer is in-phase with front steer for highway stability and sideslip minimization.

The table data is stored in JSON or YAML format:
  vTable: vehicle speed breakpoints [m/s]
  fTable: front steer angle breakpoints [deg]
  rTable: 2D array of rear steer angles [deg], shape (len(vTable), len(fTable))
"""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Iterable, Sequence

import numpy as np
from scipy.interpolate import RegularGridInterpolator


@dataclass(frozen=True)
class LookupTable:
    """Immutable wrapper around a 2-D interpolator for rear-steer angles."""

    velocity_knots: Sequence[float]   # speed breakpoints [m/s]
    front_knots: Sequence[float]      # front steer breakpoints [deg]
    values_deg: np.ndarray            # raw table values [deg], shape (Nv, Nf)
    _interpolator: RegularGridInterpolator

    def desired_rear_deg(self, velocity: float, front_deg: float) -> float:
        """Interpolate and return the rear steer angle in degrees.
        
        Clamps inputs to knot boundaries so extrapolation is flat outside the table.
        """
        v_clamped = float(np.clip(abs(velocity), self.velocity_knots[0], self.velocity_knots[-1]))
        f_clamped = float(np.clip(front_deg, self.front_knots[0], self.front_knots[-1]))
        return float(self._interpolator((v_clamped, f_clamped)))

    def desired_rear_rad(self, velocity: float, front_deg: float) -> float:
        """Interpolate and return the rear steer angle in radians."""
        return float(np.radians(self.desired_rear_deg(velocity, front_deg)))

    def compute_rear_angle(self, vx: float, delta_f_rad: float) -> float:
        """Compute rear steering angle [rad] given velocity [m/s] and front steer [rad]."""
        front_deg = float(np.degrees(delta_f_rad))
        return self.desired_rear_rad(vx, front_deg)


def _to_numpy(values: Iterable[float]) -> np.ndarray:
    """Convert an iterable of numbers to a 1-D float64 numpy array."""
    arr = np.asarray(list(values), dtype=float)
    if arr.ndim != 1:
        raise ValueError("Lookup table axes must be 1-D sequences.")
    return arr


def load_lookup_table(path: str | Path | None = None) -> LookupTable:
    """Read a LUT JSON file and return a ready-to-use LookupTable.

    Parameters
    ----------
    path : str | Path | None
        Path to the JSON table. If None, resolves from centralized configs/steering/lut.json.

    Returns
    -------
    LookupTable
        LookupTable instance with bilinear interpolation configured.
    """
    if path is None:
        from vehicle_model.config.loader import get_configs_root
        candidate = get_configs_root() / "steering" / "lut.json"
        if not candidate.exists():
            # Fallback relative to project root
            candidate = Path("configs/steering/lut.json").resolve()
        path = candidate
    else:
        path = Path(path)

    if not path.exists():
        raise FileNotFoundError(f"Rear-axle steering lookup table not found at: {path}")

    data = json.loads(path.read_text(encoding="utf-8"))
    velocity = _to_numpy(data["vTable"])
    front = _to_numpy(data["fTable"])
    values = np.asarray(data["rTable"], dtype=float)

    # bounds_error=False + fill_value=None -> flat extrapolation outside table
    interpolator = RegularGridInterpolator(
        (velocity, front),
        values,
        bounds_error=False,
        fill_value=None,
    )
    return LookupTable(
        velocity_knots=velocity,
        front_knots=front,
        values_deg=values,
        _interpolator=interpolator,
    )


__all__ = (
    "LookupTable",
    "load_lookup_table",
)
