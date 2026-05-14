"""
Rear-axle steering lookup table (LUT) loader.

The LUT encodes the relationship between front-steer angle and rear-steer
angle as a function of vehicle speed.  This reflects the calibrated
Ackermann rear-axle steering behaviour of the physical vehicle:

  - At low speed:  rear steer is in-phase with front steer (tighter turns)
  - At high speed: rear steer is counter-phase to front steer (stability)

The table is stored in lut.json:
  vTable : vehicle speed breakpoints [m/s]
  fTable : front steer angle breakpoints [degrees]
  rTable : 2-D array of rear steer angles [degrees], shape (len(vTable), len(fTable))

scipy's RegularGridInterpolator performs bilinear interpolation between
breakpoints and clamps (extrapolates flat) outside the table range.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence

import numpy as np
from scipy.interpolate import RegularGridInterpolator


@dataclass(frozen=True)
class LookupTable:
    """
    Immutable wrapper around a 2-D interpolator for rear-steer angles.

    Frozen so that the LUT cannot be accidentally mutated after loading.
    """

    velocity_knots: Sequence[float]   # speed breakpoints [m/s]
    front_knots:    Sequence[float]   # front steer breakpoints [deg]
    values_deg:     np.ndarray        # raw table values [deg], shape (Nv, Nf)
    _interpolator:  RegularGridInterpolator

    def desired_rear_deg(self, velocity: float, front_deg: float) -> float:
        """Interpolate and return the rear steer angle in degrees."""
        return float(self._interpolator((velocity, front_deg)))

    def desired_rear_rad(self, velocity: float, front_deg: float) -> float:
        """Interpolate and return the rear steer angle in radians."""
        return np.radians(self.desired_rear_deg(velocity, front_deg))


def _to_numpy(values: Iterable[float]) -> np.ndarray:
    """Convert a list/iterable to a 1-D float64 numpy array."""
    arr = np.asarray(list(values), dtype=float)
    if arr.ndim != 1:
        raise ValueError("Lookup table axes must be 1-D sequences.")
    return arr


def load_lookup_table(path: str | Path) -> LookupTable:
    """
    Read a LUT JSON file and return a ready-to-use LookupTable.

    Parameters
    ----------
    path : path to the JSON file (absolute or relative to working directory)

    Returns
    -------
    LookupTable instance with bilinear interpolation configured
    """
    data         = json.loads(Path(path).read_text(encoding="utf-8"))
    velocity     = _to_numpy(data["vTable"])
    front        = _to_numpy(data["fTable"])
    values       = np.asarray(data["rTable"], dtype=float)

    # bounds_error=False + fill_value=None → flat extrapolation outside table.
    # This prevents crashes when v or steer angle slightly exceeds table bounds.
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
