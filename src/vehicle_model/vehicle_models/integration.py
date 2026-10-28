"""Numerical integration helpers shared by vehicle models."""

from __future__ import annotations

from collections.abc import Callable

import numpy as np


def rk4_step(
    derivative: Callable[..., np.ndarray],
    state: np.ndarray,
    dt: float,
    *args,
) -> np.ndarray:
    """Advance one step with classical fourth-order Runge-Kutta."""
    y = np.asarray(state, dtype=float)
    k1 = derivative(None, y, *args)
    k2 = derivative(None, y + 0.5 * dt * k1, *args)
    k3 = derivative(None, y + 0.5 * dt * k2, *args)
    k4 = derivative(None, y + dt * k3, *args)
    return y + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)


def input_at(inputs, index: int, expected_size: int) -> np.ndarray:
    """Return input vector at ``index`` from vector, NxM, or MxN input arrays."""
    arr = np.asarray(inputs, dtype=float)
    if arr.ndim == 1:
        if arr.size != expected_size:
            raise ValueError(f"Expected {expected_size} inputs, got {arr.size}.")
        return arr
    if arr.ndim != 2:
        raise ValueError("Inputs must be a vector or 2D array.")
    if arr.shape[1] == expected_size:
        return arr[min(index, arr.shape[0] - 1), :]
    if arr.shape[0] == expected_size:
        return arr[:, min(index, arr.shape[1] - 1)]
    raise ValueError(f"Cannot interpret input array with shape {arr.shape}.")


def scalar_at(values, index: int, default: float = 0.0) -> float:
    if values is None:
        return default
    arr = np.asarray(values, dtype=float)
    if arr.ndim == 0:
        return float(arr)
    return float(arr[min(index, arr.shape[0] - 1)])
