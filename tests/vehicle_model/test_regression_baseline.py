"""Golden-value regression tests for the existing vehicle and tyre models (Plan.md, M0).

Each case runs a short deterministic simulation and compares the result with the
stored baseline in ``data/regression_baseline.json``. The baseline freezes the
behaviour of commit 1e45e33 so that refactorings cannot change results unnoticed.

A failing case does not say the model is wrong, only that its output changed.
If a change is intended, regenerate the baseline and justify it in the commit:

    REGRESSION_UPDATE=1 python -m pytest tests/vehicle_model/test_regression_baseline.py
"""

from __future__ import annotations

import json
import os
from pathlib import Path

import numpy as np
import pytest

from vehicle_model.core.types import SevenDOFControlInput, SevenDOFState
from vehicle_model.roads.surface import MuSplitRoad, UniformRoad
from vehicle_model.tire_model import available_tyres, load_tyre
from vehicle_model.vehicle_models import (
    DynamicSingleTrack,
    FourteenDOFConfig,
    FourteenDOFModel,
    FourteenDOFState,
    KinematicSingleTrack,
    SevenDOFModel,
)

BASELINE = Path(__file__).parent / "data" / "regression_baseline.json"
UPDATE = os.getenv("REGRESSION_UPDATE") == "1"
RTOL = 1e-6
ATOL = 1e-9


def _kinematic_single_track() -> np.ndarray:
    model = KinematicSingleTrack()
    t = np.arange(0.0, 2.0 + 1e-9, 0.01)
    x, _ = model.sim_continuous(np.array([0.0, 0.0, 0.0, 10.0, 0.0]), np.array([0.5, 0.1]), t)
    return x[::50]


def _dynamic_single_track() -> np.ndarray:
    model = DynamicSingleTrack()
    t = np.arange(0.0, 1.0 + 1e-9, 0.01)
    x, _ = model.sim_continuous(np.array([0.0, 0.0, 15.0, 0.0, 0.0, 0.0, 0.0]), np.array([0.2, 0.05]), t)
    return x[::25]


def _seven_dof_run(road, steer_f: float, t_brake: float, vx0: float, duration: float) -> np.ndarray:
    model = SevenDOFModel()
    p = model.params
    state = SevenDOFState(vx=vx0, omega=np.full(4, vx0 / p.Reff))
    u = SevenDOFControlInput(steer_f=steer_f, T_brake=np.full(4, t_brake))
    rows = []
    for k in range(int(round(duration / 0.01))):
        state, tel = model.step(state, u, 0.01, road=road)
        if (k + 1) % 25 == 0:
            rows.append(np.concatenate([state.to_array(), tel.Fz]))
    return np.array(rows)


def _seven_dof_step_steer() -> np.ndarray:
    return _seven_dof_run(UniformRoad(mu=1.0), steer_f=0.05, t_brake=0.0, vx0=20.0, duration=2.0)


def _seven_dof_split_mu_braking() -> np.ndarray:
    return _seven_dof_run(MuSplitRoad(mu_left=1.0, mu_right=0.3), steer_f=0.0, t_brake=600.0, vx0=25.0, duration=1.5)


def _fourteen_dof_step_steer() -> np.ndarray:
    cfg = FourteenDOFConfig()
    model = FourteenDOFModel(config=cfg)
    v0 = 20.0
    state = FourteenDOFState(vx=v0, omega=np.full(4, v0 / cfg.wheel_radius))
    delta = np.array([0.04, 0.04, 0.0, 0.0])
    rows = []
    for k in range(200):
        state, _ = model.step(state, delta, np.zeros(4), np.zeros(4), 0.005)
        if (k + 1) % 50 == 0:
            rows.append(state.as_array())
    return np.array(rows)


def _tyre_force_grid() -> np.ndarray:
    kappa = np.array([-0.3, -0.05, 0.0, 0.02, 0.1, 0.5])
    alpha = np.array([-0.2, -0.02, 0.0, 0.03, 0.15, 0.3])
    K, A = np.meshgrid(kappa, alpha)
    K, A = K.ravel(), A.ravel()
    Fz = np.full(K.size, 4000.0)
    mu = np.full(K.size, 0.9)
    out = []
    for name in available_tyres():
        Fx, Fy = load_tyre(name).forces(K, A, Fz, mu)
        out.append(np.concatenate([np.asarray(Fx), np.asarray(Fy)]))
    return np.array(out)


CASES = {
    "kinematic_single_track": _kinematic_single_track,
    "dynamic_single_track": _dynamic_single_track,
    "seven_dof_step_steer": _seven_dof_step_steer,
    "seven_dof_split_mu_braking": _seven_dof_split_mu_braking,
    "fourteen_dof_step_steer": _fourteen_dof_step_steer,
    "tyre_force_grid": _tyre_force_grid,
}


def _load() -> dict:
    if BASELINE.is_file():
        return json.loads(BASELINE.read_text(encoding="utf-8"))
    return {}


@pytest.mark.parametrize("name", list(CASES))
def test_regression_baseline(name):
    result = np.asarray(CASES[name](), dtype=float)
    assert np.isfinite(result).all()

    if UPDATE:
        data = _load()
        data[name] = {"tyres": available_tyres()} if name == "tyre_force_grid" else {}
        data[name]["values"] = result.tolist()
        BASELINE.parent.mkdir(parents=True, exist_ok=True)
        BASELINE.write_text(json.dumps(data, indent=1, sort_keys=True) + "\n", encoding="utf-8")
        pytest.skip("baseline updated")

    data = _load()
    assert name in data, f"No baseline for {name}; run with REGRESSION_UPDATE=1"
    if name == "tyre_force_grid":
        assert data[name]["tyres"] == available_tyres(), "tyre set changed; regenerate baseline"
    np.testing.assert_allclose(result, np.asarray(data[name]["values"]), rtol=RTOL, atol=ATOL)
