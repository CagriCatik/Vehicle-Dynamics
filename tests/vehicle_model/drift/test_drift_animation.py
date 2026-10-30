"""Drift plan D6: exports and GIF made from recorded data only (frame mapping, file set, CSV = history)."""

from __future__ import annotations

import csv
import math

import numpy as np
import pytest

from vehicle_model.visualization.drift_animation import frame_indices

FILES = ("run_config_resolved.yaml", "metadata.json", "state_history.csv", "wheel_forces.csv", "controller_diagnostics.csv",
         "kpi.json", "drift_animation.gif", "trajectory.png", "sideslip_and_yaw.png", "wheel_torques_and_slip.png",
         "phase_plane.png", "equilibrium_residuals.png", "report.md")


def test_frames_map_to_recorded_samples_only():
    idx = frame_indices(1201, 0.005, 20, 1.0)            # 6 s at 5 ms, 20 fps real time
    assert idx[0] == 0 and idx[-1] <= 1200 and np.all(np.diff(idx) == 10)
    fast = frame_indices(1201, 0.005, 20, 2.0)
    assert len(fast) == 61 and np.all(np.diff(fast) == 20)
    assert frame_indices(3, 0.005, 20, 1.0).tolist() == [0]


@pytest.fixture(scope="module")
def exported(tmp_path_factory):
    from vehicle_model.core.types import SevenDOFState
    from vehicle_model.drift import DriftEquilibriumSolver, DriftReference
    from vehicle_model.drift.control_adapter import DriftLQRController
    from vehicle_model.drift.linearization import linearise
    from vehicle_model.drift.lqr import design_lqr
    from vehicle_model.drift.path_reference import CircleReference
    from vehicle_model.roads.surface import UniformRoad
    from vehicle_model.simulation.drift.runner import DriftRunResult, export_run, record
    from vehicle_model.vehicle_models import SevenDOFModel
    m = SevenDOFModel.from_yaml()
    sol = DriftEquilibriumSolver(m)
    eq = sol.solve(DriftReference(13.0, math.radians(-20.0), 0.9))
    lqr = design_lqr(linearise(sol.plant(0.9), eq), m.params)
    plant = SevenDOFModel.from_yaml(road=UniformRoad(mu=0.9))
    x0 = SevenDOFState(vx=eq.vx, vy=eq.vy, r=eq.yaw_rate, omega=eq.omega.copy())
    h, diags = record(plant, DriftLQRController(m.params, eq, lqr), x0, 0.6, 0.005)
    circ = CircleReference.through_state(0.0, 0.0, eq.beta, eq.radius, "left")
    res = DriftRunResult("test", "drift_lqr", h, diags, m.params, circ, eq.to_dict(), eq.beta, eq.yaw_rate,
                         {"x": 1.0}, "PASS", {"name": "test"})
    out = export_run(res, tmp_path_factory.mktemp("d6"), gif=True, plots=True, fps=20, dpi=50)
    return out, h, diags


@pytest.mark.slow
def test_every_required_file_is_written(exported):
    out, _, _ = exported
    for f in FILES:
        assert (out / f).is_file(), f


@pytest.mark.slow
def test_csv_reproduces_the_recorded_history(exported):
    out, h, diags = exported
    rows = list(csv.DictReader(open(out / "state_history.csv")))
    assert len(rows) == h.time.size == len(diags)
    X = np.array([float(r["X"]) for r in rows])
    Y = np.array([float(r["Y"]) for r in rows])
    assert np.allclose(X, h.X, rtol=0, atol=1e-9) and np.allclose(Y, h.Y, rtol=0, atol=1e-9)
    assert len(list(csv.DictReader(open(out / "controller_diagnostics.csv")))) == h.time.size


@pytest.mark.slow
def test_gif_opens_with_several_frames_and_finite_data(exported):
    from PIL import Image
    out, h, _ = exported
    im = Image.open(out / "drift_animation.gif")
    assert im.n_frames == len(frame_indices(h.time.size, 0.005, 20, 1.0)) > 1
    assert np.all(np.isfinite(h.X)) and np.all(np.isfinite(h.Y)) and np.all(np.isfinite(h.psi))
