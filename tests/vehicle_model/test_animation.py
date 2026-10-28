"""Unit tests for the real-time vehicle dynamics animation generator."""

from __future__ import annotations

from pathlib import Path
import numpy as np
import pytest

from vehicle_model.simulation.scenario_runner import SimulationHistory
from vehicle_model.visualization.animation import animate_maneuver
from vehicle_model.visualization.plan_view import VehicleParams


@pytest.mark.slow
def test_animate_maneuver_smoke(tmp_path: Path):
    """Verify animate_maneuver generates a valid .gif file."""
    t = np.linspace(0.0, 1.0, 15)
    hist = SimulationHistory(
        time=t,
        vx=np.full_like(t, 20.0),
        vy=np.zeros_like(t),
        r=np.zeros_like(t),
        omega=np.zeros((len(t), 4)),
        X=t * 20.0,
        Y=np.sin(t * 2.0),
        psi=np.zeros_like(t),
        steer_f=np.zeros_like(t),
        steer_r=np.zeros_like(t),
        T_drive=np.zeros((len(t), 4)),
        T_brake=np.zeros((len(t), 4)),
        kappa=np.zeros((len(t), 4)),
        alpha=np.zeros((len(t), 4)),
        Fz=np.zeros((len(t), 4)),
        mu=np.zeros((len(t), 4)),
        Fx_wheel=np.zeros((len(t), 4)),
        Fy_wheel=np.zeros((len(t), 4)),
        Fx_body=np.zeros((len(t), 4)),
        Fy_body=np.zeros((len(t), 4)),
    )

    out_gif = tmp_path / "test_animation.gif"
    res = animate_maneuver(
        histories={"TestCar": hist},
        output_path=out_gif,
        title="Smoke Test Animation",
        fps=10,
        playback_speed=1.0,
        include_closeup=True,
    )

    assert res.exists()
    assert res.stat().st_size > 1000


def test_animate_maneuver_empty_raises():
    """Verify animate_maneuver raises ValueError when histories is empty."""
    with pytest.raises(ValueError, match="At least one"):
        animate_maneuver(histories={}, output_path="/tmp/dummy.gif")


@pytest.mark.slow
def test_animate_maneuver_with_reference_and_pylons(tmp_path: Path):
    """Verify animate_maneuver renders correctly with reference path and cones."""
    t = np.linspace(0.0, 0.5, 8)
    hist = SimulationHistory(
        time=t,
        vx=np.full_like(t, 10.0),
        vy=np.zeros_like(t),
        r=np.zeros_like(t),
        omega=np.zeros((len(t), 4)),
        X=t * 10.0,
        Y=np.sin(t * 3.0),
        psi=np.zeros_like(t),
        steer_f=np.zeros_like(t),
        steer_r=np.zeros_like(t),
        T_drive=np.zeros((len(t), 4)),
        T_brake=np.zeros((len(t), 4)),
        kappa=np.zeros((len(t), 4)),
        alpha=np.zeros((len(t), 4)),
        Fz=np.zeros((len(t), 4)),
        mu=np.zeros((len(t), 4)),
        Fx_wheel=np.zeros((len(t), 4)),
        Fy_wheel=np.zeros((len(t), 4)),
        Fx_body=np.zeros((len(t), 4)),
        Fy_body=np.zeros((len(t), 4)),
    )

    out_gif = tmp_path / "test_ref_pylons.gif"
    ref_x = np.linspace(0.0, 10.0, 50)
    ref_y = np.sin(ref_x)
    pylons = [(2.5, 0.0), (5.0, 0.0)]

    res = animate_maneuver(
        histories={"NMPC_4WS": hist, "FWS": hist},
        output_path=out_gif,
        title="Ref Path & Cones Test",
        fps=10,
        reference_path=(ref_x, ref_y),
        pylons=pylons,
        include_closeup=True,
    )

    assert res.exists()
    assert res.stat().st_size > 1000

