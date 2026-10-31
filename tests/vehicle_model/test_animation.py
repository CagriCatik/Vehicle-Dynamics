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



def _ride_history(n: int = 12) -> dict:
    t = np.arange(1, n + 1) * 0.1
    h = {"t": t, "s_road": 10.0 * t, "zs": 0.01 * np.sin(t), "theta": 0.01 * np.sin(t), "ddzs": np.cos(t), "Fz_FL": 3000.0 + 0 * t}
    for w in ("FL", "RL"):
        h[f"zr_{w}"], h[f"zu_{w}"] = 0.01 * np.sin(t), 0.01 * np.sin(t)
    return h


def _slope_history(n: int = 12) -> dict:
    t = np.arange(1, n + 1) * 0.5
    return {"t": t, "X": 3.0 * t, "Z": -0.6 * t, "speed": 3.0 + 0 * t, "grade_pct": -20.0 + 0 * t, "abs_active_n": 0 * t,
            "hdc_active": 1.0 + 0 * t, "hdc_infeasible": 0 * t}


def test_side_views_share_one_vehicle_and_render(tmp_path: Path):
    from vehicle_model.visualization.run_animation import animate_side_view, animate_slope
    a = animate_side_view([("passive", _ride_history()), ("skyhook", _ride_history())], tmp_path / "ride.gif", "ride",
                          [("pitch", "theta", 57.3, "deg")], max_frames=4)
    b = animate_slope([("HDC", _slope_history())], tmp_path / "slope.gif", "slope", [("speed", "speed", 3.6, "km/h")], v_set_kph=10.0, max_frames=4)
    assert a.stat().st_size > 0 and b.stat().st_size > 0


def test_plan_view_scene_elements_and_history_conversion(tmp_path: Path):
    from types import SimpleNamespace

    from vehicle_model.visualization.run_animation import animate_runs, history_dict
    t = np.arange(1, 13) * 0.1
    z = 0 * t
    r = SimpleNamespace(time=t, X=20.0 * t, Y=z, psi=z, vx=20.0 + z, vy=2.0 + z, r=z, steer_f_deg=2.0 + z, steer_r_deg=z)
    h = history_dict(r)
    assert h["steer_f"][0] == pytest.approx(np.radians(2.0)) and h["speed"][0] == pytest.approx(np.hypot(20.0, 2.0))
    assert h["beta"][0] == pytest.approx(np.arctan2(2.0, 20.0))
    calls = []
    out = animate_runs([("run", h)], tmp_path / "plan.gif", "plan", decorate=lambda ax, x, y: calls.append(ax), reference=(h["X"], h["Y"]),
                       pylons=[(5.0, 1.0)], max_frames=4, footer="note")
    assert out.stat().st_size > 0 and len(calls) == 2                # overview and close-up
