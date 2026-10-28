"""Unit and Physics Tests for Handling Limits and Understeer Analysis.

Conforms to:
- ISO 4138: Steady-State Circular Driving Behaviour
- ISO 8855: Handling Terms and Definitions
- DIN 70000 / Küçükay / Mitschke handling dynamics
"""

from __future__ import annotations

import math
from pathlib import Path
import numpy as np
import pytest

from vehicle_model.controllers import PassiveBaselineController
from vehicle_model.core.types import SevenDOFParams
from vehicle_model.simulation.handling_limits import (
    HandlingCharacteristics,
    compute_gg_diagram,
    evaluate_understeer_gradient,
    plot_handling_and_limits,
    simulate_steady_state_circular_drive,
)
from vehicle_model.simulation.scenario_runner import SimulationHistory
from vehicle_model.vehicle_models.seven_dof import SevenDOFModel


@pytest.fixture
def test_vehicle() -> SevenDOFModel:
    return SevenDOFModel.from_yaml("configs/vehicles/seven_dof_passenger.yaml")


def test_steady_state_circular_simulation(test_vehicle: SevenDOFModel):
    """Verify ISO 4138 circular drive runs with finite numerical states."""
    controller = PassiveBaselineController()
    hist = simulate_steady_state_circular_drive(
        model=test_vehicle,
        controller=controller,
        radius_m=35.0,
        v_start_kph=20.0,
        v_end_kph=35.0,
        speed_ramp_rate_kph_per_s=3.0,
        mu=1.0,
        dt=0.01,
    )

    assert isinstance(hist, SimulationHistory)
    assert len(hist.time) > 100
    assert not np.isnan(hist.vx).any()
    assert not np.isnan(hist.r).any()
    assert not np.isnan(hist.steer_f).any()

    # Vehicle should accelerate smoothly along the circular track
    assert hist.vx[-1] > hist.vx[0]
    assert np.all(hist.vx > 0.0)

    # Lateral acceleration should be positive in left-hand turn (r > 0)
    ay = hist.vx * hist.r
    assert np.all(ay > 0.0)


def test_evaluate_understeer_gradient_understeer():
    """Verify evaluation correctly classifies an understeering vehicle."""
    n_pts = 200
    time = np.linspace(0.0, 10.0, n_pts)
    vx = np.linspace(5.0, 20.0, n_pts)
    r = vx / 40.0  # constant radius R = 40m
    ay = vx * r    # lateral acceleration [m/s^2]

    L = 2.80
    delta_ackermann = math.atan(L / 40.0)
    # Synthetic understeer: steer angle increases linearly with ay
    k_us_target = 0.003  # rad / (m/s^2)
    steer_f = delta_ackermann + k_us_target * ay
    sideslip = -0.001 * ay

    hist = SimulationHistory(
        time=time,
        vx=vx,
        vy=sideslip * vx,
        r=r,
        omega=np.zeros((n_pts, 4)),
        X=np.zeros(n_pts),
        Y=np.zeros(n_pts),
        psi=np.zeros(n_pts),
        steer_f=steer_f,
        steer_r=np.zeros(n_pts),
        T_drive=np.zeros((n_pts, 4)),
        T_brake=np.zeros((n_pts, 4)),
        kappa=np.zeros((n_pts, 4)),
        alpha=np.zeros((n_pts, 4)),
        Fz=np.full((n_pts, 4), 3000.0),
        mu=np.ones((n_pts, 4)),
        Fx_wheel=np.zeros((n_pts, 4)),
        Fy_wheel=np.zeros((n_pts, 4)),
        Fx_body=np.zeros((n_pts, 4)),
        Fy_body=np.zeros((n_pts, 4)),
    )

    eval_res = evaluate_understeer_gradient(
        history=hist,
        wheelbase_m=L,
        radius_m=40.0,
    )

    assert eval_res.classification == "understeer"
    assert eval_res.understeer_gradient_rad_per_ms2 > 0.0
    assert eval_res.understeer_gradient_deg_per_g > 0.0
    # For understeer, characteristic speed must be real and positive
    assert eval_res.characteristic_speed_mps is not None
    assert eval_res.characteristic_speed_mps > 0.0
    # Critical speed is undefined (None) for understeering cars
    assert eval_res.critical_speed_mps is None


def test_evaluate_understeer_gradient_oversteer():
    """Verify evaluation correctly identifies oversteer and computes critical speed."""
    n_pts = 200
    time = np.linspace(0.0, 10.0, n_pts)
    vx = np.linspace(5.0, 20.0, n_pts)
    r = vx / 40.0
    ay = vx * r

    L = 2.80
    delta_ackermann = math.atan(L / 40.0)
    # Synthetic oversteer: steer angle decreases with ay
    k_us_target = -0.002  # rad / (m/s^2)
    steer_f = delta_ackermann + k_us_target * ay
    sideslip = 0.002 * ay

    hist = SimulationHistory(
        time=time,
        vx=vx,
        vy=sideslip * vx,
        r=r,
        omega=np.zeros((n_pts, 4)),
        X=np.zeros(n_pts),
        Y=np.zeros(n_pts),
        psi=np.zeros(n_pts),
        steer_f=steer_f,
        steer_r=np.zeros(n_pts),
        T_drive=np.zeros((n_pts, 4)),
        T_brake=np.zeros((n_pts, 4)),
        kappa=np.zeros((n_pts, 4)),
        alpha=np.zeros((n_pts, 4)),
        Fz=np.full((n_pts, 4), 3000.0),
        mu=np.ones((n_pts, 4)),
        Fx_wheel=np.zeros((n_pts, 4)),
        Fy_wheel=np.zeros((n_pts, 4)),
        Fx_body=np.zeros((n_pts, 4)),
        Fy_body=np.zeros((n_pts, 4)),
    )

    eval_res = evaluate_understeer_gradient(
        history=hist,
        wheelbase_m=L,
        radius_m=40.0,
    )

    assert eval_res.classification == "oversteer"
    assert eval_res.understeer_gradient_rad_per_ms2 < 0.0
    assert eval_res.understeer_gradient_deg_per_g < 0.0
    # Oversteer cars have critical speed (instability boundary)
    assert eval_res.critical_speed_mps is not None
    assert eval_res.critical_speed_mps > 0.0
    assert eval_res.characteristic_speed_mps is None


def test_compute_gg_diagram():
    """Verify G-G envelope physics and friction scaling."""
    params = SevenDOFParams()

    gg_dry = compute_gg_diagram(params, mu=1.0)
    gg_wet = compute_gg_diagram(params, mu=0.5)

    assert gg_dry.mu == 1.0
    assert gg_wet.mu == 0.5

    # Dry envelope must strictly enclose wet envelope (higher lateral and braking capacity)
    assert gg_dry.ay_limit_positive_g > gg_wet.ay_limit_positive_g
    assert abs(gg_dry.ax_brake_limit_g) > abs(gg_wet.ax_brake_limit_g)

    # Friction circle limits cannot exceed mu*g
    assert gg_dry.ay_limit_positive_g <= 1.05 * 1.0
    assert gg_wet.ay_limit_positive_g <= 1.05 * 0.5

    # Boundary arrays must be valid closed contours
    assert len(gg_dry.boundary_ay_g) == len(gg_dry.boundary_ax_g)
    assert not np.isnan(gg_dry.boundary_ay_g).any()
    assert not np.isnan(gg_dry.boundary_ax_g).any()


def test_plot_handling_and_limits(tmp_path: Path, test_vehicle: SevenDOFModel):
    """Verify handling and limits 4-panel diagnostic dashboard renders to disk."""
    controller = PassiveBaselineController()
    hist = simulate_steady_state_circular_drive(
        model=test_vehicle,
        controller=controller,
        radius_m=35.0,
        v_start_kph=18.0,
        v_end_kph=30.0,
        speed_ramp_rate_kph_per_s=4.0,
        mu=1.0,
        dt=0.01,
    )

    h_res = evaluate_understeer_gradient(
        history=hist,
        model_params=test_vehicle.params,
        radius_m=35.0,
        controller_name="Passive",
    )

    gg_res = compute_gg_diagram(test_vehicle.params, mu=1.0)

    out_file = tmp_path / "test_dashboard.png"
    plot_handling_and_limits(
        histories={"Passive": hist},
        handling_results={"Passive": h_res},
        gg_result=gg_res,
        save_path=out_file,
    )

    assert out_file.exists()
    assert out_file.stat().st_size > 1000  # File should be non-empty image
