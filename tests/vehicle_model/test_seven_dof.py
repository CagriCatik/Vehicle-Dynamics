"""Unit and physical consistency tests for the 7-DOF two-track vehicle dynamics model."""

from __future__ import annotations

import math
import numpy as np
import pytest

from vehicle_model.core.types import (
    FL,
    FR,
    RL,
    RR,
    SevenDOFControlInput,
    SevenDOFParams,
    SevenDOFState,
)
from vehicle_model.roads.surface import MuSplitRoad, UniformRoad
from vehicle_model.steering.four_wheel_steering import FourWheelSteering, SteeringConfig, SteeringMode
from vehicle_model.tire_model.combined_slip import CombinedSlipPacejkaParams, CombinedSlipTireModel
from vehicle_model.vehicle_models.seven_dof import SevenDOFModel


@pytest.fixture
def standard_params() -> SevenDOFParams:
    return SevenDOFParams(
        mass=1200.0,
        Iz=1800.0,
        lf=1.30,
        lr=1.50,
        wf=1.60,
        wr=1.60,
        hcog=0.50,
        Iw=1.50,
        Reff=0.32,
    )


@pytest.fixture
def seven_dof_model(standard_params: SevenDOFParams) -> SevenDOFModel:
    return SevenDOFModel(params=standard_params)


def test_seven_dof_state_serialization():
    """State to array and array to state roundtrip."""
    state = SevenDOFState(
        vx=15.0,
        vy=-0.5,
        r=0.12,
        omega=np.array([45.0, 46.0, 44.5, 45.2]),
        X=100.0,
        Y=-25.0,
        psi=0.35,
    )
    arr = state.to_array()
    assert arr.shape == (10,)
    assert arr[0] == 15.0
    assert arr[2] == 0.12
    assert arr[9] == 0.35

    restored = SevenDOFState.from_array(arr)
    assert math.isclose(restored.vx, state.vx)
    assert math.isclose(restored.vy, state.vy)
    assert math.isclose(restored.r, state.r)
    np.testing.assert_allclose(restored.omega, state.omega)
    assert math.isclose(restored.X, state.X)
    assert math.isclose(restored.Y, state.Y)
    assert math.isclose(restored.psi, state.psi)


def test_normal_load_conservation(seven_dof_model: SevenDOFModel, standard_params: SevenDOFParams):
    """Verify sum(Fz) == m * g under all static and dynamic acceleration cases."""
    mg = standard_params.mass * standard_params.g

    test_cases = [
        (0.0, 0.0),    # Static rest
        (3.0, 0.0),    # Acceleration
        (-6.0, 0.0),   # Heavy braking
        (0.0, 5.0),    # Steady cornering
        (2.0, 4.0),    # Combined acceleration and cornering
        (-5.0, 8.0),   # Severe trail-braking with inside wheel unweighting
        (10.0, 15.0),  # Extreme limits (wheel lift)
    ]

    for ax, ay in test_cases:
        Fz = seven_dof_model.compute_normal_loads(ax, ay)
        assert len(Fz) == 4
        # Physical constraint: no negative normal loads (tires cannot pull the ground)
        assert np.all(Fz >= 0.0), f"Negative Fz detected for ax={ax}, ay={ay}: {Fz}"
        # Conservation of vertical force on level ground
        assert math.isclose(np.sum(Fz), mg, rel_tol=1e-9), f"Load conservation violated for ax={ax}, ay={ay}: sum={np.sum(Fz)}, expected={mg}"


def test_dynamic_load_transfer_direction(seven_dof_model: SevenDOFModel):
    """Verify that load transfer follows physical laws."""
    Fz_static = seven_dof_model.compute_normal_loads(0.0, 0.0)

    # 1. Longitudinal acceleration (ax > 0) -> weight transfers to rear
    Fz_acc = seven_dof_model.compute_normal_loads(3.0, 0.0)
    assert (Fz_acc[RL] + Fz_acc[RR]) > (Fz_static[RL] + Fz_static[RR])
    assert (Fz_acc[FL] + Fz_acc[FR]) < (Fz_static[FL] + Fz_static[FR])

    # 2. Braking (ax < 0) -> weight transfers to front
    Fz_brake = seven_dof_model.compute_normal_loads(-5.0, 0.0)
    assert (Fz_brake[FL] + Fz_brake[FR]) > (Fz_static[FL] + Fz_static[FR])
    assert (Fz_brake[RL] + Fz_brake[RR]) < (Fz_static[RL] + Fz_static[RR])

    # 3. Left turn (ay > 0) -> centrifugal force transfers weight to right (outer) wheels
    Fz_turn = seven_dof_model.compute_normal_loads(0.0, 4.0)
    assert (Fz_turn[FR] + Fz_turn[RR]) > (Fz_turn[FL] + Fz_turn[RL])


def test_combined_slip_friction_circle_limit():
    """Verify that resultant force never exceeds Kamm's friction circle limit mu * Fz."""
    tire = CombinedSlipTireModel()
    Fz = 3000.0
    mu = 0.85
    F_max = mu * Fz

    kappas = np.linspace(-0.5, 0.5, 21)
    alphas = np.linspace(-0.3, 0.3, 21)

    for k in kappas:
        for a in alphas:
            Fx, Fy = tire.compute_forces(float(k), float(a), Fz, mu)
            resultant = math.hypot(Fx, Fy)
            assert resultant <= F_max + 1e-6, f"Friction circle exceeded: resultant={resultant}, F_max={F_max}"


def test_combined_slip_lateral_reduction_under_braking():
    """Verify that increasing longitudinal slip degrades available lateral force."""
    tire = CombinedSlipTireModel()
    Fz = 3000.0
    mu = 1.0
    alpha = math.radians(4.0)  # Moderate cornering slip angle

    # Pure lateral force at zero braking slip
    _, Fy_pure = tire.compute_forces(kappa=0.0, alpha=alpha, Fz=Fz, mu=mu)

    # Lateral force under heavy braking slip (kappa = -0.3)
    _, Fy_braking = tire.compute_forces(kappa=-0.30, alpha=alpha, Fz=Fz, mu=mu)

    # Lateral force under full wheel lockup (kappa = -1.0)
    _, Fy_lockup = tire.compute_forces(kappa=-1.0, alpha=alpha, Fz=Fz, mu=mu)

    assert Fy_pure > Fy_braking > Fy_lockup
    assert Fy_lockup < 0.30 * Fy_pure, "Under wheel lockup, lateral force capability must drastically drop"


def test_wheel_spin_on_low_friction(seven_dof_model: SevenDOFModel):
    """High drive torque on low-mu road causes wheel spin (kappa >> 0)."""
    low_mu_road = UniformRoad(mu=0.2)
    state = SevenDOFState(
        vx=5.0,
        vy=0.0,
        r=0.0,
        omega=np.full(4, 5.0 / 0.32),
    )
    # Apply huge drive torque to rear wheels (800 Nm each)
    u = SevenDOFControlInput(
        T_drive=np.array([0.0, 0.0, 800.0, 800.0]),
    )

    next_state, telemetry = seven_dof_model.step(state, u, dt=0.05, road=low_mu_road)

    # Rear wheels must accelerate much faster than the vehicle body
    assert next_state.omega[RL] > state.omega[RL]
    assert telemetry.kappa[RL] > 0.30, f"Expected wheel spin, got kappa={telemetry.kappa[RL]}"


def test_wheel_lockup_on_low_friction(seven_dof_model: SevenDOFModel):
    """Heavy braking torque on low-mu road without ABS induces wheel lockup (kappa -> -1.0)."""
    low_mu_road = UniformRoad(mu=0.2)
    v0 = 15.0
    state = SevenDOFState(
        vx=v0,
        vy=0.0,
        r=0.0,
        omega=np.full(4, v0 / 0.32),
    )
    u = SevenDOFControlInput(
        T_brake=np.full(4, 1500.0),
    )

    # Simulate 0.3s of unmodulated braking
    s = state
    for _ in range(30):
        s, tele = seven_dof_model.step(s, u, dt=0.01, road=low_mu_road)

    # Wheels should decelerate rapidly and lock up
    assert np.all(tele.kappa < -0.80), f"Expected lockup on low mu, got kappa={tele.kappa}"
    assert s.vx > 10.0, "Vehicle body cannot stop in 0.3s on low mu, while wheels lock"


def test_torque_vectoring_generates_direct_yaw_moment(seven_dof_model: SevenDOFModel):
    """Differential rear drive torque creates positive yaw moment without steering angle."""
    state = SevenDOFState(
        vx=15.0,
        vy=0.0,
        r=0.0,
        omega=np.full(4, 15.0 / 0.32),
    )
    # Zero steer angle, but 500 Nm more torque on RR than RL
    u = SevenDOFControlInput(
        steer_f=0.0,
        steer_r=0.0,
        T_drive=np.array([100.0, 100.0, 50.0, 550.0]),
    )

    # Step forward 20 ms to let wheel inertia accelerate and develop slip
    next_state, telemetry = seven_dof_model.step(state, u, dt=0.02)
    dstate, _, _, _ = seven_dof_model.evaluate_dynamics(next_state, u)
    dr_dt = dstate[2]

    # Positive yaw acceleration (turning left) due to right wheel thrust
    assert dr_dt > 0.10, f"Expected positive yaw acceleration from TV, got dr_dt={dr_dt}"
    assert telemetry.Fx_wheel[RR] > telemetry.Fx_wheel[RL]


def test_four_wheel_steering_counter_phase_agility(seven_dof_model: SevenDOFModel):
    """At low speed, counter-phase 4WS increases yaw rate vs front-only steering."""
    fws = FourWheelSteering(SteeringConfig(v_switch=12.0, k_low_speed=-0.35))
    steer_f_cmd = math.radians(10.0)

    # Front-only
    sf_fo, sr_fo = fws.compute_steering_angles(steer_f_cmd, vx=6.0, mode=SteeringMode.FRONT_ONLY)
    assert sr_fo == 0.0

    # 4WS
    sf_4ws, sr_4ws = fws.compute_steering_angles(steer_f_cmd, vx=6.0, mode=SteeringMode.SPEED_SCHEDULED)
    assert sr_4ws < -0.02, f"Expected negative counter-phase rear steer, got {sr_4ws}"

    state = SevenDOFState(vx=6.0, vy=0.0, r=0.0, omega=np.full(4, 6.0 / 0.32))

    u_fo = SevenDOFControlInput(steer_f=sf_fo, steer_r=sr_fo)
    u_4ws = SevenDOFControlInput(steer_f=sf_4ws, steer_r=sr_4ws)

    dstate_fo, _, _, _ = seven_dof_model.evaluate_dynamics(state, u_fo)
    dstate_4ws, _, _, _ = seven_dof_model.evaluate_dynamics(state, u_4ws)

    # Counter-phase 4WS must produce significantly higher yaw acceleration for agility
    assert dstate_4ws[2] > dstate_fo[2] * 1.25


def test_mu_split_straight_line_braking_perturbation(seven_dof_model: SevenDOFModel):
    """Braking in a straight line on mu-split induces perturbing yaw moment towards high-mu side."""
    # Left wheels at Y > 0 (high mu = 1.0), right wheels at Y < 0 (low mu = 0.2)
    road = MuSplitRoad(mu_left=1.0, mu_right=0.2, split_y=0.0)

    state = SevenDOFState(
        vx=15.0,
        vy=0.0,
        r=0.0,
        omega=np.full(4, 15.0 / 0.32),
        X=0.0,
        Y=0.0,  # Centered on split line
        psi=0.0,
    )
    # Symmetric brake demand, zero steering
    u = SevenDOFControlInput(
        steer_f=0.0,
        steer_r=0.0,
        T_brake=np.full(4, 800.0),
    )

    # Step forward 20 ms to develop brake slip
    next_state, telemetry = seven_dof_model.step(state, u, dt=0.02, road=road)
    dstate, _, _, _ = seven_dof_model.evaluate_dynamics(next_state, u, road=road)
    dr_dt = dstate[2]

    # Left wheels (FL, RL) have mu=1.0, right wheels (FR, RR) have mu=0.2
    assert telemetry.mu[FL] > 0.8
    assert telemetry.mu[FR] < 0.4
    assert abs(telemetry.Fx_wheel[FL]) > abs(telemetry.Fx_wheel[FR]) * 1.5

    # Left brake drag pulls vehicle to the left (counter-clockwise => positive Mz/dr_dt in ISO)
    assert dr_dt > 0.10, f"Expected perturbing yaw acceleration towards high-mu side, got dr_dt={dr_dt}"

