"""Unit and physics tests for active chassis control systems (ABS, TCS, TV, Integrated)."""

from __future__ import annotations

import math
import numpy as np
import pytest

from vehicle_model.controllers.abs import ABSConfig, AntilockBrakingSystem
from vehicle_model.controllers.integrated_controller import IntegratedChassisController
from vehicle_model.controllers.tcs import TCSConfig, TractionControlSystem
from vehicle_model.controllers.torque_vectoring import TorqueVectoringConfig, TorqueVectoringController
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
from vehicle_model.simulation.scenario_runner import ScenarioRunner
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


def test_abs_prevents_wheel_lockup_on_ice(standard_params: SevenDOFParams):
    """Verify ABS prevents wheel lockup (kappa -> -1.0) under severe braking on ice."""
    model = SevenDOFModel(params=standard_params)
    road = UniformRoad(mu=0.20)
    abs_sys = AntilockBrakingSystem(ABSConfig(slip_threshold=-0.18, target_slip=-0.14))

    v0 = 15.0
    omega0 = v0 / standard_params.Reff
    init_state = SevenDOFState(
        vx=v0, vy=0.0, r=0.0, omega=np.full(4, omega0), X=0.0, Y=0.0, psi=0.0
    )

    # 1. Unregulated braking: 1400 Nm per wheel on ice
    def unreg_ctrl(t, state, tele):
        return SevenDOFControlInput(T_brake=np.full(4, 1400.0))

    runner = ScenarioRunner(model=model, road=road)
    hist_unreg = runner.run(unreg_ctrl, init_state, duration=1.0, dt=0.005)

    # Unregulated wheels must lock up completely
    min_slip_unreg = np.min(hist_unreg.kappa)
    assert min_slip_unreg < -0.90, f"Expected lockup without ABS, got {min_slip_unreg}"

    # 2. ABS-regulated braking
    def abs_ctrl(t, state, tele):
        demanded = np.full(4, 1400.0)
        modulated = abs_sys.modulate_brake_torques(demanded, tele, state.vx)
        return SevenDOFControlInput(T_brake=modulated)

    hist_abs = runner.run(abs_ctrl, init_state, duration=1.0, dt=0.005)

    # ABS must prevent lockup and maintain slip near optimal zone
    min_slip_abs = np.min(hist_abs.kappa)
    assert min_slip_abs > -0.30, f"ABS failed to prevent lockup: min slip was {min_slip_abs}"
    # Wheel angular speed must remain positive
    assert np.all(hist_abs.omega > 5.0), "ABS allowed wheels to stop spinning"


def test_abs_preserves_steerability_during_braking(standard_params: SevenDOFParams):
    """Simultaneous steering and heavy braking: ABS preserves lateral force and allows vehicle to turn."""
    model = SevenDOFModel(params=standard_params)
    road = UniformRoad(mu=0.40)  # Wet asphalt
    abs_sys = AntilockBrakingSystem()

    v0 = 18.0
    omega0 = v0 / standard_params.Reff
    init_state = SevenDOFState(vx=v0, vy=0.0, r=0.0, omega=np.full(4, omega0))

    steer_f = math.radians(12.0)  # Command significant turn

    # Unregulated
    def unreg_ctrl(t, state, tele):
        return SevenDOFControlInput(steer_f=steer_f, T_brake=np.full(4, 1600.0))

    runner = ScenarioRunner(model=model, road=road)
    hist_unreg = runner.run(unreg_ctrl, init_state, duration=1.5, dt=0.005)

    # ABS regulated
    def abs_ctrl(t, state, tele):
        T_b = abs_sys.modulate_brake_torques(np.full(4, 1600.0), tele, state.vx)
        return SevenDOFControlInput(steer_f=steer_f, T_brake=T_b)

    hist_abs = runner.run(abs_ctrl, init_state, duration=1.5, dt=0.005)

    # Unregulated locks front wheels, losing lateral force Fy -> travels mostly straight
    # ABS preserves lateral force -> turns substantially more (higher final Y position)
    assert hist_abs.Y[-1] > hist_unreg.Y[-1] + 0.50, (
        f"ABS did not preserve steerability: Y_abs={hist_abs.Y[-1]}, Y_unreg={hist_unreg.Y[-1]}"
    )


def test_tcs_prevents_wheel_spin(standard_params: SevenDOFParams):
    """TCS prevents excessive drive slip when launching on slippery surface."""
    model = SevenDOFModel(params=standard_params)
    road = UniformRoad(mu=0.25)
    tcs = TractionControlSystem(TCSConfig(slip_threshold=0.18, target_slip=0.12))

    init_state = SevenDOFState(
        vx=3.0, vy=0.0, r=0.0, omega=np.full(4, 3.0 / standard_params.Reff)
    )

    # Unregulated: full throttle 700 Nm on rear wheels
    def unreg_ctrl(t, state, tele):
        return SevenDOFControlInput(T_drive=np.array([0.0, 0.0, 700.0, 700.0]))

    runner = ScenarioRunner(model=model, road=road)
    hist_unreg = runner.run(unreg_ctrl, init_state, duration=1.0, dt=0.005)
    max_slip_unreg = np.max(hist_unreg.kappa)
    assert max_slip_unreg > 0.60, f"Expected wheelspin, got {max_slip_unreg}"

    # TCS regulated
    def tcs_ctrl(t, state, tele):
        demanded = np.array([0.0, 0.0, 700.0, 700.0])
        modulated = tcs.modulate_drive_torques(demanded, tele)
        return SevenDOFControlInput(T_drive=modulated)

    hist_tcs = runner.run(tcs_ctrl, init_state, duration=1.0, dt=0.005)
    max_slip_tcs = np.max(hist_tcs.kappa)
    assert max_slip_tcs < 0.28, f"TCS failed to prevent wheelspin: max slip was {max_slip_tcs}"


@pytest.mark.slow
def test_integrated_controller_mu_split_braking(standard_params: SevenDOFParams):
    """Integrated controller maintains vehicle stability and heading during mu-split emergency braking."""
    model = SevenDOFModel(params=standard_params)
    # Left mu = 1.0, Right mu = 0.2
    road = MuSplitRoad(mu_left=1.0, mu_right=0.2, split_y=0.0)

    v0 = 20.0
    omega0 = v0 / standard_params.Reff
    init_state = SevenDOFState(
        vx=v0, vy=0.0, r=0.0, omega=np.full(4, omega0), X=0.0, Y=0.0, psi=0.0
    )

    # Unregulated: driver commands 1500 Nm straight-line braking
    def unreg_ctrl(t, state, tele):
        return SevenDOFControlInput(steer_f=0.0, T_brake=np.full(4, 1500.0))

    runner = ScenarioRunner(model=model, road=road)
    hist_unreg = runner.run(unreg_ctrl, init_state, duration=1.5, dt=0.005)

    # Integrated Controller (ABS + TV/ESC + 4WS)
    ivmc = IntegratedChassisController(
        params=standard_params,
        enable_4ws=True,
        enable_tv=True,
        enable_abs=True,
        enable_tcs=True,
    )

    def ivmc_ctrl(t, state, tele):
        return ivmc.compute_control(
            t=t,
            state=state,
            driver_steer_f=0.0,
            driver_T_drive=np.zeros(4),
            driver_T_brake=np.full(4, 1500.0),
            telemetry=tele,
            dt=0.005,
            mu_est=0.6,
        )

    hist_ivmc = runner.run(ivmc_ctrl, init_state, duration=1.5, dt=0.005)

    # In unregulated mu-split braking, the vehicle spins aggressively due to asymmetric brake force
    max_yaw_rate_unreg = np.max(np.abs(hist_unreg.r))
    max_yaw_rate_ivmc = np.max(np.abs(hist_ivmc.r))

    # IVMC must significantly attenuate perturbing yaw rate (>50% reduction)
    assert max_yaw_rate_ivmc < max_yaw_rate_unreg * 0.55, (
        f"IVMC failed to stabilize mu-split yaw perturbation: "
        f"r_ivmc={max_yaw_rate_ivmc:.3f}, r_unreg={max_yaw_rate_unreg:.3f}"
    )
    # IVMC stops with much smaller heading deviation (>55% reduction)
    assert abs(hist_ivmc.psi[-1]) < abs(hist_unreg.psi[-1]) * 0.45

