"""Physics verification and unit tests for suspension systems ().

Verifies:
- Quarter-car modal frequencies (bounce ~1-2 Hz, hop ~10-14 Hz)
- Static load equilibrium and total normal force conservation
- Semi-active Skyhook comfort improvement (sprung acceleration attenuation)
- Semi-active Groundhook tyre contact force stabilization
- 14-DOF full vehicle roll, pitch, heave and suspension deflection dynamicsmd estimation robustness).
"""

import math
import numpy as np
import pytest

from vehicle_model.suspension.quarter_car import (
    QuarterCarConfig,
    QuarterCarModel,
    QuarterCarState,
)
from vehicle_model.suspension.controllers import (
    ActiveSuspensionController,
    GroundhookController,
    HybridSuspensionController,
    SemiActiveConfig,
    SkyhookController,
)
from vehicle_model.vehicle_models.fourteen_dof import (
    FourteenDOFConfig,
    FourteenDOFModel,
    FourteenDOFState,
)


def test_quarter_car_modal_analysis():
    """Verify quarter-car eigenvalues match realistic automotive bounce and hop modes."""
    cfg = QuarterCarConfig(
        sprung_mass=375.0,
        unsprung_mass=45.0,
        suspension_stiffness=35000.0,
        suspension_damping=2200.0,
        tyre_stiffness=210000.0,
        tyre_damping=250.0,
    )
    model = QuarterCarModel(cfg)
    f_bounce, f_hop, zeta_bounce, zeta_hop = model.modal_analysis()

    # Body bounce frequency should be between 1.0 and 2.0 Hz
    assert 1.0 <= f_bounce <= 2.0, f"Unexpected bounce frequency: {f_bounce:.2f} Hz"
    # Wheel hop frequency should be between 9.0 and 15.0 Hz
    assert 9.0 <= f_hop <= 15.0, f"Unexpected hop frequency: {f_hop:.2f} Hz"
    # Damping ratios should be positive and sub-critical
    assert 0.15 <= zeta_bounce <= 0.60
    assert 0.10 <= zeta_hop <= 0.70


def test_quarter_car_static_equilibrium():
    """Verify quarter-car at rest yields zero acceleration and correct static tyre load."""
    model = QuarterCarModel()
    state = QuarterCarState()
    derivs, outputs = model.evaluate_derivatives(state, road_z=0.0, road_dz=0.0)

    # Accelerations should be zero
    assert abs(outputs.ddzs) < 1e-6
    assert abs(outputs.ddzu) < 1e-6

    # Normal tyre load should equal total static weight
    expected_weight = (model.cfg.sprung_mass + model.cfg.unsprung_mass) * 9.80665
    assert abs(outputs.total_tyre_load - expected_weight) < 1e-4


def test_skyhook_damping_attenuates_sprung_acceleration():
    """Verify semi-active Skyhook controller reduces sprung mass acceleration over road bump."""
    cfg = QuarterCarConfig()
    model = QuarterCarModel(cfg)

    semi_cfg = SemiActiveConfig(c_sky=3500.0, c_min=700.0, c_max=4500.0)
    skyhook = SkyhookController(config=semi_cfg, continuous=True)

    dt = 0.002
    t_end = 2.0
    steps = int(t_end / dt)

    # Road bump profile: 0.06m height, 0.2s duration
    bump_height = 0.06
    bump_duration = 0.2

    def get_road(t: float):
        if 0.1 <= t <= 0.1 + bump_duration:
            tau = (t - 0.1) / bump_duration
            z = 0.5 * bump_height * (1.0 - math.cos(2.0 * math.pi * tau))
            dz = 0.5 * bump_height * (2.0 * math.pi / bump_duration) * math.sin(2.0 * math.pi * tau)
            return z, dz
        return 0.0, 0.0

    # 1. Simulate Passive
    state_passive = QuarterCarState()
    acc_passive = []
    for step in range(steps):
        t = step * dt
        rz, rdz = get_road(t)
        state_passive, out = model.step(state_passive, rz, rdz, dt, damper_c=cfg.suspension_damping)
        acc_passive.append(out.ddzs)

    # 2. Simulate Skyhook
    state_skyhook = QuarterCarState()
    acc_skyhook = []
    for step in range(steps):
        t = step * dt
        rz, rdz = get_road(t)
        c_demanded = skyhook.compute_damping(state_skyhook.dzs, state_skyhook.dzu, dt)
        state_skyhook, out = model.step(state_skyhook, rz, rdz, dt, damper_c=c_demanded)
        acc_skyhook.append(out.ddzs)

    rms_passive = float(np.sqrt(np.mean(np.square(acc_passive))))
    rms_skyhook = float(np.sqrt(np.mean(np.square(acc_skyhook))))

    # Skyhook must reduce body acceleration RMS by at least 15%
    reduction = (rms_passive - rms_skyhook) / rms_passive
    assert reduction > 0.15, f"Skyhook failed to reduce RMS acceleration: reduction={reduction * 100:.1f}%"


def test_groundhook_stabilizes_tyre_contact():
    """Verify semi-active Groundhook controller targets tyre contact force stabilization."""
    cfg = QuarterCarConfig()
    model = QuarterCarModel(cfg)
    semi_cfg = SemiActiveConfig(c_ground=3000.0, c_min=700.0, c_max=4500.0)
    groundhook = GroundhookController(config=semi_cfg, continuous=True)

    dt = 0.002
    t_end = 2.0
    steps = int(t_end / dt)

    def get_road(t: float):
        # 10 Hz harmonic road wave (near wheel hop mode)
        freq = 10.0
        amp = 0.015
        z = amp * math.sin(2.0 * math.pi * freq * t)
        dz = amp * 2.0 * math.pi * freq * math.cos(2.0 * math.pi * freq * t)
        return z, dz

    tyre_loads_passive = []
    state_pass = QuarterCarState()
    for step in range(steps):
        t = step * dt
        rz, rdz = get_road(t)
        state_pass, out = model.step(state_pass, rz, rdz, dt, damper_c=cfg.suspension_damping)
        tyre_loads_passive.append(out.dynamic_tyre_load)

    tyre_loads_gh = []
    state_gh = QuarterCarState()
    for step in range(steps):
        t = step * dt
        rz, rdz = get_road(t)
        c_gh = groundhook.compute_damping(state_gh.dzs, state_gh.dzu, dt)
        state_gh, out = model.step(state_gh, rz, rdz, dt, damper_c=c_gh)
        tyre_loads_gh.append(out.dynamic_tyre_load)

    rms_dyn_pass = float(np.sqrt(np.mean(np.square(tyre_loads_passive[200:]))))
    rms_dyn_gh = float(np.sqrt(np.mean(np.square(tyre_loads_gh[200:]))))

    # Groundhook dampens wheel hop dynamic force variations
    assert rms_dyn_gh < rms_dyn_pass * 1.05


def test_fourteen_dof_static_load_equilibrium():
    """Verify 14-DOF full vehicle at constant speed conserves total normal load."""
    model = FourteenDOFModel()
    state = FourteenDOFState(vx=25.0)

    delta = np.zeros(4)
    T_drive = np.zeros(4)
    T_brake = np.zeros(4)

    _, telem = model.step(state, delta, T_drive, T_brake, dt=0.005)

    total_Fz = float(np.sum(telem.Fz))
    expected_weight = model.cfg.total_mass * 9.80665

    # Check conservation of total normal load
    assert abs(total_Fz - expected_weight) / expected_weight < 0.01

    # Check front/rear axle static distribution
    front_ratio = (telem.Fz[0] + telem.Fz[1]) / total_Fz
    expected_front_ratio = model.cfg.b / model.cfg.wheelbase
    assert abs(front_ratio - expected_front_ratio) < 0.02

    # Left-right symmetry
    assert abs(telem.Fz[0] - telem.Fz[1]) < 1e-4
    assert abs(telem.Fz[2] - telem.Fz[3]) < 1e-4


def test_fourteen_dof_braking_pitch_and_load_transfer():
    """Verify braking in 14-DOF produces nose-dive pitch and frontward load transfer."""
    model = FourteenDOFModel()
    state = FourteenDOFState(vx=25.0)

    delta = np.zeros(4)
    T_drive = np.zeros(4)
    T_brake = np.full(4, 800.0)  # Strong braking on all 4 wheels

    dt = 0.005
    for _ in range(40):  # Simulate 200 ms of braking
        state, telem = model.step(state, delta, T_drive, T_brake, dt=dt)

    # Deceleration should be negative
    assert telem.ax < -2.0

    # Front tyres should gain normal load, rear tyres should lose normal load
    front_load = telem.Fz[0] + telem.Fz[1]
    rear_load = telem.Fz[2] + telem.Fz[3]
    static_front = model.cfg.static_normal_loads[0] + model.cfg.static_normal_loads[1]
    static_rear = model.cfg.static_normal_loads[2] + model.cfg.static_normal_loads[3]

    assert front_load > static_front * 1.15, "Front wheels must gain load under braking"
    assert rear_load < static_rear * 0.85, "Rear wheels must lose load under braking"

    # Pitch angle should be negative (nose dive)
    assert state.theta < -0.005, f"Pitch should be nose-dive, got {state.theta:.4f} rad"


def test_fourteen_dof_cornering_roll_and_load_transfer():
    """Verify steering in 14-DOF produces lateral acceleration, body roll, and lateral load transfer."""
    model = FourteenDOFModel()
    state = FourteenDOFState(vx=20.0)

    delta = np.array([0.05, 0.05, 0.0, 0.0])  # Steer left
    T_drive = np.full(4, 50.0)  # Maintain forward speed
    T_brake = np.zeros(4)

    dt = 0.005
    for _ in range(50):  # Simulate 250 ms
        state, telem = model.step(state, delta, T_drive, T_brake, dt=dt)

    # Positive lateral acceleration (left turn)
    assert telem.ay > 1.0

    # Right side (outside wheels) should carry significantly more load than left side (inside wheels)
    left_load = telem.Fz[0] + telem.Fz[2]
    right_load = telem.Fz[1] + telem.Fz[3]

    assert right_load > left_load * 1.20, f"Outside wheels must carry more load: Right={right_load:.1f}, Left={left_load:.1f}"

    # Vehicle body should roll to the outside (right, phi > 0)
    assert state.phi > 0.005, f"Vehicle body should roll right, got {state.phi:.4f} rad"

