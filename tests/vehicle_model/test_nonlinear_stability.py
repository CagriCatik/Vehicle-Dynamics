"""M11 acceptance: equilibria with checkable residuals, local stability consistent with the linearised and the
nonlinear dynamics, reproducible recovery regions with separately reported uncertainty, exchangeable models."""

from __future__ import annotations

import math

import numpy as np
import pytest

from vehicle_model.handling_analysis import LinearSingleTrack, linear_params_from_seven_dof
from vehicle_model.handling_analysis.equilibrium import (RESIDUAL_TOL, EquilibriumAnalysis, OperatingPoint,
                                                         with_params)
from vehicle_model.roads.surface import MuSplitRoad, UniformRoad
from vehicle_model.simulation import stability_region as sr
from vehicle_model.tire_model import SlipStiffnessScaledTyre, load_tyre
from vehicle_model.vehicle_models import SevenDOFModel, TransientSevenDOFModel

V = 25.0


def _model(mu=1.0, **kw):
    return SevenDOFModel.from_yaml(road=UniformRoad(mu=mu), **kw)


def _rear_scaled(scale, mu=1.0):
    b = _model(mu)
    return SevenDOFModel(params=b.params, tyre=b.tyre_front, tyre_rear=SlipStiffnessScaledTyre(b.tyre_rear, scale),
                         road=UniformRoad(mu=mu))


# ----------------------------------------------------------------------------- equilibria (6.1)
@pytest.mark.parametrize("steer_deg", [0.0, 1.0, 3.0])
def test_equilibrium_residuals_and_steady_state_kinematics(steer_deg):
    an = EquilibriumAnalysis(_model())
    eq = an.solve(OperatingPoint(V, math.radians(steer_deg)))
    assert eq.accepted and eq.support == "four_wheel"
    assert all(eq.residual[g] <= RESIDUAL_TOL[g] for g in eq.residual)
    # vy_dot = 0 and vx_dot = 0 in the body frame: a_y = r vx and a_x = -r vy
    assert eq.ay == pytest.approx(eq.r * eq.state.vx, abs=1e-6)
    assert eq.ax == pytest.approx(-eq.r * eq.state.vy, abs=1e-6)
    assert eq.state.vx == V


def test_equilibria_are_mirror_symmetric():
    an = EquilibriumAnalysis(_model())
    a = an.solve(OperatingPoint(V, math.radians(2.0)))
    b = an.solve(OperatingPoint(V, math.radians(-2.0)))
    assert a.beta == pytest.approx(-b.beta, abs=1e-9) and a.r == pytest.approx(-b.r, abs=1e-9)


def test_operating_point_validation_and_uniform_road_requirement():
    with pytest.raises(ValueError, match="sum to 1"):
        OperatingPoint(V, drive_split=(0.5, 0.5, 0.5, 0.5))
    with pytest.raises(ValueError, match="uniform road"):
        EquilibriumAnalysis(SevenDOFModel.from_yaml(road=MuSplitRoad(mu_left=1.0, mu_right=0.3)))


def test_tv_yaw_moment_and_rear_steer_act_in_the_expected_direction():
    an = EquilibriumAnalysis(_model(0.6))
    base = an.solve(OperatingPoint(V, math.radians(1.0)))
    tv = an.solve(OperatingPoint(V, math.radians(1.0), tv_yaw_moment=300.0))
    inphase = an.solve(OperatingPoint(V, math.radians(1.0), rear_steer_ratio=0.2))
    assert base.accepted and tv.accepted and inphase.accepted
    assert tv.r > base.r                               # positive moment adds yaw rate
    assert inphase.r < base.r                          # in-phase rear steer: less yaw rate ...
    assert inphase.beta > base.beta                    # ... and sideslip closer to zero


def test_no_equilibrium_is_accepted_beyond_the_tv_fold():
    """A positive TV moment uses up the rear friction circle: beyond ~450 N m (1 deg, mu 0.6) no steady state exists.

    Expected physical limit: the solver stops at a finite residual and the point is not accepted,
    neither from the linear guess nor by continuation from the last point before the fold.
    """
    an = EquilibriumAnalysis(_model(0.6))
    before = an.solve(OperatingPoint(V, math.radians(1.0), tv_yaw_moment=400.0))
    assert before.accepted
    op = OperatingPoint(V, math.radians(1.0), tv_yaw_moment=1000.0)
    for eq in (an.solve(op), an.solve(op, an.unknowns(before))):
        assert not eq.accepted and max(eq.residual.values()) > 1e-2


# ----------------------------------------------------------------------------- local stability (6.2)
def test_jacobian_step_study_has_a_plateau():
    an = EquilibriumAnalysis(_model())
    lin = an.linearise(an.solve(OperatingPoint(V, math.radians(2.0))))
    changes = [s["eigenvalue_change"] for s in lin.step_study]
    assert lin.converged and min(changes) < 1e-6
    assert changes[0] > 1e-3                            # the largest step shows truncation error
    assert lin.step not in (an.jacobian_steps[0],)


@pytest.mark.parametrize("speed", [20.0, 40.0])
def test_lateral_eigenvalues_match_the_linear_single_track_straight_ahead(speed):
    m = _model()
    an = EquilibriumAnalysis(m)
    lin = an.linearise(an.solve(OperatingPoint(speed)))
    ref = LinearSingleTrack(linear_params_from_seven_dof(m.params, m.tyre_front, m.tyre_rear)).modal_properties(speed)
    lat = sorted((md.eigenvalue for md in lin.lateral_modes()), key=lambda z: (z.real, z.imag))
    exp = sorted(ref.eigenvalues, key=lambda z: (z.real, z.imag))
    assert len(lat) == 2
    for a, b in zip(lat, exp):                          # wheel dynamics and drag: a few percent apart
        assert abs(a - b) / abs(b) < 0.05


def test_stability_switch_matches_the_linear_critical_speed():
    m = _rear_scaled(0.6)                               # oversteering: finite critical speed
    v_crit = LinearSingleTrack(linear_params_from_seven_dof(m.params, m.tyre_front, m.tyre_rear)).critical_speed
    an = EquilibriumAnalysis(m)
    below = an.linearise(an.solve(OperatingPoint(0.9 * v_crit)))
    above = an.linearise(an.solve(OperatingPoint(1.1 * v_crit)))
    assert below.stability() == "stable" and above.stability() == "unstable"
    assert above.critical_mode.group == "lateral_yaw"


def test_linear_prediction_matches_the_nonlinear_model_for_stable_and_unstable_equilibria():
    an = EquilibriumAnalysis(_model(0.6))
    op = OperatingPoint(V, math.radians(3.0))
    stable = an.solve(op)
    saddle = an.solve(op, an._guess_from(op, math.radians(7.6), -0.9 * 0.6 * 9.81 / V))
    assert stable.accepted and saddle.accepted and saddle.beta > 0.1
    for eq, expected in ((stable, "stable"), (saddle, "unstable")):
        assert an.linearise(eq).stability() == expected
        chk = an.linear_check(eq)
        assert chk["same_tendency"]
        assert max(chk["relative_error_vy"], chk["relative_error_r"]) < 0.05


@pytest.mark.slow
def test_saddle_node_structure_beyond_the_limit():
    """mu 0.6, 90 km/h: one equilibrium at small steer; at 3 deg a stable one and an unstable pair on the spin side."""
    an = EquilibriumAnalysis(_model(0.6))
    small = an.find_all(OperatingPoint(V, math.radians(0.5)))
    large = an.find_all(OperatingPoint(V, math.radians(3.0)))
    assert len(small) == 1
    stab = sorted((an.linearise(e).stability(), math.copysign(1, e.r)) for e in large)
    assert stab == [("stable", 1.0), ("unstable", -1.0), ("unstable", -1.0)]


# ----------------------------------------------------------------------------- exchangeable models
def test_transient_tyre_model_has_the_same_equilibrium_and_extra_states():
    op = OperatingPoint(V, math.radians(2.0))
    a = EquilibriumAnalysis(_model()).solve(op)
    an_t = EquilibriumAnalysis(TransientSevenDOFModel.from_yaml(road=UniformRoad(mu=1.0)))
    b = an_t.solve(op)
    assert b.accepted and "relaxation" in b.residual and an_t.body.size == 15
    assert b.beta == pytest.approx(a.beta, abs=1e-8) and b.r == pytest.approx(a.r, abs=1e-8)
    assert an_t.linearise(b).stability() == "stable"


@pytest.mark.parametrize("variant", ["fiala", "kc", "cog"])
def test_other_tyres_chassis_and_cog_through_the_existing_interfaces(variant):
    if variant == "fiala":
        m = _model(tyre_name="fiala_passenger_synthetic", tyre_rear_name="fiala_passenger_synthetic")
    elif variant == "kc":
        m = SevenDOFModel.from_yaml("configs/vehicles/reference_passenger_car.yaml", road=UniformRoad(mu=1.0),
                                    kc_config="configs/suspension/kc_reference_passenger_car.yaml")
    else:
        b = _model()
        m = with_params(b, lf=b.params.lf - 0.2, lr=b.params.lr + 0.2)
    an = EquilibriumAnalysis(m)
    eq = an.solve(OperatingPoint(V, math.radians(1.5)))
    assert eq.accepted and an.linearise(eq).converged


def test_with_params_refuses_a_kc_chassis():
    m = SevenDOFModel.from_yaml("configs/vehicles/reference_passenger_car.yaml", road=UniformRoad(mu=1.0),
                                kc_config="configs/suspension/kc_reference_passenger_car.yaml")
    with pytest.raises(ValueError, match="K&C"):
        with_params(m, lf=1.0)


def test_slip_stiffness_scaling_keeps_the_friction_peak():
    base = load_tyre("magic_formula_passenger_synthetic")
    soft = SlipStiffnessScaledTyre(base, cornering=0.5, longitudinal=2.0)
    Fz, mu, h = np.full(2, 4000.0), np.ones(2), 1e-5
    c = lambda t: (t.forces(np.zeros(2), np.array([-h, h]), Fz, mu)[1][1] - t.forces(np.zeros(2), np.array([-h, h]), Fz, mu)[1][0]) / (2 * h)
    assert c(soft) == pytest.approx(0.5 * c(base), rel=1e-4)
    alphas = np.radians(np.linspace(0.5, 30, 300))
    peak = lambda t: np.max(np.abs(t.forces(np.zeros(300), alphas, np.full(300, 4000.0), np.ones(300))[1]))
    assert peak(soft) == pytest.approx(peak(base), rel=2e-3)
    assert np.all(soft.slip_stiffness(Fz, mu) == pytest.approx(2.0 * base.slip_stiffness(Fz, mu)))
    with pytest.raises(ValueError):
        SlipStiffnessScaledTyre(base, cornering=0.0)


# ----------------------------------------------------------------------------- recovery regions (6.3, 6.4)
CRIT = sr.RecoveryCriteria()


def _hist(beta, r, dt=0.01):
    beta, r = np.asarray(beta, float), np.asarray(r, float)
    return np.arange(1, beta.size + 1) * dt, beta, r


@pytest.mark.parametrize("case, expected", [
    ("returned", "returned"), ("loss", "loss_of_control"), ("offset", "large_deviation"),
    ("drifting", "inconclusive"), ("failure", "numerical_failure")])
def test_classification_covers_every_outcome(case, expected):
    n = 200
    if case == "returned":
        t, b, r = _hist(np.r_[np.linspace(0.2, 0.0, n // 2), np.zeros(n // 2)], np.zeros(n))
    elif case == "loss":
        t, b, r = _hist(np.linspace(0.0, 0.6, n), np.zeros(n))
    elif case == "offset":
        t, b, r = _hist(np.full(n, math.radians(5.0)), np.zeros(n))
    elif case == "drifting":
        t, b, r = _hist(np.linspace(0.1, 0.3, n), np.zeros(n))
    else:
        t, b, r = _hist(np.zeros(n), np.zeros(n))
    assert sr.classify(t, b, r, 0.0, 0.0, CRIT, numerical_failure=case == "failure") == expected


PROBLEM = sr.RecoveryProblem("passive", speed=80 / 3.6, steer=math.radians(1.0), mu=0.6, duration=3.0)


def test_recovery_runs_from_near_and_far_initial_states():
    ref = sr.reference_state(PROBLEM)
    assert ref["settled"]
    assert ref["beta"] == pytest.approx(ref["equilibrium_beta"], abs=1e-3)   # passive: closed loop = open loop
    near = sr.simulate(PROBLEM, ref["beta"] + math.radians(3.0), ref["r"], (ref["beta"], ref["r"]))
    far = sr.simulate(PROBLEM, math.radians(-20.0), math.radians(40.0), (ref["beta"], ref["r"]))
    assert near.outcome == "returned" and near.t_end < PROBLEM.duration
    assert far.outcome == "loss_of_control" and far.max_abs_beta >= CRIT.loss_beta


def test_a_failing_controller_is_an_outcome_not_a_crash(monkeypatch):
    class Broken:
        def compute_control(self, *a, **k):
            raise RuntimeError("solver exploded")

    monkeypatch.setattr(sr, "_controllers", lambda: {"passive": lambda p: Broken()})
    sr._setup.cache_clear()
    prob = sr.RecoveryProblem("passive", speed=20.0, duration=0.2)
    try:
        run = sr.simulate(prob, 0.05, 0.0, (0.0, 0.0))
    finally:
        sr._setup.cache_clear()
    assert run.outcome == "numerical_failure" and "solver exploded" in run.error


def test_unknown_controller_is_rejected():
    with pytest.raises(ValueError, match="unknown controller"):
        sr.RecoveryProblem("magic", speed=20.0)


@pytest.mark.slow
def test_map_is_reproducible_serial_and_parallel_with_separate_uncertainty():
    prob = sr.RecoveryProblem("passive", speed=80 / 3.6, steer=math.radians(1.0), mu=0.6, duration=3.0)
    bg, rg = np.radians([-20.0, 0.0, 20.0]), np.radians([-40.0, 0.0, 40.0])
    a = sr.compute_map(prob, bg, rg, workers=1)
    b = sr.compute_map(prob, bg, rg, workers=2)
    assert a.outcomes.tolist() == b.outcomes.tolist() and a.certainty.tolist() == b.certainty.tolist()
    assert a.refinement["outcomes"] == b.refinement["outcomes"]
    assert a.outcomes[1, 1] == "returned" and a.outcomes[0, 2] == "loss_of_control"
    s = a.summary()
    lo, hi = s["returned_share_bounds"]
    assert lo <= s["returned_share"] <= hi
    assert set(s["sensitivity"]) == {"step", "duration"} and s["sensitivity"]["step"]["rerun"] >= s["boundary_points"] - 1
    assert a.refinement["shape"] == [5, 5] and 0.0 <= a.refinement["returned_share"] <= 1.0


def test_heuristic_boundary_is_small_at_the_equilibrium_and_large_far_away():
    S = sr.heuristic_boundary(PROBLEM, np.radians([-0.4, 15.0]), np.radians([7.6, -40.0]))
    assert S[0, 0] < 0.5 and S[1, 1] > 1.0


@pytest.mark.parametrize("controller", ["robust_lqr", "gs_lqr_dob", "tightening_mpc", "adaptive_lqr"])
def test_m12_laws_are_available_in_recovery_maps(controller):
    """Plan 6.4: the robust controllers of M12 are mapped like the others (designed from the nominal vehicle)."""
    prob = sr.RecoveryProblem(controller, speed=80 / 3.6, steer=math.radians(1.0), mu=0.6, duration=1.5)
    ref = sr.reference_state(prob)
    run = sr.simulate(prob, ref["beta"] + math.radians(2.0), ref["r"], (ref["beta"], ref["r"]))
    assert run.outcome in sr.OUTCOMES and run.outcome != "numerical_failure", run.error
    assert controller in sr.CONTROLLER_DESCRIPTIONS
