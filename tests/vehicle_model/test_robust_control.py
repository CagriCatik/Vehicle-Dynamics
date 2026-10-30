"""Uncertainty model, robust and adaptive laws checked against their derivations, constraint-aware NMPC,
supervisory layer."""

from __future__ import annotations

import dataclasses
import math

import numpy as np
import pytest

from vehicle_model.controllers.framework import ControlOutput, ControlTarget, PassiveBaselineController
from vehicle_model.controllers.motion_control import MotionContext, MotionController, MotionRequest
from vehicle_model.controllers.robust_laws import (AdaptationSettings, AdaptiveLQRYawLaw, DisturbanceObserverLaw, RobustLQRYawLaw,
                                                   TighteningMPCYawLaw, closed_loop_abscissa, robust_stability)
from vehicle_model.controllers.supervisor import SupervisorConfig, SupervisoryController
from vehicle_model.controllers.yaw_laws import INPUTS_4WS_TV, INPUTS_TV, YawDesignModel, YawRatePI
from vehicle_model.core.types import SevenDOFParams, SevenDOFState
from vehicle_model.handling_analysis import LinearSingleTrack, linear_params_from_seven_dof
from vehicle_model.simulation.uncertainty import (DelayedActuation, UncertaintyCase, plant_factory, sensor_config,
                                                  wrap_controller)
from vehicle_model.vehicle_models import SevenDOFModel

NOMINAL = SevenDOFModel.from_yaml()
P = NOMINAL.params
DESIGN = linear_params_from_seven_dof(P, NOMINAL.tyre_front, NOMINAL.tyre_rear)


def _ctx(steer=0.0, r_ref=0.0, mu=1.0, dt=0.005):
    return MotionContext(steer, r_ref, mu, dt, 4000.0, P.max_steer_r)


# ----------------------------------------------------------------------------- uncertainty model (7.1)
def test_plant_deviates_but_the_controller_keeps_the_nominal_design():
    from vehicle_model.roads.surface import UniformRoad
    case = UncertaintyCase("x", mass_factor=1.2, Iz_factor=1.25, cog_shift_m=-0.1, cornering_scale_rear=0.7, mu=0.5)
    plant = plant_factory(NOMINAL, case)(UniformRoad(mu=1.0))
    assert plant.params.mass == pytest.approx(1.2 * P.mass) and plant.params.Iz == pytest.approx(1.25 * P.Iz)
    assert plant.params.lf == pytest.approx(P.lf + 0.1) and plant.params.wheelbase == pytest.approx(P.wheelbase)
    assert plant.road.mu == 0.5 and plant.tyre_rear.cornering == 0.7
    assert case.plant_deviates and not UncertaintyCase().plant_deviates
    assert NOMINAL.params.mass == P.mass                      # nominal model untouched


def test_uncertainty_case_validation_and_sensor_corruption():
    with pytest.raises(ValueError, match="unknown"):
        UncertaintyCase.from_dict({"name": "a", "mass": 2})
    with pytest.raises(ValueError):
        UncertaintyCase(mass_factor=0.0)
    cfg = sensor_config(UncertaintyCase(sensor_noise_scale=2.0, gyro_bias_radps=0.02, sensor_latency_s=0.02))
    base = sensor_config(UncertaintyCase())
    assert cfg.gyro_noise_std == pytest.approx(2 * base.gyro_noise_std) and cfg.gyro_bias_init == 0.02
    assert cfg.latency == pytest.approx(base.latency + 0.02)


class _Ramp:
    """Commands steer = step index * 0.01, so delays are countable."""
    name, information = "ramp", "test"

    def __init__(self):
        self.k = 0

    def reset(self):
        self.k = 0

    def compute_control(self, state, target, telemetry=None, dt=0.01):
        self.k += 1
        return ControlOutput(steer_f=0.01 * self.k, T_brake=np.full(4, 10.0 * self.k))


def test_actuator_delay_is_whole_steps_and_lag_is_first_order():
    d = DelayedActuation(_Ramp(), delay_s=0.03)
    outs = [d.compute_control(SevenDOFState(), ControlTarget(), dt=0.01).steer_f for _ in range(6)]
    assert outs == pytest.approx([0.01, 0.01, 0.01, 0.01, 0.02, 0.03])  # first command held until 3 steps passed
    lag = DelayedActuation(_Ramp(), tau_s=0.01)
    o1 = lag.compute_control(SevenDOFState(), ControlTarget(), dt=0.01)
    o2 = lag.compute_control(SevenDOFState(), ControlTarget(), dt=0.01)
    assert o2.steer_f == pytest.approx(0.01 + 0.5 * (0.02 - 0.01))     # dt/(tau+dt) = 0.5
    assert wrap_controller(_Ramp(), UncertaintyCase()).__class__ is _Ramp


# ----------------------------------------------------------------------------- A: prescribed degree of stability
@pytest.mark.parametrize("inputs", [INPUTS_TV, INPUTS_4WS_TV])
def test_robust_lqr_places_all_nominal_eigenvalues_left_of_minus_alpha(inputs):
    law = RobustLQRYawLaw(DESIGN, inputs, alpha=3.0)
    for v in law.speeds:
        assert closed_loop_abscissa(DESIGN, inputs, law.gain(v), v) <= -3.0 + 1e-6


def test_robust_stability_analysis_reports_the_worst_vertex():
    law = RobustLQRYawLaw(DESIGN, INPUTS_4WS_TV, alpha=3.0)
    small = robust_stability(law, DESIGN, {"C_r": (0.9, 1.0)})
    large = robust_stability(law, DESIGN, {"C_r": (0.5, 1.0), "mass": (1.0, 1.3)})
    assert small["stable"] and small["models"] == 3
    assert large["worst"]["abscissa"] >= small["worst"]["abscissa"]
    # for the 4WS+TV LQR the heavier vehicle is the worst case, not the softer rear axle (rear steer compensates it)
    assert large["worst"]["factors"]["mass"] == 1.3 and large["stable"]
    assert all(w["abscissa"] >= w["nominal"] - 1e-9 for w in large["per_speed"])


# ----------------------------------------------------------------------------- B: disturbance observer
class _Zero:
    name, information = "zero", "test"

    def reset(self):
        pass

    def request(self, state, ctx):
        return MotionRequest()


def test_disturbance_observer_estimates_a_constant_yaw_disturbance_at_its_bandwidth():
    """Linear plant (inside the tyre limits) with an extra constant yaw acceleration d: d_hat -> d."""
    w_o, d_true, dt, vx = 12.0, 0.4, 0.002, 20.0
    dob = DisturbanceObserverLaw(_Zero(), DESIGN, bandwidth=w_o, max_share=1e-9)   # observe only, no compensation
    A, B = YawDesignModel(DESIGN, INPUTS_4WS_TV).matrices(vx)
    x = np.zeros(2)
    steer = 0.01
    hist = []
    for k in range(int(1.0 / dt)):
        dob.request(SevenDOFState(vx=vx, vy=x[0], r=x[1]), _ctx(steer=steer, dt=dt))
        x = x + dt * (A @ x + B[:, 0] * steer + np.array([0.0, d_true]))
        hist.append(dob.d_hat)
    t63 = (np.argmax(np.abs(np.array(hist)) >= 0.63 * d_true) + 1) * dt
    assert hist[-1] == pytest.approx(d_true, rel=0.03)
    assert 0.5 / w_o < t63 < 4.0 / w_o                           # double pole at -w_o: of the order of 2/w_o


# ----------------------------------------------------------------------------- C: tightening MPC
def test_tightening_margins_grow_from_zero_and_converge():
    law = TighteningMPCYawLaw(DESIGN, INPUTS_4WS_TV)
    h = law.tightening(20.0)
    for k in ("beta", "r"):
        assert h[k][0] == 0.0 and np.all(np.diff(h[k]) >= 0.0)
        assert h[k][-1] - h[k][-2] < 0.5 * (h[k][1] - h[k][0])   # A_K stable: increments shrink


def test_tightening_mpc_feasible_and_infeasible_starts_are_reported_separately():
    law = TighteningMPCYawLaw(DESIGN, INPUTS_4WS_TV)
    ok = law.request(SevenDOFState(vx=20.0, vy=0.0, r=0.05), _ctx(steer=0.01, r_ref=0.05))
    assert ok.diagnostics["tightened_problem_feasible"] and ok.diagnostics["feasibility"]["feasible"]
    law.reset()
    beta_max = math.atan(0.02 * 9.81)            # stability_limits at mu = 1
    bad = law.request(SevenDOFState(vx=20.0, vy=20.0 * math.tan(2.0 * beta_max), r=0.05), _ctx(steer=0.01, r_ref=0.05))
    assert not bad.diagnostics["tightened_problem_feasible"]
    assert bad.diagnostics["slack"]["state"] > 0.0               # relaxed problem solved with slack ...
    assert not bad.diagnostics["feasibility"]["feasible"]        # ... and the plan's violation is reported


# ----------------------------------------------------------------------------- D: adaptive LQR
def test_rls_recovers_the_cornering_stiffness_factor_of_a_linear_plant():
    theta_true, dt, vx = 0.7, 0.005, 20.0
    plant = dataclasses.replace(DESIGN, C_f=DESIGN.C_f * theta_true, C_r=DESIGN.C_r * theta_true)
    lin = LinearSingleTrack(plant).state_space(vx)
    # the linear test plant has no combined slip: the yaw-moment gate (for brake-based moments) is not needed here
    law = AdaptiveLQRYawLaw(DESIGN, INPUTS_TV, settings=AdaptationSettings(max_moment_share=1.0))
    x = np.zeros(2)
    for k in range(int(8.0 / dt)):                              # excitation inside the linear range (updates gated)
        steer = 0.015 * math.sin(2 * math.pi * 0.5 * k * dt)
        r_ref = LinearSingleTrack(plant).yaw_rate_gain(vx) * steer   # a reference the plant can follow
        req = law.request(SevenDOFState(vx=vx, vy=x[0], r=x[1]), _ctx(steer=steer, r_ref=r_ref, dt=dt))
        x = x + dt * (lin.A @ x + lin.B[:, 0] * steer + np.array([0.0, req.yaw_moment / DESIGN.Iz]))
    assert law.updates > 100
    assert law.theta_rls == pytest.approx(theta_true, abs=0.05)
    assert law.theta == pytest.approx(theta_true, abs=0.08)       # low-pass follows


def test_adaptive_law_starts_at_the_nominal_design():
    law = AdaptiveLQRYawLaw(DESIGN, INPUTS_TV)
    assert law.theta == 1.0
    with pytest.raises(ValueError):
        AdaptiveLQRYawLaw(DESIGN, INPUTS_TV, settings=dataclasses.replace(law.settings, theta_grid=(0.3, 0.5)))


# ----------------------------------------------------------------------------- E: constraint-aware NMPC
def test_nmpc_projection_keeps_the_applied_input_admissible():
    from vehicle_model.controllers.dynamic_two_track_nmpc import DynamicNMPCConfig, DynamicTwoTrackNMPC
    n = DynamicTwoTrackNMPC(P, DynamicNMPCConfig(project_inputs=True))
    u, info = n._project(np.array([0.5, 0.0, 9000.0, np.nan]), np.zeros(4), 0.05)
    step = n.config.max_steer_rate_radps * 0.05
    assert u[0] == pytest.approx(step) and u[2] == n.config.max_yaw_moment_nm and u[3] == 0.0
    assert info["projected"] and info["distance"] == math.inf


def test_nmpc_soft_constraints_report_slack():
    from vehicle_model.controllers.dynamic_two_track_nmpc import DynamicNMPCConfig, DynamicTwoTrackNMPC
    n = DynamicTwoTrackNMPC(P, DynamicNMPCConfig(N=4, max_iter=5, soft_stability_constraints=True, project_inputs=True))
    out = n.compute_control(SevenDOFState(vx=20.0, vy=0.5, r=0.6), ControlTarget(target_vx=20.0, target_y=1.0,
                                                                                    mu_estimated=1.0), dt=0.05)
    sc = out.diagnostics["soft_constraints"]
    assert sc is not None and sc["max_slack_r"] >= 0.0 and out.diagnostics["input_projection"] is not None
    assert np.isfinite(out.steer_f)


# ----------------------------------------------------------------------------- supervisor (7.3)
class _Scripted:
    """Outputs per step from a script: 'ok', 'nan', 'big', 'fallback', 'degraded'."""
    information = "test"

    def __init__(self, script, name="scripted"):
        self.script, self.name, self.k = list(script), name, 0

    def reset(self):
        self.k = 0

    def compute_control(self, state, target, telemetry=None, dt=0.01):
        kind = self.script[min(self.k, len(self.script) - 1)]
        self.k += 1
        out = ControlOutput(steer_f=0.01, status="nominal")
        if kind == "nan":
            out.steer_f = float("nan")
        elif kind == "big":
            out.steer_f = 5.0
        elif kind in ("fallback", "degraded"):
            out.status = kind
        return out


def _run(sup, n, dt=0.01):
    return [sup.compute_control(SevenDOFState(vx=20.0), ControlTarget(), dt=dt) for _ in range(n)]


def test_supervisor_switches_to_fallback_and_returns_through_recovery():
    cfg = SupervisorConfig(n_unhealthy=3, recover_s=0.05, blend_s=0.03)
    sup = SupervisoryController(_Scripted(["ok"] * 2 + ["fallback"] * 4 + ["ok"] * 30), _Scripted(["ok"], "fb"), P, cfg)
    modes = [o.diagnostics["supervisor_mode"] for o in _run(sup, 20)]
    assert modes[:2] == ["nominal", "nominal"]
    assert modes[4] == "fallback"                                 # third unhealthy step
    assert "recovery" in modes and modes[-1] == "nominal"
    assert [t["to"] for t in sup.transitions] == ["fallback", "recovery", "nominal"]


def test_supervisor_never_passes_a_non_finite_command_and_clips_out_of_limit_ones():
    sup = SupervisoryController(_Scripted(["nan", "big", "ok"]), _Scripted(["ok"]), P)
    outs = _run(sup, 3)
    assert all(np.isfinite(o.steer_f) for o in outs)
    assert outs[0].diagnostics["suppressed"] and outs[1].diagnostics["suppressed"]
    assert outs[1].steer_f == pytest.approx(P.max_steer_f) and not outs[2].diagnostics["suppressed"]


def test_supervisor_degrades_when_the_fallback_fails_too():
    sup = SupervisoryController(_Scripted(["degraded"]), _Scripted(["nan"]), P, SupervisorConfig(degraded_hold_s=0.05))
    outs = _run(sup, 5)
    assert outs[0].diagnostics["supervisor_mode"] == "degraded"
    assert all(np.isfinite(o.steer_f) for o in outs)


def test_supervisor_reports_limited_and_suboptimal_primary_operation():
    class _Status(_Scripted):
        def compute_control(self, *a, **k):
            out = super().compute_control(*a, **k)
            out.status = self.script[min(self.k - 1, len(self.script) - 1)]
            return out
    sup = SupervisoryController(_Status(["saturated", "suboptimal", "nominal"]), _Scripted(["ok"]), P)
    assert [o.diagnostics["supervisor_mode"] for o in _run(sup, 3)] == ["limited", "suboptimal", "nominal"]


# ----------------------------------------------------------------------------- closed loop smoke
def test_every_robust_law_runs_closed_loop_on_the_nonlinear_plant():
    from vehicle_model.simulation.controller_benchmark import run_scenario, step_steer_scenario
    sc = step_steer_scenario(20.0, 0.03, 0.6)
    laws = [RobustLQRYawLaw(DESIGN, INPUTS_4WS_TV), DisturbanceObserverLaw(YawRatePI(), DESIGN),
            TighteningMPCYawLaw(DESIGN, INPUTS_4WS_TV), AdaptiveLQRYawLaw(DESIGN, INPUTS_4WS_TV)]
    for law in laws:
        r = run_scenario(lambda road: SevenDOFModel.from_yaml(road=road), lambda p, law=law: MotionController(P, law), sc)
        assert np.all(np.isfinite(r.history["r"])) and abs(r.history["r"][-1] - r.history["r_ref"][-1]) < 0.02, law.name


def test_adaptation_does_not_read_tyre_saturation_as_soft_tyres():
    """Regression for the first design: in sine with dwell theta fell to its bound and the vehicle spun.

    Updates are gated to the linear tyre range (measured and predicted a_y) and to small yaw moments;
    on the nominal and on the worn-rear vehicle theta stays near the linear stiffness and control holds.
    """
    from vehicle_model.simulation.controller_benchmark import run_scenario, sine_with_dwell_scenario
    from vehicle_model.simulation.uncertainty import plant_factory
    for case in (UncertaintyCase(), UncertaintyCase(cornering_scale_rear=0.7)):
        laws = []

        def fac(_p):
            laws.append(AdaptiveLQRYawLaw(DESIGN, INPUTS_4WS_TV))
            return MotionController(P, laws[-1])
        r = run_scenario(plant_factory(NOMINAL, case), fac, sine_with_dwell_scenario(80 / 3.6, math.radians(6.0)))
        assert laws[0].theta > 0.7, case
        assert r.kpis["max_sideslip_deg"] < 5.0, case
