"""Fault catalogue, deterministic injection, health monitor and degraded modes (7-DOF)."""

from __future__ import annotations

import math
import random

import numpy as np
import pytest

from vehicle_model.controllers.fault_tolerance import FaultTolerantController, ZeroYawLaw
from vehicle_model.controllers.framework import ControlTarget
from vehicle_model.controllers.motion_control import MotionController
from vehicle_model.controllers.yaw_laws import YawRatePI
from vehicle_model.core.types import SevenDOFState
from vehicle_model.simulation.fault_injection import CATALOGUE, FaultEvent, FaultInjectedController, parse_faults
from vehicle_model.vehicle_models import SevenDOFModel

VEH = "configs/vehicles/seven_dof_passenger.yaml"
DT = 0.005


@pytest.fixture(scope="module")
def p():
    return SevenDOFModel.from_yaml(VEH).params


def _stack(p, faults, aware=True, delay=0.05):
    mc = MotionController(p, YawRatePI())
    inner = FaultTolerantController(mc, p) if aware else mc
    return FaultInjectedController(inner, faults, delay, p.Reff), mc


def _tick(ctrl, p, n, pedal=0.6, steer=0.0, vx=20.0):
    s = SevenDOFState(vx=vx, r=0.05, omega=np.full(4, vx / p.Reff))
    return [ctrl.compute_control(s, ControlTarget(driver_brake_pedal=pedal, driver_steer_f=steer), None, DT) for _ in range(n)]


def test_fault_events_are_strict_and_fuzzing_raises_only_value_errors():
    assert parse_faults([{"t_start": 1, "channel": "brake", "mode": "failed", "wheel": "RR"}])[0].label == "brake RR failed"
    for bad, msg in (({"t_start": 0, "channel": "brake", "mode": "failed"}, "wheel required"),
                     ({"t_start": 0, "channel": "imu", "mode": "lost", "wheel": "FL"}, "wheel not allowed"),
                     ({"t_start": 0, "channel": "damper", "mode": "stuck"}, "14-DOF"),
                     ({"t_start": 0, "channel": "brake", "mode": "melted", "wheel": "FL"}, "mode"),
                     ({"t_start": -1, "channel": "imu", "mode": "lost"}, "t_start"),
                     ({"t_start": 0, "channel": "delay", "mode": "ticks", "severity": 2.5}, "integer"),
                     ({"t_start": 0, "channel": "brake", "mode": "limited", "wheel": "FL", "severity": 1.5}, "fraction"),
                     ({"t_start": 0, "channel": "imu", "mode": "lost", "colour": 1}, "unknown key")):
        with pytest.raises(ValueError, match=msg):
            parse_faults([bad])
    rng = random.Random(7)
    atoms = [None, True, -1, 0, 1.5, float("nan"), float("inf"), "x", "brake", "FL", [], {}, [1], {"a": 1}]
    keys = ["t_start", "duration", "channel", "mode", "severity", "wheel", "junk"]
    for _ in range(400):
        item = {k: rng.choice(atoms + list(CATALOGUE) + ["failed", "lost", "ticks"]) for k in rng.sample(keys, rng.randint(0, 6))}
        try:
            parse_faults(rng.choice([[item], item, [item, item], "nonsense"]))
        except ValueError:
            pass


def test_injector_timing_reports_delay_and_stale(p):
    ctrl, _ = _stack(p, [FaultEvent(0.02, "brake", "failed", wheel="RR"), FaultEvent(0.05, "delay", "ticks", severity=4)],
                     delay=0.05)
    outs = _tick(ctrl, p, 30)
    t = np.arange(30) * DT
    rep = [bool(e["reported"]) for e in ctrl.log]
    assert rep == list(t >= 0.07 - 1e-12)                                     # self-report after the detection delay
    assert all(o.T_brake[3] == 0.0 for o, tt in zip(outs, t) if tt >= 0.02)   # the plant never gets the failed brake
    assert ctrl.inner.t - ctrl.inner.t_stamp == pytest.approx(4 * DT)           # the delivered message is 4 ticks old
    stale, _ = _stack(p, [FaultEvent(0.02, "stale", "frozen", duration=0.05)])
    _tick(stale, p, 12)
    assert stale.inner.t - stale.inner.t_stamp == pytest.approx(stale.inner.t - 0.02)   # frozen at onset, ageing
    _tick(stale, p, 13)
    assert stale.inner.t - stale.inner.t_stamp == pytest.approx(0.0)           # fresh again after the stale period


def test_no_command_leak_after_the_self_report(p):
    ctrl, mc = _stack(p, [FaultEvent(0.0, "brake", "failed", wheel="FL"), FaultEvent(0.0, "brake", "failed", wheel="RR")])
    _tick(ctrl, p, 40)
    leaks_after = [e["leaks"] for e in ctrl.log if e["reported"]]
    assert leaks_after and not any(any(v for v in d.values()) for d in leaks_after)
    leaks_before = sum(any(e["leaks"].values()) for e in ctrl.log if not e["reported"]) * DT
    assert leaks_before == pytest.approx(0.05, abs=DT + 1e-9)                   # bounded by the detection delay
    assert mc.actuation.health.brake_ok == (False, True, True, False)
    assert ctrl.inner.system_state == "degraded"
    all4, _ = _stack(p, [FaultEvent(0.0, "brake", "failed", wheel=w) for w in ("FL", "FR", "RL", "RR")])
    outs = _tick(all4, p, 20)
    assert all4.inner.system_state == "unavailable" and outs[-1].status == "infeasible"


def test_imu_loss_falls_back_and_recovers(p):
    ctrl, mc = _stack(p, [FaultEvent(0.01, "imu", "lost", duration=0.05)])
    outs = _tick(ctrl, p, 20, steer=0.03)
    assert isinstance(mc.law, ZeroYawLaw) and ctrl.inner.channels.get("imu") == "unavailable"
    assert all(np.all(np.isfinite(o.T_brake)) for o in outs)
    _tick(ctrl, p, int(0.25 / DT), steer=0.03)                                 # recovery_time after the fault ends
    assert isinstance(mc.law, YawRatePI) and ctrl.inner.system_state == "nominal"
    unaware, _ = _stack(p, [FaultEvent(0.0, "imu", "lost")], aware=False)
    with pytest.raises(ValueError, match="non-finite"):
        _tick(unaware, p, 2, steer=0.03)                                       # without the monitor the NaN reaches the allocation


def test_front_steer_stuck_requests_a_controlled_stop(p):
    ctrl, _ = _stack(p, [FaultEvent(0.0, "steer_front", "stuck")])
    outs = _tick(ctrl, p, 20, pedal=0.0, steer=0.02)
    assert ctrl.inner.system_state == "fallback" and float(np.sum(outs[-1].T_brake)) > 0.0
    assert outs[-1].steer_f == outs[0].steer_f                                 # held at onset by the plant fault


def test_aware_without_faults_is_identical_to_unaware(p):
    a, _ = _stack(p, [])
    b, _ = _stack(p, [], aware=False)
    for oa, ob in zip(_tick(a, p, 30, steer=0.03), _tick(b, p, 30, steer=0.03)):
        assert np.array_equal(oa.T_brake, ob.T_brake) and np.array_equal(oa.T_drive, ob.T_drive) and oa.steer_f == ob.steer_f


@pytest.mark.slow
def test_closed_loop_fault_catalogue(tmp_path):
    from vehicle_model.experiments.fault_tolerance import parse, run
    spec = parse({"type": "fault_tolerance_benchmark", "name": "t", "vehicle": VEH, "workers": 1,
                  "manoeuvres": [{"name": "smu", "type": "split_mu_braking", "speed_kph": 90}],
                  "fault_sets": [{"name": "rr", "faults": [{"t_start": 0.5, "channel": "brake", "mode": "failed", "wheel": "RR"}]},
                                 {"name": "drift", "faults": [{"t_start": 0.5, "channel": "mu_estimate", "mode": "drift", "severity": 0.6}]},
                                 {"name": "seq", "faults": [{"t_start": 0.5, "duration": 1.0, "channel": "brake", "mode": "failed", "wheel": "RR"},
                                                            {"t_start": 2.0, "duration": 0.5, "channel": "imu", "mode": "lost"}]}],
                  "controllers": [{"name": "unaware", "law": "pi", "aware": False}, {"name": "aware", "law": "pi", "aware": True}],
                  "outputs": {"directory": str(tmp_path), "formats": ["csv", "json", "md"]}})
    res = {(r["fault_set"], r["controller"]): r for r in run(spec, tmp_path)["results"]}
    rr_a, rr_u = res[("rr", "aware")]["kpis"], res[("rr", "unaware")]["kpis"]
    assert rr_a["leak_after_report_s"] == 0.0 and rr_a["leak_before_report_s"] == pytest.approx(0.05, abs=0.006)
    assert rr_u["leak_after_report_s"] > 1.0
    assert abs(rr_a["final_heading_deg"]) < abs(rr_u["final_heading_deg"]) and rr_a["worst_state"] == "degraded"
    assert res[("drift", "aware")]["kpis"]["undetected"] == "mu_estimate drift"          # reported, not hidden
    assert "non-finite" in res[("seq", "unaware")]["error"] and not res[("seq", "aware")]["error"]
