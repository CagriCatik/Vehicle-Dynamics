"""Drift plan D4: supervisor mode logic (documented criteria, timeouts, no HOLD outside tolerance, FAILED final)."""

from __future__ import annotations

import math

import pytest

from vehicle_model.drift.supervisor import (APPROACH, DISABLED, EXIT, FAILED, HOLD, INITIATE, RECOVER, DriftSupervisor,
                                            SupervisorSettings)

B, R, V, RC = math.radians(-20.0), math.radians(29.0), 13.0, math.radians(23.0)


def _sup(**kw):
    return DriftSupervisor(SupervisorSettings(**kw), B, R, V, RC)


def _feed(sup, t0, t1, beta, r, v, dt=0.01, straight=False):
    t = t0
    while t < t1 - 1e-9:
        t += dt
        sup.update(t, dt, beta, r, v, straight)
    return t


def test_nominal_sequence_with_reasons():
    s = _sup(request_s=0.5, exit_s=10.0)
    t = _feed(s, 0.0, 0.6, 0.0, 0.0, V)
    assert s.mode == APPROACH
    t = _feed(s, t, 2.0, math.radians(1.0), RC, V)            # steady cornering
    assert s.mode == INITIATE
    t = _feed(s, t, 4.0, B + math.radians(2.0), R, V)         # inside the capture region
    assert s.mode == HOLD
    t = _feed(s, t, 10.5, B, R, V)
    assert s.mode == EXIT
    _feed(s, t, 11.0, 0.01, RC, V, straight=True)
    assert s.mode == DISABLED
    assert [tr.target for tr in s.transitions] == [APPROACH, INITIATE, HOLD, EXIT, DISABLED]
    assert all(tr.reason for tr in s.transitions)


def test_hold_is_not_reported_outside_the_capture_region():
    s = _sup(request_s=0.0)
    t = _feed(s, 0.0, 2.0, math.radians(1.0), RC, V)
    _feed(s, t, 20.0, B + math.radians(8.0), R, V)            # 8 deg off: outside capture (5 deg)
    assert s.mode != HOLD and HOLD not in [tr.target for tr in s.transitions]


def test_loss_in_hold_and_timeouts_lead_to_recover_then_failed():
    s = _sup(request_s=0.0, recover_timeout_s=2.0)
    t = _feed(s, 0.0, 2.0, math.radians(1.0), RC, V)
    t = _feed(s, t, 4.0, B, R, V)
    assert s.mode == HOLD
    t = _feed(s, t, 4.2, B - math.radians(20.0), R, V)        # drift lost
    assert s.mode == RECOVER
    _feed(s, t, 7.0, math.radians(30.0), R, V)                # no recovery within 2 s
    assert s.mode == FAILED
    _feed(s, 7.0, 9.0, 0.0, RC, V, straight=True)
    assert s.mode == FAILED                                    # final


def test_spin_fails_from_any_mode_and_approach_times_out():
    s = _sup(request_s=0.0)
    _feed(s, 0.0, 0.5, math.radians(70.0), 0.0, V)
    assert s.mode == FAILED and "spin" in s.transitions[-1].reason
    a = _sup(request_s=0.0, approach_timeout_s=1.0)
    _feed(a, 0.0, 2.0, 0.0, 0.0, V)                           # never reaches cornering
    assert a.mode == RECOVER and a.transitions[-1].reason == "approach timeout"


def test_no_request_below_minimum_speed():
    s = _sup(request_s=0.0)
    _feed(s, 0.0, 1.0, 0.0, 0.0, 3.0)
    assert s.mode == DISABLED
