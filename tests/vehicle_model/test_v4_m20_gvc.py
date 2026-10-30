"""Plan V4.0 M20: GVC-like load-transfer control (own simplified reference strategy)."""

from __future__ import annotations

import math

import numpy as np
import pytest

from vehicle_model.controllers.g_vectoring import GVCConfig, GVectoringControl
from vehicle_model.vehicle_models import SevenDOFModel

VEH = "configs/vehicles/seven_dof_passenger.yaml"


@pytest.fixture(scope="module")
def p():
    return SevenDOFModel.from_yaml(VEH).params


def _ramp(g, sign=1.0, n=60, rate=math.radians(20.0), dt=0.01, vx=22.0, brake=0.0, abs_active=False):
    out = []
    for k in range(n):
        out.append(g.request(sign * rate * k * dt, vx, brake, abs_active, dt))
    return np.array(out)


def test_turn_in_decelerates_within_limits_and_turn_out_releases(p):
    c = GVCConfig(max_decel=0.5, max_jerk=2.0)
    g = GVectoringControl(p, c)
    a = _ramp(g)
    assert a.min() < -0.05 and a.min() >= -0.5 - 1e-12 and np.all(a <= 0.0)
    assert np.all(np.abs(np.diff(a)) <= 2.0 * 0.01 + 1e-12)                  # jerk limit of the request
    assert g.phase == "turn-in"
    hold = [g.request(math.radians(20.0) * 0.59, 22.0, 0.0, False, 0.01) for _ in range(200)]
    assert hold[-1] == pytest.approx(0.0, abs=1e-6) and g.phase == "release"    # released, never positive
    mirrored = _ramp(GVectoringControl(p, c), sign=-1.0)
    assert mirrored == pytest.approx(a)                                         # left/right symmetric


def test_gvc_yields_and_can_be_disabled(p):
    assert np.all(_ramp(GVectoringControl(p, GVCConfig(enabled=False))) == 0.0)
    assert np.all(_ramp(GVectoringControl(p), brake=0.3) == 0.0)                # driver brakes: driver has priority
    assert np.all(_ramp(GVectoringControl(p), abs_active=True) == 0.0)          # ABS active: safety function first
    assert np.all(_ramp(GVectoringControl(p), vx=3.0) == 0.0)


def test_spec_gvc_block():
    from vehicle_model.experiments.spec import parse_controller_comparison
    base = {"name": "x", "vehicle": VEH, "road": {"type": "uniform", "mu": 1.0},
            "manoeuvre": {"type": "steer_ramp", "speed_kph": 80, "steer_deg": 3.0}}
    ok = parse_controller_comparison({**base, "controllers": [{"law": "no_yaw", "gvc": {"enabled": True, "c_xy": 0.3}},
                                                              {"law": "no_yaw", "gvc": {"enabled": False}}]})
    assert ok.controllers[0].label.endswith("[GVC C_xy 0.3]") and parse_controller_comparison(ok.to_dict()).controllers == ok.controllers
    for ctrl, msg in (({"law": "passive", "gvc": {"enabled": True}}, "passive"),
                      ({"law": "no_yaw", "gvc": {"c_xy": 5.0}}, "c_xy"), ({"law": "no_yaw", "gvc": {"gain": 1}}, "unknown key")):
        with pytest.raises(ValueError, match=msg):
            parse_controller_comparison({**base, "controllers": [ctrl]})


def _run(text, tmp_path):
    import yaml

    from vehicle_model.experiments.runner import run_experiment
    from vehicle_model.experiments.spec import parse_experiment
    res = run_experiment(parse_experiment(yaml.safe_load(text), "<t>"), tmp_path, verify=False)["results"]
    return {r["controller"]: r["kpis"] for r in res}


def test_disabled_gvc_vanishes_and_the_mechanism_exists_in_the_7dof_plant(tmp_path):
    k = _run(f"""
name: g
vehicle: {VEH}
road: {{type: uniform, mu: 1.0}}
manoeuvre: {{type: steer_ramp, speed_kph: 80, steer_deg: 3.0, steer_rate_degps: 20, accel_pedal: 0.05, duration_s: 2.0}}
controllers:
  - {{law: no_yaw}}
  - {{law: no_yaw, gvc: {{enabled: false}}}}
  - {{law: no_yaw, gvc: {{enabled: true}}}}
outputs: {{formats: [json]}}
""", tmp_path)
    base, off, on = k["no yaw control"], k["no yaw control [GVC off]"], k["no yaw control [GVC]"]
    assert {x: v for x, v in off.items() if not x.startswith("computation")} == \
           {x: v for x, v in base.items() if not x.startswith("computation")}       # enabled: false = absent
    # the reported mechanism: deceleration on turn-in loads the front axle (measured +80 N over GVC off)
    assert on["front_axle_load_gain_n"] > base["front_axle_load_gain_n"] + 50.0
    assert on["speed_loss_kph"] > base["speed_loss_kph"]


def test_dynamic_mechanism_in_the_14dof_plant():
    """A GVC-sized deceleration (0.05 g by the brakes) pitches the 14-DOF body nose-down and loads the front axle."""
    from vehicle_model.vehicle_models.fourteen_dof import FourteenDOFConfig, FourteenDOFModel, FourteenDOFState
    cfg = FourteenDOFConfig.from_yaml("configs/suspension/fourteen_dof.yaml")
    m = FourteenDOFModel(cfg)
    T = 0.05 * 9.81 * cfg.total_mass * cfg.wheel_radius / 4.0
    s, s0 = FourteenDOFState(vx=22.0), FourteenDOFState(vx=22.0)
    for _ in range(600):
        s, tel = m.step(s, np.zeros(4), np.zeros(4), np.full(4, T), 0.001)
        s0, tel0 = m.step(s0, np.zeros(4), np.zeros(4), np.zeros(4), 0.001)
    front, front0 = tel.Fz[0] + tel.Fz[1], tel0.Fz[0] + tel0.Fz[1]
    rigid = cfg.total_mass * abs(tel.ax - tel0.ax) * cfg.cg_height / cfg.wheelbase
    # measured: the 14-DOF model transfers about two thirds of the rigid-body value m a_x h / L at steady state
    # (89 % at the pitch overshoot); the front axle gains load and the body pitches nose-down
    assert 0.5 * rigid < front - front0 < 1.0 * rigid
    assert s.theta < s0.theta                                                    # more nose-down than coasting
