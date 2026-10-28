"""M8 acceptance: free vibration, energy behaviour, frequency responses and defined road excitations.

The nonlinear quarter-car simulation (RK4, bump stops, unilateral tyre) is checked
against the analytical linear references of suspension/vertical_analysis.py, with
amplitudes small enough that the bump stops stay inactive and the tyre in contact.
"""

from __future__ import annotations

import dataclasses
import math

import numpy as np
import pytest
from scipy.signal import welch

from vehicle_model.roads.road_profile import ISO8608_CLASSES, HalfSineBump, ISO8608Profile
from vehicle_model.suspension.controllers import GroundhookController, SkyhookController
from vehicle_model.suspension.quarter_car import QuarterCarConfig, QuarterCarModel, QuarterCarState
from vehicle_model.suspension.vertical_analysis import energy, frequency_response, modes, power_balance

CFG = QuarterCarConfig()
UNDAMPED = dataclasses.replace(CFG, suspension_damping=0.0, tyre_damping=0.0)


def _simulate(cfg, T, dt, road=lambda t: (0.0, 0.0), state=None, damper=None):
    m = QuarterCarModel(cfg)
    s = state or QuarterCarState()
    t, hist = 0.0, []
    for _ in range(int(round(T / dt))):
        zr, dzr = road(t)
        c = None if damper is None else damper(s)
        s_next, out = m.step_with_road(s, t, dt, road, damper_c=c)
        hist.append((t, s, out, zr, dzr, c))
        s, t = s_next, t + dt
    return hist, s


# ----------------------------------------------------------------------------- modes
def test_undamped_natural_frequencies_match_the_characteristic_equation():
    ms, mu, ks, kt = CFG.sprung_mass, CFG.unsprung_mass, CFG.suspension_stiffness, CFG.tyre_stiffness
    # ms mu w^4 - (ks (ms + mu) + kt ms) w^2 + ks kt = 0
    b = ks * (ms + mu) + kt * ms
    w2 = np.sort(np.roots([ms * mu, -b, ks * kt]).real)
    f = [m[0] for m in modes(UNDAMPED)]
    assert np.allclose(f, np.sqrt(w2) / (2 * math.pi), rtol=1e-10)
    assert all(z == pytest.approx(0.0, abs=1e-12) for _, z in modes(UNDAMPED))


def test_modes_agree_with_the_model_s_own_modal_analysis():
    f_b, f_h, z_b, z_h = QuarterCarModel(CFG).modal_analysis()
    (f1, z1), (f2, z2) = modes(CFG)
    assert (f_b, f_h, z_b, z_h) == pytest.approx((f1, f2, z1, z2), rel=1e-12)


def test_free_vibration_frequencies_and_energy_conservation_without_damping():
    dt, T = 5e-4, 20.0
    hist, _ = _simulate(UNDAMPED, T, dt, state=QuarterCarState(zs=0.01, zu=0.004))  # excites both modes
    zs = np.array([h[1].zs for h in hist])
    spec = np.abs(np.fft.rfft(zs - zs.mean()))
    freq = np.fft.rfftfreq(len(zs), dt)
    (f1, _), (f2, _) = modes(UNDAMPED)
    peaks = freq[np.argmax(np.where(freq < 5, spec, 0))], freq[np.argmax(np.where(freq > 6, spec, 0))]
    assert peaks[0] == pytest.approx(f1, abs=1 / T) and peaks[1] == pytest.approx(f2, abs=1 / T)
    E = np.array([energy(UNDAMPED, h[1]) for h in hist])
    assert np.max(np.abs(E / E[0] - 1.0)) < 1e-6                 # RK4, 40 000 steps


def test_damped_free_vibration_dissipates_exactly_the_damper_power():
    dt, T = 2e-4, 3.0
    hist, s_end = _simulate(CFG, T, dt, state=QuarterCarState(zs=0.02, zu=0.0))
    E = np.array([energy(CFG, h[1]) for h in hist] + [energy(CFG, s_end)])
    D = np.array([power_balance(CFG, h[1])[0] for h in hist] + [power_balance(CFG, s_end)[0]])
    assert np.all(np.diff(E) <= 1e-12)                          # never gains energy
    dissipated = np.sum(0.5 * (D[1:] + D[:-1])) * dt
    assert E[0] - E[-1] == pytest.approx(dissipated, rel=1e-3)
    # bounce decay follows the modal damping ratio: envelope ratio per period exp(-2 pi zeta / sqrt(1 - zeta^2))
    (f1, z1), _ = modes(CFG)
    zs = np.array([h[1].zs for h in hist])
    peaks = [i for i in range(1, len(zs) - 1) if zs[i] > zs[i - 1] and zs[i] >= zs[i + 1] and zs[i] > 1e-4]
    assert len(peaks) >= 2


@pytest.mark.parametrize("f_hz", [1.0, 3.0, 8.0, 12.0])
def test_simulated_frequency_response_matches_the_analytical_one(f_hz):
    Z, dt = 0.002, 1e-4
    w = 2 * math.pi * f_hz
    road = lambda t: (Z * math.sin(w * t), Z * w * math.cos(w * t))
    T_settle = 8.0 / (2 * math.pi * modes(CFG)[0][0] * modes(CFG)[0][1]) + 1.0  # ~8 bounce time constants
    periods = max(5, int(round(f_hz * 2)))
    hist, _ = _simulate(CFG, T_settle + periods / f_hz, dt, road)
    tail = [h for h in hist if h[0] >= T_settle]
    t = np.array([h[0] for h in tail])
    basis = np.column_stack([np.sin(w * t), np.cos(w * t), np.ones_like(t)])

    def amplitude(y):
        a, b, _ = np.linalg.lstsq(basis, y, rcond=None)[0]
        return math.hypot(a, b)

    H = frequency_response(CFG, f_hz)
    acc = np.array([h[2].ddzs for h in tail])
    load = np.array([h[2].dynamic_tyre_load for h in tail])
    travel = np.array([h[1].zs - h[1].zu for h in tail])
    assert all(h[2].in_contact for h in tail)
    assert amplitude(acc) / Z == pytest.approx(abs(H["body_acceleration"][0]), rel=0.01)
    assert amplitude(load) / Z == pytest.approx(abs(H["dynamic_tyre_load"][0]), rel=0.01)
    assert amplitude(travel) / Z == pytest.approx(abs(H["suspension_travel"][0]), rel=0.01)


def test_energy_balance_with_road_input():
    dt, T = 1e-4, 1.5
    w = 2 * math.pi * 4.0
    road = lambda t: (0.003 * math.sin(w * t), 0.003 * w * math.cos(w * t))
    hist, s_end = _simulate(CFG, T, dt, road)
    E0, E1 = energy(CFG, hist[0][1], hist[0][3]), energy(CFG, s_end, road(T)[0])
    P = [power_balance(CFG, h[1], h[3], h[4]) for h in hist] + [power_balance(CFG, s_end, *road(T))]
    net_power = np.array([pr - pd for pd, pr in P])
    net = (net_power.sum() - 0.5 * (net_power[0] + net_power[-1])) * dt   # trapezoid, O(dt^2)
    assert E1 - E0 == pytest.approx(net, rel=1e-4)


# ----------------------------------------------------------------------------- road excitations
def test_iso8608_class_values_and_variance():
    assert ISO8608_CLASSES["A"] == pytest.approx(16e-6) and ISO8608_CLASSES["H"] == pytest.approx(16e-6 * 4**7)
    a, c = ISO8608Profile("A", seed=1), ISO8608Profile("C", seed=1)
    assert c.variance / a.variance == pytest.approx(16.0)       # two classes: 4^2
    x = np.arange(0.0, 2000.0, 0.05)
    z = c.height(x)
    assert np.std(z) == pytest.approx(math.sqrt(c.variance), rel=0.05)


def test_iso8608_profile_has_the_target_psd():
    road = ISO8608Profile("B", seed=4)
    dx = 0.05
    x = np.arange(0.0, 4000.0, dx)
    n, pxx = welch(road.height(x), fs=1.0 / dx, nperseg=2**14)
    band = (n > 0.03) & (n < 2.0)
    ratio = pxx[band] / road.psd(n[band])
    assert np.median(ratio) == pytest.approx(1.0, rel=0.1)
    assert np.all((ratio > 0.4) & (ratio < 2.5))


def test_slope_is_the_derivative_of_the_height():
    for prof in (ISO8608Profile("D", seed=2), HalfSineBump(x0=1.0, length=0.6, peak=0.04)):
        x = np.linspace(0.2, 1.9, 400)
        h = 1e-6
        assert np.allclose(prof.slope(x), (prof.height(x + h) - prof.height(x - h)) / (2 * h), rtol=1e-4, atol=1e-6)


def test_road_class_seed_reproducibility_and_validation():
    assert np.array_equal(ISO8608Profile("C", seed=3).height([1.0, 2.0]), ISO8608Profile("C", seed=3).height([1.0, 2.0]))
    with pytest.raises(ValueError):
        ISO8608Profile("Z")


def test_high_bump_lifts_the_wheel_and_the_model_reports_it():
    bump = HalfSineBump(x0=1.0, length=0.3, peak=0.08)
    v = 15.0
    road = lambda t: (float(bump.height(v * t)[0]), float(v * bump.slope(v * t)[0]))
    hist, _ = _simulate(CFG, 1.0, 1e-4, road)
    contact = np.array([h[2].in_contact for h in hist])
    loads = np.array([h[2].total_tyre_load for h in hist])
    assert not contact.all() and np.all(loads >= 0.0)
    assert np.all(loads[~contact] == 0.0)


# ----------------------------------------------------------------------------- semi-active damping
def _rms_harmonic(f_hz, controller, Z=0.003, T=4.0, dt=2e-4):
    w = 2 * math.pi * f_hz
    road = lambda t: (Z * math.sin(w * t), Z * w * math.cos(w * t))
    c = controller() if controller else None
    hist = _simulate(CFG, T, dt, road, damper=(lambda st: c.compute_damping(st.dzs, st.dzu, dt)) if c else None)[0]
    tail = hist[len(hist) // 2:]
    return (float(np.sqrt(np.mean([h[2].ddzs ** 2 for h in tail]))),
            float(np.sqrt(np.mean([h[2].dynamic_tyre_load ** 2 for h in tail]))))


@pytest.mark.slow
def test_semi_active_trade_off_between_comfort_and_road_holding():
    """Skyhook: better comfort at body bounce, worse wheel load at wheel hop; groundhook: the reverse."""
    (f_bounce, _), (f_hop, _) = modes(CFG)
    passive_b, sky_b, ground_b = (_rms_harmonic(f_bounce, c) for c in (None, SkyhookController, GroundhookController))
    passive_h, sky_h, ground_h = (_rms_harmonic(f_hop, c) for c in (None, SkyhookController, GroundhookController))
    assert sky_b[0] < 0.8 * passive_b[0]          # comfort at body bounce
    assert sky_h[1] > 1.3 * passive_h[1]          # paid with wheel-load variation at wheel hop
    assert ground_h[1] < passive_h[1]             # groundhook helps (slightly) at wheel hop
    assert ground_b[1] > passive_b[1]             # but the body mode is under-damped at bounce


@pytest.mark.slow
def test_skyhook_improves_comfort_on_an_iso_class_c_road():
    road_profile = ISO8608Profile("C", seed=11)
    v, dt, T = 20.0, 5e-4, 10.0
    road = lambda t: (float(road_profile.height(v * t)[0]), float(v * road_profile.slope(v * t)[0]))
    sky = SkyhookController()
    acc = {}
    for name, damper in (("passive", None), ("skyhook", lambda st: sky.compute_damping(st.dzs, st.dzu, dt))):
        hist = _simulate(CFG, T, dt, road, damper=damper)[0][2000:]
        acc[name] = float(np.sqrt(np.mean([h[2].ddzs ** 2 for h in hist])))
    assert acc["skyhook"] < 0.9 * acc["passive"]
