"""Low-speed manoeuvring and parking with rear-axle steering (4WS vs 2WS).

At walking pace the calibrated look-up table steers the rear axle against the front
(counter-phase, about -8 deg at 30 deg front): the instantaneous centre of rotation
moves forward, the turning radius shrinks and the vehicle needs less space.

To compare like with like, the driver works in *phases* that end on an event, not
after a fixed time: "reverse at full lock until the heading has turned by 90 deg"
reaches the same final heading with 2WS and 4WS; what differs is the space used.
Each phase has a signed target speed (negative = reverse), a front road-wheel angle
(reached at a limited steering rate) and an end condition:

    ("heading", deg)     |psi - psi_0| has turned by at least ``deg`` within the phase
    ("heading_to", deg)  the heading relative to the start pose has reached ``deg``
    ("lateral", m)       the CoG has moved ``m`` sideways (|Y - Y_0|) within the phase
    ("distance", m)      path length within the phase
    ("time", s)          duration of the phase

Driving to a position (``lateral``) rather than to a heading keeps the comparison fair
where the more agile 4WS vehicle would otherwise end a "turn by" phase earlier and with
less offset.

Speed is held by a P controller on the drive torque; at the end the vehicle brakes to
standstill. The space need is measured on the body outline (corners), not on the CoG:
:func:`swept_extent` gives the bounding box of all body corners over the manoeuvre.

Scenarios (all on dry asphalt, mu = 1):

    perpendicular_parking   reverse 90 deg into a bay from the aisle (bay width needed)
    u_turn                  U-turn at full lock on a narrow street (street width needed)
    parallel_exit           pull out of a tight parallel bay (forward clearance)
    tight_alley             forward 90 deg turn from a narrow alley into another
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from vehicle_model.core.types import SevenDOFControlInput, SevenDOFState
from vehicle_model.roads.surface import UniformRoad
from vehicle_model.steering.four_wheel_steering import FourWheelSteering, SteeringConfig, SteeringMode
from vehicle_model.vehicle_models.seven_dof import SevenDOFModel

from .results import ScenarioMetrics, ScenarioRunResult

BODY_LENGTH = 4.50       # [m] as drawn by the animation
BODY_WIDTH = 1.80        # [m]


@dataclass(frozen=True)
class Phase:
    speed: float             # [m/s] signed target speed (negative = reverse)
    steer_deg: float         # [deg] front road-wheel angle requested by the driver
    until: str               # "heading", "heading_to", "lateral", "distance" or "time"
    value: float             # [deg], [m] or [s]


@dataclass(frozen=True)
class ParkingScenario:
    name: str
    title: str
    description: str
    phases: tuple[Phase, ...]
    start: tuple[float, float, float] = (0.0, 0.0, 0.0)    # X, Y [m], psi [deg] of the CoG
    obstacles: tuple[tuple[float, float, float, float], ...] = ()   # rectangles (x0, y0, width, height) for drawing


def body_overhangs(lf: float, lr: float, length: float = BODY_LENGTH) -> tuple[float, float]:
    """Front and rear overhang [m] so that the body is ``length`` long, split in proportion to lf and lr."""
    k = length / (1.5 * (lf + lr))
    return 1.5 * lf * k - lf, 1.5 * lr * k - lr


def body_corners(X, Y, psi, lf: float, lr: float, length: float = BODY_LENGTH, width: float = BODY_WIDTH) -> np.ndarray:
    """Body corners (n, 4, 2) from CoG positions (same outline as the animation draws)."""
    f_oh, r_oh = body_overhangs(lf, lr, length)
    x_front, x_rear = lf + f_oh, -(lr + r_oh)
    local = np.array([[x_front, width / 2], [x_front, -width / 2], [x_rear, -width / 2], [x_rear, width / 2]])
    X, Y, psi = (np.atleast_1d(np.asarray(a, dtype=float)) for a in (X, Y, psi))
    c, s = np.cos(psi)[:, None], np.sin(psi)[:, None]
    return np.stack([X[:, None] + c * local[:, 0] - s * local[:, 1], Y[:, None] + s * local[:, 0] + c * local[:, 1]], axis=-1)


def clearance(result: ScenarioRunResult, lf: float, lr: float, rect: tuple[float, float, float, float],
              n_edge: int = 12) -> float:
    """Smallest distance [m] between the body outline and a rectangle (x0, y0, w, h) over the manoeuvre; 0 = contact.

    Sampled: points along the body edges against the rectangle, and the rectangle corners
    against the body (exact enough for a 1 cm statement with 12 points per edge).
    """
    from matplotlib.path import Path as MplPath

    x0, y0, w, h = rect
    rc = np.array([[x0, y0], [x0 + w, y0], [x0 + w, y0 + h], [x0, y0 + h]])
    best = np.inf
    f = np.linspace(0.0, 1.0, n_edge, endpoint=False)
    for corners in body_corners(result.X, result.Y, result.psi, lf, lr):
        edge_pts = np.concatenate([corners[i] + f[:, None] * (corners[(i + 1) % 4] - corners[i]) for i in range(4)])
        dx = np.maximum(np.maximum(x0 - edge_pts[:, 0], edge_pts[:, 0] - (x0 + w)), 0.0)
        dy = np.maximum(np.maximum(y0 - edge_pts[:, 1], edge_pts[:, 1] - (y0 + h)), 0.0)
        d = float(np.min(np.hypot(dx, dy)))
        if d == 0.0 or MplPath(corners).contains_points(rc).any():
            return 0.0
        best = min(best, d)
    return float(best)


def required_gap_ahead(result: ScenarioRunResult, lf: float, lr: float, lane_edge_y: float = 0.95) -> float:
    """Gap [m] to a car ahead (same lane, its side at ``lane_edge_y``) that the body needs to pull out.

    The furthest x any body point reaches while still at ``y <= lane_edge_y``, minus the
    front of the body at the start. Sampled along the body edges.
    """
    corners = body_corners(result.X, result.Y, result.psi, lf, lr)
    f = np.linspace(0.0, 1.0, 12, endpoint=False)
    pts = np.concatenate([corners[:, i, None, :] + f[None, :, None] * (corners[:, (i + 1) % 4, None, :] - corners[:, i, None, :])
                          for i in range(4)], axis=1).reshape(-1, 2)
    inside = pts[np.abs(pts[:, 1]) <= lane_edge_y]
    return float(np.max(inside[:, 0]) - np.max(corners[0, :, 0]))


def swept_extent(result: ScenarioRunResult, lf: float, lr: float) -> dict:
    """Bounding box of the body over the whole manoeuvre: x/y min and max, width and depth [m]."""
    pts = body_corners(result.X, result.Y, result.psi, lf, lr).reshape(-1, 2)
    xmin, ymin = pts.min(axis=0)
    xmax, ymax = pts.max(axis=0)
    return {"x_min": float(xmin), "x_max": float(xmax), "y_min": float(ymin), "y_max": float(ymax),
            "extent_x": float(xmax - xmin), "extent_y": float(ymax - ymin)}


def simulate_phased_manoeuvre(model: SevenDOFModel, scenario: ParkingScenario, enable_4ws: bool = True,
                              dt: float = 0.01, steer_rate_deg_s: float = 40.0, max_duration_s: float = 60.0,
                              controller_name: str | None = None) -> ScenarioRunResult:
    """Run the driver phases on the plant; 4WS uses the calibrated look-up table, 2WS keeps the rear straight."""
    p = model.params
    x0, y0, psi0_deg = scenario.start
    psi0 = math.radians(psi0_deg)
    v_start = scenario.phases[0].speed
    state = SevenDOFState(vx=v_start, X=x0, Y=y0, psi=psi0, omega=np.full(4, v_start / p.Reff))
    road = UniformRoad(mu=1.0)
    steering = FourWheelSteering(SteeringConfig.from_yaml())
    name = controller_name or ("4WS Calibrated LUT" if enable_4ws else "2WS Front-Only")

    hist = {k: [] for k in ("t", "X", "Y", "psi", "vx", "vy", "r", "beta", "sf", "sr", "ay")}
    t, steer, phase_i = 0.0, 0.0, 0
    phase_t0, phase_s0, phase_psi0, phase_y0, path = 0.0, 0.0, psi0, y0, 0.0
    stopping = False
    max_step = math.radians(steer_rate_deg_s) * dt
    while t < max_duration_s:
        if not stopping:
            ph = scenario.phases[phase_i]
            done = ((ph.until == "heading" and abs(math.degrees(state.psi - phase_psi0)) >= ph.value)
                    or (ph.until == "distance" and path - phase_s0 >= ph.value)
                    or (ph.until == "time" and t - phase_t0 >= ph.value)
                    or (ph.until == "lateral" and abs(state.Y - phase_y0) >= ph.value)
                    or (ph.until == "heading_to"
                        and (math.degrees(state.psi - psi0) - ph.value) * (math.degrees(phase_psi0 - psi0) - ph.value) <= 0.0))
            if done:
                phase_i += 1
                phase_t0, phase_s0, phase_psi0, phase_y0 = t, path, state.psi, state.Y
                if phase_i == len(scenario.phases):
                    stopping = True
        if stopping:
            v_target, steer_target = 0.0, steer
        else:
            ph = scenario.phases[phase_i]
            v_target, steer_target = ph.speed, math.radians(ph.steer_deg)
        steer += float(np.clip(steer_target - steer, -max_step, max_step))

        if enable_4ws:
            delta_f, delta_r = steering.compute_steering_angles(cmd_steer_f=steer, vx=abs(state.vx), vy=state.vy,
                                                                mode=SteeringMode.LOOKUP_TABLE)
        else:
            delta_f, delta_r = steer, 0.0

        if stopping:
            t_drive, t_brake = np.zeros(4), np.full(4, 400.0)
            if abs(state.vx) < 0.05:
                break
        elif v_target * state.vx < 0 and abs(state.vx) > 0.1:      # direction change: brake first
            t_drive, t_brake = np.zeros(4), np.full(4, 400.0)
        else:
            t_drive, t_brake = np.full(4, float(np.clip(300.0 * (v_target - state.vx), -250.0, 250.0))), np.zeros(4)

        u = SevenDOFControlInput(steer_f=delta_f, steer_r=delta_r, T_drive=t_drive, T_brake=t_brake).clamped(p)
        state, telem = model.step(state, u, dt=dt, road=road)
        t += dt
        path += math.hypot(state.vx, state.vy) * dt
        hist["t"].append(t)
        for k, v in (("X", state.X), ("Y", state.Y), ("psi", state.psi), ("vx", state.vx), ("vy", state.vy),
                     ("r", state.r), ("beta", math.degrees(math.atan2(state.vy, max(abs(state.vx), 0.2)))),
                     ("sf", math.degrees(u.steer_f)), ("sr", math.degrees(u.steer_r)),
                     ("ay", float(np.sum(telem.Fy_body) / p.mass))):
            hist[k].append(v)

    h = {k: np.array(v) for k, v in hist.items()}
    result = ScenarioRunResult(
        controller_name=name,
        metrics=ScenarioMetrics(scenario_name=scenario.title, controller_name=name),
        time=h["t"], X=h["X"], Y=h["Y"], psi=h["psi"], vx=h["vx"], vy=h["vy"], r=h["r"], beta_deg=h["beta"],
        steer_f_deg=h["sf"], steer_r_deg=h["sr"], ay_mps2=h["ay"])
    ext = swept_extent(result, p.lf, p.lr)
    m = result.metrics
    m.peak_sideslip_deg = float(np.max(np.abs(h["beta"])))
    m.peak_yaw_rate_deg_s = float(np.degrees(np.max(np.abs(h["r"]))))
    m.final_heading_dev_deg = float(np.degrees(h["psi"][-1] - psi0))
    m.max_lateral_dev_m = ext["extent_y"]
    m.stability_status = "PASS" if phase_i >= len(scenario.phases) else "INCOMPLETE"
    return result


# Start poses are the CoG; X along the aisle/street, Y across it.
PARKING_SCENARIOS: dict[str, ParkingScenario] = {
    "perpendicular_parking": ParkingScenario(
        "perpendicular_parking", "Reverse perpendicular parking (90 deg into a bay)",
        "Drive past the bay, then reverse at full lock until the car has turned 90 deg, and straighten into the bay. "
        "Space need: how far the car sweeps into the aisle and along it.",
        (Phase(1.5, 0.0, "distance", 4.0), Phase(-1.2, -32.0, "heading", 82.0), Phase(-1.0, 0.0, "distance", 2.5)),
        start=(0.0, 3.0, 0.0)),
    "u_turn": ParkingScenario(
        "u_turn", "U-turn on a narrow street (full lock)",
        "Turn around in one move at full lock. Space need: street width (across Y) the body sweeps.",
        (Phase(1.5, 0.0, "distance", 2.0), Phase(1.5, 32.0, "heading", 172.0), Phase(1.5, 0.0, "distance", 2.0)),
        start=(0.0, 0.0, 0.0)),
    "parallel_exit": ParkingScenario(
        "parallel_exit", "Pulling out of a tight parallel bay",
        "Parked in a row; steer to full lock at standstill, pull out at full lock until the car is 2.1 m out, "
        "counter-steer back to the original heading, drive on. Space need: the gap to the car ahead that the body "
        "needs (required_gap_ahead).",
        (Phase(0.0, 32.0, "time", 1.0), Phase(1.0, 32.0, "lateral", 2.1), Phase(1.5, -32.0, "heading_to", 2.0),
         Phase(1.5, 0.0, "distance", 4.0)),
        # car ahead 2.0 m in front, car behind 1.0 m behind, curb 0.3 m from the parked car's side
        start=(0.0, 0.0, 0.0), obstacles=((4.09, -0.95, 4.5, 1.9), (-7.91, -0.95, 4.5, 1.9), (-12.0, -1.45, 30.0, 0.25))),
    "tight_alley": ParkingScenario(
        "tight_alley", "90 deg turn between two narrow alleys",
        "Forward at walking pace, full lock left until turned 90 deg. Space need: the corner the body sweeps.",
        (Phase(2.0, 0.0, "distance", 3.0), Phase(2.0, 32.0, "heading", 82.0), Phase(2.0, 0.0, "distance", 4.0)),
        start=(0.0, 0.0, 0.0)),
}


def run_parking_comparison(model_factory, name: str, dt: float = 0.01) -> tuple[ScenarioRunResult, ScenarioRunResult, dict]:
    """4WS and 2WS on the same scenario (fresh plant each); returns both results and their swept extents."""
    sc = PARKING_SCENARIOS[name]
    r4 = simulate_phased_manoeuvre(model_factory(), sc, enable_4ws=True, dt=dt)
    model2 = model_factory()
    r2 = simulate_phased_manoeuvre(model2, sc, enable_4ws=False, dt=dt)
    p = model2.params
    return r4, r2, {"4WS": swept_extent(r4, p.lf, p.lr), "2WS": swept_extent(r2, p.lf, p.lr)}


def parking_summary(name: str, r4: ScenarioRunResult, r2: ScenarioRunResult, lf: float, lr: float) -> dict:
    """Space need of 4WS and 2WS: swept extent, clearances to the obstacles, gap needed ahead (parallel exit)."""
    sc = PARKING_SCENARIOS[name]
    out = {}
    for label, r in (("4WS", r4), ("2WS", r2)):
        d = {"duration_s": float(r.time[-1]), "final_heading_deg": r.metrics.final_heading_dev_deg,
             "max_rear_steer_deg": float(np.max(np.abs(r.steer_r_deg))), "completed": r.metrics.stability_status == "PASS",
             **swept_extent(r, lf, lr)}
        if sc.obstacles:
            d["clearances_m"] = [clearance(r, lf, lr, o) for o in sc.obstacles]
        if name == "parallel_exit":
            d["required_gap_ahead_m"] = required_gap_ahead(r, lf, lr)
        out[label] = d
    return out


def render_parking(name: str, model_factory, out_dir, gif: bool = True, dt: float = 0.01, png: bool = True) -> dict:
    """Run 4WS and 2WS, write ``<name>.gif`` (plan view + steering panel) and ``<name>.png``; return the summary."""
    from pathlib import Path

    from .animation import animate_4ws_detailed_vehicle, plot_4ws_comparison

    out_dir = Path(out_dir)
    r4, r2, _ = run_parking_comparison(model_factory, name, dt)
    p = model_factory().params
    summary = parking_summary(name, r4, r2, p.lf, p.lr)
    a, b = summary["4WS"], summary["2WS"]
    if name == "parallel_exit":
        foot = (f"Gap needed to the car ahead: 4WS {a['required_gap_ahead_m']:.2f} m, 2WS {b['required_gap_ahead_m']:.2f} m. "
                f"Clearance ahead / curb: 4WS {a['clearances_m'][0]:.2f} / {a['clearances_m'][2]:.2f} m, "
                f"2WS {b['clearances_m'][0]:.2f} / {b['clearances_m'][2]:.2f} m (rear swings toward the curb with 4WS).")
    else:
        foot = (f"Space swept by the body: 4WS {a['extent_x']:.2f} x {a['extent_y']:.2f} m, "
                f"2WS {b['extent_x']:.2f} x {b['extent_y']:.2f} m (along x, across y).")
    sc = PARKING_SCENARIOS[name]
    if png:
        plot_4ws_comparison(r4, r2, out_dir / f"{name}.png", sc.title, p.lf, p.lr, sc.obstacles, footer=foot)
    if gif:
        animate_4ws_detailed_vehicle(r4, r2, out_dir / f"{name}.gif", title=PARKING_SCENARIOS[name].title,
                                     lf=p.lf, lr=p.lr, obstacles=PARKING_SCENARIOS[name].obstacles, max_frames=110,
                                     footer=foot)
    return summary


__all__ = ("PARKING_SCENARIOS", "ParkingScenario", "Phase", "body_corners", "body_overhangs", "clearance", "required_gap_ahead",
           "parking_summary", "render_parking", "run_parking_comparison",
           "simulate_phased_manoeuvre", "swept_extent")
