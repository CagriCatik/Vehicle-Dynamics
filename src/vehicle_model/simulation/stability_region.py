"""Numerical recovery regions in the (sideslip, yaw rate) plane (Plan V3.0 M11 6.3, 6.4).

A **problem** (:class:`RecoveryProblem`) fixes vehicle, tyres, friction, the operating
point (speed, front steer held by the driver) and a controller. From each initial
state ``(beta0, r0)`` at the operating speed (wheels free rolling) the closed loop
is simulated while the driver holds the steer and a speed-holding pedal. Each run
is classified (:func:`classify`) into exactly one outcome:

    returned            inside the target region around the reference for ``hold`` seconds
    loss_of_control     |beta| >= ``loss_beta`` (explicit criterion; beta = atan2(vy, vx), so a spin counts)
    large_deviation     settled (sideslip and yaw-rate rates small over the last ``hold`` seconds)
                        but outside the target region
    numerical_failure   non-finite state or an exception in model or controller
    inconclusive        none of the above when the simulation time ends

Reference of the target region: the end state of the same closed loop started at the
open-loop equilibrium of the operating point (:class:`~vehicle_model.handling_analysis.equilibrium.EquilibriumAnalysis`),
so every controller is judged against its own steady state for the same driver input.

A map over a grid of initial states is a *numerically determined recovery region*,
not a proven region of attraction: it holds for the grid points, the step size and
the duration used. :func:`compute_map` therefore reports the uncertainty separately:

* classification certainty per point: points next to a different outcome are
  ``boundary``; boundary and undecided points are simulated again with a smaller
  step and a longer duration; a changed outcome marks the point ``sensitive``;
* grid: midpoints between neighbouring points of different outcome are simulated
  (one refinement level), and the region share on the refined grid is reported
  next to the coarse one (midpoints between equal outcomes inherit them: an
  assumption, stated in the result).

Runs are independent and deterministic, so a process pool gives the same result as
a serial run.
"""

from __future__ import annotations

import math
import os
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict, dataclass, field, replace
from functools import lru_cache

import numpy as np

OUTCOMES = ("returned", "large_deviation", "loss_of_control", "numerical_failure", "inconclusive")
ESC_DEADBAND = math.radians(2.0)   # [rad/s] yaw-rate error below which the ESC variant does not intervene


def _controllers():
    from vehicle_model.controllers.framework import FourWheelSteeringController, PassiveBaselineController
    from vehicle_model.controllers.integrated_controller import IntegratedChassisController
    from vehicle_model.controllers.stability_control import AllocatedStabilityController, StabilityControlConfig
    from vehicle_model.steering import SteeringMode

    return {
        "passive": lambda p: PassiveBaselineController(p),
        "esc": lambda p: AllocatedStabilityController(p, StabilityControlConfig(deadband=ESC_DEADBAND)),
        "4ws": lambda p: FourWheelSteeringController(params=p, mode=SteeringMode.LOOKUP_TABLE, enable_yaw_damping=True),
        "tv": lambda p: AllocatedStabilityController(p),
        "ivmc": lambda p: IntegratedChassisController(p),
        "ivmc_no_phase_plane": lambda p: IntegratedChassisController(p, enable_phase_plane=False),
    }


CONTROLLERS = ("passive", "esc", "4ws", "tv", "ivmc", "ivmc_no_phase_plane")
CONTROLLER_DESCRIPTIONS = {
    "passive": "driver steer, open differential, no control",
    "esc": "front steer by the driver, yaw-rate PI through brake/drive allocation, deadband 2 deg/s",
    "4ws": "rear-axle steering (lookup table) with yaw damping",
    "tv": "torque vectoring: yaw-rate PI through the allocation, no deadband",
    "ivmc": "integrated control (4WS, TV, ABS/TCS, heuristic phase-plane envelope)",
    "ivmc_no_phase_plane": "integrated control without the heuristic phase-plane envelope",
}


@dataclass(frozen=True)
class RecoveryCriteria:
    target_beta: float = math.radians(1.0)       # [rad] half-width of the target region
    target_r: float = math.radians(2.0)          # [rad/s]
    hold: float = 0.5                            # [s]
    loss_beta: float = math.radians(30.0)        # [rad]
    settle_beta_rate: float = math.radians(0.5)  # [rad/s]   "settled": |beta_dot| below this ...
    settle_r_rate: float = math.radians(1.0)     # [rad/s^2] ... and |r_dot| below this over the last hold


@dataclass(frozen=True)
class RecoveryProblem:
    """Everything a run needs; plain data, so it can be sent to worker processes."""

    controller: str
    speed: float                      # [m/s]
    steer: float = 0.0                # [rad] front steer held by the driver
    mu: float = 1.0
    vehicle: str | None = None        # vehicle YAML, None: default
    tyre_front: str | None = None
    tyre_rear: str | None = None
    dt: float = 0.01
    duration: float = 5.0
    criteria: RecoveryCriteria = field(default_factory=RecoveryCriteria)
    pedal_gain: float = 0.05          # [1/(m/s)] speed-holding pedal around the equilibrium pedal

    def __post_init__(self):
        if self.controller not in CONTROLLERS:
            raise ValueError(f"unknown controller {self.controller!r}; known: {CONTROLLERS}")


@dataclass
class RunOutcome:
    beta0: float
    r0: float
    outcome: str
    t_end: float
    final_beta: float
    final_r: float
    max_abs_beta: float
    history: dict | None = None       # t, beta, r, beta_dot, vx, steer_f, steer_r (when recorded)
    error: str = ""


# ----------------------------------------------------------------------------- classification
def classify(t: np.ndarray, beta: np.ndarray, r: np.ndarray, beta_ref: float, r_ref: float,
             crit: RecoveryCriteria, numerical_failure: bool = False) -> str:
    """Outcome of one trajectory (see module docstring); the arrays start at the first step."""
    if numerical_failure:
        return "numerical_failure"
    if t.size == 0:
        return "inconclusive"
    if np.max(np.abs(beta)) >= crit.loss_beta:
        return "loss_of_control"
    dt = float(t[1] - t[0]) if t.size > 1 else float(t[0])
    n_hold = max(1, int(round(crit.hold / dt)))
    inside = (np.abs(beta - beta_ref) < crit.target_beta) & (np.abs(r - r_ref) < crit.target_r)
    if inside.size >= n_hold and np.all(inside[-n_hold:]):
        return "returned"
    return "large_deviation" if _settled(t, beta, r, crit) else "inconclusive"


def _settled(t, beta, r, crit: RecoveryCriteria) -> bool:
    """Sideslip and yaw-rate rates below the settle limits over the last ``hold`` seconds."""
    if t.size < 2:
        return False
    dt = float(t[1] - t[0])
    n_hold = max(1, int(round(crit.hold / dt)))
    if t.size <= n_hold:
        return False
    db = np.abs(np.diff(beta[-n_hold - 1:])) / dt
    dr = np.abs(np.diff(r[-n_hold - 1:])) / dt
    return bool(np.max(db) < crit.settle_beta_rate and np.max(dr) < crit.settle_r_rate)


# ----------------------------------------------------------------------------- one run
@lru_cache(maxsize=8)
def _setup(problem: RecoveryProblem):
    """Model factory, controller factory, equilibrium (open loop) and pedal of the operating point."""
    from vehicle_model.handling_analysis.equilibrium import EquilibriumAnalysis, OperatingPoint
    from vehicle_model.roads.surface import UniformRoad
    from vehicle_model.vehicle_models import SevenDOFModel

    def model():
        return SevenDOFModel.from_yaml(problem.vehicle, tyre_name=problem.tyre_front, road=UniformRoad(mu=problem.mu),
                                       tyre_rear_name=problem.tyre_rear)

    m = model()
    eq = EquilibriumAnalysis(m).solve(OperatingPoint(problem.speed, problem.steer))
    if not eq.accepted:
        raise ValueError(f"no equilibrium at {problem.speed:.1f} m/s, steer {math.degrees(problem.steer):.2f} deg, "
                         f"mu {problem.mu}: the operating point is beyond the steady-state limit")
    pedal0 = eq.drive_total / (4.0 * m.params.max_drive_torque)
    return model, _controllers()[problem.controller], eq, pedal0


def equilibrium(problem: RecoveryProblem):
    """Open-loop equilibrium of the operating point (accepted, else ``ValueError``)."""
    return _setup(problem)[2]


def simulate(problem: RecoveryProblem, beta0: float, r0: float, reference: tuple[float, float] | None,
             record: bool = False, stop_early: bool = True) -> RunOutcome:
    """Closed-loop run from ``(beta0, r0)``; ``reference = None`` runs to the end (reference run)."""
    from vehicle_model.controllers.framework import ControlTarget
    from vehicle_model.core.types import SevenDOFControlInput, SevenDOFState

    make_model, make_ctrl, _, pedal0 = _setup(problem)
    model = make_model()
    p = model.params
    ctrl = make_ctrl(p)
    crit = problem.criteria
    v = problem.speed
    s = SevenDOFState(vx=v * math.cos(beta0), vy=v * math.sin(beta0), r=r0)
    _, _, _, v_wx, _ = model.wheel_velocities(s, SevenDOFControlInput(steer_f=problem.steer))
    s.omega = v_wx / p.Reff
    n_steps = int(round(problem.duration / problem.dt))
    n_hold = max(1, int(round(crit.hold / problem.dt)))
    t_l, b_l, r_l, vx_l, sf_l, sr_l = [], [], [], [], [], []
    tel, failed, err, run_in = None, False, "", 0
    try:
        for k in range(n_steps):
            pedal = float(np.clip(pedal0 + problem.pedal_gain * (v - s.vx), 0.0, 1.0))
            out = ctrl.compute_control(s, ControlTarget(target_vx=v, driver_steer_f=problem.steer,
                                                        driver_accel_pedal=pedal, mu_estimated=problem.mu),
                                       tel, problem.dt)
            s, tel = model.step(s, SevenDOFControlInput(steer_f=out.steer_f, steer_r=out.steer_r,
                                                        T_drive=out.T_drive, T_brake=out.T_brake), problem.dt)
            y = s.to_array()
            if not np.all(np.isfinite(y)):
                failed, err = True, "non-finite state"
                break
            beta = math.atan2(s.vy, s.vx)
            t_l.append((k + 1) * problem.dt)
            b_l.append(beta)
            r_l.append(s.r)
            vx_l.append(s.vx)
            sf_l.append(out.steer_f)
            sr_l.append(out.steer_r)
            if not stop_early or reference is None:
                continue
            if abs(beta) >= crit.loss_beta:
                break
            inside = abs(beta - reference[0]) < crit.target_beta and abs(s.r - reference[1]) < crit.target_r
            run_in = run_in + 1 if inside else 0
            if run_in >= n_hold:
                break
    except Exception as ex:  # a failing model or controller is an outcome, not a crash of the map
        failed, err = True, f"{type(ex).__name__}: {ex}"
    t, b, r = np.array(t_l), np.array(b_l), np.array(r_l)
    ref = reference if reference is not None else ((b[-1], r[-1]) if b.size else (0.0, 0.0))
    outcome = classify(t, b, r, ref[0], ref[1], crit, failed)
    hist = None
    if record:
        bd = np.gradient(b, problem.dt) if b.size > 1 else np.zeros_like(b)
        hist = {"t": t, "beta": b, "r": r, "beta_dot": bd, "vx": np.array(vx_l), "steer_f": np.array(sf_l),
                "steer_r": np.array(sr_l)}
    return RunOutcome(beta0, r0, outcome, float(t[-1]) if t.size else 0.0,
                      float(b[-1]) if b.size else float("nan"), float(r[-1]) if r.size else float("nan"),
                      float(np.max(np.abs(b))) if b.size else float("nan"), hist, err)


def reference_state(problem: RecoveryProblem) -> dict:
    """Closed-loop steady state from the open-loop equilibrium: centre of the target region."""
    eq = equilibrium(problem)
    run = simulate(problem, eq.beta, eq.r, None, record=True)
    h = run.history
    return {"beta": run.final_beta, "r": run.final_r,
            "settled": run.outcome != "numerical_failure" and _settled(h["t"], h["beta"], h["r"], problem.criteria),
            "equilibrium_beta": eq.beta, "equilibrium_r": eq.r, "max_abs_beta": run.max_abs_beta}


# ----------------------------------------------------------------------------- maps
def _worker(args):
    problem, beta0, r0, ref = args
    return simulate(problem, beta0, r0, ref)


def _run_all(problem: RecoveryProblem, points, ref, workers: int) -> list[RunOutcome]:
    tasks = [(problem, b, r, ref) for b, r in points]
    if workers <= 1 or len(tasks) < 2:
        return [_worker(a) for a in tasks]
    with ProcessPoolExecutor(max_workers=workers) as ex:
        return list(ex.map(_worker, tasks, chunksize=max(1, len(tasks) // (4 * workers))))


def default_workers() -> int:
    return max(1, min(8, (os.cpu_count() or 2) - 1))


def _boundary(codes: np.ndarray) -> np.ndarray:
    """Points with a 4-neighbour of a different outcome."""
    b = np.zeros(codes.shape, dtype=bool)
    b[1:, :] |= codes[1:, :] != codes[:-1, :]
    b[:-1, :] |= codes[:-1, :] != codes[1:, :]
    b[:, 1:] |= codes[:, 1:] != codes[:, :-1]
    b[:, :-1] |= codes[:, :-1] != codes[:, 1:]
    return b


@dataclass
class StabilityMap:
    problem: RecoveryProblem
    beta_grid: np.ndarray            # [rad]
    r_grid: np.ndarray               # [rad/s]
    outcomes: np.ndarray             # (n_beta, n_r) outcome names
    reference: dict
    certainty: np.ndarray            # "interior" | "boundary" | "sensitive"
    sensitivity: dict                # re-simulation results: step, duration
    refinement: dict                 # refined grid share of "returned"
    runs: list = field(default_factory=list, repr=False)

    def share(self, outcome: str = "returned") -> float:
        return float(np.mean(self.outcomes == outcome))

    def counts(self) -> dict:
        return {o: int(np.sum(self.outcomes == o)) for o in OUTCOMES}

    def summary(self) -> dict:
        ret = self.outcomes == "returned"
        unsure = self.certainty != "interior"
        return {"controller": self.problem.controller, "points": int(self.outcomes.size), "counts": self.counts(),
                "returned_share": self.share(),
                "returned_share_bounds": [float(np.mean(ret & ~unsure)), float(np.mean(ret | unsure))],
                "returned_share_refined": self.refinement.get("returned_share"),
                "boundary_points": int(np.sum(self.certainty != "interior")),
                "sensitive_points": int(np.sum(self.certainty == "sensitive")),
                "sensitivity": self.sensitivity, "reference": self.reference}


def compute_map(problem: RecoveryProblem, beta_grid, r_grid, workers: int | None = None,
                dt_factor: float = 0.5, duration_factor: float = 2.0, refine: bool = True) -> StabilityMap:
    """Recovery map on ``beta_grid x r_grid`` with the uncertainty studies of the module docstring."""
    workers = default_workers() if workers is None else int(workers)
    beta_grid = np.asarray(beta_grid, dtype=float)
    r_grid = np.asarray(r_grid, dtype=float)
    ref = reference_state(problem)
    ref_t = (ref["beta"], ref["r"])
    pts = [(b, r) for b in beta_grid for r in r_grid]
    runs = _run_all(problem, pts, ref_t, workers)
    shape = (beta_grid.size, r_grid.size)
    outcomes = np.array([o.outcome for o in runs], dtype=object).reshape(shape)

    # classification certainty: boundary and undecided points again with a smaller step / longer duration
    bnd = _boundary(outcomes)
    undecided = np.isin(outcomes, ("inconclusive", "large_deviation"))
    certainty = np.where(bnd, "boundary", "interior").astype(object)
    sens = {}
    for label, prob, mask in (
            ("step", replace(problem, dt=problem.dt * dt_factor), bnd | undecided),
            ("duration", replace(problem, duration=problem.duration * duration_factor), undecided)):
        idx = np.argwhere(mask)
        again = _run_all(prob, [(beta_grid[i], r_grid[j]) for i, j in idx], ref_t, workers)
        changed = [(int(i), int(j), outcomes[i, j], o.outcome) for (i, j), o in zip(idx, again)
                   if o.outcome != outcomes[i, j]]
        for i, j, _, _ in changed:
            certainty[i, j] = "sensitive"
        sens[label] = {"factor": dt_factor if label == "step" else duration_factor, "rerun": int(len(idx)),
                       "changed": len(changed),
                       "changes": [{"beta_deg": math.degrees(beta_grid[i]), "r_degps": math.degrees(r_grid[j]),
                                    "from": a, "to": b} for i, j, a, b in changed]}
    certainty[(certainty == "interior") & undecided] = "boundary"

    refinement = _refine(problem, beta_grid, r_grid, outcomes, ref_t, workers) if refine else {}
    return StabilityMap(problem, beta_grid, r_grid, outcomes, ref, certainty, sens, refinement, runs)


def _refine(problem, beta_grid, r_grid, outcomes, ref_t, workers) -> dict:
    """One refinement level: midpoints between differing neighbours are simulated, the others inherit."""
    nb, nr = outcomes.shape
    fine = np.empty((2 * nb - 1, 2 * nr - 1), dtype=object)
    fine[::2, ::2] = outcomes
    fb = np.interp(np.arange(2 * nb - 1) / 2.0, np.arange(nb), beta_grid)
    fr = np.interp(np.arange(2 * nr - 1) / 2.0, np.arange(nr), r_grid)
    todo = []
    for i in range(2 * nb - 1):
        for j in range(2 * nr - 1):
            if i % 2 == 0 and j % 2 == 0:
                continue
            parents = {outcomes[i // 2 + di, j // 2 + dj] for di in ((0, 1) if i % 2 else (0,))
                       for dj in ((0, 1) if j % 2 else (0,))}
            if len(parents) == 1:
                fine[i, j] = parents.pop()
            else:
                todo.append((i, j))
    runs = _run_all(problem, [(fb[i], fr[j]) for i, j in todo], ref_t, workers)
    for (i, j), o in zip(todo, runs):
        fine[i, j] = o.outcome
    return {"shape": list(fine.shape), "simulated_midpoints": len(todo),
            "inherited_midpoints": int(fine.size - outcomes.size - len(todo)),
            "returned_share": float(np.mean(fine == "returned")),
            "coarse_returned_share": float(np.mean(outcomes == "returned")),
            "outcomes": fine.tolist(), "beta_grid": fb.tolist(), "r_grid": fr.tolist()}


def heuristic_boundary(problem: RecoveryProblem, beta_grid, r_grid) -> np.ndarray:
    """Stability index ``S`` of the heuristic phase-plane monitor at each initial state (inside: ``S < 1``).

    The monitor of :mod:`vehicle_model.controllers.phase_plane` judges ``(beta, beta_dot)`` with
    ``beta_dot = a_y / v - r``; ``a_y`` is evaluated by the model at the initial state with the
    driver's steer, so the comparison uses the same information the monitor has in the loop.
    """
    from vehicle_model.controllers.phase_plane import PhasePlaneEnvelopeController
    from vehicle_model.core.types import SevenDOFControlInput, SevenDOFState

    make_model, _, _, _ = _setup(problem)
    model = make_model()
    mon = PhasePlaneEnvelopeController()
    u = SevenDOFControlInput(steer_f=problem.steer)
    S = np.empty((len(beta_grid), len(r_grid)))
    for i, b in enumerate(beta_grid):
        for j, r in enumerate(r_grid):
            s = SevenDOFState(vx=problem.speed * math.cos(b), vy=problem.speed * math.sin(b), r=r)
            _, _, _, v_wx, _ = model.wheel_velocities(s, u)
            s.omega = v_wx / model.params.Reff
            _, _, ax, ay = model.evaluate_dynamics(s, u)
            S[i, j] = mon.evaluate(s.vx, s.vy, s.r, ay, problem.mu, ax).stability_index
    return S


def problem_dict(problem: RecoveryProblem) -> dict:
    d = asdict(problem)
    d["steer_deg"] = math.degrees(d.pop("steer"))
    d["criteria"] = {k: (math.degrees(v) if k != "hold" else v) for k, v in d["criteria"].items()}
    return d


__all__ = ("CONTROLLERS", "CONTROLLER_DESCRIPTIONS", "ESC_DEADBAND", "OUTCOMES", "RecoveryCriteria",
           "RecoveryProblem", "RunOutcome", "StabilityMap", "classify", "compute_map", "default_workers",
           "equilibrium", "heuristic_boundary", "problem_dict", "reference_state", "simulate")
