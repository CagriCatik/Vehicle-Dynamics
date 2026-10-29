"""Experiment specification from YAML (Plan.md M9; experiment types: Plan V3.0 §12).

:func:`load_experiment` and :func:`parse_experiment` dispatch on the top-level key
``type`` (:mod:`.types`); this module defines the default type
``controller_comparison``. Such an experiment selects vehicle, tyres, optional K&C, road, manoeuvre, controllers,
an optional parameter sweep, the seed and the outputs. Validation is strict:
unknown keys or values raise ``ValueError`` with the offending path, so a typo
cannot silently fall back to a default.

Example (``configs/experiments/sine_with_dwell_comparison.yaml``)::

    name: sine_with_dwell_comparison
    vehicle: configs/vehicles/reference_passenger_car.yaml
    tyres: {front: magic_formula_passenger_synthetic, rear: magic_formula_passenger_synthetic}
    kc: configs/suspension/kc_reference_passenger_car.yaml        # optional
    road: {type: uniform, mu: 1.0}          # uniform | split_mu (mu_left, mu_right)
    manoeuvre: {type: sine_with_dwell, speed_kph: 80, amplitude_deg: 8}
    controllers:
      - {law: passive}
      - {law: pi}
      - {law: gs_lqr, inputs: 4ws_tv}
      - {law: ltv_mpc, inputs: tv, estimated_state: true}
    sweep: {parameter: manoeuvre.speed_kph, values: [60, 80, 100]}   # optional
    dt: 0.005
    seed: 0
    outputs: {directory: output/experiments/sine_with_dwell_comparison, formats: [csv, json, png, md]}
"""

from __future__ import annotations

import copy
from dataclasses import dataclass, field
from pathlib import Path

import yaml

LAWS = ("passive", "pi", "lqr", "gs_lqr", "ltv_mpc")
FEEDBACK_MODES = ("ideal", "estimated", "sensor_constrained")
_LAW_NAMES = {"pi": "PI", "lqr": "LQR", "gs_lqr": "GS-LQR", "ltv_mpc": "LTV-MPC"}
INPUT_SETS = ("tv", "4ws_tv")
ROADS = {"uniform": {"mu"}, "split_mu": {"mu_left", "mu_right"}}
MANOEUVRES = {
    "step_steer": {"speed_kph", "steer_deg", "duration_s"},
    "sine_with_dwell": {"speed_kph", "amplitude_deg"},
    "split_mu_braking": {"speed_kph"},
}
FORMATS = ("csv", "json", "png", "md", "gif")
DEFAULT_FORMATS = ("csv", "json", "png", "md")  # gif is opt-in (rendering time)
_TOP = {"type", "name", "description", "vehicle", "tyres", "kc", "road", "manoeuvre", "controllers", "sweep", "dt",
        "seed", "outputs", "verification"}


@dataclass(frozen=True)
class ControllerSpec:
    law: str
    inputs: str = "tv"
    feedback: str = "ideal"         # Plan V3.0 M10 5.4: ideal (Mode A), estimated (B), sensor_constrained (C)

    @property
    def estimated_state(self) -> bool:
        return self.feedback != "ideal"

    @property
    def label(self) -> str:
        if self.law == "passive":
            return "passive"
        lab = f"{_LAW_NAMES[self.law]} ({self.inputs.replace('_', '+').upper()})"
        return lab + {"ideal": "", "estimated": " + observer", "sensor_constrained": " + sensors only"}[self.feedback]


@dataclass(frozen=True)
class ExperimentSpec:
    name: str
    vehicle: str
    tyres: dict
    road: dict
    manoeuvre: dict
    controllers: tuple
    kc: str | None = None
    sweep: dict | None = None
    dt: float = 0.005
    seed: int = 0
    outputs: dict = field(default_factory=dict)
    verification: bool = True
    description: str = ""
    source: str = ""
    kind = "controller_comparison"          # experiment type (class attribute, not a field)

    def with_parameter(self, path: str, value) -> ExperimentSpec:
        """Copy with ``section.key`` set (used by sweeps), validated again."""
        section, key = path.split(".", 1)
        data = self.to_dict()
        data[section] = dict(data[section], **{key: value})
        data.pop("sweep", None)
        return parse_controller_comparison(data, self.source)

    def to_dict(self) -> dict:
        d = {"type": self.kind, "name": self.name, "vehicle": self.vehicle, "tyres": dict(self.tyres), "road": dict(self.road),
             "manoeuvre": dict(self.manoeuvre), "dt": self.dt, "seed": self.seed, "outputs": dict(self.outputs),
             "verification": self.verification, "description": self.description,
             "controllers": [{"law": c.law, "inputs": c.inputs, "feedback": c.feedback} for c in self.controllers]}
        if self.kc:
            d["kc"] = self.kc
        if self.sweep:
            d["sweep"] = copy.deepcopy(self.sweep)
        return d


def _require(cond: bool, msg: str) -> None:
    if not cond:
        raise ValueError(msg)


def _keys(d: dict, allowed: set, where: str, required: set = frozenset()) -> None:
    _require(isinstance(d, dict), f"{where} must be a mapping")
    unknown = set(d) - set(allowed)
    _require(not unknown, f"{where}: unknown key(s) {sorted(unknown)}; allowed: {sorted(allowed)}")
    missing = set(required) - set(d)
    _require(not missing, f"{where}: missing key(s) {sorted(missing)}")


def parse_experiment(data: dict, source: str = "<dict>"):
    """Validated specification of the experiment type named by ``data["type"]`` (default ``controller_comparison``)."""
    from .types import DEFAULT_TYPE, get_type
    _require(isinstance(data, dict), f"{source} must be a mapping")
    try:
        kind = get_type(data.get("type", DEFAULT_TYPE))
    except ValueError as exc:
        raise ValueError(f"{source}: {exc}") from None
    return kind.parse(data, source)


def parse_controller_comparison(data: dict, source: str = "<dict>") -> ExperimentSpec:
    _require(data.get("type", ExperimentSpec.kind) == ExperimentSpec.kind, f"{source}: type must be {ExperimentSpec.kind!r}")
    _keys(data, _TOP, source, {"name", "vehicle", "road", "manoeuvre", "controllers"})
    _require(Path(data["vehicle"]).is_file(), f"{source}: vehicle file not found: {data['vehicle']}")
    tyres = data.get("tyres") or {}
    _keys(tyres, {"front", "rear"}, f"{source}: tyres")
    if data.get("kc"):
        _require(Path(data["kc"]).is_file(), f"{source}: K&C file not found: {data['kc']}")

    road = dict(data["road"])
    _require(road.get("type") in ROADS, f"{source}: road.type must be one of {sorted(ROADS)}")
    _keys(road, ROADS[road["type"]] | {"type"}, f"{source}: road", ROADS[road["type"]] | {"type"})
    for k in ROADS[road["type"]]:
        _require(0.0 < float(road[k]) <= 2.0, f"{source}: road.{k} must lie in (0, 2]")

    man = dict(data["manoeuvre"])
    _require(man.get("type") in MANOEUVRES, f"{source}: manoeuvre.type must be one of {sorted(MANOEUVRES)}")
    _keys(man, MANOEUVRES[man["type"]] | {"type"}, f"{source}: manoeuvre", {"type", "speed_kph"})
    _require(5.0 <= float(man["speed_kph"]) <= 250.0, f"{source}: manoeuvre.speed_kph must lie in [5, 250]")

    ctrls = data["controllers"]
    _require(isinstance(ctrls, list) and ctrls, f"{source}: controllers must be a non-empty list")
    specs = []
    for i, c in enumerate(ctrls):
        _keys(c, {"law", "inputs", "estimated_state", "feedback"}, f"{source}: controllers[{i}]", {"law"})
        _require(c["law"] in LAWS, f"{source}: controllers[{i}].law must be one of {LAWS}")
        inputs = c.get("inputs", "tv")
        _require(inputs in INPUT_SETS, f"{source}: controllers[{i}].inputs must be one of {INPUT_SETS}")
        _require(not (c["law"] == "pi" and inputs != "tv"), f"{source}: controllers[{i}]: the PI law has yaw-moment input only")
        est = c.get("estimated_state", False)
        _require(isinstance(est, bool), f"{source}: controllers[{i}].estimated_state must be true or false")
        feedback = c.get("feedback", "estimated" if est else "ideal")
        _require(feedback in FEEDBACK_MODES, f"{source}: controllers[{i}].feedback must be one of {FEEDBACK_MODES}")
        _require(not (est and feedback == "ideal"), f"{source}: controllers[{i}]: estimated_state: true contradicts feedback: ideal")
        _require(not (feedback != "ideal" and c["law"] == "passive"), f"{source}: controllers[{i}]: passive has no state feedback")
        specs.append(ControllerSpec(c["law"], inputs, feedback))

    sweep = data.get("sweep")
    if sweep is not None:
        _keys(sweep, {"parameter", "values"}, f"{source}: sweep", {"parameter", "values"})
        section = str(sweep["parameter"]).split(".", 1)
        _require(len(section) == 2 and section[0] in ("manoeuvre", "road"),
                 f"{source}: sweep.parameter must be 'manoeuvre.<key>' or 'road.<key>'")
        _require(isinstance(sweep["values"], list) and sweep["values"], f"{source}: sweep.values must be a non-empty list")

    dt = float(data.get("dt", 0.005))
    _require(0.0 < dt <= 0.02, f"{source}: dt must lie in (0, 0.02] s")
    outputs = dict(data.get("outputs") or {})
    _keys(outputs, {"directory", "formats"}, f"{source}: outputs")
    formats = outputs.setdefault("formats", list(DEFAULT_FORMATS))
    _require(set(formats) <= set(FORMATS), f"{source}: outputs.formats must be a subset of {FORMATS}")
    outputs.setdefault("directory", f"output/experiments/{data['name']}")

    spec = ExperimentSpec(
        name=str(data["name"]), vehicle=str(data["vehicle"]), tyres=dict(tyres), road=road, manoeuvre=man,
        controllers=tuple(specs), kc=data.get("kc"), sweep=sweep, dt=dt, seed=int(data.get("seed", 0)),
        outputs=outputs, verification=bool(data.get("verification", True)),
        description=str(data.get("description", "")), source=source,
    )
    if sweep is not None:  # every sweep point must be valid on its own
        for v in sweep["values"]:
            spec.with_parameter(sweep["parameter"], v)
    return spec


def load_experiment(path: str | Path):
    with open(path, "r", encoding="utf-8") as f:
        return parse_experiment(yaml.safe_load(f) or {}, str(path))


__all__ = ("ControllerSpec", "ExperimentSpec", "load_experiment", "parse_controller_comparison", "parse_experiment")
