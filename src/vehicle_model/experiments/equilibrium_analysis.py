"""Experiment type ``equilibrium_analysis``: nonlinear equilibria and local stability (6.2).

For every point of a sweep (and every curve of an optional series) the equilibria of the
two-track model are computed (:mod:`vehicle_model.handling_analysis.equilibrium`), linearised
with a step-size study, and their modes reported. The main branch follows the sweep by
continuation; ``search_all`` adds every further equilibrium reached from a seed grid
(saddles and unstable foci beyond the handling limit). ``linear_check`` compares the
linear prediction with the nonlinear model after a small perturbation.

Example (``configs/experiments/equilibrium_steer_sweep.yaml``)::

    type: equilibrium_analysis
    name: equilibrium_steer_sweep
    vehicle: configs/vehicles/seven_dof_passenger.yaml
    road: {type: uniform, mu: 0.6}
    operating_point: {speed_kph: 90}
    sweep: {parameter: steer_deg, values: [0, 0.5, 1, 1.5, 2, 3, 4]}
    series: {parameter: mu, values: [1.0, 0.6]}
    search_all: true
    linear_check: main            # main | all | none
    outputs: {directory: output/experiments/equilibrium_steer_sweep}

Parameters that ``sweep`` and ``series`` may vary (all also settable in ``operating_point``):

    speed_kph, steer_deg, rear_steer_ratio, tv_yaw_moment_nm     operating point
    mu                                                          road friction
    cornering_scale_front, cornering_scale_rear                 tyre cornering stiffness factor
                                                                (SlipStiffnessScaledTyre; peak friction kept)
    cog_shift_m                                                 centre of gravity moved forward (lf - d, lr + d)

``model: transient`` analyses the variant with tyre relaxation states; ``kc`` adds the
quasi-static K&C chassis (not together with ``cog_shift_m``). ``drive_split`` and
``brake_torque_nm`` of ``operating_point`` fix the torque distribution.
"""

from __future__ import annotations

import csv
import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from .spec import _keys, _require
from .types import provenance, write_json

KIND = "equilibrium_analysis"
_TOP = {"type", "name", "description", "vehicle", "tyres", "kc", "model", "road", "operating_point", "sweep",
        "series", "search_all", "linear_check", "outputs", "verification"}
SWEEPABLE = ("speed_kph", "steer_deg", "rear_steer_ratio", "tv_yaw_moment_nm", "mu", "cornering_scale_front",
             "cornering_scale_rear", "cog_shift_m")
_OP_EXTRA = ("drive_split", "brake_torque_nm")
_DEFAULTS = {"speed_kph": 80.0, "steer_deg": 0.0, "rear_steer_ratio": 0.0, "tv_yaw_moment_nm": 0.0,
             "cornering_scale_front": 1.0, "cornering_scale_rear": 1.0, "cog_shift_m": 0.0,
             "drive_split": [0.25, 0.25, 0.25, 0.25], "brake_torque_nm": [0.0, 0.0, 0.0, 0.0]}
_RANGES = {"speed_kph": (5.0, 250.0), "steer_deg": (-30.0, 30.0), "rear_steer_ratio": (-1.0, 1.0),
           "tv_yaw_moment_nm": (-5000.0, 5000.0), "mu": (0.05, 2.0), "cornering_scale_front": (0.2, 5.0),
           "cornering_scale_rear": (0.2, 5.0), "cog_shift_m": (-0.5, 0.5)}
LINEAR_CHECK = ("main", "all", "none")
FORMATS = ("csv", "json", "png", "md")
CHECK_TOL = 0.05   # linear check: max relative deviation of vy, r from the linear prediction


@dataclass(frozen=True)
class EquilibriumSpec:
    name: str
    vehicle: str
    base: dict                       # operating point and model parameters (SWEEPABLE + _OP_EXTRA + mu)
    sweep: dict                      # {parameter, values}
    series: dict | None = None
    tyres: dict = field(default_factory=dict)
    kc: str | None = None
    model: str = "seven_dof"
    search_all: bool = False
    linear_check: str = "main"
    outputs: dict = field(default_factory=dict)
    verification: bool = True
    description: str = ""
    source: str = ""
    kind = KIND

    def points(self):
        """``(series value | None, sweep value, configuration)`` in sweep order per series."""
        series = self.series["values"] if self.series else [None]
        for sv in series:
            for xv in self.sweep["values"]:
                cfg = dict(self.base)
                if self.series:
                    cfg[self.series["parameter"]] = sv
                cfg[self.sweep["parameter"]] = xv
                yield sv, xv, cfg

    def to_dict(self) -> dict:
        op = {k: v for k, v in self.base.items() if k != "mu"}
        return {"type": KIND, "name": self.name, "description": self.description, "vehicle": self.vehicle,
                "tyres": dict(self.tyres), "kc": self.kc, "model": self.model,
                "road": {"type": "uniform", "mu": self.base["mu"]}, "operating_point": op,
                "sweep": dict(self.sweep), "series": dict(self.series) if self.series else None,
                "search_all": self.search_all, "linear_check": self.linear_check, "outputs": dict(self.outputs),
                "verification": self.verification}


def _number(v, key, where):
    _require(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v), f"{where}: {key} must be a number")
    lo, hi = _RANGES[key]
    _require(lo <= float(v) <= hi, f"{where}: {key} must lie in [{lo}, {hi}]")
    return float(v)


def _axis(d, where: str, other: str | None = None) -> dict:
    _require(isinstance(d, dict), f"{where} must be a mapping")
    _keys(d, {"parameter", "values"}, where, {"parameter", "values"})
    _require(d["parameter"] in SWEEPABLE, f"{where}.parameter must be one of {SWEEPABLE}")
    _require(d["parameter"] != other, f"{where}.parameter must differ from the sweep parameter")
    vals = d["values"]
    _require(isinstance(vals, list) and vals, f"{where}.values must be a non-empty list")
    return {"parameter": d["parameter"], "values": [_number(v, d["parameter"], where) for v in vals]}


def parse(data: dict, source: str = "<dict>") -> EquilibriumSpec:
    _require(data.get("type") == KIND, f"{source}: type must be {KIND!r}")
    _keys(data, _TOP, source, {"type", "name", "vehicle", "road", "sweep"})
    _require(Path(data["vehicle"]).is_file(), f"{source}: vehicle file not found: {data['vehicle']}")
    tyres = dict(data.get("tyres") or {})
    _keys(tyres, {"front", "rear"}, f"{source}: tyres")
    kc = data.get("kc")
    _require(kc is None or Path(kc).is_file(), f"{source}: kc file not found: {kc}")
    model = data.get("model", "seven_dof")
    _require(model in ("seven_dof", "transient"), f"{source}: model must be 'seven_dof' or 'transient'")
    road = dict(data["road"])
    _keys(road, {"type", "mu"}, f"{source}: road", {"type", "mu"})
    _require(road["type"] == "uniform", f"{source}: road.type must be 'uniform' (equilibria need position-independent friction)")
    op = dict(data.get("operating_point") or {})
    _keys(op, set(SWEEPABLE) - {"mu"} | set(_OP_EXTRA), f"{source}: operating_point")
    base = {**_DEFAULTS, **op, "mu": road["mu"]}
    for k in SWEEPABLE:
        base[k] = _number(base[k], k, f"{source}: operating_point" if k != "mu" else f"{source}: road")
    split = base["drive_split"]
    _require(isinstance(split, list) and len(split) == 4 and abs(sum(map(float, split)) - 1.0) < 1e-9,
             f"{source}: operating_point.drive_split needs 4 shares that sum to 1")
    brake = base["brake_torque_nm"]
    _require(isinstance(brake, list) and len(brake) == 4 and min(map(float, brake)) >= 0.0,
             f"{source}: operating_point.brake_torque_nm needs 4 non-negative values")
    base["drive_split"], base["brake_torque_nm"] = [float(x) for x in split], [float(x) for x in brake]
    sweep = _axis(data["sweep"], f"{source}: sweep")
    series = _axis(data["series"], f"{source}: series", sweep["parameter"]) if data.get("series") else None
    uses_cog = "cog_shift_m" in (sweep["parameter"], (series or {}).get("parameter")) or base["cog_shift_m"] != 0.0
    _require(not (kc and uses_cog), f"{source}: cog_shift_m cannot be combined with kc (the K&C set fixes the geometry)")
    lc = data.get("linear_check", "main")
    _require(lc in LINEAR_CHECK, f"{source}: linear_check must be one of {LINEAR_CHECK}")
    search_all = data.get("search_all", False)
    _require(isinstance(search_all, bool), f"{source}: search_all must be true or false")
    outputs = dict(data.get("outputs") or {})
    _keys(outputs, {"directory", "formats"}, f"{source}: outputs")
    outputs.setdefault("formats", list(FORMATS))
    _require(set(outputs["formats"]) <= set(FORMATS), f"{source}: outputs.formats must be a subset of {FORMATS}")
    outputs.setdefault("directory", f"output/experiments/{data['name']}")
    return EquilibriumSpec(str(data["name"]), str(data["vehicle"]), base, sweep, series, tyres, kc, model, search_all,
                           lc, outputs, bool(data.get("verification", True)), str(data.get("description", "")), source)


def describe(spec: EquilibriumSpec) -> str:
    s = f"equilibria over {spec.sweep['parameter']} ({len(spec.sweep['values'])} points)"
    if spec.series:
        s += f" x {spec.series['parameter']} ({len(spec.series['values'])})"
    return s + (", all branches" if spec.search_all else ", main branch")


def build_model(spec: EquilibriumSpec, cfg: dict):
    """Model for one configuration: vehicle, tyres (scaled), K&C, friction, CoG shift."""
    from vehicle_model.handling_analysis.equilibrium import with_params
    from vehicle_model.roads.surface import UniformRoad
    from vehicle_model.tire_model import SlipStiffnessScaledTyre
    from vehicle_model.vehicle_models import SevenDOFModel, TransientSevenDOFModel

    cls = TransientSevenDOFModel if spec.model == "transient" else SevenDOFModel
    m = cls.from_yaml(spec.vehicle, tyre_name=spec.tyres.get("front"), road=UniformRoad(mu=float(cfg["mu"])),
                      tyre_rear_name=spec.tyres.get("rear"), kc_config=spec.kc)
    if cfg["cornering_scale_front"] != 1.0:
        m.tyre_front = SlipStiffnessScaledTyre(m.tyre_front, cfg["cornering_scale_front"])
    if cfg["cornering_scale_rear"] != 1.0:
        m.tyre_rear = SlipStiffnessScaledTyre(m.tyre_rear, cfg["cornering_scale_rear"])
    d = cfg["cog_shift_m"]
    if d:
        m = with_params(m, lf=m.params.lf - d, lr=m.params.lr + d)
    return m


def operating_point(cfg: dict):
    from vehicle_model.handling_analysis.equilibrium import OperatingPoint
    return OperatingPoint(cfg["speed_kph"] / 3.6, math.radians(cfg["steer_deg"]), cfg["rear_steer_ratio"],
                          cfg["tv_yaw_moment_nm"], tuple(cfg["drive_split"]), tuple(cfg["brake_torque_nm"]))


def analyse(spec: EquilibriumSpec) -> list[dict]:
    """All points: equilibria with linearisation (and linear checks as configured)."""
    from vehicle_model.handling_analysis.equilibrium import EquilibriumAnalysis

    rows, guess, last_series = [], None, object()
    for sv, xv, cfg in spec.points():
        if sv != last_series:
            guess, last_series = None, sv
        an = EquilibriumAnalysis(build_model(spec, cfg))
        op = operating_point(cfg)
        main = an.solve(op, guess)
        if not main.accepted and guess is not None:
            retry = an.solve(op)
            main = retry if retry.accepted else main
        found = [main] if main.accepted else []
        if spec.search_all:
            for e in an.find_all(op):
                if not any(abs(e.state.vy - f.state.vy) < 1e-4 * op.speed and abs(e.r - f.r) < 1e-5 for f in found):
                    found.append(e)
        eqs = []
        for e in found:
            an.linearise(e)
            is_main = e is main
            if spec.linear_check == "all" or (spec.linear_check == "main" and is_main):
                an.linear_check(e)
            eqs.append({**e.to_dict(), "branch": "main" if is_main else "other"})
        guess = an.unknowns(main) if main.accepted else guess
        rows.append({"series": sv, "sweep": xv, "config": cfg, "main_accepted": main.accepted,
                     "main_attempt": None if main.accepted else main.to_dict(),
                     "equilibria": sorted(eqs, key=lambda d: d["beta_deg"])})
    return rows


def checks(rows: list[dict]) -> list[dict]:
    """Acceptance checks of the analysis: residuals, Jacobian convergence, linear-vs-nonlinear agreement."""
    from vehicle_model.handling_analysis.equilibrium import JACOBIAN_CONVERGED, RESIDUAL_TOL

    eqs = [e for r in rows for e in r["equilibria"]]
    lin = [e["linearisation"] for e in eqs]
    lc = [e["linear_check"] for e in eqs if e["linear_check"]]
    worst_lc = max((max(c["relative_error_vy"], c["relative_error_r"]) for c in lc), default=0.0)
    missing = [r for r in rows if not r["main_accepted"]]
    out = [
        {"name": "points without an accepted equilibrium", "passed": None, "value": len(missing),
         "tolerance": float("nan"),
         "description": "reported, not judged: beyond a fold of the main branch no steady state exists, and the "
                        "solver stops at a finite residual (listed per point); a solver failure looks the same, "
                        "so such points are a limit to examine, not a proof of non-existence"},
        {"name": "accepted equilibria meet the residual tolerances",
         "passed": all(max(e["residual"][g] / RESIDUAL_TOL[g] for g in e["residual"]) <= 1.0 for e in eqs),
         "value": max((max(e["residual"][g] / RESIDUAL_TOL[g] for g in e["residual"]) for e in eqs), default=0.0),
         "tolerance": 1.0, "description": "largest residual relative to RESIDUAL_TOL over all accepted equilibria"},
        {"name": "Jacobian step study converged", "passed": all(l["converged"] for l in lin),
         "value": max((min(s["eigenvalue_change"] for s in l["step_study"]) for l in lin), default=0.0),
         "tolerance": JACOBIAN_CONVERGED,
         "description": "relative eigenvalue change between neighbouring difference steps at the chosen step"},
        {"name": "linear prediction matches the nonlinear model", "passed": worst_lc <= CHECK_TOL
         and all(c["same_tendency"] for c in lc), "value": worst_lc, "tolerance": CHECK_TOL,
         "description": f"max deviation of vy, r after a small perturbation, relative to the linear excursion "
                        f"({len(lc)} equilibria checked); decay or growth agree"},
    ]
    return out


def run(spec: EquilibriumSpec, out: Path, verify: bool | None = None) -> dict:
    from .verification import verify_model
    formats = set(spec.outputs.get("formats", ()))
    first = next(spec.points())[2]
    verification = (verify_model(lambda: build_model(spec, {**first, "mu": 1.0}))
                    if (spec.verification if verify is None else verify) else [])
    rows = analyse(spec)
    summary = {"experiment": spec.to_dict(), "source": spec.source, "provenance": provenance(),
               "verification": verification, "checks": checks(rows), "results": rows}
    if "json" in formats:
        write_json(out / "results.json", summary)
    if "csv" in formats:
        _csv(spec, rows, out / "equilibria.csv")
    if "png" in formats:
        _plot(spec, rows, out / "equilibria.png")
        _plot_eigenvalues(spec, rows, out / "eigenvalues.png")
    if "md" in formats:
        (out / "report.md").write_text(_report(spec, summary), encoding="utf-8")
    return summary


def _lateral(e: dict) -> dict | None:
    """Least damped lateral/yaw mode of an equilibrium (dict form)."""
    lat = [m for m in e["linearisation"]["modes"] if m["group"] == "lateral_yaw"]
    return max(lat, key=lambda m: m["re"]) if lat else None


_CSV = ("series", "sweep", "branch", "beta_deg", "r_degps", "ay", "vx", "drive_total_nm", "max_abs_kappa",
        "max_abs_alpha_deg", "support", "stability", "max_real", "critical_mode_group", "lateral_re", "lateral_im",
        "lateral_damping", "lateral_frequency_hz", "residual_translational", "residual_yaw", "residual_wheel",
        "jacobian_step", "jacobian_converged", "check_error_vy", "check_error_r")


def _csv(spec, rows, path: Path) -> None:
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow([spec.series["parameter"] if spec.series else "series", spec.sweep["parameter"], *_CSV[2:]])
        for r in rows:
            for e in r["equilibria"]:
                lin, lat, lc = e["linearisation"], _lateral(e), e["linear_check"] or {}
                w.writerow([r["series"], r["sweep"], e["branch"], e["beta_deg"], e["r_degps"], e["ay"], e["vx"],
                            e["drive_total_nm"], e["max_abs_kappa"], e["max_abs_alpha_deg"], e["support"],
                            lin["stability"], lin["max_real"], lin["critical_mode_group"],
                            lat["re"] if lat else "", lat["im"] if lat else "", lat["damping_ratio"] if lat else "",
                            lat["natural_frequency_hz"] if lat else "", e["residual"]["translational"],
                            e["residual"]["yaw"], e["residual"]["wheel"], lin["step"], lin["converged"],
                            lc.get("relative_error_vy", ""), lc.get("relative_error_r", "")])


_COLOURS = ("#3987e5", "#d95926", "#199e70", "#c98500", "#d55181", "#008300")  # validated categorical slots 1-6
_LABELS = {"speed_kph": "speed [km/h]", "steer_deg": "front steer [deg]", "rear_steer_ratio": "rear/front steer ratio [-]",
           "tv_yaw_moment_nm": "TV yaw moment [N m]", "mu": "friction mu [-]",
           "cornering_scale_front": "front cornering stiffness factor [-]",
           "cornering_scale_rear": "rear cornering stiffness factor [-]", "cog_shift_m": "CoG shift forward [m]"}


def _series_label(spec, sv) -> str:
    if not spec.series:
        return "main branch"
    return f"{spec.series['parameter']} = {sv:g}"


def _plot(spec, rows, path: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D

    from vehicle_model.visualization.theme import TEXT_MUTED, apply_dark_theme, style_axis
    apply_dark_theme()
    fig, ax = plt.subplots(2, 2, figsize=(14, 9))
    series = list(dict.fromkeys(r["series"] for r in rows))
    for sv, c in zip(series, _COLOURS * 4):
        rs = [r for r in rows if r["series"] == sv]
        main = [(r["sweep"], next(e for e in r["equilibria"] if e["branch"] == "main")) for r in rs if r["main_accepted"]]
        x = [m[0] for m in main]
        for a, key in ((ax[0, 0], "ay"), (ax[0, 1], "beta_deg")):
            a.plot(x, [e[key] for _, e in main], color=c, lw=2, label=_series_label(spec, sv))
            for r in rs:
                for e in r["equilibria"]:
                    stable = e["linearisation"]["stability"] == "stable"
                    a.plot(r["sweep"], e[key], "o", ms=7 if e["branch"] == "main" else 6, mec=c,
                           mfc=c if stable else "none", mew=2)
        lat = [_lateral(e) for _, e in main]
        ax[1, 0].plot(x, [m["damping_ratio"] if m else np.nan for m in lat], color=c, lw=2, marker="o", ms=6)
        for r in rs:
            for e in r["equilibria"]:
                ax[1, 1].plot(r["sweep"], e["linearisation"]["max_real"], "o", mec=c, mew=2, ms=6,
                              mfc=c if e["branch"] == "main" else "none")
        ax[1, 1].plot(x, [e["linearisation"]["max_real"] for _, e in main], color=c, lw=2)
    ax[1, 1].axhline(0.0, color=TEXT_MUTED, lw=1, ls="--")
    xl = _LABELS[spec.sweep["parameter"]]
    style_axis(ax[0, 0], "lateral acceleration of the equilibria", xl, "a_y [m/s²]")
    style_axis(ax[0, 1], "sideslip angle of the equilibria", xl, "beta [deg]")
    style_axis(ax[1, 0], "damping ratio of the lateral/yaw mode (main branch)", xl, "zeta [-]")
    style_axis(ax[1, 1], "largest real part of the eigenvalues (> 0: unstable)", xl, "max Re(lambda) [1/s]")
    handles, _ = ax[0, 0].get_legend_handles_labels()
    handles += [Line2D([], [], ls="", marker="o", mec=TEXT_MUTED, mfc=TEXT_MUTED, mew=2, label="stable"),
                Line2D([], [], ls="", marker="o", mec=TEXT_MUTED, mfc="none", mew=2, label="unstable")]
    ax[0, 0].legend(handles=handles, fontsize=8, loc="best")
    fig.suptitle(f"{spec.name}: equilibria of the 7-DOF model, line = main branch by continuation "
                 f"(synthetic parameters)")
    fig.tight_layout()
    fig.savefig(path, dpi=130, facecolor=fig.get_facecolor())
    plt.close(fig)


def _plot_eigenvalues(spec, rows, path: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    from vehicle_model.visualization.theme import TEXT_MUTED, apply_dark_theme, style_axis
    apply_dark_theme()
    fig, ax = plt.subplots(figsize=(9, 6.5))
    series = list(dict.fromkeys(r["series"] for r in rows))
    for sv, c in zip(series, _COLOURS * 4):
        pts = []
        for r in rows:
            if r["series"] != sv:
                continue
            for e in r["equilibria"]:
                if e["branch"] != "main":
                    continue
                for m in e["linearisation"]["modes"]:
                    if m["group"] == "lateral_yaw":
                        pts.append((r["sweep"], m["re"], m["im"]))
        if not pts:
            continue
        pts = np.array(pts)
        for half in (pts[:, 2] >= 0.0, pts[:, 2] < 0.0):   # locus in sweep order, upper and lower half plane
            ax.plot(pts[half, 1], pts[half, 2], "-", color=c, lw=1.2, alpha=0.7)
        ax.plot(pts[:, 1], pts[:, 2], "o", color=c, ms=5, alpha=0.9, label=_series_label(spec, sv))
        for xv, re, im in pts[_labelled(pts)]:
            ax.annotate(f"{xv:g}", (re, im), textcoords="offset points", xytext=(4, 4), fontsize=7, color=c)
    ax.axvline(0.0, color=TEXT_MUTED, lw=1, ls="--")
    ax.legend(fontsize=8)
    style_axis(ax, f"lateral/yaw eigenvalues on the main branch, labels: {_LABELS[spec.sweep['parameter']]}",
               "Re(lambda) [1/s]", "Im(lambda) [rad/s]")
    fig.tight_layout()
    fig.savefig(path, dpi=130, facecolor=fig.get_facecolor())
    plt.close(fig)


def _labelled(pts: np.ndarray) -> np.ndarray:
    """Indices of the labelled points: upper half plane (or real), first and last sweep value."""
    upper = pts[:, 2] >= 0.0
    xs = pts[:, 0]
    return upper & ((xs == xs.min()) | (xs == xs.max()))


def _report(spec, summary: dict) -> str:
    pv = summary["provenance"]
    L = [f"# Equilibrium analysis: {spec.name}", ""]
    if spec.description:
        L += [spec.description, ""]
    op = ", ".join(f"{k}: {v}" for k, v in spec.base.items() if k not in (spec.sweep["parameter"],
                                                                         (spec.series or {}).get("parameter")))
    L += ["## Setup", "", "| item | value |", "|---|---|", f"| specification | `{spec.source}` |",
          f"| vehicle | `{spec.vehicle}`, model {spec.model}" + (f", K&C `{spec.kc}`" if spec.kc else "") + " |",
          f"| tyres | {spec.tyres or 'from the vehicle file'} |", f"| fixed | {op} |",
          f"| sweep | {spec.sweep['parameter']}: {spec.sweep['values']} |"]
    if spec.series:
        L.append(f"| series | {spec.series['parameter']}: {spec.series['values']} |")
    L += [f"| search | {'all equilibria from a seed grid' if spec.search_all else 'main branch by continuation'} |",
          f"| code | git {pv['git_commit']}, Python {pv['python']}, numpy {pv['numpy']}, scipy {pv['scipy']} |", "",
          "All parameters are synthetic; results describe the virtual vehicle, not a real one.", ""]
    if summary["verification"]:
        ok = all(c["passed"] for c in summary["verification"])
        L += [f"Model verification: **{'PASS' if ok else 'FAIL'}** "
              f"({sum(c['passed'] for c in summary['verification'])}/{len(summary['verification'])} checks).", ""]
    L += ["## Checks of the analysis", "", "| check | result | value | tolerance |", "|---|---|---|---|"]
    for c in summary["checks"]:
        res = "info" if c["passed"] is None else ("PASS" if c["passed"] else "FAIL")
        tol = "–" if math.isnan(c["tolerance"]) else f"{c['tolerance']:.3g}"
        L.append(f"| {c['name']} | {res} | {c['value']:.3g} | {tol} |")
    sp = spec.series["parameter"] if spec.series else "series"
    L += ["", "## Equilibria", "",
          f"| {sp} | {spec.sweep['parameter']} | branch | beta [deg] | r [deg/s] | a_y [m/s²] | stability | "
          "max Re [1/s] | lateral mode | zeta | max residual |", "|---" * 11 + "|"]
    for r in summary["results"]:
        if not r["main_accepted"]:
            a = r["main_attempt"]
            L.append(f"| {r['series']} | {r['sweep']} | main | no accepted equilibrium: solver stopped at "
                     f"beta {a['beta_deg']:.2f}, a_y {a['ay']:.2f}, residual {max(a['residual'].values()):.1e} "
                     f"| | | | | | | |")
        for e in r["equilibria"]:
            lat = _lateral(e)
            lm = f"{lat['re']:.2f} ± {abs(lat['im']):.2f}j" if lat else "–"
            res = max(e["residual"].values())
            L.append(f"| {r['series']} | {r['sweep']} | {e['branch']} | {e['beta_deg']:.2f} | {e['r_degps']:.2f} | "
                     f"{e['ay']:.2f} | {e['linearisation']['stability']} | {e['linearisation']['max_real']:.3f} | "
                     f"{lm} | {lat['damping_ratio']:.2f} | {res:.1e} |" if lat else
                     f"| {r['series']} | {r['sweep']} | {e['branch']} | {e['beta_deg']:.2f} | {e['r_degps']:.2f} | "
                     f"{e['ay']:.2f} | {e['linearisation']['stability']} | {e['linearisation']['max_real']:.3f} | – | – | "
                     f"{res:.1e} |")
    L += ["", "## Measures and limits", "",
          "- Equilibrium: all body-state derivatives zero (vx held by the drive torque); residual tolerances in",
          "  `handling_analysis/equilibrium.py` (`RESIDUAL_TOL`). Navigation states are excluded.",
          "- Stability from the eigenvalues of the central-difference Jacobian; the step is chosen from a step",
          "  study (`JACOBIAN_STEPS`). |Re| below `MARGINAL_TOL` = marginal. The slowest mode is usually the",
          "  speed mode (drag only).",
          "- Local statements hold near the equilibrium only; the extent of the stable region is the subject of",
          "  the `stability_map` experiment type.",
          "- `search_all` reports the equilibria found from a seed grid, not a proof that no further ones exist.",
          "  Solutions with |kappa| > 0.2 are dropped (wheel spin-up held by the speed-holding torque).", ""]
    return "\n".join(L)


__all__ = ("KIND", "SWEEPABLE", "EquilibriumSpec", "analyse", "build_model", "checks", "describe", "operating_point",
           "parse", "run")
