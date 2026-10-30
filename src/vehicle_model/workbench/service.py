"""Application layer of the workbench (Plan V3.0 M15): everything the HTTP adapter needs, testable without it.

No second configuration logic: options come from the experiment specification module
(``experiments.spec``: laws, inputs, feedback modes, roads, manoeuvres, formats), every
configuration is validated by the experiment type's own ``parse`` and saved as the YAML the
CLI reads. Runs are the CLI itself in a separate process (:mod:`.jobs`), so a run started here is
reproduced exactly by ``python -m vehicle_model.experiments run <yaml> --out <dir>``.

Directories (relative to the repository root, the working directory):

    output/workbench/configs/<name>.yaml      configurations saved from the workbench
    output/workbench/runs/<job id>/           one run: job.json, log.txt, config.yaml, the experiment outputs
    output/workbench/exports/<id>.zip         exported technical reports

Results of CLI runs under ``output/experiments/`` are listed as well (read only).
"""

from __future__ import annotations

import csv
import io
import json
import math
import re
import shutil
import zipfile
from datetime import datetime, timezone
from pathlib import Path

import yaml

FIGURES = (".png", ".gif")
TABLES = (".csv",)
TEXT = (".md", ".yaml", ".json", ".txt")
MAX_POINTS = 1500


class WorkbenchError(ValueError):
    """A request the workbench refuses (invalid configuration, unknown or forbidden path)."""


class Workbench:
    def __init__(self, root: str | Path = "output/workbench", repo: str | Path = "."):
        self.repo = Path(repo).resolve()
        self.root = (self.repo / root).resolve()
        self.configs = self.root / "configs"
        self.runs = self.root / "runs"
        self.exports = self.root / "exports"
        for d in (self.configs, self.runs, self.exports):
            d.mkdir(parents=True, exist_ok=True)
        self.result_roots = (self.runs, (self.repo / "output" / "experiments").resolve())
        self.config_roots = (self.configs, (self.repo / "configs" / "experiments").resolve(),
                             (self.repo / "configs" / "critical").resolve())

    # ------------------------------------------------------------------ options (from the spec module)
    def options(self) -> dict:
        from vehicle_model.experiments import spec as S
        from vehicle_model.experiments.types import experiment_types
        from vehicle_model.tire_model.factory import available_tyres
        rel = lambda p: p.relative_to(self.repo).as_posix()
        return {
            "experiment_types": list(experiment_types()),
            "vehicles": [rel(p) for p in sorted((self.repo / "configs" / "vehicles").glob("*.yaml"))],
            "tyres": available_tyres(),
            "kc": [rel(p) for p in sorted((self.repo / "configs" / "suspension").glob("kc_*.yaml"))],
            "laws": list(S.LAWS), "inputs": list(S.INPUT_SETS), "feedback": list(S.FEEDBACK_MODES),
            "roads": {k: sorted(v) for k, v in S.ROADS.items()},
            "manoeuvres": {k: sorted(v) for k, v in S.MANOEUVRES.items()},
            "sweep_parameters": [f"manoeuvre.{k}" for k in sorted(set().union(*S.MANOEUVRES.values()))]
                                + [f"road.{k}" for k in sorted(set().union(*S.ROADS.values()))],
            "formats": list(S.FORMATS), "default_formats": list(S.DEFAULT_FORMATS),
        }

    # ------------------------------------------------------------------ configurations
    def validate(self, data: dict) -> dict:
        from vehicle_model.experiments.spec import parse_experiment
        from vehicle_model.experiments.types import get_type
        if not isinstance(data, dict):
            return {"valid": False, "error": "the configuration must be a mapping"}
        try:
            spec = parse_experiment(copy_plain(data), "<workbench>")
        except (ValueError, OSError, TypeError, KeyError) as ex:
            return {"valid": False, "error": str(ex)}
        return {"valid": True, "kind": spec.kind, "description": get_type(spec.kind).describe(spec),
                "yaml": yaml.safe_dump(data, sort_keys=False)}

    def save_config(self, data: dict) -> dict:
        v = self.validate(data)
        if not v["valid"]:
            raise WorkbenchError(v["error"])
        name = _slug(str(data.get("name", "experiment")))
        path = self.configs / f"{name}.yaml"
        path.write_text(f"# saved by the workbench {now()}\n" + yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
        return {"path": self._rel(path), **v}

    def parse_text(self, text: str) -> dict:
        try:
            data = yaml.safe_load(text) or {}
        except yaml.YAMLError as ex:
            raise WorkbenchError(f"YAML syntax: {ex}") from None
        if not isinstance(data, dict):
            raise WorkbenchError("the YAML document must be a mapping")
        return data

    def validate_text(self, text: str) -> dict:
        try:
            return self.validate(self.parse_text(text))
        except WorkbenchError as ex:
            return {"valid": False, "error": str(ex)}

    def save_text(self, text: str) -> dict:
        return self.save_config(self.parse_text(text))

    def list_configs(self) -> list:
        from vehicle_model.experiments.spec import load_experiment
        from vehicle_model.experiments.types import get_type
        out = []
        for root in self.config_roots:
            for p in sorted(root.glob("*.yaml")):
                try:
                    spec = load_experiment(p)
                    out.append({"path": self._rel(p), "name": spec.name, "kind": spec.kind,
                                "description": get_type(spec.kind).describe(spec), "saved": root == self.configs})
                except (ValueError, OSError) as ex:
                    out.append({"path": self._rel(p), "name": p.stem, "kind": "?", "description": f"invalid: {ex}",
                                "saved": root == self.configs})
        return out

    def read_config(self, path: str) -> dict:
        p = self._inside(path, self.config_roots + self.result_roots)
        text = p.read_text(encoding="utf-8")
        return {"path": self._rel(p), "yaml": text, "data": yaml.safe_load(text) or {}}

    # ------------------------------------------------------------------ results
    def list_results(self) -> list:
        out = []
        for root in self.result_roots:
            if not root.is_dir():
                continue
            for d in sorted(root.iterdir()):
                f = d / "results.json"
                if not f.is_file():
                    continue
                try:
                    s = json.loads(f.read_text(encoding="utf-8"))
                except (OSError, json.JSONDecodeError):
                    continue
                exp = s.get("experiment", {})
                out.append({"id": self._rel(d), "name": exp.get("name", d.name), "kind": exp.get("type", "controller_comparison"),
                            "created": s.get("provenance", {}).get("created_utc", ""),
                            "commit": s.get("provenance", {}).get("git_commit", ""),
                            "source": "workbench" if root == self.runs else "cli"})
        return sorted(out, key=lambda r: r["created"], reverse=True)

    def result(self, rid: str) -> dict:
        d = self._inside(rid, self.result_roots)
        s = json.loads((d / "results.json").read_text(encoding="utf-8"))
        files = sorted(self._rel(p) for p in d.rglob("*") if p.is_file())
        return {"id": self._rel(d), "experiment": s.get("experiment", {}), "provenance": s.get("provenance", {}),
                "verification": s.get("verification", []), "kpis": kpi_rows(s), "files": files,
                "figures": [f for f in files if f.endswith(FIGURES)],
                "histories": [f for f in files if "/histories/" in f and f.endswith(".csv")],
                "tables": [f for f in files if f.endswith(TABLES) and "/histories/" not in f],
                "report": (d / "report.md").read_text(encoding="utf-8") if (d / "report.md").is_file() else ""}

    def file(self, path: str) -> Path:
        return self._inside(path, self.result_roots + self.config_roots + (self.exports,))

    def history(self, path: str, columns: list | None = None) -> dict:
        p = self._inside(path, self.result_roots)
        with open(p, newline="", encoding="utf-8") as f:
            rows = list(csv.reader(f))
        head, body = rows[0], rows[1:]
        step = max(1, math.ceil(len(body) / MAX_POINTS))
        body = body[::step]
        cols = {}
        for j, h in enumerate(head):
            if columns and h not in columns and h != "t":
                continue
            vals = [r[j] for r in body]
            try:
                cols[h] = [float(x) for x in vals]
            except ValueError:
                cols[h] = vals
        return {"path": self._rel(p), "columns": cols, "stride": step}

    def compare(self, ids: list) -> dict:
        """KPI table across results (one row per result, sweep point and controller) and the shared KPIs."""
        rows, keys = [], None
        for rid in ids:
            r = self.result(rid)
            for k in r["kpis"]:
                rows.append({"result": r["id"], "name": r["experiment"].get("name", ""), **k})
                ks = {x for x, v in k.items() if isinstance(v, (int, float)) and x not in ("sweep_value",)}
                keys = ks if keys is None else keys & ks
        return {"rows": rows, "kpis": sorted(keys or [])}

    def export_report(self, ids: list, title: str = "Workbench comparison") -> dict:
        """Technical report (Markdown with figures, configurations and KPI comparison) as one ZIP file."""
        if not ids:
            raise WorkbenchError("select at least one result")
        cmp_ = self.compare(ids)
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        name = f"report_{stamp}"
        buf = io.BytesIO()
        L = [f"# {title}", "", f"Exported {now()} by the Vehicle-Dynamics workbench. Synthetic parameters, virtual "
             "models only; every run is reproducible with the configuration and command given below.", ""]
        with zipfile.ZipFile(buf, "w", zipfile.ZIP_DEFLATED) as z:
            for rid in ids:
                r = self.result(rid)
                exp, prov = r["experiment"], r["provenance"]
                L += [f"## {exp.get('name', rid)} ({exp.get('type', 'controller_comparison')})", "",
                      f"- result: `{r['id']}`", f"- commit {prov.get('git_commit', '?')}, created {prov.get('created_utc', '?')}",
                      f"- reproduce: `{self.reproduce_command(rid)}`", ""]
                if r["verification"]:
                    ok = sum(bool(c.get("passed")) for c in r["verification"])
                    L += [f"Model verification: {ok}/{len(r['verification'])} checks passed.", ""]
                cfg = yaml.safe_dump(exp, sort_keys=False)
                z.writestr(f"{name}/{_slug(r['id'])}/config.yaml", cfg)
                for fig in r["figures"]:
                    if fig.endswith(".png"):
                        arc = f"{_slug(r['id'])}/{Path(fig).name}"
                        z.write(self.file(fig), f"{name}/{arc}")
                        L += [f"![{Path(fig).stem}]({arc})", ""]
            if cmp_["rows"]:
                ks = [k for k in ("yaw_rate_rms_error", "max_sideslip_deg", "yaw_rate_rms_error_degps",
                                  "lateral_displacement", "max_yaw_moment", "stopping_distance") if k in cmp_["kpis"]] \
                    or cmp_["kpis"][:6]
                L += ["## KPI comparison", "", "| result | point | controller | " + " | ".join(ks) + " |",
                      "|---" * (3 + len(ks)) + "|"]
                for row in cmp_["rows"]:
                    L.append(f"| {row['name']} | {row.get('point', '')} | {row.get('controller', '')} | "
                             + " | ".join(_fmt(row.get(k)) for k in ks) + " |")
            z.writestr(f"{name}/report.md", "\n".join(L) + "\n")
        out = self.exports / f"{name}.zip"
        out.write_bytes(buf.getvalue())
        return {"path": self._rel(out), "markdown": "\n".join(L)}

    def reproduce_command(self, rid: str) -> str:
        d = self._inside(rid, self.result_roots)
        job = d / "job.json"
        if job.is_file():
            return json.loads(job.read_text(encoding="utf-8"))["command_line"]
        src = json.loads((d / "results.json").read_text(encoding="utf-8")).get("source", "<config>")
        return f"python -m vehicle_model.experiments run {src} --out {self._rel(d)}"

    # ------------------------------------------------------------------ paths
    def _rel(self, p: Path) -> str:
        return Path(p).resolve().relative_to(self.repo).as_posix()

    def _inside(self, path: str, roots) -> Path:
        p = (self.repo / str(path)).resolve()
        if not any(p == r or r in p.parents for r in roots):
            raise WorkbenchError(f"path outside the workbench and result directories: {path}")
        if not p.exists():
            raise WorkbenchError(f"not found: {path}")
        return p


def kpi_rows(summary: dict) -> list:
    """Flat KPI rows of a results.json (controller comparison, robust benchmark; empty for other types)."""
    rows = []
    for r in summary.get("results", []):
        k = r.get("kpis")
        if not isinstance(k, dict):
            continue
        row = {"point": r.get("point", r.get("case", "")), "controller": r.get("controller", ""),
               "feedback": r.get("feedback", "")}
        row.update({x: v for x, v in k.items() if isinstance(v, (int, float)) and not isinstance(v, bool)})
        rows.append(row)
    return rows


def copy_plain(d):
    return json.loads(json.dumps(d))


def now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def _slug(s: str) -> str:
    return re.sub(r"[^A-Za-z0-9_.-]+", "_", s).strip("_") or "experiment"


def _fmt(x) -> str:
    return "–" if x is None or (isinstance(x, float) and not math.isfinite(x)) else (f"{x:.4g}" if isinstance(x, float) else str(x))


__all__ = ("Workbench", "WorkbenchError", "kpi_rows")
