"""Experiment runs as separate processes.

A job is exactly the CLI command ``python -m vehicle_model.experiments run <config> --out <run dir>``
in its own process: the simulation never runs in the web server's event loop, a long run does
not block the interface, and cancelling terminates the process (its whole process group).
Progress comes from the ``[progress] done/total label`` lines the experiment types print
(:func:`vehicle_model.experiments.types.report_progress`); types without them show elapsed time
and the log only.

Every job is persisted in ``<run dir>/job.json`` (command line, configuration copy, status, return
code, times), so the list survives a restart of the workbench; a job whose process is gone
without a recorded end is reported as ``lost``.
"""

from __future__ import annotations

import json
import os
import re
import shutil
import signal
import subprocess
import sys
import threading
import time
from datetime import datetime, timezone
from pathlib import Path

PROGRESS = re.compile(r"^\[progress\] (\d+)/(\d+) ?(.*)$")
ACTIVE = ("running",)


class JobManager:
    def __init__(self, workbench):
        self.wb = workbench
        self._procs: dict[str, subprocess.Popen] = {}
        self._lock = threading.Lock()

    def start(self, config_path: str) -> dict:
        from .service import WorkbenchError
        cfg = self.wb._inside(config_path, self.wb.config_roots)
        v = self.wb.validate(_load(cfg))
        if not v["valid"]:
            raise WorkbenchError(v["error"])
        jid = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S") + "_" + re.sub(r"\W+", "_", cfg.stem)[:40]
        run = self.wb.runs / jid
        k = 1
        while run.exists():
            run = self.wb.runs / f"{jid}_{k}"
            k += 1
        run.mkdir(parents=True)
        shutil.copy(cfg, run / "config.yaml")
        rel_cfg, rel_out = self.wb._rel(cfg), self.wb._rel(run)
        argv = [sys.executable, "-m", "vehicle_model.experiments", "run", rel_cfg, "--out", rel_out]
        job = {"id": run.name, "config": rel_cfg, "out": rel_out, "kind": v["kind"],
               "command_line": f"python -m vehicle_model.experiments run {rel_cfg} --out {rel_out}",
               "status": "running", "started": time.time(), "ended": None, "returncode": None}
        _save(run, job)
        log = open(run / "log.txt", "w", encoding="utf-8")
        env = {**os.environ, "PYTHONUNBUFFERED": "1", "MPLBACKEND": "Agg"}
        src = str(self.wb.repo / "src")
        env["PYTHONPATH"] = src + (os.pathsep + env["PYTHONPATH"] if env.get("PYTHONPATH") else "")
        proc = subprocess.Popen(argv, cwd=self.wb.repo, stdout=log, stderr=subprocess.STDOUT, env=env,
                                start_new_session=True)
        with self._lock:
            self._procs[run.name] = proc
        threading.Thread(target=self._watch, args=(run, proc, log), daemon=True).start()
        return self.status(run.name)

    def _watch(self, run: Path, proc, log) -> None:
        rc = proc.wait()
        log.close()
        job = _read(run)
        if job["status"] == "running":
            job["status"] = "done" if rc == 0 else "failed"
        job.update({"returncode": rc, "ended": time.time()})
        _save(run, job)
        with self._lock:
            self._procs.pop(run.name, None)

    def cancel(self, jid: str) -> dict:
        run = self._run(jid)
        with self._lock:
            proc = self._procs.get(jid)
        if proc is None or proc.poll() is not None:
            return self.status(jid)
        job = _read(run)
        job["status"] = "cancelled"
        _save(run, job)
        try:
            os.killpg(proc.pid, signal.SIGTERM)       # the run and its worker processes
        except (ProcessLookupError, PermissionError, AttributeError):
            proc.terminate()
        return self.status(jid)

    def status(self, jid: str) -> dict:
        run = self._run(jid)
        job = _read(run)
        with self._lock:
            alive = jid in self._procs
        if job["status"] == "running" and not alive:
            job["status"] = "lost"                      # no process and no recorded end (workbench restarted)
        log = (run / "log.txt").read_text(encoding="utf-8", errors="replace") if (run / "log.txt").exists() else ""
        prog = None
        for line in log.splitlines():
            m = PROGRESS.match(line.strip())
            if m:
                prog = {"done": int(m[1]), "total": int(m[2]), "label": m[3]}
        end = job["ended"] or time.time()
        return {**job, "elapsed_s": round(end - job["started"], 1), "progress": prog,
                "log_tail": "\n".join(l for l in log.splitlines() if not l.startswith("[progress]"))[-4000:],
                "has_results": (run / "results.json").is_file()}

    def list(self) -> list:
        out = []
        for d in sorted(self.wb.runs.iterdir(), reverse=True):
            if (d / "job.json").is_file():
                out.append(self.status(d.name))
        return out

    def wait(self, jid: str, timeout: float = 600.0) -> dict:
        t0 = time.time()
        while time.time() - t0 < timeout:
            s = self.status(jid)
            if s["status"] not in ACTIVE:
                return s
            time.sleep(0.2)
        raise TimeoutError(jid)

    def _run(self, jid: str) -> Path:
        from .service import WorkbenchError
        if not re.fullmatch(r"[\w.-]+", jid or ""):
            raise WorkbenchError(f"invalid job id {jid!r}")
        run = self.wb.runs / jid
        if not (run / "job.json").is_file():
            raise WorkbenchError(f"unknown job {jid}")
        return run


def _load(p: Path) -> dict:
    import yaml
    return yaml.safe_load(p.read_text(encoding="utf-8")) or {}


def _read(run: Path) -> dict:
    return json.loads((run / "job.json").read_text(encoding="utf-8"))


def _save(run: Path, job: dict) -> None:
    tmp = run / "job.json.tmp"
    tmp.write_text(json.dumps(job, indent=2), encoding="utf-8")
    tmp.replace(run / "job.json")


__all__ = ("JobManager",)
