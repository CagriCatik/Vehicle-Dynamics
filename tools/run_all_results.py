"""Regenerate every result in output/: all experiments, then all example scripts, figures and animations.

    python tools/run_all_results.py                   # everything
    python tools/run_all_results.py --only hill        # items whose name contains "hill"

Experiments run one after the other (each parallelises internally); example scripts run in a small pool. A failing
item does not stop the run. Log: output/run_all_log.csv (stage, item, status, seconds, last output line).
"""

from __future__ import annotations

import argparse
import csv
import os
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
LOG = ROOT / "output" / "run_all_log.csv"
TIMEOUT_S = 3600
# run after the experiments they read
LATE = ("actuation_trace", "brake_functions_figures", "traction_figures", "roll_stability_steering_figures", "g_vectoring_figures",
        "ride_control_figures", "hill_descent_figures")
LAST = ("chassis_control_animations", "stage_animations")     # in this order, after all results exist
ARGS = {"nmpc_path_tracking_simulation": ["--headless"]}


def run(stage: str, name: str, cmd: list) -> dict:
    t0 = time.time()
    env = dict(os.environ, MPLBACKEND="Agg", PYTHONPATH=str(ROOT / "src"))
    try:
        p = subprocess.run(cmd, cwd=ROOT, env=env, capture_output=True, text=True, timeout=TIMEOUT_S)
        lines = [ln for ln in (p.stdout + p.stderr).splitlines() if ln.strip() and "findfont" not in ln]
        status, last = ("ok" if p.returncode == 0 else f"exit {p.returncode}"), (lines[-1] if lines else "")
    except subprocess.TimeoutExpired:
        status, last = "timeout", ""
    r = {"stage": stage, "item": name, "status": status, "seconds": round(time.time() - t0, 1), "last_line": last[:200]}
    print(f"[{stage}] {name}: {status} ({r['seconds']} s)", flush=True)
    rows = [q for q in csv.DictReader(open(LOG, newline="", encoding="utf-8")) if q["item"] != name]
    with open(LOG, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(r))
        w.writeheader()
        w.writerows([*rows, r])
    return r


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--only", default="")
    ap.add_argument("--workers", type=int, default=4)
    args = ap.parse_args()
    py = sys.executable
    LOG.parent.mkdir(exist_ok=True)
    if not args.only or not LOG.exists():
        with open(LOG, "w", newline="", encoding="utf-8") as f:
            csv.writer(f).writerow(["stage", "item", "status", "seconds", "last_line"])
    keep = lambda n: args.only in n
    configs = [p for p in sorted((ROOT / "configs/experiments").glob("*.yaml")) if keep(p.stem)]
    results = [run("experiment", p.stem, [py, "-m", "vehicle_model.experiments", "run", str(p.relative_to(ROOT))]) for p in configs]
    scripts = [p for p in sorted((ROOT / "examples").glob("*.py")) if keep(p.stem)]
    demos = [p for p in scripts if p.stem not in LATE and p.stem not in LAST]
    with ThreadPoolExecutor(args.workers) as ex:
        results += list(ex.map(lambda p: run("example", p.stem, [py, str(p.relative_to(ROOT)), *ARGS.get(p.stem, [])]), demos))
    for p in scripts:
        if p.stem in LATE:
            results.append(run("figure", p.stem, [py, str(p.relative_to(ROOT))]))
    for name in LAST:
        for p in scripts:
            if p.stem == name:
                results.append(run("animation", p.stem, [py, str(p.relative_to(ROOT))]))
    bad = [r for r in results if r["status"] != "ok"]
    print(f"{len(results) - len(bad)} of {len(results)} ok; failed: {[r['item'] for r in bad]}")
    return 1 if bad else 0


if __name__ == "__main__":
    raise SystemExit(main())
