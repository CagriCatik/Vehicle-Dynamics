"""Command line: ``python -m vehicle_model.experiments {run,validate,list} ...``."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from .runner import run_experiment
from .spec import load_experiment
from .types import get_type


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="python -m vehicle_model.experiments",
                                 description="Run YAML-defined vehicle dynamics experiments.")
    sub = ap.add_subparsers(dest="cmd", required=True)
    run = sub.add_parser("run", help="run an experiment and write results and report")
    run.add_argument("spec", type=Path)
    run.add_argument("--out", type=Path, default=None, help="output directory (default: from the specification)")
    run.add_argument("--no-verification", action="store_true", help="skip the model verification checks")
    val = sub.add_parser("validate", help="check a specification without running it")
    val.add_argument("spec", type=Path, nargs="+")
    lst = sub.add_parser("list", help="list the specifications in a directory")
    lst.add_argument("directory", type=Path, nargs="?", default=Path("configs/experiments"))
    args = ap.parse_args(argv)

    if args.cmd == "list":
        for p in sorted(args.directory.glob("*.yaml")):
            spec = load_experiment(p)
            print(f"{p}: [{spec.kind}] {get_type(spec.kind).describe(spec)}")
        return 0
    if args.cmd == "validate":
        failed = 0
        for p in args.spec:
            try:
                load_experiment(p)
                print(f"{p}: valid")
            except (ValueError, OSError) as exc:
                failed += 1
                print(f"{p}: INVALID: {exc}")
        return 1 if failed else 0

    spec = load_experiment(args.spec)
    summary = run_experiment(spec, args.out, verify=False if args.no_verification else None)
    out = args.out or Path(spec.outputs["directory"])
    failed = [c["name"] for c in summary["verification"] if not c["passed"]]
    print(f"{spec.name}: {len(summary['results'])} run(s) written to {out}")
    if failed:
        print("verification FAILED: " + ", ".join(failed))
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
