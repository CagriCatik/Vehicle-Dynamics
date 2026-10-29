"""Command line of the scenario book.

    python -m vehicle_model.scenario_book list [--category C]
    python -m vehicle_model.scenario_book info NAME
    python -m vehicle_model.scenario_book run NAME [NAME ...] [--category C] [--set key=value ...] [--no-gif] [--out DIR]
    python -m vehicle_model.scenario_book book [--write PATH]

Exit codes: 0 success, 1 unknown scenario or invalid parameter, 2 a scenario failed.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
import traceback
from pathlib import Path

import yaml

from vehicle_model.scenario_book.catalog import CATEGORIES, book_markdown, catalog, run_entry


def _parse_sets(items: list[str]) -> dict:
    out = {}
    for item in items:
        if "=" not in item:
            raise ValueError(f"--set expects key=value, got {item!r}")
        k, v = item.split("=", 1)
        out[k.strip()] = yaml.safe_load(v)
    return out


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="python -m vehicle_model.scenario_book", description="Run catalogued driving scenarios.")
    sub = ap.add_subparsers(dest="cmd", required=True)
    p_list = sub.add_parser("list", help="list scenarios")
    p_list.add_argument("--category", choices=CATEGORIES)
    p_info = sub.add_parser("info", help="describe one scenario")
    p_info.add_argument("name")
    p_run = sub.add_parser("run", help="run scenarios")
    p_run.add_argument("names", nargs="*")
    p_run.add_argument("--category", choices=CATEGORIES, help="run every scenario of a category")
    p_run.add_argument("--set", dest="sets", action="append", default=[], metavar="KEY=VALUE")
    p_run.add_argument("--no-gif", action="store_true", help="skip animations (much faster)")
    p_run.add_argument("--out", help="output directory (one scenario) or parent directory (several)")
    p_book = sub.add_parser("book", help="print the scenario book as Markdown")
    p_book.add_argument("--write", metavar="PATH")
    args = ap.parse_args(argv)
    cat = catalog()

    if args.cmd == "list":
        for c in CATEGORIES if args.category is None else (args.category,):
            print(f"\n[{c}]")
            for e in cat.values():
                if e.category == c:
                    print(f"  {e.name:32s} {e.title}")
        return 0
    if args.cmd == "info":
        e = cat.get(args.name)
        if e is None:
            print(f"unknown scenario {args.name!r}; see 'list'", file=sys.stderr)
            return 1
        print(f"{e.name}  [{e.category}]\n{e.title}\n\n{e.description}\n\nparameters: "
              f"{json.dumps(e.params) if e.params else 'none'}\nruntime:    {e.runtime}\noutputs:    {e.outputs}")
        return 0
    if args.cmd == "book":
        md = book_markdown()
        if args.write:
            Path(args.write).write_text(md + "\n", encoding="utf-8")
            print(f"written {args.write}")
        else:
            print(md)
        return 0

    names = list(args.names) + ([e.name for e in cat.values() if e.category == args.category] if args.category else [])
    if not names:
        print("nothing to run: give scenario names or --category", file=sys.stderr)
        return 1
    try:
        overrides = _parse_sets(args.sets)
        for n in names:
            if n not in cat:
                raise KeyError(f"unknown scenario {n!r}; see 'list'")
            cat[n].resolve(overrides)
    except (KeyError, TypeError, ValueError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    failed = []
    for n in names:
        out = None if args.out is None else (Path(args.out) if len(names) == 1 else Path(args.out) / n)
        t0 = time.perf_counter()
        print(f"[run] {n} ...", flush=True)
        try:
            run_entry(n, out, overrides, gif=not args.no_gif)
            print(f"[ok]  {n} in {time.perf_counter() - t0:.0f} s -> {out or Path('output/scenario_book') / n}")
        except Exception:  # report and continue with the next scenario
            traceback.print_exc()
            failed.append(n)
            print(f"[fail] {n}")
    return 2 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
