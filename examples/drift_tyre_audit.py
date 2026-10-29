"""Drift plan D1: audit of the bundled tyre models over the large-slip range a drift needs.

Run:  python examples/drift_tyre_audit.py
Out:  output/drift/tyre_audit/{tyre_audit.png, tyre_audit.json}
"""

from __future__ import annotations

import json
from pathlib import Path

from vehicle_model.drift import audit_tyre
from vehicle_model.tire_model import load_tyre
from vehicle_model.visualization.drift_plots import plot_tyre_audit

TYRES = ("magic_formula_passenger_synthetic", "simplified_combined_synthetic", "fiala_passenger_synthetic")


def main(out_dir: str = "output/drift/tyre_audit") -> int:
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    audits = [audit_tyre(load_tyre(n), n) for n in TYRES]
    plot_tyre_audit(audits, out / "tyre_audit.png")
    (out / "tyre_audit.json").write_text(json.dumps([a.summary() for a in audits], indent=2), encoding="utf-8")
    for a in audits:
        print(f"{a.name}: {'PASS' if a.passed else 'FAIL'}")
        for c in a.checks:
            flag = {True: "pass", False: "FAIL", None: "info"}[c["passed"]]
            print(f"  [{flag}] {c['name']}: {c['detail']}")
    print(f"written to {out}")
    return 0 if all(a.passed for a in audits) else 1


if __name__ == "__main__":
    raise SystemExit(main())
