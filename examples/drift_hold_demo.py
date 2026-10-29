"""Drift plan D3: hold a computed drift equilibrium with LQR and recover from perturbations (S01-S03).

Run:  python examples/drift_hold_demo.py [configs/drift/drift_lqr.yaml]
Out:  output/drift/lqr_hold/ (state histories, KPIs, linearisation, plots, report)
"""

from vehicle_model.simulation.drift.hold import main

if __name__ == "__main__":
    raise SystemExit(main())
