"""Stationary drift equilibria over speed x sideslip x friction (both directions).

Run:  python examples/drift_equilibria_demo.py [configs/drift/equilibrium_sweep.yaml]
Out:  output/drift/equilibrium_sweep/ (CSV, JSON, PNG, report, resolved config, metadata)
"""

from vehicle_model.simulation.drift.equilibrium_sweep import main

if __name__ == "__main__":
    raise SystemExit(main())
