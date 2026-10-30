"""Initiation from straight driving, hold and exit, as a sweep with success rates (S04, S05).

Run:  python examples/drift_initiation_demo.py [configs/drift/initiation.yaml]
Out:  output/drift/initiation/ (KPIs per run, mode log, state histories, sequence plot, report)
"""

from vehicle_model.simulation.drift.scenarios import main

if __name__ == "__main__":
    raise SystemExit(main())
