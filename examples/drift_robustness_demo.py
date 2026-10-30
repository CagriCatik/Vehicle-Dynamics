"""Robustness of the drift hold against plant, tyre, actuator, sensor and road variations.

Run:  python examples/drift_robustness_demo.py [configs/drift/robustness.yaml]
Out:  output/drift/robustness/ (kpi.json, report.md, resolved config, metadata)
"""

from vehicle_model.simulation.drift.robustness import main

if __name__ == "__main__":
    raise SystemExit(main())
