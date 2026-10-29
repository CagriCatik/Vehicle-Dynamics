"""Drift plan D5: circle tracking with a radial outer loop, set-point transition and figure-eight (S06, S07).

Run:  python examples/drift_circle_demo.py [configs/drift/circle_hold.yaml]
Out:  output/drift/circle_tracking/
"""

from vehicle_model.simulation.drift.tracking import main

if __name__ == "__main__":
    raise SystemExit(main())
