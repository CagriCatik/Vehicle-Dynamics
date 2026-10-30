"""Drift plan D6: animated, fully exported drift runs (circle hold vs open loop, full sequence, figure-eight).

Run:  python examples/drift_figure_eight_demo.py [configs/drift/animation.yaml]
Out:  output/drift/runs/<scenario>/<variant>/ (CSV, JSON, GIF, PNG, report) and circle_hold/comparison.gif
"""

from vehicle_model.simulation.drift.showcase import main

if __name__ == "__main__":
    raise SystemExit(main())
