"""Comprehensive Split-Mu Benchmark Suite & Dynamics Analysis Module.

Provides automated test environments for evaluating vehicle stability, tractive
efficiency, and active chassis control under severe lateral/longitudinal friction asymmetry:

1. Scenario 1: Split-μ Straight-Line Emergency Braking (100 km/h ECE-R13H / ISO).
2. Scenario 2: Split-μ Full-Throttle Launch / Acceleration (TCS & Torque Vectoring).
3. Scenario 3: Split-μ Braking in a Turn (ISO 7975 Combined Slip & Roll Load Transfer).
4. Scenario 4: Diagonal Split-μ Bridge Transition (Staggered Wheel Entry & Driver Workload).
5. Scenario 5: Split-μ Evasive Double Lane Change (ISO 3888-2 Obstacle Avoidance across Friction Drop).

References:
- ISO 7975:2019: Passenger cars — Braking in a turn — Open-loop test method.
- ISO 3888-2:2011: Passenger cars — Test track for a severe lane-change manoeuvre — Obstacle avoidance.
- ECE-R13H: Uniform provisions concerning the approval of passenger cars with regard to braking.
- Küçükay, F. (2022): Grundlagen der Fahrzeugtechnik, Springer Vieweg, Kap. 6 & 7.
- Mitschke, M.; Wallentowitz, H. (2014): Dynamik der Kraftfahrzeuge, Springer Vieweg, Kap. 14.

Compatibility facade: the implementation lives in the package ``split_mu/`` (one submodule per responsibility); this module re-exports its public names unchanged.
"""

from __future__ import annotations

from .split_mu.animation import (  # noqa: F401
    animate_split_mu_friction_comparison,
    animate_split_mu_scenario,
)
from .split_mu.braking import (  # noqa: F401
    simulate_split_mu_braking,
    simulate_split_mu_braking_in_turn,
)
from .split_mu.friction_sweep import (  # noqa: F401
    generate_split_mu_friction_report_md,
    simulate_split_mu_friction_sweep,
)
from .split_mu.launch import (  # noqa: F401
    simulate_split_mu_launch,
)
from .split_mu.plots import (  # noqa: F401
    _draw_split_mu_road_corridor,
    plot_split_mu_friction_sweep,
    plot_split_mu_scenario_dashboard,
)
from .split_mu.results import (  # noqa: F401
    SplitMuFrictionSweepPoint,
    SplitMuFrictionSweepSummary,
    SplitMuMetrics,
    SplitMuResult,
)
from .split_mu.suite import (  # noqa: F401
    run_all_split_mu_scenarios,
)
from .split_mu.transitions import (  # noqa: F401
    simulate_split_mu_diagonal_transition,
    simulate_split_mu_evasive_dlc,
)

__all__ = (
    "SplitMuMetrics",
    "SplitMuResult",
    "SplitMuFrictionSweepPoint",
    "SplitMuFrictionSweepSummary",
    "simulate_split_mu_braking",
    "simulate_split_mu_launch",
    "simulate_split_mu_braking_in_turn",
    "simulate_split_mu_diagonal_transition",
    "simulate_split_mu_evasive_dlc",
    "simulate_split_mu_friction_sweep",
    "plot_split_mu_scenario_dashboard",
    "plot_split_mu_friction_sweep",
    "animate_split_mu_scenario",
    "animate_split_mu_friction_comparison",
    "generate_split_mu_friction_report_md",
    "run_all_split_mu_scenarios",
)
