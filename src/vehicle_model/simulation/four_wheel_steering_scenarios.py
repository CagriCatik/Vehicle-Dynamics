"""Four-Wheel Steering (4WS) Multi-Scenario Dynamics & Benchmark Suite.

Simulates, validates, and visualizes 6 comprehensive scenarios:
1. Low-Speed Turning Circle (ISO 7401 / SAE J266, 12 km/h) - Counter-phase agility.
2. Low-Speed 90° Urban Cornering & Curb Clearance (15 km/h) - Virtual wheelbase shortening.
3. High-Speed Evasive Double Lane Change (ISO 3888-2, 80 km/h) - Zero-sideslip crab translation.
4. High-Speed Slalom Maneuver (70 km/h, 30m pylons) - Phase-lag elimination.
5. High-Speed Autobahn Lane Change (130 km/h) - High-speed parallel crabbing.
6. Transient Step Steer Response (ISO 7401, 100 km/h) - Yaw damping & lateral latency.

Renders physical 2D vehicle bodies (draw_vehicle) with steerable front and rear wheels,
and comprehensive telemetry overlays (speed, front steer, rear steer, sideslip beta, yaw rate).

Conforms to:
- Küçükay, F. (2022): Grundlagen der Fahrzeugtechnik, Springer Vieweg.
- Heißing, B.; Ersoy, M.; Gies, M. (2020): Fahrwerkhandbuch, Springer Vieweg.
- Rajamani, R. (2012): Vehicle Dynamics and Control, Springer.

Compatibility facade: the implementation lives in the package ``four_wheel_steering/`` (one submodule per responsibility); this module re-exports its public names unchanged.
"""

from __future__ import annotations

from .four_wheel_steering.animation import (  # noqa: F401
    animate_4ws_detailed_vehicle,
)
from .four_wheel_steering.braking import (  # noqa: F401
    simulate_split_mu_braking_4ws,
)
from .four_wheel_steering.high_speed import (  # noqa: F401
    simulate_autobahn_lane_change,
    simulate_high_speed_double_lane_change,
    simulate_high_speed_slalom,
    simulate_high_speed_step_steer,
    simulate_sine_with_dwell,
)
from .four_wheel_steering.low_speed import (  # noqa: F401
    simulate_low_speed_turning_circle,
    simulate_parallel_parking,
    simulate_urban_90deg_cornering,
)
from .four_wheel_steering.plots import (  # noqa: F401
    plot_4ws_nine_scenario_dashboard,
    plot_4ws_six_scenario_dashboard,
)
from .four_wheel_steering.results import (  # noqa: F401
    ScenarioMetrics,
    ScenarioRunResult,
)

__all__ = (
    "ScenarioMetrics",
    "ScenarioRunResult",
    "simulate_low_speed_turning_circle",
    "simulate_urban_90deg_cornering",
    "simulate_parallel_parking",
    "simulate_high_speed_double_lane_change",
    "simulate_high_speed_slalom",
    "simulate_autobahn_lane_change",
    "simulate_high_speed_step_steer",
    "simulate_split_mu_braking_4ws",
    "simulate_sine_with_dwell",
    "plot_4ws_six_scenario_dashboard",
    "plot_4ws_nine_scenario_dashboard",
    "animate_4ws_detailed_vehicle",
)
