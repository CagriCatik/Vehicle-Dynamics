"""Bounded control allocation with feasibility feedback (Plan.md, Abschnitt 10; M5)."""

from .wheel_forces import (
    STATUS_INFEASIBLE,
    STATUS_NOMINAL,
    STATUS_SATURATED,
    ActuatorHealth,
    AllocationResult,
    AllocatorConfig,
    WheelForceAllocator,
)
from .wls import WLSResult, solve_box_wls

__all__ = (
    "ActuatorHealth",
    "AllocationResult",
    "AllocatorConfig",
    "STATUS_INFEASIBLE",
    "STATUS_NOMINAL",
    "STATUS_SATURATED",
    "WLSResult",
    "WheelForceAllocator",
    "solve_box_wls",
)
