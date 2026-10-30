"""Automated Scenario Matrix Runner for Virtual Automotive Validation.

Executes a comprehensive matrix of standardized test maneuvers:
- ISO 3888-2 Double Lane Change
- Split-mu Emergency Braking
- FMVSS 126 Sine with Dwell ESC Yaw Stability
- ISO 7401 Step Steer Transient Response
- ISO 2631 Ride Comfort & Road Holding Bump Traversalmd vertical dynamics).

Compatibility facade: the implementation lives in the package ``validation/`` (base class plus scenario mixins by topic); this module re-exports it unchanged.
"""

from __future__ import annotations

from .validation.runner import ValidationMatrixRunner  # noqa: F401
