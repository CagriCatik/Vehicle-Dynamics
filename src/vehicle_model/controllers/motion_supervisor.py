"""Vehicle motion supervisor: function registry, arbitration record and allocation diagnostics.

A **coordinator, not a second controller**: the control itself stays in the small functions (ABS/TCS, yaw laws,
CBC, HDC, GVC, AFS overlay, regen slip control, health monitor), and the arbitration rules live where the
commands are formed (:func:`~vehicle_model.controllers.motion_control.arbitrate_yaw`,
:func:`~vehicle_model.controllers.motion_control.arbitrate_longitudinal`, the steering overlay that finalises
the front angle once). The supervisor

* declares every function (``FUNCTIONS``: domain, priority class, the one place that owns its final command),
* records per tick which functions are enabled / eligible / active and their request,
* logs transitions, conflicts (a request suppressed by a higher priority) and hand-overs,
* reports allocation diagnostics: residual, active bounds and the condition number of the effectiveness matrix
  over the actuators that are still free (the multi-domain allocation stays separate per domain, motion management).

Priority classes (high to low): ``safety`` (ABS/TCS, RSC, HDC, safe stop) > ``driver`` > ``stability``
(yaw law/ESC, CBC) > ``agility`` (GVC, AFS overlay, torque vectoring) > ``comfort`` (regen blending, damping).
One owner per physical actuator: wheel torques are formed only by the ActuationLayer (allocation + slip control),
the front road-wheel angle only by the steering overlay, active strut forces only by the vertical allocation (14-DOF).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

PRIORITIES = ("safety", "driver", "stability", "agility", "comfort")


@dataclass(frozen=True)
class FunctionSpec:
    name: str
    domain: str          # longitudinal | yaw | wheel_slip | front_steer | rear_steer | vertical | health
    priority: str
    owner: str           # the single place that turns its request into the final command
    model: str           # 7dof | 14dof | both
    module: str          # where the function is implemented (package-relative)


FUNCTIONS = (
    FunctionSpec("abs", "wheel_slip", "safety", "ActuationLayer (slip control after allocation)", "both", "controllers/slip_control.py"),
    FunctionSpec("tcs", "wheel_slip", "safety", "ActuationLayer (slip control after allocation)", "both", "controllers/slip_control.py"),
    FunctionSpec("hdc", "longitudinal", "safety", "arbitrate_longitudinal (braking floor)", "7dof", "controllers/hill_descent.py"),
    FunctionSpec("safe_stop", "longitudinal", "safety", "health monitor -> driver brake demand", "7dof", "controllers/fault_tolerance.py"),
    FunctionSpec("rsc", "longitudinal+yaw", "safety", "WheelForceAllocator (14-DOF loop)", "14dof", "controllers/roll_stability.py"),
    FunctionSpec("driver", "longitudinal", "driver", "arbitrate_longitudinal (base)", "both", "controllers/motion_control.py"),
    FunctionSpec("yaw_law", "yaw", "stability", "arbitrate_yaw -> WheelForceAllocator", "7dof", "controllers/yaw_laws.py"),
    FunctionSpec("cbc", "yaw", "stability", "arbitrate_yaw (larger magnitude, never summed)", "7dof", "controllers/cornering_brake.py"),
    FunctionSpec("ebd", "longitudinal", "stability", "WheelForceAllocator (axle share / brake limits)", "7dof", "controllers/brake_distribution.py"),
    FunctionSpec("afs", "front_steer", "agility", "SteeringOverlay.finalise (once)", "7dof", "controllers/active_front_steering.py"),
    FunctionSpec("rear_steer", "rear_steer", "agility", "yaw law request (4WS inputs)", "7dof", "controllers/yaw_laws.py"),
    FunctionSpec("gvc", "longitudinal", "agility", "arbitrate_longitudinal (additive, yields to floors)", "7dof", "controllers/g_vectoring.py"),
    FunctionSpec("regen", "wheel_slip", "comfort", "BrakeByWireActuator (blending, regen slip control)", "7dof", "controllers/regenerative_slip.py"),
    FunctionSpec("health", "health", "safety", "FaultTolerantController (masks, fallbacks)", "7dof", "controllers/fault_tolerance.py"),
    FunctionSpec("damping", "vertical", "comfort", "CornerDamping (14-DOF)", "14dof", "suspension/preview_control.py"),
    FunctionSpec("active_pitch", "vertical", "comfort", "VerticalForceAllocator (14-DOF)", "14dof", "suspension/preview_control.py"),
)


def _find_motion_controller(ctrl):
    seen = 0
    while ctrl is not None and not hasattr(ctrl, "actuation") and seen < 8:
        ctrl = getattr(ctrl, "inner", None)
        seen += 1
    return ctrl


def function_states(diag: dict, out) -> dict:
    """Activity of each 7-DOF function in one tick, from the MotionController diagnostics."""
    d = diag or {}
    arb = d.get("arbitration") or {}
    la = d.get("longitudinal_arbitration") or {}
    trace = d.get("trace") or {}
    steering = d.get("steering") or {}
    hdc = d.get("hdc") or {}
    gvc = d.get("gvc") or {}
    health = d.get("health") or {}
    st = {
        "abs": bool(np.any(d.get("abs_active", False))),
        # the allocation already caps a wheel at its grip limit (information mode A knows mu): ABS then rarely acts
        "grip_limit": bool(np.any(d.get("grip_limited", False))),
        "tcs": bool(np.any(d.get("tcs_active", False))),
        "yaw_law": arb.get("winner") == "law" and arb.get("law") not in (None, 0.0),
        "cbc": arb.get("winner") == "cbc",
        "hdc": hdc.get("status") in ("active", "infeasible") and hdc.get("ax_cmd", 0.0) < 0.0,
        "gvc": "gvc" in str(la.get("winner", "")),
        "afs": abs(float(steering.get("overlay", 0.0) or 0.0)) > 1e-6,
        "rear_steer": abs(float(getattr(out, "steer_r", 0.0) or 0.0)) > 1e-6,
        "regen": float(trace.get("regen_power_w", 0.0) or 0.0) > 1.0,
        "health": health.get("system", "nominal") != "nominal" if health else False,
    }
    return st


class VehicleMotionSupervisor:
    """Wraps the controller stack; records function activity, transitions, conflicts and allocation diagnostics."""

    def __init__(self, inner):
        self.inner = inner
        self.information = getattr(inner, "information", None)
        self.reset()

    @property
    def name(self) -> str:
        return "supervised " + str(getattr(self.inner, "name", "controller"))

    def receive(self, *args):
        if hasattr(self.inner, "receive"):
            self.inner.receive(*args)

    def reset(self) -> None:
        self.t = 0.0
        self.state = {}
        self.events = []          # (t, function, "on"/"off")
        self.conflicts = []       # (t, domain, winner, suppressed)
        self.active_time = {}
        self.conditions = []
        if hasattr(self.inner, "reset"):
            self.inner.reset()

    def compute_control(self, state, target, telemetry=None, dt: float = 0.01):
        out = self.inner.compute_control(state, target, telemetry, dt)
        diag = dict(out.diagnostics or {})
        st = function_states(diag, out)
        for k, v in st.items():
            if v != self.state.get(k, False):
                self.events.append((round(self.t, 6), k, "on" if v else "off"))
            if v:
                self.active_time[k] = self.active_time.get(k, 0.0) + dt
        self.state = st
        la = diag.get("longitudinal_arbitration") or {}
        if la.get("suppressed"):
            self.conflicts.append((round(self.t, 6), "longitudinal", la.get("winner"), tuple(la["suppressed"])))
        arb = diag.get("arbitration") or {}
        if arb.get("law") not in (None, 0.0) and arb.get("cbc") not in (None, 0.0):
            loser = "cbc" if arb.get("winner") == "law" else "law"
            self.conflicts.append((round(self.t, 6), "yaw", arb.get("winner"), (loser,)))
        cond = self._condition(telemetry)
        if not np.isnan(cond):
            self.conditions.append(cond)
        diag["supervisor"] = {"active": [k for k, v in st.items() if v], "condition": cond,
                              "events": len(self.events), "conflicts": len(self.conflicts)}
        diag["function_status"] = {**(diag.get("function_status") or {}), "supervisor": ",".join(k for k, v in st.items() if v)}
        out.diagnostics = diag
        self.t += dt
        return out

    def _condition(self, telemetry) -> float:
        """Condition number of the (F_x, M_z) effectiveness over the wheels not at a bound (inf if < 2 free)."""
        mc = _find_motion_controller(self.inner)
        alloc = None if mc is None else mc.actuation.last_allocation
        if alloc is None or telemetry is None:
            return float("nan")
        free = ~np.asarray(alloc.at_bound, bool)
        if np.count_nonzero(free) < 2:
            return float("inf")
        B = mc.actuation.allocator.effectiveness(np.asarray(telemetry.steer_angles, float))[:, free]
        sv = np.linalg.svd(B / np.array([[1000.0], [1000.0]]), compute_uv=False)
        return float(sv[0] / max(sv[-1], 1e-12))

    def summary(self) -> dict:
        c = np.asarray(self.conditions, float)
        fin = c[np.isfinite(c)]
        return {"active_time_s": dict(self.active_time), "transitions": len(self.events), "conflicts": len(self.conflicts),
                "conflict_domains": sorted({x[1] for x in self.conflicts}),
                "condition_median": float(np.median(fin)) if fin.size else float("nan"),
                # share of ticks where the free wheels cannot shape F_x and M_z independently (cond > 1e3 or < 2 free)
                "ill_conditioned_share": float(np.mean(~np.isfinite(c) | (c > 1e3))) if c.size else 0.0}


__all__ = ("FUNCTIONS", "FunctionSpec", "PRIORITIES", "VehicleMotionSupervisor", "function_states")
