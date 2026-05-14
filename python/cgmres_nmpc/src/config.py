"""
Central configuration loader for the CGMRES-NMPC simulation.

Reads config.yaml once at import time and exposes the values as
module-level constants so every other module can do a simple:

    from src.config import WB, Q_Y, ...

rather than parsing YAML themselves.  All physical units are SI
(metres, radians, seconds) unless the constant name ends in _DEG.
"""

import math
from pathlib import Path

import yaml

PROJECT_ROOT = Path(__file__).resolve().parents[1]


def load_config(config_path=None):
    """Read and parse config.yaml relative to the NMPC package root."""
    path = Path(config_path) if config_path is not None else PROJECT_ROOT / "config.yaml"
    if not path.is_absolute():
        path = PROJECT_ROOT / path
    with path.open("r", encoding="utf-8") as f:
        config = yaml.safe_load(f)
    return config


# Loaded once; every import of this module shares the same dict.
global_config = load_config()

# ── Vehicle physical parameters ────────────────────────────────────────
WB            = global_config["vehicle"]["wheelbase"]          # distance between axles [m]
U_A_MAX       = global_config["vehicle"]["max_accel"]          # max longitudinal accel [m/s²]
U_STEER_MAX   = math.radians(global_config["vehicle"]["max_steer_front_deg"])  # max front steer [rad]
VEHICLE_RADIUS = global_config["vehicle"]["radius"]            # collision-check radius [m]

# ── MPC terminal/regularisation penalty weights ────────────────────────
# phi_* weights penalise constraint violations at the end of the horizon.
PHI_V     = global_config["controller"]["phi_v"]
PHI_STEER = global_config["controller"]["phi_steer"]
PHI_OBS   = global_config["controller"].get("phi_obs", 0.01)  # obstacle barrier weight

# ── Stage cost weights (used inside the MPC cost rollout) ──────────────
# Higher Q → tighter tracking of that state variable.
Q_X   = global_config["controller"]["q_x"]    # longitudinal position (unused, vehicle drives forward)
Q_Y   = global_config["controller"]["q_y"]    # lateral error — primary tracking objective
Q_YAW = global_config["controller"]["q_yaw"]  # heading angle error
Q_V   = global_config["controller"]["q_v"]    # speed error

# ── File paths ─────────────────────────────────────────────────────────
LUT_PATH = str(PROJECT_ROOT / global_config["paths"]["lut_json"])  # rear-axle steering lookup table

# ── Reference trajectory shape ─────────────────────────────────────────
TRAJ_V = global_config["trajectory"]["target_v"]   # desired longitudinal speed [m/s]
TRAJ_A = global_config["trajectory"]["amplitude"]  # sinusoidal path amplitude [m]
TRAJ_F = global_config["trajectory"]["frequency"]  # sinusoidal path frequency [Hz]

# ── Static obstacles (list of [x, y, radius] dicts) ───────────────────
# Empty list means no obstacles; the MPC barrier term is simply skipped.
OBSTACLES = global_config.get("obstacles", [])
