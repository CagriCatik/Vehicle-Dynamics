"""
Entry point for the CGMRES-NMPC vehicle simulation.

Reads configuration from config.yaml, constructs the vehicle plant and
NMPC controller, then launches the interactive matplotlib dashboard.
The dashboard runs the closed-loop simulation in real time.
"""

import math
from src.config import global_config
from src.models.vehicle import KinematicBicycleSystem
from src.controllers.cgmres import NMPCControllerCGMRES
from src.utils.dashboard import RealTimeDashboard


def main():
    # ── Simulation timing ──────────────────────────────────────────────
    dt             = global_config["simulation"]["dt"]           # seconds per step
    iteration_time = global_config["simulation"]["iteration_time"]  # total sim seconds
    max_iter       = int(iteration_time / dt)

    # ── Initial vehicle pose and speed ─────────────────────────────────
    init_x   = global_config["initial_state"]["x"]
    init_y   = global_config["initial_state"]["y"]
    init_yaw = math.radians(global_config["initial_state"]["yaw_deg"])
    init_v   = global_config["initial_state"]["v"]

    # ── Build plant (physical vehicle) and controller ──────────────────
    plant      = KinematicBicycleSystem(init_x, init_y, init_yaw, init_v)
    controller = NMPCControllerCGMRES(init_x)

    # ── Launch real-time dashboard (blocks until window is closed) ─────
    dashboard = RealTimeDashboard(plant, controller, dt, max_iter, init_x)
    print("Starting NMPC simulation dashboard...")
    dashboard.run()


if __name__ == "__main__":
    main()
