"""Runs all split-mu scenarios for the standard controllers and writes their figures."""

from __future__ import annotations

from pathlib import Path
from typing import Dict, List, Optional
from vehicle_model.controllers import (
    FourWheelSteeringController,
    IntegratedChassisController,
    PassiveBaselineController,
)
from vehicle_model.vehicle_models.seven_dof import SevenDOFModel

from .animation import animate_split_mu_scenario
from .braking import simulate_split_mu_braking, simulate_split_mu_braking_in_turn
from .launch import simulate_split_mu_launch
from .plots import plot_split_mu_scenario_dashboard
from .results import SplitMuResult
from .transitions import simulate_split_mu_diagonal_transition, simulate_split_mu_evasive_dlc


# =============================================================================
# BATCH EXECUTION RUNNER
# =============================================================================
def run_all_split_mu_scenarios(
    output_dir: str | Path = "output/split_mu_scenarios",
    include_controllers: Optional[List[str]] = None,
    generate_gifs: bool = True,
) -> Dict[str, Dict[str, SplitMuResult]]:
    """Execute the complete suite of 5 split-mu testing scenarios.

    Args:
        output_dir: Destination folder for output PNG dashboards and GIFs.
        include_controllers: List of controller IDs to run ('passive', 'four_wheel_steering', 'integrated_ivmc').
        generate_gifs: If True, generates high-definition animated GIFs for all 5 scenarios.

    Returns:
        Nested dict: {scenario_id: {controller_key: SplitMuResult}}
    """
    out_dir = Path(output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    if include_controllers is None:
        include_controllers = ["passive", "four_wheel_steering", "ivmc_2ws", "full_ivmc_4ws"]

    controllers_map = {
        "passive": ("Passive Baseline (Open Diff)", PassiveBaselineController()),
        "four_wheel_steering": ("Active 4WS (Standalone)", FourWheelSteeringController.from_yaml("configs/steering/four_wheel_steering.yaml", enable_abs=False)),
        "ivmc_2ws": ("IVMC (2WS: TV + ESC)", IntegratedChassisController.from_yaml("configs/controllers/integrated_controller.yaml", enable_4ws=False)),
        "full_ivmc_4ws": ("Full-Stack IVMC + 4WS", IntegratedChassisController.from_yaml("configs/controllers/integrated_controller.yaml", enable_4ws=True)),
        "integrated_ivmc": ("Full-Stack IVMC + 4WS", IntegratedChassisController.from_yaml("configs/controllers/integrated_controller.yaml", enable_4ws=True)),
    }

    selected_controllers = {k: controllers_map[k] for k in include_controllers if k in controllers_map}

    all_scenario_results: Dict[str, Dict[str, SplitMuResult]] = {}

    # 1. Straight-Line Braking
    res_s1: Dict[str, SplitMuResult] = {}
    for c_key, (disp_name, ctrl) in selected_controllers.items():
        model = SevenDOFModel.from_yaml("configs/vehicles/seven_dof_passenger.yaml")
        res_s1[c_key] = simulate_split_mu_braking(
            model=model,
            controller=ctrl,
            v_initial_kph=100.0,
            controller_name=disp_name,
        )
    all_scenario_results["mu_split_braking"] = res_s1
    plot_split_mu_scenario_dashboard(
        res_s1,
        out_dir / "scenario_1_split_mu_braking.png",
        title="Scenario 1: Split-μ Straight-Line Emergency Braking (100 km/h ECE-R13H)",
    )
    if generate_gifs:
        animate_split_mu_scenario(
            res_s1,
            out_dir / "scenario_1_split_mu_braking.gif",
            title="Scenario 1: Split-μ Emergency Braking (100 km/h) — Dynamic Multi-Agent Simulation",
        )

    # 2. Full-Throttle Launch
    res_s2: Dict[str, SplitMuResult] = {}
    for c_key, (disp_name, ctrl) in selected_controllers.items():
        model = SevenDOFModel.from_yaml("configs/vehicles/seven_dof_passenger.yaml")
        res_s2[c_key] = simulate_split_mu_launch(
            model=model,
            controller=ctrl,
            v_initial_kph=5.0,
            controller_name=disp_name,
        )
    all_scenario_results["mu_split_launch"] = res_s2
    plot_split_mu_scenario_dashboard(
        res_s2,
        out_dir / "scenario_2_split_mu_launch.png",
        title="Scenario 2: Split-μ Full-Throttle Launch & Acceleration (TCS / TV Efficiency)",
    )
    if generate_gifs:
        animate_split_mu_scenario(
            res_s2,
            out_dir / "scenario_2_split_mu_launch.gif",
            title="Scenario 2: Split-μ Full-Throttle Launch — Dynamic Multi-Agent Simulation",
        )

    # 3. Braking in a Turn (Outer Low-μ)
    res_s3: Dict[str, SplitMuResult] = {}
    for c_key, (disp_name, ctrl) in selected_controllers.items():
        model = SevenDOFModel.from_yaml("configs/vehicles/seven_dof_passenger.yaml")
        res_s3[c_key] = simulate_split_mu_braking_in_turn(
            model=model,
            controller=ctrl,
            radius_m=80.0,
            v_initial_kph=48.0,
            low_mu_on_outer=True,
            controller_name=disp_name,
        )
    all_scenario_results["mu_split_braking_in_turn"] = res_s3
    plot_split_mu_scenario_dashboard(
        res_s3,
        out_dir / "scenario_3_split_mu_braking_in_turn.png",
        title="Scenario 3: Split-μ Braking in a Turn (ISO 7975 Outer Low-μ Adhesion Limit)",
    )
    if generate_gifs:
        animate_split_mu_scenario(
            res_s3,
            out_dir / "scenario_3_split_mu_braking_in_turn.gif",
            title="Scenario 3: Split-μ Braking in a Turn (ISO 7975) — Dynamic Multi-Agent Simulation",
        )

    # 4. Diagonal Bridge Transition
    res_s4: Dict[str, SplitMuResult] = {}
    for c_key, (disp_name, ctrl) in selected_controllers.items():
        model = SevenDOFModel.from_yaml("configs/vehicles/seven_dof_passenger.yaml")
        res_s4[c_key] = simulate_split_mu_diagonal_transition(
            model=model,
            controller=ctrl,
            v_initial_kph=80.0,
            angle_deg=35.0,
            controller_name=disp_name,
        )
    all_scenario_results["mu_split_diagonal_transition"] = res_s4
    plot_split_mu_scenario_dashboard(
        res_s4,
        out_dir / "scenario_4_split_mu_diagonal_transition.png",
        title="Scenario 4: Diagonal Split-μ Bridge Transition (Staggered Wheel Entry & Driver Reaction)",
    )
    if generate_gifs:
        animate_split_mu_scenario(
            res_s4,
            out_dir / "scenario_4_split_mu_diagonal_transition.gif",
            title="Scenario 4: Diagonal Split-μ Bridge Transition — Dynamic Multi-Agent Simulation",
        )

    # 5. Evasive Double Lane Change
    res_s5: Dict[str, SplitMuResult] = {}
    for c_key, (disp_name, ctrl) in selected_controllers.items():
        model = SevenDOFModel.from_yaml("configs/vehicles/seven_dof_passenger.yaml")
        res_s5[c_key] = simulate_split_mu_evasive_dlc(
            model=model,
            controller=ctrl,
            v_initial_kph=75.0,
            controller_name=disp_name,
        )
    all_scenario_results["mu_split_evasive_dlc"] = res_s5
    plot_split_mu_scenario_dashboard(
        res_s5,
        out_dir / "scenario_5_split_mu_evasive_dlc.png",
        title="Scenario 5: Split-μ Severe Double Lane Change (ISO 3888-2 Obstacle Evasion on Slick Lane)",
    )
    if generate_gifs:
        animate_split_mu_scenario(
            res_s5,
            out_dir / "scenario_5_split_mu_evasive_dlc.gif",
            title="Scenario 5: Split-μ Evasive DLC (ISO 3888-2) — Dynamic Multi-Agent Simulation",
        )

    return all_scenario_results
