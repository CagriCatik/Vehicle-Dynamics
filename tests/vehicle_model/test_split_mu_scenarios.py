"""Unit and Physics Tests for Split-Mu Dynamics and Scenario Validation.

Conforms to:
- ISO 7975: Braking in a Turn
- ISO 3888-2: Obstacle Avoidance / Severe Double Lane Change
- ECE-R13H: Passenger Car Braking on Split-Mu Surfaces
- Küçükay, F. (2022): Grundlagen der Fahrzeugtechnik, Springer Vieweg.
- Mitschke, M.; Wallentowitz, H. (2014): Dynamik der Kraftfahrzeuge.
"""

from __future__ import annotations

import math
from pathlib import Path
import numpy as np
import pytest

from vehicle_model.controllers import (
    FourWheelSteeringController,
    IntegratedChassisController,
    PassiveBaselineController,
)
from vehicle_model.roads.surface import (
    CheckerboardRoad,
    CurvedSplitRoad,
    DiagonalSplitRoad,
    MuSplitRoad,
)
from vehicle_model.simulation.split_mu_scenarios import (
    SplitMuFrictionSweepSummary,
    SplitMuMetrics,
    SplitMuResult,
    animate_split_mu_scenario,
    plot_split_mu_friction_sweep,
    plot_split_mu_scenario_dashboard,
    simulate_split_mu_braking,
    simulate_split_mu_braking_in_turn,
    simulate_split_mu_diagonal_transition,
    simulate_split_mu_evasive_dlc,
    simulate_split_mu_friction_sweep,
    simulate_split_mu_launch,
)
from vehicle_model.vehicle_models.seven_dof import SevenDOFModel


@pytest.fixture
def test_vehicle() -> SevenDOFModel:
    return SevenDOFModel.from_yaml("configs/vehicles/seven_dof_passenger.yaml")


@pytest.fixture
def ivmc_controller() -> IntegratedChassisController:
    return IntegratedChassisController.from_yaml("configs/controllers/integrated_controller.yaml")


# =============================================================================
# 1. 2D SPATIAL ROAD SURFACE TESTS
# =============================================================================
class TestSplitMuRoadSurfaces:
    """Validate mathematical formulation of advanced 2D split-mu road surfaces."""

    def test_diagonal_split_road(self):
        """Verify angled boundary transition and smooth sigmoid blending."""
        # 45 deg angle dividing line at x=20, y=0
        road = DiagonalSplitRoad(
            mu_high=1.0,
            mu_low=0.20,
            x0=20.0,
            y0=0.0,
            theta_deg=45.0,
            transition_width=0.50,
        )

        # Well into the high-mu side (d > 0)
        mu_high = road.get_friction(10.0, 10.0)
        assert abs(mu_high - 1.0) < 0.05

        # Well into the low-mu side (d < 0)
        mu_low = road.get_friction(30.0, -10.0)
        assert abs(mu_low - 0.20) < 0.05

        # Exactly on the dividing line
        mu_mid = road.get_friction(20.0, 0.0)
        assert abs(mu_mid - 0.60) < 0.02

    def test_curved_split_road(self):
        """Verify circular arc split friction in polar coordinates."""
        # Split radius at 80m, center at (0, 80)
        road = CurvedSplitRoad(
            center_x=0.0,
            center_y=80.0,
            radius_split_m=80.0,
            mu_inner=1.0,
            mu_outer=0.25,
            transition_width=0.30,
        )

        # Deep inside the curve (distance to (0, 80) is 10m < 80m)
        mu_in = road.get_friction(0.0, 70.0)
        assert abs(mu_in - 1.0) < 0.05

        # Deep outside the curve (distance to (0, 80) is 95m > 80m)
        mu_out = road.get_friction(0.0, -15.0)  # dist = 95m > 80m
        assert abs(mu_out - 0.25) < 0.05

        # Exactly on the split circle (r = 80m)
        mu_center = road.get_friction(0.0, 0.0)  # dist to (0, 80) is 80m
        assert abs(mu_center - 0.625) < 0.02

    def test_checkerboard_road(self):
        """Verify 2D periodic alternating friction patches."""
        road = CheckerboardRoad(
            patch_length_x=10.0,
            patch_width_y=3.0,
            mu_high=0.90,
            mu_low=0.30,
        )

        # Patch (0, 0) -> even sum -> mu_high
        assert abs(road.get_friction(5.0, 1.5) - 0.90) < 0.02
        # Patch (1, 0) -> odd sum -> mu_low
        assert abs(road.get_friction(15.0, 1.5) - 0.30) < 0.02
        # Patch (1, 1) -> even sum -> mu_high
        assert abs(road.get_friction(15.0, 4.5) - 0.90) < 0.02


# =============================================================================
# 2. SCENARIO 1: EMERGENCY BRAKING (ECE-R13H)
# =============================================================================
class TestSplitMuBraking:
    """Validate split-mu emergency braking physics and stabilization."""

    @pytest.mark.slow
    def test_split_mu_braking_passive_vs_ivmc(self, test_vehicle: SevenDOFModel, ivmc_controller: IntegratedChassisController):
        """Verify IVMC drastically attenuates yaw perturbation compared to passive baseline."""
        # Passive baseline
        res_pass = simulate_split_mu_braking(
            model=test_vehicle,
            controller=None,
            v_initial_kph=80.0,
            duration_s=2.5,
            controller_name="Passive Baseline",
        )
        assert isinstance(res_pass, SplitMuResult)
        assert len(res_pass.history.time) > 20
        assert not np.isnan(res_pass.history.vx).any()

        # IVMC active control
        res_ivmc = simulate_split_mu_braking(
            model=test_vehicle,
            controller=ivmc_controller,
            v_initial_kph=80.0,
            duration_s=2.5,
            controller_name="Integrated IVMC",
        )
        assert isinstance(res_ivmc, SplitMuResult)

        # Key physical verification:
        # IVMC active differential braking (GMR) and counter-steer must hold heading
        # significantly tighter than passive chassis
        assert res_ivmc.metrics.final_heading_dev_deg < res_pass.metrics.final_heading_dev_deg
        assert res_ivmc.metrics.max_lateral_dev_m < res_pass.metrics.max_lateral_dev_m
        assert res_ivmc.metrics.final_heading_dev_deg < 5.0  # Stable lane-keeping


# =============================================================================
# 3. SCENARIO 2: LAUNCH & ACCELERATION (TCS / TV)
# =============================================================================
class TestSplitMuLaunch:
    """Validate split-mu full-throttle launch and traction control."""

    @pytest.mark.slow
    def test_split_mu_launch_traction(self, test_vehicle: SevenDOFModel, ivmc_controller: IntegratedChassisController):
        """Verify torque vectoring and TCS achieve superior tractive speed over open-diff."""
        res_pass = simulate_split_mu_launch(
            model=test_vehicle,
            controller=None,
            duration_s=2.5,
            controller_name="Passive Baseline",
        )
        res_ivmc = simulate_split_mu_launch(
            model=test_vehicle,
            controller=ivmc_controller,
            duration_s=2.5,
            controller_name="Integrated IVMC",
        )

        # IVMC with active torque vectoring should accelerate faster
        assert res_ivmc.metrics.final_speed_kph > res_pass.metrics.final_speed_kph
        assert not np.isnan(res_ivmc.history.vx).any()


# =============================================================================
# 4. SCENARIO 3: BRAKING IN A TURN (ISO 7975)
# =============================================================================
class TestSplitMuBrakingInTurn:
    """Validate ISO 7975 combined slip and roll load transfer on split-mu."""

    @pytest.mark.slow
    def test_braking_in_turn_execution(self, test_vehicle: SevenDOFModel, ivmc_controller: IntegratedChassisController):
        """Verify circular turn braking executes with finite states and valid radial metrics."""
        res = simulate_split_mu_braking_in_turn(
            model=test_vehicle,
            controller=ivmc_controller,
            radius_m=80.0,
            v_initial_kph=45.0,
            low_mu_on_outer=True,
            duration_s=2.0,
            controller_name="Integrated IVMC",
        )

        assert isinstance(res, SplitMuResult)
        assert len(res.history.time) > 20
        assert not np.isnan(res.history.X).any()
        assert not np.isnan(res.history.Y).any()
        assert res.metrics.max_lateral_dev_m >= 0.0


# =============================================================================
# 5. SCENARIO 4: DIAGONAL TRANSITION
# =============================================================================
class TestSplitMuDiagonalTransition:
    """Validate staggered wheel entry across angled friction boundaries."""

    def test_diagonal_bridge_transition(self, test_vehicle: SevenDOFModel, ivmc_controller: IntegratedChassisController):
        """Verify smooth traversal and driver closed-loop response across diagonal split."""
        res = simulate_split_mu_diagonal_transition(
            model=test_vehicle,
            controller=ivmc_controller,
            v_initial_kph=80.0,
            angle_deg=30.0,
            duration_s=2.0,
            controller_name="Integrated IVMC",
        )

        assert isinstance(res, SplitMuResult)
        assert not np.isnan(res.history.r).any()
        assert res.metrics.stability_status in ("PASS", "DEGRADED")
        assert res.metrics.max_lateral_dev_m < 1.0


# =============================================================================
# 6. SCENARIO 5: SEVERE DOUBLE LANE CHANGE (ISO 3888-2)
# =============================================================================
class TestSplitMuEvasiveDLC:
    """Validate ISO 3888-2 obstacle avoidance with lane change onto low-mu."""

    def test_evasive_dlc_split_mu(self, test_vehicle: SevenDOFModel, ivmc_controller: IntegratedChassisController):
        """Verify IVMC stabilizes vehicle during severe lane reversal on slippery lane."""
        res_ivmc = simulate_split_mu_evasive_dlc(
            model=test_vehicle,
            controller=ivmc_controller,
            v_initial_kph=70.0,
            duration_s=2.5,
            controller_name="Integrated IVMC",
        )

        assert isinstance(res_ivmc, SplitMuResult)
        assert not np.isnan(res_ivmc.history.psi).any()
        # Active control holds sideslip within recoverable handling envelope
        assert res_ivmc.metrics.max_sideslip_deg < 12.0


# =============================================================================
# 7. DASHBOARD AND ANIMATION VISUALIZATION TESTS
# =============================================================================
class TestSplitMuVisualization:
    """Validate 2D planar vehicle visualization, equal aspect ratio dashboard, and animated GIF generation."""

    @pytest.mark.slow
    def test_dashboard_and_animation_generation(
        self,
        test_vehicle: SevenDOFModel,
        ivmc_controller: IntegratedChassisController,
        tmp_path: Path,
    ):
        """Verify generation of both publication PNG dashboard and high-def animated GIF."""
        res_pass = simulate_split_mu_braking(
            model=test_vehicle,
            controller=None,
            v_initial_kph=80.0,
            duration_s=1.0,
            controller_name="Passive Baseline",
        )
        res_ivmc = simulate_split_mu_braking(
            model=test_vehicle,
            controller=ivmc_controller,
            v_initial_kph=80.0,
            duration_s=1.0,
            controller_name="Integrated IVMC",
        )
        results = {"passive": res_pass, "integrated_ivmc": res_ivmc}

        png_path = tmp_path / "test_split_mu_dashboard.png"
        gif_path = tmp_path / "test_split_mu_animation.gif"

        out_png = plot_split_mu_scenario_dashboard(results, png_path, title="Unit Test Dashboard")
        assert out_png.exists()
        assert out_png.stat().st_size > 10000

        out_gif = animate_split_mu_scenario(results, gif_path, title="Unit Test Animation", fps=10)
        assert out_gif.exists()
        assert out_gif.stat().st_size > 10000

    @pytest.mark.slow
    def test_friction_sweep_execution_and_plotting(self, tmp_path: Path):
        """Verify parametric friction sweep execution and 6-panel diagram export."""
        summary = simulate_split_mu_friction_sweep(
            mu_low_levels=[0.20, 0.60, 1.00],
            v_initial_kph=60.0,
            include_controllers=["passive", "integrated_ivmc"],
        )
        assert isinstance(summary, SplitMuFrictionSweepSummary)
        assert len(summary.sweep_points) == 3
        assert "passive" in summary.yaw_rate_max
        assert "integrated_ivmc" in summary.yaw_rate_max

        out_png = tmp_path / "test_friction_sweep.png"
        plot_split_mu_friction_sweep(summary, out_png)
        assert out_png.exists()
        assert out_png.stat().st_size > 10000

    @pytest.mark.slow
    def test_friction_sweep_all_four_vehicles(self, tmp_path: Path):
        """Verify parametric friction sweep with all 4 vehicles: Passive, Standalone 4WS, IVMC 2WS, and Full IVMC+4WS."""
        summary = simulate_split_mu_friction_sweep(
            mu_low_levels=[0.30, 0.70],
            v_initial_kph=60.0,
        )
        assert set(summary.controllers) == {"passive", "four_wheel_steering", "ivmc_2ws", "full_ivmc_4ws"}
        assert len(summary.controllers) == 4
        out_png = tmp_path / "test_sweep_4veh.png"
        plot_split_mu_friction_sweep(summary, out_png)
        assert out_png.exists()


