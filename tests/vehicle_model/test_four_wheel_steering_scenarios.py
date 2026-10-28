"""Unit and Regression Tests for Four-Wheel Steering (4WS) Multi-Scenario Suite.

Validates:
1. Low-Speed Turning Circle reduction (ISO 7401 / SAE J266)
2. High-Speed Evasive Double Lane Change sideslip minimization (ISO 3888-2)
3. Step Steer transient response latency (ISO 7401)
4. Split-μ emergency braking disturbance rejection (ECE-R13H)
"""

from __future__ import annotations

import pytest
import numpy as np

from vehicle_model.controllers.framework import FourWheelSteeringController, PassiveBaselineController
from vehicle_model.controllers.integrated_controller import IntegratedChassisController
from vehicle_model.simulation.four_wheel_steering_scenarios import (
    simulate_high_speed_double_lane_change,
    simulate_high_speed_step_steer,
    simulate_low_speed_turning_circle,
    simulate_parallel_parking,
    simulate_sine_with_dwell,
    simulate_split_mu_braking_4ws,
)
from vehicle_model.steering.four_wheel_steering import SteeringMode
from vehicle_model.vehicle_models.seven_dof import SevenDOFModel


@pytest.fixture
def test_vehicle() -> SevenDOFModel:
    return SevenDOFModel.from_yaml("configs/vehicles/seven_dof_passenger.yaml")


class TestFourWheelSteeringScenarios:
    """Validate 4WS across low-speed, high-speed, transient, and emergency scenarios."""

    @pytest.mark.slow
    def test_low_speed_turning_circle_reduction(self, test_vehicle: SevenDOFModel):
        """Verify 4WS counter-phase significantly reduces turning circle radius compared to 2WS."""
        res_2ws = simulate_low_speed_turning_circle(
            model=test_vehicle,
            enable_4ws=False,
            v_kph=12.0,
            front_steer_deg=25.0,
            duration_s=6.0,
            controller_name="2WS",
        )
        res_4ws = simulate_low_speed_turning_circle(
            model=test_vehicle,
            steer_mode=SteeringMode.LOOKUP_TABLE,
            enable_4ws=True,
            v_kph=12.0,
            front_steer_deg=25.0,
            duration_s=6.0,
            controller_name="4WS LUT",
        )

        r_2ws = res_2ws.metrics.min_turning_radius_m
        r_4ws = res_4ws.metrics.min_turning_radius_m

        assert r_2ws is not None and r_4ws is not None
        # 4WS counter-phase should shorten turning radius by at least 15%
        assert r_4ws < r_2ws
        assert (r_2ws - r_4ws) / r_2ws > 0.15

    @pytest.mark.slow
    def test_high_speed_double_lane_change_sideslip(self, test_vehicle: SevenDOFModel):
        """Verify 4WS in-phase steering minimizes body sideslip beta during 80 km/h double lane change."""
        ctrl_2ws = PassiveBaselineController()
        ctrl_4ws = FourWheelSteeringController(mode=SteeringMode.LOOKUP_TABLE, enable_yaw_damping=True)

        res_2ws = simulate_high_speed_double_lane_change(
            model=test_vehicle,
            controller=ctrl_2ws,
            v_kph=80.0,
            duration_s=3.0,
            controller_name="2WS",
        )
        res_4ws = simulate_high_speed_double_lane_change(
            model=test_vehicle,
            controller=ctrl_4ws,
            v_kph=80.0,
            duration_s=3.0,
            controller_name="4WS LUT + Damping",
        )

        beta_2ws = res_2ws.metrics.peak_sideslip_deg
        beta_4ws = res_4ws.metrics.peak_sideslip_deg

        assert beta_2ws is not None and beta_4ws is not None
        # 4WS in-phase should suppress peak sideslip compared to front-only
        assert beta_4ws < beta_2ws
        assert not np.isnan(res_4ws.X).any()

    def test_high_speed_step_steer_transient(self, test_vehicle: SevenDOFModel):
        """Verify step steer response produces finite lateral acceleration and bounded sideslip."""
        ctrl_4ws = FourWheelSteeringController(mode=SteeringMode.LOOKUP_TABLE, enable_yaw_damping=True)
        res = simulate_high_speed_step_steer(
            model=test_vehicle,
            controller=ctrl_4ws,
            v_kph=100.0,
            step_steer_deg=3.0,
            duration_s=2.0,
            controller_name="4WS LUT",
        )

        assert res.metrics.response_delay_ms is not None
        assert res.metrics.response_delay_ms >= 0.0
        assert not np.isnan(res.ay_mps2).any()

    @pytest.mark.slow
    def test_split_mu_braking_active_yaw_damping(self, test_vehicle: SevenDOFModel):
        """Verify active yaw damping on rear wheels reduces yaw deviation during split-mu braking."""
        ctrl_open_loop = FourWheelSteeringController(mode=SteeringMode.LOOKUP_TABLE, enable_yaw_damping=False)
        ctrl_closed_loop = FourWheelSteeringController(mode=SteeringMode.LOOKUP_TABLE, enable_yaw_damping=True, k_yaw_damping=0.08)

        res_open = simulate_split_mu_braking_4ws(
            model=test_vehicle,
            controller=ctrl_open_loop,
            v_kph=100.0,
            duration_s=2.5,
            controller_name="4WS Open-Loop",
        )
        res_closed = simulate_split_mu_braking_4ws(
            model=test_vehicle,
            controller=ctrl_closed_loop,
            v_kph=100.0,
            duration_s=2.5,
            controller_name="4WS Closed-Loop",
        )

        # Closed-loop rear yaw damping must suppress lateral path perturbation compared to open-loop 4WS
        assert res_closed.metrics.max_lateral_dev_m <= res_open.metrics.max_lateral_dev_m

    @pytest.mark.slow
    def test_parallel_parking_sweep_reduction(self, test_vehicle: SevenDOFModel):
        """Verify 4WS counter-phase achieves successful parking entry and reduced lateral offset."""
        res_2ws = simulate_parallel_parking(
            model=test_vehicle,
            enable_4ws=False,
            duration_s=7.2,
            controller_name="2WS",
        )
        res_4ws = simulate_parallel_parking(
            model=test_vehicle,
            enable_4ws=True,
            duration_s=7.2,
            controller_name="4WS LUT",
        )

        assert res_2ws.metrics.max_lateral_dev_m is not None
        assert res_4ws.metrics.max_lateral_dev_m is not None
        assert not np.isnan(res_4ws.X).any()
        # 4WS vehicle should park deeper into the parking space (lower Y) and align with minimal heading error
        assert res_4ws.Y[-1] < res_2ws.Y[-1]
        assert res_4ws.metrics.final_heading_dev_deg < 2.0

    @pytest.mark.slow
    def test_nhtsa_sine_with_dwell_stability(self, test_vehicle: SevenDOFModel):
        """Verify 4WS active yaw damping prevents spin-out and maintains stability during FMVSS 126 test."""
        ctrl_2ws = PassiveBaselineController()
        ctrl_4ws = FourWheelSteeringController(mode=SteeringMode.LOOKUP_TABLE, enable_yaw_damping=True)

        res_2ws = simulate_sine_with_dwell(
            model=test_vehicle,
            controller=ctrl_2ws,
            v_kph=80.0,
            duration_s=4.0,
            controller_name="2WS",
        )
        res_4ws = simulate_sine_with_dwell(
            model=test_vehicle,
            controller=ctrl_4ws,
            v_kph=80.0,
            duration_s=4.0,
            controller_name="4WS LUT + Damping",
        )

        # 4WS must suppress sideslip compared to unassisted baseline
        assert res_4ws.metrics.peak_sideslip_deg < res_2ws.metrics.peak_sideslip_deg
        assert res_4ws.metrics.stability_status == "STABLE"

