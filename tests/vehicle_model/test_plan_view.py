from __future__ import annotations

import math

import numpy as np

from vehicle_model.visualization.plan_view import (
    VehicleParams,
    _translate_all,
    _vehicle_parts,
    draw_vehicle,
    draw_vehicle_trajectory,
    vehicle_polygons,
)


def test_vehicle_parts_keep_rear_wheels_unsteered() -> None:
    params = VehicleParams()

    _, front_right_0, front_left_0, rear_right_0, rear_left_0 = _vehicle_parts(
        params, yaw=0.0, steer_front=0.0
    )
    _, front_right, front_left, rear_right, rear_left = _vehicle_parts(
        params, yaw=0.0, steer_front=math.radians(20.0)
    )

    assert not np.allclose(front_right, front_right_0)
    assert not np.allclose(front_left, front_left_0)
    np.testing.assert_allclose(rear_right, rear_right_0)
    np.testing.assert_allclose(rear_left, rear_left_0)


def test_vehicle_parts_translate_from_rear_axle_reference() -> None:
    params = VehicleParams()
    x = 3.0
    y = -2.0

    body, *_ = _translate_all(_vehicle_parts(params, yaw=0.0, steer_front=0.0), x, y)

    np.testing.assert_allclose(
        body[:, 0],
        np.array([x - params.rear_overhang, y + 0.5 * params.width]),
    )


def test_vehicle_polygons_public_api_returns_body_and_four_wheels() -> None:
    polygons = vehicle_polygons(1.0, 2.0, yaw=0.2, steer_front=0.1, params=VehicleParams())

    assert len(polygons) == 5
    for polygon in polygons:
        assert polygon.shape == (2, 5)


def test_draw_vehicle_smoke_with_headless_matplotlib() -> None:
    import matplotlib

    matplotlib.use("Agg", force=True)
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots()
    plt.sca(ax)
    try:
        draw_vehicle(0.0, 0.0, 0.0, math.radians(10.0), VehicleParams(), arrow_color="tab:green")
        assert len(ax.lines) >= 8
    finally:
        plt.close(fig)


def test_vehicle_parts_steers_rear_wheels_in_4ws() -> None:
    """When steer_rear is non-zero, rear wheel polygons must rotate."""
    params = VehicleParams()
    _, _, _, rr_0, rl_0 = _vehicle_parts(params, yaw=0.0, steer_front=0.0, steer_rear=0.0)
    _, _, _, rr_steered, rl_steered = _vehicle_parts(
        params, yaw=0.0, steer_front=0.0, steer_rear=math.radians(10.0)
    )

    assert not np.allclose(rr_steered, rr_0)
    assert not np.allclose(rl_steered, rl_0)


def test_draw_vehicle_trajectory_renders_snapshots() -> None:
    """draw_vehicle_trajectory must plot path and snapshot polygons on the provided axis."""
    import matplotlib

    matplotlib.use("Agg", force=True)
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots()
    try:
        X = np.linspace(0.0, 50.0, 50)
        Y = np.sin(X / 10.0)
        psi = np.arctan(np.gradient(Y, X))
        steer_f = np.full(50, 0.05)
        steer_r = np.full(50, -0.02)

        draw_vehicle_trajectory(
            ax=ax,
            X=X,
            Y=Y,
            psi=psi,
            steer_f=steer_f,
            steer_r=steer_r,
            n_snapshots=5,
            color="#2ca02c",
        )
        # Should have 1 trajectory line + 5 snapshots * (5 polygons + 1 centerline + 1 ref star) lines
        assert len(ax.lines) > 20
    finally:
        plt.close(fig)


def test_cog_reference_point_shifts_the_vehicle_back_by_lr() -> None:
    """Simulation results give the CoG; with cog_to_rear_axle set the body is drawn around it."""
    rear = VehicleParams()
    cog = VehicleParams(cog_to_rear_axle=1.5)
    yaw = 0.4
    body_rear = vehicle_polygons(0.0, 0.0, yaw, 0.0, rear)[0]
    body_cog = vehicle_polygons(1.5 * math.cos(yaw), 1.5 * math.sin(yaw), yaw, 0.0, cog)[0]
    np.testing.assert_allclose(body_cog, body_rear, atol=1e-12)
