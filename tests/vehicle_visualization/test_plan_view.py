from __future__ import annotations

import math

import numpy as np

from vehicle_visualization.plan_view import VehicleParams, _translate_all, _vehicle_parts, draw_vehicle, vehicle_polygons


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
