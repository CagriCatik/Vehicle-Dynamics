"""Demo helpers for the lateral Magic Formula tire model."""

from __future__ import annotations

from pathlib import Path

import numpy as np

from vehicle_model.config import load_settings

from .magic_formula_lateral import MagicFormulaLateral


def _tire_demo_config(config_path: str | Path | None = None) -> dict:
    return load_settings(config_path)["tire_demo"]


def create_default_model(config_path: str | Path | None = None) -> MagicFormulaLateral:
    config = _tire_demo_config(config_path)
    data_path = Path(__file__).resolve().parent / config["tire_data_dir"]
    return MagicFormulaLateral([config["front_tire_name"], config["rear_tire_name"]], data_path, config_path=config_path)


def calculate_lateral_curves(n_points: int | None = None, config_path: str | Path | None = None):
    config = _tire_demo_config(config_path)
    n_samples = config["n_points"] if n_points is None else n_points
    model = create_default_model(config_path)
    Fz = np.ones(n_samples) * config["normal_load"]
    kappa = np.linspace(config["slip_ratio_min"], config["slip_ratio_max"], n_samples)
    alpha = np.linspace(config["slip_angle_min"], config["slip_angle_max"], n_samples)
    gamma = np.ones(n_samples) * config["camber"]
    pressure = np.ones(n_samples) * config["pressure"]
    vx = np.linspace(config["velocity_min"], config["velocity_max"], n_samples)

    curves = {
        "front_pure": np.zeros(n_samples),
        "front_combined": np.zeros(n_samples),
        "rear_pure": np.zeros(n_samples),
        "rear_combined": np.zeros(n_samples),
    }
    for idx in range(n_samples):
        model.online_params(Fz[idx], pressure[idx], gamma[idx], kappa[idx], alpha[idx], vx[idx], config["front_tire_name"])
        curves["front_pure"][idx], _ = model.calculateFy0()
        curves["front_combined"][idx], _ = model.calculateFy()
        model.online_params(Fz[idx], pressure[idx], gamma[idx], kappa[idx], alpha[idx], vx[idx], config["rear_tire_name"])
        curves["rear_pure"][idx], _ = model.calculateFy0()
        curves["rear_combined"][idx], _ = model.calculateFy()
    return curves


def main(argv: list[str] | None = None) -> None:
    import argparse
    import matplotlib.pyplot as plt

    config_parser = argparse.ArgumentParser(add_help=False)
    config_parser.add_argument("--config", type=Path, default=None, help="Optional YAML config override.")
    config_args, _ = config_parser.parse_known_args(argv)
    config = _tire_demo_config(config_args.config)

    parser = argparse.ArgumentParser(
        description="Plot lateral Magic Formula tire-force curves.",
        parents=[config_parser],
    )
    parser.add_argument("--points", type=int, default=config["n_points"], help="Number of samples per curve.")
    parser.add_argument("--no-show", action="store_true", help="Build figures and exit without opening a GUI window.")
    parser.add_argument("--output-dir", type=Path, default=None, help="Optional directory for PNG exports.")
    args = parser.parse_args(argv)

    demo = calculate_lateral_curves(args.points, args.config)
    fig_front = plt.figure()
    ax_front = fig_front.add_subplot(1, 1, 1)
    ax_front.plot(demo["front_pure"], label="Pure lateral force, front")
    ax_front.plot(demo["front_combined"], label="Combined lateral force, front")
    ax_front.legend()
    ax_front.grid()

    fig_rear = plt.figure()
    ax_rear = fig_rear.add_subplot(1, 1, 1)
    ax_rear.plot(demo["rear_pure"], label="Pure lateral force, rear")
    ax_rear.plot(demo["rear_combined"], label="Combined lateral force, rear")
    ax_rear.legend()
    ax_rear.grid()

    if args.output_dir is not None:
        args.output_dir.mkdir(parents=True, exist_ok=True)
        fig_front.savefig(args.output_dir / "front_lateral_force.png", dpi=150, bbox_inches="tight")
        fig_rear.savefig(args.output_dir / "rear_lateral_force.png", dpi=150, bbox_inches="tight")

    if args.no_show:
        plt.close(fig_front)
        plt.close(fig_rear)
    else:
        plt.show()


if __name__ == "__main__":
    main()
