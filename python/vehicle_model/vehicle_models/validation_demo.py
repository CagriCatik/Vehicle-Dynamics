"""Validate single-track models against the bundled example run."""

from __future__ import annotations

import argparse
import math
from collections import deque
from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from matplotlib.lines import Line2D
from matplotlib.patches import Circle, Polygon

from vehicle_model.config import load_settings
from vehicle_visualization.plan_view import VehicleParams as PlanViewVehicleParams
from vehicle_visualization.plan_view import vehicle_polygons

from .config import vehicle_params_from_config
from .data_loader import load_data
from .dynamic_single_track import DynamicSingleTrack
from .kinematic_single_track import KinematicSingleTrack
from .validation_telemetry import build_validation_telemetry


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def _plan_view_params_from_model(model_params: Any, plan_view_config: dict[str, Any]) -> PlanViewVehicleParams:
    wheelbase = model_params.lf + model_params.lr
    width = model_params.width
    length = model_params.length
    rear_overhang = model_params.lr
    front_overhang = max(0.0, length - wheelbase - rear_overhang)
    return PlanViewVehicleParams(
        wheelbase=wheelbase,
        width=width,
        length=rear_overhang + wheelbase + front_overhang,
        tire_radius=model_params.tire_radius,
        tire_width=plan_view_config["tire_width"],
        wheel_track=plan_view_config["wheel_track_width_ratio"] * width,
        rear_overhang=rear_overhang,
        front_overhang=front_overhang,
    )


class Vehicle:
    def __init__(self, params: Any):
        self.params = params
        self.times = None
        self.true_states = None
        self.pred_states = None
        self.inputs = None
        self.ekin_inputs = None
        self.roll_data = None
        self.wheel_data = None
        self.states = None

    def gen_data(self, directory: str | Path, dataset: str, data_limit: int = -1, config_path=None) -> None:
        self.times, states, self.inputs, self.ekin_inputs = load_data(
            Path(directory) / f"{dataset}.csv",
            idx=data_limit,
            config_path=config_path,
        )
        self.true_states = states[:, :7]
        self.pred_states = np.zeros(self.true_states.shape)
        self.pred_states[0, :] = self.true_states[0, :]
        self.wheel_data = states[:, -5:-1]
        self.roll_data = states[:, -1]
        self.states = states


class Simulation:
    def __init__(
        self,
        vehicle_data: Vehicle,
        model: str,
        sample_time: float,
        base_horizon: int,
        average_wheel_speeds: bool,
    ):
        self.vehicle = vehicle_data
        self.N_SAMPLES = len(vehicle_data.inputs)
        self.SAMPLING_TIME = sample_time
        self.times = vehicle_data.times
        self.states = vehicle_data.states
        self.inputs = vehicle_data.inputs
        self.true_states = vehicle_data.true_states
        self.states_pred = vehicle_data.pred_states
        self.wheel_data = vehicle_data.wheel_data
        if average_wheel_speeds:
            self.wheel_data = np.mean(vehicle_data.wheel_data, axis=1, keepdims=True)
        self.roll_data = vehicle_data.roll_data
        self.dxdt_ek = np.zeros(vehicle_data.true_states.shape)
        self.base_horizon = base_horizon
        self.steer_inputs = vehicle_data.ekin_inputs[:, 2]
        self.veh_model = model

    def calc_steer_diff(self, steering: float, std_dev: float) -> tuple[float, float]:
        if abs(steering) > std_dev:
            return 0.0, abs(steering) - std_dev
        return std_dev - abs(steering), 0.0

    def _model_instance(self):
        if self.veh_model == "dynamic":
            return DynamicSingleTrack(self.vehicle.params)
        if self.veh_model == "kinematic":
            return KinematicSingleTrack(self.vehicle.params)
        raise ValueError(f"Unsupported validation model: {self.veh_model}")

    def _prepare_kinematic_state(self, model: KinematicSingleTrack) -> None:
        kin_states = np.zeros((len(self.true_states), model.n_states))
        kin_inputs = np.zeros((len(self.steer_inputs), model.n_inputs))
        kin_states[:, :2] = self.states[:, :2]
        kin_states[:, 2] = self.states[:, 4]
        kin_states[:, 3] = self.states[:, 2]
        kin_states[:, 4] = self.states[:, 5]
        kin_inputs[:, 0] = self.states[:, 7]
        kin_inputs[1:, 1] = np.diff(self.steer_inputs)
        self.true_states = kin_states
        self.inputs = kin_inputs
        self.states_pred = np.zeros(kin_states.shape)
        self.states_pred[0, :] = kin_states[0, :]
        self.dxdt_ek = np.zeros(kin_states.shape)

    def open_sim(
        self,
        multi_step: bool,
        step_base: int,
        c1_scaler: float,
        c2_scaler: float,
        horizon_fixed: bool = True,
        min_step: int = 1,
    ) -> None:
        self.base_horizon = step_base
        step = step_base
        model = self._model_instance()
        if self.veh_model == "kinematic":
            self._prepare_kinematic_state(model)

        std_steer = np.std(self.steer_inputs)
        self.step_ls = [step]
        if not multi_step:
            for idx in range(self.N_SAMPLES - 1):
                inputs = np.vstack((self.inputs[idx, :], self.inputs[idx + 1, :])).T
                x_next, dxdt_next = model.sim_continuous(self.true_states[idx, :], inputs, [0, self.SAMPLING_TIME])
                self.states_pred[idx + 1, :] = x_next[-1, :]
                self.dxdt_ek[idx + 1, :] = dxdt_next[-1, :]
            return

        idx = 0
        while idx < self.N_SAMPLES - 1:
            if horizon_fixed:
                step = min(step, self.N_SAMPLES - 1 - idx)
            else:
                step = step_base
                in_range_diff, out_range_diff = self.calc_steer_diff(self.steer_inputs[idx], std_steer)
                step += math.floor(in_range_diff * c1_scaler * step) - math.ceil(out_range_diff * c2_scaler * step)
                step = min(step, self.N_SAMPLES - 1 - idx)
                step = max(min_step, step)

            inputs = np.vstack((self.inputs[idx : idx + step, :]))
            x_next, dxdt_next, self.step_ls = model.sim_continuous_multistep(
                self.true_states[idx, :],
                inputs,
                [0, self.SAMPLING_TIME],
                step,
                self.step_ls,
                wheel_v=self.wheel_data[idx : idx + step, :],
                roll=self.roll_data[idx : idx + step],
            )
            self.states_pred[idx + 1 : idx + step + 1, :] = x_next[1:, :]
            self.dxdt_ek[idx + 1 : idx + step + 1, :] = dxdt_next[1:, :]
            idx += step


class Plotting:
    def __init__(
        self,
        simulation: Simulation,
        vx_start: float,
        left_np: np.ndarray,
        right_np: np.ndarray,
        pit_np: np.ndarray,
        whole_track: bool,
        plot_config: dict[str, Any],
        plan_view_config: dict[str, Any],
    ):
        self.simulation = simulation
        self.pit_np = pit_np
        self.plot_config = plot_config
        self.plan_view_config = plan_view_config
        model = simulation.veh_model
        history_multiplier = plot_config["arrow_history_horizon_multiplier"]
        horizon = simulation.base_horizon + history_multiplier
        self.vx = simulation.states[:, 2]
        self.steering_angle = simulation.steer_inputs
        self.step_ls = simulation.step_ls
        start_candidates = np.where(self.vx > vx_start)[0]
        self.id = int(start_candidates[0]) if start_candidates.size else 0

        self.x = simulation.states[self.id :, 0]
        self.y = simulation.states[self.id :, 1]
        self.vx = self.vx[self.id :]
        self.steering_angle = self.steering_angle[self.id :]
        self.x_pred = simulation.states_pred[self.id :, 0]
        self.y_pred = simulation.states_pred[self.id :, 1]
        self.angles = simulation.states[self.id :, 4]
        self.steer_true = simulation.states[self.id :, 5]
        if model == "kinematic":
            self.angles_pred = simulation.states_pred[self.id :, 2]
            self.steer_pred = simulation.states_pred[self.id :, 4]
        else:
            self.angles_pred = simulation.states_pred[self.id :, 4]
            self.steer_pred = simulation.states_pred[self.id :, 5]
        self.plan_view_params = _plan_view_params_from_model(simulation.vehicle.params, plan_view_config)
        self.telemetry_config = plot_config.get("telemetry", {})
        self.telemetry_enabled = bool(self.telemetry_config.get("enabled", False))
        self.telemetry = build_validation_telemetry(simulation) if self.telemetry_enabled else None
        self.telemetry_time = np.asarray([])
        self.telemetry_signals: dict[str, np.ndarray] = {}
        self.telemetry_axes = []
        self.telemetry_panels = []
        if self.telemetry is not None:
            self.telemetry_time = self.telemetry.time[self.id :] - self.telemetry.time[self.id]
            self.telemetry_signals = {
                key: np.asarray(value[self.id :], dtype=float)
                for key, value in self.telemetry.signals.items()
            }

        self.fig, self.ax = self._create_figure()
        self.ax.plot(left_np[:, 0], left_np[:, 1], label="left bound")
        self.ax.plot(right_np[:, 0], right_np[:, 1], label="right bound")
        if self.pit_np.any():
            self.ax.plot(self.pit_np[:, 0], self.pit_np[:, 1], label="pit")
        self.ax.set_xlabel("$X [m]$", fontsize=plot_config["label_font_size"])
        self.ax.set_ylabel("$Y [m]$", fontsize=plot_config["label_font_size"])
        self.ax.set_title(plot_config["title"].format(model=model), fontsize=plot_config["title_font_size"])

        self.focus_on = not whole_track
        if whole_track:
            margin = plot_config["axis_margin"]
            self.ax.set_xlim(min(self.x) - margin, max(self.x) + margin)
            self.ax.set_ylim(min(self.y) - margin, max(self.y) + margin)
            self.ax.set_aspect("equal", "box")

        self.arrow1_storage = deque(maxlen=horizon)
        self.arrow2_storage = deque(maxlen=horizon)
        self.length = plot_config["arrow_length"]
        self._add_initial_arrows()

        self.true_patches = self.create_vehicle_patches(
            self.x[0], self.y[0], self.angles[0], self.steer_true[0], "true"
        )
        self.pred_patches = self.create_vehicle_patches(
            self.x_pred[0], self.y_pred[0], self.angles_pred[0], self.steer_pred[0], "predicted"
        )
        for true_patch, pred_patch in zip(self.true_patches, self.pred_patches):
            self.ax.add_patch(pred_patch)
            self.ax.add_patch(true_patch)

        steering_config = plot_config["steering_wheel"]
        self.steering_wheel_radius = steering_config["radius"]
        self.steering_wheel_offset_x = steering_config["offset_x"]
        self.steering_wheel_offset_y = steering_config["offset_y"]
        self.text_offset_x = steering_config["text_offset_x"]
        self.text_offset_y = -self.steering_wheel_radius - steering_config["text_offset_y_extra"]
        self.draw_steering_wheel(
            self.ax,
            self.x[0] + self.steering_wheel_offset_x,
            self.y[0] + self.steering_wheel_offset_y,
            self.steering_wheel_radius,
            self.angles[0],
        )
        self.steering_angle_text = self.ax.text(
            self.x[0] + self.steering_wheel_offset_x,
            self.y[0] + self.steering_wheel_offset_y + self.text_offset_y,
            "",
            ha="center",
        )
        self._init_telemetry_plots()

    def _create_figure(self):
        if not self.telemetry_enabled:
            return plt.subplots(figsize=tuple(self.plot_config["figure_size"]))

        panels = self.telemetry_config["panels"]
        panel_columns = self.telemetry_config["panel_columns"]
        panel_rows = math.ceil(len(panels) / panel_columns)
        fig = plt.figure(figsize=tuple(self.telemetry_config["figure_size"]))
        width_ratios = [self.telemetry_config["map_width_ratio"]]
        width_ratios.extend([self.telemetry_config["panel_width_ratio"]] * panel_columns)
        gs = fig.add_gridspec(
            panel_rows,
            panel_columns + 1,
            width_ratios=width_ratios,
            hspace=self.telemetry_config["hspace"],
            wspace=self.telemetry_config["wspace"],
        )
        track_ax = fig.add_subplot(gs[:, 0])
        self.telemetry_axes = [
            fig.add_subplot(gs[idx // panel_columns, 1 + idx % panel_columns])
            for idx in range(len(panels))
        ]
        return fig, track_ax

    def _finite_limits(self, keys: list[str], frame_mask=None):
        arrays = []
        for key in keys:
            values = self.telemetry_signals.get(key)
            if values is None:
                continue
            data = values if frame_mask is None else values[frame_mask]
            finite = np.asarray(data)[np.isfinite(data)]
            if finite.size:
                arrays.append(finite)
        if not arrays:
            return None

        merged = np.concatenate(arrays)
        low = float(np.min(merged))
        high = float(np.max(merged))
        min_span = self.telemetry_config["min_y_span"]
        if high - low < min_span:
            center = 0.5 * (high + low)
            low = center - 0.5 * min_span
            high = center + 0.5 * min_span
        padding = self.telemetry_config["y_padding_ratio"] * (high - low)
        return low - padding, high + padding

    def _init_telemetry_plots(self) -> None:
        if not self.telemetry_enabled or self.telemetry is None:
            return

        panel_columns = self.telemetry_config["panel_columns"]
        for panel_idx, (ax, panel_config) in enumerate(zip(self.telemetry_axes, self.telemetry_config["panels"])):
            ax.set_title(panel_config["title"], fontsize=self.telemetry_config["title_font_size"])
            ax.set_ylabel(panel_config["ylabel"], fontsize=self.telemetry_config["label_font_size"])
            if panel_idx // panel_columns == math.ceil(len(self.telemetry_config["panels"]) / panel_columns) - 1:
                ax.set_xlabel(self.telemetry_config["x_label"], fontsize=self.telemetry_config["label_font_size"])
            ax.grid(True, linestyle="--", alpha=self.telemetry_config["grid_alpha"])
            ax.axhline(
                0.0,
                color=self.telemetry_config["zero_line_color"],
                linewidth=self.telemetry_config["zero_line_width"],
            )
            keys = []
            lines = []
            for series_config in panel_config["series"]:
                key = series_config["key"]
                keys.append(key)
                line, = ax.plot(
                    [],
                    [],
                    label=series_config["label"],
                    color=series_config["color"],
                    linestyle=series_config.get("linestyle", "-"),
                    linewidth=self.telemetry_config["line_width"],
                )
                lines.append((line, key))
            limits = self._finite_limits(keys)
            if limits is not None:
                ax.set_ylim(*limits)
            cursor = ax.axvline(
                0.0,
                color=self.telemetry_config["cursor_color"],
                linewidth=self.telemetry_config["cursor_line_width"],
            )
            ax.legend(loc="upper right", fontsize=self.telemetry_config["legend_font_size"])
            self.telemetry_panels.append({"ax": ax, "lines": lines, "cursor": cursor, "keys": keys})
        self._update_telemetry(0)

    def _telemetry_window(self, frame: int):
        if self.telemetry_time.size == 0:
            return np.asarray([], dtype=bool), 0.0, 1.0, 0.0

        frame = min(frame, self.telemetry_time.size - 1)
        current_time = self.telemetry_time[frame]
        window_sec = self.telemetry_config["window_sec"]
        if window_sec > 0.0:
            x_min = max(0.0, current_time - window_sec)
            x_max = max(window_sec, current_time)
            mask = (self.telemetry_time >= x_min) & (self.telemetry_time <= current_time)
            return mask, x_min, x_max, current_time

        x_max = max(float(self.telemetry_time[-1]), 1.0)
        mask = self.telemetry_time <= current_time
        return mask, 0.0, x_max, current_time

    def _update_telemetry(self, frame: int) -> None:
        if not self.telemetry_enabled or self.telemetry is None:
            return

        mask, x_min, x_max, current_time = self._telemetry_window(frame)
        x_data = self.telemetry_time[mask]
        for panel in self.telemetry_panels:
            for line, key in panel["lines"]:
                values = self.telemetry_signals.get(key)
                if values is not None:
                    line.set_data(x_data, values[mask])
            panel["cursor"].set_xdata([current_time, current_time])
            panel["ax"].set_xlim(x_min, x_max)
            if self.telemetry_config["autoscale_y"]:
                limits = self._finite_limits(panel["keys"], mask)
                if limits is not None:
                    panel["ax"].set_ylim(*limits)

    def _add_initial_arrows(self) -> None:
        true_arrow = plt.Arrow(
            self.x[0],
            self.y[0],
            self.length * np.cos(self.angles[0]),
            self.length * np.sin(self.angles[0]),
            width=self.plot_config["arrow_width"],
            color=self.plot_config["true_arrow_color"],
            label="True position",
        )
        pred_arrow = plt.Arrow(
            self.x_pred[0],
            self.y_pred[0],
            self.length * np.cos(self.angles_pred[0]),
            self.length * np.sin(self.angles_pred[0]),
            width=self.plot_config["arrow_width"],
            color=self.plot_config["predicted_arrow_color"],
            label="Model position",
        )
        self.ax.add_patch(true_arrow)
        self.ax.add_patch(pred_arrow)
        self.ax.legend()

    def vehicle_patch_polygons(self, x, y, yaw, steer):
        return vehicle_polygons(x, y, yaw, steer, self.plan_view_params)

    def create_vehicle_patches(self, x, y, yaw, steer, series):
        polygons = self.vehicle_patch_polygons(x, y, yaw, steer)
        if series == "true":
            colors = self.plot_config["true_vehicle_colors"]
            alpha = self.plot_config["true_vehicle_alpha"]
        else:
            colors = self.plot_config["predicted_vehicle_colors"]
            alpha = self.plot_config["predicted_vehicle_alpha"]
        return [
            Polygon(
                poly.T,
                closed=True,
                fill=False,
                linewidth=self.plot_config["vehicle_linewidth"],
                edgecolor=color,
                alpha=alpha,
            )
            for poly, color in zip(polygons, colors)
        ]

    def update_vehicle_patches(self, patches, x, y, yaw, steer):
        for patch, polygon in zip(patches, self.vehicle_patch_polygons(x, y, yaw, steer)):
            patch.set_xy(polygon.T)

    def draw_steering_wheel(self, ax, center_x, center_y, radius, angle):
        steering_config = self.plot_config["steering_wheel"]
        color = steering_config["color"]
        outer_circle = Circle((center_x, center_y), radius, fill=False, color=color)
        outer_circle.is_steering_wheel_component = True
        ax.add_patch(outer_circle)

        inner_circle = Circle((center_x, center_y), radius * steering_config["inner_radius_ratio"], fill=True, color=color)
        inner_circle.is_steering_wheel_component = True
        ax.add_patch(inner_circle)

        dx = radius * np.cos(np.radians(angle))
        dy = radius * np.sin(np.radians(angle))
        spoke = Line2D([center_x - dx, center_x + dx], [center_y - dy, center_y + dy], color=color)
        spoke.is_steering_wheel_component = True
        ax.add_line(spoke)

    def update(self, frame):
        self._trim_arrow_history()
        self._add_frame_arrows(frame)
        self.update_vehicle_patches(
            self.pred_patches, self.x_pred[frame], self.y_pred[frame], self.angles_pred[frame], self.steer_pred[frame]
        )
        self.update_vehicle_patches(
            self.true_patches, self.x[frame], self.y[frame], self.angles[frame], self.steer_true[frame]
        )
        self._update_steering_wheel(frame)
        self._update_telemetry(frame)

    def _trim_arrow_history(self) -> None:
        if len(self.arrow1_storage) == self.arrow1_storage.maxlen:
            self.arrow1_storage.popleft().remove()
        if len(self.arrow2_storage) == self.arrow2_storage.maxlen:
            self.arrow2_storage.popleft().remove()

    def _add_frame_arrows(self, frame: int) -> None:
        true_arrow = plt.Arrow(
            self.x[frame],
            self.y[frame],
            self.length * np.cos(self.angles[frame]),
            self.length * np.sin(self.angles[frame]),
            width=self.plot_config["arrow_width"],
            color=self.plot_config["true_arrow_color"],
        )
        pred_arrow = plt.Arrow(
            self.x_pred[frame],
            self.y_pred[frame],
            self.length * np.cos(self.angles_pred[frame]),
            self.length * np.sin(self.angles_pred[frame]),
            width=self.plot_config["arrow_width"],
            color=self.plot_config["predicted_arrow_color"],
        )
        self.ax.add_patch(true_arrow)
        self.ax.add_patch(pred_arrow)
        self.arrow1_storage.append(true_arrow)
        self.arrow2_storage.append(pred_arrow)

    def _update_steering_wheel(self, frame: int) -> None:
        for patch in [patch for patch in self.ax.patches if getattr(patch, "is_steering_wheel_component", False)]:
            patch.remove()
        for line in [line for line in self.ax.lines if getattr(line, "is_steering_wheel_component", False)]:
            line.remove()

        steering_config = self.plot_config["steering_wheel"]
        if self.focus_on:
            window_size = self.plot_config["focus_window_size"]
            x_center = self.x[frame]
            y_center = self.y[frame]
            self.ax.set_xlim(
                x_center - window_size * self.plot_config["focus_x_half_width_ratio"],
                x_center + window_size * self.plot_config["focus_x_half_width_ratio"],
            )
            self.ax.set_ylim(
                y_center - window_size * self.plot_config["focus_y_half_width_ratio"],
                y_center + window_size * self.plot_config["focus_y_half_width_ratio"],
            )
            self.ax.set_aspect("equal", "box")
            self.steering_wheel_radius = window_size * steering_config["focus_radius_ratio"]
            center_x = self.x[frame] + window_size * steering_config["focus_center_x_ratio"]
            center_y = self.y[frame] + window_size * steering_config["focus_center_y_ratio"]
            text_x = center_x
            text_y = center_y - window_size * steering_config["focus_text_y_ratio"]
        else:
            center_x = self.x[frame] + self.steering_wheel_offset_x
            center_y = self.y[frame] + self.steering_wheel_offset_y
            text_x = center_x + self.text_offset_x
            text_y = center_y + self.text_offset_y

        self.steering_angle_text.set_position((text_x, text_y))
        steering_wheel_angle_deg = np.degrees(self.steering_angle[frame] / steering_config["steering_ratio"])
        self.draw_steering_wheel(self.ax, center_x, center_y, self.steering_wheel_radius, steering_wheel_angle_deg)
        self.steering_angle_text.set_text(f"Steering wheel angle: {steering_wheel_angle_deg:.2f} deg")

        speed_text = f"Car speed: {self.vx[frame]:.2f} m/s"
        horizon_text = f"Horizon: {self.step_ls[frame + self.id]} step"
        if self.pit_np.any():
            self.ax.legend(["left bound", "right bound", "pitlane", "True position", "Model position", speed_text, horizon_text])
        else:
            self.ax.legend(["left bound", "right bound", "True position", "Model position", speed_text, horizon_text])

    def animate(self):
        self.animation = FuncAnimation(
            self.fig,
            self.update,
            frames=range(len(self.x)),
            interval=self.plot_config["animation_interval_ms"],
        )
        plt.show()


def _parse_args(argv=None):
    config_parser = argparse.ArgumentParser(add_help=False)
    config_parser.add_argument("--config", type=Path, default=None, help="Optional YAML config override.")
    config_args, _ = config_parser.parse_known_args(argv)
    settings = load_settings(config_args.config)
    validation_config = settings["validation"]

    parser = argparse.ArgumentParser(
        description="Run vehicle model validation on the example TMS dataset.",
        parents=[config_parser],
    )
    parser.add_argument("--model", choices=validation_config["allowed_models"], default=validation_config["default_model"])
    parser.add_argument("--base-step", type=int, default=validation_config["base_step"])
    parser.add_argument("--vx-start", type=float, default=validation_config["vx_start"])
    parser.add_argument("--max-rows", type=int, default=validation_config["max_rows"], help="Limit loaded CSV rows.")
    parser.add_argument("--whole-track", action="store_true", default=validation_config["plot"]["whole_track"])
    parser.add_argument("--focus", dest="whole_track", action="store_false")
    parser.add_argument("--no-show", action="store_true", help="Run setup and simulation without opening the animation window.")
    return parser.parse_args(argv), settings


def main(argv=None):
    args, settings = _parse_args(argv)
    validation_config = settings["validation"]
    data_config = validation_config["data"]
    data_schema = validation_config["data_schema"]
    plot_config = validation_config["plot"]
    plan_view_config = settings["visualization"]["plan_view"]

    bounds_path = PACKAGE_ROOT / data_config["racetracks_dir"] / data_config["racetrack"]
    pit_np = np.loadtxt(bounds_path / data_config["pitlane"], delimiter=data_schema["delimiter"], dtype=np.float64)
    left_np = np.loadtxt(bounds_path / data_config["left_bound"], delimiter=data_schema["delimiter"], dtype=np.float64)
    right_np = np.loadtxt(bounds_path / data_config["right_bound"], delimiter=data_schema["delimiter"], dtype=np.float64)

    validation_vehicle = Vehicle(vehicle_params_from_config(args.config))
    validation_vehicle.gen_data(
        PACKAGE_ROOT / data_config["example_runs_dir"],
        data_config["dataset"],
        data_limit=args.max_rows,
        config_path=args.config,
    )
    simulation = Simulation(
        validation_vehicle,
        args.model,
        sample_time=validation_config["sample_time"],
        base_horizon=validation_config["base_horizon"],
        average_wheel_speeds=validation_config["average_wheel_speeds"],
    )
    adaptive_config = validation_config["adaptive_horizon"]
    simulation.open_sim(
        validation_config["multi_step"],
        args.base_step,
        adaptive_config["c1_scaler"],
        adaptive_config["c2_scaler"],
        validation_config["horizon_fixed"],
        adaptive_config["min_step"],
    )
    plotting = Plotting(
        simulation,
        args.vx_start,
        left_np,
        right_np,
        pit_np,
        args.whole_track,
        plot_config=plot_config,
        plan_view_config=plan_view_config,
    )
    if args.no_show:
        plt.close(plotting.fig)
    else:
        plotting.animate()


if __name__ == "__main__":
    main()
