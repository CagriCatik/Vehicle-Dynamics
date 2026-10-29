# Scenario book

Generated from `src/vehicle_model/scenario_book/catalog.py` (`python -m vehicle_model.scenario_book book --write docs/SCENARIO_BOOK.md`). Do not edit by hand.

```bash
python -m vehicle_model.scenario_book list [--category parking]
python -m vehicle_model.scenario_book info u_turn
python -m vehicle_model.scenario_book run u_turn [--set dt=0.005] [--no-gif] [--out DIR]
python -m vehicle_model.scenario_book run --category parking
```

Outputs go to `output/scenario_book/<name>/` unless `--out` is given. All vehicles and parameters are synthetic; results are simulation results, not measurements.

## parking

| name | scenario | parameters (defaults) | runtime | outputs |
|---|---|---|---|---|
| `perpendicular_parking` | **Reverse perpendicular parking (90 deg into a bay)** – Drive past the bay, then reverse at full lock until the car has turned 90 deg, and straighten into the bay. Space need: how far the car sweeps into the aisle and along it. | `dt=0.01` | ~1.5 min with GIF, 20 s without | <name>.gif (plan view + steering panel), <name>.png, summary.json (swept extent, clearances) |
| `u_turn` | **U-turn on a narrow street (full lock)** – Turn around in one move at full lock. Space need: street width (across Y) the body sweeps. | `dt=0.01` | ~1.5 min with GIF, 20 s without | <name>.gif (plan view + steering panel), <name>.png, summary.json (swept extent, clearances) |
| `parallel_exit` | **Pulling out of a tight parallel bay** – Parked in a row; steer to full lock at standstill, pull out at full lock until the car is 2.1 m out, counter-steer back to the original heading, drive on. Space need: the gap to the car ahead that the body needs (required_gap_ahead). | `dt=0.01` | ~1.5 min with GIF, 20 s without | <name>.gif (plan view + steering panel), <name>.png, summary.json (swept extent, clearances) |
| `tight_alley` | **90 deg turn between two narrow alleys** – Forward at walking pace, full lock left until turned 90 deg. Space need: the corner the body sweeps. | `dt=0.01` | ~1.5 min with GIF, 20 s without | <name>.gif (plan view + steering panel), <name>.png, summary.json (swept extent, clearances) |

## four_wheel_steering

| name | scenario | parameters (defaults) | runtime | outputs |
|---|---|---|---|---|
| `4ws_turning_circle` | **Low-speed turning circle: 2WS vs 4WS** – Constant speed, constant front steer: rear counter-steers, the circle shrinks. | `v_kph=12.0`, `front_steer_deg=25.0`, `duration_s=7.0` | ~1 min with GIF | comparison.png (paths + steering panel), animation.gif, summary.json |
| `4ws_urban_corner` | **Urban 90 deg corner: 2WS vs 4WS** – Right-angle turn at a junction with a curb; driver model follows the corner. | `v_kph=15.0`, `duration_s=5.0` | ~1 min with GIF | comparison.png (paths + steering panel), animation.gif, summary.json |
| `4ws_parallel_parking` | **Reverse parallel parking (time-based driver): 2WS vs 4WS** – Reverse S-curve into a bay with a fixed (time-based) steering programme; see parking/ for event-based ones. | `duration_s=7.2` | ~1 min with GIF | comparison.png (paths + steering panel), animation.gif, summary.json |
| `4ws_double_lane_change` | **Double lane change: 2WS vs 4WS** – ISO 3888-2-like obstacle avoidance; rear steers in phase at speed. | `v_kph=80.0`, `duration_s=3.5` | ~1 min with GIF | comparison.png (paths + steering panel), animation.gif, summary.json |
| `4ws_slalom` | **High-speed slalom: 2WS vs 4WS** – Slalom through 30 m cones. | `v_kph=70.0`, `duration_s=4.0` | ~1 min with GIF | comparison.png (paths + steering panel), animation.gif, summary.json |
| `4ws_motorway_lane_change` | **Motorway lane change: 2WS vs 4WS** – Single lane change at motorway speed. | `v_kph=130.0`, `duration_s=3.5` | ~1 min with GIF | comparison.png (paths + steering panel), animation.gif, summary.json |
| `4ws_step_steer` | **Step steer: 2WS vs 4WS** – Step steer input; yaw-rate response and sideslip. | `v_kph=100.0`, `step_steer_deg=3.0`, `duration_s=2.5` | ~1 min with GIF | comparison.png (paths + steering panel), animation.gif, summary.json |
| `4ws_sine_with_dwell` | **Sine with dwell: 2WS vs 4WS** – Sine with dwell modelled on FMVSS 126 (simulation, not the regulatory test). | `v_kph=80.0`, `steer_amp_deg=5.0`, `duration_s=4.5` | ~1 min with GIF | comparison.png (paths + steering panel), animation.gif, summary.json |
| `4ws_split_mu_braking` | **Split-mu braking: 2WS vs 4WS (yaw damping)** – Braking on split friction; rear-steer yaw damping alone cannot prevent the spin (see split_mu_matrix). | `v_kph=100.0`, `mu_left=1.0`, `mu_right=0.2`, `duration_s=3.0` | ~1 min with GIF | comparison.png (paths + steering panel), animation.gif, summary.json |

## split_mu

| name | scenario | parameters (defaults) | runtime | outputs |
|---|---|---|---|---|
| `split_mu_matrix` | **Split-mu braking across actuator configurations** – Six configurations (passive, 4WS, 4WS+ABS, ESC/TV, allocation, IVMC) with the same pedal; which stay straight and why (brake vs steering yaw moment). | `v_kph=100.0`, `mu_high=1.0`, `mu_low=0.2`, `pedal=1.0`, `duration=3.0` | ~30 s | matrix.md, summary.json |
| `split_mu_suite` | **Split-mu scenario suite** – Braking, launch, friction transitions and evasive manoeuvres on split friction for several controllers. | – | ~7 min with GIFs | dashboards, GIFs, reports, summary.json |

## yaw_stability

| name | scenario | parameters (defaults) | runtime | outputs |
|---|---|---|---|---|
| `exp_sine_with_dwell_comparison` | **YAML experiment sine_with_dwell_comparison** – Runs configs/experiments/sine_with_dwell_comparison.yaml (edit the YAML to change vehicle, manoeuvre, controllers, sweep). | – | ~1 min | report.md, kpis.csv, results.json, histories, plots |
| `exp_split_mu_braking` | **YAML experiment split_mu_braking** – Runs configs/experiments/split_mu_braking.yaml (edit the YAML to change vehicle, manoeuvre, controllers, sweep). | – | ~1 min | report.md, kpis.csv, results.json, histories, plots |
| `exp_step_steer_speed_sweep` | **YAML experiment step_steer_speed_sweep** – Runs configs/experiments/step_steer_speed_sweep.yaml (edit the YAML to change vehicle, manoeuvre, controllers, sweep). | – | ~1 min | report.md, kpis.csv, results.json, histories, plots |

## validation

| name | scenario | parameters (defaults) | runtime | outputs |
|---|---|---|---|---|
| `val_iso_3888_dlc` | **Validation matrix: ISO 3888-2 double lane change** – Scenario of configs/validation/matrix.yaml with its controllers and pass/fail thresholds. | – | ~30 s | figures, summary.json (criteria per controller) |
| `val_step_steer_iso_7401` | **Validation matrix: ISO 7401 step steer** – Scenario of configs/validation/matrix.yaml with its controllers and pass/fail thresholds. | – | ~30 s | figures, summary.json (criteria per controller) |
| `val_iso_4138_circle` | **Validation matrix: ISO 4138 steady-state circle** – Scenario of configs/validation/matrix.yaml with its controllers and pass/fail thresholds. | – | ~30 s | figures, summary.json (criteria per controller) |
| `val_low_speed_slalom_4ws` | **Validation matrix: Low-speed slalom with 4WS** – Scenario of configs/validation/matrix.yaml with its controllers and pass/fail thresholds. | – | ~30 s | figures, summary.json (criteria per controller) |
| `val_high_speed_lane_change` | **Validation matrix: High-speed lane change** – Scenario of configs/validation/matrix.yaml with its controllers and pass/fail thresholds. | – | ~30 s | figures, summary.json (criteria per controller) |
| `val_fishhook_ncap` | **Validation matrix: Fishhook (NCAP-like)** – Scenario of configs/validation/matrix.yaml with its controllers and pass/fail thresholds. | – | ~30 s | figures, summary.json (criteria per controller) |
| `val_mu_split_braking` | **Validation matrix: Split-mu braking** – Scenario of configs/validation/matrix.yaml with its controllers and pass/fail thresholds. | – | ~30 s | figures, summary.json (criteria per controller) |
| `val_braking_in_a_turn` | **Validation matrix: Braking in a turn** – Scenario of configs/validation/matrix.yaml with its controllers and pass/fail thresholds. | – | ~30 s | figures, summary.json (criteria per controller) |
| `val_sine_with_dwell` | **Validation matrix: Sine with dwell** – Scenario of configs/validation/matrix.yaml with its controllers and pass/fail thresholds. | – | ~30 s | figures, summary.json (criteria per controller) |
| `val_ride_comfort_bump` | **Validation matrix: Ride comfort over a bump** – Scenario of configs/validation/matrix.yaml with its controllers and pass/fail thresholds. | – | ~30 s | figures, summary.json (criteria per controller) |

