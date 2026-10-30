# Scenario book

Generated from `src/vehicle_model/scenario_book/catalog.py` (`python -m vehicle_model.scenario_book book --write docs/35_scenario_book.md`). Do not edit by hand.

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
| `exp_active_front_steering_sine_with_dwell` | **YAML experiment active_front_steering_sine_with_dwell** – Runs configs/experiments/active_front_steering_sine_with_dwell.yaml (edit the YAML to change vehicle, manoeuvre, controllers, sweep). | – | ~1 min | report.md, kpis.csv, results.json, histories, plots |
| `exp_active_front_steering_step_steer` | **YAML experiment active_front_steering_step_steer** – Runs configs/experiments/active_front_steering_step_steer.yaml (edit the YAML to change vehicle, manoeuvre, controllers, sweep). | – | ~1 min | report.md, kpis.csv, results.json, histories, plots |
| `exp_actuator_dynamics_split_mu_braking` | **YAML experiment actuator_dynamics_split_mu_braking** – Runs configs/experiments/actuator_dynamics_split_mu_braking.yaml (edit the YAML to change vehicle, manoeuvre, controllers, sweep). | – | ~1 min | report.md, kpis.csv, results.json, histories, plots |
| `exp_brake_functions_brake_in_turn` | **YAML experiment brake_functions_brake_in_turn** – Runs configs/experiments/brake_functions_brake_in_turn.yaml (edit the YAML to change vehicle, manoeuvre, controllers, sweep). | – | ~1 min | report.md, kpis.csv, results.json, histories, plots |
| `exp_brake_functions_brake_in_turn_low_mu` | **YAML experiment brake_functions_brake_in_turn_low_mu** – Runs configs/experiments/brake_functions_brake_in_turn_low_mu.yaml (edit the YAML to change vehicle, manoeuvre, controllers, sweep). | – | ~1 min | report.md, kpis.csv, results.json, histories, plots |
| `exp_brake_functions_mu_step` | **YAML experiment brake_functions_mu_step** – Runs configs/experiments/brake_functions_mu_step.yaml (edit the YAML to change vehicle, manoeuvre, controllers, sweep). | – | ~1 min | report.md, kpis.csv, results.json, histories, plots |
| `exp_brake_functions_split_mu` | **YAML experiment brake_functions_split_mu** – Runs configs/experiments/brake_functions_split_mu.yaml (edit the YAML to change vehicle, manoeuvre, controllers, sweep). | – | ~1 min | report.md, kpis.csv, results.json, histories, plots |
| `exp_brake_functions_straight_braking` | **YAML experiment brake_functions_straight_braking** – Runs configs/experiments/brake_functions_straight_braking.yaml (edit the YAML to change vehicle, manoeuvre, controllers, sweep). | – | ~1 min | report.md, kpis.csv, results.json, histories, plots |
| `exp_g_vectoring_emergency_braking` | **YAML experiment g_vectoring_emergency_braking** – Runs configs/experiments/g_vectoring_emergency_braking.yaml (edit the YAML to change vehicle, manoeuvre, controllers, sweep). | – | ~1 min | report.md, kpis.csv, results.json, histories, plots |
| `exp_g_vectoring_lane_change` | **YAML experiment g_vectoring_lane_change** – Runs configs/experiments/g_vectoring_lane_change.yaml (edit the YAML to change vehicle, manoeuvre, controllers, sweep). | – | ~1 min | report.md, kpis.csv, results.json, histories, plots |
| `exp_g_vectoring_lift_off` | **YAML experiment g_vectoring_lift_off** – Runs configs/experiments/g_vectoring_lift_off.yaml (edit the YAML to change vehicle, manoeuvre, controllers, sweep). | – | ~1 min | report.md, kpis.csv, results.json, histories, plots |
| `exp_g_vectoring_steer_ramp` | **YAML experiment g_vectoring_steer_ramp** – Runs configs/experiments/g_vectoring_steer_ramp.yaml (edit the YAML to change vehicle, manoeuvre, controllers, sweep). | – | ~1 min | report.md, kpis.csv, results.json, histories, plots |
| `exp_regen_slip_braking` | **YAML experiment regen_slip_braking** – Runs configs/experiments/regen_slip_braking.yaml (edit the YAML to change vehicle, manoeuvre, controllers, sweep). | – | ~1 min | report.md, kpis.csv, results.json, histories, plots |
| `exp_regen_slip_coast_low_mu` | **YAML experiment regen_slip_coast_low_mu** – Runs configs/experiments/regen_slip_coast_low_mu.yaml (edit the YAML to change vehicle, manoeuvre, controllers, sweep). | – | ~1 min | report.md, kpis.csv, results.json, histories, plots |
| `exp_sine_with_dwell_comparison` | **YAML experiment sine_with_dwell_comparison** – Runs configs/experiments/sine_with_dwell_comparison.yaml (edit the YAML to change vehicle, manoeuvre, controllers, sweep). | – | ~1 min | report.md, kpis.csv, results.json, histories, plots |
| `exp_split_mu_braking` | **YAML experiment split_mu_braking** – Runs configs/experiments/split_mu_braking.yaml (edit the YAML to change vehicle, manoeuvre, controllers, sweep). | – | ~1 min | report.md, kpis.csv, results.json, histories, plots |
| `exp_step_steer_speed_sweep` | **YAML experiment step_steer_speed_sweep** – Runs configs/experiments/step_steer_speed_sweep.yaml (edit the YAML to change vehicle, manoeuvre, controllers, sweep). | – | ~1 min | report.md, kpis.csv, results.json, histories, plots |
| `exp_traction_corner_exit` | **YAML experiment traction_corner_exit** – Runs configs/experiments/traction_corner_exit.yaml (edit the YAML to change vehicle, manoeuvre, controllers, sweep). | – | ~1 min | report.md, kpis.csv, results.json, histories, plots |
| `exp_traction_motor_faults` | **YAML experiment traction_motor_faults** – Runs configs/experiments/traction_motor_faults.yaml (edit the YAML to change vehicle, manoeuvre, controllers, sweep). | – | ~1 min | report.md, kpis.csv, results.json, histories, plots |
| `exp_traction_mu_step_launch` | **YAML experiment traction_mu_step_launch** – Runs configs/experiments/traction_mu_step_launch.yaml (edit the YAML to change vehicle, manoeuvre, controllers, sweep). | – | ~1 min | report.md, kpis.csv, results.json, histories, plots |
| `exp_traction_split_mu_launch` | **YAML experiment traction_split_mu_launch** – Runs configs/experiments/traction_split_mu_launch.yaml (edit the YAML to change vehicle, manoeuvre, controllers, sweep). | – | ~1 min | report.md, kpis.csv, results.json, histories, plots |

## solver

| name | scenario | parameters (defaults) | runtime | outputs |
|---|---|---|---|---|
| `exp_dynamic_nmpc_solver_benchmark` | **Solver benchmark dynamic_nmpc_solver_benchmark** – Runs configs/experiments/dynamic_nmpc_solver_benchmark.yaml (edit the YAML to change the NMPC variants, manoeuvre, road). | – | ~2 min | report.md, kpis.csv, results.json, histories, plot |

## nonlinear_stability

| name | scenario | parameters (defaults) | runtime | outputs |
|---|---|---|---|---|
| `exp_equilibrium_speed_stiffness` | **Equilibrium analysis equilibrium_speed_stiffness** – Runs configs/experiments/equilibrium_speed_stiffness.yaml (edit the YAML to change sweep, series, vehicle, tyres, road). | – | ~1-3 min | report.md, equilibria.csv, results.json, equilibria and eigenvalue plots |
| `exp_equilibrium_steer_sweep` | **Equilibrium analysis equilibrium_steer_sweep** – Runs configs/experiments/equilibrium_steer_sweep.yaml (edit the YAML to change sweep, series, vehicle, tyres, road). | – | ~1-3 min | report.md, equilibria.csv, results.json, equilibria and eigenvalue plots |
| `exp_equilibrium_tv_4ws` | **Equilibrium analysis equilibrium_tv_4ws** – Runs configs/experiments/equilibrium_tv_4ws.yaml (edit the YAML to change sweep, series, vehicle, tyres, road). | – | ~1-3 min | report.md, equilibria.csv, results.json, equilibria and eigenvalue plots |
| `exp_stability_map_cornering` | **Stability map stability_map_cornering** – Runs configs/experiments/stability_map_cornering.yaml (edit the YAML to change grid, operating point, controllers). | – | ~10-20 min (parallel) | report.md, map.csv, results.json, maps, portraits, limit cases |
| `exp_stability_map_rear_steer` | **Stability map stability_map_rear_steer** – Runs configs/experiments/stability_map_rear_steer.yaml (edit the YAML to change grid, operating point, controllers). | – | ~10-20 min (parallel) | report.md, map.csv, results.json, maps, portraits, limit cases |
| `exp_stability_map_robust` | **Stability map stability_map_robust** – Runs configs/experiments/stability_map_robust.yaml (edit the YAML to change grid, operating point, controllers). | – | ~10-20 min (parallel) | report.md, map.csv, results.json, maps, portraits, limit cases |

## robust_control

| name | scenario | parameters (defaults) | runtime | outputs |
|---|---|---|---|---|
| `exp_robust_benchmark_sine_with_dwell` | **Robust benchmark robust_benchmark_sine_with_dwell** – Runs configs/experiments/robust_benchmark_sine_with_dwell.yaml (edit the YAML to change controllers, feedback modes, uncertainty cases). | – | ~3-5 min (parallel) | report.md, kpis.csv, results.json, KPI matrices, yaw-rate histories |

## critical_scenarios

| name | scenario | parameters (defaults) | runtime | outputs |
|---|---|---|---|---|
| `exp_critical_search_sine_with_dwell` | **Critical scenario search critical_search_sine_with_dwell** – Runs configs/experiments/critical_search_sine_with_dwell.yaml (edit the YAML to change parameter space, constraints, criterion, search). | – | ~5-10 min (parallel search, serial verification) | report.md, evaluations.csv, results.json, search.png, critical/<id>/ (replay YAML, metadata, CSV, PNG, GIF, report) |

## design

| name | scenario | parameters (defaults) | runtime | outputs |
|---|---|---|---|---|
| `exp_design_chassis_tv` | **Design optimisation design_chassis_tv** – Runs configs/experiments/design_chassis_tv.yaml (edit the YAML to change design parameters, objectives, scenario sets, search). | – | ~5-10 min (parallel) | report.md, evaluations.csv, pareto_designs.csv, design_<label>.yaml, Pareto fronts, distributions, sensitivities, results.json |

## roll_stability

| name | scenario | parameters (defaults) | runtime | outputs |
|---|---|---|---|---|
| `exp_roll_stability_fishhook_evasive` | **Roll stability benchmark roll_stability_fishhook_evasive** – Runs configs/experiments/roll_stability_fishhook_evasive.yaml (edit the YAML to change variants, manoeuvres, friction, controllers). | – | ~3-5 min (parallel, 14-DOF at 1 ms) | report.md, kpis.csv, results.json, histories, rollover_timeseries.png |

## ride_comfort

| name | scenario | parameters (defaults) | runtime | outputs |
|---|---|---|---|---|
| `exp_ride_preview_damping_active_pitch` | **Ride and pitch benchmark ride_preview_damping_active_pitch** – Runs configs/experiments/ride_preview_damping_active_pitch.yaml (edit the YAML to change roads, manoeuvres, damping modes, preview, active pitch). | – | ~1-2 min (parallel, 14-DOF at 1 ms) | report.md, kpis.csv, results.json, histories |

## hill_descent

| name | scenario | parameters (defaults) | runtime | outputs |
|---|---|---|---|---|
| `exp_hill_descent_control` | **Hill descent benchmark hill_descent_control** – Runs configs/experiments/hill_descent_control.yaml (edit the YAML to change grades, surfaces, named cases, controllers). | – | ~10 min (parallel, 7-DOF with road grade) | report.md, kpis.csv, results.json, histories |

## fault_tolerance

| name | scenario | parameters (defaults) | runtime | outputs |
|---|---|---|---|---|
| `exp_fault_tolerance_catalogue` | **Fault tolerance benchmark fault_tolerance_catalogue** – Runs configs/experiments/fault_tolerance_catalogue.yaml (edit the YAML to change manoeuvres, fault sets, controllers). | – | ~5-10 min (parallel, 7-DOF) | report.md, kpis.csv, results.json, histories |

## integration

| name | scenario | parameters (defaults) | runtime | outputs |
|---|---|---|---|---|
| `exp_motion_management_matrix` | **Vehicle motion benchmark matrix motion_management_matrix** – Runs configs/experiments/motion_management_matrix.yaml (edit the YAML to change levels, manoeuvres, paired variations). | – | ~25 min (parallel, 7-DOF and 14-DOF) | report.md, kpis.csv, stats.csv, results.json, matrix.png, histories |

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

