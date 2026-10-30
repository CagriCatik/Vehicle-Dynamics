# Rückverfolgbarkeit: Planpunkte → Themen, Module, Dokumente

Code, Konfigurationen, Tests, Dokumente und Ergebnisordner sind nach **Themen** benannt, nicht nach Planversionen
oder Meilensteinnummern. Diese Tabelle ist die einzige Stelle, die die Planpunkte der drei Pläne in
[`docs/plans/`](plans/) den heutigen Namen zuordnet.

## Plan für integrierte Bewegungsregelung ([integrated_motion_control_plan.md](plans/integrated_motion_control_plan.md))

| Planpunkt | Thema | Hauptmodule | Experiment-Konfigurationen | Dokument | Tests | Ergebnisse |
|---|---|---|---|---|---|---|
| M16 | Stellgrößenkette, Aktuatordynamik | `simulation/actuator_chain.py`, `controllers/motion_control.py` | `actuator_dynamics_split_mu_braking` | [25_actuation_chain.md](25_actuation_chain.md) | `test_actuation_chain.py` | `output/actuation_chain/` |
| M17 | Bremsfunktionen (EBD, CBC, IBC, Rekuperations-Schlupfregelung) | `controllers/brake_distribution.py`, `cornering_brake.py`, `integrated_brake.py`, `regenerative_slip.py` | `brake_functions_*`, `regen_slip_*` | [26_brake_functions.md](26_brake_functions.md) | `test_brake_functions.py` | `output/brake_functions/` |
| M18 | Traktionsregelung (eLSD, Motorhüllkurve, ITC) | `controllers/traction_distribution.py`, `integrated_traction.py` | `traction_*` | [27_traction_control.md](27_traction_control.md) | `test_traction_control.py` | `output/traction_control/` |
| M19 | Wankstabilität (RSC, 14-DOF) und aktive Vorderachslenkung (AFS) | `controllers/roll_stability.py`, `active_front_steering.py`, `simulation/roll_stability.py` | `roll_stability_fishhook_evasive`, `active_front_steering_*` | [28_roll_stability_active_steering.md](28_roll_stability_active_steering.md) | `test_roll_stability_active_steering.py` | `output/roll_stability/`, `output/active_front_steering/` |
| M20 | G-Vectoring (GVC-artige Lasttransferregelung) | `controllers/g_vectoring.py` | `g_vectoring_*` | [29_g_vectoring.md](29_g_vectoring.md) | `test_g_vectoring.py` | `output/g_vectoring/` |
| M21 | Vorschaudämpfung, aktive Nickregelung, vertikale Kraftallokation | `suspension/preview_control.py`, `allocation/vertical_forces.py`, `simulation/ride.py` | `ride_preview_damping_active_pitch` | [30_preview_damping_active_pitch.md](30_preview_damping_active_pitch.md) | `test_preview_damping_active_pitch.py` | `output/ride_control/` |
| M22 | Straßenneigung, Hill Descent Control | `roads/surface.py` (`GradedRoad`), `controllers/hill_descent.py` | `hill_descent_control` | [31_hill_descent_control.md](31_hill_descent_control.md) | `test_hill_descent_control.py` | `output/hill_descent/` |
| M23 | Fehlertoleranz (Fehlerinjektion, Health-Monitor, degradierte Modi) | `simulation/fault_injection.py`, `controllers/fault_tolerance.py` | `fault_tolerance_catalogue` | [32_fault_tolerance.md](32_fault_tolerance.md) | `test_fault_tolerance.py` | `output/fault_tolerance/` |
| M24 | Bewegungsmanagement (Funktionsregister, Arbitrierung, Benchmark-Matrix) | `controllers/motion_supervisor.py`, `experiments/motion_management_benchmark.py` | `motion_management_matrix` | [33_motion_management_benchmark.md](33_motion_management_benchmark.md) | `test_motion_management.py` | `output/motion_management/` |

Konfigurationsstufen der Benchmark-Matrix (im Plan P0–P7): `passive`, `esc`, `tv_4ws`, `brake_functions`,
`traction`, `afs_gvc_rsc`, `ride_control`, `fault_tolerant`.

## Plattformplan ([chassis_control_platform_plan.md](plans/chassis_control_platform_plan.md))

| Planpunkt | Thema | Dokument |
|---|---|---|
| M0–M3 | Modell-Audit, Referenzfahrzeug, 7-DOF-Modell, Reifenmodelle | [41_audit.md](41_audit.md), [02_vehicle_models.md](02_vehicle_models.md), [03_tyre_models.md](03_tyre_models.md) |
| M4 | Fahrwerkskinematik | [04_chassis_kinematics.md](04_chassis_kinematics.md) |
| M5 | Schlupfregelung und Allokation | [10_control_allocation.md](10_control_allocation.md) |
| M6 | Reglerbibliothek, Benchmark | [11_motion_control.md](11_motion_control.md) |
| M7 | Schätzrobustheit | [19_estimation_robustness.md](19_estimation_robustness.md) |
| M8 | Vertikaldynamik | [08_vertical_dynamics.md](08_vertical_dynamics.md) |
| M9 | Experiment-Engine | [34_experiments.md](34_experiments.md) |
| M10 | Zustandsschätzung, Baseline-Abgleich | [18_state_estimation.md](18_state_estimation.md), [42_baseline_audit.md](42_baseline_audit.md) |
| M11 | Nichtlineare Stabilität | [20_nonlinear_stability.md](20_nonlinear_stability.md) |
| M12 | Robuste Regelung | [21_robust_control.md](21_robust_control.md) |
| M13 | Suche kritischer Szenarien | [36_critical_scenarios.md](36_critical_scenarios.md) |
| M14 | Auslegungsoptimierung | [37_design_optimization.md](37_design_optimization.md) |
| M15 | Workbench | [40_workbench.md](40_workbench.md) |

## Drift-Regler-Plan ([drift_controller_plan.md](plans/drift_controller_plan.md))

| Planpunkt | Thema | Dokument |
|---|---|---|
| D0, D1 | Drift-Audit, Reifenaudit, deterministische Auswertung | [22_drift_model_validity.md](22_drift_model_validity.md) |
| D2–D5 | Driftgleichgewichte, Drift-LQR, Einleitung/Rückkehr, Bahnführung | [23_drift_controller.md](23_drift_controller.md) |
| D6, D7 | Animation und Export, Robustheit | [24_drift_results.md](24_drift_results.md) |
| D8 | optionales Drift-NMPC | nicht umgesetzt (zurückgestellt) |
