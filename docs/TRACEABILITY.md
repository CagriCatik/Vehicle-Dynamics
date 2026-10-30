# Rückverfolgbarkeit: Planpunkte → Themen, Module, Dokumente

Code, Konfigurationen, Tests, Dokumente und Ergebnisordner sind nach **Themen** benannt, nicht nach Planversionen
oder Meilensteinnummern. Diese Tabelle ist die einzige Stelle, die die Planpunkte der drei Pläne in
[`docs/plans/`](plans/) den heutigen Namen zuordnet.

## Plan für integrierte Bewegungsregelung ([integrated_motion_control_plan.md](plans/integrated_motion_control_plan.md))

| Planpunkt | Thema | Hauptmodule | Experiment-Konfigurationen | Dokument | Tests | Ergebnisse |
|---|---|---|---|---|---|---|
| M16 | Stellgrößenkette, Aktuatordynamik | `simulation/actuator_chain.py`, `controllers/motion_control.py` | `actuator_dynamics_split_mu_braking` | [ACTUATION_CHAIN.md](ACTUATION_CHAIN.md) | `test_actuation_chain.py` | `output/actuation_chain/` |
| M17 | Bremsfunktionen (EBD, CBC, IBC, Rekuperations-Schlupfregelung) | `controllers/brake_distribution.py`, `cornering_brake.py`, `integrated_brake.py`, `regenerative_slip.py` | `brake_functions_*`, `regen_slip_*` | [BRAKE_FUNCTIONS.md](BRAKE_FUNCTIONS.md) | `test_brake_functions.py` | `output/brake_functions/` |
| M18 | Traktionsregelung (eLSD, Motorhüllkurve, ITC) | `controllers/traction_distribution.py`, `integrated_traction.py` | `traction_*` | [TRACTION_CONTROL.md](TRACTION_CONTROL.md) | `test_traction_control.py` | `output/traction_control/` |
| M19 | Wankstabilität (RSC, 14-DOF) und aktive Vorderachslenkung (AFS) | `controllers/roll_stability.py`, `active_front_steering.py`, `simulation/roll_stability.py` | `roll_stability_fishhook_evasive`, `active_front_steering_*` | [ROLL_STABILITY_ACTIVE_STEERING.md](ROLL_STABILITY_ACTIVE_STEERING.md) | `test_roll_stability_active_steering.py` | `output/roll_stability/`, `output/active_front_steering/` |
| M20 | G-Vectoring (GVC-artige Lasttransferregelung) | `controllers/g_vectoring.py` | `g_vectoring_*` | [G_VECTORING.md](G_VECTORING.md) | `test_g_vectoring.py` | `output/g_vectoring/` |
| M21 | Vorschaudämpfung, aktive Nickregelung, vertikale Kraftallokation | `suspension/preview_control.py`, `allocation/vertical_forces.py`, `simulation/ride.py` | `ride_preview_damping_active_pitch` | [PREVIEW_DAMPING_ACTIVE_PITCH.md](PREVIEW_DAMPING_ACTIVE_PITCH.md) | `test_preview_damping_active_pitch.py` | `output/ride_control/` |
| M22 | Straßenneigung, Hill Descent Control | `roads/surface.py` (`GradedRoad`), `controllers/hill_descent.py` | `hill_descent_control` | [HILL_DESCENT_CONTROL.md](HILL_DESCENT_CONTROL.md) | `test_hill_descent_control.py` | `output/hill_descent/` |
| M23 | Fehlertoleranz (Fehlerinjektion, Health-Monitor, degradierte Modi) | `simulation/fault_injection.py`, `controllers/fault_tolerance.py` | `fault_tolerance_catalogue` | [FAULT_TOLERANCE.md](FAULT_TOLERANCE.md) | `test_fault_tolerance.py` | `output/fault_tolerance/` |
| M24 | Bewegungsmanagement (Funktionsregister, Arbitrierung, Benchmark-Matrix) | `controllers/motion_supervisor.py`, `experiments/motion_management_benchmark.py` | `motion_management_matrix` | [MOTION_MANAGEMENT_BENCHMARK.md](MOTION_MANAGEMENT_BENCHMARK.md) | `test_motion_management.py` | `output/motion_management/` |

Konfigurationsstufen der Benchmark-Matrix (im Plan P0–P7): `passive`, `esc`, `tv_4ws`, `brake_functions`,
`traction`, `afs_gvc_rsc`, `ride_control`, `fault_tolerant`.

## Plattformplan ([chassis_control_platform_plan.md](plans/chassis_control_platform_plan.md))

| Planpunkt | Thema | Dokument |
|---|---|---|
| M0–M3 | Modell-Audit, Referenzfahrzeug, 7-DOF-Modell, Reifenmodelle | [AUDIT.md](AUDIT.md), [VEHICLE_MODELS.md](VEHICLE_MODELS.md), [TYRE_MODELS.md](TYRE_MODELS.md) |
| M4 | Fahrwerkskinematik | [CHASSIS_KINEMATICS.md](CHASSIS_KINEMATICS.md) |
| M5 | Schlupfregelung und Allokation | [CONTROL_ALLOCATION.md](CONTROL_ALLOCATION.md) |
| M6 | Reglerbibliothek, Benchmark | [MOTION_CONTROL.md](MOTION_CONTROL.md) |
| M7 | Schätzrobustheit | [ESTIMATION_ROBUSTNESS.md](ESTIMATION_ROBUSTNESS.md) |
| M8 | Vertikaldynamik | [VERTICAL_DYNAMICS.md](VERTICAL_DYNAMICS.md) |
| M9 | Experiment-Engine | [EXPERIMENTS.md](EXPERIMENTS.md) |
| M10 | Zustandsschätzung, Baseline-Abgleich | [STATE_ESTIMATION.md](STATE_ESTIMATION.md), [BASELINE_AUDIT.md](BASELINE_AUDIT.md) |
| M11 | Nichtlineare Stabilität | [NONLINEAR_STABILITY.md](NONLINEAR_STABILITY.md) |
| M12 | Robuste Regelung | [ROBUST_CONTROL.md](ROBUST_CONTROL.md) |
| M13 | Suche kritischer Szenarien | [CRITICAL_SCENARIOS.md](CRITICAL_SCENARIOS.md) |
| M14 | Auslegungsoptimierung | [DESIGN_OPTIMIZATION.md](DESIGN_OPTIMIZATION.md) |
| M15 | Workbench | [WORKBENCH.md](WORKBENCH.md) |

## Drift-Regler-Plan ([drift_controller_plan.md](plans/drift_controller_plan.md))

| Planpunkt | Thema | Dokument |
|---|---|---|
| D0, D1 | Drift-Audit, Reifenaudit, deterministische Auswertung | [DRIFT_MODEL_VALIDITY.md](DRIFT_MODEL_VALIDITY.md) |
| D2–D5 | Driftgleichgewichte, Drift-LQR, Einleitung/Rückkehr, Bahnführung | [DRIFT_CONTROLLER.md](DRIFT_CONTROLLER.md) |
| D6, D7 | Animation und Export, Robustheit | [DRIFT_RESULTS.md](DRIFT_RESULTS.md) |
| D8 | optionales Drift-NMPC | nicht umgesetzt (zurückgestellt) |
