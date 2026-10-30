# Vehicle Dynamics Documentation Index

Dieser Ordner enthält die vollständige wissenschaftliche und technische Dokumentation der **Vehicle Dynamics & Integrated Chassis Control Development Platform**.

> [!TIP]
> **Zentraler Einstiegspunkt:**
> Das vollständige Inhaltsverzeichnis mit Roadmap, thematischem Register und A–Z-Stichwortindex befindet sich in:
> 👉 **[docs/index.md](index.md)**

---

## Schnelleinstieg nach Kapiteln

| Nr. | Thema | Dokument |
| :---: | :--- | :--- |
| **01** | Systemarchitektur & ISO 8855 Koordinaten | [ARCHITECTURE.md](ARCHITECTURE.md) |
| **02** | Mehrkörper-Fahrzeugmodelle (7-DOF & 14-DOF) | [VEHICLE_MODELS.md](VEHICLE_MODELS.md) |
| **03** | Reifenmodelle (Fiala, MF 5.2-Teilmenge, Combined Slip, Relaxation) | [TYRE_MODELS.md](TYRE_MODELS.md) |
| **04** | Aktorik, EPS & Brake-by-Wire | [ACTUATORS.md](ACTUATORS.md) |
| **05** | Integrierte Regelung (IVMC, TV, Phasenraum) | [CHASSIS_CONTROL.md](CHASSIS_CONTROL.md) |
| **06** | **Allradlenkung & Hinterachslenkung (4WS)** | [REAR_WHEEL_STEERING.md](REAR_WHEEL_STEERING.md) |
| **07** | **Nichtlineares MPC & Pfadverfolgung (SLSQP)** | [NMPC_CONTROL.md](NMPC_CONTROL.md) & [NMPC_PATH_TRACKING.md](NMPC_PATH_TRACKING.md) |
| **08** | Neuromuskuläres Preview-Fahrermodell | [DRIVER_MODEL.md](DRIVER_MODEL.md) |
| **09** | Vertikal- & Wankdynamik (Skyhook / ARSDC) | [SUSPENSION_DYNAMICS.md](SUSPENSION_DYNAMICS.md) |
| **10** | Zustandsschätzung (EKF, RLS, PSO) | [STATE_ESTIMATION.md](STATE_ESTIMATION.md) |
| **11** | Virtuelle Validierungsplattform (ISO / ECE) | [VALIDATION_PLATFORM.md](VALIDATION_PLATFORM.md) |
| **12** | Fahrzeughandling & Eigenlenkgradient (ISO 4138, lineare Referenz) | [HANDLING_LIMITS.md](HANDLING_LIMITS.md) & [HANDLING_ANALYSIS.md](HANDLING_ANALYSIS.md) |
| **13** | Split-Mu Dynamik & Multi-Mu Benchmark-Matrix | [SPLIT_MU_DYNAMICS.md](SPLIT_MU_DYNAMICS.md) |
| **14** | Visualisierung, Telemetrie & GIF-Engine | [VISUALIZATION_GUIDE.md](VISUALIZATION_GUIDE.md) |
| **15** | Literatur- und Quellenverzeichnis | [REFERENCES.md](REFERENCES.md) |
| **16** | Lenkgeometrie, Fahrwerks-K&C, Rollzentren, Eigenlenkbudget | [CHASSIS_KINEMATICS.md](CHASSIS_KINEMATICS.md) |
| **17** | Schlupfregelung, Control Allocation, ESC/TV | [CONTROL_ALLOCATION.md](CONTROL_ALLOCATION.md) |
| **18** | Reglerbibliothek (PI, LQR, GS-LQR, LTV-MPC), Solverdiagnostik, Benchmark | [MOTION_CONTROL.md](MOTION_CONTROL.md) |
| **19** | Virtuelle Sensoren, Schwimmwinkelbeobachter, Monte-Carlo-Robustheit | [ESTIMATION_ROBUSTNESS.md](ESTIMATION_ROBUSTNESS.md) |
| **20** | Vertikaldynamik: Verifikation, Frequenzgänge, ISO-8608-Straßen | [VERTICAL_DYNAMICS.md](VERTICAL_DYNAMICS.md) |
| **21** | Virtuelles Versuchslabor: YAML-Experimente, Verifikation, Berichte, CI | [EXPERIMENTS.md](EXPERIMENTS.md) |
| **22** | Plattformplan: Abgleich mit dem Code (Baseline, G01–G07) | [BASELINE_AUDIT.md](BASELINE_AUDIT.md) |
| **23** | Nichtlineare Stabilität: Gleichgewichte, Eigenwerte, Rückführregionen | [NONLINEAR_STABILITY.md](NONLINEAR_STABILITY.md) |
| **24** | Robuste und adaptive Regelung, Überwachung, Robustheitsbenchmark | [ROBUST_CONTROL.md](ROBUST_CONTROL.md) |
| **25** | Drift-Regler: Modellgültigkeit, Konventionen, Reifenaudit, deterministische Auswertung | [DRIFT_MODEL_VALIDITY.md](DRIFT_MODEL_VALIDITY.md) |
| **26** | Drift-Regler: Verfahren (Gleichgewichte, Linearisierung, LQR, Halten) | [DRIFT_CONTROLLER.md](DRIFT_CONTROLLER.md) |
| **27** | Drift-Regler: Ergebnisse und Gate-Abnahme | [DRIFT_RESULTS.md](DRIFT_RESULTS.md) |
| **28** | Adversarial Virtual Testing: kritische Szenarien suchen, prüfen, minimieren, als Regressionstest einfrieren | [CRITICAL_SCENARIOS.md](CRITICAL_SCENARIOS.md) |
| **29** | Fahrzeugauslegung und Mehrzieloptimierung: NSGA-II, Pareto-Analyse, unabhängige Nachprüfung | [DESIGN_OPTIMIZATION.md](DESIGN_OPTIMIZATION.md) |
| **30** | Interactive Engineering Workbench: Weboberfläche über der Experiment-Engine | [WORKBENCH.md](WORKBENCH.md) |
| **31** | Stellgrößenkette: Ausgangsstand, Blocker A–H, Aktuatordynamik | [ACTUATION_CHAIN.md](ACTUATION_CHAIN.md) |
| **32** | EBD, CBC, IBC, regenerative Schlupfregelung (MSR) | [BRAKE_FUNCTIONS.md](BRAKE_FUNCTIONS.md) |
| **33** | virtuelles eLSD, Traktionsverteilung, Motorhüllkurve, Integrated Traction Control | [TRACTION_CONTROL.md](TRACTION_CONTROL.md) |
| **34** | Roll Stability Control (14-DOF) und Active Front Steering | [ROLL_STABILITY_ACTIVE_STEERING.md](ROLL_STABILITY_ACTIVE_STEERING.md) |
| **35** | GVC-artige Lasttransferregelung (eigene Referenzstrategie) | [G_VECTORING.md](G_VECTORING.md) |
| **36** | prädiktive Dämpfung, aktive Nickregelung, vertikale Kraftallokation (14-DOF) | [PREVIEW_DAMPING_ACTIVE_PITCH.md](PREVIEW_DAMPING_ACTIVE_PITCH.md) |
| **37** | Straßenneigung im Plant, Hill Descent Control | [HILL_DESCENT_CONTROL.md](HILL_DESCENT_CONTROL.md) |
| **38** | Fehlerkatalog, Health-Monitor, degradierte Modi | [FAULT_TOLERANCE.md](FAULT_TOLERANCE.md) |
| **39** | Vehicle Motion Supervisor, Arbitrierung, Benchmark-Matrix passive bis fault_tolerant | [MOTION_MANAGEMENT_BENCHMARK.md](MOTION_MANAGEMENT_BENCHMARK.md) |
| **40** | Rückverfolgbarkeit: Planpunkte → Themen, Module, Dokumente | [TRACEABILITY.md](TRACEABILITY.md) |
