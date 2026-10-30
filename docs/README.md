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
| **16** | Lenkgeometrie, Fahrwerks-K&C, Rollzentren, Eigenlenkbudget (M4) | [CHASSIS_KINEMATICS.md](CHASSIS_KINEMATICS.md) |
| **17** | Schlupfregelung, Control Allocation, ESC/TV (M5) | [CONTROL_ALLOCATION.md](CONTROL_ALLOCATION.md) |
| **18** | Reglerbibliothek (PI, LQR, GS-LQR, LTV-MPC), Solverdiagnostik, Benchmark (M6) | [MOTION_CONTROL.md](MOTION_CONTROL.md) |
| **19** | Virtuelle Sensoren, Schwimmwinkelbeobachter, Monte-Carlo-Robustheit (M7) | [ESTIMATION_ROBUSTNESS.md](ESTIMATION_ROBUSTNESS.md) |
| **20** | Vertikaldynamik: Verifikation, Frequenzgänge, ISO-8608-Straßen (M8) | [VERTICAL_DYNAMICS.md](VERTICAL_DYNAMICS.md) |
| **21** | Virtuelles Versuchslabor: YAML-Experimente, Verifikation, Berichte, CI (M9) | [EXPERIMENTS.md](EXPERIMENTS.md) |
| **22** | Masterplan V3.0: Abgleich mit dem Code (Baseline, G01–G07, Reihenfolge M10) | [V3_BASELINE.md](V3_BASELINE.md) |
| **23** | Nichtlineare Stabilität: Gleichgewichte, Eigenwerte, Rückführregionen (M11) | [NONLINEAR_STABILITY.md](NONLINEAR_STABILITY.md) |
| **24** | Robuste und adaptive Regelung, Überwachung, Robustheitsbenchmark (M12) | [ROBUST_CONTROL.md](ROBUST_CONTROL.md) |
| **25** | Drift-Regler: Modellgültigkeit, Konventionen, Reifenaudit, deterministische Auswertung (Drift-Plan D0/D1) | [DRIFT_MODEL_VALIDITY.md](DRIFT_MODEL_VALIDITY.md) |
| **26** | Drift-Regler: Verfahren (Gleichgewichte, Linearisierung, LQR, Halten) | [DRIFT_CONTROLLER.md](DRIFT_CONTROLLER.md) |
| **27** | Drift-Regler: Ergebnisse und Gate-Abnahme | [DRIFT_RESULTS.md](DRIFT_RESULTS.md) |
| **28** | Adversarial Virtual Testing: kritische Szenarien suchen, prüfen, minimieren, als Regressionstest einfrieren (M13) | [CRITICAL_SCENARIOS.md](CRITICAL_SCENARIOS.md) |
| **29** | Fahrzeugauslegung und Mehrzieloptimierung: NSGA-II, Pareto-Analyse, unabhängige Nachprüfung (M14) | [DESIGN_OPTIMIZATION.md](DESIGN_OPTIMIZATION.md) |
| **30** | Interactive Engineering Workbench: Weboberfläche über der Experiment-Engine (M15) | [WORKBENCH.md](WORKBENCH.md) |
| **31** | V4.0: Ausgangsstand, Blocker A–H, M16 Stellgrößenkette und Aktuatordynamik | [V4_BASELINE.md](V4_BASELINE.md) |
| **32** | V4.0 M17: EBD, CBC, IBC, regenerative Schlupfregelung (MSR) | [V4_BRAKING.md](V4_BRAKING.md) |
