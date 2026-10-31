"""Copy curated result figures from output/ into docs/assets/ and embed them in the documentation.

The figures are produced by the example scripts (output/ is not tracked). This script
picks a curated set per document, shrinks them for the repository (PNG at most
1600 px wide; GIF at half size, every second frame, 128 colours) and writes a
"Ergebnisbilder" block between the markers ``<!-- figures:start -->`` and
``<!-- figures:end -->`` at the end of each document. Rerunning replaces the block.

    python tools/publish_doc_figures.py            # copy + embed
    python tools/publish_doc_figures.py --check    # only list missing sources

Regenerate the sources first (see the "Quelle" in each caption).
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from PIL import Image, ImageSequence

ROOT = Path(__file__).resolve().parents[1]
OUT, DOCS, ASSETS = ROOT / "output", ROOT / "docs", ROOT / "docs" / "assets"
START, END = "<!-- figures:start -->", "<!-- figures:end -->"
NOTE = ("Alle Bilder sind Simulationsergebnisse mit synthetischen Parametern, keine Messungen. "
        "Neu erzeugen: Quellskript ausführen, dann `python tools/publish_doc_figures.py`.")

# document -> [(source under output/, caption)]
FIGURES: dict[str, list[tuple[str, str]]] = {
    "02_vehicle_models.md": [
        ("demo_7dof/seven_dof_maneuver.png",
         "7-DOF-Zweispurmodell mit Hinterachslenkung und Torque Vectoring auf Split-µ: Bahn, Längs- und Querbewegung, "
         "Gierrate und Lenkwinkel, Raddrehzahlen, Längsschlupf, Radlasten, Antriebs- und Bremsmomente, Reifenkräfte. "
         "Quelle: `examples/seven_dof_4ws_tv_demo.py`."),
    ],
    "03_tyre_models.md": [
        ("tyres/tyre_models_overview.png",
         "Magic-Formula-Reifen (synthetisch) gegen Fiala: Seitenkraft und Rückstellmoment bei drei Radlasten, kombinierter "
         "Schlupf, Selbstähnlichkeit in µ, Relaxation und Lenkwinkelsprung des 7-DOF-Modells mit Relaxationslängen. "
         "Quelle: `examples/tyre_models_demo.py`."),
    ],
    "06_handling_analysis.md": [
        ("handling_analysis/reference_vehicle_handling.png",
         "Lineare Fahrdynamik des Referenzfahrzeugs: stationäre Gierverstärkung (charakteristische Geschwindigkeit "
         "≈ 194 km/h), Eigenfrequenz, Dämpfung, Frequenzgang der Gierrate und Lenkwinkelsprung 7-DOF gegen linear. "
         "Quelle: `examples/handling_analysis_reference_vehicle.py`."),
    ],
    "04_chassis_kinematics.md": [
        ("kc_analysis/kc_understeer.png",
         "Eigenlenken mit K&C: Untersteuerwinkel und lokaler Eigenlenkgradient mit schrittweise zugeschalteten "
         "Rollzentren/Ackermann, Rolllenken und Elastokinematik; lineares Budget 2,19 °/g; Parameterempfindlichkeit. "
         "Quelle: `examples/kc_understeer_analysis.py`."),
    ],
    "07_handling_limits.md": [
        ("handling_limits/steady_state_handling_characteristics.png",
         "Stationäre Kreisfahrt nach dem Vorbild von ISO 4138 (Simulation): Lenkwinkel und lokaler Eigenlenkgradient "
         "über der Querbeschleunigung, dynamischer Schwimmwinkel und g-g-Diagramm für vier Konfigurationen. "
         "Quelle: `examples/handling_limits_and_understeer_demo.py`."),
    ],
    "10_control_allocation.md": [
        ("slip_and_allocation/slip_and_allocation.png",
         "Schlupfregelung und beschränkte Allokation: mittlere Verzögerung bezogen auf µg ohne ABS, mit Alt-ABS und mit "
         "PI-Schlupfregelung (Werte über 1, weil die Spitzenreibung des MF-Reifens bei geringer Radlast über µ liegt); "
         "Split-µ-Bremsung mit fester Lenkung; Lenkwinkelsprung mit TV; Bremsenausfall vorn links. "
         "Quelle: `examples/slip_and_allocation_demo.py`."),
    ],
    "11_motion_control.md": [
        ("controller_benchmark/controller_benchmark.png",
         "Gierrate im Sine with Dwell (80 km/h, 8°) für alle Regler auf derselben Strecke, nominal und auf zwei "
         "Streckenvarianten (+15 % Masse/Trägheit, Hinterachs-Schräglaufsteifigkeit −20 %). Quelle: "
         "`examples/controller_benchmark.py`."),
        ("split_mu_matrix/split_mu_matrix.png",
         "Split-µ-Vollbremsung (100 km/h, µ 1,0/0,2, Lenkung gerade): Bahn und Gierwinkel je Konfiguration. Nur die "
         "Allokation, die das Bremsmoment auf der Hoch-µ-Seite reduziert, bleibt gerade. Quelle: "
         "`examples/split_mu_matrix_demo.py`."),
    ],
    "19_estimation_robustness.md": [
        ("monte_carlo/monte_carlo.png",
         "Monte Carlo über 25 Streckenstichproben (Seed 2026), Sine with Dwell 80 km/h: Gierratenverhältnis 1 s nach "
         "Lenkende und Querversatz, passiv gegen PI (TV), mit den nach FMVSS 126 nachgebildeten Grenzen. "
         "Quelle: `examples/monte_carlo_robustness.py`."),
    ],
    "18_state_estimation.md": [
        ("estimation/estimation_and_pso_validation.png",
         "EKF-Schwimmwinkelbeobachter und RLS-Reibwertschätzer gegen die Wahrheit, IMU-Biasschätzung, PSO-Konvergenz. "
         "**Ehrlicher Befund:** Der EKF verfehlt die Schwimmwinkelspitzen deutlich (≈ 3° statt 6°), und der "
         "RLS-Reibwert bleibt bei 1,0 bzw. 0,7, während der wahre Wert bei 0,35–0,45 liegt: In diesem Manöver "
         "konvergiert er nicht. Das war der Anlass für den sensorbasierten Schätzer `estimation/sensor_friction.py`. "
         "Quelle: `examples/estimation_and_pso_demo.py`."),
    ],
    "08_vertical_dynamics.md": [
        ("vertical_dynamics/vertical_dynamics.png",
         "Viertelfahrzeug: analytischer gegen simulierten Frequenzgang von Aufbaubeschleunigung und dynamischer Radlast, "
         "Skyhook und Groundhook, Straßenklassen A–E nach ISO 8608 mit erzeugtem Klasse-C-Profil. "
         "Quelle: `examples/vertical_dynamics_demo.py`."),
    ],
    "09_suspension_dynamics.md": [
        ("suspension/suspension_and_14dof_validation.png",
         "Viertelfahrzeug (passiv, Skyhook, Groundhook) über eine Schwelle und 14-DOF-Fahrzeug bei Bremsen mit Lenken: "
         "Nick-, Hub- und Wankbewegung, Radlasten, Bahn. Quelle: `examples/suspension_dynamics_demo.py`."),
    ],
    "34_experiments.md": [
        ("experiments/sine_with_dwell_comparison/base.png",
         "Ausgabe des YAML-Experiments `sine_with_dwell_comparison`: Gierrate, Schwimmwinkel und Bahn für passiv, PI, "
         "GS-LQR (4WS+TV) und LTV-MPC mit Beobachter. Quelle: "
         "`python -m vehicle_model.experiments run configs/experiments/sine_with_dwell_comparison.yaml`."),
        ("experiments/dynamic_nmpc_solver_benchmark/solver_benchmark.png",
         "Experimenttyp `solver_benchmark`: dynamisches NMPC im Spurwechsel (90 km/h, 3,5 m, µ = 0,75) mit vier "
         "Varianten (die vierte: constraint-aware aus robust control). Links Querposition (alle überschwingen, N = 6 um 1,2 m), Mitte Rechenzeit je Lösung "
         "(Wanduhr), rechts KKT-Stationarität relativ zu den Kosten (logarithmisch): auch als `nominal` gemeldete "
         "Lösungen erreichen teils Werte um 1. Quelle: "
         "`python -m vehicle_model.experiments run configs/experiments/dynamic_nmpc_solver_benchmark.yaml`."),
    ],
    "20_nonlinear_stability.md": [
        ("experiments/equilibrium_steer_sweep/equilibria.png",
         "Gleichgewichte über dem Vorderachslenkwinkel bei 90 km/h, µ = 1,0 und 0,6 (Linie: Hauptast, gefüllt: stabil, "
         "offen: instabil). Bei µ = 0,6 sättigt a_y bei 5,85 m/s², die Dämpfung des Quer-/Giermodes fällt von 0,98 auf "
         "0,14; ab 2° erscheinen auf der Schleuderseite ein Sattel und ein instabiler Fokus. Quelle: "
         "`python -m vehicle_model.experiments run configs/experiments/equilibrium_steer_sweep.yaml`."),
        ("experiments/equilibrium_steer_sweep/eigenvalues.png",
         "Quer-/Gier-Eigenwerte entlang des Hauptastes (Beschriftung: Lenkwinkel in Grad): Mit dem Lenkwinkel wandern sie "
         "zur imaginären Achse, bei µ = 0,6 deutlich schneller. Quelle: wie oben."),
        ("experiments/equilibrium_speed_stiffness/equilibria.png",
         "Geradeausfahrt über der Geschwindigkeit mit Hinterachs-Schräglaufsteifigkeit × 1,0 / 0,6 / 0,45: rechts unten "
         "der größte Realteil, der bei der kritischen Geschwindigkeit (linear 108 bzw. 79 km/h) das Vorzeichen wechselt. "
         "Quelle: `python -m vehicle_model.experiments run configs/experiments/equilibrium_speed_stiffness.yaml`."),
        ("experiments/equilibrium_tv_4ws/equilibria.png",
         "Konstantes TV-Giermoment und festes Hinterachslenkverhältnis bei 90 km/h, 1°, µ = 0,6: Mit positivem Moment "
         "steigt a_y immer steiler bis zu einer Faltung (ab etwa 450 Nm ohne Hinterachslenkung kein Gleichgewicht, "
         "Reibkreis der Hinterräder ausgeschöpft); gleichsinnige Hinterachslenkung verschiebt sie. Quelle: "
         "`python -m vehicle_model.experiments run configs/experiments/equilibrium_tv_4ws.yaml`."),
        ("experiments/stability_map_cornering/stability_maps.png",
         "Numerische Rückführregionen bei 80 km/h, 1° gehaltenem Lenkwinkel, µ = 0,6, 15 × 15 Anfangszustände je "
         "Regler. Kreuze: empfindliche Punkte (Klasse ändert sich mit halbem Zeitschritt oder doppelter Dauer); "
         "gestrichelt: heuristische Phase-Plane-Grenze S = 1. Quelle: "
         "`python -m vehicle_model.experiments run configs/experiments/stability_map_cornering.yaml`."),
        ("experiments/stability_map_cornering/region_comparison.png",
         "Anteil zurückgekehrter Anfangszustände je Regler mit Schranken aus Rand- und empfindlichen Punkten und dem "
         "Anteil auf dem verfeinerten Gitter (Raute). Quelle: wie oben."),
        ("experiments/stability_map_robust/stability_maps.png",
         "Rückführregionen der Regler aus robust control (alle mit Hinterachslenkung und Giermoment) neben passiv und TV, gleicher "
         "Betriebspunkt: Die Regionen schrumpfen auf ein Band um β0 = 0. Quelle: "
         "`python -m vehicle_model.experiments run configs/experiments/stability_map_robust.yaml`."),
        ("experiments/stability_map_rear_steer/stability_maps.png",
         "Ursache: der unveränderte baseline GS-LQR mit Hinterachslenkung (33 %) gegen GS-LQR und Robust LQR nur mit "
         "Giermoment (je 94 %). Quelle: "
         "`python -m vehicle_model.experiments run configs/experiments/stability_map_rear_steer.yaml`."),
        ("experiments/stability_map_cornering/phase_portraits.png",
         "Phasenporträts (β–r, β–β̇, r–ṙ) für passiv, TV und IVMC; Farbe = Klasse, Punkt = Start. Quelle: wie oben."),
    ],
    "21_robust_control.md": [
        ("experiments/robust_benchmark_sine_with_dwell/matrix_max_sideslip_deg.png",
         "Maximaler Schwimmwinkel je Regler und Unsicherheitsfall in den drei Rückführmodi (logarithmische Farbskala; "
         "≈ 90° = Kontrollverlust). Die Aktuatorverzögerung destabilisiert alle Gesetze mit hoher Verstärkung; der "
         "Basis-PI, die Tightening MPC und die überwachte Variante bleiben beherrschbar. Quelle: "
         "`python -m vehicle_model.experiments run configs/experiments/robust_benchmark_sine_with_dwell.yaml`."),
        ("experiments/robust_benchmark_sine_with_dwell/matrix_yaw_rate_rms_error_degps.png",
         "Gierratenfehler (RMS) gegen die gemeinsame Referenz, gleiche Anordnung. Quelle: wie oben."),
        ("experiments/robust_benchmark_sine_with_dwell/yaw_rate_histories.png",
         "Gierrate über der Zeit bei idealer Rückführung, ein Feld je Unsicherheitsfall. Quelle: wie oben."),
        ("experiments/dynamic_nmpc_solver_benchmark/solver_benchmark.png",
         "Dynamisches NMPC im Spurwechsel mit der constraint-aware Variante (weiche Stabilitätsgrenzen, Projektion "
         "der Stellgröße). Quelle: "
         "`python -m vehicle_model.experiments run configs/experiments/dynamic_nmpc_solver_benchmark.yaml`."),
    ],
    "36_critical_scenarios.md": [
        ("experiments/critical_search_sine_with_dwell/search.png",
         "Suche nach kritischen Sine-with-Dwell-Bedingungen für den baseline GS-LQR: Verlauf (links oben, gestrichelt: "
         "Kriterium) und Stichproben je Parameterpaar (rot kritisch, blau nicht kritisch, grau verworfen; Stern: "
         "minimale Fälle, Kreuz: nominal). Quelle: "
         "`python -m vehicle_model.experiments run configs/experiments/critical_search_sine_with_dwell.yaml`."),
        ("experiments/critical_search_sine_with_dwell/critical/01_max_sideslip_deg/timeseries.png",
         "Fall 01, minimal: nur 64 ms Aktuatortotzeit, sonst nominal (80 km/h, 5°, µ 1). GS-LQR (rot) dreht sich, "
         "gleicher Regler ohne Totzeit (blau) und baseline PI im selben Fall (grün) bleiben stabil. Quelle: wie oben."),
        ("experiments/critical_search_sine_with_dwell/critical/02_max_sideslip_deg/timeseries.png",
         "Fall 02, minimal: 88,5 km/h, 3,7° Amplitude (kleiner als nominal), 24 ms Totzeit. Die Hinterachslenkung "
         "schwingt auf +9,6° und bleibt an der Grenze −10°. Quelle: wie oben."),
        ("experiments/critical_search_sine_with_dwell/critical/01_max_sideslip_deg/scenario.gif",
         "Fall 01 als Animation aus den aufgezeichneten Daten: kritischer Lauf (rot) gegen Nominalparameter (blau). "
         "Quelle: wie oben."),
    ],
    "37_design_optimization.md": [
        ("experiments/design_chassis_tv/pareto_fronts.png",
         "Pareto-Analyse der Fahrwerks-, Lenk- und TV-Auslegung: alle Zielpaare (grau gültige Entwürfe, blau die "
         "Pareto-Menge über alle vier Ziele, Kreuz Referenzfahrzeug, Sterne ausgewählte Entwürfe). Quelle: "
         "`python -m vehicle_model.experiments run configs/experiments/design_chassis_tv.yaml`."),
        ("experiments/design_chassis_tv/sensitivities.png",
         "Standardisierte Regressionskoeffizienten jedes Ziels auf die normierten Auslegungsparameter (R² je Zeile). "
         "Quelle: wie oben."),
        ("experiments/design_chassis_tv/objective_distributions.png",
         "Verteilung der Zielwerte über alle gültigen Entwürfe, Referenz grün. Quelle: wie oben."),
    ],
    "22_drift_model_validity.md": [
        ("drift/tyre_audit/tyre_audit.png",
         "Reifenaudit bis ±45° Schräglauf: Seitenkraft (grau: außerhalb der erklärten Gültigkeit), Längskraft und "
         "Ausnutzung im kombinierten Schlupf für MF, Simplified Combined und Fiala. Quelle: `python examples/drift_tyre_audit.py`."),
    ],
    "24_drift_results.md": [
        ("drift/runs/full_sequence/supervised/drift_animation.gif",
         "Überwachter Drift aus der Geradeausfahrt (13 m/s, µ 0,9, Ziel β −20°): Annäherung, Einleitung durch den LQR, "
         "Halten, Austritt mit Gegenlenken, Normalfahrt. Nur aufgezeichnete Daten der 7-DOF-Strecke; doppelte "
         "Wiedergabegeschwindigkeit. Quelle: `python examples/drift_figure_eight_demo.py`."),
        ("drift/equilibrium_sweep/equilibrium_sweep.png",
         "Stationäre Driftgleichgewichte über |β| je Geschwindigkeit und Reibwert: Radius, Lenkwinkel, Moment je "
         "Hinterrad, Hinterradschlupf; offene Kreise: Reifen extrapoliert, Kreuze: kein Gleichgewicht. Quelle: "
         "`python examples/drift_equilibria_demo.py`."),
        ("drift/lqr_hold/sideslip_and_yaw.png",
         "Halten und Störungen links/rechts, offen (gestrichelt) gegen LQR (durchgezogen). Quelle: "
         "`python examples/drift_hold_demo.py`."),
        ("drift/initiation/initiation_sequence.png",
         "Moduswechsel und Zeitverläufe einer Einleitung bis zum Austritt (11 m/s, 15°, µ 0,9). Quelle: "
         "`python examples/drift_initiation_demo.py`."),
        ("drift/circle_tracking/trajectory.png",
         "Kreisbahnen bei ±2 m Startversatz (offen, ESC, Drift-LQR fest, mit Außenschleife) und die Acht-Sequenz "
         "(Geometrie nicht erreicht). Quelle: `python examples/drift_circle_demo.py`."),
    ],
    "13_rear_wheel_steering.md": [
        ("four_wheel_steering/parking/u_turn.gif",
         "Wenden auf schmaler Straße, 2WS (grau) gegen 4WS (türkis): unten die Radlenkwinkel über der Zeit. Die "
         "Hinterachse lenkt mit −8° gegen die Vorderachse; die überstrichene Breite sinkt von 12,75 m auf 11,10 m. "
         "Quelle: `examples/parking_4ws_demo.py`."),
        ("four_wheel_steering/parking/parallel_exit.gif",
         "Ausparken aus enger Längsparklücke: 4WS braucht 1,88 m statt 1,96 m Lücke zum Vordermann, das Heck schwenkt "
         "aber 13 cm näher an den Bordstein (Nachteil der Gegenphase). Quelle: `examples/parking_4ws_demo.py`."),
        ("four_wheel_steering/parking/perpendicular_parking.gif",
         "Rückwärts senkrecht einparken (90°): überstrichene Fläche 8,76 × 9,35 m (4WS) gegen 9,65 × 9,66 m (2WS). "
         "Quelle: `examples/parking_4ws_demo.py`."),
        ("four_wheel_steering/scenario_3_double_lane_change_4ws.gif",
         "Doppelspurwechsel bei 80 km/h: die Hinterachse lenkt gleichphasig mit. Quelle: "
         "`examples/four_wheel_steering_benchmark.py`."),
        ("four_wheel_steering/textbook_4ws_characteristics.png",
         "Kennfeld der Hinterachslenkung: Übersetzungsverhältnis über der Geschwindigkeit, Kennlinienschar, "
         "Isolinienfeld und stationäre Schwimmwinkelverstärkung 2WS gegen 4WS. Quelle: "
         "`examples/four_wheel_steering_benchmark.py`."),
        ("four_wheel_steering/lut_3d_surface.png",
         "Kalibrierte Lookup-Tabelle δ_r(v_x, δ_f): gegenphasig unter ≈ 36 km/h, gleichphasig darüber. "
         "Quelle: `examples/four_wheel_steering_benchmark.py`."),
        ("four_wheel_steering/four_wheel_steering_dashboard.png",
         "Neun 4WS-Szenarien im Überblick (2WS, drehzahlgeführte Kennlinie, Lookup-Tabelle, mit Gierdämpfung). "
         "Quelle: `examples/four_wheel_steering_benchmark.py`."),
    ],
    "14_split_mu_dynamics.md": [
        ("split_mu_scenarios/scenario_1_split_mu_braking.gif",
         "Split-µ-Bremsung aus 100 km/h mit Fahrermodell, das die Spur hält, und 1600 N m *gesamtem* Bremsmoment (60/40), "
         "also einer moderaten Bremsung (im Bild als „Emergency Braking“ bezeichnet). "
         "Quelle: `examples/split_mu_scenarios_demo.py`."),
        ("split_mu_scenarios/scenario_1_split_mu_braking.png",
         "Dieselbe Bremsung als Dashboard: Bahnen, Gierrate, Querabweichung und Störgiermoment. Der lenkende Fahrer ist "
         "der Unterschied zur Konfigurationsmatrix in 11_motion_control.md, in der die Lenkung gerade gehalten wird und "
         "IVMC mit der alten additiven Stellkette sich eindreht (über die gemeinsame Allokation, Standard, "
         "bleibt er gerade). Quelle: `examples/split_mu_scenarios_demo.py`."),
        ("split_mu_scenarios/split_mu_friction_sweep_analysis.png",
         "Reibwert-Sweep der Niedrig-µ-Seite: Gierrate, Gierwinkel, Querabweichung, Bremsweg und Schwimmwinkel je "
         "Regler. Quelle: `examples/split_mu_scenarios_demo.py`."),
        ("benchmark/multi_mu_scenarios_dashboard.png",
         "Multi-µ-Matrix: Kennwerte von acht Manövern über dem Reibwert für vier Konfigurationen. "
         "Quelle: `examples/multi_mu_scenarios_benchmark.py`."),
    ],
    "12_chassis_control.md": [
        ("benchmark/double_lane_change_comparison.png",
         "Doppelspurwechsel 80 km/h bei µ = 0,75: passiv, 4WS und IVMC. Quelle: `examples/integrated_control_benchmark.py`."),
        ("benchmark/mu_split_comparison.png",
         "Split-µ-Bremsung ohne Regelung gegen IVMC über die gemeinsame Allokation: Gierwinkel 0° statt > 150°. Die "
         "Allokation bremst das Eisrad nur leicht (Schlupf ≈ −0,01) und begrenzt die Hoch-µ-Seite entsprechend; die "
         "Legende „−0,15 optimal“ im Bild stammt aus der alten ABS-Kette. Quelle: `examples/abs_tcs_esc_benchmark.py`."),
        ("advanced_control/phase_plane_stability_analysis.png",
         "Phasenebene (β, β̇) mit Eingriffsgrenzen; Schwimmwinkel und Gierrate passiv gegen Phasenebenen-IVMC. "
         "Quelle: `examples/advanced_chassis_control_demo.py`."),
        ("e2e_analysis/e2e_liftoff_oversteer_phase_plane.png",
         "Lastwechsel beim Bremsen in der Kurve: Bahn in der (β, r)-Ebene, Schwimmwinkel gegen die dynamische Grenze, "
         "Bedrohungsmaß und Gierrate. Die Regelung verlangsamt das Ausbrechen, verhindert es aber nicht (β ≈ −33° nach "
         "2,5 s). Quelle: `examples/e2e_deep_analysis_and_benchmarking.py`."),
        ("advanced_control/14dof_coupled_fishhook_stability.png",
         "14-DOF-Fishhook: Wankwinkel, Lastverlagerungsverhältnis, Radlasten und dynamische Wanksteifigkeitsverteilung "
         "passiv gegen Wankregelung. Quelle: `examples/advanced_chassis_control_demo.py`."),
    ],
    "16_nmpc_control.md": [
        ("advanced_control/dynamic_two_track_nmpc_tracking.png",
         "Dynamisches Zweispur-NMPC, Ausweichen bei 90 km/h: Bahn, Lenkwinkel vorn/hinten, Reibkreisausnutzung bezogen "
         "auf µF_z. Die Bahn schießt auf 4,8 m über (Ziel 3,5 m). Werte knapp über 1 sind keine Verletzung: Die "
         "Spitzenreibung des MF-Reifens liegt bei geringer Radlast über µ. Quelle: "
         "`examples/driver_and_nmpc_dynamics_demo.py`."),
        ("e2e_analysis/e2e_nmpc_kamm_friction_evasion.png",
         "NMPC-Spurwechsel 90 km/h (E2E-Test): Bahn, koordinierte Lenkwinkel, Radmomente und Reibkreisausnutzung. "
         "Quelle: `examples/e2e_deep_analysis_and_benchmarking.py`."),
    ],
    "17_nmpc_path_tracking.md": [
        ("animations/nmpc_slalom.gif",
         "Kinematisches Pfadfolge-NMPC im Slalom mit Hinterachslenkung aus der Lookup-Tabelle. "
         "Quelle: `examples/generate_maneuver_animations.py`."),
        ("nmpc_path_tracking/nmpc_path_tracking_simulation.png",
         "Pfadfolge-NMPC (Maßstabsmodell): Bahn, Lenkwinkel, Querfehler und Geschwindigkeit. Quelle: "
         "`python examples/nmpc_path_tracking_simulation.py --headless`."),
    ],
    "15_driver_model.md": [
        ("advanced_control/driver_closed_loop_workload.png",
         "Fahrermodell im Doppelspurwechsel: Bahn, Zerlegung von Totzeit und Verzögerung, Lenkaufwand und Lenkumkehrungen "
         "passiv gegen IVMC-unterstützt. Quelle: `examples/driver_and_nmpc_dynamics_demo.py`."),
        ("e2e_analysis/e2e_driver_split_mu_stability.png",
         "Fahrer im geschlossenen Kreis beim Reibwertübergang: Basis- gegen verbesserten Fahrer (Prädiktion, Adaption). "
         "Der verbesserte Fahrer lenkt stärker und schwingt mehr. Quelle: `examples/e2e_deep_analysis_and_benchmarking.py`."),
    ],
    "38_validation_platform.md": [
        ("animations/iso_3888_dlc.gif",
         "Doppelspurwechsel nach dem Vorbild von ISO 3888-2 mit allen vier Konfigurationen. "
         "Quelle: `python examples/run_validation_platform.py --animations`."),
        ("validation_report/figures/iso_3888_dlc.png",
         "Doppelspurwechsel: Bahn mit Pylonen, Gierrate, Schwimmwinkel, Querbeschleunigung. Quelle: "
         "`examples/run_validation_platform.py`."),
        ("validation_report/figures/sine_with_dwell.png",
         "Sine with Dwell nach dem Vorbild von FMVSS 126 (Simulation, keine Prüfung nach Vorschrift). "
         "Quelle: `examples/run_validation_platform.py`."),
        ("validation_report/figures/mu_split_braking.png",
         "Split-µ-Bremsung mit gerader Lenkung und 1450 N m *gesamtem* Bremsmoment, also etwa 12 % Pedal: eine moderate "
         "Bremsung, keine Vollbremsung. Mit der alten additiven Stellkette bleibt IVMC hier unter der 15°-Grenze (≈ 10°) und "
         "dreht sich ab etwa 20 % Pedal ein; über die gemeinsame Allokation (Standard) bleibt er bei jedem "
         "Pedal gerade (Konfigurationsmatrix in 11_motion_control.md, Test "
         "`test_additive_esc_holds_only_at_the_moderate_brake_level_of_the_validation_matrix`). "
         "Quelle: `examples/run_validation_platform.py`."),
        ("validation_report/figures/fishhook_ncap.png",
         "Fishhook nach NCAP-Vorbild. Quelle: `examples/run_validation_platform.py`."),
    ],
    "39_visualization_guide.md": [
        ("animations/multi_mu_lane_change_comparison.gif",
         "Spurwechsel auf mehreren Reibwerten mit Nahkamera-Panel und HUD. Quelle: `examples/multi_mu_scenarios_benchmark.py`."),
        ("four_wheel_steering/parking/tight_alley.gif",
         "4WS-Animation mit Lenkwinkel-Panel: unten Vorder- und Hinterachswinkel mit Cursor auf dem aktuellen Bild. "
         "Fahrzeuge werden an der Hinterachse gezeichnet, aus dem Schwerpunkt über l_r berechnet "
         "(`VehicleParams.cog_to_rear_axle`). Quelle: `examples/parking_4ws_demo.py`."),
    ],
}

# Animations of every topic, all from the one renderer (visualization/run_animation.py). Experiments:
# examples/stage_animations.py and examples/chassis_control_animations.py; side views for ride and grade.
ANIMATIONS: dict[str, list[tuple[str, str]]] = {
    "02_vehicle_models.md": [("demo_7dof/split_mu_4ws_tv.gif", "7-DOF-Modell: Split-µ-Bremsung passiv gegen GS-LQR mit 4WS und Torque Vectoring.")],
    "03_tyre_models.md": [("tyres/tyre_models_step_steer.gif", "Gleiches Manöver mit linearem, Fiala- und Magic-Formula-Reifen: Lenkwinkelsprung 5°, 80 km/h.")],
    "04_chassis_kinematics.md": [("kc_analysis/step_steer_kc.gif", "Lenkwinkelsprung 80 km/h mit und ohne Kinematik und Elastokinematik.")],
    "06_handling_analysis.md": [("handling_analysis/step_steer_speeds.gif", "Lenkwinkelsprung 2° bei 60, 100 und 140 km/h, passives Fahrzeug.")],
    "07_handling_limits.md": [("handling_limits/towards_the_limit.gif", "Lenkwinkelsprung 2°, 4° und 8° bei 80 km/h: Annäherung an die Haftgrenze.")],
    "08_vertical_dynamics.md": [("vertical_dynamics/bump_response.gif", "Hindernis 6 cm bei 60 km/h, passiv gegen Skyhook (Seitenansicht, Vertikalbewegung überhöht).")],
    "09_suspension_dynamics.md": [("suspension/damping_iso_c.gif", "ISO-8608-Klasse C bei 60 km/h: passiv, Skyhook, Groundhook.")],
    "10_control_allocation.md": [("slip_and_allocation/low_mu_braking.gif", "Vollbremsung auf µ 0,3: blockierende Räder gegen Allokation mit Schlupfregelung.")],
    "11_motion_control.md": [("controller_benchmark/sine_with_dwell.gif", "Reglerbibliothek im Sine with Dwell, 80 km/h, 6°."),
                             ("split_mu_matrix/split_mu_matrix.gif", "Split-µ-Vollbremsung mit gerader Lenkung über die Konfigurationen.")],
    "12_chassis_control.md": [("advanced_control/lift_off.gif", "Gaswegnahme in der Kurve: ohne Gierregelung gegen Gierraten-PI.")],
    "18_state_estimation.md": [("estimation/feedback_modes.gif", "Gleicher Regler mit idealer, geschätzter und rein sensorbasierter Rückführung.")],
    "19_estimation_robustness.md": [("monte_carlo/mu_variation.gif", "Sine with Dwell mit PI bei drei Reibwerten.")],
    "20_nonlinear_stability.md": [("experiments/stability_map_cornering/comparison.gif", "Rückkehr aus β 12°, r −25 °/s: Läufe mit den Reglern der Stabilitätskarte; Bahn aus den aufgezeichneten Zuständen integriert."),
                                  ("experiments/equilibrium_speed_stiffness/comparison.gif", "180 km/h nach 0,3° Lenkwinkel: Mit geringerer Hinterachssteifigkeit entsteht eine große Gierschwingung."),
                                  ("experiments/equilibrium_steer_sweep/comparison.gif", "Einschwingen auf die stationäre Kurvenfahrt, 90 km/h, Richtung Haftgrenze.")],
    "21_robust_control.md": [("experiments/robust_benchmark_sine_with_dwell/comparison.gif", "Sine with Dwell mit Aktuatorverzögerung: Basisregler gegen robuste Verfahren.")],
    "25_actuation_chain.md": [("actuation_chain/split_mu_braking/actuation_trace.png", "Stellgrößen-Trace requested, allocated, issued, realised bei Split-µ-Bremsung. Quelle: `examples/actuation_trace.py`."),
                              ("actuation_chain/split_mu_braking.gif", "Split-µ-Vollbremsung mit Aktuatordynamik.")],
    "26_brake_functions.md": [("brake_functions/braking_functions.png", "Ergebnisse der Bremsfunktionen. Quelle: `examples/brake_functions_figures.py`."),
                              ("brake_functions/split_mu.gif", "Split-µ-Vollbremsung: Stabilität gegen Bremsweg."),
                              ("brake_functions/brake_in_turn.gif", "Bremsen in der Kurve ohne Gierregelung, mit PI, mit PI + CBC."),
                              ("brake_functions/regen_coast_low_mu.gif", "Schleppmoment auf µ 0,2: Rekuperation abschalten gegen Schlupfregelung.")],
    "27_traction_control.md": [("traction_control/traction_functions.png", "Ergebnisse der Traktionsfunktionen. Quelle: `examples/traction_figures.py`."),
                               ("traction_control/split_mu_launch.gif", "Anfahren auf Split-µ: ohne TCS, offene Kopplung, eLSD, ITC."),
                               ("traction_control/corner_exit/comparison.gif", "Beschleunigen aus der Kurve.")],
    "28_roll_stability_active_steering.md": [("roll_stability/rsc_afs.png", "Wankstabilität und aktive Vorderachslenkung. Quelle: `examples/roll_stability_steering_figures.py`."),
                                             ("roll_stability/fishhook_rsc.gif", "14-DOF-Fishhook: Radabheben (rote Räder) passiv, mit RSC, nur aktive Wankregelung."),
                                             ("active_front_steering/sine_with_dwell.gif", "Sine with Dwell: aktive Vorderachslenkung gegen 2WS und Torque Vectoring.")],
    "29_g_vectoring.md": [("g_vectoring/gvc.png", "GVC-artige Lasttransferregelung. Quelle: `examples/g_vectoring_figures.py`."),
                          ("g_vectoring/lane_change.gif", "Spurwechsel mit konstantem Pedal, GVC aus und an.")],
    "30_preview_damping_active_pitch.md": [("ride_control/ride.png", "Fahrkomfort und Nicken über alle Szenarien. Quelle: `examples/ride_control_figures.py`."),
                                           ("ride_control/brake_pitch.gif", "Bremsen auf welliger Straße: Nicken mit und ohne aktive Nickregelung."),
                                           ("ride_control/bump_preview.gif", "Hindernis: Skyhook gegen Vorschau-Skyhook."),
                                           ("ride_control/iso_C_comfort.gif", "ISO C: Komfort gegen Radlast.")],
    "31_hill_descent_control.md": [("hill_descent/hdc.png", "Hill Descent Control über Gefälle und Oberflächen. Quelle: `examples/hill_descent_figures.py`."),
                                   ("hill_descent/hdc_20pct_dry.gif", "20 % Gefälle, trocken: HDC hält 10 km/h."),
                                   ("hill_descent/hdc_ice_infeasible.gif", "20 % Gefälle auf Eis: physikalisch nicht haltbar, als infeasible gemeldet."),
                                   ("hill_descent/hdc_grade_change.gif", "Gefällewechsel 5 → 20 → 5 %, mit und ohne ausgefallene Bremse.")],
    "32_fault_tolerance.md": [("fault_tolerance/split_mu_three_brakes_failed.gif", "Split-µ-Bremsung mit drei ausgefallenen Bremsen, ohne und mit Monitor."),
                              ("fault_tolerance/imu_lost_split_mu.gif", "Verlust des Gierratensensors: Rückfall auf ausgeglichene Bremskräfte."),
                              ("fault_tolerance/sequence_recovery.gif", "Fehlerfolge mit degradierten Modi und Rückkehr zu nominal.")],
    "33_motion_management_benchmark.md": [("motion_management/matrix/matrix.png", "Primärkennzahl je Manöver: Medianänderung gegenüber der Stufe passive über die gepaarten Ziehungen."),
                                          ("motion_management/levels_sine_with_dwell.gif", "Sine with Dwell über die Konfigurationsstufen."),
                                          ("motion_management/levels_actuator_fault.gif", "Zwei ausgefallene Bremsen auf Split-µ über die Stufen."),
                                          ("motion_management/levels_corner_exit.gif", "Beschleunigen aus der Kurve über die Stufen.")],
    "37_design_optimization.md": [("experiments/design_chassis_tv/comparison.gif", "Extreme Pareto-Auslegungen im Sine with Dwell auf µ 0,6.")],
}
for _doc, _items in ANIMATIONS.items():
    FIGURES.setdefault(_doc, []).extend(_items)


def asset_path(doc: str, src: str) -> Path:
    name = Path(src).name
    if sum(Path(s).name == name for s, _ in FIGURES[doc]) > 1:   # same file name from several runs: prefix the run
        name = f"{Path(src).parent.name}_{name}"
    return ASSETS / Path(doc).stem.lower() / name


def shrink(src: Path, dst: Path) -> None:
    dst.parent.mkdir(parents=True, exist_ok=True)
    im = Image.open(src)
    if src.suffix.lower() == ".gif":
        frames, durations = [], []
        for i, f in enumerate(ImageSequence.Iterator(im)):
            if i % 2:
                continue
            f = f.convert("RGB")
            f = f.resize((f.width // 2, f.height // 2), Image.LANCZOS)
            frames.append(f.quantize(colors=128, method=Image.Quantize.MEDIANCUT))
            durations.append(2 * f.info.get("duration", im.info.get("duration", 66)))
        frames[0].save(dst, save_all=True, append_images=frames[1:], duration=durations, loop=0, optimize=True)
    else:
        im = im.convert("RGB")
        if im.width > 1600:
            im = im.resize((1600, round(im.height * 1600 / im.width)), Image.LANCZOS)
        im.quantize(colors=256, method=Image.Quantize.MEDIANCUT).save(dst, optimize=True)


def block(doc: str) -> str:
    lines = [START, "", "## Ergebnisbilder", "", f"> {NOTE}", ""]
    for src, caption in FIGURES[doc]:
        rel = asset_path(doc, src).relative_to(DOCS).as_posix()
        lines += [f"![{Path(src).stem}]({rel})", "", f"*{caption}*", ""]
    return "\n".join(lines + [END])


def embed(doc: str) -> None:
    p = DOCS / doc
    text = p.read_text(encoding="utf-8")
    new = block(doc)
    if START in text:
        head, rest = text.split(START, 1)
        text = head + new + rest.split(END, 1)[1]
    else:
        text = text.rstrip("\n") + "\n\n---\n\n" + new + "\n"
    p.write_text(text, encoding="utf-8")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--check", action="store_true", help="only report missing sources")
    args = ap.parse_args(argv)
    missing = [(d, s) for d, items in FIGURES.items() for s, _ in items if not (OUT / s).exists()]
    for d, s in missing:
        print(f"missing: output/{s} (for {d})")
    if args.check or missing:
        return 1 if missing else 0
    total = 0
    for doc, items in FIGURES.items():
        for src, _ in items:
            dst = asset_path(doc, src)
            shrink(OUT / src, dst)
            total += dst.stat().st_size
        embed(doc)
        print(f"{doc}: {len(items)} figure(s)")
    print(f"docs/assets: {total / 1e6:.1f} MB")
    return 0


if __name__ == "__main__":
    sys.exit(main())
