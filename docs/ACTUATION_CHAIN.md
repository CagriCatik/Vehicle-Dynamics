# Stellgrößenkette: Ausgangsstand, Architekturkonsolidierung und Aktuatordynamik

Grundlage: [Plan integrierte Bewegungsregelung](plans/integrated_motion_control_plan.md). Dieses Dokument hält den geprüften Ausgangsstand fest, gleicht
die Blocker A–H mit Code und Tests ab und beschreibt, was Stellgrößenkette dafür geändert hat. Alle Parameter sind synthetisch;
keine Aussage über ein reales Fahrzeug, keine Norm- oder Homologationsaussage.

## 1. Ausgangsstand (Plan 0, Schritte 1–3)

| Punkt | Wert |
|---|---|
| Plananalyse | `main` bei `992cc39` (29.10.2026) |
| Ausgangscommit | `main` bei `cacecb4` (30.10.2026): Plattformplan und Drift-Regler abgeschlossen; `origin/main` bei `e6a5e74` |
| Arbeitszweig | Konsolidierungszweig (lokal, inzwischen in `main`) |
| Arbeitsbaum | uncommittete Nutzeränderungen an `.gitignore`, `docs/index.md`; unversioniert `integrated_motion_control_plan.md`, `drift_controller_plan.md`, `output/`: nicht angefasst |
| Umgebung | Python 3.13.5; NumPy 2.5.4, SciPy 1.18.1, Matplotlib 3.11.2, PyYAML 6.0.3, pytest 9.1.1; optional FastAPI/uvicorn (Workbench, Workbench) |
| Umfang | 175 Quelldateien (36 700 Zeilen), 64 Testdateien, 54 YAML-Konfigurationen, 36 Dokumente |
| Schnelle Tests | `pytest -q -m "not slow"`: **661 bestanden, 2 übersprungen** (3 min) |
| Volle Suite | `pytest -q`: **727 bestanden, 2 übersprungen** (19 min) |
| DSL-Validierung | `python -m vehicle_model.experiments validate configs/experiments/*.yaml`: 13 von 13 gültig |
| Referenzexperiment | `run configs/experiments/split_mu_braking.yaml `: passiv dreht sich (Kurswinkel 995°, Bremsweg 82 m), PI und LQR bleiben gerade (0,004°/0,005°, 151 m) |
| CI | `.github/workflows/ci.yml`: schnelle Tests unter Python 3.10 und 3.13, Validierung der Spezifikationen, volle Suite wöchentlich; alle Quelldateien parsen mit der 3.10-Grammatik |

Vorarbeiten sind nicht stillschweigend als bestanden angenommen (Blocker H): Die Abnahmen Zustandsschätzung bis Workbench stehen mit
Nachweisen in `BASELINE_AUDIT.md` Abschnitte 5–11.

## 2. Blocker A–H gegen Code und Tests

| Blocker | Befund im Code | Status | Nachweis |
|---|---|---|---|
| **A** geteilte Stellautorität | `IntegratedChassisController` nutzt inzwischen standardmäßig die gemeinsame `ActuationLayer` (`actuation="allocation"`); die alte additive Kette ist nur noch als `actuation="legacy"` wählbare Referenz | gelöst, in Stellgrößenkette geprüft | `test_split_mu_matrix.py`; `test_single_owner_of_the_wheel_torques` (Ausgabe = genau die Momente der ActuationLayer, Standard ist die Allokation) |
| **B** 2-DOF-Allokation | `WheelForceAllocator` verteilt `(F_x, M_z)` auf vier Längskräfte; keine Lenk-, Vertikal- oder Energiedomäne | bewusst so belassen; Erweiterung erst mit eigenen Domänen | `CONTROL_ALLOCATION.md`; neue Tests zur Wirkungsmatrix des Trace |
| **C** Informationszugriff | Modi A/B/C umgesetzt; im Modus C erreicht keine Strecken-Telemetrie den Regler | gelöst | `test_plant_telemetry_never_reaches_the_controller_in_mode_c` |
| **D** Split-µ-Widerspruch | Ursache: die Validierungsmatrix bremst mit ≈ 12 % Pedal (1450 N·m gesamt), MOTION_CONTROL mit Vollbremsung; beide Dokumente erklären das | gelöst (`ebba6d0`) | `test_additive_esc_holds_only_at_the_moderate_brake_level_of_the_validation_matrix` |
| **E** 7-DOF vs. 14-DOF | 7-DOF hat keine Dämpfer- oder Hubkräfte; 14-DOF existiert | Regel für roll stability and active steering/preview damping and active pitch: RSC mit Kontakt und Preview/Pitch nur gegen 14-DOF | wird in roll stability and active steering/preview damping and active pitch als Konfigurationsprüfung durchgesetzt |
| **F** Brems-/Rekuperationsschnittstelle | `BrakeByWireActuator` liefert Reib- und Rekuperationsmoment (beide Bremsmomente ≥ 0), `ElectricPowertrainActuator` nur Antriebsmoment ≥ 0 | **in Stellgrößenkette festgelegt**: Rekuperation geht genau einmal als Bremsmoment in die Strecke (`T_brake = Reib + Rekup`), nie über `T_drive` | `test_regen_enters_once_as_braking_torque`, `test_abs_flag_drops_regen_on_that_wheel` |
| **G** Experiment-DSL | Parser, Fabrik und Runner nur gemeinsam erweitern | in Stellgrößenkette so umgesetzt (Feld `actuators`) | `test_spec_actuators_field_is_backward_compatible_and_strict` |
| **H** Status Plattformplan | Zustandsschätzung bis Workbench abgenommen | erledigt | `BASELINE_AUDIT.md` |

## 3. Stellgrößenkette: Änderungen

### 3.1 Stellgrößenkette requested → allocated → issued → realised (Arbeitspaket 6)

`MotionController` schreibt je Schritt `diagnostics["trace"]` mit den verallgemeinerten Kräften `(F_x [N], M_z [N·m])`:

| Stufe | Bedeutung | Quelle |
|---|---|---|
| `requested` | Wunsch des Regelgesetzes und Pedalkraft, vor jeder Grenze | `MotionRequest`, Pedale |
| `allocated` | beschränkte Allokation, vor ABS/TCS | `AllocationResult.achieved` |
| `issued` | Radmomente am Reglerausgang, nach ABS/TCS | `ActuationLayer.last_issued` (gleiche Wirkungsmatrix) |
| `realised` | was die Strecke in diesem Schritt erhält, nach Aktuatordynamik | `ActuatorChain` bzw. Reglerausgabe ohne Aktuatoren |

`ControlOutput.allocated_yaw_moment` behält seinen Namen und ist das *issued*-Giermoment (Kompatibilität).
`run_scenario` zeichnet `Mz_requested`, `Mz_alloc_pre_slip`, `Mz_issued`, `Mz_realised`, `T_issued_abs`,
`T_realised_abs` und `regen_power_w` auf; Kennzahlen `issued_to_realised_lag_ms` (Verschiebung maximaler
Kreuzkorrelation, 0–300 ms; 0 bei identischen Signalen oder konstantem Befehl) und `regen_energy_kj` (mechanisch am
Rad, ohne Batterie- oder Wirkungsgradmodell). Ohne Aktuatordynamik stimmen issued und realised bis auf den
Lenkwinkel überein, mit dem die Wirkungsmatrix ausgewertet wird (Regler: Telemetrie des letzten Schritts, Strecke:
aktueller Befehl; gemessen ≤ 0,25 % des Spitzenwerts).

### 3.2 Aktuatordynamik im geschlossenen Kreis (Arbeitspaket 3)

`simulation/actuator_chain.py` (`ActuatorChain`) setzt die vorhandenen Aktuatormodelle zwischen Regler und Strecke:
Lenkaktuatoren vorn/hinten (Verzögerung, Rate, Totzeit), Elektroantriebe (Verzögerung, Momentrate, Moment-/
Leistungshüllkurve, Batteriegrenze), Brake-by-Wire (asymmetrischer Druckauf-/-abbau, Rekuperationsüberblendung,
Rekuperation an ABS-Rädern abgeschaltet). Parameter aus `configs/braking/brake_by_wire.yaml` und
`configs/powertrains/`. Ideal bleibt Vergleich: `ideal=True` ist bitgleich zum unverpackten Regler (Test).
Experimente wählen je Regler `actuators: ideal | dynamic` (Standard `ideal`, rückwärtskompatibel; die Workbench bietet
das Feld an).

### 3.3 Unveränderte Legacy-Funktionen

IVMC `actuation="legacy"`, alle Szenarien, Live-Plots, GIF/PNG/CSV/JSON/Markdown-Exporte, CLI-Befehle, bestehende
YAML-Dateien (keine Schlüssel geändert oder entfernt).

## 4. Ergebnis: Split-µ-Bremsung mit idealen und dynamischen Aktuatoren

`configs/experiments/actuator_dynamics_split_mu_braking.yaml` (90 km/h, µ links 1,0 / rechts 0,2, Vollbremsung, gerade Lenkung;
gleiche Strecke, Reifen, Integration, Seed 0), Ergebnisse `output/actuation_chain/split_mu_braking/`
(`kpis.csv`, `results.json`, `metadata.json`, `histories/`, `actuation_trace.png` aus
`examples/actuation_trace.py`).

| Regler | Aktuatoren | Kurswinkel am Ende | Bremsweg | max. \|M_z\| issued | Verzug issued→realised | Rekuperation (mech.) |
|---|---|---|---|---|---|---|
| passiv | ideal | 1024° (dreht sich) | 81,9 m | – | 0 ms | – |
| passiv | dynamisch | 1026° (dreht sich) | 81,8 m | – | 0 ms (konstanter Befehl) | 22 kJ (Räder blockieren früh) |
| PI (TV) | ideal | 0,003° | 150,8 m | 320 N·m | 0 ms | – |
| PI (TV) | dynamisch | 0,003° | 151,1 m | 14 N·m | 15 ms | 332 kJ |

Befunde:

- **Der Trace macht einen bisher verdeckten Eingriff sichtbar:** Mit idealen Aktuatoren steht die volle Bremsung
  sofort an, das ABS greift im ersten Schritt ein und dreht das allokierte Giermoment (−84 N·m) auf +320 N·m am
  Reglerausgang. Mit Druckaufbau (τ = 35 ms) entfällt diese Spitze; dort ist issued = allocated, und realised folgt
  mit ≈ 15 ms. Die Gierratenspitze sinkt von 0,30 auf 0,05 °/s.
- **Geradeauslauf und Bremsweg** des PI hängen kaum an der Aktuatordynamik (Δ Bremsweg 0,25 m). Der PI bremst auf
  beiden Seiten nur bis zur Haftgrenze der Niedrigreibwertseite (Längskräfte links/rechts gleich), daher der Bremsweg
  151 m gegenüber 82 m des sich drehenden passiven Fahrzeugs: der bekannte Zielkonflikt Gierstabilität gegen Bremsweg.
- **Rekuperation:** Die PI-Bremsung liegt innerhalb der Rekuperationsgrenze (4 × 500 N·m), daher werden 332 kJ
  mechanisch am Rad zurückgewonnen (Bewegungsenergie bei 90 km/h: 375 kJ; der Rest geht an Roll- und Luftwiderstand
  und an die Reibbremse unter 2 m/s). Ohne Batterie- und Wirkungsgradmodell ist das keine elektrische Energiebilanz.

## 5. Tests

`tests/vehicle_model/test_actuation_chain.py` (11 Tests, davon 1 `slow`):

| Plan-Testliste Stellgrößenkette | Test |
|---|---|
| Alt- vs. Neu-IVMC | `test_split_mu_matrix.py` (beide Pfade); `test_single_owner_of_the_wheel_torques` |
| falsche Radreihenfolge, Spiegelung | `test_chain_is_mirror_symmetric_and_replays_identically` |
| deaktivierte ABS/TCS, Sättigung | bestehende Allokations- und Schlupfreglertests; ABS-Flag schaltet Rekuperation ab |
| Brake-only, Motor-only | `test_regen_enters_once_as_braking_torque`, `test_dynamic_chain_lags_and_converges` |
| Sensormodus C ohne Truth | `test_sensor_constrained_feedback.py` |
| ausgefallene Einzelbremse | `test_failed_brake_issues_no_torque_and_the_others_take_over`: FR 0, RR übernimmt bis zur Haftgrenze, FL/RL reduziert, Giermoment 0, Fehlbetrag 888 N als `infeasible` ausgewiesen |
| Reset/Replay | gleiche Befehlsfolge → bitgleiche Ausgaben |
| Split-µ mit/ohne Eingriff | `test_split_mu_braking_with_dynamic_actuators` (`slow`) |
| Trace-Konsistenz | `test_motion_controller_trace_stages_are_consistent`, `test_ideal_chain_is_a_bit_identical_pass_through`, `test_actuation_lag_estimate` |

## 6. Gate Stellgrößenkette

| Abnahmekriterium | Bewertung |
|---|---|
| Alle aktiven Funktionen nutzen die koordinierte Ausgabestelle; keine zwei Owner je Radmoment | erfüllt (Test); die Legacy-Kette nur auf ausdrückliche Wahl |
| Altes und neues Verhalten belegt | erfüllt: ideale Aktuatoren bitgleich zum bisherigen Verhalten, Legacy-IVMC weiter testbar |
| Split-µ-Differenz nachvollziehbar erklärt | erfüllt und hier erneut gegen die Tests geprüft |
| keine ungeklärten Regressionen | erfüllt: volle Suite nach Stellgrößenkette 739 bestanden, 2 übersprungen (vorher 727; +12 neue Tests) |

Offene Punkte für die folgenden Meilensteine: Allokation bleibt 2-DOF (B); Rekuperation ohne Batterie-,
Wirkungsgrad- und Thermomodell; der Verzug ist eine Kreuzkorrelationsschätzung und für konstante Befehle 0.
