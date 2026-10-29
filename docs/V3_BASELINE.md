# V3.0-Baseline: Abgleich des Masterplans V3.0 mit dem Code

Bezug: [Plan.md](../Plan.md) V3.0, Abschnitt 15, Schritt 2 („V3-Baseline dokumentieren“). Geprüfter Stand: Basis-Commit `8419ede` des Plans und die drei danach entstandenen Änderungen: CI-Aktionen auf v7, sensorbasierter Reibwertschätzer (G02), Aufteilung der Szenariomodule (G06).

Grundsatz des Plans: „Niemals annehmen, dass eine Funktion fehlt, nur weil sie in diesem Plan als Entwicklungsaufgabe aufgeführt ist.“ Jede Aussage unten ist am Code geprüft.

---

## 1. Prüfung der Verbesserungspunkte G01–G07

| Punkt | Aussage im Plan | Befund im Code | Stand |
|---|---|---|---|
| **G01** NMPC-Zulässigkeit | Statusklassen vorhanden; harte Zustands- und Stellratengrenzen fehlen; das dynamische NMPC hat weiche Reibkreisstrafen und eine eigene Radmomentenaufteilung | bestätigt: `dynamic_two_track_nmpc.py` hat nur Box-Grenzen auf den Stellgrößen, Kamm-Strafe ab 0,90 im Kostenterm, eigene Momentenaufteilung (seit M6 wird das erzeugte Giermoment ehrlich gemeldet); `nmpc.py` hat Stellgrößenklemmen im Vorhersagemodell | erledigt: `controllers/constraints.py` (hard/soft/penalty/advisory/plant), Zulässigkeit getrennt vom Solverstatus für LTV-MPC und dynamisches NMPC; harte Lenkratengrenze im dynamischen NMPC; Zulässigkeitsprüfung auch im kinematischen NMPC; beide NMPCs über die `ActuationLayer` (§5.3), Ausnahmen dokumentiert (`MOTION_CONTROL.md` 1.1); IVMC standardmäßig über die `ActuationLayer` (legacy wählbar); KKT-Residuen, Kostenverbesserung und Warmstartgüte in allen drei Optimierern (`MOTION_CONTROL.md` 3.2) |
| **G02** Sensorik | `EstimatedStateFeedback` reicht Reifen-, Radlast- und Reibwerttelemetrie an Allokation und Schlupfregelung weiter | bestätigt (dokumentiert in `information`). Zusätzlich lesen `motion_control.py`, `slip_control.py`, `abs.py`, `tcs.py`, `integrated_controller.py` Telemetrie (Fz, µ, F_y, κ). **Neu:** `estimation/sensor_friction.py` schätzt µ nur aus Sensoren und Stellbefehlen (Grundlage für Mode C) | erledigt (strukturell): Mode C `EstimatedStateFeedback(mode="C")`, Sensor-Telemetrie aus `SensorFrictionEstimator.as_telemetry`, YAML `feedback: sensor_constrained`; Kosten und Schätzfehler in ESTIMATION_ROBUSTNESS.md 2.1 |
| **G03** Stabilitätsgrenzen | Phase-Plane-Grenzen heuristisch | bestätigt: `phase_plane.py` berechnet β- und β̇-Grenzen aus Fahrbahn und Geschwindigkeit nach Formeln, nicht aus der Dynamik des Fahrzeugmodells | erledigt (M11): modellbasierte Gleichgewichte, Eigenwerte und Rückführregionen; die Heuristik bleibt als Vergleichsbasis ([NONLINEAR_STABILITY.md](NONLINEAR_STABILITY.md)) |
| **G04** Szenariozugriff | YAML-Experimente nur für Gierstabilität und fünf Regler | bestätigt (`experiments/spec.py`: drei Manöver, `passive/pi/lqr/gs_lqr/ltv_mpc`); NMPC-Pfadfolge, 4WS-, Split-µ- und Validierungsszenarien nur über Python | offen (M10/M12) |
| **G05** 4WS-Split-µ | Fall als `xfail` dokumentiert | bestätigt (Audit 4.7): Das Fahrzeug dreht sich in allen Varianten ein, auch ohne Regler und mit ABS-Option | erledigt: Konfigurationsmatrix `four_wheel_steering/split_mu_matrix.py`, Grenzabbildung als bestandene Tests, kein `xfail` mehr |
| **G06** Große Szenariomodule | sehr umfangreiche Dateien | **erledigt (strukturell):** Die drei Module (2612/2015/1665 Zeilen) sind Pakete mit Teilmodulen nach Verantwortung, das größte hat 836 Zeilen; alte Importpfade über Kompatibilitätsfassaden. Offen: Innerhalb der einzelnen Szenariofunktionen sind Manöverdefinition, Fahrer, Ausführung und Kennzahlen noch nicht getrennt | teilweise |
| **G07** Mehrzieloptimierung | PSO und Monte Carlo als Grundlage; keine gemeinsame Optimierungsinfrastruktur | bestätigt: `estimation/pso_optimizer.py` (einzielig), `simulation/robustness.py` (Monte Carlo); kein Pareto-Modul | offen (M14) |

## 2. Bestand je Bereich (Plan §2 und Schritt 2)

| Bereich | Vorhanden | Bemerkung |
|---|---|---|
| Fahrzeugmodelle | kinematisches und dynamisches Einspurmodell, lineares 2-DOF-Modell, 7-DOF, transientes 7-DOF, 14-DOF, Viertelfahrzeug | alle mit eigenen Verifikationstests (M1–M3, M8) |
| Regler | PI, LQR, GS-LQR, LTV-MPC (gemeinsame `MotionController`-Kette), SLSQP-NMPC kinematisch und dynamisch, 4WS-LUT und Gierdämpfung, ABS/TCS (alt und PI-Schlupfregelung), IVMC, Phase-Plane, aktive Wankstabilisierung | **nicht** über die gemeinsame Kette: beide NMPCs, `integrated_controller.py`, alte ABS/TCS, `FourWheelSteeringController` |
| Gemeinsame Schnittstellen | `VehicleController` (`compute_control`), `MotionLaw`, `TyreModel`, `RoadSurface`, `WheelForceAllocator`/`ActuationLayer` | die Allokation nutzen nur `motion_control.py` und `stability_control.py` |
| Solverdiagnostik | `SolverMonitor` (nominal/suboptimal/fallback/degraded, Zeitbudget gemeldet) in beiden NMPCs und der LTV-MPC | keine Constraint-Verletzungsprüfung, keine KKT-Residuen (G01) |
| Zustandsschätzung | EKF, kinematischer und gemischter Schwimmwinkelbeobachter, RLS-Reibwert (Ground Truth, deklariert), sensorbasierter Reibwert (neu), virtuelle Sensoren mit Abtastung, Latenz und Ausfall | keine Radlast- und Seitenkraftschätzer als eigene Komponenten (im Reibwertschätzer intern vorhanden) |
| Control Allocation | exakte box-beschränkte WLS, Reibkreisgrenzen, Aktuatorausfall, Statusmeldung | Aktuatordynamik und Stellraten nicht im Allokator |
| Experimente | YAML → Verifikation → Vergleich → CSV/JSON/PNG/GIF/Markdown, CLI, CI | ein Experimenttyp (`controller_comparison` im Sinne von §12); `run_experiment` verzweigt nicht nach Typ |
| Regressionsabdeckung | 44 Testdateien, 326 Testfunktionen (über Parametrisierung rund 470 Testfälle), Regressionsbaseline mit 6 Fällen, CI schnell (3.10, 3.13) und wöchentlich voll | Szenarien der alten Module nur über ihre Bestandstests |

## 3. Noch bestehende Architekturprobleme (priorisiert)

1. **Zwei Stellgrößenketten.** `MotionController` (Allokation, ABS/TCS) und die älteren Regler (NMPC, IVMC, 4WS) mit eigenen Momentenaufteilungen. Plan V3.0 §4.2 verlangt einen Pfad. (M10 §5.3)
2. **Ground-Truth-Zugriff in der Stellebene.** Allokation und Schlupfregelung lesen F_z, µ, F_y und κ aus der Telemetrie. Mode C braucht Schätzer für diese Größen. Der Reibwertschätzer ist der erste Baustein; Radlast und Seitenkraft schätzt er intern bereits, sie müssen als eigene Schnittstelle herausgelöst werden. (M10 §5.4)
3. **Constraint-Semantik.** Hart, weich, Strafterm oder Hinweisgrenze sind nirgends ausgewiesen; die Zulässigkeit der realisierten Stellgrößen wird nicht getrennt vom Solverstatus geprüft. (M10 §5.1–5.2)
4. ~~**Experiment-Runner ohne Typ-Schnittstelle.**~~ Erledigt: `experiments/types.py`, zweiter Typ `solver_benchmark` (EXPERIMENTS.md 2.0–2.1).
5. **Szenariofunktionen monolithisch.** Nach der Paketaufteilung enthält jede `simulate_*`-Funktion weiterhin Straße, Fahrer, Schleife, Kennzahlen; eine Umstellung auf `controller_benchmark.Scenario`/`run_scenario` wäre der nächste Schritt (G06, Rest).

## 4. Hinweise zum Plan selbst

- **§2.2** nennt „Kinematisches NMPC“. Es ist ein SLSQP-Single-Shooting-NMPC auf einem kinematischen Einspurmodell zur Pfadfolge (`nmpc.py`). Die Bezeichnung trifft zu, solange „C/GMRES“ nicht verwendet wird.
- **§2.3** führt GIF-Ausgabe als vorhanden: für YAML-Experimente seit M9 als optionales Format `gif`.
- **§4.1** sieht `verification/` nur bei Bedarf vor. Die Prüfungen liegen derzeit in `experiments/verification.py` und den Tests, was dazu passt.
- **§5.5** fordert, dass „ein erwartetes physikalisches Versagen als bestandener Test“ gilt. Der heutige `xfail` (G05) ist das Gegenteil: ein erwarteter *Softwarefehler*. Die Umsetzung sollte den Fall in eine Konfigurationsmatrix überführen, in der „4WS ohne ABS/ESC dreht sich ein“ ein bestandener Test der Grenzabbildung ist und „mit ABS + ESC stabil“ ein zweiter.
- **§5.6 Punkt 6** (CI-Zustand prüfen): Der erste CI-Lauf war grün (Python 3.10 und 3.13). Die Aktionen sind auf v7 (Node.js 24) aktualisiert; ein Lauf mit v7 steht nach dem nächsten Push aus.

## 5. Empfohlene Reihenfolge für M10

1. **G05:** 4WS-Split-µ-Matrix (fünf Konfigurationen aus §5.5) mit Grenzabbildung statt `xfail`. Klein, schließt eine dokumentierte Lücke und braucht nur vorhandene Bausteine.
2. **G01/G03 vorbereitend:** Constraint-Semantik und Zulässigkeitsprüfung der realisierten Stellgrößen, getrennt vom Solverstatus, zuerst für LTV-MPC und dynamisches NMPC.
3. **§5.3:** Dynamisches NMPC über die `ActuationLayer`, damit es Allokationsstatus und erreichte Kräfte meldet wie die anderen Regler.
4. **§5.4 Mode C:** Radlast-, Seitenkraft- und Reibwertschätzung als eigene sensorbasierte Komponenten, Allokation und Schlupfregelung wahlweise mit Schätzwerten; Benchmark Mode A/B/C.
5. **§12:** Typ-Schnittstelle im Experiment-Runner, dann `solver_benchmark` als zweiter Typ.

M11 (Gleichgewichte, lokale Stabilität, Stabilitätskarten) kann auf vorhandenen Bausteinen aufsetzen: `handling_analysis` für die Linearisierung, das 7-DOF-Modell, `controller_benchmark.run_scenario` für Trajektorien und `phase_plane.py` als Vergleichsbasis.

## 6. Abnahme M10 (Plan §5.6)

Stand: 2026-10-29. Bewertung je Kriterium mit Nachweis; „erfüllt mit Einschränkung“ heißt: Das Kriterium ist erfüllt, aber ein dokumentierter Rest bleibt.

| # | Kriterium | Bewertung | Nachweis |
|---|---|---|---|
| 1 | Dieselbe Stellgrößenkette oder begründete Ausnahmen | erfüllt | `ActuationLayer` (Allokation + ABS/TCS) in `MotionController`, beiden NMPCs, IVMC (Standard `actuation: allocation`, `legacy` wählbar und gekennzeichnet). Ausnahmen begründet: reiner 4WS-Lenkregler, passive Referenz (`MOTION_CONTROL.md` 1.1). Bericht trennt Soll, erreichbar, ausgegeben, realisiert (`diagnostics["actuation"]`) |
| 2 | Solverstatus und Zulässigkeit getrennt | erfüllt | `SolverReport` (Status, KKT-Residuen, Verbesserung, Warmstart) neben `FeasibilityReport` (hard → `feasible`, plant → `realisable`); Benchmark meldet beides getrennt; Tests `test_constraints.py`, `test_solver_diagnostics.py` (`MOTION_CONTROL.md` 3.1–3.2) |
| 3 | Keine undeklarierten Ground-Truth-Zugriffe | erfüllt mit Einschränkung | Jeder Regler und Schätzer deklariert `information`; Mode C (`feedback: sensor_constrained`) gibt keine Streckentelemetrie weiter (Spionagetest). Bei der Abnahmeprüfung gefunden und behoben: EKF und Reibwertschätzer wurden mit der *wahren* Geschwindigkeit initialisiert; jetzt aus den Raddrehzahlen (Test). Einschränkung: Modes A und B greifen deklariert auf idealisierte Größen zu; das Sensormodell bildet IMU-Beschleunigungen aus dem wahren Zustand (wie ein realer Sensor), der Lenkwinkel hinten kommt als Sensorkanal aus der Telemetrie |
| 4 | 4WS-Split-µ-Fall erklärt und abgesichert | erfüllt | Konfigurationsmatrix (sieben Konfigurationen), erwartetes Eindrehen ohne Bremsmomentausgleich als bestandener Grenztest; Widerspruch zum Validierungsfall aufgelöst (Validierung bremst mit ≈ 12 % Pedal); `SPLIT_MU_DYNAMICS.md`, `test_split_mu_matrix.py` |
| 5 | V2.0-Regressionsfälle bestehen | erfüllt | volle Testsuite lokal: 517 bestanden, 2 übersprungen, inklusive `slow` |
| 6 | CI-Zustand geprüft und dokumentiert | erfüllt mit Einschränkung | Letzter gepushter Commit (`992cc39`) grün auf Python 3.10 und 3.13 mit Aktionen v7 (Schnelllauf, Spezifikationsvalidierung). Der wöchentliche Volllauf mit `slow`-Tests ist auf GitHub noch nie gelaufen (kein `schedule`-/`workflow_dispatch`-Lauf); lokal nicht gepushte Commits sind noch nicht in der CI geprüft |
| 7 | YAML-Experimente und Visualisierungen unverändert verfügbar | erfüllt | Alle vorhandenen Spezifikationen gültig ohne `type` (Test `test_bundled_experiments_are_valid`); neue Typ-Schnittstelle additiv (`EXPERIMENTS.md` 2.0); Ausgaben und Dokumentationsbilder neu erzeugt (`tools/publish_doc_figures.py --check`) |

**Offene Punkte aus M10** (keine Abnahmehindernisse, aber dokumentierte Schwächen):

- Dynamisches NMPC: SLSQP bricht über die relative Kostenänderung ab, teils deutlich vor dem KKT-Punkt (relative Stationarität bis ≈ 1); Spurwechsel überschwingt um ≥ 0,5 m; Rechenzeit 0,2–1,6 s je Lösung, nicht echtzeitfähig (`MOTION_CONTROL.md` 3.2, `EXPERIMENTS.md` 2.1).
- Mode C: Gyro-Offset dreht das Fahrzeug bei Geradeausbremsung um ≈ 5°, Reibwert auf Eis überschätzt (0,3–0,47 statt 0,2), Geschwindigkeitsreferenz driftet bei Allradbremsung um bis zu 2,7 m/s, Blockieren unter ≈ 14 km/h (IVMC) (`ESTIMATION_ROBUSTNESS.md` 2.1). Gehört zu M12 (robuste Regelung).
- IVMC mit geschwindigkeitsgeführter Gewichtung: Split-µ-Bremsung bis zum Stand endet bei 6,6° Gierwinkel (`MOTION_CONTROL.md` 1.2).
- `soft`-Grenzen mit Schlupfvariablen nutzt kein Regler.
- ~~Stabilitätsgrenzen sind Faustregeln (advisory); modellbasierte Grenzen folgen in M11.~~ Mit M11 gibt es modellbasierte Gleichgewichte und Rückführregionen; die Heuristik ist als Vergleich bewertet (Abschnitt 7).

## 7. Abnahme M11 (Plan §6.5)

Stand: 2026-10-29. Grundlage: [NONLINEAR_STABILITY.md](NONLINEAR_STABILITY.md), Experimente `equilibrium_steer_sweep`,
`equilibrium_speed_stiffness`, `equilibrium_tv_4ws`, `stability_map_cornering`, Tests `test_nonlinear_stability.py` und
`test_experiments.py`.

| # | Kriterium | Bewertung | Nachweis |
|---|---|---|---|
| 1 | Gleichgewichtszustände besitzen überprüfbare Residuen | erfüllt | Residuen je Zustandsgruppe in SI-Einheiten mit dokumentierten Toleranzen (`RESIDUAL_TOL`), dazu Konvergenz der Lastverlagerung und Kontaktzustand; erreicht 1e-14 m/s², größtes Residuum/Toleranz 7,5e-7 in allen mitgelieferten Experimenten. Probe: `a_y = r v_x`, `a_x = −r v_y` auf 1e-6 (Test). Punkte ohne Gleichgewicht (TV-Faltung) werden mit erreichtem Residuum ausgewiesen, nicht angenommen |
| 2 | Lokale Stabilitätsaussagen stimmen mit der linearisierten Dynamik überein | erfüllt | Schrittweitenstudie der Jacobi-Matrix (Plateau 1e-8, Konvergenz in allen Punkten); nichtlineare Simulation nach kleiner Störung gegen `expm(A t)`: Abweichung ≤ 2,3 % bis 4° Lenkwinkel, 4,6 % bei 95 % Reibwertausnutzung, Auf-/Abklingen überall gleich, auch für Sattel und instabilen Fokus. Unabhängig: Quermoden auf 1–3 % gleich dem linearen Einspurmodell; Stabilitätswechsel bei der linearen kritischen Geschwindigkeit (30 m/s stabil, 40 m/s instabil bei v_krit = 30,08 m/s) |
| 3 | Stabilitätsregionen reproduzierbar berechnet | erfüllt | Läufe deterministisch; seriell und mit Prozess-Pool identische Karten, Sicherheiten und Verfeinerung (Test); Spezifikation, Commit und Versionen im Ergebnis |
| 4 | Numerische Unsicherheit separat ausgewiesen | erfüllt | Je Punkt Sicherheit (innen, Rand, empfindlich); Nachrechnung mit halbem Zeitschritt und doppelter Dauer mit gezählten Änderungen; Schranken des Anteils; eine Gitterverfeinerung (Abweichung ≤ 1,6 Prozentpunkte). Einschränkung: Mittelpunkte zwischen Nachbarn gleicher Klasse werden geerbt, nicht simuliert (ausgewiesen) |
| 5 | Wechsel von Reifen- oder Fahrzeugmodell über vorhandene Schnittstellen | erfüllt | Analyse auf jedem Modell der `SevenDOFModel`-Familie mit jedem `TyreModel`: getestet mit Magic Formula, Fiala, skalierter Steifigkeit (`SlipStiffnessScaledTyre`), Reifenrelaxation (15 Zustände, gleiches Gleichgewicht), K&C-Fahrwerk und verschobenem Schwerpunkt; im YAML über `vehicle`, `tyres`, `kc`, `model`, `cornering_scale_*`, `cog_shift_m` |
| 6 | Heuristische Phase-Plane-Regelung bleibt erhalten und dient als Vergleichsbasis | erfüllt | `phase_plane.py` unverändert, aktiv in IVMC; jede Karte vergleicht den Index `S < 1` mit „zurückgekehrt" und rechnet IVMC mit und ohne Hülle. Ergebnis: Die Hülle vergrößert die Region (96 % gegen 89 %); als Grenze der Rückführbarkeit ist die Heuristik stark konservativ (Übereinstimmung 32 % passiv) und an einem Punkt zu optimistisch |

**Offene Punkte aus M11:**

- Robuste Regler (Plan 6.4) werden nach M12 in die Kartenregler aufgenommen.
- Gleichgewichte nur mit gehaltener Geschwindigkeit; die Suche nach weiteren Gleichgewichten ist ein Startraster, kein
  Vollständigkeitsbeweis. Auf der Schleuderseite bei kleinem Lenkwinkel wurde keines gefunden (Existenz hängt an Effekten
  zweiter Ordnung).
- Wirkung von 4WS und TV auf die lokale Stabilität nur als feste Stellgrößen; Regelkreise nur über die Rückführregionen.
- Eine Karte (15 × 15, sechs Regler) braucht etwa 7 min auf 7 Prozessen; feinere oder weitere Betriebspunkte
  (Geradeausfahrt, Eis, hohe Geschwindigkeit) sind mit derselben Spezifikation möglich, aber noch nicht gerechnet.
- Beobachtet, nicht behoben: ESC und TV klassifizieren identisch (Totband wirkt nur nahe dem Ziel); das 4WS-Kennfeld
  vergrößert die Region nicht.

## 8. Abnahme M12 (Plan §7.5)

Stand: 2026-10-29. Grundlage: [ROBUST_CONTROL.md](ROBUST_CONTROL.md), Experimente `robust_benchmark_sine_with_dwell`
und `dynamic_nmpc_solver_benchmark`, Tests `test_robust_control.py` und `test_experiments.py`.

| # | Kriterium | Bewertung | Nachweis |
|---|---|---|---|
| 1 | Jeder Regler besitzt eine dokumentierte mathematische Formulierung | erfüllt | A–E mit Herleitung im Moduldocstring und in ROBUST_CONTROL.md 2 (Lyapunov-Argument für α, Fehlerdynamik des Beobachters, Tightening-Margen, RLS mit Gültigkeitsbedingungen, exakte Strafe der weichen NMPC-Grenzen); jede Herleitung durch einen Test gegen ihre Aussage geprüft (Eigenwerte ≤ −α, Beobachterzeitkonstante, wachsende und konvergierende Margen, RLS-Konvergenz) |
| 2 | Der Regler bleibt austauschbar | erfüllt | A–D sind `MotionLaw`s derselben Kette (Referenz, Allokation, ABS/TCS); E ist eine Option des NMPC; die Überwachung umschließt jeden `VehicleController`; im Benchmark per YAML wählbar |
| 3 | Unabhängige nichtlineare Strecke | erfüllt | 7-DOF-Zweispurmodell mit MF-Reifen; die Gesetze nutzen das lineare Einspurmodell; die Strecke weicht je Fall zusätzlich ab, der Regler bekommt nur die nominalen Parameter |
| 4 | Keine automatische Neuoptimierung vor jedem Versuch | erfüllt | Entwurf einmal aus dem nominalen Fahrzeug; die Fabrik verwirft die Streckenparameter. Die adaptive Regelung passt zur Laufzeit an (Plan D), aus einer vorab entworfenen Tabelle, ohne Optimierung |
| 5 | Verbesserungen gegenüber fester V2.0-Baseline nachgewiesen | erfüllt mit Einschränkung | Baseline: PI und GS-LQR aus M6, unverändert. Nachgewiesen: GS-LQR + DOB bei Parameterabweichungen (worn_rear_tyres 2,0 gegen 3,7 °/s), Tightening MPC bei µ = 0,5 (1,6 gegen 2,1 °/s) und ohne Kontrollverlust, constraint-aware NMPC (75 % gegen 42 % `nominal`, 0 % `degraded`). Einschränkung: keine neue Methode ist über alle Fälle besser; 120 von 144 Vergleichen mit dem PI sind auf mindestens einer Folgekennzahl schlechter |
| 6 | Solver- und Constraint-Fehler bleiben sichtbar | erfüllt | Solverstatus, Zulässigkeitsbericht, Infeasibility Rate, Fallback Rate und Degraded-Anteil je Lauf; die relaxierte Lösung der Tightening MPC wird als unzulässiger Plan gezählt (bis 40 % der Schritte), nicht versteckt; NMPC-Projektion mit Abstand protokolliert; Überwachung mit Modus, Grund und unterdrückten Ausgaben |
| 7 | Berichte enthalten Fälle, in denen ein fortgeschrittener Regler schlechter abschneidet | erfüllt | Bericht vergleicht jeden Regler mit der Baseline und listet Verschlechterungen; dokumentiert u. a.: Rückführregionen der Gesetze A–D kleiner als die des passiven Fahrzeugs (Hinterachslenkung),  Robust LQR verliert bei µ = 0,5 trotz linearer Reserve die Kontrolle, alle Gesetze hoher Verstärkung bei 80 ms Aktuatorverzögerung, DOB und adaptiver LQR bei gestörten Sensoren im Modus C; dazu die Entwurfsbefunde (DOB mit linearem Modell 14,9°, adaptive Schätzung ohne Gültigkeitsbedingungen 89°) |

**Offene Punkte aus M12:**

- Aktuatorverzögerung ist ungelöst: Ein verzögerungsbewusster Entwurf (Prädiktion über die Totzeit, Smith-Prädiktor
  oder MPC mit Verzögerungszuständen) fehlt; nur der PI und die Tightening MPC bleiben dort beherrschbar.
- Lineare Robustheitsnachweise (A, C) decken Sättigung, Reibwert und Verzögerung nicht ab; der Benchmark zeigt die Lücke.
- Mode-C-Schwächen aus M10 (Gyro-Offset, Reibwertschätzung auf Eis) wirken weiter; DOB und adaptive Schätzung leiden
  unter Gyro-Bias. Eine Bias-Schätzung im Beobachter wäre der nächste Schritt.
- ~~Die Robust-Regler sind noch nicht in die Rückführregionen von M11 aufgenommen (Plan 6.4).~~ Aufgenommen
  (`stability_map_robust`, `stability_map_rear_steer`, NONLINEAR_STABILITY.md 4.5). Befund: Alle Gesetze mit
  Hinterachslenkung aus dem linearen Entwurf, auch der unveränderte V2.0 GS-LQR, verkleinern die Rückführregion auf
  23–44 % (passiv 74 %); mit Giermoment allein 94 %. Eine Begrenzung der Hinterachslenkung außerhalb des linearen
  Bereichs fehlt.
- Benchmark nur mit Sine with Dwell; weitere Manöver (Lenkwinkelsprung, Split-µ) sind mit derselben Spezifikation
  möglich, aber noch nicht gerechnet.

