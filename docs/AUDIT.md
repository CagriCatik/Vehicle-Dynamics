# Repository-Audit – Ist-Zustand (Master Engineering Plan V2.0, Abschnitt 3)

**Datum:** 2026-10-27  
**Basis-Commit:** `4190954` (Branch `main`) plus uncommittete Änderungen; seit `041f48c` vollständig versioniert  
**Zweck:** Ist-Zustand, Testbaseline, Architekturinventar und inkrementeller Integrationsplan.

---

## 1. Git- und Working-Tree-Zustand

Stand bei Audit-Beginn: Eine große Umstrukturierung (`python/` → `src/vehicle_model/`, neue Verzeichnisse `configs/`, `docs/`, `examples/` und erweiterte Tests, ca. 20 000 Zeilen) lag uncommittet im Working Tree.

| Befund | Erledigung |
|---|---|
| Uncommittete Umstrukturierung | Mit Commit `041f48c` auf `main` gesichert. Die Struktur unter `src/vehicle_model/` ist verbindlich (Plan.md, Abschnitt 18). |
| `matlab/` gelöscht, kein Ersatz | Entfernung bewusst entschieden. Der Code ist über die Git-Historie (`4190954`) erreichbar. |
| `python/cgmres_nmpc`, `python/vehicle_model`, `python/vehicle_visualization` | Nach `src/vehicle_model/` portiert, inklusive Daten, Reifen-YAMLs und Racetracks. |
| `tests/cgmres_nmpc/test_controller.py`, `tests/vehicle_visualization/test_plan_view.py` gelöscht | Nachfolger: `tests/vehicle_model/test_nmpc_controller.py` und `test_plan_view.py`. Die 1:1-Abdeckung der alten Testfälle ist noch offen (M0). |
| Altes `Plan.md` („GEMINI – MASTER ENGINEERING INSTRUCTION“) | Durch Master Engineering Plan V2.0 ersetzt. |

## 2. Testbaseline

```
.venv/bin/python -m pytest -q
140 passed, 2 skipped in 613 s (10 min 13 s)
```

- Alle Tests sind grün.
- Eine Laufzeit von 10 Minuten ist für den Entwicklungszyklus zu lang. Es gibt keine Trennung zwischen schnellen Unit-Tests und langsamen Szenario- und E2E-Tests (keine `pytest`-Marker).

**Nach M0:** 21 Tests mit Laufzeit ≥ 2 s tragen den Marker `slow`. Drei davon verursachen rund 410 der 600 s (`test_multi_mu_dashboard_and_report_generation` 212 s, `test_circle_handling_limit_scaling` 114 s, `test_parallel_parking_sweep_reduction` 85 s).

```
pytest -m "not slow"   # 141 passed, 2 skipped, 1 xfailed, 21 deselected in 13 s
pytest                 # 162 passed, 2 skipped, 1 xfailed in 579 s
```

## 3. Architekturinventar (`src/vehicle_model/`)

| Paket | Inhalt | Bezug Plan V2.0 |
|---|---|---|
| `vehicle_models/kinematic_single_track.py` | Zustand `[x, y, yaw, v, steer]`, nur Frontlenkung | V1 teilweise: **keine Hinterachs- bzw. 4WS-Geometrie** |
| `vehicle_models/dynamic_single_track.py` | Nichtlineares Einspurmodell | V2 nichtlinear vorhanden; **lineares 2-DOF-Referenzmodell fehlt** |
| `vehicle_models/seven_dof.py` | 7-DOF-Zweispurmodell, quasistatische Radlastverlagerung (Fixpunktiteration), austauschbare `TyreModel` | V3 vorhanden |
| `vehicle_models/fourteen_dof.py`, `suspension/quarter_car.py` | Aufbau- und Vertikaldynamik | V4/V5 als Ansatz vorhanden |
| `tire_model/` | `TyreModel`-Protocol mit `TyreProvenance`; Linear, Fiala/Brush, vereinfachtes Combined Slip, Pacejka (Querrichtung) | Abschnitt 7 weitgehend abgedeckt; **kein transientes Relaxationsmodell** |
| `roads/surface.py` | Uniform, Split-µ, Transition, Patch, Diagonal, Curved, Checkerboard | Abschnitt 12 gut abgedeckt (Reibwert pro Radaufstandspunkt) |
| `actuators/` | Lenk-, Brems- und Antriebsaktuatoren | Abschnitt 9 als Ansatz |
| `controllers/` | ABS, TCS, Torque Vectoring, Active Roll, Phase Plane, Fahrermodell, integrierter Regler, 2 NMPC | Abschnitt 10: **LQR und Gain-Scheduled LQR fehlen**; keine eigenständige Control Allocation |
| `estimation/` | EKF-Schwimmwinkel, Reibwertschätzer, Sensoren, PSO | Abschnitt 11 als Ansatz |
| `simulation/` | Szenario-Runner, Split-µ, 4WS, Multi-µ, Handling Limits, Validierungsmatrix, KPI, Report | Abschnitt 14: Szenarien sind über mehrere sehr große Module verteilt (bis 2600 Zeilen pro Datei) |
| `visualization/` | Plan View, Animation, NMPC-Dashboard | Abschnitt 13: Visualisierung ist vom Kern getrennt |

**Lücke zu Abschnitt 6:** Es gibt kein eigenständiges Modul `handling_analysis`. `simulation/handling_limits.py` enthält stationäre Kreisfahrt, Untersteuergradient und g-g-Diagramm. Es fehlen eine Eigenwert- und Frequenzganganalyse der linearisierten Querdynamik (Eigenwerte gibt es nur in `quarter_car.py`), die charakteristische und die kritische Geschwindigkeit als analytische Referenz sowie standardisierte transiente Kennwerte.

## 4. Befunde gegen die Grundsätze des Plans

### 4.1 Falsche Bezeichnung „C/GMRES“ (Abschnitt 3, 10.1): kritisch

`controllers/nmpc.py` beschreibt sich als „NMPC … with C/GMRES formulation“ und `NMPCControllerCGMRES`. Tatsächlich wird das Problem per Single Shooting (nur Stellgrößen als Entscheidungsvariablen) mit **`scipy.optimize.minimize(method="SLSQP")`** gelöst. Dasselbe gilt für `DynamicTwoTrackNMPC`, das sich fälschlich als „multiple shooting“ mit „strict“ Reibkreis-Constraints beschrieb (tatsächlich: weiche Strafterme). Eine Kontinuierung oder ein GMRES-Verfahren gibt es nicht.

Die falsche Bezeichnung steht auch an diesen Stellen:
- `controllers/nmpc.py:1`, `:293`, `:392` (`name` = „NMPC Path Tracking Controller (C/GMRES)“)
- `controllers/__init__.py`, `visualization/nmpc_dashboard.py`
- `examples/generate_maneuver_animations.py` (Szenario `cgmres_nmpc_slalom`), `examples/cgmres_nmpc_simulation.py`
- `configs/controllers/cgmres_nmpc.yaml`, `docs/cgmres_nmpc.md`, `docs/ARCHITECTURE.md:22`, `docs/VISUALIZATION_GUIDE.md:100`, `src/vehicle_model/README.md:33`

Der Original-Code in `HEAD` hat SLSQP immerhin im Docstring offengelegt. Die Portierung hat die Bezeichnung „C/GMRES“ dagegen verstärkt.

### 4.2 Parameterherkunft (Abschnitt 4.2, 7): mittel

- `configs/tyres/simplified_combined_oem_{front,rear}.yaml` geben als Quelle „Calibrated D-segment passenger car … characteristics“ an und verwenden den Namen `oem`. Eine Kalibrierung ist nicht belegt, und die Kategorie (`data_class`) fehlt. Diese Daten müssen als synthetisch gekennzeichnet werden.
- `configs/vehicles/seven_dof_passenger.yaml` beschreibt das Fahrzeug als „Realistic D-segment“ mit `mass: 1200 kg`. Das ist für ein D-Segment-Fahrzeug untypisch leicht. Es fehlen außerdem `id`, `version` und Herkunftskategorie.
- Positiv: Die Reifen-YAMLs haben bereits einen `provenance`-Block mit `data_class: synthetic`, Gültigkeitsbereichen und Limitations.

### 4.3 Kontaktverlust (Abschnitt 8): mittel, *behoben in M2*

`SevenDOFModel.compute_normal_loads` begrenzt negative Radlasten auf 0 (`np.maximum`) und skaliert danach **alle** Radlasten auf `m·g` zurück. Das erhält die Vertikalkraftbilanz, verfälscht aber die Wank- und Nickmomentenbilanz der übrigen Räder. Ein Kontaktverlust wird weder als Ereignis gemeldet noch protokolliert. Das ist genau die „stille Näherung“, die Abschnitt 8 und 13 ausschließen.

**Behebung (M2, `1889c7a`):** [`vehicle_models/normal_loads.py`](../src/vehicle_model/vehicle_models/normal_loads.py) behandelt den Kontakt einseitig. Die statisch unbestimmte Verwindungsmode wird auf das zulässige Intervall begrenzt. Ein abhebendes Rad trägt dann exakt null, und Kraft-, Nick- und Wankbilanz bleiben exakt. Kippen (Druckmittelpunkt außerhalb des Stützpolygons) wird als `tip_over` mit dem nicht abstützbaren Moment gemeldet. Telemetrie und `SimulationHistory` führen den Kontaktzustand mit und listen Abhebe- und Aufsetzereignisse.

**Gleicher Befund in den Vertikalmodellen (M2, gefunden bei der Behebung):** `QuarterCarModel` und `FourteenDOFModel` begrenzten die ausgegebene Radlast auf ≥ 0. Die Bewegungsgleichung der ungefederten Masse verwendete aber weiter die beidseitige Reifenfeder: Ein abgehobenes Rad wurde von einer nicht existierenden Reifen-Zugkraft zurückgezogen. Im Viertelfahrzeug war außerdem das Vorzeichen der ausgegebenen Radlast falsch (`static + k_t (z_u − z_r)`): Hob das Rad ab, stieg die gemeldete Last (3 cm über der Gleichgewichtslage: 10,4 kN statt 0). Beides ist behoben, Tests in `test_vertical_contact.py`.

### 4.4 Kleinere Befunde

- Der Docstring in `seven_dof.py` verweist auf `docs/vehicle_dynamics/seven_dof_model.md`. Diese Datei existiert nicht. *(erledigt in M0)*
- ~~`pyproject.toml`-Package-Data ist defekt.~~ Korrektur: Die Reifen-YAMLs liegen doppelt vor, in `configs/tyres/` und als Fallback in `src/vehicle_model/config/tyres/`. Das ist eine Konfigurationsduplikation (Plan.md, Abschnitt 2.2). Seit M0 prüft `test_config_provenance.py`, dass die Kopien identisch bleiben.
- In `HANDLING_LIMITS.md` war der statische Vorderachslastanteil mit 52,5 % angegeben, korrekt ist l_r/L = 53,6 %. *(erledigt in M0)*
- Die NMPC-Dokumentation enthielt Parameterwerte, Code-Ausschnitte und Benchmark-Zahlen, die nicht zum Code passten oder nicht belegt waren. *(korrigiert in M0)*
- „Calibrated LUT“ bezeichnet in den 4WS-Szenarien eine synthetisch abgestimmte Tabelle. Die Bezeichnung ist missverständlich, aber nicht als reale Kalibrierung ausgewiesen. Offen.
- `scipy` wurde in `controllers/nmpc.py`, `controllers/dynamic_two_track_nmpc.py`, `steering/lut.py` und `handling_analysis/linear_single_track.py` importiert, war aber weder in `pyproject.toml` noch in `requirements.txt` deklariert; eine frische Installation scheiterte beim Import. *(behoben in M5)*
- Das bestehende ABS (`controllers/abs.py`) regelt mit festen Schlupfschwellen (κ = −0,20) und ohne Gedächtnis. Mit dem MF-Reifen blockieren bei µ = 0,2 die Räder (Bremswirkungsgrad 0,66, ohne ABS 0,62), weil der Schlupf der Kraftspitze mit µ sinkt. Ersatz in M5: `controllers/slip_control.py` (0,98 / 1,02 / 1,08 auf µ 1,0 / 0,5 / 0,2). Die Bestandsszenarien nutzen weiter das alte ABS. Offen.
- In `actuators/brake_actuator.py` war die Zeitkonstante des Rekuperationsmoments fest codiert (0,02 s), und keiner der Aktuatoren hatte die geforderte ideale Variante (Plan.md, Abschnitt 9). *(behoben in M5)*
- Sehr große Szenario-Module (`split_mu_scenarios.py` 2612, `four_wheel_steering_scenarios.py` 2015, `validation_matrix.py` 1665 Zeilen) widersprechen dem Grundsatz „ein Modul, eine Verantwortung“.

### 4.5 Geradeausfahrt des dynamischen Einspurmodells (Abschnitt 15.1): hoch, *behoben in M1*

`DynamicSingleTrack` erzeugte bei Lenkwinkel null und Schräglaufwinkel null Achsseitenkräfte von etwa 800 N (vorn) und 1600 N (hinten). Das Fahrzeug gierte und driftete bei Geradeausfahrt (nach 2 s: ψ ≈ −0,03 rad).

Korrigierte Ursache: nicht die MF6.2-Dateien (dieser Pfad ist inaktiv, weil die Dateien `VEHICLE_*_TIRE` fehlen), sondern die Verschiebungsterme `Shfy`, `Shry`, `Svfy` und `Svry` der vereinfachten Magic Formula in `default.yaml`. Ihre Herkunft ist nicht dokumentiert (vermutlich ein Fit an den Autoverse-Simulatorlauf).

Behebung: Die Verschiebungsterme sind null; die alten Werte bleiben über Commit `4190954` reproduzierbar. `default.yaml` liegt ebenfalls doppelt vor (`configs/` und Paket-Fallback). Ein Test sichert jetzt die Gleichheit beider Kopien.

### 4.6 Reifen `pacejka_*_synthetic` (Abschnitt 7): mittel, *behoben in M6 (Seitenzug korrigiert, Standardreifen umgestellt)*

Die Standardreifen des 7-DOF-Modells erreichen Fy = µ·Fz bereits bei etwa 2° Schräglaufwinkel und werden dort hart abgeschnitten. Die normierte Schräglaufsteifigkeit beträgt etwa 36/rad, für Pkw-Reifen typisch sind 15–25/rad. Die harte Begrenzung ist eine nicht stetige, nicht gemeldete Näherung. Das Referenzfahrzeug verwendet deshalb die Fiala-Reifen.

Bei der Untersuchung in M3 kamen zwei weitere Mängel des Adapters hinzu. $F_x$ ist linear bis zu einer harten Grenze (`min(c_κ F_z κ, µ F_z)`). Und µ skaliert die gesamte Querkraftkurve, also auch die Schräglaufsteifigkeit: Bei µ = 0,3 sinkt sie um 70 %. Physikalisch hängt die Steifigkeit bei kleinem Schlupf kaum von µ ab. Außerdem beschrieb `docs/TYRE_MODELS.md` den Adapter als „flagship MF 6.2“. Für Fiala war ein Rückstellmoment dokumentiert, das nicht implementiert ist, und mehrere Formeln, Dateinamen und Ausgaben stimmten nicht mit dem Code überein.

**Stand nach M3:** Die Grenzen stehen in den Parametersätzen (`limitations`) und in `docs/TYRE_MODELS.md`, Abschnitt 3.5. Als Ersatz steht `magic_formula_passenger_synthetic` bereit (MF 5.2-Teilmenge, `de523ef`). Der Adapter bleibt vorerst Standardreifen von `seven_dof_passenger` und damit der Bestandsszenarien. Ein Wechsel ändert deren Ergebnisse und abgestimmte Schwellwerte und ist deshalb eine eigene Entscheidung.

**Versuch (M4, in einem separaten Worktree, nicht übernommen):** Mit `magic_formula_passenger_synthetic` als Standard für `seven_dof_passenger` und das 14-DOF-Modell schlagen 6 von 319 Tests fehl. Vier sind erwartet: drei Regressionsbaselines und der Test der Reifenwahl-Reihenfolge. Zwei zeigen eine echte Abhängigkeit:
- `test_e2e_dynamic_nmpc_trajectory_tracking_kamm_circle`: Das dynamische NMPC schafft den Spurwechsel nicht mehr (`Y > 2,5 m` verfehlt).
- `test_split_mu_braking_active_yaw_damping` (4WS-Gierdämpfung bei Split-µ-Bremsung): Schwellwert verfehlt; der Betrag wurde in diesem Lauf nicht protokolliert.

Beide Regler sind auf den sehr steifen Legacy-Reifen abgestimmt (C_α/F_z ≈ 33/rad statt 18/rad). Der Wechsel des Standardreifens wird deshalb zusammen mit der Neuabstimmung dieser Regler in M5 (Gierdämpfung) und M6 (NMPC) vorgenommen, nicht durch Lockern der Tests.

**Seitliche Zugkraft (M6, behoben):** Die Option `symmetric` des Adapters zog nur F_y(κ = 0, α = 0) ab. Die laterale MF hängt aber über ihre Combined-Slip-Terme von κ ab. Beim Bremsen und Antreiben erzeugte der Reifen deshalb bei α = 0 bis zu ~320 N Seitenkraft je Rad, an allen Rädern mit gleichem Vorzeichen. Eine symmetrische Geradeausbremsung auf gleichmäßiger Fahrbahn endete mit Fahrermodell 25° neben der Ausgangsrichtung. Der Adapter verwendet jetzt den in α ungeraden Anteil ½(F_y(α) − F_y(−α)). Ein generischer Symmetrietest für alle Reifenmodelle schlägt am alten Adapter fehl. Nur die Regressionsfälle mit diesem Reifen haben sich geändert; alle übrigen Tests bleiben grün (395 bestanden).

**Folgerung für das dynamische NMPC (M6):** Mit dem korrigierten Reifen und der neuen Lösungsklassifikation schafft das dynamische NMPC den Spurwechsel auch auf der MF-Strecke (Y = 3,7 m bei Soll 3,5 m). Die in M4 beobachtete Abhängigkeit ist damit entfallen.

**Umstellung des Standardreifens (M6):** `seven_dof_passenger` (jetzt mit `tyres:`-Abschnitt in der YAML) und der Code-Standard von 7-DOF- und 14-DOF-Modell verwenden `magic_formula_passenger_synthetic`. Im Probelauf scheiterten nur erwartete Tests: drei Regressionsbaselines, neu erzeugt, und der Test der Reifenwahl. Dazu kam das Reibkreis-Kriterium des NMPC-E2E-Tests. Es bezog die Ausnutzung auf µ·F_z, beim MF-Reifen liegt die Spitzenreibung bei geringer Last aber über µ (Lastempfindlichkeit, M3). Das Kriterium bezieht sich jetzt auf die Spitzenreibung des Reifens (Ausnutzung ≤ 1,05). Der Spurwechsel gelingt auch mit dem MF-Reifen (Y = 3,7 m).

### 4.7 4WS-Split-µ-Bremsszenario (M6): mittel, offen (M9)

In `simulate_split_mu_braking_4ws` (100 km/h, µ 1,0 / 0,2) dreht sich das Fahrzeug mit und ohne Gierdämpfung über die Hinterachslenkung ein (Gierwinkel ~87–90°), mit beiden Reifensätzen, auch mit ABS-Option und bei µ_rechts = 0,7. Der Test `test_split_mu_braking_active_yaw_damping` verglich die maximale Querabweichung zweier eindrehender Fahrzeuge und bestand zufällig. Er prüft jetzt das eigentliche Ziel (kein Eindrehen, Querabweichung < 0,5 m) und ist ein striktes `xfail` mit dokumentiertem Grund. Für die Stabilisierung braucht es ABS und ein Bremsgiermoment (siehe `stability_control.py`); das Szenario nutzt beides nicht. Überarbeitung zusammen mit der Aufteilung der Szenariomodule.

### 4.8 Virtuelle Sensoren (M7): niedrig, *behoben in M7*

`estimation/sensors.py` nannte im Docstring eine Latenz, `configs/estimators/sensors.yaml` ein `lag_tau`. Beides war nicht implementiert. Der Reibwertschätzer liest Reifenkräfte, Schlüpfe und Radlasten aus der Telemetrie und ist damit keine realisierbare Schätzung; er ist jetzt als Ground-Truth-Referenz deklariert (offen: sensorbasierter Ersatz).

### 4.9 Straßenanregung im Viertelfahrzeug (M8): niedrig, *behoben in M8*

`QuarterCarModel.step` hält die Straße über den RK4-Schritt konstant: Die Anregung eilt um dt/2 nach, und die Energiebilanz ist nur erster Ordnung genau (15 % bei dt = 1e-4 s und 4 Hz). Neu ist `step_with_road`, das die Straße an den Stützstellen auswertet (Bilanz 4e-6). In `SUSPENSION_DYNAMICS.md` waren für Skyhook „> 20 %“ und für Groundhook „maximiert den Kraftschluss“ angegeben. Gemessen sind −18 % (ISO C) bzw. eine Verschlechterung der Radlastschwankung auf breitbandiger Anregung; die Doku ist korrigiert.

## 5. Zuordnung der Befunde zur Roadmap (Plan.md, Abschnitt 19)

| Meilenstein | Offene Punkte aus diesem Audit | Abnahme |
|---|---|---|
| **M0 – Repository Audit & Stabilization** | ~~Stand sichern~~ (`041f48c`). ~~C/GMRES-Bezeichnung korrigieren~~ (`0c5877a`). ~~Parameterherkunft korrigieren~~ (`1e45e33`). ~~Toten Doku-Verweis beheben~~ (`1e45e33`). ~~Regressionstests für die Bestandsmodelle~~ (`f8d164a`, `test_regression_baseline.py`). ~~Abdeckung der portierten Alt-Tests prüfen~~ (alle alten Testfälle übernommen, keine abgeschwächt). ~~`pytest`-Marker `slow`~~ (Schnelllauf 13 s). | `grep -ri "c/gmres"` findet nur noch Hinweise auf „nicht implementiert“; `pytest -m "not slow"` < 60 s; volle Suite grün |
| **M1 – Virtual Reference Vehicle & Handling Fundamentals** | ~~Geradeausfahrt-Defekt von `DynamicSingleTrack`~~ (`8525151`). ~~Referenzfahrzeug mit dokumentierter Herkunft und Konsistenzprüfung~~ (`731033f`). ~~Lineares 2-DOF-Einspurmodell und Paket `handling_analysis`~~ (`f56b01b`). ~~4WS-Geometrie im kinematischen Einspurmodell~~ (`50e123d`). ~~Abgleich linear gegen 7-DOF und Kreisfahrt~~. Offen: Vereinheitlichung der Schnittstellen von kinematischem und dynamischem Einspurmodell (gemeinsame Parameter und Integratoren gibt es bereits; ein gemeinsames Modell-Interface folgt mit der Simulation Engine in M9). Bericht: `docs/HANDLING_ANALYSIS.md`. | Analytische und numerische Gierverstärkung < 1 % Abweichung bei kleinem a_y (erreicht: ≤ 0,32 %); Prüfkriterien V1 (erfüllt) |
| **M2 – Nonlinear Four-Wheel Vehicle Dynamics** | ~~Kontaktverlust explizit behandeln: Ereignisflag in der Telemetrie, momentenkonsistente Lastverteilung statt Clip-und-Renormierung~~ (`1889c7a`). ~~Einseitiger Reifenkontakt in Viertelfahrzeug und 14-DOF, Vorzeichen der Viertelfahrzeug-Radlast~~. ~~Kräfte-, Momenten- und Radmomentenbilanz, Symmetrie- und Konvergenztests~~ (`test_seven_dof_balances.py`, `test_normal_loads.py`). Offen: Fixpunkt-Toleranz der Lastverlagerung (1e-6 m/s²) begrenzt die Selbstkonvergenz auf ~1e-8; die stabilitätsbasierte Unterteilung löst die schnelle Radschlupfmode nicht genau auf (Fehler bei dt = 5 ms < 1e-6). | Bilanzen exakt (rel. 1e-12); RK4-Ordnung 4,2–5,1; Abhebe- und Kippschwellen analytisch bestätigt |
| **M3 – Advanced Tyre Physics** | ~~Transientes Reifenmodell mit Relaxationslänge~~ (`358c899`, `TyreRelaxation`, `TransientSevenDOFModel`). ~~Dokumentierte, parametrierte Magic Formula mit Combined Slip und Rückstellmoment~~ (`de523ef`). ~~Grenzen des Legacy-Adapters dokumentieren, `TYRE_MODELS.md` an den Code angleichen~~. ~~Standardreifen auf die MF umgestellt~~ (M6, siehe 4.6). Offen: Sturz, Fülldruck, lastabhängige Relaxationslänge (nicht implementiert und nicht behauptet). | Grenzfälle, Sättigung, Vorzeichen, Ellipse (Gitter, F_z 1,5–7 kN, µ 0,2–1), µ-Selbstähnlichkeit, analytische Relaxationsantwort; Austausch ohne Änderung des Fahrzeugkerns |
| **M4 – Steering, Suspension Kinematics & Load Transfer** | ~~Ackermann-Geometrie und Einzelradlenkwinkel~~ (`steering/geometry.py`). ~~Synthetische K&C-Kennfelder, Bump, Roll und Compliance Steer~~ (`suspension/kinematics.py`). ~~Rollzentren und Wanksteifigkeitsverteilung in der Lastverteilung~~. ~~Analyse des Fahrwerkseinflusses auf das Eigenlenkverhalten~~ (`understeer_budget.py`, `examples/kc_understeer_analysis.py`). ~~M1-Aussage zum Eigenlenkgradienten bei kleinem a_y korrigiert~~. Offen: Lenkübersetzung (M5); Sturz, Nachlauf, Spreizung und Lenkrollradius (nicht modelliert und nicht behauptet). Bericht: `docs/CHASSIS_KINEMATICS.md`. | Ackermann-Bedingung (rel. 1e-12), Querlastsumme = m h, deaktivierte K&C = Basismodell, stationäre Gierrate gegen lineares Budget ≤ 0,28 % |
| **M5 – Actuators, ABS, TCS, ESC & Torque Vectoring** | ~~Eigenständige Control Allocation mit Machbarkeitsrückmeldung~~ (`allocation/`, `9a5f3d1`). ~~Schlupfregelung, die auch bei niedrigem µ funktioniert~~ (`slip_control.py`, `38b115e`). ~~ESC/TV über die Allokation, Split-µ, Aktuatorausfall~~ (`stability_control.py`). ~~Ideale Aktuatorvarianten, Lenkübersetzung, `tau_regen` konfigurierbar~~ (`bdb2454`). ~~`scipy` als Abhängigkeit deklariert~~ (`a6c9b9c`). Offen: Bestandsregler (`abs.py`, `integrated_controller.py`) auf Schlupfregelung und Allokation umstellen; Aktuatordynamik in die Szenarien einbinden; Split-µ mit Lenkeingriff (M6). Bericht: `docs/CONTROL_ALLOCATION.md`. | Split-µ stabil (passiv: Eindrehen), ABS ≥ 95 % µg auf µ 0,2–1,0, TV auf 5 % an der Referenz, Bremsenausfall kompensiert und als `infeasible` gemeldet, ungültige Anforderungen abgewiesen |
| **M6 – Advanced Vehicle Motion Control** | ~~Einheitliches Controller-Interface und Regelkette~~ (`motion_control.py`). ~~PI-, LQR-, GS-LQR-, LTV-MPC-Gesetze, integrierte 4WS/TV~~ (`yaw_laws.py`). ~~Solverdiagnostik und Degraded Mode für NMPC und MPC~~ (`solver_diagnostics.py`). ~~Benchmark mit gleicher Strecke, identischen Szenarien und Kennzahlen, Robustheitsvarianten~~ (`controller_benchmark.py`). ~~Seitliche Zugkraft des Legacy-Reifens~~ (4.6). Offen: Kennzahl für den Rückschwinger nach Lenkende (LTV-MPC 4WS+TV, Spitze +0,33 rad/s); Zustands- und Endkostenterme der MPC; NMPC im Spurwechsel häufig `degraded` (15 Iterationen); 4WS-Split-µ-Szenario (4.7). Bericht: `docs/MOTION_CONTROL.md`. | Sine with Dwell nach FMVSS-Vorbild: passiv dreht sich ein, alle Regler Verhältnis ≤ 0,02 und Querversatz ≥ 2,2 m, auch auf zwei Streckenvarianten; Riccati, Stabilität und exakte Folge auf dem Entwurfsmodell |
| **M7 – Virtual Sensors, Estimation & Robustness** | ~~Sensor-Abtastung, Latenz, Lenkwinkelverzögerung, Ausfälle~~ (behauptet, aber fehlend; 4.8). ~~Informationszugriff der Beobachter deklariert und durchgesetzt~~. ~~Kinematischer und gemischter Schwimmwinkelbeobachter, Beobachterstudie~~. ~~Regler mit Sensoren und Beobachter~~. ~~Monte-Carlo-Robustheit mit Streuungen und Grenzverletzungen~~. Offen: sensorbasierter Reibwertschätzer (der vorhandene liest Telemetrie-Reifenkräfte); UKF; Aktuator- und Sensorfehler als Variationsgrößen. Bericht: `docs/ESTIMATION_ROBUSTNESS.md`. | Beobachter nur mit Messungen (Signatur und Studie erzwingen es); gemischter Beobachter < 1,3° RMS in vier Fällen; Monte Carlo wiederholbar (Seed), Passiv 25/25, PI 0/25 Verletzungen |
| **M8 – Vertical Vehicle Dynamics & Advanced Chassis Systems** | ~~Analytische Referenzen (Moden, Frequenzgänge, Energie- und Leistungsbilanz)~~. ~~ISO-8608-Straßen und Einzelhindernis~~. ~~Halteglied-Fehler der Straßenanregung im Viertelfahrzeug~~ (4.9). ~~Unbelegte Skyhook/Groundhook-Aussagen der Doku korrigiert~~. Offen: Halbfahrzeug (nach Bedarf), ISO-2631-Komfortbewertung, Groundhook-Abstimmung. Bericht: `docs/VERTICAL_DYNAMICS.md`. | Energieerhaltung 1e-6, Energiebilanz 1e-4, Frequenzgang ≤ 1 %, ISO-Dichte, Zielkonflikt Skyhook/Groundhook |
| **M9 – Virtual Engineering & Research Laboratory** | Große Szenario-Module aufteilen (`split_mu_scenarios.py`, `four_wheel_steering_scenarios.py`, `validation_matrix.py`). | gemäß Plan |
