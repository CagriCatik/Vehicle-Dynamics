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

### 4.3 Kontaktverlust (Abschnitt 8): mittel

`SevenDOFModel.compute_normal_loads` begrenzt negative Radlasten auf 0 (`np.maximum`) und skaliert danach **alle** Radlasten auf `m·g` zurück. Das erhält die Vertikalkraftbilanz, verfälscht aber die Wank- und Nickmomentenbilanz der übrigen Räder. Ein Kontaktverlust wird weder als Ereignis gemeldet noch protokolliert. Das ist genau die „stille Näherung“, die Abschnitt 8 und 13 ausschließen.

### 4.4 Kleinere Befunde

- Der Docstring in `seven_dof.py` verweist auf `docs/vehicle_dynamics/seven_dof_model.md`. Diese Datei existiert nicht. *(erledigt in M0)*
- ~~`pyproject.toml`-Package-Data ist defekt.~~ Korrektur: Die Reifen-YAMLs liegen doppelt vor, in `configs/tyres/` und als Fallback in `src/vehicle_model/config/tyres/`. Das ist eine Konfigurationsduplikation (Plan.md, Abschnitt 2.2). Seit M0 prüft `test_config_provenance.py`, dass die Kopien identisch bleiben.
- In `HANDLING_LIMITS.md` war der statische Vorderachslastanteil mit 52,5 % angegeben, korrekt ist l_r/L = 53,6 %. *(erledigt in M0)*
- Die NMPC-Dokumentation enthielt Parameterwerte, Code-Ausschnitte und Benchmark-Zahlen, die nicht zum Code passten oder nicht belegt waren. *(korrigiert in M0)*
- „Calibrated LUT“ bezeichnet in den 4WS-Szenarien eine synthetisch abgestimmte Tabelle. Die Bezeichnung ist missverständlich, aber nicht als reale Kalibrierung ausgewiesen. Offen.
- Sehr große Szenario-Module (`split_mu_scenarios.py` 2612, `four_wheel_steering_scenarios.py` 2015, `validation_matrix.py` 1665 Zeilen) widersprechen dem Grundsatz „ein Modul, eine Verantwortung“.

### 4.5 Geradeausfahrt des dynamischen Einspurmodells (Abschnitt 15.1): hoch

`DynamicSingleTrack` erzeugt bei Lenkwinkel null und Schräglaufwinkel null Achsseitenkräfte von etwa 800 N (vorn) und 1600 N (hinten). Das Fahrzeug giert und driftet bei Geradeausfahrt (nach 2 s: ψ ≈ −0,03 rad). Ursache sind die Offset-Koeffizienten (PHY, PVY) der MF6.2-Beispielreifen. Beim Zusammenfassen beider Räder einer Achse heben sie sich nicht wie links/rechts gespiegelt auf. Die `TyreModel`-Implementierungen und das 7-DOF-Modell sind nicht betroffen (geprüft). Der Befund ist als `xfail(strict=True)` in `test_single_track_models.py` festgehalten.

## 5. Zuordnung der Befunde zur Roadmap (Plan.md, Abschnitt 19)

| Meilenstein | Offene Punkte aus diesem Audit | Abnahme |
|---|---|---|
| **M0 – Repository Audit & Stabilization** | ~~Stand sichern~~ (`041f48c`). ~~C/GMRES-Bezeichnung korrigieren~~ (`0c5877a`). ~~Parameterherkunft korrigieren~~ (`1e45e33`). ~~Toten Doku-Verweis beheben~~ (`1e45e33`). ~~Regressionstests für die Bestandsmodelle~~ (`f8d164a`, `test_regression_baseline.py`). ~~Abdeckung der portierten Alt-Tests prüfen~~ (alle alten Testfälle übernommen, keine abgeschwächt). ~~`pytest`-Marker `slow`~~ (Schnelllauf 13 s). | `grep -ri "c/gmres"` findet nur noch Hinweise auf „nicht implementiert“; `pytest -m "not slow"` < 60 s; volle Suite grün |
| **M1 – Virtual Reference Vehicle & Handling Fundamentals** | Geradeausfahrt-Defekt von `DynamicSingleTrack` beheben (4.5). Referenzfahrzeug mit dokumentierter Herkunft. Lineares 2-DOF-Einspurmodell. Paket `handling_analysis` (Eigenwerte, Dämpfung, Eigenfrequenz, Gierverstärkung, Untersteuergradient, v_char/v_crit, Frequenzgang), aufbauend auf `simulation/handling_limits.py` statt parallel dazu. Hinterachs- bzw. 4WS-Geometrie im kinematischen Einspurmodell (Grenzfall δ_f = δ_r). | Analytische und numerische Gierverstärkung < 1 % Abweichung bei kleinem a_y; Prüfkriterien V1 |
| **M2 – Nonlinear Four-Wheel Vehicle Dynamics** | Kontaktverlust explizit behandeln: Ereignisflag in der Telemetrie, momentenkonsistente Lastverteilung statt Clip-und-Renormierung in `SevenDOFModel.compute_normal_loads`. | Invariantentest für Kräfte- und Momentenbilanz |
| **M3 – Advanced Tyre Physics** | Transientes Reifenmodell mit Relaxationslänge fehlt. | gemäß Plan |
| **M5 – Actuators, ABS, TCS, ESC & Torque Vectoring** | Eigenständige Control Allocation mit Machbarkeitsrückmeldung (Paket `allocation/`) fehlt. | gemäß Plan |
| **M6 – Advanced Vehicle Motion Control** | LQR und Gain-Scheduled LQR fehlen. | gemäß Plan |
| **M9 – Virtual Engineering & Research Laboratory** | Große Szenario-Module aufteilen (`split_mu_scenarios.py`, `four_wheel_steering_scenarios.py`, `validation_matrix.py`). | gemäß Plan |
