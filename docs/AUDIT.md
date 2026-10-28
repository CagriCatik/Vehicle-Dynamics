# Repository-Audit – Ist-Zustand (Master Engineering Plan V2.0, Abschnitt 3)

**Datum:** 2026-10-27  
**Basis-Commit:** `4190954` (Branch `main`) plus uncommittete Änderungen im Working Tree  
**Zweck:** Ist-Zustand, Testbaseline, Architekturinventar und inkrementeller Integrationsplan.

---

## 1. Git- und Working-Tree-Zustand

| Befund | Bewertung |
|---|---|
| Im Working Tree liegt eine große uncommittete Umstrukturierung: `python/` → `src/vehicle_model/`, neue Verzeichnisse `configs/`, `docs/`, `examples/`, `tests/vehicle_model/*` (ca. 20 000 Zeilen Python). | **Hohes Risiko:** Diese Arbeit ist nicht versioniert. Ein Datenverlust wäre nicht wiederherstellbar. |
| `matlab/` ist zum Löschen *gestaged* (10 `.m`-Dateien), und es gibt keinen Ersatz im Working Tree. | Widerspricht der Vorgabe, „die ursprüngliche Funktionalität zu erhalten“. Die Dateien sind nur noch über `HEAD` erreichbar. |
| `python/cgmres_nmpc`, `python/vehicle_model` und `python/vehicle_visualization` sind gelöscht und wurden nach `src/vehicle_model/` portiert. | Die Portierung wirkt vollständig (Daten, Reifen-YAMLs und Racetracks liegen unter `src/`). |
| `tests/cgmres_nmpc/test_controller.py` und `tests/vehicle_visualization/test_plan_view.py` sind gelöscht. Nachfolger: `tests/vehicle_model/test_nmpc_controller.py` und `test_plan_view.py`. | Ob die alten Testfälle 1:1 abgedeckt sind, wurde noch nicht geprüft. |
| `Plan.md` (1610 Zeilen, „GEMINI – MASTER ENGINEERING INSTRUCTION“) ist ein Vorläufer dieses Plans und ebenfalls untracked. | Soll neben Plan V2.0 als historisches Dokument bestehen bleiben oder ersetzt werden? Das ist noch zu entscheiden. |

## 2. Testbaseline

```
.venv/bin/python -m pytest -q
140 passed, 2 skipped in 613 s (10 min 13 s)
```

- Alle Tests sind grün.
- Eine Laufzeit von 10 Minuten ist für den Entwicklungszyklus zu lang. Es gibt keine Trennung zwischen schnellen Unit-Tests und langsamen Szenario- und E2E-Tests (keine `pytest`-Marker).

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

`controllers/nmpc.py` beschreibt sich als „NMPC … with C/GMRES formulation“ und `NMPCControllerCGMRES`. Tatsächlich wird das Problem per Direct Multiple Shooting mit **`scipy.optimize.minimize(method="SLSQP")`** gelöst. Eine Kontinuierung oder ein GMRES-Verfahren gibt es nicht.

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

- Der Docstring in `seven_dof.py` verweist auf `docs/vehicle_dynamics/seven_dof_model.md`. Diese Datei existiert nicht.
- `pyproject.toml` deklariert `config/tyres/*.yaml` als Package-Data. Die Reifen-Configs liegen jedoch im Top-Level-Verzeichnis `configs/tyres/` und sind daher in einer installierten (nicht-editable) Variante nicht verfügbar.
- Sehr große Szenario-Module (`split_mu_scenarios.py` 2612, `four_wheel_steering_scenarios.py` 2015, `validation_matrix.py` 1665 Zeilen) widersprechen dem Grundsatz „ein Modul, eine Verantwortung“.

## 5. Inkrementeller Integrationsplan

Jeder Meilenstein endet mit einer grünen Test-Suite. Interne Schnittstellenänderungen erhalten Deprecation-Aliase.

| M | Inhalt | Abnahme |
|---|---|---|
| **M0 – Stand sichern** | Umstrukturierung auf einem Feature-Branch committen. Entscheiden, ob MATLAB unter `legacy/matlab/` erhalten bleibt (empfohlen) oder bewusst entfernt wird. | Working Tree sauber, kein Code nur noch in `HEAD` |
| **M1 – Ehrliche Bezeichnungen** | `NMPCControllerCGMRES` umbenennen in `SLSQPPathTrackingNMPC` (der alte Name bleibt als `DeprecationWarning`-Alias). Docs, Examples und Config-Namen korrigieren. `oem`-Reifendaten umbenennen und mit `data_class: synthetic` kennzeichnen. Fahrzeug-YAML um `id/version/provenance` ergänzen. | `grep -ri "c/gmres"` findet nur noch Hinweise auf „nicht implementiert“ |
| **M2 – Testinfrastruktur** | `pytest`-Marker `slow`/`e2e`. Schneller Default-Lauf (Ziel < 60 s), vollständiger Lauf in CI. Regressionstests für die Bestandsmodelle anlegen, bevor refaktoriert wird. | `pytest -m "not slow"` < 60 s |
| **M3 – Analytische Referenz** | Lineares 2-DOF-Einspurmodell und Modul `handling_analysis` (Eigenwerte, Dämpfung, Eigenfrequenz, Gierverstärkung, Untersteuergradient, v_char/v_crit, Frequenzgang). Konsistenztest gegen das nichtlineare Einspurmodell im linearen Bereich. | Analytische und numerische Gierverstärkung < 1 % Abweichung bei kleinem a_y |
| **M4 – V1 mit 4WS** | Hinterachslenkung im kinematischen Einspurmodell (Momentanpol-Geometrie, Grenzfall δ_f = δ_r). | Tests gemäß Prüfkriterien V1 |
| **M5 – Kontaktverlust** | Explizite Radabhebe-Behandlung mit Ereignisflag in der Telemetrie. Momentenkonsistente Lastverteilung. | Invariantentest für Kräfte- und Momentenbilanz |
| **M6+** | Transientes Reifenmodell, LQR/GS-LQR, eigenständige Control Allocation mit Machbarkeitsrückmeldung, Multi-Rate-Engine, Aufteilung der großen Szenario-Module | gemäß Plan V2.0, Abschnitte 7–15 |
