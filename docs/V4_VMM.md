# V4.0 M24: Vehicle Motion Supervisor, Integration und Benchmark-Matrix P0–P7

Masterplan V4.0 (`Plan_v2.md`), M24. Synthetische Parameter, virtuelle Modelle; Manöver „-inspired“ (ISO 7975,
FMVSS 126, ISO 7401, ISO 3888-2, NHTSA-Fishhook), keine Normprüfungen und keine Serienreifeaussage.

## 1. M24.1 Funktionsregister und deterministische Arbitrierung

- `controllers/motion_supervisor.py`: Register `FUNCTIONS` (16 Funktionen) mit Domäne, Prioritätsklasse
  (`safety > driver > stability > agility > comfort`), Modell (7-DOF/14-DOF) und der **einen** Stelle, die ihr
  finales Kommando bildet. Radmomente entstehen nur in der ActuationLayer (Allokation + Schlupfregelung), der
  Vorderradwinkel nur im Lenk-Overlay, aktive Federbeinkräfte nur in der vertikalen Allokation.
- `arbitrate_longitudinal` (`controllers/motion_control.py`): Fahrerpedal als Basis; aktive Bremsböden (HDC) →
  **stärkste** Bremsung (`min`), nie Summe; additive Agilitätsanfragen (GVC) nur ohne aktiven Boden, sonst
  unterdrückt und protokolliert. Für die bisherigen Einzelfälle (nur GVC, nur HDC) bitgleich zum alten Code
  (M20/M22-Tests unverändert grün). `arbitrate_yaw` (M17) bleibt die Gier-Arbitrierung (ESC/TV vs. CBC).
- `VehicleMotionSupervisor`: Hülle, die **nur beobachtet** (Test: Kommandos bitgleich mit und ohne): Aktivität je
  Funktion je Takt, Übergänge, Konflikte (unterdrückte Anfragen), Konditionierung der Allokation.
- Ein unbeschränkter Summationsoperator existiert nicht; eine begründete Sollgeschwindigkeitsreduktion bei
  unerreichbaren Vektorwünschen ist **nicht** umgesetzt (nur das Residuum wird berichtet).

## 2. M24.2 Mehrdomänen-Allokation

Die Domänen bleiben getrennt, jede mit eigener Effektivitätsmatrix und harten Grenzen: Längs/Gier
(`WheelForceAllocator`, 2 × 4), Lenkung (Overlay, finalisiert einmal), vertikal (`VerticalForceAllocator`, 3 × 4,
14-DOF). Keine äußere Koordinationsiteration (kein Nachweis, dass die Wechselwirkung sie erfordert). Diagnose des
Supervisors: Median der Konditionszahl der Längs/Gier-Matrix über die **freien** Räder und der Zeitanteil, in dem
die freien Räder `F_x` und `M_z` nicht unabhängig stellen können (Konditionszahl > 10³ oder < 2 freie Räder).
Bei Vollbremsung auf Split-µ liegt dieser Anteil bei ~97 %: Alle Räder sind an der Haftgrenze, die Allokation hat
keine freie Autorität mehr.

## 3. M24.3 Benchmark-Matrix (`configs/experiments/v4_m24_vmm.yaml`, Typ `vmm_benchmark`)

| Stufe | Inhalt (dieser Plattform) |
|---|---|
| P0 | passives Fahrzeug, feste 60:40-Bremse, 2WS, kein ABS |
| P1 | ABS + TCS + ESC-artiger Gierraten-PI (Totband 3 °/s) über die gemeinsame Allokation |
| P2 | P1 + TV + 4WS (GS-LQR) |
| P3 | P2 + EBD + CBC + IBC + regenerative Schlupfregelung (mit Aktuatordynamik) + HDC |
| P4 | P3 + eLSD + Motorhüllkurve + ITC (Toleranzband nur in Traktionsphasen) |
| P5 | P4 + AFS mit variabler Übersetzung + GVC; RSC im 14-DOF-Fishhook. **Abweichung:** Die LQR-Eingangssätze kombinieren AFS und Hinterachslenkung nicht; P5 nutzt AFS + TV (ohne 4WS) |
| P6 | P5 + Vorschaudämpfung + aktive Nickregelung (nur 14-DOF-Fahrbahnmanöver; im 7-DOF identisch mit P5) |
| P7 | P6 + Health-Monitor/degradierte Modi + Supervisor-Protokoll |

15 Manöver: Bremsen trocken / µ 0,3, Split-µ-Bremsen, Bremsen in der Kurve, Sine with dwell, Lenkwinkelsprung,
Spurwechsel, Fishhook (14-DOF), µ-Sprung, Beschleunigen aus der Kurve (µ 0,6), Gefälle 20 % (µ 0,6), unebene
Fahrbahn ISO C (14-DOF), Bremsen auf ISO-B-Fahrbahn (14-DOF), Ausfall zweier Bremsen auf Split-µ, Sensorverzögerung
100 ms + Reibwertschätzfehler +60 %.

**Methodik**: 4 gepaarte Ziehungen je Manöver (Ziehung 0 nominal; Geschwindigkeit ±5 %, µ ±10 %, Masse und
Gierträgheit ±10 %, Schwerpunkthöhe ±10 %), für alle Stufen identisch; Regler auf **Nominalparametern** ausgelegt,
die Strecke erhält die gezogenen Werte. Statistik je Manöver/Stufe/Kennzahl: Median, Quartile, schlechtester Fall,
Mittelwert mit Bootstrap-95-%-Intervall (bei 4 Ziehungen breit, so ausgewiesen), gepaarte Mediandifferenz zu P0.
Stufen, die auf einem Modell konstruktionsbedingt identisch sind, laufen einmal und sind als *shared* markiert.
Ergebnisse: `output/v4/m24/vmm/` (`kpis.csv`, `stats.csv`, `results.json`, `report.md`, `matrix.png`).

## 4. Ergebnisse (Median über 4 gepaarte Ziehungen, [Quartile], schlechtester Fall)

| Manöver | Kennzahl | P0 | P1 | P2 | P3 | P4 | P5 | P6 | P7 |
|---|---|---|---|---|---|---|---|---|---|
| 01_braking_dry | stopping_distance | 58 | 41 | 41 | 43 | 43 | 43 | 43 | 43 |
| 01_braking_dry | max_lateral_offset_m | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| 02_braking_low_mu | stopping_distance | 128 | 82,1 | 82,1 | 85,8 | 85,8 | 85,8 | 85,8 | 85,8 |
| 02_braking_low_mu | max_abs_slip | 1,01 | 0,0334 | 0,0334 | 0,0206 | 0,0206 | 0,0206 | 0,0206 | 0,0206 |
| 03_split_mu_braking | max_lateral_offset_m | 0,701 | 0,164 | 0,00469 | 0,042 | 0,042 | 0,014 | 0,014 | 0,014 |
| 03_split_mu_braking | max_sideslip_deg | 89,7 | 0,0139 | 0,0039 | 0,0327 | 0,0327 | 0,0323 | 0,0323 | 0,0323 |
| 03_split_mu_braking | stopping_distance | 79,7 | 147 | 147 | 157 | 157 | 149 | 149 | 149 |
| 03_split_mu_braking | v_end_kph | 3,58 | 3,58 | 3,58 | 3,58 | 3,58 | 3,59 | 3,59 | 3,59 |
| 04_brake_in_turn | max_sideslip_deg | 20 | 1,96 | 1,29 | 1,43 (w 89,7) | 1,29 (w 89,6) | 1,24 | 1,24 | 1,24 |
| 04_brake_in_turn | yaw_error_rms_degps | 1,73 | 2,79 | 0,981 | 2,37 (w 50,3) | 1,94 (w 48,5) | 2,1 | 2,1 | 2,1 |
| 05_sine_with_dwell | max_sideslip_deg | 69,8 | 2,98 | 1,77 | 6,87 | 6,87 | 1,48 | 1,48 | 1,48 |
| 05_sine_with_dwell | yaw_error_rms_degps | 36,6 | 3,7 | 2,22 | 7,91 | 7,91 | 4,2 | 4,2 | 4,2 |
| 06_step_steer | yaw_error_rms_degps | 1,94 | 1,74 | 0,802 | 1,57 | 1,5 | 2,2 | 2,2 | 2,2 |
| 06_step_steer | max_sideslip_deg | 0,948 | 0,9 | 0,295 | 0,774 | 0,705 | 0,726 | 0,726 | 0,726 |
| 07_lane_change | yaw_error_rms_degps | 2,75 | 1,93 | 0,485 (w 2,17) | 1,49 (w 55,6) | 1,17 | 0,734 | 0,734 | 0,734 |
| 07_lane_change | max_sideslip_deg | 1,49 | 1,41 | 0,716 (w 1,39) | 1,16 (w 89,7) | 1,01 | 1,15 | 1,15 | 1,15 |
| 08_fishhook | ltr_max | 1 | 1 | 1 | 1 | 1 | 0,768 | 0,768 | 0,768 |
| 08_fishhook | two_wheel_lift_time_s | 0,13 (w 0,395) | 0,13 (w 0,395) | 0,13 (w 0,395) | 0,13 (w 0,395) | 0,13 (w 0,395) | 0 | 0 | 0 |
| 08_fishhook | lift_time_s | 0,195 (w 0,58) | 0,195 (w 0,58) | 0,195 (w 0,58) | 0,195 (w 0,58) | 0,195 (w 0,58) | 0 (w 2,42) | 0 (w 2,42) | 0 (w 2,42) |
| 09_mu_step_braking | stopping_distance | 113 | 64,4 | 64,4 | 68,5 | 68,5 | 68,5 | 68,5 | 68,5 |
| 09_mu_step_braking | max_abs_slip | 0,996 | 0,103 | 0,103 | 0,248 | 0,248 | 0,248 | 0,248 | 0,248 |
| 10_corner_exit | max_sideslip_deg | 89,6 | 0,691 | 0,49 | 0,608 | 0,596 | 1,69 | 1,69 | 1,69 |
| 10_corner_exit | yaw_error_rms_degps | 18,3 | 1,71 | 2,57 | 2,91 | 2,66 | 1,79 | 1,79 | 1,79 |
| 11_hill_descent | v_max_kph | 72,2 | 72,2 | 72,2 | 12,8 | 12,8 | 12,8 | 12,8 | 12,8 |
| 11_hill_descent | v_end_kph | 72,2 | 72,2 | 72,2 | 10 | 10 | 10 | 10 | 10 |
| 12_uneven_road | body_accel_rms | 0,7 | 0,7 | 0,7 | 0,7 | 0,7 | 0,7 | 0,551 | 0,551 |
| 12_uneven_road | tyre_load_variation | 0,196 | 0,196 | 0,196 | 0,196 | 0,196 | 0,196 | 0,217 | 0,217 |
| 13_braking_turn_rough_road | pitch_rms_deg | 0,658 | 0,658 | 0,658 | 0,658 | 0,658 | 0,658 | 0,524 | 0,524 |
| 13_braking_turn_rough_road | body_accel_rms | 0,407 | 0,407 | 0,407 | 0,407 | 0,407 | 0,407 | 0,313 | 0,313 |
| 14_actuator_fault | max_lateral_offset_m | 6,96 | 5,07 | 0,198 | 0,383 | 0,383 | 0,516 (w 0,961) | 0,516 (w 0,961) | 0,00249 |
| 14_actuator_fault | max_sideslip_deg | 89,7 | 0,0369 | 0,0297 | 0,0815 | 0,0815 | 0,208 (w 0,357) | 0,208 (w 0,357) | 0,0537 |
| 14_actuator_fault | stopping_distance | 65 | 231 | 236 | 249 | 249 | 274 | 274 | 223 |
| 14_actuator_fault | v_end_kph | 3,56 | 24,2 | 25,7 | 30,7 | 30,7 | 20,4 (w 35,8) | 20,4 (w 35,8) | 20,1 |
| 15_sensor_delay_mu_error | max_sideslip_deg | 69,8 | 8,12 | 89,7 | 89,7 | 89,7 | 1,28 | 1,28 | 1,28 |
| 15_sensor_delay_mu_error | yaw_error_rms_degps | 36,6 | 9,93 | 63,1 | 64,4 | 64,4 | 10,1 | 10,1 | 10,1 |

(w …) = schlechteste der 4 Ziehungen, wenn deutlich vom Median verschieden. Einheiten: Strecken m, Winkel °, Gierfehler °/s, Geschwindigkeit km/h, Zeiten s, Beschleunigung m/s².

## 5. Befunde und Zielkonflikte

- **Grundfunktionen wirken wie erwartet**: ABS (P1) verkürzt den Bremsweg trocken 58 → 41 m und auf µ 0,3
  128 → 82 m (P0 blockiert, Schlupf 1,0); HDC (ab P3) hält 10 km/h am 20-%-Gefälle (ohne: 72 km/h); RSC (ab P5)
  senkt im Fishhook die LTR 1,0 → 0,77 und verhindert das Abheben beider Räder einer Seite in allen Ziehungen;
  Vorschaudämpfung + Nickregelung (P6) senken die Aufbaubeschleunigung auf ISO C um 21 % und das Bremsnicken um 20 %
  bei höherer Radlastschwankung (0,196 → 0,217: Komfort gegen Fahrsicherheit).
- **Stufen sind nicht monoton besser**: Die Aktuatordynamik ab P3 verschlechtert die 4WS-LQR-Regelung deutlich
  (Sine with dwell β 1,8° → 6,9°; Lenkwinkelsprung Gierfehler 0,80 → 1,57 °/s; in einer Ziehung Schleudern beim
  Bremsen in der Kurve und im Spurwechsel). Der LQR ist ohne Aktuatordynamik ausgelegt; die Hinterachslenkung (35 ms,
  35 °/s) schwingt (GIF `levels_sine_with_dwell.gif`). P5 (AFS statt 4WS) stellt die Stabilität wieder her (β 1,5°).
  Eine Auslegung mit Aktuatormodell ist offen.
- **Brems- und Bremszeitkosten der Stabilität**: Auf Split-µ halten alle geregelten Stufen die Spur (P0 dreht sich,
  β 90°), mit Bremswegen nahe der select-low-Grenze (147–157 m; ohne Lenkkorrektur des Fahrers). EBD/IBC/Regen mit
  Aktuatordynamik (P3) verlängern den Bremsweg um 2–4 m (Druckaufbau).
- **Bremsausfall**: Nur P7 (Health-Monitor) hält in allen Ziehungen die Spur (0,0025 m gegenüber 0,08–0,96 m bei P5
  ohne Monitor), steht aber wie P1–P5 am Laufende noch nicht (20 km/h): Stabilität vor Bremsweg.
- **Robustheit gegen Signalfehler**: 100 ms Eingangsverzögerung **oder** +60 % Reibwertüberschätzung allein lassen den
  4WS-LQR (P2–P4) schleudern; PI (P1) und AFS+TV-LQR (P5) bleiben stabil. Der Monitor (P7) erkennt die Reibwertdrift
  nicht und hält die Gierregelung bei 100 ms Verzögerung bewusst aktiv (M23); er ändert hier nichts.
- **Beschleunigen aus der Kurve (µ 0,6)**: P2/P3 bremsen unter Vollgas stark (Endgeschwindigkeit 28 km/h statt 52–83),
  P4 (eLSD, ITC in Traktionsphasen) behält die Geschwindigkeit bei gleichem Schwimmwinkel; P5 lenkt mit variabler
  Übersetzung stärker ein (β 1,7°, nicht gleichartig vergleichbar).

## 6. Plausibilitätsprüfung und Korrekturen

Durchsicht der ersten Läufe (Rauchtest und erste Matrix), alle Befunde behoben oder offen ausgewiesen:

1. **Leere Funktionsblöcke verschwanden**: `hdc: {}` / `gvc: {}` wurden als „kein Block“ gespeichert; HDC wirkte in
   P3+ nicht (alle Stufen 71 km/h am Gefälle). Korrektur: ein angegebener Block bedeutet „aktiv mit Standardwerten“;
   Regressionstest. Ältere Konfigurationen nutzten keine leeren Blöcke (M20-Ergebnisse unberührt).
2. **ITC-Toleranzband immer aktiv**: Das Band aus M18 schaltet das Giermoment an der Bandgrenze ein und aus, in
   **jedem** Manöver. Mit Aktuatordynamik führte das im Sine with dwell zum Schleudern (β 89°) und auf Split-µ zu
   einem Bremsweg am theoretischen Limit (53 m), weil die Gierregelung während der Bremsung ruhte. Korrektur:
   `itc_scope: traction` (Band nur bei Antriebsanforderung), Standard `always` lässt M18 bitgleich.
3. **Bremsweg allein irreführend**: P0 „bremst“ mit zwei ausgefallenen Bremsen kürzer (65 m) als ohne Ausfall, weil
   es sich dreht (β 89,7°); P7 hält die Spur, steht am Laufende aber noch nicht. Primärkennzahlen daher mit
   Schwimmwinkel und Endgeschwindigkeit.
4. **Konditionszahl-Maximum sinnlos** (bis 10⁹, wenn nur Räder einer Seite frei sind): ersetzt durch Median und
   Zeitanteil schlechter Konditionierung.
5. **Radabhebezeit im Fishhook**: RSC zeigte in einer Ziehung 2,42 s Abheben gegenüber 0,58 s passiv. Ursache: ein
   einzelnes kurveninneres Rad hebt bei LTR 0,88 ab (Dreirad-Zustand unter RSC-Bremsung), passiv heben beide Räder
   einer Seite ab (0,40 s, Vorstufe des Überschlags). Primärkennzahl daher die Zweirad-Abhebezeit.
6. **Gierfehler mit variabler Übersetzung**: Die Referenz der Kennzahl folgt dem Fahrerlenkwinkel mit fester
   Übersetzung; P5 lenkt bei 80 km/h 5,4 % weniger (gewollt), was im Lenkwinkelsprung etwa 0,8 °/s konstanten
   „Fehler“ erzeugt. Kennzahl unverändert (sonst ändern sich die M19-Ergebnisse), hier ausgewiesen.
7. **ABS im Modus A kaum aktiv**: Die Allokation kennt µ und begrenzt jedes Rad bereits an der Haftgrenze; der
   Supervisor protokolliert diesen Zustand daher eigens (`grip_limit`).

## 7. Abnahme M24

| Kriterium | Bewertung |
|---|---|
| P0–P7 mit Feature-Flags lauffähig, reproduzierbar | erfüllt (YAML, gepaarte Ziehungen mit Seed, Provenienz in `results.json`) |
| fachlich zulässige 7-DOF/14-DOF-Matrix | erfüllt: 14-DOF-Manöver variieren nur dort vorhandene Funktionen; identische Stufen markiert |
| Export CSV, JSON, PNG, Markdown, GIF | erfüllt (`output/v4/m24/`) |
| Methodik und Grenzen explizit | erfüllt (Abschnitte 3, 6, 8) |
| Supervisor als Koordinator, ein Besitzer je Aktuator | erfüllt (Register, Arbitrierung, beobachtende Hülle; Test) |
| Regressionen der bestehenden Demos | erfüllt (volle Testsuite) |
| Konfidenzintervalle methodisch belastbar | **eingeschränkt**: 4 Ziehungen; Intervalle breit und als solche ausgewiesen |

Tests: `tests/vehicle_model/test_v4_m24_vmm.py` (7 Tests, davon einer `slow`).

## 8. Grenzen

- Offene Fahrerführung (keine Lenkkorrektur): Split-µ-Bremswege der geregelten Stufen liegen nahe der
  select-low-Grenze, weil nur die Bremsen den Kurs halten.
- P5 ersetzt 4WS durch AFS (Eingangssätze); Pfad-Kennzahlen mit variabler Übersetzung sind nicht gleichartig.
- Die 14-DOF-Manöver laufen nicht über die 7-DOF-Bewegungsregelung; Längs-/Querfunktionen wirken dort nicht.
- Keine Laufzeit-p95-Auswertung im Bericht (Rechenzeiten sind Wanduhrzeit und nicht reproduzierbar).
