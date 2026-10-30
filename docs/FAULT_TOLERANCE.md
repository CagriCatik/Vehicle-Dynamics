# Fehlertolerante Bewegungsregelung und degradierte Modi

Grundlage: [Plan integrierte Bewegungsregelung](plans/integrated_motion_control_plan.md). 7-DOF-Modell, gemeinsamer MotionController-Pfad. Synthetische Parameter,
deterministische Fehlerfolgen, virtuelle Modelle; **kein** Nachweis von Fail-operational-Eigenschaften über die
benannten Bedingungen hinaus.

## 1. Architektur (zwei getrennte Seiten)

| Seite | Datei | Aufgabe |
|---|---|---|
| Strecke/Sensorik | `simulation/fault_injection.py` | `FaultEvent(t_start, duration, channel, mode, severity, wheel)` aus YAML (strikt geprüft); verfälscht die Reglereingänge (Sensor-/Kommunikationsfehler) und die Streckeneingänge (Aktuatorfehler); liefert **Aktuator-Selbstmeldungen** nach einer deklarierten Erkennungszeit (50 ms) |
| Regler | `controllers/fault_tolerance.py` | `FaultTolerantController`: Health-Monitor und Rekonfiguration um den MotionController; sieht **nur** die verfälschte, zeitgestempelte Eingangsnachricht und die Selbstmeldungen, nie die Fehlerliste |
| Allokation | `allocation/wheel_forces.py` | `ActuatorHealth` kompatibel erweitert um `drive_scale`/`brake_scale` (Standard 1,0: alle Grenzen unverändert) |
| Experiment | `experiments/fault_tolerance.py` | Typ `fault_tolerance_benchmark`: Manöver × Fehlerfolgen × Regler (gleicher Regler mit/ohne Monitor) |

Fehlerkatalog (`channel: modes`): `brake: failed | limited | stuck`, `motor: failed | limited`, `steer_front: stuck`,
`steer_rear: stuck`, `imu: lost`, `wheel_speed: invalid | frozen` (Raddrehzahl und daraus abgeleiteter Schlupf in
Zustand und Telemetrie), `mu_estimate: drift`, `load_sensor: bias`, `delay: ticks`, `stale: frozen`.
`damper`/`preview` sind 14-DOF-Kanäle und werden hier mit einem Modellgrenzen-Fehler abgelehnt.
`limited` sättigt bei `severity` × Maximalmoment.

## 2. Monitor und Reaktionen

Zustände je Kanal `nominal < limited < degraded < fallback < unavailable`; Systemzustand = schlechtester Kanal,
begrenzt auf `degraded` für ein verlorenes redundantes Element, `fallback` für einen kontrollierten Halt bzw. eine
unbrauchbar alte Nachricht, `unavailable` nur ohne verbleibende Bremse (Ausgangsstatus dann `infeasible`).
Rückkehr zu `nominal` 0,2 s nach Wegfall der Ursache (Hysterese).

| Erkennung | Reaktion |
|---|---|
| Bremse/Motor `failed` (Selbstmeldung) | Kanal `unavailable`, Health-Maske: kein Kommando mehr an den Kanal |
| Bremse/Motor `limited` | Grenze der Allokation auf den gemeldeten Anteil |
| Bremse `stuck` | `degraded`, **bleibt** in der Allokation (gemessen: Herausnehmen lässt sein Moment unberücksichtigt wirken, Querversatz 0,44 m statt 0,009 m) |
| Gierrate nicht endlich (IMU) | Gierregelgesetz → `ZeroYawLaw` (Giermoment 0: links/rechts ausgeglichene Bremskräfte, select-low-artig) |
| Raddrehzahl ungültig oder eingefroren | Ersatzwert (Mittel der gültigen Räder; Schlupf des Nachbarrads derselben Achse), Bremse dieses Rads auf 50 % |
| Radlast weicht > 25 % von m g/4 vom quasistatischen Schätzwert ab | Lasten durch den Schätzwert ersetzt |
| Nachrichtenalter > 30 ms / > 100 ms / > 250 ms | `limited` / `degraded` (Gierregelung **bleibt**) / `fallback` (`ZeroYawLaw`) |
| Vorderlenkung `stuck` | `fallback`: kontrollierter Halt mit 3 m/s² |
| Reibwertdrift | **nicht erkennbar** durch Plausibilisierung; als `undetected` berichtet |

## 3. Ergebnisse (`configs/experiments/fault_tolerance_catalogue.yaml`, 3 Manöver × 16 Fehlerfolgen × 2 Regler, `output/fault_tolerance/`)

Gleicher PI-Gierregler, ohne (`unaware`) und mit Monitor (`aware`).

| Manöver | Fehler | unaware | aware |
|---|---|---|---|
| Split-µ 90 km/h | RR ausgefallen | Bremsweg 211 m, Querversatz 0,79 m, Kurs 0,54°, Kommandoleck 13,9 s | 223 m, 0,014 m, 0,002°, Leck nur 0,05 s (Erkennungszeit) |
| | RR, FL, FR ausgefallen | 303 m, **14,8 m**, 7,1° | 320 m, 0,019 m, 0,002° |
| | IMU verloren | **Abbruch** (NaN erreicht die Allokation) | 151 m, 0,09 m, 0,07° |
| | Raddrehzahl FL ungültig | **NaN** im Lauf | wie fehlerfrei (151 m) |
| | Reibwertdrift +60 % | 138 m, 0,53 m | gleich; **nicht erkannt** (`undetected`) |
| Sine with dwell 80 km/h, 6° | RR, FL ausgefallen | β 5,5°, Kurs −23,9° | β 4,55°, −20,3° |
| | IMU verloren | Abbruch | β **73,7°**, Dreher: identisch mit dem passiven Fahrzeug aus Wankstabilität und aktive Lenkung (ohne Gierratensensor kein ESC) |
| | Verzögerung 150 ms | β 5,7° | gleich (Gierregelung bleibt) |
| Bremsen in der Kurve 80 km/h | Vorderlenkung klemmt | 76,5 m | kontrollierter Halt, 67,2 m |
| | Fehlerfolge (RR 0,5–1,5 s, IMU 2,0–2,5 s) | Abbruch | 83,5 m; Zustände nominal → degraded (0,555 s) → nominal (1,705 s) → degraded (2,005 s) → nominal (2,705 s) |

Befunde:

- **Kein Kommandoleck nach der Selbstmeldung**: In allen Läufen erreicht ein ausgefallener Kanal nur während der
  Erkennungszeit (50 ms je Fehler) ein Kommando; ohne Monitor über die ganze Bremsung (bis 43 s Summe).
- Der Monitor hält den Kurs bei Bremsausfällen (Split-µ: 0,002° statt bis 7°), um den Preis längerer Bremswege
  (+5 bis +17 m), weil die Allokation das Giermoment ausgleicht.
- Ein verlorener Gierratensensor ist **kein beherrschter Fehler**: Der Rückfall verhindert den Absturz und hält die
  Bremsung auf Split-µ gerade, kann aber im Sine with dwell das Schleudern nicht verhindern (wie das passive
  Fahrzeug). Fail-operational wird daher nicht behauptet.
- Reibwertdrift ist mit Plausibilisierung allein nicht erkennbar; sie wird berichtet, nicht verdeckt.
- Unwirksam gemessen (ehrlich aufgeführt): eingefrorene Raddrehzahl RL (der eingefrorene Schlupf liegt nahe am
  ABS-Ziel), Bremse FL auf 15 % auf Split-µ (FL steht auf µ 0,2, benötigt weniger als die Grenze).

## 4. Plausibilitätsprüfung und Korrekturen

Die erste Auswertung enthielt Fehler, die eine kritische Durchsicht fand:

1. **Rückfall bei Verzögerung schädlich**: Der ursprüngliche Rückfall bei 150 ms alter Nachricht (Gierregelung aus)
   ließ das Fahrzeug im Sine with dwell und beim Bremsen in der Kurve schleudern (β 74° bzw. 90°), während der PI
   auf verzögerten Daten stabil blieb (β 5,7°). Korrektur: Gierregelung bleibt bei konstanter Verzögerung, Rückfall
   erst bei > 250 ms (eingefrorene Nachricht).
2. **Klemmende Bremse maskiert**: verschlechterte den Querversatz (0,44 statt 0,009 m). Korrektur: bleibt in der
   Allokation.
3. **`limited` inkonsistent**: Die Injektion skalierte das Kommando (Verstärkungsverlust), der Monitor nahm eine
   Kapazitätsgrenze an; die Reaktion war wirkungslos. Korrektur: `limited` = Sättigung bei Anteil × Maximalmoment.
4. **Raddrehzahlfehler ohne Wirkung**: Die Schlupfregelung liest den Schlupf aus der Telemetrie, der Fehler verfälschte
   nur den Zustand; beide Regler blieben unberührt. Korrektur: Raddrehzahl und Schlupf werden verfälscht; ohne Monitor
   entsteht nun NaN, mit Monitor fehlerfreies Verhalten.
5. **Kennzahl `infeasible_s` irreführend**: zählte den vorhandenen Allokationsstatus (Pedalwunsch über Haftgrenze, auch
   ohne Fehler 11,8 s). Ersetzt durch `unavailable_s` (keine Bremse mehr).

Animationen (`python examples/chassis_control_animations.py fault_tolerance`): `output/fault_tolerance/split_mu_three_brakes_failed.gif`,
`imu_lost_split_mu.gif`, `sequence_recovery.gif`.

## 5. Abnahme Fehlertoleranz

| Kriterium | Bewertung |
|---|---|
| keine unbemerkten Kommandolecks zu defekten Kanälen | erfüllt nach der Erkennungszeit (Test, Benchmark: 0 s danach) |
| Grenzen und degradierte Zustände korrekt | erfüllt (Tests: Maske, Skalen, `unavailable` → `infeasible`) |
| Safety-Recovery-Lauf dokumentiert | erfüllt (Fehlerfolge mit Rückkehr zu `nominal`, Test) |
| ein, zwei, drei Stellpfade gestört; Fehlerreihenfolge/Reset; Fuzzing der YAML | erfüllt (Benchmark, Tests) |
| ohne Monolith | erfüllt: Injektion, Monitor, Allokation getrennt; Monitor ist ein Wrapper |
| stuck rear steering mit RSC, Energieausfall während Rekuperation | **nicht abgedeckt**: RSC läuft im 14-DOF-Modell, der Fehlerpfad im 7-DOF; Rekuperationsausfall nicht modelliert |
| Fail-operational | **nicht behauptet** (IMU-Verlust im Sine with dwell nicht beherrscht) |

Tests: `tests/vehicle_model/test_fault_tolerance.py` (7 Tests, davon einer `slow`).

## 6. Grenzen

- Aktuatorfehler werden über Selbstmeldungen mit fester Erkennungszeit erkannt, nicht über modellbasierte Residuen.
- Der Monitor kennt keine Reibwertplausibilisierung; kleine Lastsensorfehler unter 25 % bleiben unerkannt.
- Die Reaktion auf eine klemmende Hinterachslenkung ist nur die Meldung; die benchmarkten Regler nutzen keine
  Hinterachslenkung.
