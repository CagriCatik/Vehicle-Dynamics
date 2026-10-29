# Fahrdynamikregelung: Reglerbibliothek, Solverdiagnostik und Benchmark (M6)

Module:
- [`controllers/motion_control.py`](../src/vehicle_model/controllers/motion_control.py): einheitliche Regelkette.
- [`controllers/yaw_laws.py`](../src/vehicle_model/controllers/yaw_laws.py): PI, LQR, GS-LQR, LTV-MPC.
- [`controllers/solver_diagnostics.py`](../src/vehicle_model/controllers/solver_diagnostics.py): Solverdiagnostik und Degraded Mode.
- [`simulation/controller_benchmark.py`](../src/vehicle_model/simulation/controller_benchmark.py): Benchmark.

**M6-Abnahme:** Unterschiedliche Regler werden mit derselben virtuellen Strecke, identischen Szenarien und definierten Kennzahlen verglichen. Siehe Abschnitt 4.

---

## 1. Einheitliche Regelkette

Die Kette trennt die Ebenen nach Plan.md, Abschnitt 10:

| Ebene | Umsetzung |
|---|---|
| Fahreranforderung | Lenkwinkel, Pedale (`ControlTarget`) |
| Sollbewegung | `YawReference`: $r_\text{ref} = v\delta/(L + EG_\text{ref}v^2)$, $EG_\text{ref}$ ≈ 0,56 °/g, begrenzt auf $0{,}85\,\hat\mu g/v$; Soll-Schwimmwinkel 0 |
| Regelgesetz | austauschbar (`MotionLaw`): Giermoment und/oder Hinterachslenkwinkel |
| Control Allocation | `WheelForceAllocator` (M5) |
| Aktuatorregelung | ABS/TCS (M5) |
| Fahrzeugphysik | Strecke, außerhalb der Kette |

Alle Gesetze laufen über dieselbe Referenz und dieselbe Stellebene. Das ist die Voraussetzung für einen fairen Vergleich. Jedes Gesetz deklariert seinen Informationszugriff. Alle mitgelieferten Gesetze nutzen Zustand und Telemetrie einschließlich Fahrbahnreibwert, also `full_state_ideal` (Plan.md, Abschnitt 10.2: idealisierter Full-State-Feedback-Modus, ausdrücklich gekennzeichnet). Der M5-Regler `AllocatedStabilityController` ist jetzt diese Kette mit dem PI-Gesetz, mit unverändertem Verhalten.

## 2. Regelgesetze

**Entwurfsmodell**, unabhängig von der Strecke: lineares Einspurmodell aus `handling_analysis` mit Zuständen $[v_y, r]$ und Eingängen $[\delta_f, \delta_r, M_z]$. Es wird um das Integral des Gierratenfehlers erweitert, damit eine Modellabweichung keinen bleibenden Fehler hinterlässt. Im Benchmark ist das Entwurfsmodell das linearisierte Referenzfahrzeug **ohne** K&C, die Strecke hat K&C. Die Abweichung ist gewollt.

**Stellgrößen:** nur Giermoment (TV) oder Hinterachslenkung plus Giermoment (integrierte 4WS/TV).

**Vorsteuerung:** Gleichgewicht $(x_{ss}, u_{ss})$ mit $r = r_\text{ref}$ beim Fahrerlenkwinkel. Bei TV ist $v_y$ frei; bei 4WS+TV ist der Soll-Schwimmwinkel 0.

| Gesetz | Verfahren |
|---|---|
| PI | Gierratenfehler, Anti-Windup ($K_p$ = 15 000 N m s/rad, $K_i$ = 100 000 N m/rad) |
| LQR | $u = u_{ss} - K[x - x_{ss}; z]$, $K$ aus der kontinuierlichen Riccati-Gleichung, fester Entwurfspunkt 20 m/s; Anti-Windup durch Integrationsstopp bei Sättigung |
| GS-LQR | wie LQR; Verstärkungen auf dem Raster 5–40 m/s, linear interpoliert, außerhalb geklemmt |
| LTV-MPC | Modell bei der aktuellen Geschwindigkeit, exakt diskretisiert (ZOH, `expm`), über den Horizont gehalten (N = 15, 20 ms). Quadratische Kosten auf Zustand, Integral, Stellgröße und Stellrate; Box-Grenzen. Das kondensierte Problem ist ein beschränktes lineares Kleinste-Quadrate-Problem, gelöst mit `scipy.optimize.lsq_linear` (BVLS) |

**Abstimmung des Integralgewichts:** Strecke mit K&C, Entwurfsmodell ohne.

| Gesetz | $q_z$ = 500 | $q_z$ = 5000 | $q_z$ = 1,5·10⁴ | $q_z$ = 5·10⁴ |
|---|---|---|---|---|
| GS-LQR (TV) | 95 % nach ~2 s | **0,28 s**, kein Überschwingen | | 0,13 s, 9 % Überschwingen |
| LTV-MPC (TV) | Endwert 81 % nach 3 s | 1,15 s | **0,22 s**, 0,6 % | 0,13 s, 9 % |

Die MPC braucht ein stärkeres Integralgewicht, weil ihre Horizontkosten das prädizierte Integral zusätzlich zurück auf null ziehen.

## 3. Solverdiagnostik und Degraded Mode

Beide SLSQP-NMPCs meldeten vor M6 unabhängig vom Solverergebnis den Status `nominal`. Im Spurwechsel konvergiert das dynamische NMPC in 83 % der Schritte; in den übrigen 17 % übernahm es die nicht konvergierte, aber verbesserte Lösung, ohne das zu melden. Das dynamische NMPC meldete außerdem die Giermoment-*Anforderung* als „allokiert“. Jetzt meldet es das Moment, das seine begrenzte Momentenaufteilung tatsächlich erzeugt.

Jede Lösung wird klassifiziert, für NMPC und LTV-MPC gleich:

| Status | Bedingung | verwendete Stellgröße |
|---|---|---|
| `nominal` | Solver konvergiert | Lösung |
| `suboptimal` | nicht konvergiert, aber besser als der Warmstart | Lösung |
| `fallback` | gescheitert, nicht endlich oder schlechter als der Warmstart | verschobener Vorgängerplan |
| `degraded` | N aufeinanderfolgende nicht-nominale Lösungen (Vorgabe 5) | wie `fallback`/`suboptimal`; als unzuverlässig gekennzeichnet |

Ein Zeitbudget kann gesetzt werden. SciPy-Solver lassen sich nicht unterbrechen; eine Überschreitung wird deshalb gemeldet (`over_budget`), nicht verhindert. Tests injizieren Solverfehler: erst `fallback` mit verschobenem Plan, dann `degraded`.

## 4. Benchmark

`run_benchmark` startet jeden Regler auf einer **frischen** Strecke aus derselben Fabrik. Die Szenarien sind Daten; die Kennzahlen werden zentral gegen eine gemeinsame Gierratenreferenz berechnet. Läufe sind deterministisch und voneinander unabhängig (getestet).

**Strecke:** Referenzfahrzeug, MF-Reifen, K&C-Satz. Szenarien: Lenkwinkelsprung 72 km/h; Sine with Dwell 80 km/h, 8° Radlenkwinkel, nach FMVSS 126 nachgebildet (Simulation, keine Prüfung nach Vorschrift); Split-µ-Vollbremsung 90 km/h (1,0 / 0,2). Ergebnisse: [`examples/controller_benchmark.py`](../examples/controller_benchmark.py), Ausgabe in `output/controller_benchmark/kpis.md`.

**Lenkwinkelsprung** (Gierrate in rad/s, $t_{95}$ in s):

| Regler | RMS-Fehler Gierrate | $t_{95}$ | Überschwingen | max. Schwimmwinkel |
|---|---|---|---|---|
| passiv | 0,083 | nicht erreicht | 0 | 0,32° |
| PI (TV) | 0,027 | 0,18 | 0 % | 0,89° |
| LQR (TV) / GS-LQR (TV) | 0,033 | 0,28 | 0 % | 0,89° |
| GS-LQR (4WS+TV) | **0,017** | **0,07** | 0 % | 0,82° |
| LTV-MPC (TV) | 0,031 | 0,22 | 0,6 % | 0,89° |
| LTV-MPC (4WS+TV) | 0,018 | 0,07 | 6,4 % | 0,83° |

**Sine with Dwell.** Grenzwerte nach FMVSS-Vorbild: Gierratenverhältnis ≤ 0,35 nach 1,0 s und ≤ 0,20 nach 1,75 s, Querversatz ≥ 1,83 m.

| Regler | Verhältnis 1,0 s | Verhältnis 1,75 s | Querversatz | max. Schwimmwinkel |
|---|---|---|---|---|
| passiv | 0,98 | 1,15 | 3,16 m | 36° (dreht sich ein) |
| PI (TV) | 0,006 | 0,001 | 2,96 m | 2,4° |
| GS-LQR (TV) | 0,006 | 0,002 | 2,26 m | 1,2° |
| GS-LQR (4WS+TV) | 0,002 | 0,000 | 2,57 m | 2,8° |
| LTV-MPC (TV) | 0,013 | 0,002 | 2,41 m | 2,1° |
| LTV-MPC (4WS+TV) | 0,004 | 0,000 | 2,68 m | 4,7° |

Alle Regler stoßen an das Momentenlimit von 4000 N m. Rund 20–30 % der Schritte melden `infeasible`, weil die Allokation das angeforderte Moment im Kraftschluss nicht vollständig stellen kann. Das wird gemeldet, nicht verschwiegen.

**Split-µ-Bremsung:** passiv dreht sich ein (88°). Alle Regler bleiben auf 0,01–0,02° und 2–4 cm Versatz, mit 151–152 m Bremsweg. Das entspricht der Select-low-Grenze bei fester Lenkung (M5); die Regler unterscheiden sich hier kaum, weil alle durch dieselbe Allokation begrenzt sind.

### Split-µ-Konfigurationsmatrix (V3.0 M10 §5.5)

[`four_wheel_steering/split_mu_matrix.py`](../src/vehicle_model/simulation/four_wheel_steering/split_mu_matrix.py), 100 km/h, µ 1,0 / 0,2, gleiches Pedal für alle, Lenkung gerade gehalten. Giermomente: Spitze der ersten 0,3 s, aufgeteilt in Längskraft- (Bremse) und Querkraftanteil (Lenkung).

| Konfiguration | Giermoment durch | Pedal 0,3: Gierwinkel / Brems-Mz / Lenk-Mz | Pedal 1,0: Gierwinkel / Brems-Mz / Lenk-Mz |
|---|---|---|---|
| passiv | – | 89,0° / 3746 / 947 N m | 88,0° / 4049 / 165 N m |
| 4WS | Lenkung | 88,8° / 3744 / 1293 N m | 88,2° / 4047 / 258 N m |
| 4WS + ABS | Lenkung | 94,4° / 3720 / 1309 N m | 90,6° / 4047 / 746 N m |
| 4WS + ABS + ESC/TV | Bremsmoment addieren | 106,4° / 3502 / 1603 N m | 100,8° / 4091 / 1830 N m |
| TV (Allokation) | Bremsmoment umverteilen | **0,0°** / 955 / 47 N m | **0,0°** / 1311 / 68 N m |
| IVMC | Bremsmoment addieren | 78,3° / 3502 / 1603 N m | 103,1° / 4091 / 1830 N m |

Auf homogener Fahrbahn bremsen alle Konfigurationen gerade. Die Hinterachslenkung erreicht höchstens ein Drittel des Bremsgiermoments, weil die gebremsten Reifen kaum Seitenkraft übertragen (Kammscher Kreis). ESC, das Bremsmoment nur addiert, greift auf der Niedrig-µ-Seite ein und ändert die Asymmetrie kaum. Nur die Allokation senkt das Bremsmoment auf der Hoch-µ-Seite (Select-low-ähnlich) und bleibt gerade, mit 2,2 m/s² Verzögerung. Die eindrehenden Konfigurationen sind erwartete physikalische Grenzen und werden als solche getestet.

**Robustheit** (Plan.md, Abschnitt 10.4), Sine with Dwell auf Streckenvarianten, Regler unverändert:

| Variante | passiv | ungünstigster Regler (Verhältnis 1,0 s) |
|---|---|---|
| +15 % Masse und Gierträgheit | 0,013, stabil (mehr Untersteuern) | LTV-MPC (TV): 0,023 |
| Schräglaufsteifigkeit hinten −20 % | 0,95, dreht sich ein | PI (TV): 0,005 |

Alle Regler halten die Grenzwerte in allen drei Varianten deutlich ein.

## 5. Befunde und Grenzen

- **LTV-MPC (4WS+TV)** erzeugt im Sine with Dwell nach dem Ende des Lenkeingriffs eine Gierratenspitze von etwa +0,33 rad/s (bei ~2,75 s). Die Gierratenverhältnisse erfassen sie nicht, weil sie erst 1,0 s bzw. 1,75 s nach Lenkende gemessen werden. Eine Kennzahl für den Rückschwinger fehlt noch im Kennzahlensatz.
- Die Regler arbeiten mit idealer Zustandsinformation, einschließlich Fahrbahnreibwert. Sensor- und Schätzerschicht folgen in M7.
- **LTV-MPC:** LTI-Modell über den Horizont; nur Box-Grenzen auf den Stellgrößen, keine Zustandsgrenzen; keine formale Stabilitätsgarantie (kein Endkostenterm aus der Riccati-Lösung).
- **NMPC (Pfadfolge):** Die neue Diagnostik zeigt, dass das dynamische NMPC im anspruchsvollen Spurwechsel des E2E-Tests (90 km/h, µ = 0,75, 15 Iterationen) in 25–48 % der Schritte `degraded` ist. Es erreicht den Zielversatz trotzdem (Y = 3,7–3,9 m bei Soll 3,5 m). Mehr Iterationen oder ein besserer Warmstart sind offen. Das Zeitbudget wird nur gemeldet. Die NMPCs sind nicht im Benchmark, weil sie eine andere Aufgabe lösen (Pfadfolge statt Gierstabilisierung bei Fahrervorgabe).
- Nicht umgesetzt: ein separates Gierraten-PID mit D-Anteil (das PI-Gesetz deckt die Referenz ab).
- **4WS-Split-µ-Szenario:** in V3.0 (M10 §5.5) als Konfigurationsmatrix aufgelöst, siehe Abschnitt „Split-µ-Konfigurationsmatrix“.

## 6. Verifikation

| Test | Prüfung |
|---|---|
| [test_yaw_laws.py](../tests/vehicle_model/test_yaw_laws.py) | Gleichgewicht ist stationärer Punkt; Riccati-Residuum; geschlossener Kreis Hurwitz (alle Geschwindigkeiten); Interpolation und Klemmung der Verstärkungen; LQR und MPC folgen auf ihrem Entwurfsmodell exakt (4WS: Schwimmwinkel 0); Stellgrenzen im ganzen MPC-Plan; Solverausfall → `fallback` → `degraded`; Protokolle; PI-Kette = M5-Regler |
| [test_solver_diagnostics.py](../tests/vehicle_model/test_solver_diagnostics.py) | Klassifikation, Zeitbudget, Erholung; injizierter NMPC-Ausfall nutzt den verschobenen Plan; gemeldetes Giermoment = erzeugtes Moment |
| [test_controller_benchmark.py](../tests/vehicle_model/test_controller_benchmark.py) | Sine-with-Dwell-Signal; Kennzahlen an einer synthetischen Historie; Reproduzierbarkeit und Unabhängigkeit der Läufe; Abnahme: passiv dreht sich ein, PI und GS-LQR halten die Grenzwerte (Marker `slow`) |
