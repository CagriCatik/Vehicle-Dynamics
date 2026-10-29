# Drift-Regler: Ergebnisse und Abnahme (Drift-Plan)

Ergebnisse der Pakete D0–D5 mit Gate-Bewertung (PASS / FAIL / INFEASIBLE). Verfahren:
[DRIFT_CONTROLLER.md](DRIFT_CONTROLLER.md); Modellgrenzen: [DRIFT_MODEL_VALIDITY.md](DRIFT_MODEL_VALIDITY.md). Alle
Parameter synthetisch, nur virtuelle Modellvalidierung.

## Gate-Übersicht

| Gate | Bewertung | Nachweis |
|---|---|---|
| D0 Audit, Baseline | PASS | Testbasis 607 bestanden, 2 übersprungen bei `9a1dbf4`; Wiederverwendungsstellen im Code geprüft; Referenzbeispiel läuft unverändert ([DRIFT_MODEL_VALIDITY.md](DRIFT_MODEL_VALIDITY.md) 1–2) |
| D1 Reifen, deterministische Auswertung | PASS | Audit aller drei Reifen bestanden (mit Extrapolationswarnung jenseits 20°); Rohmodell reihenfolgeabhängig um 1,6e-7, `DeterministicPlant` bitgleich; Nichtkonvergenz gemeldet bzw. im strengen Modus Ausnahme (Tests) |
| D2 Driftgleichgewichte | PASS (mit Extrapolation) | nichttriviale, physikalisch geprüfte Gleichgewichte in beiden Richtungen; innerhalb der Reifengültigkeit nur bis etwa 15° Schwimmwinkel (`valid`), darüber `valid_extrapolated`; 6 von 140 Punkten ohne Gleichgewicht, 2 als Kurvenfahrt statt Drift erkannt |
| D3 Halten mit LQR | PASS | Gleichgewicht offen instabil (Eigenwert +3,4 1/s), stabilisierbar (PBH 7/7); LQR hält in beiden Richtungen und regelt alle 14 Störfälle aus, ohne Driftverlust; offen geht jede Störung verloren |
| D4 Einleitung, Austritt, Recovery | PASS mit Grenze | End-to-End aus Geradeausfahrt ohne Zustandseingriff: 30 von 36 Läufen erfolgreich (83 %), µ 0,6 18/18, µ 0,9 12/18; nicht erreicht: 25° bei µ 0,9 (Einleitung überschwingt, korrekter Abbruch bei 13 und 15 m/s, Dreher in der Recovery bei 11 m/s → FAILED gemeldet). Überwachter Austritt ohne Überschwingen gegenüber 21–47° bei sofortiger Abschaltung |
| D5 Kreisführung, Übergang, Acht | Kreis PASS, Übergang PASS, Acht: Ablauf PASS / Geometrie FAIL | Radiale Außenschleife beseitigt ±2 m Startversatz (0,00 / 0,06 m), fester LQR behält 1,4 m; gestufte Einleitung auf 25° bei µ 0,9 erreicht (die D4-Grenze), mit 8° Überschwingen; Acht: beide Driftkreise erreicht und symmetrisch, aber Mittelpunktabstand 1,34 × (R_l + R_r) statt 1 (Kreise berühren sich nicht) |

## D2: Driftgleichgewichte

`configs/drift/equilibrium_sweep.yaml`: v = 8/10/13/16/20 m/s × |β| = 5…35° × µ = 0,9/0,6 × links/rechts = 140 Punkte
(8 min). Ergebnisse: `output/drift/equilibrium_sweep/` (CSV, JSON, PNG, Bericht).

| Klasse | Punkte |
|---|---|
| `valid` | 50 |
| `valid_extrapolated` | 82 |
| `infeasible`: kein Gleichgewicht (`residual_too_large`) | 6 (|β| = 35° bei 8 m/s für µ 0,9 und 0,6, bei 10 m/s für µ 0,9; je Richtung) |
| `infeasible`: Hinterachse unter dem Seitenkraftmaximum (Kurvenfahrt) | 2 (20 m/s, 5°, µ 0,9) |

Auswahl (Linksdrift; R Radius, δ Vorderachslenkwinkel, T Moment je Hinterrad; * Reifen extrapoliert):

| µ | v [m/s] | 5° | 10° | 20° | 30° |
|---|---|---|---|---|---|
| 0,9 | 8 | R 9 m, δ +7°, T 258 | R 9, δ +1°, 281* | R 10, δ −10°, 374* | R 12, δ −21°, 463* |
| 0,9 | 13 | R 20, δ +5°, 263 | R 22, δ −3°, 268 | R 26, δ −14°, 333* | R 29, δ −25°, 431* |
| 0,9 | 20 | Kurvenfahrt | R 50, δ −4°, 275 | R 60, δ −16°, 333* | R 68, δ −27°, 427* |
| 0,6 | 13 | R 31, δ +5°, 191 | R 33, δ −6°, 168 | R 40, δ −17°, 205* | R 46, δ −27°, 249* |

Befunde:

- Rechtsdrift ist das exakte Spiegelbild der Linksdrift (Test).
- Das offene Differential bestimmt den Driftzustand: Das entlastete kurveninnere Hinterrad dreht mit großem Schlupf
  (κ ≈ 0,9 bei 20°), das äußere bleibt bei κ ≈ 0,1.
- Gegenlenken (δ < 0 bei Linksdrift) tritt erst ab etwa 10° Schwimmwinkel auf; bei 5° wird in die Kurve gelenkt. Das
  Vorzeichen ist Ergebnis, nicht Vorgabe.
- Der Radius wächst mit dem Schwimmwinkel und fällt mit dem Reibwert; das nötige Moment steigt mit dem Schwimmwinkel.
- Ab etwa 20° Schwimmwinkel liegen die Hinterachs-Schräglaufwinkel jenseits der erklärten Gültigkeit des MF-Reifens
  (20°); diese Ergebnisse beruhen auf dem extrapolierten Reifenmodell.

## D3: Halten und Störungen (S01–S03)

`configs/drift/drift_lqr.yaml`: 13 m/s, |β| = 20°, µ = 0,9 (R 25,6 m, r 29,1°/s, δ_f −14,4°, T 333 N·m;
`valid_extrapolated`), 6 s, RMS ab 2 s. Ergebnisse: `output/drift/lqr_hold/`.

Linearisierung: instabile Eigenwerte +3,40 und +0,04 1/s (Driftmode aus `v_y`/`r`, Geschwindigkeitsmode),
PBH-Rang 7/7, Steuerbarkeitsrang 3 von 7 (schnelle Radmoden nicht erreichbar, aber stabil), Schritt 1e-6
(konvergiert). Geschlossener Kreis (linear): langsamster Eigenwert −0,46 1/s (Geschwindigkeit).

| Störung (Linksdrift) | offen: Driftverlust | LQR: β-RMSE [°] | LQR: r-RMSE [°/s] | LQR: Fehlernorm Anfang → Ende | LQR: Sättigung |
|---|---|---|---|---|---|
| keine (Halten) | nein (6 s) | 0,00 | 0,00 | 0 → 0 | 0 % |
| β +3° | ja | 0,41 | 0,19 | 1,05 → 0,14 | 1 % |
| β −3° | ja | 0,41 | 0,20 | 1,05 → 0,15 | 1 % |
| r +0,1 rad/s | ja | 0,06 | 0,06 | 1,00 → 0,02 | 1 % |
| r −0,1 rad/s | ja | 0,05 | 0,04 | 1,00 → 0,02 | 0 % |
| v −1 m/s | ja | 2,41 | 1,25 | 1,00 → 0,37 | 0 % |
| v +1 m/s | ja | 2,03 | 1,14 | 1,00 → 0,46 | 0 % |
| inneres Hinterrad −10 rad/s | ja | 0,14 | 0,09 | 0 → 0,02 | 0 % |

Rechtsdrift: identische Werte (gespiegelt). Die vorläufigen Engineering-Ziele des Plans (β-RMSE ≤ 3°, r-RMSE ≤ 5°/s
nach Einregelphase) sind in allen Fällen eingehalten. Geschwindigkeitsstörungen klingen am langsamsten ab (Zeitkonstante
etwa 2 s, die langsame Geschwindigkeitsmode).

Offener Kreis: Aus dem exakten Gleichgewicht bleibt er 6 s auf dem Sollzustand (Residuum nahe null), jede der
Störungen führt dagegen zum Dreher bzw. Austritt innerhalb von 1–4 s; das bestätigt die Instabilität des
Gleichgewichts.

**Entwurfsbefund:** Die Grundform des LQR (Lenkwinkel direkt als Eingang) mit nachträglicher Ratenbegrenzung hielt keine
Störung zuverlässig; die Ursache war die Lenkratengrenze (gefordert bis 4200°/s). Mit der Lenkrate als Eingang im
Entwurf ist die Begrenzung kaum noch aktiv ([DRIFT_CONTROLLER.md](DRIFT_CONTROLLER.md) 3).

## D4: Einleitung, Halten, Austritt (S04, S05)

`configs/drift/initiation.yaml`: Start in Geradeausfahrt bei Driftgeschwindigkeit, Driftanforderung bei 0,5 s,
Austrittsanforderung bei 25 s, 45 s je Lauf; v = 11/13/15 m/s × β* = 15/20/25° × µ = 0,9/0,6 × links/rechts = 36 Läufe
(parallel, etwa 16 min). Ergebnisse: `output/drift/initiation/` (Bericht, `kpi.csv`, `mode_log.json` mit allen
Moduswechseln und Gründen, Zeitverläufe, Ablaufbild).

**Erfolg (HOLD erreicht, kein FAILED, Normalfahrt wiederhergestellt): 30 von 36 (83 %).** Links und rechts identisch.

| µ | β* | 11 m/s | 13 m/s | 15 m/s |
|---|---|---|---|---|
| 0,9 | 15° | PASS (HOLD nach 2,2 s) | PASS (2,2 s) | PASS (2,1 s) |
| 0,9 | 20° | PASS (15,0 s) | PASS (7,6 s) | PASS (2,0 s) |
| 0,9 | 25° | FAIL: Zeitüberschreitung, RECOVER, Dreher → FAILED | FAIL: Zeitüberschreitung, Recovery zur Normalfahrt | FAIL: wie 13 m/s |
| 0,6 | 15/20/25° | PASS (1,9–2,4 s) | PASS | PASS |

Über die erfolgreichen Läufe: Zeit bis HOLD Median 2,2 s (max. 15,0 s), β-RMSE im HOLD ≤ 2,0°, Austritt bis zur
Normalfahrt ≤ 3,5 s, Schwimmwinkelspitze im Austritt höchstens 0,14° über dem Driftwinkel, minimale Geschwindigkeit
im Austritt ≥ 84 % der Driftgeschwindigkeit. Vergleich mit sofortiger Abschaltung (Moment 0, Lenkrampe auf 0) aus
demselben HOLD: kein Dreher in den 30 Fällen, aber Schwimmwinkelspitzen von 21–47°.

**Grenze 25° bei µ 0,9:** Der LQR, aus der Kurvenfahrt gestartet, überschwingt auf etwa −36°, das Moment sättigt und
die Geschwindigkeit steigt bis 25 m/s; der Einfangbereich wird in 15 s nicht erreicht. Der Automat bricht korrekt ab
(INITIATE → RECOVER); bei 11 m/s dreht sich das Fahrzeug in der Recovery, was als FAILED gemeldet wird. Ein gestufter
Weg (zuerst 15°, dann Sollpunktwechsel entlang des Zweigs) gehört zu D5.

**Entwurfsbefunde** (Verfahren und Zahlen in [DRIFT_CONTROLLER.md](DRIFT_CONTROLLER.md) 5): kein stationärer Weg aus
der Geradeausfahrt in den Drift (getrennte Gleichgewichtszweige); Einleitung ohne Momentenstoß, der LQR bricht das
Heck selbst aus; Austritt mit Gegenlenken in Fahrtrichtung (`k_β = 1`), teilweises Gegenlenken (0,5; 0,8) dreht sich;
rampierte Sollgeschwindigkeit nötig (Sollsprung erzeugte einen Dreher bei 3 m/s); Gegenlenken nur des Driftanteils
(bei 11 m/s hat normale Kurvenfahrt β = +4°; vorher blieb der Austritt dort hängen, 26/36 → 30/36).

## D5: Kreisführung, Sollpunktwechsel, Acht (S06, S07)

`configs/drift/circle_hold.yaml`: Gleichgewicht 13 m/s, β −20°, R 25,6 m, µ 0,9; Schedule über 6…26°; Kreis mit
Gleichgewichtsradius, radial um 0 / +2 / −2 m versetzt (außen / innen); 25 s. Ergebnisse: `output/drift/circle_tracking/`.

| Startversatz | offen (u*) | ESC | Drift-LQR fest | Drift-LQR + Außenschleife |
|---|---|---|---|---|
| 0 m | Drift verloren (Radialfehler 19 m) | Drift beendet, 13,4 m | 0,00 m | 0,00 m |
| +2 m | verloren, 20,0 m | beendet, 14,9 m | 1,39 m | **0,00 m** |
| −2 m | verloren, 18,1 m | beendet, 12,0 m | 1,41 m | **0,06 m** |

(Radialfehler RMS der letzten 10 s.) Befunde:

- Der offene Kreis fällt auch ohne Störung nach 8,6 s vom instabilen Gleichgewicht ab (in D3 über 6 s noch nicht sichtbar).
- **ESC und Drift widersprechen sich:** Das ESC regelt die Gierrate auf die Referenz aus dem Fahrerlenkwinkel; der ist im
  Drift ein Gegenlenkwinkel nach rechts. Es beendet den Drift (β-RMSE 15,6°), und das Fahrzeug fährt danach einen
  Rechtskreis. Der Drift braucht deshalb die exklusive Drift-Regelung (Plan 5.2).
- Der feste LQR hält den Drift, aber auf einem um den Startversatz verschobenen Kreis (Radialfehler pendelt ±1,4 m);
  erst die Außenschleife führt auf den Sollkreis. Sie bewegt dabei β um höchstens etwa ±3°.

**Gestufte Einleitung (Sollpunktwechsel):** aus der Geradeausfahrt auf 15° eingeleitet (HOLD nach 2,7 s), dann
TRANSITION entlang des Zweigs auf 25° mit 1°/s (bis 12,7 s), Halten bis 50 s, Austritt, Normalfahrt bei 52,8 s: PASS.
Damit ist der 25°-Drift bei µ 0,9 erreichbar, den die direkte Einleitung in D4 nicht schafft. Schwäche: Nach Erreichen
des Sollpunkts schwingt β auf 33° über (8° über dem Ziel), die Geschwindigkeit steigt bis 16 m/s; β-RMSE im
anschließenden HOLD 3,2° (langsame Geschwindigkeitsmode).

**Acht:** Linksdrift 12 s, Austritt (3,1 s, Spitze 20°, min. 12,5 m/s), Rechtsdrift aus der Normalfahrt eingeleitet,
12 s, Austritt.

| Abschnitt | erreicht | Kreisausgleich R | radiale Streuung um den Ausgleichskreis | β Mittel / Std. |
|---|---|---|---|---|
| links | ja | 26,3 m | 0,52 m | −21,4° / 1,3° |
| rechts | ja | 26,3 m | 0,52 m | +21,4° / 1,3° |

Ablauf: PASS (beide Abschnitte, kein FAILED, exakt gespiegelt). **Geometrie: FAIL** — Mittelpunktabstand 70,2 m statt
52,6 m (Verhältnis 1,34). Ursache: Zwischen den Kreisen verlässt das Fahrzeug den Drift und leitet neu ein; die
Außenschleife kann nur den Radius, nicht den Mittelpunkt verschieben. Eine echte Acht braucht eine Mittelpunkt- bzw.
Querablagenführung oder einen direkten Wechsel links → rechts ohne Austritt (offen).

## Offene Punkte nach D5

- Einfangbereich systematisch bestimmen (größere Störungen, Rasterung wie in M11), statt nur der definierten Störungen.
- Kein Integralanteil: Bei Parameterabweichungen bleibt eine stationäre Abweichung zu erwarten (D7).
- 25° bei µ 0,9 direkt nicht einleitbar; gestuft (D5) erreicht, mit 8° Überschwingen nach dem Sollpunktwechsel.
- Der Austritt verliert bis 16 % Geschwindigkeit; die Recovery bei 11 m/s nach misslungener Einleitung dreht sich.
- Acht geometrisch nicht erreicht (Mittelpunktführung fehlt).
- Nächstes Paket: D6, Animation (GIF) und vollständiger Telemetrieexport.
