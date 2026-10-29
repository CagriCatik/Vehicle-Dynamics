# Drift-Regler: Ergebnisse und Abnahme (Drift-Plan)

Ergebnisse der Pakete D0–D3 mit Gate-Bewertung (PASS / FAIL / INFEASIBLE). Verfahren:
[DRIFT_CONTROLLER.md](DRIFT_CONTROLLER.md); Modellgrenzen: [DRIFT_MODEL_VALIDITY.md](DRIFT_MODEL_VALIDITY.md). Alle
Parameter synthetisch, nur virtuelle Modellvalidierung.

## Gate-Übersicht

| Gate | Bewertung | Nachweis |
|---|---|---|
| D0 Audit, Baseline | PASS | Testbasis 607 bestanden, 2 übersprungen bei `9a1dbf4`; Wiederverwendungsstellen im Code geprüft; Referenzbeispiel läuft unverändert ([DRIFT_MODEL_VALIDITY.md](DRIFT_MODEL_VALIDITY.md) 1–2) |
| D1 Reifen, deterministische Auswertung | PASS | Audit aller drei Reifen bestanden (mit Extrapolationswarnung jenseits 20°); Rohmodell reihenfolgeabhängig um 1,6e-7, `DeterministicPlant` bitgleich; Nichtkonvergenz gemeldet bzw. im strengen Modus Ausnahme (Tests) |
| D2 Driftgleichgewichte | PASS (mit Extrapolation) | nichttriviale, physikalisch geprüfte Gleichgewichte in beiden Richtungen; innerhalb der Reifengültigkeit nur bis etwa 15° Schwimmwinkel (`valid`), darüber `valid_extrapolated`; 6 von 140 Punkten ohne Gleichgewicht, 2 als Kurvenfahrt statt Drift erkannt |
| D3 Halten mit LQR | PASS | Gleichgewicht offen instabil (Eigenwert +3,4 1/s), stabilisierbar (PBH 7/7); LQR hält in beiden Richtungen und regelt alle 14 Störfälle aus, ohne Driftverlust; offen geht jede Störung verloren |

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

## Offene Punkte nach D3

- Einfangbereich systematisch bestimmen (größere Störungen, Rasterung wie in M11), statt nur der definierten Störungen.
- Kein Integralanteil: Bei Parameterabweichungen bleibt eine stationäre Abweichung zu erwarten (D7).
- Nächstes Paket: D4, Drifteinleitung aus normaler Fahrt, Austritt und Recovery mit Zustandsautomat.
