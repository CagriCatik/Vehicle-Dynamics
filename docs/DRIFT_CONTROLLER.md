# Drift-Regler: Verfahren (Drift-Plan D1–D4)

Autonomer Drift auf der bestehenden 7-DOF-Strecke nach `Plan_v3.md`. Dieses Dokument beschreibt die Verfahren;
Ergebnisse stehen in [DRIFT_RESULTS.md](DRIFT_RESULTS.md), Modellgrenzen und Konventionen in
[DRIFT_MODEL_VALIDITY.md](DRIFT_MODEL_VALIDITY.md). Alle Parameter sind synthetisch; es gibt keine Aussage über ein
reales Fahrzeug.

| Teil | Modul | Paket |
|---|---|---|
| Reifenaudit | `drift/tyre_audit.py` | D1 |
| Deterministische Streckenauswertung | `drift/plant_adapter.py` | D1 |
| Domänentypen | `drift/types.py` | D2 |
| Driftgleichgewichte | `drift/equilibrium.py`, `simulation/drift/equilibrium_sweep.py` | D2 |
| Linearisierung | `drift/linearization.py` | D3 |
| LQR | `drift/lqr.py` | D3 |
| Regler-Adapter | `drift/control_adapter.py` | D3 |
| Kennzahlen | `drift/metrics.py` | D3 |
| Halten und Störung | `simulation/drift/hold.py` | D3 |
| Zustandsautomat | `drift/supervisor.py` | D4 |
| Gain-Schedule entlang eines Gleichgewichtszweigs | `drift/schedule.py` | D4 (für D5) |
| Überwachter Regler (Einleitung, Halten, Austritt) | `drift/control_adapter.py` (`DriftSupervisedController`) | D4 |
| Einleitung, Austritt, Erfolgsquoten | `simulation/drift/scenarios.py` | D4 |
| Plots | `visualization/drift_plots.py` | D1–D3 |

Abhängigkeitsrichtung wie im Plan (3.2): `drift/` kennt weder Matplotlib noch Dateien; `simulation/drift/` liest
Konfigurationen, ruft den bestehenden `ScenarioRunner` und schreibt Ergebnisse; `visualization/` bekommt nur Daten.
Kräfte entstehen ausschließlich in `SevenDOFModel.evaluate_dynamics`.

## 1. Stationäre Driftgleichgewichte (D2)

**Konfiguration V1:** Hinterradantrieb mit gleichem Moment an beiden Hinterrädern (offenes Differential,
`T_drive = [0, 0, T, T]`), nur Vorderachslenkung, keine Bremse, Vorderräder frei rollend.

**Problem.** Zustände `q = [v_x, v_y, r, ω_FL, ω_FR, ω_RL, ω_RR]`. Die Sollbedingungen `|v| = v_ref` und
`atan2(v_y, v_x) = β_ref` werden durch Einsetzen erfüllt, `v_x = v cos β`, `v_y = v sin β`. Unbekannt sind
`z = [r, ω × 4, δ_f, T]` (7), Gleichungen die sieben Ableitungen der Strecke (`v̇_x = v̇_y = ṙ = ω̇_i = 0`). Das ist
das 9×9-Problem des Plans (Unbekannte `q, δ_f, T`; sieben Dynamikresiduen plus Geschwindigkeit und Schwimmwinkel) mit
exakt eliminierten kinematischen Bedingungen: quadratisch, keine Überbestimmung. Der Radius `R = v/|r|` ist Ergebnis.

**Löser.** `scipy.optimize.least_squares` (Trust Region Reflective) mit skalierten Residuen (Beschleunigungen / g,
Gierbeschleunigung / (m g L / I_z), Radbeschleunigungen / (m g R / I_w)) und Schranken aus den Fahrzeuggrenzen
(Lenkwinkel, Antriebsmoment, `ω ≥ 0`). Startwerte: festes Raster aus Gierrate (0,5/0,7/0,9 · µ g / v, Vorzeichen der
Drift), Lenkwinkel (−20/−10/0/10°, gespiegelt) und Hinterradschlupf (0,1/0,3/0,6), 300 Auswertungen je Start; findet
kein Start ein zulässiges Gleichgewicht, wird der beste Kandidat mit 3000 Auswertungen verfeinert. Auswertung über
`DeterministicPlant` (reihenfolgeunabhängig, Toleranz der Lastverlagerung 1e-12). Sweeps über Geschwindigkeit ×
Schwimmwinkel × Reibwert setzen benachbarte Punkte warm fort (je Richtung und Reibwert).

**Annahme (`valid`)** erfordert alle Bedingungen:

| Bedingung | Begründung |
|---|---|
| Residuen je Gruppe innerhalb `RESIDUAL_TOL` (wie M11) | Gleichgewicht |
| Lastverlagerung konvergiert, vier Räder in Kontakt | Kontaktbedingung |
| Lenkwinkel und Moment nicht auf der Grenze | sonst reicht der Aktuator nicht (`actuator_limit`) |
| `|r| ≥ 0,05 rad/s`, `|β| ≥ 2°` | keine triviale Geradeausfahrt |
| `r · β < 0` | Gierrate nach innen, Geschwindigkeit nach außen (Driftlage) |
| beide Hinterräder jenseits des Schräglaufwinkels ihres Seitenkraftmaximums bei ihrer Last | Hinterachse gesättigt: Drift, nicht schnelle Kurvenfahrt |
| Raddrehzahlen ≥ 0 | |

`valid_extrapolated`: wie `valid`, aber ein Reifen arbeitet außerhalb seiner erklärten Gültigkeit. Alles andere ist
`infeasible` mit Gründen; es wird kein erzwungener Betriebspunkt gemeldet.

**Befund beim Entwurf:** Die Bedingung `r · β < 0` allein ließ schnelle normale Kurvenfahrt durch (bei 20 m/s und
β = −5° liegt der Hinterachs-Schräglauf mit 6,6° unter dem Maximum bei 6,7°). Deshalb die Sättigungsbedingung.

## 2. Linearisierung (D3)

`Δq̇ = A Δq + B Δu` mit `u = [δ_f, T]`, zentrale Differenzen der deterministischen Auswertung mit Schrittweitenstudie
(1e-2 … 1e-7 relativ; verwendet wird der Schritt nach der kleinsten Änderung von Eigenwerten und `B`). Geprüft und
berichtet: endliche Matrizen, Eigenwerte, instabile Moden, PBH-Test `rang [A − λI, B] = 7` für jede instabile Mode
(Stabilisierbarkeit), Rang der Steuerbarkeitsmatrix (informativ) und Konditionszahlen in skalierten Koordinaten. Ein
unzulässiges Gleichgewicht wird nicht linearisiert.

## 3. LQR (D3)

**Grundform:** `u = u* − K (q − q*)` mit `K = R⁻¹ Bᵀ P` aus der Riccati-Gleichung, entworfen in skalierten Koordinaten
(Zustände durch typische Größen, Eingänge durch die Stellgrenzen geteilt), Gewichte dimensionslos.

**Lenkrate im Entwurf (Standard):** Der Lenkwinkel wird Zustand, seine Rate Eingang:

    x_a = [Δq, Δδ_f],  u_a = [δ̇_f, ΔT],   A_a = [[A, b_δ], [0, 0]],  B_a = [[0, b_T], [1, 0]],

die Lenkrate wird gegen die Ratengrenze des Fahrzeugs (50°/s) skaliert gewichtet. Der Regler integriert die
kommandierte Rate (begrenzt) zum Lenkwinkel.

**Befund beim Entwurf:** Die Grundform verlangt nach einer Gierratenstörung von 0,1 rad/s bis 4200°/s Lenkrate am Rad
(95. Perzentil etwa 750°/s); nachträglich auf 50°/s begrenzt, war die Grenze fast ständig aktiv, und Störungen von 3°
Schwimmwinkel beendeten den Drift (Dreher oder Austritt). Mit der Lenkrate im Entwurf ist die Begrenzung in höchstens
2 % der Schritte aktiv, und alle Störungen klingen ab.

**Verweigerung:** Ist eine instabile Mode nicht erreichbar (PBH-Rang < 7), wirft der Entwurf `NotStabilisable`;
Gewichtsvektoren falscher Länge werden abgewiesen.

**Adapter (`DriftLQRController`):** einzige Verbindung zum `VehicleController`-Port. Informationsmodus
`ideal_full_state` (wahrer Zustand). Stellgrößen: Vorderachslenkwinkel innerhalb ±35° und der Ratengrenze,
Hinterachsmoment in `[0, 1000 N·m]` gleich an beiden Hinterrädern, keine Bremse, Hinterachslenkung 0. Die begrenzte
Stellgröße geht an die Strecke; Sättigung steht in der Diagnose (`DriftControlDiagnostics`).

## 4. Halten und Störungen (D3, Szenarien S01–S03)

`simulation/drift/hold.py`: je Richtung Gleichgewicht, Linearisierung, Entwurf; dann Simulation der 7-DOF-Strecke über
den bestehenden `ScenarioRunner` aus dem Gleichgewicht (`hold`) und aus definierten Störungen (Schwimmwinkel ±3°,
Gierrate ±0,1 rad/s, Geschwindigkeit ±1 m/s, kurveninneres Hinterrad −10 rad/s; für Rechtsdrift gespiegelt), jeweils
mit konstantem `u*` (offen) und mit LQR. Nichts wird teleportiert: Der Runner integriert die Strecke vom Anfangszustand.

Kennzahlen (`drift/metrics.py`, Plan 7.1): RMS von `β − β_ref`, `r − r_ref`, `v − v_ref` im Haltefenster
(`t ≥ settle_s`), maximaler Schwimmwinkelfehler, Fehlernorm am Anfang und Ende
(`√((Δβ/0,05 rad)² + (Δr/0,1 rad/s)² + (Δv/1 m/s)²)`), Sättigungsanteil, Driftverlust (`|β − β_ref| ≥ 30°`).

## 5. Einleitung, Austritt und Recovery (D4)

**Ausgangslage, gemessen (13 m/s, µ 0,9).** Die Driftgleichgewichte bilden über β einen stetigen, überall offen
instabilen und stabilisierbaren Zweig von β = −1° bis −25° (R 20–27 m, δ_f von +18,6° bis −19,9°). Geradeausfahrt
liegt aber auf einem anderen Zweig: der stabilen Kurvenfahrt über dem Lenkwinkel (β zwischen 0 und +1°, κ_RL ≤ 0,05).
Bei gleichem Lenkwinkel (18,6°) existieren beide nebeneinander (Kurvenfahrt β +0,8°, κ_RL 0,05; Driftzweig β −1°,
κ_RL 0,35). Es gibt also keinen quasistationären Weg aus der Geradeausfahrt in den Drift: Die Einleitung muss ein
Übergangsvorgang sein.

**Zustandsautomat** (`drift/supervisor.py`, nur diskrete Entscheidungen, jeder Wechsel mit Grund protokolliert):
DISABLED → APPROACH (Driftanforderung, v ≥ 5 m/s) → INITIATE (stationäre Kurvenfahrt erreicht: |r − r_c| < 2°/s und
|β̇| < 2°/s für 0,5 s) → HOLD (Einfangbereich |β − β*| < 5°, |r − r*| < 8°/s, |v − v*| < 2,5 m/s für 0,5 s) → EXIT
(Austrittsanforderung) → DISABLED (Normalfahrt wiederhergestellt). Abbruch: INITIATE → RECOVER bei Zeitüberschreitung
(15 s) oder Divergenz, HOLD → RECOVER bei |β − β*| > 15°, RECOVER → FAILED nach 15 s; jeder Modus → FAILED bei
|β| ≥ 60°. HOLD wird nie außerhalb des Einfangbereichs gemeldet; FAILED ist endgültig. Schwellen in
`configs/drift/initiation.yaml`.

**Regelgesetze je Modus** (`DriftSupervisedController`):

| Modus | Gesetz |
|---|---|
| DISABLED | Lenkwinkel mit Ratengrenze auf 0, Moment P-geregelt auf eine mit ≤ 1,5 m/s² **rampierte** Sollgeschwindigkeit |
| APPROACH | Lenkwinkel auf δ_c der stabilen Kurvenfahrt bei 0,6 µ g (`approach_equilibrium`, Bisektion über δ), Moment T_c + P |
| INITIATE, HOLD | Drift-LQR (D3), stoßfrei übernommen (Lenkzustand = letzter Befehl). **Kein Momentenstoß**: Der LQR bricht aus der Kurvenfahrt heraus selbst das Heck aus und fängt das Fahrzeug |
| EXIT, RECOVER | `release`: Moment T_c, Lenkziel `δ_c + k_β β_d` (β_d Schwimmwinkel nur in Driftlage, β·r < 0), dann `straight`: Lenkrampe auf 0 mit Geschwindigkeitsregelung |
| FAILED | Moment 0, Lenkrampe auf 0 (definierte, begrenzte Stellgröße, keine Rettung) |

**Herleitung des Austrittsgesetzes.** Mit `δ = δ_c + β` wird der vordere Schräglaufwinkel
`α_v ≈ δ − β − l_v r / v = δ_c − l_v r / v` unabhängig von β: Die Vorderachse behält ihre Kraft, während die Hinterachse
wieder haftet (Lenken in Fahrtrichtung, wie ein Fahrer, der ins Rutschen lenkt). Gemessen aus den Gleichgewichten
−15/−20/−25° (13 m/s, µ 0,9), Spitzen-Schwimmwinkel / minimale Geschwindigkeit:

| k_β | −20° | −25° |
|---|---|---|
| 0 (nur δ_c) | 40° / 2,8 m/s | 42° / 2,7 m/s |
| 0,5 | Dreher | Dreher |
| 0,8 | Dreher | Dreher (mit T_c) |
| 1,0 | **20° / 12,2 m/s** | **25° / 12,2 m/s** |
| 1,2 | 21° / 12,6 m/s | 26° / 12,9 m/s |

Teilweises Gegenlenken ist schlechter als keines; k_β = 1 ist Standard (`exit_countersteer_gain`).

**Befunde beim Entwurf** (alle verworfen, dokumentiert):

- Momentenstoß aus der Kurvenfahrt mit Übergabe an den LQR erst im Einfangbereich: Bei β = −19° war die Gierrate
  89°/s statt 29°/s; das Gegenlenken (+6° → −14° bei 50°/s mindestens 0,4 s) kam zu spät, alle Varianten drehten sich.
- Austritt mit einem LQR um das Kurvenfahrtgleichgewicht: dreht sich (lineare Auslegung fern ihres Arbeitspunkts).
- Austritt entlang des Driftzweigs mit Gain-Scheduling (`drift/schedule.py`): erreicht das Zweigende bei ≤ 1°/s,
  scheitert aber an der Übergabe zur Kurvenfahrt.
- Geschwindigkeitsregelung mit Sollwertsprung: 1000 N·m bei 3 m/s nach einem Austritt → Leistungsübersteuern → Dreher.
- Gegenlenken des gesamten Schwimmwinkels im Austritt: Bei 11 m/s hat normale Kurvenfahrt β = +4°; Gegenlenken lenkt
  dann weiter ein und das Austrittskriterium |β| < 3° wird nie erreicht. Deshalb nur der Driftanteil (β·r < 0).

## 6. Reproduktion

```bash
python examples/drift_tyre_audit.py                      # D1
python examples/drift_equilibria_demo.py                 # D2, configs/drift/equilibrium_sweep.yaml (~8 min)
python examples/drift_hold_demo.py                       # D3, configs/drift/drift_lqr.yaml (~3 min)
python examples/drift_initiation_demo.py                 # D4, configs/drift/initiation.yaml (36 Läufe, ~16 min parallel)
python -m pytest tests/vehicle_model/drift -q
```

Konfigurationen werden streng geprüft (unbekannte Schlüssel, Wertebereiche, Dateien, Reifennamen, Gewichtsdimensionen)
und aufgelöst in den Ergebnisordner geschrieben (`run_config_resolved.yaml`).

## 7. Grenzen (Stand D4)

- Nur Hinterradantrieb mit offenem Differential und Frontlenkung; Torque Vectoring und Allrad folgen nicht vor D7.
- Informationsmodus nur `ideal_full_state`; geschätzte und sensorbeschränkte Rückführung folgen (D7).
- Lokale Aussage: Die Linearisierung gilt nahe dem Gleichgewicht; Einfangbereich und globales Verhalten sind nicht
  bewiesen, nur für die simulierten Störungen und die 36 Einleitungen des Sweeps gezeigt.
- Einleitung nur über den Einfangbereich des LQR aus stationärer Kurvenfahrt; bei 25° und µ 0,9 reicht er nicht.
- Driftwinkel ab etwa 15–20° liegen an der Hinterachse außerhalb der erklärten Reifengültigkeit (`valid_extrapolated`).
