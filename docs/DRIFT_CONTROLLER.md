# Drift-Regler: Verfahren (Drift-Plan D1–D3)

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

## 5. Reproduktion

```bash
python examples/drift_tyre_audit.py                      # D1
python examples/drift_equilibria_demo.py                 # D2, configs/drift/equilibrium_sweep.yaml (~8 min)
python examples/drift_hold_demo.py                       # D3, configs/drift/drift_lqr.yaml (~3 min)
python -m pytest tests/vehicle_model/drift -q
```

Konfigurationen werden streng geprüft (unbekannte Schlüssel, Wertebereiche, Dateien, Reifennamen, Gewichtsdimensionen)
und aufgelöst in den Ergebnisordner geschrieben (`run_config_resolved.yaml`).

## 6. Grenzen (Stand D3)

- Nur Hinterradantrieb mit offenem Differential und Frontlenkung; Torque Vectoring und Allrad folgen nicht vor D7.
- Informationsmodus nur `ideal_full_state`; geschätzte und sensorbeschränkte Rückführung folgen (D7).
- Lokale Aussage: Die Linearisierung gilt nahe dem Gleichgewicht; Einfangbereich und globales Verhalten sind nicht
  bewiesen, nur für die simulierten Störungen gezeigt.
- Driftwinkel ab etwa 15–20° liegen an der Hinterachse außerhalb der erklärten Reifengültigkeit (`valid_extrapolated`).
