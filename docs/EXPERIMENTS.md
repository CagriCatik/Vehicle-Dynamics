# Virtuelles Versuchslabor: YAML-Experimente, Verifikation, Berichte und CI (M9)

Paket: [`src/vehicle_model/experiments/`](../src/vehicle_model/experiments/). Beispiele: [`configs/experiments/`](../configs/experiments/).

**M9-Abnahme:** Ein Anwender wählt über YAML Fahrzeug, Reifenmodell, Fahrmanöver, Regler und Fahrbahnmodell, führt die Simulation reproduzierbar aus und erhält ein vollständiges Vergleichs- und Verifikationsergebnis.

---

## 1. Nutzung

```bash
python -m vehicle_model.experiments list                       # vorhandene Experimente
python -m vehicle_model.experiments validate configs/experiments/*.yaml
python -m vehicle_model.experiments run configs/experiments/sine_with_dwell_comparison.yaml
python -m vehicle_model.experiments run <spec.yaml> --out <Verzeichnis> [--no-verification]
```

Rückgabecode: 0 bei Erfolg, 1 bei ungültiger Spezifikation (`validate`), 2 bei fehlgeschlagener Modellverifikation (`run`).

## 2. Spezifikation

```yaml
name: sine_with_dwell_comparison
vehicle: configs/vehicles/reference_passenger_car.yaml
tyres: {front: magic_formula_passenger_synthetic, rear: magic_formula_passenger_synthetic}   # optional
kc: configs/suspension/kc_reference_passenger_car.yaml                                       # optional
road: {type: uniform, mu: 1.0}               # oder {type: split_mu, mu_left: 1.0, mu_right: 0.2}
manoeuvre: {type: sine_with_dwell, speed_kph: 80, amplitude_deg: 8}
controllers:
  - {law: passive}
  - {law: pi}                                 # Giermoment (TV)
  - {law: gs_lqr, inputs: 4ws_tv}             # Hinterachslenkung + Giermoment
  - {law: ltv_mpc, inputs: tv, estimated_state: true}   # Rückführung über Sensoren und Beobachter
sweep: {parameter: manoeuvre.speed_kph, values: [60, 80, 100]}   # optional
dt: 0.005
seed: 0
verification: true
outputs: {directory: output/experiments/x, formats: [csv, json, png, md]}   # gif optional
```

| Abschnitt | Werte |
|---|---|
| `road.type` | `uniform` (`mu`), `split_mu` (`mu_left`, `mu_right`) |
| `manoeuvre.type` | `step_steer` (`speed_kph`, `steer_deg`, `duration_s`), `sine_with_dwell` (`speed_kph`, `amplitude_deg`), `split_mu_braking` (`speed_kph`) |
| `controllers[].law` | `passive`, `pi`, `lqr`, `gs_lqr`, `ltv_mpc` |
| `controllers[].inputs` | `tv`, `4ws_tv` (nicht für `pi`) |
| `controllers[].estimated_state` | Rückführung über virtuelle Sensoren und den gemischten Schwimmwinkelbeobachter (M7) |
| `sweep.parameter` | `manoeuvre.<Schlüssel>` oder `road.<Schlüssel>`; jeder Punkt wird einzeln validiert |

Die Validierung ist streng: Unbekannte Schlüssel, unbekannte Werte, fehlende Dateien und Werte außerhalb der Grenzen führen zu einem `ValueError` mit dem betroffenen Pfad. Ein Tippfehler fällt so nie still auf einen Standardwert zurück.

## 3. Ablauf und Ergebnisse

1. **Modellverifikation** am konfigurierten Fahrzeug, mit seinen Reifen und K&C ([`verification.py`](../src/vehicle_model/experiments/verification.py)):
   - Parameterkonsistenz,
   - statische Radlasten,
   - Reifensymmetrie (keine Seitenkraft bei α = 0),
   - stationäre Gierrate gegen das lineare Eigenlenkbudget (≤ 1 %),
   - Spiegelsymmetrie,
   - Determinismus.
2. **Vergleich:** je Sweep-Punkt jeder Regler auf einer frischen Strecke, über den Benchmark aus M6.
3. **Ausgaben:**

| Datei | Inhalt |
|---|---|
| `results.json` | Spezifikation, Verifikation, Kennzahlen je Punkt und Regler, Herkunft (Git-Commit, Python-, numpy- und scipy-Version, Zeitstempel) |
| `kpis.csv` | eine Zeile je Punkt und Regler |
| `histories/*.csv` | Zeitverläufe: t, X, Y, ψ, $v_x$, r, $r_\text{ref}$, β, Lenkwinkel, Giermoment angefordert und erzeugt |
| `<punkt>.png` | Gierrate, Schwimmwinkel, Bahn je Regler |
| `<punkt>.gif` | Draufsicht-Animation aller Regler (Format `gif`, optional) |
| `report.md` | Engineering-Bericht: Aufbau, Verifikationstabelle mit PASS/FAIL, Kennzahltabellen, Informationszugriff und Grenzen |

**Reproduzierbarkeit:** Gleiche Spezifikation und gleicher Code ergeben identische Verläufe und Kennzahlen; ausgenommen sind die gemessenen Rechenzeiten. Der Bericht nennt den Git-Commit.

## 4. Beispiele

| Spezifikation | Inhalt |
|---|---|
| `sine_with_dwell_comparison.yaml` | passiv, PI, GS-LQR (4WS+TV), LTV-MPC mit Beobachter; Fahrzeug mit K&C |
| `split_mu_braking.yaml` | Vollbremsung auf Split-µ, passiv gegen PI und LQR |
| `step_steer_speed_sweep.yaml` | Lenkwinkelsprung bei 50/80/110 km/h, TV gegen 4WS+TV |

Ergebnis des ersten Beispiels: Verifikation **PASS** (6/6, lineare Referenz 0,05 %). Gierratenverhältnis nach 1,0 s: passiv 0,98 (dreht sich ein), PI 0,006, GS-LQR (4WS+TV) 0,002, LTV-MPC mit Beobachter 0,027.

## 5. Continuous Integration

[`.github/workflows/ci.yml`](../.github/workflows/ci.yml):

| Auslöser | Umfang |
|---|---|
| Push auf `main`, Pull Request | Schnelllauf `pytest -m "not slow"` auf Python 3.10 und 3.13; Validierung aller Experimentspezifikationen |
| wöchentlich, manuell | volle Testsuite inklusive `slow`; ein Beispielexperiment mit Modellverifikation |

Installation in der CI: `pip install -e ".[test]"`; pytest ist das optionale Extra `test`. Lokal geprüft wurde nur die Syntax unter Python 3.11 (`compileall`); ein Lauf unter 3.10 erfolgt erst in der CI.

## 6. Grenzen und offene Punkte

- Manöver und Regler sind auf die Gierstabilisierung beschränkt (Lenkwinkelsprung, Sine with Dwell, Split-µ-Bremsung). Die Bestandsszenarien in `simulation/` (4WS, Split-µ-Varianten, Validierungsmatrix) und die Pfadfolge-NMPCs sind nicht über YAML erreichbar.
- Die großen Szenariomodule sind seit V3.0 aufgeteilt in die Pakete `simulation/split_mu/`, `simulation/four_wheel_steering/` und `simulation/validation/` (Teilmodule nach Verantwortung: Ergebnisse, Szenariogruppen, Sweep, Grafiken, Animation; Basisklasse plus Szenario-Mixins), größtes Teilmodul 836 Zeilen; die alten Module sind Kompatibilitätsfassaden. Offen: Trennung von Manöver, Fahrer, Ausführung und Kennzahlen innerhalb der Szenariofunktionen.
- Monte-Carlo-Studien (M7) laufen über Python (`simulation/robustness.py`), noch nicht über die YAML-Spezifikation.
- Die GIF-Ausgabe zeigt eine schematische Draufsicht (Rechteck je Fahrzeug), nicht die detaillierte Fahrzeuggrafik von `visualization/plan_view.py`.
