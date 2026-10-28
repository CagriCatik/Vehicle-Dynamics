# Schlupfregelung, beschränkte Control Allocation und Stabilitätsregelung (M5)

Module:
- [`allocation/`](../src/vehicle_model/allocation/): beschränkte Control Allocation mit Machbarkeitsrückmeldung.
- [`controllers/slip_control.py`](../src/vehicle_model/controllers/slip_control.py): ABS und TCS.
- [`controllers/stability_control.py`](../src/vehicle_model/controllers/stability_control.py): ESC und Torque Vectoring.
- [`actuators/`](../src/vehicle_model/actuators/): ideale Varianten und Lenkübersetzung.

Alle Szenarien nutzen das Referenzfahrzeug mit dem Magic-Formula-Reifensatz (M3). Bei ihm hängt der Schlupf der Kraftspitze wie bei einem physikalischen Reifen vom Reibwert ab.

---

## 1. Control Allocation

### 1.1 Löser

Box-beschränkte gewichtete kleinste Quadrate (Härkegård 2002):

$$\min_u\ \lVert W_v(Bu - v)\rVert^2 + \varepsilon\lVert W_u u\rVert^2 \quad\text{unter}\quad u_{min} \le u \le u_{max}$$

Mit $\varepsilon > 0$ ist das Problem streng konvex. Das Optimum ist dann die Lösung des reduzierten Gleichungsproblems für genau eine Zuordnung jeder Stellgröße zu {frei, untere Grenze, obere Grenze}. Für bis zu sechs Stellgrößen werden alle $3^n$ Zuordnungen durchgerechnet ([`wls.py`](../src/vehicle_model/allocation/wls.py)): exakt, deterministisch, ohne Iterationsgrenzen. Abgleich mit `scipy.optimize.lsq_linear` (BVLS) an 200 Zufallsproblemen: Abweichung ≤ 1e-7. Rechenzeit: etwa 1,4 ms je Aufruf.

### 1.2 Radkräfte

Stellgrößen sind die Längskräfte $F_i$ der vier Räder im Radsystem. Anforderung $v = (F_x, M_z)$:

$$F_x = \sum_i \cos\delta_i\,F_i, \qquad M_z = \sum_i (x_i\sin\delta_i - y_i\cos\delta_i)\,F_i$$

Grenzen je Rad. Alle werden durchgesetzt und gemeldet:

| Grenze | Wert |
|---|---|
| verfügbarer Kraftschluss | $F_\text{avail} = 0{,}95\sqrt{(\mu F_z)^2 - F_y^2}$ (Reibkreis abzüglich der genutzten Seitenkraft) |
| Antrieb | $\min(T_\text{drive,max}/R,\ F_\text{avail})$, wenn das Rad angetrieben und der Antrieb intakt ist, sonst 0 |
| Bremse | $-\min(T_\text{brake,max}/R,\ F_\text{avail})$, wenn die Bremse intakt ist, sonst 0 |

- **Priorität:** Giermoment vor Längskraft (Gewichtsverhältnis 100 nach Normierung). Stabilität geht vor dem Verzögerungs- oder Beschleunigungswunsch des Fahrers.
- **Verteilung:** Unter gleich guten Lösungen wird $\sum F_i^2/F_\text{avail,i}$ minimiert. Eine Kraftanforderung verteilt sich dadurch proportional zum verfügbaren Kraftschluss: alle Räder sind gleich ausgelastet und erreichen ihre Grenze gemeinsam. Daraus ergibt sich auch die Bremskraftverteilung.

Jede Allokation meldet die erreichten Werte, das Residuum, die aktiven Grenzen (davon reibwertbegrenzt) und einen Status:

| Status | Bedeutung |
|---|---|
| `nominal` | Anforderung erfüllt, keine Grenze aktiv |
| `saturated` | Anforderung erfüllt, mindestens eine Grenze aktiv (auch: ausgefallener Aktuator) |
| `infeasible` | Anforderung nicht erreichbar; `residual` nennt die Abweichung je Achse |

Nicht endliche Anforderungen lösen einen `ValueError` aus. Das erfüllt die M5-Abnahme „unzulässige Stellanforderungen werden erkannt“.

## 2. Schlupfregelung (ABS, TCS)

PI-Regler auf den Radschlupf mit Anti-Windup. Der Integrator speichert die Momentenreduktion:

$$e = \kappa - \kappa^*, \qquad I \leftarrow \text{clip}(I + s K_i e\,dt,\ 0,\ T_\text{req}), \qquad T = T_\text{req} - \text{clip}(I + s K_p e,\ 0,\ T_\text{req})$$

Dabei gilt $s = -1$ beim Bremsen und $+1$ beim Antreiben. Der Regler nimmt nur Moment weg. Unterhalb von 1,5 m/s wird das Bremsmoment durchgereicht, damit das Fahrzeug zum Stillstand kommt.

**Zielschlupf:** Der Schlupf der Kraftspitze sinkt mit dem Reibwert (MF-Satz: etwa $0{,}11\mu$). $\kappa^*$ wird deshalb aus einer Kalibriertabelle über einen Reibwertschätzwert $\hat\mu$ je Rad interpoliert. $\hat\mu$ startet niedrig (0,1) und wird nur angehoben, aus der beobachteten Kraftschlussausnutzung: $\hat\mu \leftarrow \max(\hat\mu,\ |F|/(0{,}98\,F_z))$. Die Messungen begründen diese Wahl:

| Startwert | µ = 1,0 | µ = 0,5 | µ = 0,2 |
|---|---|---|---|
| fester Zielschlupf, µ̂ = 0,2 auf trockener Straße | 0,39 | | |
| µ̂ startet bei 0,1, adaptiv | 0,98 | 1,02 | 1,08 |
| µ̂ = 1,0, nicht absenkbar | 0,98 | 0,96 | 0,81 |

Werte sind Bremswirkungsgrade (mittlere Verzögerung 20 → 5 m/s bezogen auf µg). Werte über 1 entstehen, weil die Spitzenreibung des Reifens bei geringer Radlast über µ liegt (Lastempfindlichkeit, M3). Ein zu niedriger Schätzwert bremst also viel zu schwach; die Adaption korrigiert ihn in wenigen Zyklen. Ein zu hoher Schätzwert kostet weniger als blockierende Räder (0,62).

**Vergleich mit dem bisherigen ABS** ([`controllers/abs.py`](../src/vehicle_model/controllers/abs.py), unverändert): Es arbeitet mit festen Schwellen (κ = −0,20) und ohne Gedächtnis. Mit dem MF-Reifen erreicht es 0,90 / 0,73 / 0,66, bei µ = 0,2 blockieren die Räder (Median κ = −0,64). Ohne ABS: 0,63 / 0,60 / 0,62. Das bisherige ABS bleibt für die Bestandsszenarien erhalten; siehe Audit.

**TCS:** Auf µ = 0,2 dreht der Reifen ohne TCS durch (κ > 0,5). Mit TCS hält er den Zielschlupf und beschleunigt besser.

## 3. Stabilitätsregelung und Torque Vectoring

`AllocatedStabilityController` implementiert das Protokoll `VehicleController`. Vorausgesetzte Aktuatorarchitektur: by-wire, Antriebs- und Bremsmoment jedes Rads einzeln stellbar. Die Lenkung bleibt beim Fahrer.

1. **Referenzgierrate** $r_\text{ref} = v\delta/(L + EG_\text{ref}v^2)$ mit $EG_\text{ref}$ = 0,001 rad/(m/s²) ≈ 0,56 °/g, begrenzt auf $0{,}85\,\hat\mu g/v$.
2. **Giermoment:** PI mit Anti-Windup ($K_p$ = 15 000 N m s/rad, $K_i$ = 100 000 N m/rad, $|M_z| \le$ 4000 N m). Mit Totband arbeitet der Regler wie ein ESC, ohne Totband als kontinuierliches TV.
3. **Längskraftwunsch** aus den Pedalen.
4. **Allokation** wie in Abschnitt 1.
5. **Innerer Regelkreis:** ABS und TCS.

Genutzte Information: Gierrate und Geschwindigkeit; aus der Telemetrie Radlasten, Seitenkräfte, Schlüpfe und der Fahrbahnreibwert. Der Reibwert ist also ideal bekannt. Für die reale Schätzung siehe `estimation/`.

| Szenario (`examples/slip_and_allocation_demo.py`) | passiv | allokiert |
|---|---|---|
| Split-µ-Bremsung (links 1,0, rechts 0,2), 90 km/h, Vollbremsung, Lenkung fest | dreht sich ein (88° nach 1,6 s) | Gierwinkel 0,01°, Versatz 4 cm, Bremsweg 151 m, Status `infeasible` |
| Lenkwinkelsprung 72 km/h, Fahrzeug mit K&C (EG 2,2 °/g) | Gierrate 0,189 rad/s | 0,265 rad/s (Referenz 0,262), 95 % nach 0,17 s, ohne Überschwingen |
| Bremsen mit Pedal 0,5, Bremse vorn links ausgefallen | dreht sich ein (−112° nach 1,5 s) | Gierwinkel 0,00°, Status `infeasible` |

**Split-µ bei fester Lenkung:** Ohne Gegenlenken muss das Giermoment aus den Bremskräften null bleiben. Der erreichbare Bremsweg liegt deshalb nahe dem Select-low-Grenzwert ($v^2/(2\cdot 0{,}2\,g)$ = 159 m). Die Allokation nutzt zusätzlich die Lastverteilung und liegt mit 151 m darunter. Ein kürzerer Bremsweg erfordert ein Gegenlenken, das das Giermoment über Seitenkräfte abstützt. Das ist integrierte Fahrdynamikregelung mit Lenkeingriff und gehört zu M6.

Der Status `infeasible` ist bei Vollbremsung korrekt: Das Pedal fordert etwa 32 kN, der Kraftschluss erlaubt deutlich weniger. Bei ausgefallener Bremse liegt die giermomentneutral erreichbare Verzögerung bei etwa 3,5 m/s². Die Allokation meldet den Rest als Residuum.

## 4. Aktuatoren

- **Ideale Varianten** (Plan.md, Abschnitt 9): Lenk-, Brems- und Antriebsaktuator haben `ideal=True`. Dann entfallen Verzögerung, Stellrate bzw. Slew-Rate und Totzeit; Winkel-, Moment- und Leistungsgrenzen sowie Fehlermodi bleiben.
- **Lenkübersetzung:** `SteeringActuatorConfig.steering_ratio` und `step_handwheel()`, konstante Übersetzung.
- Die Zeitkonstante des Rekuperationsmoments war fest im Code (0,02 s). Sie ist jetzt ein dokumentierter Konfigurationswert (`tau_regen`).

## 5. Verifikation

| Test | Prüfung |
|---|---|
| [test_control_allocation.py](../tests/vehicle_model/test_control_allocation.py) | Löser = scipy BVLS (200 Zufallsprobleme); ohne Grenzen = gewichtete Pseudoinverse; Priorität entscheidet bei Unerfüllbarkeit; Wirksamkeitsmatrix; gleiche Kraftschlussausnutzung; Reibkreisgrenze; `infeasible` mit Giermomentpriorität; `saturated` ohne Anforderungsverlust; Bremsenausfall (bis zur analytischen Grenze 2 × Griff hinten links); Spiegelsymmetrie; ungültige Eingaben |
| [test_slip_and_stability_control.py](../tests/vehicle_model/test_slip_and_stability_control.py) | ABS hält den Zielschlupf ohne Grenzzyklus und erreicht > 95 % (µ = 1,0 / 0,5 / 0,2); µ̂ adaptiert auf den wahren Wert; Überschätzung schlägt Blockieren; TCS; Split-µ stabil, wo das passive Fahrzeug sich eindreht; TV erreicht die Referenz auf 5 %; Bremsenausfall kompensiert und gemeldet; Konfigurationsprüfung |
| [test_actuator_variants.py](../tests/vehicle_model/test_actuator_variants.py) | ideale Varianten folgen sofort, Grenzen und Fehler bleiben wirksam; Lenkübersetzung; `tau_regen` wirkt |

## 6. Grenzen

- Die Allokation ist quasistatisch: Sie ordnet Kräfte zu, Raddrehdynamik und Schlupf übernimmt der innere Regelkreis. Die Wirksamkeitsmatrix gilt für die aktuellen Radwinkel und Lasten.
- Seitenkräfte werden nicht zugeordnet, sondern nur als belegter Reibkreis berücksichtigt. Lenkeingriffe folgen in M6.
- ABS/TCS: Die Kalibriertabelle gehört zum MF-Reifensatz. Es gibt keine Radbeschleunigungsschwellen, keine Druckmodulationsphasen und keine Hydraulik; Aktuatordynamik liegt in `actuators/`, ist aber noch nicht in den Szenarien verschaltet. Die Reifenkraft stammt ideal aus der Telemetrie.
- Die Stabilitätsregelung nutzt den Fahrbahnreibwert ideal. Mit Totband (ESC-Betrieb) ist sie nicht systematisch untersucht.
- Die Bestandsregler (`abs.py`, `tcs.py`, `torque_vectoring.py`, `integrated_controller.py`) sind unverändert. Sie verwenden diese Allokation nicht.
