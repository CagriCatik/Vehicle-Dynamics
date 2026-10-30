# V4.0 M19: Roll Stability Control (RSC) und Active Front Steering (AFS)

Masterplan V4.0 (`Plan_v2.md`), M19. Synthetische Parameter, virtuelle Modelle; Manöver an NHTSA-Fishhook, ISO 7401
und den Sine-with-Dwell-Test **angelehnt**, keine Normkonformität (FMVSS 126 nur als Szenarienvorbild).

## 1. RSC auf dem 14-DOF-Modell

Wankstabilität mit Radlasten und Radkontakt braucht die Hub-, Wank- und Radfreiheitsgrade des 14-DOF-Modells (Blocker E);
das 7-DOF-Modell wird dafür nicht verwendet.

- `controllers/roll_stability.py` (`RollStabilitySupervisor`): beurteilt `LTR = |F_z,links − F_z,rechts| / ΣF_z`, eine
  Vorhersage `LTR + T·dLTR/dt` (gefiltert) und den Radkontakt; Modi inaktiv → aktiv (`LTR_vorh ≥ 0,75`) → Notfall (Rad ohne
  Kontakt oder `LTR ≥ 0,95`), Rückkehr erst nach Hysterese (`< 0,55` für 0,3 s) und Mindestaktivzeit (0,4 s). Er **fordert
  an**: Bremskraft und ein untersteuerndes Giermoment an die gemeinsame Allokation (die dann die Außenräder bremst),
  Antriebsrücknahme und, falls die aktive Federung vorhanden und gesund ist, den vorhandenen
  `ActiveRollDistributionController` (nicht überschrieben).
- `simulation/roll_stability.py`: geschlossener Kreis auf dem 14-DOF-Modell (1 ms Strecke, 5 ms Regler, Informationsmodus
  A, Gas gelöst); Experimenttyp `rollover_benchmark` (`experiments/rollover.py`) für die Matrix.
- **LTR = 1 oder ein abgehobenes Rad ist kein Überschlagsbeweis** (Plan). Der Bericht nennt Kontaktverlust (ein Rad /
  zwei Räder) und den größten Wankwinkel getrennt; Wankwinkel über 15° liegen außerhalb der Kleinwinkelkinematik der
  Federung und würden als „außerhalb der Modellgültigkeit“ ausgewiesen (im Benchmark nicht erreicht, größter Wert 10,9°;
  in einer Vorstudie mit h_S 1,25 m rechnete das Modell 160°: solche Werte sind keine Vorhersage).

### 1.1 Auslegung (gemessen, Fishhook 80 km/h, 8°, µ 1)

| Variante | h_S 0,95 m: LTR / Abheben / zwei Räder / Gierfehler | h_S 1,1 m: Abheben / zwei Räder / Gierfehler |
|---|---|---|
| nur Bremsen, kein Giermoment | 0,94 / 0,54 s / 0 / **130 °/s** (dreht sich) | 1,0 s / 0,37 s / **138 °/s** |
| Notbremsung 0,8 g, Giermoment | 0,86 / 0,65 s / 0 / 36 °/s | 1,4 s / 0 / **94 °/s** (dreht sich) |
| **0,2 g, Notfall 0,35 g, k_yaw 1,0 (Standard)** | **0,76 / 0 / 0 / 30 °/s** | 2,7 s / 0 / **22 °/s** |

Reines Bremsen und starkes Bremsen drehen das Fahrzeug; wirksam ist das untersteuernde Giermoment der Außenradbremsung,
das die Querbeschleunigung und damit `m a_y h / s` senkt.

### 1.2 Benchmark (`configs/experiments/v4_m19_rollover.yaml`, 80 Läufe)

Vier Beladungsvarianten (nominal; teilbeladen h_S 0,75 m; beladen h_S 0,95 m; extrem h_S 1,1 m; je 1520 kg Aufbau außer
nominal), Fishhook 80 km/h 8° und Ausweichen 120 km/h (eine Sinusperiode 4°, 0,6 Hz), µ 1,0 und 0,6; fünf Regler.
Auszug µ 1,0, Fishhook (Ergebnisse `output/v4/m19/rollover/`):

| Variante | Regler | LTR max | Abheben | zwei Räder | Wankwinkel | Gierfehler max | Geschw.-Verlust |
|---|---|---|---|---|---|---|---|
| beladen 0,95 m | passiv | 1,00 | 0,21 s | 0,15 s | 7,7° | 43 °/s | 59 km/h |
| | nur aktive Wankregelung | 1,00 | 1,69 s | 0,35 s | 4,1° | 38 °/s | 49 km/h |
| | RSC | 0,76 | 0 | 0 | 4,9° | 30 °/s | 41 km/h |
| | RSC + aktive Wankregelung | 1,00 | 0,06 s | 0,05 s | 4,0° | 24 °/s | 45 km/h |
| | RSC + aktive Wankregelung (ausgefallen) | 0,76 | 0 | 0 | 4,9° | 30 °/s | 41 km/h |
| extrem 1,1 m | passiv | 1,00 | 1,14 s | 0,95 s | 10,9° | 42 °/s | 52 km/h |
| | nur aktive Wankregelung | 1,00 | 3,29 s | 0,82 s | 5,3° | 42 °/s | 46 km/h |
| | RSC | 0,89 | 2,66 s | 0 | 7,4° | 22 °/s | 50 km/h |
| | RSC + aktive Wankregelung | 1,00 | 1,14 s | 0,08 s | 4,5° | 37 °/s | 48 km/h |

Befunde:

- **RSC beseitigt das Abheben zweier Räder** in allen Fällen, in denen das passive Fahrzeug es zeigt (beladen und extrem,
  Fishhook µ 1), und halbiert den Gierfehler. Beim extremen Schwerpunkt hebt ein Rad dafür länger ab (2,7 s gegen 1,1 s):
  Das kurveninnere Hinterrad entlastet unter der Bremsung. Das ist kein Gewinn auf jeder Kennzahl.
- **Aktive Wankregelung allein** halbiert den Wankwinkel, verlängert aber das Abheben (0,21 → 1,69 s, 1,14 → 3,29 s):
  Sie stellt den Aufbau auf, verringert aber die Lastverlagerung `m a_y h / s` nicht und verschiebt sie zwischen den
  Achsen. Wankwinkel ist kein Maß für das Kipprisiko.
- **RSC + aktive Wankregelung** ist beim Abheben schlechter als RSC allein (beladen 0,06/0,05 s gegen 0/0).
- **Ausgefallene aktive Federung**: RSC arbeitet allein über die Bremsen weiter (Ergebnis identisch mit RSC; Test).
- Bei µ 0,6 und beim Ausweichen 120 km/h hebt kein Rad ab (Reibwert begrenzt die Querbeschleunigung); RSC greift bei
  den hohen Varianten trotzdem ein und senkt dort den Gierfehler (z. B. teilbeladen, Ausweichen µ 1: 38 → 11 °/s).
- Die offenen Manöver überfordern das Fahrzeug teils auch ohne Kippgefahr (passiv dreht sich im Ausweichen 120 km/h,
  Geschwindigkeitsverlust 95 km/h); RSC ist kein ESC und greift dort bei LTR < 0,75 nicht ein.

## 2. AFS auf der 7-DOF-Stellkette

- `controllers/active_front_steering.py`: Der Frontlenkwinkel wird **an einer Stelle** im Motion Controller finalisiert,
  `δ_v = sat(δ_Fahrer · g(v) + Überlagerung)`; `g(v)` ist eine variable Übersetzung (Option `ratio: variable`:
  1,25 / 1,0 / 0,85 bei 0 / 15 / 35 m/s), die Überlagerung kommt vom Gierregelgesetz (Eingänge `afs` oder `afs_tv`
  des LQR/GS-LQR, Entwurfsmodell um den Frontlenkeingang erweitert), begrenzt auf ±5° und 40 °/s; bei ausgefallenem AFS
  ist sie 0 und die Übersetzung fest. Die physische Lenkdynamik ist der vorhandene `SteeringActuator` (`actuators: dynamic`).
- DSL: `inputs: afs | afs_tv`, Block `steering: {ratio: fixed|variable, afs_failed: bool}`.

### 2.1 Lenkwinkelsprung 80 km/h, 2° (`v4_m19_afs_step`)

| Variante | t95 Gierrate | Überschwingen | RMS-Fehler | max. Frontwinkel | max. Lenkrate |
|---|---|---|---|---|---|
| 2WS passiv | 0,18 s | 0,10 | 0,034 rad/s | 2,0° | – (Sprung) |
| TV | 0,14 s | 0,10 | 0,025 | 2,0° | – |
| 4WS+TV | 0,055 s | 0,08 | 0,014 | 2,0° | – |
| AFS | 0,075 s | 0,19 | 0,023 | 4,4° | 40 °/s (Ratengrenze) |
| AFS+TV | 0,11 s | 0,15 | 0,027 | 5,5° | 40 °/s |
| AFS, variable Übersetzung | 0,08 s | 0,15 | 0,025 | 4,3° | 40 °/s |
| AFS mit Lenkaktuator | 0,105 s | **0,49** | 0,035 | 4,3° | 48 °/s |
| AFS ausgefallen | 0,175 s | 0,10 | 0,033 | 2,0° | – |

### 2.2 Sine with Dwell 80 km/h, 6° (`v4_m19_afs_swd`)

| Variante | Gierratenverhältnis 1 s / 1,75 s | max. β | max. Frontwinkel | max. Lenkrate |
|---|---|---|---|---|
| 2WS passiv | 1,24 / 1,22 (dreht sich) | 74° | 6,0° | 26 °/s |
| TV | 0,005 / 0,001 | 1,4° | 6,0° | 26 °/s |
| 4WS+TV | 0,000 / 0,000 | 1,8° | 6,0° | 26 °/s |
| AFS | 0,000 / 0,000 | 2,1° | 5,5° | 66 °/s |
| AFS+TV | 0,000 / 0,000 | 1,3° | 11,0° | 66 °/s |
| AFS mit Lenkaktuator | 0,000 / 0,000 | 2,7° | 7,1° | 60 °/s (Aktuatorgrenze) |
| AFS ausgefallen | 1,24 / 1,22 (dreht sich) | 74° | 6,0° | 26 °/s |

Befunde:

- **AFS wirkt mit physikalisch wirksamen Lenkwinkeln**: allein hält es den Sine with Dwell stabil (passiv dreht sich),
  indem es dem Fahrer entgegenlenkt (5,5° statt 6°); im Sprung erreicht es die Gierrate schneller als 2WS und TV, mit mehr
  Überschwingen. 4WS+TV ist im Sprung am besten.
- **Gesättigte Stellraten sind sichtbar**: Die Überlagerung steht an ihrer Ratengrenze (40 °/s; im Sine with Dwell
  Fahrer 26 °/s + 40 °/s = 66 °/s), mit Lenkaktuator an dessen Grenze (60 °/s).
- **Aktuatorverzug kostet**: Mit dem Lenkaktuator (45 ms) steigt das Überschwingen im Sprung von 0,19 auf 0,49; der
  LQR ist ohne Verzug entworfen.
- **Vorzeichen**: zu wenig Gierrate → Überlagerung in Lenkrichtung, im Sine with Dwell gegen die Drehung (Test).
- **AFS+TV** nutzt Giermoment und Überlagerung zugleich; der Frontwinkel erreicht 11° (Überlagerung in Lenkrichtung zum
  Fahrerwinkel): mehr Lenkeingriff, aber nicht besser als AFS oder TV allein.

## 3. Abnahme M19

| Kriterium | Bewertung |
|---|---|
| RSC bewahrt Stellgrenzen | erfüllt: Anfragen an die gemeinsame Allokation (Brems- und Haftgrenzen), begrenzte Verzögerung und Giermoment, Aktivkräfte im Hüllband des vorhandenen Reglers |
| auch nicht rettbare Grenzfälle protokolliert | erfüllt: Kontaktverlust, LTR, Wankwinkel, Modellgültigkeit und Ergebnisklasse je Lauf; Verschlechterungen (Einzelradabheben, aktive Wankregelung) ausgewiesen |
| AFS mit physikalisch wirksamen Lenkwinkeln, aktive Wirkung gegenüber 2WS/4WS | erfüllt: realisierter Frontwinkel im Strecken-Input; Sine with Dwell stabil gegenüber 2WS; Vergleich zu TV und 4WS+TV |
| Vorzeichen und gesättigte Stellraten nachweisbar | erfüllt: Vorzeichentest, Raten an Überlagerungs- und Aktuatorgrenze in den Kennzahlen |
| Frontlenkwinkel nur an einer Stelle finalisiert | erfüllt: `SteeringOverlay.finalise` im Motion Controller |

Tests: `tests/vehicle_model/test_v4_m19_rsc_afs.py` (7 Tests, 2 `slow`).

## 4. Grenzen

- RSC in Informationsmodus A (Radlasten und Kontakt aus der 14-DOF-Strecke); keine Radlastschätzung.
- Kein Fahrer im Kreis, keine DLC-/Spurwechselbahn für AFS (nur offene Manöver).
- 14-DOF-Wankkinematik nur für kleine Winkel; Überschlag selbst wird nicht modelliert.
- AFS-LQR ohne Aktuatorverzug entworfen (Überschwingen mit Lenkaktuator).
