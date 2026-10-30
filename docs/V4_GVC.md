# V4.0 M20: GVC-artige Lasttransferregelung

Masterplan V4.0 (`Plan_v2.md`), M20. **Eigene vereinfachte Referenzstrategie**, nicht der proprietäre Mazda-Algorithmus
(Quelle [R10] nur für das veröffentlichte Prinzip). Synthetische Parameter, virtuelle Modelle; keine zusätzliche
Seitenkraft wird erzeugt.

## 1. Strategie

`controllers/g_vectoring.py` (`GVectoringControl`), im Motion Controller zur Längskraftanforderung addiert:

- erwartete Querbeschleunigung aus dem Fahrerlenkwinkel `a_y = v² δ / (L (1 + K v²))`, Querruck aus der gefilterten
  Lenkgeschwindigkeit (kein gemessener Querruck nötig);
- **Einlenken** (`a_y · ȧ_y > 0`, |ȧ_y| über einer kleinen Totzone): `a_x = −C_xy |ȧ_y|`, gefiltert (0,1 s), begrenzt auf
  `max_decel` (Standard 0,05 g) und einen Ruck von 2 m/s³ der Anforderung;
- **Auslenken**: glatte Freigabe auf 0, nie eine positive Anforderung;
- **Vorrang**: Fahrerbremspedal über 5 %, aktives ABS oder unter 5 m/s → keine Anforderung;
- `enabled: false` → Anforderung exakt 0; die Funktion verschwindet (Test: Kennzahlen bitgleich zum Lauf ohne GVC).
- Die Bremsunterstützung zur Gierstabilisierung (Untervariante des Plans) ist **nicht** umgesetzt.

DSL: Block `gvc: {enabled, c_xy, max_decel_g}` je Regler; Manöver `steer_ramp`, `lane_change`, `lift_off` mit
konstantem Fahrpedal (gleiche Fahrerführung für GVC aus/an).

## 2. Mechanismus in den Strecken

- **7-DOF** (quasistatische Lastverlagerung): Die GVC-Verzögerung erhöht die Vorderachslast beim Einlenken
  (Lenkrampe µ 1: +123 N gegenüber +43 N ohne GVC; mit C_xy 0,3: +155 N). Wirkung auf die Querdynamik nur über die
  Lastempfindlichkeit der Reifen.
- **14-DOF** (dynamisch): Eine Verzögerung in GVC-Größe (0,05 g über die Bremsen) nickt den Aufbau nach vorn und belastet
  die Vorderachse; stationär überträgt das Modell etwa zwei Drittel des Starrkörperwerts `m a_x h / L` (89 % im
  Überschwingen), weil die Längskräfte teilweise unterhalb des Schwerpunkts angreifen. Test vorhanden.

## 3. Ergebnisse (`configs/experiments/v4_m20_gvc_*.yaml`, `output/v4/m20/`, Abbildung `gvc.png`)

### 3.1 Lenkrampe 80 km/h, 3° bei 20 °/s, konstantes Pedal

| µ | Variante | Gierraten-RMS-Fehler | max. Gierfehler | t63 Gierrate | max. β | Vorderachslast + | Geschwindigkeitsverlust | Längsruck max. |
|---|---|---|---|---|---|---|---|---|
| 1,0 | ohne Gierregelung, GVC aus | 0,0319 rad/s | 8,56 °/s | 0,18 s | 1,83° | 43 N | 4,39 km/h | 1,9 m/s³ |
| 1,0 | … GVC | 0,0321 | 8,51 | 0,18 | 1,79° | 123 N | 4,68 | 3,8 |
| 1,0 | … GVC C_xy 0,3 | 0,0324 | 8,51 | 0,18 | 1,78° | 155 N | 4,86 | 3,8 |
| 1,0 | PI / PI + GVC | 0,0182 / 0,0184 | 4,60 / 4,58 | 0,15 / 0,15 | 1,64° / 1,61° | 43 / 127 N | 3,96 / 4,25 | 4,5 / 6,7 |
| 0,5 | ohne Gierregelung, GVC aus | 0,132 | 11,3 | 0,225 | 28,1° | 47 N | 11,8 | 7,6 |
| 0,5 | … GVC | 0,128 | 11,1 | 0,22 | 27,0° | 116 N | 11,8 | 8,5 |
| 0,5 | … GVC C_xy 0,3 | 0,121 | 10,5 | 0,215 | 24,9° | 146 N | 11,3 | 3,5 |

### 3.2 Spurwechsel, Gaswegnahme in der Kurve, Notbremsung

- **Spurwechsel** (2,5°, 0,5 Hz): max. Gierfehler 5,45 → 5,39 °/s, β 1,09° → 1,08°; Geschwindigkeitsverlust 0,3 → 1,8 km/h
  (C_xy 0,3: 3,2 km/h). Die Gierrate ist praktisch unverändert.
- **Gaswegnahme in der Kurve** (Pedal 0,3 → 0 bei 80 km/h, 3°): Ohne Gierregelung dreht sich das Fahrzeug mit und ohne
  GVC (β 89,5°); GVC wirkt nur beim Einlenken, nicht auf das Lastwechselübersteuern. Mit PI stabil, GVC ändert nichts
  Wesentliches.
- **Notbremsung während der Kurvenfahrt**: identisch mit/ohne GVC. Im Manöver steht der Lenkwinkel von Beginn an, GVC
  ist daher nie aktiv; der Vorrang von Bremspedal und ABS ist im Test belegt (Anforderung 0).

Befunde:

- Der berichtete Mechanismus (Verzögerung → Vorderachslast) existiert in beiden Strecken; seine Wirkung auf Gierrate
  und Schwimmwinkel ist bei µ 1 im 7-DOF-Modell **sehr klein** (< 1 % Gierfehler, −2 % β), bei µ 0,5 jenseits der
  Grenze größer (β −4 %, mit C_xy 0,3 −11 %). Eine Verbesserung des Lenkverhaltens wird nicht behauptet.
- Kosten: Geschwindigkeitsverlust (+0,3 bis +1,5 km/h je Manöver) und Längsruck: Die Anforderung ist auf 2 m/s³ begrenzt,
  die gemessene Fahrzeugbeschleunigung zeigt bis 3,8 m/s³, weil Rad- und Reifendynamik nachschwingen.

## 4. Abnahme M20

| Kriterium | Bewertung |
|---|---|
| Mechanismus im verwendeten Plant vorhanden | erfüllt: 7-DOF (Vorderachslast) und 14-DOF (Nicken, Vorderachslast) |
| Priorität der Sicherheitsfunktionen | erfüllt: Bremspedal, ABS und kleine Geschwindigkeit schalten GVC ab (Test) |
| GVC kann mit `enabled: false` vollständig verschwinden | erfüllt: Kennzahlen bitgleich (Test, Benchmarks) |
| kein nachträglich verbesserter Solltrajektorienvergleich | erfüllt: gleiche offene Fahrerführung, gleiche Referenz, alle Läufe berichtet, auch ohne Wirkung |

Tests: `tests/vehicle_model/test_v4_m20_gvc.py` (5 Tests).

## 5. Grenzen

- Keine Fahrerregelung (offene Manöver); keine subjektive Bewertung, Steueraufwand nur über die gleiche Fahrervorgabe.
- 14-DOF nur für den Mechanismus geprüft, die Benchmarks laufen im 7-DOF-Modell.
- Keine Bremsunterstützung zur Gierstabilisierung (Untervariante nicht umgesetzt).
