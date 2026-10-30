# Interactive Engineering Workbench (M15)

Masterplan V3.0, Abschnitt 10. Eine grafische Forschungsumgebung über der vorhandenen YAML/CLI-Experimentinfrastruktur:
Versuch konfigurieren, ausführen, Ergebnisse ansehen und vergleichen, technischen Bericht exportieren. Die Oberfläche
ist ein Adapter; Konfiguration, Validierung und Simulation bleiben in der Experiment-Engine.

```bash
pip install -e ".[workbench]"          # optional: fastapi, uvicorn, httpx
python -m vehicle_model.workbench       # dann http://127.0.0.1:8765 öffnen (aus dem Repository-Wurzelverzeichnis)
```

## 1. Technologieentscheidung (Plan 10.5)

FastAPI-Backend mit einer Browseroberfläche in reinem HTML/CSS/JavaScript (kein Build-Schritt), als optionales Extra
`workbench`; die Kernbibliothek bleibt ohne neue Abhängigkeit.

| Maßstab | Bewertung |
|---|---|
| Wartbarkeit | Logik in `service.py` und `jobs.py` (ohne HTTP testbar), Routen in `app.py`, Oberfläche in `static/` |
| Plattformunabhängigkeit | Browser und Python; keine nativen GUI-Bibliotheken |
| lange Rechenprozesse | jeder Lauf ist ein eigener Prozess der CLI; Fortschritt, Abbruch, Log |
| Wiederverwendung | Optionen aus `experiments.spec`, Validierung durch den Parser des Experimenttyps, Ergebnisse sind die vorhandenen PNG/GIF/CSV/JSON/Markdown-Ausgaben |
| Distribution | `pip install -e ".[workbench]"`, ein Befehl zum Start |
| Abhängigkeiten | fastapi, uvicorn (httpx nur für Tests) |

Erwogen und verworfen: PySide6 (große Abhängigkeit, Distribution aufwendig), Streamlit (Neuausführungsmodell passt
schlecht zu langen, abbrechbaren Läufen), reiner `http.server` (mehr eigener Code für Routing und Validierung).

## 2. Architektur

```
Browser (static/index.html, app.js)
   │  JSON über HTTP, 127.0.0.1
FastAPI-Adapter (workbench/app.py) ── nur Routen
   │
Anwendungsschicht (workbench/service.py, jobs.py)
   │  parse_experiment / get_type(kind).parse      subprocess: python -m vehicle_model.experiments run <yaml> --out <dir>
Experiment-Engine (experiments/*)  ◄────────────────────────────────────────────────┘
```

- **Keine zweite Konfigurationslogik (Plan 10.1):** Auswahllisten (Fahrzeuge, Reifen, K&C, Regelgesetze, Eingänge,
  Rückführmodi, Fahrbahnen, Manöver, Sweep-Parameter, Formate) stammen aus `experiments.spec` und den Konfigurations-
  verzeichnissen. Das Formular stellt nur das YAML-Mapping eines `controller_comparison` zusammen. Jede Validierung
  läuft über den Parser des Experimenttyps, gespeichert wird genau das YAML, das die CLI liest. Alle anderen
  Experimenttypen (Robustheit, Stabilitätskarten, kritische Szenarien, Auslegung, …) werden im YAML-Editor der
  Oberfläche mit derselben Validierung bearbeitet.
- **Ausführung getrennt vom Event Loop (Plan 10.2):** Ein Lauf ist der CLI-Befehl in einem eigenen Prozess (eigene
  Prozessgruppe). Der Server bleibt während eines Laufs bedienbar (Test), Abbrechen beendet die Prozessgruppe mit den
  Worker-Prozessen. Fortschritt kommt aus `[progress] k/n`-Zeilen der Experimenttypen
  (`types.report_progress`: Reglervergleich je Lauf, Auslegung je Generation, kritische Suche je Stufe); Typen ohne
  diese Zeilen zeigen Laufzeit und Log.
- **Reproduzierbarkeit über die CLI (Plan 10.6):** Jeder Lauf speichert in `job.json` die Befehlszeile; die Oberfläche
  zeigt sie zu jedem Ergebnis an. Test: dieselbe Konfiguration über die CLI ergibt dieselben Kennzahlen (bis auf die
  Rechenzeit).

Verzeichnisse: `output/workbench/configs/` (gespeicherte YAML), `output/workbench/runs/<id>/` (job.json, log.txt,
config.yaml, Ausgaben des Laufs), `output/workbench/exports/` (Berichte). Ergebnisse von CLI-Läufen unter
`output/experiments/` werden mit angezeigt. Dateien werden nur aus diesen Verzeichnissen ausgeliefert (Test gegen
Pfade außerhalb).

## 3. Oberfläche

| Bereich | Inhalt |
|---|---|
| **Configure** | Fahrzeug, Reifen vorn/hinten, Fahrwerk (K&C), Regler (Gesetz, Eingänge, Rückführung; mehrere), Fahrbahn, Manöver mit Anfangsgeschwindigkeit, Parameter-Sweep, Schrittweite, Seed, Ausgabeformate; YAML-Vorschau mit Live-Validierung; vorhandene Konfigurationen laden und wiederverwenden; YAML-Editor für alle Experimenttypen; Speichern, Speichern und Starten |
| **Runs** | Läufe mit Status, Fortschritt, Laufzeit, Befehlszeile, Log; Abbrechen; Ergebnis öffnen |
| **Results** | Übersicht (Kennzahlen, Bericht, Modellprüfungen, Provenienz, Reproduktionsbefehl); **Vehicle dynamics**: Bahn, Gierrate mit Referenz, Schwimmwinkel, Längs- und Querbeschleunigung, Lenkwinkel vorn/hinten, Seitenkräfte und Radlasten je Rad; **Motion control**: Giermoment Soll/Ist (Allokation), Gierrate Soll/Ist, Reifenausnutzung, Schlupf, Bremsmoment, Längskräfte je Rad, Reglerstatus; **Nonlinear stability / Optimization / Critical scenarios**: die Bilder des jeweiligen Typs (Phasenporträts, Stabilitätskarten, Gleichgewichte, Eigenwerte; Pareto-Fronten, Sensitivitäten, Verteilungen); **Tables**: CSV-Tabellen inline (z. B. Pareto-Entwürfe) und als Download |
| **Compare & report** | Kennzahlen mehrerer Ergebnisse mit relativer Abweichung zur ersten Zeile, überlagerte Zeitverläufe eines Kanals, Differenzdiagramm, animierte Fahrzeugbewegung aus den aufgezeichneten Bahnen; Export eines technischen Berichts (ZIP: Markdown mit Konfiguration, Provenienz, Reproduktionsbefehl, Bildern und Kennzahlvergleich) |

Diagramme: SVG-Linien mit Fadenkreuz und Tooltip, eine y-Achse je Diagramm, Legende ab zwei Reihen, Farben aus der
Palette der vorhandenen Plots. Die bestehenden Matplotlib-Plots, PNG/GIF/CSV-Exporte bleiben unverändert und werden
angezeigt (Plan 10.4).

Erweiterung der Engine für das Dashboard: `run_scenario` zeichnet zusätzlich `v_y`, `a_x`, `a_y` (aus den
Reifenkräften, ohne Luftwiderstand) und `F_z`, `F_x`, `F_y` je Rad auf; die Zeitverläufe der Reglervergleiche
enthalten diese Kanäle, dazu Geschwindigkeit, Reifenausnutzung, Schlupf, Bremsmoment und Status. Ältere Ergebnisse
ohne diese Spalten werden angezeigt, die fehlenden Diagramme entfallen.

## 4. Grenzen

- Lokales Werkzeug ohne Anmeldung, an 127.0.0.1 gebunden; nicht für den Betrieb im Netz gedacht.
- Das Formular deckt den Reglervergleich ab; andere Typen werden als YAML bearbeitet (mit Validierung), nicht über
  eigene Formulare.
- Fortschritt nur für Typen mit `[progress]`-Zeilen; Robustheitsbenchmark und Stabilitätskarten zeigen Laufzeit und Log.
- Die Animation zeigt die aufgezeichneten Bahnen als maßstäbliche Rechtecke; die detaillierten GIF-Animationen der
  Engine bleiben die Referenz.
- Kein Browser-Test der Oberfläche in der Testsuite; geprüft sind Backend, Prozesssteuerung und die Auslieferung der
  Oberfläche (`test_workbench.py`), das JavaScript per Syntaxprüfung.

## 5. Abnahme (Plan 10.6)

`tests/vehicle_model/test_workbench.py`: Konfiguration über die HTTP-Schnittstelle, Validierung durch den Parser der
Engine, Speichern als YAML, Start eines Laufs als CLI-Prozess, Fortschritt bis `2/2`, Ergebnis mit Kennzahlen,
Zeitverläufen (Radlasten tragen das Gewicht) und Bildern, Vergleich zweier Läufe, Export des Berichts als ZIP und
Reproduktion derselben Konfiguration über die CLI mit identischen Kennzahlen; ein langer Lauf blockiert den Server nicht
und lässt sich abbrechen.
