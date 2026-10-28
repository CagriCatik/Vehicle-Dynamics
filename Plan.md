# VEHICLE DYNAMICS – MASTER ENGINEERING PLAN V2.0

**Projekt:** Vehicle Dynamics & Integrated Chassis Control Development Platform  
**Entwicklungsansatz:** Simulation-First / Virtual Engineering  
**Zielplattform:** Generisches, realistisch skaliertes Pkw-Modell  
**Repository:** https://github.com/CagriCatik/Vehicle-Dynamics  
**Projektstatus:** M0 bis M8 abgeschlossen; nächster Meilenstein M9. Ist-Zustand siehe `docs/AUDIT.md`, M1-Bericht siehe `docs/HANDLING_ANALYSIS.md`, M2 (Kontakt und Bilanzen) siehe `docs/VEHICLE_MODELS.md`, Abschnitt 2.3, M3 (Reifen) siehe `docs/TYRE_MODELS.md`, M4 (Kinematik) siehe `docs/CHASSIS_KINEMATICS.md`, M5 (Allokation, Schlupfregelung) siehe `docs/CONTROL_ALLOCATION.md`, M6 (Regler, Benchmark) siehe `docs/MOTION_CONTROL.md`, M7 (Sensorik, Schätzung, Robustheit) siehe `docs/ESTIMATION_ROBUSTNESS.md`, M8 (Vertikaldynamik) siehe `docs/VERTICAL_DYNAMICS.md`

---

## 1. Projektziel und wissenschaftlicher Anspruch

Ziel ist die schrittweise Entwicklung einer modularen Forschungs-, Lern- und Simulationsplattform für Fahrzeugdynamik, Fahrzeugregelung und integrierte Fahrwerksysteme.

Die Plattform soll es ermöglichen, ein vollständig konfigurierbares virtuelles Fahrzeug unter reproduzierbaren Bedingungen zu simulieren, seine dynamischen Eigenschaften zu analysieren, verschiedene Regelungsverfahren zu untersuchen und deren Auswirkungen objektiv zu vergleichen.

Folgende Fachgebiete gehören zum Gesamtumfang:

1. Längs-, Quer- und Vertikaldynamik.
2. Lineare und nichtlineare Fahrzeugmodelle.
3. Reifenmodellierung einschließlich Combined Slip.
4. Fahrwerkskinematik und Elastokinematik.
5. Vorderachs-, Hinterachs- und Allradlenkung.
6. Torque Vectoring und Antriebsstrang.
7. ABS, Antriebsschlupfregelung und ESC.
8. Passive, semiaktive und aktive Fahrwerksysteme.
9. Vehicle Motion Control und Control Allocation.
10. Klassische und modellprädiktive Regelungsverfahren.
11. Zustandsschätzung und Sensitivitätsanalyse.
12. Virtuelle Fahrmanöver und variable Fahrbahnverhältnisse.
13. Fahrzeughandling-, Stabilitäts- und Frequenzganganalysen.
14. Numerische Verifikation, automatisierte Experimente und Ergebnisvisualisierung.

**Das Projekt benötigt keine realen Messdaten.**

Alle Modelle werden mit physikalisch plausiblen, dokumentierten Parametern und geeigneten mathematischen Referenzlösungen entwickelt.

Die Plattform muss zwischen folgenden Nachweisarten unterscheiden:

- Mathematische Verifikation: Stimmen Formulierung und mathematische Eigenschaften?
- Numerische Verifikation: Wird das mathematische Modell mit ausreichender Genauigkeit berechnet?
- Physikalische Konsistenzprüfung: Werden Kräfte-, Momenten-, Energie- und Kontaktbedingungen eingehalten?
- Modellvergleich: Stimmen unterschiedliche Modellimplementierungen innerhalb gemeinsamer Annahmen überein?
- Virtuelle Robustheitsprüfung: Wie verändern sich die Ergebnisse bei Parameter- und Störgrößenvariationen?

Experimentelle Fahrzeugvalidierung ist ausdrücklich kein Pflichtbestandteil.

Das Projekt darf keine Fahrzeuggenauigkeit, OEM-Validierung oder Serienreife behaupten, die nicht nachgewiesen wurde.

---

## 2. Verbindliche Entwicklungsgrundsätze

### 2.1 Engineering First

Jedes neue physikalische Modell benötigt vor der Implementierung eine Beschreibung seiner Zustände, Eingänge, Ausgänge, Parameter, Koordinatensysteme, Differentialgleichungen, Randbedingungen und Gültigkeitsgrenzen.

Komplexität ist nur dann gerechtfertigt, wenn sie einen erkennbaren physikalischen oder regelungstechnischen Erkenntnisgewinn bietet.

### 2.2 Structure First

Softwarearchitektur und mathematische Modellierung müssen voneinander getrennt bleiben.

Verbindliche Prinzipien:

- Ein Modul besitzt eine klar abgegrenzte Verantwortung.
- Fahrzeugmodelle sind unabhängig von Reglerimplementierungen.
- Regler sind unabhängig von konkreten Visualisierungssystemen.
- Reifen-, Lenkungs-, Brems- und Antriebsmodelle bleiben austauschbar.
- Konfigurationsdaten dürfen nicht in mehreren Modulen dupliziert werden.
- Numerische Algorithmen dürfen keine versteckten globalen Zustände verwenden.
- Ein simulationsfähiger Projektstand muss nach jedem Meilenstein erhalten bleiben.
- Änderungen an bestehenden Schnittstellen erfordern Tests und gegebenenfalls einen Migrationsadapter.

### 2.3 Reproducibility First

Jedes Simulationsergebnis muss mit der ursprünglichen Konfiguration, der Softwareversion und den verwendeten numerischen Methoden reproduzierbar sein.

Zufallsbasierte Experimente benötigen dokumentierte Seeds.

### 2.4 Physics Before Control

Die Entwicklung komplexer Controller darf nicht vor der hinreichenden Verifikation der zugrunde liegenden Fahrzeug- und Aktuatormodelle erfolgen.

Vorhandene Controller bleiben erhalten, werden jedoch schrittweise auf gemeinsame Schnittstellen umgestellt.

---

## 3. Repository-Audit und Bestandsintegration

Die Entwicklung erfolgt im bestehenden Repository. Es wird kein paralleles Ersatzprojekt aufgebaut.

Zuerst sind folgende Punkte zu prüfen:

- Aktueller Git-Branch, Working Tree und uncommittete Änderungen.
- Vorhandene Fahrzeugmodelle und deren mathematische Gleichungen.
- Bestehende Reifenmodellimplementierungen.
- YAML-Konfigurationen und Parameterherkunft.
- Vorhandene NMPC- und Lenkungsimplementierungen.
- MATLAB-Beispiele.
- Testabdeckung und ausführbare Demonstratoren.
- Bestehende Visualisierungen und CSV-Exporte.
- Bereits lokal vorhandene Erweiterungen für 4WS und Torque Vectoring.

Besonderes Augenmerk gilt dem bisherigen Verzeichnis `python/cgmres_nmpc` (jetzt `src/vehicle_model/controllers/nmpc.py`). Eine SLSQP-basierte NMPC-Implementierung darf nicht als mathematisch echtes C/GMRES bezeichnet werden.

Die ursprüngliche Funktionalität muss erhalten bleiben. Vor einem größeren Refactoring sind geeignete Regressionstests anzulegen.

Falls eine lokal entwickelte Chassis-Erweiterung vorhanden ist, muss sie zuerst auf ihren tatsächlichen Implementierungsstand und mögliche Überschneidungen mit anderen Fahrzeugmodellen geprüft werden.

**Ergebnis:** Dokumentierter Ist-Zustand, Testbaseline, Architekturinventar und inkrementeller Integrationsplan.

---

## 4. Virtuelles Referenzfahrzeug

Ein vollständig synthetisches, physikalisch plausibles Referenzfahrzeug bildet die Grundlage für alle weiteren Untersuchungen.

### 4.1 Grundparameter

Das Fahrzeugmodell soll mindestens folgende Parameter enthalten:

- Gesamtmasse und gegebenenfalls gefederte und ungefederte Massen.
- Radstand und Spurweiten.
- Schwerpunktposition in drei Raumrichtungen.
- Trägheitsmomente um Längs-, Quer- und Hochachse.
- Radhalbmesser und Radträgheitsmomente.
- Vorder- und Hinterachsreifenparameter.
- Statische Achslasten.
- Aerodynamische Eigenschaften.
- Rollwiderstand.
- Lenk- und Aktuatorbegrenzungen.
- Antriebs- und Bremsmomentgrenzen.
- Wank- und Nicksteifigkeiten.
- Feder- und Dämpferparameter.

Als erste Konfiguration ist ein konventioneller Pkw mit Frontlenkung und symmetrischem linken/rechten Fahrwerk vorzusehen.

Weitere Konfigurationen werden daraus abgeleitet:

- Frontantrieb.
- Heckantrieb.
- Allradantrieb.
- Vier unabhängig angetriebene Räder.
- Frontlenkung.
- Hinterachslenkung.
- Allradlenkung.
- Passives und aktives Fahrwerk.

### 4.2 Parameterkennzeichnung

Jeder Parametersatz erhält einen Identifikator, eine Versionsnummer und eine Kennzeichnung der Herkunft.

Zulässige Kategorien sind:

- Analytisch hergeleitet.
- Synthetisch definiert.
- Aus dokumentierten allgemeinen technischen Annahmen abgeleitet.
- Durch numerische Optimierung für einen virtuellen Versuch angepasst.

Parameter dürfen nicht ohne Kennzeichnung als reale Fahrzeugdaten ausgegeben werden.

Abgeleitete Parameter wie Radstand, statische Achslasten oder geometrische Hebelarme sollen nach Möglichkeit automatisch berechnet und auf Konsistenz geprüft werden.

---

## 5. Fahrzeugmodellhierarchie

Die Plattform entwickelt mehrere unabhängig nutzbare Modellklassen.

### Modellklasse V1 – Kinematisches Einspurmodell

Zweck: Bahnbewegung, Kurvengeometrie, einfache Lenkungsstrategien und Rangiermanöver.

Berücksichtigt werden Vorwärts- und Rückwärtsfahrt, Front- und Hinterachslenkung sowie die geometrisch korrekte Behandlung von 4WS.

Prüfkriterien:

- Geradeausfahrt bei Null-Lenkwinkel.
- Analytischer Kurvenradius.
- Konsistente Vorzeichen.
- Grenzfall identischer Vorder- und Hinterachslenkwinkel.
- Numerische Konvergenz.

### Modellklasse V2 – Dynamisches Einspurmodell

Zweck: Untersuchung der Querdynamik und des Fahrzeughandling.

Zustände mindestens: Längsgeschwindigkeit, Quergeschwindigkeit und Giergeschwindigkeit sowie die für eine globale Trajektorie notwendigen kinematischen Zustände.

Das Modell berücksichtigt Vorder- und Hinterachsquerkräfte, nichtlineare Reifenreaktionen und die Kopplung zwischen Längs- und Querbewegung.

Zusätzlich wird eine linearisierte 2-DOF-Querdynamikversion als analytische Referenz entwickelt.

### Modellklasse V3 – Nichtlineares Zweispurmodell mit Einzelraddynamik

Zweck: Torque Vectoring, ABS, ESC und individuelle Radkraftregelung.

Sieben dynamische Freiheitsgrade:

- Längsbewegung.
- Querbewegung.
- Gierbewegung.
- Vier Raddrehbewegungen.

Zusätzliche globale Positions- und Orientierungszustände zählen nicht als weitere dynamische Freiheitsgrade.

Das Modell enthält individuelle Radkoordinaten, Radkräfte, Radlasten, Lenkwinkel, Radschlupfwerte, Antriebs- und Bremsmomente.

### Modellklasse V4 – Erweiterte Aufbaudynamik

Ergänzungen:

- Wanken.
- Nicken.
- Heben.
- Aufbaumassen und ungefederte Massen.
- Federwege und Dämpferkräfte.
- Veränderliche Radlasten.

### Modellklasse V5 – Erweitertes Mehrkörper-/Vertikaldynamikmodell

Optionales Langfristziel mit zusätzlichen Freiheitsgraden, fahrwerkskinematischen Kopplungen und gegebenenfalls 14-DOF-Klasse.

Die konkrete Zahl der Freiheitsgrade muss aus den tatsächlich gewählten unabhängigen Zuständen und Modellannahmen folgen.

**Wichtig:** Höhere Modelltreue ersetzt nicht automatisch Modelle geringerer Ordnung. Alle Modellklassen bleiben für passende Untersuchungszwecke verfügbar.

---

## 6. Eigenlenkverhalten und Fahrdynamikanalyse

Ein eigenständiges Modul `handling_analysis` wird verpflichtender Bestandteil des Projekts.

### 6.1 Stationäre Analysen

Zu implementieren sind:

- Stationäre Kreisfahrt.
- Gierverstärkung.
- Eigenlenk- und Untersteuergradient.
- Charakteristische Geschwindigkeit, sofern für die Modellkonfiguration definiert.
- Kritische Geschwindigkeit, sofern mathematisch relevant.
- Lenkwinkelbedarf in Abhängigkeit von Geschwindigkeit und Querbeschleunigung.
- Achsseitenkräfte und Schräglaufwinkel.
- Einfluss von Vorder- und Hinterachslenkung.

### 6.2 Transiente Analysen

Zu implementieren sind:

- Lenkwinkelsprung.
- Lenkwinkelrampe.
- Sinusanregung.
- Frequenzsweep.
- Giergeschwindigkeitsantwort.
- Querbeschleunigungsantwort.
- Schwimmwinkelantwort.
- Überschwingen und Einschwingdauer.

### 6.3 Systemanalyse

Die linearisierte Querdynamik soll systematisch auf Eigenwerte, Eigenvektoren, Dämpfungsgrade, Eigenfrequenzen, Stabilitätsbereiche, Übertragungsfunktionen und Frequenzgänge untersucht werden.

Die analytischen Ergebnisse bilden Referenzen für das nichtlineare Modell innerhalb des gemeinsamen gültigen Betriebsbereichs.

---

## 7. Reifenmodellierung

Eine einheitliche `TyreModel`-Schnittstelle kapselt die Kraft- und Momentenberechnung.

Zu unterstützende Modelle:

1. Lineares Reifenmodell.
2. Fiala-Reifenmodell.
3. Vereinfachte Magic Formula.
4. Parametrisierte Magic Formula mit dokumentierter Formulierung.
5. Combined-Slip-Modell.
6. Transientes Reifenmodell mit Relaxationsdynamik.

Erforderliche physikalische Größen:

- Longitudinalschlupf.
- Schräglaufwinkel.
- Normalkraft.
- Longitudinal- und Querkraft.
- Reifenrückstellmoment, sofern vom Modell unterstützt.
- Sturzempfindlichkeit, sofern unterstützt.
- Reibwertabhängigkeit.
- Lastempfindlichkeit.
- Effektiver Abrollradius.
- Relaxationslänge.

Ein einzelnes Reifenmodell darf nur die Effekte behaupten, die seine Formulierung tatsächlich berücksichtigt.

Keine frei erfundenen MF-5.2- oder MF-6.2-Datensätze als angeblich kalibrierte reale Reifen verwenden.

Die Reifenmodule benötigen Tests für niedrigen Schlupf, Kraftsättigung, Combined Slip, Kontaktverlust, Vorzeichen und stetige Übergänge.

---

## 8. Fahrwerkskinematik und Radlastdynamik

Neue, getrennte Subsysteme:

### Steering Geometry

- Ackermann-Geometrie.
- Front- und Hinterachslenkwinkel.
- Einzelradlenkwinkel.
- Geschwindigkeitabhängige Hinterachslenkung.
- Phasengleiche und gegenphasige 4WS.
- Lenkübersetzung, Verzögerung und Stellraten.

### Suspension Kinematics & Compliance

- Spur- und Sturzveränderung.
- Nachlauf und Spreizung.
- Lenkrollradius.
- Bump Steer.
- Roll Steer.
- Compliance Steer.
- Synthetische und austauschbare K&C-Kennfelder.

### Load Transfer

- Statische Radlastverteilung.
- Longitudinale Radlastverlagerung.
- Laterale Radlastverlagerung.
- Einfluss der Schwerpunktshöhe.
- Einfluss der Spurweite.
- Wanksteifigkeitsverteilung.
- Rollzentren und geeignete geometrische Radlastanteile.
- Dynamische Radlastantwort bei entsprechend erweitertem Modell.

Negative Radlasten dürfen nicht stillschweigend als gültige Kontaktkräfte verwendet werden. Ein Kontaktverlust erfordert eine ausdrücklich modellierte Behandlung.

---

## 9. Antrieb, Bremsen und Aktuatoren

Alle Fahrzeugaktuatoren müssen über gemeinsame, austauschbare Schnittstellen angebunden werden.

### Antrieb

- Front-, Heck- und Allradantrieb.
- Einzelradantrieb.
- Ideales und verzögertes Antriebsmoment.
- Drehzahlabhängige Momentengrenzen.
- Leistungsgrenzen.
- Wirkungsgrad- und Verlustmodelle.
- Regeneratives Bremsmoment.

### Bremsen

- Einzelradbremsmoment.
- Stellverzögerung.
- Bremsmomentsättigung.
- ABS mit Radschlupfregelung.
- Antriebsschlupfregelung.
- Elektronische Stabilitätsregelung.
- Bremskraftverteilung.
- Regeneratives Brake Blending.

### Lenkungsaktuatoren

- Winkel- und Stellratenbegrenzung.
- Verzögerungen.
- Dynamische Aktuatormodelle.
- Fehlermodi.
- Stellsättigung.

Die Modellkomplexität der Aktuatoren muss konfigurierbar sein. Eine ideale Aktuatorvariante bleibt für analytische Vergleiche verfügbar.

---

## 10. Integrated Vehicle Motion Control

Die Regelungsarchitektur trennt strikt zwischen:

- Fahrer- beziehungsweise Trajektorienanforderung.
- Sollbewegungsberechnung.
- Vehicle Motion Controller.
- Control Allocation.
- Aktuatorregelung.
- Fahrzeugphysik.
- Zustandsschätzung und Rückführung.

### 10.1 Controller-Bibliothek

Unterstützt werden:

- Open-Loop-Baseline.
- PID.
- Giergeschwindigkeits-PID.
- LQR.
- Gain-Scheduled LQR.
- LTV-MPC.
- NMPC.
- C/GMRES ausschließlich bei tatsächlich implementierter Algorithmik.

Bestehende LTV-MPC- und NMPC-Implementierungen werden übernommen, sofern ihre mathematischen Eigenschaften, Schnittstellen und Solver korrekt dokumentiert sind.

PSO dient ausschließlich der Offline-Optimierung, etwa für Kostenfunktionsgewichte oder Reglerparameter.

### 10.2 Regler- und Streckenentkopplung

Das interne Vorhersagemodell des Reglers und das Simulationsstreckenmodell müssen unabhängig konfigurierbar sein.

Ein Controller darf keine versteckten Ground-Truth-Zugriffe auf den Simulationskern haben.

Ein idealisierter Full-State-Feedback-Modus bleibt als ausdrücklich gekennzeichnete Referenz zulässig.

### 10.3 Control Allocation

Der Allocator verteilt Soll-Längskraft, Soll-Giermoment und gegebenenfalls weitere Bewegungsanforderungen auf verfügbare Aktuatoren.

Zu berücksichtigen:

- Individuelle Reifen- und Radlastgrenzen.
- Brems- und Antriebsmomentgrenzen.
- Aktuatordynamik.
- Stellraten.
- Schlupfgrenzen.
- Prioritäten zwischen Fahrstabilität und Fahranforderungen.
- Machbarkeit und Restfehler der Sollanforderung.

Nicht erfüllbare Anforderungen müssen als solche sichtbar werden.

### 10.4 Robustheit

Controller werden nicht nur unter nominellen Parametern getestet.

Erforderlich sind Untersuchungen für veränderte Masse, Schwerpunktlage, Reifensteifigkeit, Reibwerte, Verzögerungen, Störgrößen und Stellgrenzen.

Solverversagen, Zeitüberschreitungen und unzulässige Optimierungsergebnisse müssen explizit behandelt werden.

---

## 11. Zustandsschätzung und virtuelle Sensorik

Das Projekt soll eine unabhängige Sensor- und Estimatorschicht erhalten.

Virtuelle Sensoren:

- Gierratensensor.
- Längs- und Querbeschleunigungssensor.
- Raddrehzahlsensoren.
- Lenkwinkelsensoren.
- Optional Positions- und Geschwindigkeitssensoren.

Sensorfehler sind konfigurierbar:

- Rauschen.
- Bias.
- Abtastrate.
- Verzögerung.
- Quantisierung.
- Signalausfall.

Estimatoren:

- Deterministische Beobachter.
- Lineare Kalman-Filter.
- Extended Kalman Filter.
- Optional Unscented Kalman Filter.

Besondere Untersuchungsschwerpunkte sind Schwimmwinkelschätzung, Geschwindigkeitsrekonstruktion und die Grenzen der Reibwertidentifikation.

Synthetische Messungen dürfen zur Funktionsprüfung von Estimatoren verwendet werden. Solche Selbsttests sind nicht als unabhängige experimentelle Validierung zu bezeichnen.

---

## 12. Straßen- und Umgebungsmodelle

Die Fahrbahn wird als eigenständiges Modell implementiert.

Das Modell muss Reibwerte an den jeweiligen Weltkoordinaten der einzelnen Radaufstandspunkte bereitstellen.

Zu unterstützen:

- Konstanter Reibwert.
- Trockene Fahrbahn.
- Nasse Fahrbahn.
- Niedrigreibwertbereiche.
- Split-µ.
- Räumlich variable Reibwertkarten.
- Übergänge zwischen Reibwertzonen.
- Zeitabhängige Änderungen.
- Optional Straßenquerneigung und Längsneigung.
- Später vertikale Straßenanregung.

Die bisherigen longitudinalen Reibwertabschnitte bleiben als einfacher Road-Adapter erhalten.

Eine zweidimensionale Reibwertkarte muss unterschiedliche Reibwerte unter allen vier Rädern ermöglichen.

---

## 13. Simulation Engine

Der Simulationskern ist deterministisch, headless ausführbar und unabhängig von grafischen Benutzeroberflächen.

### Numerische Integration

Erforderlich:

- Fixed-Step-Verfahren.
- RK4.
- Optionale adaptive Integration.
- Definierte Anfangsbedingungen.
- Saubere Behandlung von Modellereignissen.
- Numerische Fehlermeldungen und Statusinformationen.

### Multi-Rate-Simulation

Folgende Zeitskalen sind getrennt einstellbar:

- Physikalischer Integrationsschritt.
- Sensorabtastung.
- Estimator-Abtastung.
- Controller-Abtastung.
- Aktuatoraktualisierung.
- Logging und Visualisierung.

Abtastraten und Zero-Order-Hold-Verhalten müssen eindeutig definiert werden.

### Modellgrenzen

Jedes Modell muss seine Zustandsdimension, Eingänge, Parameter und unterstützten physikalischen Phänomene deklarieren.

Nicht unterstützte Funktionen müssen zu einem klaren Fehler oder einer dokumentierten Modellbeschränkung führen – niemals zu einer stillen Näherung unbekannter Bedeutung.

---

## 14. Simulationsexperimente und Szenariokatalog

Ein zentrales Experiment-Framework ermöglicht den automatisierten Vergleich von Fahrzeugmodellen, Reglern und Parametervariationen.

Pflichtszenarien:

| ID | Szenario | Untersuchung |
|---|---|---|
| S01 | Geradeausfahrt | Kräftegleichgewicht, Roll- und Luftwiderstand |
| S02 | Stationäre Kreisfahrt | Eigenlenkverhalten und Gierverstärkung |
| S03 | Lenkwinkelsprung | Transientes Gierverhalten |
| S04 | Sinusanregung | Frequenzgang und Dämpfung |
| S05 | Doppelte Spurwechselaufgabe | Spurhaltung und Regelung |
| S06 | Split-µ-Bremsung | ABS, ESC und Gierstabilität |
| S07 | Nasse Kurvenpassage | Reibwertwechsel und Seitenkraftgrenzen |
| S08 | Bremsen bei Kurvenfahrt | Combined Slip und Control Allocation |
| S09 | Beschleunigung bei niedrigem Reibwert | Antriebsschlupfregelung |
| S10 | 2WS gegen 4WS | Hinterachslenkung und Fahrzeugreaktion |
| S11 | Torque Vectoring | Giermomentenregelung |
| S12 | 4WS + Torque Vectoring | Integrierte Regelung |
| S13 | Unterschiedliche Beladungen | Modell- und Reglerrobustheit |
| S14 | Aktuatorausfall | Rückfallverhalten |
| S15 | Unebene Fahrbahn | Vertikale Dynamik und Radlastvariation |

Jedes Szenario erhält:

- YAML-Konfiguration.
- Anfangsbedingungen.
- Fahrzeug- und Reifenkonfiguration.
- Controllerkonfiguration.
- Fahrbahnmodell.
- Simulations- und Abtastzeiten.
- Definierte Eingangsmanöver.
- Erwartete physikalische Eigenschaften.
- Auswertungsgrößen und Abnahmekriterien.
- Konfigurierbare Ausgabeformate.

Szenariovergleiche müssen bei gleichen Ausgangsbedingungen und passend definierten Randbedingungen erfolgen.

---

## 15. Physics & Numerical Verification Framework

Da keine realen Messdaten verwendet werden, ist dieser Teil besonders wichtig.

### 15.1 Physikalische Invarianten

Automatisierte Prüfungen für:

- Kräfte- und Momentengleichgewichte.
- Symmetrisches Fahrzeugverhalten.
- Vorzeichen der Giermomente.
- Wheel-Torque-Balance.
- Reifen-Kraftgrenzen.
- Normalkraftbilanz.
- Kontaktverlust.
- Konsistente Radkoordinatentransformationen.
- Konsistente Leistungs- und Energiebilanzen.
- Einhaltung physikalischer Stellgrenzen.

### 15.2 Analytische Referenzfälle

Mindestens:

- Geradeausfahrt unter definierten Kräften.
- Kinematische Kreisfahrt.
- Stationäre lineare Einspurantwort.
- Einfacher Beschleunigungs- und Bremsvorgang.
- Lineares Feder-Masse-Dämpfer-System.
- Radrotation unter vorgegebenem Drehmoment.
- Lineare Systemantworten bei bekannten Eigenwerten.

### 15.3 Numerische Konvergenz

Ergebnisse werden bei mehreren Integrationsschrittweiten verglichen.

Die erwartete Konvergenzordnung wird dort geprüft, wo Glattheit und Modellformulierung dies rechtfertigen. Bei Diskontinuitäten, Sättigungen und Kontaktwechseln müssen angepasste Kriterien verwendet werden.

### 15.4 Modellübergreifende Konsistenz

Beispiele:

- Zweispurmodell gegen Einspurmodell bei symmetrischen Bedingungen.
- Nichtlineares Reifenmodell gegen lineare Näherung bei kleinem Schlupf.
- Ideales Aktuatormodell gegen dynamisches Modell mit verschwindender Verzögerung.
- Erweiterte Vertikaldynamik gegen vereinfachtes Modell in geeigneten Grenzfällen.

### 15.5 Robustheitstests

Systematische Variationen von Parametern, Startzuständen und Störungen.

Monte-Carlo-Experimente sind möglich, müssen aber auf fest definierten Verteilungen beziehungsweise Wertebereichen beruhen.

Ergebnisse werden mit Verteilungen, Worst-Case-Werten und Verletzungsraten berichtet, nicht nur mit Mittelwerten.

---

## 16. KPI-System und Engineering-Auswertung

Jede Simulation soll standardisierte Kennzahlen erzeugen.

### Handling

- Gierverstärkung.
- Untersteuergradient.
- Schwimmwinkel.
- Querbeschleunigung.
- Dämpfungsgrad.
- Einschwingzeit.
- Überschwingen.

### Regelung

- Tracking-RMSE.
- Maximale Regelabweichung.
- Stellgrößenbedarf.
- Stellratengrenzverletzungen.
- Solverlaufzeit.
- Solverfehler.
- Häufigkeit von Sättigungen.
- Nicht erfüllte Sollmomente und Sollkräfte.

### Bremsen und Reifen

- Bremsweg.
- Verzögerung.
- Radschlupf.
- Reifenutilisierung.
- Einzelradkräfte.
- ABS- und ESC-Eingriffe.

### Vertikaldynamik

- Aufbaubeschleunigungen.
- Wank- und Nickwinkel.
- Radlastvariation.
- Federwege.
- Dämpferkräfte.
- Radkontaktverlust.

Jede Kennzahl erhält eine dokumentierte mathematische Definition, Einheit und Interpretationsgrenze.

---

## 17. Visualisierung und Reporting

**Bestehende Live-Matplotlib-Plots müssen erhalten bleiben.**

Zusätzlich wird ein unabhängiges Offline-Visualisierungssystem entwickelt.

### Visualisierungen

- 2D-Fahrzeuganimationen.
- Referenz- und Ist-Trajektorien.
- Animierte Lenkwinkel.
- Einzelradkräfte.
- Reifen-Kraftpfeile.
- Reibwertzonen.
- Giergeschwindigkeit und Schwimmwinkel.
- Radlasten.
- Schlupfwerte.
- Torque-Vectoring-Momente.
- Controllervergleiche.
- Eigenwert- und Frequenzgangplots.
- Parameter-Sensitivitätsdiagramme.
- Robustheitsdiagramme.

### Ausgabeformate

Pflicht:

- CSV.
- PNG.
- GIF.
- YAML-Konfigurationssnapshot.
- Maschinenlesbare KPI-Datei.
- Markdown- oder HTML-Engineering-Report.

Optional:

- Parquet für größere Datenmengen.
- MP4-Export.
- Interaktives Dashboard.

Die Simulation muss unabhängig von der Generierung der Grafiken ausführbar bleiben.

---

## 18. Zielarchitektur und Repository-Struktur

Die Paketstruktur unter `src/vehicle_model/` ist mit Commit `041f48c` festgelegt und bleibt erhalten. Weitere Umzüge ganzer Verzeichnisse finden nicht statt; neue Subsysteme werden als zusätzliche Pakete innerhalb dieser Struktur angelegt.

Struktur (Bestand und geplante Ergänzungen):

| Pfad | Verantwortung | Status |
|---|---|---|
| `configs/vehicles/` | Virtuelle Referenzfahrzeuge | vorhanden |
| `configs/tyres/` | Reifenparametersätze | vorhanden |
| `configs/roads/` | Straßenmodelle | vorhanden |
| `configs/controllers/` | Reglerkonfigurationen | vorhanden |
| `configs/scenarios/` | Virtuelle Fahrmanöver | vorhanden |
| `configs/{braking,powertrains,steering,suspension,estimators,validation}/` | Subsystem- und Validierungskonfigurationen | vorhanden |
| `configs/experiments/` | Versuchs- und Vergleichsmatrizen | geplant |
| `src/vehicle_model/core/` | Gemeinsame Datentypen und Modellverträge | vorhanden |
| `src/vehicle_model/config/` | Konfigurationsladen und -validierung | vorhanden |
| `src/vehicle_model/vehicle_models/` | Fahrzeugphysik | vorhanden |
| `src/vehicle_model/tire_model/` | Reifenphysik | vorhanden |
| `src/vehicle_model/steering/` | Lenkung | vorhanden |
| `src/vehicle_model/suspension/` | Fahrwerk und K&C | vorhanden |
| `src/vehicle_model/actuators/` | Lenk-, Brems- und Antriebsaktuatoren (Antrieb und Bremsen) | vorhanden |
| `src/vehicle_model/controllers/` | Regler | vorhanden |
| `src/vehicle_model/allocation/` | Stellgrößenverteilung | vorhanden (M5) |
| `src/vehicle_model/estimation/` | Zustandsbeobachter und virtuelle Sensoren | vorhanden |
| `src/vehicle_model/roads/` | Fahrbahn | vorhanden |
| `src/vehicle_model/simulation/` | Simulation Engine und Szenarien | vorhanden |
| `src/vehicle_model/handling_analysis/` | Eigenlenk- und Stabilitätsanalyse | vorhanden (M1) |
| `src/vehicle_model/verification/` | Physik- und Numerikprüfung | geplant |
| `src/vehicle_model/experiments/` | Experimentautomatisierung | geplant |
| `src/vehicle_model/visualization/` | Live- und Offline-Visualisierung | vorhanden |
| `examples/` | Demonstratoren | vorhanden |
| `tests/` | Automatisierte Tests | vorhanden |
| `docs/` | Technische Dokumentation | vorhanden |
| `output/` | Reproduzierbare Simulationsergebnisse (nicht versioniert) | vorhanden |

Die früheren Verzeichnisse `python/cgmres_nmpc/`, `python/vehicle_model/` und `python/vehicle_visualization/` sind nach `src/vehicle_model/` migriert. Der MATLAB-Code wurde bewusst entfernt und ist über die Git-Historie (Commit `4190954`) weiterhin erreichbar.

Nachgewiesene Duplikate werden schrittweise konsolidiert. Bestehende öffentliche APIs sind durch Kompatibilitätsadapter zu erhalten, bis eine dokumentierte Migration abgeschlossen ist.

### Schnittstellenregeln

- Physikmodelle dürfen keine Plotting- oder CLI-Abhängigkeiten haben.
- Controller erhalten Zustandsinformationen nur über definierte Schnittstellen.
- Simulation Engine und Modelle kommunizieren über typisierte Datenverträge.
- Die Konfigurationsschicht validiert Eingaben vor Simulationsbeginn.
- Visualisierung verwendet persistierte oder übergebene Simulationsresultate.
- Modell- und Reglerimplementierungen dürfen nicht von einer konkreten GUI abhängen.

---

# 19. Verbindliche Implementierungsroadmap

Die Meilensteine bauen aufeinander auf. Ein Meilenstein gilt erst nach ausgeführter Verifikation und dokumentiertem Abnahmeergebnis als abgeschlossen.

## M0 – Repository Audit & Stabilization

**Aufgaben**

- Bestehende Implementierungen prüfen.
- Tests und Demos ausführen.
- Architektur und Abhängigkeiten dokumentieren.
- Lokale Chassis-Erweiterungen inventarisieren.
- Modellkonventionen vereinheitlichen.
- Fehlerhafte Algorithmusbezeichnungen identifizieren.
- Bestehende Funktionen durch Regressionstests absichern.

**Ergebnisse**

- Baseline-Audit.
- Testreport.
- Architekturübersicht.
- Liste technischer Schulden.
- Priorisierter Migrationsplan.

**Abnahme:** Der tatsächliche Zustand des Repositories ist nachvollziehbar; funktionierende Bestandteile können reproduzierbar ausgeführt werden.

## M1 – Virtual Reference Vehicle & Handling Fundamentals

**Aufgaben**

- Virtuelles Referenzfahrzeug anlegen.
- Parameter und deren Herkunft dokumentieren.
- Analytisches lineares Einspurmodell bereitstellen.
- Kinematisches und dynamisches Einspurmodell vereinheitlichen.
- Eigenlenk- und Untersteueranalyse entwickeln.
- Stationäre Kreisfahrt implementieren.
- Eigenwert- und Frequenzganganalyse ergänzen.
- Vorhandene Chassis-Funktionen kontrolliert integrieren, soweit verfügbar.

**Abnahme:** Fahrzeugparameter sind konsistent und die definierten analytischen Referenzfälle werden innerhalb festgelegter numerischer Toleranzen erfüllt.

## M2 – Nonlinear Four-Wheel Vehicle Dynamics

**Aufgaben**

- Nichtlineares Zweispurmodell entwickeln.
- Einzelraddrehdynamik integrieren.
- Individualisierte Antriebs- und Bremsmomente.
- Longitudinal- und Lateralschlupf.
- Individuelle Reifenkräfte.
- Radaufstandspunkt-Koordinaten.
- Quasistatische Radlastverlagerung.
- Erste variable Reibwertzonen.

**Abnahme:** Kräfte-, Momenten- und Drehmomentbilanzen sind konsistent; symmetrische Grenzfälle und definierte Konvergenztests bestehen.

## M3 – Advanced Tyre Physics

**Aufgaben**

- Austauschbare Reifenmodell-Bibliothek.
- Fiala und Magic Formula.
- Combined-Slip-Mechanismen.
- Last- und Reibwertempfindlichkeit.
- Transiente Reifenrelaxation.
- Reifenrückstellmoment bei unterstützter Modellformulierung.
- Reifenkennlinien und Kraftgrenzendarstellungen.

**Abnahme:** Grenzfälle, Sättigung, Kraftrichtungen und transientes Verhalten sind nachweisbar konsistent; Modelle lassen sich ohne Änderung des Fahrzeugkerns austauschen.

## M4 – Steering, Suspension Kinematics & Load Transfer

**Aufgaben**

- Ackermann-Geometrie.
- Einzelradlenkwinkel.
- 4WS-Strategien.
- Synthetische K&C-Kennfelder.
- Bump Steer und Compliance Steer.
- Rollzentrum und Wanksteifigkeitsverteilung.
- Erweiterte Radlastverlagerung.
- Analyse des Einflusses der Fahrwerksparameter auf das Eigenlenkverhalten.

**Abnahme:** Lenkgeometrie und Lastverteilung erfüllen definierte geometrische und physikalische Prüfungen; deaktivierte Zusatzeffekte reproduzieren das Basismodell.

## M5 – Actuators, ABS, TCS, ESC & Torque Vectoring

**Aufgaben**

- Dynamische Lenkungs-, Antriebs- und Bremsaktuatoren.
- Einzelradmomentregelung.
- ABS.
- Antriebsschlupfregelung.
- ESC.
- Torque-Vectoring-Controller.
- Beschränkte Control Allocation.
- Priorisierung von Fahrstabilität und Stellanforderungen.
- Fehler- und Sättigungsverhalten.

**Abnahme:** Split-µ-Bremsung, Schlupfregelung, Torque Vectoring und Aktuatorfehler sind reproduzierbar simulierbar; unzulässige Stellanforderungen werden erkannt.

## M6 – Advanced Vehicle Motion Control

**Aufgaben**

- Einheitliches Controller-Interface.
- PID- und LQR-Referenzen.
- Gain Scheduling.
- LTV-MPC.
- NMPC.
- Unabhängige Regler- und Streckenmodelle.
- Constraint Handling.
- Warm Start.
- Solverdiagnostik.
- Degraded-Mode-Verhalten.
- Integrierte 4WS-/Torque-Vectoring-Regelung.

**Abnahme:** Unterschiedliche Regler können mit derselben virtuellen Strecke, identischen Szenarien und definierten Kennzahlen verglichen werden.

## M7 – Virtual Sensors, Estimation & Robustness

**Aufgaben**

- Virtuelle Sensorschnittstellen.
- Rauschen, Bias und Verzögerungen.
- Zustandsbeobachter.
- Schwimmwinkelschätzung.
- Optional Reibwertschätzung.
- Systematische Parametervariation.
- Monte-Carlo-Studien.
- Offline-PSO-Optimierung.
- Robustheits- und Fehlerszenarien.

**Abnahme:** Beobachter besitzen keinen unerlaubten Ground-Truth-Zugriff; Robustheitsversuche sind wiederholbar und dokumentieren ihre Streuungen sowie Grenzverletzungen.

## M8 – Vertical Vehicle Dynamics & Advanced Chassis Systems

**Aufgaben**

- Quarter-Car.
- Half-Car nach Bedarf.
- Heben, Wanken und Nicken.
- Gefederte und ungefederte Massen.
- Passive Dämpfung.
- Skyhook und Groundhook.
- Semiaktive Dämpfung.
- Aktive Wankstabilisierung.
- Optional aktive Federung.
- Kopplung mit der planaren Fahrdynamik.

**Abnahme:** Freie Schwingungen, Energieverhalten, Frequenzgänge und definierte Straßenanregungen werden korrekt berechnet.

## M9 – Virtual Engineering & Research Laboratory

**Aufgaben**

- Einheitlicher CLI-Szenariorunner.
- Batch- und Parameter-Sweep-Ausführung.
- Automatisierte Modell- und Reglervergleiche.
- Automatisierte KPI-Berechnung.
- Physik- und Numerikverifikation.
- Regressionsprüfung.
- CSV-/PNG-/GIF-Ausgabe.
- Automatische Engineering-Reports.
- GitHub-Actions-CI.
- Vollständige technische Dokumentation.

**Abnahme:** Ein Anwender kann über YAML ein Fahrzeug, Reifenmodell, Fahrmanöver, Regler und Fahrbahnmodell auswählen, die Simulation reproduzierbar ausführen und anschließend ein vollständiges Vergleichs- und Verifikationsergebnis erhalten.

---

## 20. Qualitätssicherung

Für jeden Meilenstein müssen folgende Dimensionen geprüft werden:

- Physikalische Korrektheit.
- Mathematische Konsistenz.
- Numerische Stabilität.
- Architektur und Modularität.
- Testabdeckung.
- Robustheit.
- Reproduzierbarkeit.
- Performance.
- Dokumentation.
- Nutzbarkeit.
- Visualisierung.

Jede Dimension wird anhand definierter, nachprüfbarer Kriterien bewertet.

Ein Qualitätsscore von 10/10 darf nur vergeben werden, wenn sämtliche vorab vereinbarten Kriterien nachweislich erfüllt wurden.

Fehlende experimentelle Messdaten führen nicht automatisch zum Scheitern eines Meilensteins, da experimentelle Validierung nicht zum definierten Projektumfang gehört.

Ebenso darf ein rein virtuelles Ergebnis nicht als experimentell validiert bezeichnet werden.

---

## 21. Technische Entwicklungsvorgaben

### Python und Numerik

Bevorzugte Werkzeuge:

- Python.
- NumPy.
- SciPy.
- PyYAML.
- Pytest.
- Matplotlib.
- Ruff.
- Typisierte Python-Schnittstellen.

Neue größere Abhängigkeiten sind nur mit dokumentierter technischer Begründung aufzunehmen.

### Konfiguration

- YAML ist die primäre Konfigurationsquelle.
- Physikalische Konstanten und Modellparameter werden zentral verwaltet.
- Jede Konfiguration wird vor der Ausführung validiert.
- Vollständig aufgelöste Konfigurationen werden mit dem Versuch gespeichert.
- Keine versteckten physikalischen Standardwerte.
- Rückwärtskompatibilität wird dokumentiert.

### Git-Workflow

- Bestehende Änderungen erhalten.
- Kleine, fachlich zusammenhängende Änderungen.
- Tests nach jeder relevanten Änderung.
- Keine destruktiven Git-Kommandos zur Bereinigung.
- Keine automatischen Pushes ohne ausdrückliche Freigabe.

### Dokumentation

Jedes relevante Subsystem erhält:

- Physikalische Beschreibung.
- Gleichungen.
- Parameterdefinitionen.
- Konfigurationsbeispiel.
- Gültigkeitsgrenzen.
- Verifikationsstrategie.
- Ausführbares Demonstrationsbeispiel.
- Bekannte Einschränkungen.

---

## 22. Unmittelbarer Arbeitsauftrag an Gemini, Codex oder Claude

Rolle: Principal Vehicle Dynamics Engineer, Control Systems Researcher, Numerical Simulation Engineer und Senior Python Software Architect.

Ziel: Den vorliegenden Masterplan schrittweise im bestehenden Vehicle-Dynamics-Repository umsetzen.

Verbindliches Vorgehen:

1. Repository und Git-Zustand vollständig untersuchen.
2. Vorhandene Tests und Simulationen ausführen.
3. Den tatsächlichen Implementierungsstand gegen diesen Masterplan abgleichen.
4. Bereits vorhandene Funktionalität nicht neu und parallel implementieren.
5. Architekturprobleme und physikalische Inkonsistenzen priorisieren.
6. M0 abschließen und die Ergebnisse dokumentieren.
7. Anschließend M1 mit dem virtuellen Referenzfahrzeug und den fahrdynamischen Grundanalysen umsetzen.
8. Neue physikalische Modelle immer zusammen mit mathematischen Referenztests und numerischen Konsistenzprüfungen entwickeln.
9. Bestehende Live-Visualisierungen unverändert verfügbar halten.
10. Schrittweise durch die nachfolgenden Meilensteine fortfahren, sofern ihre Voraussetzungen erfüllt sind.

Nach jedem Meilenstein sind bereitzustellen:

- Implementierungsbericht.
- Architekturänderungen.
- Mathematische Modellbeschreibung.
- Verifikationsbericht.
- Ausgeführte Testkommandos und Ergebnisse.
- Simulationsbeispiele.
- Generierte Visualisierungen.
- Bekannte Grenzen.
- Nächste technisch begründete Arbeitsschritte.

**Abschlussbedingung:** Nicht die bloße Existenz von Dateien oder Klassen, sondern funktionierende Integration, ausgeführte Tests, physikalische Konsistenz und reproduzierbare Simulationsergebnisse.

Der zentrale Entwicklungsgrundsatz lautet:

**Build a physically consistent, modular and reproducible virtual vehicle dynamics laboratory. Prioritize mathematical correctness, engineering traceability, controller robustness and numerical verification over feature count or visual complexity.**