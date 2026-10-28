# Literatur- und Quellenverzeichnis (References & Bibliography)

Dieses Dokument bietet eine strukturierte Übersicht über die wissenschaftlichen Grundlagen, Monografien, Lehrbücher und Fachaufsätze, auf denen die mathematische Modellbildung, Reifenmechanik, Fahrwerktechnik und Fahrdynamikregelung dieses Projekts aufbauen.

---

## 1. Standardwerke der Fahrzeugtechnik & Fahrdynamik

### [1] Küçükay, Ferit
- **Titel:** *Grundlagen der Fahrzeugtechnik: Antriebe, Getriebe, Energieverbrauch, Bremsen, Fahrdynamik, Fahrkomfort*
- **Verlag:** Springer Vieweg, Springer Fachmedien Wiesbaden, 2022.
- **ISBN:** 978-3-658-36066-5 / 978-3-658-36067-2 (eBook).
- **Inhaltlicher Bezug im Projekt:**
  - Grundlegende Modellierung der Längs-, Quer- und Vertikaldynamik.
  - Dynamische Radlastverlagerung unter Längs- und Querbeschleunigung (Kap. Bremsen & Fahrdynamik).
  - Komfortkriterien und Vertikalschwingungen von Fahrbahn-Fahrzeug-Systemen (Kap. Fahrkomfort).
  - Stationäre Kreisfahrt und Untersteuergradient (Kap. 6: Querdynamik und Eigenlenkverhalten).
  - Verwendet in: [VEHICLE_MODELS.md](VEHICLE_MODELS.md), [SUSPENSION_DYNAMICS.md](SUSPENSION_DYNAMICS.md), [HANDLING_LIMITS.md](HANDLING_LIMITS.md).

### [2] Mitschke, Manfred; Wallentowitz, Henning
- **Titel:** *Dynamik der Kraftfahrzeuge*
- **Reihe:** VDI-Buch
- **Verlag:** Springer Vieweg, Springer Fachmedien Wiesbaden, 5. überarbeitete und ergänzte Auflage, 2014.
- **ISBN:** 978-3-658-05067-2 / 978-3-658-05068-9 (eBook).
- **Inhaltlicher Bezug im Projekt:**
  - Das klassische Standardwerk für Einspur- und Zweispurmodelle (lineares und nichtlineares Fahrverhalten).
  - Wankdynamik und Aufteilung des Wankmoments auf Vorder- und Hinterachse ($k_{\text{roll},f}$).
  - Torque Vectoring, Fahrgrenzen und Übersteuer-/Untersteuer-Charakteristik nach ISO 8855.
  - Verwendet in: [VEHICLE_MODELS.md](VEHICLE_MODELS.md), [CHASSIS_CONTROL.md](CHASSIS_CONTROL.md), [HANDLING_LIMITS.md](HANDLING_LIMITS.md).

### [3] Heißing, Bernd; Ersoy, Metin; Gies, Stefan (Hrsg.)
- **Titel:** *Fahrwerkhandbuch: Grundlagen, Fahrdynamik, Komponenten, Systeme, Mechatronik, Perspektiven*
- **Reihe:** ATZ/MTZ-Fachbuch
- **Verlag:** Springer Vieweg, Springer Fachmedien Wiesbaden, 5. Auflage, 2020.
- **ISBN:** 978-3-658-27806-9.
- **Inhaltlicher Bezug im Projekt:**
  - Kinematik und Elastokinematik von Radaufhängungen.
  - Allradlenkung (4WS: Allradlenksysteme, geschwindigkeitsabhängige Gleichphasen- und Gegenphasenlenkung).
  - Mechatronische Fahrwerksysteme, elektromechanische Lenkaktuatoren (EPS) und aktive Stabilisatoren.
  - Verwendet in: [ACTUATORS.md](ACTUATORS.md), [CHASSIS_CONTROL.md](CHASSIS_CONTROL.md).

### [4] Schramm, Dieter; Hiller, Manfred; Bardini, Roberto
- **Titel:** *Modellbildung und Simulation der Dynamik von Kraftfahrzeugen*
- **Verlag:** Springer Vieweg, Berlin / Heidelberg, 4. Auflage, 2020.
- **ISBN:** 978-3-662-61348-1.
- **Inhaltlicher Bezug im Projekt:**
  - Mehrkörperdynamik (MKS) räumlicher Fahrwerk- und Gesamtfahrzeugmodelle (14-DOF Vollfahrzeug).
  - Kopplung von Aufbaufreiheitsgraden (Hub, Wank, Nick) mit Radbewegungen und Reifenkontakt.
  - Verwendet in: [VEHICLE_MODELS.md](VEHICLE_MODELS.md), [SUSPENSION_DYNAMICS.md](SUSPENSION_DYNAMICS.md).

### [5] Gillespie, Thomas D.
- **Titel:** *Fundamentals of Vehicle Dynamics*
- **Verlag:** Society of Automotive Engineers (SAE International), Warrendale, PA, 1992.
- **ISBN:** 978-1-56091-199-9.
- **Inhaltlicher Bezug im Projekt:**
  - Internationaler Standard für Normalenlasttransfer, Rollzentren, Nickpole und stationäres Eigenlenkverhalten.
  - Verwendet in: [VEHICLE_MODELS.md](VEHICLE_MODELS.md), [HANDLING_LIMITS.md](HANDLING_LIMITS.md).

---

## 2. Reifenmechanik & Kraftschlussübertragung

### [6] Pacejka, Hans B.
- **Titel:** *Tire and Vehicle Dynamics*
- **Verlag:** Elsevier / Butterworth-Heinemann, Oxford, 3rd Edition, 2012.
- **ISBN:** 978-0-08-097016-5.
- **Inhaltlicher Bezug im Projekt:**
  - Das fundamentale Standardwerk zur Reifenmodellierung (Magic Formula MF 6.2).
  - Struktur- und bürstenmechanische Betrachtung von Reifenschlupf ($\kappa$) und Schräglaufwinkel ($\alpha$).
  - Kombinierter Kraftschluss (Kammscher Reibkreis / Reibellipsen-Kopplung) und Rückstellmoment ($M_z$).
  - Verwendet in: [TYRE_MODELS.md](TYRE_MODELS.md).

### [7] Fiala, Ernst
- **Titel:** *Seitenkräfte am rollenden Luftreifen*
- **Zeitschrift:** Zeitschrift des Vereins Deutscher Ingenieure (VDI), Band 96, Nr. 29, S. 973–979, 1954.
- **Inhaltlicher Bezug im Projekt:**
  - Analytisches Bürstenmodell mit parabolischer Druckverteilung im Reifenlatsch.
  - Exakte analytische Trennung von elastischer Haft- und Gleitzone (`FialaTyre`).
  - Verwendet in: [TYRE_MODELS.md](TYRE_MODELS.md).

---

## 3. Regelungstechnik, Zustandsschätzung & Fahrwerk-Mechatronik

### [8] Isermann, Rolf (Hrsg.)
- **Titel:** *Fahrdynamik-Regelung: Modellbildung, Fahrerassistenzsysteme, Mechatronik*
- **Verlag:** Vieweg+Teubner Verlag / Springer Fachmedien Wiesbaden, 2010.
- **ISBN:** 978-3-8348-0314-6.
- **Inhaltlicher Bezug im Projekt:**
  - Modellgestützter Reglerentwurf für mechatronische Fahrwerksysteme (ABS, ASR, ESP/ESC).
  - Rekursive Schätzverfahren (RLS) zur Reibwertschätzung ($\mu$) und Zustandsschätzer.
  - Verwendet in: [CHASSIS_CONTROL.md](CHASSIS_CONTROL.md), [STATE_ESTIMATION.md](STATE_ESTIMATION.md).

### [9] Rajamani, Rajesh
- **Titel:** *Vehicle Dynamics and Control*
- **Reihe:** Mechanical Engineering Series
- **Verlag:** Springer, New York, 2nd Edition, 2012.
- **ISBN:** 978-1-4614-1432-2.
- **Inhaltlicher Bezug im Projekt:**
  - Phasenflächen-Stabilitätsanalyse im $(\beta, \dot{\beta})$-Zustandsraum für nichtlineares Schleuder- und Driftverhalten.
  - Extended Kalman Filter (EKF) zur Schätzung des Schwimmwinkels und Sensor-Bias-Kompensation.
  - Differentialbremsen zur Giermomentenerzeugung (Direct Yaw Moment Control / DYC) und Kippvermeidung ($LTR$).
  - Verwendet in: [CHASSIS_CONTROL.md](CHASSIS_CONTROL.md), [STATE_ESTIMATION.md](STATE_ESTIMATION.md).

### [10] Lunze, Jan
- **Titel:** *Regelungstechnik 1: Systemtheoretische Grundlagen, Analyse und Entwurf einschleifiger Regelungen* (12. Aufl., 2020) & *Regelungstechnik 2: Mehrgrößensysteme, Digitale Regelung* (10. Aufl., 2020)
- **Verlag:** Springer Vieweg, Berlin / Heidelberg.
- **ISBN:** 978-3-662-60745-9 / 978-3-662-60747-3.
- **Inhaltlicher Bezug im Projekt:**
  - Zustandsraumdarstellung, Steuerbarkeit, Beobachtbarkeit, Linear-Quadratische Regler (LQR).
  - Ljapunow-Stabilitätstheorie und robuste Regelung.
  - Verwendet in: [CHASSIS_CONTROL.md](CHASSIS_CONTROL.md), [ARCHITECTURE.md](ARCHITECTURE.md).

---

## 4. Modellprädiktive Regelung (NMPC)

### [11] Ohtsuka, Toshiyuki
- **Titel:** *A Continuation/GMRES Method for Fast Computation of Nonlinear Model Predictive Control*
- **Zeitschrift:** Automatica, Vol. 40, No. 4, S. 563–574, 2004.
- **DOI:** [10.1016/j.automatica.2003.11.005](https://doi.org/10.1016/j.automatica.2003.11.005)
- **Inhaltlicher Bezug im Projekt:**
  - Mathematische Grundlage für den Echtzeit-C/GMRES-Algorithmus zur Lösung der Karush-Kuhn-Tucker-Bedingungen.
  - Verwendet in: [NMPC_CONTROL.md](NMPC_CONTROL.md), [cgmres_nmpc.md](cgmres_nmpc.md).

### [12] Borrelli, Francesco; Bemporad, Alberto; Frazzoli, Emilio
- **Titel:** *Predictive Control for Automotive Applications*
- **Verlag:** Springer, London, 2014.
- **Inhaltlicher Bezug im Projekt:**
  - Trajektorienfolgeregelung, Hindernisausweichoptimierung und Aktuatorkopplung im prädiktiven Zeithorizont.
  - Verwendet in: [NMPC_CONTROL.md](NMPC_CONTROL.md).

---

## 5. Fahrermodelle & Fahrkomfort

### [13] MacAdam, Charles C.
- **Titel:** *Application of an Optimal Preview Control for Simulation of Closed-Loop Automobile Driving*
- **Zeitschrift:** IEEE Transactions on Systems, Man, and Cybernetics, Vol. 11, No. 6, S. 393–399, 1981.
- **Inhaltlicher Bezug im Projekt:**
  - Optimal-Vorausschau-Fahrermodell (Preview Horizon $T_p$, Vorausschaufehler).
  - Verwendet in: [DRIVER_MODEL.md](DRIVER_MODEL.md).

### [14] Salvucci, Dario D.; Gray, Richard
- **Titel:** *A Two-Point Visual Control Model of Steering*
- **Zeitschrift:** ACM Transactions on Applied Perception, Vol. 1, No. 2, S. 139–148, 2004.
- **Inhaltlicher Bezug im Projekt:**
  - Kognitives Zwei-Punkt-Fahrermodell (Fernpunkt zur Krümmungsantizipation, Nahpunkt zur Spurhaltung).
  - Verwendet in: [DRIVER_MODEL.md](DRIVER_MODEL.md).

### [15] Karnopp, Dean
- **Titel:** *Vibration Control Using Semi-Active Force Generators*
- **Zeitschrift:** ASME Journal of Engineering for Industry, Vol. 96, No. 2, S. 619–626, 1974.
- **Inhaltlicher Bezug im Projekt:**
  - Theoretische Herleitung und Realisierungsbedingung der Skyhook-Dämpfungsregel für semi-aktive Fahrwerke.
  - Verwendet in: [SUSPENSION_DYNAMICS.md](SUSPENSION_DYNAMICS.md).

---

## 6. Internationale Prüfnormen & Standards

- **ISO 8855:2011:** *Road vehicles — Vehicle dynamics and road-holding ability — Vocabulary and coordinate systems.* (Definiert das Zwischenachsen-Fahrzeugkoordinatensystem $X, Y, Z$).
- **ISO 4138:2012:** *Passenger cars — Steady-state circular driving behaviour — Open-loop test methods.* (Stationäre Kreisfahrt, Untersteuergradient $K_{\text{us}}$, charakteristische Geschwindigkeit $v_{\text{char}}$).
- **ISO 3888-2:2011:** *Passenger cars — Test track for a severe lane-change manoeuvre — Part 2: Obstacle avoidance (Elchtest / Double Lane Change).*
- **ISO 7401:2011:** *Road vehicles — Lateral transient response test methods — Open-loop test methods (Step Steer).*
- **ISO 7975:2019:** *Passenger cars — Braking in a turn — Open-loop test method.*
- **ISO 2631-1:1997:** *Mechanical vibration and shock — Evaluation of human exposure to whole-body vibration — Part 1: General requirements.*
- **FMVSS 126 / ECE-R13H:** *Federal Motor Vehicle Safety Standard No. 126: Electronic Stability Control Systems for Light Vehicles (NHTSA Sine with Dwell Test).*
- **NHTSA NCAP Dynamic Fishhook:** *National Highway Traffic Safety Administration: Dynamic Roll Stability Test Procedure (80 km/h Lenkwinkelsprung-Umkehr).*
