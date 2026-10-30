# Literatur- und Quellenverzeichnis (References & Bibliography)

Dieses Dokument bietet eine strukturierte Übersicht über die wissenschaftlichen Grundlagen, Monografien, Lehrbücher und Fachaufsätze, auf denen die mathematische Modellbildung, Reifenmechanik, Fahrwerktechnik und Fahrdynamikregelung dieses Projekts aufbauen.

!!! note "Prüfstand der Angaben"
    Alle DOI und ISBN wurden gegen [Crossref](https://www.crossref.org/) geprüft (Stand 30.10.2026); dabei wurden falsche
    ISBN (Küçükay, Heißing, Schramm, Isermann, Lunze) und zwei falsche Quellen (Borrelli, Salvucci) korrigiert. Fiala (1954)
    ist älter als das DOI-System und nicht so prüfbar. Die Kapitelangaben zu Küçükay folgen den bei Crossref hinterlegten
    Kapiteltiteln; der Buchinhalt selbst lag nicht vor, Seitenzitate werden daher nicht angegeben.

---

## 1. Standardwerke der Fahrzeugtechnik & Fahrdynamik

### [1] Küçükay, Ferit
- **Titel:** *Grundlagen der Fahrzeugtechnik: Antriebe, Getriebe, Energieverbrauch, Bremsen, Fahrdynamik, Fahrkomfort*
- **Verlag:** Springer Vieweg, Springer Fachmedien Wiesbaden, 2022.
- **ISBN:** 978-3-658-36726-8 / 978-3-658-36727-5 (eBook).
- **DOI:** [10.1007/978-3-658-36727-5](https://doi.org/10.1007/978-3-658-36727-5)
- **Inhaltlicher Bezug im Projekt:**
  - Grundlegende Modellierung der Längs-, Quer- und Vertikaldynamik.
  - Rad- und Achskräfte, Kraftschlussbeanspruchung (Kap. 5), Abbremsung und Bremssysteme (Kap. 18), Wanken und lateraler Radlasttransfer (Kap. 25).
  - Fahrbahnanregung (Kap. 32), Modelle zur Vertikaldynamik (Kap. 33), Sitzkomfort (Kap. 34), Aufbaufedern und -dämpfer (Kap. 35).
  - Eigenlenkverhalten (Kap. 22) und stationäre Kreisfahrt (Kap. 27).
  - Verwendet in: [02_vehicle_models.md](02_vehicle_models.md), [09_suspension_dynamics.md](09_suspension_dynamics.md), [07_handling_limits.md](07_handling_limits.md).

### [2] Mitschke, Manfred; Wallentowitz, Henning
- **Titel:** *Dynamik der Kraftfahrzeuge*
- **Reihe:** VDI-Buch
- **Verlag:** Springer Vieweg, Springer Fachmedien Wiesbaden, 5. überarbeitete und ergänzte Auflage, 2014.
- **ISBN:** 978-3-658-05067-2 / 978-3-658-05068-9 (eBook).
- **DOI:** [10.1007/978-3-658-05068-9](https://doi.org/10.1007/978-3-658-05068-9)
- **Inhaltlicher Bezug im Projekt:**
  - Das klassische Standardwerk für Einspur- und Zweispurmodelle (lineares und nichtlineares Fahrverhalten).
  - Wankdynamik und Aufteilung des Wankmoments auf Vorder- und Hinterachse ($k_{\text{roll},f}$).
  - Torque Vectoring, Fahrgrenzen und Übersteuer-/Untersteuer-Charakteristik nach ISO 8855.
  - Verwendet in: [02_vehicle_models.md](02_vehicle_models.md), [12_chassis_control.md](12_chassis_control.md), [07_handling_limits.md](07_handling_limits.md).

### [3] Heißing, Bernd; Ersoy, Metin; Gies, Stefan (Hrsg.)
- **Titel:** *Fahrwerkhandbuch: Grundlagen, Fahrdynamik, Komponenten, Systeme, Mechatronik, Perspektiven*
- **Reihe:** ATZ/MTZ-Fachbuch
- **Verlag:** Springer Vieweg, Wiesbaden, 2013.
- **ISBN:** 978-3-658-01991-4 / 978-3-658-01992-1 (eBook).
- **DOI:** [10.1007/978-3-658-01992-1](https://doi.org/10.1007/978-3-658-01992-1)
- **Inhaltlicher Bezug im Projekt:**
  - Kinematik und Elastokinematik von Radaufhängungen.
  - Allradlenkung (4WS: Allradlenksysteme, geschwindigkeitsabhängige Gleichphasen- und Gegenphasenlenkung).
  - Mechatronische Fahrwerksysteme, elektromechanische Lenkaktuatoren (EPS) und aktive Stabilisatoren.
  - Verwendet in: [05_actuators.md](05_actuators.md), [12_chassis_control.md](12_chassis_control.md).

### [4] Schramm, Dieter; Hiller, Manfred; Bardini, Roberto
- **Titel:** *Modellbildung und Simulation der Dynamik von Kraftfahrzeugen*
- **Verlag:** Springer Vieweg, Berlin / Heidelberg, 2018.
- **ISBN:** 978-3-662-54480-8 / 978-3-662-54481-5 (eBook).
- **DOI:** [10.1007/978-3-662-54481-5](https://doi.org/10.1007/978-3-662-54481-5)
- **Inhaltlicher Bezug im Projekt:**
  - Mehrkörperdynamik (MKS) räumlicher Fahrwerk- und Gesamtfahrzeugmodelle (14-DOF Vollfahrzeug).
  - Kopplung von Aufbaufreiheitsgraden (Hub, Wank, Nick) mit Radbewegungen und Reifenkontakt.
  - Verwendet in: [02_vehicle_models.md](02_vehicle_models.md), [09_suspension_dynamics.md](09_suspension_dynamics.md).

### [5] Gillespie, Thomas D.
- **Titel:** *Fundamentals of Vehicle Dynamics*
- **Verlag:** Society of Automotive Engineers (SAE International), Warrendale, PA, 1992.
- **ISBN:** 978-1-56091-199-9.
- **DOI:** [10.4271/r-114](https://doi.org/10.4271/r-114)
- **Inhaltlicher Bezug im Projekt:**
  - Internationaler Standard für Normalenlasttransfer, Rollzentren, Nickpole und stationäres Eigenlenkverhalten.
  - Verwendet in: [02_vehicle_models.md](02_vehicle_models.md), [07_handling_limits.md](07_handling_limits.md).

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
  - Verwendet in: [03_tyre_models.md](03_tyre_models.md).

### [7] Fiala, Ernst
- **Titel:** *Seitenkräfte am rollenden Luftreifen*
- **Zeitschrift:** Zeitschrift des Vereins Deutscher Ingenieure (VDI), Band 96, Nr. 29, S. 973–979, 1954.
- **Inhaltlicher Bezug im Projekt:**
  - Analytisches Bürstenmodell mit parabolischer Druckverteilung im Reifenlatsch.
  - Exakte analytische Trennung von elastischer Haft- und Gleitzone (`FialaTyre`).
  - Verwendet in: [03_tyre_models.md](03_tyre_models.md).

---

## 3. Regelungstechnik, Zustandsschätzung & Fahrwerk-Mechatronik

### [8] Isermann, Rolf (Hrsg.)
- **Titel:** *Fahrdynamik-Regelung: Modellbildung, Fahrerassistenzsysteme, Mechatronik*
- **Verlag:** Vieweg, Wiesbaden, 2006.
- **ISBN:** 978-3-8348-0109-8 / 978-3-8348-9049-8 (eBook).
- **DOI:** [10.1007/978-3-8348-9049-8](https://doi.org/10.1007/978-3-8348-9049-8)
- **Inhaltlicher Bezug im Projekt:**
  - Modellgestützter Reglerentwurf für mechatronische Fahrwerksysteme (ABS, ASR, ESP/ESC).
  - Rekursive Schätzverfahren (RLS) zur Reibwertschätzung ($\mu$) und Zustandsschätzer.
  - Verwendet in: [12_chassis_control.md](12_chassis_control.md), [18_state_estimation.md](18_state_estimation.md).

### [9] Rajamani, Rajesh
- **Titel:** *Vehicle Dynamics and Control*
- **Reihe:** Mechanical Engineering Series
- **Verlag:** Springer, New York, 2nd Edition, 2012.
- **ISBN:** 978-1-4614-1432-2 / 978-1-4614-1433-9 (eBook).
- **DOI:** [10.1007/978-1-4614-1433-9](https://doi.org/10.1007/978-1-4614-1433-9)
- **Inhaltlicher Bezug im Projekt:**
  - Phasenflächen-Stabilitätsanalyse im $(\beta, \dot{\beta})$-Zustandsraum für nichtlineares Schleuder- und Driftverhalten.
  - Extended Kalman Filter (EKF) zur Schätzung des Schwimmwinkels und Sensor-Bias-Kompensation.
  - Differentialbremsen zur Giermomentenerzeugung (Direct Yaw Moment Control / DYC) und Kippvermeidung ($LTR$).
  - Verwendet in: [12_chassis_control.md](12_chassis_control.md), [18_state_estimation.md](18_state_estimation.md).

### [10] Lunze, Jan
- **Titel:** *Regelungstechnik 1: Systemtheoretische Grundlagen, Analyse und Entwurf einschleifiger Regelungen* (12. Aufl., 2020) & *Regelungstechnik 2: Mehrgrößensysteme, Digitale Regelung* (10. Aufl., 2020)
- **Verlag:** Springer Vieweg, Berlin / Heidelberg.
- **ISBN:** 978-3-662-60745-9 / 978-3-662-60746-6 (eBook).
- **DOI:** [10.1007/978-3-662-60746-6](https://doi.org/10.1007/978-3-662-60746-6)
- **Inhaltlicher Bezug im Projekt:**
  - Zustandsraumdarstellung, Steuerbarkeit, Beobachtbarkeit, Linear-Quadratische Regler (LQR).
  - Ljapunow-Stabilitätstheorie und robuste Regelung.
  - Verwendet in: [12_chassis_control.md](12_chassis_control.md), [01_architecture.md](01_architecture.md).

---

## 4. Modellprädiktive Regelung (NMPC)

### [11] Ohtsuka, Toshiyuki
- **Titel:** *A Continuation/GMRES Method for Fast Computation of Nonlinear Model Predictive Control*
- **Zeitschrift:** Automatica, Vol. 40, No. 4, S. 563–574, 2004.
- **DOI:** [10.1016/j.automatica.2003.11.005](https://doi.org/10.1016/j.automatica.2003.11.005)
- **Inhaltlicher Bezug im Projekt:**
  - Hintergrundliteratur zu schnellen NMPC-Verfahren. C/GMRES ist im Repository **nicht** implementiert; die NMPC-Regler verwenden SLSQP.
  - Erwähnt in: [16_nmpc_control.md](16_nmpc_control.md), [17_nmpc_path_tracking.md](17_nmpc_path_tracking.md).

### [12] Borrelli, Francesco; Bemporad, Alberto; Morari, Manfred
- **Titel:** *Predictive Control for Linear and Hybrid Systems*
- **Verlag:** Cambridge University Press, 2017.
- **DOI:** [10.1017/9781139061759](https://doi.org/10.1017/9781139061759)
- **Inhaltlicher Bezug im Projekt:**
  - Trajektorienfolgeregelung, Hindernisausweichoptimierung und Aktuatorkopplung im prädiktiven Zeithorizont.
  - Verwendet in: [16_nmpc_control.md](16_nmpc_control.md).

---

## 5. Fahrermodelle & Fahrkomfort

### [13] MacAdam, Charles C.
- **Titel:** *Application of an Optimal Preview Control for Simulation of Closed-Loop Automobile Driving*
- **Zeitschrift:** IEEE Transactions on Systems, Man, and Cybernetics, Vol. 11, No. 6, S. 393–399, 1981.
- **DOI:** [10.1109/tsmc.1981.4308705](https://doi.org/10.1109/tsmc.1981.4308705)
- **Inhaltlicher Bezug im Projekt:**
  - Optimal-Vorausschau-Fahrermodell (Preview Horizon $T_p$, Vorausschaufehler).
  - Verwendet in: [15_driver_model.md](15_driver_model.md).

### [14] Salvucci, Dario D.; Gray, Richard
- **Titel:** *A Two-Point Visual Control Model of Steering*
- **Zeitschrift:** Perception, Vol. 33, S. 1233–1248, 2004.
- **DOI:** [10.1068/p5343](https://doi.org/10.1068/p5343)
- **Inhaltlicher Bezug im Projekt:**
  - Kognitives Zwei-Punkt-Fahrermodell (Fernpunkt zur Krümmungsantizipation, Nahpunkt zur Spurhaltung).
  - Verwendet in: [15_driver_model.md](15_driver_model.md).

### [15] Karnopp, Dean; Crosby, Michael J.; Harwood, R. A.
- **Titel:** *Vibration Control Using Semi-Active Force Generators*
- **Zeitschrift:** ASME Journal of Engineering for Industry, Vol. 96, No. 2, S. 619–626, 1974.
- **DOI:** [10.1115/1.3438373](https://doi.org/10.1115/1.3438373)
- **Inhaltlicher Bezug im Projekt:**
  - Theoretische Herleitung und Realisierungsbedingung der Skyhook-Dämpfungsregel für semi-aktive Fahrwerke.
  - Verwendet in: [09_suspension_dynamics.md](09_suspension_dynamics.md).

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

## 7. Integrierte Fahrwerkregelung und Allokation

### [16] Johansen, Tor A.; Fossen, Thor I.
- **Titel:** *Control allocation: A survey*
- **Zeitschrift:** Automatica, Vol. 49, S. 1087–1103, 2013.
- **DOI:** [10.1016/j.automatica.2013.01.035](https://doi.org/10.1016/j.automatica.2013.01.035)
- **Inhaltlicher Bezug im Projekt:** Beschränkte Kraftallokation in [10_control_allocation.md](10_control_allocation.md), [33_motion_management_benchmark.md](33_motion_management_benchmark.md).

### [17] Breuer, Bert; Bill, Karlheinz H. (Hrsg.)
- **Titel:** *Bremsenhandbuch*
- **Verlag:** Vieweg, 2004.
- **DOI:** [10.1007/978-3-322-99535-3](https://doi.org/10.1007/978-3-322-99535-3)
- **Inhaltlicher Bezug im Projekt:** Bremsfunktionen in [26_brake_functions.md](26_brake_functions.md).

### [18] Yamakado, Makoto; Takahashi, Junya; Saito, Shinjiro; Yokoyama, Atsushi; Abe, Masato
- **Titel:** *Improvement in vehicle agility and stability by G-Vectoring control*
- **Zeitschrift:** Vehicle System Dynamics, Vol. 48, S. 231–254, 2010.
- **DOI:** [10.1080/00423111003706748](https://doi.org/10.1080/00423111003706748)
- **Inhaltlicher Bezug im Projekt:** Veröffentlichtes Prinzip der Lasttransferregelung in [29_g_vectoring.md](29_g_vectoring.md).

### [19] Guiggiani, Massimo
- **Titel:** *The Science of Vehicle Dynamics*
- **Verlag:** Springer, 2023.
- **DOI:** [10.1007/978-3-031-06461-6](https://doi.org/10.1007/978-3-031-06461-6)
- **Inhaltlicher Bezug im Projekt:** Ergänzend zu Querdynamik und Eigenlenkverhalten.
