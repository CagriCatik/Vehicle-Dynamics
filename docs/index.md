# Vehicle Dynamics & Chassis Control Documentation Hub

Willkommen im zentralen technischen Dokumentations-Hub der **Vehicle Dynamics & Integrated Chassis Control Development Platform**.

Dieses Verzeichnis stellt die vollständige mathematische Herleitung, physikalische Modellierung, regelungstechnische Architektur, Sensorik- und Schätztheorie sowie die normgerechte Validierung nach internationalen Prüfstandards (ISO, ECE, FMVSS, NHTSA) bereit.

---

## 1. Systematische Dokumentations-Roadmap

```mermaid
graph TD
    HUB([Dokumentations-Hub])

    subgraph Grundlagen & Physik
        ARCH[01. Systemarchitektur & ISO 8855<br/>docs/ARCHITECTURE.md]
        PLANT[02. Mehrkörper-Fahrzeugmodelle 7-DOF / 14-DOF<br/>docs/VEHICLE_MODELS.md]
        TYRE[03. Reifenkraftschluss & Modelle Fiala, MF6.2<br/>docs/TYRE_MODELS.md]
        ACT[04. Aktorik, EPS & Bremsdynamik<br/>docs/ACTUATORS.md]
    end

    subgraph Aktive Fahrwerksregelung & Lenkung
        CTRL[05. Integrierte Regelung IVMC & Torque Vectoring<br/>docs/CHASSIS_CONTROL.md]
        RWS[06. Allradlenkung & Hinterachslenkung 4WS<br/>docs/REAR_WHEEL_STEERING.md]
        NMPC[07. Nichtlineares MPC & C/GMRES Pfadverfolgung<br/>docs/NMPC_CONTROL.md & docs/cgmres_nmpc.md]
        DRV[08. Neuromuskuläres Preview-Fahrermodell<br/>docs/DRIVER_MODEL.md]
        SUSP[09. Vertikal- & Wankdynamik Skyhook / ARSDC<br/>docs/SUSPENSION_DYNAMICS.md]
    end

    subgraph Schätzung, Grenzbereich & Validierung
        EST[10. Zustandsschätzung EKF, RLS & PSO<br/>docs/STATE_ESTIMATION.md]
        VAL[11. Virtuelle Validierungsplattform ISO/ECE<br/>docs/VALIDATION_PLATFORM.md]
        LIM[12. Fahrzeughandling & Eigenlenkgradient ISO 4138<br/>docs/HANDLING_LIMITS.md]
        SPLIT[13. Split-Mu Dynamik & Multi-Mu Benchmark<br/>docs/SPLIT_MU_DYNAMICS.md]
        VIZ[14. Visualisierung, Telemetrie & Animationen<br/>docs/VISUALIZATION_GUIDE.md]
        REF_LIT[15. Literatur- & Quellenverzeichnis<br/>docs/REFERENCES.md]
    end

    HUB --> ARCH
    HUB --> PLANT
    HUB --> TYRE
    HUB --> ACT
    HUB --> CTRL
    HUB --> RWS
    HUB --> NMPC
    HUB --> DRV
    HUB --> SUSP
    HUB --> EST
    HUB --> VAL
    HUB --> LIM
    HUB --> SPLIT
    HUB --> VIZ
    HUB --> REF_LIT
```

---

## 2. Kapitelübersicht (Chapter Directory)

### [Kapitel 1: System Architecture & ISO 8855 Conventions](ARCHITECTURE.md)
- Gesamte Softwarearchitektur, Komponentenbeziehungen und Ausführungsreihenfolge.
- ISO 8855 Zwischenachsensystem ($X$-vorwärts, $Y$-links, $Z$-aufwärts).
- Vorzeichenkonventionen für Gierrate ($r$), Schwimmwinkel ($\beta$), Schräglaufwinkel ($\alpha_i$) und Lenkwinkel ($\delta_f, \delta_r$).
- Zentrales Konfigurationssystem (`configs/`) mit modularer YAML-Hierarchie.

### [Kapitel 2: Multi-Body Vehicle Models](VEHICLE_MODELS.md)
- **7-DOF Nichtlineares Zweispurmodell:** 3 translatorische/rotatorische Chassis-Freiheitsgrade ($v_x, v_y, r$) gekoppelt mit 4 unabhängigen Raddynamiken ($\omega_i$).
- **Dynamischer Radlasttransfer:** Längs- und Quer-Gewichtsverlagerung unter exakter Normalkrafterhaltung ($\sum F_{zi} \equiv mg$ mit Fehler $< 10^{-11}\text{ N}$).
- **14-DOF Gesamtfahrzeugmodell:** Aufbau mit 6 Freiheitsgraden (Hub, Wanken, Nicken, Gieren, Längs, Quer) und 4 Radmassen mit je 2 Freiheitsgraden (Radeinfederung und Rotation).
- **Einspurmodelle:** Kinetisches und dynamisches Einspurmodell (*Bicycle Model*).

### [Kapitel 3: Interchangeable Tyre Framework & Provenance](TYRE_MODELS.md)
- Verbindlicher `TyreProvenance`-Standard (Formulierung, Gültigkeitsbereich, Nennlast, Reibwert).
- **`LinearTyre`:** Lineare Schräglaufsteifigkeit für kleine Schlupfwerte mit degressiver Radlastabhängigkeit.
- **`FialaTyre`:** Analytisches Bürstenmodell mit parabolischer Druckverteilung, Haft-/Gleitgrenzen und kombiniertem Schlupf.
- **`SimplifiedCombinedSlipTyre`:** Reibkreisskalierter Schlupfvektor strictly bounded by Kamm's Circle.
- **`PacejkaTyre`:** Magic Formula (MF 6.2) Quer- und Längsdynamikkopplung.
- Automatisierte Reifendiagnose-Plot-Suite (`python -m vehicle_model.tire_model.plots --all`).

### [Kapitel 4: Actuator Dynamics & Brake Systems](ACTUATORS.md)
- **Electric Power Steering (`SteeringActuator`):** PT1-Verzögerung ($\tau \approx 45\text{ ms}$), Zahnstangenanschläge ($\pm 35^\circ$), Ratenbegrenzung ($60^\circ/\text{s}$), Totzeit und injizierbare Fehlermodi (`stuck`, `free`, `offset`).
- **4-Motor Elektroantrieb (`ElectricPowertrainActuator`):** Inverter-Verzögerung ($\tau \approx 20\text{ ms}$), Drehmomentgradienten ($8000\text{ Nm/s}$), Feldschwächbereich ($T_{\max} = \min(T_{\max}, P_{\max}/\omega)$) und DC-Batterieleistungsdeckel.
- **Brake-by-Wire (`BrakeByWireActuator`):** Asymmetrische Druckaufbau- und Abbaugradienten (ABS-Druckabbau $36000\text{ Nm/s}$ vs. Druckaufbau $22000\text{ Nm/s}$) und Rekuperations-Blending.

### [Kapitel 5: Integrated Chassis Control (IVMC, TV & Phase-Plane)](CHASSIS_CONTROL.md)
- **Integrated Vehicle Motion Control (IVMC):** Hierarchische Regelung aus stationärer Giervorsteuerung, PID/LQR-Gierdämpfung, Allradlenkungskoordination und beschränkter Drehmomentallokation.
- **Aktives Torque Vectoring:** Erzeugung eines direkten Giermoments über radindividuelle Längskraftdifferenzen:
  $$M_{z,\text{tv}} = (F_{x,FR} + F_{x,RR} - F_{x,FL} - F_{x,RL}) \cdot \frac{w}{2}$$
- **$(\beta, \dot{\beta})$ Phasenraum-Hüllkurvenregelung:** Dynamische Stabilitätsgrenzen über variablem Reibwert ($\mu$), Sliding-Mode-Rückstellmoment und Lastwechselübersteuern-Kompensation.
- **Active Roll Stiffness Distribution Control (ARSDC):** Dynamische Wanksteifigkeitsverteilung ($\kappa_{\text{roll},f} \in [0.32, 0.82]$) zur Ausnutzung der Reifenlastdegressivität und Überschlagsverhinderung ($LTR < 0.70$).

### [Kapitel 6: Four-Wheel Steering & Rear-Axle Kinematics (Hinterachslenkung)](REAR_WHEEL_STEERING.md)
- **Kinematik & Geschwindigkeitsbereiche:**
  - Gegenphasenlenkung ($\delta_r < 0$) bei $v_x < 50\,\text{km/h}$ zur Wendekreisverkleinerung (virtuelle Radstandsverkürzung).
  - Gleichphasenlenkung ($\delta_r > 0$) bei $v_x > 60\,\text{km/h}$ zur Schwimmwinkelbeseitigung ($\beta \to 0$) und verzögerungsfreien Querbeschleunigung (*Crab-Translation*).
- **Analytische Herleitung der Schwimmwinkelfreiheit ($\beta_{\text{ss}} = 0$):** Exakte Formel über Vorder- und Hinterachsschräglaufsteifigkeiten sowie Inversionsgeschwindigkeit $v_{\text{switch}} \approx 36\,\text{km/h}$.
- **2D Bilineare Lookup-Tabelle (`lut.json`):** Kalibriertes Kennfeld über $(v_x, \delta_f)$ mit Ratenbegrenzung ($35^\circ/\text{s}$) und Maximalwinkel ($10^\circ$).
- **Aktive Gierdämpfung:** Rückgekoppelte Überlagerung $\Delta\delta_r = k_r \cdot (r - r_{\text{target}})$.
- **Reibwertgrenzen:** Phänomenologie der Querkraftsättigung auf Schnee/Eis ($\mu \le 0.40$) und Zusammenspiel mit IVMC-Differenzbremsen.

### [Kapitel 7: Non-Linear Model Predictive Control & C/GMRES](NMPC_CONTROL.md) & [docs/cgmres_nmpc.md](cgmres_nmpc.md)
- **Continuation / GMRES (C/GMRES):** Fortsetzungsmethode nach T. Ohtsuka zur Lösung der KKT-Optimalitätsbedingungen in Echtzeit ohne Matrixinversion.
- **Pfadverfolgung mit Hindernisausweich-Barrieren:** Raumbezogene Referenztrajektorie $Y_{\text{ref}}(X)$, weiche logarithmische Barrierenfunktionen und Warm-Starting.
- **Elektronisch gekoppelte 4WS-Hinterachse:** Einbindung der Hinterachslenkung $\delta_r = \operatorname{LUT}(v_x, \delta_f)$ in den Prädiktionsrollout.
- **Dynamic Two-Track NMPC:** Nichtlineare Prädiktion über dem vollen Zweispurmodell mit expliziter Einhaltung der Kamm'schen Reibkreise an allen 4 Rädern:
  $$\rho_i = \frac{\sqrt{F_{x,i}^2 + F_{y,i}^2}}{\mu_i F_{z,i}} \le 1.0$$

### [Kapitel 8: Closed-Loop Neuromuscular Preview Driver Model](DRIVER_MODEL.md)
- MacAdam Optimal Preview und Salvucci Two-Point Visual Driver Models.
- Menschliche Transporttotzeit ($\tau_d \approx 150-200\text{ ms}$), neuromuskuläre Verzögerung ($\tau_n \approx 100\text{ ms}$) und Vorschauhorizont ($T_p \approx 1{,}0-1{,}5\text{ s}$).
- Regelkreisstabilität Fahrer-Fahrzeug, Vermeidung fahrerinduzierter Schwingungen (*Pilot-Induced Oscillations*, PIO) und Fahrer-Arbeitslastbewertung bei $\mu$-Split.

### [Kapitel 9: Suspension Dynamics & Ride Comfort](SUSPENSION_DYNAMICS.md)
- 2-DOF Viertelfahrzeugmodell und Modalanalyse (Aufbauhub ~1.1 Hz vs. Radeigenfrequenz ~10.5 Hz).
- Semi-aktive Dämpferstrategien: Skyhook-Dämpfung (Aufbaubeschleunigungsisolation) und Groundhook-Dämpfung (Radlastschwankungsminimierung).
- 14-DOF Gesamtfahrzeug-Vertikal-, Nick- und Wankdynamik bei scharfen Brems- und Lenkmanövern.

### [Kapitel 10: State Estimation & Optimization Suite](STATE_ESTIMATION.md)
- Sensorsuite mit realistischem IMU-Rauschen, Beschleunigungsmesser-Bias und Gyro-Drift.
- Extended Kalman Filter (EKF) zur Schätzung des Karosserieschwimmwinkels ($\beta$) und Bias-Zustände.
- Recursive Least Squares (RLS) mit exponentiellem Vergessensfaktor zur Online-Reibwertidentifikation ($\mu$).
- Particle Swarm Optimization (PSO) zur automatisierten Fahrwerksparameter- und Regleroptimierung.

### [Kapitel 11: Automotive Virtual Validation Platform](VALIDATION_PLATFORM.md)
- Automatisierte Ausführung von 10 standardisierten virtuellen Prüfmanövern (DLC, $\mu$-Split, Sine with Dwell, Step Steer, Bodenwelle, Bremsen in Kurve, Fishhook, Slalom, Autobahnspurwechsel, Kreisfahrt).
- Mathematische Definition standardisierter KPIs (Gierratenfehler, Schwimmwinkel, Kursabweichung, Bahnabweichung, Kamm-Ausnutzung, Effektivbeschleunigung, Eigenlenkgradient).
- Pass/Fail-Schwellenwerte und Regressionsberichte (HTML, Markdown, Telemetrie-Plots).

### [Kapitel 12: Vehicle Handling Limits & Understeer Characteristics](HANDLING_LIMITS.md)
- ISO 4138 stationäre Kreisfahrt mit konstantem Radius und quasi-stationärer Geschwindigkeitsrampe.
- Quantitative Auswertung des Eigenlenkgradienten $K_{\text{us}}(a_y) = d\delta_f / da_y$ und ISO 8855 Lenkcharakteristik.
- Charakteristische Geschwindigkeit $v_{\text{char}}$ und kritische Geschwindigkeit $v_{\text{crit}}$.
- Schwimmwinkelgradient $d\beta / da_y$, Gierverstärkung und kombiniertes G-G-Beschleunigungsdiagramm ($a_x$ vs. $a_y$).

### [Kapitel 13: Split-Mu Dynamics & Multi-Friction Matrix Benchmark](SPLIT_MU_DYNAMICS.md)
- Physik asymmetrischer Reibwertteilungen ($\mu_{\text{high}} = 1{,}0$ vs. $\mu_{\text{low}} \in [0{,}10, 0{,}80]$).
- Analytische Herleitung des Störgiermoments:
  $$M_{z,\text{disturb}} \approx \frac{1}{2} mg (\mu_L - \mu_R) \frac{w}{2} > 4{,}5\,\text{kNm}$$
- 5 Standardisierte Split-$\mu$-Szenarien: Vollbremsung (ECE-R13H), Kavalierstart (TCS/TV), Bremsen in der Kurve (ISO 7975), Diagonale Brückenfuge, Ausweichtest (ISO 3888-2).
- Räumliche 2D-Fahrbahnmodelle: `DiagonalSplitRoad`, `CurvedSplitRoad`, `CheckerboardRoad`.
- **Multi-Mu Matrix Framework (`multi_mu_matrix.py`):** Systematische Gesamtevaluierung aller 6 Fahrdynamikmanöver über 4 Reibwertstufen ($\mu = 1{,}00, 0{,}70, 0{,}40, 0{,}20$).

### [Kapitel 14: Visualization & Animation Guide](VISUALIZATION_GUIDE.md)
- 2D Plan-View Rendering Engine (`draw_vehicle`) mit exakten Abmessungen, lenkbaren Rädern und Vektorindikatoren.
- 20 FPS GIF-Engine (`animate_maneuver`) mit Spurverfolgungskamera, synchronisiertem Mehrfahrzeugvergleich und Live-HUD.
- Unified Dark Automotive Telemetry Design System (`#0b1120`, `#0f172a`, `#1e293b`).

### [Kapitel 15: Literatur- und Quellenverzeichnis (References & Bibliography)](REFERENCES.md)
- Standardwerke der Fahrzeugtechnik: Küçükay, Mitschke, Wallentowitz, Heißing, Ersoy, Schramm, Gillespie.
- Reifenmechanik: Hans B. Pacejka, Ernst Fiala.
- Regelungstechnik & Prädiktion: Ohtsuka, Borrelli, Isermann, Rajamani.
- Prüfnormen: ISO 8855, ISO 4138, ISO 3888-2, ISO 7401, ISO 7975, FMVSS 126, ECE-R13H, NHTSA NCAP.

---

## 3. Thematischer Index (Thematic Classification)

### I. Physikalische Modelle & Reifenkraftschluss
- [7-DOF Nichtlineares Zweispurmodell](VEHICLE_MODELS.md#1-the-7-dof-two-track-vehicle-model)
- [Dynamischer Radlasttransfer ($F_{z,i}$)](VEHICLE_MODELS.md#12-dynamic-vertical-load-transfer)
- [14-DOF Gesamtfahrzeugmodell](VEHICLE_MODELS.md#2-the-14-dof-full-vehicle-model)
- [Fiala Reifenkraftmodell](TYRE_MODELS.md#2-fialatyre-analytical-brush-model)
- [Pacejka Magic Formula MF 6.2](TYRE_MODELS.md#4-pacejkatyre-magic-formula-62)
- [Kamm'scher Reibkreis & Combined Slip](TYRE_MODELS.md#3-simplifiedcombinedsliptyre-normalized-kamm-circle)

### II. Regelungstechnik, Aktorik & aktive Fahrwerke
- [Integrated Vehicle Motion Control (IVMC)](CHASSIS_CONTROL.md#1-integrated-vehicle-motion-controller-ivmc)
- [Allradlenkung (4WS / Hinterachslenkung)](REAR_WHEEL_STEERING.md)
- [2D Bilineares Hinterachskennfeld (`lut.json`)](REAR_WHEEL_STEERING.md#5-2d-bilineare-lookup-tabelle-lutjson)
- [Aktives Torque Vectoring](CHASSIS_CONTROL.md#3-active-torque-vectoring-direct-yaw-moment)
- [$(\beta, \dot{\beta})$ Phasenraum-Stabilitätsgrenzen](CHASSIS_CONTROL.md#4-beta-dot-beta-phase-plane-envelope-control)
- [Aktive Wanksteifigkeitsverteilung (ARSDC)](CHASSIS_CONTROL.md#5-active-roll-stiffness-distribution-control-arsdc)
- [EPS, E-Antrieb & Brake-by-Wire Aktorik](ACTUATORS.md)
- [Semi-aktive Dämpfung (Skyhook / Groundhook)](SUSPENSION_DYNAMICS.md#2-semi-active-suspension-control-policies)

### III. Autonome Bahnplanung & Prädiktion
- [C/GMRES NMPC Pfadverfolgung](cgmres_nmpc.md)
- [Slaved 4WS NMPC Integration](cgmres_nmpc.md#41-kinematisches-ackermann-bicycle-modell-mit-4ws-kopplung)
- [Dynamischer Two-Track Kamm NMPC](NMPC_CONTROL.md#2-dynamic-two-track-nmpc-with-4-wheel-kamm-friction-bounds)
- [Warm-Starting & SQP Rechenzeitoptimierung](cgmres_nmpc.md#6-warm-starting--rechenzeit-optimierung)

### IV. Schätzung, Fahrer & Parametersuche
- [Extended Kalman Filter (EKF) für Schwimmwinkel $\beta$](STATE_ESTIMATION.md#2-extended-kalman-filter-ekf-for-sideslip-estimation)
- [Recursive Least Squares (RLS) Reibwertschätzung](STATE_ESTIMATION.md#3-recursive-least-squares-rls-friction-estimation)
- [Particle Swarm Optimization (PSO) Regler-Tuning](STATE_ESTIMATION.md#4-offline-particle-swarm-optimization-pso)
- [MacAdam / Salvucci Vorschau-Fahrermodelle](DRIVER_MODEL.md)

### V. Grenzbereich, Prüfstände & Reibwerte
- [ISO 4138 Kreisfahrt & Eigenlenkgradient](HANDLING_LIMITS.md)
- [G-G-Diagramm & Beschleunigungsgrenzen](HANDLING_LIMITS.md#5-handling-limits-and-the-g-g-acceleration-envelope)
- [Split-Mu Fahrdynamik & Giermomentenverzögerung](SPLIT_MU_DYNAMICS.md)
- [Multi-Mu Benchmark-Matrix ($\mu = 0{,}2 \dots 1{,}0$)](SPLIT_MU_DYNAMICS.md) & [multi_mu_scenarios_report.md](../output/benchmark/multi_mu_scenarios_report.md)
- [Automotive Virtual Validation Platform (10 Szenarien)](VALIDATION_PLATFORM.md)

---

## 4. Alphabetisches Stichwort- & Begriffsverzeichnis (A–Z Master Index)

| Stichwort / Begriff | Kapitel & Dokument | Relevante Gleichungen / Aspekte |
| :--- | :--- | :--- |
| **Ackermann-Geometrie** | [Kap. 2](VEHICLE_MODELS.md) & [Kap. 6](REAR_WHEEL_STEERING.md) | $\delta_{\text{ack}} = L / R$, Spurdifferenzwinkel |
| **Active Roll Stiffness (ARSDC)** | [Kap. 5](CHASSIS_CONTROL.md#5-active-roll-stiffness-distribution-control-arsdc) | $\kappa_{\text{roll},f} \in [0.32, 0.82]$, Wankmomentverteilung |
| **Allradlenkung (4WS / RWS)** | [Kap. 6](REAR_WHEEL_STEERING.md) | Gegenphase, Gleichphase, $v_{\text{switch}} \approx 36\,\text{km/h}$ |
| **Bilineare Lookup-Tabelle (LUT)** | [Kap. 6](REAR_WHEEL_STEERING.md#5-2d-bilineare-lookup-tabelle-lutjson) & [Kap. 7](cgmres_nmpc.md#41-kinematisches-ackermann-bicycle-modell-mit-4ws-kopplung) | `configs/steering/lut.json`, $\delta_r(v_x, \delta_f)$ |
| **Bremsen in der Kurve (ISO 7975)** | [Kap. 11](VALIDATION_PLATFORM.md) & [Kap. 13](SPLIT_MU_DYNAMICS.md) | Kombinierte Brems- und Querkräfte, Lasttransfer |
| **Brake-by-Wire (BBW)** | [Kap. 4](ACTUATORS.md#3-brake-by-wire-brakebywireactuator) | Asymmetrische Gradienten (Aufbau $22\,\text{kN/s}$, Abbau $36\,\text{kN/s}$) |
| **C/GMRES Algorithmus** | [Kap. 7](cgmres_nmpc.md) | Fortsetzungsmethode, Krylow-Unterraum, $\dot{\mathbf{F}} = -\zeta \mathbf{F}$ |
| **Charakteristische Geschwindigkeit** | [Kap. 12](HANDLING_LIMITS.md#4-characteristic-and-critical-speeds) | $v_{\text{char}} = \sqrt{L / K_{\text{us}}}$ (für $K_{\text{us}} > 0$) |
| **Crab-Translation (Hundegang)** | [Kap. 6](REAR_WHEEL_STEERING.md#22-gleichphasenlenkung-bei-hohen-geschwindigkeiten-v_x--v_textswitch) | Parallele Karosserieverschiebung bei Gleichphasen-4WS |
| **Direct Multiple Shooting** | [Kap. 7](cgmres_nmpc.md#4-implementierungsarchitektur-im-fahrzeug-framework) | Diskrete Zustandsintegration über Horizont $N$ |
| **Double Lane Change (ISO 3888-2)** | [Kap. 11](VALIDATION_PLATFORM.md) & [Kap. 13](SPLIT_MU_DYNAMICS.md) | Elchtest / Hindernisausweichen bei $80\,\text{km/h}$ |
| **Driver-in-the-Loop** | [Kap. 8](DRIVER_MODEL.md) | MacAdam Vorschauhorizont, Totzeit $\tau_d = 160\,\text{ms}$ |
| **Dynamic Load Transfer ($F_{z,i}$)** | [Kap. 2](VEHICLE_MODELS.md#12-dynamic-vertical-load-transfer) | $\Delta F_{z,\text{long}} = m a_x h / L$, $\Delta F_{z,\text{lat}} = m a_y h / w$ |
| **Eigenlenkgradient $K_{\text{us}}$** | [Kap. 12](HANDLING_LIMITS.md#3-the-understeer-gradient-k_textus) | $K_{\text{us}} = d\delta_f / da_y - L g / v_x^2$ |
| **Extended Kalman Filter (EKF)** | [Kap. 10](STATE_ESTIMATION.md#2-extended-kalman-filter-ekf-for-sideslip-estimation) | Nichtlineare Schwimmwinkelschätzung $\hat{\beta}$ |
| **Fiala Reifenmodell** | [Kap. 3](TYRE_MODELS.md#2-fialatyre-analytical-brush-model) | Bürstentheorie, Haftung und Gleitung |
| **Fishhook Manöver (NHTSA)** | [Kap. 11](VALIDATION_PLATFORM.md) & [Kap. 13](SPLIT_MU_DYNAMICS.md) | Dynamische Lenkumkehr, Wankstabilitätsprüfung |
| **Gegenphasenlenkung** | [Kap. 6](REAR_WHEEL_STEERING.md#21-gegenphasenlenkung-bei-niedrigen-geschwindigkeiten-v_x--v_textswitch) | Hinterräder entgegengesetzt zu Vorderrädern ($\delta_r < 0$) |
| **Giermomenten-Aufbauverzögerung (GMR)**| [Kap. 13](SPLIT_MU_DYNAMICS.md) | Druckanstiegsbegrenzung bei $\mu$-Split-Bremsung |
| **Gleichphasenlenkung** | [Kap. 6](REAR_WHEEL_STEERING.md#22-gleichphasenlenkung-bei-hohen-geschwindigkeiten-v_x--v_textswitch) | Hinterräder gleichsinnig zu Vorderrädern ($\delta_r > 0$) |
| **Groundhook-Dämpfung** | [Kap. 9](SUSPENSION_DYNAMICS.md#2-semi-active-suspension-control-policies) | Minimierung der Reifenaufstandskraftschwankungen |
| **Handling Limits (Grenzbereich)** | [Kap. 12](HANDLING_LIMITS.md) | G-G-Diagramm, $a_{y,\max} \approx \mu \cdot g$ |
| **Hinterachslenkung** | [Kap. 6](REAR_WHEEL_STEERING.md) | Siehe *Allradlenkung (4WS)* |
| **Integrated Chassis Control (IVMC)** | [Kap. 5](CHASSIS_CONTROL.md) | Koppelung von 4WS, Torque Vectoring & Brems-ESC |
| **ISO 8855 Koordinatensystem** | [Kap. 1](ARCHITECTURE.md#1-the-iso-8855-intermediate-axis-system) | $X$-vor, $Y$-links, $Z$-auf |
| **Kamm'scher Kreis** | [Kap. 3](TYRE_MODELS.md) & [Kap. 7](NMPC_CONTROL.md) | $\sqrt{F_x^2 + F_y^2} \le \mu F_z$ |
| **Kritische Geschwindigkeit** | [Kap. 12](HANDLING_LIMITS.md#4-characteristic-and-critical-speeds) | $v_{\text{crit}} = \sqrt{-L / K_{\text{us}}}$ (für Übersteuern $K_{\text{us}} < 0$) |
| **Multi-Mu Matrix Framework** | [Kap. 13](SPLIT_MU_DYNAMICS.md) & [Report](../output/benchmark/multi_mu_scenarios_report.md) | 6 Manöver $\times$ 4 Reibwerte ($\mu \in [0.20, 1.00]$) |
| **Neuromuskuläre Verzögerung** | [Kap. 8](DRIVER_MODEL.md) | Menschliche Muskelzeitkonstante $\tau_n \approx 100\,\text{ms}$ |
| **NMPC (Nichtlineares MPC)** | [Kap. 7](NMPC_CONTROL.md) & [docs/cgmres_nmpc.md](cgmres_nmpc.md) | Prädiktive Trajektorienoptimierung |
| **Pacejka Magic Formula (MF 6.2)** | [Kap. 3](TYRE_MODELS.md#4-pacejkatyre-magic-formula-62) | $y(x) = D \sin(C \arctan(B x - E (B x - \arctan(B x))))$ |
| **Particle Swarm Optimization (PSO)**| [Kap. 10](STATE_ESTIMATION.md#4-offline-particle-swarm-optimization-pso) | Globale stochastische Parameteridentifikation |
| **Phasenraum-Hüllkurve $(\beta, \dot{\beta})$**| [Kap. 5](CHASSIS_CONTROL.md#4-beta-dot-beta-phase-plane-envelope-control) | Schwimmwinkel-Phasenebene Stabilitätsgrenzen |
| **Recursive Least Squares (RLS)** | [Kap. 10](STATE_ESTIMATION.md#3-recursive-least-squares-rls-friction-estimation) | Online-Schätzung des Reibwerts $\hat{\mu}$ |
| **Schwimmwinkel $\beta$** | [Kap. 1](ARCHITECTURE.md) & [Kap. 6](REAR_WHEEL_STEERING.md) | $\beta = \arctan(v_y / v_x)$, Karosseriewinkel |
| **Schwimmwinkelfreiheit ($\beta = 0$)** | [Kap. 6](REAR_WHEEL_STEERING.md#32-analytische-bedingung-fr-schwimmwinkelfreiheit-beta_textss-equiv-0) | Analytische 4WS-Lenkbedingung |
| **Sine with Dwell (FMVSS 126)** | [Kap. 11](VALIDATION_PLATFORM.md) & [Kap. 13](SPLIT_MU_DYNAMICS.md) | US-ESC-Stabilitätsprüfnorm bei $80\,\text{km/h}$ |
| **Skyhook-Dämpfung** | [Kap. 9](SUSPENSION_DYNAMICS.md#2-semi-active-suspension-control-policies) | $F_{\text{sky}} = -c_{\text{sky}} \dot{z}_s$, Hubdämpfung |
| **Split-Mu Bremsung (ECE-R13H)** | [Kap. 13](SPLIT_MU_DYNAMICS.md) | Asymmetrische Reibung links/rechts |
| **Step Steer (ISO 7401)** | [Kap. 11](VALIDATION_PLATFORM.md) & [Kap. 13](SPLIT_MU_DYNAMICS.md) | Sprunglenkung, Reaktionszeit- & Dämpfungsprüfung |
| **Torque Vectoring** | [Kap. 5](CHASSIS_CONTROL.md#3-active-torque-vectoring-direct-yaw-moment) | Direktes Giermoment $M_{z,\text{tv}} = \Delta F_x \cdot w / 2$ |
| **Understeer (Untersteuern)** | [Kap. 12](HANDLING_LIMITS.md) | $K_{\text{us}} > 0$, Schiebt über Vorderräder |
| **Validation Platform** | [Kap. 11](VALIDATION_PLATFORM.md) | Virtuelle Testplattform für Regressionsprüfung |
| **Virtueller Radstand** | [Kap. 6](REAR_WHEEL_STEERING.md#21-gegenphasenlenkung-bei-niedrigen-geschwindigkeiten-v_x--v_textswitch) | $l_{\text{virt}} < l$ bei Gegenphasenlenkung |
| **Warm-Starting** | [Kap. 7](cgmres_nmpc.md#6-warm-starting--rechenzeit-optimierung) | Initialisierung mit verschobener Vorlösung |
| **Zweispurmodell (7-DOF)** | [Kap. 2](VEHICLE_MODELS.md#1-the-7-dof-two-track-vehicle-model) | Vollständige ebene Dynamik + 4 Raddrehzahlen |
