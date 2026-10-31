# Suspension Dynamics & Ride Comfort

The vehicle suspension system governs two competing performance objectives:
1. **Ride Comfort (ISO 2631):** Isolating vehicle passengers from high-frequency road vibrations and vertical body acceleration ($\ddot{z}_s$).
2. **Road Holding & Handling:** Maintaining consistent vertical tyre contact force ($F_z$) across rough roads to ensure traction and cornering grip.

This document details the quarter-car and 14-DOF suspension models and semi-active damping policies (`src/vehicle_model/suspension/`, 14-DOF model in `src/vehicle_model/vehicle_models/`).

---

## 1. 2-DOF Quarter-Car Model & Modal Analysis

The quarter-car model captures the fundamental vertical dynamics of a single suspension corner:

```
      ┌─────────────────────────┐
      │   Sprung Mass (m_s)     │ ◄─── Chassis body quarter mass
      └────────────┬────────────┘
                   │
            ┌──────┴──────┐
         k_s│             │c_s, F_act
      Spring│             │Damper / Actuator
            └──────┬──────┘
                   │
      ┌────────────┴────────────┐
      │   Unsprung Mass (m_u)   │ ◄─── Wheel, tyre, knuckle, hub
      └────────────┬────────────┘
                   │
            ┌──────┴──────┐
         k_t│ Tyre Spring │
            └──────┬──────┘
                   │
             Road Profile z_r
```

### 1.1 Equations of Motion
$$\begin{aligned}
m_s \ddot{z}_s &= -k_s(z_s - z_u) - c_s(\dot{z}_s - \dot{z}_u) + F_{\text{act}} \\
m_u \ddot{z}_u &= k_s(z_s - z_u) + c_s(\dot{z}_s - \dot{z}_u) - k_t(z_u - z_r) - F_{\text{act}}
\end{aligned}$$

Where:
- $m_s$: Sprung quarter mass [kg] (typically $350-450\text{ kg}$)
- $m_u$: Unsprung wheel assembly mass [kg] (typically $40-55\text{ kg}$)
- $k_s$: Suspension spring stiffness [N/m]
- $c_s$: Passive shock absorber damping coefficient [$\text{N}\cdot\text{s/m}$]
- $k_t$: Radial tyre carcass stiffness [N/m] (typically $180-220\text{ kN/m}$)
- $z_r$: Road surface elevation profile [m]

### 1.2 Natural Frequencies & Mode Shapes
The quarter-car system exhibits two distinct uncoupled natural modes:
1. **Sprung Mass Body Bounce Mode:**
   $$f_s \approx \frac{1}{2\pi} \sqrt{\frac{k_s}{m_s}} \approx 1.1 - 1.4\text{ Hz}$$
   Human sensitivity to motion sickness and vibration discomfort peaks in this frequency band ($1.0 - 2.0\text{ Hz}$).
2. **Unsprung Wheel Hop Mode:**
   $$f_u \approx \frac{1}{2\pi} \sqrt{\frac{k_t}{m_u}} \approx 10.0 - 12.0\text{ Hz}$$
   Wheel hop induces rapid oscillations in tyre normal force $\Delta F_z$, degrading traction and ABS braking.

---

## 2. Semi-Active Suspension Control Policies

Passive shock absorbers require compromising between soft damping (good ride comfort, poor wheel control) and stiff damping (firm handling, harsh ride). Semi-active dampers resolve this by continuously modulating damping force $F_{\text{act}}$ via magnetorheological (MR) fluid or solenoid valves.

### 2.1 Skyhook Damping (Karnopp Formulation)
The Skyhook policy acts as a virtual damper connected between the sprung mass and an imaginary stationary ceiling in the inertial frame:

$$F_{\text{sky}} = -C_{\text{sky}} \dot{z}_s$$

#### Semi-Active Realizability Condition
Because a semi-active damper cannot inject active mechanical energy, it can only generate force dissipative to relative suspension velocity $(\dot{z}_s - \dot{z}_u)$:

$$F_{\text{act}} = \begin{cases}
-C_{\text{sky}} \dot{z}_s & \text{if } \dot{z}_s (\dot{z}_s - \dot{z}_u) \ge 0 \\
0 & \text{otherwise}
\end{cases}$$

**Measured (default parameters, `tests/vehicle_model/test_vertical_dynamics.py`):** body acceleration RMS −33 % at the bounce frequency (1.45 Hz) and −18 % on an ISO 8608 class C road at 72 km/h, compared with the passive damper. The price is road holding: near wheel hop (11.5 Hz) the dynamic tyre load rises by about 75 %.

### 2.2 Groundhook Damping
Groundhook ties the virtual damper to the stationary road frame to minimize tyre deflection:
$$F_{\text{ground}} = C_{\text{ground}} \dot{z}_u$$
$$\text{Condition: } \dot{z}_u (\dot{z}_s - \dot{z}_u) \le 0$$
Intended to reduce dynamic tyre load fluctuations $\Delta F_z$. **Measured (default tuning):** only −2.6 % dynamic tyre load at wheel hop; at the bounce frequency the body mode runs at $c_{min}$ and the dynamic tyre load doubles, and on an ISO 8608 class C road the dynamic tyre load RMS is 8 % *higher* than with the passive damper. With this tuning, groundhook does not improve road holding on broadband roads.

---

## 3. Fourteen Degree-of-Freedom (14-DOF) Full Vehicle Dynamics

The `FourteenDOFModel` ([fourteen_dof.py](https://github.com/CagriCatik/Vehicle-Dynamics/blob/main/src/vehicle_model/vehicle_models/fourteen_dof.py)) provides complete multi-body coupled chassis simulation:

```mermaid
graph TD
    CHASSIS[6-DOF Sprung Chassis<br/>Surge x_s, Sway y_s, Heave z_s<br/>Roll phi, Pitch theta, Yaw psi]

    C1[Corner FL: z_u,FL, omega_FL]
    C2[Corner FR: z_u,FR, omega_FR]
    C3[Corner RL: z_u,RL, omega_RL]
    C4[Corner RR: z_u,RR, omega_RR]

    ARB_F[Front Anti-Roll Bar K_arb,f]
    ARB_R[Rear Anti-Roll Bar K_arb,r]

    CHASSIS <==>|Springs & Dampers| C1
    CHASSIS <==>|Springs & Dampers| C2
    CHASSIS <==>|Springs & Dampers| C3
    CHASSIS <==>|Springs & Dampers| C4

    C1 <==>|Roll Coupling| ARB_F <==> C2
    C3 <==>|Roll Coupling| ARB_R <==> C4
```

### 3.1 Braking Pitch & Cornering Roll Dynamics
- **Braking Pitch ($\theta$):** Inertial deceleration moment $m a_x h_{\text{cog}}$ compresses front suspension and extends rear suspension (pitch dive).
- **Cornering Roll ($\phi$):** Lateral centrifugal moment $m a_y h_{\text{cog}}$ rolls the sprung mass towards the outside corners, transferring vertical load through suspension linkages and anti-roll bars.

---

## 4. Source Code Cross-Reference

| Component | Class Name | Source File | Unit Tests |
| :--- | :--- | :--- | :--- |
| **Quarter-Car Plant** | `QuarterCarModel` | [quarter_car.py](https://github.com/CagriCatik/Vehicle-Dynamics/blob/main/src/vehicle_model/suspension/quarter_car.py) | [test_suspension.py](https://github.com/CagriCatik/Vehicle-Dynamics/blob/main/tests/vehicle_model/test_suspension.py) |
| **Skyhook / Groundhook Controllers** | `SkyhookController`, `GroundhookController` | [controllers.py](https://github.com/CagriCatik/Vehicle-Dynamics/blob/main/src/vehicle_model/suspension/controllers.py) | [test_suspension.py](https://github.com/CagriCatik/Vehicle-Dynamics/blob/main/tests/vehicle_model/test_suspension.py) |
| **14-DOF Vehicle Plant** | `FourteenDOFModel` | [fourteen_dof.py](https://github.com/CagriCatik/Vehicle-Dynamics/blob/main/src/vehicle_model/vehicle_models/fourteen_dof.py) | [test_suspension.py](https://github.com/CagriCatik/Vehicle-Dynamics/blob/main/tests/vehicle_model/test_suspension.py) |

---

## 5. Verification and road excitations

See [08_vertical_dynamics.md](08_vertical_dynamics.md): analytical frequency responses, energy balance, ISO 8608 roads and the semi-active trade-off.

---

<!-- figures:start -->

## Ergebnisbilder

> Alle Bilder sind Simulationsergebnisse mit synthetischen Parametern, keine Messungen. Neu erzeugen: Quellskript ausführen, dann `python tools/publish_doc_figures.py`.

![suspension_and_14dof_validation](assets/09_suspension_dynamics/suspension_and_14dof_validation.png)

*Viertelfahrzeug (passiv, Skyhook, Groundhook) über eine Schwelle und 14-DOF-Fahrzeug bei Bremsen mit Lenken: Nick-, Hub- und Wankbewegung, Radlasten, Bahn. Quelle: `examples/suspension_dynamics_demo.py`.*

![damping_iso_c](assets/09_suspension_dynamics/damping_iso_c.gif)

*ISO-8608-Klasse C bei 60 km/h: passiv, Skyhook, Groundhook.*

<!-- figures:end -->
