# Integrated Chassis Control & Stability Systems

Modern active vehicle dynamics systems combine multiple control effectors—electric power steering, four-wheel steering, independent electric powertrains, active anti-roll bars, and brake-by-wire.

This document details the active chassis control algorithms implemented in `src/vehicle_model/controllers/` and `src/vehicle_model/steering/`:
1. **Integrated Vehicle Motion Control (IVMC)** (`IntegratedChassisController`)
2. **Speed-Scheduled Four-Wheel Steering (4WS) & Calibrated LUT** (`FourWheelSteeringController`)
3. **Phase-Plane Envelope Controller ($(\beta, \dot{\beta})$ Stability Manifold)** (`PhasePlaneEnvelopeController`)
4. **Active Roll Stiffness Distribution Control (ARSDC)** (`ActiveRollController`)

---

## 1. Integrated Vehicle Motion Control (IVMC) Architecture

All active chassis controllers adhere to the common `VehicleController` protocol:
```python
output: ControlOutput = controller.compute_control(state, target, telemetry, dt)
```

IVMC coordinates active steering and torque vectoring through a **three-tier hierarchical architecture**:

```mermaid
graph TD
    subgraph 1. Upper Layer: Driver Intention
        DRV[Driver Steer delta_f & Target Speed v_x] --> REF[Neutral-Steer Target Reference<br/>r_des & beta_des = 0]
        FRICTION[Estimated Road Friction mu] --> LIMIT[Physical Adhesion Clamping<br/>|r_des| <= 0.85 * mu * g / v_x]
        REF --> LIMIT
    end

    subgraph 2. Middle Layer: Stability Regulation
        LIMIT --> COMP[Yaw Rate & Sideslip Tracking Errors<br/>e_r = r - r_des,  beta]
        COMP --> PID[Dual PID / LQR Feedback Law]
        PID --> MZ[Total Desired Corrective Yaw Moment M_z]
    end

    subgraph 3. Lower Layer: Constrained Control Allocation
        MZ --> ALLOC{QP / Daisy-Chain Allocator}
        ALLOC -->|Low/Medium Demand| TV_DRIVE[Electric Powertrain Torque Vectoring]
        ALLOC -->|High Demand / Braking| TV_BRAKE[Hydraulic Differential Caliper Braking]
        ALLOC -->|Chassis Agility & Stability| RWS_STEER[Active Rear-Wheel Steer delta_r]
    end
```

### 1.1 Reference State Generation (Bicycle Model Preview)
The desired yaw rate represents the steady-state response of an ideal neutral-steer vehicle (Küçükay 2022, Kap. 6; Mitschke & Wallentowitz 2014, Kap. 12):
$$r_{\text{des}} = \frac{v_x}{L + K_{\text{us}} v_x^2} \delta_f$$

Where the understeer gradient $K_{\text{us}}$ is defined by front and rear axle cornering stiffnesses:
$$K_{\text{us}} = \frac{m}{L}\left(\frac{l_r}{C_{\alpha,f}} - \frac{l_f}{C_{\alpha,r}}\right)$$

#### Physical Friction Boundary Enforcement
Because the tyre cannot generate forces exceeding the road surface coefficient of friction $\mu$, the reference yaw rate is strictly bounded (Rajamani 2012, Sec. 7.2):
$$|r_{\text{des}}| \le \frac{0.85 \mu g}{v_x}$$
This prevents the controller from demanding unachievable lateral acceleration on low-$\mu$ surfaces (snow, ice).

### 1.2 Feedback Regulation Law
The corrective yaw moment stabilizes yaw rate tracking error $e_r = r - r_{\text{des}}$ and penalizes high body sideslip angle $\beta$ (Isermann 2010, Kap. 10; Lunze 2020):
$$M_{z,\text{stab}} = K_p (r - r_{\text{des}}) + K_d \dot{r} + K_\beta \beta$$

Where:
- $K_p$: Proportional yaw damping gain [$\text{Nm}/(\text{rad/s})$]
- $K_d$: Derivative yaw acceleration damping gain [$\text{Nm}/(\text{rad/s}^2)$]
- $K_\beta$: Body sideslip suppression gain [$\text{Nm/rad}$]

---

## 2. Four-Wheel Steering (4WS) & Calibrated 2D Lookup Table

Four-Wheel Steering electronically controls the rear-axle steer angle $\delta_r$ in coordination with the front-axle steer $\delta_f$ (Heißing, Ersoy & Gies 2020, Kap. 8; Küçükay 2022, Kap. 7).

### 2.1 Speed-Scheduled Operating Modes
```
Low Speed (v < 35 km/h)          Medium Speed (35-50 km/h)         High Speed (v > 50 km/h)
      Counter-Phase                     Transition                         In-Phase
   δ_r opposite to δ_f                 δ_r ≈ 0.0                     δ_r same sign as δ_f
  Virtual Wheelbase Shortened       Neutral Handover              Crab Translation (β → 0)
    High Parking Agility                                             Highway Stability
```

1. **Counter-Phase Steering (Low Speed):**
   - Rear wheels steer opposite to the front wheels ($\operatorname{sgn}(\delta_r) = -\operatorname{sgn}(\delta_f)$).
   - Dynamically shortens the virtual wheelbase, reducing the minimum turning circle by up to 25% in urban driving and tight slaloms (Heißing et al. 2020).
2. **In-Phase Steering (High Speed):**
   - Rear wheels steer in the same direction as front wheels ($\operatorname{sgn}(\delta_r) = +\operatorname{sgn}(\delta_f)$).
   - Generates immediate lateral acceleration without requiring the vehicle to yaw first, producing **pure crab translation** and reducing body sideslip angle $\beta$ by up to **40%**.

### 2.2 Calibrated 2D Bilinear Lookup Table (`lut.json`)
The platform includes an offline-calibrated 2D lookup table (`LookupTable` in [lut.py](../src/vehicle_model/steering/lut.py)) that computes the optimal rear steer angle $\delta_r^*(v_x, \delta_f)$ that minimizes body sideslip:

$$\delta_r = \operatorname{LUT}(v_x, \delta_f)$$
- Bilinear interpolation over grid knots $v_x \in [0, 35]\text{ m/s}$ and $\delta_f \in [-25^\circ, +25^\circ]$
- Safe boundary extrapolation with angle clamping to physical rack limits ($|\delta_r| \le 5.0^\circ$)

---

## 3. $(\beta, \dot{\beta})$ Phase-Plane Envelope Controller

Linear controllers lose authority when tyres saturate near the adhesion limit. The Phase-Plane Envelope Controller ([phase_plane.py](../src/vehicle_model/controllers/phase_plane.py)) partitions the non-linear body sideslip state space $(\beta, \dot{\beta})$ into dynamically stable, approaching, and unstable regions (Rajamani 2012, Sec. 7.4; Isermann 2010):

```
                      d(beta)/dt
                          ▲
                          │      Boundary Limit 
                     .....┼.....     (Oversteer Spin Boundary)
                ..''      │      ``..
            .''           │           ``.
          ┌───────────────┼───────────────┐
          │  UNSTABLE     │   APPROACHING │
          │  EMERGENCY    │   SOFT TV     │
──────────┼───────────────┼───────────────┼──────────► beta
          │  ZONE         │   ZONE        │
          │               │   LINEAR CORE │
          └───────────────┼───────────────┘
            ``..          │           ..''
                ``..      │      ..''
                     ``···┼···'''
                          │
```

### 3.1 Dynamic Stability Boundaries
The stability boundaries scale dynamically with available tyre-road friction coefficient $\mu$ (Rajamani 2012):
$$\beta_{\max}(\mu) = \arcsin\left(\frac{0.02 \mu g}{\max(v_x, 2.0)}\right)$$
$$\dot{\beta}_{\max}(\mu) = \frac{\mu g}{v_x}$$

### 3.2 Stability Index ($S_{\text{phase}}$)
The normalized state distance from the origin is defined as:
$$S_{\text{phase}} = \left|\frac{\beta}{\beta_{\max}}\right| + \left|\frac{\dot{\beta}}{\dot{\beta}_{\max}}\right|$$

- **Region I ($S_{\text{phase}} < 0.85$): Linear Core**
  - Chassis is inside the stable manifold; controller remains passive.
- **Region II ($0.85 \le S_{\text{phase}} < 1.0$): Soft Approach Zone**
  - Gentle corrective torque vectoring proportional to boundary penetration.
- **Region III ($S_{\text{phase}} \ge 1.0$): Critical Saturation Zone**
  - Non-linear sliding-mode restoring yaw moment:
    $$M_{z,\text{stab}} = -M_{z,\max} \operatorname{sgn}(\beta + \tau_{\text{phase}} \dot{\beta})$$
  - Active autonomous counter-steering $\delta_{\text{cs}}$ applied directly to the front rack to catch lift-off oversteer and prevent spinout.

---

## 4. 14-DOF Active Roll Stiffness Distribution Control (ARSDC)

In aggressive steering reversals (e.g., NHTSA Dynamic Fishhook), excessive lateral load transfer causes inner-wheel lift and vehicle rollover. The Active Roll Controller ([active_roll.py](../src/vehicle_model/controllers/active_roll.py)) modulates the front-to-rear dynamic roll stiffness ratio (Mitschke & Wallentowitz 2014, Kap. 13; Küçükay 2022, Kap. 8):

$$\kappa_{\text{roll},f} = \frac{K_{\text{roll},f}}{K_{\text{roll},f} + K_{\text{roll},r}} \in [0.32, 0.82]$$

### 4.1 Physical Mechanism: Tyre Load Degressivity Exploitation
Pneumatic tyres exhibit non-linear vertical load degressivity ($\partial^2 F_y / \partial F_z^2 < 0$, Pacejka 2012):
- Increasing roll stiffness at the **front axle** increases lateral load transfer $\Delta F_{z,f}$ between front-left and front-right wheels.
- Because of degressivity, the gain in $F_y$ on the outside tyre is smaller than the loss of $F_y$ on the unloaded inside tyre.
- Consequently, total front axle cornering force decreases, introducing **stabilizing understeer** and shedding dangerous yaw energy without requiring friction brake intervention.

### 4.2 Lateral Load Transfer Ratio ($LTR$) Rollover Metric
$$LTR = \frac{|F_{z,\text{left}} - F_{z,\text{right}}|}{F_{z,\text{left}} + F_{z,\text{right}}}$$
- $LTR = 0$: Perfect lateral balance
- $LTR = 1.0$: Impending rollover (complete inner-wheel lift)
- The ARSDC controller actively clamps $LTR < 0.70$, maintaining continuous four-wheel road contact in 80 km/h dynamic fishhook maneuvers (Rajamani 2012, Sec. 8.4).

---

## 5. Source Code Cross-Reference

| Controller | Class Name | Source File | Unit Tests |
| :--- | :--- | :--- | :--- |
| **Integrated IVMC** | `IntegratedChassisController` | [integrated_controller.py](../src/vehicle_model/controllers/integrated_controller.py) | [test_chassis_controllers.py](../tests/vehicle_model/test_chassis_controllers.py) |
| **4WS Coordinator** | `FourWheelSteeringController` | [four_wheel_steering.py](../src/vehicle_model/controllers/four_wheel_steering.py) | [test_rear_steering_lut.py](../tests/vehicle_model/test_rear_steering_lut.py) |
| **Phase-Plane Envelope** | `PhasePlaneEnvelopeController` | [phase_plane.py](../src/vehicle_model/controllers/phase_plane.py) | [test_phase_plane.py](../tests/vehicle_model/test_phase_plane.py) |
| **Active Roll Stiffness** | `ActiveRollController` | [active_roll.py](../src/vehicle_model/controllers/active_roll.py) | [test_active_roll.py](../tests/vehicle_model/test_active_roll.py) |

---

## 6. Literature & Textbooks References

For complete bibliographic details, ISBNs, and full citations, refer to the master [REFERENCES.md](REFERENCES.md):
- **Küçükay, F. (2022):** *Grundlagen der Fahrzeugtechnik: Antriebe, Getriebe, Energieverbrauch, Bremsen, Fahrdynamik, Fahrkomfort*. Springer Vieweg.
- **Mitschke, M.; Wallentowitz, H. (2014):** *Dynamik der Kraftfahrzeuge*. 5. Aufl., Springer Vieweg.
- **Heißing, B.; Ersoy, M.; Gies, S. (2020):** *Fahrwerkhandbuch: Grundlagen, Fahrdynamik, Komponenten, Systeme, Mechatronik, Perspektiven*. 5. Aufl., Springer Vieweg.
- **Isermann, R. (2010):** *Fahrdynamik-Regelung: Modellbildung, Fahrerassistenzsysteme, Mechatronik*. Vieweg+Teubner.
- **Rajamani, R. (2012):** *Vehicle Dynamics and Control*. 2nd Ed., Springer.
- **Lunze, J. (2020):** *Regelungstechnik 1 & 2*. Springer Vieweg.

