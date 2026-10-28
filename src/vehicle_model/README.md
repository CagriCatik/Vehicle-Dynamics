# Vehicle Model Source Package (`src/vehicle_model`)

This package contains the consolidated, flattened core Python implementation for non-linear vehicle dynamics, multi-body kinematics, interchangeable tyre modeling, state estimation, and integrated chassis control conforming to [Plan.md](../../Plan.md).

---

## Package Architecture & Subsystems

```text
src/vehicle_model/
├── core/                      # Data structures: SevenDOFParams, SevenDOFState, Telemetry
├── vehicle_models/            # Physics plants:
│   ├── seven_dof.py           # 7-DOF two-track model (vx, vy, r, omega_FL..RR, dynamic load transfer)
│   ├── fourteen_dof.py        # 14-DOF full vehicle (surge, sway, yaw, heave, pitch, roll, 4 hops, 4 spins)
│   ├── dynamic_single_track.py# Non-linear dynamic bicycle model
│   └── kinematic_single_track.py # Kinematic bicycle model
│
├── tire_model/                # Interchangeable tyre framework with explicit provenance:
│   ├── linear.py              # Linear small-slip model with load degressivity
│   ├── fiala.py               # Fiala analytical brush tyre model
│   ├── simplified_combined.py # Normalised slip vector model bounded by Kamm's circle
│   └── pacejka.py             # Magic Formula MF 6.2 with longitudinal-lateral coupling
│
├── controllers/               # Control algorithms & chassis arbitration:
│   ├── integrated_controller.py # Master IVMC (4WS + TV + ABS + TCS + Phase-Plane)
│   ├── driver_model.py        # Closed-loop neuromuscular preview driver model (Salvucci / MacAdam)
│   ├── dynamic_two_track_nmpc.py # Dynamic Two-Track NMPC with combined-slip Kamm circle constraints
│   ├── phase_plane.py         # Nonlinear (beta, beta_dot) stability boundary & envelope controller
│   ├── active_roll.py         # 14-DOF Active Roll Stiffness Distribution Controller (ARSDC)
│   ├── torque_vectoring.py    # Direct yaw moment controller and wheel torque allocator
│   ├── abs.py                 # Antilock Braking System (slip-ratio threshold modulation)
│   ├── tcs.py                 # Traction Control System (drive-slip regulation)
│   ├── nmpc/                  # C/GMRES Continuation-based Nonlinear MPC path tracker
│   └── framework.py           # Unified VehicleController protocol & baseline wrappers
│
├── steering/                  # Steering dynamics & multi-axle coordination:
│   ├── four_wheel_steering.py # 4WS coordinator (counter-phase agility & in-phase stability)
│   └── lut.py                 # 2D bilinear lookup-table rear-axle steering model (lut.json)
│
├── actuators/                 # Hardware actuator dynamics & fault modeling:
│   ├── steering_actuator.py   # EPS steering with first-order lag, rate limit, and transport delay
│   ├── electric_powertrain.py # 4-motor EV actuator with field-weakening and battery limits
│   └── brake_by_wire.py       # Asymmetric hydraulic brake dynamics & regen blending
│
├── suspension/                # Vertical dynamics & ride control:
│   ├── quarter_car.py         # 2-DOF quarter-car sprung & unsprung dynamics
│   └── semi_active.py         # Continuous Skyhook and Groundhook suspension controllers
│
├── estimation/                # State observers & offline tuning:
│   ├── ekf_observer.py        # Extended Kalman Filter for body sideslip angle and IMU biases
│   ├── friction_estimator.py  # Recursive Least Squares (RLS) road-friction estimator
│   ├── sensor_suite.py        # Sensor corruption model (IMU white noise, bias drift, quantization)
│   └── pso_optimizer.py       # Particle Swarm Optimization for offline controller tuning
│
├── simulation/                # Scenario simulation & automated validation:
│   ├── scenario_runner.py     # Deterministic RK4 multi-step maneuver simulation engine
│   ├── validation_matrix.py   # 9-scenario automated virtual testing matrix runner
│   ├── kpi.py                 # Standardized automotive KPI evaluation engine
│   └── report_generator.py    # Traceable Markdown report and HTML dashboard compiler
│
├── roads/                     # Road surface modeling (Uniform, Split-mu, Spatial friction patches)
├── visualization/             # 2D Plan-view vehicle rendering (body, 4 steerable wheels, snapshots)
└── logging.py                 # Centralized timestamped logging infrastructure
```

---

## Testing & Verification

Run the full automated pytest suite (142 tests across 20 test modules):
```bash
pytest tests/vehicle_model -v
```

**Results:** **142 passed, 0 failed**. 100% of multi-body physics, force conservation, state estimation, closed-loop E2E scenarios, and chassis control tests pass deterministically.

### End-to-End (E2E) Benchmarking & Algorithmic Improvements
Comprehensive closed-loop benchmarking is executed via:
```bash
python examples/e2e_deep_analysis_and_benchmarking.py
```
Outputs and telemetry figures are compiled in [output/e2e_analysis/](../../output/e2e_analysis/):
1. **Driver-in-the-Loop Split-$\mu$ Surface Transition**: Calibrated cerebellar forward prediction $(\hat{x}, \hat{y}, \hat{\psi})$ compensating for human cognitive latency ($\tau_d = 180\text{ ms}$) and muscle lag ($\tau_n = 100\text{ ms}$), completely eliminating 1 Hz pilot-induced oscillations (PIO).
2. **Dynamic Two-Track NMPC High-Speed Corridor Evasion (90 km/h)**: Combines zero-sideslip 4WS coordination with terminal Lyapunov weights, enforcing strict adherence to individual tire Kamm friction circle boundaries ($\rho \le 1.03$).
3. **14-DOF Dynamic NHTSA Fishhook Rollover Suppression**: Calibrated Active Roll Stiffness Distribution Controller ($K_\phi = 45,000\text{ N}\cdot\text{m/rad}, C_\phi = 4,500\text{ N}\cdot\text{m/(rad/s)}$) suppressing dynamic chassis roll by **31.5%** and preventing tire lift-off ($\min(F_z) > 1900\text{ N}$).
4. **Combined-Slip Lift-Off Oversteer Phase-Plane Recovery**: Dynamic friction circle contraction $\mu_{\text{lat}} = \mu \sqrt{1 - (a_x/\mu g)^2}$ under trail braking, coupled with stabilizing differential braking and active counter-steer, achieving **53.5% sideslip envelope containment** and preventing spin-out.
