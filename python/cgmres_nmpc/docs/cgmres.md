# CGMRES and NMPC — Algorithm Documentation

This document explains the theoretical background of Nonlinear Model Predictive Control (NMPC) and the Continuation/GMRES (C/GMRES) method, then maps the theory onto the actual code in this project.

---

## 1. What is Model Predictive Control (MPC)?

Model Predictive Control is a control strategy that repeatedly solves an optimization problem online to find the best sequence of control inputs.  The key idea is:

> **At each time step, look N steps into the future, find the optimal plan, apply the first step, then repeat.**

This is called the **receding horizon** principle.

### 1.1 The basic MPC loop

```
┌──────────────────────────────────────────────────┐
│  At time t, with current state x(t):             │
│                                                  │
│  1. PREDICT: simulate N steps ahead using model  │
│  2. OPTIMISE: find inputs u* that minimise cost  │
│  3. APPLY: send only u*[0] to the actuator       │
│  4. WAIT for dt seconds                          │
│  5. MEASURE new state x(t+dt)                    │
│  6. GO TO 1                                      │
└──────────────────────────────────────────────────┘
```

The advantage over a simple PID controller is that MPC can **see ahead**: it knows the road curves to the left in 2 seconds, so it starts steering now, rather than reacting after the fact.

---

## 2. From linear MPC to Nonlinear MPC (NMPC)

**Linear MPC** assumes the vehicle model and cost function are quadratic. The optimization problem then has a closed-form solution (a matrix equation).

**Nonlinear MPC (NMPC)** removes that assumption.  The vehicle model is a nonlinear function (trigonometric terms, velocity-dependent yaw rate), and the cost can include nonlinear terms like obstacle barriers.  The optimization must be solved numerically, which is slower but can handle real vehicle dynamics.

---

## 3. The Continuation/GMRES (C/GMRES) method

C/GMRES is a specific algorithm designed to solve the NMPC optimization problem extremely fast — fast enough for real-time control.

### 3.1 The key insight

Instead of solving the optimal control problem **from scratch** at every time step, C/GMRES observes:

> The optimal solution at time t+dt is usually very close to the optimal solution at time t.

C/GMRES treats the optimal solution as a **continuously evolving quantity** and uses Newton's method (specifically, GMRES as its linear solver) to track it as time flows forward.

### 3.2 The mathematical structure

Define the **Hamiltonian** of the optimal control problem:

```
H(x, u, λ) = ℓ(x, u) + λᵀ · f(x, u)
```

where:

- `ℓ(x, u)` is the running cost (tracking error + input effort)
- `λ` is the co-state (Lagrange multiplier, analogous to a shadow price)
- `f(x, u)` is the vehicle dynamics (state derivative)

The optimality condition (Pontryagin's minimum principle) says that at the optimum:

```
∂H/∂u = 0    (for all steps in the horizon)
```

This is a system of equations `F(U, t) = 0` where `U` is the full input sequence vector.

### 3.3 The continuation equation

Differentiating `F(U, t) = 0` with respect to time:

```
∂F/∂U · dU/dt + ∂F/∂t = 0
```

Rearranging:

```
dU/dt = -(∂F/∂U)⁻¹ · ∂F/∂t
```

C/GMRES solves this linear system at each step using **GMRES** (Generalized Minimal Residual), an iterative Krylov-subspace method that avoids explicitly forming the Jacobian matrix.  The solution `dU/dt` tells us how to update the input sequence from one step to the next.

### 3.4 The stabilised variant (C/GMRES with regularisation)

In practice a stabilising term is added:

```
dU/dt = -(∂F/∂U)⁻¹ · (∂F/∂t + ζ · F)
```

The term `ζ · F` drives `F` toward zero if it drifts due to model mismatch or numerical errors.  `ζ` is the `zeta` parameter in `config.yaml`.

---

## 4. This implementation

This project uses **NMPC solved with scipy SLSQP** rather than the original C/GMRES continuation method.  The class is named `NMPCControllerCGMRES` to preserve interface compatibility with the original design, but the solver is a general constrained nonlinear optimizer.

The approach is sometimes called **direct multiple shooting** NMPC:

### 4.1 Decision variables

At each control step, the optimizer searches over a 2N-dimensional vector:

```
U = [u_a(0), u_a(1), ..., u_a(N-1),   ← N acceleration commands
     u_sf(0), u_sf(1), ..., u_sf(N-1)] ← N front steer commands
```

### 4.2 Cost function

The cost function `J(U)` rolls out the bicycle model N steps and accumulates:

```
J(U) = Σᵢ₌₀ᴺ⁻¹ [
    Q_Y   · (yᵢ - y_ref(xᵢ))²        ← lateral tracking
  + Q_YAW · angle_diff(yawᵢ, yaw_ref(xᵢ))²  ← heading tracking
  + Q_V   · (vᵢ - v_ref)²             ← speed tracking
  + Σⱼ 80·r_safe²/(dⱼ² + ε)          ← obstacle barrier
]
+ R_steer · ||u_sf||²                  ← steer effort
+ R_accel · ||u_a||²                   ← accel effort
```

**Cost weights from config.yaml:**

| Weight | Value | What it penalises |
|--------|-------|-------------------|
| `Q_Y = 25` | lateral error squared |
| `Q_YAW = 3` | heading error squared |
| `Q_V = 20` | speed error squared |
| `R_steer = 0.05` | sum of squared steer inputs |
| `R_accel = 0.01` | sum of squared accel inputs |

Higher Q → tighter tracking of that state.  Higher R → smoother, smaller inputs.

### 4.3 Constraints

```
-U_A_MAX    ≤ u_a(i)  ≤ U_A_MAX      for i = 0..N-1
-U_STEER_MAX ≤ u_sf(i) ≤ U_STEER_MAX  for i = 0..N-1
```

These are box constraints enforced by SLSQP's `bounds` argument.

### 4.4 Prediction model

Inside the cost rollout, the bicycle kinematic model is integrated using forward Euler:

```
xᵢ₊₁   = xᵢ   + dt · vᵢ · cos(yawᵢ)
yᵢ₊₁   = yᵢ   + dt · vᵢ · sin(yawᵢ)
yawᵢ₊₁ = yawᵢ + dt · (vᵢ/L) · (sin(δ_f) - sin(δ_r))
vᵢ₊₁   = clip(vᵢ + dt · u_a, 0, 10)
```

where `δ_r` is obtained from the LUT at each step.

### 4.5 Variable step size dt

The prediction step size `dt` grows from near-zero at startup:

```
dt(t) = tf · (1 - exp(-α · t)) / N
```

This ramp prevents the controller from making aggressive early moves based on a very short (near-zero-length) prediction.  At steady state (`t >> 1/α`), `dt → tf/N`.

---

## 5. The reference path

The reference is **spatial** (a function of x-position), not temporal.

```
y_ref(x)   = A · sin(2π · (x - x₀) / λ)
yaw_ref(x) = atan2(dy/dx, 1) = atan2(A · (2π/λ) · cos(2π·(x-x₀)/λ), 1)
```

Parameters from `config.yaml`:

- **Amplitude** A = 1.5 m (peak lateral deviation)
- **Wavelength** λ = 16.67 m (derived from 2.5 m/s ÷ 0.15 Hz)

The spatial formulation means the slalom shape looks the same regardless of speed.  If the reference were temporal (`y(t) = A·sin(2π·f·t)`), driving faster would stretch the slalom into longer wavelength, distorting the intended path shape.

---

## 6. The Ackermann vehicle model with rear-axle steering

### 6.1 Ackermann steering geometry

Ackermann steering is the geometry used in standard wheeled vehicles.  It ensures that when cornering, all wheels roll on concentric circles around the same instantaneous centre of rotation — minimising tyre slip.  The kinematic bicycle model abstracts this to a single front wheel and a single rear wheel on the centreline.

This vehicle is **not a conventional 4-wheel-steer (4WS)** system where both axles are independently commanded.  Instead, the rear axle is **electronically slaved** to the front axle: given a front steer command and the current speed, the vehicle hardware produces a rear steer angle defined by a pre-calibrated lookup table.  The MPC only optimises over **front steer and acceleration** — rear steer is a consequence, not a decision.

### 6.2 Bicycle model yaw rate with rear-axle steering

For a front-steer-only vehicle the yaw rate is:

```
ψ̇ = (v / L) · tan(δ_f)   ≈ (v / L) · δ_f   (small angle)
```

With the slaved rear-axle steer included:

```
ψ̇ = (v / L) · (sin(δ_f) - sin(δ_r))
```

The sin formulation (rather than tan) was chosen to match the original LUT calibration convention.

**Effect of rear steer on yaw rate:**

- `δ_r` has the same sign as `δ_f` → **in-phase** → yaw rate increases → tighter turning radius (low speed agility)
- `δ_r` has the opposite sign → **counter-phase** → yaw rate decreases → more stable at high speed

### 6.3 The rear-steer lookup table

The LUT encodes the calibrated rear-axle behaviour of the physical vehicle:

| Speed [m/s] | Front steer −30° | 0° | +30° |
|-------------|------------------|----|------|
| 0           | +8° (in-phase)   | 0° | −8°  |
| 5           | +7.5°            | 0° | −7.5°|
| 10          | +1.5°            | 0° | −1.5°|
| 20          | −2.4° (counter)  | 0° | +2.4°|

At low speed the rear steer amplifies rotation; at high speed it opposes the front steer for straight-line stability.  Bilinear interpolation is used between breakpoints; values outside the table range are flat-extrapolated.

---

## 7. Warm starting

At each control step, the previous SLSQP solution is used as the initial guess:

```python
u_guess = np.concatenate([self.u1, self.u2])
res = minimize(_mpc_cost, u_guess, ...)
```

This works well because the optimal input sequence changes smoothly from one step to the next.  The improvement over a cold start (random or zero initial guess) is typically 50–70% fewer solver iterations per step.

---

## 8. Obstacle avoidance

Obstacles are handled via a **log-barrier** penalty term:

```
obs_cost = 80 · r_safe² / (d² + ε)
```

where `d` is the Euclidean distance from the predicted vehicle position to the obstacle centre, and `r_safe = r_obstacle + r_vehicle` is the minimum safe distance.

The `80 · r_safe²` numerator scales the barrier intensity with the obstacle size.  The barrier is only activated when `d < 4 · r_safe` to avoid distorting the cost landscape far from obstacles.

This is called a **soft constraint** — the optimizer will try very hard to avoid penetrating the barrier but is not mathematically forbidden from doing so.  A truly hard constraint would require constraint-specific handling in SLSQP.

---

## 9. Glossary

| Term | Meaning |
|------|---------|
| **NMPC** | Nonlinear Model Predictive Control |
| **C/GMRES** | Continuation / Generalized Minimal Residual method |
| **SLSQP** | Sequential Least Squares Programming (the scipy solver used here) |
| **Horizon N** | Number of prediction steps |
| **tf** | Total prediction window duration at steady state [s] |
| **α (alpha)** | Time constant for the dt ramp-up |
| **ζ (zeta)** | Stabilisation gain for the C/GMRES continuation term |
| **Receding horizon** | Principle of re-solving at every step and applying only the first command |
| **Warm start** | Using the previous solution as the initial guess for the current solve |
| **4WS** | 4-Wheel Steering — both front and rear axles steer |
| **LUT** | Lookup Table — a pre-computed function stored as a grid |
| **GMRES** | Generalized Minimal Residual — an iterative solver for linear systems |
| **Hamiltonian** | Function combining cost and dynamics used in optimal control theory |
| **Co-state (λ)** | Adjoint variable; encodes sensitivity of future cost to current state |

---

## 10. Further reading

- Ohtsuka, T. (2004). *A continuation/GMRES method for fast computation of nonlinear receding horizon control.* Automatica, 40(4), 563–574.
- Rawlings, J. B., Mayne, D. Q., & Diehl, M. (2017). *Model Predictive Control: Theory, Computation, and Design.* Nob Hill Publishing.
- Rajamani, R. (2011). *Vehicle Dynamics and Control.* Springer. (Chapter on kinematic bicycle model)
- scipy.optimize.minimize documentation: [https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.minimize.html](https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.minimize.html)
