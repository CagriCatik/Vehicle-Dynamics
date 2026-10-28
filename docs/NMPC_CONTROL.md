# Non-Linear Model Predictive Control (NMPC) Suite

Model Predictive Control (MPC) solves an optimal control problem repeatedly over a future finite prediction horizon, computing the optimal sequence of control inputs and applying only the first step before updating the state estimate.

The platform implements two complementary NMPC architectures:
1. **Continuation / GMRES (C/GMRES) NMPC Path Tracker** (`NMPCControllerCGMRES` & `NMPCPathTrackingController`)
2. **Dynamic Two-Track NMPC with 4-Wheel Kamm Friction Bounds** (`DynamicTwoTrackNMPC`)

---

## 1. Continuation / GMRES (C/GMRES) Real-Time Path Tracker

The C/GMRES algorithm (developed by T. Ohtsuka) solves the non-linear Karush-Kuhn-Tucker (KKT) optimality conditions in real-time by tracking the solution trajectory continuously through time rather than solving a non-linear program (NLP) from scratch at each step.

### 1.1 Mathematical Formulation

Let the system dynamics and constraints be:
$$\dot{\mathbf{x}}(t) = \mathbf{f}(\mathbf{x}(t), \mathbf{u}(t))$$
$$\mathbf{C}(\mathbf{x}(t), \mathbf{u}(t)) \le \mathbf{0}$$

Over the horizon $\tau \in [0, T_f]$, the optimal control problem minimizes the objective:
$$J = \Phi(\mathbf{x}(t + T_f)) + \int_t^{t + T_f} L(\mathbf{x}(\tau), \mathbf{u}(\tau)) d\tau$$

#### Hamiltonian Formulation
$$H(\mathbf{x}, \mathbf{u}, \boldsymbol{\lambda}) = L(\mathbf{x}, \mathbf{u}) + \boldsymbol{\lambda}^T \mathbf{f}(\mathbf{x}, \mathbf{u})$$

The Euler-Lagrange optimality conditions yield:
$$\dot{\boldsymbol{\lambda}} = -\left(\frac{\partial H}{\partial \mathbf{x}}\right)^T, \quad \frac{\partial H}{\partial \mathbf{u}} = \mathbf{0}, \quad \boldsymbol{\lambda}(t + T_f) = \left(\frac{\partial \Phi}{\partial \mathbf{x}}\right)^T$$

### 1.2 Continuation Method & GMRES Linear Solver
Instead of iteratively solving $\mathbf{F}(\mathbf{U}, \mathbf{x}, t) = \mathbf{0}$ at each step, C/GMRES differentiates the optimality condition with respect to real time $t$:
$$\dot{\mathbf{F}} = \frac{\partial \mathbf{F}}{\partial \mathbf{U}} \dot{\mathbf{U}} + \frac{\partial \mathbf{F}}{\partial \mathbf{x}} \dot{\mathbf{x}} + \frac{\partial \mathbf{F}}{\partial t} = -\zeta \mathbf{F}$$

This transforms the non-linear optimization problem into a linear equation in $\dot{\mathbf{U}}$:
$$\frac{\partial \mathbf{F}}{\partial \mathbf{U}} \dot{\mathbf{U}} = -\zeta \mathbf{F} - \frac{\partial \mathbf{F}}{\partial \mathbf{x}} \dot{\mathbf{x}} - \frac{\partial \mathbf{F}}{\partial t}$$

The **Generalized Minimal Residual (GMRES)** algorithm solves this linear system efficiently using forward-difference Jacobian-vector products without computing or inverting the full Hessian matrix, achieving millisecond-level update rates.

### 1.3 Cost Function & Trajectory Tracking
In [nmpc.py](../src/vehicle_model/controllers/nmpc.py), the stage cost penalizes lateral path deviation, heading error, velocity error, and control action:
$$L = q_y (y - y_{\text{ref}})^2 + q_{\psi} (\psi - \psi_{\text{ref}})^2 + q_v (v - v_{\text{ref}})^2 + r_{\delta} u_{\delta}^2 + r_a u_a^2$$

#### 4WS Rear Steer LUT Integration
The front steer command computed by C/GMRES is seamlessly passed to the calibrated rear-axle lookup table:
$$\delta_r = \operatorname{LUT}(v_x, \delta_{f,\text{opt}})$$
Providing coordinated 4WS yaw damping and zero-sideslip crab translation during high-speed trajectory tracking.

---

## 2. Dynamic Two-Track NMPC with 4-Wheel Kamm Friction Bounds

While kinematic models suffice at low lateral acceleration ($a_y < 0.3g$), high-speed obstacle avoidance and split-$\mu$ maneuvers demand an NMPC controller that explicitly accounts for dynamic tyre slip, dynamic load transfer, and friction circle limits.

The `DynamicTwoTrackNMPC` controller ([dynamic_two_track_nmpc.py](../src/vehicle_model/controllers/dynamic_two_track_nmpc.py)) implements predictive control directly over the non-linear two-track chassis.

### 2.1 State & Control Vectors
- **Plant State ($n_x = 6$):**
  $$\mathbf{x} = \begin{bmatrix} X & Y & \psi & v_x & v_y & r \end{bmatrix}^T$$
- **Control Input ($n_u = 4$):**
  $$\mathbf{u} = \begin{bmatrix} \delta_f & \delta_r & M_{z,\text{tv}} & a_{x,\text{req}} \end{bmatrix}^T$$

Where:
- $\delta_f, \delta_r$: Front and rear steer angles [rad]
- $M_{z,\text{tv}}$: Direct yaw moment from active torque vectoring [Nm]
- $a_{x,\text{req}}$: Longitudinal chassis acceleration demand [$\text{m/s}^2$]

### 2.2 Discrete Integration & Runge-Kutta 4th-Order (RK4)
$$\mathbf{x}_{k+1} = \mathbf{x}_k + \frac{\Delta t}{6}(k_1 + 2 k_2 + 2 k_3 + k_4)$$
Where each stage evaluates the non-linear continuous dynamics with individual wheel slips $\alpha_i, \kappa_i$ and load transfer $F_{z,i}$.

### 2.3 Strict Kamm's Friction Circle Boundary Constraints
At every prediction step $k \in \{0, \dots, N-1\}$ and for every wheel $i \in \{FL, FR, RL, RR\}$, the total horizontal tyre force must remain inside the friction circle:

$$\rho_{i,k} = \frac{\sqrt{F_{x,i,k}^2 + F_{y,i,k}^2}}{\mu_i F_{z,i,k}} \le 1.0$$

The controller includes penalty barrier functions in the optimization cost:
$$J_{\text{kamm}} = w_{\text{kamm}} \sum_{k=0}^{N-1} \sum_{i=1}^4 \max\left(0, \rho_{i,k} - 1.0\right)^2$$

### 2.4 Multi-Objective Horizon Cost
The total objective function over prediction horizon $N$ is:
$$J = \sum_{k=1}^N \left[ \|\mathbf{x}_k - \mathbf{x}_{\text{target}}\|_{\mathbf{Q}}^2 + \|\mathbf{u}_{k-1}\|_{\mathbf{R}}^2 + \|\Delta \mathbf{u}_{k-1}\|_{\mathbf{S}}^2 \right] + J_{\text{kamm}}$$

Where:
- $\mathbf{Q} = \operatorname{diag}(0, q_y, q_\psi, q_v, q_\beta, q_r)$: State tracking weights
- $\mathbf{R} = \operatorname{diag}(r_{\delta f}, r_{\delta r}, r_{Mz}, r_{ax})$: Control effort weights
- $\mathbf{S}$: Slew-rate penalty weights ensuring smooth actuator demands

---

## 3. Comparative Benchmark: C/GMRES Slalom Path Tracking

In the standardized 20 FPS animation benchmark (`cgmres_nmpc_slalom`), the controllers were evaluated on a high-speed slalom course ($\lambda = 35\text{ m}$, $A = 1.6\text{ m}$, $v_x = 36\text{ km/h}$):

| Controller Configuration | Path Tracking Performance | Peak Sideslip Angle ($\beta$) | Slalom Cones Cleared |
| :--- | :--- | :---: | :---: |
| **Passive Baseline (Open-Loop Driver)** | Drifts off centerline ($Y > 9\text{ m}$) | $3.01^\circ$ | Failed (Collided) |
| **NMPC Front-Steer Only (FWS)** | Accurate tracking ($|e_y| < 0.25\text{ m}$) | $4.24^\circ$ | 100% Cleared |
| **NMPC + Coordinated 4WS LUT** | Accurate tracking ($|e_y| < 0.18\text{ m}$) | **$2.73^\circ$ (35% reduction)** | 100% Cleared |

---

## 4. Source Code Cross-Reference

| Module | Class Name | Source File | Unit Tests |
| :--- | :--- | :--- | :--- |
| **C/GMRES Algorithm** | `NMPCControllerCGMRES` | [nmpc.py](../src/vehicle_model/controllers/nmpc.py) | [test_nmpc_controller.py](../tests/vehicle_model/test_nmpc_controller.py) |
| **NMPC Vehicle Controller** | `NMPCPathTrackingController` | [nmpc.py](../src/vehicle_model/controllers/nmpc.py) | [test_nmpc_controller.py](../tests/vehicle_model/test_nmpc_controller.py) |
| **Dynamic Two-Track NMPC** | `DynamicTwoTrackNMPC` | [dynamic_two_track_nmpc.py](../src/vehicle_model/controllers/dynamic_two_track_nmpc.py) | [test_dynamic_two_track_nmpc.py](../tests/vehicle_model/test_dynamic_two_track_nmpc.py) |
