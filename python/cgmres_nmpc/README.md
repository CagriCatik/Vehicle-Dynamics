# CGMRES-NMPC Vehicle Path Tracking Simulator

A real-time **Nonlinear Model Predictive Control (NMPC)** simulation for autonomous vehicle path tracking using a kinematic bicycle model of an Ackermann-steering vehicle with LUT-based rear-axle steering.

---

## What this project does

The simulator drives a model vehicle along a sinusoidal slalom path using NMPC.  At every time step the controller:

1. Predicts the vehicle's future trajectory over a short horizon (N steps).
2. Optimises the steering and acceleration commands to minimise lateral error, heading error, and speed error while avoiding obstacles.
3. Applies only the first command from the optimal sequence, then repeats (receding-horizon principle).

An interactive dashboard shows the result in real time.

---

## Algorithm overview

| Aspect | Detail |
| --- | --- |
| **Control method** | Nonlinear MPC via direct multiple shooting |
| **Solver** | scipy SLSQP (Sequential Least Squares Programming) |
| **Vehicle model** | Kinematic bicycle (Ackermann geometry) |
| **Rear-axle steer** | Velocity-dependent lookup table — slaved to front steer, not a free input |
| **Reference path** | Sinusoidal slalom — spatial, not temporal |
| **Horizon** | N = 8 steps, tf = 1.5 s steady-state prediction |
| **Warm starting** | Previous optimal solution reused as initial guess |

See [docs/cgmres.md](docs/cgmres.md) for a full explanation of the algorithm and mathematical formulation.

---

## Project structure

```text
CGMRES_NMPC/
├── main.py                   Entry point — launches the live dashboard
├── config.yaml               All tuning parameters and vehicle specs
├── requirements.txt          Python dependencies
├── data/
│   └── lut.json              LUT data: f(velocity, front_steer) → rear_steer [deg]
├── docs/
│   └── cgmres.md             Algorithm documentation and theory
└── src/
    ├── config.py             Loads config.yaml and exposes constants
    ├── models/
    │   ├── lut.py            Rear-axle steering lookup table loader
    │   └── vehicle.py        Kinematic bicycle model + state history
    ├── controllers/
    │   └── cgmres.py         NMPC controller (OCP setup, cost, solver)
    └── utils/
        ├── dashboard.py      Real-time matplotlib dashboard
        └── visualization.py  Legacy plot utilities (not used in main flow)
```

---

## Requirements

- Python 3.9 or newer
- Dependencies listed in `requirements.txt`:

```text
matplotlib
numpy
scipy
PyYAML
```

---

## Installation

```bash
# Clone or download the project
cd CGMRES_NMPC

# Create and activate a virtual environment (recommended)
python -m venv venv

# Windows
venv\Scripts\activate

# macOS / Linux
source venv/bin/activate

# Install dependencies
pip install -r requirements.txt
```

---

## Running the simulation

### Interactive dashboard

```bash
python main.py
```

The dashboard opens with:

- **Left panel**: Bird's-eye map with the vehicle, reference path, and trajectory.
- **Right panels**: Strip charts for velocity, front steer, rear steer, and acceleration.
- **Bottom slider**: Drag to change the target speed (0.5 – 5.0 m/s) while the simulation runs.

### Headless test (no GUI)

```bash
python -m pytest -q tests/cgmres_nmpc
```

Runs 200 steps and prints a table of position, lateral error, heading, speed, and steer angle every 20 steps.  Useful for quick sanity checks without a display.

---

## Configuration

All parameters live in `config.yaml`.  Key sections:

```yaml
simulation:
  dt: 0.05           # time step [s]
  iteration_time: 120.0  # total simulation time [s]

initial_state:
  x: 0.0             # starting position [m]
  y: 0.0
  yaw_deg: 0.0       # starting heading [deg]
  v: 0.05            # starting speed [m/s]

vehicle:
  wheelbase: 0.60          # axle-to-axle distance [m]
  max_accel: 4.5           # actuator limit [m/s²]
  max_steer_front_deg: 25  # steering lock limit [deg]
  radius: 0.30             # collision radius [m]

trajectory:
  target_v: 2.5    # desired speed [m/s]
  amplitude: 1.5   # slalom lateral swing [m]
  frequency: 0.15  # slalom spatial frequency [Hz]

controller:
  N: 8             # prediction horizon (steps)
  tf: 1.5          # steady-state prediction window [s]
  alpha: 0.5       # dt ramp-up time constant
  q_y: 25.0        # lateral error weight
  q_yaw: 3.0       # heading error weight
  q_v: 20.0        # speed error weight

obstacles: []      # add [x, y, radius] entries to place cones
```

### Adding obstacles

```yaml
obstacles:
  - [5.0, 0.8, 0.15]   # cone at x=5, y=0.8, radius=0.15 m
  - [10.0, -0.5, 0.15]
```

---

## Tuning guide

| Parameter | Effect of increasing |
| --- | --- |
| `q_y` | Tighter lateral tracking; may increase steering activity |
| `q_yaw` | Smoother heading; helps at high speed |
| `q_v` | Faster speed convergence; may conflict with lateral tracking |
| `N` | Longer look-ahead; better anticipation but slower solve |
| `tf` | Wider prediction window; smoother at high speed |
| `amplitude` | Tighter slalom; harder to track — reduce `target_v` if tracking degrades |

---

## Key design decisions

**Spatial reference, not temporal** — the reference path is defined as y(x), not y(t).  This means the slalom shape is fixed in space; the vehicle tracks through the same physical path regardless of speed.

**Rear-axle steering via LUT** — this is an Ackermann-steering vehicle, not a conventional 4WS system.  The rear steer angle is not a free control input; the MPC only optimises front steer and acceleration.  The rear angle is derived from a pre-calibrated table that maps (speed, front_steer) → rear_steer, mirroring real hardware where the rear axle is electronically slaved to the front.

**Variable horizon dt** — the per-step prediction time grows from near-zero at startup to `tf/N` at steady state (`dt = tf*(1 - exp(-alpha*t))/N`).  This prevents large transients before the warm-start has converged.

**Warm starting** — the previous SLSQP solution is shifted and reused as the initial guess.  This cuts solver iterations by roughly 50–70% compared to a cold start.
