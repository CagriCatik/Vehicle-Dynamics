# Vehicle Model

Python vehicle-dynamics package for single-track vehicle models, lateral tire-force evaluation, and validation demos.

## Contents

- `vehicle_models/`: kinematic and dynamic single-track models.
- `vehicle_models/config/vehicle.py`: YAML-backed typed vehicle parameters.
- `vehicle_models/integration.py`: shared RK4 integration helpers.
- `config/default.yaml`: default vehicle, validation, visualization, and tire-demo settings.
- `tire_model/`: lateral Magic Formula tire model and tire-parameter data.
- `data/`: example vehicle run and TMS track boundaries used by validation demos.

## Setup

Run these commands from the repository root:

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install -r python\vehicle_model\requirements.txt
python -m pip install -e .
```

On macOS/Linux:

```bash
python -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r python/vehicle_model/requirements.txt
python -m pip install -e .
```

The editable install makes `vehicle_model` importable from any working
directory inside the virtual environment.

## Quick Check

```powershell
python -m pytest -q tests\vehicle_model
```

Expected result:

```text
8 passed
```

The repository-level test suite also includes visualization and controller smoke tests:

```powershell
python -m pytest -q
```

## Basic Usage

```python
import numpy as np

from vehicle_model import DynamicSingleTrack, KinematicSingleTrack

dynamic = DynamicSingleTrack()
state = np.array([0.0, 0.0, 12.0, 0.0, 0.0, 0.02, 0.0])
command = np.array([0.5, 0.0])  # ax [m/s^2], steer_rate [rad/s]
dxdt = dynamic.derivative_eqs(None, state, command, roll=0.0)

kinematic = KinematicSingleTrack()
x, dxdt_hist = kinematic.sim_continuous(
    np.zeros(kinematic.n_states),
    np.array([[1.0, 0.0], [1.0, 0.0]]),
    np.array([0.0, 0.1, 0.2]),
)
```

State conventions:

- Dynamic model state: `[x, y, vx, vy, yaw, steer, yaw_rate]`
- Kinematic model state: `[x, y, yaw, speed, steer]`
- Input vector: `[ax, steer_rate]`
- Units: SI, angles in radians.

## Demos

Lateral tire-force curves:

```powershell
python -m vehicle_model.tire_model.lateral_demo
```

Headless export:

```powershell
python -m vehicle_model.tire_model.lateral_demo --no-show --output-dir output\tire_model
```

Dataset validation animation:

```powershell
python -m vehicle_model.vehicle_models.validation_demo
```

The validation animation includes live telemetry panels for:

- speed and prediction error
- steering input, steering rate, and yaw response
- front/rear lateral tire forces
- longitudinal drive, drag, rolling, and net force
- slip angles, slip ratio, and tire-utilization estimates

Quick non-GUI smoke run:

```powershell
python -m vehicle_model.vehicle_models.validation_demo --max-rows 60 --vx-start 0 --no-show
```

## Configuration

Default settings live in:

```text
python/vehicle_model/config/default.yaml
```

Use an override YAML when you want to change vehicle parameters, validation
defaults, plotting behavior, or tire-demo operating points:

```powershell
python -m vehicle_model.vehicle_models.validation_demo --config path\to\override.yaml --no-show
python -m vehicle_model.tire_model.lateral_demo --config path\to\override.yaml --no-show
```

Overrides are deep-merged into `default.yaml`, so an override can be small:

```yaml
vehicle:
  mass: 900.0
  max_acc: 8.0

validation:
  base_step: 10
  vx_start: 20.0
```

You can also set the same override for a terminal session:

```powershell
$env:VEHICLE_MODEL_CONFIG = "path\to\override.yaml"
```

If you choose not to install the package, run from the repository root and set
`PYTHONPATH` manually for that terminal:

```powershell
$env:PYTHONPATH = "$PWD\python"
```

The validation animation uses the CSV files under `vehicle_model/data`. It can take a moment to load because the example run is several megabytes.

## Notes

- Run modules with `python -m ...` from the repository root. Directly executing files inside package folders will bypass package-relative imports.
- `DynamicSingleTrack` and `KinematicSingleTrack` are the public class names for the single-track models.
- The canonical validation entry point is `vehicle_model.vehicle_models.validation_demo`.
- Roll/bank angle is treated as radians consistently in the dynamic model.
