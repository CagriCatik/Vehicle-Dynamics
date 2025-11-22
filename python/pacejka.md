# Pacejka‑Style Tire Model

## 1. What the script does

`pacejka_tire_model.m` implements a *Pacejka Magic Formula* tire model that can be used in vehicle dynamics studies.  
It calculates:

* **Pure‑slip forces**  
  * Longitudinal force `Fx0(kappa, Fz)`  
  * Lateral force `Fy0(alpha, γ, Fz)`  
* **Combined‑slip forces** (simple cosine weighting)  
  * `Fx(kappa, alpha, Fz, γ)`  
  * `Fy(alpha, kappa, Fz, γ)`  
* **Pneumatic trail and aligning moment**  
  * Trail `t(alpha, kappa, Fz)`  
  * Aligning moment `Mz(alpha, kappa, Fz, γ) = –t·Fy`

All curves are plotted on a black background and exported to PNG files.

---

## 2. The Pacejka Magic Formula

The *Magic Formula* is a semi‑empirical representation of tire forces:

```matlab
F(x) = D * sin( C * atan( B*(x + Sh) - E*(B*(x + Sh) - atan(B*(x + Sh))) ) ) + Sv
```

| Symbol | Meaning | Typical units |
|--------|---------|---------------|
| `x`    | Slip variable (kappa or alpha) | [-] |
| `B`    | Stiffness factor | rad⁻¹ (alpha) / 1 (kappa) |
| `C`    | Shape factor | [-] |
| `D`    | Peak force | N (Fx) / N (Fy) |
| `E`    | Curvature factor | [-] |
| `Sh`   | Horizontal shift | rad (alpha) / 1 (kappa) |
| `Sv`   | Vertical shift | N |

In this script the formula is wrapped in the anonymous function `mf`.  
All force calculations use the same pattern, only the parameters change.

---

## 3. Pure‑slip forces

### 3.1 Longitudinal force `Fx0`

```matlab
B = Kx_small(Fz) / (Cx * μx(Fz)*Fz)      % Bx
C = Cx
D = μx(Fz) * Fz                          % Dx
E = Ex
Sh = 0
Sv = 0
Fx0 = mf(kappa, B, C, D, E, Sh, Sv)
```

* `Kx_small(Fz)` – small‑slip stiffness, load‑dependent (`Kx0 * (Fz/Fz_nom)^Kx_exp`)  
* `μx(Fz)` – peak friction coefficient, linear slope with load

### 3.2 Lateral force `Fy0`

```matlab
B = Ca_small(Fz) / (Cy * μy(Fz)*Fz)      % By
C = Cy
D = μy(Fz) * Fz                          % Dy
E = Ey
Sh = Sh_y_gamma * γ                       % camber shift
Sv = Sv_y_gamma * γ * (Fz/Fz_nom) * Fz     % vertical shift
Fy0 = mf(alpha, B, C, D, E, Sh, Sv)
```

* `Ca_small(Fz)` – cornering stiffness, load‑dependent  
* `μy(Fz)` – peak lateral friction coefficient  
* `Sh_y_gamma` and `Sv_y_gamma` control how camber moves the lateral curve.

---

## 4. Combined‑slip weighting

A simple cosine weighting is applied to the pure‑slip forces:

```matlab
Gx(alpha, Fz) = cos( Cgx * atan( Bgx(Fz) * alpha ) )
Gy(kappa, Fz) = cos( Cgy * atan( Bgy(Fz) * kappa ) )
```

* `Bgx`, `Bgy` – stiffness of the weighting, decrease with load (`B0 * (Fz_nom/Fz)^Bg_exp`)  
* `Cgx`, `Cgy` – shape factors (usually ~1.1)

Combined forces:

```matlab
Fx = Fx0 * Gx(alpha, Fz)
Fy = Fy0 * Gy(kappa, Fz)
```

---

## 5. Trail and aligning moment

Trail is modeled as a cosine function of slip angle and slip ratio:

```matlab
trail_pure(alpha, Fz) = trail0(Fz) * cos( Ct * atan( Bt(Fz) * alpha ) )
trail_comb(alpha, kappa, Fz) = trail_pure(alpha, Fz) *
                               cos( Ct * atan( Gt_kappa * Btk(Fz) * kappa ) )
```

* `trail0(Fz) = t0 * (Fz/Fz_nom)^t_exp`  
* `Bt`, `Btk` – constants (only `t0` and `t_exp` vary with load)

Aligning moment:

```matlab
Mz(alpha, kappa, Fz, γ) = – trail_comb(alpha, kappa, Fz) * Fy(alpha, kappa, Fz, γ)
```

---

## 6. Parameter list

| Section | Parameter | Typical value | Units | Description |
|---------|-----------|---------------|-------|-------------|
| Vehicle | `veh.m` | 1500 | kg | Vehicle mass |
| Vehicle | `veh.static_wdist` | 0.6 | [-] | Front axle load fraction |
| Tire – longitudinal | `par.mu_x0` | 1.00 | [-] | Peak longitudinal friction |
| Tire – longitudinal | `par.Cx` | 1.65 | [-] | Shape factor |
| Tire – longitudinal | `par.Ex` | 0.97 | [-] | Curvature |
| Tire – longitudinal | `par.Kx0` | 110e3 | N | Small‑slip stiffness at nominal load |
| Tire – longitudinal | `par.Kx_exp` | 0.8 | [-] | Load‑sensitivity exponent |
| Tire – longitudinal | `par.mu_x_load_slope` | -0.05 | [-] | Linear load dependence of μx |
| Tire – lateral | `par.mu_y0` | 0.95 | [-] | Peak lateral friction |
| Tire – lateral | `par.Cy` | 1.30 | [-] | Shape factor |
| Tire – lateral | `par.Ey` | 1.00 | [-] | Curvature |
| Tire – lateral | `par.Ca0` | 80e3 | N/rad | Cornering stiffness at nominal load |
| Tire – lateral | `par.Ca_exp` | 0.9 | [-] | Load‑sensitivity exponent |
| Tire – lateral | `par.mu_y_load_slope` | -0.06 | [-] | Linear load dependence of μy |
| Camber | `par.Sh_y_gamma` | 0.015 | rad/rad | Horizontal shift per rad of camber |
| Camber | `par.Sv_y_gamma` | 0.0 | N/rad | Vertical shift gain |
| Combined slip | `par.Cgx` | 1.1 | [-] | Shape factor for Gx |
| Combined slip | `par.Cgy` | 1.1 | [-] | Shape factor for Gy |
| Combined slip | `par.Bgx0` | 6.0 | [-] | Base B for Gx |
| Combined slip | `par.Bgy0` | 8.0 | [-] | Base B for Gy |
| Combined slip | `par.Bg_exp` | 0.6 | [-] | Load‑sensitivity of B |
| Trail | `par.t0` | 0.15 | m | Trail at nominal load and small slip |
| Trail | `par.t_exp` | 0.6 | [-] | Load‑sensitivity of trail |
| Trail | `par.Bt0` | 5.0 | [-] | Trail drop‑off vs slip angle |
| Trail | `par.Ct` | 1.1 | [-] | Trail shape factor |
| Trail | `par.Gt_kappa` | 1.0 | [-] | Trail reduction vs slip ratio |
| Trail | `par.Btk0` | 5.0 | [-] | Trail reduction vs slip ratio |

All parameters are adjustable; the script works for any reasonable set.

---

## 7. Domains

| Variable | Range | Resolution | Notes |
|----------|-------|------------|-------|
| `dom.kappa` | –0.25 … 0.25 | 801 pts | Longitudinal slip ratio |
| `dom.alpha` | –15° … 15° | 801 pts | Slip angle (deg → rad) |
| `dom.gamma_list` | 0°, –2°, –4° | 3 pts | Camber angles |
| `dom.loads` | 3000 N … 5000 N | 3 pts | Nominal per‑tire loads |

---

## 8. Typical usage

```matlab
% Load the script
run test.m

% Example: forces at 10 % longitudinal slip, 6° lateral slip, no camber
kappa = 0.10;
alpha = deg2rad(6);
Fz    = 4000;        % nominal load

Fx0   = make_Fx0(kappa, Fz);
Fy0   = make_Fy0(alpha, Fz, 0);
Fx    = make_Fx(kappa, alpha, Fz, 0);
Fy    = make_Fy(alpha, kappa, Fz, 0);
Mz    = make_Mz(alpha, kappa, Fz, 0);

fprintf('Fx0=%.1f N, Fy0=%.1f N, Fx=%.1f N, Fy=%.1f N, Mz=%.1f N*m\n', ...
        Fx0, Fy0, Fx, Fy, Mz);
```

The resulting values match the console output produced by the script.

---

## 9. Extending the model

* **More realistic combined‑slip** – replace the cosine weighting with a full Pacejka 7‑ or 9‑parameter formulation.  
* **Dynamic tire models** – add slip‑rate, hysteresis, or load‑transfer effects.  
* **Vehicle integration** – feed `Fx`, `Fy`, and `Mz` into a vehicle dynamics solver (e.g., `vehicleDynamics` toolbox or custom 6‑DOF model).  

The script’s anonymous functions make it easy to swap in new equations or add extra terms.

---

## 10. Summary

`pacejka_tire_model.m` is a compact, self‑contained MATLAB example that demonstrates how to:

1. Build a **load‑sensitive** Pacejka Magic Formula tire model.  
2. Apply a **combined‑slip weighting** that reduces forces with slip‑ratio/angle.  
3. Include **camber shifts** and a **pneumatic‑trail based aligning moment**.  
4. Visualise the results in a clean black‑theme layout and export the figures.
