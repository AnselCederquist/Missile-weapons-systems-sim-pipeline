# Project 09 — Attitude Control Simulator

## Overview

13-state attitude control simulator extending the 6-DOF flight mechanics of Project 06 with quaternion attitude representation, fin-actuated moment control, LQR attitude controller with dynamic pressure gain scheduling, and TPN guidance coupling from Project 08. Demonstrates a full inner/outer loop architecture: TPN generates acceleration commands, a guidance-to-attitude interface converts them to desired Euler angles, and the LQR drives fin deflections to track that attitude.

## Pipeline Integration

| Source | Data |
|--------|------|
| Project 04 | CD(Mach, α) via `aero_interpolator.py` |
| Project 06 | Propulsion model, atmosphere model, missile mass/geometry |
| Project 08 | TPN guidance law, maneuvering target model |

## State Vector

```
[0:3]   x, y, z         position (m, inertial frame)
[3:6]   vx, vy, vz      velocity (m/s, inertial frame)
[6:10]  q0, q1, q2, q3  quaternion (body-to-inertial, [w,x,y,z])
[10:13] p, q_r, r       angular rates (rad/s, body frame)
```

Quaternion attitude replaces the Euler angle representation used in Project 06, eliminating the gimbal lock singularity at θ = ±90° that occurs during high-angle terminal maneuvers. Quaternion renormalized every RK4 integration step to prevent drift.

---

## Development Methodology

This section documents the full development history of the controller architecture, including what was tried, what failed, and why the final design was chosen. This is preserved as a reference for anyone extending or replicating this work.

### Starting Point

Project 09 began as an extension of the 6-DOF simulator in Project 06. The initial architecture used Euler angles for attitude representation and a simple proportional-derivative (PD) controller driving fin deflections. A basic version of this starting point is preserved in `pd_controller_reference.py` for comparison.

The initial PD controller took pitch and yaw attitude errors directly and computed fin commands:

```
δ_pitch = Kp_pitch × θ_err + Kd_pitch × q_rate
δ_yaw   = Kp_yaw   × ψ_err + Kd_yaw   × r_rate
```

This approach was fast to implement and easy to tune but had fundamental problems:

- No systematic method to choose gains — pure trial and error
- Separate pitch/yaw gains with no coupling awareness
- No dynamic pressure scaling — gains valid only at one flight condition
- Euler angles caused gimbal lock during large attitude maneuvers
- No stability guarantee from the gain selection process

### Transition to Quaternion Attitude

The first structural change replaced Euler angles with quaternions throughout the state vector and kinematics. This eliminated gimbal lock and allowed the simulator to handle the large attitude excursions that occur during terminal guidance. The quaternion-to-Euler conversion is retained only for human-readable output and plotting.

### Normal Force Correction

An early version of the simulator produced physically unrealistic behavior — the missile would accelerate beyond Mach 15 during aggressive maneuvers. The root cause was that the normal force was not projected perpendicular to the velocity vector before being applied. The component along the velocity vector was injecting energy into the system on every timestep. Fixing this required explicitly subtracting the velocity-parallel component of the normal force before rotating to the inertial frame:

```python
F_normal_b -= np.dot(F_normal_b, v_hat_b) * v_hat_b
```

This single change resolved the energy injection problem and produced physically stable flight trajectories.

### Gravity Compensation Logic

The guidance-to-attitude interface converts TPN acceleration commands to desired body attitude. Early versions applied gravity compensation unconditionally — `a_cmd_z += G` at all times. This caused the missile to over-climb well above the target altitude and never descend, resulting in long loitering flights that missed by hundreds of meters. The fix was to apply gravity compensation only when the missile is below target altitude:

```python
grav_comp = G if pos[2] < target_alt else 0.0
```

This is a simple but effective heuristic. It allows the missile to maintain altitude against gravity during the climb phase, then lets the guidance law handle the terminal descent without fighting an artificial upward bias.

### Terminal Phase LOS Blend

TPN guidance degenerates at very short range — as the missile approaches the target, the LOS rate becomes large and noisy even with perfect sensors, and the acceleration command saturates without improving accuracy. A terminal blend was added that linearly interpolates between the TPN-derived attitude and a direct line-of-sight attitude command within 500m of the target:

```python
if r_norm < 500.0:
    blend = (500.0 - r_norm) / 500.0
    alpha_cmd = alpha_cmd * (1 - blend) + alpha_los * blend
```

This reduces PN degeneracy in the terminal phase and produced measurable improvement in miss distance.

### Hybrid PD+LQR Attempt

After implementing the LQR, a hybrid architecture was attempted that used the PD controller for rate stabilization and the LQR for attitude tracking, with the intent of combining the LQR's systematic gain selection with the PD's intuitive rate damping. In practice the hybrid produced worse results than either controller alone. The two controllers had different bandwidth assumptions and fought each other in the terminal phase, producing oscillatory fin commands. The hybrid was abandoned and the pure LQR was retained.

### LQR Design and Tuning History

The LQR linearizes the rotational dynamics at trim and solves the CARE to find the optimal gain matrix K. The key design decision was the Q and R weight matrices.

The dominant challenge was rate damping. Low rate penalties (Q_rates < 12) produced violent ±70° pitch oscillations — the controller had enough authority to excite the dynamics but not enough damping to settle them. The rate penalties were progressively increased until the system stabilized:

```
Q = diag([2, 2, 1, 15, 15, 15])   # phi, theta, psi, p, q, r
```

The Q_rate = 15 value is the stability floor for this missile geometry and flight condition. Values below 12 consistently caused oscillation.

The R matrix controls how aggressively each channel uses fin authority. R_pitch was the primary tuning parameter:

- R_pitch = 40 (symmetric): stable but insufficient terminal authority
- R_pitch = 25-30: marginally better but caused peak altitude reduction
- R_pitch = 10-15: improved terminal tracking, introduced minor early-flight pitch sensitivity
- R_pitch = 7 (final): best terminal authority while maintaining stable flight

The final R = [40, 7, 30] gives the pitch channel roughly 6× more authority than roll, with yaw between them.

Dynamic pressure gain scheduling updates K every 1s of flight as q̄ changes with altitude and velocity. This keeps the LQR near its optimal operating point throughout the engagement.

### Engagement Geometry Tuning

The engagement geometry had as much effect on miss distance as the controller tuning. Key findings:

- At 4000m range, closing velocity at CPA was ~475 m/s — too fast for meaningful terminal correction
- Moving the target to 2500m reduced closing velocity to ~430 m/s and cut miss distance from 143m to 39m
- Launch angle is sensitive: ±0.3° produced 10-20m miss distance changes
- Sustain thrust of 800N was optimal — lower thrust reduced peak altitude, higher thrust increased closing velocity

### Reproducibility: Static Seed and Target Lateral Offset

Early runs used a maneuvering target with random evasion (`max_accel=3.0*9.81`, no seed). This made results non-reproducible — the same settings would produce different miss distances on successive runs depending on the random target maneuver sequence. Two changes were made to establish a deterministic baseline:

**Static seed.** `ManeuveringTarget` accepts a `seed` parameter that initializes its internal RNG. Setting `seed=42` makes every run with identical settings produce identical output, which is required for meaningful parameter sweeps and controller tuning.

**Non-maneuvering target.** `max_accel=0.0` removes evasive maneuvers entirely. This isolates guidance and control performance from target behavior — a necessary step before adding target uncertainty back in. Early runs with `max_accel=3*9.81` and random seeds produced miss distances ranging from 82m to 658m with identical controller settings, making it impossible to attribute changes in miss distance to controller tuning.

**Zero lateral offset.** The original target position included a 300m cross-range offset (`target_pos = [4000, 300, 500]`). With the yaw channel initially under-powered, this offset was causing the missile to miss laterally regardless of pitch tuning. Moving the target to `y=0` isolated the pitch plane problem and allowed altitude and downrange geometry to be tuned independently. Cross-range capability can be verified separately once the pitch plane is working.

### Final Miss Distance Analysis

The 39m residual miss is horizontal — the missile reaches exact target altitude (508m) at CPA but is traveling at ~466 m/s with ~0.1s remaining to intercept. At that closing speed, 0.1s is ~47m of travel, which is geometrically consistent with the observed miss. This is a kinematic floor for TPN at this engagement geometry, not a control bandwidth problem. The LQR is tracking desired attitude to within 1° throughout the engagement.

---

## Architecture

### Equations of Motion

Forces resolved in inertial frame:

- **Thrust** — along body X axis, rotated to inertial via R_bi
- **Drag** — along negative velocity vector; CD from Project 04 DATCOM plus induced drag `CD_i = 0.1(α² + β²)`
- **Gravity** — inertial -Z
- **Normal force** — `F_N = q̄ × S × CN_α × α` in body frame, projected perpendicular to velocity vector before rotation to inertial

Moments from fin deflections, aerodynamic stability derivatives (CM_α, CM_δ, CM_q, CN_β, CN_δ, CN_r), and gyroscopic coupling terms. Angular acceleration from Euler's rigid body equations.

Integration via 4th-order Runge-Kutta at dt = 0.01s.

### Actuator Model

4-fin + configuration. Control mixing:

```
δ_pitch = (d1 - d3) / 2
δ_yaw   = (d2 - d4) / 2
δ_roll  = (d1 + d2 + d3 + d4) / 4
```

First-order lag τ = 0.05s, rate limit 100 deg/s, position limit ±20°.

### LQR Attitude Controller

```
ẋ = Ax + Bu ,  x = [φ_err, θ_err, ψ_err, p, q, r] ,  u = [δ_roll, δ_pitch, δ_yaw]

A'P + PA - PBR⁻¹B'P + Q = 0  →  K = R⁻¹B'P

Q = diag([2, 2, 1, 15, 15, 15])
R = diag([40, 7, 30])
```

All 6 closed-loop eigenvalues verified negative real at initialization and after every gain schedule update.

### Guidance-to-Attitude Interface

1. Rotate `a_cmd` from inertial to body frame via current quaternion
2. Add gravity compensation: `a_cmd_z += G` when below target altitude
3. Solve for required AoA: `α_cmd = m × a_cmd_z / (q̄ × S × CN_α)`
4. Clamp to ±8°
5. Desired attitude = flight path angle ± commanded AoA offset
6. Filter through first-order smoother τ = 0.2s
7. Terminal phase LOS blend within 500m

## Nominal Engagement

| Parameter | Value |
|-----------|-------|
| Launch angle | 21.2° |
| Initial speed | 25 m/s |
| Target | [2500, 0, 500] m — stationary |
| Boost thrust | 3000 N for 2s |
| Sustain thrust | 800 N through 15s |
| TPN N | 4.0, zero noise, non-maneuvering target |

```
Miss distance:   39.0 m
Hit:             False (lethal radius 20 m)
Flight time:     17.8 s
Peak Mach:       1.64
Peak altitude:   545 m
CPA altitude:    508 m
```

## Limitations

**Point-mass normal force model.** AoA commanded directly without a fin→moment→AoA→lift chain. A coupled rigid-body model would require static margin analysis and trim solutions at each flight condition.

**LQR linearization validity.** CARE solved at a single trim point. Control authority degrades beyond ~15° attitude error from the linearization point. Post-CPA attitude divergence is consistent with operating outside the valid linearization region.

**Inner/outer loop bandwidth mismatch.** At closing velocities above ~400 m/s the attitude loop cannot complete the terminal correction in the remaining time-of-flight. Requires closing velocity feedforward or explicit time-to-go weighting in the guidance law.

**No roll stabilization.** Roll channel requires fin cant angle for effectiveness. A roll autopilot would decouple pitch/yaw channels.

**Constant speed of sound.** Fixed at 343 m/s. Altitude-dependent sound speed would improve Mach accuracy above ~3000m.

## Files

```
09_attitude_control/
├── src/
│   ├── attitude_sim.py          — main 13-state simulator (LQR)
│   ├── lqr.py                   — CARE LQR + gain scheduling, 6 tests
│   ├── actuator.py              — 4-fin model, 5 tests
│   ├── quaternion.py            — quaternion math, 6 tests
│   ├── animate_3d.py            — 3D engagement GIF generator
│   └── pd_controller_reference.py  — reference PD controller (see note)
└── results/
    └── figures/
        ├── attitude_sim.png
        └── engagement_3d.gif
```

`pd_controller_reference.py` is not used by the main simulator. It is preserved as a reference implementation of the proportional-derivative approach that preceded the LQR. The file includes a drop-in interface matching `LQRController`, documented default gains with known tuning observations, an approximate closed-loop pole analysis function for the pitch channel, and a `gain_schedule()` no-op stub for interface compatibility. Anyone extending this project who wants to compare PD vs LQR performance can swap controllers with a single import change — see the file header for instructions.

## Running

```bash
cd 09_attitude_control/src

python quaternion.py             # 6 tests
python actuator.py               # 5 tests
python lqr.py                    # 6 tests
python attitude_sim.py           # nominal engagement
python animate_3d.py             # 3D GIF (requires pillow)
```