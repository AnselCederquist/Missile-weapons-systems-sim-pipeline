# Project 09 — Attitude Control Simulator

## Overview

13-state attitude control simulator extending the 6-DOF flight mechanics of Project 06 with quaternion attitude representation, fin-actuated moment control, LQR attitude controller with dynamic pressure gain scheduling, and TPN guidance coupling from Project 08. Demonstrates a full inner/outer loop architecture: TPN generates acceleration commands, a guidance-to-attitude interface converts them to desired Euler angles, and the LQR drives fin deflections to track that attitude.

Three guidance modes are implemented: TPN (True Proportional Navigation from Project 08), OGL (Optimal Guidance Law — zero-effort miss formulation), and HYBRID (TPN midcourse, OGL terminal phase switch at 200m).

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

This section documents the full development history of the controller architecture, including what was tried, what failed, and why the final design was chosen.

### Starting Point

Project 09 began as an extension of the 6-DOF simulator in Project 06. The initial architecture used Euler angles for attitude representation and a simple proportional-derivative (PD) controller driving fin deflections. A basic version of this starting point is preserved in `pd_controller_reference.py` for comparison.

The initial PD controller took pitch and yaw attitude errors directly and computed fin commands:

```
δ_pitch = Kp_pitch × θ_err + Kd_pitch × q_rate
δ_yaw   = Kp_yaw   × ψ_err + Kd_yaw   × r_rate
```

This approach was fast to implement and easy to tune but had fundamental problems: no systematic gain selection, no dynamic pressure scaling, Euler angle gimbal lock, and no stability guarantee.

### Transition to Quaternion Attitude

Euler angles replaced with quaternions throughout the state vector and kinematics. Eliminated gimbal lock and allowed handling of large attitude excursions during terminal guidance. Quaternion-to-Euler conversion retained only for output and plotting.

### Normal Force Correction

An early version produced physically unrealistic behavior — the missile accelerated beyond Mach 15 during aggressive maneuvers. Root cause: the normal force was not projected perpendicular to the velocity vector before being applied. The component along the velocity vector injected energy into the system each timestep. Fix:

```python
F_normal_b -= np.dot(F_normal_b, v_hat_b) * v_hat_b
```

This single change resolved the energy injection problem and produced physically stable trajectories.

### Gravity Compensation Logic

Early versions applied gravity compensation unconditionally — `a_cmd_z += G` at all times — causing the missile to over-climb and never descend, missing by hundreds of meters. Fix: apply compensation only when below target altitude:

```python
grav_comp = G if pos[2] < target_alt else 0.0
```

### Terminal Phase LOS Blend

TPN guidance degenerates at very short range — LOS rate saturates and commands become noisy. A terminal blend was added that linearly interpolates between TPN-derived attitude and direct LOS within 500m of the target:

```python
if r_norm < 500.0:
    blend = (500.0 - r_norm) / 500.0
    alpha_cmd = alpha_cmd * (1 - blend) + alpha_los * blend
```

### Hybrid PD+LQR Attempt

A hybrid architecture using PD for rate stabilization and LQR for attitude tracking was attempted. In practice it produced worse results than either controller alone — mismatched bandwidth assumptions caused oscillatory fin commands in the terminal phase. Abandoned.

### LQR Design and Tuning History

The LQR linearizes rotational dynamics at trim and solves the CARE for the optimal gain matrix K.

**Rate damping floor.** Q_rates < 12 produced violent ±70° pitch oscillations — enough authority to excite the dynamics but not enough damping to settle them. Q_rate = 15 is the stability floor for this geometry.

**R_pitch sweep.** R_pitch was the primary tuning parameter. Values from 40 down to 4 were tested. R_pitch = 4 caused post-CPA oscillation. R_pitch = 7–15 was the stable range. R_pitch = 15 produced the best miss distance (35.7m) and was adopted as final.

Final weights:
```
Q = diag([2, 2, 1, 15, 15, 15])   # phi, theta, psi, p, q, r
R = diag([40, 15, 30])             # roll, pitch, yaw
```

### Engagement Geometry Tuning

Engagement geometry had as much effect on miss distance as controller tuning:
- At 4000m range, closing velocity at CPA was ~475 m/s — too fast for terminal correction
- Moving target to 2500m reduced closing velocity and cut miss distance from 143m to ~37m
- Launch angle sensitive: ±0.3° produced 10–20m miss distance changes
- Sustain thrust 800N optimal — lower thrust reduced peak altitude, higher increased closing velocity

### Reproducibility: Static Seed and Target Lateral Offset

Early runs with `max_accel=3*9.81` and no seed produced miss distances ranging from 82m to 658m with identical settings, making tuning impossible. Two fixes established a deterministic baseline:

**Static seed.** `seed=42` in `ManeuveringTarget` makes every run reproducible.

**Non-maneuvering target.** `max_accel=0.0` isolates guidance/control performance from target behavior.

**Zero lateral offset.** Original target at [4000, 300, 500] caused lateral misses that masked pitch tuning. Moving to y=0 isolated the pitch plane problem. Cross-range capability can be verified separately.

### Optimal Guidance Law (OGL) Implementation

After reaching 37.1m with TPN, an Optimal Guidance Law was implemented to address the kinematic floor. OGL uses the zero-effort miss (ZEM) formulation:

```
ZEM   = r_rel + t_go × v_rel
t_go  = R / V_closing
a_cmd = (N / t_go²) × ZEM
```

Unlike TPN, command magnitude grows automatically as t_go shrinks — providing terminal aggressiveness that TPN cannot match due to its accel_limit cap.

**Pure OGL** (full flight) produced worse results — large ZEM at launch caused excessive early commands the attitude controller couldn't follow, causing tumbling.

**HYBRID mode** (TPN midcourse + OGL terminal at 200m switch) was implemented to combine TPN's stable midcourse with OGL's terminal aggressiveness. Both OGL and HYBRID converge to identical results because CPA occurs at ~150m range, inside the 200m switch threshold — the OGL has less than 0.1s to act before intercept. The switch radius would need to be much larger (~600-800m) to be effective, but at that range OGL caused over-climb divergence.

### Terminal Phase Fixes

Three additional fixes were applied to address command lag at CPA:

**Dynamic TAU_DES.** Attitude smoother time constant set to zero inside 300m, so the LQR receives raw guidance commands immediately at close range rather than filtered ones:
```python
tau_eff = TAU_DES if R > 300.0 else 0.0
```

**Dynamic dt.** Integration step halved to dt/2 inside 300m, doubling guidance update rate in the terminal phase.

**Dynamic alpha clamp.** Guidance clamp widened from ±8° to ±12° inside 300m to allow larger attitude corrections.

### Final Miss Distance Analysis

35.7m miss against a stationary non-maneuvering target at 2500m range, 500m altitude, TPN N=4, zero seeker noise. The residual miss is horizontal — the missile reaches target altitude at CPA but is traveling at ~465 m/s. At that speed, the ~0.1s window at CPA corresponds to ~47m of travel, which is geometrically consistent with the observed miss. This is a kinematic floor for this engagement geometry, not a control bandwidth problem.

---

## Architecture

### Equations of Motion

Forces resolved in inertial frame:
- **Thrust** — along body X axis, rotated to inertial via R_bi
- **Drag** — along negative velocity vector; CD from Project 04 DATCOM plus induced drag `CD_i = 0.1(α² + β²)`
- **Gravity** — inertial -Z
- **Normal force** — `F_N = q̄ × S × CN_α × α` in body frame, projected perpendicular to velocity vector before rotation to inertial

Moments from fin deflections, aerodynamic stability derivatives (CM_α, CM_δ, CM_q, CN_β, CN_δ, CN_r), and gyroscopic coupling. Angular acceleration from Euler's rigid body equations. Integration via RK4 at dt = 0.01s (halved to 0.005s inside 300m of target).

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
R = diag([40, 15, 30])
```

All 6 closed-loop eigenvalues verified negative real at initialization and after every gain schedule update (every 1s of flight).

### Optimal Guidance Law

```
ZEM   = r_rel + t_go × v_rel     # zero-effort miss vector
t_go  = max(R / V_c, 0.15)       # time-to-go, floor at 0.15s
a_cmd = (N / t_go²) × ZEM        # OGL command, N=6
```

Acceleration magnitude clamped to 40 m/s². Used in HYBRID mode for R < 200m; TPN used for midcourse.

### Guidance-to-Attitude Interface

1. Rotate `a_cmd` from inertial to body frame via current quaternion
2. Gravity compensation: `a_cmd_z += G` when below target altitude
3. Solve for required AoA: `α_cmd = m × a_cmd_z / (q̄ × S × CN_α)`
4. Clamp to ±8° (±12° inside 300m)
5. Desired attitude = flight path angle ± commanded AoA offset
6. First-order smoother τ=0.2s (τ=0 inside 300m)
7. Terminal LOS blend within 500m

## Nominal Engagement

| Parameter | Value |
|-----------|-------|
| Launch angle | 21.2° |
| Initial speed | 25 m/s |
| Target | [2500, 0, 500] m — stationary |
| Boost thrust | 3000 N for 2s |
| Sustain thrust | 800 N through 15s |
| Guidance | TPN N=4.0, zero noise, non-maneuvering target, seed=42 |

```
Miss distance:   35.7 m
Hit:             False (lethal radius 20 m)
Flight time:     14.4 s
Peak Mach:       1.67
Peak altitude:   556 m
CPA altitude:    512 m
```

## Limitations

**Point-mass normal force model.** AoA commanded directly without a fin→moment→AoA→lift chain. A coupled rigid-body model would require static margin analysis and trim solutions at each flight condition.

**LQR linearization validity.** CARE solved at a single trim point. Control authority degrades beyond ~15° attitude error. Post-CPA attitude divergence is consistent with operating outside the valid linearization region.

**Inner/outer loop bandwidth mismatch.** At closing velocities above ~400 m/s the attitude loop cannot complete terminal correction in the remaining time-of-flight. The OGL/HYBRID attempt confirmed this is a kinematic floor, not a guidance quality problem — both laws give identical results because CPA occurs inside the OGL activation range.

**No roll stabilization.** Roll channel requires fin cant angle for effectiveness. A roll autopilot would decouple pitch/yaw channels.

**Constant speed of sound.** Fixed at 343 m/s.

## Files

```
09_attitude_control/
├── src/
│   ├── attitude_sim.py          — main 13-state simulator (TPN/OGL/HYBRID)
│   ├── lqr.py                   — CARE LQR + gain scheduling, 6 tests
│   ├── actuator.py              — 4-fin model, 5 tests
│   ├── quaternion.py            — quaternion math, 6 tests
│   ├── animate_3d.py            — 3D engagement GIF (TPN + HYBRID overlay)
│   └── pd_controller_reference.py  — reference PD controller (not used by main sim)
└── results/
    └── figures/
        ├── attitude_sim.png
        ├── attitude_sim_ogl.png
        └── engagement_3d.gif
```

`pd_controller_reference.py` is preserved as a reference implementation of the PD approach that preceded the LQR. Drop-in interface matches `LQRController` — swap with a single import change. See file header for instructions.

## Running

```bash
cd 09_attitude_control/src

python quaternion.py             # 6 tests
python actuator.py               # 5 tests
python lqr.py                    # 6 tests
python attitude_sim.py           # runs TPN + HYBRID, prints comparison summary
python animate_3d.py             # 3D GIF with TPN and HYBRID trajectories (requires pillow)
```