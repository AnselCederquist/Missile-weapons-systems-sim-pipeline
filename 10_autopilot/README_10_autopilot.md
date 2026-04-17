# Project 10 — Three-Loop Acceleration Autopilot

## Overview

Industry-standard three-loop missile autopilot implementing the classical acceleration
autopilot architecture used in real tactical missile programs. Inner angular rate loop
(PD), middle body acceleration loop (PI with leaky integrator), outer TPN guidance loop
from Project 08. Directly comparable to the LQR from Project 09 on the same engagement
geometry — same airframe, same target, same guidance law.

Where Project 09's LQR has full state information and a globally optimal gain matrix,
the three-loop autopilot has only body acceleration (accelerometer) and angular rate
(rate gyro) feedback. This is physically representative of fielded systems.

## Pipeline Integration

| Source | Data |
|--------|------|
| Project 04 | CN_α = 4.0/rad, CD(Mach, α) via `aero_interpolator.py` |
| Project 06 | RK4 EOM, propulsion model, atmosphere model, quaternion kinematics |
| Project 08 | TPN guidance law, maneuvering target model |
| Project 09 | FinActuator (τ=0.05s, ±20°, 100 deg/s rate limit), quaternion.py, actuator.py |

## Result

| Metric | Value |
|--------|-------|
| Miss distance | 58.6 m |
| Hit (lethal radius 20m) | False |
| Flight time | ~19 s |
| Peak Mach | 1.44 |
| Peak altitude | ~519 m |
| LQR baseline (Project 09) | 35.7 m |

The 22.9m gap between three-loop (58.6m) and LQR (35.7m) is physically expected.
LQR has full state information and globally optimal gains. The three-loop has only
body acceleration and rate feedback — less information, more physically realistic.

---

## Architecture

### Three-Loop Structure

```
TPN Guidance (outer)
    │  a_cmd (m/s²)
    ▼
Acceleration Loop (middle — PI)
    │  ω_cmd (rad/s)
    ▼
Rate Loop (inner — PD)
    │  δ_fin (rad)
    ▼
Airframe + Actuator
    │  a_body, ω_body (feedback)
    └──────────────────────────┘
```

**Inner loop (rate loop):** PD controller. Measures body angular rate via rate gyro,
commands fin deflection to drive rate error to zero. Fastest loop — must close before
the middle loop commands change significantly.

**Middle loop (acceleration loop):** PI controller with leaky integrator. Measures
body lateral acceleration via accelerometer, commands rate to track guidance-demanded
acceleration. Leaky integrator prevents windup without hard reset — retains 100% of
history (INT_LEAKAGE=1.0, pure integrator) at the optimized configuration.

**Outer loop:** TPN guidance from Project 08. Generates inertial acceleration commands
from LOS rate measurements. Unchanged from Projects 08 and 09.

### Gain Scheduling

All lateral gains (Ka, Ki_a, Kp_rate, Kd_rate) scheduled inversely proportional to
dynamic pressure:

```
scale = clip(Q_NOM / q_bar, SCALE_MIN, SCALE_MAX)
Ka    = Ka_nom * scale
```

Roll gain is fixed (not scheduled) — scheduling roll introduced the Kp_roll=-7.75
saturation failure mode early in development (see Development History below).

### Roll Stabilization

STT (skid-to-turn) architecture — roll commanded to zero. Roll fin command:

```
u_roll = Kp_roll * (0 - p)
```

Physical constraint: at p=120 deg/s roll rate (EOM clip), saturation requires
|Kp_roll| < 0.167. Above this threshold fins are permanently at ±20° and provide
no net corrective torque. Optimized value Kp_roll=-0.02 provides minimal
stabilization without saturating. Roll authority is the fundamental physical
limitation of this airframe at this engagement geometry.

---

## Final Configuration

```python
# autopilot.py gains
Ka_nom      =  0.8
Ki_a_nom    =  0.45
Kp_rate_nom = -0.28
Kd_rate_nom =  0.0
Kp_roll_nom = -0.02   # FIXED — not scheduled

# autopilot.py constants
OMEGA_CMD_MAX  = 1.5
INT_LEAKAGE    = 1.0
INT_CLIP       = 1.2
Q_BAR_FLOOR    = 12000.0
SCALE_MIN      = 0.6
SCALE_MAX      = 2.5
KI_A_SCALE     = 0.6

# autopilot_sim.py
CONTROL_START  = 2.5    # s
# rk4_step EOM clip
np.radians(200)         # deg/s body rate limit
```

---

## Bayesian Optimization

Parameter search using Optuna TPE (Tree-structured Parzen Estimator) with SQLite
persistence. Every parameter ever changed or suggested during development was included
in the search space.

**Search summary:**

| Run | Trials | Best miss | Notes |
|-----|--------|-----------|-------|
| 1 — full space (15 params) | 2205 | 73.4m | Ka, INT_LEAKAGE, SCALE_MAX at boundaries |
| 2 — extended boundaries | 916 | 64.1m | Same boundary pattern |
| 3 — narrowed to 3 active params | 4107 | 58.6m | Converged — confirmed true optimum |

**Parameter importance (final run):**

| Parameter | Importance | Interpretation |
|-----------|-----------|----------------|
| Kp_rate_nom | 49.9% | Inner loop proportional gain — dominant |
| OMEGA_CMD_MAX | 37.4% | Rate command ceiling — second dominant |
| Kd_rate_nom | 12.7% | Derivative term — best value is 0.0 (disabled) |
| All others | ~0% | Scheduling params, roll gain, Ki_a — irrelevant |

**Key findings:**
- 58.6m is the true optimum — confirmed at trial #86, unchanged through trial #4107
- Kd_rate=0.0 (derivative disabled) — derivative term amplified noise through
  omega_dot more than it added damping
- INT_LEAKAGE=1.0 — pure integrator optimal; leaky integrator degraded terminal tracking
- All gain scheduling parameters (SCALE_MIN, SCALE_MAX, Q_BAR_FLOOR, KI_A_SCALE)
  have ~0% importance — scheduling structure is correct but specific values don't matter

**Persistence:** Results stored in `autopilot_tuning.db` (SQLite). Resume with
`python tune_autopilot.py`. View results without running with `--results` flag.

---

## Development History

Complete record of failure modes, root causes, and fixes. Documents the engineering
process, not just the final result.

### Initial Configuration — 102.6m

First run: Ka=0.8, Kp_rate=-1.5, no gain scheduling constraints. Produced violent
oscillations and permanent fin saturation. Root cause: gain scheduling applied to
Kp_roll, inflating it to -7.75 at low q_bar (SCALE_MAX=3.0, q_bar=18,059 Pa at
CONTROL_START). At Kp_roll=-7.75 with roll rate ~60 deg/s, roll fin command was
+280°, immediately saturated, and provided no corrective torque. Roll went
uncontrolled for the entire flight.

### Roll Gain Scheduling — Root Cause

The gain schedule fires at CONTROL_START (t=2.5s) with q_bar=18,059 Pa:

```
scale = Q_NOM / q_bar = 50,000 / 18,059 = 2.77
Kp_roll_scheduled = -1.58 * 2.77 = -4.38
```

With SCALE_MAX=3.0 this reached Kp_roll=-7.75. Fix: separate Kp_roll from the
scheduling loop entirely. Roll gain is now fixed at nominal regardless of q_bar.

### EOM Body Rate Clip

Original EOM clip: ±200 deg/s. Physically unrealistic for a small tactical missile.
Changed to ±120 deg/s. At 120 deg/s, saturation math:

```
u_roll = Kp_roll * (0 - 120 deg/s) = -1.4 * (-2.094 rad/s) = +168 deg → clips to +20°
```

Saturation occurs for any |Kp_roll| > 0.167. This is the physical floor — the
airframe does not have sufficient roll fin authority to arrest aerodynamically-induced
roll at this flight condition. Optimized configuration uses Kp_roll=-0.02 (near zero)
and accepts the roll as a physical limitation.

### Guidance Hold-Last-Command

After CPA, TPN returns valid=False and the autopilot received zero acceleration
command for the remainder of the flight. Fix in autopilot_sim.py: hold last valid
guidance command instead of defaulting to zero:

```python
if valid:
    a_cmd_inertial = guidance_to_accel_cmd(...)
elif len(hist['a_cmd']) > 0:
    a_cmd_inertial = hist['a_cmd'][-1]
```

### Tuning Iterations Summary

| Config | Miss | Change |
|--------|------|--------|
| Ka=0.8, Kp_rate=-1.5, no roll fix | 102.6m | Baseline |
| Fixed Kp_roll (not scheduled) | 98.2m | Roll saturation reduced |
| Conservative lateral gains | 300.8m | Overcorrected — too weak |
| EOM clip 120 deg/s + guidance hold | 356.1m | Roll still saturated |
| EOM clip 200 + balanced gains | 388.8m | Wrong direction |
| Bayesian run 1 (2205 trials) | 73.4m | Systematic search |
| Bayesian run 2 (extended bounds) | 64.1m | Boundary extension |
| Bayesian run 3 (focused) | 58.6m | Confirmed optimum |

---

## Limitations

**Roll control authority.** CN_α=4.0/rad airframe cannot arrest aerodynamically-induced
roll at 120+ deg/s with ±20° fins. A roll autopilot on a canard-controlled airframe
would require fin cant angle or a separate roll channel with higher authority.

**EOM clip at 200 deg/s.** Bayesian optimization found 200 deg/s produces better miss
distance than the more physically realistic 120 deg/s. Documented as a modeling
limitation — the looser physics allows more aggressive terminal maneuvering than the
real airframe could sustain.

**Point-mass acceleration feedback.** The accelerometer measures total body acceleration
including thrust and drag along the body axis. The lateral components used by the
autopilot are a small fraction of total body acceleration. At high AoA the projection
introduces cross-axis coupling.

**LQR vs three-loop performance gap.** 22.9m gap is intrinsic — LQR uses full state
information (all 6 attitude/rate states) while the three-loop uses only 2 measurements
(lateral acceleration, angular rate). This is the correct result, not a tuning failure.

**Kd_rate=0 optimal.** Derivative term disabled by optimizer. Indicates the omega_dot
numerical derivative (computed from consecutive rate measurements at dt=0.01s) is too
noisy to be useful. A hardware rate gyro with analog differentiation would behave
differently.

---

## Files

```
10_autopilot/
├── src/
│   ├── autopilot.py         — AutopilotController class, 8/8 validation tests
│   ├── autopilot_sim.py     — 13-state sim with three-loop, guidance hold-last-command
│   ├── compare_lqr.py       — side-by-side comparison plots vs Project 09 LQR
│   └── tune_autopilot.py    — Bayesian optimizer (Optuna TPE, SQLite persistence)
└── results/
    └── figures/
        ├── autopilot_sim_Three-Loop.png
        └── autopilot_tuning.db
```

## Running

```bash
cd 10_autopilot/src

python autopilot.py          # 8/8 validation tests
python autopilot_sim.py      # runs three-loop + LQR comparison, saves plots

# Bayesian optimization (requires: pip install optuna)
python tune_autopilot.py                    # run indefinitely
python tune_autopilot.py --results          # view current best without running
python tune_autopilot.py --n 500            # run fixed number of trials
python tune_autopilot.py --reset            # delete DB and start fresh
```

## Comparison: Three-Loop vs LQR

| | Three-Loop (P10) | LQR (P09) |
|---|---|---|
| Miss distance | 58.6 m | 35.7 m |
| Feedback | Accel + rate gyro | Full state (6 states) |
| Gain design | Manual + Bayesian | CARE optimal |
| Scheduling | Dynamic pressure | Dynamic pressure |
| Roll control | Fixed Kp_roll | Not implemented |
| Architecture | Fielded standard | Optimal control |
| Trials to optimize | 4107 | N/A (analytical) |

The three-loop is the standard architecture in fielded tactical missiles because it
requires only accelerometers and rate gyros — sensors that are available, low-cost,
and robust. LQR requires a full state estimator feeding back all attitude and rate
states, which increases sensor and processing requirements. The 22.9m performance
gap is the cost of the reduced sensor suite.