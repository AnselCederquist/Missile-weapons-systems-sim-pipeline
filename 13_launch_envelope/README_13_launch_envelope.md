# Project 13 — Launch Envelope Calculator

## Overview

Computes No-Escape Zone (NEZ) and Firing Envelope Zone (FEZ) boundaries for a missile against maneuvering and non-maneuvering aerial targets across a range-bearing launch geometry grid. Standard deliverable in missile systems engineering — shows where a shooter must be to achieve a required Pk against a defined target.

The simulation chain is:
1. Target state parameterization (speed, heading, altitude, g-load)
2. Launch geometry sweep: 20 range bins x 36 bearing bins = 720 grid points
3. Per-geometry engagement via simplified 3-DOF point-mass model with TPN guidance
4. Miss distance calibration against Project 09 full 13-state pipeline (factor 0.518)
5. Pk computation via Project 12 lethality model at each grid point
6. NEZ/FEZ boundary extraction from Pk contour

Two target configurations compared: non-maneuvering (0g) and evasive (3g lateral).

## Pipeline Integration

| Source | Data Used |
|--------|-----------|
| Project 09 | Miss distance calibration baseline (35.7m at nominal geometry) |
| Project 12 | `pk_at_miss()` for Pk computation at each grid point |
| Project 08 | TPN guidance law formulation (N=4) |

## Key Results

**Sweep Parameters**
- Range: 500-6000m, 20 bins
- Bearing: -180 to +170 deg, 36 bins
- Altitude differential: 490m (target at 500m, missile at 10m)
- Monte Carlo: 3 seeds per grid point
- Miss distance calibration: 3-DOF x 0.518 = full-pipeline equivalent

**Envelope Summary**

| Configuration | Max FEZ Range | Max Pk | Mean Pk |
|---------------|---------------|--------|---------|
| Non-maneuvering (0g) | 2816 m | 0.041 | 0.0001 |
| 3g maneuver | 2237 m | 0.025 | 0.0000 |

- FEZ shrinks 21% under 3g evasion (2816m to 2237m)
- Max Pk drops 39% (0.041 to 0.025) under evasion
- No NEZ at any geometry (Pk never reaches 0.5) — correct for this warhead size and engagement model fidelity
- Engagement corridor is narrow and forward-aspect only — physically correct for a boost-sustain TPN-guided missile against a head-on target

**Miss Distance Map**
- Head-on corridor: 5-50m calibrated miss (green), tapering to 500-1000m off-axis (red)
- Forward-aspect engagement geometry clearly visible — missile cannot intercept from beam or tail-chase aspects at these ranges
- Miss distance map is the primary deliverable; Pk contour is a downstream product of the calibrated lethality model

## Development Methodology

### 3-DOF vs 13-State Fidelity

The launch envelope sweep requires 720+ engagement simulations per target configuration. Running the full 13-state attitude control simulator (Project 09) at each grid point is computationally prohibitive. A simplified 3-DOF point-mass model was used for sweep speed.

The 3-DOF model produces systematically larger miss distances than the full pipeline because it lacks:
- Quaternion attitude dynamics and LQR control
- Fin actuator model with rate/position limits
- Guidance-to-attitude interface with terminal phase corrections
- Dynamic pressure gain scheduling

**Calibration approach:** At the nominal engagement geometry (2500m range, 0 deg bearing, 490m altitude differential), the 3-DOF produces 68.9m miss vs the full pipeline's 35.7m. The ratio 35.7/68.9 = 0.518 is applied as a multiplicative correction to all 3-DOF miss distances before Pk computation. This is standard practice in operational envelope tools — fast low-fidelity models calibrated against high-fidelity spot checks.

**Limitation:** The calibration factor is derived from a single geometry. The 3-DOF/13-state ratio likely varies across the envelope — the calibration is most accurate near the nominal geometry and degrades at extreme ranges and off-axis bearings. A production tool would calibrate at multiple spot-check geometries and interpolate the correction factor.

## Architecture

**3-DOF point-mass engagement model:**
- Thrust along velocity vector (boost 3000N / sustain 800N)
- CD0 = 0.3 aerodynamic drag
- TPN guidance (N=4) with 0.002 rad/s LOS rate noise
- 0.5g gravity compensation
- First-order guidance lag (tau=0.1s)
- 200 m/s^2 acceleration limit
- Euler integration at dt=0.05s (coarser than full pipeline for speed)

**Launch geometry sweep:**
- `TargetState` dataclass: speed, heading, altitude, g-load
- `LaunchGeometry` dataclass: range, bearing, altitude differential
- `sweep_range_bearing()`: nested loop over range x bearing grid, n_mc seeds per point
- Returns `pk_grid`, `pk_gauss_grid`, `miss_grid`

**NEZ/FEZ boundary extraction:**
- Per-bearing scan: find max range where Pk exceeds threshold
- NEZ threshold: Pk > 0.5 (no-escape zone)
- FEZ threshold: Pk > 0.001 (firing envelope zone)

**Pk computation:**
- Miss distance calibrated by factor 0.518 before Pk evaluation
- `pk_at_miss()` from Project 12 lethality model (Gurney + Mott + cookie-cutter)
- Gaussian kill function also computed for comparison

## Files

```
13_launch_envelope/
├── src/
│   ├── launch_envelope.py      # 3-DOF engagement, sweep, NEZ/FEZ extraction, 10 tests
│   ├── envelope_sim.py         # Polar contour plots, comparison visualization
│   └── bifurcation_overlay.py  # Analytical capture boundary (arXiv:2604.09065)
├── results/
│   └── figures/
│       ├── envelope_pk_Non-maneuvering.png
│       ├── envelope_pk_3g_maneuver.png
│       ├── envelope_miss_Non-maneuvering.png
│       ├── envelope_comparison.png
│       ├── bifurcation_overlay_Nonmaneuvering.png
│       └── bifurcation_overlay_3g_maneuver.png
└── README.md
```

## Usage

```bash
cd 13_launch_envelope/src
python launch_envelope.py    # Validation: 10/10 pass
python envelope_sim.py       # Full sweep + all plots (~2 min)
python bifurcation_overlay.py  # Analytical capture boundary overlay (~2 min)
```

## Bifurcation Boundary Validation (arXiv:2604.09065)

The analytical capture boundary from circular pursuit theory is overlaid on the
numerical Pk contours. The capture-to-limit-cycle transition (Shekhawat & Sinha)
characterizes the maximum range at which the missile can kinematically intercept
the target, assuming perfect guidance and unlimited warhead lethality.

| Config | Analytical capture | Numerical FEZ | FEZ inside boundary |
|--------|-------------------|---------------|---------------------|
| Non-maneuvering | 9322 m | 2816 m | YES |
| 3g maneuver | 3790 m | 2237 m | YES |

The FEZ sits well inside the analytical capture boundary in both cases. The gap
represents the combined effect of guidance imperfection (non-zero miss distance)
and finite warhead lethality (Pk < 1 at non-zero miss). Speed ratio mu = 9.0
(missile avg 449 m/s vs target 50 m/s) confirms the missile has overwhelming
kinematic advantage — performance is warhead-limited, not kinematics-limited.

Evasion shrinks the analytical boundary by 59% (9322m to 3790m) but the FEZ
only shrinks 21% (2816m to 2237m), because the FEZ is already constrained by
lethality well inside the kinematic limit.

## Known Limitations

- **Single-geometry calibration.** The 3-DOF to 13-state correction factor (0.518) is derived from one nominal engagement geometry. The ratio likely varies across the envelope — most accurate near the calibration point, degrading at extreme ranges and off-axis bearings. A production tool would use multi-point calibration with interpolated correction.

- **Euler integration.** The 3-DOF uses forward Euler at dt=0.05s for sweep speed. The full pipeline uses RK4 at dt=0.01s (halved to 0.005s in the terminal phase). Euler integration introduces energy drift at coarse timesteps that accumulates over long engagements.

- **No attitude dynamics.** Lateral acceleration commands applied directly to the point mass — no fin deflection, no autopilot lag beyond the first-order guidance lag (tau=0.1s), no angle of attack limits. This is the standard point-mass assumption for envelope analysis.

- **Constant target maneuver.** Target g-load is constant throughout the engagement. Real evasion involves variable-timing bang-bang maneuvers (modeled in Project 08 Monte Carlo but not in the envelope sweep).

- **No minimum range.** The 3-DOF model does not enforce a minimum engagement range (arming distance, minimum guidance time). Real systems have a dead zone at very short range where the missile cannot maneuver fast enough to intercept.

- **Narrow FEZ corridor.** The envelope shows engagement capability only in a narrow head-on sector. This is physically correct for a forward-aspect TPN-guided missile with fixed launch angle (21.2 deg), but a real system with variable launch angle and mid-course guidance would have a wider envelope.

- **No NEZ boundary.** Pk never reaches 0.5 at any geometry — the warhead lethal radius (11.1m from Project 12) is smaller than the best calibrated miss distance (~35m). A larger warhead or a higher-fidelity engagement model producing smaller miss distances would generate a visible NEZ.

## References

- Driels, M. *Weaponeering: Conventional Weapon System Effectiveness*, AIAA Education Series, 2013 — canonical NEZ/FEZ methodology.
- Zarchan, P. *Tactical and Strategic Missile Guidance*, 6th ed., Ch. 14-15 — launch envelope computation, engagement zone definitions.
- arXiv:2604.09065 — Shekhawat & Sinha, *A Study of the Circular Pursuit Dynamics using Bifurcation Theoretic Computational Approach* — circular pursuit capture-to-limit-cycle transition provides analytical boundary conditions for validating numerically computed NEZ/FEZ contours. The launch envelope boundary is exactly where circular pursuit transitions from capture to escape behavior; the bifurcation analysis gives a formal criterion to cross-validate simulation-derived Pk contours.