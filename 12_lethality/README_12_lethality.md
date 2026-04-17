# Project 12 — Fragmentation Lethality Model

## Overview

Full physics-based fragmentation lethality model that closes the end-to-end kill chain from seeker-guided engagement (Project 11) to system-level kill probability. The model replaces the arbitrary 20 m lethal radius assumption used in earlier projects with a first-principles calculation grounded in warhead physics.

The simulation chain is:
1. Gurney equation → initial fragment launch velocity from charge-to-metal ratio
2. Mott distribution → statistical fragment mass and count
3. Spherical spreading with beam geometry → realistic fragment spatial density at miss distance r
4. Aerodynamic drag decay → fragment velocity as function of range
5. Cookie-cutter vulnerability → single-fragment kill probability based on kinetic energy threshold (80 J)
6. Binomial ensemble → overall Pk(r) = 1 − (1 − Pk_single)^n_lethal

System-level Pk is computed as the statistical expectation E[Pk(r)] over the empirical miss distance distribution from Project 11 seeker Monte Carlo. This allows direct quantification of how seeker noise, glint, and ECM affect overall weapon effectiveness.

Sensitivity analysis was performed on charge-to-metal ratio (C/M) and target vulnerable area fraction (Av/Ap) to understand design trades.

## Pipeline Integration

| Source          | Data Used |
|-----------------|-----------|
| Project 11      | Real miss distance arrays from 50-run Monte Carlo (`seeker_miss_distances.pkl`) |
| Project 09/10   | Context for kinematic miss floor (~35.7 m baseline) |

## Key Results

**Warhead Parameters**
- Case: 150 mm long × 100 mm diameter, 3.0 kg steel
- Charge mass: 1.5 kg
- C/M ratio: 0.500
- Gurney velocity: 1708 m/s
- Mott distribution: N ≈ 5000 fragments, μ ≈ 0.30 g

**Lethality Curve**
- Lethal radius (Pk = 0.5): **11.1 m**
- Pk(10 m) = 0.575
- Pk(20 m) = 0.167
- Pk(30 m) = 0.068

**System-Level Kill Probability** (real seeker Monte Carlo, 50 runs per config)

| Configuration              | Mean Miss | System Pk |
|----------------------------|-----------|-----------|
| Ideal sensor (baseline)    | 35.7 m    | 0.0440    |
| Seeker — base noise        | 35.3 m    | 0.0454    |
| Seeker — noise + glint     | 35.3 m    | 0.0454    |
| Seeker — ECM spoofing      | 35.7 m    | 0.0184    |

## Development Methodology

Early versions produced unrealistic lethal radii (<3 m or >25 m) due to incorrect fragment density scaling. Multiple iterations were required to balance spherical spreading, beam geometry, and Gaussian roll-off without artificial multipliers.

The final density model uses:
- Proper 4πr² spherical surface area
- Thin equatorial belt approximation for the fragment spray pattern
- Gentle Gaussian falloff from beam center
- No arbitrary scaling factors

This produces a smooth Pk curve with a physically plausible lethal radius of 11.1 m for a 4.5 kg warhead against an aerial target with 25% vulnerable area.

## Architecture

**Gurney equation** — fragment launch velocity from C/M ratio.

**Mott distribution** — fragment mass statistics tuned to realistic values for steel casing.

**Fragment spatial density** — 4πr² spreading with beam geometry (broadside detonation assumption).

**Velocity decay** — aerodynamic drag on individual fragments as function of range.

**Vulnerability model** — cookie-cutter criterion (KE ≥ 80 J) applied to vulnerable area fraction (Av/Ap = 0.25).

**System Pk** — statistical mean of Pk(r) evaluated over empirical miss distance samples from Project 11.

## Files
12_lethality/
├── src/
│   ├── lethality.py           # Core physics: Gurney, Mott, density, Pk calculation
│   └── lethality_sim.py       # Visualization, system Pk, sensitivity analysis
├── results/
│   └── figures/
│       ├── pk_vs_miss.png
│       ├── system_pk_overlay.png
│       └── lethality_sensitivity.png
└── README.md

## Known Limitations

- **Broadside detonation only.** Fragment pattern assumes warhead detonates
  perpendicular to the target. Real proximity fuzes detonate at varying
  angles relative to the target flight path — fragment-target intersection
  geometry is aspect-dependent. No fuze delay or lead-angle modeling.

- **Static target geometry.** Target presented area (8 m^2) and vulnerable
  fraction (0.25) are fixed constants. Real targets have aspect-dependent
  presented area (head-on vs broadside vs belly) and component-level
  vulnerability varying by azimuth and elevation.

- **Cookie-cutter kill criterion.** Binary KE threshold (80 J) — fragment
  either kills or doesn't. Real terminal ballistics involves penetration
  mechanics (Thor equations, residual velocity behind armor), incendiary
  effects, and synergistic damage from multiple sub-lethal hits. No
  blast overpressure contribution modeled.

- **No fragment shape effects on drag.** All fragments use a single drag
  coefficient (CD=1.0 for cubes). Real natural fragmentation produces
  irregular shapes with CD varying 0.8-1.5 depending on tumble state
  and Mach regime. Heininen (2022) provides shape-dependent drag tables
  that could replace the current model.

- **Range estimation uses true range.** The seeker model (Project 11)
  passes true range to the lethality model for r_apparent reconstruction.
  A real proximity fuze has its own range estimation error (doppler,
  RF gate) that would shift the detonation point and fragment pattern
  relative to the target.

- **Mott distribution is semi-empirical.** The Mott constant B=0.68 was
  calibrated to produce realistic fragment statistics for a 3 kg steel
  case. Real Mott parameters are determined experimentally via arena
  tests or hydrocode (CALE/CTH) simulation. The model does not account
  for controlled fragmentation (scored casings, preformed fragments).

- **Miss distance sample size.** System Pk is computed from 50-run
  Monte Carlo per seeker configuration (Project 11). Statistical
  uncertainty on Pk estimates is approximately +/-0.01 at this sample
  size. Higher-fidelity assessment would use 500-1000 runs per config.

- **No warhead-airframe coupling.** Warhead detonation does not feed back
  into the flight simulation — no structural breakup, no aerodynamic
  effects of proximity detonation on missile trajectory. The engagement
  ends at CPA; detonation is assumed instantaneous at that point.

- **Lethal radius (11.1 m) is on the generous side** for MANPADS-class
  warheads (published Stinger: ~5-7 m). Consistent with a slightly
  larger "small SAM" class at 4.5 kg total warhead mass. Sensitivity
  analysis (C/M and Av/Ap sweeps) bounds the uncertainty.

## References

- Gurney, R.W. *The Initial Velocities of Fragments from Bombs, Shells,
  and Grenades*, BRL Report 405, 1943.
- Mott, N.F. *Fragmentation of Shell Cases*, Proc. Royal Soc. A, 1947.
- Catovic, A. & Kljuno, E. *Terminal-ballistics model for estimating
  munition lethal radius*, J. Defence Modeling & Simulation, 2022 —
  80 J incapacitation threshold, 1 frag/m^2 lethal radius definition.
- Heininen, T. *A Simplified Method for Computing the Lethality of
  Fragmenting Ammunition*, 2022 — methodology basis for the
  Gurney+Mott+cookie-cutter pipeline implemented here.
- Driels, M. *Weaponeering: Conventional Weapon System Effectiveness*,
  AIAA Education Series, 2013 — canonical Pk(r) and system Pk formulation.
- Ball, R.E. *The Fundamentals of Aircraft Combat Survivability*,
  AIAA, 2003 — vulnerable area fraction methodology.

## Usage

```bash
cd 12_lethality/src
python lethality.py          # Validation: 10/10 pass
python lethality_sim.py      # Full analysis + generates all plots