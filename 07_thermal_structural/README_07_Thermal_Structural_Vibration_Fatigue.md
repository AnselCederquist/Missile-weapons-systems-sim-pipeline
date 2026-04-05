# Project 07 — Thermal-Structural Analysis

Coupled aeroheating, transient thermal, static structural, modal, random vibration,
and fatigue analysis of the missile fin assembly and nose cone under flight loads
derived from the Project 06 6-DOF trajectory. Full pipeline integration: Project 06
Mach/altitude history → Python aeroheating → Ansys Workbench coupled analysis chain.

## Overview

Structural survival through the flight envelope requires more than a room-temperature
static load check. This project evaluates the fin assembly and nose cone under the
combined thermal and mechanical environment of the full 43.2-second flight:
aerodynamic heating from the supersonic trajectory, 10,000g axial setback load at
launch, broadband random vibration from motor burn, and fatigue under cyclic loading.

The analysis chain is:
Python aeroheating (DKR method)
→ time-varying heat flux CSVs
→ Ansys Transient Thermal (43.2s)
→ Ansys Static Structural (thermal only)
→ Ansys Static Structural (thermal + 10,000g axial)
→ Ansys Modal (prestressed, 6 modes)
→ Ansys Random Vibration (flat PSD, MIL-STD-810)
→ Fatigue Tool (Ti-6Al-4V S-N curve, stress life)

Both the fin assembly (from Project 02) and a new nose cone geometry (conical,
Ti-6Al-4V, 150mm length, 50mm base radius, 5mm wall, 10mm tip fillet) are analyzed
through the full chain.

Key result: fin leading edge is the critical location — 248.79°C peak temperature,
4.46 MPa combined stress, 415 Hz first bending mode, 10.57 MPa random vibration
1σ stress. All safety factors are large — the design is structurally sound throughout
the flight envelope. Random vibration governs over static setback for the fin.
Thermal stress contributes 73% of the fin combined stress — material thermal
properties are the dominant design driver, not structural geometry.

## Pipeline Integration

| Upstream project | Contribution |
|---|---|
| Project 02 | Fin assembly geometry (Parasolid), Ti-6Al-4V material selection |
| Project 06 | Mach(t), altitude(t), speed(t) → aeroheating inputs via run_sixdof() |

## Geometry

**Fin assembly** — tapered trapezoidal fin with dovetail tab, Ti-6Al-4V annealed.
Same geometry as Project 02. Imported as Parasolid into Ansys Workbench.

**Nose cone** — conical shell modeled in Onshape:
- Length: 118.38 mm (150mm nominal minus 10mm tip fillet radius geometry adjustment,
  actual tip-to-base height as modeled)
- Base radius: 50 mm
- Wall thickness: 5 mm
- Tip: 10mm radius fillet — chosen to avoid degenerate mesh point and to
  represent a realistic blunt nose tip. Note: nose tip radius is an aeroheating
  design variable — smaller radius concentrates heat flux. The 10mm fillet used
  here produces lower heat flux than a sharper tip would.
- Material: Ti-6Al-4V annealed

## Material

Ti-6Al-4V annealed from Ansys Granta Design Typical Materials library.

| Property | Value | Unit |
|---|---|---|
| Density | 4429 | kg/m³ |
| Young's Modulus | 111.2 | GPa |
| Poisson's Ratio | 0.3387 | — |
| Thermal Expansion Coefficient | 8.789e-6 | /°C |
| Thermal Conductivity | 7.187 | W/m·°C |
| Specific Heat | 522.6 | J/kg·°C |

Temperature-dependent yield strength (Granta tabular data):

| Temperature (°C) | Yield Strength (MPa) |
|---|---|
| 20 | 880 |
| 100 | 830 |
| 200 | 790 |
| 300 | 730 |
| 400 | 620 |
| 500 | 480 |

S-N curve for fatigue (manually entered from literature, Ti-6Al-4V annealed,
R=-1, fully reversed):

| Cycles | Stress (MPa) |
|---|---|
| 1,000 | 1200 |
| 10,000 | 950 |
| 100,000 | 750 |
| 1,000,000 | 620 |
| 10,000,000 | 550 |
| 100,000,000 | 500 |

Endurance limit ~500 MPa at 10^8 cycles. Data from published literature —
not Granta certified. Used in preference to Ansys default flat S-N curve
which produced non-conservative infinite life estimate before correction.

## Aeroheating (Python)

**Method:** Detra-Kemp-Riddell (DKR) stagnation point heat flux formula,
lumped capacitance surface temperature model, isentropic stagnation temperature.

**Inputs:** Mach(t), altitude(t), speed(t) from Project 06 run_sixdof()

**Key parameters:**

| Parameter | Nose Tip | Fin LE |
|---|---|---|
| Surface radius | 10 mm | 2 mm |
| Material thickness | 5 mm | 4 mm |
| Emissivity | 0.3 | 0.3 |
| DKR constant k | 1.83e-4 | 1.83e-4 |

**Python results:**

| Metric | Nose Tip | Fin LE |
|---|---|---|
| Max stagnation temp | 96.7°C | 96.7°C |
| Max heat flux | 0.15 MW/m² | 0.33 MW/m² |
| Max surface temp (lumped capacitance) | 227°C | 485°C |

Python lumped capacitance results are used to derive heat flux boundary
conditions for Ansys. Ansys transient thermal results supersede Python surface
temperatures as the primary thermal output — the Ansys model accounts for
conduction through the wall and radiation cooling, which the lumped model
does not fully resolve.

**Heat flux tabular input to Ansys (condensed from 4324-row CSV):**

Fin LE (W/mm²):

| Time (s) | Heat Flux (W/mm²) |
|---|---|
| 0 | 0 |
| 1 | 0.02 |
| 2 | 0.05 |
| 5 | 0.18 |
| 8 | 0.33 |
| 10 | 0.22 |
| 15 | 0.15 |
| 20 | 0.12 |
| 30 | 0.10 |
| 43.2 | 0.08 |

Nose tip (W/mm²): same time points, peak 0.10 W/mm² at t=8s.

Heat flux peaks at t=8s coinciding with maximum dynamic pressure and Mach
number in the Project 06 trajectory (qbar_max = 100.4 kPa, Mach_max = 1.25).

## Mesh

| Component | Element Size | Nodes | Elements | Method |
|---|---|---|---|---|
| Fin | 0.85 mm | 73,469 | 15,088 | Hex dominant |
| Nose cone | 4.0 mm | — | 19,884 | Automatic (tet) |

Fin mesh size consistent with Project 02 finest convergence level (0.9mm).
Nose cone mesh coarsened from 2mm to 4mm to satisfy Ansys 2026 R1 Student
license node limit (512k nodes). Informal 3-level convergence check performed
on static structural only — no formal GCI computed. See Project 02 for full
5-level Richardson extrapolation mesh convergence methodology.

## Analysis Justification

**Transient vs steady-state thermal:** Heat flux is time-varying — peaks at
burnout (t=8s) and decays as speed drops post-burnout. Steady-state would
assume peak flux for infinite time, severely overpredicting temperature.
Transient thermal captures the actual temperature rise history.

**Prestressed modal vs plain modal:** The 10,000g setback load and thermal
stress field alter the structural stiffness matrix. Prestressed modal (linking
Static Structural solution to Modal setup) captures stress stiffening effects
on natural frequencies. The effect is small at these stress levels but
represents industry-standard practice for missile structural analysis.

**Thermal-only static structural run:** Isolates the thermal stress contribution
from the mechanical contribution. Demonstrates that thermal stress dominates
(73% of fin combined stress, ~100% of nose cone combined stress), which has
direct implications for material selection and thermal management design.

**1σ vs 3σ for random vibration:** Ansys outputs 1σ (68.269% probability) by
default. Aerospace design convention uses 3σ (99.73% probability) for structural
margins. Safety factors reported at both 1σ and 3σ for completeness.

## Ansys Solver Settings

**Transient Thermal:**
- Solver: Program Controlled (MAPDL)
- Nonlinear: Yes (radiation BC introduces T^4 nonlinearity)
- Step End Time: 43.2 s
- Initial Time Step: 0.1 s / Min: 0.01 s / Max: 1.0 s
- Store Results: All Time Points
- Radiosity Solver: Program Controlled
- Flux Convergence: 1e-4
- Max Radiosity Iterations: 1000
- Solver Tolerance: 1e-7 W/mm²
- Over Relaxation: 0.1
- Hemicube Resolution: 10
- Heat Convergence: Program Controlled
- Temperature Convergence: Program Controlled
- Nonlinear Formulation: Program Controlled

**Static Structural:**
- Solver: Program Controlled (MAPDL)
- Large Deflection: Off
- Environment Temperature: 22°C
- Imported Body Temperature: Source Time = 43.2s, Binary File mapping,
  Manual body scope (Material ID = 1)

**Modal:**
- Solver: Block Lanczos (Program Controlled)
- Max Modes: 6
- Prestress: from Static Structural solution (thermal + mechanical combined)
- Large Deflection: Off — Ansys warning noted. At deformation levels of
  0.55 μm (fin) and 0.085 μm (cone), large deflection effects are negligible.

**Random Vibration:**
- Method: Mode Superposition
- Modes included: all 6 from Modal
- Output scale: 1 Sigma (68.269% probability)
- Results reported at both 1σ and 3σ

**Fatigue Tool:**
- Domain: Time
- Loading: Fully Reversed (R = -1)
- Analysis Type: Stress Life (S-N)
- Mean Stress Theory: None
- Stress Component: Equivalent Von Mises
- Fatigue Strength Factor Kf: 1.0
- Design Life: 1e8 cycles
- S-N curve: manually entered Ti-6Al-4V literature data (see Material section)

## Boundary Conditions

**Transient Thermal — both components:**
- Initial Temperature: 22°C (uniform)
- Heat Flux: tabular time-varying on leading edge face (fin) / tip fillet face (cone)
- Radiation: To Ambient, emissivity 0.3, ambient 22°C, all exposed outer surfaces
  excluding heat flux face, tab inner contact faces, and tab root faces

**Static Structural — both components:**
- Fixed Support: bottom tab face (fin) / base circular face (cone)
- Acceleration: 98,100 mm/s² (10,000g) along missile body axis (X direction)
- Imported Body Temperature: mapped from Transient Thermal solution at t=43.2s

**Random Vibration — both components:**
- PSD Acceleration: X axis, scoped to Fixed Support boundary condition
- Flat PSD, simplified MIL-STD-810 Method 514 missile/rocket environment:

| Frequency (Hz) | PSD (g²/Hz) | PSD ((mm/s²)²/Hz) |
|---|---|---|
| 20 | 0.04 | 3,849,444 |
| 80 | 0.04 | 3,849,444 |
| 350 | 0.04 | 3,849,444 |
| 500 | 0.04 | 3,849,444 |
| 2000 | 0.04 | 3,849,444 |

## Results

### Transient Thermal

| Component | Max Temp (°C) | Location | Min Temp (°C) |
|---|---|---|---|
| Fin | 248.79 | Leading edge tip | 22.077 |
| Nose cone | 128.91 | Nose tip | 22.0 |

### Static Structural — Thermal Only (acceleration suppressed)

| Component | Max Von Mises (MPa) | Max Deformation (μm) | Location |
|---|---|---|---|
| Fin | 3.263 | 0.437 | Leading edge |
| Nose cone | 1.066 | 0.064 | Tip |

### Static Structural — Combined (thermal + 10,000g axial)

| Component | Max Von Mises (MPa) | Max Deformation (μm) | Yield at temp (MPa) | Static SF |
|---|---|---|---|---|
| Fin | 4.459 | 0.550 | 774 | 173x |
| Nose cone | 1.064 | 0.085 | ~840 | 789x |

### Thermal vs Mechanical Stress Decomposition

| Component | Thermal only (MPa) | Combined (MPa) | Mechanical (MPa) | Thermal fraction |
|---|---|---|---|---|
| Fin | 3.263 | 4.459 | 1.196 | 73% |
| Nose cone | 1.066 | 1.064 | ~0 | ~100% |

Thermal stress dominates both components. The mechanical setback load contributes
only 27% of fin combined stress and is negligible for the nose cone. This means
thermal management and material thermal properties are more important design
drivers than structural geometry optimization for these components.

### Modal (prestressed from combined static structural)

**Fin:**

| Mode | Frequency (Hz) |
|---|---|
| 1 | 415.52 |
| 2 | 1754.3 |
| 3 | 2524.7 |
| 4 | 4610.3 |
| 5 | 5815.2 |
| 6 | — |

Mode 1 is the first bending mode — maximum deformation at the fin tip,
zero at the fixed root. Mode shape confirms tip as the critical location
for dynamic response.

**Nose cone:**

| Mode | Frequency (Hz) |
|---|---|
| 1 | 5570.4 |
| 2 | 5576.5 |
| 3 | 7399.8 |
| 4 | 7399.7 |
| 5 | 10426 |
| 6 | 10426 |

Nose cone modes appear in near-repeated pairs due to axisymmetric geometry —
two orthogonal bending directions have nearly identical frequencies. Physically
correct. Small differences between pairs due to mesh asymmetry from tetrahedral
elements.

### Random Vibration (1σ, flat PSD 0.04 g²/Hz, 20-2000 Hz)

| Component | Von Mises 1σ (MPa) | Deformation 1σ (mm) | Von Mises 3σ (MPa) | SF at 3σ |
|---|---|---|---|---|
| Fin | 10.566 | 0.00103 | 31.70 | 24x |
| Nose cone | ~0 (1.35e-6) | ~0 (4.35e-10) | ~0 | >>100x |

Random vibration is the governing load case for the fin — 10.57 MPa 1σ vs
4.46 MPa static combined. The nose cone first mode (5570 Hz) is well above
the 2000 Hz PSD cutoff — essentially no vibrational energy input to the
nose cone structure under this environment.

### Fatigue (stress life, R=-1, Ti-6Al-4V S-N curve, Kf=1)

| Component | Fatigue Life | Ansys SF (capped) | Estimated actual SF |
|---|---|---|---|
| Fin | 1e8 cycles | >15 | ~90x (500/5.52) |
| Nose cone | 1e8 cycles | >15 | ~1754x (500/0.285) |

Both components have infinite fatigue life under this combined load case.
Applied stress is far below the Ti-6Al-4V endurance limit (~500 MPa at 10^8
cycles). Ansys fatigue safety factor display is capped at 15 — actual values
estimated from endurance limit divided by max stress.

## Summary Table

| Component | Max Temp | Combined Stress | Yield at Temp | Static SF | Mode 1 | RV Stress 1σ | RV SF 3σ | Fatigue Life |
|---|---|---|---|---|---|---|---|---|
| Fin LE | 249°C | 4.46 MPa | 774 MPa | 173x | 415 Hz | 10.57 MPa | 24x | 1e8 cycles |
| Nose tip | 129°C | 1.06 MPa | ~840 MPa | 789x | 5570 Hz | ~0 MPa | >>100x | 1e8 cycles |
| Fin RT (Proj 02) | 22°C | 1.334 MPa | 880 MPa | 660x | — | — | — | — |

## Implications

- **Fin LE is the design-critical location** across all load cases — highest
  temperature, highest stress, lowest safety factor, only component with
  meaningful random vibration response.

- **Random vibration governs over static setback for the fin** — 10.57 MPa 1σ
  vs 4.46 MPa static. This is typical for missile fin design where broadband
  motor burn vibration exceeds the quasi-static inertial load. The 24x safety
  factor at 3σ provides substantial margin.

- **Thermal stress dominates** — thermal loading contributes 73% of fin combined
  stress. This means Ti-6Al-4V's thermal conductivity (7.19 W/m·°C) and thermal
  expansion coefficient (8.789e-6 /°C) are more important design parameters than
  geometric stiffness for this component. A material with lower thermal expansion
  would reduce thermal stress more effectively than geometric changes.

- **Thermal loading degrades the fin safety factor from 660x (Project 02 RT) to
  173x** — a 3.8x reduction. Still large, but demonstrates that the room-temperature
  FEA alone is not sufficient for a complete structural assessment.

- **Nose cone is thermally and structurally benign** in all load cases. The blunt
  10mm tip fillet and thicker wall distribute heat flux effectively. If the missile
  were to fly faster or longer, the fin LE would be the first component to approach
  yield — nose cone is not the design-limiting component.

- **No resonance risk under motor burn** — fin Mode 1 at 415 Hz is well above
  typical solid rocket motor fundamental excitation frequencies (<100 Hz for most
  tactical motors). Nose cone modes all above 5500 Hz — completely decoupled from
  realistic excitation sources.

- **Fatigue is not a concern** for a single-use ballistic missile at these load
  levels. Infinite life at 1e8 cycles with large margins. Fatigue would become
  relevant for a reusable or multi-shot system.

  System-level FMEA covering structural failure modes (fin launch failure, fin
  resonance, nose cone thermal failure) is in `08_gnc_monte_carlo/src/fmea.py`.
  All structural items have RPN < 10 — near-zero occurrence on a validated design.

## Assumptions

- Missile body acts as perfect heat sink at fin root — no heat conduction into
  body modeled
- Heat flux applied uniformly across the leading edge face — spatial concentration
  at the tip not resolved
- Radiation-only on exposed surfaces — forced convective cooling in flight not
  modeled (conservative, overpredicts peak temperature)
- Emissivity 0.3 for polished Ti-6Al-4V throughout flight (oxidation increases
  emissivity over time — conservative assumption)
- Lumped capacitance in Python aeroheating — uniform temperature through wall
  thickness assumed
- Fully reversed loading (R=-1) for fatigue — conservative, actual loading is
  pulsed positive (0 to max), not fully reversed
- Kf=1 — no notch sensitivity correction applied at tab-fin fillet
- Fixed support on bottom tab face only — simplification of slot contact mechanics
- Flat PSD random vibration spectrum — real motor burn PSD is frequency-dependent
- DKR formula applied at Mach 1.25 — formula developed for hypersonic reentry
- Temperature field assumed static during random vibration solve — no thermal-
  vibration coupling
- Linear structural response assumed in random vibration — valid at these
  deformation levels

## Limitations

- **DKR heat flux overprediction:** Method developed for hypersonic reentry
  (M > 5), applied here at low supersonic (M = 1.25). May overpredict heat flux
  by up to 2x at these speeds. Ansys temperatures should be treated as conservative
  upper bounds. A more appropriate low-supersonic method (e.g. Van Driest, reference
  temperature method) would improve accuracy.

- **Mesh convergence:** Informal 3-level check on static structural only, no formal
  GCI. Fin 0.85mm mesh consistent with Project 02 finest level. Nose cone 4mm mesh
  coarser than ideal due to student license node limit — stress at tip fillet may
  not be fully converged. A finer nose cone mesh would likely show higher tip stress.

- **No forced convection BC:** Radiation only on exposed surfaces. In supersonic
  flight, forced convective cooling from the boundary layer would increase heat loss
  and reduce peak temperature. Omitting convection is conservative.

- **No thermal contact resistance at fin root:** Perfect thermal contact with missile
  body assumed. Real joint would have finite contact resistance, reducing heat flow
  into body and increasing fin temperature slightly.

- **Fixed support BC:** Bottom tab face only for fin, base face for cone. More
  physically realistic BC for the fin would use inner slot contact faces with
  frictionless support. Same limitation documented in Project 02. Cone attachment
  in reality would be threaded — more compliant than fixed support.

- **Fatigue S-N data:** Manually entered from literature, not Granta certified.
  Kf=1 ignores stress concentration at tab-fin fillet — actual Kf ~1.2-1.5 for
  a machined Ti fillet, which would reduce fatigue life estimate by 20-50%.
  Fatigue SF Ansys display capped at 15 — actual estimated values ~90x (fin)
  and ~1754x (cone).

- **Temperature-dependent modulus:** Not confirmed as active in structural solve.
  Ti-6Al-4V Young's modulus decreases ~10% from RT to 250°C — minor effect at
  these temperatures but worth confirming in future work.

- **Random vibration PSD:** Simplified flat spectrum 0.04 g²/Hz, 20-2000 Hz.
  Real solid rocket motor PSD typically has higher energy density at low
  frequencies and roll-off at high frequencies. This flat spectrum may
  overpredict high-frequency response and underpredict low-frequency response.

- **No spatial heat flux variation:** Heat flux applied uniformly on leading edge
  face. In reality heat flux is highest at the tip and decreases spanwise. A
  spatially varying BC would produce a more accurate temperature gradient.

- **Nose cone geometry simplified:** No internal stiffeners, no attachment threads,
  no payload interface. Real nose cone structural response may differ from this
  simple conical shell.

## How to Recreate

**Python aeroheating:**
python 07_thermal_structural/src/aeroheating.py
python 07_thermal_structural/src/export_heatflux.py

Requires Project 06 `run_sixdof()` on sys.path.

**Ansys:**
Open `07_thermal_structural/ansys/thermal_structural.wbpj` in Ansys Workbench
2026 R1 Student. All systems are pre-configured with boundary conditions and
results. Re-solve in upstream-to-downstream order per the analysis chain above.

## Repository Structure

    07_thermal_structural/
    ├── README_07_Thermal_Structural.md
    ├── cad/
    │   ├── fin_assembly.x_t
    │   └── nose_cone.x_t
    ├── src/
    │   ├── aeroheating.py
    │   └── export_heatflux.py
    ├── ansys/
    │   └── thermal_structural.wbpj
    └── results/
        ├── aeroheating.csv
        ├── fin_LE_heatflux.csv
        ├── nose_heatflux.csv
        └── figures/