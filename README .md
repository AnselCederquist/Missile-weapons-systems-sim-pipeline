# Weapons Systems Simulation Pipeline

**Ansel Cederquist** · Mechanical Engineer — Simulation, GNC \& Structural Analysis  
[LinkedIn](https://www.linkedin.com/in/ansel-cederquist-154080375/) · [Email](mailto:anselcederquist@outlook.com)

\---

> \*\*One-paragraph summary for recruiters:\*\*  
> Full weapons system simulation pipeline: aerodynamic coefficient database generated via Ansys Fluent CFD across Mach 0.8–3.0 → fed into a 6-DOF flight mechanics simulator with Proportional Navigation guidance → validated with Extended Kalman Filter state estimation across 500 Monte Carlo engagement scenarios. Structural and thermo-structural analysis of munition components conducted in Ansys Mechanical under high-g setback and sustained burn conditions.

\---

## Repository Structure

```
weapons-systems-sim-pipeline/
│
├── 01\_airfoil\_study/               # Project 1 — Airfoil Aerodynamic Study
│   ├── README.md
│   ├── xflr5\_project/              # XFLR5 project files
│   ├── postprocess/
│   │   └── plot\_polars.py          # Cl/Cd polar plots, parametric comparison
│   ├── results/
│   │   ├── figures/                # Polar plots, pressure distributions
│   │   └── data/                   # Raw XFLR5 output files
│   └── report/
│       └── airfoil\_study.pdf       # AIAA-format technical report
│
├── 02\_munition\_fea/                # Project 2 — Structural FEA of Munition Component
│   ├── README.md
│   ├── cad/
│   │   └── fin\_assembly.step       # Exported CAD geometry
│   ├── ansys/
│   │   └── fin\_assembly.wbpj       # Ansys Workbench project
│   ├── postprocess/
│   │   ├── mesh\_convergence.py     # Mesh convergence + Richardson extrapolation
│   │   └── stress\_plots.py
│   ├── results/
│   │   ├── figures/                # Stress contours, deformation plots
│   │   └── mesh\_convergence.csv    # Element count vs. max stress data
│   └── report/
│       └── munition\_fea.pdf
│
├── 03\_nozzle\_cfd/                  # Project 3 — Nozzle/Inlet CFD Analysis
│   ├── README.md
│   ├── cad/
│   │   └── nozzle\_profile.step
│   ├── ansys/
│   │   └── nozzle.wbpj             # Ansys Fluent project (density-based, compressible)
│   ├── postprocess/
│   │   ├── mesh\_convergence.py
│   │   └── validate\_isentropic.py  # Compare CFD vs. isentropic relations
│   ├── results/
│   │   └── figures/                # Mach contours, pressure plots
│   └── report/
│       └── nozzle\_cfd.pdf
│
├── 04\_missile\_aero\_database/       # Project 4 — Missile CFD Aerodynamic Database
│   ├── README.md
│   ├── cad/
│   │   └── missile\_body.step       # Ogive-cylinder-fin geometry
│   ├── ansys/
│   │   └── missile\_aero.wbpj       # Ansys Fluent project — parametric sweep setup
│   ├── postprocess/
│   │   ├── extract\_coefficients.py # Parse Fluent reports → CL, CD, Cm tables
│   │   └── plot\_aero\_database.py   # 3D surface plots, coefficient curves
│   ├── results/
│   │   ├── aero\_database.csv       # Full coefficient table — feeds 6-DOF sim
│   │   └── figures/
│   └── report/
│       └── missile\_aero\_database.pdf
│
├── 05\_kalman\_filter/               # Project 5 — Extended Kalman Filter Target Tracker
│   ├── README.md
│   ├── src/
│   │   ├── ekf.py                  # EKF implementation (predict + update)
│   │   ├── target\_model.py         # Maneuvering target dynamics
│   │   └── sensor\_model.py         # Radar measurement model + noise
│   ├── notebooks/
│   │   └── ekf\_demo.ipynb          # Interactive Jupyter walkthrough
│   ├── tests/
│   │   └── test\_ekf.py             # Unit tests
│   ├── results/
│   │   └── figures/                # True vs. estimated trajectory plots
│   └── report/
│       └── kalman\_filter.pdf
│
├── 06\_6dof\_missile\_sim/            # Project 6 — 6-DOF Missile Flight Simulator (CAPSTONE)
│   ├── README.md
│   ├── src/
│   │   ├── missile.py              # Missile body — mass, inertia, geometry
│   │   ├── aerodynamics.py         # Aero model — loads aero\_database.csv
│   │   ├── propulsion.py           # Thrust curve model
│   │   ├── guidance.py             # Proportional Navigation (PN, TPN, APN)
│   │   ├── equations\_of\_motion.py  # 6-DOF EOM — forces, moments, kinematics
│   │   ├── integrator.py           # RK4 / scipy solve\_ivp wrapper
│   │   └── monte\_carlo.py          # Monte Carlo engagement analysis
│   ├── notebooks/
│   │   ├── single\_engagement.ipynb # Single trajectory walkthrough
│   │   └── monte\_carlo.ipynb       # 500-run Monte Carlo with miss distance stats
│   ├── tests/
│   │   └── test\_guidance.py
│   ├── results/
│   │   ├── figures/                # 3D trajectory plots, miss distance histograms
│   │   └── monte\_carlo\_results.csv
│   └── report/
│       └── 6dof\_missile\_sim.pdf
│
├── 07\_thermal\_structural/          # Project 7 — Thermo-Structural Rocket Motor Casing
│   ├── README.md
│   ├── cad/
│   │   └── motor\_casing.step
│   ├── ansys/
│   │   └── motor\_casing.wbpj       # Linked: Steady-State Thermal → Structural
│   ├── postprocess/
│   │   └── temperature\_plots.py    # Temperature vs. time, stress margin plots
│   ├── results/
│   │   └── figures/                # Temp distribution, thermal stress contours
│   └── report/
│       └── thermal\_structural.pdf
│
├── docs/
│   ├── pipeline\_overview.png       # Block diagram — the full integrated pipeline
│   └── resume\_paragraph.md         # Copy-paste ready resume bullet text
│
├── environment.yml                 # Conda environment — reproduce results instantly
├── requirements.txt                # pip alternative
└── README.md                       # This file
```

\---

## Projects Overview

### 01 · Airfoil Aerodynamic Study

Parametric aerodynamic analysis of NACA airfoil series using XFLR5/XFOIL across a sweep of Reynolds numbers and angles of attack. Cl/Cd polar optimization for UAV cruise and missile canard applications.

**Tools:** XFLR5, Python (NumPy, Pandas, Matplotlib)  
**Key result:** NACA 0012 selected for missile canard — CLmax 1.08 at stall AoA 11°, L/D 60.4 at Re=500k M=0.3. Parametric study across 4 airfoils (NACA 0006/0008/0012/0015), 3 Reynolds numbers (500k/1M/2M), and 2 Mach numbers (0.3/0.5) — 24 total polar sweeps. NACA 0006 data unreliable at Re=500k (unresolved boundary layer). Thicker airfoils show higher CLmax and later stall; NACA 0012 optimal tradeoff between aerodynamic efficiency and structural depth for fin application.

── Airfoil Summary ───────────────────────────────
| Polar                      | CLmax | Stall AoA | Max L/D | CD@CL=0.5 |
| -------------------------- | ----- | --------- | ------- | --------- |
| NACA 0006 Re=0.500M M=0.30 | 0.695 | 7.0       | 42.9    | 0.0138    |
| NACA 0006 Re=0.500M M=0.50 | 0.643 | 5.5       | 41.6    | 0.0145    |
| NACA 0006 Re=1.000M M=0.30 | 0.618 | 5.5       | 51.7    | 0.0104    |
| NACA 0006 Re=1.000M M=0.50 | 0.650 | 5.5       | 49.6    | 0.0115    |
| NACA 0006 Re=2.000M M=0.30 | 0.728 | 10.0      | 63.2    | 0.0075    |
| NACA 0006 Re=2.000M M=0.50 | 0.626 | 5.0       | 63.5    | 0.0074    |
| NACA 0008 Re=0.500M M=0.30 | 0.822 | 8.0       | 48.8    | 0.0101    |
| NACA 0008 Re=0.500M M=0.50 | 0.751 | 6.5       | 47.9    | 0.0103    |
| NACA 0008 Re=1.000M M=0.30 | 0.908 | 20.0      | 61.8    | 0.0084    |
| NACA 0008 Re=1.000M M=0.50 | 0.781 | 6.5       | 59.9    | 0.0085    |
| NACA 0008 Re=2.000M M=0.30 | 0.947 | 8.5       | 76.7    | 0.0071    |
| NACA 0008 Re=2.000M M=0.50 | 0.816 | 6.5       | 73.5    | 0.0076    |
| NACA 0012 Re=0.500M M=0.30 | 1.077 | 11.0      | 60.4    | 0.0094    |
| NACA 0012 Re=0.500M M=0.50 | 0.936 | 8.5       | 56.9    | 0.0105    |
| NACA 0012 Re=1.000M M=0.30 | 1.219 | 12.5      | 73.5    | 0.0083    |
| NACA 0012 Re=1.000M M=0.50 | 1.020 | 8.5       | 68.1    | 0.0084    |
| NACA 0012 Re=2.000M M=0.30 | 1.355 | 13.0      | 88.4    | 0.0072    |
| NACA 0012 Re=2.000M M=0.50 | 1.104 | 9.0       | 82.3    | 0.0074    |
| NACA 0015 Re=0.500M M=0.30 | 1.155 | 13.0      | 63.1    | 0.0100    |
| NACA 0015 Re=0.500M M=0.50 | 0.997 | 10.0      | 55.2    | 0.0104    |
| NACA 0015 Re=1.000M M=0.30 | 1.290 | 14.0      | 75.0    | 0.0082    |
| NACA 0015 Re=1.000M M=0.50 | 1.106 | 10.0      | 67.7    | 0.0086    |
| NACA 0015 Re=2.000M M=0.30 | 1.449 | 15.5      | 88.9    | 0.0073    |
| NACA 0015 Re=2.000M M=0.50 | 1.224 | 11.0      | 82.9    | 0.0077    |


\---

### 02 · Structural FEA — Munition Component

Static structural FEA of a Ti-6Al-4V missile fin assembly under 10,000g axial 
setback load in Ansys Mechanical. Five-level mesh convergence study with 
Richardson extrapolation and GCI analysis.

**Tools:** Creo Parametric 12.4, Ansys Mechanical (Student), Python (NumPy, 
Matplotlib, Pandas)

**Methodology:** **Methodology:** Global mesh refinement across five levels (3.0 → 0.85mm, 
960 → 124,018 elements) within Ansys Student 128k node limit. Fixed support 
BC singularity identified at outer root tab corners and excluded from 
convergence reporting. True stress concentration identified at negative-X 
inner fillet (R3 radius, fin-to-tab transition) — location stabilized 
spatially with mesh refinement confirming a real geometric stress concentration 
rather than a numerical artifact. At coarser mesh levels the maximum 
non-singularity stress initially appeared on the positive-X inner fillet before 
migrating and stabilizing at the negative-X fillet with further refinement.

Richardson extrapolation was applied to total deformation (GCI < 0.001%, fully 
converged). Richardson extrapolation for Von Mises stress was invalid due to 
non-monotonic convergence (1.532 → 1.382 → 1.289 → 1.336 → 1.378 MPa) — 
attributed to low-quality elements near the fillet. Stress reported as mean ± 
1σ of finest three mesh levels. 

Persistent low-quality elements identified at the tab-to-fin geometric 
transition zone due to the abrupt cross-section change from rectangular root 
tab to tapered fin profile. This meshing artifact persisted across all attempted 
mesh control strategies including global element size refinement (3.0 → 0.85mm), 
Face Sizing applied directly to fillet surfaces, Sphere of Influence centered on 
the fillet region, and Hex Dominant and Sweep automatic meshing methods. The 
low-quality elements are concentrated along the two short Z-direction edges at 
the tab-fin junction — the 4mm thickness edges where the flat tab end face meets 
the R3 fillet surface on both positive-X and negative-X sides, creating an 
abrupt 90° surface normal transition the hex sweep mesher cannot resolve cleanly.

Future work: apply a small chamfer (1-2mm) in Creo Parametric along these two 
Z-direction edges at the tab-fin junction. This would smooth the surface normal 
transition and allow the hex sweep mesher to generate higher quality elements 
through the transition volume without significantly altering structural geometry 
or stress results. Additionally, local mesh refinement (Face Sizing or Sphere of 
Influence on fillet surfaces) is recommended over global refinement for future 
studies — would reduce total element count by ~90% while achieving equivalent 
resolution at the critical zone.

**Key result:** Max Von Mises stress 1.334 ± 0.036 MPa at inner root fillet — 
safety factor 660× vs Ti-6Al-4V yield (880 MPa). Total deformation converged 
to 1.363 μm (GCI < 0.001%). Fin geometry significantly overdesigned for this 
load case — weight optimization via geometry reduction is recommended as 
future work.

\---

### 03 · Nozzle CFD Analysis

Compressible flow simulation of a De Laval nozzle in Ansys Fluent (density-based, coupled solver). Mach number and pressure distributions validated against isentropic flow relations. 3-level mesh convergence with Richardson extrapolation.

**Tools:** Solid Edge, Ansys Meshing, Ansys Fluent, Ansys CFD-Post, Python  
**Key result:** *\[e.g. "Throat Mach number = 1.003 vs. theoretical 1.0 — 0.3% error; exit pressure recovery within 1.1% of isentropic prediction"]*

\---

### 04 · Missile Aerodynamic Database (CFD)

Full aerodynamic coefficient database (CL, CD, Cm, CN) for a generic ogive-cylinder-fin missile body generated via Ansys Fluent across Mach 0.8–3.0 and AoA 0°–20°. Database directly feeds the 6-DOF simulator.

**Tools:** Solid Edge, Ansys Meshing, Ansys Fluent, Ansys CFD-Post, Python  
**Key result:** *\[e.g. "108-condition sweep (4 Mach × 5 AoA × 3 control states) — full coefficient tables exported to CSV for 6-DOF ingestion"]*

\---

### 05 · Extended Kalman Filter — Target Tracker

EKF implementation for tracking a maneuvering target from simulated noisy radar measurements. Compared against true trajectory generated by constant-acceleration target model.

**Tools:** Python (NumPy, SciPy, Matplotlib)  
**Key result:** *\[e.g. "RMS position error 4.2m at SNR=20dB against 3g maneuvering target — 6× improvement over unfiltered measurement"]*

\---

### 06 · 6-DOF Missile Flight Simulator *(Capstone)*

Full 6-degree-of-freedom flight mechanics simulator for a guided missile. Aerodynamic inputs from Project 04 CFD database. Proportional Navigation guidance (PN, TPN, APN variants). Monte Carlo miss distance analysis across 500 engagement scenarios.

**Tools:** Python (NumPy, SciPy, Matplotlib)  
**Key result:** *\[e.g. "Mean miss distance 0.41m (P50) / 0.87m (P90) against 3g maneuvering target across 500 Monte Carlo runs with ±15% aero uncertainty"]*

\---

### 07 · Thermo-Structural Analysis — Rocket Motor Casing

Coupled thermo-structural FEA of a solid rocket motor casing in Ansys Mechanical. Steady-state and transient thermal analysis under sustained burn conditions (internal heat flux, external convection), with temperature field imported as body load into static structural analysis.

**Tools:** Solid Edge, Ansys Mechanical (Thermal + Structural linked)  
**Key result:** *\[e.g. "Peak wall temperature 487°C at burnout (8s); combined thermal-mechanical stress 278 MPa — within Inconel 718 allowable with 1.6× margin"]*

\---

## Integrated Pipeline

```
\[Project 01]              \[Project 04]
Airfoil Study      →      Missile CFD Aero Database (CL, CD, Cm vs. Mach, AoA)
XFLR5 / XFOIL             Ansys Fluent / Ansys Meshing
                                    │
                                    ▼
                          \[Project 06]
                          6-DOF Missile Flight Simulator
                          Python — Forces, Moments, EOM, PN Guidance
                                    │
                    ┌───────────────┤
                    ▼               ▼
           \[Project 05]     Monte Carlo Analysis
           EKF Target        500 engagement runs
           Tracker           Miss distance statistics
           Python

\[Project 02]              \[Project 07]
Munition FEA       +      Thermo-Structural Rocket Casing
Ansys Mechanical          Ansys Thermal → Structural (coupled)
High-g setback            Sustained burn conditions
```

\---

## Resume Bullet (Copy-Paste Ready)

**For defense / weapons engineer applications:**

> Developed full weapons system simulation pipeline: aerodynamic coefficient database (CL, CD, Cm) generated via Ansys Fluent CFD across Mach 0.8–3.0 → ingested into a 6-DOF flight mechanics simulator with Proportional Navigation guidance → validated against Extended Kalman Filter state estimation across 500 Monte Carlo engagement scenarios; supplemented with Ansys thermo-structural analysis of rocket motor casing under sustained burn and high-g loading conditions.

**For simulation / analysis engineer applications:**

> Built integrated simulation pipeline spanning CFD (Ansys Fluent, compressible Mach 0.8–3.0), FEA (Ansys Mechanical, nonlinear structural and coupled thermo-structural), and flight dynamics (6-DOF Python, PN guidance, Monte Carlo); applied mesh convergence and Richardson extrapolation methodology across CFD and FEA studies to ensure grid-independent solutions.

\---

## Environment Setup

```bash
# Clone the repo
git clone https://github.com/AnselCederquist/Weapons-systems-sim-pipeline.git
cd Weapons-systems-sim-pipeline

# Python environment (conda)
conda env create -f environment.yml
conda activate weapons-sim

# Or pip
pip install -r requirements.txt
```

**requirements.txt:**

```
numpy>=1.24
scipy>=1.10
matplotlib>=3.7
pandas>=2.0
jupyter>=1.0
pytest>=7.0
```

**External tools required (free):**

* [Ansys Student](https://www.ansys.com/academic/students) — CFD (Fluent) + FEA (Mechanical), no enrollment required
* [Solid Edge Community Edition](https://www.siemens.com/solidedge-community) — CAD geometry, no enrollment required
* [XFLR5](http://www.xflr5.tech/) — airfoil analysis

\---

## Disclaimer

All simulations use generic, publicly available geometry and reference data. No export-controlled, classified, or proprietary information is contained in this repository. This project is for educational and portfolio purposes only.

\---

## Author

**Ansel Cederquist**  
B.S. Mechanical Engineering, CSU Sacramento (2025) · GPA 3.87  
CSWA Certified · Ansys Mechanical · Ansys Fluent · Python  
Davis, CA · 530-304-2696 · anselcederquist@outlook.com

