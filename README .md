# Weapons Systems Simulation Pipeline

**Ansel Cederquist** · Mechanical Engineer — Simulation, GNC & Structural Analysis  
[LinkedIn](https://www.linkedin.com/in/ansel-cederquist-154080375/) · [Email](mailto:anselcederquist@outlook.com)

---

> **One-paragraph summary for recruiters:**  
> Full weapons system simulation pipeline: aerodynamic coefficient database generated via Ansys Fluent CFD across Mach 0.8–3.0 → fed into a 6-DOF flight mechanics simulator with Proportional Navigation guidance → validated with Extended Kalman Filter state estimation across 500 Monte Carlo engagement scenarios. Structural and thermo-structural analysis of munition components conducted in Ansys Mechanical under high-g setback and sustained burn conditions.

---

## Repository Structure

```
weapons-systems-sim-pipeline/
│
├── 01_airfoil_study/               # Project 1 — Airfoil Aerodynamic Study
│   ├── README.md
│   ├── xflr5_project/              # XFLR5 project files
│   ├── postprocess/
│   │   └── plot_polars.py          # Cl/Cd polar plots, parametric comparison
│   ├── results/
│   │   ├── figures/                # Polar plots, pressure distributions
│   │   └── data/                   # Raw XFLR5 output files
│   └── report/
│       └── airfoil_study.pdf       # AIAA-format technical report
│
├── 02_munition_fea/                # Project 2 — Structural FEA of Munition Component
│   ├── README.md
│   ├── cad/
│   │   └── fin_assembly.step       # Exported CAD geometry
│   ├── ansys/
│   │   └── fin_assembly.wbpj       # Ansys Workbench project
│   ├── postprocess/
│   │   ├── mesh_convergence.py     # Mesh convergence + Richardson extrapolation
│   │   └── stress_plots.py
│   ├── results/
│   │   ├── figures/                # Stress contours, deformation plots
│   │   └── mesh_convergence.csv    # Element count vs. max stress data
│   └── report/
│       └── munition_fea.pdf
│
├── 03_nozzle_cfd/                  # Project 3 — Nozzle/Inlet CFD Analysis
│   ├── README.md
│   ├── cad/
│   │   └── nozzle_profile.step
│   ├── ansys/
│   │   └── nozzle.wbpj             # Ansys Fluent project (density-based, compressible)
│   ├── postprocess/
│   │   ├── mesh_convergence.py
│   │   └── validate_isentropic.py  # Compare CFD vs. isentropic relations
│   ├── results/
│   │   └── figures/                # Mach contours, pressure plots
│   └── report/
│       └── nozzle_cfd.pdf
│
├── 04_missile_aero_database/       # Project 4 — Missile CFD Aerodynamic Database
│   ├── README.md
│   ├── cad/
│   │   └── missile_body.step       # Ogive-cylinder-fin geometry
│   ├── ansys/
│   │   └── missile_aero.wbpj       # Ansys Fluent project — parametric sweep setup
│   ├── postprocess/
│   │   ├── extract_coefficients.py # Parse Fluent reports → CL, CD, Cm tables
│   │   └── plot_aero_database.py   # 3D surface plots, coefficient curves
│   ├── results/
│   │   ├── aero_database.csv       # Full coefficient table — feeds 6-DOF sim
│   │   └── figures/
│   └── report/
│       └── missile_aero_database.pdf
│
├── 05_kalman_filter/               # Project 5 — Extended Kalman Filter Target Tracker
│   ├── README.md
│   ├── src/
│   │   ├── ekf.py                  # EKF implementation (predict + update)
│   │   ├── target_model.py         # Maneuvering target dynamics
│   │   └── sensor_model.py         # Radar measurement model + noise
│   ├── notebooks/
│   │   └── ekf_demo.ipynb          # Interactive Jupyter walkthrough
│   ├── tests/
│   │   └── test_ekf.py             # Unit tests
│   ├── results/
│   │   └── figures/                # True vs. estimated trajectory plots
│   └── report/
│       └── kalman_filter.pdf
│
├── 06_6dof_missile_sim/            # Project 6 — 6-DOF Missile Flight Simulator (CAPSTONE)
│   ├── README.md
│   ├── src/
│   │   ├── missile.py              # Missile body — mass, inertia, geometry
│   │   ├── aerodynamics.py         # Aero model — loads aero_database.csv
│   │   ├── propulsion.py           # Thrust curve model
│   │   ├── guidance.py             # Proportional Navigation (PN, TPN, APN)
│   │   ├── equations_of_motion.py  # 6-DOF EOM — forces, moments, kinematics
│   │   ├── integrator.py           # RK4 / scipy solve_ivp wrapper
│   │   └── monte_carlo.py          # Monte Carlo engagement analysis
│   ├── notebooks/
│   │   ├── single_engagement.ipynb # Single trajectory walkthrough
│   │   └── monte_carlo.ipynb       # 500-run Monte Carlo with miss distance stats
│   ├── tests/
│   │   └── test_guidance.py
│   ├── results/
│   │   ├── figures/                # 3D trajectory plots, miss distance histograms
│   │   └── monte_carlo_results.csv
│   └── report/
│       └── 6dof_missile_sim.pdf
│
├── 07_thermal_structural/          # Project 7 — Thermo-Structural Rocket Motor Casing
│   ├── README.md
│   ├── cad/
│   │   └── motor_casing.step
│   ├── ansys/
│   │   └── motor_casing.wbpj       # Linked: Steady-State Thermal → Structural
│   ├── postprocess/
│   │   └── temperature_plots.py    # Temperature vs. time, stress margin plots
│   ├── results/
│   │   └── figures/                # Temp distribution, thermal stress contours
│   └── report/
│       └── thermal_structural.pdf
│
├── docs/
│   ├── pipeline_overview.png       # Block diagram — the full integrated pipeline
│   └── resume_paragraph.md         # Copy-paste ready resume bullet text
│
├── environment.yml                 # Conda environment — reproduce results instantly
├── requirements.txt                # pip alternative
└── README.md                       # This file
```

---

## Projects Overview

### 01 · Airfoil Aerodynamic Study

Parametric aerodynamic analysis of NACA airfoil series using XFLR5/XFOIL across a sweep of Reynolds numbers and angles of attack. Cl/Cd polar optimization for UAV cruise and missile canard applications.

**Tools:** XFLR5, Python (Matplotlib)  
**Key result:** NACA 0012 selected — CLmax 1.08, stall at 11°, L/D 60.4; best tradeoff between aerodynamic efficiency and structural depth for missile canard application. NACA 0006 data unreliable at Re=500k (zero CD values indicate unresolved boundary layer).

| Airfoil | CLmax | Stall AoA | Max L/D | CD @ CL=0.5 |
|---|---|---|---|---|
| NACA 0006 | — | — | — | — |
| NACA 0008 | 0.82 | 8° | 48.8 | 0.0101 |
| NACA 0012 | 1.08 | 11° | 60.4 | 0.0094 |
| NACA 0015 | 1.16 | 13° | 63.1 | 0.0100 |

---

### 02 · Structural FEA — Munition Component

Static structural and linear buckling FEA of a fin assembly under artillery-level setback loads (10,000g+) in Ansys Mechanical. Includes 4-level mesh convergence study with Richardson extrapolation to grid-independent solution.

**Tools:** Solid Edge, Ansys Mechanical  
**Key result:** *[e.g. "Max von Mises stress 312 MPa at fin root — 1.8× safety margin against Ti-6Al-4V yield; mesh-independent at 185k elements (<1.2% change from 740k mesh)"]*

---

### 03 · Nozzle CFD Analysis

Compressible flow simulation of a De Laval nozzle in Ansys Fluent (density-based, coupled solver). Mach number and pressure distributions validated against isentropic flow relations. 3-level mesh convergence with Richardson extrapolation.

**Tools:** Solid Edge, Ansys Meshing, Ansys Fluent, Ansys CFD-Post, Python  
**Key result:** *[e.g. "Throat Mach number = 1.003 vs. theoretical 1.0 — 0.3% error; exit pressure recovery within 1.1% of isentropic prediction"]*

---

### 04 · Missile Aerodynamic Database (CFD)

Full aerodynamic coefficient database (CL, CD, Cm, CN) for a generic ogive-cylinder-fin missile body generated via Ansys Fluent across Mach 0.8–3.0 and AoA 0°–20°. Database directly feeds the 6-DOF simulator.

**Tools:** Solid Edge, Ansys Meshing, Ansys Fluent, Ansys CFD-Post, Python  
**Key result:** *[e.g. "108-condition sweep (4 Mach × 5 AoA × 3 control states) — full coefficient tables exported to CSV for 6-DOF ingestion"]*

---

### 05 · Extended Kalman Filter — Target Tracker

EKF implementation for tracking a maneuvering target from simulated noisy radar measurements. Compared against true trajectory generated by constant-acceleration target model.

**Tools:** Python (NumPy, SciPy, Matplotlib)  
**Key result:** *[e.g. "RMS position error 4.2m at SNR=20dB against 3g maneuvering target — 6× improvement over unfiltered measurement"]*

---

### 06 · 6-DOF Missile Flight Simulator *(Capstone)*

Full 6-degree-of-freedom flight mechanics simulator for a guided missile. Aerodynamic inputs from Project 04 CFD database. Proportional Navigation guidance (PN, TPN, APN variants). Monte Carlo miss distance analysis across 500 engagement scenarios.

**Tools:** Python (NumPy, SciPy, Matplotlib)  
**Key result:** *[e.g. "Mean miss distance 0.41m (P50) / 0.87m (P90) against 3g maneuvering target across 500 Monte Carlo runs with ±15% aero uncertainty"]*

---

### 07 · Thermo-Structural Analysis — Rocket Motor Casing

Coupled thermo-structural FEA of a solid rocket motor casing in Ansys Mechanical. Steady-state and transient thermal analysis under sustained burn conditions (internal heat flux, external convection), with temperature field imported as body load into static structural analysis.

**Tools:** Solid Edge, Ansys Mechanical (Thermal + Structural linked)  
**Key result:** *[e.g. "Peak wall temperature 487°C at burnout (8s); combined thermal-mechanical stress 278 MPa — within Inconel 718 allowable with 1.6× margin"]*

---

## Integrated Pipeline

```
[Project 01]              [Project 04]
Airfoil Study      →      Missile CFD Aero Database (CL, CD, Cm vs. Mach, AoA)
XFLR5 / XFOIL             Ansys Fluent / Ansys Meshing
                                    │
                                    ▼
                          [Project 06]
                          6-DOF Missile Flight Simulator
                          Python — Forces, Moments, EOM, PN Guidance
                                    │
                    ┌───────────────┤
                    ▼               ▼
           [Project 05]     Monte Carlo Analysis
           EKF Target        500 engagement runs
           Tracker           Miss distance statistics
           Python

[Project 02]              [Project 07]
Munition FEA       +      Thermo-Structural Rocket Casing
Ansys Mechanical          Ansys Thermal → Structural (coupled)
High-g setback            Sustained burn conditions
```

---

## Resume Bullet (Copy-Paste Ready)

**For defense / weapons engineer applications:**

> Developed full weapons system simulation pipeline: aerodynamic coefficient database (CL, CD, Cm) generated via Ansys Fluent CFD across Mach 0.8–3.0 → ingested into a 6-DOF flight mechanics simulator with Proportional Navigation guidance → validated against Extended Kalman Filter state estimation across 500 Monte Carlo engagement scenarios; supplemented with Ansys thermo-structural analysis of rocket motor casing under sustained burn and high-g loading conditions.

**For simulation / analysis engineer applications:**

> Built integrated simulation pipeline spanning CFD (Ansys Fluent, compressible Mach 0.8–3.0), FEA (Ansys Mechanical, nonlinear structural and coupled thermo-structural), and flight dynamics (6-DOF Python, PN guidance, Monte Carlo); applied mesh convergence and Richardson extrapolation methodology across CFD and FEA studies to ensure grid-independent solutions.

---

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

---

## Disclaimer

All simulations use generic, publicly available geometry and reference data. No export-controlled, classified, or proprietary information is contained in this repository. This project is for educational and portfolio purposes only.

---

## Author

**Ansel Cederquist**  
B.S. Mechanical Engineering, CSU Sacramento (2025) · GPA 3.87  
CSWA Certified · Ansys Mechanical · Ansys Fluent · Python  
Davis, CA · 530-304-2696 · anselcederquist@outlook.com
