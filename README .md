# Weapons Systems Simulation Pipeline

**Ansel Cederquist** · Mechanical Engineer — Simulation, GNC \& Structural Analysis  
[LinkedIn](https://www.linkedin.com/in/ansel-cederquist-154080375/) · [Email](mailto:anselcederquist@outlook.com)

---

> > Full weapons system simulation pipeline (in progress): NACA airfoil aerodynamic study (XFLR5, 24-polar parametric sweep), Ti-6Al-4V fin assembly structural FEA under 10,000g setback (Ansys Mechanical, Richardson extrapolation, GCI), and De Laval nozzle compressible CFD (density-based Fluent, isentropic validation, mesh convergence) completed. Aerodynamic coefficient database (Project 04), 6-DOF flight mechanics simulator with Proportional Navigation guidance (Project 06), Extended Kalman Filter state estimation (Project 05), and thermo-structural rocket motor casing analysis (Project 07) in progress.

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
├── 03_nozzle_cfd/                  # Project 3 — Nozzle CFD Analysis
│   ├── README.md
│   ├── cad/
│   │   ├── nozzle_study.stp
│   │   ├── Nozzle_study_2d_conical.step
│   │   ├── Nozzle_study_2d_conical_axisymmetric.step
│   │   ├── Nozzle_study_3d_conical.step
│   │   └── Nozzle_3d_conical_PlumeAnalysis.x_t
│   ├── ansys/
│   │   └── Nozzle_2d.wbpj          # All 4 studies (FFF, FFF-1, FFF-2, FFF-3)
│   ├── postprocess/
│   │   ├── mesh_convergence.py     # Richardson extrapolation + GCI
│   │   └── validate_isentropic.py  # CFD vs isentropic theory
│   ├── results/
│   │   └── figures/                # Contours, centerline plots, convergence plots
│   └── report/
│       └── nozzle_cfd.pdf
│
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

Parametric aerodynamic analysis of NACA airfoil series using XFLR5/XFOIL across a sweep of Reynolds numbers and angles of attack. Cl/Cd polar optimization for missile canard application.

**Tools:** XFLR5, Python (NumPy, Pandas, Matplotlib)

**Key result:** NACA 0012 selected for missile canard — CLmax 1.08 at stall AoA 11°, L/D 60.4 at Re=500k M=0.3. Parametric study across 4 airfoils (NACA 0006/0008/0012/0015), 3 Reynolds numbers (500k/1M/2M), and 2 Mach numbers (0.3/0.5) — 24 total polar sweeps. NACA 0006 data unreliable at Re=500k (unresolved boundary layer). Results valid for subsonic regime (M < 0.4); Prandtl-Glauert correction applied — M=0.5 cases are approximate. Full polar table and validity discussion in project README.

---

### 02 · Structural FEA — Munition Component

Static structural FEA of a Ti-6Al-4V missile fin assembly under 10,000g axial setback load in Ansys Mechanical. Five-level mesh convergence study with Richardson extrapolation and GCI analysis.

**Tools:** Creo Parametric 12.4, Ansys Mechanical (Student), Python (NumPy, Matplotlib, Pandas)

**Key result:** Max Von Mises stress 1.334 ± 0.036 MPa at inner root fillet (mean ± 1σ of finest three mesh levels — stress non-monotonic due to low-quality elements at tab-fin junction; Richardson extrapolation invalid for stress). Safety factor 660× vs Ti-6Al-4V yield (880 MPa). Total deformation Richardson-extrapolated to 1.363 μm (GCI < 0.001%, fully converged). Fixed support singularity identified and excluded from convergence reporting. Full methodology, convergence analysis, and safety factor justification in project README.
---


### 03 · Nozzle CFD Analysis

Compressible flow simulation of a conical De Laval nozzle (Ae/A* = 16.67) in Ansys Fluent across four mesh refinement levels. Mach number and pressure distributions validated against isentropic flow relations. Supplemented by hot-fire exhaust plume study at representative rocket chamber conditions (P₀ = 9.8 MPa, T₀ = 3710 K).

**Tools:** Onshape, Ansys Fluent (density-based and pressure-based coupled, compressible), Ansys Meshing, Python (NumPy, Matplotlib, SciPy)

**Key result:** Exit Mach Richardson-extrapolated to 2.745 (GCI < 0.5%, grid-independent) at off-design cold-flow condition (P0/P_back = 5.94; normal shock present in divergent section — see Project 03 README). Throat Mach 0.829 (GCI 4.1%); mass flow conservation 0.010% on fine mesh. Hot-fire plume study (P0/P = 96.7) confirms fully started nozzle at representative chamber conditions.

---

### 04 · Missile Aerodynamic Database (CFD)

Full aerodynamic coefficient database (CL, CD, Cm, CN) for a generic ogive-cylinder-fin missile body generated via Ansys Fluent across Mach 0.8–3.0 and AoA 0°–20°. Database directly feeds the 6-DOF simulator.

**Tools:** Solid Edge, Ansys Meshing, Ansys Fluent, Ansys CFD-Post, Python  
**Key result:** In progress.



---

### 05 · Extended Kalman Filter — Target Tracker

EKF implementation for tracking a maneuvering target from simulated noisy radar measurements. Compared against true trajectory generated by constant-acceleration target model.

**Tools:** Python (NumPy, SciPy, Matplotlib)  
**Key result:** In progress.

---

### 06 · 6-DOF Missile Flight Simulator *(Capstone)*

Full 6-degree-of-freedom flight mechanics simulator for a guided missile. Aerodynamic inputs from Project 04 CFD database. Proportional Navigation guidance (PN, TPN, APN variants). Monte Carlo miss distance analysis across 500 engagement scenarios.

**Tools:** Python (NumPy, SciPy, Matplotlib)  
**Key result:** In progress.

---

### 07 · Thermo-Structural Analysis — Rocket Motor Casing

Coupled thermo-structural FEA of a solid rocket motor casing in Ansys Mechanical. Steady-state and transient thermal analysis under sustained burn conditions (internal heat flux, external convection), with temperature field imported as body load into static structural analysis.

**Tools:** Solid Edge, Ansys Mechanical (Thermal + Structural linked)  
**Key result:** In progress.

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


**For defense / weapons engineer applications:**

> Developing full weapons system simulation pipeline: planning aerodynamic coefficient database (CL, CD, Cm) generation via Ansys Fluent CFD across Mach 0.8–3.0 → ingestion into a 6-DOF flight mechanics simulator with Proportional Navigation guidance → validation against Extended Kalman Filter state estimation across 500 Monte Carlo engagement scenarios; supplemented with Ansys thermo-structural analysis of rocket motor casing under sustained burn and high-g loading conditions.

**For simulation / analysis engineer applications:**

> Building integrated simulation pipeline spanning CFD (Ansys Fluent, compressible Mach 0.8–3.0), FEA (Ansys Mechanical, nonlinear structural and coupled thermo-structural), and flight dynamics (6-DOF Python, PN guidance, Monte Carlo); applied mesh convergence and Richardson extrapolation methodology across CFD and FEA studies to ensure grid-independent solutions.

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

