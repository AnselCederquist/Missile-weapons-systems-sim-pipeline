# Weapons Systems Simulation Pipeline

**Ansel Cederquist** · Mechanical Engineer — Simulation, GNC & Structural Analysis  
[LinkedIn](https://www.linkedin.com/in/ansel-cederquist-154080375/) · [Email](mailto:anselcederquist@outlook.com)

---

> Full weapons system simulation pipeline: NACA airfoil aerodynamic study (XFLR5, 24-polar parametric sweep), Ti-6Al-4V fin assembly structural FEA under 10,000g setback (Ansys Mechanical, Richardson extrapolation, GCI), De Laval nozzle compressible CFD (density-based Fluent, isentropic validation, mesh convergence), missile aerodynamic database (Digital DATCOM), Extended Kalman Filter state estimation, 6-DOF flight mechanics simulator, coupled thermal-structural-vibration-fatigue analysis (Ansys Workbench), full missile assembly CAD (Onshape), proportional navigation GNC with 500-run Monte Carlo engagement analysis, 13-state quaternion attitude control simulator with CARE-solved LQR and TPN/OGL/HYBRID guidance modes, three-loop acceleration autopilot, IR/RF seeker model, fragmentation lethality model, and launch envelope calculator. Thirteen projects, fully integrated pipeline.

---

## Repository Structure

```
weapons-systems-sim-pipeline/
│
├── 01_airfoil_study/               # Project 01 — Airfoil Aerodynamic Study
│   ├── README.md
│   ├── xflr5_project/
│   ├── postprocess/
│   │   └── plot_polars.py
│   └── results/
│       ├── figures/
│       └── data/
│
├── 02_munition_fea/                # Project 02 — Structural FEA: Missile Fin Assembly
│   ├── README.md
│   ├── cad/
│   │   └── fin_assembly.step
│   ├── ansys/
│   │   └── fin_assembly.wbpj
│   ├── postprocess/
│   │   ├── mesh_convergence.py
│   │   └── stress_plots.py
│   └── results/
│       ├── figures/
│       └── mesh_convergence.csv
│
├── 03_nozzle_cfd/                  # Project 03 — De Laval Nozzle CFD Analysis
│   ├── README.md
│   ├── cad/
│   │   ├── nozzle_study.stp
│   │   ├── Nozzle_study_2d_conical.step
│   │   ├── Nozzle_study_2d_conical_axisymmetric.step
│   │   ├── Nozzle_study_3d_conical.step
│   │   └── Nozzle_3d_conical_PlumeAnalysis.x_t
│   ├── ansys/
│   │   └── Nozzle_2d.wbpj
│   ├── postprocess/
│   │   ├── mesh_convergence.py
│   │   └── validate_isentropic.py
│   └── results/
│       └── figures/
│
├── 04_missile_aero_database/       # Project 04 — Missile Aerodynamic Database
│   ├── README_04_Missile_dynamic_database.md
│   ├── datcom/
│   │   ├── missile.dcm
│   │   └── datcom.out
│   ├── postprocess/
│   │   ├── parse_datcom.py
│   │   ├── aero_interpolator.py
│   │   ├── barrowman.py
│   │   ├── export_csv.py
│   │   └── plot_aero_database.py
│   └── results/
│       ├── aero_database.csv
│       └── figures/
│           └── aero_database.png
│
├── 05_kalman_filter/               # Project 05 — Extended Kalman Filter
│   ├── README.md
│   ├── src/
│   │   ├── ekf.py
│   │   ├── target_model.py
│   │   └── sensor_model.py
│   ├── notebooks/
│   │   └── ekf_demo.ipynb
│   ├── tests/
│   │   └── test_ekf.py
│   └── results/
│       └── figures/
│
├── 06_6dof_missile_sim/            # Project 06 — 6-DOF Flight Mechanics Simulator
│   ├── README_06_6DOF_Sim.md
│   ├── src/
│   │   └── sixdof.py
│   ├── tests/
│   │   └── test_sixdof.py
│   └── results/
│       └── figures/
│           ├── sixdof_trajectory.png
│           └── sixdof_3d_trajectory.png
│
├── 07_thermal_structural_vibration_fatigue/  # Project 07 — Thermal-Structural Analysis
│   ├── README_07_Thermal_Structural.md
│   ├── cad/
│   │   ├── fin_assembly.x_t
│   │   └── nose_cone.x_t
│   ├── src/
│   │   ├── aeroheating.py
│   │   └── export_heatflux.py
│   ├── ansys/
│   │   └── thermal_structural.wbpj
│   └── results/
│       ├── aeroheating.csv
│       ├── fin_LE_heatflux.csv
│       ├── nose_heatflux.csv
│       └── figures/
│
├── 08_gnc_monte_carlo/             # Project 08 — GNC / Monte Carlo Engagement Analysis
│   ├── README_08_GNC_MonteCarlo.md
│   ├── src/
│   │   ├── target.py
│   │   ├── guidance.py
│   │   ├── engagement.py
│   │   └── monte_carlo.py
│   ├── tests/
│   │   └── test_guidance.py
│   └── results/
│       ├── miss_distances.npy
│       ├── hit_flags.npy
│       └── figures/
│           ├── single_engagement.png
│           ├── single_engagement.gif
│           ├── miss_distance_histogram.png
│           ├── pn_comparison.png
│           ├── launch_envelope.png
│           ├── pk_curve.png
│           ├── sensitivity.png
│           └── ecm_comparison.png
│
├── 09_attitude_control/            # Project 09 — Attitude Control Simulator
│   ├── README_09_AttitudeControl.md
│   ├── src/
│   │   ├── attitude_sim.py
│   │   ├── lqr.py
│   │   ├── actuator.py
│   │   ├── quaternion.py
│   │   ├── animate_3d.py
│   │   └── pd_controller_reference.py
│   │   ├── tune_attitude.py
│   └── results/
│       └── figures/
│           ├── attitude_sim.png
│           ├── attitude_sim_ogl.png
│           └── engagement_3d.gif
│
├── 10_autopilot/                   # Project 10 — Three-Loop Acceleration Autopilot
│   ├── README_10_Autopilot.md
│   ├── src/
│   │   ├── autopilot.py
│   │   ├── autopilot_sim.py
│   │   ├── compare_lqr.py
│   │   └── tune_autopilot.py
│   └── results/
│       └── figures/
│           ├── autopilot_sim_—_Three-Loop.png
│           └── engagement_3d.gif
│
├── 11_seeker_model/                # Project 11 — IR/RF Seeker Model
│   ├── README_11_Seeker.md
│   ├── src/
│   │   ├── seeker.py
│   │   └── seeker_sim.py
│   └── results/
│       ├── seeker_miss_distances.pkl
│       └── figures/
│           └── seeker_monte_carlo.png
│
├── 12_lethality/                   # Project 12 — Fragmentation Lethality Model
│   ├── README_12_Lethality.md
│   ├── src/
│   │   ├── lethality.py
│   │   └── lethality_sim.py
│   └── results/
│       └── figures/
│           ├── lethality_sensitivity.png
│           ├── pk_vs_miss.png
│           └── system_pk_overlay.png
│
├── 13_launch_envelope/             # Project 13 — Launch Envelope Calculator
│   ├── README_13_LaunchEnvelope.md
│   ├── src/
│   │   ├── launch_envelope.py
│   │   └── envelope_sim.py
│   └── results/
│       └── figures/
│           ├── envelope_comparison.png
│           ├── envelope_miss_Non-maneuvering.png
│           ├── envelope_pk_3g_maneuver.png
│           └── envelope_pk_Non-maneuvering.png
│
├── cad/                            # Full Missile Assembly CAD
│   ├── README_CAD.md
│   ├── assembly/
│   │   └── Rocket_Assembly.x_t
│   ├── parts/
│   │   ├── Rocket_body_tube.x_t
│   │   ├── Rocket_Cone_Tip(forAssembly).x_t
│   │   ├── Rocket_Fin.x_t
│   │   └── Exhaust_Nozzle_scaled(forAssembly).x_t
│   └── drawings/
│       ├── Rocket_Assembly.pdf
│       ├── Rocket_Nozzle_Scaled.pdf
│       ├── Rocket_Fin.pdf
│       ├── Rocket_Body_tube.pdf
│       └── Nose_Cone(forAssembly).pdf
│   ├── propulsion/
│   │   ├── tactical_motor_boost_sustain.ric
│   │   ├── thrust_curve.png
│   │   └── README_CAD_Propulsion.md
│
├── requirements.txt
└── README.md
```

---

## Projects Overview

### 01 · Airfoil Aerodynamic Study

Parametric aerodynamic analysis of NACA airfoil series using XFLR5/XFOIL across a sweep of Reynolds numbers and angles of attack. Cl/Cd polar optimization for missile canard application.

**Tools:** XFLR5, Python (NumPy, Pandas, Matplotlib)

**Key result:** NACA 0012 selected — CLmax 1.08 at stall AoA 11°, L/D 60.4 at Re=500k M=0.3. 24 total polar sweeps across 4 airfoils, 3 Reynolds numbers, 2 Mach numbers. NACA 0006 unreliable at Re=500k (unresolved laminar separation bubble). M=0.5 cases approximate (Prandtl-Glauert near breakdown). Results feed Project 02 fin geometry and Project 04 aero database.

---

### 02 · Structural FEA — Missile Fin Assembly

Static structural FEA of a Ti-6Al-4V missile fin assembly under 10,000g axial setback load. Five-level mesh convergence study with Richardson extrapolation and GCI analysis.

**Tools:** Creo Parametric 12.4, Ansys Mechanical (Student), Python (NumPy, Matplotlib, Pandas)

**Key result:** Max Von Mises stress 1.334 ± 0.036 MPa at inner root fillet (mean ± 1σ, finest three mesh levels — non-monotonic convergence due to low-quality elements at tab-fin junction). Safety factor 660× vs yield. Deformation Richardson-extrapolated to 1.363 μm (GCI < 0.001%). Fixed support singularity identified and excluded. Fin geometry carried into Projects 04, 07, and CAD assembly.

---

### 03 · De Laval Nozzle CFD Analysis

Compressible flow simulation of a conical De Laval nozzle (Ae/A* = 16.67) across four mesh refinement levels. Mach and pressure distributions validated against isentropic flow relations. Supplemented by hot-fire exhaust plume study at representative rocket chamber conditions (P₀ = 9.8 MPa, T₀ = 3710 K).

**Tools:** Onshape, Ansys Fluent (density-based and pressure-based coupled), Ansys Meshing, Python

**Key result:** Exit Mach Richardson-extrapolated to 2.745 (GCI < 0.5%). Mass flow conservation 0.010% on fine mesh. Normal shock captured in divergent section at off-design cold-flow condition (P0/P = 5.94 vs design 293). Hot-fire plume study confirms fully started nozzle at representative chamber conditions. Nozzle geometry scaled and carried into CAD assembly.

---

### 04 · Missile Aerodynamic Database

Full aerodynamic coefficient database (CL, CD, CM) for a conical-nose / cylindrical-body / trapezoidal-fin missile across Mach 0.8–3.0 and AoA 0–20°. Digital DATCOM primary method, Barrowman subsonic cross-validation. Database packaged as a SciPy interpolation module feeding directly into the Project 06 6-DOF simulator.

**Tools:** Digital DATCOM, Python (NumPy, SciPy)

**Key result:** CL at M=0.8–1.2, CM at M=1.6–3.0. Barrowman CNa=14.19/rad vs DATCOM CLA=5.14/rad — method mismatch documented (DATCOM single equivalent wing vs Barrowman 4-fin interference). Interpolation module operational and integrated into Projects 05, 06, 07.

---

### 05 · Extended Kalman Filter — State Estimator

6-state EKF fusing simulated IMU accelerometer (100 Hz, σ=0.5 m/s²) and GPS position/velocity (10 Hz, σ=5m/0.1m/s) on a boost-sustain missile trajectory. Ground truth integrates EOM with Project 04 DATCOM drag.

**Tools:** Python (NumPy, Matplotlib)

**Key result:** Position RMSE 0.98m, velocity RMSE 0.39 m/s. Residuals zero-mean, bounded ±3m, no secular drift. EKF applied to Project 06 6-DOF trajectory as full pipeline integration.

---

### 06 · 6-DOF Flight Mechanics Simulator

Full 12-state 6-DOF flight mechanics simulator for a boost-sustain tactical missile. Integrates Project 04 DATCOM aerodynamic database and Project 05 EKF navigation solution. RK4 integration, variable-mass propulsion, aerodynamic drag and pitch moment, pitch damping. 3D trajectory with 1° yaw offset demonstrating full 3D capability.

**Tools:** Python (NumPy, Matplotlib)

**Key result:** 43.2s flight, 11.4km range, 2006m apogee, Mach 1.25 peak. EKF position RMSE 1.11m. Mach/altitude history feeds Project 07 aeroheating.

---

### 07 · Thermal-Structural-Vibration-Fatigue Analysis

Coupled aeroheating, transient thermal, static structural, prestressed modal, random vibration, and fatigue analysis of the missile fin assembly and nose cone under flight loads derived from the Project 06 trajectory.

**Tools:** Python (NumPy, Matplotlib), Onshape, Ansys Workbench 2026 R1

**Key result:** Fin LE critical — 249°C peak, 4.46 MPa combined stress, SF 173× static, SF 24× at random vibration 3σ, Mode 1 at 415 Hz, infinite fatigue life. Thermal stress 73% of combined fin stress — thermal management is the dominant design driver. Nose cone benign in all load cases (SF 789×, Mode 1 5570 Hz).

---

### Missile Assembly CAD

Full tactical missile assembly in Onshape. All dimensions consistent with the simulation pipeline — body diameter, fin geometry, nose cone, and nozzle match parameters used in Projects 02–07. Formal engineering drawings with GD&T included.

**Tools:** Onshape, Creo Parametric 12.4

**Key features:** 100mm OD body, 1000mm length, 4× fin dovetail attachment, nozzle counterbore, bulkheads at x=250/420mm, nose cone spigot interface. XCG=500mm, XCP=750mm, static margin 25% body length consistent with Projects 04/06. Engineering drawings for all parts in `cad/drawings/`. GD&T applied to fin assembly and nose cone — tolerances tied directly to Project 02 and 07 FEA boundary conditions and critical stress/thermal locations.

### Propulsion Section — CAD Assembly & Internal Ballistics Validation

Dual-grain propulsion section modeled in Onshape and validated using openMotor v0.6.1 internal ballistics simulation. Finocyl boost grain (180mm, 38mm core) and BATES sustain grain (416mm, 38mm core) share a 25mm throat / 60mm exit nozzle (Ae/A* = 5.76). Propellant: custom 68/18/14 HTPB/AP/Al composite (a=0.045 mm/(s·Paⁿ), n=0.35, Tc=3500K). Thrust curve shows clear two-phase profile — finocyl boost peak followed by BATES neutral-burn sustain plateau, closely matching the Project 06 step-function assumption.

**Key results:** 2.04s burn time, 10,242 Ns impulse, 235.2s delivered Isp, 4.44 kg propellant mass (89% of Project 06 assumption), peak 9.82 MPa, 76.82% volume loading. Burn time delta vs Project 06 (2s vs 15s) documented as physical packaging constraint — maximum web thickness in 84mm grain limits burn duration to ~2s at realistic HTPB/AP/Al regression rates. Nozzle Ae/A* reduced from Project 03's 16.67 to 5.76 due to 100mm body tube OD constraint (Project 04).

**Tools:** Onshape, openMotor v0.6.1

---

### 08 · GNC / Monte Carlo Engagement Analysis

Proportional navigation guidance law implementation, 500-run Monte Carlo engagement simulation, and full performance analysis against a 3g maneuvering drone target. Three PN variants (PPN, TPN, APN) implemented and compared. All 8 validation tests pass.

**Tools:** Python (NumPy, Matplotlib, SciPy)

**Key result:** TPN N=5 achieves CEP50=374m and Pk=0.8% against a fully maneuvering 3g drone at randomized launch geometries spanning 1–8km. TPN outperforms PPN by 8× in CEP50. Performance bounded by decoupled vertical channel and absence of end-game guidance refinement — both documented modeling choices. Sensitivity analysis confirms N=5 optimal. ECM (5× seeker noise) has minimal effect on CEP50. System-level FMEA (20 failure modes, RPN analysis) covers full pipeline from aero database through GNC — structural failures all RPN < 10.

---

### 09 · Attitude Control Simulator

13-state attitude control simulator extending Project 06 with quaternion attitude representation, 4-fin actuator model, CARE-solved LQR attitude controller with dynamic pressure gain scheduling, and TPN/OGL/HYBRID guidance modes. Full inner/outer loop architecture with complete development methodology documentation including failed approaches (hybrid PD+LQR, pure OGL, engagement geometry sensitivity). Includes a PD controller reference implementation for comparison.

**Tools:** Python (NumPy, SciPy, Matplotlib)

**Key result:** 35.6m miss distance against a stationary target at 2500m range,
500m altitude. Peak Mach 1.67, 14.4s flight time, CPA altitude 512m. TPN and
HYBRID (TPN+OGL) guidance modes produce identical results — confirmed the residual
miss is a kinematic floor, not a guidance or control quality problem. LQR tracks
desired attitude to within 1° throughout the engagement. 17 validation tests across
all subsystems pass. Bayesian optimization over 1112 trials confirmed manual result —
best found 35.6m (0.1m improvement, within noise). guidance_clamp accounts for 97%
of miss distance variance; LQR Q/R weights collectively <1%.

**Controller architecture:**
- Quaternion kinematics replacing Euler angles (eliminates gimbal lock)
- CARE-solved LQR: Q = diag([2,2,1,15,15,15]), R = diag([40,15,30])
- Dynamic pressure gain scheduling every 1s
- Gravity compensation below target altitude, guidance clamp ±8° (±12° inside 300m)
- First-order attitude smoother τ=0.2s (disabled inside 300m)
- Terminal phase LOS blend within 500m
- OGL (zero-effort miss / t_go²) available as alternative guidance law
- HYBRID mode: TPN midcourse, OGL terminal at 200m switch radius

---

### 10 · Three-Loop Acceleration Autopilot

Industry-standard three-loop missile autopilot: inner angular rate loop (PD),
middle body acceleration loop (PI), outer TPN guidance loop from Project 08.
Directly comparable to the LQR from Project 09 on the same engagement geometry.
Demonstrates the physically grounded alternative to optimal control used in real
tactical missile programs — accelerometer and rate gyro feedback only, no full
state information.

**Tools:** Python (NumPy, SciPy, Matplotlib), Optuna

**Key result:** 58.6m miss distance against a stationary target at 2500m range,
500m altitude. LQR baseline (Project 09): 35.7m. Miss distance gap is physically
expected — LQR has full state information and globally optimal gains; the three-loop
has only body acceleration and rate feedback.

**Bayesian optimization:** 1707 trials, Optuna TPE, SQLite persistence. Three
parameters account for 100% of variance: Kp_rate (50%), OMEGA_CMD_MAX (37%),
Kd_rate (13%). All scheduling parameters, roll gain, and Ki_a have ~0% importance.
Roll stabilization confirmed as a physical floor — at 120+ deg/s roll rate, fin
authority is insufficient to arrest roll regardless of gain tuning (requires
|Kp_roll| < 0.167 to avoid saturation, providing negligible corrective torque).

**Controller architecture:**
- Inner loop: rate loop (PD) — angular rate feedback → fin deflection
- Middle loop: acceleration loop (PI, leaky integrator) — body accel feedback → rate command
- Outer loop: TPN guidance (unchanged from Project 08)
- Gain scheduling inversely proportional to dynamic pressure
- Roll stabilization fixed at nominal (not scheduled) — Kp_roll = -0.02

**Final gains:**
```
Ka_nom      = 0.8     OMEGA_CMD_MAX = 1.5
Ki_a_nom    = 0.45    INT_LEAKAGE   = 1.0
Kp_rate_nom = -0.28   INT_CLIP      = 1.2
Kd_rate_nom = 0.0     SCALE_MIN     = 0.6
Kp_roll_nom = -0.02   SCALE_MAX     = 2.5
```

---


### 11 · IR/RF Seeker Model

Two-axis gimbaled seeker with PI track loop, physically-grounded noise model (Gaussian LOS rate + range-scaled glint + radome boresight error + ECM spoofing), range estimator with ECM bias injection, persistent track-loop slip, and lock-loss detection with sticky reacquire. Replaces the ideal sensor assumption in Project 09 attitude_sim with a one-line swap. 50-run Monte Carlo quantifies miss distance impact across four sensor error regimes.

**Tools:** Python (NumPy, Matplotlib)

**Key result:** TPN with LQR attitude control exhibits a kinematic miss-distance floor of 35.6m robust to zero-mean seeker noise up to 0.02 rad/s. Seeker degradation manifests as increased miss-distance variance (σ: 0 → 1.1m under noise → 6.8m under ECM) rather than shifted mean miss — a structural property of TPN, where unbiased angular measurements produce symmetric miss distributions around the kinematic floor. Only persistent bias error modes shift the mean: ECM spoofing reaches 2% Pk as favorable bias realizations drive the missile inside 20m lethal radius. 8/8 validation tests pass. Engineering implication: seeker specs should prioritize bias minimization (radome calibration, thermal stability) over noise floor reduction, and counter-ECM has higher priority than counter-jamming for this class of missile.

**arXiv tie-in:** ECM spoofing based on Andersson & Dán, *Active Bayesian Inference for Robust Control under Sensor False Data Injection Attacks* (arXiv:2604.11410).


---

### 12 · Fragmentation Lethality Model

Physics-based fragmentation warhead lethality model closing the end-to-end kill chain from seeker-guided engagement (Project 11) through system-level kill probability. Gurney equation for fragment launch velocity, Mott distribution for fragment mass statistics (B=0.68, mu=0.30g, N=5000), spherical spreading with beam geometry for spatial density, aerodynamic drag decay, and cookie-cutter vulnerability model (80 J KE threshold per Catovic 2022). System Pk computed as E[Pk(r)] over empirical miss distance distributions loaded from Project 11 seeker Monte Carlo. Sensitivity analysis on C/M ratio and target vulnerable area fraction.

**Tools:** Python (NumPy, Matplotlib)

**Key result:** Lethal radius 11.1m (Pk=0.5) for a 4.5kg warhead (3.0kg case, 1.5kg Comp B, C/M=0.500) against an aerial target with 8m^2 presented area and 25% vulnerable fraction. Pk(10m)=0.575, Pk(20m)=0.167, Pk(30m)=0.068. System Pk from Project 11 seeker MC: ideal sensor 0.044, base noise 0.045, noise+glint 0.045, ECM spoofing 0.018. ECM halves system Pk relative to ideal — confirms Project 11 finding that persistent bias is the dominant sensor degradation mode at the system level. 10/10 validation tests pass.

**References:** Gurney (BRL 405, 1943), Mott (Proc. Royal Soc. A, 1947), Catovic & Kljuno (J. Defence Modeling & Simulation, 2022), Heininen (2022), Driels *Weaponeering* (AIAA, 2013).

---

### 13 · Launch Envelope Calculator

Computes No-Escape Zone (NEZ) and Firing Envelope Zone (FEZ) boundaries for a missile against maneuvering and non-maneuvering targets across launch geometries. Simplified 3-DOF point-mass engagement model with TPN guidance sweeps range (500-6000m) x bearing (+/-180 deg) grid, computing Pk at each point via Project 12 lethality model. Polar Pk contour plots with NEZ/FEZ boundary extraction. Comparison between non-maneuvering and 3g maneuvering targets demonstrates envelope shrinkage under evasion.

**Tools:** Python (NumPy, Matplotlib)

**Key result:** 20x36 range-bearing grid, 3 MC seeds per point, two target configs (non-maneuvering, 3g). Maneuvering target shrinks FEZ boundary vs non-maneuvering as expected. 3-DOF miss distances (50-1000m) are larger than full 13-state pipeline (35.7m) due to simplified dynamics — documented as modeling choice for sweep speed. Full-fidelity production version would swap in Project 11 seeker_sim engagement model. 10/10 validation tests pass.

**arXiv tie-in:** NEZ/FEZ boundary validation against Shekhawat & Sinha, *A Study of the Circular Pursuit Dynamics using Bifurcation Theoretic Computational Approach* (arXiv:2604.09065) — circular pursuit capture-to-limit-cycle transition provides analytical boundary conditions for numerically computed Pk contours

---

## Integrated Pipeline

```
[Project 01]              [Project 03]           [Project 02]
Airfoil Study        →    Nozzle CFD        →    Fin FEA
XFLR5 (NACA 0012)         Ansys Fluent            Ansys Mechanical
      │                        │                        │
      │                   Nozzle geometry          Fin geometry
      │                   → CAD Assembly           → CAD Assembly
      ▼                                                 │
[Project 04]                                            ▼
Aero Database  ─────────────────────────────→  [Project 07]
DATCOM                                          Thermal-Structural
      │                                         Vibration-Fatigue
      ▼                                         Ansys Workbench
[Project 05]
EKF Navigator
Python IMU+GPS
      │
      ▼
[Project 06]
6-DOF Flight Sim ──── Mach(t)/alt(t) ──────→  [Project 07]
Python RK4 EOM                                  (aeroheating input)
      │
      ▼
[Project 08]──────────────────────────────────────────────────────┐
GNC / Monte Carlo                                                  │
PN Guidance + CEP                                                  │
      │                                                            │
      ▼                                                            │
[Project 09]              [Project 10]          [Project 11]      │
Attitude Control   →   Three-Loop Autopilot  +  Seeker Model  ────┤
LQR + Quaternion       (compare vs LQR)         Gimbal + Noise    │
TPN / OGL / HYBRID                                                 │
      │                                                            │
      └──────────────────────────────────────────────────────────→│
                                                                   ▼
                                                        [Project 12]
                                                        Lethality Model
                                                        Gurney + Mott + Pk
                                                                   │
                                                                   ▼
                                                        [Project 13]
                                                        Launch Envelope
                                                        NEZ / FEZ / Pk contours
```

---

## Environment Setup

```bash
git clone https://github.com/AnselCederquist/Weapons-systems-sim-pipeline.git
cd Weapons-systems-sim-pipeline

conda env create -f environment.yml
conda activate weapons-sim

# or
pip install -r requirements.txt
```

**requirements.txt:**
```
numpy>=1.24
scipy>=1.10
matplotlib>=3.7
pandas>=2.0
optuna>=3.6.0
jupyter>=1.0
pytest>=7.0
pillow>=9.0
```

**External tools (all free):**

- [Ansys Student](https://www.ansys.com/academic/students) — Fluent, Mechanical, Modal, Random Vibration
- [XFLR5](http://www.xflr5.tech/) — airfoil analysis
- [Digital DATCOM](https://github.com/jbgaya/Digital_Datcom) — missile aero database
- [Onshape](https://www.onshape.com) — CAD (free education account)
- Creo Parametric 12.4 student, SolidWorks student, or Fusion 360 — any STEP/Parasolid export

---

## Disclaimer

All simulations use generic, publicly available geometry and reference data. No export-controlled, classified, or proprietary information is contained in this repository. This project is for educational and portfolio purposes only.

---

## Author

**Ansel Cederquist**  
B.S. Mechanical Engineering, CSU Sacramento (2025) · Magna Cum Laude · GPA 3.87  
CSWA Certified · Ansys Mechanical · Ansys Fluent · Python  
Davis, CA · 530-304-2696 · anselcederquist@outlook.com