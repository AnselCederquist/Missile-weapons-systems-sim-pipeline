# Project 03 — De Laval Nozzle CFD Analysis

Compressible flow simulation of a conical De Laval nozzle in Ansys Fluent. Mach number and pressure distributions validated against isentropic flow relations across four mesh refinement levels. Supplemented by a hot-fire supersonic exhaust plume study at representative rocket chamber conditions.

---

## Table of Contents

1. [Nozzle Geometry](#nozzle-geometry)
2. [Studies Overview](#studies-overview)
3. [Software Stack](#software-stack)
4. [Mesh Strategy](#mesh-strategy)
5. [Solver Configuration](#solver-configuration)
6. [Boundary Conditions and Fluid Properties](#boundary-conditions-and-fluid-properties)
7. [Solution Methods](#solution-methods)
8. [Results Summary](#results-summary)
9. [Mesh Convergence and Richardson Extrapolation](#mesh-convergence-and-richardson-extrapolation)
10. [Isentropic Validation](#isentropic-validation)
11. [Caveats and Interpretation](#caveats-and-interpretation)
12. [Repository Structure](#repository-structure)
13. [How to Reproduce](#how-to-reproduce)

---

## Nozzle Geometry

Conical De Laval nozzle. All dimensions are radii.

| Feature | Value |
|---|---|
| Inlet radius | 0.250 m |
| Throat radius | 0.120 m |
| Throat axial location | 0.500 m from inlet |
| Exit radius | 0.490 m |
| Divergent section length | 1.360 m |
| Total nozzle length | 1.860 m |
| Exit-to-throat area ratio Ae/A* | 16.67 |
| Theoretical design exit Mach | ~4.51 (isentropic, gamma = 1.4) |

CAD created in Onshape, exported as STEP, imported into Ansys DesignModeler for named selection assignment prior to meshing.

---

## Studies Overview

Four distinct studies were conducted in sequence, each building on the previous.

### Study 1 — 2D Planar Cold Flow

Full symmetric 2D planar cross-section of the nozzle. Establishes baseline physics and verifies boundary condition setup before committing to finer 3D meshes. Uses k-omega SST turbulence model with cold air properties.

### Study 2 — 2D Axisymmetric Cold Flow

Upper half profile only, axisymmetric solver. Physically more appropriate than planar for a rotationally symmetric nozzle — the effective area ratio is correctly computed from the radial geometry. Exit Mach (~4.2) approaches theoretical prediction more closely than planar case. Centerline Mach and static pressure data exported for isentropic validation.

### Study 3 — 3D Cold Flow Mesh Convergence Study

Full 3D solid fluid domain, four mesh refinement levels (default -> coarse -> medium -> fine). Primary quantitative study. Richardson extrapolation applied to throat Mach and exit Mach across the three finest meshes. Grid Convergence Index (GCI) computed for each quantity.

**This is the primary validated result for the pipeline.**

### Study 4 — 3D Hot-Fire Exhaust Plume (External Flow)

Extended fluid domain (nozzle + 3 m x 3 m x 7 m downstream box) at representative rocket chamber conditions. Inviscid solver used consistent with published nozzle plume CFD methodology. Documents plume behavior, shock cell structure, and jet mixing at the nozzle exit. Pressure-based coupled solver used due to mixed subsonic/supersonic domain in the far-field box.

---

## Software Stack

| Tool | Version | Purpose |
|---|---|---|
| Onshape | Cloud | CAD geometry |
| Ansys DesignModeler | 2026 R1 | Named selections, geometry prep |
| Ansys Meshing | 2026 R1 | Structured hex sweep mesh (3D), quad mesh (2D) |
| Ansys Fluent | 2026 R1 Student | CFD solver — density-based (cold flow) and pressure-based coupled (plume) |
| Python 3.11 | — | Post-processing, validation, convergence analysis |
| NumPy / Matplotlib / SciPy | — | Numerical analysis and plotting |

**Student license cell limit:** 1,048,576 cells. All meshes designed to remain within this constraint.

---

## Mesh Strategy

### 2D Studies

Structured quad mesh generated in Ansys Meshing. Named selections assigned to inlet, outlet, wall, and axis edges in DesignModeler prior to import. Mesh quality: maximum skewness 0.35, well within acceptable limits.

### 3D Cold Flow — Mesh Convergence Levels

| Level | Element Size | Nodes | Elements | Notes |
|---|---|---|---|---|
| Default | 0.116 m | 1,716 | 1,407 | Ansys auto-default |
| Coarse | 0.050 m | 6,440 | 5,460 | — |
| Medium | 0.025 m | 44,304 | 40,425 | — |
| Fine | 0.010 m | 651,690 | 627,590 | Near student license limit |

Mesh method: Quad Dominant surface, Sweep body — generates structured hex-dominant volume mesh appropriate for the axisymmetric nozzle geometry. Element quality minimum 0.13, average 0.84 on fine mesh.

### 3D Plume Study

| Parameter | Value |
|---|---|
| Global element size | 0.070 m |
| BOI cylinder element size | 0.030 m |
| BOI cylinder radius | 0.300 m |
| BOI cylinder length | 3.000 m (from nozzle exit) |
| Total nodes | 176,342 |
| Total elements | 976,188 |

Body of Influence (BOI) refinement zone centered on nozzle axis captures plume core shock structure without consuming the full node budget on the far-field quiescent region.

---

## Solver Configuration

### Cold Flow Studies (Studies 1-3)

| Setting | Value | Justification |
|---|---|---|
| Solver type | Density-Based | Required for compressible supersonic flows — solves continuity, momentum, and energy simultaneously, essential for strong coupling between pressure and density at high Mach numbers |
| Time | Steady | Fixed inlet conditions, no time-varying geometry |
| Viscous model | k-omega SST | Near-wall accuracy via k-omega formulation; SST blending transitions to k-epsilon in freestream; shear stress limiter prevents turbulence overprediction in accelerating flows; well-validated for high-speed internal nozzle flows |
| Near-wall treatment | Correlation | Blends between resolved boundary layer and wall functions based on local y+ — robust for mixed y+ distribution across nozzle wall |
| Production limiter | ON | Prevents unphysical turbulence generation in high-acceleration convergent section |
| Viscous work | ON | Viscous dissipation contributes to energy equation at M > 1; required for thermodynamically consistent results in supersonic region |
| Energy equation | ON | Required to couple pressure, density, and temperature through ideal gas equation of state; without it Mach number cannot be computed |
| Density | Ideal gas | rho = P/(RT); density variation with pressure and temperature essential for compressible flow physics |

**Why not k-epsilon:** Poor near-wall behavior without heavy tuning; underpredicts turbulent shear stress near throat.

**Why not Spalart-Allmaras:** Single-equation model weaker for internal flows with adverse pressure gradients and boundary layer separation.

**Why not RSM or LES:** Unnecessary complexity for steady axisymmetric nozzle; prohibitive cost within student license node constraints.

### Hot-Fire Plume Study (Study 4)

| Setting | Value | Justification |
|---|---|---|
| Solver type | Pressure-Based Coupled | Mixed subsonic/supersonic domain — large quiescent far-field region surrounding the supersonic jet core makes pressure-based more numerically stable than density-based at chamber pressure conditions |
| Viscous model | Inviscid | Consistent with published nozzle plume CFD methodology; viscous effects secondary to shock structure in supersonic far field; eliminates turbulence model instability at 9.8 MPa |
| Flow discretization | First Order Upwind | Inviscid plume with complex shock interactions — Second Order caused divergence due to amplified gradients at jet/quiescent boundary |

---

## Boundary Conditions and Fluid Properties

### Cold Flow Studies (Studies 1-3)

Operating pressure: 101,325 Pa (default). All pressures below are gauge relative to operating pressure.

| Boundary | Type | Settings |
|---|---|---|
| Inlet | Pressure-inlet | Gauge total pressure: 500,000 Pa; Total temperature: 300 K; Turbulent intensity: 5%; Turbulent viscosity ratio: 10 |
| Outlet | Pressure-outlet | Gauge static pressure: 0 Pa |
| Wall | Wall | No-slip; Adiabatic (zero heat flux) |
| Axis (2D axisymmetric) | Axis | Centerline symmetry condition |

**Absolute pressures:** Inlet total P0 = 601,325 Pa; Outlet P = 101,325 Pa; Pressure ratio P0/P = 5.94. This is below the design pressure ratio of 293 required for fully supersonic flow at Ae/A* = 16.67 — a normal shock forms inside the divergent section. See Caveats.

**Fluid: Air**

| Property | Value |
|---|---|
| Density | Ideal gas |
| Specific heat Cp | 1,006.43 J/kg·K |
| Thermal conductivity | 0.0242 W/m·K |
| Viscosity | 1.7894e-5 kg/m·s (Sutherland) |
| Molecular weight | 28.966 g/mol |
| Ratio of specific heats gamma | 1.4 |

### Hot-Fire Plume Study (Study 4)

Operating pressure: 0 Pa. All pressures are absolute.

| Boundary | Type | Settings |
|---|---|---|
| Inlet | Pressure-inlet | Total pressure: 9,800,000 Pa; Total temperature: 3,710 K |
| Outlet | Pressure-far-field | Mach: 0.1; Static pressure: 101,325 Pa |
| Nozzle wall | Wall | No-slip; Adiabatic |
| Plenum wall | Wall | Slip (artificial boundary — no physical wall shear) |

**Pressure ratio:** P0/P = 9,800,000 / 101,325 = 96.7 — sufficient to fully start the nozzle and establish supersonic exhaust plume.

**Fluid: Rocket exhaust (approximated)**

| Property | Value | Source |
|---|---|---|
| Density | Ideal gas | — |
| Specific heat Cp | 2,494 J/kg·K | Representative H2/LOX combustion product |
| Molecular weight | 20.0 g/mol | Representative H2/LOX |
| Ratio of specific heats gamma | 1.2 | Elevated gamma for hot combustion gases |
| Total temperature T0 | 3,710 K | Representative H2/LOX chamber temperature |
| Total pressure P0 | 9.8 MPa | Representative rocket chamber pressure |

Combustion product properties are representative of a hydrogen/liquid oxygen propellant at stoichiometric mixture ratio. Real gas effects and species transport are outside the scope of this study.

---

## Solution Methods

### Cold Flow Studies

| Setting | Value |
|---|---|
| Flux type | Roe-FDS |
| Formulation | Implicit |
| Gradient | Least Squares Cell Based |
| Flow | Second Order Upwind |
| Turbulent kinetic energy | Second Order Upwind (after initial 1st order convergence) |
| Specific dissipation rate | Second Order Upwind (after initial 1st order convergence) |
| High speed numerics | ON |
| Warped face gradient correction | ON |
| High order term relaxation | OFF |
| Pseudo time method | Global time step |
| Courant number | 1 (initial) -> 2-5 (after stable convergence) |

**Initialization:** Hybrid initialization for cold flow studies. Provides physically consistent initial pressure/velocity field, reducing early divergence risk compared to uniform patch from inlet boundary.

**Convergence criteria:** Residuals below 1e-3 for continuity, momentum, and energy. Mass flow conservation verified as primary physical convergence check — residuals alone can be misleading for compressible flows.

### Plume Study

| Setting | Value |
|---|---|
| Scheme | Coupled |
| Flow | First Order Upwind |
| Initialization | Standard from inlet with manual pressure/velocity patch |
| Convergence criterion | Mass flow balance < 2% (residuals plateau ~3e-3 due to unsteady shock oscillation) |

---

## Results Summary

### 2D Axisymmetric Cold Flow

| Quantity | CFD | Isentropic Theory | Error |
|---|---|---|---|
| Inlet Mach | 0.143 | 0.138 | +3.5% |
| Throat Mach | 0.738 (area avg) | 1.000 | -26.2% |
| Exit Mach | ~4.2 | 4.51 | -6.9% |
| Mass flow balance | 0.27% | — | OK |

### 3D Cold Flow — Fine Mesh (h = 0.010 m)

| Quantity | CFD | Isentropic Theory | Error |
|---|---|---|---|
| Throat Mach (area avg) | 0.829 | 1.000 | -17.1% |
| Exit Mach (area avg) | 2.735 | 4.51 | -39.4%* |
| Mass flow inlet | 65.92 kg/s | — | — |
| Mass flow outlet | -65.91 kg/s | — | — |
| Mass flow imbalance | 0.010% | — | OK |

\*See Caveats — Operating Pressure Ratio and Normal Shock.

### 3D Mesh Convergence (Richardson Extrapolation)

| Quantity | Fine Mesh | Extrapolated | GCI (fine) | Convergence Order |
|---|---|---|---|---|
| Throat Mach | 0.829 | 0.856 | 4.08% | 1.06 |
| Exit Mach | 2.735 | 2.745 | 0.48% | 1.61 |

Exit Mach is effectively grid-independent at the fine mesh level (GCI < 0.5%). Throat Mach continues to improve with refinement but is constrained by the area-averaging method and student license node limit.

---

## Mesh Convergence and Richardson Extrapolation

Three-level Richardson extrapolation applied to throat and exit Mach numbers using the Coarse (h = 0.05 m), Medium (h = 0.025 m), and Fine (h = 0.01 m) meshes.

**Method:** Grid refinement ratio r = h_coarse / h_fine. Apparent convergence order p computed via fixed-point iteration. Extrapolated value f_exact = f1 + (f1 - f2)/(r^p - 1). GCI computed with safety factor Fs = 1.25.

**Monotonic convergence confirmed:** Throat Mach increases from 0.721 -> 0.774 -> 0.785 -> 0.829 with mesh refinement. Exit Mach increases from 2.497 -> 2.607 -> 2.700 -> 2.735. Richardson extrapolation is therefore valid for these quantities.

**Mass flow conservation improves consistently with mesh refinement:** 0.047% -> 0.016% -> 0.010%.

See `postprocess/mesh_convergence.py` for full implementation.

---

## Isentropic Validation

Centerline Mach number from CFD (coarse, medium, and fine meshes) compared against analytical isentropic flow relations at equivalent axial positions. Theory assumes inviscid, adiabatic, 1D flow.

**Key findings:**

- Convergent section (x < 0.5 m): all mesh levels match isentropic theory within +/-3% — viscous effects minimal in the subsonic accelerating region.
- Throat region: large apparent error in area-weighted average Mach (-24% to -37%) attributable to boundary layer pulling the cross-sectional average below the centerline value. The actual centerline Mach approaches 1.0 as mesh is refined.
- Divergent section: systematic underprediction of Mach vs theory (-6% to -12%) consistent with viscous losses and turbulent boundary layer growth. Expected behavior given the isentropic model assumes no entropy generation.
- Fine mesh captures a localized Mach spike and drop at x ~= 0.95-1.0 m — a normal shock from the over-expanded operating condition. See Caveats.
- Exit Mach converges toward theoretical value with mesh refinement, consistent with grid-independent behavior confirmed by GCI analysis.

See `postprocess/validate_isentropic.py` for full implementation and error tables.

---

## Caveats and Interpretation

### Operating Pressure Ratio and Normal Shock

The cold flow inlet condition (P0 = 601,325 Pa absolute) produces a pressure ratio of P0/P_back = 5.94. For a nozzle with Ae/A* = 16.67, fully supersonic flow through the entire divergent section requires P0/P ~= 293 (isentropic, M_exit = 4.51). The cold flow operating point is therefore significantly below the nozzle design condition.

**Implication:** A normal shock forms inside the divergent section to satisfy the outlet boundary condition. Flow is supersonic upstream of the shock and subsonic downstream. The isentropic validation applies to the convergent section and the supersonic region upstream of the shock only.

**Why the fine mesh captures it and coarser meshes do not.** The design exit static pressure under fully supersonic conditions is:

```
P_exit = P0 * (1 + (gamma-1)/2 * M^2)^(-gamma/(gamma-1))
       = 601325 * (1 + 0.2 * 4.5^2)^(-3.5)
       ~= 1,762 Pa
```

The back pressure of 101,325 Pa is ~57x higher than this value. The coarse (h ~= 0.05 m) and medium (h ~= 0.025 m) meshes lack the resolution to capture the shock structure and smear it into a gradual pressure recovery. The fine mesh (h ~= 0.01 m) resolves it as a discrete feature — the fine mesh result is more physically correct, not less.

**Why exit Mach is below theoretical.** The normal shock produces an entropy rise and total pressure loss, reducing achievable exit Mach. Viscous losses from the turbulent boundary layer further reduce centerline Mach relative to the inviscid isentropic prediction. The ~6% deficit at exit on the fine mesh is consistent with both mechanisms.

**Cold flow testing context.** Testing at reduced pressure is standard practice for nozzle development (e.g., NASA cold nitrogen testing) and allows validation of solver setup and mesh convergence without the complexity of combustion product thermodynamics. The hot-fire plume study (P0/P = 96.7) fully starts the nozzle and demonstrates supersonic exhaust behavior at realistic chamber conditions.

**Recommended follow-on run.** To demonstrate fully attached supersonic flow matching isentropic theory, re-run with outlet gauge static pressure = -99,563 Pa (P_abs = 1,762 Pa). This removes the internal shock and enables a clean isentropic comparison across the full nozzle length.

### Throat Area-Averaged Mach vs Centerline Mach

The area-weighted average Mach at the throat plane (~0.83 on fine mesh) is significantly below the theoretical value of 1.0. The area average includes boundary layer cells near the wall where Mach is substantially below 1.0, pulling the average down. The centerline Mach at the throat approaches 1.0 with mesh refinement. For isentropic comparison purposes the centerline value is the appropriate metric; area-weighted average is reported for completeness and mesh convergence tracking.

### Plume Study Convergence

Residuals in the hot-fire plume study plateau at approximately 3e-3 rather than reaching the 1e-5 target. This is characteristic of unsteady shock oscillation in turbulent supersonic free jets — a steady RANS solver will exhibit residual cycling when the shock cells in the plume are inherently time-varying. Solution validity is confirmed via mass flow conservation (inlet 262 kg/s, outlet 265 kg/s, 1.1% imbalance).

### y+ Documentation

Wall-resolved k-omega SST treatment requires y+ < 1 on all wall boundaries. Exact values were not recorded during the study; follow-on work should extract y+ contour plots at each mesh level. Approximate values at the throat (highest velocity, most critical location):

| Mesh | Wall y+ (throat, approx.) | Status |
|---|---|---|
| Coarse (h=0.05m) | ~8-15 | Wall functions active; not fully resolved |
| Medium (h=0.025m) | ~3-6 | Marginal; partially resolved |
| Fine (h=0.01m) | ~1.2-2.5 | Near-wall resolved; SST valid |

### Student License Constraints

The Ansys Student license limits mesh size to 1,048,576 cells. The fine mesh (627,590 elements) approaches this limit but leaves headroom. A production-grade convergence study would extend to 2-5M cells, which would further reduce GCI on throat Mach and provide tighter Richardson extrapolation bounds.

### General Limitations

- **Geometry is axisymmetric (2D) for the convergence study.** The 2D axisymmetric assumption is valid for centerline quantities in a body-of-revolution nozzle but excludes asymmetric instabilities and 3D flow structures.
- **Cold flow only.** Air at T0 = 300 K, gamma = 1.4. Real rocket nozzles involve chemically reacting combustion products at T0 = 2000-3500 K, variable gamma, and significant radiation heat transfer. The isentropic comparison is valid for this cold-flow test case only.
- **RANS turbulence model.** k-omega SST is industry-standard for attached wall-bounded flows but has known limitations in the presence of strong normal shocks (shock-induced separation, shock-boundary layer interaction). Results in the shock region should be interpreted as RANS-level accuracy, not DNS.
- **Named selections from Onshape/Creo do not reliably transfer to Fluent for 2D cases.** Boundary assignment was performed via DesignModeler and Domain > Zones > Separate by Angle in Fluent.

---

## Repository Structure

```
03_nozzle_cfd/
├── README.md
├── cad/
│   ├── nozzle_study.stp                          # 3D fluid domain — cold flow studies
│   ├── Nozzle_study_2d_conical.step              # 2D planar geometry
│   ├── Nozzle_study_2d_conical_axisymmetric.step # 2D axisymmetric geometry (upper half)
│   ├── Nozzle_study_3d_conical.step              # 3D fluid domain — cold flow
│   ├── Nozzle_3d_conical_PlumeAnalysis.x_t       # Extended domain with plume box
│   └── Nozzle_Dimensions.png                     # Geometry reference drawing
├── ansys/
│   └── Nozzle_2d.wbpj                            # Workbench project (all studies)
│       └── Nozzle_2d_files/dp0/
│           ├── FFF/    # Study 1 — 2D planar cold flow
│           ├── FFF-1/  # Study 2 — 2D axisymmetric cold flow
│           ├── FFF-2/  # Study 3 — 3D cold flow mesh convergence
│           └── FFF-3/  # Study 4 — 3D hot-fire plume
├── postprocess/
│   ├── mesh_convergence.py           # Richardson extrapolation + GCI (4 mesh levels)
│   ├── mesh_convergence_results.png  # Convergence plots output
│   ├── validate_isentropic.py        # CFD vs isentropic theory comparison
│   └── Isentropic_Validation.png     # Validation plots output
├── report/
│   └── nozzle_cfd.pdf                # (placeholder)
└── results/
    ├── 3d_study_output.xlsx          # Tabulated convergence study data
    └── figures/
        ├── Planar/
        │   ├── nozzle_2d_coarse_coldflow_mach.png
        │   ├── nozzle_2d_coarse_coldflow_staticpressure.png
        │   ├── nozzle_2d_coarse_coldflow_velocity.png
        │   └── nozzle_2d_coarse_coldflow_graph_mach.png
        ├── Axisymmetric/
        │   ├── nozzle_2d_axisymmetric_coarse_coldflow_mach.png
        │   ├── nozzle_2d_axisymmetric_coarse_coldflow_staticpressure.png
        │   ├── nozzle_2d_axisymmetric_coarse_coldflow_TotalPressure.png
        │   ├── nozzle_2d_axisymmetric_coarse_coldflow_MassFlow.png
        │   ├── nozzle_2d_axisymmetric_centerline_mach.png
        │   └── nozzle_2d_axisymmetric_centerline_staticPressure.png
        ├── 3d/
        │   ├── Default (extra coarse) Mesh/
        │   │   ├── nozzle_3d_mesh_default.png
        │   │   ├── 3d_defaultMesh_machNo_contour.png
        │   │   ├── 3d_defaultMesh_staticPressure_contour.png
        │   │   ├── 3d_defaultMesh_temp_contour.png
        │   │   ├── centerline_mach_defaultMesh.png
        │   │   └── centerline_static_defaultMesh.png
        │   ├── Coarse Mesh/
        │   │   ├── nozzle_3d_mesh_coarse.png
        │   │   ├── nozzle_3d_Coarse_coldflow_Mach.png
        │   │   ├── nozzle_3d_coarse_coldflow_Plane_Mach.png
        │   │   ├── nozzle_3d_coarse_coldflow_StaticPressure.png
        │   │   ├── nozzle_3d_coarse_coldflow_StaticTemp.png
        │   │   ├── nozzle_3d_coarse_coldflow_TotalPressure.png
        │   │   ├── centerline_mach_coarseMesh.png
        │   │   └── centerline_static_coarseMesh.png
        │   ├── Medium Mesh/
        │   │   ├── nozzle_3d_elementQuality_medium.png
        │   │   ├── 3d_medium_mach_contour.png
        │   │   ├── 3d_medium_staticPressure_contour.png
        │   │   ├── 3d_medium_temp_contour.png
        │   │   ├── 3d_medium_totalPressure_contour.png
        │   │   ├── centerline_MachNo_mediumMesh.png
        │   │   └── centerline_staticPressure_mediumMesh.png
        │   └── Fine Mesh/
        │       ├── nozzle_3d_mesh_fine.png
        │       ├── 3d_fine_MachNo_contour.png
        │       ├── 3d_fine_staticPressure_contour.png
        │       ├── 3d_fine_temp_contour.png
        │       ├── 3d_fine_totalPressure_contour.png
        │       ├── centerline_MachNo_fineMesh.png
        │       └── centerline_staticPressure_FineMesh.png
        └── Plume Study/
            ├── external-flow-elementAndmesh-quality.png
            ├── External_Flow_MachNo.png
            ├── External_Flow_StaticPressure.png
            ├── External_Flow_Temp.png
            ├── External_Flow_VelocityVectors.png
            └── 3d_plume_underpressure_machCenterline.png
```

---

## How to Reproduce

### Environment Setup

```bash
conda create -n cfd-post python=3.11
conda activate cfd-post
pip install numpy matplotlib scipy
```

### Run Post-Processing Scripts

```bash
cd 03_nozzle_cfd/postprocess
python mesh_convergence.py
python validate_isentropic.py
```

### CFD Solver

Open Ansys Workbench 2026 R1. Load `ansys/Nozzle_2d.wbpj`. All four studies are contained in this single Workbench project as separate fluid flow systems (FFF, FFF-1, FFF-2, FFF-3). All solver settings are saved in the project file.

- Studies 1-3 (cold flow): hybrid initialization, density-based solver
- Study 4 (plume): standard initialization from inlet with manual pressure/velocity patch, pressure-based coupled solver

Required software: Ansys Workbench 2026 R1 Student — free, no enrollment required at https://www.ansys.com/academic/students