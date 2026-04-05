# Project 01 — Airfoil Aerodynamic Study

Parametric aerodynamic analysis of four NACA symmetric airfoil profiles using
XFLR5/XFOIL. Cl/Cd polars generated across a sweep of Reynolds numbers and
angles of attack for missile canard application. Results feed airfoil selection
for Project 04 missile body geometry.

## Overview

Designing a missile canard requires knowing how an airfoil behaves across the relevant flight envelope before committing to a geometry. Get it wrong and the fin stalls too early, 
generates too much drag, or lacks the structural depth to survive launch loads. This study answers the question systematically: run 24 parametric polar sweeps across four NACA symmetric 
profiles at three Reynolds numbers and two Mach numbers, extract the key aerodynamic metrics, and select the best candidate on objective criteria.

The NACA 4-digit symmetric series (0006, 0008, 0012, 0015) is the standard starting point for missile fin design. Symmetric profiles produce zero pitching moment at zero AoA, which 
simplifies stability analysis. The thickness parameter (6%, 8%, 12%, 15%) is the primary design variable -- thinner sections have lower wave drag at supersonic speeds but reduced 
structural depth and lower CLmax at subsonic conditions. The tradeoff is not obvious from first principles, which is why a parametric sweep is the right approach.

XFLR5/XFOIL uses a panel method with viscous boundary layer coupling and the e^n transition model (Ncrit=9, free transition) to compute lift, drag, and moment polars. 
Prandtl-Glauert compressibility correction is applied for the M=0.5 cases. The tool is valid in the subsonic regime and appropriate for this application -- canard aerodynamics 
at launch and low-speed flight phases where M < 0.5. Transonic and supersonic aerodynamics for the full flight envelope are handled in Project 04 via Digital DATCOM.

Key finding: NACA 0012 selected. CLmax = 1.08 at Re=500k M=0.3, stall at 11 deg, L/D = 60.4. NACA 0015 has marginally higher CLmax but the added thickness increases drag without 
meaningful L/D gain at the relevant Reynolds number. NACA 0006 shows unreliable results at Re=500k due to an unresolved laminar separation bubble. NACA 0012 is the optimal tradeoff 
between lift capability, aerodynamic efficiency, and structural depth for a thin fin application.

This result feeds directly into Project 02 (fin structural FEA geometry) and Project 04 (missile aero database fin section definition).

---

## Table of Contents

1. [Study Parameters](#study-parameters)
2. [Software Stack](#software-stack)
3. [Results Summary](#results-summary)
4. [Limitations and Validity Bounds](#limitations-and-validity-bounds)
5. [Repository Structure](#repository-structure)
6. [How to Reproduce](#how-to-reproduce)

---

## Study Parameters

| Parameter | Values |
|---|---|
| Airfoils | NACA 0006, NACA 0008, NACA 0012, NACA 0015 |
| Reynolds numbers | 500,000 / 1,000,000 / 2,000,000 |
| Mach numbers | 0.3 / 0.5 |
| AoA sweep | -2° to 20° (1° increments) |
| Total polar sweeps | 24 |
| Transition model | Free transition (XFOIL e^n method, Ncrit=9) |
| Compressibility correction | Prandtl-Glauert |

---

## Software Stack

| Tool | Version | Purpose |
|---|---|---|
| XFLR5 | 6.x | Airfoil analysis, polar generation |
| Python 3.11 | — | Post-processing, polar comparison plots |
| NumPy / Pandas / Matplotlib | — | Data handling and visualization |

---

## Results Summary

**Selected airfoil: NACA 0012** — optimal tradeoff between aerodynamic
efficiency and structural depth for missile canard application.

| Polar | CLmax | Stall AoA | Max L/D | CD@CL=0.5 |
|---|---|---|---|---|
| NACA 0006 Re=0.500M M=0.30 | 0.695 | 7.0 | 42.9 | 0.0138 |
| NACA 0006 Re=0.500M M=0.50 | 0.643 | 5.5 | 41.6 | 0.0145 |
| NACA 0006 Re=1.000M M=0.30 | 0.618 | 5.5 | 51.7 | 0.0104 |
| NACA 0006 Re=1.000M M=0.50 | 0.650 | 5.5 | 49.6 | 0.0115 |
| NACA 0006 Re=2.000M M=0.30 | 0.728 | 10.0 | 63.2 | 0.0075 |
| NACA 0006 Re=2.000M M=0.50 | 0.626 | 5.0 | 63.5 | 0.0074 |
| NACA 0008 Re=0.500M M=0.30 | 0.822 | 8.0 | 48.8 | 0.0101 |
| NACA 0008 Re=0.500M M=0.50 | 0.751 | 6.5 | 47.9 | 0.0103 |
| NACA 0008 Re=1.000M M=0.30 | 0.908 | 20.0 | 61.8 | 0.0084 |
| NACA 0008 Re=1.000M M=0.50 | 0.781 | 6.5 | 59.9 | 0.0085 |
| NACA 0008 Re=2.000M M=0.30 | 0.947 | 8.5 | 76.7 | 0.0071 |
| NACA 0008 Re=2.000M M=0.50 | 0.816 | 6.5 | 73.5 | 0.0076 |
| NACA 0012 Re=0.500M M=0.30 | 1.077 | 11.0 | 60.4 | 0.0094 |
| NACA 0012 Re=0.500M M=0.50 | 0.936 | 8.5 | 56.9 | 0.0105 |
| NACA 0012 Re=1.000M M=0.30 | 1.219 | 12.5 | 73.5 | 0.0083 |
| NACA 0012 Re=1.000M M=0.50 | 1.020 | 8.5 | 68.1 | 0.0084 |
| NACA 0012 Re=2.000M M=0.30 | 1.355 | 13.0 | 88.4 | 0.0072 |
| NACA 0012 Re=2.000M M=0.50 | 1.104 | 9.0 | 82.3 | 0.0074 |
| NACA 0015 Re=0.500M M=0.30 | 1.155 | 13.0 | 63.1 | 0.0100 |
| NACA 0015 Re=0.500M M=0.50 | 0.997 | 10.0 | 55.2 | 0.0104 |
| NACA 0015 Re=1.000M M=0.30 | 1.290 | 14.0 | 75.0 | 0.0082 |
| NACA 0015 Re=1.000M M=0.50 | 1.106 | 10.0 | 67.7 | 0.0086 |
| NACA 0015 Re=2.000M M=0.30 | 1.449 | 15.5 | 88.9 | 0.0073 |
| NACA 0015 Re=2.000M M=0.50 | 1.224 | 11.0 | 82.9 | 0.0077 |

**Selection rationale.** NACA 0015 has marginally higher CLmax but the added
thickness over NACA 0012 increases CD at cruise AoA without meaningful L/D
gain at Re=500k. NACA 0012 provides CLmax 1.08, stall at 11°, L/D 60.4 at
Re=500k M=0.3 — best tradeoff between lift capability, drag, and structural
depth for a thin fin application. NACA 0006 data at Re=500k is unreliable due
to unresolved laminar separation bubble; results excluded from selection
consideration at that Reynolds number.

---

## Limitations and Validity Bounds

**Mach number validity.** XFLR5 applies a Prandtl-Glauert (P-G)
compressibility correction to panel method results. P-G is a linearized
correction valid only in the subsonic regime — it begins to break down above
approximately M=0.3-0.4 and fails entirely at transonic and supersonic
conditions. The M=0.5 sweep cases in this study are at the margin of P-G
validity and should be treated as approximate. Results at M=0.3 are more
reliable. No transonic or supersonic aerodynamics can be extracted from XFLR5.

**2D vs 3D effects.** All polars are 2D (infinite-span) results. Real fin
aerodynamics include finite-span corrections, tip vortex losses, and body-fin
interference that reduce effective L/D substantially relative to 2D
predictions. The L/D = 60.4 reported for NACA 0012 at Re=500k, M=0.3 is a
2D upper bound, not a fin-level estimate.

**Scope of this study.** The XFLR5 analysis is valid for the low-speed/launch
phase of the flight envelope only (M < 0.4). Transonic and supersonic
aerodynamics — which govern the majority of the flight profile for this
vehicle — are handled in Project 04 via Digital DATCOM and Barrowman
cross-validation, where compressible and shock-dominated flow effects are
properly accounted for.

**Validity by Mach number:**

| Mach | Tool validity | Notes |
|---|---|---|
| 0.3 | Valid (low-speed subsonic) | P-G correction within acceptable range |
| 0.5 | Marginal | P-G linearization near breakdown; results approximate |
| >0.7 | Not valid | Transonic regime requires different methods (DATCOM, CFD) |
| >1.0 | Not valid | Supersonic: shock-expansion theory or CFD required |

---

## Repository Structure

01_airfoil_study/
├── README.md
├── xflr5_project/              # XFLR5 project files
├── postprocess/
│   └── plot_polars.py          # Cl/Cd polar plots, parametric comparison
├── results/
│   ├── figures/                # Polar plots, pressure distributions
│   └── data/                   # Raw XFLR5 output files
└── report/
└── airfoil_study.pdf

---

## How to Reproduce

### XFLR5

Open XFLR5. Load project file from `xflr5_project/`. All 24 polars are saved
in the project. Re-run via Analysis → Batch Analysis if needed.

### Post-Processing
```bash
cd 01_airfoil_study/postprocess
python plot_polars.py
```

Output figures saved to `results/figures/`.

**Dependencies:** numpy, pandas, matplotlib (included in root requirements.txt)
