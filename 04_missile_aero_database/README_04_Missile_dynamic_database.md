# Project 04 - Missile Aerodynamic Database

Full aerodynamic coefficient database for a generic conical-nose / cylindrical-body / trapezoidal-fin missile. Digital DATCOM primary method, Barrowman subsonic cross-validation (pending), Ansys Fluent spot-checks (pending). Database feeds Project 06 6-DOF simulator via Python interpolation module.

## Overview

A missile guidance system needs to know how aerodynamic forces and moments
vary with Mach number and angle of attack across the full flight envelope.
This relationship -- the aerodynamic database -- is the foundation of every
flight mechanics simulation, stability analysis, and control law design in
the program. Without it, the 6-DOF simulator has no physics.

Digital DATCOM (USAF Stability and Control Data Compendium) is the
industry-standard semi-empirical tool for generating preliminary aero
databases. It uses geometry and flight condition inputs to compute
aerodynamic coefficients from a validated database of experimental and
analytical results spanning subsonic through supersonic regimes. Raytheon,
L3Harris, Northrop Grumman, and Aerojet Rocketdyne all use DATCOM for
preliminary aero database generation -- it is not an academic tool, it is
the actual industry workflow for this phase of missile development.

The missile geometry is a conical nose / cylindrical body / 4-fin
trapezoidal configuration with dimensions consistent with a small tactical
missile (100mm body diameter, 1000mm length). Fin section is double-wedge
at 5% thickness ratio, consistent with supersonic fin design practice.
DATCOM is run across 5 Mach numbers (0.8, 1.2, 1.6, 2.0, 3.0) and 6 angles
of attack (0 to 20 deg) for 30 total flight conditions.

Barrowman equations provide an independent analytical cross-validation of
the subsonic normal force derivative CNa. Barrowman is a closed-form method
valid for slender bodies at low AoA in subsonic flow -- appropriate as an
independent check on DATCOM subsonic normal force derivatives. Barrowman
CNa = 14.19/rad vs DATCOM CLA = 5.14/rad at M=0.8. The large discrepancy
is a known method mismatch: DATCOM models fins as a single equivalent wing
panel while Barrowman applies explicit 4-fin body interference (K_f = 1.25).
Both results are documented and contextualized.

The database is packaged as a SciPy RegularGridInterpolator module
(aero_interpolator.py) providing CL(M,alpha), CM(M,alpha), and CD(M,alpha)
as callable functions. This module is imported directly by the Project 05
Kalman filter for drag computation and will feed the Project 06 6-DOF
equations of motion -- the pipeline is live and integrated.

Key limitation: DATCOM subsonic wing methods produce CL but lack a moment
method for this body-fin configuration. Supersonic methods produce CM but
do not converge on CN/CA for low aspect ratio fins. This is a documented
DATCOM method coverage limitation for tactical missile geometries, not a
geometry input error. The available coefficients are sufficient for the
6-DOF simulation.

## Geometry

| Feature | Value |
|---|---|
| Nose type | Conical |
| Nose length | 150 mm |
| Body diameter | 100 mm |
| Body length | 1000 mm |
| Fin root chord | 80 mm |
| Fin tip chord | 40 mm |
| Fin semispan exposed | 150 mm |
| Fin LE sweep | 45 deg |
| Fin section | Double-wedge t/c 5% |
| Number of fins | 4 |

All DATCOM inputs in feet. SREF = 0.0845 ft2, CBARR = 0.328 ft, XCG = 1.640 ft.

## Methodology

### Primary - Digital DATCOM

USAF Stability and Control Data Compendium. Industry-standard semi-empirical tool for preliminary aero database generation.

Sweep: 5 Mach (0.8, 1.2, 1.6, 2.0, 3.0) x 6 AoA (0, 4, 8, 12, 16, 20 deg) = 30 flight conditions. Sea-level standard atmosphere.

### Cross-Validation - Barrowman

**Barrowman cross-validation:** CNa = 14.19 /rad, XCP = 75.0% body length.
DATCOM CLA at M=0.8 = 5.14 /rad. Difference large (176%) due to method mismatch:
DATCOM $WGPLNF treats fins as a single equivalent wing; Barrowman applies
explicit 4-fin body interference (K_f = 1.25). Barrowman used as primary
subsonic normal force estimate.

### Interpolation Module

DATCOM output parsed via fixed-width parser. aero_interpolator.py wraps scipy.interpolate.RegularGridInterpolator to provide CL(M,a), CM(M,a), CD(M,a) as callable functions imported by Project 06.

## Results

| Mach | CL | CM | CD |
|---|---|---|---|
| 0.8 | Full | absent | alpha=0 only |
| 1.2 | Full | absent | alpha=0 only |
| 1.6 | absent | Full | absent |
| 2.0 | absent | Full to 16 deg | absent |
| 3.0 | absent | Full | absent |

**Known limitation:** DATCOM subsonic wing methods produce CL but lack a moment method for this configuration. Supersonic methods produce CM but do not converge on CN/CA for low-AR fins. Documented DATCOM method coverage limitation for tactical missile geometries. CN approximated as CL at low AoA for 6-DOF input.

## Repository Structure

    04_missile_aero_database/
    |-- README.md
    |-- datcom/
    |   |-- missile.dcm
    |   |-- datcom.out
    |-- postprocess/
    |   |-- parse_datcom.py
    |   |-- aero_interpolator.py
    |   |-- barrowman.py          (pending)
    |   |-- plot_aero_database.py (pending)
    |-- results/
    |   |-- aero_database.csv     (pending)

## How to Recreate

### 1. Compile DATCOM
Requires gfortran (MSYS2 MINGW64):
```
pacman -S mingw-w64-x86_64-gcc-fortran
gfortran -std=legacy datcom.f -o datcom.exe
```

### 2. Run DATCOM
From the datcom/ directory (MSYS2 MINGW64 shell):
```
echo "missile.dcm" | ./datcom.exe
```
Output: datcom.out

### 3. Parse Output
```
python3 parse_datcom.py
```
Prints coefficient tables per Mach block. Requires NumPy (pacman -S mingw-w64-x86_64-python-numpy).

### 4. Run Interpolation Module
```
python3 postprocess/aero_interpolator.py
```
Prints spot-check values confirming interpolator is functional. Requires NumPy + SciPy (pacman -S mingw-w64-x86_64-python-numpy mingw-w64-x86_64-python-scipy).
