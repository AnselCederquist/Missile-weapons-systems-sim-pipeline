# Missile Assembly — CAD

Full tactical missile assembly modeled in Onshape. All parts are dimensionally 
consistent with the simulation pipeline — body diameter, fin geometry, nose cone, 
and nozzle match the parameters used in Projects 02–07.

## Assembly Overview

| Part | File | Source |
|---|---|---|
| Body tube | Rocket_body_tube.x_t | Modeled in Onshape |
| Nose cone | Rocket_Cone_Tip(forAssembly).x_t | Modeled in Onshape (Project 07) |
| Fin assembly ×4 | Rocket_Fin.x_t | Modeled in Creo (Project 02) |
| Nozzle | Exhaust_Nozzle_scaled(forAssembly).x_t | Modeled in Onshape (scaled from Project 03) |
| Full assembly | Rocket_Assembly.x_t | Onshape assembly |

## Key Dimensions

| Parameter | Value | Reference |
|---|---|---|
| Body OD | 100mm | Aero database reference diameter |
| Body wall thickness | 3mm | — |
| Total body length | 1000mm | L_body from 6-DOF sim |
| Nose cone length | 128.38mm | Onshape (10mm tip fillet) |
| Total assembly length | ~1050mm | Including nozzle protrusion |
| XCG from nose | 500mm | Project 06 mass properties |
| XCP from nose | 750mm | Project 04 Barrowman |
| Static margin | 250mm (25% L_body) | Projects 04/06 |
| Fin span | 60mm | Project 02 |
| Fin thickness | 4mm | Project 02 |

## Section Breakdown

| Section | X from nose (mm) | Notes |
|---|---|---|
| Nose cone | 0 – 128 | Ti-6Al-4V, 10mm tip fillet, 5mm wall |
| Warhead bay | 128 – 250 | Representative internal mass |
| Avionics bay | 250 – 420 | GNC electronics, IMU |
| Motor casing | 420 – 900 | Propellant grain, largest section |
| Fin attach ring | 900 – 1000 | 15mm wall, 4× dovetail slots, nozzle counterbore |
| Nozzle | 1000 – 1050 | Scaled from Project 03 CFD geometry ×0.128 |

## Design Features

**Bulkheads** — internal rings at x=250, 420mm. OD=100mm, ID=40mm, 4mm thick.
Passthrough hole for wiring and plumbing. Forward interface ring at x=0 (nose end)
provides positive stop for nose cone shoulder.

**Nose cone attachment** — spigot on aft face of nose cone (OD=94mm, length=15mm)
slides into body tube ID. Cone base shoulder contacts forward interface ring.

**Fin attachment** — dovetail slots cut into fin attach ring. Slot: 20mm wide,
10mm deep, 4mm thick. 1mm deep × 60mm long × 4mm wide recess cut on outer
surface so fin plate sits flush with body tube OD rather than proud of it.
4× slots at 90° spacing via circular pattern.

**Nozzle attachment** — nozzle inlet (OD=72mm) seats into 20mm deep counterbore
in aft face of fin attach ring (ID=72mm). 4mm wall nozzle casing.
Nozzle scaled from Project 03 CFD geometry by factor 0.128 to fit 100mm body.

## GD&T

Formal GD&T applied to the fin assembly and nose cone drawings. These are the
two components structurally analyzed in Projects 02 and 07 — tolerances are
tied directly to FEA boundary condition assumptions and critical stress
locations. Other parts (body tube, nozzle, assembly) are visualization
geometry and do not require manufacturing tolerances.

### Fin Assembly (`drawings/Rocket_Fin.pdf`)

Datum A established at tab bottom face — the fixed support boundary condition
in Project 02 FEA. All geometric tolerances reference Datum A.

| Feature | Tolerance | Type | Reason |
|---|---|---|---|
| Tab bottom face | Datum A | — | Fixed support BC — all geometric tolerances reference this face |
| Tab depth (10mm) | ±0.1mm | Size | Controls seating depth in body tube dovetail slot |
| Tab width (20mm) | ±0.2mm | Size | Controls dovetail slot fit and lateral constraint |
| Tab bottom face | ⏥ 0.05mm | Flatness | Ensures uniform contact at fixed support — warped face changes load distribution |
| R3 fillet | ⌓ 0.4mm / A | Profile of a surface | Critical stress concentration from Project 02 — tighter radius increases stress, looser radius weakens tab-fin junction |
| R2 fillet | ⌓ 0.3mm / A | Profile of a surface | Secondary stress location at tab corner |
| Fin plate face | ⟂ 0.1mm / A | Perpendicularity | Ensures fin stands perpendicular to tab bottom — misalignment would introduce bending moment under axial setback load |

### Nose Cone (`drawings/Nose_Cone(forAssembly).pdf`)

Datum A established at base circular face — the fixed support boundary
condition in Project 07 FEA.

| Feature | Tolerance | Type | Reason |
|---|---|---|---|
| Base circular face | Datum A | — | Fixed support BC in Project 07 |
| Base diameter (100mm) | ±0.2mm | Size | Controls fit with body tube spigot interface |
| Wall thickness (5mm) | ±0.2mm | Size | Governs thermal mass and heat conduction path — directly affects transient thermal result in Project 07 |
| Tip fillet radius (R10mm) | ±0.5mm | Size | Aeroheating critical location — smaller radius concentrates heat flux; documented in Project 07 aeroheating section |
| Tip fillet | ⌓ 0.8mm / A | Profile of a surface | Controls actual tip shape — profile error at tip changes effective heat flux concentration |
| Base face | ⏥ 0.1mm | Flatness | Ensures uniform fixed support contact |
| Cone wall | ⏭ 0.1mm | Parallelism | Ensures uniform wall thickness — thickness variation would create asymmetric thermal gradients |
| Cone half-angle | ±0.5° | Angular (title block) | General angular tolerance covering cone half-angle — not structurally critical but controls external aero profile |

### Parts Without GD&T

| Part | Reason |
|---|---|
| Body tube | Visualization geometry — not structurally analyzed; interfaces (spigot bore, dovetail slots) are controlled by mating part tolerances |
| Nozzle (scaled) | Scaled visualization from Project 03 CFD fluid domain — not the primary CFD geometry; no structural or thermal analysis performed |
| Assembly drawing | Assembly drawing shows relationships between parts — individual part tolerances control fit, no additional assembly-level GD&T required at this stage |


## Notes

- **Nose cone retention tab:** An insert spigot tab was added to the aft face of
  the nose cone to mechanically lock it to the body tube in the assembly. This
  feature is not present in the Project 07 thermal-structural geometry. It sits
  aft of the heated leading tip region and has no effect on the aeroheating or
  structural results reported in Project 07.

- **Nozzle scaling:** Nozzle geometry is scaled from the Project 03 CFD fluid
  domain by factor 0.128 to fit the 100mm body diameter. The De Laval profile
  is preserved. This is a visualization geometry for the assembly — the primary
  CFD results and mesh convergence study in Project 03 used the full-size domain.

- Internal bay contents (warhead, avionics, propellant grain) are representative
  shapes for visualization only — not structurally analyzed.

- Material shown as Ti-6Al-4V throughout for visual consistency with Projects 02
  and 07. Real missile body tube would likely be aluminum or composite for mass
  reduction.

## File Formats

All parts exported as Parasolid (.x_t) from Onshape for compatibility with Ansys
and other CAD tools. Assembly exported as STEP (.step) for universal import.
Drawings exported as PDF.