# Missile Assembly — CAD

Full tactical missile assembly modeled in Onshape. All parts are dimensionally 
consistent with the simulation pipeline — body diameter, fin geometry, nose cone, 
and nozzle match the parameters used in Projects 02–07.

## Assembly Overview

| Part | File | Source |
|---|---|---|
| Body tube | body_tube.x_t | Modeled in Onshape |
| Nose cone | nose_cone.x_t | Modeled in Onshape (Project 07) |
| Fin assembly ×4 | fin_assembly.x_t | Modeled in Creo (Project 02) |
| Nozzle | nozzle.x_t | Modeled in Onshape (scaled from Project 03) |
| Full assembly | missile_assembly.x_t | Onshape assembly |

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

## Notes

- Nozzle geometry is scaled from the Project 03 CFD fluid domain for visual
  consistency. The CFD study used a larger representative geometry — the scaled
  nozzle preserves the correct De Laval profile but is not the primary CFD result.
- Internal bay contents (warhead, avionics, propellant grain) are representative
  shapes for visualization purposes only — not structurally analyzed.
- Material shown as Ti-6Al-4V throughout for consistency with Projects 02 and 07.
  Real missile body tube would likely be aluminum or composite for mass reduction.

## File Formats

All parts exported as Parasolid (.x_t) from Onshape for compatibility with Ansys
and other CAD tools. Assembly exported as STEP (.step) for universal import.
Drawings exported as PDF.