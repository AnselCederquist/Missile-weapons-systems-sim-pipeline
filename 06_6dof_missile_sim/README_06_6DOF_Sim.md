# Project 06 - 6-DOF Flight Mechanics Simulator

Capstone of the weapons systems simulation pipeline. Full 6-degree-of-freedom 
flight mechanics simulator for a boost-sustain tactical missile. Integrates the 
aerodynamic database from Project 04, EKF navigation from Project 05, and a 
variable-mass boost-sustain propulsion model. RK4 numerical integration. 
EKF state estimation overlaid on 6-DOF truth trajectory.

## Overview

A 6-DOF flight mechanics simulator is the central tool in any missile design 
program. It integrates all upstream subsystem models -- aerodynamics, propulsion, 
mass properties, atmosphere -- into a single time-domain simulation that predicts 
where the missile goes, how fast it gets there, and what forces it experiences 
along the way. This is the simulation that validates whether the design actually 
works before hardware is built.

This project implements the full 6-DOF equations of motion: three translational 
states (x, y, z position) and three rotational states (roll, pitch, yaw), each 
with corresponding velocity/rate states for 12 states total. Forces include 
boost-sustain thrust with variable mass, aerodynamic drag from the Project 04 
DATCOM database via aero_interpolator.get_CD(mach, alpha), and gravity. 
Moments include pitch/yaw aerodynamic CM from the DATCOM database and pitch 
damping Cmq. RK4 integration at dt=0.01s.

The EKF from Project 05 is applied directly to the 6-DOF trajectory, fusing 
simulated IMU and GPS measurements to produce a navigation solution tracking 
the truth trajectory. A 1 deg yaw offset at launch introduces lateral drift, 
demonstrating the full 3D capability of the simulation. The 3D trajectory plot 
shows the 6-DOF truth, EKF navigation solution, and Project 05 standalone EKF 
on the same axes -- the pipeline is fully integrated.

Key result: 43.2s flight, 11.4km range, 2006m apogee, Mach 1.25 peak. 
EKF position RMSE 1.11m over the full trajectory.

## State Vector

| Index | State | Description |
|---|---|---|
| 0 | x | Downrange position (m) |
| 1 | y | Lateral position (m) |
| 2 | z | Altitude (m) |
| 3 | vx | Downrange velocity (m/s) |
| 4 | vy | Lateral velocity (m/s) |
| 5 | vz | Vertical velocity (m/s) |
| 6 | phi | Roll angle (rad) |
| 7 | theta | Pitch angle (rad) |
| 8 | psi | Yaw angle (rad) |
| 9 | p | Roll rate (rad/s) |
| 10 | q | Pitch rate (rad/s) |
| 11 | r | Yaw rate (rad/s) |

## Mass and Propulsion

| Parameter | Value |
|---|---|
| Launch mass | 20 kg |
| Burnout mass | 15 kg |
| Propellant mass | 5 kg |
| Boost thrust | 3000 N (t = 0-2s) |
| Sustain thrust | 800 N (t = 2-10s) |
| Burnout time | 10 s |
| Mass flow rate | 0.625 kg/s (sustain phase only) |

## Aerodynamics

| Parameter | Value | Source |
|---|---|---|
| Reference area | 0.00785 m2 | Project 04 |
| Reference length | 0.100 m | Project 04 |
| CD(M, alpha) | From aero_interpolator.py | Project 04 DATCOM |
| CM(M, alpha) | From aero_interpolator.py | Project 04 DATCOM |
| Cmq | -50 /rad | Pitch damping -- standard approximation |
| XCG | 0.50 m from nose | Project 04 |
| XCP | 0.75 m from nose | Project 04 Barrowman |
| Static margin | 0.25 m (25% body length) | XCP - XCG |

## Results

| Metric | Value |
|---|---|
| Flight time | 43.2 s |
| Range | 11,405 m |
| Max altitude | 2,006 m |
| Max Mach | 1.25 |
| Apogee time | 21.9 s |
| Apogee downrange | 6,556 m |
| Burnout speed | 380 m/s |
| Burnout Mach | 1.11 |
| Max dynamic pressure | 111.7 kPa |
| Impact speed | 258 m/s |
| Impact Mach | 0.75 |
| EKF position RMSE | 1.11 m |
| EKF velocity RMSE | 0.33 m/s |

## Design Considerations and Limitations

**Euler angle singularity:** The 3-2-1 Euler angle representation has a 
singularity at theta = +/-90 deg (pitch straight up or down). The pitch 
angle state diverges numerically as the missile pitches over during the 
ballistic arc. Mitigated by clamping theta in the kinematic equations and 
decoupling translational from rotational dynamics -- translational EOM are 
computed in the inertial frame directly, bypassing the corrupted DCM. 
Attitude states are propagated for informational purposes but not fed back 
into force calculations. Full fix requires quaternion attitude representation.

**Translational vs rotational coupling:** As a consequence of the Euler 
singularity fix, thrust direction is computed from the DCM (which diverges) 
but drag is computed directly from inertial velocity. This is physically 
consistent for a passively stable missile flying near its trim condition but 
would break down for large AoA maneuvers. Acceptable for a ballistic 
trajectory study.

**FPA dip at t=8-10s:** A visible dip in flight path angle and dynamic 
pressure occurs at the boost-to-sustain thrust cutoff (t=10s). The abrupt 
thrust reduction from 800N to 0N produces a transient speed reduction. 
Physical but cosmetically noisy -- a thrust ramp-down would smooth it.

**Drag only aerodynamics in translational EOM:** CD is evaluated at alpha=0 
and current Mach. Lift force is not included in the translational EOM -- 
acceptable for near-zero AoA ballistic flight but would require the full 
aero_interpolator.get_CL(mach, alpha) call with attitude estimate for a 
maneuvering trajectory.

**Planar trajectory:** Launch is in the XZ plane with a 1 deg yaw offset 
to demonstrate 3D capability. No crosswind or lateral guidance modeled.

**No guidance law:** This simulation is a ballistic trajectory -- the missile 
follows the initial launch condition with no in-flight guidance corrections. 
A full GNC implementation would add a proportional navigation (PN) guidance 
law computing fin deflection commands to intercept a moving target, a control 
allocator mapping commands to moments, and a miss distance metric. This is 
scoped as a follow-on project.

**No control surfaces:** Fin deflection is not modeled. The missile is 
passively stable (positive static margin 0.25m) but uncontrolled. Adding 
control would require actuator dynamics, a control law, and updated moment 
equations including control moment contributions.

**Fixed atmosphere:** Standard exponential atmosphere model with sea-level 
density. No wind, no gusts, no atmospheric turbulence. Lateral drift is from 
initial yaw offset only.

**Constant CD at alpha=0:** Drag coefficient evaluated at zero angle of attack 
for all timesteps. At non-zero AoA the induced drag component is larger -- this 
underpredicts drag during the boost phase when thrust is not perfectly aligned 
with velocity. Acceptable for a ballistic study but would require full 
aero_interpolator.get_CD(mach, alpha) with attitude estimate for higher fidelity.

**RK4 fixed timestep:** dt=0.01s fixed step. Adaptive step size (e.g. RK45) 
would improve accuracy during the high-acceleration boost phase and reduce 
integration error at thrust transitions.

## Pipeline Integration

| Upstream project | Contribution |
|---|---|
| Project 04 | CD(M,alpha), CM(M,alpha) via aero_interpolator.py |
| Project 05 | EKF applied to 6-DOF trajectory -- navigation solution |

## How to Recreate
```
python 06_6dof_missile_sim/src/sixdof.py
```

Requires NumPy, Matplotlib, and Projects 04 and 05 postprocess/src on 
sys.path. Outputs saved to results/figures/.

## Repository Structure

    06_6dof_missile_sim/
    |-- README_06_6DOF_Sim.md
    |-- src/
    |   |-- sixdof.py               # 6-DOF EOM + EKF overlay + plots
    |-- results/
        |-- figures/
            |-- sixdof_trajectory.png     # 6-panel: trajectory, Mach, FPA, qbar, EKF
            |-- sixdof_3d_trajectory.png  # 3D trajectory with EKF overlay
```

Then commit and push:
```
cd /d D:\Weapons-systems-sim-pipeline
git add 06_6dof_missile_sim/
git add README.md
git commit -m "Project 06: 6-DOF flight mechanics simulator

- sixdof.py: 12-state RK4, boost-sustain propulsion, DATCOM aero
- EKF on 6-DOF trajectory: position RMSE 1.11m
- 3D trajectory with 1 deg yaw offset and EKF overlay
- Pipeline integration: Projects 04+05 feeding 06
- README_06_6DOF_Sim.md"
git push