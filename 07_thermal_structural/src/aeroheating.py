"""
aeroheating.py -- Aerodynamic Heating Analysis
Project 07 -- Thermal-Structural

Computes stagnation point and fin leading edge heat flux and surface
temperature history from Project 06 trajectory data.
Inputs: Mach, altitude, dynamic pressure from sixdof.py run_sixdof()
Outputs: temperature history CSV for Ansys thermal-structural boundary conditions

Methods:
- Stagnation temperature: isentropic relations
- Stagnation heat flux: Fay-Riddell simplified (laminar stagnation point)
- Fin LE heat flux: Fay-Riddell with leading edge radius
- Surface temperature: simplified thermal response (lumped capacitance)
"""

import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import sys
import os
sys.path.append("D:/Weapons-systems-sim-pipeline/06_6dof_missile_sim/src")
from sixdof import run_sixdof, atmosphere, a_sound

# ---------------------------------------------------------------
# Run Project 06 trajectory
# ---------------------------------------------------------------
print("Loading Project 06 trajectory...")
states, ts, speed, mach = run_sixdof()
dt = ts[1] - ts[0]

altitude  = states[:, 2]
n         = len(ts)

# ---------------------------------------------------------------
# Atmosphere properties along trajectory
# ---------------------------------------------------------------
gamma = 1.4
R_air = 287.0   # J/kg·K
T_sl  = 288.15  # K sea level temperature

def temperature(alt):
    """ISA temperature approximation"""
    alt = max(alt, 0.0)
    if alt < 11000:
        return T_sl - 0.00649 * alt
    return 216.65

def density(alt):
    return atmosphere(alt)

T_static  = np.array([temperature(h) for h in altitude])
rho_arr   = np.array([density(h)     for h in altitude])

# ---------------------------------------------------------------
# Stagnation conditions
# ---------------------------------------------------------------
# Stagnation temperature (isentropic)
T_stag = T_static * (1 + (gamma - 1) / 2 * mach**2)

# Stagnation pressure ratio
P_ratio = (1 + (gamma - 1) / 2 * mach**2) ** (gamma / (gamma - 1))

# Dynamic pressure
qbar = 0.5 * rho_arr * speed**2

print(f"  Max Mach:              {mach.max():.2f}")
print(f"  Max stagnation temp:   {T_stag.max():.1f} K  ({T_stag.max()-273.15:.1f} C)")
print(f"  Max dynamic pressure:  {qbar.max()/1000:.1f} kPa")

# ---------------------------------------------------------------
# Heat flux -- Fay-Riddell simplified
# Stagnation point: q = C * sqrt(rho * rho_stag) * V^3 / R_nose^0.5
# Simplified form: q_stag = k * rho^0.5 * V^3 (Detra-Kemp-Riddell)
# ---------------------------------------------------------------
# Nose tip
R_nose    = 0.010   # m -- nose tip radius (blunt conical tip, ~10mm)
k_DKR     = 1.83e-4  # Detra-Kemp-Riddell constant (SI units)

q_nose = k_DKR * np.sqrt(rho_arr / R_nose) * speed**3  # W/m^2

# Fin leading edge
R_fin_LE  = 0.002   # m -- fin leading edge radius (~2mm, sharp wedge)
q_fin_LE  = k_DKR * np.sqrt(rho_arr / R_fin_LE) * speed**3  # W/m^2

print(f"\n  Max nose heat flux:    {q_nose.max()/1e6:.2f} MW/m^2")
print(f"  Max fin LE heat flux:  {q_fin_LE.max()/1e6:.2f} MW/m^2")

# ---------------------------------------------------------------
# Surface temperature rise -- lumped capacitance
# dT/dt = (q_in - q_rad) / (rho_mat * cp_mat * thickness)
# Radiation: q_rad = epsilon * sigma * T^4
# ---------------------------------------------------------------
# Ti-6Al-4V properties
rho_ti    = 4430.0   # kg/m^3
cp_ti     = 560.0    # J/kg·K
thickness = 0.004    # m -- fin thickness at leading edge
epsilon   = 0.3      # emissivity (polished Ti)
sigma_sb  = 5.67e-8  # W/m^2·K^4

def compute_surface_temp(q_flux, T_init=288.15):
    T_surf = np.zeros(n)
    T_surf[0] = T_init
    for i in range(1, n):
        q_in  = q_flux[i-1]
        q_rad = epsilon * sigma_sb * T_surf[i-1]**4
        dTdt  = (q_in - q_rad) / (rho_ti * cp_ti * thickness)
        T_surf[i] = T_surf[i-1] + dTdt * dt
        T_surf[i] = max(T_surf[i], T_init)  # don't go below ambient
    return T_surf

print("\nComputing surface temperatures...")
T_nose_surf   = compute_surface_temp(q_nose)
T_fin_surf    = compute_surface_temp(q_fin_LE)

print(f"  Max nose surface temp: {T_nose_surf.max():.1f} K  ({T_nose_surf.max()-273.15:.1f} C)")
print(f"  Max fin LE surface temp: {T_fin_surf.max():.1f} K  ({T_fin_surf.max()-273.15:.1f} C)")

# ---------------------------------------------------------------
# Ti-6Al-4V property degradation check
# Yield strength vs temperature (approximate from literature)
# ---------------------------------------------------------------
T_yield_data = np.array([293, 373, 473, 573, 673, 773])  # K
Y_yield_data = np.array([880, 830, 790, 730, 620, 480])  # MPa

T_nose_max_C = T_nose_surf.max() - 273.15
T_fin_max_C  = T_fin_surf.max()  - 273.15

yield_at_nose = np.interp(T_nose_surf.max(), T_yield_data, Y_yield_data)
yield_at_fin  = np.interp(T_fin_surf.max(),  T_yield_data, Y_yield_data)

print(f"\n=== Ti-6Al-4V Property Degradation ===")
print(f"  Nose tip max temp:     {T_nose_max_C:.1f} C")
print(f"  Yield strength at nose temp: {yield_at_nose:.0f} MPa  (RT: 880 MPa)")
print(f"  Retention:             {yield_at_nose/880*100:.1f}%")
print(f"")
print(f"  Fin LE max temp:       {T_fin_max_C:.1f} C")
print(f"  Yield strength at fin temp:  {yield_at_fin:.0f} MPa  (RT: 880 MPa)")
print(f"  Retention:             {yield_at_fin/880*100:.1f}%")

if T_fin_surf.max() > 573:
    print(f"\n  WARNING: Fin LE exceeds 300C -- significant yield strength degradation")
else:
    print(f"\n  OK: Fin LE below 300C -- Ti-6Al-4V properties largely retained")

# ---------------------------------------------------------------
# Export CSV for Ansys
# ---------------------------------------------------------------
os.makedirs("D:/Weapons-systems-sim-pipeline/07_thermal_structural/results", exist_ok=True)

df = pd.DataFrame({
    "time_s":          ts,
    "mach":            mach,
    "altitude_m":      altitude,
    "qbar_kPa":        qbar / 1000,
    "T_static_K":      T_static,
    "T_stagnation_K":  T_stag,
    "q_nose_Wm2":      q_nose,
    "q_fin_LE_Wm2":    q_fin_LE,
    "T_nose_surf_K":   T_nose_surf,
    "T_fin_surf_K":    T_fin_surf,
})
df.to_csv("D:/Weapons-systems-sim-pipeline/07_thermal_structural/results/aeroheating.csv",
          index=False)
print(f"\naeroheating.csv saved ({len(df)} rows)")

# ---------------------------------------------------------------
# Plots
# ---------------------------------------------------------------
os.makedirs("D:/Weapons-systems-sim-pipeline/07_thermal_structural/results/figures",
            exist_ok=True)

fig, axes = plt.subplots(2, 3, figsize=(15, 8))
fig.suptitle("Aerodynamic Heating -- Project 07", fontsize=13)

axes[0,0].plot(ts, mach, "b-", lw=1.5)
axes[0,0].set_xlabel("Time (s)"); axes[0,0].set_ylabel("Mach")
axes[0,0].set_title("Mach Number"); axes[0,0].grid(True, alpha=0.3)

axes[0,1].plot(ts, T_stag - 273.15, "r-", lw=1.5)
axes[0,1].set_xlabel("Time (s)"); axes[0,1].set_ylabel("Temperature (C)")
axes[0,1].set_title("Stagnation Temperature"); axes[0,1].grid(True, alpha=0.3)

axes[0,2].plot(ts, q_nose/1e6,   "r-",  lw=1.5, label="Nose tip")
axes[0,2].plot(ts, q_fin_LE/1e6, "b--", lw=1.5, label="Fin LE")
axes[0,2].set_xlabel("Time (s)"); axes[0,2].set_ylabel("Heat flux (MW/m²)")
axes[0,2].set_title("Aeroheating Heat Flux"); axes[0,2].legend(); axes[0,2].grid(True, alpha=0.3)

axes[1,0].plot(ts, T_nose_surf - 273.15, "r-",  lw=1.5, label="Nose tip")
axes[1,0].plot(ts, T_fin_surf  - 273.15, "b--", lw=1.5, label="Fin LE")
axes[1,0].axhline(300, color="k", ls=":", lw=1, label="300C threshold")
axes[1,0].set_xlabel("Time (s)"); axes[1,0].set_ylabel("Surface temp (C)")
axes[1,0].set_title("Surface Temperature"); axes[1,0].legend(); axes[1,0].grid(True, alpha=0.3)

axes[1,1].plot(T_yield_data - 273.15, Y_yield_data, "ko-", lw=1.5)
axes[1,1].axvline(T_nose_max_C, color="r", ls="--", label=f"Nose {T_nose_max_C:.0f}C")
axes[1,1].axvline(T_fin_max_C,  color="b", ls="--", label=f"Fin LE {T_fin_max_C:.0f}C")
axes[1,1].set_xlabel("Temperature (C)"); axes[1,1].set_ylabel("Yield strength (MPa)")
axes[1,1].set_title("Ti-6Al-4V Yield vs Temperature"); axes[1,1].legend()
axes[1,1].grid(True, alpha=0.3)

axes[1,2].plot(ts, qbar/1000, "g-", lw=1.5)
axes[1,2].set_xlabel("Time (s)"); axes[1,2].set_ylabel("Dynamic pressure (kPa)")
axes[1,2].set_title("Dynamic Pressure"); axes[1,2].grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig("D:/Weapons-systems-sim-pipeline/07_thermal_structural/results/figures/aeroheating.png",
            dpi=150, bbox_inches="tight")
print("Plot saved")
