"""
test_sixdof.py -- Validation tests for 6-DOF flight mechanics simulator
1. Trajectory sanity checks -- range, apogee, flight time, burnout
2. Energy budget check -- work-energy theorem
3. Static margin check -- passive stability
4. EKF RMSE check -- navigation accuracy on 6-DOF trajectory
"""

import numpy as np
import sys
sys.path.append("D:/Weapons-systems-sim-pipeline/06_6dof_missile_sim/src")
sys.path.append("D:/Weapons-systems-sim-pipeline/04_missile_aero_database/postprocess")

# ---------------------------------------------------------------
# Import sim parameters and run simulation
# ---------------------------------------------------------------
from sixdof import (
    run_sixdof,
    m_launch, m_burnout, t_sustain,
    XCG, XCP, g, rho0, a_sound, A_ref,
    atmosphere, thrust, mass
)

print("Running 6-DOF simulation for tests...")
states, ts, speed, mach = run_sixdof()
dt = ts[1] - ts[0]

print("=== 6-DOF Simulation Validation Tests ===\n")

# ---------------------------------------------------------------
# TEST 1 -- Trajectory sanity checks
# ---------------------------------------------------------------
print("TEST 1: Trajectory sanity checks")

flight_time  = ts[-1]
max_range    = states[-1, 0]
max_alt      = states[:, 2].max()
max_mach     = mach.max()
burnout_done = flight_time >= t_sustain

assert flight_time > 30.0,   f"FAIL: flight time {flight_time:.1f}s < 30s"
assert max_range  > 8000.0,  f"FAIL: range {max_range:.0f}m < 8000m"
assert max_alt    > 1500.0,  f"FAIL: apogee {max_alt:.0f}m < 1500m"
assert max_mach   > 1.0,     f"FAIL: max Mach {max_mach:.2f} < 1.0"
assert burnout_done,          f"FAIL: burnout did not complete (t={flight_time:.1f}s)"

print(f"  Flight time:  {flight_time:.1f} s        PASS")
print(f"  Range:        {max_range:.0f} m      PASS")
print(f"  Max altitude: {max_alt:.0f} m       PASS")
print(f"  Max Mach:     {max_mach:.2f}          PASS")
print(f"  Burnout:      complete             PASS")

# ---------------------------------------------------------------
# TEST 2 -- Energy budget
# ---------------------------------------------------------------
print("\nTEST 2: Energy budget check")

dt = ts[1] - ts[0]

# Total thrust work
thrust_work = sum(thrust(ts[i]) * speed[i] * dt for i in range(len(ts)))

# Drag work (energy lost to drag)
drag_work = 0.0
for i in range(len(ts)):
    rho   = atmosphere(states[i, 2])
    qbar  = 0.5 * rho * speed[i]**2
    CD    = 0.3  # conservative estimate
    F_d   = qbar * A_ref * CD
    drag_work += F_d * speed[i] * dt

# Final kinetic + potential energy
KE_final = 0.5 * m_burnout * speed[-1]**2
PE_final = m_burnout * g * states[-1, 2]

# Energy balance -- final energy should be less than thrust work
energy_balance = thrust_work - drag_work - KE_final - PE_final
assert energy_balance > 0, f"FAIL: energy balance {energy_balance:.0f} J < 0 (unphysical)"

print(f"  Thrust work:    {thrust_work/1e6:.2f} MJ")
print(f"  Drag work:      {drag_work/1e6:.2f} MJ")
print(f"  Final KE+PE:    {(KE_final+PE_final)/1e6:.2f} MJ")
print(f"  Energy balance: {energy_balance/1e6:.2f} MJ  PASS")

# ---------------------------------------------------------------
# TEST 3 -- Static margin
# ---------------------------------------------------------------
print("\nTEST 3: Static margin check")

static_margin = XCP - XCG
assert static_margin > 0,    "FAIL: negative static margin -- missile unstable"
assert static_margin < 0.5,  "FAIL: static margin > 0.5m -- overstabilized"

print(f"  XCG: {XCG:.2f} m from nose")
print(f"  XCP: {XCP:.2f} m from nose")
print(f"  Static margin: {static_margin:.2f} m ({static_margin*100:.0f}% body length)  PASS")

# ---------------------------------------------------------------
# TEST 4 -- EKF RMSE on 6-DOF trajectory
# ---------------------------------------------------------------
print("\nTEST 4: EKF navigation accuracy")

np.random.seed(42)
sigma_a   = 0.5
sigma_gps = 5.0
sigma_v   = 0.1
n_ekf     = len(ts)

accel_meas = np.diff(states[:,3:6], axis=0)/dt + np.random.randn(n_ekf-1,3)*sigma_a
gps_pos    = states[:,0:3] + np.random.randn(n_ekf,3)*sigma_gps
gps_vel    = states[:,3:6] + np.random.randn(n_ekf,3)*sigma_v

Q     = np.eye(6); Q[0:3,0:3] *= 1.0; Q[3:6,3:6] *= 10.0
R_gps = np.eye(6)
R_gps[0:3,0:3] *= sigma_gps**2
R_gps[3:6,3:6] *= sigma_v**2
H     = np.eye(6)

x_est = np.zeros((n_ekf, 6))
P     = np.eye(6) * 100.0
x_est[0] = states[0,:6] + np.random.randn(6)*[10,10,10,1,1,1]

for i in range(1, n_ekf):
    xp = x_est[i-1].copy()
    a  = accel_meas[i-1] if i-1 < len(accel_meas) else np.zeros(3)
    xp[0:3] += xp[3:6]*dt
    xp[3]   += a[0]*dt
    xp[4]   += a[1]*dt
    xp[5]   += (a[2] - g)*dt
    F_      = np.eye(6); F_[0,3]=dt; F_[1,4]=dt; F_[2,5]=dt
    P       = F_ @ P @ F_.T + Q*dt
    if i % 10 == 0:
        zm = np.concatenate([gps_pos[i], gps_vel[i]])
        y_ = zm - H @ xp
        S_ = H @ P @ H.T + R_gps
        K_ = P @ H.T @ np.linalg.inv(S_)
        xp = xp + K_ @ y_
        P  = (np.eye(6) - K_ @ H) @ P
    x_est[i] = xp

pos_rmse = np.sqrt(np.mean((x_est[:,:3] - states[:,:3])**2))
vel_rmse = np.sqrt(np.mean((x_est[:,3:6] - states[:,3:6])**2))

assert pos_rmse < 5.0,  f"FAIL: position RMSE {pos_rmse:.2f}m > 5m"
assert vel_rmse < 2.0,  f"FAIL: velocity RMSE {vel_rmse:.2f}m/s > 2 m/s"

print(f"  Position RMSE: {pos_rmse:.2f} m   PASS")
print(f"  Velocity RMSE: {vel_rmse:.2f} m/s  PASS")

print("\n=== ALL TESTS PASSED ===")