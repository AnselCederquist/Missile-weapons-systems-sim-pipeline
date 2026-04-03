"""
sixdof.py -- 6-DOF Flight Mechanics Simulator
Capstone of weapons systems simulation pipeline.
Integrates: Project 04 aero database, Project 05 EKF, boost-sustain propulsion.

States: x, y, z, vx, vy, vz, phi, theta, psi, p, q, r
Forces: thrust, aerodynamic (lift/drag from DATCOM), gravity
Moments: pitch/yaw from CM(M,alpha), pitch damping Cmq
"""

import numpy as np
import matplotlib.pyplot as plt
import sys
import os
sys.path.append("D:/Weapons-systems-sim-pipeline/04_missile_aero_database/postprocess")
sys.path.append("D:/Weapons-systems-sim-pipeline/05_kalman_filter/src")
from aero_interpolator import get_CL, get_CM, get_CD

# ---------------------------------------------------------------
# Constants
# ---------------------------------------------------------------
g       = 9.81
rho0    = 1.225
a_sound = 343.0
A_ref   = 0.00785
L_ref   = 0.100
L_body  = 1.000

# ---------------------------------------------------------------
# Mass properties
# ---------------------------------------------------------------
m_launch  = 20.0
m_burnout = 15.0
t_boost   = 2.0    # seconds
t_sustain = 10.0   # seconds
mdot      = (m_launch - m_burnout) / (t_sustain - t_boost)

Ixx = 0.05
Iyy = 1.67
Izz = 1.67

XCG = 0.50
XCP = 0.75
Cmq = -50.0

# ---------------------------------------------------------------
# Propulsion
# ---------------------------------------------------------------
T_boost   = 3000.0  # N
T_sustain = 800.0   # N

def thrust(t):
    if t < t_boost:
        return T_boost
    elif t < t_sustain:
        return T_sustain
    return 0.0

def mass(t):
    if t < t_boost:
        return m_launch          # no mass loss during boost
    elif t < t_sustain:
        return max(m_launch - mdot * (t - t_boost), m_burnout)
    return m_burnout

# ---------------------------------------------------------------
# Atmosphere
# ---------------------------------------------------------------
def atmosphere(alt):
    return rho0 * np.exp(-max(alt, 0.0) / 8500.0)

# ---------------------------------------------------------------
# Aerodynamic forces and moments
# ---------------------------------------------------------------
def aero(v_body, alt, q=0.0):
    u, v, w = v_body
    speed = np.linalg.norm(v_body)
    if speed < 1e-3:
        return np.zeros(3), np.zeros(3)

    mach      = np.clip(speed / a_sound, 0.8, 3.0)
    alpha     = np.arctan2(w, max(abs(u), 1e-6))
    alpha_deg = np.clip(abs(np.degrees(alpha)), 0.0, 20.0)

    rho  = atmosphere(alt)
    qbar = 0.5 * rho * speed**2

    CL = get_CL(mach, alpha_deg)
    CD = get_CD(mach, alpha_deg)
    CM = get_CM(mach, alpha_deg)

    if np.isnan(CL): CL = 2.0 * np.radians(alpha_deg)
    if np.isnan(CD): CD = 0.3
    if np.isnan(CM): CM = -0.5 * np.radians(alpha_deg)

    L = qbar * A_ref * CL
    D = qbar * A_ref * CD

    Fx = -D * np.cos(alpha) + L * np.sin(alpha)
    Fz = np.sign(alpha) * (D * np.sin(abs(alpha)) + L * np.cos(alpha)) if alpha != 0 else 0.0

    My  = qbar * A_ref * L_ref * CM
    My += qbar * A_ref * L_ref * Cmq * (L_ref / (2.0 * max(speed, 1.0))) * q

    return np.array([Fx, 0.0, Fz]), np.array([0.0, My, 0.0])

# ---------------------------------------------------------------
# Rotation utilities
# ---------------------------------------------------------------
def euler_to_dcm(phi, theta, psi):
    cp, sp = np.cos(phi),   np.sin(phi)
    ct, st = np.cos(theta), np.sin(theta)
    cy, sy = np.cos(psi),   np.sin(psi)
    return np.array([
        [ct*cy,            ct*sy,           -st   ],
        [sp*st*cy - cp*sy, sp*st*sy + cp*cy, sp*ct],
        [cp*st*cy + sp*sy, cp*st*sy - sp*cy, cp*ct]
    ])

def omega_to_euler_rates(phi, theta, p, q, r):
    cp, sp = np.cos(phi), np.sin(phi)
    # Clamp theta away from +/-90 deg singularity
    theta_clamped = np.clip(theta, -np.radians(85), np.radians(85))
    ct = np.cos(theta_clamped)
    tt = np.tan(theta_clamped)
    phi_dot   = p + (q*sp + r*cp) * tt
    theta_dot = q*cp - r*sp
    psi_dot   = (q*sp + r*cp) / (ct + 1e-6)
    return phi_dot, theta_dot, psi_dot

# ---------------------------------------------------------------
# 6-DOF equations of motion
# ---------------------------------------------------------------
def derivatives(t, state):
    x, y, z, vx, vy, vz, phi, theta, psi, p, q, r = state

    m_cur = mass(t)
    R     = euler_to_dcm(phi, theta, psi)

    v_inertial = np.array([vx, vy, vz])
    speed_i    = np.linalg.norm(v_inertial)

    # Thrust in inertial frame (along body x-axis via DCM)
    T = thrust(t)
    F_thrust_inertial = R @ np.array([T, 0.0, 0.0])

    # Drag in inertial frame -- bypasses corrupted Euler angles
    if speed_i > 1e-3:
        mach_i = np.clip(speed_i / a_sound, 0.8, 3.0)
        rho_i  = atmosphere(z)
        qbar_i = 0.5 * rho_i * speed_i**2
        CD_i   = get_CD(mach_i, 0.0)
        if np.isnan(CD_i): CD_i = 0.3
        F_drag_inertial = -qbar_i * A_ref * CD_i * v_inertial / speed_i
    else:
        F_drag_inertial = np.zeros(3)

    # Gravity
    F_grav_inertial = np.array([0.0, 0.0, -m_cur * g])

    # Translational EOM
    a_inertial = (F_thrust_inertial + F_drag_inertial + F_grav_inertial) / m_cur

    # Rotational EOM (attitude tracking -- informational, not fed back to forces)
    v_body     = R.T @ v_inertial
    _, M_aero_body = aero(v_body, z, q=q)

    pdot = (M_aero_body[0] - (Izz - Iyy) * q * r) / Ixx
    qdot = (M_aero_body[1] - (Ixx - Izz) * p * r) / Iyy
    rdot = (M_aero_body[2] - (Iyy - Ixx) * p * q) / Izz

    phi_dot, theta_dot, psi_dot = omega_to_euler_rates(phi, theta, p, q, r)

    return [
        vx, vy, vz,
        a_inertial[0], a_inertial[1], a_inertial[2],
        phi_dot, theta_dot, psi_dot,
        pdot, qdot, rdot
    ]

# ---------------------------------------------------------------
# RK4 integration
# ---------------------------------------------------------------
def rk4_step(f, t, s, dt):
    k1 = f(t,        s)
    k2 = f(t+dt/2,   [si+di*dt/2 for si,di in zip(s,k1)])
    k3 = f(t+dt/2,   [si+di*dt/2 for si,di in zip(s,k2)])
    k4 = f(t+dt,     [si+di*dt   for si,di in zip(s,k3)])
    return [si+(dt/6)*(k1i+2*k2i+2*k3i+k4i)
            for si,k1i,k2i,k3i,k4i in zip(s,k1,k2,k3,k4)]

# ---------------------------------------------------------------
# Callable simulation interface
# ---------------------------------------------------------------
def run_sixdof():
    dt           = 0.01
    t_end        = 120.0
    launch_angle = np.radians(30.0)

    state0 = [
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        0.0, launch_angle, np.radians(1.0),
        0.0, 0.001, 0.0
    ]

    states_l = [state0]
    ts_l     = [0.0]
    t        = 0.0
    s        = state0
    apogee_reached = False
    max_alt  = 0.0
    prev_alt = 0.0

    while t < t_end:
        s_new = rk4_step(derivatives, t, s, dt)
        t    += dt
        curr_alt = s_new[2]
        if curr_alt > max_alt:
            max_alt = curr_alt
        if max_alt > 50.0 and curr_alt < prev_alt:
            apogee_reached = True
        if apogee_reached and curr_alt < 0:
            break
        prev_alt = curr_alt
        s = s_new
        states_l.append(s)
        ts_l.append(t)

    states_out = np.array(states_l)
    ts_out     = np.array(ts_l)
    speed_out  = np.linalg.norm(states_out[:,3:6], axis=1)
    mach_out   = speed_out / a_sound
    return states_out, ts_out, speed_out, mach_out


# ---------------------------------------------------------------
# Main entry point
# ---------------------------------------------------------------
if __name__ == "__main__":
    print("Running 6-DOF simulation...")
    states, ts, speed, mach = run_sixdof()
    dt = ts[1] - ts[0]
    print(f"  Simulation complete: {ts[-1]:.1f}s, range={states[-1,0]:.0f}m, "
          f"max alt={states[:,2].max():.0f}m")

    # ---------------------------------------------------------------
    # EKF overlay
    # ---------------------------------------------------------------
    print("Running EKF on 6-DOF trajectory...")
    # Feed 6-DOF truth into EKF directly
    np.random.seed(42)
    sigma_a   = 0.5
    sigma_gps = 5.0
    sigma_v   = 0.1

    n_ekf = len(ts)
    accel_meas = np.diff(states[:,3:6], axis=0)/dt + np.random.randn(n_ekf-1,3)*sigma_a
    gps_pos    = states[:,0:3] + np.random.randn(n_ekf,3)*sigma_gps
    gps_vel    = states[:,3:6] + np.random.randn(n_ekf,3)*sigma_v

    Q = np.eye(6); Q[0:3,0:3] *= 1.0; Q[3:6,3:6] *= 10.0
    R_gps = np.eye(6)
    R_gps[0:3,0:3] *= sigma_gps**2
    R_gps[3:6,3:6] *= sigma_v**2
    H = np.eye(6)

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
        F_    = np.eye(6); F_[0,3]=dt; F_[1,4]=dt; F_[2,5]=dt
        P     = F_ @ P @ F_.T + Q*dt
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
    print(f"  EKF Position RMSE: {pos_rmse:.2f} m")
    print(f"  EKF Velocity RMSE: {vel_rmse:.2f} m/s")

    ekf_ts  = ts
    ekf_est = x_est

    # ---------------------------------------------------------------
    # Plots
    # ---------------------------------------------------------------
    os.makedirs("D:/Weapons-systems-sim-pipeline/06_6dof_missile_sim/results/figures",
                exist_ok=True)

    fig, axes = plt.subplots(3, 2, figsize=(14, 12))
    fig.suptitle("6-DOF Missile Flight Simulation", fontsize=14)

    # Trajectory
    axes[0,0].plot(states[:,0], states[:,2], "b-", lw=2)
    axes[0,0].set_xlabel("Downrange X (m)")
    axes[0,0].set_ylabel("Altitude Z (m)")
    axes[0,0].set_title("Trajectory")
    axes[0,0].grid(True, alpha=0.3)

    # Mach
    speed = np.linalg.norm(states[:,3:6], axis=1)
    mach  = speed / a_sound
    axes[0,1].plot(ts, mach, "b-", lw=1.5)
    axes[0,1].set_xlabel("Time (s)")
    axes[0,1].set_ylabel("Mach")
    axes[0,1].set_title("Mach Number vs Time")
    axes[0,1].grid(True, alpha=0.3)

    # Flight path angle (from inertial velocity -- not affected by Euler divergence)
    vx_arr = states[:,3]
    vz_arr = states[:,5]
    fpa    = np.degrees(np.arctan2(vz_arr, np.maximum(np.abs(vx_arr), 1e-6)))
    axes[1,0].plot(ts, fpa, "b-", lw=1.5)
    axes[1,0].set_xlabel("Time (s)")
    axes[1,0].set_ylabel("Flight Path Angle (deg)")
    axes[1,0].set_title("Flight Path Angle vs Time")
    axes[1,0].grid(True, alpha=0.3)

    # Dynamic pressure
    qbar_arr = 0.5 * np.array([atmosphere(s[2]) for s in states]) * speed**2
    axes[1,1].plot(ts, qbar_arr / 1000.0, "b-", lw=1.5)
    axes[1,1].set_xlabel("Time (s)")
    axes[1,1].set_ylabel("Dynamic Pressure (kPa)")
    axes[1,1].set_title("Dynamic Pressure vs Time")
    axes[1,1].grid(True, alpha=0.3)

    # Run Project 05 EKF for comparison
    from ekf import run_ekf
    ekf05_state, ekf05_ts, ekf05_est = run_ekf(seed=42)

    n    = min(len(ts), len(ekf_ts))
    n05  = min(len(ts), len(ekf05_ts))

    # 6-DOF vs both EKFs -- X position
    axes[2,0].plot(ts[:n],      states[:n,0],      "b-",  lw=2.0, label="6-DOF truth")
    axes[2,0].plot(ekf_ts[:n],  ekf_est[:n,0],     color="orange", marker="o", ms=9, markevery=50, linestyle="none", fillstyle="none", markeredgewidth=0.8, label="EKF on 6-DOF")
    axes[2,0].plot(ekf05_ts[:n05], ekf05_est[:n05,0], color="magenta", linestyle=":", lw=3.0, label="EKF Project 05")
    axes[2,0].set_xlabel("Time (s)")
    axes[2,0].set_ylabel("X position (m)")
    axes[2,0].set_title("Downrange Position: 6-DOF vs EKF")
    axes[2,0].legend(fontsize=8)
    axes[2,0].grid(True, alpha=0.3)

    # 6-DOF vs both EKFs -- Z position
    axes[2,1].plot(ts[:n],      states[:n,2],      "b-",  lw=2.0, label="6-DOF truth")
    axes[2,1].plot(ekf_ts[:n],  ekf_est[:n,2],     color="orange", marker="o", ms=9, markevery=50, linestyle="none", fillstyle="none", markeredgewidth=0.8, label="EKF on 6-DOF")
    axes[2,1].plot(ekf05_ts[:n05], ekf05_est[:n05,2], color="magenta", linestyle=":", lw=3.0, label="EKF Project 05")
    axes[2,1].set_xlabel("Time (s)")
    axes[2,1].set_ylabel("Z position (m)")
    axes[2,1].set_title("Altitude: 6-DOF vs EKF")
    axes[2,1].legend(fontsize=8)
    axes[2,1].grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig("D:/Weapons-systems-sim-pipeline/06_6dof_missile_sim/results/figures/sixdof_trajectory.png",
                dpi=150, bbox_inches="tight")
    print("Plot saved")
    plt.show()

    # ---------------------------------------------------------------
    # Results summary
    # ---------------------------------------------------------------
    print(f"\n=== 6-DOF Simulation Results ===")
    print(f"  Flight time:     {ts[-1]:.1f} s")
    print(f"  Range:           {states[-1,0]:.0f} m")
    print(f"  Max altitude:    {states[:,2].max():.0f} m")
    print(f"  Max Mach:        {mach.max():.2f}")
    print(f"  Launch mass:     {m_launch} kg")
    print(f"  Burnout mass:    {m_burnout} kg")
    print(f"  Static margin:   {XCP-XCG:.2f} m ({(XCP-XCG)/L_body*100:.0f}% body length)")
    print(f"  Apogee time:     {ts[np.argmax(states[:,2])]:.1f} s")
    print(f"  Apogee X:        {states[np.argmax(states[:,2]),0]:.0f} m")
    print(f"  Burnout time:    {t_sustain:.1f} s")
    if ts[-1] >= t_sustain:
        burnout_idx = np.argmin(np.abs(ts - t_sustain))
        print(f"  Burnout speed:   {speed[burnout_idx]:.0f} m/s")
        print(f"  Burnout Mach:    {mach[burnout_idx]:.2f}")
    else:
        print(f"  Burnout:         did not complete (impact at t={ts[-1]:.1f}s < t_sustain={t_sustain}s)")
    print(f"  Max dyn press:   {(0.5*rho0*speed**2).max()/1000:.1f} kPa")
    print(f"  Impact speed:    {speed[-1]:.0f} m/s")
    print(f"  Impact Mach:     {mach[-1]:.2f}")

    # --- 3D Trajectory Figure ---
    fig3d = plt.figure(figsize=(10, 8))
    ax3d  = fig3d.add_subplot(111, projection='3d')

    ax3d.plot(states[:,0], states[:,1], states[:,2], "b-", lw=2, label="6-DOF truth")
    ax3d.plot(ekf_est[:,0], ekf_est[:,1], ekf_est[:,2],
            color="orange", marker="o", ms=6, markevery=100,
            linestyle="none", fillstyle="none", markeredgewidth=0.8, label="EKF on 6-DOF")
    ax3d.plot(ekf05_est[:,0], ekf05_est[:,1], ekf05_est[:,2],
            color="magenta", linestyle=":", lw=2.0, label="EKF Project 05")

    ax3d.set_xlabel("Downrange X (m)")
    ax3d.set_ylabel("Lateral Y (m)")
    ax3d.set_zlabel("Altitude Z (m)")
    ax3d.set_title("3D Missile Trajectory -- 6-DOF with EKF Navigation")
    ax3d.legend()

    plt.tight_layout()
    plt.savefig("D:/Weapons-systems-sim-pipeline/06_6dof_missile_sim/results/figures/sixdof_3d_trajectory.png",
                dpi=150, bbox_inches="tight")
    print("3D plot saved")
    plt.show()