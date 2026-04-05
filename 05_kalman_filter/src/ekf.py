import numpy as np
import matplotlib.pyplot as plt
import sys
sys.path.append("D:/Weapons-systems-sim-pipeline/04_missile_aero_database/postprocess")
from aero_interpolator import get_CD

# --- Constants ---
g       = 9.81
m       = 20.0
rho0    = 1.225
A_ref   = 0.00785
a_sound = 343.0

# --- Thrust profile ---
T_boost   = 3000.0
T_sustain = 500.0
t_boost   = 2.0
t_sustain = 10.0

def thrust(t):
    if t < t_boost:
        return T_boost
    elif t < t_sustain:
        return T_sustain
    return 0.0

def drag(v, alt=0.0):
    speed = np.linalg.norm(v)
    if speed < 1e-3:
        return np.zeros(3)
    mach = speed / a_sound
    cd   = get_CD(np.clip(mach, 0.8, 3.0), 0.0)
    if np.isnan(cd): cd = 0.3
    rho  = rho0 * np.exp(-alt / 8500.0)
    F_d  = 0.5 * rho * speed**2 * A_ref * cd
    return -F_d * v / speed

# --- Truth simulation ---
dt    = 0.01
t_end = 60.0
ts    = np.arange(0, t_end, dt)
n     = len(ts)
angle = np.radians(45.0)
state = np.zeros((n, 6))

for i in range(1, n):
    t  = ts[i-1]
    x, y, z, vx, vy, vz = state[i-1]
    v   = np.array([vx, vy, vz])
    alt = max(z, 0.0)
    T   = thrust(t)
    speed = np.linalg.norm(v)
    if speed < 1e-3:
        Tv = np.array([np.cos(angle)*T, 0, np.sin(angle)*T])
    else:
        Tv = T * v / speed
    Fd = drag(v, alt)
    ax = (Tv[0] + Fd[0]) / m
    ay = (Tv[1] + Fd[1]) / m
    az = (Tv[2] + Fd[2]) / m - g
    state[i] = [x+vx*dt, y+vy*dt, z+vz*dt+0.5*az*dt**2,
                vx+ax*dt, vy+ay*dt, vz+az*dt]
    if state[i, 2] < 0 and i > 10:
        state = state[:i]
        ts    = ts[:i]
        n     = i
        break

# --- Noise ---
np.random.seed(42)
sigma_a   = 0.5
sigma_gps = 5.0
sigma_v   = 0.1
accel_meas = np.diff(state[:,3:6], axis=0)/dt + np.random.randn(n-1,3)*sigma_a
gps_pos    = state[:,0:3] + np.random.randn(n,3)*sigma_gps
gps_vel    = state[:,3:6] + np.random.randn(n,3)*sigma_v

# --- EKF ---
x_est = np.zeros((n, 6))
P     = np.eye(6) * 100.0
Q     = np.eye(6)
Q[0:3,0:3] *= 1.0
Q[3:6,3:6] *= 10.0
R_gps = np.eye(6)
R_gps[0:3,0:3] *= sigma_gps**2
R_gps[3:6,3:6] *= sigma_v**2
H     = np.eye(6)
x_est[0] = state[0] + np.random.randn(6)*[10,10,10,1,1,1]

for i in range(1, n):
    x_p = x_est[i-1].copy()
    a   = accel_meas[i-1] if i-1 < len(accel_meas) else np.zeros(3)
    x_p[0:3] += x_p[3:6]*dt
    x_p[3]   += a[0]*dt
    x_p[4]   += a[1]*dt
    x_p[5]   += (a[2] - g)*dt
    F    = np.eye(6)
    F[0,3] = dt; F[1,4] = dt; F[2,5] = dt
    P    = F @ P @ F.T + Q*dt
    if i % 10 == 0:
        z_meas = np.concatenate([gps_pos[i], gps_vel[i]])
        y      = z_meas - H @ x_p
        S      = H @ P @ H.T + R_gps
        K      = P @ H.T @ np.linalg.inv(S)
        x_p    = x_p + K @ y
        P      = (np.eye(6) - K @ H) @ P
    x_est[i] = x_p

# --- Plot ---
if __name__ == "__main__":
    import os
    os.makedirs("D:/Weapons-systems-sim-pipeline/05_kalman_filter/results/figures", exist_ok=True)

    fig, axes = plt.subplots(3, 2, figsize=(12, 12))
    fig.suptitle("6-State EKF -- Boost-Sustain Missile Trajectory", fontsize=13)
    labels     = ["X (m)", "Z (m)", "Vx (m/s)", "Vz (m/s)", "X Error (m)", "Z Error (m)"]
    state_idx  = [0, 2, 3, 5, None, None]
    gps_idx    = [0, 2, None, None, None, None]

    for plot_i, ax in enumerate(axes.flat):
        if plot_i == 4:
            err = x_est[:,0] - state[:,0]
            ax.plot(ts, err, "g-", lw=1.0)
            ax.axhline(0, color="k", lw=0.8, ls="--")
            ax.fill_between(ts, err, 0, alpha=0.2, color="green")
        elif plot_i == 5:
            err = x_est[:,2] - state[:,2]
            ax.plot(ts, err, "g-", lw=1.0)
            ax.axhline(0, color="k", lw=0.8, ls="--")
            ax.fill_between(ts, err, 0, alpha=0.2, color="green")
        else:
            si = state_idx[plot_i]
            ax.plot(ts, state[:,si], "k-", lw=1.5, label="Truth")
            ax.plot(ts, x_est[:,si], "b--", lw=1.0, label="EKF")
            if gps_idx[plot_i] is not None:
                ax.plot(ts, gps_pos[:,gps_idx[plot_i]], "r.", ms=1, alpha=0.3, label="GPS")
            ax.legend(fontsize=7)
        ax.set_ylabel(labels[plot_i])
        ax.set_xlabel("Time (s)")
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig("D:/Weapons-systems-sim-pipeline/05_kalman_filter/results/figures/ekf_trajectory.png",
                dpi=150, bbox_inches="tight")
    print("Plot saved")
    plt.show()

    pos_rmse = np.sqrt(np.mean((x_est[:,[0,2]] - state[:,[0,2]])**2))
    vel_rmse = np.sqrt(np.mean((x_est[:,[3,5]] - state[:,[3,5]])**2))
    print(f"Position RMSE: {pos_rmse:.2f} m")
    print(f"Velocity RMSE: {vel_rmse:.2f} m/s")

def run_ekf(seed=42, Q_scale=1.0, R_scale=1.0):
    """Callable interface for testing. Returns (state, ts, x_est)."""
    np.random.seed(seed)

    _ts    = np.arange(0, 60.0, dt)
    _n     = len(_ts)
    _state = np.zeros((_n, 6))

    for i in range(1, _n):
        t  = _ts[i-1]
        x, y, z, vx, vy, vz = _state[i-1]
        v   = np.array([vx, vy, vz])
        alt = max(z, 0.0)
        T   = thrust(t)
        speed = np.linalg.norm(v)
        if speed < 1e-3:
            Tv = np.array([np.cos(angle)*T, 0, np.sin(angle)*T])
        else:
            Tv = T * v / speed
        Fd  = drag(v, alt)
        ax_ = (Tv[0] + Fd[0]) / m
        ay_ = (Tv[1] + Fd[1]) / m
        az_ = (Tv[2] + Fd[2]) / m - g
        _state[i] = [x+vx*dt, y+vy*dt, z+vz*dt+0.5*az_*dt**2,
                     vx+ax_*dt, vy+ay_*dt, vz+az_*dt]
        if _state[i, 2] < 0 and i > 10:
            _state = _state[:i]
            _ts    = _ts[:i]
            _n     = i
            break

    _accel = np.diff(_state[:,3:6], axis=0)/dt + np.random.randn(_n-1,3)*sigma_a
    _gps_p = _state[:,0:3] + np.random.randn(_n,3)*sigma_gps
    _gps_v = _state[:,3:6] + np.random.randn(_n,3)*sigma_v

    _Q = Q * Q_scale
    _R = R_gps * R_scale
    _x = np.zeros((_n, 6))
    _P = np.eye(6) * 100.0
    _x[0] = _state[0] + np.random.randn(6)*[10,10,10,1,1,1]

    for i in range(1, _n):
        xp = _x[i-1].copy()
        a  = _accel[i-1] if i-1 < len(_accel) else np.zeros(3)
        xp[0:3] += xp[3:6]*dt
        xp[3]   += a[0]*dt
        xp[4]   += a[1]*dt
        xp[5]   += (a[2] - g)*dt
        F_      = np.eye(6)
        F_[0,3] = dt; F_[1,4] = dt; F_[2,5] = dt
        _P      = F_ @ _P @ F_.T + _Q*dt
        if i % 10 == 0:
            zm = np.concatenate([_gps_p[i], _gps_v[i]])
            y_ = zm - H @ xp
            S_ = H @ _P @ H.T + _R
            K_ = _P @ H.T @ np.linalg.inv(S_)
            xp = xp + K_ @ y_
            _P = (np.eye(6) - K_ @ H) @ _P
        _x[i] = xp

    return _state, _ts, _x