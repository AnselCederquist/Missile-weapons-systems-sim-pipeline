"""
engagement.py -- Single engagement simulation
Project 08 -- GNC/Monte Carlo

Point mass missile model with True PN guidance homing on
a maneuvering target. Integrates missile and target states
simultaneously, detects closest point of approach (CPA),
and returns full engagement history for analysis and animation.

Missile model: 6-state point mass with thrust, drag, gravity,
and lateral acceleration commands from guidance law.
Propulsion: boost-sustain from Project 06 parameters.
Drag: from Project 04 aero_interpolator.
"""

import numpy as np
import sys
import os
sys.path.append("D:/Weapons-systems-sim-pipeline/04_missile_aero_database/postprocess")
sys.path.append("D:/Weapons-systems-sim-pipeline/08_gnc_monte_carlo/src")

from aero_interpolator import get_CD
from target import ManeuveringTarget
from guidance import GuidanceLaw

# ---------------------------------------------------------------
# Constants
# ---------------------------------------------------------------
g        = 9.81
rho0     = 1.225
a_sound  = 343.0
A_ref    = 0.00785

# ---------------------------------------------------------------
# Propulsion (from Project 06)
# ---------------------------------------------------------------
m_launch  = 20.0
m_burnout = 15.0
t_boost   = 2.0
t_sustain = 15.0
T_boost   = 3000.0
T_sustain = 800.0
mdot      = (m_launch - m_burnout) / (t_sustain - t_boost)

def thrust(t):
    if t < t_boost:
        return T_boost
    elif t < t_sustain:
        return T_sustain
    return 0.0

def mass(t):
    if t < t_boost:
        return m_launch
    elif t < t_sustain:
        return max(m_launch - mdot * (t - t_boost), m_burnout)
    return m_burnout

def atmosphere(alt):
    return rho0 * np.exp(-max(alt, 0.0) / 8500.0)

# ---------------------------------------------------------------
# Missile point mass EOM
# ---------------------------------------------------------------
def missile_derivatives(t, state, a_lateral):
    """
    Point mass missile EOM with thrust, drag, gravity,
    and lateral acceleration command.

    State: [x, y, z, vx, vy, vz]
    """
    x, y, z, vx, vy, vz = state
    v = np.array([vx, vy, vz])
    speed = np.linalg.norm(v)

    m = mass(t)
    T = thrust(t)

    # Thrust along velocity vector
    if speed > 1e-3:
        v_hat = v / speed
    else:
        v_hat = np.array([1.0, 0.0, 0.0])

    F_thrust = T * v_hat

    # Drag
    mach = np.clip(speed / a_sound, 0.8, 3.0)
    rho  = atmosphere(z)
    qbar = 0.5 * rho * speed**2
    CD   = get_CD(mach, 0.0)
    if np.isnan(CD):
        CD = 0.3
    F_drag = -qbar * A_ref * CD * v_hat

    # Gravity
    F_grav = np.array([0.0, 0.0, -m * g])

    # Lateral acceleration (perpendicular to velocity)
    # Project a_lateral onto plane perpendicular to velocity
    # Full 3D acceleration command from guidance
    a_lat = np.array(a_lateral)

    # Total acceleration
    a = (F_thrust + F_drag + F_grav) / m + a_lat

    return np.array([vx, vy, vz, a[0], a[1], a[2]])


def rk4_step(f, t, s, dt, *args):
    k1 = f(t,        s, *args)
    k2 = f(t+dt/2,   s+k1*dt/2, *args)
    k3 = f(t+dt/2,   s+k2*dt/2, *args)
    k4 = f(t+dt,     s+k3*dt,   *args)
    return s + (dt/6) * (k1 + 2*k2 + 2*k3 + k4)


# ---------------------------------------------------------------
# Closest Point of Approach detection
# ---------------------------------------------------------------
def detect_cpa(r_m, r_t, r_m_prev, r_t_prev):
    """
    Detect if closest point of approach has been passed.
    Returns True if range is increasing after having decreased.
    """
    R_now  = np.linalg.norm(r_t - r_m)
    R_prev = np.linalg.norm(r_t_prev - r_m_prev)
    return R_now > R_prev


# ---------------------------------------------------------------
# Single engagement
# ---------------------------------------------------------------
def run_engagement(
        launch_pos=None,
        launch_angle_deg=20.0,
        launch_azimuth_deg=0.0,
        target_pos=None,
        target_heading_deg=90.0,
        target_speed=50.0,
        target_max_accel_g=3,
        guidance_variant='TPN',
        N=4.0,
        noise_std=0.05,
        accel_limit_g=10.0,
        dt=0.01,
        t_max=60.0,
        lethal_radius=20.0,
        wind=None,
        seed=None,
        verbose=False):
    """
    Run a single missile-target engagement.

    Parameters
    ----------
    launch_pos : array [3] missile launch position (m)
    launch_angle_deg : elevation angle at launch (deg)
    launch_azimuth_deg : azimuth angle at launch (deg)
    target_pos : array [3] target initial position (m)
    target_heading_deg : target initial heading (deg)
    target_speed : target speed (m/s)
    target_max_accel_g : target max lateral accel (g)
    guidance_variant : 'PPN', 'TPN', or 'APN'
    N : navigation constant
    noise_std : seeker noise std (rad/s)
    accel_limit_g : missile accel limit (g)
    dt : integration timestep (s)
    t_max : max engagement time (s)
    lethal_radius : kill radius (m)
    wind : array [3] wind velocity (m/s)
    seed : random seed

    Returns
    -------
    result : dict with keys:
        miss_distance : float (m)
        hit : bool
        t_cpa : float time of CPA (s)
        missile_history : array [N, 6]
        target_history : array [N, 6]
        time_history : array [N]
        guidance_history : array [N, 3]
        seeker_valid_history : array [N] bool
        cpa_idx : int index of CPA
    """
    rng = np.random.default_rng(seed)

    # Default positions
    if launch_pos is None:
        launch_pos = np.array([0.0, 0.0, 10.0])
    else:
        launch_pos = np.array(launch_pos, dtype=float)

    if target_pos is None:
        target_pos = np.array([4000.0, 500.0, 500.0])
    else:
        target_pos = np.array(target_pos, dtype=float)

    if wind is None:
        wind = np.zeros(3)
    else:
        wind = np.array(wind, dtype=float)

   # Initial missile state
    el  = np.radians(launch_angle_deg)
    az  = np.radians(launch_azimuth_deg)
    v0  = 50.0
    # Blend launch angle with LOS direction to reduce initial guidance transient
    los_init     = np.array(target_pos, dtype=float) - launch_pos
    los_init_hat = los_init / np.linalg.norm(los_init)
    v_launch     = np.array([np.cos(el)*np.cos(az),
                              np.cos(el)*np.sin(az),
                              np.sin(el)])
    v_init = los_init_hat
    vx0 = v0 * v_init[0]
    vy0 = v0 * v_init[1]
    vz0 = v0 * v_init[2]

    missile_state = np.array([
        launch_pos[0], launch_pos[1], launch_pos[2],
        vx0, vy0, vz0
    ], dtype=float)

    # Target
    target = ManeuveringTarget(
        pos0=target_pos,
        heading_deg=target_heading_deg,
        speed=target_speed,
        altitude=target_pos[2],
        max_accel=target_max_accel_g * g,
        seed=rng.integers(0, 100000)
    )

    # Guidance
    guidance = GuidanceLaw(
        variant=guidance_variant,
        N=N,
        noise_std=noise_std,
        accel_limit=accel_limit_g * g,
        tau=0.1,
        seed=rng.integers(0, 100000)
    )

    # History
    missile_history  = [missile_state.copy()]
    target_history   = [target.state.copy()]
    guidance_history = [np.zeros(3)]
    seeker_valid_history = [True]
    time_history     = [0.0]

    t = 0.0
    a_lat = np.zeros(3)
    cpa_idx = 0
    min_range = np.inf
    cpa_found = False

    prev_missile_pos = missile_state[0:3].copy()
    prev_target_pos  = target.pos.copy()

    # ── Main integration loop ──
    while t < t_max:
        r_m = missile_state[0:3]
        v_m = missile_state[3:6]
        r_t = target.pos.copy()
        v_t = target.vel.copy()

        # Apply wind to effective missile velocity
        v_m_eff = v_m + wind

        # Guidance command
        a_t_est = target.accel_cmd if hasattr(target, 'accel_cmd') else np.zeros(3)
        a_lat, seeker_valid = guidance.compute(
            r_m, r_t, v_m_eff, v_t,
            a_t=a_t_est,
            dt=dt
        )

        # Integrate missile
        missile_state = rk4_step(
            missile_derivatives, t, missile_state, dt, a_lat
        )

        # Integrate target
        target.step(dt)

        t += dt

        if verbose and abs(t % 1.0) < dt:
            alt_err = target.pos[2] - missile_state[2]
            print(f"  t={t:.1f}s R={R:.0f}m "
                f"speed={np.linalg.norm(missile_state[3:6]):.0f}m/s "
                f"seeker={'OK' if seeker_valid else 'LOST'} "
                f"a_lat={np.linalg.norm(a_lat):.1f}m/s2 "
                f"a_z={a_lat[2]:.1f} "
                f"alt={missile_state[2]:.0f}m "
                f"alt_err={alt_err:.0f}m")

        # Range check
        R = np.linalg.norm(target.pos - missile_state[0:3])

        # CPA detection
        if R < min_range:
            min_range = R
            cpa_idx = len(time_history) - 1
        elif not cpa_found and R > min_range + 50.0:
            cpa_found = True

        # Hit detection
        if R < lethal_radius:
            missile_history.append(missile_state.copy())
            target_history.append(target.state.copy())
            guidance_history.append(a_lat.copy())
            seeker_valid_history.append(seeker_valid)
            time_history.append(t)
            break

        # Ground impact
        if missile_state[2] < 0:
            break

       # Terminate when seeker lost and range increasing (post-CPA)
        if not seeker_valid and cpa_found:
            break

        # Store history
        missile_history.append(missile_state.copy())
        target_history.append(target.state.copy())
        guidance_history.append(a_lat.copy())
        seeker_valid_history.append(seeker_valid)
        time_history.append(t)

        prev_missile_pos = missile_state[0:3].copy()
        prev_target_pos  = target.pos.copy()

    # Convert to arrays
    missile_history  = np.array(missile_history)
    target_history   = np.array(target_history)
    guidance_history = np.array(guidance_history)
    time_history     = np.array(time_history)
    seeker_valid_history = np.array(seeker_valid_history)

    miss_distance = min_range
    hit = miss_distance < lethal_radius

    return {
        'miss_distance':       miss_distance,
        'hit':                 hit,
        't_cpa':               time_history[min(cpa_idx, len(time_history)-1)],
        'missile_history':     missile_history,
        'target_history':      target_history,
        'guidance_history':    guidance_history,
        'seeker_valid_history': seeker_valid_history,
        'time_history':        time_history,
        'cpa_idx':             cpa_idx,
        'min_range':           min_range,
    }


# ---------------------------------------------------------------
# Animated engagement plot
# ---------------------------------------------------------------
def animate_engagement(result, save_path=None, interval=20):
    """
    Animate a single engagement in 3D.
    Shows missile trajectory, target trajectory, and LOS vector.

    Parameters
    ----------
    result : dict from run_engagement()
    save_path : str path to save .gif (None = display only)
    interval : int animation frame interval (ms)
    """
    import matplotlib.pyplot as plt
    import matplotlib.animation as animation
    from mpl_toolkits.mplot3d import Axes3D

    mh = result['missile_history']
    th = result['target_history']
    ts = result['time_history']
    cpa = result['cpa_idx']

    # Downsample for animation speed
    step = max(1, len(ts) // 200)
    idx  = np.arange(0, len(ts), step)

    fig = plt.figure(figsize=(12, 8))
    ax  = fig.add_subplot(111, projection='3d')

    # Static full trajectories (faded)
    ax.plot(mh[:,0], mh[:,1], mh[:,2], 'b-', alpha=0.15, lw=1, label='_nolegend_')
    ax.plot(th[:,0], th[:,1], th[:,2], 'r-', alpha=0.15, lw=1, label='_nolegend_')

    # Animated elements
    missile_trail, = ax.plot([], [], [], 'b-', lw=1.5, label='Missile')
    target_trail,  = ax.plot([], [], [], 'r-', lw=1.5, label='Target')
    missile_dot,   = ax.plot([], [], [], 'bo', ms=6)
    target_dot,    = ax.plot([], [], [], 'rs', ms=6)
    los_line,      = ax.plot([], [], [], 'g--', lw=0.8, alpha=0.6, label='LOS')
    time_text      = ax.text2D(0.02, 0.95, '', transform=ax.transAxes, fontsize=9)
    range_text     = ax.text2D(0.02, 0.90, '', transform=ax.transAxes, fontsize=9)

    # CPA marker
    ax.plot([mh[cpa,0]], [mh[cpa,1]], [mh[cpa,2]],
            'b*', ms=12, label=f'CPA missile')
    ax.plot([th[min(cpa, len(th)-1),0]],
            [th[min(cpa, len(th)-1),1]],
            [th[min(cpa, len(th)-1),2]],
            'r*', ms=12, label=f'CPA target')

    # Launch marker
    ax.plot([mh[0,0]], [mh[0,1]], [mh[0,2]], 'g^', ms=10, label='Launch')

    ax.set_xlabel('Downrange X (m)')
    ax.set_ylabel('Lateral Y (m)')
    ax.set_zlabel('Altitude Z (m)')
    ax.set_title(f'Missile Engagement — Miss distance: {result["miss_distance"]:.1f}m '
                 f'{"HIT" if result["hit"] else "MISS"}')
    ax.legend(loc='upper right', fontsize=8)

    def init():
        missile_trail.set_data([], [])
        missile_trail.set_3d_properties([])
        target_trail.set_data([], [])
        target_trail.set_3d_properties([])
        missile_dot.set_data([], [])
        missile_dot.set_3d_properties([])
        target_dot.set_data([], [])
        target_dot.set_3d_properties([])
        los_line.set_data([], [])
        los_line.set_3d_properties([])
        return missile_trail, target_trail, missile_dot, target_dot, los_line

    def update(frame):
        i = idx[frame]
        trail_start = max(0, frame - 50)

        # Trails
        trail_idx = idx[trail_start:frame+1]
        missile_trail.set_data(mh[trail_idx,0], mh[trail_idx,1])
        missile_trail.set_3d_properties(mh[trail_idx,2])
        t_trail = th[np.minimum(trail_idx, len(th)-1)]
        target_trail.set_data(t_trail[:,0], t_trail[:,1])
        target_trail.set_3d_properties(t_trail[:,2])

        # Current positions
        missile_dot.set_data([mh[i,0]], [mh[i,1]])
        missile_dot.set_3d_properties([mh[i,2]])
        ti = min(i, len(th)-1)
        target_dot.set_data([th[ti,0]], [th[ti,1]])
        target_dot.set_3d_properties([th[ti,2]])

        # LOS line
        los_line.set_data([mh[i,0], th[ti,0]], [mh[i,1], th[ti,1]])
        los_line.set_3d_properties([mh[i,2], th[ti,2]])

        # Text
        R = np.linalg.norm(th[ti,:3] - mh[i,:3])
        time_text.set_text(f't = {ts[i]:.1f}s')
        range_text.set_text(f'R = {R:.0f}m')

        return missile_trail, target_trail, missile_dot, target_dot, los_line, time_text, range_text

    anim = animation.FuncAnimation(
        fig, update, frames=len(idx),
        init_func=init, interval=interval, blit=False
    )

    if save_path:
        os.makedirs(os.path.dirname(save_path), exist_ok=True)
        writer = animation.PillowWriter(fps=30)
        anim.save(save_path, writer=writer)
        print(f"Animation saved: {save_path}")
    else:
        plt.show()

    plt.close()
    return anim


# ---------------------------------------------------------------
# Static engagement plot
# ---------------------------------------------------------------
def plot_engagement(result, save_path=None):
    """
    Static 3D plot of a single engagement with annotations.
    """
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D

    mh  = result['missile_history']
    th  = result['target_history']
    ts  = result['time_history']
    cpa = result['cpa_idx']

    fig = plt.figure(figsize=(12, 8))
    ax  = fig.add_subplot(111, projection='3d')

    ax.plot(mh[:,0], mh[:,1], mh[:,2], 'b-', lw=2, label='Missile')
    ax.plot(th[:,0], th[:,1], th[:,2], 'r-', lw=2, label='Target')

    # CPA
    ax.plot([mh[cpa,0]], [mh[cpa,1]], [mh[cpa,2]], 'b*', ms=14)
    ti = min(cpa, len(th)-1)
    ax.plot([th[ti,0]], [th[ti,1]], [th[ti,2]], 'r*', ms=14)

    # LOS at CPA
    ax.plot([mh[cpa,0], th[ti,0]],
            [mh[cpa,1], th[ti,1]],
            [mh[cpa,2], th[ti,2]],
            'g--', lw=1.5, label=f'Miss distance: {result["miss_distance"]:.1f}m')

    # Launch and impact
    ax.plot([mh[0,0]], [mh[0,1]], [mh[0,2]], 'g^', ms=12, label='Launch')
    ax.plot([th[0,0]], [th[0,1]], [th[0,2]], 'rs', ms=10, label='Target start')

    ax.set_xlabel('Downrange X (m)')
    ax.set_ylabel('Lateral Y (m)')
    ax.set_zlabel('Altitude Z (m)')
    ax.set_title(f'Single Engagement — Miss: {result["miss_distance"]:.1f}m '
                 f'{"HIT" if result["hit"] else "MISS"} | '
                 f't_cpa={result["t_cpa"]:.1f}s')
    ax.legend(fontsize=9)
    plt.tight_layout()

    if save_path:
        os.makedirs(os.path.dirname(save_path), exist_ok=True)
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"Plot saved: {save_path}")

    plt.show()
    plt.close()


# ---------------------------------------------------------------
# Main — run and plot a single engagement
# ---------------------------------------------------------------
if __name__ == "__main__":
    print("Running single engagement...")

    result = run_engagement(
        launch_pos=[0, 0, 10.0],
        launch_angle_deg=20.0,
        launch_azimuth_deg=4.3,
        target_pos=[4000.0, 300.0, 500.0],
        target_heading_deg=200.0,
        target_speed=50.0,
        target_max_accel_g=1.5,
        guidance_variant='TPN',
        N=4.0,
        noise_std=0.05,
        dt=0.01,
        t_max=60.0,
        lethal_radius=20.0,
        seed=42,
        verbose=True
    )

    print(f"\n=== Engagement Results ===")
    print(f"  Miss distance:  {result['miss_distance']:.2f} m")
    print(f"  Hit:            {result['hit']}")
    print(f"  Time to CPA:    {result['t_cpa']:.1f} s")
    print(f"  Flight time:    {result['time_history'][-1]:.1f} s")


    os.makedirs("D:/Weapons-systems-sim-pipeline/08_gnc_monte_carlo/results/figures",
                exist_ok=True)

    plot_engagement(
        result,
        save_path="D:/Weapons-systems-sim-pipeline/08_gnc_monte_carlo/results/figures/single_engagement.png"
    )

    print("Saving animation (this may take a minute)...")
    animate_engagement(
        result,
        save_path="D:/Weapons-systems-sim-pipeline/08_gnc_monte_carlo/results/figures/single_engagement.gif"
    )