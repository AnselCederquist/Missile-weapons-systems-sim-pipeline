"""
attitude_sim.py -- 13-State Attitude Control Simulator
Project 09 -- Attitude Control Simulator

Extends Project 06 (6-DOF) with:
  - Quaternion attitude representation (replaces Euler angles)
  - Fin deflection model (actuator.py)
  - LQR attitude controller with dynamic pressure gain scheduling (lqr.py)
  - Guidance interface from Project 08 TPN  -OR-  Optimal Guidance Law (OGL)

State vector (13 states):
  [0:3]   x, y, z         position (m, inertial frame)
  [3:6]   vx, vy, vz      velocity (m/s, inertial frame)
  [6:10]  q0, q1, q2, q3  quaternion (body-to-inertial, [w,x,y,z])
  [10:13] p, q_rate, r    angular rates (rad/s, body frame)

Guidance options (set GUIDANCE_MODE at top of run_attitude_sim):
  'TPN'  -- True Proportional Navigation from Project 08
  'OGL'  -- Optimal Guidance Law (zero-effort miss / t_go^2)
             Directly addresses TPN kinematic floor at high closing velocity.
"""

import sys
import os
import numpy as np
import matplotlib.pyplot as plt

print(sys.path)

sys.path.insert(0, os.path.join(os.path.dirname(__file__),
    '..', '..', '04_missile_aero_database', 'postprocess'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__),
    '..', '..', '08_gnc_monte_carlo', 'src'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__),
    '..', '..', '09_attitude_control', 'src'))

try:
    from aero_interpolator import AeroInterpolator
    aero = AeroInterpolator()
    USE_DATCOM = True
except ImportError:
    USE_DATCOM = False

try:
    from guidance import GuidanceLaw
    from target import ManeuveringTarget
    USE_GUIDANCE = True
except ImportError:
    USE_GUIDANCE = False

from quaternion import (
    quat_norm, quat_to_rotmat,
    quat_kinematics, quat_to_euler, euler_to_quat
)
from actuator import (
    FinActuator, compute_moments, compute_angular_acceleration,
    compute_aoa_sideslip, S_REF, C_REF
)
from lqr import LQRController


# ---------------------------------------------------------------
# Missile parameters
# ---------------------------------------------------------------
MASS_LAUNCH          = 20.0
MASS_BURNOUT         = 15.0
MASS_PROP            = MASS_LAUNCH - MASS_BURNOUT
THRUST_BOOST         = 3000.0
THRUST_SUSTAIN       = 800.0
T_BOOST_END          = 2.0
T_SUSTAIN_END        = 15.0
G                    = 9.81
RHO0                 = 1.225
H_SCALE              = 8500.0
SPEED_OF_SOUND       = 343.0
CD0                  = 0.3
CN_ALPHA             = 4.0
ALPHA_CLAMP_DEG      = 8.0
BETA_CLAMP_DEG       = 8.0
AERO_ALPHA_CLAMP_DEG = 10.0
GRAV_COMP_BELOW      = 1.0
GRAV_COMP_ABOVE      = 0.0
TAU_DES       = 0.2
TPN_N         = 4.0
TPN_ACCEL_LIM = 10.0
TPN_NOISE_STD = 0.0

# OGL parameters
OGL_N           = 6.0    # navigation constant (higher = more aggressive)
OGL_T_GO_MIN    = 0.15   # minimum t_go (s) — prevents gain singularity at CPA
OGL_ACCEL_LIMIT = 40.0   # m/s^2 — acceleration command limit


def atmosphere(alt):
    alt = max(0.0, alt)
    return RHO0 * np.exp(-alt / H_SCALE)


def get_cd(mach, alpha_deg):
    if USE_DATCOM:
        try:
            return float(aero.get_CD(mach, alpha_deg))
        except Exception:
            pass
    return CD0


def propulsion(t):
    if t < T_BOOST_END:
        return THRUST_BOOST, MASS_PROP / T_SUSTAIN_END
    elif t < T_SUSTAIN_END:
        return THRUST_SUSTAIN, MASS_PROP / T_SUSTAIN_END
    return 0.0, 0.0


def missile_mass(t):
    _, mdot = propulsion(t)
    return max(MASS_LAUNCH - mdot * min(t, T_SUSTAIN_END), MASS_BURNOUT)


# ---------------------------------------------------------------
# Optimal Guidance Law
# ---------------------------------------------------------------

def compute_ogl(pos, r_t, vel_m, vel_t, N=OGL_N, t_go_min=OGL_T_GO_MIN):
    """
    Optimal Guidance Law (OGL) — zero-effort miss formulation.

    Command: a_cmd = (N / t_go^2) * ZEM
    where:   ZEM   = r_rel + t_go * v_rel   (zero-effort miss vector)
             t_go  = R / V_closing           (time-to-go estimate)

    Unlike TPN, command magnitude grows as t_go shrinks — automatically
    aggressive in the terminal phase. This directly addresses the kinematic
    floor observed with TPN at high closing velocity.

    Parameters
    ----------
    pos   : missile position (m, inertial)
    r_t   : target position (m, inertial)
    vel_m : missile velocity (m/s, inertial)
    vel_t : target velocity (m/s, inertial)
    N     : navigation constant (default OGL_N)
    t_go_min : minimum t_go to prevent singularity (s)

    Returns
    -------
    a_cmd : acceleration command (m/s^2, inertial)
    valid : bool
    """
    r_rel = r_t - pos
    v_rel = vel_t - vel_m
    R     = np.linalg.norm(r_rel)

    if R < 1.0:
        return np.zeros(3), False

    r_hat = r_rel / R
    V_c   = -np.dot(v_rel, r_hat)   # closing velocity (positive when closing)

    if V_c < 1.0:
        return np.zeros(3), False

    t_go  = max(R / V_c, t_go_min)
    ZEM   = r_rel + t_go * v_rel    # zero-effort miss vector

    a_cmd = (N / t_go**2) * ZEM

    # Clamp magnitude
    a_mag = np.linalg.norm(a_cmd)
    if a_mag > OGL_ACCEL_LIMIT:
        a_cmd = a_cmd * OGL_ACCEL_LIMIT / a_mag

    return a_cmd, True


# ---------------------------------------------------------------
# Equations of motion
# ---------------------------------------------------------------

def derivatives(t, state, fin_deflections):
    pos   = state[0:3]
    vel_i = state[3:6]
    q     = quat_norm(state[6:10])
    omega = state[10:13]

    mass      = missile_mass(t)
    thrust, _ = propulsion(t)
    R_bi      = quat_to_rotmat(q)
    vel_b     = R_bi.T @ vel_i
    V         = max(np.linalg.norm(vel_b), 1.0)

    alt   = pos[2]
    rho   = atmosphere(alt)
    q_bar = 0.5 * rho * V**2
    mach  = V / SPEED_OF_SOUND

    alpha, beta = compute_aoa_sideslip(vel_b)
    cd = get_cd(mach, np.degrees(alpha))

    vel_hat_i  = vel_i / max(np.linalg.norm(vel_i), 0.1)
    F_drag_i   = -cd * q_bar * S_REF * vel_hat_i
    F_grav_i   = np.array([0.0, 0.0, -mass * G])
    F_thrust_i = R_bi @ np.array([thrust, 0.0, 0.0])

    alpha_f = np.clip(alpha, -np.radians(AERO_ALPHA_CLAMP_DEG), np.radians(AERO_ALPHA_CLAMP_DEG))
    beta_f  = np.clip(beta,  -np.radians(AERO_ALPHA_CLAMP_DEG), np.radians(AERO_ALPHA_CLAMP_DEG))
    F_normal_b = np.array([0.0,
                            -q_bar * S_REF * CN_ALPHA * beta_f,
                             q_bar * S_REF * CN_ALPHA * alpha_f])
    if V > 1.0:
        v_hat_b = vel_b / V
        F_normal_b -= np.dot(F_normal_b, v_hat_b) * v_hat_b
    F_normal_i = R_bi @ F_normal_b

    cd_induced = 0.1 * (alpha_f**2 + beta_f**2)
    F_drag_i   = F_drag_i * (1.0 + cd_induced / max(cd, 0.01))

    acc_i = (F_thrust_i + F_drag_i + F_grav_i + F_normal_i) / mass

    p_r, q_r, r_r = omega
    moments   = compute_moments(fin_deflections, alpha, beta, p_r, q_r, r_r, q_bar, V)
    omega_dot = compute_angular_acceleration(moments, p_r, q_r, r_r)
    dq        = quat_kinematics(q, omega)

    state_dot = np.zeros(13)
    state_dot[0:3]   = vel_i
    state_dot[3:6]   = acc_i
    state_dot[6:10]  = dq
    state_dot[10:13] = omega_dot
    return state_dot


def rk4_step(t, state, fin_deflections, dt):
    k1 = derivatives(t,        state,            fin_deflections)
    k2 = derivatives(t + dt/2, state + dt/2*k1,  fin_deflections)
    k3 = derivatives(t + dt/2, state + dt/2*k2,  fin_deflections)
    k4 = derivatives(t + dt,   state + dt*k3,    fin_deflections)
    ns = state + (dt/6) * (k1 + 2*k2 + 2*k3 + k4)
    ns[6:10]  = quat_norm(ns[6:10])
    ns[10:13] = np.clip(ns[10:13], -np.radians(200), np.radians(200))
    return ns


# ---------------------------------------------------------------
# Guidance interface
# ---------------------------------------------------------------

def flight_path_attitude(vel_i):
    V = np.linalg.norm(vel_i)
    if V < 1.0:
        return np.zeros(3)
    v = vel_i / V
    return np.array([0.0,
                     np.arctan2(v[2], np.sqrt(v[0]**2 + v[1]**2)),
                     np.arctan2(v[1], v[0])])


def guidance_to_attitude(vel_i, a_cmd, t, pos, q_current=None,
                         target_alt=500.0, r_t=None):
    V = np.linalg.norm(vel_i)
    if V < 10.0:
        return flight_path_attitude(vel_i)

    v         = vel_i / V
    theta_fpa = np.arctan2(v[2], np.sqrt(v[0]**2 + v[1]**2))
    psi_fpa   = np.arctan2(v[1], v[0])

    rho   = atmosphere(pos[2])
    q_bar = 0.5 * rho * V**2
    mass  = missile_mass(t)
    denom = q_bar * S_REF * CN_ALPHA
    if denom < 10.0:
        return np.array([0.0, theta_fpa, psi_fpa])

    if q_current is not None:
        R_bi = quat_to_rotmat(quat_norm(q_current))
        a_b  = R_bi.T @ a_cmd
    else:
        a_b = a_cmd.copy()

    grav_comp = G * GRAV_COMP_BELOW if pos[2] < target_alt else G * GRAV_COMP_ABOVE
    a_cmd_z   = a_b[2] + grav_comp

    r_norm_now = np.linalg.norm(r_t - pos) if r_t is not None else 999.0
    clamp_deg  = 12.0 if r_norm_now < 300.0 else ALPHA_CLAMP_DEG
    alpha_cmd  = np.clip(mass * a_cmd_z / denom,
                         -np.radians(clamp_deg), np.radians(clamp_deg))
    beta_cmd   = np.clip(mass * a_b[1]  / denom,
                         -np.radians(clamp_deg), np.radians(clamp_deg))

    # Terminal blend: within 500m mix in direct LOS correction
    if r_t is not None:
        r_vec  = r_t - pos
        r_norm = np.linalg.norm(r_vec)
        if r_norm < 500.0:
            blend     = (500.0 - r_norm) / 500.0
            los_theta = np.arctan2(r_vec[2], np.sqrt(r_vec[0]**2 + r_vec[1]**2))
            los_psi   = np.arctan2(r_vec[1], r_vec[0])
            alpha_los = np.clip(los_theta - theta_fpa,
                                -np.radians(ALPHA_CLAMP_DEG), np.radians(ALPHA_CLAMP_DEG))
            beta_los  = np.clip(-(los_psi - psi_fpa),
                                -np.radians(BETA_CLAMP_DEG),  np.radians(BETA_CLAMP_DEG))
            alpha_cmd = alpha_cmd * (1 - blend) + alpha_los * blend
            beta_cmd  = beta_cmd  * (1 - blend) + beta_los  * blend

    return np.array([0.0, theta_fpa + alpha_cmd, psi_fpa - beta_cmd])


# ---------------------------------------------------------------
# Main simulation
# ---------------------------------------------------------------

def run_attitude_sim(
    launch_angle_deg = 21.2,
    target_pos       = None,
    guidance_mode    = 'HYBRID',    # 'TPN', 'OGL', or 'HYBRID'
    use_guidance     = True,
    dt               = 0.01,
    t_max            = 60.0,
    verbose          = True,
    gain_schedule    = True,
):
    if target_pos is None:
        target_pos = np.array([2500.0, 0.0, 500.0])

    target_alt   = float(target_pos[2])
    launch_angle = np.radians(launch_angle_deg)
    v0           = 25.0

    pos0   = np.array([0.0, 0.0, 10.0])
    vel0_i = np.array([v0*np.cos(launch_angle), 0.0, v0*np.sin(launch_angle)])
    q0     = euler_to_quat(0.0, launch_angle, 0.0)
    state  = np.concatenate([pos0, vel0_i, q0, np.zeros(3)])

    actuator   = FinActuator()
    controller = LQRController(q_bar=50000.0)

    tpn_guidance = None
    if use_guidance and USE_GUIDANCE:
        tpn_guidance = GuidanceLaw(variant='TPN', N=4.0, noise_std=0.0,
                                   accel_limit=10.0, tau=0.1)

    target = None
    if use_guidance and USE_GUIDANCE:
        target = ManeuveringTarget(
            pos0=target_pos,
            heading_deg=180.0,
            speed=50.0,
            max_accel=0.0,
            seed=42
        )

    hist = {k: [] for k in
            ['t','pos','vel','euler','omega','fin','a_cmd','euler_des','mach','qbar','range']}

    t             = 0.0
    CONTROL_START = 2.0
    euler_smooth  = np.array([0.0, launch_angle, 0.0])
    euler_des     = np.array([0.0, launch_angle, 0.0])
    hit           = False
    min_range     = np.inf
    last_range    = np.inf
    last_gs_time  = 0.0

    if verbose:
        tpn_n = tpn_guidance.N if tpn_guidance is not None else 'N/A'
        print(f"{'='*60}")
        print(f"Project 09 — Attitude Control Simulation")
        print(f"{'='*60}")
        print(f"Launch angle: {launch_angle_deg}°  |  Target: {target_pos}")
        print(f"Guidance: {guidance_mode}" +
              (f"  TPN N={tpn_n}" if guidance_mode == 'TPN' else
               f"  OGL N={OGL_N}  t_go_min={OGL_T_GO_MIN}s  accel_limit={OGL_ACCEL_LIMIT}m/s²"
               + (f"  switch R=200m" if guidance_mode == 'HYBRID' else '')))
        print(f"v0={v0}m/s  CONTROL_START={CONTROL_START}s  TAU_DES={TAU_DES}s")
        print(f"CN_ALPHA={CN_ALPHA}  guidance_clamp=±{ALPHA_CLAMP_DEG}°  "
              f"aero_clamp=±{AERO_ALPHA_CLAMP_DEG}°")
        print(f"THRUST_BOOST={THRUST_BOOST}N  THRUST_SUSTAIN={THRUST_SUSTAIN}N  "
              f"T_BOOST={T_BOOST_END}s  T_SUSTAIN={T_SUSTAIN_END}s")
        print(f"grav_comp: below={GRAV_COMP_BELOW}G  above={GRAV_COMP_ABOVE}G")
        print(f"  LQR Q [phi,tht,psi,p,q,r]: {controller.Q_diag}")
        print(f"  LQR R [roll,pitch,yaw]:     {controller.R_diag}")
        print(f"{'='*60}")

    while t < t_max:
        pos   = state[0:3]
        vel_i = state[3:6]
        q     = state[6:10]
        omega = state[10:13]
        euler = quat_to_euler(q)
        V     = np.linalg.norm(vel_i)
        alt   = pos[2]
        rho   = atmosphere(alt)
        q_bar = 0.5 * rho * V**2
        mach  = V / SPEED_OF_SOUND

        if alt < 0.0 and t > 0.5:
            if verbose: print(f"  t={t:.1f}s: Ground impact")
            break

        r_t = target.pos if target is not None else target_pos
        R   = np.linalg.norm(r_t - pos)

        if R < 20.0:
            hit = True
            if verbose: print(f"  t={t:.1f}s: HIT! R={R:.1f}m")
            break

        if R > last_range + 50.0 and t > 2.0:
            if verbose: print(f"  t={t:.1f}s: CPA. Miss={min_range:.1f}m")
            break

        min_range  = min(min_range, R)
        last_range = R

        if gain_schedule and (t - last_gs_time) >= 1.0:
            try:
                controller.gain_schedule(max(q_bar, 5000.0))
            except Exception:
                pass
            last_gs_time = t

        # Guidance command
        a_cmd = np.zeros(3)
        if use_guidance and t >= CONTROL_START:
            vel_t   = target.vel if target is not None else np.zeros(3)
            accel_t = getattr(target, 'accel', np.zeros(3))
            if guidance_mode == 'TPN':
                a_out, valid = tpn_guidance.compute(pos, r_t, vel_i, vel_t, accel_t, dt)
                if valid:
                    a_cmd = a_out
            elif guidance_mode == 'OGL':
                a_out, valid = compute_ogl(pos, r_t, vel_i, vel_t)
                if valid:
                    a_cmd = a_out
            elif guidance_mode == 'HYBRID':
                R_now = np.linalg.norm(r_t - pos)
                if R_now < 200.0:
                    a_out, valid = compute_ogl(pos, r_t, vel_i, vel_t)
                else:
                    a_out, valid = tpn_guidance.compute(pos, r_t, vel_i, vel_t, accel_t, dt)
                if valid:
                    a_cmd = a_out

        if t < CONTROL_START:
            fin_deflections = np.zeros(4)
            euler_des        = euler.copy()
            euler_smooth     = euler.copy()
        else:
            tau_eff   = TAU_DES if R > 300.0 else 0.0
            euler_raw = guidance_to_attitude(vel_i, a_cmd, t, pos,
                                             q_current=q,
                                             target_alt=target_alt,
                                             r_t=r_t)
            alpha_s       = dt / (tau_eff + dt) if tau_eff > 0.0 else 1.0
            euler_smooth += alpha_s * (euler_raw - euler_smooth)
            euler_des     = euler_smooth.copy()

            u = controller.compute(euler, omega, euler_des)
            actuator.set_commands(u[0], u[1], u[2])
            actuator.step(dt)
            fin_deflections = actuator.deflections

        hist['t'].append(t)
        hist['pos'].append(pos.copy())
        hist['vel'].append(vel_i.copy())
        hist['euler'].append(euler.copy())
        hist['omega'].append(omega.copy())
        hist['fin'].append(fin_deflections.copy())
        hist['a_cmd'].append(a_cmd.copy())
        hist['euler_des'].append(euler_des.copy())
        hist['mach'].append(mach)
        hist['qbar'].append(q_bar)
        hist['range'].append(R)

        if verbose and abs(t % 1.0) < dt * 1.5 and not (dt < 0.008 and abs(t % 1.0) > dt * 0.5):
            print(f"  t={t:5.1f}s  R={R:6.0f}m  V={V:5.0f}m/s  alt={alt:6.0f}m  "
                  f"θ={np.degrees(euler[1]):+6.1f}°  ψ={np.degrees(euler[2]):+5.1f}°  "
                  f"θdes={np.degrees(euler_des[1]):+5.1f}°  ψdes={np.degrees(euler_des[2]):+5.1f}°  "
                  f"δp={np.degrees(fin_deflections[0]):+5.1f}°  δy={np.degrees(fin_deflections[1]):+5.1f}°")

        if target is not None:
            target.step(dt)
        dt_eff = dt / 2.0 if R < 300.0 else dt
        state  = rk4_step(t, state, fin_deflections, dt_eff)
        t += dt_eff

    for k in hist:
        hist[k] = np.array(hist[k])

    result = {'hit': hit, 'miss_distance': min_range,
              't_final': t, 'history': hist}

    if verbose:
        print(f"{'='*60}")
        print(f"Miss distance:  {min_range:.1f} m")
        print(f"Hit:            {hit}")
        print(f"Flight time:    {t:.1f} s")
        if len(hist['mach']) > 0:
            print(f"Peak Mach:      {np.max(hist['mach']):.2f}")
            print(f"Peak altitude:  {np.max(hist['pos'][:,2]):.0f} m")
        print(f"{'='*60}")

    return result


# ---------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------

def plot_results(result, save_dir=None, title_suffix=''):
    hist = result['history']
    if len(hist['t']) == 0:
        return

    t     = hist['t']
    pos   = hist['pos']
    euler = hist['euler']
    omega = hist['omega']
    fin   = hist['fin']
    mach  = hist['mach']

    fig, axes = plt.subplots(3, 2, figsize=(14, 12))
    fig.suptitle(f'Project 09 — Attitude Control{title_suffix}', fontsize=13)

    axes[0,0].plot(pos[:,0]/1000, pos[:,2], 'b-', lw=1.5)
    axes[0,0].set_xlabel('Downrange (km)'); axes[0,0].set_ylabel('Altitude (m)')
    axes[0,0].set_title('Trajectory'); axes[0,0].grid(True, alpha=0.3)

    axes[0,1].plot(t, np.degrees(euler[:,0]), label='Roll φ')
    axes[0,1].plot(t, np.degrees(euler[:,1]), label='Pitch θ')
    axes[0,1].plot(t, np.degrees(euler[:,2]), label='Yaw ψ')
    if len(hist['euler_des']) > 0:
        axes[0,1].plot(t, np.degrees(hist['euler_des'][:,1]), 'k--',
                       alpha=0.5, label='θ desired')
        axes[0,1].plot(t, np.degrees(hist['euler_des'][:,2]), 'g--',
                       alpha=0.5, label='ψ desired')
    axes[0,1].set_xlabel('Time (s)'); axes[0,1].set_ylabel('Angle (deg)')
    axes[0,1].set_title('Euler Angles vs Desired')
    axes[0,1].legend(fontsize=8); axes[0,1].grid(True, alpha=0.3)

    axes[1,0].plot(t, np.degrees(omega[:,0]), label='p (roll)')
    axes[1,0].plot(t, np.degrees(omega[:,1]), label='q (pitch)')
    axes[1,0].plot(t, np.degrees(omega[:,2]), label='r (yaw)')
    axes[1,0].set_xlabel('Time (s)'); axes[1,0].set_ylabel('Rate (deg/s)')
    axes[1,0].set_title('Body Angular Rates')
    axes[1,0].legend(fontsize=8); axes[1,0].grid(True, alpha=0.3)

    for i in range(4):
        axes[1,1].plot(t, np.degrees(fin[:,i]), label=f'Fin {i+1}', alpha=0.8)
    axes[1,1].axhline( 20, color='r', ls='--', alpha=0.5, label='±20° limit')
    axes[1,1].axhline(-20, color='r', ls='--', alpha=0.5)
    axes[1,1].set_xlabel('Time (s)'); axes[1,1].set_ylabel('Deflection (deg)')
    axes[1,1].set_title('Fin Deflections')
    axes[1,1].legend(fontsize=7); axes[1,1].grid(True, alpha=0.3)

    axes[2,0].plot(t, mach, 'g-', lw=1.5)
    axes[2,0].set_xlabel('Time (s)'); axes[2,0].set_ylabel('Mach')
    axes[2,0].set_title('Mach Number'); axes[2,0].grid(True, alpha=0.3)

    axes[2,1].plot(t, hist['range'], 'm-', lw=1.5)
    axes[2,1].axhline(20, color='r', ls='--', alpha=0.7, label='Lethal radius 20m')
    axes[2,1].set_xlabel('Time (s)'); axes[2,1].set_ylabel('Range (m)')
    axes[2,1].set_title('Range to Target')
    axes[2,1].legend(fontsize=8); axes[2,1].grid(True, alpha=0.3)

    plt.tight_layout()
    if save_dir:
        os.makedirs(save_dir, exist_ok=True)
        fname = 'attitude_sim_ogl.png' if 'OGL' in title_suffix else 'attitude_sim.png'
        path  = os.path.join(save_dir, fname)
        plt.savefig(path, dpi=150, bbox_inches='tight')
        print(f"Saved: {path}")
    plt.show()
    plt.close()


# ---------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------

if __name__ == '__main__':
    # Force Project 09 folder
    RESULTS_DIR = os.path.join(
        os.path.dirname(__file__), '..', '..', '09_attitude_control', 'results', 'figures'
    )
    os.makedirs(RESULTS_DIR, exist_ok=True)

    # ---- OGL run ----
    print("\n" + "="*60)
    print("RUNNING: Hybrid TPN+OGL")
    print("="*60)
    result_ogl = run_attitude_sim(
        launch_angle_deg = 21.2,
        target_pos       = np.array([2500.0, 0.0, 500.0]),
        guidance_mode    = 'HYBRID',
        use_guidance     = True,
        dt               = 0.01,
        t_max            = 60.0,
        verbose          = True,
        gain_schedule    = True,
    )
    plot_results(result_ogl, save_dir=RESULTS_DIR, title_suffix=' — OGL')

    # ---- TPN run (baseline comparison) ----
    print("\n" + "="*60)
    print("RUNNING: TPN (baseline)")
    print("="*60)
    result_tpn = run_attitude_sim(
        launch_angle_deg = 21.2,
        target_pos       = np.array([2500.0, 0.0, 500.0]),
        guidance_mode    = 'TPN',
        use_guidance     = True,
        dt               = 0.01,
        t_max            = 60.0,
        verbose          = True,
        gain_schedule    = True,
    )
    plot_results(result_tpn, save_dir=RESULTS_DIR, title_suffix=' — TPN')

    # ---- Summary ----
    print("\n" + "="*60)
    print("COMPARISON SUMMARY")
    print("="*60)
    print(f"  OGL miss distance: {result_ogl['miss_distance']:.1f} m  "
          f"({'HIT' if result_ogl['hit'] else 'miss'})")
    print(f"  TPN miss distance: {result_tpn['miss_distance']:.1f} m  "
          f"({'HIT' if result_tpn['hit'] else 'miss'})")
    print("="*60)