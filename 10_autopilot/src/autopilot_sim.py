"""
autopilot_sim.py -- 13-State Simulator with Three-Loop Autopilot
Project 10 -- Three-Loop Autopilot

Identical to attitude_sim.py (Project 09) except:
  - AutopilotController replaces LQRController
  - Body-frame accelerometer measurement added for middle loop feedback
  - __main__ runs both autopilot and LQR back-to-back and prints comparison

All EOM, guidance, actuator, and quaternion code is unchanged from Project 09.
"""

import sys
import os
import numpy as np
import matplotlib.pyplot as plt

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
    from target   import ManeuveringTarget
    USE_GUIDANCE = True
except ImportError:
    USE_GUIDANCE = False

from quaternion import (quat_norm, quat_to_rotmat,
                        quat_kinematics, quat_to_euler, euler_to_quat)
from actuator   import (FinActuator, compute_moments,
                        compute_angular_acceleration,
                        compute_aoa_sideslip, S_REF, C_REF)
from autopilot  import AutopilotController, compute_accel_body

# ---------------------------------------------------------------
# Missile parameters (identical to attitude_sim.py)
# ---------------------------------------------------------------
MASS_LAUNCH   = 20.0
MASS_BURNOUT  = 15.0
MASS_PROP     = MASS_LAUNCH - MASS_BURNOUT
THRUST_BOOST  = 3000.0
THRUST_SUSTAIN = 800.0
T_BOOST_END   = 2.0
T_SUSTAIN_END = 15.0
G             = 9.81
RHO0          = 1.225
H_SCALE       = 8500.0
SPEED_OF_SOUND = 343.0
CD0           = 0.3
CN_ALPHA      = 4.0
ALPHA_CLAMP_DEG      = 8.0
BETA_CLAMP_DEG       = 8.0
AERO_ALPHA_CLAMP_DEG = 10.0
GRAV_COMP_BELOW      = 1.0
GRAV_COMP_ABOVE      = 0.0


def atmosphere(alt):
    return RHO0 * np.exp(-max(0.0, alt) / H_SCALE)


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
# Equations of motion (identical to attitude_sim.py)
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

    alpha_f = np.clip(alpha, -np.radians(AERO_ALPHA_CLAMP_DEG),
                              np.radians(AERO_ALPHA_CLAMP_DEG))
    beta_f  = np.clip(beta,  -np.radians(AERO_ALPHA_CLAMP_DEG),
                              np.radians(AERO_ALPHA_CLAMP_DEG))
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
    ns[10:13] = np.clip(ns[10:13], -np.radians(200), np.radians(200))  # realistic 200 deg/s
    return ns


# ---------------------------------------------------------------
# Guidance-to-accel interface
# ---------------------------------------------------------------

def guidance_to_accel_cmd(vel_i, a_cmd_inertial, t, pos,
                          q_current=None, target_alt=500.0, r_t=None):
    """
    Returns the acceleration command in inertial frame with gravity compensation
    and terminal LOS blend — identical logic to attitude_sim.py
    guidance_to_attitude but returns accel rather than Euler angles.
    The autopilot receives the inertial accel command directly and rotates
    it to body frame internally.
    """
    V = np.linalg.norm(vel_i)
    if V < 10.0:
        return np.zeros(3)

    grav_comp = G * GRAV_COMP_BELOW if pos[2] < target_alt else G * GRAV_COMP_ABOVE
    a_out = a_cmd_inertial.copy()
    a_out[2] += grav_comp

    # Terminal LOS blend within 500m
    if r_t is not None:
        r_vec  = r_t - pos
        r_norm = np.linalg.norm(r_vec)
        if r_norm < 500.0 and r_norm > 1.0:
            blend    = (500.0 - r_norm) / 500.0
            mass     = missile_mass(t)
            rho      = atmosphere(pos[2])
            q_bar    = 0.5 * rho * V**2
            denom    = max(q_bar * S_REF * CN_ALPHA, 10.0)
            a_los    = r_vec / r_norm * np.linalg.norm(a_out)
            a_out    = a_out * (1 - blend) + a_los * blend

    return np.clip(a_out, -40.0, 40.0)


# ---------------------------------------------------------------
# Main simulation
# ---------------------------------------------------------------

def run_autopilot_sim(
    launch_angle_deg = 21.2,
    target_pos       = None,
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

    actuator    = FinActuator()
    controller  = AutopilotController(q_bar=50_000.0)

    tpn_guidance = None
    if use_guidance and USE_GUIDANCE:
        tpn_guidance = GuidanceLaw(variant='TPN', N=4.0, noise_std=0.0,
                                   accel_limit=10.0, tau=0.1)

    target = None
    if use_guidance and USE_GUIDANCE:
        target = ManeuveringTarget(
            pos0=target_pos, heading_deg=180.0, speed=50.0,
            max_accel=0.0, seed=42)

    hist = {k: [] for k in
            ['t','pos','vel','euler','omega','fin','a_cmd','a_body','mach','qbar','range']}

    t             = 0.0
    CONTROL_START = 2.5
    hit           = False
    min_range     = np.inf
    last_range    = np.inf
    last_gs_time  = 0.0

    if verbose:
        print(f"{'='*60}")
        print(f"Project 10 — Three-Loop Autopilot Simulation")
        print(f"{'='*60}")
        print(f"Launch angle: {launch_angle_deg}°  |  Target: {target_pos}")
        print(f"Ka={controller.Ka_nom}  Ki_a={controller.Ki_a_nom}  "
              f"Kp_rate={controller.Kp_rate_nom}  Kp_roll={controller.Kp_roll_nom}")
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
        a_cmd_inertial = np.zeros(3)
        if use_guidance and t >= CONTROL_START and tpn_guidance is not None:
            vel_t   = target.vel if target is not None else np.zeros(3)
            accel_t = getattr(target, 'accel', np.zeros(3))
            a_out, valid = tpn_guidance.compute(pos, r_t, vel_i, vel_t, accel_t, dt)
            if valid:
                a_cmd_inertial = guidance_to_accel_cmd(
                    vel_i, a_out, t, pos, q_current=q,
                    target_alt=target_alt, r_t=r_t)
            elif len(hist['a_cmd']) > 0:
                a_cmd_inertial = hist['a_cmd'][-1]  # hold last valid command

        # Body-frame accelerometer measurement
        thrust, _ = propulsion(t)
        a_body = compute_accel_body(state, thrust, missile_mass(t),
                                    rho, S_REF, CN_ALPHA,
                                    CD0, AERO_ALPHA_CLAMP_DEG)

        if t < CONTROL_START:
            fin_deflections = np.zeros(4)
        else:
            if abs(t - CONTROL_START) < dt * 1.5:
                controller.gain_schedule(max(q_bar, 5000.0))
            u = controller.compute(euler, omega, a_body, a_cmd_inertial, dt)
            actuator.set_commands(u[0], u[1], u[2])
            actuator.step(dt)
            fin_deflections = actuator.deflections

        hist['t'].append(t)
        hist['pos'].append(pos.copy())
        hist['vel'].append(vel_i.copy())
        hist['euler'].append(euler.copy())
        hist['omega'].append(omega.copy())
        hist['fin'].append(fin_deflections.copy())
        hist['a_cmd'].append(a_cmd_inertial.copy())
        hist['a_body'].append(a_body.copy())
        hist['mach'].append(mach)
        hist['qbar'].append(q_bar)
        hist['range'].append(R)

        if verbose and abs(t % 1.0) < dt * 1.5 and t not in [h for h in hist['t'][:-1]]:
            print(f"  t={t:5.1f}s  R={R:6.0f}m  V={V:5.0f}m/s  alt={alt:6.0f}m  "
                  f"θ={np.degrees(euler[1]):+6.1f}°  "
                  f"az={a_body[2]:+6.1f}m/s²  "
                  f"δp={np.degrees(fin_deflections[0] if len(fin_deflections)>0 else 0):+5.1f}°")

        if target is not None:
            target.step(dt)
        state = rk4_step(t, state, fin_deflections, dt)
        t += dt

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

    t      = hist['t']
    pos    = hist['pos']
    euler  = hist['euler']
    omega  = hist['omega']
    fin    = hist['fin']
    mach   = hist['mach']
    a_body = hist['a_body']
    a_cmd  = hist['a_cmd']

    fig, axes = plt.subplots(3, 2, figsize=(14, 12))
    fig.suptitle(f'Project 10 — Three-Loop Autopilot{title_suffix}', fontsize=13)

    axes[0,0].plot(pos[:,0]/1000, pos[:,2], 'b-', lw=1.5)
    axes[0,0].set_xlabel('Downrange (km)'); axes[0,0].set_ylabel('Altitude (m)')
    axes[0,0].set_title('Trajectory'); axes[0,0].grid(True, alpha=0.3)

    axes[0,1].plot(t, np.degrees(euler[:,1]), label='Pitch θ')
    axes[0,1].plot(t, np.degrees(euler[:,2]), label='Yaw ψ')
    axes[0,1].set_xlabel('Time (s)'); axes[0,1].set_ylabel('Angle (deg)')
    axes[0,1].set_title('Euler Angles')
    axes[0,1].legend(fontsize=8); axes[0,1].grid(True, alpha=0.3)

    axes[1,0].plot(t, a_body[:,2], label='az measured', alpha=0.8)
    if len(a_cmd) > 0:
        from quaternion import quat_to_rotmat, quat_norm
        axes[1,0].set_title('Normal Acceleration Tracking')
    axes[1,0].set_xlabel('Time (s)'); axes[1,0].set_ylabel('Accel (m/s²)')
    axes[1,0].legend(fontsize=8); axes[1,0].grid(True, alpha=0.3)

    for i in range(min(4, fin.shape[1])):
        axes[1,1].plot(t, np.degrees(fin[:,i]), label=f'Fin {i+1}', alpha=0.8)
    axes[1,1].axhline( 20, color='r', ls='--', alpha=0.5)
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
        path = os.path.join(save_dir, f'autopilot_sim{title_suffix.replace(" ","_")}.png')
        plt.savefig(path, dpi=150, bbox_inches='tight')
        print(f"Saved: {path}")
    plt.show()
    plt.close()


# ---------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------

if __name__ == '__main__':
    RESULTS_DIR = os.path.join(os.path.dirname(__file__), '..', 'results', 'figures')

    print("\n" + "="*60)
    print("RUNNING: Three-Loop Autopilot")
    print("="*60)
    result_ap = run_autopilot_sim(
        launch_angle_deg = 21.2,
        target_pos       = np.array([2500.0, 0.0, 500.0]),
        use_guidance     = True,
        dt               = 0.01,
        t_max            = 60.0,
        verbose          = True,
        gain_schedule    = True,
    )
    plot_results(result_ap, save_dir=RESULTS_DIR, title_suffix=' — Three-Loop')

    # LQR baseline comparison
    try:
        from attitude_sim import run_attitude_sim
        print("\n" + "="*60)
        print("RUNNING: LQR (Project 09 baseline)")
        print("="*60)
        result_lqr = run_attitude_sim(
            launch_angle_deg = 21.2,
            target_pos       = np.array([2500.0, 0.0, 500.0]),
            guidance_mode    = 'TPN',
            use_guidance     = True,
            dt               = 0.01,
            t_max            = 60.0,
            verbose          = True,
            gain_schedule    = True,
        )
        print("\n" + "="*60)
        print("COMPARISON SUMMARY")
        print("="*60)
        print(f"  Three-Loop miss: {result_ap['miss_distance']:.1f} m  "
              f"({'HIT' if result_ap['hit'] else 'miss'})")
        print(f"  LQR miss:        {result_lqr['miss_distance']:.1f} m  "
              f"({'HIT' if result_lqr['hit'] else 'miss'})")
        print("="*60)
    except ImportError:
        print("attitude_sim.py not found — skipping LQR comparison")