"""
seeker_sim.py -- Seeker-in-the-Loop Engagement Simulation
Project 11 -- Seeker Model

Wraps Project 09 attitude_sim with a SeekerModel replacing ideal LOS measurements.
Runs four configurations back-to-back and prints a comparison table:
  1. Ideal sensor (baseline — matches Project 09)
  2. Seeker with base noise only
  3. Seeker with noise + glint
  4. Seeker with ECM spoofing

Monte Carlo over N_MC engagements per configuration, reporting:
  - Mean/median/std miss distance
  - Pk (fraction with miss < LETHAL_RADIUS)
  - Mean lock-loss fraction

v_apparent note:
  Velocity reconstruction from filtered LOS rate introduces systematic bias
  that can accidentally improve guidance (seeker beats ideal — physically wrong).
  Current approach: seeker provides r_app for position (gimbal-degraded LOS),
  guidance uses true vel_t for velocity. This isolates the position-degradation
  effect. Future work: direct LOS-rate interface to guidance law, bypassing
  Cartesian reconstruction entirely.
"""

import sys
import os
import numpy as np
import matplotlib.pyplot as plt
import pickle

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(THIS_DIR, '..', '..', '04_missile_aero_database', 'postprocess'))
sys.path.insert(0, os.path.join(THIS_DIR, '..', '..', '08_gnc_monte_carlo', 'src'))
sys.path.insert(0, os.path.join(THIS_DIR, '..', '..', '09_attitude_control', 'src'))

from attitude_sim import (
    run_attitude_sim, atmosphere, propulsion, missile_mass,
    SPEED_OF_SOUND, G, GRAV_COMP_BELOW, GRAV_COMP_ABOVE,
    ALPHA_CLAMP_DEG, BETA_CLAMP_DEG, CN_ALPHA, TAU_DES,
    rk4_step, flight_path_attitude, guidance_to_attitude,
)
from seeker import SeekerModel

try:
    from guidance import GuidanceLaw
    from target   import ManeuveringTarget
    USE_GUIDANCE = True
except ImportError:
    USE_GUIDANCE = False

try:
    from quaternion import quat_norm, quat_to_rotmat, quat_kinematics, quat_to_euler, euler_to_quat
    from actuator   import FinActuator, compute_moments, compute_angular_acceleration, compute_aoa_sideslip, S_REF
    from lqr        import LQRController
except ImportError as e:
    print(f"Import error: {e}")
    sys.exit(1)

# ---------------------------------------------------------------
# Constants
# ---------------------------------------------------------------
LETHAL_RADIUS = 20.0    # m
N_MC          = 50      # Monte Carlo runs per configuration
TARGET_POS    = np.array([2500.0, 0.0, 500.0])
LAUNCH_ANGLE  = 21.2    # deg
DT            = 0.01    # s
T_MAX         = 60.0    # s
CONTROL_START = 2.0     # s


# ---------------------------------------------------------------
# Single engagement with seeker
# ---------------------------------------------------------------

def run_seeker_engagement(seeker_cfg: dict, seed: int = 0) -> dict:
    """
    Run one engagement with a given seeker configuration.

    Parameters
    ----------
    seeker_cfg : dict with keys:
        use_seeker  : bool — if False uses ideal sensor (Project 09 baseline)
        noise_std   : float
        glint_std   : float
        ecm_mode    : bool
        ecm_bias    : float
        ecm_noise_std : float
    seed : int — for target noise and seeker RNG

    Returns
    -------
    dict with miss_distance, hit, lock_loss_frac, t_final
    """
    launch_angle = np.radians(LAUNCH_ANGLE)
    v0           = 25.0

    pos0   = np.array([0.0, 0.0, 10.0])
    vel0_i = np.array([v0*np.cos(launch_angle), 0.0, v0*np.sin(launch_angle)])
    q0     = euler_to_quat(0.0, launch_angle, 0.0)
    state  = np.concatenate([pos0, vel0_i, q0, np.zeros(3)])

    actuator   = FinActuator()
    controller = LQRController(q_bar=50000.0)

    tpn_guidance = GuidanceLaw(
        variant='TPN', N=4.0, noise_std=0.0,
        accel_limit=10.0, tau=0.1, seed=seed
    ) if USE_GUIDANCE else None

    target = ManeuveringTarget(
        pos0=TARGET_POS, heading_deg=180.0,
        speed=50.0, max_accel=0.0, seed=seed
    ) if USE_GUIDANCE else None

    # Seeker
    seeker = None
    if seeker_cfg.get('use_seeker', False):
        seeker = SeekerModel(
            noise_std     = seeker_cfg.get('noise_std',    0.02),
            glint_std     = seeker_cfg.get('glint_std',    0.0),
            ecm_mode      = seeker_cfg.get('ecm_mode',     False),
            ecm_bias      = seeker_cfg.get('ecm_bias',     0.0),
            ecm_noise_std = seeker_cfg.get('ecm_noise_std',0.0),
            seed          = seed,
        )
        seeker.reset(pos0, TARGET_POS)
        # Range deception bias for ECM (meters) — scales with angle bias
        seeker._R_est_bias = seeker_cfg.get('ecm_bias', 0.0) * 1000.0

    target_alt  = float(TARGET_POS[2])
    euler_smooth = np.array([0.0, launch_angle, 0.0])
    euler_des    = np.array([0.0, launch_angle, 0.0])
    hit         = False
    min_range   = np.inf
    last_range  = np.inf
    last_gs_t   = 0.0
    t           = 0.0
    lock_loss_count = 0
    total_frames    = 0

    while t < T_MAX:
        pos   = state[0:3]
        vel_i = state[3:6]
        q     = state[6:10]
        omega = state[10:13]
        euler = quat_to_euler(q)
        V     = np.linalg.norm(vel_i)
        alt   = pos[2]
        rho   = atmosphere(alt)
        q_bar = 0.5 * rho * V**2

        if alt < 0.0 and t > 0.5:
            break

        r_t = target.pos if target is not None else TARGET_POS
        R   = np.linalg.norm(r_t - pos)

        # Track min range BEFORE break checks
        min_range  = min(min_range, R)

        if R < LETHAL_RADIUS:
            hit = True
            break
        if R > last_range + 50.0 and t > 2.0:
            break

        last_range = R

        if (t - last_gs_t) >= 1.0:
            try:
                controller.gain_schedule(max(q_bar, 5000.0))
            except Exception:
                pass
            last_gs_t = t

        # Guidance
        a_cmd = np.zeros(3)
        if t >= CONTROL_START and tpn_guidance is not None:
            vel_t   = target.vel if target is not None else np.zeros(3)
            accel_t = getattr(target, 'accel', np.zeros(3))
            total_frames += 1

            if seeker is not None:
                # Seeker-in-the-loop
                r_app, v_app, skr_valid = seeker.update(pos, r_t, vel_i, vel_t, DT)
                if not skr_valid:
                    lock_loss_count += 1
                    a_cmd = np.zeros(3)   # coast when lock lost
                    valid = False
                else:
                    # Use r_app (seeker-degraded position), vel_t (truth — v_app biased)
                    a_out, valid = tpn_guidance.compute(
                        pos, r_app, vel_i, vel_t, accel_t, DT)
                    if valid:
                        a_cmd = a_out
            else:
                # Ideal sensor
                a_out, valid = tpn_guidance.compute(
                    pos, r_t, vel_i, vel_t, accel_t, DT)
                if valid:
                    a_cmd = a_out

        # Controller
        if t < CONTROL_START:
            fin_deflections = np.zeros(4)
            euler_des       = euler.copy()
            euler_smooth    = euler.copy()
        else:
            tau_eff    = TAU_DES if R > 300.0 else 0.0
            euler_raw  = guidance_to_attitude(vel_i, a_cmd, t, pos,
                                              q_current=q,
                                              target_alt=target_alt,
                                              r_t=r_t)
            alpha_s        = DT / (tau_eff + DT) if tau_eff > 0.0 else 1.0
            euler_smooth  += alpha_s * (euler_raw - euler_smooth)
            euler_des      = euler_smooth.copy()

            u = controller.compute(euler, omega, euler_des)
            actuator.set_commands(u[0], u[1], u[2])
            actuator.step(DT)
            fin_deflections = actuator.deflections

        if target is not None:
            target.step(DT)

        dt_eff = DT / 2.0 if R < 300.0 else DT
        state  = rk4_step(t, state, fin_deflections, dt_eff)
        t     += dt_eff

    lock_loss_frac = (lock_loss_count / max(total_frames, 1))
    return {
        'miss_distance':  min_range,
        'hit':            hit,
        'lock_loss_frac': lock_loss_frac,
        't_final':        t,
    }


# ---------------------------------------------------------------
# Monte Carlo over configurations
# ---------------------------------------------------------------

CONFIGS = [
    {
        'label':        'Ideal sensor (baseline)',
        'use_seeker':   False,
        'noise_std':    0.0,
        'glint_std':    0.0,
        'ecm_mode':     False,
        'ecm_bias':     0.0,
        'ecm_noise_std':0.0,
    },
    {
        'label':        'Seeker — base noise',
        'use_seeker':   True,
        'noise_std':    0.02,
        'glint_std':    0.0,
        'ecm_mode':     False,
        'ecm_bias':     0.0,
        'ecm_noise_std':0.0,
    },
    {
        'label':        'Seeker — noise + glint',
        'use_seeker':   True,
        'noise_std':    0.02,
        'glint_std':    0.05,
        'ecm_mode':     False,
        'ecm_bias':     0.0,
        'ecm_noise_std':0.0,
    },
    {
        'label':        'Seeker — ECM spoofing',
        'use_seeker':   True,
        'noise_std':    0.02,
        'glint_std':    0.025,
        'ecm_mode':     True,
        'ecm_bias':     0.08,
        'ecm_noise_std':0.04,
    },
]


def run_monte_carlo(cfg: dict, n: int = N_MC) -> dict:
    misses     = []
    hits       = []
    lock_fracs = []
    for seed in range(n):
        r = run_seeker_engagement(cfg, seed=seed)
        misses.append(r['miss_distance'])
        hits.append(r['hit'])
        lock_fracs.append(r['lock_loss_frac'])
    misses = np.array(misses)
    return {
        'label':       cfg['label'],
        'mean_miss':   np.mean(misses),
        'median_miss': np.median(misses),
        'std_miss':    np.std(misses),
        'pk':          np.mean(hits),
        'lock_loss':   np.mean(lock_fracs),
        'misses':      misses,
    }


# ---------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------

def plot_results(results: list, save_dir: str = None):
    fig, axes = plt.subplots(1, 3, figsize=(15, 5))
    fig.suptitle('Project 11 — Seeker Model: Monte Carlo Results', fontsize=13)

    labels  = [r['label'] for r in results]
    colors  = ['#4da6ff', '#44ff88', '#ffaa44', '#ff4444']

    # Miss distance CDFs
    ax = axes[0]
    for i, r in enumerate(results):
        sorted_m = np.sort(r['misses'])
        cdf      = np.arange(1, len(sorted_m)+1) / len(sorted_m)
        ax.plot(sorted_m, cdf, color=colors[i], label=r['label'], lw=1.5)
    ax.axvline(LETHAL_RADIUS, color='r', ls='--', alpha=0.6, label=f'Lethal {LETHAL_RADIUS}m')
    ax.set_xlabel('Miss distance (m)')
    ax.set_ylabel('CDF')
    ax.set_title('Miss Distance CDF')
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    # Pk bar chart
    ax = axes[1]
    pks = [r['pk'] for r in results]
    bars = ax.bar(range(len(results)), pks, color=colors)
    ax.set_xticks(range(len(results)))
    ax.set_xticklabels([r['label'] for r in results], rotation=12, ha='right', fontsize=8)
    ax.set_ylabel('Pk')
    ax.set_title(f'Pk (miss < {LETHAL_RADIUS}m)')
    ax.set_ylim(0, max(max(pks)*1.2, 0.1))
    ax.grid(True, alpha=0.3, axis='y')
    for bar, pk in zip(bars, pks):
        ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.01,
                f'{pk:.3f}', ha='center', va='bottom', fontsize=8)

    # Mean miss bar chart
    ax = axes[2]
    means = [r['mean_miss'] for r in results]
    stds  = [r['std_miss']  for r in results]
    bars  = ax.bar(range(len(results)), means, yerr=stds,
                   color=colors, capsize=4, error_kw={'linewidth':1.2})
    ax.set_xticks(range(len(results)))
    ax.set_xticklabels([r['label'] for r in results], rotation=12, ha='right', fontsize=8)
    ax.set_ylabel('Mean miss distance (m)')
    ax.set_title('Mean ± Std Miss Distance')
    ax.grid(True, alpha=0.3, axis='y')
    for bar, m in zip(bars, means):
        ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 1,
                f'{m:.1f}m', ha='center', va='bottom', fontsize=8)

    plt.tight_layout()
    if save_dir:
        os.makedirs(save_dir, exist_ok=True)
        path = os.path.join(save_dir, 'seeker_monte_carlo.png')
        plt.savefig(path, dpi=150, bbox_inches='tight')
        print(f"Saved: {path}")
    plt.show()
    plt.close()


# ---------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------

if __name__ == '__main__':
    RESULTS_DIR = os.path.join(THIS_DIR, '..', 'results', 'figures')

    print(f"\n{'='*70}")
    print(f"  Project 11 — Seeker Model: Monte Carlo ({N_MC} runs per config)")
    print(f"{'='*70}\n")

    all_results = []
    for cfg in CONFIGS:
        print(f"  Running: {cfg['label']} ...")
        mc = run_monte_carlo(cfg, n=N_MC)
        all_results.append(mc)
        print(f"    mean={mc['mean_miss']:.1f}m  median={mc['median_miss']:.1f}m  "
              f"std={mc['std_miss']:.1f}m  Pk={mc['pk']:.3f}  "
              f"lock_loss={mc['lock_loss']:.3f}")

    print(f"\n{'='*70}")
    print(f"  SUMMARY")
    print(f"{'='*70}")
    hdr = f"  {'Config':<35}  {'Mean':>7}  {'Median':>7}  {'Std':>6}  {'Pk':>6}  {'LockLoss':>9}"
    print(hdr)
    print(f"  {'-'*68}")
    for r in all_results:
        print(f"  {r['label']:<35}  {r['mean_miss']:7.1f}m  "
              f"{r['median_miss']:7.1f}m  {r['std_miss']:6.1f}m  "
              f"{r['pk']:6.3f}  {r['lock_loss']:9.3f}")
    print(f"{'='*70}\n")

    plot_results(all_results, save_dir=RESULTS_DIR)

    # === Save real miss distances for Project 12 ===
    # Saved to 11_seeker/results/ (not figures/)
    miss_data = {r['label']: r['misses'] for r in all_results}
    save_path = os.path.join(THIS_DIR, '..', 'results', 'seeker_miss_distances.pkl')
    with open(save_path, 'wb') as f:
        pickle.dump(miss_data, f)
    print(f"Saved real miss distances to: {save_path}")