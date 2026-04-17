"""
compare_lqr.py -- Three-Loop Autopilot vs LQR Comparison
Project 10 -- Three-Loop Autopilot

Runs identical engagement with both controllers and produces:
  - Side-by-side trajectory plots
  - Acceleration tracking comparison
  - Fin deflection activity comparison
  - Summary table

Run from 10_autopilot/src/
"""

import sys
import os
import numpy as np
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.join(os.path.dirname(__file__),
    '..', '..', '09_attitude_control', 'src'))

from autopilot_sim import run_autopilot_sim

try:
    from attitude_sim import run_attitude_sim
    HAS_LQR = True
except ImportError:
    HAS_LQR = False
    print("WARNING: attitude_sim.py not found — LQR comparison unavailable")


ENGAGEMENT = dict(
    launch_angle_deg = 21.2,
    target_pos       = np.array([2500.0, 0.0, 500.0]),
    use_guidance     = True,
    dt               = 0.01,
    t_max            = 60.0,
    verbose          = False,
    gain_schedule    = True,
)

RESULTS_DIR = os.path.join(os.path.dirname(__file__), '..', 'results', 'figures')


def run_comparison():
    print("Running Three-Loop Autopilot...")
    result_ap = run_autopilot_sim(**ENGAGEMENT)

    result_lqr = None
    if HAS_LQR:
        print("Running LQR baseline...")
        result_lqr = run_attitude_sim(
            **{k: v for k, v in ENGAGEMENT.items()},
            guidance_mode='TPN',
        )

    print_summary(result_ap, result_lqr)
    plot_comparison(result_ap, result_lqr)
    return result_ap, result_lqr


def print_summary(result_ap, result_lqr):
    print("\n" + "="*60)
    print("PROJECT 10 — AUTOPILOT vs LQR COMPARISON")
    print("="*60)

    def fmt(r, label):
        if r is None:
            return
        h = r['history']
        miss = r['miss_distance']
        t    = r['t_final']
        mach = np.max(h['mach']) if len(h['mach']) > 0 else 0
        alt  = np.max(h['pos'][:,2]) if len(h['pos']) > 0 else 0
        fin_rms = np.sqrt(np.mean(h['fin']**2)) if len(h['fin']) > 0 else 0
        print(f"\n  {label}:")
        print(f"    Miss distance:    {miss:.1f} m")
        print(f"    Hit (R<20m):      {r['hit']}")
        print(f"    Flight time:      {t:.1f} s")
        print(f"    Peak Mach:        {mach:.2f}")
        print(f"    Peak altitude:    {alt:.0f} m")
        print(f"    Fin RMS activity: {np.degrees(fin_rms):.2f} deg")

    fmt(result_ap,  "Three-Loop Autopilot")
    fmt(result_lqr, "LQR (Project 09 baseline)")

    if result_lqr is not None:
        delta = result_ap['miss_distance'] - result_lqr['miss_distance']
        print(f"\n  Delta (Autopilot - LQR): {delta:+.1f} m")
        if delta < 0:
            print("  Three-Loop Autopilot BEATS LQR")
        elif delta == 0:
            print("  IDENTICAL miss distance")
        else:
            print("  LQR beats Three-Loop Autopilot — tune autopilot gains")
    print("="*60)


def plot_comparison(result_ap, result_lqr):
    os.makedirs(RESULTS_DIR, exist_ok=True)

    ap  = result_ap['history']
    lqr = result_lqr['history'] if result_lqr is not None else None

    fig, axes = plt.subplots(2, 3, figsize=(18, 10))
    fig.suptitle('Project 10 — Three-Loop Autopilot vs LQR', fontsize=13)

    # Trajectory
    axes[0,0].plot(ap['pos'][:,0]/1000, ap['pos'][:,2],
                   'b-', lw=1.5, label='Three-Loop')
    if lqr is not None:
        axes[0,0].plot(lqr['pos'][:,0]/1000, lqr['pos'][:,2],
                       'r--', lw=1.5, label='LQR')
    axes[0,0].set_xlabel('Downrange (km)'); axes[0,0].set_ylabel('Altitude (m)')
    axes[0,0].set_title('Trajectory'); axes[0,0].legend(); axes[0,0].grid(True, alpha=0.3)

    # Range to target
    axes[0,1].plot(ap['t'], ap['range'], 'b-', lw=1.5, label='Three-Loop')
    if lqr is not None:
        axes[0,1].plot(lqr['t'], lqr['range'], 'r--', lw=1.5, label='LQR')
    axes[0,1].axhline(20, color='k', ls=':', alpha=0.5, label='Lethal 20m')
    axes[0,1].set_xlabel('Time (s)'); axes[0,1].set_ylabel('Range (m)')
    axes[0,1].set_title('Range to Target'); axes[0,1].legend(); axes[0,1].grid(True, alpha=0.3)

    # Pitch angle
    axes[0,2].plot(ap['t'], np.degrees(ap['euler'][:,1]),
                   'b-', lw=1.5, label='Three-Loop')
    if lqr is not None:
        axes[0,2].plot(lqr['t'], np.degrees(lqr['euler'][:,1]),
                       'r--', lw=1.5, label='LQR')
    axes[0,2].set_xlabel('Time (s)'); axes[0,2].set_ylabel('Pitch (deg)')
    axes[0,2].set_title('Pitch Angle'); axes[0,2].legend(); axes[0,2].grid(True, alpha=0.3)

    # Normal acceleration
    if 'a_body' in ap and len(ap['a_body']) > 0:
        axes[1,0].plot(ap['t'], ap['a_body'][:,2], 'b-', lw=1.0, label='az meas (AP)')
    axes[1,0].set_xlabel('Time (s)'); axes[1,0].set_ylabel('Accel (m/s²)')
    axes[1,0].set_title('Normal Acceleration'); axes[1,0].legend(); axes[1,0].grid(True, alpha=0.3)

    # Fin deflections — pitch channel
    axes[1,1].plot(ap['t'], np.degrees(ap['fin'][:,0]),
                   'b-', lw=1.0, alpha=0.8, label='Three-Loop δp')
    if lqr is not None:
        axes[1,1].plot(lqr['t'], np.degrees(lqr['fin'][:,0]),
                       'r--', lw=1.0, alpha=0.8, label='LQR δp')
    axes[1,1].axhline( 20, color='k', ls=':', alpha=0.4)
    axes[1,1].axhline(-20, color='k', ls=':', alpha=0.4)
    axes[1,1].set_xlabel('Time (s)'); axes[1,1].set_ylabel('Deflection (deg)')
    axes[1,1].set_title('Pitch Fin Deflection'); axes[1,1].legend(); axes[1,1].grid(True, alpha=0.3)

    # Mach
    axes[1,2].plot(ap['t'], ap['mach'], 'b-', lw=1.5, label='Three-Loop')
    if lqr is not None:
        axes[1,2].plot(lqr['t'], lqr['mach'], 'r--', lw=1.5, label='LQR')
    axes[1,2].set_xlabel('Time (s)'); axes[1,2].set_ylabel('Mach')
    axes[1,2].set_title('Mach Number'); axes[1,2].legend(); axes[1,2].grid(True, alpha=0.3)

    plt.tight_layout()
    path = os.path.join(RESULTS_DIR, 'autopilot_vs_lqr.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    print(f"Saved: {path}")
    plt.show()
    plt.close()


if __name__ == '__main__':
    run_comparison()