"""
lethality_sim.py -- Lethality Assessment Visualization
Project 12 -- Lethality

Generates:
  1. Pk(miss distance) curve for the warhead model
  2. Overlay with synthetic miss distance PDF from Projects 08/09/10
  3. System-level Pk via overlap integral
  4. Sensitivity: Pk vs warhead C/M ratio and target vulnerability

Run:
  python lethality_sim.py
"""

import os
import sys
import io
import numpy as np
import matplotlib.pyplot as plt

# Force fresh plots every run
def save_plot(fig, filename, save_dir):
    os.makedirs(save_dir, exist_ok=True)
    path = os.path.join(save_dir, filename)
    if os.path.exists(path):
        os.remove(path)        # delete old file to force update
    fig.savefig(path, dpi=150, bbox_inches='tight')
    print(f"Saved: {path}")
    plt.close(fig)

# Force UTF-8 stdout on Windows (handles mu, squared, degree symbols)
if sys.platform == 'win32' and hasattr(sys.stdout, 'buffer'):
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', line_buffering=True)

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, THIS_DIR)

from lethality import (
    WarheadParams, TargetParams, gurney_velocity, mott_distribution,
    pk_at_miss, pk_curve, system_pk, fragment_velocity_at_range,
)

RESULTS_DIR = os.path.join(THIS_DIR, '..', 'results', 'figures')


def load_miss_samples():
    """
    Loads miss distances for Project 12.
    - First tries to load REAL data from seeker_sim.py (preferred)
    - Falls back to synthetic data if the real file is not found or fails to load
    """
    pkl_path = os.path.join(THIS_DIR, '..', '..', '11_seeker', 'results', 'seeker_miss_distances.pkl')
    
    if os.path.exists(pkl_path):
        try:
            import pickle
            with open(pkl_path, 'rb') as f:
                data = pickle.load(f)
            print(f"✅ Loaded REAL miss distances from seeker_sim.py: {pkl_path}")
            print(f"   ({len(next(iter(data.values())))} runs per config)")
            return data
        except Exception as e:
            print(f"⚠️  Could not load real miss data: {e}")
    
    # Fallback: synthetic data
    print("⚠️  Using SYNTHETIC miss samples (real data file not found or failed to load)")
    rng = np.random.default_rng(42)
    return {
        'Ideal sensor (baseline)':   np.abs(rng.normal(35.7, 3.0, 1000)),
        'Seeker — base noise':       np.abs(rng.normal(38.5, 9.0, 1000)),
        'Seeker — noise + glint':    np.abs(rng.normal(43.0, 11.0, 1000)),
        'Seeker — ECM spoofing':     np.abs(rng.normal(52.0, 14.0, 1000)),
    }


# ---------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------

def plot_pk_curve(wp, tp, save_dir=None):
    """Plot Pk vs miss distance."""
    r_arr, pk_arr = pk_curve(80.0, 500, wp, tp)

    fig, ax = plt.subplots(figsize=(10, 6))
    ax.plot(r_arr, pk_arr, 'r-', lw=2.0, label='Pk(miss)')
    ax.axhline(0.5, color='gray', ls='--', alpha=0.5, label='Pk=0.5')

    # Find lethal radius (Pk=0.5 crossing)
    idx = np.searchsorted(-pk_arr, -0.5)
    if idx < len(r_arr):
        r_lethal = r_arr[idx]
        ax.axvline(r_lethal, color='orange', ls='--', alpha=0.7,
                   label=f'Lethal radius ~ {r_lethal:.1f}m')

    V0 = gurney_velocity(wp)
    _, _, N_total, mu = mott_distribution(wp)

    ax.set_xlabel('Miss Distance (m)', fontsize=11)
    ax.set_ylabel('Kill Probability Pk', fontsize=11)
    ax.set_title(f'Project 12 - Pk vs Miss Distance\n'
                 f'C/M={wp.M_charge/wp.M_case:.2f}  V0={V0:.0f}m/s  '
                 f'N={N_total}  mu={mu*1000:.1f}g', fontsize=12)
    ax.set_xlim(0, 80)
    ax.set_ylim(0, 1.05)
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    if save_dir:
        save_plot(fig, 'pk_vs_miss.png', save_dir)


def plot_system_pk_overlay(wp, tp, save_dir=None):
    """
    Overlay miss distance PDF with Pk(r) curve.
    Computes system-level Pk for each guidance configuration.
    """
    r_arr, pk_arr = pk_curve(120.0, 600, wp, tp)
    miss_data = load_miss_samples()

    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    fig.suptitle('Project 12 - Kill Chain Closure: Miss PDF x Pk(r)', fontsize=13)

    colors = ['#4da6ff', '#44ff88', '#ffaa44', '#ff4444']

    # Left: PDF + Pk overlay
    ax = axes[0]
    ax2 = ax.twinx()
    ax2.plot(r_arr, pk_arr, 'r-', lw=2.0, alpha=0.8, label='Pk(r)')
    ax2.set_ylabel('Pk', color='r', fontsize=11)
    ax2.tick_params(axis='y', labelcolor='r')
    ax2.set_ylim(0, 1.05)

    for i, (label, misses) in enumerate(miss_data.items()):
        ax.hist(misses, bins=40, alpha=0.4, color=colors[i], density=True, label=label)

    ax.set_xlabel('Miss Distance (m)', fontsize=11)
    ax.set_ylabel('Miss Distance PDF', fontsize=11)
    ax.set_xlim(0, 120)
    ax.legend(fontsize=8, loc='upper right')
    ax.grid(True, alpha=0.3)
    ax.set_title('Miss Distribution vs Pk(r)')

    # Right: System Pk bar chart
    ax = axes[1]
    labels = list(miss_data.keys())
    sys_pks = [system_pk(misses, wp, tp) for misses in miss_data.values()]

    bars = ax.bar(range(len(labels)), sys_pks, color=colors[:len(labels)])
    ax.set_xticks(range(len(labels)))
    ax.set_xticklabels(labels, rotation=15, ha='right', fontsize=9)
    ax.set_ylabel('System Pk', fontsize=11)
    ax.set_title('System-Level Kill Probability')
    ax.set_ylim(0, max(max(sys_pks) * 1.3, 0.1))
    ax.grid(True, alpha=0.3, axis='y')

    for bar, pk in zip(bars, sys_pks):
        ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.005,
                f'{pk:.4f}', ha='center', va='bottom', fontsize=9)

    plt.tight_layout()
    if save_dir:
        save_plot(fig, 'system_pk_overlay.png', save_dir)

    return dict(zip(labels, sys_pks))


def plot_sensitivity(save_dir=None):
    """Pk sensitivity to C/M ratio and target vulnerability."""
    tp = TargetParams()

    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    fig.suptitle('Project 12 - Lethality Sensitivity Analysis', fontsize=13)

    # Left: vary C/M ratio
    ax = axes[0]
    cm_ratios = [0.2, 0.3, 0.5, 0.7, 1.0]
    for cm in cm_ratios:
        wp = WarheadParams(M_case=3.0, M_charge=3.0*cm)
        r_arr, pk_arr = pk_curve(60.0, 300, wp, tp)
        ax.plot(r_arr, pk_arr, lw=1.5, label=f'C/M={cm:.1f}')
    ax.set_xlabel('Miss Distance (m)')
    ax.set_ylabel('Pk')
    ax.set_title('Pk Sensitivity to C/M Ratio')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # Right: vary vulnerable fraction
    ax = axes[1]
    vuln_fracs = [0.10, 0.15, 0.25, 0.35, 0.50]
    wp = WarheadParams()
    for vf in vuln_fracs:
        tp_v = TargetParams(vulnerable_frac=vf)
        r_arr, pk_arr = pk_curve(60.0, 300, wp, tp_v)
        ax.plot(r_arr, pk_arr, lw=1.5, label=f'Av/Ap={vf:.2f}')
    ax.set_xlabel('Miss Distance (m)')
    ax.set_ylabel('Pk')
    ax.set_title('Pk Sensitivity to Target Vulnerability')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    if save_dir:
        save_plot(fig, 'lethality_sensitivity.png', save_dir)


# ---------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------

if __name__ == '__main__':
    print(f"\n{'='*60}")
    print(f"  Project 12 - Fragmentation Lethality Assessment")
    print(f"{'='*60}\n")

    wp = WarheadParams()
    tp = TargetParams()

    V0 = gurney_velocity(wp)
    masses, counts, N_total, mu = mott_distribution(wp)

    print(f"  Warhead: M_case={wp.M_case}kg  M_charge={wp.M_charge}kg  "
          f"C/M={wp.M_charge/wp.M_case:.3f}")
    print(f"  Gurney velocity: {V0:.0f} m/s")
    print(f"  Mott distribution: N={N_total}  mu={mu*1000:.2f}g")
    print(f"  Target: Ap={tp.presented_area}m^2  Av/Ap={tp.vulnerable_frac}")
    print()

    # Pk at key miss distances
    for r in [0.1, 1.0, 2.0, 5.0, 10.0, 15.0, 20.0, 30.0, 50.0]:
        print(f"  Pk({r:5.1f}m) = {pk_at_miss(r, wp, tp):.4f}")

    # Generate plots
    print(f"\n  Generating plots...")
    os.makedirs(RESULTS_DIR, exist_ok=True)
    plot_pk_curve(wp, tp, save_dir=RESULTS_DIR)
    sys_pks = plot_system_pk_overlay(wp, tp, save_dir=RESULTS_DIR)
    plot_sensitivity(save_dir=RESULTS_DIR)

    print(f"\n  System-level Pk:")
    for label, pk in sys_pks.items():
        print(f"    {label}: Pk_sys = {pk:.4f}")

    print(f"\n{'='*60}")