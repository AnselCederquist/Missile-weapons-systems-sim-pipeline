"""
envelope_sim.py -- Launch Envelope Visualization
Project 13 -- Launch Envelope

Generates:
  1. Pk polar contour (range × bearing)
  2. Miss distance polar contour
  3. NEZ / FEZ boundary overlay
  4. Comparison: non-maneuvering vs maneuvering target

Run:
  python envelope_sim.py
"""

import os
import sys
import numpy as np
import matplotlib.pyplot as plt

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, THIS_DIR)

def save_plot(fig, filename, save_dir):
    """Save plot, overwriting if exists (Windows file lock workaround)."""
    os.makedirs(save_dir, exist_ok=True)
    path = os.path.join(save_dir, filename)
    if os.path.exists(path):
        try:
            os.remove(path)
        except Exception:
            pass
    fig.savefig(path, dpi=150, bbox_inches='tight')
    print(f"  Saved: {path}")

from launch_envelope import (
    TargetState, LaunchGeometry, sweep_range_bearing,
    extract_boundaries, run_simplified_engagement,
)

RESULTS_DIR = os.path.join(THIS_DIR, '..', 'results', 'figures')


# ---------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------

def plot_polar_pk(sweep_result: dict, boundaries: dict,
                  title_suffix: str = '', save_dir: str = None):
    """Polar contour plot of Pk vs range and bearing."""
    ranges   = sweep_result['ranges']
    bearings = sweep_result['bearings']
    pk_grid  = sweep_result['pk_grid']

    bear_rad = np.radians(bearings)
    R_mesh, B_mesh = np.meshgrid(ranges, bear_rad, indexing='ij')

    fig, ax = plt.subplots(subplot_kw={'projection': 'polar'}, figsize=(9, 9))

    # Pk contour
    levels = [0.001, 0.005, 0.01, 0.02, 0.05, 0.1, 0.2, 0.5, 0.8]
    cs = ax.contourf(B_mesh, R_mesh, pk_grid, levels=levels,
                     cmap='RdYlGn', extend='both', alpha=0.8)
    cbar = plt.colorbar(cs, ax=ax, pad=0.1, shrink=0.8)
    cbar.set_label('Kill Probability Pk', fontsize=10)

    # NEZ boundary
    nez_r = boundaries['nez_range']
    fez_r = boundaries['fez_range']
    b_rad = np.radians(boundaries['bearings'])

    if np.any(nez_r > 0):
        ax.plot(b_rad, nez_r, 'b-', lw=2.5, label=f'NEZ (Pk>{boundaries["pk_nez_thr"]})')
    ax.plot(b_rad, fez_r, 'r--', lw=2.0, label=f'FEZ (Pk>{boundaries["pk_fez_thr"]})')

    tgt = sweep_result['target']
    ax.set_title(f'Project 13 — Launch Envelope{title_suffix}\n'
                 f'Target: {tgt.speed}m/s  hdg={tgt.heading_deg}°  '
                 f'alt={tgt.altitude}m  g={tgt.g_load}g',
                 fontsize=11, pad=20)
    ax.set_theta_zero_location('N')
    ax.set_theta_direction(-1)
    ax.legend(fontsize=9, loc='upper right', bbox_to_anchor=(1.3, 1.1))

    plt.tight_layout()
    if save_dir:
        os.makedirs(save_dir, exist_ok=True)
        suffix = title_suffix.replace(' ', '_').replace('—', '').strip('_')
        suffix = title_suffix.replace(' ', '_').replace('—', '').strip('_')
        save_plot(fig, f'envelope_pk{("_" + suffix) if suffix else ""}.png', save_dir)
    plt.close()


def plot_miss_contour(sweep_result: dict, title_suffix: str = '',
                      save_dir: str = None):
    """Polar contour plot of miss distance."""
    ranges   = sweep_result['ranges']
    bearings = sweep_result['bearings']
    miss_grid = sweep_result['miss_grid']

    bear_rad = np.radians(bearings)
    R_mesh, B_mesh = np.meshgrid(ranges, bear_rad, indexing='ij')

    fig, ax = plt.subplots(subplot_kw={'projection': 'polar'}, figsize=(9, 9))

    levels = [5, 10, 20, 50, 100, 200, 500, 1000]
    cs = ax.contourf(B_mesh, R_mesh, np.clip(miss_grid, 0, 1500),
                     levels=levels, cmap='RdYlGn_r', extend='max', alpha=0.8)
    cbar = plt.colorbar(cs, ax=ax, pad=0.1, shrink=0.8)
    cbar.set_label('Mean Miss Distance (m)', fontsize=10)

    tgt = sweep_result['target']
    ax.set_title(f'Project 13 — Miss Distance Map{title_suffix}\n'
                 f'Target: {tgt.speed}m/s  hdg={tgt.heading_deg}°  '
                 f'g={tgt.g_load}g',
                 fontsize=11, pad=20)
    ax.set_theta_zero_location('N')
    ax.set_theta_direction(-1)

    plt.tight_layout()
    if save_dir:
        os.makedirs(save_dir, exist_ok=True)
        suffix = title_suffix.replace(' ', '_').replace('—', '').strip('_')
        path = os.path.join(save_dir, f'envelope_miss{("_" + suffix) if suffix else ""}.png')
        suffix = title_suffix.replace(' ', '_').replace('—', '').strip('_')
        save_plot(fig, f'envelope_miss{("_" + suffix) if suffix else ""}.png', save_dir)
    plt.close()


def plot_comparison(results_list: list, save_dir: str = None):
    """Side-by-side NEZ/FEZ comparison for multiple target configs."""
    n = len(results_list)
    fig, axes = plt.subplots(1, n, subplot_kw={'projection': 'polar'},
                              figsize=(8 * n, 8))
    if n == 1:
        axes = [axes]

    colors = ['#4da6ff', '#ff4444', '#44ff88', '#ffaa44']

    for i, (sweep, bounds, label) in enumerate(results_list):
        ax = axes[i]
        ranges   = sweep['ranges']
        bearings = sweep['bearings']
        pk_grid  = sweep['pk_grid']

        bear_rad = np.radians(bearings)
        R_mesh, B_mesh = np.meshgrid(ranges, bear_rad, indexing='ij')

        levels = [0.001, 0.01, 0.05, 0.1, 0.2, 0.5]
        ax.contourf(B_mesh, R_mesh, pk_grid, levels=levels,
                    cmap='RdYlGn', extend='both', alpha=0.7)

        b_rad = np.radians(bounds['bearings'])
        if np.any(bounds['nez_range'] > 0):
            ax.plot(b_rad, bounds['nez_range'], 'b-', lw=2.5,
                    label=f'NEZ (Pk>{bounds["pk_nez_thr"]})')
        ax.plot(b_rad, bounds['fez_range'], 'r--', lw=2.0,
                label=f'FEZ (Pk>{bounds["pk_fez_thr"]})')

        ax.set_title(label, fontsize=11, pad=15)
        ax.set_theta_zero_location('N')
        ax.set_theta_direction(-1)
        ax.legend(fontsize=8, loc='upper right', bbox_to_anchor=(1.3, 1.1))

    fig.suptitle('Project 13 — Launch Envelope Comparison', fontsize=13, y=1.02)
    plt.tight_layout()
    if save_dir:
        os.makedirs(save_dir, exist_ok=True)
        path = os.path.join(save_dir, 'envelope_comparison.png')
        save_plot(fig, 'envelope_comparison.png', save_dir)
    plt.close()


# ---------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------

if __name__ == '__main__':
    print(f"\n{'='*60}")
    print(f"  Project 13 — Launch Envelope Calculator")
    print(f"{'='*60}\n")

    os.makedirs(RESULTS_DIR, exist_ok=True)

    # ---- Config 1: Non-maneuvering target ----
    tgt_nom = TargetState(speed=50.0, heading_deg=180.0, altitude=500.0, g_load=0.0)
    print(f"  Config 1: Non-maneuvering target ({tgt_nom.speed}m/s, hdg={tgt_nom.heading_deg}°)")
    sweep_nom = sweep_range_bearing(
        tgt_nom,
        range_bounds=(500, 6000),
        bearing_bounds=(-180, 170),
        n_range=20, n_bearing=36,
        alt_diff=490.0, n_mc=3,
    )
    bounds_nom = extract_boundaries(sweep_nom, pk_nez=0.5, pk_fez=0.001)
    plot_polar_pk(sweep_nom, bounds_nom, title_suffix=' — Non-maneuvering',
                  save_dir=RESULTS_DIR)
    plot_miss_contour(sweep_nom, title_suffix=' — Non-maneuvering',
                      save_dir=RESULTS_DIR)

    # ---- Config 2: Maneuvering target (3g) ----
    tgt_man = TargetState(speed=50.0, heading_deg=180.0, altitude=500.0, g_load=3.0)
    print(f"\n  Config 2: Maneuvering target ({tgt_man.speed}m/s, {tgt_man.g_load}g)")
    sweep_man = sweep_range_bearing(
        tgt_man,
        range_bounds=(500, 6000),
        bearing_bounds=(-180, 170),
        n_range=20, n_bearing=36,
        alt_diff=490.0, n_mc=3,
    )
    bounds_man = extract_boundaries(sweep_man, pk_nez=0.5, pk_fez=0.001)
    plot_polar_pk(sweep_man, bounds_man, title_suffix=' — 3g maneuver',
                  save_dir=RESULTS_DIR)

    # ---- Comparison ----
    comparison = [
        (sweep_nom, bounds_nom, f'Non-maneuvering\n{tgt_nom.speed}m/s hdg={tgt_nom.heading_deg}°'),
        (sweep_man, bounds_man, f'3g maneuver\n{tgt_man.speed}m/s hdg={tgt_man.heading_deg}°'),
    ]
    plot_comparison(comparison, save_dir=RESULTS_DIR)

    # ---- Summary ----
    print(f"\n{'='*60}")
    print(f"  SUMMARY")
    print(f"{'='*60}")
    for label, sweep, bounds in [('Non-maneuvering', sweep_nom, bounds_nom),
                                  ('3g maneuver', sweep_man, bounds_man)]:
        max_fez = np.max(bounds['fez_range'])
        max_nez = np.max(bounds['nez_range'])
        mean_pk = np.mean(sweep['pk_grid'])
        max_pk  = np.max(sweep['pk_grid'])
        print(f"\n  {label}:")
        print(f"    Max FEZ range:  {max_fez:.0f}m")
        print(f"    Max NEZ range:  {max_nez:.0f}m")
        print(f"    Mean Pk (grid): {mean_pk:.4f}")
        print(f"    Max Pk:         {max_pk:.4f}")
    print(f"\n{'='*60}")