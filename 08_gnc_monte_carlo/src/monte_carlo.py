"""
monte_carlo.py -- Monte Carlo engagement analysis
Project 08 -- GNC/Monte Carlo

Runs 500 engagement simulations with randomized:
- Launch position offset
- Launch angle
- Target initial position and heading
- Target maneuver timing and direction
- Seeker noise realizations
- Wind

Outputs:
- Miss distance histogram with CEP
- PN variant comparison (PPN vs TPN vs APN)
- Launch envelope heatmap
- Pk vs launch range curve
- Miss distance sensitivity plots
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.colors import LogNorm
import os
import sys

_root = "D:/Weapons-systems-sim-pipeline"
sys.path.insert(0, os.path.join(_root, "08_gnc_monte_carlo/src"))

from engagement import run_engagement

RESULTS_DIR = "D:/Weapons-systems-sim-pipeline/08_gnc_monte_carlo/results"
FIGURES_DIR = os.path.join(RESULTS_DIR, "figures")
os.makedirs(FIGURES_DIR, exist_ok=True)

# ---------------------------------------------------------------
# Monte Carlo parameters
# ---------------------------------------------------------------
N_RUNS        = 500
LETHAL_RADIUS = 20.0    # m
DT            = 0.02   # s
T_MAX         = 60.0   # s

# Nominal engagement parameters
NOMINAL = dict(
    launch_pos          = [0, 0, 10.0],
    launch_angle_deg    = 20.0,
    launch_azimuth_deg  = 0.0,
    target_pos          = [4000.0, 300.0, 500.0],
    target_heading_deg  = 110.0,
    target_speed        = 50.0,
    target_max_accel_g  = 3.0,
    guidance_variant    = 'TPN',
    N                   = 5.0,
    noise_std           = 0.05,
    accel_limit_g       = 10.0,
    dt                  = DT,
    t_max               = T_MAX,
    lethal_radius       = LETHAL_RADIUS,
)

# ---------------------------------------------------------------
# Randomization ranges
# ---------------------------------------------------------------
RAND = dict(
    launch_pos_std      = 50.0,    # m, position error
    launch_angle_std    = 5.0,     # deg
    launch_az_std       = 5.0,     # deg
    target_pos_std      = 500.0,   # m, target position scatter
    target_heading_range= 360.0,   # deg, fully random heading
    wind_std            = 5.0,     # m/s lateral wind
)


def randomize_params(nominal, rng):
    """Generate randomized engagement parameters from nominal."""
    p = nominal.copy()

    # Launch position jitter
    p['launch_pos'] = [
        nominal['launch_pos'][0] + rng.normal(0, RAND['launch_pos_std']),
        nominal['launch_pos'][1] + rng.normal(0, RAND['launch_pos_std']),
        nominal['launch_pos'][2]
    ]

    # Launch angle jitter
    p['launch_angle_deg']   = nominal['launch_angle_deg']   + rng.normal(0, RAND['launch_angle_std'])
    p['launch_azimuth_deg'] = nominal['launch_azimuth_deg'] + rng.normal(0, RAND['launch_az_std'])

    # Target position scatter
    p['target_pos'] = [
        nominal['target_pos'][0] + rng.normal(0, RAND['target_pos_std']),
        nominal['target_pos'][1] + rng.normal(0, RAND['target_pos_std']),
        nominal['target_pos'][2] + rng.normal(0, 100.0)
    ]
    p['target_pos'][2] = max(p['target_pos'][2], 100.0)

    # Target heading — fully random
    p['target_heading_deg'] = rng.uniform(0, 360)

    # Wind
    p['wind'] = [
        rng.normal(0, RAND['wind_std']),
        rng.normal(0, RAND['wind_std']),
        0.0
    ]

    return p


# ---------------------------------------------------------------
# CEP calculation
# ---------------------------------------------------------------
def compute_cep(miss_distances, percentile=50):
    """Circular Error Probable — radius containing percentile% of misses."""
    return np.percentile(miss_distances, percentile)


# ---------------------------------------------------------------
# Main Monte Carlo run
# ---------------------------------------------------------------
def run_monte_carlo(n_runs=N_RUNS, nominal=None, seed=0, verbose=True):
    """
    Run Monte Carlo engagement analysis.

    Returns
    -------
    results : list of engagement result dicts
    miss_distances : array [n_runs]
    hit_flags : array [n_runs] bool
    """
    if nominal is None:
        nominal = NOMINAL

    rng = np.random.default_rng(seed)
    miss_distances = np.zeros(n_runs)
    hit_flags      = np.zeros(n_runs, dtype=bool)
    results        = []

    print(f"Running Monte Carlo ({n_runs} runs)...")
    for i in range(n_runs):
        if verbose and i % 50 == 0:
            print(f"  Run {i}/{n_runs}...")

        params = randomize_params(nominal, rng)
        params['seed'] = int(rng.integers(0, 1000000))

        result = run_engagement(**params)
        miss_distances[i] = result['miss_distance']
        hit_flags[i]      = result['hit']
        results.append(result)

    pk = hit_flags.mean() * 100
    cep50 = compute_cep(miss_distances, 50)
    cep90 = compute_cep(miss_distances, 90)
    cep95 = compute_cep(miss_distances, 95)

    print(f"\n=== Monte Carlo Results ({n_runs} runs) ===")
    print(f"  Pk (lethal radius {LETHAL_RADIUS}m): {pk:.1f}%")
    print(f"  CEP50:  {cep50:.1f} m")
    print(f"  CEP90:  {cep90:.1f} m")
    print(f"  CEP95:  {cep95:.1f} m")
    print(f"  Mean miss distance: {miss_distances.mean():.1f} m")
    print(f"  Std miss distance:  {miss_distances.std():.1f} m")
    print(f"  Min miss distance:  {miss_distances.min():.2f} m")
    print(f"  Max miss distance:  {miss_distances.max():.1f} m")

    return results, miss_distances, hit_flags


# ---------------------------------------------------------------
# Plot 1 — Miss distance histogram
# ---------------------------------------------------------------
def plot_miss_histogram(miss_distances, hit_flags, save=True):
    """Miss distance histogram with CEP annotations."""
    fig, ax = plt.subplots(figsize=(10, 6))

    cep50 = compute_cep(miss_distances, 50)
    cep90 = compute_cep(miss_distances, 90)
    cep95 = compute_cep(miss_distances, 95)
    pk    = hit_flags.mean() * 100

    bins = np.linspace(0, min(miss_distances.max(), 2000), 50)
    ax.hist(miss_distances[~hit_flags], bins=bins,
            color='steelblue', alpha=0.7, label='Miss')
    ax.hist(miss_distances[hit_flags], bins=bins,
            color='crimson', alpha=0.8, label='Hit (kill)')

    ax.axvline(LETHAL_RADIUS, color='red', lw=2, ls='--',
               label=f'Lethal radius {LETHAL_RADIUS}m')
    ax.axvline(cep50, color='orange', lw=1.5, ls='-.',
               label=f'CEP50 = {cep50:.1f}m')
    ax.axvline(cep90, color='darkorange', lw=1.5, ls=':',
               label=f'CEP90 = {cep90:.1f}m')
    ax.axvline(cep95, color='saddlebrown', lw=1.5, ls=':',
               label=f'CEP95 = {cep95:.1f}m')

    ax.set_xlabel('Miss distance (m)', fontsize=12)
    ax.set_ylabel('Count', fontsize=12)
    ax.set_title(f'Miss Distance Distribution — {len(miss_distances)} runs | '
                 f'Pk = {pk:.1f}%', fontsize=13)
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)
    plt.tight_layout()

    if save:
        plt.savefig(os.path.join(FIGURES_DIR, 'miss_distance_histogram.png'),
                    dpi=150, bbox_inches='tight')
        print("Saved: miss_distance_histogram.png")
    plt.show()
    plt.close()


# ---------------------------------------------------------------
# Plot 2 — PN variant comparison
# ---------------------------------------------------------------
def plot_pn_comparison(n_runs=100, seed=1, save=True):
    """Compare miss distance distributions for PPN, TPN, APN."""
    print("\nRunning PN variant comparison...")
    variants = ['PPN', 'TPN', 'APN']
    colors   = ['steelblue', 'crimson', 'forestgreen']
    results_by_variant = {}

    for variant in variants:
        print(f"  Running {variant}...")
        nominal_v = NOMINAL.copy()
        nominal_v['guidance_variant'] = variant
        _, miss_dist, hit_flags = run_monte_carlo(
            n_runs=n_runs, nominal=nominal_v, seed=seed, verbose=False)
        results_by_variant[variant] = (miss_dist, hit_flags)

    fig, axes = plt.subplots(1, 3, figsize=(15, 5), sharey=True)
    fig.suptitle('PN Variant Comparison — Miss Distance Distributions', fontsize=13)

    for ax, variant, color in zip(axes, variants, colors):
        miss_dist, hit_flags = results_by_variant[variant]
        cep50 = compute_cep(miss_dist, 50)
        pk    = hit_flags.mean() * 100
        bins  = np.linspace(0, min(miss_dist.max(), 200), 40)

        ax.hist(miss_dist[~hit_flags], bins=bins, color=color,
                alpha=0.6, label='Miss')
        ax.hist(miss_dist[hit_flags], bins=bins, color='crimson',
                alpha=0.8, label='Hit')
        ax.axvline(LETHAL_RADIUS, color='red', lw=1.5, ls='--')
        ax.axvline(cep50, color='orange', lw=1.5, ls='-.',
                   label=f'CEP50={cep50:.1f}m')
        ax.set_title(f'{variant}\nPk={pk:.1f}% | CEP50={cep50:.1f}m', fontsize=11)
        ax.set_xlabel('Miss distance (m)')
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

    axes[0].set_ylabel('Count')
    plt.tight_layout()

    if save:
        plt.savefig(os.path.join(FIGURES_DIR, 'pn_comparison.png'),
                    dpi=150, bbox_inches='tight')
        print("Saved: pn_comparison.png")
    plt.show()
    plt.close()
    return results_by_variant


# ---------------------------------------------------------------
# Plot 3 — Launch envelope heatmap
# ---------------------------------------------------------------
def plot_launch_envelope(save=True):
    """
    Miss distance heatmap vs launch range and elevation angle.
    Shows effective engagement zone.
    """
    print("\nRunning launch envelope sweep...")
    ranges = np.linspace(1000, 8000, 10)
    angles = np.linspace(10, 60, 8)
    miss_grid = np.zeros((len(angles), len(ranges)))
    pk_grid   = np.zeros((len(angles), len(ranges)))

    rng = np.random.default_rng(42)

    for i, angle in enumerate(angles):
        for j, rng_val in enumerate(ranges):
            # Target at specified range
            target_x = rng_val * np.cos(np.radians(angle)) * 0.8
            target_z = rng_val * np.sin(np.radians(angle)) * 0.6 + 100

            misses = []
            hits   = []
            for _ in range(10):  # 20 runs per grid point
                r = run_engagement(
                    launch_pos=[0, 0, 0],
                    launch_angle_deg=angle,
                    target_pos=[target_x,
                                rng.uniform(-200, 200),
                                max(target_z, 100)],
                    target_heading_deg=rng.uniform(0, 360),
                    target_speed=50.0,
                    target_max_accel_g=3.0,
                    guidance_variant='TPN',
                    N=4.0,
                    noise_std=0.05,
                    dt=DT,
                    t_max=T_MAX,
                    lethal_radius=LETHAL_RADIUS,
                    seed=int(rng.integers(0, 1000000))
                )
                misses.append(r['miss_distance'])
                hits.append(r['hit'])

            miss_grid[i, j] = np.median(misses)
            pk_grid[i, j]   = np.mean(hits) * 100

        print(f"  Angle {angle:.0f}° done")

    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    fig.suptitle('Launch Envelope Analysis', fontsize=13)

    # Miss distance heatmap
    im1 = axes[0].pcolormesh(ranges/1000, angles, miss_grid,
                              cmap='RdYlGn_r', shading='auto')
    plt.colorbar(im1, ax=axes[0], label='Median miss distance (m)')
    axes[0].set_xlabel('Launch range (km)')
    axes[0].set_ylabel('Launch elevation (deg)')
    axes[0].set_title('Median miss distance')
    axes[0].contour(ranges/1000, angles, miss_grid,
                    levels=[LETHAL_RADIUS], colors='white', linewidths=2)

    # Pk heatmap
    im2 = axes[1].pcolormesh(ranges/1000, angles, pk_grid,
                              cmap='RdYlGn', shading='auto', vmin=0, vmax=100)
    plt.colorbar(im2, ax=axes[1], label='Pk (%)')
    axes[1].set_xlabel('Launch range (km)')
    axes[1].set_ylabel('Launch elevation (deg)')
    axes[1].set_title('Probability of kill (%)')
    axes[1].contour(ranges/1000, angles, pk_grid,
                    levels=[50], colors='white', linewidths=2)

    plt.tight_layout()

    if save:
        plt.savefig(os.path.join(FIGURES_DIR, 'launch_envelope.png'),
                    dpi=150, bbox_inches='tight')
        print("Saved: launch_envelope.png")
    plt.show()
    plt.close()
    return miss_grid, pk_grid, ranges, angles


# ---------------------------------------------------------------
# Plot 4 — Pk vs launch range
# ---------------------------------------------------------------
def plot_pk_vs_range(save=True):
    """Probability of kill vs launch range curve."""
    print("\nRunning Pk vs range sweep...")
    ranges   = np.linspace(500, 9000, 18)
    pk_vals  = []
    cep_vals = []
    n_per    = 50

    rng = np.random.default_rng(99)

    for r in ranges:
        misses = []
        hits   = []
        for _ in range(n_per):
            result = run_engagement(
                launch_pos=[0, 0, 0],
                launch_angle_deg=30.0,
                target_pos=[r * 0.85,
                            rng.uniform(-300, 300),
                            rng.uniform(300, 700)],
                target_heading_deg=rng.uniform(0, 360),
                target_speed=50.0,
                target_max_accel_g=3.0,
                guidance_variant='TPN',
                N=4.0,
                noise_std=0.05,
                dt=DT,
                t_max=T_MAX,
                lethal_radius=LETHAL_RADIUS,
                seed=int(rng.integers(0, 1000000))
            )
            misses.append(result['miss_distance'])
            hits.append(result['hit'])

        pk_vals.append(np.mean(hits) * 100)
        cep_vals.append(np.percentile(misses, 50))
        print(f"  Range {r:.0f}m: Pk={pk_vals[-1]:.1f}%")

    fig, ax1 = plt.subplots(figsize=(10, 6))
    ax2 = ax1.twinx()

    ax1.plot(ranges/1000, pk_vals, 'b-o', lw=2, ms=5, label='Pk (%)')
    ax1.axhline(50, color='blue', ls='--', lw=1, alpha=0.5, label='Pk=50%')
    ax1.set_xlabel('Launch range (km)', fontsize=12)
    ax1.set_ylabel('Probability of kill (%)', color='blue', fontsize=12)
    ax1.set_ylim(0, 105)
    ax1.tick_params(axis='y', labelcolor='blue')

    ax2.plot(ranges/1000, cep_vals, 'r-s', lw=2, ms=5, label='CEP50 (m)')
    ax2.axhline(LETHAL_RADIUS, color='red', ls='--', lw=1, alpha=0.5,
                label=f'Lethal radius {LETHAL_RADIUS}m')
    ax2.set_ylabel('CEP50 (m)', color='red', fontsize=12)
    ax2.tick_params(axis='y', labelcolor='red')

    ax1.set_title('Probability of Kill vs Launch Range\n'
                  f'TPN N=5, target 50m/s 3g, noise σ=0.05 rad/s', fontsize=12)
    ax1.grid(True, alpha=0.3)

    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax1.legend(lines1 + lines2, labels1 + labels2, fontsize=9, loc='center right')

    plt.tight_layout()

    if save:
        plt.savefig(os.path.join(FIGURES_DIR, 'pk_curve.png'),
                    dpi=150, bbox_inches='tight')
        print("Saved: pk_curve.png")
    plt.show()
    plt.close()
    return ranges, pk_vals, cep_vals


# ---------------------------------------------------------------
# Plot 5 — Sensitivity analysis
# ---------------------------------------------------------------
def plot_sensitivity(save=True):
    """Miss distance sensitivity to key parameters."""
    print("\nRunning sensitivity analysis...")
    n_per = 50
    rng   = np.random.default_rng(7)

    # N sensitivity
    N_vals    = [2, 3, 4, 5, 6]
    N_medians = []
    N_p90     = []
    for N in N_vals:
        nom = NOMINAL.copy()
        nom['N'] = N
        _, miss, _ = run_monte_carlo(n_per, nominal=nom,
                                     seed=int(rng.integers(0, 1e6)),
                                     verbose=False)
        N_medians.append(np.median(miss))
        N_p90.append(np.percentile(miss, 90))

    # Target acceleration sensitivity
    accel_vals    = [1, 2, 3, 4, 5]
    accel_medians = []
    accel_p90     = []
    for a in accel_vals:
        nom = NOMINAL.copy()
        nom['target_max_accel_g'] = a
        _, miss, _ = run_monte_carlo(n_per, nominal=nom,
                                     seed=int(rng.integers(0, 1e6)),
                                     verbose=False)
        accel_medians.append(np.median(miss))
        accel_p90.append(np.percentile(miss, 90))

    # Noise sensitivity
    noise_vals    = [0.01, 0.05, 0.1, 0.2, 0.5]
    noise_medians = []
    noise_p90     = []
    for ns in noise_vals:
        nom = NOMINAL.copy()
        nom['noise_std'] = ns
        _, miss, _ = run_monte_carlo(n_per, nominal=nom,
                                     seed=int(rng.integers(0, 1e6)),
                                     verbose=False)
        noise_medians.append(np.median(miss))
        noise_p90.append(np.percentile(miss, 90))

    fig, axes = plt.subplots(1, 3, figsize=(15, 5))
    fig.suptitle('Miss Distance Sensitivity Analysis', fontsize=13)

    for ax, x_vals, medians, p90s, xlabel, title in [
        (axes[0], N_vals, N_medians, N_p90,
         'Navigation constant N', 'vs Navigation Constant'),
        (axes[1], accel_vals, accel_medians, accel_p90,
         'Target max accel (g)', 'vs Target Maneuverability'),
        (axes[2], noise_vals, noise_medians, noise_p90,
         'Seeker noise σ (rad/s)', 'vs Seeker Noise'),
    ]:
        ax.plot(x_vals, medians, 'b-o', lw=2, ms=6, label='Median')
        ax.plot(x_vals, p90s, 'r--s', lw=1.5, ms=5, label='90th pctile')
        ax.axhline(LETHAL_RADIUS, color='green', ls=':', lw=1.5,
                   label=f'Lethal radius {LETHAL_RADIUS}m')
        ax.set_xlabel(xlabel, fontsize=10)
        ax.set_ylabel('Miss distance (m)', fontsize=10)
        ax.set_title(f'Miss distance {title}', fontsize=10)
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

    plt.tight_layout()

    if save:
        plt.savefig(os.path.join(FIGURES_DIR, 'sensitivity.png'),
                    dpi=150, bbox_inches='tight')
        print("Saved: sensitivity.png")
    plt.show()
    plt.close()


# ---------------------------------------------------------------
# ECM scenario
# ---------------------------------------------------------------
def plot_ecm_comparison(n_runs=100, save=True):
    """Compare nominal vs ECM (5x seeker noise) miss distributions."""
    print("\nRunning ECM comparison...")

    nom_baseline = NOMINAL.copy()
    nom_ecm      = NOMINAL.copy()
    nom_ecm['noise_std'] = NOMINAL['noise_std'] * 5.0

    _, miss_baseline, hits_baseline = run_monte_carlo(
        n_runs, nominal=nom_baseline, seed=10, verbose=False)
    _, miss_ecm, hits_ecm = run_monte_carlo(
        n_runs, nominal=nom_ecm, seed=10, verbose=False)

    fig, ax = plt.subplots(figsize=(10, 6))
    bins = np.linspace(0, min(max(miss_baseline.max(), miss_ecm.max()), 300), 50)

    ax.hist(miss_baseline, bins=bins, alpha=0.6, color='steelblue',
            label=f'Nominal σ={NOMINAL["noise_std"]} rad/s | '
                  f'Pk={hits_baseline.mean()*100:.1f}%')
    ax.hist(miss_ecm, bins=bins, alpha=0.6, color='crimson',
            label=f'ECM σ={nom_ecm["noise_std"]:.2f} rad/s | '
                  f'Pk={hits_ecm.mean()*100:.1f}%')
    ax.axvline(LETHAL_RADIUS, color='black', lw=2, ls='--',
               label=f'Lethal radius {LETHAL_RADIUS}m')
    ax.set_xlabel('Miss distance (m)', fontsize=12)
    ax.set_ylabel('Count', fontsize=12)
    ax.set_title('ECM Robustness — Nominal vs Jammed Seeker', fontsize=12)
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)
    plt.tight_layout()

    if save:
        plt.savefig(os.path.join(FIGURES_DIR, 'ecm_comparison.png'),
                    dpi=150, bbox_inches='tight')
        print("Saved: ecm_comparison.png")
    plt.show()
    plt.close()


# ---------------------------------------------------------------
# Main
# ---------------------------------------------------------------
if __name__ == "__main__":
    print("=== Project 08 — GNC Monte Carlo Analysis ===\n")

    # 1. Main Monte Carlo
    results, miss_distances, hit_flags = run_monte_carlo(
        n_runs=N_RUNS, seed=0)
    plot_miss_histogram(miss_distances, hit_flags)

    # Save raw results
    np.save(os.path.join(RESULTS_DIR, 'miss_distances.npy'), miss_distances)
    np.save(os.path.join(RESULTS_DIR, 'hit_flags.npy'), hit_flags)
    print("Raw results saved.")

    # 2. PN variant comparison
    plot_pn_comparison(n_runs=200)

    # 3. Launch envelope
    plot_launch_envelope()

    # 4. Pk vs range
    plot_pk_vs_range()

    # 5. Sensitivity
    plot_sensitivity()

    # 6. ECM
    plot_ecm_comparison()

    print("\n=== All analyses complete ===")