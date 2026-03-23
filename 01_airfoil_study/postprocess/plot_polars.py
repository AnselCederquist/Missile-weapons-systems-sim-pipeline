import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import os

# ── Configuration ──────────────────────────────────────────────────────────────
# os.path.dirname(__file__) gets the directory this script lives in
# We then navigate up one level (..) to get to 01_airfoil_study/
# and then into results/data/ and results/figures/
# This makes the paths work regardless of where you run the script from
DATA_DIR = os.path.join(os.path.dirname(__file__), '..', 'results', 'data')
FIG_DIR  = os.path.join(os.path.dirname(__file__), '..', 'results', 'figures')

# All airfoils, Reynolds numbers, and Mach numbers to analyze
AIRFOIL_NAMES = ['NACA 0006', 'NACA 0008', 'NACA 0012', 'NACA 0015']
RE_VALUES     = [0.500, 1.000, 2.000]   # in millions (e.g. 0.500 = Re 500,000)
MACH_VALUES   = [0.30, 0.50]

# Build the full file list automatically from all combinations
AIRFOILS = {}
for foil in AIRFOIL_NAMES:
    for re in RE_VALUES:
        for mach in MACH_VALUES:
            label    = f"{foil} Re={re:.3f}M M={mach:.2f}"
            filename = f"{foil}_T1_Re{re:.3f}_M{mach:.2f}_N9.0.txt"
            AIRFOILS[label] = filename

# Plot colors for each airfoil — one color per airfoil, consistent across all plots
# Using hex codes for precise control over appearance
COLORS = ['#e63946', '#f4a261', '#2a9d8f', '#457b9d']

# CD threshold below which data is considered unreliable (e.g. NACA 0006 at low Re)
# XFLR5 sometimes reports CD=0 when it cannot resolve the boundary layer on thin airfoils
# Filtering these out prevents divide-by-zero and inf values in L/D calculations
CD_MIN = 0.001


# ── Helper: compute valid L/D ─────────────────────────────────────────────────
def compute_ld(df):
    """
    Compute lift-to-drag ratio, filtering out rows where CD is unreliably small.
    Returns a filtered DataFrame and the corresponding L/D Series.

    Filtering CD < CD_MIN removes:
      - NACA 0006 zero-CD artifacts at low Reynolds number
      - Any other unphysical zero-drag points from unconverged solutions
    """
    df_valid = df[df['CD'] > CD_MIN].copy()
    ld = df_valid['CL'] / df_valid['CD']
    return df_valid, ld


# ── Parser ─────────────────────────────────────────────────────────────────────
def parse_xflr5_polar(filepath):
    """
    Parse an XFLR5 exported polar text file into a pandas DataFrame.

    XFLR5 exports polars as a .txt file with a header block containing
    run conditions (Re, Mach, NCrit), followed by column headers, a dashes
    separator line, and then the numeric data rows.

    This function:
    1. Reads all lines from the file
    2. Finds the header row by looking for a line containing both 'alpha' and 'CL'
    3. Skips the dashes separator line below the header
    4. Parses all remaining numeric rows into a DataFrame

    Parameters
    ----------
    filepath : str
        Full path to the XFLR5 exported polar .txt file

    Returns
    -------
    pd.DataFrame
        DataFrame with columns: alpha, CL, CD, CDp, Cm, Top_Xtr, Bot_Xtr, etc.
        (exact columns depend on XFLR5 export settings)
    """
    with open(filepath, 'r') as f:
        lines = f.readlines()

    # Search line by line for the data header row
    # XFLR5 always includes 'alpha' and 'CL' in the column header line
    header_idx = None
    for i, line in enumerate(lines):
        if 'alpha' in line.lower() and 'CL' in line:
            header_idx = i
            break

    if header_idx is None:
        raise ValueError(f"Could not find data header in {filepath}")

    # Split the header line into individual column name strings
    headers = lines[header_idx].split()

    # Parse the numeric data rows
    # header_idx + 1 is the dashes line (------), so we skip to + 2
    data = []
    for line in lines[header_idx + 2:]:
        line = line.strip()
        if line:  # skip blank lines
            try:
                # Convert each space-separated value to float and store as a row
                data.append([float(x) for x in line.split()])
            except ValueError:
                # Stop parsing if we hit a non-numeric line (end of data block)
                break

    # Build DataFrame — only use as many header names as there are data columns
    df = pd.DataFrame(data, columns=headers[:len(data[0])])
    return df


# ── Load all polars ────────────────────────────────────────────────────────────
# Loop through each airfoil, build the full file path, and parse it
# Store each DataFrame in a dict keyed by airfoil display name
polars = {}
for name, filename in AIRFOILS.items():
    filepath = os.path.join(DATA_DIR, filename)
    try:
        polars[name] = parse_xflr5_polar(filepath)
        print(f"Loaded {name}: {len(polars[name])} points")
    except Exception as e:
        # If a file is missing or malformed, warn but continue with the rest
        print(f"Warning: could not load {name}: {e}")


# ── Plot 1: CL vs Alpha (Lift Curve) ──────────────────────────────────────────
# The lift curve shows how lift coefficient (CL) increases with angle of attack (alpha)
# Key features to observe:
#   - Linear region: CL increases linearly from ~-10 to ~10 degrees
#   - CL slope (dCL/dalpha): roughly 0.1/deg for all symmetric NACA foils (thin airfoil theory)
#   - CLmax: peak CL before stall — thicker airfoils tend to have slightly higher CLmax
#   - Stall angle: where CL drops — thinner airfoils stall more abruptly
fig, ax = plt.subplots(figsize=(8, 6))
for (name, df), color in zip(polars.items(), COLORS):
    ax.plot(df['alpha'], df['CL'], label=name, color=color, linewidth=2)

ax.set_xlabel('Angle of Attack α (deg)', fontsize=12)
ax.set_ylabel('Lift Coefficient CL', fontsize=12)
ax.set_title('CL vs α — NACA Thickness Comparison\nAll Re and Mach conditions', fontsize=13)
ax.legend(fontsize=11)
ax.grid(True, alpha=0.3)
ax.axhline(0, color='white', linewidth=0.5)   # zero-lift reference line
ax.axvline(0, color='white', linewidth=0.5)   # zero-AoA reference line
plt.tight_layout()
plt.savefig(os.path.join(FIG_DIR, 'CL_vs_Alpha_all.png'), dpi=150)
plt.show()
print("Saved: CL_vs_Alpha_all.png")


# ── Plot 2: Drag Polar (CL vs CD) ─────────────────────────────────────────────
# The drag polar plots lift (CL) on the Y axis against drag (CD) on the X axis
# This is the most important efficiency plot — a curve further LEFT means less
# drag for the same lift (more aerodynamically efficient)
# Key features:
#   - Thinner airfoils (0006, 0008) sit further left = lower drag
#   - The "bucket" shape shows the low-drag laminar flow region
#   - Missile canards need enough CL for control authority with minimum drag penalty
fig, ax = plt.subplots(figsize=(8, 6))
for (name, df), color in zip(polars.items(), COLORS):
    ax.plot(df['CD'], df['CL'], label=name, color=color, linewidth=2)

ax.set_xlabel('Drag Coefficient CD', fontsize=12)
ax.set_ylabel('Lift Coefficient CL', fontsize=12)
ax.set_title('Drag Polar — NACA Thickness Comparison\nAll Re and Mach conditions', fontsize=13)
ax.legend(fontsize=11)
ax.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig(os.path.join(FIG_DIR, 'drag_polar_all.png'), dpi=150)
plt.show()
print("Saved: drag_polar_all.png")


# ── Plot 3: L/D vs Alpha (Aerodynamic Efficiency) ────────────────────────────
# Lift-to-Drag ratio (L/D) is the single best measure of aerodynamic efficiency
# Higher L/D = more lift generated per unit of drag = more efficient
# For a missile canard:
#   - Peak L/D angle tells you the most efficient operating AoA
#   - Higher peak L/D = less drag penalty for a given maneuver demand
#   - Thinner airfoils generally have higher peak L/D at low AoA
# Note: L/D is undefined at AoA=0 for symmetric airfoils (CL=0, so 0/CD = 0)
# Note: CD values below CD_MIN are filtered out to remove NACA 0006 zero-CD artifacts
fig, ax = plt.subplots(figsize=(8, 6))
for (name, df), color in zip(polars.items(), COLORS):
    # Filter unreliable low-CD points before computing L/D
    df_valid, ld = compute_ld(df)
    ax.plot(df_valid['alpha'], ld, label=name, color=color, linewidth=2)

ax.set_xlabel('Angle of Attack α (deg)', fontsize=12)
ax.set_ylabel('Lift-to-Drag Ratio L/D', fontsize=12)
ax.set_title('L/D vs α — NACA Thickness Comparison\nAll Re and Mach conditions', fontsize=13)
ax.legend(fontsize=11)
ax.grid(True, alpha=0.3)
ax.set_xlim(-5, 15)   # crop x-axis to the useful operating range
ax.set_ylim(-5, 100)  # cap y-axis to prevent inf values distorting the plot
plt.tight_layout()
plt.savefig(os.path.join(FIG_DIR, 'LD_vs_Alpha_all.png'), dpi=150)
plt.show()
print("Saved: LD_vs_Alpha_all.png")


# ── Summary Table ─────────────────────────────────────────────────────────────
# Print a concise numerical summary of key aerodynamic metrics for each airfoil
# This table is what goes into the technical report and README key results section
#
# Metrics explained:
#   CLmax       — maximum lift coefficient before stall
#   Stall AoA   — angle of attack at CLmax (stall onset)
#   Max L/D     — peak lift-to-drag ratio (best efficiency point)
#   CD @ CL=0.5 — drag at a representative cruise/maneuvering lift condition
print("\n── Airfoil Summary ───────────────────────────────")
print(f"{'Polar':<35} {'CLmax':>8} {'Stall AoA':>10} {'Max L/D':>10} {'CD@CL=0.5':>10}")
print("-" * 75)
for name, df in polars.items():
    # Find maximum CL value across all analyzed angles of attack
    cl_max = df['CL'].max()

    # Find the angle of attack at which CLmax occurs (stall onset)
    stall_aoa = df.loc[df['CL'].idxmax(), 'alpha']

    # Filter out zero-CD rows, then find peak L/D
    df_valid, ld = compute_ld(df)
    max_ld = ld.max() if len(ld) > 0 else float('nan')

    # Find the data point where CL is closest to 0.5 (representative maneuver condition)
    # then read off the drag coefficient at that point
    idx = (df['CL'] - 0.5).abs().idxmin()
    cd_at_cl05 = df.loc[idx, 'CD']

    print(f"{name:<35} {cl_max:>8.3f} {stall_aoa:>10.1f} {max_ld:>10.1f} {cd_at_cl05:>10.4f}")


# ── Focused plots: one per Mach number, all airfoils, all Re ─────────────────
# This produces cleaner comparison plots by fixing Mach and varying Re
# Shows how each airfoil's performance scales with Reynolds number
# Color = airfoil family, linestyle = Reynolds number

for mach in MACH_VALUES:
    fig, axes = plt.subplots(1, 3, figsize=(18, 6))
    fig.suptitle(f'NACA Airfoil Comparison — Mach {mach:.2f}', fontsize=14)

    # One color per airfoil, one linestyle per Re
    foil_colors   = {'NACA 0006': '#e63946', 'NACA 0008': '#f4a261',
                     'NACA 0012': '#2a9d8f', 'NACA 0015': '#457b9d'}
    re_linestyles = {0.500: '-', 1.000: '--', 2.000: ':'}
    re_labels     = {0.500: 'Re=500k', 1.000: 'Re=1M', 2.000: 'Re=2M'}

    for foil in AIRFOIL_NAMES:
        for re in RE_VALUES:
            label = f"{foil} Re={re:.3f}M M={mach:.2f}"
            if label not in polars:
                continue
            df           = polars[label]
            color        = foil_colors[foil]
            ls           = re_linestyles[re]
            legend_label = f"{foil} {re_labels[re]}"

            # CL vs Alpha — all data points valid
            axes[0].plot(df['alpha'], df['CL'], color=color,
                         linestyle=ls, linewidth=1.5, label=legend_label)

            # Drag polar — all data points valid
            axes[1].plot(df['CD'], df['CL'], color=color,
                         linestyle=ls, linewidth=1.5, label=legend_label)

            # L/D vs Alpha — filter zero-CD rows before computing
            df_valid, ld = compute_ld(df)
            axes[2].plot(df_valid['alpha'], ld, color=color,
                         linestyle=ls, linewidth=1.5, label=legend_label)

    axes[0].set_xlabel('Alpha (deg)'); axes[0].set_ylabel('CL')
    axes[0].set_title('CL vs Alpha');  axes[0].grid(True, alpha=0.3)

    axes[1].set_xlabel('CD');          axes[1].set_ylabel('CL')
    axes[1].set_title('Drag Polar');   axes[1].grid(True, alpha=0.3)

    axes[2].set_xlabel('Alpha (deg)'); axes[2].set_ylabel('L/D')
    axes[2].set_title('L/D vs Alpha'); axes[2].grid(True, alpha=0.3)
    axes[2].set_xlim(-5, 15)
    axes[2].set_ylim(-5, 100)          # cap y-axis to prevent inf distorting plot

    # Single legend for the whole figure
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc='lower center', ncol=6,
               fontsize=8, bbox_to_anchor=(0.5, -0.05))

    plt.tight_layout()
    fname = f'comparison_M{str(mach).replace(".", "")}_all_airfoils.png'
    plt.savefig(os.path.join(FIG_DIR, fname), dpi=150, bbox_inches='tight')
    plt.show()
    print(f"Saved: {fname}")