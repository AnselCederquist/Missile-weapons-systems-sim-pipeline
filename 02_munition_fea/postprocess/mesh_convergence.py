import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import os

# ── Configuration ──────────────────────────────────────────────────────────────
# Navigate from postprocess/ up to 02_munition_fea/ then into results/figures/
FIG_DIR = os.path.join(os.path.dirname(__file__), '..', 'results', 'figures')
os.makedirs(FIG_DIR, exist_ok=True)

# ── Mesh Convergence Data ──────────────────────────────────────────────────────
# Data recorded from Ansys Mechanical Static Structural analysis
# Geometry: Ti-6Al-4V missile fin assembly under 10,000g axial setback load
# Stress reported at negative-X inner fillet (R3 radius, fin-to-tab transition)
# Peak stress at fixed support excluded — identified as BC-induced singularity

data = {
    'element_size_mm':   [3.0,    2.0,    1.0,    0.9,    0.85],
    'element_count':     [960,    2050,   15088,  23660,  26400],
    'node_count':        [5621,   11808,  73469,  111278, 124018],
    'von_mises_MPa':     [1.532,  1.382,  1.289,  1.336,  1.378],
    'deformation_mm':    [0.001292, 0.001339, 0.001362, 0.001362, 0.001363],
    'eq_quality_min':    [0.21043, 0.05190, 0.03360, 0.03850, 0.03880],
    'eq_quality_avg':    [0.88574, 0.81177, 0.82863, 0.81461, 0.82994],
    'skewness_avg':      [0.10680, 0.21332, 0.19467, 0.19924, 0.19387],
    'solve_time_s':      [16.604, 16.643, 28.741, 32.788, 34.833],
}

df = pd.DataFrame(data)

# Characteristic mesh size h = element size (mm)
# Used as x-axis for convergence plots
h = df['element_size_mm'].values

# ── Material Properties ────────────────────────────────────────────────────────
# Ti-6Al-4V (Grade 5 Titanium) — common aerospace/defense alloy
# Source: Ansys Granta Materials Library
YIELD_STRENGTH_MPA  = 880.0    # 0.2% proof stress, MPa
TENSILE_UTS_MPA     = 950.0    # Ultimate tensile strength, MPa
ELASTIC_MODULUS_GPA = 113.8    # Young's modulus, GPa
DENSITY_KG_M3       = 4430.0   # kg/m³
MATERIAL_NAME       = 'Ti-6Al-4V'

# ── Load Case ─────────────────────────────────────────────────────────────────
# Artillery/rocket launch setback load
G_LOAD       = 10000           # g-load during setback
G_MS2        = 9.81            # m/s²
ACCELERATION = G_LOAD * G_MS2  # 98,100 m/s²

print("=" * 60)
print("PROJECT 02 — FIN ASSEMBLY FEA CONVERGENCE ANALYSIS")
print("=" * 60)
print(f"Material:      {MATERIAL_NAME}")
print(f"Load:          {G_LOAD:,}g setback ({ACCELERATION:,.0f} m/s²)")
print(f"Yield strength:{YIELD_STRENGTH_MPA} MPa")
print()

# ── Print Convergence Table ────────────────────────────────────────────────────
print("── Mesh Convergence Data ──────────────────────────────────────────")
print(f"{'Size(mm)':>9} {'Elements':>10} {'Nodes':>8} {'σ_vm(MPa)':>11} {'δ_max(mm)':>11} {'Solve(s)':>9}")
print("-" * 62)
for _, row in df.iterrows():
    print(f"{row['element_size_mm']:>9.2f} {int(row['element_count']):>10,} "
          f"{int(row['node_count']):>8,} {row['von_mises_MPa']:>11.4f} "
          f"{row['deformation_mm']:>11.6f} {row['solve_time_s']:>9.3f}")
print()

# ── Richardson Extrapolation ───────────────────────────────────────────────────
# Richardson extrapolation estimates the grid-independent (h→0) solution
# Uses the three finest mesh levels for best accuracy
# Formula: f_exact ≈ f_1 + (f_1 - f_2) / (r^p - 1)
# where r = mesh refinement ratio, p = observed order of convergence
#
# NOTE: Stress is oscillating (~1.29-1.38 MPa) due to low quality elements
# near the critical zone — Richardson extrapolation applied to deformation
# which converged cleanly, and used as a secondary check on stress.

print("── Richardson Extrapolation (Deformation) ─────────────────────────")

# Use three finest meshes for Richardson
h1, h2, h3 = h[-1], h[-2], h[-3]        # 0.85, 0.90, 1.00 mm
f1 = df['deformation_mm'].values[-1]     # finest mesh result
f2 = df['deformation_mm'].values[-2]
f3 = df['deformation_mm'].values[-3]

# Refinement ratio
r21 = h2 / h1   # ratio between mesh 2 and mesh 1
r32 = h3 / h2   # ratio between mesh 3 and mesh 2

# Observed order of convergence p
# Solve iteratively: p = ln((f3-f2)/(f2-f1)) / ln(r)
# Use average refinement ratio for simplicity
r_avg = (r21 + r32) / 2
eps32 = f3 - f2
eps21 = f2 - f1

if abs(eps21) > 1e-12:
    p_deform = abs(np.log(abs(eps32 / eps21)) / np.log(r_avg))
else:
    p_deform = 2.0  # default to 2nd order if differences too small

# Richardson extrapolated value (grid-independent estimate)
f_rich_deform = f1 + (f1 - f2) / (r21**p_deform - 1)

# Grid Convergence Index (GCI) — measure of discretization error
# GCI < 1% indicates well-converged solution
Fs = 1.25   # safety factor for GCI (standard value for 3+ meshes)
GCI_deform = Fs * abs(eps21) / (r21**p_deform - 1) / abs(f1) * 100  # percent

print(f"Three finest meshes: h = {h3:.2f}, {h2:.2f}, {h1:.2f} mm")
print(f"Observed order of convergence p = {p_deform:.3f}")
print(f"Richardson extrapolated deformation = {f_rich_deform:.6f} mm")
print(f"Grid Convergence Index (GCI) = {GCI_deform:.4f}%")
print(f"Finest mesh result = {f1:.6f} mm")
print(f"Difference from extrapolated = {abs(f1-f_rich_deform)/f_rich_deform*100:.4f}%")
print()

# Apply same approach to stress (noting oscillation caveat)
print("── Richardson Extrapolation (Von Mises Stress) ─────────────────────")
s1 = df['von_mises_MPa'].values[-1]
s2 = df['von_mises_MPa'].values[-2]
s3 = df['von_mises_MPa'].values[-3]

eps32_s = s3 - s2
eps21_s = s2 - s1

if abs(eps21_s) > 1e-8:
    p_stress = abs(np.log(abs(eps32_s / eps21_s)) / np.log(r_avg))
else:
    p_stress = 2.0

s_rich = s1 + (s1 - s2) / (r21**p_stress - 1)
GCI_stress = Fs * abs(eps21_s) / (r21**p_stress - 1) / abs(s1) * 100

print(f"Observed order of convergence p = {p_stress:.3f}")
print(f"Richardson extrapolated stress  = {s_rich:.4f} MPa")
print(f"Grid Convergence Index (GCI)    = {GCI_stress:.4f}%")
print(f"NOTE: Stress oscillating due to low-quality elements near fillet.")
print(f"      Report stress as {df['von_mises_MPa'].values[2:].mean():.3f} ± "
      f"{df['von_mises_MPa'].values[2:].std():.3f} MPa (mean ± 1σ, finest 3 meshes)")
print()

# ── Safety Factor Calculations ─────────────────────────────────────────────────
# Safety factor = material strength / applied stress
# Report against both yield and UTS
# Use Richardson extrapolated stress as best estimate

print("── Safety Factor Analysis ──────────────────────────────────────────")

# Use mean of finest 3 meshes as reported stress (accounts for oscillation)
sigma_reported = df['von_mises_MPa'].values[2:].mean()
sigma_rich     = abs(s_rich)

SF_yield_reported = YIELD_STRENGTH_MPA / sigma_reported
SF_uts_reported   = TENSILE_UTS_MPA   / sigma_reported
SF_yield_rich     = YIELD_STRENGTH_MPA / sigma_rich
SF_uts_rich       = TENSILE_UTS_MPA   / sigma_rich

print(f"Reported stress (mean finest 3):     {sigma_reported:.4f} MPa")
print(f"Richardson extrapolated stress:      {sigma_rich:.4f} MPa")
print()
print(f"Safety factor vs yield  (reported):  {SF_yield_reported:.1f}×")
print(f"Safety factor vs UTS    (reported):  {SF_uts_reported:.1f}×")
print(f"Safety factor vs yield  (Richardson):{SF_yield_rich:.1f}×")
print(f"Safety factor vs UTS    (Richardson):{SF_uts_rich:.1f}×")
print()
print(f"NOTE: Safety factors >> 1 indicate the fin is significantly")
print(f"      overdesigned for this load case. Real missile fins are")
print(f"      optimized for minimum weight — further geometry refinement")
print(f"      (thinner fin, reduced tab size) would be appropriate.")
print()

# ── Plot 1: Von Mises Stress Convergence ───────────────────────────────────────
# Shows how peak stress changes with mesh refinement
# Oscillating behavior indicates mesh quality influence near critical zone
fig, ax = plt.subplots(figsize=(8, 5))

ax.plot(h, df['von_mises_MPa'], 'o-', color='#e63946', linewidth=2,
        markersize=8, markerfacecolor='white', markeredgewidth=2,
        label='Von Mises stress at inner fillet')

# Richardson extrapolated value as horizontal dashed line
ax.axhline(abs(s_rich), color='#e63946', linestyle='--', linewidth=1.5,
           alpha=0.7, label=f'Richardson extrapolated: {abs(s_rich):.3f} MPa')

# Yield strength reference (way off scale but mark it)
ax.set_xlabel('Element Size (mm)', fontsize=12)
ax.set_ylabel('Von Mises Stress (MPa)', fontsize=12)
ax.set_title('Mesh Convergence — Von Mises Stress\nFin Assembly Root Fillet, 10,000g Setback Load', fontsize=13)
ax.legend(fontsize=10)
ax.grid(True, alpha=0.3)
ax.invert_xaxis()   # coarse → fine left to right

# Annotate the oscillation
ax.annotate('Oscillation due to\nlow-quality elements\nnear fillet',
            xy=(1.0, 1.289), xytext=(1.8, 1.18),
            arrowprops=dict(arrowstyle='->', color='gray'),
            fontsize=9, color='gray')

plt.tight_layout()
plt.savefig(os.path.join(FIG_DIR, 'convergence_von_mises.png'), dpi=150)
plt.show()
print("Saved: convergence_von_mises.png")

# ── Plot 2: Deformation Convergence ───────────────────────────────────────────
fig, ax = plt.subplots(figsize=(8, 5))

ax.plot(h, df['deformation_mm'] * 1000, 's-', color='#2a9d8f', linewidth=2,
        markersize=8, markerfacecolor='white', markeredgewidth=2,
        label='Max total deformation')

ax.axhline(f_rich_deform * 1000, color='#2a9d8f', linestyle='--', linewidth=1.5,
           alpha=0.7, label=f'Richardson extrapolated: {f_rich_deform*1000:.4f} μm')

ax.set_xlabel('Element Size (mm)', fontsize=12)
ax.set_ylabel('Max Total Deformation (μm)', fontsize=12)
ax.set_title('Mesh Convergence — Total Deformation\nFin Assembly, 10,000g Setback Load', fontsize=13)
ax.legend(fontsize=10)
ax.grid(True, alpha=0.3)
ax.invert_xaxis()

# Annotate convergence
ax.annotate(f'GCI = {GCI_deform:.3f}%\n(converged)',
            xy=(1.0, f_rich_deform * 1000),
            xytext=(2.0, (df['deformation_mm'].values[0]) * 1000 * 0.998),
            arrowprops=dict(arrowstyle='->', color='gray'),
            fontsize=9, color='gray')

plt.tight_layout()
plt.savefig(os.path.join(FIG_DIR, 'convergence_deformation.png'), dpi=150)
plt.show()
print("Saved: convergence_deformation.png")

# ── Plot 3: Element Count vs Solve Time ───────────────────────────────────────
# Shows computational cost scaling with mesh refinement
fig, ax = plt.subplots(figsize=(8, 5))

ax.plot(df['element_count'], df['solve_time_s'], 'd-', color='#457b9d',
        linewidth=2, markersize=8, markerfacecolor='white', markeredgewidth=2)

for _, row in df.iterrows():
    ax.annotate(f"{row['element_size_mm']}mm",
                xy=(row['element_count'], row['solve_time_s']),
                xytext=(5, 5), textcoords='offset points', fontsize=9)

ax.set_xlabel('Element Count', fontsize=12)
ax.set_ylabel('Solve Time (s)', fontsize=12)
ax.set_title('Computational Cost vs Mesh Refinement\nAnsys Student License (128k node limit)', fontsize=13)
ax.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig(os.path.join(FIG_DIR, 'solve_time_vs_elements.png'), dpi=150)
plt.show()
print("Saved: solve_time_vs_elements.png")

# ── Plot 4: Mesh Quality Distribution ─────────────────────────────────────────
# Shows element quality metrics across mesh levels
fig, axes = plt.subplots(1, 2, figsize=(12, 5))

# Element quality average across mesh levels
axes[0].plot(h, df['eq_quality_avg'], 'o-', color='#2a9d8f', linewidth=2,
             markersize=8, label='Average quality')
axes[0].axhline(0.7, color='red', linestyle='--', linewidth=1,
                alpha=0.7, label='Minimum acceptable (0.7)')
axes[0].set_xlabel('Element Size (mm)', fontsize=12)
axes[0].set_ylabel('Element Quality', fontsize=12)
axes[0].set_title('Average Element Quality vs Mesh Size', fontsize=12)
axes[0].legend(fontsize=10)
axes[0].grid(True, alpha=0.3)
axes[0].invert_xaxis()
axes[0].set_ylim(0, 1)

# Skewness average
axes[1].plot(h, df['skewness_avg'], 's-', color='#e63946', linewidth=2,
             markersize=8, label='Average skewness')
axes[1].axhline(0.25, color='orange', linestyle='--', linewidth=1,
                alpha=0.7, label='Excellent threshold (0.25)')
axes[1].axhline(0.5, color='red', linestyle='--', linewidth=1,
                alpha=0.7, label='Acceptable threshold (0.5)')
axes[1].set_xlabel('Element Size (mm)', fontsize=12)
axes[1].set_ylabel('Skewness', fontsize=12)
axes[1].set_title('Average Skewness vs Mesh Size', fontsize=12)
axes[1].legend(fontsize=10)
axes[1].grid(True, alpha=0.3)
axes[1].invert_xaxis()
axes[1].set_ylim(0, 1)

plt.suptitle('Mesh Quality Metrics — Fin Assembly FEA', fontsize=13)
plt.tight_layout()
plt.savefig(os.path.join(FIG_DIR, 'mesh_quality_metrics.png'), dpi=150)
plt.show()
print("Saved: mesh_quality_metrics.png")

# ── Summary ───────────────────────────────────────────────────────────────────
print("=" * 60)
print("SUMMARY")
print("=" * 60)
print(f"Mesh levels analyzed:          5 ({', '.join([str(x) for x in df['element_size_mm'].tolist()])} mm)")
print(f"Finest mesh:                   {df['element_size_mm'].values[-1]} mm "
      f"({int(df['node_count'].values[-1]):,} nodes)")
print(f"Node limit (student license):  128,000")
print(f"Deformation (converged):       {f_rich_deform*1000:.4f} μm "
      f"(GCI = {GCI_deform:.3f}%)")
print(f"Von Mises stress (reported):   {sigma_reported:.3f} ± "
      f"{df['von_mises_MPa'].values[2:].std():.3f} MPa")
print(f"Safety factor vs yield:        {SF_yield_reported:.0f}× "
      f"(Ti-6Al-4V σ_y = {YIELD_STRENGTH_MPA} MPa)")
print(f"Safety factor vs UTS:          {SF_uts_reported:.0f}×")
print(f"Fin status:                    PASS — significantly under yield")
print("=" * 60)