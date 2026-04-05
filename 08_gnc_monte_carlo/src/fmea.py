"""
fmea.py -- Failure Mode and Effects Analysis
Project 08 -- GNC/Monte Carlo / Weapons Systems Simulation Pipeline

System-level FMEA covering the full simulation pipeline from aerodynamics
through GNC engagement. Each failure mode is rated on:
  Severity (S):    1-10, consequence of failure reaching the end user
  Occurrence (O):  1-10, likelihood of failure occurring
  Detection (D):   1-10, likelihood of failure going undetected
  RPN:             S x O x D -- Risk Priority Number

Outputs:
  results/fmea_table.csv
  results/figures/fmea_table.png
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import os
import csv

# ---------------------------------------------------------------
# Output paths
# ---------------------------------------------------------------
RESULTS_DIR = "D:/Weapons-systems-sim-pipeline/08_gnc_monte_carlo/results"
FIGURES_DIR = os.path.join(RESULTS_DIR, "figures")
os.makedirs(FIGURES_DIR, exist_ok=True)

# ---------------------------------------------------------------
# FMEA Table
# Each entry: (ID, System, Component, Failure Mode, Effect, Cause, S, O, D, Controls)
# ---------------------------------------------------------------
FMEA_DATA = [
    # ── Seeker / Guidance ──────────────────────────────────────
    (1,  "GNC",        "Seeker",
     "Loss of target lock",
     "Guidance commands go to zero — missile flies ballistic",
     "Target exits FOV (>90° off boresight) or range < 800m with high LOS rate",
     9, 4, 3,
     "FOV check in true_pn(); R_check guard disabled inside 800m; seeker_valid flag terminates engagement post-CPA"),

    (2,  "GNC",        "Seeker",
     "Seeker noise saturation",
     "Spurious LOS rate commands cause lateral oscillation, miss distance increases",
     "High noise_std (ECM or hardware degradation)",
     6, 3, 4,
     "Gaussian noise model with σ=0.05 rad/s nominal; ECM scenario (5× noise) run in monte_carlo.py; accel limiter caps command magnitude"),

    (3,  "GNC",        "Guidance Law",
     "Navigation constant N too low (N < 2)",
     "Insufficient lateral acceleration to intercept maneuvering target",
     "Incorrect parameter selection",
     7, 2, 2,
     "Sensitivity analysis in plot_sensitivity() sweeps N=2–6; N=5 selected as optimal; test_guidance.py Test 8 validates TPN outperforms baseline"),

    (4,  "GNC",        "Guidance Law",
     "Navigation constant N too high (N > 6)",
     "Terminal phase oscillation, actuator saturation, ground-diving trajectory",
     "Overaggressive gain; no gain scheduling in terminal phase",
     7, 3, 3,
     "Accel limit 10g caps command; sensitivity analysis confirms degradation above N=6; single engagement verbose logging detects oscillation"),

    (5,  "GNC",        "Vertical Channel",
     "Altitude tracking failure",
     "Missile dives or climbs through target altitude — pure altitude miss",
     "Decoupled vertical channel P-controller gain mismatch; large initial altitude error",
     8, 5, 3,
     "P-controller gain 0.015 tuned against nominal engagement; gravity compensation term added; alt_err logged in verbose mode; documented modeling limitation"),

    (6,  "GNC",        "Actuator",
     "Acceleration command saturation",
     "Guidance cannot respond to target maneuver in terminal phase",
     "Closing velocity too high (Vc > 600 m/s) generating N·Vc·ω exceeding accel limit",
     6, 6, 4,
     "XY and Z channels limited independently (10g XY, 5g Z); first-order lag τ=0.1s smooths transitions; accel_limit_g parameter in run_engagement()"),

    (7,  "GNC",        "Actuator",
     "First-order lag too slow (τ too large)",
     "Guidance command cannot respond in terminal phase; missile overshoots CPA",
     "τ=0.1s eats 18% of tgo at R=327m, Vc=587m/s",
     6, 4, 3,
     "τ=0.1s validated against single engagement; tgo estimated from R/Vc at each verbose timestep; documented modeling limitation — real autopilot bandwidth higher"),

    # ── Missile Dynamics ───────────────────────────────────────
    (8,  "Propulsion",  "Motor",
     "Motor burnout before intercept",
     "Missile decelerates, drag rises relative to speed — guidance authority drops",
     "Target range > effective powered range; sustainer cutoff at t=15s",
     7, 5, 2,
     "t_sustain=15s covers nominal 4km engagement (CPA at ~9s); Pk vs range plot shows degradation beyond 5km; launch envelope heatmap identifies no-go zones"),

    (9,  "Propulsion",  "Motor",
     "Thrust misalignment",
     "Lateral force at launch creates initial heading error",
     "Not modeled — thrust assumed along velocity vector",
     5, 2, 5,
     "Documented assumption in README limitations; launch azimuth jitter ±5° in Monte Carlo partially captures effect"),

    (10, "Aerodynamics", "Drag Model",
     "CD evaluated at α=0 only",
     "Drag underpredicted at high AoA during boost phase — range overpredicted",
     "aero_interpolator.get_CD(mach, 0.0) called with fixed α=0",
     4, 7, 4,
     "Point mass model assumption — documented in Project 04 and Project 06 READMEs; drag error bounded by α range during engagement (<5° for most flight phases)"),

     (11, "Aerodynamics", "Aero Database",
     "DATCOM method coverage gap at supersonic low-AR fins",
     "CN/CA absent at M=1.6–3.0 — drag only, no normal force at supersonic speeds",
     "DATCOM wing method does not converge on CN/CA for low aspect ratio fins at supersonic Mach",
     5, 8, 2,
     "Documented in Project 04 README; CN approximated as CL at low AoA; engagement Mach range 0.5–2.0 partially outside validated database; future work: Fluent spot-checks"),

    # ── Navigation / State Estimation ──────────────────────────
    (12, "Navigation",  "EKF",
     "Filter divergence under high acceleration",
     "State estimate drifts — guidance law receives incorrect missile position/velocity",
     "Process noise Q undertuned during 3000N boost phase (0–2s)",
     7, 2, 3,
     "EKF RMSE 1.11m validated in Project 06 over full trajectory; residuals zero-mean, bounded ±3m; Q/R sensitivity analysis in Project 05"),

    (13, "Navigation",  "GPS",
     "GPS outage during terminal phase",
     "EKF propagates on IMU only — position error grows at ~0.5 m/s²",
     "GPS jamming or signal blockage",
     6, 2, 4,
     "Not explicitly modeled; IMU-only propagation error bounded by process noise Q; GPS update rate 10 Hz provides frequent corrections; ECM study covers seeker jamming"),

    # ── Structural ─────────────────────────────────────────────
    (14, "Structure",   "Fin Assembly",
     "Fin structural failure at launch",
     "Loss of fin — aerodynamic instability, tumbling",
     "Setback load exceeds yield at tab-fin fillet",
     10, 1, 1,
     "Project 02 FEA: SF 660× at RT, 173× at 249°C (Project 07) — no credible failure path under 10,000g; Richardson extrapolation confirms mesh-independent result"),

    (15, "Structure",   "Fin Assembly",
     "Fin resonance during motor burn",
     "High-cycle fatigue — fin failure during or after burn",
     "Motor burn PSD overlaps fin Mode 1 frequency",
     8, 1, 1,
     "Project 07 Modal: Mode 1 = 415 Hz; typical SRM fundamental < 100 Hz — no overlap; Random vibration SF 24× at 3σ MIL-STD-810; infinite fatigue life (1e8 cycles)"),

    (16, "Structure",   "Nose Cone",
     "Nose cone thermal failure",
     "Structural degradation or separation at high Mach",
     "Aeroheating at tip exceeds Ti-6Al-4V service temperature",
     7, 1, 1,
     "Project 07: peak nose tip 129°C vs Ti-6Al-4V service ~300°C; SF 789× combined stress; DKR method conservative (overpredicts heat flux at M=1.25)"),

    # ── Software / Simulation ──────────────────────────────────
    (17, "Software",    "Guidance",
     "Python __pycache__ serving stale .pyc",
     "Code changes have no effect — guidance behavior appears unchanged",
     "Cached bytecode not invalidated after source edit",
     5, 6, 6,
     "Documented recurring issue in development; mitigated by manual cache clear command before each run; automated in run script"),

    (18, "Software",    "Integration",
     "RK4 timestep too large (dt > 0.05s)",
     "Integration error accumulates — trajectory and miss distance inaccurate",
     "dt increased for speed without checking numerical stability",
     5, 3, 4,
     "dt=0.02s nominal; single engagement result stable at dt=0.01s and dt=0.02s; increasing to dt=0.05s shows visible trajectory deviation — threshold documented"),

    (19, "Software",    "Monte Carlo",
     "Insufficient runs for stable CEP estimate",
     "CEP50/Pk statistics unreliable — misleading performance assessment",
     "N_RUNS reduced to 50 for speed without noting statistical uncertainty",
     4, 5, 3,
     "N_RUNS=500 for final results; 50-run test mode flagged in code comments; CEP50 variance ~±30m between 50-run and 500-run confirmed by comparison"),

    (20, "Software",    "Aero Interpolator",
     "Mach out of interpolation range",
     "SciPy extrapolates — unphysical CD/CL values corrupt trajectory",
     "Missile decelerates to M < 0.8 or accelerates to M > 3.0",
     6, 2, 3,
     "Mach clipped to [0.8, 3.0] in engagement.py missile_derivatives(); CD=0.3 fallback if NaN returned; Mach range 0.5–2.0 in nominal engagement stays within clip"),
]

# ---------------------------------------------------------------
# Compute RPN and sort
# ---------------------------------------------------------------
rows = []
for i, entry in enumerate(FMEA_DATA):
    if len(entry) != 10:
        print(f"Entry {i} has {len(entry)} fields: {entry[0]} — {entry[3]}")
rows = []
for entry in FMEA_DATA:
    id_, system, component, mode, effect, cause, S, O, D, controls = entry
    rpn = S * O * D
    rows.append({
        'ID': id_, 'System': system, 'Component': component,
        'Failure Mode': mode, 'Effect': effect, 'Cause': cause,
        'S': S, 'O': O, 'D': D, 'RPN': rpn, 'Controls': controls
    })

rows.sort(key=lambda x: x['RPN'], reverse=True)

# ---------------------------------------------------------------
# Save CSV
# ---------------------------------------------------------------
csv_path = os.path.join(RESULTS_DIR, 'fmea_table.csv')
fieldnames = ['ID', 'System', 'Component', 'Failure Mode', 'Effect',
              'Cause', 'S', 'O', 'D', 'RPN', 'Controls']

with open(csv_path, 'w', newline='', encoding='utf-8') as f:
    writer = csv.DictWriter(f, fieldnames=fieldnames)
    writer.writeheader()
    writer.writerows(rows)

print(f"Saved: {csv_path}")

# ---------------------------------------------------------------
# Print summary to terminal
# ---------------------------------------------------------------
print(f"\n{'='*80}")
print(f"{'FMEA SUMMARY — Weapons Systems Simulation Pipeline':^80}")
print(f"{'='*80}")
print(f"{'ID':<4} {'System':<14} {'Component':<16} {'Mode':<35} {'S':>2} {'O':>2} {'D':>2} {'RPN':>4}")
print(f"{'-'*80}")
for r in rows:
    mode_short = r['Failure Mode'][:34]
    print(f"{r['ID']:<4} {r['System']:<14} {r['Component']:<16} {mode_short:<35} "
          f"{r['S']:>2} {r['O']:>2} {r['D']:>2} {r['RPN']:>4}")

high_rpn = [r for r in rows if r['RPN'] >= 100]
print(f"\nHigh RPN items (≥100): {len(high_rpn)}")
for r in high_rpn:
    print(f"  RPN={r['RPN']:>4}  [{r['System']}] {r['Component']} — {r['Failure Mode']}")

# ---------------------------------------------------------------
# Plot — formatted table figure
# ---------------------------------------------------------------
fig, ax = plt.subplots(figsize=(20, 10))
ax.axis('off')

col_labels = ['ID', 'System', 'Component', 'Failure Mode', 'Effect', 'S', 'O', 'D', 'RPN']
col_widths  = [0.03, 0.07, 0.09, 0.20, 0.22, 0.03, 0.03, 0.03, 0.04]

table_data = []
for r in rows:
    table_data.append([
        str(r['ID']),
        r['System'],
        r['Component'],
        r['Failure Mode'],
        r['Effect'],
        str(r['S']),
        str(r['O']),
        str(r['D']),
        str(r['RPN']),
    ])

table = ax.table(
    cellText=table_data,
    colLabels=col_labels,
    colWidths=col_widths,
    loc='center',
    cellLoc='left',
)

table.auto_set_font_size(False)
table.set_fontsize(7.5)
table.scale(1, 1.6)

# Color header
for j in range(len(col_labels)):
    table[0, j].set_facecolor('#2c3e50')
    table[0, j].set_text_props(color='white', fontweight='bold')

# Color RPN cells by risk level
rpn_col = col_labels.index('RPN')
for i, r in enumerate(rows):
    rpn = r['RPN']
    if rpn >= 200:
        color = '#e74c3c'   # red — high risk
    elif rpn >= 100:
        color = '#e67e22'   # orange — medium-high
    elif rpn >= 50:
        color = '#f1c40f'   # yellow — medium
    else:
        color = '#2ecc71'   # green — low

    table[i + 1, rpn_col].set_facecolor(color)
    table[i + 1, rpn_col].set_text_props(fontweight='bold')

    # Alternate row shading
    if i % 2 == 0:
        for j in range(len(col_labels)):
            if j != rpn_col:
                table[i + 1, j].set_facecolor('#f8f9fa')

# Legend
patches = [
    mpatches.Patch(color='#e74c3c', label='RPN ≥ 200 (High)'),
    mpatches.Patch(color='#e67e22', label='RPN 100–199 (Medium-High)'),
    mpatches.Patch(color='#f1c40f', label='RPN 50–99 (Medium)'),
    mpatches.Patch(color='#2ecc71', label='RPN < 50 (Low)'),
]
ax.legend(handles=patches, loc='lower right',
          bbox_to_anchor=(1.0, 0.0), fontsize=8, framealpha=0.9)

ax.set_title(
    'FMEA — Weapons Systems Simulation Pipeline\n'
    'S = Severity  |  O = Occurrence  |  D = Detection  |  RPN = S × O × D\n'
    'Sorted by RPN (highest risk first)',
    fontsize=11, fontweight='bold', pad=20
)

plt.tight_layout()
fig_path = os.path.join(FIGURES_DIR, 'fmea_table.png')
plt.savefig(fig_path, dpi=150, bbox_inches='tight')
print(f"Saved: {fig_path}")
plt.show()
plt.close()