"""
mesh_convergence.py
Project 03 - De Laval Nozzle CFD
3D Cold Flow Mesh Convergence Study
Richardson Extrapolation + GCI Analysis
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import AutoMinorLocator
from pathlib import Path

FIG_DIR = Path(__file__).parent.parent / 'results' / 'figures'
FIG_DIR.mkdir(parents=True, exist_ok=True)

# ── Mesh Data ────────────────────────────────────────────────────────────────
# Columns: label, element_size, nodes, elements, throat_mach, exit_mach,
#          mass_flow_inlet, mass_flow_outlet

mesh_data = {
    "Default":  {"h": 0.11598, "nodes": 1716,   "elements": 1407,   "throat_mach": 0.7214007, "exit_mach": 2.4974254, "mdot_in": 61.860956,  "mdot_out": -61.860951},
    "Coarse":   {"h": 0.05,    "nodes": 6440,   "elements": 5460,   "throat_mach": 0.7738636, "exit_mach": 2.6073601, "mdot_in": 63.951007,  "mdot_out": -63.921140},
    "Medium":   {"h": 0.025,   "nodes": 44304,  "elements": 40425,  "throat_mach": 0.7849474, "exit_mach": 2.6999966, "mdot_in": 65.362400,  "mdot_out": -65.372600},
    "Fine":     {"h": 0.01,    "nodes": 651690, "elements": 627590, "throat_mach": 0.8290257, "exit_mach": 2.7349051, "mdot_in": 65.916082,  "mdot_out": -65.909748},
}

labels    = list(mesh_data.keys())
h         = np.array([mesh_data[k]["h"]           for k in labels])
nodes     = np.array([mesh_data[k]["nodes"]        for k in labels])
elements  = np.array([mesh_data[k]["elements"]     for k in labels])
m_throat  = np.array([mesh_data[k]["throat_mach"]  for k in labels])
m_exit    = np.array([mesh_data[k]["exit_mach"]    for k in labels])
mdot_in   = np.array([mesh_data[k]["mdot_in"]      for k in labels])
mdot_out  = np.array([mesh_data[k]["mdot_out"]     for k in labels])
mdot_err  = np.abs((mdot_in + mdot_out) / mdot_in) * 100  # % imbalance


# ── Richardson Extrapolation (finest 3 levels) ───────────────────────────────
def richardson_extrapolation(f1, f2, f3, h1, h2, h3):
    """
    f1, f2, f3: fine → medium → coarse values
    h1, h2, h3: corresponding mesh sizes
    Returns: f_exact, order_p, GCI_fine, GCI_medium
    """
    r21 = h2 / h1
    r32 = h3 / h2
    eps21 = f2 - f1
    eps32 = f3 - f2

    # apparent order via fixed-point iteration
    def calc_p(eps21, eps32, r21, r32, p0=2.0):
        for _ in range(50):
            s = np.sign(eps32 / eps21)
            q = np.log((r21**p0 - s) / (r32**p0 - s))
            p_new = (1 / np.log(r21)) * abs(np.log(abs(eps32 / eps21)) + q)
            if abs(p_new - p0) < 1e-6:
                return p_new
            p0 = p_new
        return p0

    p = calc_p(eps21, eps32, r21, r32)
    f_exact = f1 + (f1 - f2) / (r21**p - 1)

    Fs = 1.25  # safety factor
    GCI_fine   = Fs * abs(eps21 / f1) / (r21**p - 1) * 100
    GCI_medium = Fs * abs(eps32 / f2) / (r32**p - 1) * 100

    return f_exact, p, GCI_fine, GCI_medium


# Use finest 3 levels (Coarse, Medium, Fine — indices 1,2,3)
idx = [1, 2, 3]  # Coarse, Medium, Fine

throat_exact, throat_p, throat_GCI_fine, throat_GCI_medium = richardson_extrapolation(
    m_throat[3], m_throat[2], m_throat[1],
    h[3], h[2], h[1]
)

exit_exact, exit_p, exit_GCI_fine, exit_GCI_medium = richardson_extrapolation(
    m_exit[3], m_exit[2], m_exit[1],
    h[3], h[2], h[1]
)


# ── Print Summary ─────────────────────────────────────────────────────────────
print("=" * 65)
print("3D COLD FLOW MESH CONVERGENCE STUDY")
print("De Laval Nozzle — Ansys Fluent, Density-Based, k-ω SST")
print("=" * 65)

print(f"\n{'Mesh':<10} {'h [m]':<10} {'Nodes':>8} {'Elements':>10} "
      f"{'Throat M':>10} {'Exit M':>8} {'Δṁ [%]':>8}")
print("-" * 65)
for i, k in enumerate(labels):
    print(f"{k:<10} {h[i]:<10.5f} {nodes[i]:>8,} {elements[i]:>10,} "
          f"{m_throat[i]:>10.6f} {m_exit[i]:>8.6f} {mdot_err[i]:>8.4f}")

print("\n── Richardson Extrapolation (Coarse→Medium→Fine) ──")
print(f"  Throat Mach:  extrapolated = {throat_exact:.6f}  |  p = {throat_p:.3f}  |  "
      f"GCI_fine = {throat_GCI_fine:.4f}%  |  GCI_medium = {throat_GCI_medium:.4f}%")
print(f"  Exit Mach:    extrapolated = {exit_exact:.6f}  |  p = {exit_p:.3f}  |  "
      f"GCI_fine = {exit_GCI_fine:.4f}%  |  GCI_medium = {exit_GCI_medium:.4f}%")

print(f"\n  Throat Mach fine-mesh error vs extrapolated: "
      f"{abs(m_throat[3] - throat_exact)/throat_exact*100:.3f}%")
print(f"  Exit Mach fine-mesh error vs extrapolated:   "
      f"{abs(m_exit[3] - exit_exact)/exit_exact*100:.3f}%")


# ── Plots ─────────────────────────────────────────────────────────────────────
fig, axes = plt.subplots(1, 3, figsize=(16, 5))
fig.suptitle("3D Cold Flow Mesh Convergence Study", fontsize=13, fontweight="bold")

# 1. Throat Mach vs mesh size
ax = axes[0]
ax.plot(h, m_throat, "o-", color="#2166ac", linewidth=1.8, markersize=6, label="CFD")
ax.axhline(throat_exact, color="#d73027", linestyle="--", linewidth=1.5,
           label=f"Richardson extrapolated = {throat_exact:.4f}")
ax.fill_between([h.min()*0.5, h.max()*1.2],
                throat_exact * (1 - throat_GCI_fine/100),
                throat_exact * (1 + throat_GCI_fine/100),
                alpha=0.12, color="#d73027", label=f"GCI_fine = {throat_GCI_fine:.3f}%")
for i, k in enumerate(labels):
    ax.annotate(k, (h[i], m_throat[i]), textcoords="offset points",
                xytext=(5, 5), fontsize=8, color="#555")
ax.set_xlabel("Element size h [m]")
ax.set_ylabel("Throat Mach number (area-weighted avg)")
ax.set_title("Throat Mach Convergence")
ax.legend(fontsize=8)
ax.xaxis.set_minor_locator(AutoMinorLocator())
ax.yaxis.set_minor_locator(AutoMinorLocator())
ax.grid(True, alpha=0.3)

# 2. Exit Mach vs mesh size
ax = axes[1]
ax.plot(h, m_exit, "s-", color="#1a9850", linewidth=1.8, markersize=6, label="CFD")
ax.axhline(exit_exact, color="#d73027", linestyle="--", linewidth=1.5,
           label=f"Richardson extrapolated = {exit_exact:.4f}")
ax.fill_between([h.min()*0.5, h.max()*1.2],
                exit_exact * (1 - exit_GCI_fine/100),
                exit_exact * (1 + exit_GCI_fine/100),
                alpha=0.12, color="#d73027", label=f"GCI_fine = {exit_GCI_fine:.3f}%")
for i, k in enumerate(labels):
    ax.annotate(k, (h[i], m_exit[i]), textcoords="offset points",
                xytext=(5, 5), fontsize=8, color="#555")
ax.set_xlabel("Element size h [m]")
ax.set_ylabel("Exit Mach number (area-weighted avg)")
ax.set_title("Exit Mach Convergence")
ax.legend(fontsize=8)
ax.xaxis.set_minor_locator(AutoMinorLocator())
ax.yaxis.set_minor_locator(AutoMinorLocator())
ax.grid(True, alpha=0.3)

# 3. Mass flow balance
ax = axes[2]
ax.semilogy(h, mdot_err, "^-", color="#762a83", linewidth=1.8, markersize=6)
for i, k in enumerate(labels):
    ax.annotate(k, (h[i], mdot_err[i]), textcoords="offset points",
                xytext=(5, 3), fontsize=8, color="#555")
ax.axhline(0.1, color="#d73027", linestyle="--", linewidth=1.2,
           label="0.1% threshold")
ax.set_xlabel("Element size h [m]")
ax.set_ylabel("Mass flow imbalance [%]")
ax.set_title("Mass Flow Conservation")
ax.legend(fontsize=8)
ax.xaxis.set_minor_locator(AutoMinorLocator())
ax.grid(True, alpha=0.3, which="both")

plt.tight_layout()
plt.savefig(FIG_DIR / "mesh_convergence_results.png", dpi=150, bbox_inches="tight")
plt.show()
print(f"\nFigure saved: {FIG_DIR / 'mesh_convergence_results.png'}")
