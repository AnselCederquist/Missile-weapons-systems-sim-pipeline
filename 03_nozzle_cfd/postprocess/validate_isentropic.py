"""
validate_isentropic.py
Project 03 - De Laval Nozzle CFD
Validation of CFD centerline Mach number against isentropic flow relations
Cold flow: P0 = 601,325 Pa, T0 = 300 K, air (gamma = 1.4)
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import AutoMinorLocator
from scipy.optimize import brentq
from pathlib import Path
import pandas as pd

FIG_DIR = Path(__file__).parent.parent / 'results' / 'figures'
FIG_DIR.mkdir(parents=True, exist_ok=True)

# ── Nozzle Geometry (all radii) ───────────────────────────────────────────────
R_inlet  = 0.25    # m — inlet radius
R_throat = 0.12    # m — throat radius
R_exit   = 0.49    # m — exit radius
x_inlet  = 0.0     # m
x_throat = 0.5     # m
x_exit   = 1.86    # m
gamma    = 1.4
P0       = 601325  # Pa absolute
T0       = 300.0   # K

Ae_Astar = (R_exit / R_throat) ** 2   # = 16.694


# ── Isentropic Relations ──────────────────────────────────────────────────────
def area_ratio_from_mach(M, g=1.4):
    t = 1 + (g - 1) / 2 * M**2
    return (1 / M) * (2 / (g + 1) * t) ** ((g + 1) / (2 * (g - 1)))


def mach_from_area_ratio(AoAstar, supersonic=True, g=1.4):
    if supersonic:
        return brentq(lambda M: area_ratio_from_mach(M, g) - AoAstar, 1.0, 20.0)
    else:
        return brentq(lambda M: area_ratio_from_mach(M, g) - AoAstar, 0.001, 1.0)


def isentropic_pressure(M, P0, g=1.4):
    return P0 * (1 + (g - 1) / 2 * M**2) ** (-g / (g - 1))


def isentropic_temperature(M, T0, g=1.4):
    return T0 / (1 + (g - 1) / 2 * M**2)


def nozzle_radius(x):
    """Piecewise linear radius profile."""
    if x <= x_throat:
        t = x / x_throat
        return R_inlet + (R_throat - R_inlet) * t
    else:
        t = (x - x_throat) / (x_exit - x_throat)
        return R_throat + (R_exit - R_throat) * t


# ── Build Theoretical Centerline Profile ──────────────────────────────────────
x_theory = np.linspace(x_inlet, x_exit, 500)
M_theory = np.zeros_like(x_theory)
P_theory = np.zeros_like(x_theory)

for i, x in enumerate(x_theory):
    r = nozzle_radius(x)
    AoAstar = (r / R_throat) ** 2
    supersonic = (x > x_throat)
    try:
        M_theory[i] = mach_from_area_ratio(AoAstar, supersonic=supersonic)
    except Exception:
        M_theory[i] = np.nan
    P_theory[i] = isentropic_pressure(M_theory[i], P0)

M_throat_theory = 1.0
P_throat_theory = isentropic_pressure(1.0, P0)
M_exit_theory   = mach_from_area_ratio(Ae_Astar, supersonic=True)
P_exit_theory   = isentropic_pressure(M_exit_theory, P0)


# ── Parse CFD Data ────────────────────────────────────────────────────────────
def load_csv(path):
    df = pd.read_csv(path)
    df = df.sort_values('x_m').reset_index(drop=True)
    return df['x_m'].to_numpy(), df['mach_number'].to_numpy()

results_dir = Path(__file__).parent.parent / 'results'
x_c, M_c = load_csv(results_dir / 'centerline_mach_coarse.csv')
x_m, M_m = load_csv(results_dir / 'centerline_mach_medium.csv')
x_f, M_f = load_csv(results_dir / 'centerline_mach_fine.csv')


# ── Interpolate CFD onto theory x-grid for error calculation ──────────────────
M_c_interp = np.interp(x_theory, x_c, M_c)
M_m_interp = np.interp(x_theory, x_m, M_m)
M_f_interp = np.interp(x_theory, x_f, M_f)

err_c = (M_c_interp - M_theory) / M_theory * 100
err_m = (M_m_interp - M_theory) / M_theory * 100
err_f = (M_f_interp - M_theory) / M_theory * 100


# ── Key Validation Points ─────────────────────────────────────────────────────
def nearest(x_arr, y_arr, x_target):
    idx = np.argmin(np.abs(x_arr - x_target))
    return y_arr[idx]

print("=" * 65)
print("ISENTROPIC VALIDATION — DE LAVAL NOZZLE CFD")
print("Cold flow: P0 = 601,325 Pa | T0 = 300 K | gamma = 1.4")
print(f"Ae/A* = {Ae_Astar:.4f} | Theoretical exit Mach = {M_exit_theory:.4f}")
print("=" * 65)

print(f"\n{'Location':<12} {'Theory':>8} {'Coarse':>8} {'Err%':>7} "
      f"{'Medium':>8} {'Err%':>7} {'Fine':>8} {'Err%':>7}")
print("-" * 65)

pts = [
    ("Inlet",   x_inlet  + 0.01),
    ("Throat",  x_throat),
    ("Mid-div", (x_throat + x_exit) / 2),
    ("Exit",    x_exit   - 0.01),
]

for name, xp in pts:
    th = np.interp(xp, x_theory, M_theory)
    c  = nearest(x_c, M_c, xp)
    m  = nearest(x_m, M_m, xp)
    f  = nearest(x_f, M_f, xp)
    print(f"{name:<12} {th:>8.4f} {c:>8.4f} {(c-th)/th*100:>+7.2f}% "
          f"{m:>8.4f} {(m-th)/th*100:>+7.2f}% "
          f"{f:>8.4f} {(f-th)/th*100:>+7.2f}%")

print(f"\nNote: CFD uses viscous k-ω SST solver. Isentropic theory assumes")
print(f"inviscid, adiabatic flow. Deviations reflect viscous losses and")
print(f"turbulent boundary layer — physically correct, not errors.")


# ── Plots ─────────────────────────────────────────────────────────────────────
fig, axes = plt.subplots(2, 1, figsize=(12, 10))
fig.suptitle("Isentropic Validation — De Laval Nozzle CFD\n"
             "Cold Flow: P₀ = 601,325 Pa | T₀ = 300 K | γ = 1.4",
             fontsize=12, fontweight="bold")

# Plot 1: Mach number along centerline
ax = axes[0]
ax.plot(x_theory, M_theory, "k-",  linewidth=2.0, label="Isentropic theory", zorder=5)
ax.plot(x_c, M_c, "o--", color="#2166ac", markersize=3, linewidth=1.2,
        label=f"CFD coarse  (h=0.05m)", alpha=0.8)
ax.plot(x_m, M_m, "s--", color="#1a9850", markersize=3, linewidth=1.2,
        label=f"CFD medium  (h=0.025m)", alpha=0.8)
ax.plot(x_f, M_f, "^-",  color="#d73027", markersize=3, linewidth=1.5,
        label=f"CFD fine    (h=0.01m)")
ax.axvline(x_throat, color="gray", linestyle=":", linewidth=1.2, label="Throat (x=0.5m)")
ax.axhline(1.0,      color="gray", linestyle=":", linewidth=1.0)
ax.annotate("M = 1.0", xy=(0.02, 1.02), fontsize=9, color="gray")
ax.set_xlabel("Axial position x [m]")
ax.set_ylabel("Mach number")
ax.set_title("Centerline Mach Number — CFD vs Isentropic Theory")
ax.legend(fontsize=9)
ax.xaxis.set_minor_locator(AutoMinorLocator())
ax.yaxis.set_minor_locator(AutoMinorLocator())
ax.grid(True, alpha=0.3)
ax.set_xlim(x_inlet, x_exit)

# Plot 2: Percent error vs theory
ax = axes[1]
ax.plot(x_theory, err_c, "--", color="#2166ac", linewidth=1.5,
        label="Coarse (h=0.05m)")
ax.plot(x_theory, err_m, "--", color="#1a9850", linewidth=1.5,
        label="Medium (h=0.025m)")
ax.plot(x_theory, err_f, "-",  color="#d73027", linewidth=2.0,
        label="Fine (h=0.01m)")
ax.axvline(x_throat, color="gray", linestyle=":", linewidth=1.2)
ax.axhline(0, color="black", linewidth=0.8)
ax.fill_between(x_theory, -2, 2, alpha=0.07, color="green", label="±2% band")
ax.set_xlabel("Axial position x [m]")
ax.set_ylabel("Error vs isentropic theory [%]")
ax.set_title("CFD vs Isentropic Theory — Percentage Error")
ax.legend(fontsize=9)
ax.set_ylim(-15, 15)
ax.xaxis.set_minor_locator(AutoMinorLocator())
ax.yaxis.set_minor_locator(AutoMinorLocator())
ax.grid(True, alpha=0.3)
ax.set_xlim(x_inlet, x_exit)

plt.tight_layout()
plt.savefig(FIG_DIR / "isentropic_validation.png", dpi=150, bbox_inches="tight")
plt.show()
print(f"\nFigure saved: {FIG_DIR / 'isentropic_validation.png'}")
