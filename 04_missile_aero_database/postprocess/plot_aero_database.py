import pandas as pd
import matplotlib.pyplot as plt
import os

df = pd.read_csv("D:/Weapons-systems-sim-pipeline/04_missile_aero_database/results/aero_database.csv")
outdir = "D:/Weapons-systems-sim-pipeline/04_missile_aero_database/results/figures"
os.makedirs(outdir, exist_ok=True)

fig, axes = plt.subplots(1, 2, figsize=(10, 5))
fig.suptitle("Missile Aerodynamic Database -- DATCOM", fontsize=13)

colors = {0.8:"#1f77b4", 1.2:"#ff7f0e", 1.6:"#2ca02c", 2.0:"#d62728", 3.0:"#9467bd"}

for mach, grp in df.groupby("Mach"):
    c = colors.get(mach, "black")
    lbl = f"M={mach}"
    cl = grp["CL"].dropna()
    if not cl.empty:
        axes[0].plot(grp.loc[cl.index,"Alpha_deg"], cl, marker="o", color=c, label=lbl)
    cm = grp["CM"].dropna()
    if not cm.empty:
        axes[1].plot(grp.loc[cm.index,"Alpha_deg"], cm, marker="o", color=c, label=lbl)

axes[0].set_xlabel("Alpha (deg)"); axes[0].set_ylabel("CL"); axes[0].set_title("Lift Coefficient")
axes[1].set_xlabel("Alpha (deg)"); axes[1].set_ylabel("CM"); axes[1].set_title("Moment Coefficient")

for ax in axes:
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig(f"{outdir}/aero_database.png", dpi=150, bbox_inches="tight")
print("aero_database.png saved")
plt.show()
