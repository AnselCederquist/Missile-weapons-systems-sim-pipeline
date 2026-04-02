"""
test_ekf.py -- Validation tests for 6-state EKF
1. Monte Carlo: run N trials with different seeds, check RMSE consistency
2. Filter divergence: show EKF diverges when Q/R are badly mistuned
"""

import numpy as np
import matplotlib.pyplot as plt
import sys
sys.path.append("D:/Weapons-systems-sim-pipeline/05_kalman_filter/src")
sys.path.append("D:/Weapons-systems-sim-pipeline/04_missile_aero_database/postprocess")

from ekf import run_ekf

# ---------------------------------------------------------------
# TEST 1 -- Monte Carlo
# ---------------------------------------------------------------
N_TRIALS = 50
pos_rmses = []
vel_rmses = []

print("Running Monte Carlo...")
for seed in range(N_TRIALS):
    state, ts, x_est = run_ekf(seed=seed)
    pos_rmse = np.sqrt(np.mean((x_est[:,[0,2]] - state[:,[0,2]])**2))
    vel_rmse = np.sqrt(np.mean((x_est[:,[3,5]] - state[:,[3,5]])**2))
    pos_rmses.append(pos_rmse)
    vel_rmses.append(vel_rmse)

pos_rmses = np.array(pos_rmses)
vel_rmses = np.array(vel_rmses)

print(f"\nMonte Carlo Results (N={N_TRIALS})")
print(f"  Position RMSE: mean={pos_rmses.mean():.2f}m  std={pos_rmses.std():.2f}m  "
      f"max={pos_rmses.max():.2f}m")
print(f"  Velocity RMSE: mean={vel_rmses.mean():.2f}m/s  std={vel_rmses.std():.2f}m/s  "
      f"max={vel_rmses.max():.2f}m/s")

assert pos_rmses.mean() < 5.0,  "FAIL: mean position RMSE > 5m"
assert pos_rmses.max()  < 15.0, "FAIL: max position RMSE > 15m"
assert vel_rmses.mean() < 2.0,  "FAIL: mean velocity RMSE > 2 m/s"
print("  PASS: Monte Carlo within bounds")

# ---------------------------------------------------------------
# TEST 2 -- Filter divergence under bad tuning
# ---------------------------------------------------------------
print("\nRunning divergence test...")

state, ts, x_est_good = run_ekf(seed=0, Q_scale=1.0,   R_scale=1.0)
state, ts, x_est_lowQ = run_ekf(seed=0, Q_scale=0.001, R_scale=1.0)
state, ts, x_est_lowR = run_ekf(seed=0, Q_scale=1.0,   R_scale=0.001)

rmse_good = np.sqrt(np.mean((x_est_good[:,[0,2]] - state[:,[0,2]])**2))
rmse_lowQ = np.sqrt(np.mean((x_est_lowQ[:,[0,2]] - state[:,[0,2]])**2))
rmse_lowR = np.sqrt(np.mean((x_est_lowR[:,[0,2]] - state[:,[0,2]])**2))

print(f"  Baseline RMSE:       {rmse_good:.2f} m")
print(f"  Q too small RMSE:    {rmse_lowQ:.2f} m  (filter ignores GPS)")
print(f"  R too small RMSE:    {rmse_lowR:.2f} m  (filter chases noise)")

assert rmse_lowQ > rmse_good, "FAIL: low Q should degrade performance"
assert rmse_lowR > rmse_good, "FAIL: low R should degrade performance"
print("  PASS: mistuned filter degrades as expected")

# ---------------------------------------------------------------
# Plots
# ---------------------------------------------------------------
fig, axes = plt.subplots(1, 2, figsize=(12, 5))
fig.suptitle("EKF Validation Tests", fontsize=13)

axes[0].hist(pos_rmses, bins=15, color="steelblue", edgecolor="white")
axes[0].axvline(pos_rmses.mean(), color="red", ls="--",
                label=f"Mean={pos_rmses.mean():.2f}m")
axes[0].set_xlabel("Position RMSE (m)")
axes[0].set_ylabel("Count")
axes[0].set_title(f"Monte Carlo Position RMSE (N={N_TRIALS})")
axes[0].legend()
axes[0].grid(True, alpha=0.3)

labels = ["Baseline\n(well tuned)", "Q too small\n(ignores GPS)", "R too small\n(chases noise)"]
rmses  = [rmse_good, rmse_lowQ, rmse_lowR]
colors = ["steelblue", "tomato", "orange"]
axes[1].bar(labels, rmses, color=colors, edgecolor="white")
axes[1].set_ylabel("Position RMSE (m)")
axes[1].set_title("Filter Sensitivity to Q/R Tuning")
axes[1].grid(True, alpha=0.3, axis="y")
for i, v in enumerate(rmses):
    axes[1].text(i, v + 0.05, f"{v:.2f}m", ha="center", fontsize=9)

plt.tight_layout()
plt.savefig("D:/Weapons-systems-sim-pipeline/05_kalman_filter/results/figures/ekf_validation.png",
            dpi=150, bbox_inches="tight")
print("\nValidation plot saved")
plt.show()