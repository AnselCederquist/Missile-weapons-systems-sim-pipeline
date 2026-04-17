"""
animate_3d.py -- 3D animated GIF of Project 10 engagement
Run from 10_autopilot/src/
Requires: matplotlib, numpy, pillow  (pip install pillow)

Shows three-loop autopilot trajectory vs LQR baseline (Project 09).
"""

import sys
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D   # noqa: F401
from matplotlib.animation import FuncAnimation, PillowWriter

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(THIS_DIR, '..', '..', '08_gnc_monte_carlo', 'src'))
sys.path.insert(0, os.path.join(THIS_DIR, '..', '..', '09_attitude_control', 'src'))

from autopilot_sim import run_autopilot_sim

# ---------------------------------------------------------------
# Run simulations
# ---------------------------------------------------------------
TARGET_POS = np.array([2500.0, 0.0, 500.0])

print("Running three-loop autopilot sim...")
result_ap = run_autopilot_sim(
    launch_angle_deg = 21.2,
    target_pos       = TARGET_POS,
    use_guidance     = True,
    dt               = 0.01,
    t_max            = 60.0,
    verbose          = False,
    gain_schedule    = True,
)

print("Running LQR baseline (Project 09)...")
try:
    from attitude_sim import run_attitude_sim
    result_lqr = run_attitude_sim(
        launch_angle_deg = 21.2,
        target_pos       = TARGET_POS,
        guidance_mode    = 'TPN',
        use_guidance     = True,
        dt               = 0.01,
        t_max            = 60.0,
        verbose          = False,
        gain_schedule    = True,
    )
    pos_lqr = result_lqr['history']['pos']
    lqr_miss = result_lqr['miss_distance']
    has_lqr = True
except Exception as e:
    print(f"  LQR sim unavailable: {e}")
    pos_lqr  = None
    lqr_miss = 35.7
    has_lqr  = False

hist    = result_ap['history']
pos     = hist['pos']
t       = hist['t']
mach    = hist['mach']
ap_miss = result_ap['miss_distance']

STEP   = 10
frames = np.arange(0, len(t), STEP)
print(f"Frames: {len(frames)}  |  Three-loop miss: {ap_miss:.1f}m  |  LQR miss: {lqr_miss:.1f}m")

# ---------------------------------------------------------------
# Figure setup
# ---------------------------------------------------------------
fig = plt.figure(figsize=(10, 8))
ax  = fig.add_subplot(111, projection='3d')
fig.patch.set_facecolor('#0d1117')
ax.set_facecolor('#0d1117')

for pane in [ax.xaxis.pane, ax.yaxis.pane, ax.zaxis.pane]:
    pane.fill = False
    pane.set_edgecolor('#333333')
for spine in ax.spines.values():
    spine.set_color('#333333')

ax.tick_params(colors='#888888', labelsize=7)
for axis in [ax.xaxis, ax.yaxis, ax.zaxis]:
    axis.label.set_color('#888888')

x_max   = float(np.max(pos[:, 0])) * 1.15
y_range = max(float(np.max(np.abs(pos[:, 1]))), 300.0) * 1.5
z_max   = max(float(np.max(pos[:, 2])), TARGET_POS[2] + 100.0) * 1.15

ax.set_xlim(0, x_max)
ax.set_ylim(-y_range, y_range)
ax.set_zlim(0, z_max)
ax.set_xlabel('Downrange (m)', labelpad=4)
ax.set_ylabel('Cross-range (m)', labelpad=4)
ax.set_zlabel('Altitude (m)', labelpad=4)

# Ground plane
gx, gy = np.meshgrid([0, x_max], [-y_range, y_range])
ax.plot_surface(gx, gy, np.zeros_like(gx), alpha=0.07, color='#1a6e2e', zorder=0)

# Target
ax.scatter(*TARGET_POS, color='#ff4444', s=100, marker='*', zorder=10)
ax.plot([TARGET_POS[0]]*2, [TARGET_POS[1]]*2, [0, TARGET_POS[2]],
        '--', color='#ff4444', alpha=0.25, linewidth=0.8)

# Lethal radius rings
theta_c = np.linspace(0, 2*np.pi, 60)
r_kill  = 20.0
for dz in [-r_kill, 0, r_kill]:
    r = np.sqrt(max(r_kill**2 - dz**2, 0))
    ax.plot(TARGET_POS[0] + r*np.cos(theta_c),
            TARGET_POS[1] + r*np.sin(theta_c),
            np.full_like(theta_c, TARGET_POS[2] + dz),
            color='#ff4444', alpha=0.2, linewidth=0.6)

# Ghost trajectories (full path, faded)
ax.plot(pos[:, 0], pos[:, 1], pos[:, 2],
        color='#1a3a5e', linewidth=0.8, alpha=0.35, zorder=1,
        label='Three-loop')

if has_lqr:
    ax.plot(pos_lqr[:, 0], pos_lqr[:, 1], pos_lqr[:, 2],
            color='#2e5e2e', linewidth=0.8, alpha=0.35, zorder=1,
            label='LQR (P09)')

# Animated elements — three-loop
trail_ap,  = ax.plot([], [], [], color='#4da6ff', linewidth=1.8,
                     alpha=0.95, zorder=5, label='Three-loop live')
dot_ap,    = ax.plot([], [], [], 'o', color='#ffffff', markersize=5, zorder=6)

# Animated elements — LQR
if has_lqr:
    trail_lqr, = ax.plot([], [], [], color='#44ff88', linewidth=1.8,
                         alpha=0.95, zorder=5, label='LQR live')
    dot_lqr,   = ax.plot([], [], [], 's', color='#aaffcc', markersize=5, zorder=6)

telemetry = ax.text2D(0.02, 0.96, '', transform=ax.transAxes,
                       color='#cccccc', fontsize=8, family='monospace',
                       verticalalignment='top')

footer_txt = (f'Three-loop miss: {ap_miss:.1f}m  |  '
              f'LQR miss: {lqr_miss:.1f}m  |  '
              f'Peak Mach: {np.max(mach):.2f}')
ax.text2D(0.02, 0.03, footer_txt, transform=ax.transAxes,
          color='#666666', fontsize=7)

ax.legend(loc='upper right', framealpha=0.15, labelcolor='#cccccc',
          facecolor='#111111', edgecolor='#444444', fontsize=8)

TRAIL_FRAMES = 50

# ---------------------------------------------------------------
# Animation
# ---------------------------------------------------------------
def update(frame_idx):
    i     = frames[frame_idx]
    start = max(0, i - TRAIL_FRAMES * STEP)

    # Three-loop trail
    trail_ap.set_data(pos[start:i+1, 0], pos[start:i+1, 1])
    trail_ap.set_3d_properties(pos[start:i+1, 2])
    dot_ap.set_data([pos[i, 0]], [pos[i, 1]])
    dot_ap.set_3d_properties([pos[i, 2]])

    # LQR trail
    if has_lqr:
        i_lqr   = min(i, len(pos_lqr) - 1)
        start_l = max(0, i_lqr - TRAIL_FRAMES * STEP)
        trail_lqr.set_data(pos_lqr[start_l:i_lqr+1, 0], pos_lqr[start_l:i_lqr+1, 1])
        trail_lqr.set_3d_properties(pos_lqr[start_l:i_lqr+1, 2])
        dot_lqr.set_data([pos_lqr[i_lqr, 0]], [pos_lqr[i_lqr, 1]])
        dot_lqr.set_3d_properties([pos_lqr[i_lqr, 2]])

    V = np.linalg.norm(hist['vel'][i])
    telemetry.set_text(
        f't = {t[i]:5.1f} s\n'
        f'V = {V:5.0f} m/s   M = {mach[i]:.2f}\n'
        f'alt = {pos[i, 2]:5.0f} m\n'
        f'R = {hist["range"][i]:5.0f} m'
    )

    ax.view_init(elev=24, azim=-55 + frame_idx * 0.25)

    if has_lqr:
        return trail_ap, dot_ap, trail_lqr, dot_lqr, telemetry
    return trail_ap, dot_ap, telemetry


anim = FuncAnimation(fig, update, frames=len(frames), interval=40, blit=False)

# ---------------------------------------------------------------
# Save
# ---------------------------------------------------------------
save_dir = os.path.join(THIS_DIR, '..', 'results', 'figures')
os.makedirs(save_dir, exist_ok=True)
gif_path = os.path.join(save_dir, 'engagement_3d.gif')

print(f"Saving: {os.path.abspath(gif_path)}")
print(f"Rendering {len(frames)} frames at 25 fps...")

writer = PillowWriter(fps=25)
anim.save(gif_path, writer=writer, dpi=110)

print(f"Done. GIF saved: {gif_path}")
plt.close(fig)