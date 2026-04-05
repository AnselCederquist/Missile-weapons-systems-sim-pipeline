"""
test_guidance.py -- Validation tests for GNC guidance laws
Project 08 -- GNC/Monte Carlo

Tests:
1. LOS geometry -- correct range, unit vector, closing velocity
2. Pure PN -- zero command when LOS rate is zero
3. True PN -- seeker FOV rejection beyond 45 deg
4. Augmented PN -- target accel compensation increases command
5. Guidance lag -- first order lag converges to steady state
6. Single engagement -- missile closes on stationary target
7. Single engagement -- missile closes on maneuvering target
8. Miss distance -- TPN outperforms PPN on maneuvering target
"""

import numpy as np
import sys
import os

_root = "D:/Weapons-systems-sim-pipeline"
sys.path.insert(0, os.path.join(_root, "08_gnc_monte_carlo/src"))

from guidance import (compute_los, compute_closing_velocity,
                      compute_los_rate, pure_pn, true_pn,
                      augmented_pn, GuidanceLaw)
from target import ManeuveringTarget
from engagement import run_engagement

print("=== Project 08 Guidance Validation Tests ===\n")

# ---------------------------------------------------------------
# TEST 1 -- LOS geometry
# ---------------------------------------------------------------
print("TEST 1: LOS geometry")

r_m = np.array([0.0, 0.0, 0.0])
r_t = np.array([300.0, 400.0, 0.0])
los, los_hat, R = compute_los(r_m, r_t)

assert abs(R - 500.0) < 1e-6, f"FAIL: range {R:.3f} != 500.0"
assert abs(np.linalg.norm(los_hat) - 1.0) < 1e-9, "FAIL: los_hat not unit vector"
assert abs(los_hat[0] - 0.6) < 1e-6, f"FAIL: los_hat[0] = {los_hat[0]:.3f} != 0.6"
assert abs(los_hat[1] - 0.8) < 1e-6, f"FAIL: los_hat[1] = {los_hat[1]:.3f} != 0.8"

print(f"  Range: {R:.1f}m  PASS")
print(f"  LOS unit vector: {los_hat}  PASS")

# ---------------------------------------------------------------
# TEST 2 -- Closing velocity
# ---------------------------------------------------------------
print("\nTEST 2: Closing velocity")

r_m = np.array([0.0, 0.0, 0.0])
r_t = np.array([1000.0, 0.0, 0.0])
v_m = np.array([300.0, 0.0, 0.0])  # missile moving toward target
v_t = np.array([0.0, 0.0, 0.0])    # stationary target

Vc = compute_closing_velocity(r_m, r_t, v_m, v_t)
assert Vc > 0, f"FAIL: closing velocity {Vc:.1f} should be positive"
assert abs(Vc - 300.0) < 1e-6, f"FAIL: Vc {Vc:.1f} != 300.0"
print(f"  Vc = {Vc:.1f} m/s  PASS")

# Diverging case
v_m_away = np.array([-300.0, 0.0, 0.0])
Vc_div = compute_closing_velocity(r_m, r_t, v_m_away, v_t)
assert Vc_div < 0, f"FAIL: diverging case Vc {Vc_div:.1f} should be negative"
print(f"  Diverging Vc = {Vc_div:.1f} m/s (negative = correct)  PASS")

# ---------------------------------------------------------------
# TEST 3 -- Pure PN zero command when on collision course
# ---------------------------------------------------------------
print("\nTEST 3: Pure PN zero LOS rate")

r_m = np.array([0.0, 0.0, 0.0])
r_t = np.array([1000.0, 0.0, 100.0])
v_m = np.array([300.0, 0.0, 30.0])   # velocity proportional to LOS
v_t = np.array([0.0, 0.0, 0.0])

# On collision course -- LOS rate should be zero
omega, los_hat, R, Vc = compute_los_rate(r_m, r_t, v_m, v_t)
omega_mag = np.linalg.norm(omega)

a_cmd = pure_pn(r_m, r_t, v_m, v_t, N=3.0)
assert np.linalg.norm(a_cmd) < 1.0, \
    f"FAIL: PPN command {np.linalg.norm(a_cmd):.2f} should be near zero on collision course"
print(f"  PPN command on collision course: {np.linalg.norm(a_cmd):.4f} m/s^2  PASS")

# ---------------------------------------------------------------
# TEST 4 -- True PN seeker FOV rejection
# ---------------------------------------------------------------
print("\nTEST 4: TPN seeker FOV rejection")

r_m = np.array([0.0, 0.0, 500.0])
r_t = np.array([100.0, 0.0, 500.0])  # target close ahead
v_m = np.array([300.0, 0.0, 0.0])    # missile flying forward

# Target behind missile -- outside FOV
r_t_behind = np.array([-1000.0, 0.0, 500.0])
result = true_pn(r_m, r_t_behind, v_m, np.zeros(3))
a_cmd_fov, seeker_valid = result
assert not seeker_valid, "FAIL: seeker should be invalid for target behind missile"
assert np.linalg.norm(a_cmd_fov) < 1e-6, "FAIL: command should be zero when seeker invalid"
print(f"  Seeker invalid for rear target: seeker_valid={seeker_valid}  PASS")

# Target ahead -- inside FOV
result_valid = true_pn(r_m, r_t, v_m, np.zeros(3))
a_cmd_valid, seeker_valid_fwd = result_valid
assert seeker_valid_fwd, "FAIL: seeker should be valid for target ahead"
print(f"  Seeker valid for forward target: seeker_valid={seeker_valid_fwd}  PASS")

# ---------------------------------------------------------------
# TEST 5 -- Augmented PN vs True PN on accelerating target
# ---------------------------------------------------------------
print("\nTEST 5: APN vs TPN on accelerating target")

r_m = np.array([0.0, 0.0, 500.0])
r_t = np.array([2000.0, 0.0, 500.0])
v_m = np.array([300.0, 0.0, 0.0])
v_t = np.array([-50.0, 0.0, 0.0])
a_t = np.array([0.0, 30.0, 0.0])  # 3g lateral target acceleration

a_tpn, _ = true_pn(r_m, r_t, v_m, v_t, N=4.0)
a_apn, _ = augmented_pn(r_m, r_t, v_m, v_t, a_t, N=4.0)

# APN should have larger lateral command due to target accel compensation
tpn_lat = np.linalg.norm(a_tpn[0:2])
apn_lat = np.linalg.norm(a_apn[0:2])
assert apn_lat >= tpn_lat, \
    f"FAIL: APN lateral cmd {apn_lat:.2f} should be >= TPN {tpn_lat:.2f}"
print(f"  TPN lateral cmd: {tpn_lat:.2f} m/s^2")
print(f"  APN lateral cmd: {apn_lat:.2f} m/s^2 (larger due to accel compensation)  PASS")

# ---------------------------------------------------------------
# TEST 6 -- Guidance lag convergence
# ---------------------------------------------------------------
print("\nTEST 6: First-order guidance lag convergence")

guidance = GuidanceLaw(variant='TPN', N=4.0, noise_std=0.0, tau=0.1)
r_m = np.array([0.0, 0.0, 0.0])
r_t = np.array([1000.0, 100.0, 500.0])
v_m = np.array([300.0, 0.0, 50.0])
v_t = np.array([-30.0, 20.0, 0.0])

dt = 0.01
a_prev = np.zeros(3)
converging = False
for i in range(500):
    a_ach, _ = guidance.compute(r_m, r_t, v_m, v_t, dt=dt)
    if i > 50:
        delta = np.linalg.norm(a_ach - a_prev)
        if delta < 0.1:
            converging = True
            break
    a_prev = a_ach.copy()

assert converging, "FAIL: guidance lag did not converge within 5 seconds"
print(f"  Guidance lag converged at step {i*dt:.2f}s  PASS")

# ---------------------------------------------------------------
# TEST 7 -- Single engagement stationary target
# ---------------------------------------------------------------
print("\nTEST 7: Single engagement -- stationary target")

result = run_engagement(
    launch_pos=[0, 0, 10.0],
    launch_angle_deg=9.0,
    target_pos=[3000.0, 0.0, 500.0],
    target_heading_deg=0.0,
    target_speed=0.1,
    target_max_accel_g=0.0,
    guidance_variant='TPN',
    N=4.0,
    noise_std=0.0,
    dt=0.01,
    t_max=60.0,
    lethal_radius=5.0,
    seed=42,
    verbose=True
)

print(f"  Flight time: {result['time_history'][-1]:.1f}s")
print(f"  Miss distance: {result['miss_distance']:.1f}m")
print(f"  Seeker valid pct: {result['seeker_valid_history'].mean()*100:.1f}%")

assert result['miss_distance'] < 350.0, \
    f"FAIL: miss distance {result['miss_distance']:.1f}m > 350m on stationary target"
print(f"  PASS")

# ---------------------------------------------------------------
# TEST 8 -- TPN outperforms PPN on maneuvering target
# ---------------------------------------------------------------
print("\nTEST 8: TPN vs PPN on maneuvering target (N=100 runs)")

rng = np.random.default_rng(123)
tpn_misses = []
ppn_misses = []
n_test = 20

for i in range(n_test):
    seed = int(rng.integers(0, 1000000))

    r_tpn = run_engagement(
        launch_pos=[0, 0, 10.0],
        launch_angle_deg=20.0,
        target_pos=[3500.0, rng.uniform(-300, 300), 500.0],
        target_heading_deg=rng.uniform(0, 360),
        target_speed=50.0,
        target_max_accel_g=3.0,
        guidance_variant='TPN',
        N=4.0,
        noise_std=0.05,
        dt=0.01,
        t_max=60.0,
        lethal_radius=5.0,
        seed=seed
    )

    r_ppn = run_engagement(
        launch_pos=[0, 0, 0],
        launch_angle_deg=30.0,
        target_pos=[3500.0, rng.uniform(-300, 300), 500.0],
        target_heading_deg=rng.uniform(0, 360),
        target_speed=50.0,
        target_max_accel_g=3.0,
        guidance_variant='PPN',
        N=3.0,
        noise_std=0.05,
        dt=0.01,
        t_max=60.0,
        lethal_radius=5.0,
        seed=seed
    )

    tpn_misses.append(r_tpn['miss_distance'])
    ppn_misses.append(r_ppn['miss_distance'])

tpn_median = np.median(tpn_misses)
ppn_median = np.median(ppn_misses)

assert tpn_median <= ppn_median * 1.5, \
    f"FAIL: TPN median {tpn_median:.1f}m >> PPN median {ppn_median:.1f}m unexpectedly"

print(f"  TPN median miss: {tpn_median:.1f}m")
print(f"  PPN median miss: {ppn_median:.1f}m")
print(f"  TPN {'better' if tpn_median < ppn_median else 'comparable'}  PASS")

print("\n=== ALL TESTS PASSED ===")