"""
seeker.py -- IR/RF Seeker Model
Project 11 -- Seeker Model

Models the seeker sensor that provides LOS measurements to the TPN guidance law.
Replaces the perfect position/velocity knowledge assumed in Projects 08-10.

Architecture:
  - Gimbal dynamics: 2-axis (pitch/yaw) rate loop with position and slew limits
  - Track loop: PI LOS rate estimator
  - Noise model: additive Gaussian + glint at short range + radome error
  - Seeker saturation and lock-loss detection
  - ECM mode: spoofed LOS injection (arXiv:2604.11410 — Andersson & Dán, Active Bayesian Inference for Robust Control under Sensor False Data Injection Attacks)

Integration into attitude_sim.py / autopilot_sim.py:
  Replace:
      a_out, valid = tpn_guidance.compute(pos, r_t, vel_i, vel_t, accel_t, dt)
  With:
      r_app, v_app, skr_valid = seeker.update(pos, r_t, vel_i, vel_t, dt)
      if skr_valid:
          a_out, valid = tpn_guidance.compute(pos, r_app, vel_i, v_app, accel_t, dt)
      else:
          valid = False

All parameters are module-level constants — patchable by a future tuner.
"""

import numpy as np


# ---------------------------------------------------------------
# Seeker physical parameters
# ---------------------------------------------------------------

# Gimbal
GIMBAL_FOV_DEG       = 60.0    # half-angle field of view (deg)
GIMBAL_RATE_LIM      = 30.0    # max gimbal slew rate (deg/s)
GIMBAL_POS_LIM       = 60.0    # max gimbal deflection from boresight (deg)
GIMBAL_TAU           = 0.05    # gimbal rate loop lag (s) — first-order

# Track loop (PI LOS rate estimator)
TRACK_KP             = 8.0     # proportional gain
TRACK_KI             = 2.0     # integral gain
TRACK_INTEGRATOR_LIM = 5.0     # rad/s — anti-windup clip

# Noise model
NOISE_STD_BASE       = 0.002   # rad/s — base Gaussian LOS rate noise
GLINT_RANGE          = 200.0   # m — range below which glint activates
GLINT_STD            = 0.015   # rad/s — glint noise std dev at zero range
RADOME_SLOPE         = 0.003   # radome boresight error slope (rad/rad)

# Lock-loss
LOCK_LOSS_NOISE_THR  = 0.18    # rad/s — LOS rate noise threshold for lock-loss
LOCK_REACQ_FRAMES    = 50      # frames below threshold before reacquire

# Update rate
SEEKER_DT            = 0.01    # s — seeker runs at sim dt


class SeekerModel:
    """
    Two-axis IR/RF seeker with gimbal dynamics, track loop,
    noise model, and ECM injection.

    Usage
    -----
    seeker = SeekerModel(seed=42)
    seeker.reset(missile_pos, target_pos)

    In sim loop:
        r_app, v_app, valid = seeker.update(pos, r_t, vel_i, vel_t, dt)
        a_out, g_valid = tpn_guidance.compute(pos, r_app, vel_i, v_app, accel_t, dt)
        valid = valid and g_valid
    """

    def __init__(self,
                 noise_std:     float = NOISE_STD_BASE,
                 glint_std:     float = GLINT_STD,
                 radome_slope:  float = RADOME_SLOPE,
                 ecm_mode:      bool  = False,
                 ecm_bias:      float = 0.0,
                 ecm_noise_std: float = 0.0,
                 seed:          int   = 42):
        """
        Parameters
        ----------
        noise_std     : base Gaussian LOS rate noise (rad/s)
        glint_std     : glint noise std dev at zero range (rad/s)
        radome_slope  : radome boresight error coefficient (rad/rad)
        ecm_mode      : enable ECM spoofing
        ecm_bias      : ECM LOS angle bias injection (rad)
        ecm_noise_std : ECM additive noise std dev (rad/s)
        seed          : RNG seed
        """
        self.noise_std    = noise_std
        self.glint_std    = glint_std
        self.radome_slope = radome_slope
        self.ecm_mode     = ecm_mode
        self.ecm_bias     = ecm_bias
        self.ecm_noise_std= ecm_noise_std
        self.rng          = np.random.default_rng(seed)

        # Gimbal state [pitch_angle, yaw_angle] (rad)
        self._gimbal_pos  = np.zeros(2)
        self._gimbal_rate = np.zeros(2)

        # Track loop integrator
        self._track_int   = np.zeros(2)

        # Lock state
        self._locked          = False
        self._frames_below_thr = 0
        self._los_rate_filt   = np.zeros(3)

        # Range-rate estimator (for radial velocity reconstruction)
        # Range-rate estimator (for radial velocity reconstruction)
        self._R_prev          = None
        self._R_dot_filt      = 0.0

        # Range estimator (for r_apparent reconstruction; biased under ECM)
        self._R_est           = None
        self._R_est_bias      = 0.0
        # Persistent track-loop slip (accumulates during high noise)
        self._track_slip = np.zeros(2)  # rad — accumulated angle bias

        # History for diagnostics
        self.hist = {
            'gimbal_pos':  [],
            'los_rate_raw':[],
            'los_rate_filt':[],
            'noise_total': [],
            'locked':      [],
            'r_apparent':  [],
        }

    def reset(self, missile_pos: np.ndarray, target_pos: np.ndarray):
        """Initialise gimbal to point at target from launch position."""
        r_vec = target_pos - missile_pos
        R     = np.linalg.norm(r_vec)
        if R > 1.0:
            pitch0 = np.arctan2(r_vec[2], np.sqrt(r_vec[0]**2 + r_vec[1]**2))
            yaw0   = np.arctan2(r_vec[1], r_vec[0])
        else:
            pitch0 = 0.0
            yaw0   = 0.0
        self._gimbal_pos  = np.array([pitch0, yaw0])
        self._gimbal_rate = np.zeros(2)
        self._track_int   = np.zeros(2)
        self._locked      = True
        self._frames_below_thr = 0
        self._los_rate_filt    = np.zeros(3)
        self._R_prev           = R
        self._R_dot_filt       = 0.0
        self._R_est            = R
        
        self._track_slip = np.zeros(2)
        for k in self.hist:
            self.hist[k].clear()

    # ------------------------------------------------------------------

    def update(self,
               missile_pos: np.ndarray,
               target_pos:  np.ndarray,
               missile_vel: np.ndarray,
               target_vel:  np.ndarray,
               dt:          float = SEEKER_DT):
        """
        Advance seeker by one timestep.

        Parameters
        ----------
        missile_pos, target_pos : (3,) inertial positions (m)
        missile_vel, target_vel : (3,) inertial velocities (m/s)
        dt : timestep (s)

        Returns
        -------
        r_apparent  : (3,) apparent target position seen by seeker (m)
        v_apparent  : (3,) apparent target velocity (m/s)
        valid       : bool — True if seeker is locked and target in FOV
        """
        # True LOS
        r_vec  = target_pos - missile_pos
        R      = np.linalg.norm(r_vec)
        if R < 0.5:
            return target_pos.copy(), target_vel.copy(), False

        r_hat  = r_vec / R
        v_rel  = target_vel - missile_vel

        # True LOS rate (rad/s) — cross product formulation
        los_rate_true = np.cross(r_hat, v_rel) / R   # (3,) inertial

        # ── Noise model ────────────────────────────────────────────
        # 1. Base Gaussian
        noise = self.rng.normal(0.0, self.noise_std, 3)

        # 2. Glint — grows as 1/R below GLINT_RANGE
        if R < GLINT_RANGE:
            glint_scale = (GLINT_RANGE - R) / GLINT_RANGE
            noise += self.rng.normal(0.0, self.glint_std * glint_scale, 3)
            # Glint also induces persistent centroid wander (biased component)
            # Real glint = multi-scatter off target panels → biased centroid shift
            glint_bias = self.rng.normal(0.0, self.glint_std * glint_scale * 2.0, 2)
            self._track_slip += glint_bias * dt * 5.0

        # 3. Radome boresight error — proportional to gimbal deflection
        gimbal_mag = np.linalg.norm(self._gimbal_pos)
        noise     += self.radome_slope * gimbal_mag * self.rng.normal(0.0, 1.0, 3)

        # 4. ECM injection (arXiv [87] — spoofed LOS bias + noise)
        if self.ecm_mode:
            ecm_spoof  = self.ecm_bias * r_hat
            ecm_noise  = self.rng.normal(0.0, self.ecm_noise_std, 3)
            noise     += ecm_spoof + ecm_noise

        los_rate_noisy = los_rate_true + noise
        noise_mag      = np.linalg.norm(noise)

        # ── Gimbal dynamics ────────────────────────────────────────
        los_pitch = np.arctan2(r_vec[2], np.sqrt(r_vec[0]**2 + r_vec[1]**2))
        los_yaw   = np.arctan2(r_vec[1], r_vec[0])
        los_gimbal = np.array([los_pitch, los_yaw])

        # Inject angle noise into gimbal measurement (2D projection of LOS rate noise)
        angle_noise = np.array([noise[2], -noise[1]]) / max(R / 1000.0, 0.1)
        los_gimbal_meas = los_gimbal + angle_noise

       # Track-loop slip: always active, proportional to total noise magnitude
        # Models radome refraction + thermal drift + integrator bias in track loop
        slip_rate = 2.0 * noise_mag   # rad/s of slip accumulation per rad/s of noise
        self._track_slip += self.rng.normal(0.0, slip_rate * dt, 2)
        self._track_slip = np.clip(self._track_slip,
                                   -np.radians(2.5), np.radians(2.5))
        # Slow restoring force toward zero (radome model partially corrects)
        self._track_slip *= np.exp(-0.5 * dt)

        gimbal_error = los_gimbal_meas - self._gimbal_pos + self._track_slip
        gimbal_error[1] = (gimbal_error[1] + np.pi) % (2*np.pi) - np.pi

        # PI track loop
        self._track_int  = np.clip(
            self._track_int + gimbal_error * dt,
            -TRACK_INTEGRATOR_LIM, TRACK_INTEGRATOR_LIM
        )
        rate_cmd = TRACK_KP * gimbal_error + TRACK_KI * self._track_int

        # Slew rate limit
        rate_cmd = np.clip(rate_cmd,
                           -np.radians(GIMBAL_RATE_LIM),
                            np.radians(GIMBAL_RATE_LIM))

        # First-order gimbal dynamics
        tau = max(GIMBAL_TAU, dt)
        self._gimbal_rate += (rate_cmd - self._gimbal_rate) * (dt / tau)
        self._gimbal_pos  += self._gimbal_rate * dt

        # Position limits
        self._gimbal_pos = np.clip(self._gimbal_pos,
                                   -np.radians(GIMBAL_POS_LIM),
                                    np.radians(GIMBAL_POS_LIM))

        # ── Lock-loss detection ────────────────────────────────────
        fov_half = np.radians(GIMBAL_FOV_DEG)
        in_fov   = (abs(self._gimbal_pos[0]) < fov_half and
                    abs(self._gimbal_pos[1]) < fov_half)

        if noise_mag < LOCK_LOSS_NOISE_THR and in_fov:
            self._frames_below_thr += 1
        else:
            self._frames_below_thr = 0

        if self._locked:
            if noise_mag > LOCK_LOSS_NOISE_THR or not in_fov:
                self._locked = False
        else:
            if self._frames_below_thr >= LOCK_REACQ_FRAMES and in_fov:
                self._locked = True

        # ── LOS rate filter (1st-order low-pass) ──────────────────
        alpha_filt = dt / (GIMBAL_TAU + dt)
        self._los_rate_filt = (self._los_rate_filt +
                                alpha_filt * (los_rate_noisy - self._los_rate_filt))

        # ── Apparent target position ───────────────────────────────
        # Reconstruct apparent target position from gimbal angles + range
        # Range estimated from closing velocity (1st-order range filter in real seekers)
        # Here we use true range — range estimation is a separate loop
        # Apply track-slip directly to reported pointing direction
        # (gimbal PI controller corrects fast — bias must survive to output)
        g_pitch = self._gimbal_pos[0] + self._track_slip[0]
        g_yaw   = self._gimbal_pos[1] + self._track_slip[1]
        r_apparent_hat = np.array([
            np.cos(g_pitch) * np.cos(g_yaw),
            np.cos(g_pitch) * np.sin(g_yaw),
            np.sin(g_pitch)
        ])
        # Range estimation: 5% Gaussian error + ECM bias, low-pass filtered
        if self._R_est is None:
            self._R_est = R
        R_est_noise = self.rng.normal(0.0, 0.05 * R)
        self._R_est = 0.92 * self._R_est + 0.08 * (R + R_est_noise + self._R_est_bias)
        r_apparent = missile_pos + self._R_est * r_apparent_hat

        # Apparent velocity: reconstruct from LOS rate (perpendicular) + range-rate (radial)
        # Perpendicular component from LOS rate: v_perp = cross(omega, r_vec)
        v_perp_apparent = np.cross(self._los_rate_filt, r_vec)

        # Radial component from range-rate (doppler / differentiated range)
        if self._R_prev is not None and dt > 0:
            R_dot_raw = (R - self._R_prev) / dt
            # Add range-rate noise (proportional to base noise × range)
            R_dot_noise = self.rng.normal(0.0, self.noise_std * R * 0.5)
            R_dot_meas = R_dot_raw + R_dot_noise
            # Low-pass filter
            alpha_rd = dt / (GIMBAL_TAU + dt)
            self._R_dot_filt += alpha_rd * (R_dot_meas - self._R_dot_filt)
        self._R_prev = R

        # Full v_rel = radial + perpendicular
        r_hat_app = r_apparent_hat
        v_radial_apparent = self._R_dot_filt * r_hat_app
        v_rel_apparent = v_perp_apparent + v_radial_apparent
        v_apparent = missile_vel + v_rel_apparent

        # ── History ────────────────────────────────────────────────
        self.hist['gimbal_pos'].append(np.degrees(self._gimbal_pos).copy())
        self.hist['los_rate_raw'].append(los_rate_noisy.copy())
        self.hist['los_rate_filt'].append(self._los_rate_filt.copy())
        self.hist['noise_total'].append(noise_mag)
        self.hist['locked'].append(self._locked)
        self.hist['r_apparent'].append(r_apparent.copy())

        return r_apparent, v_apparent, (self._locked and in_fov)

    def to_arrays(self):
        """Convert history lists to numpy arrays for plotting."""
        return {k: np.array(v) for k, v in self.hist.items()}


# ---------------------------------------------------------------
# Validation tests
# ---------------------------------------------------------------

if __name__ == '__main__':
    import sys
    print("=== SeekerModel Validation Tests ===\n")

    passed = 0
    total  = 0

    def check(name, condition):
        global passed, total
        total += 1
        status = "PASS" if condition else "FAIL"
        if condition: passed += 1
        print(f"  [{status}] {name}")

    # Test 1: initialises locked on target
    skr = SeekerModel(noise_std=0.0, glint_std=0.0, seed=0)
    pos_m = np.array([0.0, 0.0, 10.0])
    pos_t = np.array([2500.0, 0.0, 500.0])
    skr.reset(pos_m, pos_t)
    check("Initialises locked", skr._locked)

    # Test 2: with zero noise, apparent position ≈ true position
    r_app, v_app, valid = skr.update(
        pos_m, pos_t,
        np.array([300.0, 0.0, 50.0]),
        np.array([50.0, 0.0, 0.0]),
        dt=0.01
    )
    err = np.linalg.norm(r_app - pos_t)
    check(f"Zero noise: apparent pos error {err:.2f}m < 5m", err < 5.0)

    # Test 3: returns valid when in FOV
    check("Valid flag True when in FOV", valid)

    # Test 4: lock-loss when target exits FOV
    skr2 = SeekerModel(noise_std=0.0, glint_std=0.0, seed=0)
    skr2.reset(pos_m, pos_t)
    # Force gimbal to limit
    skr2._gimbal_pos = np.array([np.radians(65.0), 0.0])  # outside FOV
    _, _, valid2 = skr2.update(pos_m, pos_t,
                                np.zeros(3), np.zeros(3), dt=0.01)
    check("Lock-loss when gimbal outside FOV", not valid2)

    # Test 5: noise degrades apparent position
    skr3 = SeekerModel(noise_std=0.05, glint_std=0.0, seed=42)
    skr3.reset(pos_m, pos_t)
    errors = []
    for _ in range(100):
        r_a, _, _ = skr3.update(
            pos_m, pos_t,
            np.array([300.0, 0.0, 50.0]),
            np.array([50.0, 0.0, 0.0]), dt=0.01)
        errors.append(np.linalg.norm(r_a - pos_t))
    mean_err = np.mean(errors)
    check(f"Noise increases apparent pos error (mean={mean_err:.1f}m > 0)", mean_err > 0.0)

    # Test 6: glint activates at short range
    skr4 = SeekerModel(noise_std=0.0, glint_std=0.05, seed=7)
    skr4.reset(pos_m, pos_t)
    pos_close = pos_t - np.array([50.0, 0.0, 0.0])
    errors_close = []
    for _ in range(50):
        r_a, _, _ = skr4.update(
            pos_close, pos_t,
            np.array([400.0, 0.0, 0.0]),
            np.zeros(3), dt=0.01)
        errors_close.append(np.linalg.norm(r_a - pos_t))
    errors_far = []
    skr4.reset(pos_m, pos_t)
    for _ in range(50):
        r_a, _, _ = skr4.update(
            pos_m, pos_t,
            np.array([300.0, 0.0, 50.0]),
            np.zeros(3), dt=0.01)
        errors_far.append(np.linalg.norm(r_a - pos_t))
    check(f"Glint: close errors ({np.mean(errors_close):.1f}m) > far ({np.mean(errors_far):.1f}m)",
          np.mean(errors_close) > np.mean(errors_far))

    # Test 7: ECM increases apparent position error
    skr5_clean = SeekerModel(noise_std=0.001, glint_std=0.0, seed=0)
    skr5_ecm   = SeekerModel(noise_std=0.001, glint_std=0.0,
                              ecm_mode=True, ecm_bias=0.05,
                              ecm_noise_std=0.02, seed=0)
    skr5_clean.reset(pos_m, pos_t)
    skr5_ecm.reset(pos_m, pos_t)
    clean_errs, ecm_errs = [], []
    for _ in range(100):
        r_c, _, _ = skr5_clean.update(pos_m, pos_t,
                                       np.array([300.0, 0.0, 50.0]),
                                       np.array([50.0, 0.0, 0.0]), dt=0.01)
        r_e, _, _ = skr5_ecm.update(  pos_m, pos_t,
                                       np.array([300.0, 0.0, 50.0]),
                                       np.array([50.0, 0.0, 0.0]), dt=0.01)
        clean_errs.append(np.linalg.norm(r_c - pos_t))
        ecm_errs.append(  np.linalg.norm(r_e - pos_t))
    check(f"ECM: error ({np.mean(ecm_errs):.1f}m) > clean ({np.mean(clean_errs):.1f}m)",
          np.mean(ecm_errs) > np.mean(clean_errs))

    # Test 8: to_arrays returns numpy arrays
    arr = skr.to_arrays()
    check("to_arrays() returns dict of ndarrays",
          all(isinstance(v, np.ndarray) for v in arr.values()))

    print(f"\n{passed}/{total} tests passed")
    if passed < total:
        sys.exit(1)