"""
autopilot.py -- Three-Loop Acceleration Autopilot
Project 10 -- Three-Loop Autopilot
"""

import numpy as np
import time as _time

# ---------------------------------------------------------------
# TUNABLE CONSTANTS
# ---------------------------------------------------------------
DELTA_MAX_RAD  = np.radians(20.0)  # actuator hard limit (match actuator.py)
OMEGA_CMD_MAX  = 1.5               # rad/s max lateral rate command
INT_CLIP       = 1.2               # integrator hard clip
INT_LEAKAGE    = 1.00              # leaky integrator coefficient
Q_BAR_FLOOR    = 12000.0            # Pa minimum q_bar for scheduling
SCALE_MIN      = 0.6               # minimum gain scale factor
SCALE_MAX      = 2.5               # maximum gain scale factor
KI_A_SCALE     = 0.6              # Ki_a *= scale * this
PRINT_EVERY    = 100               # debug print every N compute() calls
# ---------------------------------------------------------------


class AutopilotController:

    # Nominal gains at Q_NOM — balanced from best 98.2m run + fixed roll
    Ka_nom      =  0.8
    Ki_a_nom    =  0.45
    Kp_rate_nom = -0.28
    Kd_rate_nom =  0.00
    Kp_roll_nom = -0.02   # FIXED — not scheduled

    Q_NOM = 50_000.0

    def __init__(self, q_bar=50_000.0):
        self._t0         = _time.time()
        self._call_count = 0
        self._gs_count   = 0
        self._sat_count  = 0

        self.Ka      = self.Ka_nom
        self.Ki_a    = self.Ki_a_nom
        self.Kp_rate = self.Kp_rate_nom
        self.Kd_rate = self.Kd_rate_nom
        self.Kp_roll = self.Kp_roll_nom

        self.int_error  = np.zeros(2)
        self.prev_omega = np.zeros(3)

        self.Q_diag = [self.Ka_nom, self.Ki_a_nom, self.Kp_rate_nom]
        self.R_diag = [self.Kp_roll_nom, self.Kd_rate_nom]

        self.gain_schedule(max(q_bar, Q_BAR_FLOOR))
        self.Kp_roll = self.Kp_roll_nom   # enforce fixed roll gain

        print(f"\n{'='*80}")
        print(f"[AUTOPILOT INIT]  wall=+0.0s")
        print(f"  Nominal gains (Q_NOM={self.Q_NOM:.0f} Pa):")
        print(f"    Ka_nom={self.Ka_nom}  Ki_a_nom={self.Ki_a_nom}  "
              f"Kp_rate_nom={self.Kp_rate_nom}  Kd_rate_nom={self.Kd_rate_nom}  "
              f"Kp_roll_nom={self.Kp_roll_nom} (FIXED)")
        print(f"  Constants:")
        print(f"    OMEGA_CMD_MAX={OMEGA_CMD_MAX}  INT_CLIP={INT_CLIP}  "
              f"INT_LEAKAGE={INT_LEAKAGE}")
        print(f"    Q_BAR_FLOOR={Q_BAR_FLOOR}  SCALE [{SCALE_MIN},{SCALE_MAX}]  "
              f"KI_A_SCALE={KI_A_SCALE}")
        print(f"    PRINT_EVERY={PRINT_EVERY} calls (~{PRINT_EVERY*0.01:.1f}s at dt=0.01)")
        print(f"  Scheduled gains at init:")
        print(f"    Ka={self.Ka:.5f}  Ki_a={self.Ki_a:.5f}  "
              f"Kp_rate={self.Kp_rate:.5f}  Kd_rate={self.Kd_rate:.5f}  "
              f"Kp_roll={self.Kp_roll:.5f} (FIXED)")
        print(f"{'='*80}\n")

    def gain_schedule(self, q_bar: float):
        """Lateral loops scheduled inversely with q_bar. Roll loop FIXED."""
        self._gs_count += 1
        q_bar_in = q_bar
        q_bar    = max(q_bar, Q_BAR_FLOOR)
        scale    = np.clip(self.Q_NOM / q_bar, SCALE_MIN, SCALE_MAX)

        prev = (self.Ka, self.Ki_a, self.Kp_rate, self.Kd_rate, self.Kp_roll)

        self.Ka      = self.Ka_nom      * scale
        self.Kp_rate = self.Kp_rate_nom * scale
        self.Kd_rate = self.Kd_rate_nom * scale
        self.Ki_a    = self.Ki_a_nom    * scale * KI_A_SCALE
        self.Kp_roll = self.Kp_roll_nom  # FIXED — never changes

        w = _time.time() - self._t0
        print(f"[GAIN SCHED #{self._gs_count:03d}]  wall=+{w:.1f}s  "
              f"q_bar_in={q_bar_in:.0f}  q_bar_used={q_bar:.0f}  "
              f"scale={scale:.4f}  (clip [{SCALE_MIN},{SCALE_MAX}])")
        print(f"  Ka:      {prev[0]:.5f} -> {self.Ka:.5f}  (nom {self.Ka_nom})")
        print(f"  Ki_a:    {prev[1]:.5f} -> {self.Ki_a:.5f}  "
              f"(nom {self.Ki_a_nom} * scale * {KI_A_SCALE})")
        print(f"  Kp_rate: {prev[2]:.5f} -> {self.Kp_rate:.5f}  "
              f"(nom {self.Kp_rate_nom})")
        print(f"  Kd_rate: {prev[3]:.5f} -> {self.Kd_rate:.5f}  "
              f"(nom {self.Kd_rate_nom})")
        print(f"  Kp_roll: {prev[4]:.5f} -> {self.Kp_roll:.5f}  "
              f"(FIXED nom {self.Kp_roll_nom})")

    def compute(self,
                euler:      np.ndarray,
                omega:      np.ndarray,
                accel_body: np.ndarray,
                accel_cmd:  np.ndarray,
                dt:         float = 0.01) -> np.ndarray:

        self._call_count += 1
        t_sim    = self._call_count * dt
        t_wall   = _time.time() - self._t0
        do_print = (self._call_count % PRINT_EVERY == 0)

        # Step 1: rotate inertial accel_cmd to body frame (no filter — avoids lag)
        dcm            = _euler_to_dcm(euler)
        accel_cmd_body = dcm @ accel_cmd
        a_meas         = accel_body[1:3]
        a_cmd_lat      = accel_cmd_body[1:3]

        # Step 2: acceleration loop (PI with leaky integrator + anti-windup)
        a_error        = a_cmd_lat - a_meas
        int_before     = self.int_error.copy()
        self.int_error = INT_LEAKAGE * self.int_error + a_error * dt
        self.int_error = np.clip(self.int_error, -INT_CLIP, INT_CLIP)

        omega_cmd_lat_raw = self.Ka * a_error + self.Ki_a * self.int_error
        omega_cmd_lat     = np.clip(omega_cmd_lat_raw, -OMEGA_CMD_MAX, OMEGA_CMD_MAX)
        omega_cmd         = np.array([0.0, omega_cmd_lat[0], omega_cmd_lat[1]])

        # Step 3: rate loop (PD) — no extra roll damping multiplier
        rate_error_raw = omega_cmd - omega
        rate_error     = rate_error_raw.copy()

        omega_dot      = (omega - self.prev_omega) / max(dt, 1e-6)
        self.prev_omega = omega.copy()

        u_raw    = np.zeros(3)
        u_raw[0] = self.Kp_roll * rate_error[0]
        u_raw[1] = self.Kp_rate * rate_error[1] + self.Kd_rate * omega_dot[1]
        u_raw[2] = self.Kp_rate * rate_error[2] + self.Kd_rate * omega_dot[2]

        u         = np.clip(u_raw, -DELTA_MAX_RAD, DELTA_MAX_RAD)
        saturated = not np.allclose(u_raw, u)
        if saturated:
            self._sat_count += 1

        if do_print:
            print(f"\n{'─'*75}")
            print(f"[COMPUTE #{self._call_count:06d}]  t_sim={t_sim:.2f}s  "
                  f"t_wall=+{t_wall:.1f}s  dt={dt}")
            print(f"  euler(deg)       phi={np.degrees(euler[0]):+8.3f}  "
                  f"theta={np.degrees(euler[1]):+8.3f}  psi={np.degrees(euler[2]):+8.3f}")
            print(f"  omega(deg/s)     p={np.degrees(omega[0]):+8.3f}  "
                  f"q={np.degrees(omega[1]):+8.3f}  r={np.degrees(omega[2]):+8.3f}")
            print(f"  accel_cmd(iner)  ax={accel_cmd[0]:+8.3f}  "
                  f"ay={accel_cmd[1]:+8.3f}  az={accel_cmd[2]:+8.3f}  m/s2")
            print(f"  accel_cmd(body)  ax={accel_cmd_body[0]:+8.3f}  "
                  f"ay={accel_cmd_body[1]:+8.3f}  az={accel_cmd_body[2]:+8.3f}  m/s2")
            print(f"  accel_body(meas) ax={accel_body[0]:+8.3f}  "
                  f"ay={accel_body[1]:+8.3f}  az={accel_body[2]:+8.3f}  m/s2")
            print(f"  a_meas(lat)      ay={a_meas[0]:+8.3f}  az={a_meas[1]:+8.3f}")
            print(f"  a_cmd_lat        ay={a_cmd_lat[0]:+8.3f}  az={a_cmd_lat[1]:+8.3f}")
            print(f"  a_error          ay={a_error[0]:+8.3f}  az={a_error[1]:+8.3f}")
            print(f"  int_before       {int_before}  leakage={INT_LEAKAGE}")
            print(f"  int_after        {self.int_error}  clip=+-{INT_CLIP}")
            print(f"  Ka={self.Ka:.5f}  Ki_a={self.Ki_a:.5f}  "
                  f"OMEGA_CMD_MAX={OMEGA_CMD_MAX}")
            print(f"  omega_cmd_lat_raw  {omega_cmd_lat_raw}")
            print(f"  omega_cmd_lat_clip {omega_cmd_lat}")
            print(f"  omega_cmd        p={omega_cmd[0]:+7.4f}  "
                  f"q={omega_cmd[1]:+7.4f}  r={omega_cmd[2]:+7.4f}  rad/s")
            print(f"  rate_err_raw     p={rate_error_raw[0]:+7.4f}  "
                  f"q={rate_error_raw[1]:+7.4f}  r={rate_error_raw[2]:+7.4f}")
            print(f"  Kp_rate={self.Kp_rate:.5f}  Kd_rate={self.Kd_rate:.5f}  "
                  f"Kp_roll={self.Kp_roll:.5f} (FIXED)")
            print(f"  omega_dot(d/s2)  p={np.degrees(omega_dot[0]):+8.2f}  "
                  f"q={np.degrees(omega_dot[1]):+8.2f}  r={np.degrees(omega_dot[2]):+8.2f}")
            print(f"  u_raw(deg)   roll={np.degrees(u_raw[0]):+8.3f}  "
                  f"pitch={np.degrees(u_raw[1]):+8.3f}  yaw={np.degrees(u_raw[2]):+8.3f}")
            print(f"  u_clip(deg)  roll={np.degrees(u[0]):+8.3f}  "
                  f"pitch={np.degrees(u[1]):+8.3f}  yaw={np.degrees(u[2]):+8.3f}")
            print(f"  SATURATED={saturated}  cumulative_sat={self._sat_count}")
            print(f"{'─'*75}")

        return u

    def reset(self):
        self.int_error   = np.zeros(2)
        self.prev_omega  = np.zeros(3)
        self._call_count = 0
        self._sat_count  = 0
        print("[AUTOPILOT RESET] integrator cleared  sat_count reset")


def _euler_to_dcm(euler: np.ndarray) -> np.ndarray:
    phi, theta, psi = euler
    cp, sp = np.cos(phi), np.sin(phi)
    ct, st = np.cos(theta), np.sin(theta)
    cy, sy = np.cos(psi), np.sin(psi)
    return np.array([
        [ct*cy,                  ct*sy,                 -st    ],
        [sp*st*cy - cp*sy,       sp*st*sy + cp*cy,       sp*ct ],
        [cp*st*cy + sp*sy,       cp*st*sy - sp*cy,       cp*ct ]
    ])


def compute_accel_body(state, thrust, mass, rho, S_REF, CN_ALPHA, CD0,
                       AERO_ALPHA_CLAMP_DEG=10.0):
    from quaternion import quat_norm, quat_to_rotmat
    from actuator   import compute_aoa_sideslip

    vel_i  = state[3:6]
    q      = quat_norm(state[6:10])
    R_bi   = quat_to_rotmat(q)
    vel_b  = R_bi.T @ vel_i
    V      = max(np.linalg.norm(vel_b), 1.0)
    q_bar  = 0.5 * rho * V**2

    alpha, beta = compute_aoa_sideslip(vel_b)
    alpha_f = np.clip(alpha, -np.radians(AERO_ALPHA_CLAMP_DEG),
                              np.radians(AERO_ALPHA_CLAMP_DEG))
    beta_f  = np.clip(beta,  -np.radians(AERO_ALPHA_CLAMP_DEG),
                              np.radians(AERO_ALPHA_CLAMP_DEG))

    F_thrust_b = np.array([thrust, 0.0, 0.0])
    cd         = CD0 + 0.1 * (alpha_f**2 + beta_f**2)
    vel_hat_b  = vel_b / V
    F_drag_b   = -cd * q_bar * S_REF * vel_hat_b
    F_normal_b = np.array([0.0,
                            -q_bar * S_REF * CN_ALPHA * beta_f,
                             q_bar * S_REF * CN_ALPHA * alpha_f])
    F_normal_b -= np.dot(F_normal_b, vel_hat_b) * vel_hat_b

    return (F_thrust_b + F_drag_b + F_normal_b) / mass


if __name__ == '__main__':
    import sys
    print("=== AutopilotController Validation Tests ===\n")
    ap = AutopilotController(q_bar=50_000.0)
    dt = 0.01
    passed = 0
    total  = 0

    def check(name, condition):
        global passed, total
        total += 1
        status = "PASS" if condition else "FAIL"
        if condition: passed += 1
        print(f"  [{status}] {name}")

    ap.reset()
    u = ap.compute(np.zeros(3), np.zeros(3), np.zeros(3), np.zeros(3), dt)
    check("Zero command -> zero fin deflection", np.allclose(u, 0, atol=1e-6))

    ap.reset()
    u = ap.compute(np.zeros(3), np.zeros(3), np.zeros(3),
                   np.array([0.0, 10.0, 0.0]), dt)
    check("Positive pitch accel cmd -> negative delta_pitch", u[1] < 0)

    ap.reset()
    u = ap.compute(np.zeros(3), np.array([1.0, 0.0, 0.0]),
                   np.zeros(3), np.zeros(3), dt)
    check("Positive roll rate -> opposing delta_roll", u[0] > 0)

    ap.reset()
    u = ap.compute(np.zeros(3), np.zeros(3), np.zeros(3),
                   np.array([0.0, 1000.0, 1000.0]), dt)
    check("Fin commands clamped to +-20 deg",
          np.all(np.abs(u) <= DELTA_MAX_RAD + 1e-6))

    ap.gain_schedule(100_000.0); Ka_high = ap.Ka
    ap.gain_schedule(50_000.0);  Ka_nom  = ap.Ka
    check("Ka halves at 2x q_bar", abs(Ka_high - Ka_nom * 0.5) < 1e-6)

    ap.gain_schedule(25_000.0); Ka_low = ap.Ka
    check("Ka increases at 0.5x q_bar (clipped by SCALE_MAX)", Ka_low > Ka_nom)

    ap.reset(); ap.gain_schedule(50_000.0)
    for _ in range(1000):
        ap.compute(np.zeros(3), np.zeros(3), np.zeros(3),
                   np.array([0.0, 1000.0, 0.0]), dt)
    check("Integrator clamped",
          np.all(np.abs(ap.int_error) <= INT_CLIP + 1e-6))

    ap.reset()
    check("Reset clears integrator", np.allclose(ap.int_error, 0))

    print(f"\n{passed}/{total} tests passed")
    if passed < total:
        sys.exit(1)