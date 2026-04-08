"""
actuator.py -- Fin deflection model and aerodynamic moment computation
Project 09 -- Attitude Control Simulator

4-fin configuration in + orientation (fins at 0°, 90°, 180°, 270°).
Fin deflections map to roll, pitch, and yaw moments.

Fin numbering:
  Fin 1: top    (+Z body, 90°)
  Fin 2: right  (+Y body, 0°)
  Fin 3: bottom (-Z body, 270°)
  Fin 4: left   (-Y body, 180°)

Control mixing:
  Pitch:  (delta1 - delta3) / 2   — top/bottom differential
  Yaw:    (delta2 - delta4) / 2   — right/left differential
  Roll:   (delta1 + delta2 + delta3 + delta4) / 4  — collective (requires cant angle)
"""

import numpy as np


# ---------------------------------------------------------------
# Missile geometry and aerodynamic parameters
# (consistent with Projects 02, 04, 06, 07)
# ---------------------------------------------------------------

S_REF   = 0.00785       # m², reference area (pi * (0.05)^2)
C_REF   = 0.100         # m, reference length (body diameter)
B_REF   = 0.100         # m, reference span (same as diameter for this config)

# Control effectiveness derivatives
CM_DELTA    = -0.5     # /rad, pitching moment per fin deflection (stabilizing)
CN_DELTA    = -0.5     # /rad, yawing moment per fin deflection
CL_DELTA    =  0.02     # /rad, rolling moment per fin deflection (via cant angle)

# Aerodynamic stability derivatives
CM_ALPHA    = -0.08     # /rad, pitching moment per AoA (negative = stable, XCP > XCG)
CM_Q        = -10.0     # /rad/s, pitch damping derivative (Cmq)
CN_BETA     = -0.08     # /rad, yawing moment per sideslip angle
CN_R        = -10.0     # /rad/s, yaw damping derivative

# Moments of inertia (consistent with 20kg, 100mm x 1000mm missile)
I_XX =  0.05            # kg·m², roll (thin body → small)
I_YY =  2.00            # kg·m², pitch
I_ZZ =  2.00            # kg·m², yaw
I_XZ =  0.0             # kg·m², cross product (symmetric body → 0)

# Fin limits
DELTA_MAX   = np.radians(20.0)  # rad, max fin deflection ±20°
DELTA_RATE  = np.radians(100.0) # rad/s, max deflection rate

# Actuator lag
TAU_ACT = 0.05          # s, first-order actuator lag (faster than guidance lag)


class FinActuator:
    """
    4-fin actuator model with position and rate limiting.
    State: achieved deflection for each of 4 fins.
    """

    def __init__(self):
        self.delta = np.zeros(4)        # current deflections [rad]
        self.delta_cmd = np.zeros(4)    # commanded deflections [rad]

    def reset(self):
        self.delta = np.zeros(4)
        self.delta_cmd = np.zeros(4)

    def set_commands(self, delta_roll, delta_pitch, delta_yaw):
        """
        Convert roll/pitch/yaw commands to individual fin deflections.

        delta_roll:  roll moment command (rad)
        delta_pitch: pitch moment command (rad)
        delta_yaw:   yaw moment command (rad)
        """
        # Inverse of the mixing matrix:
        # Pitch:  (d1 - d3) / 2 = delta_pitch  →  d1 = +delta_pitch, d3 = -delta_pitch
        # Yaw:    (d2 - d4) / 2 = delta_yaw    →  d2 = +delta_yaw,   d4 = -delta_yaw
        # Roll:   collective offset on all fins = delta_roll
        d1 =  delta_pitch + delta_roll
        d2 =  delta_yaw   + delta_roll
        d3 = -delta_pitch + delta_roll
        d4 = -delta_yaw   + delta_roll

        self.delta_cmd = np.clip(
            np.array([d1, d2, d3, d4]),
            -DELTA_MAX, DELTA_MAX
        )

    def step(self, dt):
        """
        First-order actuator lag with rate limiting.
        """
        for i in range(4):
            # Rate limit
            error = self.delta_cmd[i] - self.delta[i]
            max_step = DELTA_RATE * dt
            error_limited = np.clip(error, -max_step, max_step)

            # First-order lag
            d_delta = error_limited / TAU_ACT * dt
            self.delta[i] += d_delta
            self.delta[i] = np.clip(self.delta[i], -DELTA_MAX, DELTA_MAX)

    @property
    def deflections(self):
        return self.delta.copy()


def compute_moments(delta_fins, alpha, beta, p, q_rate, r,
                    q_bar, V):
    """
    Compute aerodynamic moments in body frame.

    Parameters
    ----------
    delta_fins : array [d1, d2, d3, d4] — fin deflections in rad
    alpha      : float — angle of attack (rad)
    beta       : float — sideslip angle (rad)
    p, q_rate, r : float — body angular rates (rad/s)
    q_bar      : float — dynamic pressure (Pa)
    V          : float — airspeed (m/s)

    Returns
    -------
    [L, M, N] : roll, pitch, yaw moments (N·m)
    """
    d1, d2, d3, d4 = delta_fins

    # Control mixing
    delta_pitch = (d1 - d3) / 2.0
    delta_yaw   = (d2 - d4) / 2.0
    delta_roll  = (d1 + d2 + d3 + d4) / 4.0

    # Non-dimensionalizing factor for damping
    if V > 1.0:
        damp_p = C_REF / (2.0 * V)
        damp_q = C_REF / (2.0 * V)
        damp_r = C_REF / (2.0 * V)
    else:
        damp_p = damp_q = damp_r = 0.0

    # Roll moment
    L = q_bar * S_REF * B_REF * (
        CL_DELTA * delta_roll
    )

    # Pitch moment
    M = q_bar * S_REF * C_REF * (
        CM_ALPHA * alpha
        + CM_DELTA * delta_pitch
        + CM_Q * damp_q * q_rate
    )

    # Yaw moment
    N = q_bar * S_REF * B_REF * (
        CN_BETA * beta
        + CN_DELTA * delta_yaw
        + CN_R * damp_r * r
    )

    return np.array([L, M, N])


def compute_angular_acceleration(moments, p, q_rate, r):
    """
    Euler's rigid body equations for angular acceleration.
    Assumes Ixz = 0 (symmetric body).

    I * omega_dot = moments - omega × (I * omega)

    Returns [p_dot, q_dot, r_dot]
    """
    L, M, N = moments

    # Gyroscopic terms: omega × (I * omega)
    gyro_x = (I_ZZ - I_YY) * q_rate * r
    gyro_y = (I_XX - I_ZZ) * r    * p
    gyro_z = (I_YY - I_XX) * p    * q_rate

    p_dot = (L - gyro_x) / I_XX
    q_dot = (M - gyro_y) / I_YY
    r_dot = (N - gyro_z) / I_ZZ

    return np.array([p_dot, q_dot, r_dot])


def compute_aoa_sideslip(v_body):
    """
    Compute angle of attack and sideslip from body-frame velocity.

    v_body: [u, v, w] velocity in body frame
    Returns: (alpha, beta) in radians
    """
    u, v, w = v_body
    V = np.linalg.norm(v_body)
    if V < 0.1:
        return 0.0, 0.0
    alpha = np.arctan2(w, u)    # AoA: angle between velocity and body X in XZ plane
    beta  = np.arcsin(v / V)    # Sideslip: lateral velocity component
    return alpha, beta


# ---------------------------------------------------------------
# Validation
# ---------------------------------------------------------------

if __name__ == '__main__':
    print("=== Actuator Validation ===")

    # Test 1: Neutral deflection → zero moments
    delta_neutral = np.zeros(4)
    moments = compute_moments(delta_neutral, 0.0, 0.0, 0.0, 0.0, 0.0, 50000.0, 400.0)
    assert np.allclose(moments, 0.0), f"Neutral moments nonzero: {moments}"
    print("TEST 1 PASS: Neutral deflection → zero moments")

    # Test 2: Pitch deflection → pitch moment only
    act = FinActuator()
    act.set_commands(0.0, np.radians(5.0), 0.0)
    delta_pitch_cmd = act.delta_cmd
    # d1 = +5°, d3 = -5°, d2=d4=0
    assert abs(delta_pitch_cmd[0] - np.radians(5.0)) < 1e-10
    assert abs(delta_pitch_cmd[2] + np.radians(5.0)) < 1e-10
    print("TEST 2 PASS: Pitch command mixing correct")

    # Test 3: Positive pitch deflection → negative pitch moment (stabilizing)
    act.delta = act.delta_cmd.copy()
    moments = compute_moments(act.delta, 0.0, 0.0, 0.0, 0.0, 0.0, 50000.0, 400.0)
    assert moments[1] < 0, f"Pitch moment sign wrong: {moments[1]}"
    print("TEST 3 PASS: Pitch deflection → pitch moment sign correct")

    # Test 4: Actuator rate limiting
    act2 = FinActuator()
    act2.set_commands(0.0, DELTA_MAX, 0.0)
    dt = 0.001
    for _ in range(10):
        act2.step(dt)
    # After 10ms, deflection should be limited by rate
    max_achievable = DELTA_RATE * dt * 10
    assert act2.delta[0] <= max_achievable + 1e-10, \
        f"Rate limit violated: {act2.delta[0]:.4f} > {max_achievable:.4f}"
    print("TEST 4 PASS: Actuator rate limiting")

    # Test 5: Angular acceleration from moments
    moments_test = np.array([1.0, 2.0, 3.0])
    alpha = compute_angular_acceleration(moments_test, 0.0, 0.0, 0.0)
    assert np.isclose(alpha[0], 1.0 / I_XX)
    assert np.isclose(alpha[1], 2.0 / I_YY)
    assert np.isclose(alpha[2], 3.0 / I_ZZ)
    print("TEST 5 PASS: Angular acceleration from moments")

    print("\n=== All actuator tests passed ===")