"""
pd_controller_reference.py -- Reference PD Attitude Controller
Project 09 -- Attitude Control Simulator

This file is NOT used by the main simulator (attitude_sim.py).
It is preserved as a reference implementation showing the simpler
proportional-derivative approach that was the starting point before
the LQR was adopted.

WHY THIS WAS SUPERSEDED BY THE LQR
------------------------------------
1. No systematic gain selection. Kp and Kd are chosen by trial and error
   with no stability guarantee. The LQR CARE solution guarantees all
   closed-loop eigenvalues are negative real by construction.

2. No dynamic pressure scaling. These gains are only valid at one flight
   condition. As q_bar changes with altitude and velocity, the effective
   control authority changes but the gains do not. The LQR gain scheduling
   recomputes K every 1s to account for this.

3. Separate pitch/yaw channels with no coupling. The LQR naturally handles
   cross-coupling through the full 6-state error vector.

4. No rate penalty systematization. The Q/R framework in the LQR gives a
   principled way to trade off attitude tracking vs. control effort vs.
   rate damping. PD requires iterating Kd independently.

HYBRID PD+LQR ATTEMPT
-----------------------
A hybrid was attempted that used the PD controller for rate stabilization
and the LQR for attitude tracking. This produced worse results than either
controller alone. The two controllers had mismatched bandwidth assumptions
and produced oscillatory fin commands in the terminal phase. Abandoned.

USAGE (for comparison / experimentation)
-----------------------------------------
To use this controller in attitude_sim.py, replace the LQRController
instantiation and compute() call with PDController. The interface is
intentionally matched.

    from pd_controller_reference import PDController
    controller = PDController()
    ...
    u = controller.compute(euler, omega, euler_des)
    # u[0] = delta_roll, u[1] = delta_pitch, u[2] = delta_yaw

KNOWN TUNING OBSERVATIONS
---------------------------
With the engagement geometry in attitude_sim.py (2500m target, 21.2° launch,
800N sustain thrust):

  Kp_pitch = 2.0, Kd_pitch = 0.8  →  stable but sluggish, ~120-150m miss
  Kp_pitch = 3.0, Kd_pitch = 1.2  →  slightly more responsive, similar miss
  Kp_pitch > 4.0                   →  oscillation onset
  Kd_pitch < 0.5                   →  violent oscillation

The fundamental limitation is not the gains — it is that proportional
control on attitude error does not account for the LOS rate geometry of
proportional navigation. The LQR with the guidance-to-attitude interface
does not fix this either, but the systematic gain selection handles the
stability-bandwidth tradeoff more robustly across the flight envelope.
"""

import numpy as np

# Actuator limits (must match actuator.py)
DELTA_MAX = np.radians(20.0)


class PDController:
    """
    Simple proportional-derivative attitude controller.

    Controls pitch, yaw, and roll independently using attitude error
    and angular rate feedback.

    Interface matches LQRController.compute() for drop-in comparison.
    """

    # Default gains — tuned for nominal engagement geometry
    # Adjust these for different flight conditions
    DEFAULT_GAINS = {
        'Kp_roll':  1.0,
        'Kd_roll':  0.4,
        'Kp_pitch': 2.5,
        'Kd_pitch': 1.0,
        'Kp_yaw':   1.5,
        'Kd_yaw':   0.6,
    }

    def __init__(self, gains=None):
        g = gains if gains is not None else self.DEFAULT_GAINS
        self.Kp_roll  = g['Kp_roll']
        self.Kd_roll  = g['Kd_roll']
        self.Kp_pitch = g['Kp_pitch']
        self.Kd_pitch = g['Kd_pitch']
        self.Kp_yaw   = g['Kp_yaw']
        self.Kd_yaw   = g['Kd_yaw']

    def compute(self, euler_current, omega_current,
                euler_desired, omega_desired=None):
        """
        Compute fin deflection commands.

        Parameters
        ----------
        euler_current : array [phi, theta, psi] — current Euler angles (rad)
        omega_current : array [p, q, r] — current body angular rates (rad/s)
        euler_desired : array [phi, theta, psi] — desired Euler angles (rad)
        omega_desired : array [p, q, r] — desired angular rates (rad/s), default zero

        Returns
        -------
        u : array [delta_roll, delta_pitch, delta_yaw] — fin commands (rad)
        """
        if omega_desired is None:
            omega_desired = np.zeros(3)

        # Attitude error with angle wrapping
        att_err = euler_current - euler_desired
        att_err = (att_err + np.pi) % (2 * np.pi) - np.pi

        # Rate error
        rate_err = omega_current - omega_desired

        phi_err, theta_err, psi_err = att_err
        p_err, q_err, r_err = rate_err

        # Independent PD on each channel
        delta_roll  = -(self.Kp_roll  * phi_err   + self.Kd_roll  * p_err)
        delta_pitch = -(self.Kp_pitch * theta_err  + self.Kd_pitch * q_err)
        delta_yaw   = -(self.Kp_yaw   * psi_err    + self.Kd_yaw   * r_err)

        u = np.array([delta_roll, delta_pitch, delta_yaw])
        return np.clip(u, -DELTA_MAX, DELTA_MAX)

    def gain_schedule(self, q_bar_current):
        """
        No-op — included for interface compatibility with LQRController.
        A real PD gain scheduler would scale Kp proportional to 1/q_bar
        to maintain constant closed-loop bandwidth across the flight envelope.
        """
        pass

    def describe_settings(self):
        print(f"  PD Kp [roll, pitch, yaw]: "
              f"[{self.Kp_roll}, {self.Kp_pitch}, {self.Kp_yaw}]")
        print(f"  PD Kd [roll, pitch, yaw]: "
              f"[{self.Kd_roll}, {self.Kd_pitch}, {self.Kd_yaw}]")


# ---------------------------------------------------------------
# Stability analysis (informational — not a substitute for CARE)
# ---------------------------------------------------------------

def analyze_pd_stability(Kp_pitch, Kd_pitch, I_YY=2.0, cm_delta=-0.5,
                          q_bar=50000.0, S_REF=0.00785, C_REF=0.1):
    """
    Approximate closed-loop poles for the pitch channel.

    Simplified pitch dynamics (ignoring aerodynamic damping):
        I_YY * theta_ddot = q_bar * S * c * CM_delta * delta_pitch
        delta_pitch = -(Kp * theta_err + Kd * q_rate)

    Closed-loop characteristic equation:
        I_YY * s^2 + Kd * eff * s + Kp * eff = 0

    where eff = q_bar * S * c * |CM_delta|
    """
    eff = q_bar * S_REF * C_REF * abs(cm_delta)
    a   = I_YY
    b   = Kd_pitch * eff
    c   = Kp_pitch * eff

    discriminant = b**2 - 4*a*c
    if discriminant >= 0:
        s1 = (-b + np.sqrt(discriminant)) / (2*a)
        s2 = (-b - np.sqrt(discriminant)) / (2*a)
        print(f"  Overdamped poles: s1={s1:.3f}, s2={s2:.3f}")
        stable = s1 < 0 and s2 < 0
    else:
        sigma = -b / (2*a)
        omega = np.sqrt(-discriminant) / (2*a)
        print(f"  Underdamped poles: {sigma:.3f} ± {omega:.3f}j")
        zeta  = -sigma / np.sqrt(sigma**2 + omega**2)
        print(f"  Damping ratio: {zeta:.3f}")
        stable = sigma < 0

    print(f"  Stable: {stable}")
    return stable


if __name__ == '__main__':
    print("=== PD Controller Reference ===\n")

    ctrl = PDController()
    ctrl.describe_settings()

    print("\n--- Zero error test ---")
    u = ctrl.compute(np.zeros(3), np.zeros(3), np.zeros(3))
    assert np.allclose(u, 0.0), f"Zero error should give zero command: {u}"
    print(f"Zero error → zero command: PASS")

    print("\n--- Pitch correction direction test ---")
    # Positive pitch error (nose too high) → negative pitch command (push nose down)
    u = ctrl.compute(np.array([0.0, 0.1, 0.0]), np.zeros(3), np.zeros(3))
    assert u[1] < 0, f"Positive pitch error should give negative command: {u[1]:.4f}"
    print(f"Positive θ_err → δ_pitch = {np.degrees(u[1]):.2f}°: PASS")

    print("\n--- Approximate closed-loop pole analysis (nominal q_bar) ---")
    analyze_pd_stability(
        Kp_pitch=PDController.DEFAULT_GAINS['Kp_pitch'],
        Kd_pitch=PDController.DEFAULT_GAINS['Kd_pitch'],
    )

    print("\n--- Gain schedule no-op test ---")
    ctrl.gain_schedule(50000.0)
    print("gain_schedule() no-op: PASS")

    print("\n=== Reference implementation ready ===")
    print("To use in attitude_sim.py:")
    print("  from pd_controller_reference import PDController")
    print("  controller = PDController()")