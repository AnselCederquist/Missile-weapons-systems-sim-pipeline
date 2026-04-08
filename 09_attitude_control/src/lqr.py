"""
lqr.py -- LQR Attitude Controller
Project 09 -- Attitude Control Simulator

State for LQR:  x = [phi_err, theta_err, psi_err, p, q_rate, r]
Control input:  u = [delta_roll, delta_pitch, delta_yaw]  (fin deflection commands, rad)
"""

import numpy as np
from scipy.linalg import solve_continuous_are

from actuator import (
    I_XX, I_YY, I_ZZ,
    S_REF, C_REF, B_REF,
    CL_DELTA, CM_DELTA, CN_DELTA,
    DELTA_MAX,
)

# ---------------------------------------------------------------
# Nominal flight condition for linearization
# ---------------------------------------------------------------
RHO_NOM   = 0.9
V_NOM     = 425.0
Q_BAR_NOM = 0.5 * RHO_NOM * V_NOM**2   # ~81,000 Pa

# Default LQR weights — adjust here to tune
DEFAULT_Q = [2.0, 2.0, 1.0, 15.0, 15.0, 15.0]   # [phi, theta, psi, p, q, r]
DEFAULT_R = [40.0, 7.0, 30.0]                    # [delta_roll, delta_pitch, delta_yaw]


def build_system_matrices(q_bar=Q_BAR_NOM):
    eff_roll  = q_bar * S_REF * B_REF * CL_DELTA
    eff_pitch = q_bar * S_REF * C_REF * CM_DELTA
    eff_yaw   = q_bar * S_REF * B_REF * CN_DELTA

    A = np.zeros((6, 6))
    A[0, 3] = 1.0
    A[1, 4] = 1.0
    A[2, 5] = 1.0

    B = np.zeros((6, 3))
    B[3, 0] = eff_roll  / I_XX
    B[4, 1] = eff_pitch / I_YY
    B[5, 2] = eff_yaw   / I_ZZ

    return A, B


def design_lqr(q_bar=Q_BAR_NOM, Q_diag=None, R_diag=None):
    if Q_diag is None:
        Q_diag = DEFAULT_Q
    if R_diag is None:
        R_diag = DEFAULT_R

    A, B = build_system_matrices(q_bar)
    Q = np.diag(Q_diag)
    R = np.diag(R_diag)

    P       = solve_continuous_are(A, B, Q, R)
    K       = np.linalg.solve(R, B.T @ P)
    A_cl    = A - B @ K
    eigvals = np.linalg.eigvals(A_cl)

    return K, A, B, eigvals, Q_diag, R_diag


class LQRController:
    """
    LQR attitude controller with dynamic-pressure gain scheduling.
    Q and R weights stored on instance for runtime inspection.
    """

    def __init__(self, q_bar=Q_BAR_NOM, Q_diag=None, R_diag=None):
        self.K, _, _, self.eigvals, self.Q_diag, self.R_diag = design_lqr(
            q_bar, Q_diag, R_diag)
        self._verify_stability()

    def _verify_stability(self):
        unstable = [e for e in self.eigvals if e.real >= 0]
        if unstable:
            raise ValueError(f"LQR closed-loop unstable: {unstable}")

    def compute(self, euler_current, omega_current,
                euler_desired, omega_desired=None):
        if omega_desired is None:
            omega_desired = np.zeros(3)

        att_error  = euler_current - euler_desired
        att_error  = (att_error + np.pi) % (2 * np.pi) - np.pi
        rate_error = omega_current - omega_desired

        x_error = np.concatenate([att_error, rate_error])
        u = -self.K @ x_error
        return np.clip(u, -DELTA_MAX, DELTA_MAX)

    def gain_schedule(self, q_bar_current):
        self.K, _, _, self.eigvals, self.Q_diag, self.R_diag = design_lqr(
            q_bar_current, self.Q_diag, self.R_diag)
        self._verify_stability()

    def describe_settings(self):
        print(f"  LQR Q [phi,tht,psi,p,q,r]: {self.Q_diag}")
        print(f"  LQR R [roll,pitch,yaw]:     {self.R_diag}")

    def print_gains(self):
        state_labels = ['phi_err', 'theta_err', 'psi_err', 'p', 'q', 'r']
        input_labels = ['delta_roll', 'delta_pitch', 'delta_yaw']
        print("LQR Gain Matrix K (3x6):")
        print(f"  {'':>12}" + "".join(f"  {l:>10}" for l in state_labels))
        for i, il in enumerate(input_labels):
            row = "".join(f"  {self.K[i, j]:>10.4f}" for j in range(6))
            print(f"  {il:>12}{row}")
        print("\nClosed-loop eigenvalues:")
        for i, e in enumerate(self.eigvals):
            print(f"  λ{i+1} = {e.real:+.3f} {e.imag:+.3f}j  "
                  f"({'STABLE' if e.real < 0 else 'UNSTABLE'})")


if __name__ == '__main__':
    print("=== LQR Controller Validation ===\n")

    A, B = build_system_matrices()
    assert A.shape == (6, 6)
    assert B.shape == (6, 3)
    print("TEST 1 PASS: System matrices correct shape")

    C_mat = np.hstack([np.linalg.matrix_power(A, i) @ B for i in range(6)])
    rank  = np.linalg.matrix_rank(C_mat)
    assert rank == 6
    print(f"TEST 2 PASS: System fully controllable (rank={rank})")

    ctrl = LQRController()
    assert all(e.real < 0 for e in ctrl.eigvals)
    print("TEST 3 PASS: Closed-loop stable")

    u = ctrl.compute(np.zeros(3), np.zeros(3), np.zeros(3))
    assert np.allclose(u, 0.0)
    print("TEST 4 PASS: Zero error → zero command")

    u = ctrl.compute(np.array([0.0, 0.1, 0.0]), np.zeros(3), np.zeros(3))
    assert u[1] > 0, f"Pitch sign wrong: {u[1]:.4f}"
    print(f"TEST 5 PASS: Pitch correction ({np.degrees(u[1]):.2f}°)")

    ctrl_low = LQRController(q_bar=20000.0)
    assert all(e.real < 0 for e in ctrl_low.eigvals)
    print("TEST 6 PASS: Gain scheduling stable")

    print("\n=== Settings ===")
    ctrl.describe_settings()
    print("\n=== Gain Matrix ===")
    ctrl.print_gains()
    print("\n=== All tests passed ===")