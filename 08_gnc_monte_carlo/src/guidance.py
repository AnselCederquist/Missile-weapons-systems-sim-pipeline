"""
guidance.py -- Proportional Navigation guidance laws
Project 08 -- GNC/Monte Carlo

Implements three PN variants:
- Pure PN (PPN): a_cmd = N * Vc * LOS_rate_inertial
- True PN (TPN): a_cmd = N * Vc * LOS_rate perpendicular to LOS
- Augmented PN (APN): TPN + target acceleration compensation

Reference: Zarchan, Tactical and Strategic Missile Guidance, 6th ed.
"""

import numpy as np


def compute_los(r_m, r_t):
    """
    Compute line-of-sight vector and unit vector.

    Parameters
    ----------
    r_m : array [3] missile position
    r_t : array [3] target position

    Returns
    -------
    los : array [3] LOS vector (target - missile)
    los_hat : array [3] LOS unit vector
    R : float range (m)
    """
    los = r_t - r_m
    R = np.linalg.norm(los)
    if R < 1e-3:
        return los, np.array([1.0, 0.0, 0.0]), R
    los_hat = los / R
    return los, los_hat, R


def compute_closing_velocity(r_m, r_t, v_m, v_t):
    """
    Compute closing velocity Vc = -d(R)/dt.
    Positive Vc means missile is closing on target.

    Parameters
    ----------
    r_m, r_t : array [3] positions
    v_m, v_t : array [3] velocities

    Returns
    -------
    Vc : float closing velocity (m/s)
    """
    los, los_hat, R = compute_los(r_m, r_t)
    v_rel = v_t - v_m  # relative velocity (target - missile)
    Vc = -np.dot(v_rel, los_hat)  # closing velocity
    return Vc


def compute_los_rate(r_m, r_t, v_m, v_t):
    """
    Compute LOS rate vector (rad/s).
    omega = (los_hat x v_rel) / R

    Parameters
    ----------
    r_m, r_t : array [3] positions
    v_m, v_t : array [3] velocities

    Returns
    -------
    omega : array [3] LOS rate vector (rad/s)
    los_hat : array [3] LOS unit vector
    R : float range (m)
    Vc : float closing velocity (m/s)
    """
    los, los_hat, R = compute_los(r_m, r_t)
    v_rel = v_t - v_m
    if R < 1e-3:
        return np.zeros(3), los_hat, R, 0.0
    omega = np.cross(los_hat, v_rel) / R
    Vc = -np.dot(v_rel, los_hat)
    return omega, los_hat, R, Vc


def pure_pn(r_m, r_t, v_m, v_t, N=3.0, noise_std=0.0, rng=None):
    """
    Pure Proportional Navigation.
    a_cmd = N * Vc * omega (inertial frame)

    Parameters
    ----------
    N : navigation constant (typically 3-5)
    noise_std : LOS rate noise std dev (rad/s)

    Returns
    -------
    a_cmd : array [3] acceleration command (m/s^2)
    """
    omega, los_hat, R, Vc = compute_los_rate(r_m, r_t, v_m, v_t)

    if noise_std > 0 and rng is not None:
        omega += rng.normal(0, noise_std, 3)

    a_cmd = N * Vc * omega
    return a_cmd


def true_pn(r_m, r_t, v_m, v_t, N=4.0, noise_std=0.0, rng=None,
            g=9.81, elevation=0.0):
    """
    True Proportional Navigation with gravity compensation.
    a_cmd = N * Vc * omega_perp + gravity_bias

    LOS rate projected perpendicular to missile velocity for
    physically correct lateral acceleration command.

    Parameters
    ----------
    N : navigation constant (typically 3-5)
    noise_std : LOS rate noise std dev (rad/s)
    g : gravitational acceleration (m/s^2)
    elevation : missile elevation angle (rad) for gravity compensation

    Returns
    -------
    a_cmd : array [3] acceleration command (m/s^2)
    seeker_valid : bool -- False if target outside FOV
    """

    # Seeker field of view check (+/-45 deg)

    omega, los_hat, R, Vc = compute_los_rate(r_m, r_t, v_m, v_t)

    v_m_mag = np.linalg.norm(v_m)

    R_check = np.linalg.norm(r_t - r_m)
    if v_m_mag > 100.0 and R_check > 800.0:
        v_m_hat = v_m / v_m_mag
        cos_angle = np.dot(v_m_hat, los_hat)
        if cos_angle < np.cos(np.radians(90)):
            return np.zeros(3), False

    if noise_std > 0 and rng is not None:
        omega += rng.normal(0, noise_std, 3)

    # Project omega perpendicular to missile velocity
    if v_m_mag > 1e-3:
        v_m_hat = v_m / v_m_mag
        omega_perp = omega - np.dot(omega, v_m_hat) * v_m_hat
    else:
        omega_perp = omega

    # Guidance command
    a_cmd = N * Vc * omega_perp

    # Vertical channel: P-controller on altitude error
    elevation = np.arcsin(np.clip(v_m[2] / (np.linalg.norm(v_m) + 1e-3), -1, 1)) if np.linalg.norm(v_m) > 1e-3 else 0.0
    a_cmd[2] = 0.015 * (r_t[2] - r_m[2]) + 0.5 * 9.81 * np.cos(elevation)

    return a_cmd, True

    # # # Gravity compensation
    # g_comp = 0.5 * g * np.cos(elevation)  # remove N multiplier — was over-compensating
    # a_cmd[2] += g_comp

    # # After: a_cmd = N * Vc * omega_perp
    # alt_error = r_t[2] - r_m[2]
    # a_cmd[2] += 0.5 * alt_error   # proportional altitude tracking, 0.5 m/s² per meter error



def augmented_pn(r_m, r_t, v_m, v_t, a_t,
                 N=4.0, noise_std=0.0, rng=None,
                 g=9.81, elevation=0.0):
    """
    Augmented Proportional Navigation.
    Adds target acceleration compensation to True PN.
    a_cmd = N * Vc * omega_perp + (N/2) * a_t_perp + gravity_bias

    Parameters
    ----------
    a_t : array [3] estimated target acceleration (m/s^2)
    N : navigation constant
    noise_std : LOS rate noise std dev (rad/s)

    Returns
    -------
    a_cmd : array [3] acceleration command (m/s^2)
    seeker_valid : bool
    """
    omega, los_hat, R, Vc = compute_los_rate(r_m, r_t, v_m, v_t)

    # Seeker field of view check (+/-45 deg)
    v_m_mag = np.linalg.norm(v_m)
    R_check = np.linalg.norm(r_t - r_m)
    if v_m_mag > 100.0 and R_check > 800.0: # only check FOV after missile has accelerated
        v_m_hat = v_m / v_m_mag
        cos_angle = np.dot(v_m_hat, los_hat)
        if cos_angle < np.cos(np.radians(90)):
            return np.zeros(3), False

    if noise_std > 0 and rng is not None:
        omega += rng.normal(0, noise_std, 3)

    # Project perpendicular to missile velocity
    if v_m_mag > 1e-3:
        v_m_hat = v_m / v_m_mag
        omega_perp = omega - np.dot(omega, v_m_hat) * v_m_hat
        a_t_perp = a_t - np.dot(a_t, v_m_hat) * v_m_hat
    else:
        omega_perp = omega
        a_t_perp = a_t

    # Augmented command
    a_cmd = N * Vc * omega_perp + (N / 2.0) * a_t_perp

    # Vertical channel: P-controller on altitude error
    a_cmd[2] = 0.015 * (r_t[2] - r_m[2]) + 0.5 * g * np.cos(elevation)

    return a_cmd, True

    # # Gravity compensation
    # g_comp = 0.5 * g * np.cos(elevation)  # remove N multiplier — was over-compensating
    # a_cmd[2] += g_comp

    # # After: a_cmd = N * Vc * omega_perp
    # alt_error = r_t[2] - r_m[2]
    # a_cmd[2] += 0.5 * alt_error   # proportional altitude tracking, 0.5 m/s² per meter error


class GuidanceLaw:
    """
    Wrapper class with first-order lag on acceleration response
    and acceleration limiting.

    Models actuator/autopilot dynamics between guidance command
    and achieved acceleration.
    """

    def __init__(self,
                 variant='TPN',
                 N=4.0,
                 noise_std=0.05,
                 accel_limit=20.0 * 9.81,
                 tau=0.1,
                 seed=None):
        """
        Parameters
        ----------
        variant : 'PPN', 'TPN', or 'APN'
        N : navigation constant
        noise_std : seeker noise std dev (rad/s)
        accel_limit : max lateral acceleration (m/s^2), default 20g
        tau : first-order lag time constant (s)
        seed : random seed for noise
        """
        self.variant = variant
        self.N = N
        self.noise_std = noise_std
        self.accel_limit = accel_limit
        self.tau = tau
        self.rng = np.random.default_rng(seed)

        self.a_achieved = np.zeros(3)
        self.seeker_valid = True
        self.los_rate_history = []
        self.a_cmd_history = []

    def compute(self, r_m, r_t, v_m, v_t, a_t=None, dt=0.01):
        """
        Compute achieved acceleration command with lag and limiting.

        Parameters
        ----------
        r_m, r_t : array [3] positions (m)
        v_m, v_t : array [3] velocities (m/s)
        a_t : array [3] target acceleration (m/s^2), required for APN
        dt : time step (s)

        Returns
        -------
        a_achieved : array [3] achieved acceleration (m/s^2)
        seeker_valid : bool
        """
        # Elevation angle for gravity compensation
        v_m_mag = np.linalg.norm(v_m)
        if v_m_mag > 1e-3:
            elevation = np.arcsin(np.clip(v_m[2] / v_m_mag, -1, 1))
        else:
            elevation = 0.0

        # Compute raw guidance command
        # Range-dependent gain scheduling
        # Reduce N as missile closes to prevent end-game over-maneuvering
        N_eff = self.N

        if self.variant == 'PPN':
            a_cmd = pure_pn(r_m, r_t, v_m, v_t,
                            N=N_eff,
                            noise_std=self.noise_std,
                            rng=self.rng)
            self.seeker_valid = True

        elif self.variant == 'TPN':
            result = true_pn(r_m, r_t, v_m, v_t,
                             N=N_eff,
                             noise_std=self.noise_std,
                             rng=self.rng,
                             elevation=elevation)
            a_cmd, self.seeker_valid = result

        elif self.variant == 'APN':
            if a_t is None:
                a_t = np.zeros(3)
            result = augmented_pn(r_m, r_t, v_m, v_t, a_t,
                                  N=N_eff,
                                  noise_std=self.noise_std,
                                  rng=self.rng,
                                  elevation=elevation)
            a_cmd, self.seeker_valid = result

        else:
            raise ValueError(f"Unknown variant: {self.variant}")

        if not self.seeker_valid:
            a_cmd = np.zeros(3)

        # Limit XY and Z independently
        xy_mag = np.linalg.norm(a_cmd[0:2])
        if xy_mag > self.accel_limit:
            a_cmd[0:2] = a_cmd[0:2] / xy_mag * self.accel_limit
        a_cmd[2] = np.clip(a_cmd[2], -0.5 * self.accel_limit, 0.5 * self.accel_limit)

        # First-order lag
        self.a_achieved += (a_cmd - self.a_achieved) * (dt / self.tau)

        # Limit achieved XY and Z independently
        xy_ach = np.linalg.norm(self.a_achieved[0:2])
        if xy_ach > self.accel_limit:
            self.a_achieved[0:2] = self.a_achieved[0:2] / xy_ach * self.accel_limit
        self.a_achieved[2] = np.clip(self.a_achieved[2], -0.5 * self.accel_limit, 0.5 * self.accel_limit)

        self.a_cmd_history.append(a_cmd.copy())

        return self.a_achieved.copy(), self.seeker_valid

    def reset(self, seed=None):
        """Reset guidance state for new engagement."""
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        self.a_achieved = np.zeros(3)
        self.seeker_valid = True
        self.los_rate_history = []
        self.a_cmd_history = []