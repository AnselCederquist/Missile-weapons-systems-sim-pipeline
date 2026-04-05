"""
target.py -- Maneuvering target model for GNC engagement simulation
Project 08 -- GNC/Monte Carlo

Models a maneuvering drone target with:
- Constant velocity baseline
- Bang-bang lateral evasion maneuvers
- Random heading changes
- Configurable speed, altitude, max lateral acceleration
"""

import numpy as np

class ManeuveringTarget:
    """
    6-state point mass target model.
    State: [x, y, z, vx, vy, vz]
    Evasion: bang-bang lateral acceleration with random timing
    """

    def __init__(self,
                 pos0=None,
                 heading_deg=0.0,
                 speed=50.0,
                 altitude=500.0,
                 max_accel=3.0 * 9.81,
                 maneuver_interval_range=(2.0, 4.0),
                 seed=None):
        """
        Parameters
        ----------
        pos0 : array-like [x, y, z], initial position (m)
        heading_deg : initial heading (degrees from +X axis)
        speed : target speed (m/s)
        altitude : target altitude (m) -- held constant
        max_accel : max lateral acceleration (m/s^2), default 3g
        maneuver_interval_range : (min, max) seconds between maneuver changes
        seed : random seed
        """
        self.rng = np.random.default_rng(seed)

        if pos0 is None:
            pos0 = np.array([3000.0, 0.0, altitude])
        else:
            pos0 = np.array(pos0, dtype=float)
            pos0[2] = altitude

        heading = np.radians(heading_deg)
        vel0 = np.array([
            speed * np.cos(heading),
            speed * np.sin(heading),
            0.0
        ])

        self.state = np.concatenate([pos0, vel0])
        self.speed = speed
        self.altitude = altitude
        self.max_accel = max_accel
        self.maneuver_interval_range = maneuver_interval_range

        # Evasion state
        self.accel_cmd = self._new_maneuver()
        self.next_maneuver_t = self.rng.uniform(*maneuver_interval_range)
        self.history = [self.state.copy()]
        self.t = 0.0

    def _new_maneuver(self):
        """Generate new random lateral acceleration command."""
        # Random direction in the horizontal plane perpendicular to velocity
        mag = self.max_accel * self.rng.choice([-1.0, 1.0])
        # Lateral direction = perpendicular to current velocity in XY plane
        vx, vy = self.state[3], self.state[4]
        v_mag = np.sqrt(vx**2 + vy**2) + 1e-6
        # Perpendicular unit vector in XY plane
        perp = np.array([-vy / v_mag, vx / v_mag, 0.0])
        return mag * perp

    def _derivatives(self, state, accel):
        """Point mass EOM with lateral acceleration."""
        vx, vy, vz = state[3], state[4], state[5]
        ax, ay, az = accel
        # Altitude hold -- cancel vertical velocity
        az -= vz * 2.0  # damping to hold altitude
        return np.array([vx, vy, vz, ax, ay, az])

    def step(self, dt):
        """Advance target state by dt seconds using RK4."""
        self.t += dt

        # Check for maneuver change
        if self.t >= self.next_maneuver_t:
            self.accel_cmd = self._new_maneuver()
            self.next_maneuver_t = self.t + self.rng.uniform(*self.maneuver_interval_range)

        # RK4
        s = self.state
        a = self.accel_cmd
        k1 = self._derivatives(s, a)
        k2 = self._derivatives(s + k1*dt/2, a)
        k3 = self._derivatives(s + k2*dt/2, a)
        k4 = self._derivatives(s + k3*dt, a)
        self.state = s + (dt/6) * (k1 + 2*k2 + 2*k3 + k4)

        # Enforce altitude hold
        self.state[2] = self.altitude
        self.state[5] = 0.0

        # Enforce constant speed
        v = self.state[3:6]
        v_mag = np.linalg.norm(v)
        if v_mag > 1e-3:
            self.state[3:6] = v / v_mag * self.speed

        self.history.append(self.state.copy())
        return self.state.copy()

    @property
    def pos(self):
        return self.state[0:3]

    @property
    def vel(self):
        return self.state[3:6]

    def reset(self, pos0=None, heading_deg=None, seed=None):
        """Reset target for a new engagement."""
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        if pos0 is not None:
            self.state[0:3] = pos0
            self.state[2] = self.altitude
        if heading_deg is not None:
            heading = np.radians(heading_deg)
            self.state[3:6] = np.array([
                self.speed * np.cos(heading),
                self.speed * np.sin(heading),
                0.0
            ])
        self.accel_cmd = self._new_maneuver()
        self.next_maneuver_t = self.rng.uniform(*self.maneuver_interval_range)
        self.history = [self.state.copy()]
        self.t = 0.0