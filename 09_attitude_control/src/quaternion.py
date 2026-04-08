"""
quaternion.py -- Quaternion math utilities
Project 09 -- Attitude Control Simulator

Convention: q = [qw, qx, qy, qz] where qw is scalar part.
Represents rotation from body frame to inertial frame.

All functions operate on numpy arrays.
"""

import numpy as np


# ---------------------------------------------------------------
# Core quaternion operations
# ---------------------------------------------------------------

def quat_mult(p, q):
    """
    Hamilton product of two quaternions p and q.
    p, q: array-like [w, x, y, z]
    Returns: [w, x, y, z]
    """
    pw, px, py, pz = p
    qw, qx, qy, qz = q
    return np.array([
        pw*qw - px*qx - py*qy - pz*qz,
        pw*qx + px*qw + py*qz - pz*qy,
        pw*qy - px*qz + py*qw + pz*qx,
        pw*qz + px*qy - py*qx + pz*qw,
    ])


def quat_conj(q):
    """
    Quaternion conjugate (= inverse for unit quaternion).
    q: [w, x, y, z]
    Returns: [w, -x, -y, -z]
    """
    return np.array([q[0], -q[1], -q[2], -q[3]])


def quat_norm(q):
    """
    Normalize quaternion to unit length.
    """
    n = np.linalg.norm(q)
    if n < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])
    return q / n


def quat_rotate(q, v):
    """
    Rotate vector v (inertial frame) into body frame using quaternion q.
    q: unit quaternion [w, x, y, z] (body-to-inertial)
    v: 3-vector in inertial frame
    Returns: 3-vector in body frame
    """
    # v_body = q* ⊗ [0, v] ⊗ q
    q_conj = quat_conj(q)
    v_quat = np.array([0.0, v[0], v[1], v[2]])
    result = quat_mult(quat_mult(q_conj, v_quat), q)
    return result[1:]


def quat_rotate_inv(q, v):
    """
    Rotate vector v (body frame) into inertial frame using quaternion q.
    q: unit quaternion [w, x, y, z] (body-to-inertial)
    v: 3-vector in body frame
    Returns: 3-vector in inertial frame
    """
    # v_inertial = q ⊗ [0, v] ⊗ q*
    v_quat = np.array([0.0, v[0], v[1], v[2]])
    result = quat_mult(quat_mult(q, v_quat), quat_conj(q))
    return result[1:]


def quat_to_rotmat(q):
    """
    Rotation matrix from body frame to inertial frame.
    q: [w, x, y, z]
    Returns: 3x3 rotation matrix R such that v_inertial = R @ v_body
    """
    qw, qx, qy, qz = q
    R = np.array([
        [1 - 2*(qy**2 + qz**2),  2*(qx*qy - qw*qz),      2*(qx*qz + qw*qy)     ],
        [2*(qx*qy + qw*qz),       1 - 2*(qx**2 + qz**2),  2*(qy*qz - qw*qx)     ],
        [2*(qx*qz - qw*qy),       2*(qy*qz + qw*qx),      1 - 2*(qx**2 + qy**2) ],
    ])
    return R


def quat_kinematics(q, omega):
    """
    Quaternion kinematic equation.
    dq/dt = 0.5 * q ⊗ [0, omega]

    q: unit quaternion [w, x, y, z] (body-to-inertial)
    omega: angular velocity in body frame [p, q_rate, r] (rad/s)
    Returns: dq/dt [w, x, y, z]
    """
    p, q_rate, r = omega
    omega_quat = np.array([0.0, p, q_rate, r])
    dq = 0.5 * quat_mult(q, omega_quat)
    return dq


def quat_to_euler(q):
    """
    Convert quaternion to Euler angles (3-2-1 convention: roll, pitch, yaw).
    q: [w, x, y, z]
    Returns: [phi, theta, psi] in radians
    Note: singularity at theta = ±90°
    """
    qw, qx, qy, qz = q

    # Roll (phi)
    sinr_cosp = 2.0 * (qw*qx + qy*qz)
    cosr_cosp = 1.0 - 2.0 * (qx**2 + qy**2)
    phi = np.arctan2(sinr_cosp, cosr_cosp)

    # Pitch (theta)
    sinp = 2.0 * (qw*qy - qz*qx)
    sinp = np.clip(sinp, -1.0, 1.0)
    theta = -np.arcsin(sinp)

    # Yaw (psi)
    siny_cosp = 2.0 * (qw*qz + qx*qy)
    cosy_cosp = 1.0 - 2.0 * (qy**2 + qz**2)
    psi = np.arctan2(siny_cosp, cosy_cosp)

    return np.array([phi, theta, psi])


def euler_to_quat(phi, theta, psi):
    """
    Convert Euler angles (3-2-1: roll, pitch, yaw) to quaternion.
    Returns: [w, x, y, z]
    """
    cy = np.cos(psi * 0.5)
    sy = np.sin(psi * 0.5)
    cp = np.cos(theta * 0.5)
    sp = np.sin(theta * 0.5)
    cr = np.cos(phi * 0.5)
    sr = np.sin(phi * 0.5)

    qw = cr*cp*cy - sr*sp*sy
    qx = sr*cp*cy + cr*sp*sy
    qy = -cr*sp*cy + sr*cp*sy
    qz = cr*cp*sy + sr*sp*cy

    return quat_norm(np.array([qw, qx, qy, qz]))


def quat_error(q_current, q_desired):
    """
    Compute quaternion error: q_err = q_desired^{-1} ⊗ q_current
    Returns vector part of error quaternion [ex, ey, ez]
    For small errors, this approximates the attitude error in body frame.
    """
    q_err = quat_mult(quat_conj(q_desired), q_current)
    # Ensure shortest path (positive scalar part)
    if q_err[0] < 0:
        q_err = -q_err
    return q_err[1:]  # return vector part only


# ---------------------------------------------------------------
# Validation tests
# ---------------------------------------------------------------

if __name__ == '__main__':
    print("=== Quaternion Validation ===")

    # Test 1: Identity quaternion rotation
    q_identity = np.array([1.0, 0.0, 0.0, 0.0])
    v = np.array([1.0, 2.0, 3.0])
    v_rot = quat_rotate_inv(q_identity, v)
    assert np.allclose(v_rot, v), f"Identity rotation failed: {v_rot}"
    print("TEST 1 PASS: Identity quaternion rotation")

    # Test 2: 90° rotation about Z axis
    q_90z = euler_to_quat(0, 0, np.pi/2)
    v_x = np.array([1.0, 0.0, 0.0])
    v_rot = quat_rotate_inv(q_90z, v_x)
    assert np.allclose(v_rot, [0.0, 1.0, 0.0], atol=1e-10), f"90° Z rotation failed: {v_rot}"
    print("TEST 2 PASS: 90° Z-axis rotation")

    # Test 3: Euler round-trip
    phi, theta, psi = 0.3, 0.2, 1.1
    q = euler_to_quat(phi, theta, psi)
    euler_back = quat_to_euler(q)
    assert np.allclose([phi, theta, psi], euler_back, atol=1e-10), \
        f"Euler round-trip failed: {euler_back}"
    print("TEST 3 PASS: Euler angle round-trip")

    # Test 4: Quaternion normalization preserved through multiplication
    q1 = euler_to_quat(0.1, 0.2, 0.3)
    q2 = euler_to_quat(0.4, 0.5, 0.6)
    q3 = quat_mult(q1, q2)
    assert abs(np.linalg.norm(q3) - 1.0) < 1e-10, f"Norm not preserved: {np.linalg.norm(q3)}"
    print("TEST 4 PASS: Quaternion norm preserved")

    # Test 5: Rotation matrix orthogonality
    q = euler_to_quat(0.3, 0.2, 0.1)
    R = quat_to_rotmat(q)
    assert np.allclose(R @ R.T, np.eye(3), atol=1e-10), "Rotation matrix not orthogonal"
    print("TEST 5 PASS: Rotation matrix orthogonality")

    # Test 6: Kinematic equation produces unit quaternion when integrated
    q0 = euler_to_quat(0, 0, 0)
    omega = np.array([0.1, 0.2, 0.3])
    dt = 0.001
    q = q0.copy()
    for _ in range(1000):
        dq = quat_kinematics(q, omega)
        q = quat_norm(q + dq * dt)
    assert abs(np.linalg.norm(q) - 1.0) < 1e-6, f"Kinematic norm drift: {np.linalg.norm(q)}"
    print("TEST 6 PASS: Kinematic integration norm stability")

    print("\n=== All quaternion tests passed ===")