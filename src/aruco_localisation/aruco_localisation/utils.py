"""
Utility functions for quaternion / homogeneous-transform conversions.

Kept separate from the node implementations (per the plan's step 20
"practical design guidance") so detector, tf_publisher and
optitrack_transform can all share the same math without duplicating it.
"""

import numpy as np
import cv2


def rvec_to_quaternion(rvec):
    """Convert an OpenCV Rodrigues rotation vector to a ROS-style
    (x, y, z, w) quaternion.

    Args:
        rvec: (3,) or (3,1) rotation vector from cv2.solvePnP / cv2.Rodrigues.

    Returns:
        np.ndarray of shape (4,): [x, y, z, w]
    """
    rot_matrix, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64))
    return rotation_matrix_to_quaternion(rot_matrix)


def rotation_matrix_to_quaternion(r):
    """Convert a 3x3 rotation matrix to a (x, y, z, w) quaternion.

    Uses the standard Shepperd's method for numerical stability.
    """
    m = np.asarray(r, dtype=np.float64)
    trace = m[0, 0] + m[1, 1] + m[2, 2]

    if trace > 0.0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (m[2, 1] - m[1, 2]) * s
        y = (m[0, 2] - m[2, 0]) * s
        z = (m[1, 0] - m[0, 1]) * s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s

    q = np.array([x, y, z, w], dtype=np.float64)
    return q / np.linalg.norm(q)


def quaternion_to_rotation_matrix(q):
    """Convert a (x, y, z, w) quaternion to a 3x3 rotation matrix."""
    x, y, z, w = q
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    wx, wy, wz = s * w * x, s * w * y, s * w * z
    xx, xy, xz = s * x * x, s * x * y, s * x * z
    yy, yz, zz = s * y * y, s * y * z, s * z * z

    return np.array([
        [1.0 - (yy + zz), xy - wz, xz + wy],
        [xy + wz, 1.0 - (xx + zz), yz - wx],
        [xz - wy, yz + wx, 1.0 - (xx + yy)],
    ])


def pose_to_matrix(translation, quaternion_xyzw):
    """Build a 4x4 homogeneous transform from a translation and quaternion.

    Args:
        translation: (3,) iterable [x, y, z]
        quaternion_xyzw: (4,) iterable [x, y, z, w]

    Returns:
        4x4 np.ndarray homogeneous transform.
    """
    t = np.asarray(translation, dtype=np.float64).reshape(3)
    r = quaternion_to_rotation_matrix(quaternion_xyzw)

    m = np.eye(4)
    m[:3, :3] = r
    m[:3, 3] = t
    return m


def matrix_to_pose(matrix):
    """Decompose a 4x4 homogeneous transform into (translation, quaternion_xyzw)."""
    m = np.asarray(matrix, dtype=np.float64)
    translation = m[:3, 3].copy()
    quaternion = rotation_matrix_to_quaternion(m[:3, :3])
    return translation, quaternion


def invert_transform(matrix):
    """Efficiently invert a rigid-body 4x4 homogeneous transform."""
    m = np.asarray(matrix, dtype=np.float64)
    r = m[:3, :3]
    t = m[:3, 3]

    inv = np.eye(4)
    inv[:3, :3] = r.T
    inv[:3, 3] = -r.T @ t
    return inv
