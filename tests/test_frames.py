import math

import numpy as np
import pytest

from vland.frames import (CAMERA_OFFSET_BODY, body_to_level, body_to_ned, enu_to_ned, optical_to_body,
                          quat_to_rotmat, yaw_from_quat)


def quat_from_euler(roll, pitch, yaw):
    """ZYX (yaw-pitch-roll) Euler angles -> quaternion (w, x, y, z)."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy)


def test_optical_axes_map_to_body():
    off = CAMERA_OFFSET_BODY
    # A point straight along the lens axis is straight below the drone.
    assert np.allclose(optical_to_body([0, 0, 5]) - off, [0, 0, 5])
    # Image right is body right; image down is towards the tail.
    assert np.allclose(optical_to_body([1, 0, 0]) - off, [0, 1, 0])
    assert np.allclose(optical_to_body([0, 1, 0]) - off, [-1, 0, 0])


def test_rotation_matrix_is_orthonormal():
    R = quat_to_rotmat(*quat_from_euler(0.1, -0.2, 2.0))
    assert np.allclose(R @ R.T, np.eye(3))
    assert math.isclose(np.linalg.det(R), 1.0)


def test_body_to_ned_yaw_only():
    # Facing east (yaw 90 deg): body forward points east.
    q = quat_from_euler(0, 0, math.pi / 2)
    assert np.allclose(body_to_ned([1, 0, 0], q), [0, 1, 0], atol=1e-9)
    assert np.allclose(body_to_ned([0, 1, 0], q), [-1, 0, 0], atol=1e-9)


def test_tilt_compensation():
    # Nose down 10 deg (pitch -10 deg) at 5 m: a marker straight below the
    # camera is seen 'ahead' in the body frame, but is actually behind.
    q = quat_from_euler(0, math.radians(-10), 0)
    p_body = np.array([0.0, 0.0, 5.0])        # on the lens axis
    p_ned = body_to_ned(p_body, q)
    assert p_ned[0] == pytest.approx(-5 * math.sin(math.radians(10)), abs=1e-9)
    assert p_ned[2] == pytest.approx(5 * math.cos(math.radians(10)), abs=1e-9)


def test_level_frame_ignores_yaw():
    p_body = np.array([1.0, 2.0, 5.0])
    a = body_to_level(p_body, quat_from_euler(0.05, -0.08, 0.3))
    b = body_to_level(p_body, quat_from_euler(0.05, -0.08, 2.5))
    assert np.allclose(a, b, atol=1e-9)
    # With no tilt, the level frame is the body frame.
    assert np.allclose(body_to_level(p_body, quat_from_euler(0, 0, 1.2)), p_body, atol=1e-9)


def test_yaw_from_quat():
    assert yaw_from_quat(quat_from_euler(0.1, 0.1, 1.0)) == pytest.approx(1.0)


def test_enu_to_ned():
    assert np.allclose(enu_to_ned(np.array([3.0, 2.0, 1.0])), [2, 3, -1])
