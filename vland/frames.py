"""Coordinate frames and the conversions between them.

Frames used in this project (all right-handed):

- Camera optical frame (OpenCV convention): x right in the image, y down in
  the image, z out of the lens. solvePnP returns the marker position here.
- Body frame, FRD (PX4 convention): x forward, y right, z down.
- Local NED (PX4 convention): x north, y east, z down, origin at home.
- Gazebo world, ENU: x east, y north, z up. Only used for ground truth.

The camera is mounted looking straight down with the top of the image
towards the nose of the drone (see models/x500_vision_down). So in the body
frame: image right = body right, image down = body backward, lens axis =
body down. That fixed mounting is the matrix R_BODY_FROM_OPTICAL below.
"""
from __future__ import annotations

import math

import numpy as np

# Columns are the optical axes (x_opt, y_opt, z_opt) expressed in body FRD.
#   x_opt (image right) -> +y body (right)
#   y_opt (image down)  -> -x body (backward)
#   z_opt (lens axis)   -> +z body (down)
R_BODY_FROM_OPTICAL = np.array([
    [0.0, -1.0, 0.0],
    [1.0,  0.0, 0.0],
    [0.0,  0.0, 1.0],
])

# Camera position in the body frame (metres, FRD). From the model SDF the
# camera sits 5 cm below the body origin, centred.
CAMERA_OFFSET_BODY = np.array([0.0, 0.0, 0.05])


def optical_to_body(p_optical: np.ndarray) -> np.ndarray:
    """Point seen by the camera -> position relative to the body origin, FRD."""
    return R_BODY_FROM_OPTICAL @ np.asarray(p_optical, float) + CAMERA_OFFSET_BODY


def quat_to_rotmat(w: float, x: float, y: float, z: float) -> np.ndarray:
    """Rotation matrix from a unit quaternion (Hamilton convention).

    PX4/MAVSDK attitude quaternions rotate vectors from the body frame (FRD)
    to the local frame (NED): v_ned = R @ v_body.
    """
    n = math.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z),     2 * (x * z + w * y)],
        [2 * (x * y + w * z),     1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y),     2 * (y * z + w * x),     1 - 2 * (x * x + y * y)],
    ])


def body_to_ned(p_body: np.ndarray, q_wxyz: tuple[float, float, float, float]) -> np.ndarray:
    """Rotate a body-frame vector into NED using the vehicle attitude.

    This matters: when the drone tilts to move, a camera fixed to the body
    tilts with it. At 5 m altitude a 10 deg tilt shifts where the marker
    appears by ~0.9 m. Rotating through the full attitude removes that error.
    """
    return quat_to_rotmat(*q_wxyz) @ np.asarray(p_body, float)


def body_to_level(p_body: np.ndarray, q_wxyz: tuple[float, float, float, float]) -> np.ndarray:
    """Rotate a body-frame vector into the *level* (heading-aligned) frame.

    The level frame shares the drone's heading but is horizontal: x forward
    along the ground, y right, z straight down. Only roll and pitch are
    removed, so the result doesn't depend on the estimated yaw at all.

    The landing controller works in this frame and commands body-frame
    velocities, which PX4 also interprets in this heading-aligned frame.
    A heading-estimate error (measured at ~6 deg in this sim) then can't
    rotate the correction the wrong way.
    """
    yaw = yaw_from_quat(q_wxyz)
    c, s = math.cos(yaw), math.sin(yaw)
    r_level_from_ned = np.array([[c, s, 0.0], [-s, c, 0.0], [0.0, 0.0, 1.0]])
    return r_level_from_ned @ body_to_ned(p_body, q_wxyz)


def yaw_from_quat(q_wxyz: tuple[float, float, float, float]) -> float:
    """Heading in radians (0 = north, +pi/2 = east)."""
    w, x, y, z = q_wxyz
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def enu_to_ned(p_enu: np.ndarray) -> np.ndarray:
    """Gazebo world (ENU) -> NED: swap x/y, negate z."""
    e, n, u = p_enu
    return np.array([n, e, -u])
