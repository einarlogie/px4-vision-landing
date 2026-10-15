"""Detector tests on synthetic images: draw a marker at a known pose with the
same pinhole model, then check detection + solvePnP recover that pose."""
import math

import cv2
import numpy as np
import pytest

from tools.make_marker import make
from vland.detector import MARKER_SIZE_M, SIM_CAMERA, MarkerDetector, draw_overlay


def render(tvec, yaw=0.0, cam=SIM_CAMERA):
    """Project the pad texture onto a grey image as seen from the camera."""
    tex = make(0, 400, 0.8)                     # 1.0 m texture, 0.8 m marker
    tex = cv2.cvtColor(tex, cv2.COLOR_GRAY2BGR)
    half = 0.5                                   # texture is 1.0 m wide
    obj = np.array([[-half, half, 0], [half, half, 0], [half, -half, 0], [-half, -half, 0]])
    # Marker frame: x right, y up, z out of the marker. For it to face the
    # camera (whose z looks *into* the scene) it is flipped 180 deg about x,
    # then rotated by `yaw` about the camera axis.
    c, s = math.cos(yaw), math.sin(yaw)
    Rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])
    Rx = np.diag([1.0, -1.0, -1.0])
    rvec, _ = cv2.Rodrigues(Rz @ Rx)
    img_pts, _ = cv2.projectPoints(obj, rvec, np.asarray(tvec, float), cam.K, cam.dist)
    src = np.array([[0, 0], [400, 0], [400, 400], [0, 400]], np.float32)
    H = cv2.getPerspectiveTransform(src, img_pts.reshape(4, 2).astype(np.float32))
    out = np.full((cam.height, cam.width, 3), 120, np.uint8)
    warped = cv2.warpPerspective(tex, H, (cam.width, cam.height), flags=cv2.INTER_LINEAR)
    mask = cv2.warpPerspective(np.full((400, 400), 255, np.uint8), H, (cam.width, cam.height))
    out[mask > 0] = warped[mask > 0]
    return out


def test_focal_length_from_fov():
    # 90 deg HFOV, 640 px wide -> f = 320 px.
    assert SIM_CAMERA.fx == pytest.approx(320.0, rel=1e-6)


@pytest.mark.parametrize("tvec", [(0.0, 0.0, 3.0), (0.6, -0.4, 5.0), (-1.5, 1.0, 8.0)])
def test_recovers_position(tvec):
    dets = MarkerDetector().detect(render(tvec))
    assert len(dets) == 1 and dets[0].marker_id == 0
    t = dets[0].tvec
    # Within 2% of range: corner localisation limits accuracy at distance.
    assert np.linalg.norm(t - tvec) < 0.02 * np.linalg.norm(tvec)


def test_recovers_rotation():
    d = MarkerDetector().detect(render((0.2, 0.1, 4.0), yaw=math.radians(30)))[0]
    R, _ = cv2.Rodrigues(d.rvec)
    yaw = math.atan2(R[1, 0], R[0, 0])
    assert math.degrees(yaw) == pytest.approx(30, abs=2)


def test_no_marker():
    assert MarkerDetector().detect(np.full((480, 640, 3), 120, np.uint8)) == []


def test_overlay_does_not_change_input():
    img = render((0.0, 0.0, 3.0))
    before = img.copy()
    dets = MarkerDetector().detect(img)
    out = draw_overlay(img, dets, lines=["hello"])
    assert np.array_equal(img, before) and out.shape == img.shape


def test_marker_size_matches_texture():
    # The pad texture puts an 0.8 m marker on a 1.0 m plane.
    assert MARKER_SIZE_M == pytest.approx(0.8)
