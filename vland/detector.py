"""Marker detection and pose estimation.

Pipeline per frame:
1. ArUco detection finds the marker's four corners in pixels and decodes its
   ID from the black/white bit pattern.
2. solvePnP finds the marker pose that best explains those four corners,
   given the camera intrinsics (focal length, principal point) and the
   marker's real size. Its translation is the marker centre in the camera
   optical frame, in metres.

The camera intrinsics come from the simulated camera's field of view: a
pinhole camera with horizontal FOV h and width W pixels has focal length
fx = (W / 2) / tan(h / 2). Gazebo's camera has square pixels, no distortion
and the principal point at the image centre.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import cv2
import numpy as np

MARKER_SIZE_M = 0.80          # black square side, see models/landing_pad
MARKER_DICT = cv2.aruco.DICT_4X4_50


@dataclass(frozen=True)
class CameraModel:
    width: int
    height: int
    hfov_rad: float

    @property
    def fx(self) -> float:
        return (self.width / 2.0) / math.tan(self.hfov_rad / 2.0)

    @property
    def K(self) -> np.ndarray:
        f = self.fx
        return np.array([[f, 0, self.width / 2.0],
                         [0, f, self.height / 2.0],
                         [0, 0, 1.0]])

    @property
    def dist(self) -> np.ndarray:
        return np.zeros(5)


# Must match models/x500_vision_down/model.sdf
SIM_CAMERA = CameraModel(width=640, height=480, hfov_rad=1.5707963)


@dataclass
class MarkerDetection:
    marker_id: int
    corners: np.ndarray        # (4, 2) pixels: TL, TR, BR, BL of the marker
    rvec: np.ndarray           # (3,) rotation, Rodrigues vector, optical frame
    tvec: np.ndarray           # (3,) marker centre in the optical frame, metres

    @property
    def distance(self) -> float:
        return float(np.linalg.norm(self.tvec))

    @property
    def center_px(self) -> tuple[float, float]:
        c = self.corners.mean(axis=0)
        return float(c[0]), float(c[1])


def marker_object_points(size: float) -> np.ndarray:
    """Marker corners in the marker's own frame, in ArUco's corner order."""
    h = size / 2.0
    return np.array([[-h,  h, 0], [h,  h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float64)


class MarkerDetector:
    def __init__(self, camera: CameraModel = SIM_CAMERA, marker_size: float = MARKER_SIZE_M):
        self.camera = camera
        self.marker_size = marker_size
        params = cv2.aruco.DetectorParameters()
        # Sub-pixel corner refinement gives noticeably steadier poses.
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self._detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(MARKER_DICT), params)
        self._obj = marker_object_points(marker_size)

    def detect(self, bgr: np.ndarray) -> list[MarkerDetection]:
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self._detector.detectMarkers(gray)
        out: list[MarkerDetection] = []
        if ids is None:
            return out
        for c, i in zip(corners, ids.flatten()):
            img_pts = c.reshape(4, 2).astype(np.float64)
            # IPPE_SQUARE is the solver designed for exactly this case: four
            # coplanar points forming a square of known size.
            ok, rvec, tvec = cv2.solvePnP(self._obj, img_pts, self.camera.K, self.camera.dist,
                                          flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if ok:
                out.append(MarkerDetection(int(i), img_pts, rvec.reshape(3), tvec.reshape(3)))
        return out


def draw_overlay(img: np.ndarray, detections: list[MarkerDetection], camera: CameraModel = SIM_CAMERA,
                 lines: list[str] | None = None) -> np.ndarray:
    """Annotate a frame: marker outline and axes, ID, position, distance."""
    out = img.copy()
    h, w = out.shape[:2]
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.drawMarker(out, (w // 2, h // 2), (255, 255, 255), cv2.MARKER_CROSS, 20, 1)
    for d in detections:
        cv2.polylines(out, [d.corners.astype(np.int32)], True, (0, 255, 0), 2)
        cv2.drawFrameAxes(out, camera.K, camera.dist, d.rvec, d.tvec, 0.3, 2)
        x, y, z = d.tvec
        label = f"id {d.marker_id}  x {x:+.2f}  y {y:+.2f}  z {z:.2f}  dist {d.distance:.2f} m"
        (tw, th), _ = cv2.getTextSize(label, font, 0.45, 1)
        bottom = int(d.corners[:, 1].max()) + 8
        lx = int(min(max(d.center_px[0] - tw / 2, 4), w - tw - 4))
        ly = int(min(bottom + th + 4, h - 6))
        _panel(out, lx - 4, ly - th - 5, lx + tw + 4, ly + 5)
        cv2.putText(out, label, (lx, ly), font, 0.45, (0, 255, 255), 1, cv2.LINE_AA)
    if lines:
        widths = [cv2.getTextSize(t, font, 0.45, 1)[0][0] for t in lines]
        _panel(out, 0, 0, max(widths) + 16, 10 + 18 * len(lines))
        for k, text in enumerate(lines):
            cv2.putText(out, text, (8, 20 + 18 * k), font, 0.45, (255, 255, 255), 1, cv2.LINE_AA)
    return out


def _panel(img: np.ndarray, x0: int, y0: int, x1: int, y1: int, alpha: float = 0.55) -> None:
    """Darken a rectangle in place so text drawn on it stays readable."""
    x0, y0 = max(x0, 0), max(y0, 0)
    roi = img[y0:y1, x0:x1]
    roi[:] = (roi * (1 - alpha)).astype(img.dtype)
