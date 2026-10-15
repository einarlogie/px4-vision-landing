"""Generate the landing pad texture: an ArUco marker with a white quiet zone.

The texture maps onto a 1.0 m plane, so with --marker-frac 0.8 the marker's
black square is 0.80 m wide. That physical size is what the pose estimator
needs (vland/detector.py, MARKER_SIZE_M).
"""
import argparse

import cv2
import numpy as np


def make(marker_id: int, px: int, marker_frac: float) -> np.ndarray:
    side = int(round(px * marker_frac))
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    marker = cv2.aruco.generateImageMarker(dictionary, marker_id, side)
    img = np.full((px, px), 255, np.uint8)
    off = (px - side) // 2
    img[off:off + side, off:off + side] = marker
    return img


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--id", type=int, default=0)
    p.add_argument("--px", type=int, default=1000)
    p.add_argument("--marker-frac", type=float, default=0.8)
    p.add_argument("--out", default="models/landing_pad/materials/marker.png")
    a = p.parse_args()
    cv2.imwrite(a.out, make(a.id, a.px, a.marker_frac))
    print("wrote", a.out)
