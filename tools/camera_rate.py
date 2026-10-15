"""Measure how fast camera frames arrive, in wall-clock and simulation time.

Usage (app container): python tools/camera_rate.py --seconds 10 --save first.png
"""
import argparse
import time

import cv2

from vland.camera import GzCamera

if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--seconds", type=float, default=10.0)
    p.add_argument("--save", default="")
    a = p.parse_args()
    cam = GzCamera()
    print("topic:", cam.topic)
    first = cam.wait_first()
    n0, t0, s0 = cam.frame_count, time.monotonic(), first.stamp
    time.sleep(a.seconds)
    last = cam.latest()
    n = cam.frame_count - n0
    wall = time.monotonic() - t0
    sim = last.stamp - s0
    print(f"{n} frames in {wall:.1f} s wall ({n / wall:.1f} Hz), {sim:.1f} s sim "
          f"({n / sim if sim else 0:.1f} Hz sim), image {last.image.shape}")
    if a.save:
        cv2.imwrite(a.save, last.image)
        print("saved", a.save)
