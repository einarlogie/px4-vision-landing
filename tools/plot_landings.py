"""Plot where each run touched down relative to the pad centre.

Drawn with OpenCV (already in the image) rather than adding matplotlib.
Rings mark 5/10/15 cm. Two groups, told apart by colour *and* marker shape
(circle = nominal, triangle = with simulated occlusion) so the plot still
reads in greyscale or for colour-blind readers.

usage: python tools/plot_landings.py results/landing_runs.csv results/landing_scatter.png
"""
import csv
import sys

import cv2
import numpy as np

W = H = 640
CX, CY = 320, 340
SCALE = 17.0                     # px per cm -> +-15 cm fits
SURFACE = (251, 252, 252)        # #fcfcfb (BGR)
TEXT = (11, 11, 11)              # #0b0b0b
TEXT_2 = (78, 81, 82)            # #52514e
GRID = (212, 216, 217)
BLUE = (214, 120, 42)            # #2a78d6  categorical slot 1
ORANGE = (52, 104, 235)          # #eb6834  categorical slot 2
FONT = cv2.FONT_HERSHEY_SIMPLEX


def to_px(east_cm, north_cm):
    return int(round(CX + east_cm * SCALE)), int(round(CY - north_cm * SCALE))


def text(img, s, org, scale=0.45, color=TEXT, thick=1):
    cv2.putText(img, s, org, FONT, scale, color, thick, cv2.LINE_AA)


def circle_marker(img, p, color):
    cv2.circle(img, p, 7, SURFACE, -1, cv2.LINE_AA)          # 2 px surface ring
    cv2.circle(img, p, 5, color, -1, cv2.LINE_AA)


def triangle_marker(img, p, color):
    x, y = p
    outer = np.array([[x, y - 9], [x - 8, y + 6], [x + 8, y + 6]], np.int32)
    inner = np.array([[x, y - 6], [x - 5, y + 4], [x + 5, y + 4]], np.int32)
    cv2.fillPoly(img, [outer], SURFACE, cv2.LINE_AA)
    cv2.fillPoly(img, [inner], color, cv2.LINE_AA)


def plot(rows, out_path):
    img = np.full((H, W, 3), SURFACE, np.uint8)
    for r_cm in (5, 10, 15):
        cv2.circle(img, (CX, CY), int(r_cm * SCALE), GRID, 1, cv2.LINE_AA)
        text(img, f"{r_cm} cm", (CX + int(r_cm * SCALE * 0.71) + 4, CY - int(r_cm * SCALE * 0.71) - 4),
             0.4, TEXT_2)
    cv2.line(img, (CX - 270, CY), (CX + 270, CY), GRID, 1)
    cv2.line(img, (CX, CY - 270), (CX, CY + 270), GRID, 1)
    text(img, "E", (CX + 276, CY + 5), 0.45, TEXT_2)
    text(img, "N", (CX - 5, CY - 276), 0.45, TEXT_2)
    text(img, "Touchdown position relative to pad centre", (16, 28), 0.6, TEXT, 1)
    ok = [r for r in rows if r["outcome"] == "FINAL"]
    text(img, f"{len(ok)} of {len(rows)} runs landed by vision; ground truth from Gazebo.", (16, 50), 0.45, TEXT_2)

    for r in ok:
        p = to_px(float(r["error_e"]) * 100, float(r["error_n"]) * 100)
        (triangle_marker if r["occlusion"] else circle_marker)(img, p, ORANGE if r["occlusion"] else BLUE)

    # Legend, top right.
    lx, ly = W - 230, 90
    circle_marker(img, (lx, ly), BLUE)
    text(img, "nominal approach", (lx + 14, ly + 5), 0.45)
    triangle_marker(img, (lx, ly + 24), ORANGE)
    text(img, "3 s simulated occlusion", (lx + 14, ly + 29), 0.45)
    cv2.imwrite(out_path, img)


if __name__ == "__main__":
    src = sys.argv[1] if len(sys.argv) > 1 else "results/landing_runs.csv"
    dst = sys.argv[2] if len(sys.argv) > 2 else "results/landing_scatter.png"
    with open(src) as fh:
        plot(list(csv.DictReader(fh)), dst)
    print("wrote", dst)
