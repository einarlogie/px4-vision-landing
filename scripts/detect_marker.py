"""Milestone 2: detect the landing pad marker from the air and check the pose.

Takes off beside the pad, hovers, then drifts sideways so the marker is seen
while the drone is tilted. For every frame it estimates where the pad is
relative to the drone (camera -> body -> NED) and compares that with
Gazebo's ground truth. Writes an annotated video and a CSV.

Run in the app container:  python scripts/detect_marker.py --show
"""
import argparse
import asyncio
import csv
import logging
from pathlib import Path

import numpy as np

from vland.camera import GzCamera
from vland.detector import MarkerDetector, draw_overlay
from vland.frames import body_to_ned, enu_to_ned, optical_to_body, yaw_from_quat
from vland.groundtruth import X500_BASE_LINK_OFFSET, GroundTruth
from vland.recorder import Recorder
from vland.vehicle import Vehicle

log = logging.getLogger("detect")
DRONE = "x500_vision_down_0"
PAD = "landing_pad"


async def main(a) -> None:
    cam = GzCamera()
    gt = GroundTruth(a.world, (DRONE, PAD))
    det = MarkerDetector()
    v = Vehicle()
    await v.connect()
    await v.start_telemetry()
    gt.wait_for()
    cam.wait_first()

    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    rec = Recorder(str(out / "detect.mp4"), show=a.show)
    rows = []

    async def observe(seconds: float, phase: str):
        last_stamp = None
        t_end = asyncio.get_running_loop().time() + seconds
        while asyncio.get_running_loop().time() < t_end:
            f = cam.latest()
            if f is None or f.stamp == last_stamp:
                await asyncio.sleep(0.01)
                continue
            last_stamp = f.stamp
            dets = [d for d in det.detect(f.image) if d.marker_id == a.marker_id]
            truth = enu_to_ned(gt.position_enu(PAD) - gt.position_enu(DRONE, X500_BASE_LINK_OFFSET))
            px4_hdg = float(np.degrees(yaw_from_quat(v.state.q_wxyz))) % 360
            true_hdg = gt.heading_deg(DRONE)
            lines = [f"{phase}  t_sim {f.stamp:.1f} s  alt {-v.state.pos_ned[2]:.1f} m",
                     f"truth  N {truth[0]:+.2f}  E {truth[1]:+.2f}  D {truth[2]:+.2f}",
                     f"heading  PX4 {px4_hdg:.1f}  true {true_hdg:.1f} deg"]
            row = {"phase": phase, "t_sim": f.stamp, "truth_n": truth[0], "truth_e": truth[1], "truth_d": truth[2],
                   "px4_heading": px4_hdg, "true_heading": true_hdg}
            if dets:
                d = dets[0]
                est = body_to_ned(optical_to_body(d.tvec), v.state.q_wxyz)
                err = est - truth
                lines.append(f"vision N {est[0]:+.2f}  E {est[1]:+.2f}  D {est[2]:+.2f}   err {np.linalg.norm(err):.2f} m")
                row.update(est_n=est[0], est_e=est[1], est_d=est[2], err_m=float(np.linalg.norm(err)),
                           dist_m=d.distance)
            else:
                lines.append("vision  -- marker not detected --")
            rows.append(row)
            rec.write(draw_overlay(f.image, dets, lines=lines))
            await asyncio.sleep(0)

    try:
        await v.arm_and_takeoff(a.alt)
        await observe(a.hover, "hover")
        # Drift towards the pad: the drone tilts, so this exercises the
        # attitude compensation in body_to_ned().
        await v.start_offboard()
        heading = float(np.degrees(yaw_from_quat(v.state.q_wxyz)))
        await v.set_velocity_ned(0.6, 0.9, 0.0, heading)
        await observe(a.move, "moving")
        await v.set_velocity_ned(0.0, 0.0, 0.0, heading)
        await observe(3.0, "braking")
        await v.land_and_wait()      # straight from offboard to LAND
    finally:
        rec.close()
        await v.close()

    keys = ["phase", "t_sim", "truth_n", "truth_e", "truth_d", "est_n", "est_e", "est_d", "err_m", "dist_m",
            "px4_heading", "true_heading"]
    with open(out / "detect.csv", "w", newline="") as fh:
        w = csv.DictWriter(fh, keys)
        w.writeheader()
        w.writerows(rows)
    for phase in ("hover", "moving", "braking"):
        errs = [r["err_m"] for r in rows if r["phase"] == phase and "err_m" in r]
        n = sum(1 for r in rows if r["phase"] == phase)
        if errs:
            log.info("%-8s detected %d/%d frames, position error mean %.3f m, max %.3f m",
                     phase, len(errs), n, np.mean(errs), np.max(errs))
        else:
            log.info("%-8s detected 0/%d frames", phase, n)
    log.info("wrote %s and %s (%d frames)", out / "detect.mp4", out / "detect.csv", rec.frames)


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    p = argparse.ArgumentParser()
    p.add_argument("--alt", type=float, default=6.0)
    p.add_argument("--hover", type=float, default=8.0, help="wall-clock seconds")
    p.add_argument("--move", type=float, default=4.0, help="wall-clock seconds")
    p.add_argument("--marker-id", type=int, default=0)
    p.add_argument("--world", default="landing_pad")
    p.add_argument("--out", default="output/detect")
    p.add_argument("--show", action="store_true", help="show the annotated camera view")
    asyncio.run(main(p.parse_args()))
