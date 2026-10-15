"""Milestone 3: approach the pad with GPS, then land on it using vision.

1. Take off and fly to where the pad *should* be, plus an approach error
   (fixed via --offset or random via --random-offset), the way a real
   approach is limited by GPS and map accuracy.
2. Hand over to the vision controller (vland/landing.py).
3. After touchdown, measure the landing error against Gazebo ground truth
   and append one row to the results CSV.

Run in the app container, on a fresh sim:
    python scripts/precision_land.py --random-offset 2.0 --seed 1
    python scripts/precision_land.py --occlusion 6,3      # hide marker 3 s
"""
import argparse
import asyncio
import csv
import logging
import math
import random
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

from vland.camera import GzCamera
from vland.detector import MarkerDetector
from vland.frames import enu_to_ned
from vland.groundtruth import X500_BASE_LINK_OFFSET, GroundTruth
from vland.landing import Occlusion, precision_land
from vland.recorder import Recorder
from vland.vehicle import Vehicle

log = logging.getLogger("precision_land")
DRONE = "x500_vision_down_0"
PAD = "landing_pad"
RESULT_FIELDS = ["run_id", "utc", "seed", "offset_n", "offset_e", "occlusion", "outcome", "loss_events",
                 "vision_time_s", "error_m", "error_n", "error_e"]


async def main(a) -> None:
    rng = random.Random(a.seed)
    if a.random_offset > 0:
        # Uniform over a disc of radius random_offset.
        r, th = a.random_offset * math.sqrt(rng.random()), rng.uniform(0, 2 * math.pi)
        off_n, off_e = r * math.cos(th), r * math.sin(th)
    else:
        off_n, off_e = a.offset
    occlusion = Occlusion(*a.occlusion) if a.occlusion else None
    run_id = a.run_id or datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S")

    cam = GzCamera()
    gt = GroundTruth(a.world, (DRONE, PAD))
    det = MarkerDetector()
    v = Vehicle()
    rec = Recorder(a.video.format(run=run_id) if a.video else None, show=a.show)
    # Every run gets a row, including failures: a run that errors out is
    # recorded as ERROR rather than silently missing from the statistics.
    row = {"run_id": run_id, "seed": a.seed, "offset_n": round(off_n, 3), "offset_e": round(off_e, 3),
           "occlusion": f"{a.occlusion[0]}+{a.occlusion[1]}" if a.occlusion else "", "outcome": "ERROR"}
    try:
        await v.connect()
        await v.start_telemetry()
        gt.wait_for()
        cam.wait_first()

        await v.arm_and_takeoff(a.alt)
        await v.start_offboard()
        heading = v.heading_deg
        log.info("approach: pad expected at N %.1f E %.1f, flying to offset (%+.2f, %+.2f) m",
                 a.pad[0], a.pad[1], off_n, off_e)
        await v.goto_ned(a.pad[0] + off_n, a.pad[1] + off_e, -a.alt, heading)

        outcome = await precision_land(v, cam, det, a.marker_id, rec=rec, occlusion=occlusion,
                                       title=f"run {run_id}")
        await asyncio.sleep(1.0)   # settle on the pad before measuring
        err = enu_to_ned(gt.position_enu(DRONE, X500_BASE_LINK_OFFSET) - gt.position_enu(PAD))
        err_h = float(math.hypot(err[0], err[1]))
        log.info("RESULT %s: landing error %.3f m (N %+.3f, E %+.3f), losses %d, vision phase %.1f s sim",
                 outcome.phase.value, err_h, err[0], err[1], outcome.loss_events,
                 outcome.t_handover - outcome.t_start)
        row.update({"outcome": outcome.phase.value, "loss_events": outcome.loss_events,
                    "vision_time_s": round(outcome.t_handover - outcome.t_start, 2),
                    "error_m": round(err_h, 4), "error_n": round(float(err[0]), 4),
                    "error_e": round(float(err[1]), 4)})
    except Exception as e:
        log.error("RESULT ERROR: %s: %s", type(e).__name__, e)
        raise
    finally:
        rec.close()
        await v.close()
        if a.results:
            write_result(Path(a.results), row)


def write_result(path: Path, row: dict) -> None:
    row = {"utc": datetime.now(timezone.utc).isoformat(timespec="seconds"), **row}
    path.parent.mkdir(parents=True, exist_ok=True)
    new = not path.exists()
    with open(path, "a", newline="") as fh:
        w = csv.DictWriter(fh, RESULT_FIELDS)
        if new:
            w.writeheader()
        w.writerow(row)
    log.info("appended to %s", path)


def pair(s: str) -> tuple[float, float]:
    x, y = s.split(",")
    return float(x), float(y)


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    p = argparse.ArgumentParser()
    p.add_argument("--alt", type=float, default=6.0, help="approach altitude, m")
    p.add_argument("--pad", type=pair, default=(2.0, 3.0),
                   help="pad position N,E (m from home), as a mission plan would give it")
    p.add_argument("--offset", type=pair, default=(1.5, -1.0), help="approach error N,E (m)")
    p.add_argument("--random-offset", type=float, default=0.0, help="random approach error radius, m")
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--occlusion", type=pair, default=None,
                   help="start,duration (sim s after vision takes over) to hide the marker")
    p.add_argument("--marker-id", type=int, default=0)
    p.add_argument("--world", default="landing_pad")
    p.add_argument("--run-id", default="")
    p.add_argument("--video", default="output/land_{run}.mp4")
    p.add_argument("--results", default="results/landing_runs.csv")
    p.add_argument("--show", action="store_true")
    asyncio.run(main(p.parse_args()))
