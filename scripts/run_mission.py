"""Milestone 4: fly a power-line inspection mission from a YAML plan.

  take off -> for each tower: transit at cruise altitude, descend to the
  inspection altitude above the tower, settle, capture an image -> return to
  the pad's approach point -> vision precision landing -> measure error.

Run in the app container on the powerline world (WORLD=powerline):
    python scripts/run_mission.py missions/powerline.yaml
"""
import argparse
import asyncio
import json
import logging
import math
from datetime import datetime, timezone
from pathlib import Path

import cv2

from vland.camera import GzCamera
from vland.detector import MarkerDetector, draw_overlay
from vland.frames import enu_to_ned
from vland.groundtruth import X500_BASE_LINK_OFFSET, GroundTruth
from vland.landing import precision_land
from vland.mission import load_mission
from vland.recorder import Recorder
from vland.vehicle import Vehicle

log = logging.getLogger("mission")
DRONE = "x500_vision_down_0"
PAD = "landing_pad"


class MissionVideo:
    """Writes every camera frame to the mission video with a status line,
    except while the landing routine is writing its own annotated frames."""

    def __init__(self, cam: GzCamera, rec: Recorder, v: Vehicle):
        self.cam, self.rec, self.v = cam, rec, v
        self.status = "starting"
        self.paused = False
        self._stop = asyncio.Event()
        self._task: asyncio.Task | None = None

    async def _run(self):
        seen = None
        while not self._stop.is_set():
            f = self.cam.latest()
            if not self.paused and f is not None and f.stamp != seen:
                seen = f.stamp
                n, e, d = self.v.state.pos_ned
                self.rec.write(draw_overlay(f.image, [], lines=[
                    f"MISSION  {self.status}",
                    f"t {f.stamp:6.1f} s   N {n:+6.1f}  E {e:+6.1f}  alt {-d:5.1f} m"]))
            await asyncio.sleep(0.01)

    def start(self):
        self._task = asyncio.create_task(self._run())

    async def stop(self):
        self._stop.set()
        if self._task:
            await self._task


async def main(a) -> None:
    m = load_mission(a.mission)
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    cam = GzCamera()
    gt = GroundTruth(a.world, (DRONE, PAD))
    det = MarkerDetector()
    v = Vehicle()
    rec = Recorder(str(out / "mission.mp4"), show=a.show)
    video = MissionVideo(cam, rec, v)
    captures = []
    summary = {"mission": m.name, "utc": datetime.now(timezone.utc).isoformat(timespec="seconds")}
    try:
        await v.connect()
        await v.start_telemetry()
        gt.wait_for()
        cam.wait_first()
        video.start()
        t_start = cam.latest().stamp

        video.status = "takeoff"
        await v.arm_and_takeoff(min(5.0, m.cruise_altitude_m))
        await v.start_offboard()
        yaw = v.heading_deg
        home = v.state.pos_ned.copy()

        video.status = f"climb to cruise {m.cruise_altitude_m:.0f} m"
        await v.fly_to(home[0], home[1], -m.cruise_altitude_m, yaw, speed=2.0)

        for t in m.towers:
            video.status = f"transit to {t.id}"
            log.info("transit to %s (N %.1f, E %.1f)", t.id, t.north_m, t.east_m)
            await v.fly_to(t.north_m, t.east_m, -m.cruise_altitude_m, yaw, m.cruise_speed_m_s)
            alt = m.inspection_altitude(t)
            video.status = f"inspect {t.id}: descend to {alt:.0f} m"
            await v.fly_to(t.north_m, t.east_m, -alt, yaw, speed=1.0)
            video.status = f"inspect {t.id}: settle"
            t0 = cam.latest().stamp
            while cam.latest().stamp - t0 < m.hover_s:      # settle in *sim* time
                await asyncio.sleep(0.05)
            f = cam.latest()
            img_path = out / f"tower_{t.id}.png"
            n, e, d = v.state.pos_ned
            annotated = draw_overlay(f.image, [], lines=[
                f"{m.name}  tower {t.id}",
                f"drone N {n:+.1f} E {e:+.1f} alt {-d:.1f} m   tower top {t.height_m:.0f} m",
                f"sim time {f.stamp:.1f} s"])
            cv2.imwrite(str(img_path), annotated)
            captures.append({"tower": t.id, "image": img_path.name, "t_sim": round(f.stamp, 2),
                             "north_m": round(float(n), 2), "east_m": round(float(e), 2),
                             "altitude_m": round(float(-d), 2)})
            log.info("captured %s at alt %.1f m -> %s", t.id, -d, img_path)
            video.status = f"inspect {t.id}: captured, climb to cruise"
            await v.fly_to(t.north_m, t.east_m, -m.cruise_altitude_m, yaw, speed=1.5)

        video.status = "return to pad"
        log.info("returning to pad approach point")
        await v.fly_to(m.pad.north_m, m.pad.east_m, -m.cruise_altitude_m, yaw, m.cruise_speed_m_s)
        video.status = "descend to approach altitude"
        await v.fly_to(m.pad.north_m, m.pad.east_m, -m.pad.approach_altitude_m, yaw, speed=1.5)

        video.paused = True
        outcome = await precision_land(v, cam, det, m.pad.marker_id, rec=rec, title="MISSION landing")
        video.paused = False
        video.status = "landed"
        await asyncio.sleep(1.0)
        err = enu_to_ned(gt.position_enu(DRONE, X500_BASE_LINK_OFFSET) - gt.position_enu(PAD))
        err_h = math.hypot(err[0], err[1])
        summary.update({
            "outcome": outcome.phase.value,
            "landing_error_m": round(err_h, 4),
            "landing_error_ned_m": [round(float(err[0]), 4), round(float(err[1]), 4)],
            "marker_loss_events": outcome.loss_events,
            "mission_sim_time_s": round(outcome.t_landed - t_start, 1),
            "captures": captures,
        })
        log.info("MISSION DONE: %d towers captured, landing %s, error %.3f m, %.0f s sim",
                 len(captures), outcome.phase.value, err_h, outcome.t_landed - t_start)
    finally:
        await video.stop()
        rec.close()
        await v.close()
        (out / "summary.json").write_text(json.dumps(summary, indent=2))


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    p = argparse.ArgumentParser()
    p.add_argument("mission", nargs="?", default="missions/powerline.yaml")
    p.add_argument("--world", default="powerline")
    p.add_argument("--out", default="output/mission")
    p.add_argument("--show", action="store_true")
    asyncio.run(main(p.parse_args()))
