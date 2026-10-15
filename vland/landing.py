"""Run the precision-landing loop on the vehicle: camera -> detector ->
frames -> PrecisionLander -> offboard velocity, then hand over to PX4's land
mode for the final half metre."""
from __future__ import annotations

import asyncio
import logging
import time
from dataclasses import dataclass

from vland.camera import GzCamera
from vland.detector import MarkerDetector, draw_overlay
from vland.frames import body_to_level, optical_to_body
from vland.lander import LanderConfig, Phase, PrecisionLander, Target
from vland.recorder import Recorder
from vland.vehicle import Vehicle

log = logging.getLogger(__name__)


@dataclass
class Occlusion:
    """Hide the marker from the controller for a while, to exercise the
    lost-marker logic. Times are simulation seconds after landing starts."""
    start: float
    duration: float

    def active(self, t_rel: float) -> bool:
        return self.start <= t_rel < self.start + self.duration


@dataclass
class LandingOutcome:
    phase: Phase                 # FINAL (handed to land mode) or ABORT
    loss_events: int
    t_start: float               # sim time
    t_handover: float
    t_landed: float


async def precision_land(v: Vehicle, cam: GzCamera, det: MarkerDetector, marker_id: int,
                         rec: Recorder | None = None, cfg: LanderConfig | None = None,
                         occlusion: Occlusion | None = None, title: str = "") -> LandingOutcome:
    """Precondition: airborne, in offboard mode, roughly above the pad."""
    lander = PrecisionLander(cfg)
    t0 = None
    last_stamp = None
    last_frame_wall = time.monotonic()

    while lander.phase not in (Phase.FINAL, Phase.ABORT):
        f = cam.latest()
        if f is None or f.stamp == last_stamp:
            # Safety: PX4 drops out of offboard if setpoints stop. If the
            # camera stalls, keep the vehicle still rather than replaying the
            # last command.
            if time.monotonic() - last_frame_wall > 0.5:
                await v.set_velocity_body(0.0, 0.0, 0.0)
                last_frame_wall = time.monotonic()
            await asyncio.sleep(0.005)
            continue
        last_stamp, last_frame_wall = f.stamp, time.monotonic()
        t0 = f.stamp if t0 is None else t0
        t_rel = f.stamp - t0

        q = v.state.q_wxyz           # attitude closest in time to the frame
        dets = [d for d in det.detect(f.image) if d.marker_id == marker_id]
        occluded = occlusion is not None and occlusion.active(t_rel)
        target = None
        if dets and not occluded:
            p = body_to_level(optical_to_body(dets[0].tvec), q)
            target = Target(forward=p[0], right=p[1], height=p[2])

        altitude = -v.state.pos_ned[2]
        cmd = lander.update(f.stamp, target, altitude)
        if cmd.phase not in (Phase.FINAL, Phase.ABORT):
            await v.set_velocity_body(cmd.forward, cmd.right, cmd.down)

        if rec:
            lines = [f"{title}  {cmd.phase.value}  t {t_rel:5.1f} s  alt {altitude:4.1f} m",
                     f"cmd  fwd {cmd.forward:+.2f}  right {cmd.right:+.2f}  down {cmd.down:+.2f} m/s"]
            if target:
                lines.append(f"pad  fwd {target.forward:+.2f}  right {target.right:+.2f}  "
                             f"below {target.height:.2f} m")
            if occluded:
                lines.append("SIMULATED OCCLUSION: marker hidden from controller")
            rec.write(draw_overlay(f.image, [] if occluded else dets, lines=lines))
        await asyncio.sleep(0)

    t_handover = last_stamp
    log.info("%s after %.1f s sim (%d marker losses)", lander.phase.value, t_handover - t0, lander.loss_events)
    await v.set_velocity_body(0.0, 0.0, 0.0)
    # Straight from offboard to LAND, confirmed by telemetry. (Stopping
    # offboard first puts PX4 in HOLD; one test run then sat in HOLD on the
    # pad indefinitely. See DESIGN.md §5.)
    await v.command_land()

    # Keep recording the final descent until touchdown.
    stop = asyncio.Event()

    async def record_final():
        seen = None
        while not stop.is_set():
            f = cam.latest()
            if rec and f is not None and f.stamp != seen:
                seen = f.stamp
                dets = [d for d in det.detect(f.image) if d.marker_id == marker_id]
                rec.write(draw_overlay(f.image, dets, lines=[
                    f"{title}  {lander.phase.value} -> autopilot LAND  t {f.stamp - t0:5.1f} s",
                    f"alt {-v.state.pos_ned[2]:4.2f} m"]))
            await asyncio.sleep(0.01)

    task = asyncio.create_task(record_final())
    try:
        await v.wait_landed()
    finally:
        stop.set()
        await task
    f = cam.latest()
    return LandingOutcome(lander.phase, lander.loss_events, t0, t_handover, f.stamp if f else t_handover)
