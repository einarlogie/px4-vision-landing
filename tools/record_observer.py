"""Spawn a static 'observer' camera into the running world and record it.

For demo footage only: gives a third-person view of the drone without
screen-capturing the Gazebo GUI. Run it alongside a flight script:

    python tools/record_observer.py --look-at=3,2,1 --from=-4,-6,5 --seconds 120 --out output/observer.mp4

Positions are Gazebo world coordinates (ENU: x east, y north, z up).
Note: rendering a second camera slows the lockstep simulation; don't run it
during accuracy measurements.
"""
import argparse
import math
import time

from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.entity_factory_pb2 import EntityFactory
from gz.transport13 import Node

from vland.camera import GzCamera
from vland.recorder import Recorder


def look_at_rpy(src, dst):
    """Roll/pitch/yaw so a Gazebo camera (looking along +x) at src faces dst."""
    dx, dy, dz = (d - s for s, d in zip(src, dst))
    yaw = math.atan2(dy, dx)
    pitch = -math.atan2(dz, math.hypot(dx, dy))   # positive pitch tilts +x down
    return 0.0, pitch, yaw


def observer_sdf(src, rpy, width, height, hfov, rate):
    x, y, z = src
    r, p, yw = rpy
    return f"""<sdf version="1.9"><model name="observer"><static>true</static>
  <pose>{x} {y} {z} {r} {p} {yw}</pose>
  <link name="link"><sensor name="cam" type="camera">
    <camera><horizontal_fov>{hfov}</horizontal_fov>
      <image><width>{width}</width><height>{height}</height><format>R8G8B8</format></image>
      <clip><near>0.1</near><far>300</far></clip></camera>
    <always_on>1</always_on><update_rate>{rate}</update_rate><visualize>false</visualize>
  </sensor></link></model></sdf>"""


def triple(s):
    return tuple(float(v) for v in s.split(","))


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default="landing_pad")
    ap.add_argument("--from", dest="src", type=triple, default=(-4.0, -6.0, 5.0))
    ap.add_argument("--look-at", type=triple, default=(3.0, 2.0, 1.0))
    ap.add_argument("--hfov", type=float, default=1.0)
    ap.add_argument("--rate", type=float, default=15.0)
    ap.add_argument("--seconds", type=float, default=120.0, help="wall-clock recording time")
    ap.add_argument("--out", default="output/observer.mp4")
    a = ap.parse_args()

    node = Node()
    req = EntityFactory()
    req.sdf = observer_sdf(a.src, look_at_rpy(a.src, a.look_at), 640, 480, a.hfov, a.rate)
    req.allow_renaming = False
    ok, rep = node.request(f"/world/{a.world}/create", req, EntityFactory, Boolean, 5000)
    print("spawned observer:", ok and rep.data)

    topic = f"/world/{a.world}/model/observer/link/link/sensor/cam/image"
    cam = GzCamera(topic)
    cam.wait_first(timeout=30)
    rec = Recorder(a.out, fps=a.rate)
    seen, end = None, time.monotonic() + a.seconds
    try:
        while time.monotonic() < end:
            f = cam.latest()
            if f is not None and f.stamp != seen:
                seen = f.stamp
                rec.write(f.image)
            time.sleep(0.005)
    except KeyboardInterrupt:
        pass                      # Ctrl-C / SIGINT ends the recording early
    finally:
        rec.close()
    print(f"wrote {a.out} ({rec.frames} frames)")
