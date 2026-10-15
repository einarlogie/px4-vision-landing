"""Ground-truth poses from Gazebo, used only to *measure* the system.

The controller never reads these. They're the simulator's exact model
positions, which is what makes the landing-error numbers trustworthy: the
error is measured by something independent of the camera being evaluated.
"""
from __future__ import annotations

import math
import threading
import time

import numpy as np
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node

from vland.frames import quat_to_rotmat

# PX4's body origin is the x500's base_link, which the x500 model places
# 0.24 m above the model origin (x500_base/model.sdf: <pose>0 0 .24 ...).
X500_BASE_LINK_OFFSET = np.array([0.0, 0.0, 0.24])


class GroundTruth:
    def __init__(self, world: str, names: tuple[str, ...]):
        self._node = Node()
        self._names = set(names)
        self._lock = threading.Lock()
        self._pose: dict[str, tuple[np.ndarray, tuple[float, float, float, float]]] = {}
        topic = f"/world/{world}/pose/info"
        if not self._node.subscribe(Pose_V, topic, self._on_pose):
            raise RuntimeError(f"failed to subscribe to {topic}")

    def _on_pose(self, msg: Pose_V) -> None:
        upd = {}
        for p in msg.pose:
            if p.name in self._names:
                o = p.orientation
                upd[p.name] = (np.array([p.position.x, p.position.y, p.position.z]), (o.w, o.x, o.y, o.z))
        if upd:
            with self._lock:
                self._pose.update(upd)

    def position_enu(self, name: str, body_offset: np.ndarray | None = None) -> np.ndarray | None:
        """Model position in the Gazebo world (ENU), optionally shifted by an
        offset given in the model's own frame (e.g. X500_BASE_LINK_OFFSET)."""
        with self._lock:
            pq = self._pose.get(name)
        if pq is None:
            return None
        pos, q = pq
        if body_offset is not None:
            pos = pos + quat_to_rotmat(*q) @ body_offset
        return pos.copy()

    def heading_deg(self, name: str) -> float | None:
        """True heading in the NED convention (0 = north, 90 = east)."""
        with self._lock:
            pq = self._pose.get(name)
        if pq is None:
            return None
        w, x, y, z = pq[1]
        yaw_enu = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))   # from east, CCW
        return (90.0 - math.degrees(yaw_enu)) % 360.0

    def wait_for(self, timeout: float = 10.0) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._lock:
                if self._names <= self._pose.keys():
                    return
            time.sleep(0.1)
        raise RuntimeError(f"ground truth for {self._names - self._pose.keys()} not received")
