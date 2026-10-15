"""Camera pipeline: Gazebo camera sensor -> numpy BGR image.

Gazebo publishes each rendered frame as a gz.msgs.Image on a gz-transport
topic. We subscribe with the Python bindings and keep only the latest frame:
the controller always wants the freshest measurement, and queueing stale
frames would add latency.
"""
from __future__ import annotations

import threading
import time
from dataclasses import dataclass

import numpy as np
from gz.msgs10.image_pb2 import Image
from gz.transport13 import Node

# gz.msgs.PixelFormatType values we handle (see gz-msgs image.proto)
_RGB_INT8 = 3
_BGR_INT8 = 8


@dataclass
class Frame:
    image: np.ndarray       # HxWx3 BGR, uint8 (OpenCV convention)
    stamp: float            # simulation time of the frame, seconds
    received: float         # wall-clock time we received it (time.monotonic)


def find_image_topic(node: Node, timeout: float = 10.0) -> str:
    """Return the first camera image topic Gazebo advertises."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        for t in node.topic_list():
            if t.endswith("/camera/image"):
                return t
        time.sleep(0.5)
    raise RuntimeError("no camera image topic found; is the simulator running?")


class GzCamera:
    def __init__(self, topic: str | None = None):
        self._node = Node()
        self.topic = topic or find_image_topic(self._node)
        self._lock = threading.Lock()
        self._frame: Frame | None = None
        self._count = 0
        if not self._node.subscribe(Image, self.topic, self._on_image):
            raise RuntimeError(f"failed to subscribe to {self.topic}")

    def _on_image(self, msg: Image) -> None:
        # Runs on a gz-transport thread.
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        img = buf.reshape(msg.height, msg.width, 3)
        if msg.pixel_format_type == _RGB_INT8:
            img = img[:, :, ::-1]          # RGB -> BGR
        elif msg.pixel_format_type != _BGR_INT8:
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
        frame = Frame(np.ascontiguousarray(img), stamp, time.monotonic())
        with self._lock:
            self._frame = frame
            self._count += 1

    def latest(self) -> Frame | None:
        with self._lock:
            return self._frame

    @property
    def frame_count(self) -> int:
        return self._count

    def wait_first(self, timeout: float = 15.0) -> Frame:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            f = self.latest()
            if f is not None:
                return f
            time.sleep(0.05)
        raise RuntimeError(f"no frames on {self.topic} after {timeout}s")
