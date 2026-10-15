"""Save annotated frames to video, and optionally show them live."""
from __future__ import annotations

import os
from pathlib import Path

import cv2
import numpy as np


class Recorder:
    def __init__(self, path: str | None, fps: float = 15.0, show: bool = False, window: str = "vland camera"):
        self.path = path
        self.fps = fps
        self.show = show and bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
        self.window = window
        self._writer: cv2.VideoWriter | None = None
        self.frames = 0

    def write(self, img: np.ndarray) -> None:
        if self.path:
            if self._writer is None:
                Path(self.path).parent.mkdir(parents=True, exist_ok=True)
                h, w = img.shape[:2]
                self._writer = cv2.VideoWriter(self.path, cv2.VideoWriter_fourcc(*"mp4v"), self.fps, (w, h))
            self._writer.write(img)
            self.frames += 1
        if self.show:
            cv2.imshow(self.window, img)
            cv2.waitKey(1)

    def close(self) -> None:
        if self._writer is not None:
            self._writer.release()
        if self.show:
            cv2.destroyAllWindows()
