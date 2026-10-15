"""Precision landing controller: a small state machine over vision measurements.

Input, once per camera frame:
  - t: the frame's simulation time (s). Used for all timing, so behaviour
    doesn't depend on how fast the host runs the simulation.
  - target: where the pad is in the *level* frame (forward, right, height
    below the drone), or None if the marker wasn't seen in this frame.
  - altitude: height above home from the autopilot, for the climb ceiling.

Output: a velocity command in the same level frame (forward, right, down),
which PX4 accepts as a body-frame offboard velocity setpoint.

Phases:
  SEARCH  marker not seen yet: hold position (the mission brought us close).
  TRACK   marker seen: move towards it (P-control, speed-limited) and descend
          only while inside a cone around the pad axis, so the drone never
          descends while badly off-centre.
  HOLD    marker just lost: stop and wait a moment; most losses are a frame
          or two (motion blur, glare, partial occlusion).
  CLIMB   still lost: climb slowly. A higher camera sees more ground, so the
          pad is likely to come back into view.
  FINAL   low and centred: hand over to the autopilot's land mode for the
          last ~0.5 m, where the marker is too close to the lens to track.
  ABORT   couldn't find or re-acquire the pad in time: the caller decides
          (here: a normal landing, flagged in the log).
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum


class Phase(str, Enum):
    SEARCH = "SEARCH"
    TRACK = "TRACK"
    HOLD = "HOLD"
    CLIMB = "CLIMB"
    FINAL = "FINAL"
    ABORT = "ABORT"


@dataclass
class LanderConfig:
    kp: float = 0.8                 # 1/s: horizontal speed per metre of error
    v_max: float = 1.0              # m/s horizontal speed limit
    v_max_low: float = 0.4          # m/s horizontal limit below slow_below
    descend_fast: float = 0.7       # m/s
    descend_slow: float = 0.3       # m/s, below slow_below
    slow_below: float = 2.0         # m height above pad
    cone_base: float = 0.15         # m allowed off-centre error at the pad...
    cone_slope: float = 0.12        # ...growing by this many m per m of height
    final_height: float = 0.8       # m: commit to landing below this height...
    final_tol: float = 0.10         # m: ...if within this horizontal error
    hold_time: float = 1.0          # s (sim): how long to hold after a loss
    climb_rate: float = 0.5         # m/s while re-acquiring
    max_altitude: float = 10.0      # m above home: climb ceiling
    lost_timeout: float = 15.0      # s (sim) lost before ABORT
    search_timeout: float = 20.0    # s (sim) in SEARCH before ABORT


@dataclass(frozen=True)
class Target:
    forward: float     # m, level frame
    right: float       # m
    height: float      # m, pad below the drone (level-frame down)

    @property
    def horizontal_error(self) -> float:
        return math.hypot(self.forward, self.right)


@dataclass(frozen=True)
class Command:
    forward: float
    right: float
    down: float
    phase: Phase


def _clamp_norm(x: float, y: float, limit: float) -> tuple[float, float]:
    n = math.hypot(x, y)
    if n <= limit or n == 0.0:
        return x, y
    return x * limit / n, y * limit / n


class PrecisionLander:
    def __init__(self, cfg: LanderConfig | None = None):
        self.cfg = cfg or LanderConfig()
        self.phase = Phase.SEARCH
        self._t_start: float | None = None
        self._lost_since: float | None = None
        self.last_target: Target | None = None
        self.loss_events = 0

    def update(self, t: float, target: Target | None, altitude: float) -> Command:
        c = self.cfg
        if self._t_start is None:
            self._t_start = t

        if self.phase in (Phase.FINAL, Phase.ABORT):
            return Command(0.0, 0.0, 0.0, self.phase)

        if target is not None:
            self.last_target = target
            self._lost_since = None
            self.phase = Phase.TRACK
            return self._track(target)

        # --- no marker in this frame -------------------------------------
        if self.phase == Phase.SEARCH:
            if t - self._t_start > c.search_timeout:
                self.phase = Phase.ABORT
            return Command(0.0, 0.0, 0.0, self.phase)

        if self.phase == Phase.TRACK:
            self.phase = Phase.HOLD
            self._lost_since = t
            self.loss_events += 1

        lost_for = t - self._lost_since
        if lost_for > c.lost_timeout:
            self.phase = Phase.ABORT
            return Command(0.0, 0.0, 0.0, self.phase)
        if self.phase == Phase.HOLD and lost_for > c.hold_time:
            self.phase = Phase.CLIMB
        if self.phase == Phase.CLIMB:
            climb = c.climb_rate if altitude < c.max_altitude else 0.0
            return Command(0.0, 0.0, -climb, self.phase)
        return Command(0.0, 0.0, 0.0, self.phase)

    def _track(self, tg: Target) -> Command:
        c = self.cfg
        h, err = tg.height, tg.horizontal_error
        if h <= c.final_height and err <= c.final_tol:
            self.phase = Phase.FINAL
            return Command(0.0, 0.0, 0.0, self.phase)

        low = h <= c.slow_below
        vf, vr = _clamp_norm(c.kp * tg.forward, c.kp * tg.right, c.v_max_low if low else c.v_max)

        inside_cone = err <= c.cone_base + c.cone_slope * h
        if inside_cone and h > c.final_height:
            vd = c.descend_slow if low else c.descend_fast
        else:
            vd = 0.0          # centre first, then descend
        return Command(vf, vr, vd, self.phase)
