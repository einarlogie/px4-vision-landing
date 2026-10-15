"""Mission plan: parse and validate the YAML file into plain dataclasses.

Keeping the plan as data (not code) means a new line, a different tower
order or a new pad position is a config change. Validation happens once, up
front, so a bad plan fails on the ground with a clear message rather than
mid-flight.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import yaml


class MissionError(ValueError):
    pass


@dataclass(frozen=True)
class Tower:
    id: str
    north_m: float
    east_m: float
    height_m: float


@dataclass(frozen=True)
class LandingPad:
    north_m: float
    east_m: float
    marker_id: int
    approach_altitude_m: float


@dataclass(frozen=True)
class Mission:
    name: str
    cruise_altitude_m: float
    cruise_speed_m_s: float
    clearance_m: float
    hover_s: float
    pad: LandingPad
    towers: tuple[Tower, ...]

    def inspection_altitude(self, tower: Tower) -> float:
        return tower.height_m + self.clearance_m


def _num(d: dict, key: str, where: str, positive: bool = True) -> float:
    if key not in d:
        raise MissionError(f"{where}: missing '{key}'")
    v = d[key]
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        raise MissionError(f"{where}.{key}: expected a number, got {v!r}")
    if positive and v <= 0:
        raise MissionError(f"{where}.{key}: must be > 0, got {v}")
    return float(v)


def _section(raw: dict, key: str) -> dict:
    s = raw.get(key)
    if not isinstance(s, dict):
        raise MissionError(f"missing section '{key}'")
    return s


def parse_mission(raw: dict) -> Mission:
    if not isinstance(raw, dict):
        raise MissionError("mission file must be a mapping")
    cruise = _section(raw, "cruise")
    insp = _section(raw, "inspection")
    pad_raw = _section(raw, "landing_pad")

    pad = LandingPad(
        north_m=_num(pad_raw, "north_m", "landing_pad", positive=False),
        east_m=_num(pad_raw, "east_m", "landing_pad", positive=False),
        marker_id=int(_num(pad_raw, "marker_id", "landing_pad", positive=False)),
        approach_altitude_m=_num(pad_raw, "approach_altitude_m", "landing_pad"),
    )

    towers_raw = raw.get("towers")
    if not isinstance(towers_raw, list) or not towers_raw:
        raise MissionError("'towers' must be a non-empty list")
    towers = []
    for i, t in enumerate(towers_raw):
        where = f"towers[{i}]"
        if not isinstance(t, dict) or "id" not in t:
            raise MissionError(f"{where}: each tower needs an 'id'")
        towers.append(Tower(id=str(t["id"]),
                            north_m=_num(t, "north_m", where, positive=False),
                            east_m=_num(t, "east_m", where, positive=False),
                            height_m=_num(t, "height_m", where)))
    ids = [t.id for t in towers]
    if len(set(ids)) != len(ids):
        raise MissionError(f"duplicate tower ids: {ids}")

    m = Mission(
        name=str(raw.get("name", "unnamed")),
        cruise_altitude_m=_num(cruise, "altitude_m", "cruise"),
        cruise_speed_m_s=_num(cruise, "speed_m_s", "cruise"),
        clearance_m=_num(insp, "clearance_m", "inspection"),
        hover_s=_num(insp, "hover_s", "inspection", positive=False),
        pad=pad,
        towers=tuple(towers),
    )
    # Safety: transit must never be lower than any inspection altitude, so
    # moving between towers can't clip a tower top or the conductors.
    highest = max(m.inspection_altitude(t) for t in m.towers)
    if m.cruise_altitude_m < highest:
        raise MissionError(f"cruise altitude {m.cruise_altitude_m} m is below the highest "
                           f"inspection altitude {highest} m (tower + clearance)")
    if m.cruise_speed_m_s > 10:
        raise MissionError("cruise speed above 10 m/s is not allowed in this demo")
    return m


def load_mission(path: str | Path) -> Mission:
    with open(path) as fh:
        return parse_mission(yaml.safe_load(fh))
