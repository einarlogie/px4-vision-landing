import copy
from pathlib import Path

import pytest

from vland.mission import MissionError, load_mission, parse_mission

REPO = Path(__file__).resolve().parents[1]

VALID = {
    "name": "t",
    "cruise": {"altitude_m": 20, "speed_m_s": 3},
    "inspection": {"clearance_m": 6, "hover_s": 2},
    "landing_pad": {"north_m": 2, "east_m": 3, "marker_id": 0, "approach_altitude_m": 6},
    "towers": [{"id": "A", "north_m": 10, "east_m": 0, "height_m": 12},
               {"id": "B", "north_m": 30, "east_m": 0, "height_m": 14}],
}


def variant(**changes):
    raw = copy.deepcopy(VALID)
    for path, value in changes.items():
        node = raw
        keys = path.split("__")
        for k in keys[:-1]:
            node = node[int(k)] if isinstance(node, list) else node[k]
        node[keys[-1]] = value
    return raw


def test_repo_mission_file_parses():
    m = load_mission(REPO / "missions" / "powerline.yaml")
    assert [t.id for t in m.towers] == ["T1", "T2", "T3"]
    assert m.pad.marker_id == 0


def test_valid_parses():
    m = parse_mission(VALID)
    assert m.cruise_altitude_m == 20.0
    assert m.inspection_altitude(m.towers[1]) == 20.0
    assert m.towers[0].north_m == 10.0


def test_cruise_must_clear_towers():
    with pytest.raises(MissionError, match="below the highest"):
        parse_mission(variant(cruise__altitude_m=15))


def test_duplicate_tower_ids():
    with pytest.raises(MissionError, match="duplicate"):
        parse_mission(variant(towers__1__id="A"))


def test_missing_section():
    raw = copy.deepcopy(VALID)
    del raw["landing_pad"]
    with pytest.raises(MissionError, match="landing_pad"):
        parse_mission(raw)


@pytest.mark.parametrize("bad", ["high", None, True])
def test_non_numeric_rejected(bad):
    with pytest.raises(MissionError):
        parse_mission(variant(cruise__speed_m_s=bad))


def test_negative_height_rejected():
    with pytest.raises(MissionError, match="must be > 0"):
        parse_mission(variant(towers__0__height_m=-3))


def test_empty_towers_rejected():
    with pytest.raises(MissionError, match="non-empty"):
        parse_mission(variant(towers=[]))
