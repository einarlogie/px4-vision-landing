import re
import xml.etree.ElementTree as ET
from pathlib import Path

from tools.gen_powerline_world import generate
from vland.mission import load_mission

REPO = Path(__file__).resolve().parents[1]
MISSION = REPO / "missions" / "powerline.yaml"


def _generated() -> str:
    return generate(load_mission(MISSION), (REPO / "worlds" / "landing_pad.sdf").read_text())


def test_committed_world_matches_mission():
    # worlds/powerline.sdf must be regenerated whenever the mission changes.
    assert (REPO / "worlds" / "powerline.sdf").read_text() == _generated()


def test_towers_and_pad_placed_from_mission():
    m = load_mission(MISSION)
    world = ET.fromstring(re.sub(r"<!--.*?-->", "", _generated(), flags=re.S)).find("world")
    assert world.get("name") == "powerline"
    poses = {mdl.get("name"): mdl.find("pose").text.split() for mdl in world.findall("model")
             if mdl.find("pose") is not None}
    for t in m.towers:
        x, y = map(float, poses[f"tower_{t.id}"][:2])
        assert (x, y) == (t.east_m, t.north_m)          # ENU: x = east, y = north
    pad = [i for i in world.findall("include") if i.find("name").text == "landing_pad"][0]
    px, py = map(float, pad.find("pose").text.split()[:2])
    assert (px, py) == (m.pad.east_m, m.pad.north_m)


def test_three_conductors_per_span():
    m = load_mission(MISSION)
    assert _generated().count('name="span') == 3 * (len(m.towers) - 1)
