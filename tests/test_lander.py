import math

import pytest

from vland.lander import LanderConfig, Phase, PrecisionLander, Target


def test_starts_in_search_and_holds():
    lander = PrecisionLander()
    cmd = lander.update(0.0, None, altitude=6.0)
    assert cmd.phase == Phase.SEARCH and (cmd.forward, cmd.right, cmd.down) == (0, 0, 0)


def test_search_times_out():
    lander = PrecisionLander(LanderConfig(search_timeout=5.0))
    lander.update(0.0, None, 6.0)
    assert lander.update(5.1, None, 6.0).phase == Phase.ABORT


def test_moves_towards_marker_with_speed_limit():
    lander = PrecisionLander()
    cmd = lander.update(0.0, Target(forward=3.0, right=-4.0, height=6.0), 6.0)
    assert cmd.phase == Phase.TRACK
    assert math.hypot(cmd.forward, cmd.right) == pytest.approx(lander.cfg.v_max)
    # Direction is towards the marker.
    assert cmd.forward > 0 and cmd.right < 0
    assert cmd.forward / cmd.right == pytest.approx(3.0 / -4.0)


def test_no_descent_outside_cone():
    lander = PrecisionLander()
    # At 6 m the cone allows 0.15 + 0.12*6 = 0.87 m.
    assert lander.update(0.0, Target(1.0, 0.0, 6.0), 6.0).down == 0.0
    assert lander.update(0.1, Target(0.5, 0.0, 6.0), 6.0).down == pytest.approx(lander.cfg.descend_fast)


def test_slower_near_ground():
    lander = PrecisionLander()
    cmd = lander.update(0.0, Target(0.05, 0.0, 1.5), 1.5)
    assert cmd.down == pytest.approx(lander.cfg.descend_slow)


def test_final_when_low_and_centred():
    lander = PrecisionLander()
    assert lander.update(0.0, Target(0.3, 0.0, 0.7), 0.9).phase == Phase.TRACK   # low but off-centre
    assert lander.update(0.1, Target(0.05, 0.02, 0.7), 0.9).phase == Phase.FINAL
    # FINAL is latched: later frames don't change it.
    assert lander.update(0.2, None, 0.9).phase == Phase.FINAL


def test_hold_then_climb_then_reacquire():
    cfg = LanderConfig(hold_time=1.0)
    lander = PrecisionLander(cfg)
    lander.update(0.0, Target(0.2, 0.0, 3.0), 3.0)
    assert lander.update(0.5, None, 3.0).phase == Phase.HOLD
    cmd = lander.update(1.6, None, 3.0)
    assert cmd.phase == Phase.CLIMB and cmd.down == pytest.approx(-cfg.climb_rate)
    assert lander.update(2.0, Target(0.1, 0.0, 3.5), 3.5).phase == Phase.TRACK
    assert lander.loss_events == 1


def test_climb_stops_at_ceiling():
    lander = PrecisionLander(LanderConfig(max_altitude=8.0, hold_time=0.0))
    lander.update(0.0, Target(0.0, 0.0, 7.9), 7.9)
    lander.update(0.1, None, 7.9)
    assert lander.update(0.2, None, 8.1).down == 0.0


def test_lost_too_long_aborts():
    lander = PrecisionLander(LanderConfig(lost_timeout=3.0))
    lander.update(0.0, Target(0.0, 0.0, 4.0), 4.0)
    lander.update(0.1, None, 4.0)
    assert lander.update(3.2, None, 4.0).phase == Phase.ABORT
