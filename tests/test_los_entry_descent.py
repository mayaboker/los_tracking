"""First-track descent starts promptly without changing recovery handoff."""

from dataclasses import replace

import pytest

from los_tracking import LosNavigator, State
from los_tracking.config import Tuning
from los_tracking.control import LosControl

from conftest import IDENTITY, samples


def test_entry_descent_is_bounded_by_max_descent():
    assert Tuning().los_entry_descent_m_s == 0
    assert Tuning().los_min_track_descent_m_s == 0
    with pytest.raises(ValueError, match='LOS entry descent'):
        Tuning(los_entry_descent_m_s=6.)
    with pytest.raises(ValueError, match='LOS minimum tracking descent'):
        Tuning(los_min_track_descent_m_s=6.)


def test_tracking_descent_floor_is_bounded_and_optional(config):
    tuning = replace(config.tuning, los_min_track_descent_m_s=1.,
                     elevation_kp_m_s_rad=0., elevation_kd_m_rad=0., elevation_ki_m_s2_rad=0.)
    control = LosControl(replace(config, tuning=tuning))
    control.vertical_bias = .4
    control.step(.1, 10., 0., IDENTITY, 0., errors=(.1, 0.), min_descent_m_s=1.)
    assert control.last_diagnostics['requested_vertical_speed_m_s'] == -1.
    assert control.vz_reference == pytest.approx(-.1)

    control.vertical_bias = -2.
    control.step(.1, 10., 0., IDENTITY, 0., errors=(.1, 0.), min_descent_m_s=1.)
    assert control.last_diagnostics['requested_vertical_speed_m_s'] == -2.

    control.vertical_bias = .4
    control.step(.1, 10., 0., IDENTITY, 0., errors=(-.1, 0.))
    assert control.last_diagnostics['requested_vertical_speed_m_s'] == .4


def test_tracking_descent_floor_survives_integral_freeze(config):
    tuning = replace(config.tuning, los_min_track_descent_m_s=1.,
                     elevation_kp_m_s_rad=0., elevation_kd_m_rad=0., elevation_ki_m_s2_rad=1.)
    control = LosControl(replace(config, tuning=tuning))
    control.vertical_bias = .4
    control.step(.1, 10., 0., IDENTITY, 0., errors=(.1, 0.), min_descent_m_s=1.)
    assert control.vertical_memory == 0.
    assert control.last_diagnostics['requested_vertical_speed_m_s'] == -1.


def test_tracking_floor_requires_target_below_and_releases_in_recovery(config, monkeypatch):
    tuning = replace(config.tuning, los_min_track_descent_m_s=1.)
    nav = LosNavigator(replace(config, tuning=tuning))
    limits = []
    original = nav._control.step

    def observe(*args, **kwargs):
        limits.append(kwargs['min_descent_m_s'])
        return original(*args, **kwargs)

    monkeypatch.setattr(nav._control, 'step', observe)
    _first_track(nav, bbox=(310., 280., 20., 20.))
    assert limits[-1] == 1.
    index = round(nav._last_now * 100) + 1
    nav.update(samples(index, bbox=None), now_s=index / 100, enabled=True)
    assert nav.state == State.RECOVER
    assert limits[-1] == 0.

    above = LosNavigator(replace(config, tuning=tuning))
    above_limits = []
    above_step = above._control.step

    def observe_above(*args, **kwargs):
        above_limits.append(kwargs['min_descent_m_s'])
        return above_step(*args, **kwargs)

    monkeypatch.setattr(above._control, 'step', observe_above)
    _first_track(above, bbox=(310., 100., 20., 20.))
    assert above_limits[-1] == 0.


def _first_track(nav, *, bbox):
    for index in range(35):
        result = nav.update(samples(index, bbox=bbox), now_s=index / 100, enabled=True)
        if result.state == State.TRACK:
            return result
    raise AssertionError('target was not acquired')


def test_first_track_requests_entry_descent_for_target_below(config):
    tuning = replace(config.tuning, los_entry_descent_m_s=1.5, vertical_slew_m_s2=2.)
    nav = LosNavigator(replace(config, tuning=tuning))
    result = _first_track(nav, bbox=(310., 280., 20., 20.))
    assert result.diagnostics.requested_vertical_speed_m_s == pytest.approx(-1.5, abs=.1)
    assert result.diagnostics.vertical_reference_m_s < 0


def test_target_above_does_not_force_entry_descent(config):
    tuning = replace(config.tuning, los_entry_descent_m_s=1.5)
    nav = LosNavigator(replace(config, tuning=tuning))
    result = _first_track(nav, bbox=(310., 100., 20., 20.))
    assert result.diagnostics.requested_vertical_speed_m_s == pytest.approx(0., abs=.1)


def test_reacquisition_does_not_repeat_entry_descent(config, monkeypatch):
    tuning = replace(config.tuning, los_entry_descent_m_s=1.5)
    nav = LosNavigator(replace(config, tuning=tuning))
    calls = []
    original = nav._control.begin_track

    def observe(*args, **kwargs):
        calls.append(kwargs['entry_descent_m_s'])
        return original(*args, **kwargs)

    monkeypatch.setattr(nav._control, 'begin_track', observe)
    _first_track(nav, bbox=(310., 280., 20., 20.))
    for index in range(round(nav._last_now * 100) + 1, 35):
        nav.update(samples(index, bbox=(310., 280., 20., 20.)),
                   now_s=index / 100, enabled=True)
    for index in range(35, 61):
        nav.update(samples(index, bbox=None), now_s=index / 100, enabled=True)
    assert nav.state == State.RECOVER
    for index in range(61, 81):
        result = nav.update(samples(index, bbox=(310., 280., 20., 20.)),
                            now_s=index / 100, enabled=True)
        if result.state == State.TRACK:
            break
    assert calls == [1.5, 0.0]
