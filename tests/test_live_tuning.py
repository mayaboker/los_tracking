from dataclasses import replace
import pytest
from los_tracking import LosNavigator, AltitudeHeadingHold, State
from conftest import acquire, samples


def test_live_gains_preserve_tracking_and_affect_control(config):
    nav=LosNavigator(config)
    acquire(nav)
    nav._control.lateral_memory=.01
    history=nav._attitudes
    bearing=nav._bearing
    before=(nav._heading, nav._reference,nav._control.angles,nav._control.lateral_memory)
    nav.apply_live_tuning(replace(config.tuning,azimuth_kp=1.5))
    assert nav.state==State.TRACK
    assert (nav._heading, nav._reference,nav._control.angles,nav._control.lateral_memory)==before
    assert nav._attitudes is history and nav._bearing is bearing
    r=nav.update(samples(35,bbox=(315,280,20,20)),now_s=.35,enabled=True)
    assert r.usable
    assert abs(r.command.roll_rad-before[2][0])<=config.tuning.roll_slew_rad_s*.01+1e-12
    assert nav._control.tuning.azimuth_kp==1.5
    baseline=LosNavigator(config)
    acquire(baseline)
    baseline._control.lateral_memory=.01
    unchanged=baseline.update(samples(35,bbox=(315,280,20,20)),now_s=.35,enabled=True)
    assert r.diagnostics.requested_roll_rad != pytest.approx(unchanged.diagnostics.requested_roll_rad)


def test_live_update_rejects_structural_changes_and_commit(config):
    nav=LosNavigator(config)
    with pytest.raises(ValueError,match='selected'):
        nav.apply_live_tuning(replace(config.tuning,bearing_filter_s=.3))
    assert nav.config==config
    acquire(nav)
    nav.update(samples(35,bbox=(0,0,640,480)),now_s=.35,enabled=True)
    with pytest.raises(ValueError,match='committed'):
        nav.apply_live_tuning(replace(config.tuning,azimuth_kp=1.5))
    assert nav.config==config


def test_hold_handoff_matches_live_tuned_navigator(config):
    hold=AltitudeHeadingHold(config)
    for i in range(35):hold.observe(samples(i))
    hold.command(now_s=.34,target_altitude_m=10.)
    state=(hold.control.thrust_integral,hold.control.angles,hold.heading_rad,hold.vertical.time)
    tuning=replace(config.tuning,yaw_gain_s_inv=1.)
    hold.apply_live_tuning(tuning)
    assert (hold.control.thrust_integral,hold.control.angles,hold.heading_rad,hold.vertical.time)==state
    nav=LosNavigator(config)
    nav.apply_live_tuning(tuning)
    nav.reset(handoff=hold.handoff(now_s=.34,target_altitude_m=10.))
    assert nav.config==hold.config
