from dataclasses import replace

from los_tracking import BarometerSample, LosNavigator, NavigationInput, TargetObservation

from conftest import DEFAULT_BOX, acquire, samples


def test_alignment_wait_and_replacement_are_counted_once(config):
    nav = LosNavigator(config)
    acquire(nav)
    waiting = TargetObservation('camera', 35, .345, DEFAULT_BOX)
    inputs = NavigationInput(waiting, BarometerSample('baro', 35, .35, 10), None)
    first = nav.update(inputs, now_s=.35, enabled=True)
    assert first.diagnostics.alignment_waits == 1
    assert first.diagnostics.alignment_replacements == 0

    repeated = nav.update(NavigationInput(None, samples(36).barometer, None), now_s=.36, enabled=True)
    assert repeated.diagnostics.alignment_waits == 1
    newer = TargetObservation('camera', 36, .375, DEFAULT_BOX)
    replaced = nav.update(replace(samples(37, camera=False), target=newer), now_s=.38, enabled=True)
    assert replaced.diagnostics.alignment_waits == 2
    assert replaced.diagnostics.alignment_replacements == 1
