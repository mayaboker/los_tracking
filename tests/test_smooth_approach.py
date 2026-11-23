"""Sensor-only approaches from 20, 30 and 40 m using the Bullet tuning."""

from pathlib import Path

import pytest

from los_tracking import (AttitudeSample, BarometerSample, LosNavigator, NavigationInput,
                          NavigatorConfig, State, TargetObservation)
from los_tracking.geometry import rotate

from conftest import from_rpy


PROFILE = Path(__file__).parents[1] / 'examples/bullet_profile.yaml'
TARGET = (60.0, 0.0, 1.0)
DT = .01


def target_box(position, attitude, camera):
    vector = tuple(target - current for target, current in zip(TARGET, position))
    inverse = tuple(-value for value in attitude[:3]) + (attitude[3],)
    x, y, z = rotate(inverse, vector)
    if x <= 0:
        return None
    u = camera.cx_px - camera.fx_px * y / x
    v = camera.cy_px - camera.fy_px * z / x
    size = camera.fx_px / x  # One-metre square target in this fixture.
    left, top = max(0., u - size / 2), max(0., v - size / 2)
    right, bottom = min(camera.width_px, u + size / 2), min(camera.height_px, v + size / 2)
    if right <= left or bottom <= top:
        return None
    return (left, top, right - left, bottom - top)


@pytest.mark.parametrize('start_altitude_m', (20.0, 30.0, 40.0))
def test_bullet_profile_tracks_through_smoothed_approach(start_altitude_m):
    config = NavigatorConfig.from_yaml(PROFILE)
    nav = LosNavigator(config)
    position = [0.0, 0.0, start_altitude_m]
    velocity = [0.0, 0.0, 0.0]
    roll = pitch = yaw = 0.0
    states = set()
    near_target_requests = []
    max_bottom_edge_boost = 0.0
    first_commit_distance = None
    contact = False
    for index in range(5000):
        now = index * DT
        attitude = from_rpy(roll, pitch, yaw)
        bbox = target_box(position, attitude, config.camera)
        inputs = NavigationInput(
            TargetObservation('camera', index, now, bbox) if index % 4 == 0 else None,
            BarometerSample('baro', index, now, position[2]) if index % 2 == 0 else None,
            AttitudeSample('attitude', index, now, attitude))
        result = nav.update(inputs, now_s=now, enabled=True)
        states.add(result.state)
        max_bottom_edge_boost = max(max_bottom_edge_boost, result.diagnostics.bottom_edge_boost_m_s)
        if result.state == State.COMMIT and first_commit_distance is None:
            first_commit_distance = sum((target - current) ** 2 for target, current in zip(TARGET, position)) ** .5
        assert result.state not in (State.FAULT, State.RECOVER), (
            now, position, velocity, result.reason, result.diagnostics.vision_reason)
        if position[0] > 50 and result.state == State.TRACK:
            near_target_requests.append(result.diagnostics.requested_vertical_speed_m_s)
        if result.usable:
            roll += (result.command.roll_rad - roll) * DT / .12
            pitch += (result.command.pitch_rad - pitch) * DT / .12
            yaw += result.command.yaw_rate_rad_s * DT
            thrust = result.command.thrust_n
        else:
            thrust = config.vehicle.mass_kg * config.vehicle.gravity_m_s2
        ax, ay, az = rotate(from_rpy(roll, pitch, yaw), (0, 0, thrust))
        accelerations = (ax - .2 * velocity[0], ay - .2 * velocity[1], az - 9.81)
        for axis in range(3):
            velocity[axis] += accelerations[axis] * DT
            position[axis] += velocity[axis] * DT
        if sum((target - current) ** 2 for target, current in zip(TARGET, position)) < .8 ** 2:
            contact = True
            break
    assert State.TRACK in states
    assert contact, (position, velocity, result.reason, result.diagnostics.vision_reason)
    if start_altitude_m == 40.0:
        assert State.COMMIT in states
        assert first_commit_distance < 1.5
        assert max_bottom_edge_boost > 0
    assert min(near_target_requests) > -2.5
    assert abs(velocity[2]) < 1.0
