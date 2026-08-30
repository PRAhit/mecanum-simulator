"""
Tests.

Run them with:   python3 -m pytest -v

These are the checks that stopped me getting the signs wrong. Every one of them
is something I actually got backwards at least once while writing this.
"""

import math

import drive_to
import mecanum
from robot import Robot


def test_driving_forward_spins_every_wheel_the_same_way():
    """Nothing clever should happen when you just go straight."""
    speeds = mecanum.wheel_speeds(1.0, 0, 0)
    assert all(s > 0 for s in speeds)
    assert max(speeds) - min(speeds) < 1e-9   # all four identical


def test_sliding_sideways_splits_the_wheels_into_two_pairs():
    """This is the mecanum trick, so it gets its own test."""
    front_left, front_right, rear_right, rear_left = mecanum.wheel_speeds(0, 1.0, 0)
    assert front_left < 0 and rear_right < 0     # one diagonal backwards
    assert front_right > 0 and rear_left > 0     # the other forwards


def test_spinning_turns_the_left_and_right_sides_opposite_ways():
    """Spinning on the spot should look like a tank: one side forward, one back."""
    front_left, front_right, rear_right, rear_left = mecanum.wheel_speeds(0, 0, 1.0)
    assert front_left < 0 and rear_left < 0      # left side backwards
    assert front_right > 0 and rear_right > 0    # right side forwards


def test_the_maths_undoes_itself():
    """
    Ask for a motion, work out the wheel speeds, then read those wheel speeds
    back. We should get exactly what we asked for. If this fails, one of the
    two formulas has a sign wrong.
    """
    wanted = (0.4, -0.2, 0.3)
    speeds = mecanum.wheel_speeds(*wanted)
    got = mecanum.robot_motion(speeds)
    for want, actually in zip(wanted, got):
        assert abs(want - actually) < 1e-9


def test_a_robot_facing_sideways_still_goes_where_you_point_it():
    """
    The controller works in map directions but the robot drives in its own,
    so this is where a rotation mistake would show up. Start it facing three
    different ways; it should reach the same target every time.
    """
    for start_angle in (0.0, math.pi / 2, -1.2):
        robot = Robot(theta=start_angle)
        for _ in range(500):
            vx, vy = drive_to.velocity_towards(robot, 2.0, 1.0)
            robot.drive(vx, vy, 0.0, 0.02)
        assert drive_to.arrived(robot, 2.0, 1.0)


def test_it_slows_down_as_it_arrives():
    """Speed proportional to distance means it should ease in, not slam in."""
    far = Robot(x=0.0, y=0.0)
    near = Robot(x=1.9, y=0.0)
    speed_far = math.hypot(*drive_to.velocity_towards(far, 2.0, 0.0))
    speed_near = math.hypot(*drive_to.velocity_towards(near, 2.0, 0.0))
    assert speed_far > speed_near


def test_it_stops_when_it_gets_there():
    """Sitting exactly on the target must give a clean zero, not a small twitch."""
    robot = Robot(x=2.0, y=1.0)
    assert drive_to.velocity_towards(robot, 2.0, 1.0) == (0.0, 0.0)


def test_the_square_comes_back_to_the_start():
    """Drive a full square and check we end up where we began."""
    robot = Robot()
    for vx, vy in [(0.5, 0), (0, 0.5), (-0.5, 0), (0, -0.5)]:
        for _ in range(100):
            robot.drive(vx, vy, 0.0, 0.02)
    assert abs(robot.x) < 1e-6
    assert abs(robot.y) < 1e-6
