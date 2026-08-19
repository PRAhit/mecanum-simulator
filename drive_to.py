"""
Driving to a point.

The simplest useful controller there is: look at how far away the target is,
and drive towards it at a speed proportional to that distance. Far away means
go fast, close means slow down, arrived means stop.

This is the "P" in a PID controller, on its own. It is not clever, but it is
easy to understand and it works.
"""

import math

import mecanum

GAIN = 1.5          # how hard to chase the target
TOP_SPEED = 0.8     # m/s -- do not drive faster than this
CLOSE_ENOUGH = 0.02  # metres -- stop fussing once this close


def velocity_towards(robot, target_x, target_y):
    """
    Work out which way the robot should drive to reach the target.

    Returns (vx, vy) in the robot's own directions, ready to hand to drive().
    Returns (0, 0) once it has arrived.
    """
    # How far away is it, on the map?
    away_x = target_x - robot.x
    away_y = target_y - robot.y
    distance = math.hypot(away_x, away_y)

    if distance < CLOSE_ENOUGH:
        return 0.0, 0.0

    # Speed proportional to distance, but capped so it never bolts off.
    speed = min(GAIN * distance, TOP_SPEED)

    # Point that speed at the target.
    world_vx = speed * away_x / distance
    world_vy = speed * away_y / distance

    # That answer is in map directions. The robot only understands its own
    # directions, so translate it before handing it over.
    return mecanum.world_to_body(world_vx, world_vy, robot.theta)


def arrived(robot, target_x, target_y):
    return math.hypot(target_x - robot.x, target_y - robot.y) < CLOSE_ENOUGH
