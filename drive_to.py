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

# Three numbers, and each one is there to stop a specific silly behaviour.
GAIN = 1.5           # how hard to chase the target. In PID language this is Kp.
TOP_SPEED = 0.8      # m/s -- a speed cap. Without it a target 100 m away would
                     # ask for 150 m/s. Capping like this is called saturation.
CLOSE_ENOUGH = 0.02  # metres -- a deadband. Without it the robot never settles,
                     # because it can never land on the point exactly.


def velocity_towards(robot, target_x, target_y):
    """
    Work out which way the robot should drive to reach the target.

    Returns (vx, vy) in the robot's own directions, ready to hand to drive().
    Returns (0, 0) once it has arrived.
    """
    # How far away is it, on the map? A controller would call this the error:
    # the gap between where we are and where we want to be.
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
    """
    Have we got there yet?

    Same test velocity_towards() uses to decide to stop, pulled out so a
    caller can ask the question without also asking for a velocity.
    """
    return math.hypot(target_x - robot.x, target_y - robot.y) < CLOSE_ENOUGH
