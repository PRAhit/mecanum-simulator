"""
Mecanum wheel maths.

A mecanum wheel has little rollers set at 45 degrees around its rim. Because of
those rollers, a wheel pushes the robot diagonally instead of straight ahead.
Put four of them on a robot in the right pattern and the diagonal pushes can be
combined to move the robot in ANY direction -- including straight sideways --
without turning first. That is the whole trick, and this file is the maths for it.

Wheel order used everywhere in this project:

    0 = front left      1 = front right
    3 = rear left       2 = rear right      (looking down at the robot)

Robot coordinates:
    x = forward,  y = left,  theta = anticlockwise turn
"""

import math

# --- robot dimensions (a small hobby-sized robot) -------------------------
WHEEL_RADIUS = 0.05   # metres  (5 cm wheels)
HALF_LENGTH = 0.15    # metres  (front axle to centre)
HALF_WIDTH = 0.15     # metres  (centre to left wheels)

# This combination shows up in every rotation term, so name it once.
TURN_ARM = HALF_LENGTH + HALF_WIDTH


def wheel_speeds(vx, vy, omega):
    """
    Work out how fast to spin each wheel.

    Give it the motion you WANT:
        vx    = forward speed      (m/s)
        vy    = sideways speed     (m/s, positive is left)
        omega = turning speed      (rad/s, positive is anticlockwise)

    Get back the four wheel speeds in rad/s.

    Notice the pattern of plus and minus signs. Sideways motion (vy) makes one
    diagonal pair spin forwards and the other pair backwards -- that is what
    produces a sideways push with nothing pointing sideways.
    """
    return [
        (vx - vy - TURN_ARM * omega) / WHEEL_RADIUS,   # front left
        (vx + vy + TURN_ARM * omega) / WHEEL_RADIUS,   # front right
        (vx - vy + TURN_ARM * omega) / WHEEL_RADIUS,   # rear right
        (vx + vy - TURN_ARM * omega) / WHEEL_RADIUS,   # rear left
    ]


def robot_motion(speeds):
    """
    The opposite question: the wheels are spinning at these speeds, so how is
    the robot actually moving?

    This is how a real robot estimates where it has gone, by reading the
    encoders on its motors. Returns (vx, vy, omega).
    """
    fl, fr, rr, rl = speeds
    vx = (fl + fr + rr + rl) * WHEEL_RADIUS / 4
    vy = (-fl + fr - rr + rl) * WHEEL_RADIUS / 4
    omega = (-fl + fr + rr - rl) * WHEEL_RADIUS / (4 * TURN_ARM)
    return vx, vy, omega


def body_to_world(vx, vy, theta):
    """
    Turn "forward and left, as the robot sees it" into "east and north, as the
    map sees it".

    The robot always thinks in its own directions. If it is facing north and
    drives forward, it moves north; if it is facing east, the same command moves
    it east. So before we can draw the path we have to rotate the robot's own
    velocity by whichever way it happens to be pointing.
    """
    world_x = vx * math.cos(theta) - vy * math.sin(theta)
    world_y = vx * math.sin(theta) + vy * math.cos(theta)
    return world_x, world_y


def world_to_body(world_x, world_y, theta):
    """
    The opposite of body_to_world.

    We want to drive north, but the robot only understands "forward" and
    "left" -- so this works out what that means from where it is standing.
    Same maths as above with the angle flipped, which is what undoes a rotation.
    """
    vx = world_x * math.cos(theta) + world_y * math.sin(theta)
    vy = -world_x * math.sin(theta) + world_y * math.cos(theta)
    return vx, vy
