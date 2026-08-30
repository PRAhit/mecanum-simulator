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

# How much leverage a wheel has for twisting the robot. A wheel further out
# from the centre turns the robot more, the same way a door is easier to push
# at the handle than next to the hinge. It shows up in every rotation term
# below, so it gets a name of its own.
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
    # Dividing by the wheel radius at the end is what turns "how fast the rim
    # should travel over the floor" into "how fast the axle should spin".
    # Bigger wheels spin slower for the same speed across the ground.
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

    # The same plus and minus patterns as wheel_speeds(), read the other way
    # round. Each line pulls out one kind of motion and averages it across the
    # four wheels, which is where the divide-by-four comes from.
    vx = (fl + fr + rr + rl) * WHEEL_RADIUS / 4        # + + + +   forward
    vy = (-fl + fr - rr + rl) * WHEEL_RADIUS / 4       # - + - +   sideways

    # Turning divides by the leverage as well: the same four wheel speeds spin
    # a wide robot more slowly than a narrow one.
    omega = (-fl + fr + rr - rl) * WHEEL_RADIUS / (4 * TURN_ARM)   # - + + -
    return vx, vy, omega


def body_to_world(vx, vy, theta):
    """
    Turn "forward and left, as the robot sees it" into "east and north, as the
    map sees it".

    The robot always thinks in its own directions. If it is facing north and
    drives forward, it moves north; if it is facing east, the same command moves
    it east. So before we can draw the path we have to rotate the robot's own
    velocity by whichever way it happens to be pointing.

    ----------------------------------------------------------------------
    WHY THIS FUNCTION EXISTS -- the same command, traced twice
    ----------------------------------------------------------------------

    Both traces below run the identical command through the whole program.
    The ONLY difference is which way the robot happens to be facing.

    Trace 1 -- robot facing EAST (theta = 0):

        demo.py        typed:  vx = 0.5
        wheel_speeds   ->      [10.0, 10.0, 10.0, 10.0]
        robot_motion   ->      actual_vx = 0.500
        body_to_world  ->      world_x = 0.500   world_y = 0.000
        robot.py       ->      self.x = 0.0100   self.y = 0.0000

    Trace 2 -- the EXACT same command, robot facing NORTH (theta = 90):

        demo.py        typed:  vx = 0.5                     <- identical
        wheel_speeds   ->      [10.0, 10.0, 10.0, 10.0]     <- identical
        robot_motion   ->      actual_vx = 0.500            <- identical
        body_to_world  ->      world_x = 0.000  world_y = 0.500   <- DIFFERENT
        robot.py       ->      self.x = 0.0000  self.y = 0.0100   <- went NORTH

    Same command. Same wheel speeds. Completely different place on the map,
    purely because theta was different.

    This function is where "which way am I facing?" enters the maths, and
    world_x / world_y is where it lands. Everything above this line in the
    trace is blind to heading; everything below it depends on heading.
    ----------------------------------------------------------------------
    """
    # An ordinary 2D rotation by theta -- the standard sine and cosine pair.
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
    # The same rotation run with -theta instead of +theta. Rotating backwards
    # by the same angle is exactly what cancels a rotation out.
    vx = world_x * math.cos(theta) + world_y * math.sin(theta)
    vy = -world_x * math.sin(theta) + world_y * math.cos(theta)
    return vx, vy
