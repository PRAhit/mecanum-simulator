"""
A pretend robot.

There is no real hardware here. This file just keeps track of where the robot
is and moves it a little bit at a time, which is enough to watch how it behaves.
"""

import mecanum


class Robot:
    """Remembers where the robot is and moves it when told to."""

    def __init__(self, x=0.0, y=0.0, theta=0.0):
        """Put the robot somewhere, facing some direction, and stop there."""
        # These three numbers together are all there is to know about the
        # robot at any moment. The usual name for them is the robot's "pose".
        self.x = x          # position east, in metres
        self.y = y          # position north, in metres
        self.theta = theta  # which way it is facing, in radians

    def drive(self, vx, vy, omega, dt):
        """
        Drive for a short moment of time.

        dt is that moment, in seconds -- 0.02 means 20 milliseconds. Small steps
        repeated many times add up to smooth motion, the same way a film is
        really lots of still pictures.

        The wheel speeds are worked out and then immediately converted back.
        That looks like a pointless round trip, but it is the honest thing to
        simulate: a real robot can only move by spinning its wheels, so putting
        the maths in the loop keeps the simulation tied to what the wheels can
        actually do.
        """
        # 1. what would the four wheels have to do to make that happen?
        speeds = mecanum.wheel_speeds(vx, vy, omega)

        # 2. and given the wheels are doing that, how does the robot move?
        actual_vx, actual_vy, actual_omega = mecanum.robot_motion(speeds)

        # 3. that answer is in the robot's own directions. Where does it push
        #    the robot on the map? This is the one step that depends on which
        #    way the robot is facing -- see the two traces written out in
        #    mecanum.body_to_world() for the same command landing in two
        #    completely different places.
        world_x, world_y = mecanum.body_to_world(actual_vx, actual_vy, self.theta)

        # 4. take one small step. Speed multiplied by time is distance, and
        #    over a slice this short a straight line is close enough to the
        #    truth. (The proper name for adding up steps like this is
        #    integration; this simplest version of it is called Euler's.)
        self.x += world_x * dt
        self.y += world_y * dt
        self.theta += actual_omega * dt

    def position(self):
        """Where the robot has got to, as (x, y) in metres."""
        return self.x, self.y
