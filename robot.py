"""
A pretend robot.

There is no real hardware here. This file just keeps track of where the robot
is and moves it a little bit at a time, which is enough to watch how it behaves.
"""

import mecanum


class Robot:
    """Remembers where the robot is and moves it when told to."""

    def __init__(self, x=0.0, y=0.0, theta=0.0):
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
        speeds = mecanum.wheel_speeds(vx, vy, omega)
        actual_vx, actual_vy, actual_omega = mecanum.robot_motion(speeds)

        # Where does that push the robot on the map?
        world_x, world_y = mecanum.body_to_world(actual_vx, actual_vy, self.theta)

        self.x += world_x * dt
        self.y += world_y * dt
        self.theta += actual_omega * dt

    def position(self):
        return self.x, self.y
