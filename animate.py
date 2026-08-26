"""
Make the animated picture for the README.

    python3 animate.py

Draws the robot driving a square sideways, and saves it as images/robot.gif.
This is the same simulation as demo.py -- it just draws every step instead of
only the finished path, so you can watch the robot slide rather than turn.
"""

import math
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np

from robot import Robot

STEP = 0.02
BODY = 0.18   # half the width of the robot, for drawing it


def corners(robot):
    """The four corners of the robot body, so we can draw it as a square."""
    points = [(-BODY, -BODY), (BODY, -BODY), (BODY, BODY), (-BODY, BODY)]
    out = []
    for px, py in points:
        # rotate the corner by the robot's heading, then shift to its position
        x = robot.x + px * math.cos(robot.theta) - py * math.sin(robot.theta)
        y = robot.y + px * math.sin(robot.theta) + py * math.cos(robot.theta)
        out.append((x, y))
    return np.array(out + [out[0]])   # repeat the first corner to close the shape


def main():
    # Drive the square first and remember every position along the way.
    robot = Robot()
    frames = []
    labels = []
    for (vx, vy), label in zip(
        [(0.5, 0), (0, 0.5), (-0.5, 0), (0, -0.5)],
        ["driving forwards", "sliding LEFT", "driving backwards", "sliding RIGHT"],
    ):
        for _ in range(100):
            robot.drive(vx, vy, 0.0, STEP)
            frames.append((robot.x, robot.y, robot.theta))
            labels.append(label)

    # Only keep every 4th step, otherwise the gif is 400 frames long.
    frames = frames[::4]
    labels = labels[::4]

    fig, ax = plt.subplots(figsize=(5, 5))
    ax.set_xlim(-0.45, 1.45)
    ax.set_ylim(-0.45, 1.45)
    ax.set_aspect("equal")
    ax.grid(alpha=0.3)
    ax.set_xlabel("x (metres)")
    ax.set_ylabel("y (metres)")

    trail, = ax.plot([], [], linewidth=1.5, alpha=0.6)
    body, = ax.plot([], [], linewidth=2.5, color="tab:orange")
    nose, = ax.plot([], [], linewidth=3.5, color="tab:red")
    title = ax.set_title("")
    fig.tight_layout()

    trail_x, trail_y = [], []

    def draw(i):
        x, y, theta = frames[i]
        trail_x.append(x)
        trail_y.append(y)
        trail.set_data(trail_x, trail_y)

        r = Robot(x, y, theta)
        shape = corners(r)
        body.set_data(shape[:, 0], shape[:, 1])

        # a short line showing which way the robot is facing -- it never moves
        nose.set_data([x, x + 1.6 * BODY * math.cos(theta)],
                      [y, y + 1.6 * BODY * math.sin(theta)])

        title.set_text(f"{labels[i]}   (never turns)")
        return trail, body, nose, title

    anim = animation.FuncAnimation(fig, draw, frames=len(frames), interval=40, blit=False)

    os.makedirs("images", exist_ok=True)
    anim.save("images/robot.gif", writer=animation.PillowWriter(fps=25))
    plt.close(fig)
    print(f"wrote images/robot.gif ({len(frames)} frames)")


if __name__ == "__main__":
    main()
