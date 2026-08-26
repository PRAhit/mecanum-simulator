"""
Run the demos and save the pictures.

    python3 demo.py

Everything it draws goes into the images/ folder.
"""

import math
import os

import matplotlib
matplotlib.use("Agg")  # save files instead of opening a window
import matplotlib.pyplot as plt

import drive_to
import mecanum
from robot import Robot

STEP = 0.02  # seconds per step -- the simulation moves in 20 ms nudges
OUT = "images"


def demo_sideways():
    """
    Drive a square without ever turning.

    This is the demo worth looking at. A normal car has to turn to change
    direction; this robot just slides. It ends up back where it started, still
    facing the same way, having driven a complete square.
    """
    robot = Robot()
    path_x, path_y = [robot.x], [robot.y]

    # forward, left, backward, right -- two seconds each, no turning at all
    for vx, vy in [(0.5, 0), (0, 0.5), (-0.5, 0), (0, -0.5)]:
        for _ in range(100):
            robot.drive(vx, vy, 0.0, STEP)
            path_x.append(robot.x)
            path_y.append(robot.y)

    plt.figure(figsize=(5, 5))
    plt.plot(path_x, path_y, linewidth=2)
    plt.plot(path_x[0], path_y[0], "o", markersize=9, label="start and finish")
    plt.title("A square, driven without ever turning")
    plt.xlabel("x (metres)")
    plt.ylabel("y (metres)")
    plt.axis("equal")
    plt.grid(alpha=0.3)
    plt.margins(0.15)
    plt.legend(loc="upper right", fontsize=9)
    plt.tight_layout()
    plt.savefig(f"{OUT}/square.png", dpi=110)
    plt.close()

    print(f"square:   finished at ({robot.x:.2f}, {robot.y:.2f}), "
          f"facing {math.degrees(robot.theta):.0f} degrees")


def demo_drive_to():
    """Chase a few targets in turn, using the simple controller."""
    robot = Robot()
    targets = [(2.0, 0.0), (2.0, 1.5), (0.5, 1.5), (0.0, 0.0)]
    path_x, path_y = [robot.x], [robot.y]

    for target_x, target_y in targets:
        for _ in range(1000):  # give up after 20 seconds
            if drive_to.arrived(robot, target_x, target_y):
                break
            vx, vy = drive_to.velocity_towards(robot, target_x, target_y)
            robot.drive(vx, vy, 0.0, STEP)
            path_x.append(robot.x)
            path_y.append(robot.y)

    plt.figure(figsize=(5.5, 5))
    plt.plot(path_x, path_y, linewidth=2, label="path driven")
    plt.plot([t[0] for t in targets], [t[1] for t in targets],
             "x", markersize=11, markeredgewidth=2.5, label="targets")
    plt.title("Driving to a list of targets")
    plt.xlabel("x (metres)")
    plt.ylabel("y (metres)")
    plt.axis("equal")
    plt.grid(alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig(f"{OUT}/targets.png", dpi=110)
    plt.close()

    print(f"targets:  visited {len(targets)}, "
          f"finished at ({robot.x:.2f}, {robot.y:.2f})")


def demo_wheel_speeds():
    """
    Show what each wheel does for the three basic moves.

    Same four wheels every time -- only the pattern of directions changes.
    That pattern IS the mecanum trick, so it is worth seeing side by side.
    """
    moves = [
        ("drive forward", (0.5, 0.0, 0.0)),
        ("slide left", (0.0, 0.5, 0.0)),
        ("spin on the spot", (0.0, 0.0, 1.0)),
    ]
    names = ["front left", "front right", "rear right", "rear left"]

    fig, axes = plt.subplots(1, 3, figsize=(11, 3.4), sharey=True)
    for ax, (title, motion) in zip(axes, moves):
        speeds = mecanum.wheel_speeds(*motion)
        colours = ["tab:blue" if s >= 0 else "tab:orange" for s in speeds]
        ax.bar(names, speeds, color=colours)
        ax.axhline(0, color="black", linewidth=0.8)
        ax.set_title(title)
        ax.tick_params(axis="x", rotation=30)
        ax.grid(axis="y", alpha=0.3)
    axes[0].set_ylabel("wheel speed (rad/s)")
    fig.suptitle("Blue spins forwards, orange spins backwards")
    fig.tight_layout()
    fig.savefig(f"{OUT}/wheels.png", dpi=110)
    plt.close(fig)

    for title, motion in moves:
        speeds = [round(s, 1) for s in mecanum.wheel_speeds(*motion)]
        print(f"wheels:   {title:17} -> {speeds}")


if __name__ == "__main__":
    os.makedirs(OUT, exist_ok=True)
    demo_sideways()
    demo_drive_to()
    demo_wheel_speeds()
    print(f"\nPictures saved in {OUT}/")
