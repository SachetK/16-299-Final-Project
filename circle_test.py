import time
from inverse_kinematics import *
import xarm
import numpy as np

# Circle center and radius (in inches)
cx, cy, cz = 6.0, 2.0, -0.25
radius = 1.0
n_points = 50  # resolution of the circle

# Generate circle points in the YZ plane
theta = np.linspace(0, 2 * np.pi, n_points)
points = [[cx + radius * np.cos(t), cy + radius * np.sin(t), cz] for t in theta]

duration_ms = 800
pause = (duration_ms + 100) / 1000.0

def main():
    arm = xarm.Controller("USB")

    for point in points:
        move_to_point(arm, point)


def move_to_point(arm: xarm.Controller, point):
    try:
        angles = ik_4dof(*point)
        arm.setPosition(6, angles[0], duration_ms)
        arm.setPosition(5, angles[1], duration_ms)
        arm.setPosition(4, angles[2], duration_ms)
        arm.setPosition(3, angles[3], duration_ms)
        time.sleep(pause)
    except ValueError:
        print(f"Point {point} unreachable — skipping.")

if __name__ == "__main__":
    main()