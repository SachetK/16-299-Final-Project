import time

import xarm

from inverse_kinematics import ik_4dof

points = [
    [5, 0, -0.25],
    [5, 1, -0.25],
    [6, 1, -0.25],
    [6, 0, -0.25],
    [5, 0, -0.25], # house
    [5, 0, 0.5], # pen up
    [6, 1.5, -0.25],
    [7, 0.5, -0.25],
    [6, -0.5, -0.25],
    [6, 1.5, -0.25], # roof
    [5, 0.25, 0.5], # pen up
    [5, 0.25, -0.25],
    [5, 0.75, -0.25],
    [5.5, 0.75, -0.25],
    [5.5, 0.25, -0.25],
    [5, 0.25, -0.25], # door
    [6.25, 0.25, 0.5],  # pen up
    [6.25, 0.25, -0.25],
    [6.25, 0.75, -0.25],
    [6.75, 0.75, -0.25],
    [6.75, 0.25, -0.25],
    [6.25, 0.25, -0.25],  # window
    [6.5, 0.75, 0.5],
    [6.5, 0.75, -0.25],
    [6.5, 0.25, -0.25],
    [6.5, 0.25, 0.5],
]

duration_ms = 800
pause = (duration_ms + 100) / 1000.0

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

def main():
    arm = xarm.Controller('USB')
    for point in points:
        move_to_point(arm, point)


if __name__ == '__main__':
    main()