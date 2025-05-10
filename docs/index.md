# Drawing with the HiWonder XArm 1S

### A 4-DOF Robotic Arm That Draws What You Tell It

![XArm Drawing](images/xArm.jpg)

---

## 📜 Introduction
This webpage documents my project for 16-299: Making Decisions: Robot Control, Planning, and Learning at CMU. 
Have you ever wanted your robot to draw? In this project, I transformed a HiWonder XArm 1S robotic arm into a 
4-degree-of-freedom drawing machine that can trace shapes and draw pictures. The robot calculates the correct 
joint angles to move a pen along a path on paper using analytical inverse kinematics. This tutorial walks you through 
the entire process — from concept to code to a working demo.

## 🎯 Core Objectives

These were the main goals I set out to accomplish in this project:

* Enable 3D Position Control: Precisely control the HiWonder XArm 1S in 3D space using inverse kinematics.
* Draw Flat Shapes on a Surface: Use a mounted pen to draw shapes on paper.
* Keep the Pen Vertical: Maintain a consistent pen orientation using wrist pitch compensation.
* Handle Real Hardware Constraints: Space out commands appropriately to avoid servo skipping or instability.
* Execute Complex Paths: Go beyond basic circles and lines to draw compound shapes like a house, including doors and windows.

---

## 🔧 Hardware Used/Supplies

* HiWonder XArm 1S (4 DOF version) (Provided by Professor Atkeson)
* USB connection to a host computer
* Pen holder (a couple of zip ties)
* Power supply (5–7.4V recommended)
* Drawing surface (paper or whiteboard)
* Tape/something to clamp the drawing surface down to the ground

---

## 💡 How It Works

The XArm uses four servo motors to position a pen tip in 3D space:

- **θ₁** rotates the base
- **θ₂** and **θ₃** bend the arm
- **θ₄** tilts the wrist to keep the pen vertical

Two mathematical models power the system:

- **Forward Kinematics (FK):** computes the pen's position from servo angles
- **Inverse Kinematics (IK):** solves for the angles required to reach a target position

The goal is to give the robot a list of (x, y, z) positions and have it draw by moving smoothly through them.

---
## 🧮 Step 1: Forward Kinematics

The function below computes the pen-tip position from 3 joint angles:

```python
def fk_4dof(t1_deg, t2_deg, t3_deg):
    t1 = np.radians(t1_deg)
    t2 = np.radians(90.0 - t2_deg)  # 0° up, 90° forward
    t3 = np.radians(t3_deg)  # positive bends down
    t4 = np.radians(90-t2_deg+t3_deg)

    T = (
        Rz(t1) @ Tz(L1) @
        Ry(-t2) @ Tx(L2) @
        Ry(-t3) @ Tx(L3) @
        Ry(t4) @ Tx(PX) @
        Tz(-PZ)
    )

    return (T @ np.array([0, 0, 0, 1]))[:3]
```
Each matrix (Ry, Rz, Tx, Tz) represents a transformation 
This produces a homogeneous transformation matrix from base to pen tip.

We define:

* `t2 = radians(90 - theta2)` (shoulder 0° = straight up)
* `t3 = radians(theta3)` (elbow positive = bend down)
* `t4 = radians(90 - theta2 + theta3)` (wrist compensation)

---

## 🤖 Step 2: Inverse Kinematics

Given a 3D point (x, y, z), we calculate joint angles as follows:

```python
def ik_4dof(x, y, z, theta4_override=None):
    # For all intents and purposes, we only need the elbow up solution
    # since its just drawing. Thus, only calculate the elbow up solution

    def wrap_angle(theta):
        return (theta + 180) % 360 - 180

    theta1 = math.degrees(math.atan2(y, x))
    r_tip  = math.hypot(x, y)
    r_w    = r_tip - PX
    z_w    = z + PZ
    dz     = z_w - L1
    D2     = r_w**2 + dz**2

    if D2 > ((L2+L3)**2 + 1e-6) or D2 < ((L2-L3)**2 - 1e-6):
        raise ValueError("unreachable")

    cos_m3 = (D2 - L2**2 - L3**2) / (2*L2*L3)
    cos_m3 = np.clip(cos_m3, -1.0, 1.0)
    m3     = -math.acos(cos_m3)
    theta3 = math.degrees(m3)

    phi = math.atan2(dz, r_w)
    psi = math.atan2(L3*math.sin(m3), L2+L3*math.cos(m3))
    m2 = phi - psi
    theta2 = 90.0 - math.degrees(m2)

    theta4 = theta4_override if theta4_override is not None \
             else 90 - theta2 + theta3

    joint_angles = np.array([wrap_angle(theta1), wrap_angle(theta2), wrap_angle(theta3), wrap_angle(theta4)])

    for angle in joint_angles:
        if angle > 125 or angle < -125:
            raise ValueError("angle is outside range")

    return joint_angles
```
The first joint angle is calculated by finding atan2 of x and y. This gives the orientation of the base.
Next, angles 2 and 3 are solved using the Law of Cosines, and finally, we get angle 4 of being 90 - theta2 + theta3
in order to maintain the vertical orientation of the pen.

This gives of the two valid configurations, of which we choose the elbow-up one since it keeps the arms from hitting
the base or table.

---

## 🖋️ Step 3: Drawing a Path

You can define a path as an array of 3D points, for example:

```python
path = [
    [6.0, 0.0, 1.0],
    [6.0, 0.25, 1.0],
    [6.0, 0.5, 1.0],
    ...,
    [6.0, 4.0, 1.0]
]
```

Each point is passed through `ik_4dof(...)` and sent to the XArm via USB.

```python
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
```
---
## 🌀 Step 4: Draw a House
Now that we've verified the robot can trace smooth paths, let's move on to something more complex: drawing a house! 
The house includes a square base, triangular roof, a door, and a window — all drawn using a list of 
Cartesian coordinates.

Here's the full Python script:
```python
import time
import xarm
from inverse_kinematics import ik_4dof

points = [
    [5, 0, -0.25],
    [5, 1, -0.25],
    [6, 1, -0.25],
    [6, 0, -0.25],
    [5, 0, -0.25], # house outline
    [5, 0, 0.5],   # pen up
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
    [6.25, 0.25, -0.25],  # window box
    [6.5, 0.75, 0.5],
    [6.5, 0.75, -0.25],
    [6.5, 0.25, -0.25],
    [6.5, 0.25, 0.5],     # window divider
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
```
Each line or shape is drawn point by point, with pen lifts (Z = 0.5) between sections.

---

## 🧪 Lessons Learned

* Homogeneous transforms made FK clean and modular
* Inverse kinematics can be done entirely analytically for a 4-DOF chain
* Clamping `acos` inputs avoids domain errors due to floating-point precision
* Wrist angle correction is essential for consistent drawing

---
## 🔍 Reflections & Future Improvements

This project laid the groundwork for precise robotic drawing using analytical kinematics, 
but there are several ways I would improve it going forward:

* **Higher-Precision Servos:** The current servos have noticeable backlash and positional error. 
Upgrading to more precise, feedback-enabled servos would allow for drawing smoother shapes like 
perfect circles and curves.

* **SVG Path Interpretation:** One of my original goals was to input an SVG file and translate its paths into 
robot-friendly motion commands. This would let the robot draw logos, letters, or complex illustrations. 
The basic path-following framework is already in place, and adding SVG parsing would be a powerful next step. 

* **Smoother Motion Execution:** Currently, the arm sometimes jitters or produces wavy lines when drawing. 
A future improvement would be to implement trajectory smoothing, such as interpolating between points or using easing 
functions to avoid abrupt changes in angle.

These improvements would make the robot not just functional, but genuinely expressive as a drawing tool. 
I learned a lot while working on this project and working with the robot. I'd like to thank Professor Atkeson and the TA Krishna for the opportunity and all the gracious help 
they have provided. Thank you!

## 🚀 Try It Yourself

1. Clone the repo
2. Plug in your XArm and power it
3. Run the Python script:

   ```bash
   python house_demo.py
   ```

You can define your own path arrays and modify the script to draw letters, shapes, or even signatures.

---

## 📸 Demo

[Here](images/drawing_demo.mp4) is a demo of the robot actually drawing. 
Below is the final house that it drew.
![Arm Drawing Demo](images/drawing.jpg)

---

## 🗂️ Files and Code

You can find all source code and data in the [GitHub repo](
https://github.com/SachetK/16-299-Final-Project)

Made by Sachet Korada

[Back to Top](#drawing-with-the-hiwonder-xarm-1s)
