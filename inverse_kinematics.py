import numpy as np
import math

from forward_kinematics import *

# -------- inverse kinematics --------
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

# To keep the pen facing down, we want θ4 = 90 - θ2 + θ3
# Test cases

# print("θ=[0,90,0,0] →", fk_4dof(0, 90, 0))  # Straight forward
# print("Inverse solution: ", ik_4dof(*fk_4dof(0, 90, 0)))
# print("θ=[0,45,0,45] →", fk_4dof(10, 45, 30))  # 45° case
# print("Inverse solution: ", ik_4dof(*fk_4dof(10, 30, 80)))

# print("θ=[0,30,10,10] →", fk_4dof(0, 30, 10, 10))  # 45° case
# print("θ=[0,15,0,0] →", fk_4dof(0, 15, 0, 0))  # 45° case
# print("θ=[0,0,0,0] →", fk_4dof(0, 0, 0, 0))  # Straight up
