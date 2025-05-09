import numpy as np
import xarm

# geometry (mm)
L1 = 3.125  # base height
L2 = 3.75  # upper arm
L3 = 3.75  # forearm
PX = 1.25  # wrist x-offset
PZ = 3.125  # wrist z-offset


def Rz(a): c, s = np.cos(a), np.sin(a); return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
def Ry(a): c, s = np.cos(a), np.sin(a); return np.array([[c, 0, s, 0], [0, 1, 0, 0], [-s, 0, c, 0], [0, 0, 0, 1]])
def Tx(d): return np.array([[1, 0, 0, d], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
def Tz(d): return np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, d], [0, 0, 0, 1]])

def fk_4dof(t1_deg, t2_deg, t3_deg, t4_deg):
    t1 = np.radians(t1_deg)
    t2 = np.radians(90.0 - t2_deg)  # 0° up, 90° forward
    t3 = np.radians(t3_deg)  # positive bends down
    t4 = np.radians(t4_deg)

    print(t1, t2, t3, t4)
    T = (
        Rz(t1) @ Tz(L1) @
        Ry(-t2) @ Tx(L2) @
        Ry(-t3) @ Tx(L3) @
        Ry(t4) @ Tx(PX) @
        Tz(-PZ)
    )

    return (T @ np.array([0, 0, 0, 1]))[:3]

# arm = xarm.Controller('USB')
# arm.setPosition(6, 0.0)
# arm.setPosition(5, 30.0)
# arm.setPosition(4, 10.0)
# arm.setPosition(3, 10.0)

# # Test cases
# print("θ=[0,90,0,0] →", fk_4dof(0, 90, 0, 0))  # Straight forward
# print("θ=[0,45,0,0] →", fk_4dof(0, 45, 0, 0))  # 45° case
# print("θ=[0,30,10,10] →", fk_4dof(0, 30, 10, 10))  # 45° case
# print("θ=[0,15,0,0] →", fk_4dof(0, 15, 0, 0))  # 45° case
# print("θ=[0,0,0,0] →", fk_4dof(0, 0, 0, 0))  # Straight up
