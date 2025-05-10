import numpy as np
import math

from numpy.ma.core import arctan2

from forward_kinematics import *

# -------- inverse kinematics --------
def ik_4dof(x, y, z, theta4_override=None):
    # For all intents and purposes, we only need the elbow up solution
    # since its just drawing. Thus, only calculate the elbow up solution

    def wrap_to_360(theta):
        return theta + 360

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

    return [theta1, theta2, theta3, theta4]
