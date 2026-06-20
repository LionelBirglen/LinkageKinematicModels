import numpy as np

def rrr_inverse_kinematics(L1, L2, L3, Px, Py, phi, elbow_config):
    """
    Compute inverse kinematics of a planar RRR manipulator.
    Returns joint angles theta1, theta2, theta3, and success flag.
    """
    # wrist position
    xw = Px - L3 * np.cos(phi)
    yw = Py - L3 * np.sin(phi)

    r2 = xw**2 + yw**2
    c2 = (r2 - L1**2 - L2**2) / (2 * L1 * L2)
    if abs(c2) > 1.0:
        return None, None, None, False

    s2 = elbow_config * np.sqrt(1 - c2**2)
    theta2 = np.arctan2(s2, c2)

    k1 = L1 + L2 * c2
    k2 = L2 * s2
    theta1 = np.arctan2(yw, xw) - np.arctan2(k2, k1)

    theta3 = phi - theta1 - theta2
    return theta1, theta2, theta3, True