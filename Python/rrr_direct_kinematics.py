import numpy as np

def rrr_direct_kinematics(L1, L2, L3, theta1, theta2, theta3):
    """
    Compute forward kinematics of a planar RRR manipulator.
    Returns joint coordinates P1, P2, P3.
    """
    x1 = L1 * np.cos(theta1)
    y1 = L1 * np.sin(theta1)
    P1 = np.array([x1, y1])

    theta12 = theta1 + theta2
    x2 = x1 + L2 * np.cos(theta12)
    y2 = y1 + L2 * np.sin(theta12)
    P2 = np.array([x2, y2])

    theta123 = theta12 + theta3
    x3 = x2 + L3 * np.cos(theta123)
    y3 = y2 + L3 * np.sin(theta123)
    P3 = np.array([x3, y3])

    return P1, P2, P3