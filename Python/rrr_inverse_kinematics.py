"""
rrr_inverse_kinematics.py
Inverse kinematics of a planar RRR serial manipulator.

BY:
Prof. Lionel Birglen
Polytechnique Montreal, 2025
Contact: lionel.birglen@polymtl.ca
Code provided under GNU Affero General Public License v3.0
"""

import numpy as np


def rrr_inverse_kinematics(L1, L2, L3, Px, Py, phi, elbow_config):
    """
    Compute inverse kinematics of a planar 3R serial chain.

    Parameters
    ----------
    L1, L2, L3 : float
        Link lengths
    Px, Py : float
        Desired end-effector position
    phi : float
        Desired end-effector orientation (rad)
    elbow_config : int
        +1 for elbow-up, -1 for elbow-down

    Returns
    -------
    dict with keys:
        'Positions' : dict with 'O','A','B','P' as np.ndarray shape (2,)
        'theta1','theta2','theta3' : float (rad)
        'phi'   : float
        'valid' : bool
    """
    nan2 = np.full(2, np.nan)
    sol = {
        'Positions': {'O': np.zeros(2), 'A': nan2.copy(),
                      'B': nan2.copy(), 'P': np.array([Px, Py])},
        'theta1': np.nan, 'theta2': np.nan, 'theta3': np.nan,
        'phi': phi, 'valid': False
    }

    Wx = Px - L3 * np.cos(phi)
    Wy = Py - L3 * np.sin(phi)
    R2 = Wx**2 + Wy**2

    cos_t2 = (R2 - L1**2 - L2**2) / (2 * L1 * L2)
    if abs(cos_t2) > 1:
        return sol

    theta2 = np.arctan2(elbow_config * np.sqrt(max(1 - cos_t2**2, 0)), cos_t2)
    k1 = L1 + L2 * np.cos(theta2)
    k2 = L2 * np.sin(theta2)
    theta1 = np.arctan2(Wy, Wx) - np.arctan2(k2, k1)
    theta3 = phi - (theta1 + theta2)

    A1 = theta1
    A2 = theta1 + theta2
    A = np.array([L1 * np.cos(A1), L1 * np.sin(A1)])
    B = A + np.array([L2 * np.cos(A2), L2 * np.sin(A2)])

    sol['Positions']['A'] = A
    sol['Positions']['B'] = B
    sol['theta1'] = theta1
    sol['theta2'] = theta2
    sol['theta3'] = theta3
    sol['valid']  = True
    return sol
