"""
rrr_direct_kinematics.py
Direct kinematics of a planar RRR serial manipulator.

BY:
Prof. Lionel Birglen
Polytechnique Montreal, 2025
Contact: lionel.birglen@polymtl.ca
Code provided under GNU Affero General Public License v3.0
"""

import numpy as np


def rrr_direct_kinematics(L1, L2, L3, theta1, theta2, theta3):
    """
    Compute forward kinematics of a planar 3R serial chain.

    Parameters
    ----------
    L1, L2, L3 : float
        Link lengths
    theta1 : float
        Absolute angle of link 1 w.r.t. base frame (rad)
    theta2 : float
        Relative angle between link 1 and link 2 (rad)
    theta3 : float
        Relative angle between link 2 and link 3 (rad)

    Returns
    -------
    dict with keys:
        'Positions' : dict with 'O','A','B','P' as np.ndarray shape (2,)
        'phi'       : float, absolute end-effector orientation (rad)
        'valid'     : True
    """
    A1 = theta1
    A2 = theta1 + theta2
    A3 = A2 + theta3

    O = np.zeros(2)
    A = O + L1 * np.array([np.cos(A1), np.sin(A1)])
    B = A + L2 * np.array([np.cos(A2), np.sin(A2)])
    P = B + L3 * np.array([np.cos(A3), np.sin(A3)])

    return {
        'Positions': {'O': O, 'A': A, 'B': B, 'P': P},
        'phi': A3,
        'valid': True
    }
