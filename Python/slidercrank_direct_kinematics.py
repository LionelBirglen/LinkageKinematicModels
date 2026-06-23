"""
slidercrank_direct_kinematics.py
Direct kinematics of a planar slider-crank mechanism.

BY:
Prof. Lionel Birglen
Polytechnique Montreal, 2025
Contact: lionel.birglen@polymtl.ca
Code provided under GNU Affero General Public License v3.0
"""

import numpy as np


def slidercrank_direct_kinematics(a, b, c, phi, slider_angle, config=+1):
    """
    Compute direct kinematics of a planar slider-crank.

    Parameters
    ----------
    a : float
        Crank length
    b : float
        Coupler length
    c : float
        Extension link from B to P (may be negative)
    phi : float
        Crank angle (rad)
    slider_angle : float
        Orientation of prismatic joint axis (rad)
    config : int
        +1 elbow-down, -1 elbow-up

    Returns
    -------
    dict with keys:
        'Positions' : dict with 'O','A','B','P' as np.ndarray shape (2,)
        'x_slider'  : float, displacement of B along slider axis
        'phi'       : float (rad)
        'slider_dir': np.ndarray shape (2,)
        'valid'     : bool
    """
    O = np.zeros(2)
    A = a * np.array([np.cos(phi), np.sin(phi)])

    slider_dir    = np.array([np.cos(slider_angle), np.sin(slider_angle)])
    slider_normal = np.array([-np.sin(slider_angle), np.cos(slider_angle)])

    perp_dist     = np.dot(A - O, slider_normal)
    perp_dist_abs = abs(perp_dist)
    perp_sign     = np.sign(perp_dist)

    nan2 = np.full(2, np.nan)
    sol  = {
        'Positions': {'O': O, 'A': nan2.copy(), 'B': nan2.copy(), 'P': nan2.copy()},
        'x_slider': np.nan, 'phi': phi, 'slider_dir': slider_dir, 'valid': False
    }

    if perp_dist_abs > b:
        return sol

    perp_point = A - perp_sign * perp_dist_abs * slider_normal
    along_dist = np.sqrt(max(b**2 - perp_dist_abs**2, 0.0))

    B1 = perp_point + along_dist * slider_dir
    B2 = perp_point - along_dist * slider_dir
    B  = B1 if config == 1 else B2

    P = B + c * slider_dir

    sol['Positions']['A'] = A
    sol['Positions']['B'] = B
    sol['Positions']['P'] = P
    sol['x_slider']       = float(np.dot(B - O, slider_dir))
    sol['valid']          = True
    return sol
