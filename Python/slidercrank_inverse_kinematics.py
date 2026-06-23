"""
slidercrank_inverse_kinematics.py
Inverse kinematics of a planar slider-crank mechanism.

BY:
Prof. Lionel Birglen
Polytechnique Montreal, 2025
Contact: lionel.birglen@polymtl.ca
Code provided under GNU Affero General Public License v3.0
"""

import numpy as np


def slidercrank_inverse_kinematics(a, b, c, x_slider, config, slider_angle):
    """
    Compute inverse kinematics of a planar slider-crank.

    Parameters
    ----------
    a : float
        Crank length
    b : float
        Coupler length
    c : float
        Extension link from B to P (may be negative)
    x_slider : float
        Desired displacement of B along slider axis
    config : int
        +1 elbow-down, -1 elbow-up
    slider_angle : float
        Orientation of prismatic joint axis (rad)

    Returns
    -------
    dict with keys:
        'Positions' : dict with 'O','A','B','P' as np.ndarray shape (2,)
        'phi'       : float, crank angle (rad)
        'x_slider'  : float
        'slider_dir': np.ndarray shape (2,)
        'valid'     : bool
    """
    O          = np.zeros(2)
    slider_dir = np.array([np.cos(slider_angle), np.sin(slider_angle)])
    B          = x_slider * slider_dir

    nan2 = np.full(2, np.nan)
    sol = {
        'Positions': {'O': O, 'A': nan2.copy(), 'B': B.copy(),
                      'P': B + c * slider_dir},
        'phi': np.nan, 'x_slider': x_slider,
        'slider_dir': slider_dir, 'valid': False
    }

    A1, A2, valid = _circle_intersections(O, a, B, b)
    if not valid:
        return sol

    A   = A1 if config == 1 else A2
    phi = np.arctan2(A[1] - O[1], A[0] - O[0])
    phi = np.arctan2(np.sin(phi), np.cos(phi))

    Pt = B + c * slider_dir

    sol['Positions']['A'] = A
    sol['Positions']['B'] = B
    sol['Positions']['P'] = Pt
    sol['phi']            = phi
    sol['valid']          = True
    return sol


def _circle_intersections(c1, r1, c2, r2):
    d = np.linalg.norm(c2 - c1)
    if d > (r1 + r2) or d < abs(r1 - r2):
        nan2 = np.full(2, np.nan)
        return nan2, nan2, False
    a_val = (r1**2 - r2**2 + d**2) / (2 * d)
    h     = np.sqrt(max(r1**2 - a_val**2, 0.0))
    p2    = c1 + a_val * (c2 - c1) / d
    perp  = h * np.array([[0, -1], [1, 0]]) @ ((c2 - c1) / d)
    return p2 + perp, p2 - perp, True
