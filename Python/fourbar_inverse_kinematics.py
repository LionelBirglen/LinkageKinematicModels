"""
fourbar_inverse_kinematics.py
Inverse kinematics of a planar four-bar linkage.

BY:
Prof. Lionel Birglen
Polytechnique Montreal, 2025
Contact: lionel.birglen@polymtl.ca
Code provided under GNU Affero General Public License v3.0
"""

import numpy as np


def fourbar_inverse_kinematics(geo, alpha):
    """
    Compute inverse kinematics of a planar four-bar linkage.

    Parameters
    ----------
    geo : array-like, length 7
        [a, b, c, d, e, epsilon, delta]
    alpha : float
        Desired output-link angle O→A (rad)

    Returns
    -------
    list of dict
        Two solution dicts (one per assembly mode), each with keys:
        'Positions' : dict with 'O','A','B','C' as np.ndarray shape (2,)
        'P'         : np.ndarray shape (2,)
        'phi'       : float, coupler angle (rad)
        'alpha'     : float (same as input)
        'theta'     : float, input crank angle (rad)
        'valid'     : bool
    """
    geo = np.asarray(geo, dtype=float)
    a, b, c, d, e, epsilon, delta = geo

    O = np.array([0.0, 0.0])
    C = np.array([d * np.cos(delta), d * np.sin(delta)])
    A = a * np.array([np.cos(alpha), np.sin(alpha)])

    B1, B2, valid = _circle_intersections(A, b, C, c)
    nan2 = np.array([np.nan, np.nan])

    solutions = []
    if not valid:
        for _ in range(2):
            solutions.append({
                'Positions': {'O': O, 'A': A, 'B': nan2.copy(), 'C': C},
                'P': nan2.copy(), 'phi': np.nan, 'alpha': alpha,
                'theta': np.nan, 'valid': False
            })
        return solutions

    for B in (B1, B2):
        phi   = np.arctan2(B[1] - A[1], B[0] - A[0])
        theta = np.arctan2(B[1] - C[1], B[0] - C[0])
        P     = A + e * np.array([np.cos(phi + epsilon), np.sin(phi + epsilon)])
        solutions.append({
            'Positions': {'O': O, 'A': A, 'B': B, 'C': C},
            'P': P, 'phi': phi, 'alpha': alpha,
            'theta': theta, 'valid': True
        })

    return solutions


def _circle_intersections(c1, r1, c2, r2):
    """Return (P1, P2, valid) for intersection of two circles."""
    d = np.linalg.norm(c2 - c1)
    if d > (r1 + r2) or d < abs(r1 - r2):
        nan2 = np.array([np.nan, np.nan])
        return nan2, nan2, False
    a_val = (r1**2 - r2**2 + d**2) / (2 * d)
    h     = np.sqrt(max(r1**2 - a_val**2, 0.0))
    p2    = c1 + a_val * (c2 - c1) / d
    perp  = h * np.array([[0, -1], [1, 0]]) @ ((c2 - c1) / d)
    return p2 + perp, p2 - perp, True
