"""
fourbar_direct_kinematics.py
Direct kinematics of a planar four-bar linkage.

BY:
Prof. Lionel Birglen
Polytechnique Montreal, 2025
Contact: lionel.birglen@polymtl.ca
Code provided under GNU Affero General Public License v3.0
"""

import numpy as np


def fourbar_direct_kinematics(geo, theta):
    """
    Compute forward kinematics of a planar four-bar linkage.

    Parameters
    ----------
    geo : array-like, length 7
        [a, b, c, d, e, epsilon, delta]
        a       : output link length O→A
        b       : coupler link length B→A
        c       : input crank length C→B
        d       : ground link length O→C
        e       : distance A→P along coupler
        epsilon : angle between coupler (A→B) and A→P (rad)
        delta   : angle locating C: C = [d*cos(delta), d*sin(delta)] (rad)
    theta : float
        Input crank angle C→B (rad)

    Returns
    -------
    list of dict
        Two solution dicts (one per assembly mode), each with keys:
        'Positions' : dict with 'O','A','B','C' as np.ndarray shape (2,)
        'P'         : np.ndarray shape (2,)
        'phi'       : float, coupler angle A→B (rad)
        'alpha'     : float, output link angle O→A (rad)
        'theta'     : float, input crank angle (rad)
        'valid'     : bool
    """
    geo = np.asarray(geo, dtype=float)
    a, b, c, d, e, epsilon, delta = geo

    O = np.array([0.0, 0.0])
    C = np.array([d * np.cos(delta), d * np.sin(delta)])
    B = C + c * np.array([np.cos(theta), np.sin(theta)])

    R = np.linalg.norm(O - B)

    solutions = []
    nan2 = np.array([np.nan, np.nan])

    if R > (a + b) or R < abs(a - b):
        # Both solutions invalid
        for _ in range(2):
            solutions.append({
                'Positions': {'O': O, 'A': nan2.copy(), 'B': B, 'C': C},
                'P': nan2.copy(), 'phi': np.nan, 'alpha': np.nan,
                'theta': theta, 'valid': False
            })
        return solutions

    angle_BO = np.arctan2(O[1] - B[1], O[0] - B[0])
    cos_arg  = np.clip((b**2 + R**2 - a**2) / (2 * b * R), -1.0, 1.0)
    epsilon_B = np.arccos(cos_arg)

    for k, cfg in enumerate([-1, +1]):
        phi_old = angle_BO + cfg * epsilon_B
        phi_old = np.arctan2(np.sin(phi_old), np.cos(phi_old))

        phi = phi_old + np.pi
        phi = np.arctan2(np.sin(phi), np.cos(phi))

        A = B + b * np.array([np.cos(phi_old), np.sin(phi_old)])
        alpha = np.arctan2(A[1], A[0])
        alpha = np.arctan2(np.sin(alpha), np.cos(alpha))

        P = A + e * np.array([np.cos(phi + epsilon), np.sin(phi + epsilon)])

        solutions.append({
            'Positions': {'O': O, 'A': A, 'B': B, 'C': C},
            'P': P, 'phi': phi, 'alpha': alpha,
            'theta': theta, 'valid': True
        })

    return solutions
