"""
fivebar_direct_kinematics.py
Direct kinematics of a planar five-bar linkage.

BY:
Prof. Lionel Birglen
Polytechnique Montreal, 2025
Contact: lionel.birglen@polymtl.ca
Code provided under GNU Affero General Public License v3.0
"""

import math
import numpy as np


def fivebar_direct_kinematics(geo, theta):
    """
    Direct kinematics of a planar five-bar linkage.

    Parameters
    ----------
    geo : array-like, length 8
        [a, b, c, d, e, alpha, h, eta]
        a     : left input crank length  (O→A)
        b     : left coupler / output link (A→B)
        c     : right coupler (C→B)
        d     : right input crank (D→C)
        e     : ground pivot spacing (O→D)
        alpha : angle of O→D from x-axis (degrees)
        h     : distance A→P along output link
        eta   : angle from A→B to A→P (degrees)
    theta : array-like, length 2
        [theta1, theta2] input crank angles in degrees.
        theta1 = angle of left crank O→A
        theta2 = angle of right crank D→C

    Returns
    -------
    list of 2 dicts, one per assembly mode. Each dict has:
        'Positions' : dict with 'O','A','B','C','D' as np.ndarray shape (2,)
        'phi'       : float, orientation of A→B in degrees
        'P'         : np.ndarray shape (2,), coupler point
        'theta'     : np.ndarray [theta1, theta2] in degrees
        'valid'     : bool

    Example
    -------
    geo = [0.4, 0.5, 0.6, 0.3, 1.0, 0, 0.25, 30]
    sols = fivebar_direct_kinematics(geo, [45, -30])
    """
    geo = np.asarray(geo, dtype=float)
    a, b, c, d, e, alpha, h, eta = geo

    theta1 = math.radians(theta[0])
    theta2 = math.radians(theta[1])
    eta_rad = math.radians(eta)
    alpha_rad = math.radians(alpha)

    O = np.zeros(2)
    D = np.array([e * math.cos(alpha_rad), e * math.sin(alpha_rad)])
    A = np.array([a * math.cos(theta1), a * math.sin(theta1)])
    C = D + np.array([d * math.cos(theta2), d * math.sin(theta2)])

    nan2 = np.full(2, np.nan)

    def _make_sol(B, valid):
        if valid:
            phi_rad = math.atan2(B[1] - A[1], B[0] - A[0])
            phi_deg = math.degrees(phi_rad)
            P = A + h * np.array([math.cos(phi_rad + eta_rad),
                                   math.sin(phi_rad + eta_rad)])
        else:
            phi_deg = np.nan
            B = nan2.copy()
            P = nan2.copy()
        return {
            'Positions': {'O': O, 'A': A, 'B': B, 'C': C, 'D': D},
            'phi':   phi_deg,
            'P':     P,
            'theta': np.array([math.degrees(theta1), math.degrees(theta2)]),
            'valid': valid,
        }

    d_AC = np.linalg.norm(C - A)

    if d_AC > (b + c) or d_AC < abs(b - c) or (d_AC == 0 and b == c):
        return [_make_sol(nan2, False), _make_sol(nan2, False)]

    l = (b**2 - c**2 + d_AC**2) / (2 * d_AC)
    h_int = math.sqrt(max(b**2 - l**2, 0.0))
    P2 = A + (l / d_AC) * (C - A)

    ux = -(C[1] - A[1]) / d_AC
    uy =  (C[0] - A[0]) / d_AC

    B1 = P2 + h_int * np.array([ux, uy])
    B2 = P2 - h_int * np.array([ux, uy])

    return [_make_sol(B1, True), _make_sol(B2, True)]


if __name__ == '__main__':
    geo = [0.4, 0.5, 0.6, 0.3, 1.0, 0, 0.25, 30]
    sols = fivebar_direct_kinematics(geo, [45, -30])
    for k, s in enumerate(sols):
        print(f"Sol {k+1}: valid={s['valid']}  B={s['Positions']['B'].round(4)}  P={s['P'].round(4)}")
