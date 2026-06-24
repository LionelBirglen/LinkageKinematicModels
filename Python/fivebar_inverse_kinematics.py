"""
fivebar_inverse_kinematics.py
Inverse kinematics of a planar five-bar linkage.

BY:
Prof. Lionel Birglen
Polytechnique Montreal, 2025
Contact: lionel.birglen@polymtl.ca
Code provided under GNU Affero General Public License v3.0
"""

import math
import numpy as np


def fivebar_inverse_kinematics(geo, P_des):
    """
    Inverse kinematics of a planar five-bar linkage.

    Finds all configurations (up to 4) that place the coupler point P
    at P_des by intersecting:
      - circle(O, a) ∩ circle(P_des, h)  → up to 2 candidates for A
      - for each A: circle(D, d) ∩ circle(B, c) → up to 2 candidates for C

    Parameters
    ----------
    geo : array-like, length 8
        [a, b, c, d, e, alpha, h, eta]
    P_des : array-like, length 2
        [Px, Py] desired position of coupler point P.

    Returns
    -------
    list of dicts (0 to 4 entries). Each dict has:
        'Positions' : dict with 'O','A','B','C','D' as np.ndarray shape (2,)
        'phi'       : float, orientation of A→B in degrees
        'theta'     : np.ndarray [theta1, theta2] in degrees
        'P'         : np.ndarray shape (2,)
        'valid'     : True

    Example
    -------
    geo = [0.4, 0.5, 0.6, 0.3, 1.0, 0, 0.25, 30]
    sols = fivebar_inverse_kinematics(geo, [0.5298, 0.2439])
    """
    geo = np.asarray(geo, dtype=float)
    a, b, c, d, e, alpha, h, eta = geo
    P_des = np.asarray(P_des, dtype=float).ravel()
    Px, Py = P_des[0], P_des[1]

    alpha_rad = math.radians(alpha)
    O = np.zeros(2)
    D = np.array([e * math.cos(alpha_rad), e * math.sin(alpha_rad)])

    # Step 1: find A candidates — intersection of circle(O,a) and circle(P,h)
    A_list, ok_A = _circle_intersect(O[0], O[1], a, Px, Py, h)
    if not ok_A:
        return []

    solutions = []
    for Ai in A_list:
        # φ from direction A→P, adjusted by eta
        phi_deg = math.degrees(math.atan2(Py - Ai[1], Px - Ai[0])) - eta
        phi_rad = math.radians(phi_deg)

        # B = A + b * (cos φ, sin φ)
        Bi = Ai + b * np.array([math.cos(phi_rad), math.sin(phi_rad)])

        # Step 2: find C candidates — intersection of circle(D,d) and circle(B,c)
        C_list, ok_C = _circle_intersect(D[0], D[1], d, Bi[0], Bi[1], c)
        if not ok_C:
            continue

        for Ci in C_list:
            theta1 = math.degrees(math.atan2(Ai[1] - O[1], Ai[0] - O[0]))
            theta2 = math.degrees(math.atan2(Ci[1] - D[1], Ci[0] - D[0]))
            solutions.append({
                'Positions': {'O': O, 'A': Ai, 'B': Bi, 'C': Ci, 'D': D},
                'phi':   phi_deg,
                'theta': np.array([theta1, theta2]),
                'P':     P_des.copy(),
                'valid': True,
            })

    return solutions


def _circle_intersect(x0, y0, r0, x1, y1, r1):
    """
    Intersect two circles. Returns ([P1, P2], True) or ([], False).
    """
    dx = x1 - x0
    dy = y1 - y0
    dist = math.hypot(dx, dy)
    if dist > (r0 + r1) or dist < abs(r0 - r1) or (dist == 0 and abs(r0 - r1) < 1e-12):
        return [], False
    a_val = (r0**2 - r1**2 + dist**2) / (2 * dist)
    h_val = math.sqrt(max(r0**2 - a_val**2, 0.0))
    x2 = x0 + dx * a_val / dist
    y2 = y0 + dy * a_val / dist
    rx = -dy * h_val / dist
    ry =  dx * h_val / dist
    P1 = np.array([x2 + rx, y2 + ry])
    P2 = np.array([x2 - rx, y2 - ry])
    return [P1, P2], True


if __name__ == '__main__':
    geo = [0.4, 0.5, 0.6, 0.3, 1.0, 0, 0.25, 30]
    sols = fivebar_inverse_kinematics(geo, [0.5298, 0.2439])
    print(f"Found {len(sols)} solutions")
    for k, s in enumerate(sols):
        print(f"  Sol {k+1}: theta={s['theta'].round(2)}  phi={s['phi']:.2f}")
