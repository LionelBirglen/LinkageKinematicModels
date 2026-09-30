"""
stephensonIII_direct_kinematics.py
Direct kinematics of a planar Stephenson III six-bar linkage.

Python port of stephensonIII_direct_kinematics.m (same conventions,
same solution ordering).

BY:
Prof. Lionel Birglen
Polytechnique Montreal, 2025
Contact: lionel.birglen@polymtl.ca
Code provided under GNU Affero General Public License v3.0
"""

import numpy as np


def stephensonIII_direct_kinematics(geo, theta_O):
    """
    Direct kinematics of a Stephenson III six-bar linkage.

    Parameters
    ----------
    geo : array-like, length 12
        [OA, Bx, By, OC, CD, DA, BF, FE, DE, EP, eta, delta]
        OA    : O to A (A lies on the x-axis)
        Bx, By: coordinates of the ground pivot B
        OC    : O to C (input crank)
        CD    : C to D
        DA    : D to A
        BF    : B to F
        FE    : F to E
        DE    : D to E
        EP    : E to P (output point)
        eta   : angle (deg) from EF to EP (positive CCW)
        delta : angle (deg) of body C-D-E, measured as in the MATLAB
                version (rotation of D->C by delta gives the direction
                of D->E)
    theta_O : float
        Input crank angle (deg): angle of O->C w.r.t. the x-axis (CCW)

    Returns
    -------
    list of dict
        One dict per real assembly mode (up to 4: two branches for D,
        times two for F), in the same order as the MATLAB version.
        Each dict has keys:
        'Positions' : dict 'O','A','B','C','D','F','E','P' -> np.ndarray (2,)
        'Angles'    : dict 'thetaC','thetaD','thetaF','thetaA','thetaB',
                      'thetaE','thetaO','theta_CE','theta_EP' (deg)
        'theta_FE'  : float, angle of F->E w.r.t. the x-axis (deg)
        'valid'     : bool
        'thetaO'    : float, input angle (deg)
        'thetaB'    : float, angle of B->F w.r.t. B->O (deg)
        If no configuration assembles, a single dict with valid = False
        and NaN positions is returned.
    """
    OA, Bx, By, OC, CD, DA, BF, FE, DE, EP, eta, delta = \
        np.asarray(geo, dtype=float).ravel()[:12]

    O = np.array([0.0, 0.0])
    A = np.array([OA, 0.0])
    B = np.array([Bx, By])

    theta = np.deg2rad(theta_O)
    C = O + OC * np.array([np.cos(theta), np.sin(theta)])

    R_delta = _rot(np.deg2rad(delta))
    R_eta = _rot(np.deg2rad(eta))

    sols = []
    for D in circle_intersections(C, CD, A, DA):
        if np.any(np.isnan(D)):
            continue

        # E from DE and delta (body C-D-E)
        v_CD_unit = (D - C) / np.linalg.norm(D - C)
        E = D + DE * (R_delta @ (-v_CD_unit))

        # F at the intersection of circles (B, BF) and (E, FE)
        for F in circle_intersections(B, BF, E, FE):
            if np.any(np.isnan(F)):
                continue

            # Output point P: distance EP from E, rotated eta from E->F
            v_EF = F - E
            if np.linalg.norm(v_EF) == 0:
                continue
            P = E + EP * (R_eta @ (v_EF / np.linalg.norm(v_EF)))

            sols.append(_make_solution(O, A, B, C, D, F, E, P,
                                       thetaO=theta_O))

    if not sols:
        sols = [_invalid_solution(thetaO=theta_O, thetaB=np.nan)]
    return sols


# ── Helpers shared with stephensonIII_inverse_kinematics ─────────────────────
def _rot(a):
    return np.array([[np.cos(a), -np.sin(a)],
                     [np.sin(a),  np.cos(a)]])


def rel_angle(v2, v1):
    """Angle from v1 to v2 (deg, CCW positive, in [0, 360))."""
    t = np.degrees(np.arctan2(v2[1], v2[0]) - np.arctan2(v1[1], v1[0]))
    return float(np.mod(t + 360.0, 360.0))


def vec_angle_xaxis(v):
    """Angle of v w.r.t. the x-axis (deg, in [0, 360))."""
    return float(np.mod(np.degrees(np.arctan2(v[1], v[0])) + 360.0, 360.0))


def circle_intersections(c1, r1, c2, r2, tol=0.0):
    """
    Intersections of circles (c1, r1) and (c2, r2), as in the MATLAB
    version: returns (p1, p2) with p1 = p0 + offset, p2 = p0 - offset,
    or two NaN points when the circles do not meet. tol is a small
    tolerance on the tangency tests.
    """
    c1 = np.asarray(c1, dtype=float)
    c2 = np.asarray(c2, dtype=float)
    nan2 = np.array([np.nan, np.nan])
    d = np.linalg.norm(c2 - c1)
    if d == 0 or d > r1 + r2 + tol or d < abs(r1 - r2) - tol:
        return nan2.copy(), nan2.copy()
    a = (r1**2 - r2**2 + d**2) / (2 * d)
    h = np.sqrt(max(0.0, r1**2 - a**2))
    p0 = c1 + a * (c2 - c1) / d
    if h < 1e-12:
        return p0.copy(), p0.copy()
    offset = h * np.array([[0.0, -1.0], [1.0, 0.0]]) @ ((c2 - c1) / d)
    return p0 + offset, p0 - offset


def _make_solution(O, A, B, C, D, F, E, P, thetaO, thetaB=None):
    angles = {
        'thetaC':   rel_angle(D - C, C - O),     # CD w.r.t. CO
        'thetaD':   rel_angle(A - D, D - C),     # DA w.r.t. DC
        'thetaF':   rel_angle(E - F, F - B),     # FE w.r.t. FB
        'thetaA':   rel_angle(B - A, A - O),     # AB w.r.t. AO
        'thetaB':   rel_angle(F - B, B - O),     # BF w.r.t. BO
        'thetaE':   rel_angle(P - E, E - F),     # EP w.r.t. EF
        'thetaO':   rel_angle(C - O, A - O),     # OC w.r.t. OA
        'theta_CE': vec_angle_xaxis(E - C),      # CE w.r.t. x-axis
        'theta_EP': vec_angle_xaxis(P - E),      # EP w.r.t. x-axis
    }
    return {
        'Positions': {'O': O.copy(), 'A': A.copy(), 'B': B.copy(),
                      'C': C.copy(), 'D': D.copy(), 'F': F.copy(),
                      'E': E.copy(), 'P': P.copy()},
        'Angles':    angles,
        'theta_FE':  vec_angle_xaxis(E - F),
        'valid':     True,
        'thetaO':    float(thetaO),
        'thetaB':    angles['thetaB'] if thetaB is None else float(thetaB),
    }


def _invalid_solution(thetaO=np.nan, thetaB=np.nan):
    nan2 = np.array([np.nan, np.nan])
    return {
        'Positions': {k: nan2.copy() for k in 'OABCDFEP'},
        'Angles':    {k: np.nan for k in ('thetaC', 'thetaD', 'thetaF',
                                          'thetaA', 'thetaB', 'thetaE',
                                          'thetaO', 'theta_CE', 'theta_EP')},
        'theta_FE':  np.nan,
        'valid':     False,
        'thetaO':    float(thetaO),
        'thetaB':    float(thetaB),
    }


if __name__ == '__main__':
    geo = [40, 70, 30, 50, 20, 50, 30, 30, -30, 20, 30, 60]
    for s in stephensonIII_direct_kinematics(geo, 90):
        print(np.round(s['Positions']['P'], 4), round(s['thetaB'], 4))
