"""
Q2_leg_mechanism_direct_kinematics.py
Direct kinematics of the planar Q2 leg mechanism.

Python port of Q2_leg_mechanism_direct_kinematics.m (same conventions,
same fixed branch numbering).

BY:
Prof. Lionel Birglen
Polytechnique Montreal, 2025
Contact: lionel.birglen@polymtl.ca
Code provided under GNU Affero General Public License v3.0
"""

import numpy as np

# The 17 independent parameters (12 lengths, 5 angles in rad)
PARAM_NAMES = ('AB', 'AC', 'BD', 'CD', 'FG', 'CF', 'BE', 'GK', 'GJ', 'IF',
               'IJ', 'IP', 'DCF', 'GFC', 'DBE', 'JGK', 'JIP')

JOINTS = ('A', 'B', 'C', 'D', 'E', 'F', 'G', 'K', 'J', 'I')


def Q2_leg_mechanism_direct_kinematics(thetaA, rho, parameters):
    """
    Direct kinematics of the Q2 leg mechanism.

    Parameters
    ----------
    thetaA : float
        Input crank angle of link A->C relative to ground (rad)
    rho : float
        Length of the prismatic actuator E->K
    parameters : dict
        The 17 independent parameters (lengths in consistent units,
        angles in rad):
        Lengths: AB (ground, A = (0,0), B = (AB,0)), AC (input crank),
                 BD, BE (ternary body B-D-E), CD, CF, FG (quaternary body
                 C-D-G-F), GK, GJ (ternary body G-K-J), IF (link I-F),
                 IJ, IP (ternary body I-J-P)
        Angles:  DCF : angle D-C-F at C, clockwise from C->D to C->F
                 GFC : angle G-F-C at F, clockwise from F->C to F->G
                 DBE : angle D-B-E at B, counter-clockwise from B->D to B->E
                 JGK : angle J-G-K at G, clockwise from G->K to G->J
                 JIP : angle J-I-P at I, counter-clockwise from I->J to I->P

    Returns
    -------
    list of 8 dict
        One dict per assembly mode. The index (0-based) is
        k = 4*iD + 2*iK + iI with iD, iK, iI in {0, 1} the branches of the
        D, K and I circle intersections, so a given index always refers
        to the same branch (solution k+1 in the GUI). Each dict has:
        'Positions' : dict 'A','B','C','D','E','F','G','K','J','I'
                      -> np.ndarray (2,)
        'P'         : np.ndarray (2,), point P on body I-J-P
        'phi'       : float, angle of I->J relative to ground (rad)
        'thetaA'    : float, input crank angle (rad)
        'rho'       : float, actuator length
        'valid'     : bool, True if the branch assembles. Positions that
                      could be computed before closure failed are still
                      filled, the rest (and P, phi) are NaN.
        'Twists'    : dict 'xiA', ..., 'xiI' -> np.ndarray (3,),
                      [1, E @ r] with E = [[0,-1],[1,0]] (zero-pitch twist
                      coordinates at each joint)
    """
    p = parameters
    _check_params(p)

    Emat = np.array([[0.0, -1.0], [1.0, 0.0]])
    nan2 = np.full(2, np.nan)
    nan3 = np.full(3, np.nan)
    tw = lambda X: np.r_[1.0, Emat @ X]

    # Fixed pivots and input crank
    A = np.array([0.0, 0.0])
    B = np.array([p['AB'], 0.0])
    C = A + p['AC'] * np.array([np.cos(thetaA), np.sin(thetaA)])

    def blank():
        pos = {k: nan2.copy() for k in JOINTS}
        pos['A'], pos['B'], pos['C'] = A.copy(), B.copy(), C.copy()
        twists = {'xi' + k: nan3.copy() for k in JOINTS}
        twists['xiA'], twists['xiB'], twists['xiC'] = tw(A), tw(B), tw(C)
        return {'Positions': pos, 'P': nan2.copy(), 'phi': np.nan,
                'thetaA': float(thetaA), 'rho': float(rho), 'valid': False,
                'Twists': twists}

    sol = [blank() for _ in range(8)]

    # Solve D (two branches)
    Dsol, okD = _circle_circle_intersection(B, p['BD'], C, p['CD'])
    if not okD:
        return sol                                   # all 8 branches invalid

    R_JGK = np.array([[np.cos(-p['JGK']), -np.sin(-p['JGK'])],
                      [np.sin(-p['JGK']),  np.cos(-p['JGK'])]])

    for iD in range(2):
        D = Dsol[iD]

        # Quaternary body C-D-G-F (F and G depend on D)
        xC = (D - C) / np.linalg.norm(D - C)
        yC = Emat @ xC
        F = C + p['CF'] * (np.cos(p['DCF']) * xC - np.sin(p['DCF']) * yC)
        xF = (C - F) / np.linalg.norm(C - F)
        yF = Emat @ xF
        G = F + p['FG'] * (np.cos(p['GFC']) * xF - np.sin(p['GFC']) * yF)

        # Ternary body B-D-E (E depends on D)
        xB = (D - B) / np.linalg.norm(D - B)
        yB = Emat @ xB
        E = B + p['BE'] * (np.cos(p['DBE']) * xB + np.sin(p['DBE']) * yB)

        # Solve K (two branches)
        Ksol, okK = _circle_circle_intersection(E, rho, G, p['GK'])

        for iK in range(2):
            K, J, okI, Isol = nan2, nan2, False, None
            if okK:
                K = Ksol[iK]
                # Ternary body G-K-J
                u_GK = (K - G) / np.linalg.norm(K - G)
                J = G + p['GJ'] * (R_JGK @ u_GK)
                # Solve I (two branches)
                Isol, okI = _circle_circle_intersection(F, p['IF'], J, p['IJ'])

            for iI in range(2):
                k = 4 * iD + 2 * iK + iI
                s = sol[k]
                pos = s['Positions']
                pos['D'], pos['E'], pos['F'], pos['G'] = D, E, F, G
                pos['K'], pos['J'] = K.copy(), J.copy()
                for name, X in (('D', D), ('E', E), ('F', F), ('G', G)):
                    s['Twists']['xi' + name] = tw(X)
                if okK:
                    s['Twists']['xiK'] = tw(K)
                    s['Twists']['xiJ'] = tw(J)
                if not okI:
                    continue                         # branch does not close

                I = Isol[iI]
                phi = np.arctan2(J[1] - I[1], J[0] - I[0])   # direction I->J
                # Point P on body I-J-P (as P on the fourbar coupler)
                P = I + p['IP'] * np.array([np.cos(phi + p['JIP']),
                                            np.sin(phi + p['JIP'])])
                pos['I'] = I
                s['Twists']['xiI'] = tw(I)
                s['P'] = P
                s['phi'] = float(phi)
                s['valid'] = True
    return sol


def _check_params(p):
    """All 17 independent parameters must be present."""
    if not isinstance(p, dict):
        raise TypeError('parameters must be a dict')
    for name in PARAM_NAMES:
        if name not in p:
            raise KeyError(f'Missing field "{name}" in parameter dict.')


def _circle_circle_intersection(O1, r1, O2, r2):
    """
    Intersections of circles (O1,r1) and (O2,r2), same ordering as the
    MATLAB version: returns ([P0 + offset, P0 - offset], ok).
    """
    d = np.linalg.norm(O2 - O1)
    if d == 0 or d > r1 + r2 or d < abs(r1 - r2):
        return None, False
    a = (r1**2 - r2**2 + d**2) / (2 * d)
    h = np.sqrt(max(r1**2 - a**2, 0.0))
    P0 = O1 + a * (O2 - O1) / d
    offset = h * np.array([[0.0, -1.0], [1.0, 0.0]]) @ ((O2 - O1) / d)
    return [P0 + offset, P0 - offset], True


if __name__ == '__main__':
    p = dict(AB=150, AC=90, BD=120, CD=100, FG=80, CF=90, BE=50, GK=50,
             GJ=80, IF=80, IJ=100, IP=50, DCF=np.deg2rad(80),
             GFC=np.deg2rad(110), DBE=np.deg2rad(45), JGK=np.deg2rad(30),
             JIP=-np.deg2rad(30))
    sol = Q2_leg_mechanism_direct_kinematics(0.0, 154.0, p)
    print('valid branches:', [k + 1 for k, s in enumerate(sol) if s['valid']])
    print('P of solution 2:', np.round(sol[1]['P'], 4))
