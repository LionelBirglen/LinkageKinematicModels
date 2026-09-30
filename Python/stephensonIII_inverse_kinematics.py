"""
stephensonIII_inverse_kinematics.py
Inverse kinematics of a planar Stephenson III six-bar linkage.

Python port of stephensonIII_inverse_kinematics.m (same method, same
conventions, solutions sorted by thetaO).

BY:
Prof. Lionel Birglen
Polytechnique Montreal, 2025
Contact: lionel.birglen@polymtl.ca
Code provided under GNU Affero General Public License v3.0
"""

import numpy as np

from stephensonIII_direct_kinematics import (
    circle_intersections, _rot, _make_solution, _invalid_solution)


def stephensonIII_inverse_kinematics(geo, thetaB):
    """
    Inverse kinematics of a Stephenson III six-bar linkage.

    Parameters
    ----------
    geo : array-like, length 12
        [OA, Bx, By, OC, CD, DA, BF, FE, DE, EP, eta, delta]
        (see stephensonIII_direct_kinematics; eta, delta in degrees)
    thetaB : float
        Angle (deg, CCW) of B->F, measured from the direction of O->B.

    Returns
    -------
    list of dict
        One dict per REAL assembly (every link closes), sorted by
        thetaO; at most 6 exist. Same keys as the direct kinematics,
        with 'thetaO' the crank angle (deg, in (-180, 180]) and
        'thetaB' the input. Each entry is also a solution of
        stephensonIII_direct_kinematics(geo, thetaO).
        If no solution exists, a single dict with valid = False.

    Method
    ------
    With thetaB given, F is fixed. For both assembly modes of the
    four-bar O-C-D-A (branches of D), E traces the coupler curve of
    body C-D-E as the crank turns; the solutions are the crank angles
    at which |E - F| = FE. They are bracketed by a sweep of thetaO in
    0.05 deg steps (to which the crank's exact limit angles are added)
    and refined by bisection.
    """
    geo = np.asarray(geo, dtype=float).ravel()[:12]
    OA, Bx, By, OC, CD, DA, BF, FE, DE, EP, eta, delta = geo

    O = np.array([0.0, 0.0])
    A = np.array([OA, 0.0])
    B = np.array([Bx, By])

    # F from thetaB (measured from the direction O->B)
    thB_global = np.arctan2(By, Bx) + np.deg2rad(thetaB)
    F = B + BF * np.array([np.cos(thB_global), np.sin(thB_global)])

    R_delta = _rot(np.deg2rad(delta))
    R_eta = _rot(np.deg2rad(eta))
    tolc = 1e-9 * max(1.0, abs(CD), abs(DA))

    def config(t, b):
        """C, D (branch b = 0 or 1) and E, as in the direct kinematics."""
        C = OC * np.array([np.cos(t), np.sin(t)])
        D = circle_intersections(C, CD, A, DA, tol=tolc)[b]
        if np.any(np.isnan(D)):
            return C, D, np.array([np.nan, np.nan]), False
        u = (D - C) / np.linalg.norm(D - C)
        return C, D, D + DE * (R_delta @ (-u)), True

    def residual(t, b):
        _, _, E, ok = config(t, b)
        return np.linalg.norm(E - F) - FE if ok else np.nan

    def sweep_residuals(t):
        """Vectorized residual for all angles t and both branches."""
        Cv = OC * np.vstack([np.cos(t), np.sin(t)])
        dv = A[:, None] - Cv                        # C -> A
        d = np.sqrt(np.sum(dv**2, axis=0))
        okv = (d > 0) & (d <= CD + DA + tolc) & (d >= abs(CD - DA) - tolc)
        with np.errstate(divide='ignore', invalid='ignore'):
            a = (CD**2 - DA**2 + d**2) / (2 * d)
            h = np.sqrt(np.maximum(0.0, CD**2 - a**2))
            u = dv / d
            p0 = Cv + u * a
            off = np.vstack([-u[1], u[0]]) * h
            res = np.full((2, t.size), np.nan)
            for bb in range(2):
                Dv = p0 + (1 - 2 * bb) * off         # bb=0: +offset
                ucd = (Dv - Cv) / np.sqrt(np.sum((Dv - Cv)**2, axis=0))
                Ev = Dv + DE * (R_delta @ (-ucd))
                r = np.sqrt(np.sum((Ev - F[:, None])**2, axis=0)) - FE
                r[~okv] = np.nan
                res[bb] = r
        return res

    # --- Sweep of the crank angle, plus the exact limit angles ---------
    # The four-bar O-C-D-A assembles when |CD-DA| <= |AC| <= CD+DA, with
    # |AC|^2 = OA^2 + OC^2 - 2*OA*OC*cos(thetaO).
    N = 7200
    th = np.linspace(-np.pi, np.pi, N + 1)[:-1]
    extra = []
    if OA != 0 and OC != 0:
        for L in (CD + DA, abs(CD - DA)):
            c = (OA**2 + OC**2 - L**2) / (2 * OA * OC)
            if abs(c) <= 1:
                extra += [np.arccos(c), -np.arccos(c)]
    th = np.unique(np.mod(np.concatenate([th, extra]) + np.pi,
                          2 * np.pi) - np.pi)
    th = np.append(th, th[0] + 2 * np.pi)           # close the loop

    # --- Bracket and refine the roots on each D branch ----------------
    res = sweep_residuals(th)
    roots = []                                       # (thetaO rad, branch)
    for b in range(2):
        f = res[b]
        fin = np.isfinite(f[:-1]) & np.isfinite(f[1:])
        idx = np.nonzero(fin & ((f[:-1] == 0) | (f[:-1] * f[1:] < 0)))[0]
        for i in idx:
            if f[i] == 0:
                roots.append((th[i], b))
            else:
                roots.append((_bisect(lambda x: residual(x, b),
                                      th[i], th[i + 1], f[i]), b))

    # --- Build the solutions ------------------------------------------
    tol = 1e-6 * max(1.0, np.max(np.abs(geo[:10])))
    sols = []
    for t, b in roots:
        C, D, E, ok = config(t, b)
        if not ok:
            continue
        # keep only configurations in which every link closes
        err = max(abs(np.linalg.norm(C - O) - abs(OC)),
                  abs(np.linalg.norm(D - C) - abs(CD)),
                  abs(np.linalg.norm(D - A) - abs(DA)),
                  abs(np.linalg.norm(E - F) - abs(FE)),
                  abs(np.linalg.norm(F - B) - abs(BF)))
        if err > tol:
            continue
        v_EF = F - E
        P = E + EP * (R_eta @ (v_EF / np.linalg.norm(v_EF)))
        sol = _make_solution(O, A, B, C, D, F, E, P,
                             thetaO=np.degrees(np.arctan2(C[1], C[0])),
                             thetaB=thetaB)
        # skip duplicates (same assembly found twice, e.g. at a crank limit)
        if any(np.linalg.norm(s['Positions']['C'] - C) < tol and
               np.linalg.norm(s['Positions']['D'] - D) < tol for s in sols):
            continue
        sols.append(sol)

    if not sols:
        return [_invalid_solution(thetaO=np.nan, thetaB=thetaB)]
    sols.sort(key=lambda s: s['thetaO'])
    return sols


def _bisect(fun, a, b, fa, n_iter=100):
    """Root of fun in [a, b] (sign change, fa = fun(a)) by bisection."""
    for _ in range(n_iter):
        m = 0.5 * (a + b)
        if m == a or m == b:
            break
        fm = fun(m)
        if fm == 0 or not np.isfinite(fm):
            return m
        if fa * fm < 0:
            b = m
        else:
            a, fa = m, fm
    return 0.5 * (a + b)


if __name__ == '__main__':
    geo = [40, 70, 30, 50, 20, 50, 30, 30, -30, 20, 30, 60]
    print([round(s['thetaO'], 3) for s in stephensonIII_inverse_kinematics(geo, 69)])
    geo6 = [40, 70, 30, 35.7, 35.5, 26.1, 51, 57.9, -56, 20, 30, 54.9]
    print(len(stephensonIII_inverse_kinematics(geo6, 127.07)), 'solutions')
