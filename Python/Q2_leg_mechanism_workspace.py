"""
Q2_leg_mechanism_workspace.py
Workspace of point P of the Q2 leg mechanism.

Python port of Q2_leg_mechanism_workspace.m.

BY:
Prof. Lionel Birglen
Polytechnique Montreal, 2025
Contact: lionel.birglen@polymtl.ca
Code provided under GNU Affero General Public License v3.0
"""

import time
import numpy as np

from Q2_leg_mechanism_direct_kinematics import Q2_leg_mechanism_direct_kinematics


def Q2_leg_mechanism_workspace(params, thetaA_lim, rho_lim, n=(91, 51)):
    """
    Set of positions of point P reachable for
        thetaA_lim[0] <= thetaA <= thetaA_lim[1]   (rad)
        rho_lim[0]    <= rho    <= rho_lim[1]
    for each of the 8 assembly branches of Q2_leg_mechanism_direct_kinematics.

    The input box is sampled on a regular n[0] x n[1] grid and the direct
    kinematics is evaluated at every node, so the result is exactly
    consistent with the direct kinematics used for drawing. Branch k of
    the result is branch k of the direct kinematics.

    Parameters
    ----------
    params : dict
        The 17-parameter dict of Q2_leg_mechanism_direct_kinematics
        (IP = 0 gives the workspace of I).
    thetaA_lim, rho_lim : 2-element sequences
    n : int or 2-element sequence, grid size (default (91, 51))

    Returns
    -------
    dict with keys
        'thetaA_lim', 'rho_lim', 'n' : the limits and grid size used
        'thetaA' : (n[0],) sampled thetaA values (rad)
        'rho'    : (n[1],) sampled rho values
        'X', 'Y' : (n[0], n[1], 8) coordinates of P (NaN where the branch
                   does not assemble)
        'valid'  : (n[0], n[1], 8) bool, branch assembles at the node
        'patch'  : list of 8 dicts with 'Vertices' (M x 2) and 'Faces'
                   (nF x 4, 0-based row indices into Vertices): every grid
                   cell whose 4 corners are valid, mapped to the plane.
                   Ready for a matplotlib PolyCollection
                   (Vertices[Faces]).
        'bbox'   : list of 8 [xmin, xmax, ymin, ymax] (None if empty)
        'nValid' : (8,) number of valid grid nodes per branch
        'params' : copy of params
        'time'   : computation time (s)
    """
    thetaA_lim = np.asarray(thetaA_lim, dtype=float).ravel()
    rho_lim = np.asarray(rho_lim, dtype=float).ravel()
    if thetaA_lim.size != 2 or rho_lim.size != 2 or \
            not np.all(np.isfinite(np.r_[thetaA_lim, rho_lim])):
        raise ValueError('thetaA_lim and rho_lim must be finite 2-element vectors.')
    n = np.atleast_1d(np.round(np.asarray(n, dtype=float))).astype(int)
    if n.size == 1:
        n = np.r_[n, n]
    n = np.maximum(n[:2], 2)

    t0 = time.time()
    thetaA = np.linspace(thetaA_lim.min(), thetaA_lim.max(), n[0])
    rho = np.linspace(rho_lim.min(), rho_lim.max(), n[1])

    nb = 8
    X = np.full((n[0], n[1], nb), np.nan)
    Y = np.full((n[0], n[1], nb), np.nan)
    V = np.zeros((n[0], n[1], nb), dtype=bool)

    for i in range(n[0]):
        for j in range(n[1]):
            s = Q2_leg_mechanism_direct_kinematics(thetaA[i], rho[j], params)
            for k in range(min(nb, len(s))):
                if s[k]['valid'] and np.all(np.isfinite(s[k]['P'])):
                    X[i, j, k], Y[i, j, k] = s[k]['P']
                    V[i, j, k] = True

    # Patch data: grid cells with 4 valid corners, per branch.
    # Node (i, j) has row index i + j*n[0] in Vertices (column-major, as
    # in the MATLAB version).
    I, J = np.meshgrid(np.arange(n[0] - 1), np.arange(n[1] - 1), indexing='ij')
    I, J = I.ravel(order='F'), J.ravel(order='F')
    c1 = I + J * n[0]              # (i  , j  )
    c2 = I + 1 + J * n[0]          # (i+1, j  )
    c3 = I + 1 + (J + 1) * n[0]    # (i+1, j+1)
    c4 = I + (J + 1) * n[0]        # (i  , j+1)

    patch, bbox, nValid = [], [], np.zeros(nb, dtype=int)
    for k in range(nb):
        Vk = V[:, :, k].ravel(order='F')
        nValid[k] = Vk.sum()
        if nValid[k] == 0:
            patch.append({'Vertices': np.zeros((0, 2)),
                          'Faces': np.zeros((0, 4), dtype=int)})
            bbox.append(None)
            continue
        Xk = X[:, :, k].ravel(order='F')
        Yk = Y[:, :, k].ravel(order='F')
        ok = Vk[c1] & Vk[c2] & Vk[c3] & Vk[c4]
        patch.append({'Vertices': np.column_stack([Xk, Yk]),
                      'Faces': np.column_stack([c1[ok], c2[ok], c3[ok], c4[ok]])})
        bbox.append([Xk[Vk].min(), Xk[Vk].max(), Yk[Vk].min(), Yk[Vk].max()])

    return {'thetaA_lim': [thetaA_lim.min(), thetaA_lim.max()],
            'rho_lim': [rho_lim.min(), rho_lim.max()],
            'n': n, 'thetaA': thetaA, 'rho': rho,
            'X': X, 'Y': Y, 'valid': V, 'patch': patch, 'bbox': bbox,
            'nValid': nValid, 'params': dict(params),
            'time': time.time() - t0}


if __name__ == '__main__':
    p = dict(AB=150, AC=90, BD=120, CD=100, FG=80, CF=90, BE=50, GK=50,
             GJ=80, IF=80, IJ=100, IP=50, DCF=np.deg2rad(80),
             GFC=np.deg2rad(110), DBE=np.deg2rad(45), JGK=np.deg2rad(30),
             JIP=-np.deg2rad(30))
    W = Q2_leg_mechanism_workspace(p, np.deg2rad([-180, 180]), [50, 400])
    print('valid nodes per branch:', W['nValid'], '| time %.1f s' % W['time'])
