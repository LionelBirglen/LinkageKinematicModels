"""
stephensonIII_plot.py
Plot a planar Stephenson III six-bar linkage.

Python port of stephensonIII_plot.m; same graphical style as
fourbar_plot.py.

BY:
Prof. Lionel Birglen
Polytechnique Montreal, 2025
Contact: lionel.birglen@polymtl.ca
Code provided under GNU Affero General Public License v3.0
"""

import itertools
import numpy as np
import matplotlib.pyplot as plt

from stephensonIII_direct_kinematics import (
    stephensonIII_direct_kinematics, _invalid_solution)
from stephensonIII_inverse_kinematics import stephensonIII_inverse_kinematics

# Default per-solution colors: first two match fourbar_plot (red, blue)
DEFAULT_COLORS = np.array([[1.0, 0.0, 0.0],     # red
                           [0.0, 0.0, 1.0],     # blue
                           [0.0, 0.6, 0.0],     # green
                           [0.9, 0.5, 0.0],     # orange
                           [0.55, 0.0, 0.75],   # purple
                           [0.0, 0.6, 0.6]])    # teal


def stephensonIII_plot(geo, mode, inputs, opts=None, ax=None):
    """
    Plot a Stephenson III six-bar linkage.

    Parameters
    ----------
    geo : array-like, length 12
        [OA, Bx, By, OC, CD, DA, BF, FE, DE, EP, eta, delta]
        (lengths in consistent units, eta and delta in DEGREES).
        Ground pivots: O = (0,0), A = (OA,0), B = (Bx,By).
    mode : str
        'direct'  -> inputs = thetaO (deg), angle of the crank O->C
        'inverse' -> inputs = thetaB (deg), angle of B->F w.r.t. B->O
    inputs : float
    opts : dict, optional
        'solutions'   : list of solution indices (1-based) to draw
                        (default: all valid ones; indices that match no
                        solution, e.g. [0], draw no linkage)
        'show_labels' : bool (default True)
        'clear_axes'  : bool (default True)
        'limits'      : [xmin, xmax, ymin, ymax]
        'colors'      : Nx3 array, one row per solution index
        'track'       : solution tracking. If this key is present, the
                        solutions are returned in n_slots fixed slots and
                        reordered so that each one keeps the slot of the
                        configuration it continues from in opts['track']
                        (the sol returned by the previous call): the
                        assignment minimizes the total displacement of
                        C, D, E, F and P. A vanished solution leaves its
                        slot invalid; a new one takes the first free
                        slot. Pass None on the first call.
        'n_slots'     : number of slots with 'track' (default 6)
    ax : matplotlib Axes, optional

    Returns
    -------
    ax : matplotlib Axes
    sol : list of solution dicts (n_slots of them when 'track' is used)

    Drawing conventions (identical to fourbar_plot): one color per
    solution index; ternary bodies C-D-E and F-E-P as semi-transparent
    patches; binary links O-C, A-D and B-F as lines; revolute joints as
    white-filled black circles; P as a black "x"; ground symbols at O,
    A and B; joint letters offset by OC/20.
    """
    if opts is None:
        opts = {}

    lw_link  = 1.5
    lw_joint = 1.0
    lw_gs    = 1.5
    lw_gs_b  = 1.0
    ms_joint = 80
    ms_P     = 8
    fs_label = 10

    g = np.asarray(geo, dtype=float).ravel()
    if g.size != 12:
        raise ValueError('geo must have 12 elements: '
                         '[OA Bx By OC CD DA BF FE DE EP eta delta]')

    O = np.array([0.0, 0.0])
    A = np.array([g[0], 0.0])
    B = np.array([g[1], g[2]])
    gs_len = max(np.abs(np.r_[g[0], np.linalg.norm(B), g[3:10]])) * 0.1125
    dxl = abs(g[3]) / 20

    if ax is None:
        fig, ax = plt.subplots()
        ax.set_aspect('equal')
        ax.grid(True)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_title('Stephenson III Linkage')

    if opts.get('clear_axes', True):
        ax.cla()
        ax.set_aspect('equal')
        ax.grid(True)

    colors = np.asarray(opts.get('colors', DEFAULT_COLORS))

    mode = mode.lower()
    if mode in ('direct', 'd'):
        sol = stephensonIII_direct_kinematics(g, inputs)
    elif mode in ('inverse', 'i'):
        sol = stephensonIII_inverse_kinematics(g, inputs)
    else:
        raise ValueError("mode must be 'direct' or 'inverse'")

    if 'track' in opts:
        sol = _track(sol, opts['track'], opts.get('n_slots', 6))

    # Ground symbols (always drawn)
    for G in (O, A, B):
        _draw_ground_symbol(ax, G, lw_gs, lw_gs_b, gs_len)

    n_sol = len(sol)
    idx_to_plot = opts.get('solutions', list(range(1, n_sol + 1)))
    idx_to_plot = [i for i in idx_to_plot if 1 <= i <= n_sol]

    for ii in idx_to_plot:
        s = sol[ii - 1]
        if not s['valid']:
            continue
        p = s['Positions']
        C, D, E, F, P = p['C'], p['D'], p['E'], p['F'], p['P']
        if not np.all(np.isfinite(np.r_[C, D, E, F, P])):
            continue
        col = colors[(ii - 1) % len(colors)]

        # Ternary bodies C-D-E and F-E-P, semi-transparent
        ax.add_patch(plt.Polygon([C, D, E], closed=True, facecolor=col,
                                 edgecolor=col, alpha=0.5))
        ax.add_patch(plt.Polygon([F, E, P], closed=True, facecolor=col,
                                 edgecolor=col, alpha=0.5))

        # Binary links: input crank O-C, then A-D and B-F
        for P1, P2 in ((O, C), (A, D), (B, F)):
            ax.plot([P1[0], P2[0]], [P1[1], P2[1]], '-', color=col,
                    linewidth=lw_link)

        # Point P (black "x")
        ax.plot(P[0], P[1], 'kx', markersize=ms_P, markeredgewidth=lw_link)

        # Joints as white-filled circles
        J = np.array([O, A, B, C, D, F, E])
        ax.scatter(J[:, 0], J[:, 1], s=ms_joint, facecolors='white',
                   edgecolors='black', linewidths=lw_joint, zorder=5)

        if opts.get('show_labels', True):
            for pt, lbl in zip((O, A, B, C, D, F, E, P), 'OABCDFEP'):
                ax.text(pt[0] + dxl, pt[1] + dxl, lbl, fontsize=fs_label,
                        ha='left', va='bottom', color='k', zorder=6)

    ax.set_aspect('equal')
    lims = opts.get('limits')
    if lims is not None and len(lims) == 4:
        ax.set_xlim(lims[0], lims[1])
        ax.set_ylim(lims[2], lims[3])

    return ax, sol


def _track(sol, prev, n_slots):
    """
    Reorder the valid solutions of sol into n_slots slots so that each
    keeps the slot of the closest configuration in prev (previous frame).
    Optimal assignment by exhaustive search (at most 6! = 720 cases).
    """
    out = [_invalid_solution() for _ in range(n_slots)]
    cur = [s for s in sol if s['valid']][:n_slots]
    pv = [] if not prev else \
        [j for j, s in enumerate(prev[:n_slots]) if s['valid']]

    slot = [None] * len(cur)
    if cur and pv:
        n = max(len(cur), len(pv))
        cost = np.zeros((n, n))             # dummy rows/cols cost 0
        for i, s in enumerate(cur):
            for j, jp in enumerate(pv):
                cost[i, j] = _conf_dist(s, prev[jp])
        best, best_perm = np.inf, None
        rows = np.arange(n)
        for perm in itertools.permutations(range(n)):
            c = cost[rows, list(perm)].sum()
            if c < best:
                best, best_perm = c, perm
        for i in range(len(cur)):
            if best_perm[i] < len(pv):
                slot[i] = pv[best_perm[i]]   # continues previous solution
    free = [k for k in range(n_slots) if k not in slot]
    for i in range(len(cur)):                # new solutions: first free slots
        if slot[i] is None:
            slot[i] = free.pop(0)
    for i, s in enumerate(cur):
        out[slot[i]] = s
    return out


def _conf_dist(s1, s2):
    """Total displacement of the moving joints between two configurations."""
    return sum(np.linalg.norm(s1['Positions'][k] - s2['Positions'][k])
               for k in 'CDEFP')


def _draw_ground_symbol(ax, jc, lw_hatch, lw_base, line_len):
    """Identical to the one in fourbar_plot.py."""
    jc = np.asarray(jc, dtype=float).ravel()
    num_lines = 3
    line_spacing = line_len / 2
    angle = np.pi + np.pi / 4
    R = np.array([[np.cos(angle), -np.sin(angle)],
                  [np.sin(angle),  np.cos(angle)]])
    Rx = R @ np.array([line_len, 0.0])
    for i in range(num_lines):
        v1 = jc + np.array([i * line_spacing - (num_lines - 1) / 2 * line_spacing, 0.0])
        v2 = v1 + Rx
        ax.plot([v1[0], v2[0]], [v1[1], v2[1]], 'k-', linewidth=lw_hatch)
    xs = jc[0] + np.array([-(num_lines - 1) / 2 * line_spacing,
                           (num_lines - 1) / 2 * line_spacing])
    ax.plot(xs, [jc[1], jc[1]], 'k-', linewidth=lw_base)


if __name__ == '__main__':
    g = [40, 70, 30, 50, 20, 50, 30, 30, -30, 20, 30, 60]
    fig, ax = plt.subplots()
    stephensonIII_plot(g, 'direct', 90, ax=ax)
    plt.show()
