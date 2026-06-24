"""
fivebar_plot.py
Plot a planar five-bar linkage.

BY:
Prof. Lionel Birglen
Polytechnique Montreal, 2025
Contact: lionel.birglen@polymtl.ca
Code provided under GNU Affero General Public License v3.0
"""

import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon

from fivebar_direct_kinematics import fivebar_direct_kinematics
from fivebar_inverse_kinematics import fivebar_inverse_kinematics


def fivebar_plot(geo, mode, inputs, opts=None, ax=None):
    """
    Plot a planar five-bar linkage.

    Parameters
    ----------
    geo : array-like, length 8
        [a, b, c, d, e, alpha, h, eta]
    mode : str
        'direct'  -> inputs = [theta1_deg, theta2_deg]
        'inverse' -> inputs = [Px, Py]
    inputs : array-like, length 2
    opts : dict, optional
        Keys: 'solutions' (list of 1-based indices to draw),
              'show_labels' (bool), 'clear_axes' (bool),
              'limits' ([xmin,xmax,ymin,ymax]), 'colors' (Nx3 array)
    ax : matplotlib Axes, optional

    Returns
    -------
    ax : matplotlib Axes
    sols : list of solution dicts
    """
    if opts is None:
        opts = {}

    lw_link  = 1.5
    lw_joint = 1.0
    ms_joint = 60
    ms_ee    = 8

    geo = np.asarray(geo, dtype=float)

    if ax is None:
        fig, ax = plt.subplots()
        ax.set_aspect('equal')
        ax.grid(True)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_title('Five-Bar Linkage')

    if opts.get('clear_axes', True):
        ax.cla()
        ax.set_aspect('equal')
        ax.grid(True)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_title('Five-Bar Linkage')

    mode = mode.lower()
    inputs = list(inputs)

    if mode in ('direct', 'd'):
        sols = fivebar_direct_kinematics(geo, inputs)
    elif mode in ('inverse', 'i'):
        sols = fivebar_inverse_kinematics(geo, inputs)
    else:
        raise ValueError("mode must be 'direct' or 'inverse'")

    if not sols:
        lims = opts.get('limits')
        if lims:
            ax.set_xlim(lims[0], lims[1])
            ax.set_ylim(lims[2], lims[3])
        return ax, sols

    # Which solutions to draw
    idx_to_plot = opts.get('solutions', list(range(1, len(sols) + 1)))
    idx_to_plot = [i for i in idx_to_plot if 1 <= i <= len(sols)]
    if not idx_to_plot:
        idx_to_plot = list(range(1, len(sols) + 1))

    colors_default = plt.cm.tab10(np.linspace(0, 0.4, 4))
    colors = opts.get('colors', colors_default)

    for ii in idx_to_plot:
        s = sols[ii - 1]
        if not s['valid']:
            continue

        O = s['Positions']['O']
        A = s['Positions']['A']
        B = s['Positions']['B']
        C = s['Positions']['C']
        D = s['Positions']['D']
        P = s['P']

        col = colors[(ii - 1) % len(colors)]

        # Linkage chain O→A→B→C→D→O
        xs = [O[0], A[0], B[0], C[0], D[0], O[0]]
        ys = [O[1], A[1], B[1], C[1], D[1], O[1]]
        ax.plot(xs, ys, '-', color=col, linewidth=lw_link)

        # Coupler triangle A-B-P
        tri = Polygon([A, B, P], closed=True,
                      facecolor=col, edgecolor=col, alpha=0.4)
        ax.add_patch(tri)

        # P marker (cross)
        ax.plot(P[0], P[1], 'kx', markersize=ms_ee, markeredgewidth=lw_link)

        # Joint circles O, A, B, C, D
        jx = [O[0], A[0], B[0], C[0], D[0]]
        jy = [O[1], A[1], B[1], C[1], D[1]]
        ax.scatter(jx, jy, s=ms_joint,
                   facecolors='white', edgecolors='black',
                   linewidths=lw_joint, zorder=5)

        # Labels
        if opts.get('show_labels', True):
            pts  = [O, A, B, C, D, P]
            lbls = ['O', 'A', 'B', 'C', 'D', 'P']
            dx = np.linalg.norm(A - O) / 20
            for pt, lbl in zip(pts, lbls):
                ax.text(pt[0] + dx, pt[1] + dx, lbl,
                        fontsize=10, ha='left', va='bottom', color='k')

    ax.set_aspect('equal')
    lims = opts.get('limits')
    if lims and len(lims) == 4:
        ax.set_xlim(lims[0], lims[1])
        ax.set_ylim(lims[2], lims[3])

    return ax, sols


if __name__ == '__main__':
    geo = [0.6, 0.7, 0.9, 0.6, 1.0, 0, 0.5, 45]
    fig, ax = plt.subplots()
    fivebar_plot(geo, 'direct', [120, 75], ax=ax)
    plt.show()
