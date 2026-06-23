"""
fourbar_plot.py
Plot a planar four-bar linkage.

BY:
Prof. Lionel Birglen
Polytechnique Montreal, 2025
Contact: lionel.birglen@polymtl.ca
Code provided under GNU Affero General Public License v3.0
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import Polygon
from matplotlib.collections import PatchCollection

from fourbar_direct_kinematics import fourbar_direct_kinematics
from fourbar_inverse_kinematics import fourbar_inverse_kinematics


def fourbar_plot(geo, mode, inputs, opts=None, ax=None):
    """
    Plot a four-bar linkage.

    Parameters
    ----------
    geo : array-like, length 7
        [a, b, c, d, e, epsilon, delta]
    mode : str
        'direct' (input = theta) or 'inverse' (input = alpha)
    inputs : float
        Crank angle theta (direct) or output link angle alpha (inverse) in rad
    opts : dict, optional
        Keys: 'solutions' (list of 1/2), 'show_labels' (bool),
              'clear_axes' (bool), 'limits' ([xmin,xmax,ymin,ymax]),
              'colors' (Nx3 array)
    ax : matplotlib Axes, optional

    Returns
    -------
    ax : matplotlib Axes
    sol : list of dicts from kinematics function
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

    geo = np.asarray(geo, dtype=float)
    gs_len = max(geo[:4]) * 0.1125

    if ax is None:
        fig, ax = plt.subplots()
        ax.set_aspect('equal')
        ax.grid(True)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_title('Four-Bar Linkage')

    if opts.get('clear_axes', True):
        ax.cla()
        ax.set_aspect('equal')
        ax.grid(True)

    colors = opts.get('colors', np.array([[1,0,0],[0,0,1]]))
    colors = np.asarray(colors)

    mode = mode.lower()
    if mode in ('direct', 'd'):
        sol = fourbar_direct_kinematics(geo, inputs)
    elif mode in ('inverse', 'i'):
        sol = fourbar_inverse_kinematics(geo, inputs)
    else:
        raise ValueError("mode must be 'direct' or 'inverse'")

    n_sol = len(sol)
    idx_to_plot = opts.get('solutions', list(range(1, n_sol + 1)))
    idx_to_plot = [i for i in idx_to_plot if 1 <= i <= n_sol]
    if not idx_to_plot:
        idx_to_plot = list(range(1, n_sol + 1))

    for ii in idx_to_plot:
        s = sol[ii - 1]
        if not s['valid']:
            continue

        O = s['Positions']['O']
        A = s['Positions']['A']
        B = s['Positions']['B']
        C = s['Positions']['C']
        P = s['P']

        col = colors[(ii - 1) % len(colors)]

        _draw_ground_symbol(ax, O, lw_gs, lw_gs_b, gs_len)
        _draw_ground_symbol(ax, C, lw_gs, lw_gs_b, gs_len)

        ax.plot([O[0], A[0], B[0], C[0]],
                [O[1], A[1], B[1], C[1]],
                '-', color=col, linewidth=lw_link)

        ax.plot(P[0], P[1], 'kx', markersize=ms_P, markeredgewidth=lw_link)

        tri = plt.Polygon([A, B, P], color=col, alpha=0.5)
        ax.add_patch(tri)

        jx = [O[0], A[0], B[0], C[0]]
        jy = [O[1], A[1], B[1], C[1]]
        ax.scatter(jx, jy, s=ms_joint, facecolors='white',
                   edgecolors='black', linewidths=lw_joint, zorder=5)

        if opts.get('show_labels', True):
            _label_joints(ax, s, gs_len, fs_label)

    ax.set_aspect('equal')
    lims = opts.get('limits')
    if lims and len(lims) == 4:
        ax.set_xlim(lims[0], lims[1])
        ax.set_ylim(lims[2], lims[3])

    return ax, sol


def _label_joints(ax, s, gs_len, fs):
    O = s['Positions']['O']
    A = s['Positions']['A']
    B = s['Positions']['B']
    C = s['Positions']['C']
    P = s['P']
    dx = np.linalg.norm(A - O) / 20
    dy = dx
    for pt, lbl in zip([O, A, B, C, P], ['O', 'A', 'B', 'C', 'P']):
        ax.text(pt[0] + dx, pt[1] + dy, lbl, fontsize=fs,
                ha='left', va='bottom', color='k')


def _draw_ground_symbol(ax, jc, lw_hatch, lw_base, line_len):
    jc = np.asarray(jc, dtype=float).ravel()
    num_lines   = 3
    line_spacing = line_len / 2
    angle = np.pi + np.pi / 4
    R = np.array([[np.cos(angle), -np.sin(angle)],
                  [np.sin(angle),  np.cos(angle)]])
    Rx = R @ np.array([line_len, 0.0])
    for i in range(num_lines):
        v1 = jc + np.array([i * line_spacing - (num_lines - 1) / 2 * line_spacing, 0.0])
        v2 = v1 + Rx
        ax.plot([v1[0], v2[0]], [v1[1], v2[1]], 'k-', linewidth=lw_hatch)
    xs = jc[0] + np.array([-(num_lines-1)/2*line_spacing, (num_lines-1)/2*line_spacing])
    ax.plot(xs, [jc[1], jc[1]], 'k-', linewidth=lw_base)


if __name__ == '__main__':
    g = np.array([81, 88, 92, 151, 10, np.pi/6, 0], dtype=float)
    fig, ax = plt.subplots()
    fourbar_plot(g, 'direct', np.deg2rad(106), ax=ax)
    plt.show()
