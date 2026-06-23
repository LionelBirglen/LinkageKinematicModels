"""
rrr_plot.py
Plot a planar RRR serial linkage.

BY:
Prof. Lionel Birglen
Polytechnique Montreal, 2025
Contact: lionel.birglen@polymtl.ca
Code provided under GNU Affero General Public License v3.0
"""

import numpy as np
import matplotlib.pyplot as plt

from rrr_direct_kinematics import rrr_direct_kinematics
from rrr_inverse_kinematics import rrr_inverse_kinematics


def rrr_plot(geo, mode, inputs, opts=None, ax=None):
    """
    Plot a planar RRR serial linkage.

    Parameters
    ----------
    L1, L2, L3 : float
        Link lengths
    mode : str
        'direct'  -> inputs = [theta1, theta2, theta3] (rad)
        'inverse' -> inputs = [Px, Py, phi, elbow_config]
    inputs : array-like
    opts : dict, optional
        Keys: 'show_labels', 'clear_axes', 'limits',
              'show_both', 'colors', 'line_style', 'label_subset'
    ax : matplotlib Axes, optional

    Returns
    -------
    ax : matplotlib Axes
    """
    if opts is None:
        opts = {}
    # Unpack geometry
    if isinstance(geo, dict):
        L1, L2, L3 = geo['L1'], geo['L2'], geo['L3']
    else:
        L1, L2, L3 = float(geo[0]), float(geo[1]), float(geo[2])

    lw_link  = 1.5
    lw_joint = 1.0
    lw_gs_h  = 1.5
    lw_gs_b  = 1.0
    ms_joint = 80
    ms_ee    = 8
    fs_label = 10

    gs_len = max(L1, L2, L3) * 0.1125

    if ax is None:
        fig, ax = plt.subplots()
        ax.set_aspect('equal')
        ax.grid(True)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_title('Planar RRR Linkage')

    if opts.get('clear_axes', True):
        ax.cla()
        ax.set_aspect('equal')
        ax.grid(True)

    colors = opts.get('colors', np.array([[1,0,0],[0,0.7,0],[0,0,1]]))
    colors = np.asarray(colors)
    ls = opts.get('line_style', '-')

    mode = mode.lower()
    inputs = list(inputs)

    if mode in ('direct', 'd'):
        if len(inputs) < 3:
            raise ValueError("Direct mode needs [theta1, theta2, theta3]")
        sol = rrr_direct_kinematics(L1, L2, L3, inputs[0], inputs[1], inputs[2])
        if sol['valid']:
            _draw(ax, sol, colors, ls, lw_link, lw_joint,
                  lw_gs_h, lw_gs_b, ms_joint, ms_ee, fs_label, gs_len, opts)

    elif mode in ('inverse', 'i'):
        if len(inputs) < 4:
            raise ValueError("Inverse mode needs [Px, Py, phi, elbow_config]")
        Px, Py, phi, cfg = inputs[0], inputs[1], inputs[2], inputs[3]
        sol = rrr_inverse_kinematics(L1, L2, L3, Px, Py, phi, cfg)
        if sol['valid']:
            _draw(ax, sol, colors, ls, lw_link, lw_joint,
                  lw_gs_h, lw_gs_b, ms_joint, ms_ee, fs_label, gs_len, opts)
        else:
            ax.text(0, 0, 'Unreachable', color='r', fontsize=14,
                    ha='center', va='center')

        if opts.get('show_both', False):
            sol2 = rrr_inverse_kinematics(L1, L2, L3, Px, Py, phi, -cfg)
            if sol2['valid']:
                opts2 = dict(opts)
                opts2['clear_axes']   = False
                opts2['line_style']   = '--'
                opts2['show_labels']  = True
                opts2['label_subset'] = [1]  # only A (0-indexed: index 1)
                _draw(ax, sol2, colors, '--', lw_link, lw_joint,
                      lw_gs_h, lw_gs_b, ms_joint, ms_ee, fs_label, gs_len, opts2)
    else:
        raise ValueError("mode must be 'direct' or 'inverse'")

    ax.set_aspect('equal')
    lims = opts.get('limits')
    if lims and len(lims) == 4:
        ax.set_xlim(lims[0], lims[1])
        ax.set_ylim(lims[2], lims[3])

    return ax


def _draw(ax, sol, colors, ls, lw_link, lw_joint,
          lw_gs_h, lw_gs_b, ms_joint, ms_ee, fs_label, gs_len, opts):
    O = sol['Positions']['O']
    A = sol['Positions']['A']
    B = sol['Positions']['B']
    P = sol['Positions']['P']

    _draw_ground_symbol(ax, O, lw_gs_h, lw_gs_b, gs_len)

    segs = [(O, A), (A, B), (B, P)]
    for k, (p1, p2) in enumerate(segs):
        col = colors[k % len(colors)]
        ax.plot([p1[0], p2[0]], [p1[1], p2[1]],
                color=col, linestyle=ls, linewidth=lw_link)

    ax.plot(P[0], P[1], 'kx', markersize=ms_ee, markeredgewidth=lw_link)

    jx = [O[0], A[0], B[0]]
    jy = [O[1], A[1], B[1]]
    ax.scatter(jx, jy, s=ms_joint, facecolors='white',
               edgecolors='black', linewidths=lw_joint, zorder=5)

    if opts.get('show_labels', True):
        lbls = ['O', 'A', 'B', 'P']
        pts  = [O, A, B, P]
        subset = opts.get('label_subset', list(range(4)))
        dx = gs_len * 0.6
        for k in subset:
            pt = pts[k]
            ax.text(pt[0] + dx, pt[1] + dx, lbls[k],
                    fontsize=fs_label, ha='left', va='bottom', color='k')


def _draw_ground_symbol(ax, jc, lw_hatch, lw_base, line_len):
    jc = np.asarray(jc, dtype=float).ravel()
    num_lines    = 3
    line_spacing = line_len / 2
    angle = np.pi + np.pi / 4
    R = np.array([[np.cos(angle), -np.sin(angle)],
                  [np.sin(angle),  np.cos(angle)]])
    Rx = R @ np.array([line_len, 0.0])
    for i in range(num_lines):
        v1 = jc + np.array([i * line_spacing - (num_lines-1)/2 * line_spacing, 0.0])
        v2 = v1 + Rx
        ax.plot([v1[0], v2[0]], [v1[1], v2[1]], 'k-', linewidth=lw_hatch)
    xs = jc[0] + np.array([-(num_lines-1)/2*line_spacing, (num_lines-1)/2*line_spacing])
    ax.plot(xs, [jc[1], jc[1]], 'k-', linewidth=lw_base)


if __name__ == '__main__':
    fig, ax = plt.subplots()
    rrr_plot(57, 46, 51, 'direct', np.deg2rad([39, 37, 40]), ax=ax)
    plt.show()
