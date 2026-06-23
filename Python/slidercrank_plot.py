"""
slidercrank_plot.py
Plot a planar slider-crank mechanism.

BY:
Prof. Lionel Birglen
Polytechnique Montreal, 2025
Contact: lionel.birglen@polymtl.ca
Code provided under GNU Affero General Public License v3.0
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon

from slidercrank_direct_kinematics import slidercrank_direct_kinematics
from slidercrank_inverse_kinematics import slidercrank_inverse_kinematics


def slidercrank_plot(geo, mode, inputs, opts=None, ax=None):
    """
    Plot a planar slider-crank mechanism.

    Parameters
    ----------
    a, b, c : float
        Crank, coupler, extension link lengths
    mode : str
        'direct'  -> inputs = [phi, slider_angle, config]
        'inverse' -> inputs = [x_slider, slider_angle, config]
    inputs : array-like, length 3
    opts : dict, optional
        Keys: 'show_labels', 'clear_axes', 'limits', 'show_both',
              'show_first', 'line_style', 'label_subset'
    ax : matplotlib Axes, optional

    Returns
    -------
    ax : matplotlib Axes
    """
    if opts is None:
        opts = {}
    # Unpack geometry
    if isinstance(geo, dict):
        a, b, c = geo['a'], geo['b'], geo['c']
        default_sang = geo.get('slider_angle', 0.0)
    else:
        g = list(geo); a, b, c = g[0], g[1], g[2]
        default_sang = g[3] if len(g) > 3 else 0.0

    lw_link  = 1.5
    lw_joint = 1.0
    lw_gs_h  = 1.5
    lw_gs_b  = 1.0
    lw_rail  = 0.8
    ms_joint = 80
    ms_ee    = 8
    fs_label = 10

    gs_len = max(a, b) * 0.1125

    if ax is None:
        fig, ax = plt.subplots()
        ax.set_aspect('equal')
        ax.grid(True)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_title('Slider-Crank Linkage')

    if opts.get('clear_axes', True):
        ax.cla()
        ax.set_aspect('equal')
        ax.grid(True)

    mode = mode.lower()
    inputs = list(inputs)
    if len(inputs) < 2:
        raise ValueError("inputs needs [value, config]")
    config       = inputs[1]
    slider_angle = default_sang
    ls = opts.get('line_style', '-')

    if mode in ('direct', 'd'):
        phi = inputs[0]
        sol = slidercrank_direct_kinematics(a, b, c, phi, slider_angle, config)
    elif mode in ('inverse', 'i'):
        x_s = inputs[0]
        sol = slidercrank_inverse_kinematics(a, b, c, x_s, config, slider_angle)
    else:
        raise ValueError("mode must be 'direct' or 'inverse'")

    slider_dir = np.array([np.cos(slider_angle), np.sin(slider_angle)])

    # Fixed rail
    R_lim  = a + b
    rail_s = -R_lim * slider_dir
    rail_e =  R_lim * slider_dir
    ax.plot([rail_s[0], rail_e[0]], [rail_s[1], rail_e[1]],
            'k-.', linewidth=lw_rail)

    # Fixed slider block at x = a
    blk_centre = a * slider_dir
    _draw_slider_block(ax, blk_centre, slider_dir, slider_angle, lw_gs_h, gs_len)

    # Ground symbol and fixed O label
    _draw_ground_symbol(ax, np.zeros(2), lw_gs_h, lw_gs_b, gs_len)
    if opts.get('show_labels', True):
        dx = gs_len * 0.6
        ax.text(dx, dx, 'O', fontsize=fs_label, ha='left', va='bottom', color='k')

    # Primary solution
    show_first = opts.get('show_first', True)
    if show_first:
        if sol['valid']:
            _draw(ax, sol, [(1,0,0), (0,0.7,0)], ls, lw_link, lw_joint,
                  ms_joint, ms_ee, fs_label, gs_len, opts, is_alt=False)
        else:
            ax.text(0, 0, 'Sol 1: Unreachable', color='r', fontsize=14,
                    ha='center', va='center')

    # Alternate solution
    if opts.get('show_both', False):
        if mode in ('direct', 'd'):
            sol2 = slidercrank_direct_kinematics(a, b, c, phi, slider_angle, -config)
        else:
            sol2 = slidercrank_inverse_kinematics(a, b, c, x_s, -config, slider_angle)
        if sol2['valid']:
            opts2 = dict(opts)
            opts2['clear_axes']   = False
            opts2['line_style']   = '--'
            opts2['label_subset'] = [2, 3]  # B and P (0-indexed)
            _draw(ax, sol2, [(1,0,0), (0,0.7,0)], '--', lw_link, lw_joint,
                  ms_joint, ms_ee, fs_label, gs_len, opts2, is_alt=True)

    ax.set_aspect('equal')
    lims = opts.get('limits')
    if lims and len(lims) == 4:
        ax.set_xlim(lims[0], lims[1])
        ax.set_ylim(lims[2], lims[3])

    return ax


def _draw(ax, sol, colors, ls, lw_link, lw_joint,
          ms_joint, ms_ee, fs_label, gs_len, opts, is_alt):
    O = sol['Positions']['O']
    A = sol['Positions']['A']
    B = sol['Positions']['B']
    P = sol['Positions']['P']

    ax.plot([O[0], A[0]], [O[1], A[1]],
            color=colors[0], linestyle=ls, linewidth=lw_link)
    ax.plot([A[0], B[0]], [A[1], B[1]],
            color=colors[1], linestyle=ls, linewidth=lw_link)
    ax.plot([B[0], P[0]], [B[1], P[1]],
            color=(0, 0, 1), linestyle=ls, linewidth=lw_link)

    ax.plot(P[0], P[1], 'kx', markersize=ms_ee, markeredgewidth=lw_link)

    ax.scatter([O[0], A[0], B[0]], [O[1], A[1], B[1]],
               s=ms_joint, facecolors='white', edgecolors='black',
               linewidths=lw_joint, zorder=5)

    if opts.get('show_labels', True):
        lbls   = ['O', 'A', 'B', 'P']
        pts    = [O, A, B, P]
        subset = opts.get('label_subset', [1, 2, 3])  # default: A, B, P (0-indexed)
        dx = gs_len * 0.6
        for k in subset:
            pt = pts[k]
            ax.text(pt[0] + dx, pt[1] + dx, lbls[k],
                    fontsize=fs_label, ha='left', va='bottom', color='k')


def _draw_slider_block(ax, centre, slider_dir, slider_angle, lw, gs_len):
    perp_dir = np.array([-np.sin(slider_angle), np.cos(slider_angle)])
    sl = gs_len * 3.0
    sw = gs_len * 1.8
    corners = np.array([
        centre + sl/2*slider_dir + sw/2*perp_dir,
        centre + sl/2*slider_dir - sw/2*perp_dir,
        centre - sl/2*slider_dir - sw/2*perp_dir,
        centre - sl/2*slider_dir + sw/2*perp_dir,
    ])
    poly = Polygon(corners, closed=True,
                   facecolor=(0.75, 0.85, 1.0), edgecolor='k',
                   alpha=0.6, linewidth=lw)
    ax.add_patch(poly)


def _draw_ground_symbol(ax, jc, lw_hatch, lw_base, line_len):
    jc = np.asarray(jc, dtype=float).ravel()
    num_lines    = 3
    line_spacing = line_len / 2
    angle = np.pi + np.pi / 4
    R = np.array([[np.cos(angle), -np.sin(angle)],
                  [np.sin(angle),  np.cos(angle)]])
    Rx = R @ np.array([line_len, 0.0])
    for i in range(num_lines):
        v1 = jc + np.array([i*line_spacing - (num_lines-1)/2*line_spacing, 0.0])
        v2 = v1 + Rx
        ax.plot([v1[0], v2[0]], [v1[1], v2[1]], 'k-', linewidth=lw_hatch)
    xs = jc[0] + np.array([-(num_lines-1)/2*line_spacing, (num_lines-1)/2*line_spacing])
    ax.plot(xs, [jc[1], jc[1]], 'k-', linewidth=lw_base)


if __name__ == '__main__':
    fig, ax = plt.subplots()
    slidercrank_plot({'a':50,'b':120,'c':30,'slider_angle':0}, 'direct', [np.deg2rad(30), 1], ax=ax)
    plt.show()
