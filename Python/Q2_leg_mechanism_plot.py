"""
Q2_leg_mechanism_plot.py
Plot the planar Q2 leg mechanism.

Python port of Q2_leg_mechanism_plot.m; same graphical style as
fourbar_plot.py.

BY:
Prof. Lionel Birglen
Polytechnique Montreal, 2025
Contact: lionel.birglen@polymtl.ca
Code provided under GNU Affero General Public License v3.0
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection

from Q2_leg_mechanism_direct_kinematics import (
    Q2_leg_mechanism_direct_kinematics, _check_params)

# Default per-solution colors: first two match fourbar_plot (red, blue)
DEFAULT_COLORS = np.array([[1.0, 0.0, 0.0],     # red
                           [0.0, 0.0, 1.0],     # blue
                           [0.0, 0.6, 0.0],     # green
                           [0.9, 0.5, 0.0],     # orange
                           [0.55, 0.0, 0.75],   # purple
                           [0.0, 0.6, 0.6],     # teal
                           [0.45, 0.45, 0.45],  # grey
                           [0.8, 0.0, 0.45]])   # crimson

LBL_ACBD = 'M'      # intersection of lines AC and BD
LBL_IFGJ = 'N'      # intersection of lines IF and GJ


def Q2_leg_mechanism_plot(params, mode, inputs, opts=None, ax=None):
    """
    Plot the Q2 leg mechanism.

    Parameters
    ----------
    params : dict
        The 17 parameters of Q2_leg_mechanism_direct_kinematics (lengths
        in mm, angles in rad). Ground pivots A = (0,0) and B = (AB,0).
    mode : str
        'direct' -> inputs = [thetaA (rad), rho (mm)]
        ('inverse' is not available yet: the inverse kinematics of the Q2
        mechanism is future work.)
    inputs : 2-element sequence
    opts : dict, optional
        'solutions'          : list of solution indices (1-based) to draw
                               (default: all valid ones; indices that match
                               no solution, e.g. [0], draw no linkage)
        'show_labels'        : bool (default True)
        'clear_axes'         : bool (default True)
        'limits'             : [xmin, xmax, ymin, ymax]
        'colors'             : Nx3 array, one row per solution index
        'show_construction'  : bool (default False). Draws the line
                               intersections M = (AC) x (BD) and
                               N = (IF) x (GJ) as black "x" marks, with fine
                               lines through A-C-M, B-D-M, I-F-N and G-J-N.
                               If a pair of lines is parallel, that point
                               and its lines are not drawn.
        'construction_color' : RGB of the construction lines (default black)
        'workspace'          : dict from Q2_leg_mechanism_workspace. If given,
                               the reachable region of P is drawn under the
                               mechanism, one area per branch in that
                               branch's color.
        'workspace_solutions': branches whose workspace is drawn
                               (default: 'solutions' if given, else all 8)
        'workspace_style'    : 'fill' (default), 'points' or 'both'
                               ('both': filled cells, plus dots only for the
                               isolated positions not covered by any cell)
        'workspace_alpha'    : lightness of the areas, 0..1 (default 0.15):
                               the branch color blended with the axes
                               background. Areas are drawn opaque in that
                               light color, so overlapping cells keep one
                               uniform color.
    ax : matplotlib Axes, optional

    Returns
    -------
    ax : matplotlib Axes
    sol : list of 8 solution dicts from Q2_leg_mechanism_direct_kinematics
    kin : dict with the inputs used: 'thetaA', 'rho', 'ik_error' (NaN),
          'target' (NaN, NaN)
    constr : list of 8 dicts 'M', 'N', 'dMN' with the line intersections
             of each DRAWN valid solution (NaN for the others and when
             lines are parallel), computed whether or not
             'show_construction' is set.
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
    lw_fine  = 0.5

    _check_params(params)
    p = params
    gs_len = max(p['AB'], p['AC'], p['BD'], p['CD']) * 0.1125
    dxl = p['AC'] / 20
    A = np.array([0.0, 0.0])
    B = np.array([p['AB'], 0.0])

    inputs = np.asarray(inputs, dtype=float).ravel()
    if inputs.size != 2:
        raise ValueError('inputs must be a 2-element vector.')

    if ax is None:
        fig, ax = plt.subplots()
        ax.set_aspect('equal')
        ax.grid(True)
        ax.set_xlabel('X [mm]')
        ax.set_ylabel('Y [mm]')
        ax.set_title('Q2 Leg Mechanism')

    if opts.get('clear_axes', True):
        ax.cla()
        ax.set_aspect('equal')
        ax.grid(True)

    colors = np.asarray(opts.get('colors', DEFAULT_COLORS))
    show_constr = bool(opts.get('show_construction', False))
    constr_color = opts.get('construction_color', (0.0, 0.0, 0.0))

    # --- workspace of P (drawn first, under everything) ---------------
    W = opts.get('workspace')
    if W:
        if opts.get('workspace_solutions') is not None:
            ws_idx = list(opts['workspace_solutions'])
        elif opts.get('solutions') is not None:
            ws_idx = list(opts['solutions'])
        else:
            ws_idx = list(range(1, len(W['patch']) + 1))
        ws_idx = [i for i in ws_idx if 1 <= i <= len(W['patch'])]
        style = opts.get('workspace_style', 'fill').lower()
        alpha = opts.get('workspace_alpha', 0.15)
        bg = np.asarray(matplotlib_color_to_rgb(ax.get_facecolor()))
        for ii in ws_idx:
            col_w = colors[(ii - 1) % len(colors)]
            col_l = (1 - alpha) * bg + alpha * col_w     # light, opaque
            pd = W['patch'][ii - 1]
            do_fill = style in ('fill', 'both') and len(pd['Faces']) > 0
            if do_fill:
                # Opaque faces with edges in the same color: no darkening
                # where cells overlap and no seams between cells
                pc = PolyCollection(pd['Vertices'][pd['Faces']],
                                    facecolors=[col_l], edgecolors=[col_l],
                                    linewidths=0.5, zorder=0.5)
                ax.add_collection(pc)
            if style in ('points', 'both'):
                v = W['valid'][:, :, ii - 1].ravel(order='F').copy()
                if style == 'both' and do_fill:
                    v[pd['Faces'].ravel()] = False    # isolated nodes only
                if v.any():
                    Xw = W['X'][:, :, ii - 1].ravel(order='F')
                    Yw = W['Y'][:, :, ii - 1].ravel(order='F')
                    ax.plot(Xw[v], Yw[v], '.', color=col_l, markersize=4,
                            zorder=0.6)

    # --- kinematics ---------------------------------------------------
    mode = mode.lower()
    if mode in ('direct', 'd'):
        kin = {'thetaA': float(inputs[0]), 'rho': float(inputs[1]),
               'ik_error': np.nan, 'target': np.full(2, np.nan)}
    elif mode in ('inverse', 'i'):
        raise NotImplementedError('The inverse kinematics of the Q2 leg '
                                  'mechanism is not available yet.')
    else:
        raise ValueError("mode must be 'direct'")
    sol = Q2_leg_mechanism_direct_kinematics(kin['thetaA'], kin['rho'], p)

    nan2 = np.full(2, np.nan)
    constr = [{'M': nan2.copy(), 'N': nan2.copy(), 'dMN': np.nan}
              for _ in sol]

    # Ground symbols (fixed pivots A and B)
    _draw_ground_symbol(ax, A, lw_gs, lw_gs_b, gs_len)
    _draw_ground_symbol(ax, B, lw_gs, lw_gs_b, gs_len)

    if any(s['valid'] for s in sol):
        idx = opts.get('solutions')
        idx = list(range(1, len(sol) + 1)) if idx is None else list(idx)
        idx = [i for i in idx if 1 <= i <= len(sol)]

        for ii in idx:
            s = sol[ii - 1]
            if not s['valid']:
                continue
            pos = s['Positions']
            C, D, E, F = pos['C'], pos['D'], pos['E'], pos['F']
            G, K, J, I = pos['G'], pos['K'], pos['J'], pos['I']
            P = s['P']
            has_P = np.all(np.isfinite(P))
            col = colors[(ii - 1) % len(colors)]

            # Bodies: quaternary C-D-G-F, ternary B-D-E, G-K-J, I-J-P
            bodies = [[C, D, G, F], [B, D, E], [G, K, J]]
            if has_P:
                bodies.append([I, J, P])
            for body in bodies:
                ax.add_patch(plt.Polygon(body, closed=True, facecolor=col,
                                         edgecolor=col, alpha=0.5))
            if not has_P:
                ax.plot([I[0], J[0]], [I[1], J[1]], '-', color=col,
                        linewidth=lw_link)

            # Binary links: input crank A-C, and F-I
            for P1, P2 in ((A, C), (F, I)):
                ax.plot([P1[0], P2[0]], [P1[1], P2[1]], '-', color=col,
                        linewidth=lw_link)
            # Prismatic actuator E-K
            ax.plot([E[0], K[0]], [E[1], K[1]], '-.', color=col,
                    linewidth=lw_link)

            if has_P:
                ax.plot(P[0], P[1], 'kx', markersize=ms_P,
                        markeredgewidth=lw_link, zorder=4)

            # Construction: M = (AC) x (BD), N = (IF) x (GJ)
            Mc = _line_intersection(A, C, B, D)
            Nc = _line_intersection(I, F, G, J)
            constr[ii - 1] = {'M': Mc, 'N': Nc,
                              'dMN': float(np.linalg.norm(Nc - Mc))}
            if show_constr:
                if np.all(np.isfinite(Mc)):
                    _draw_collinear(ax, [A, C, Mc], constr_color, lw_fine)
                    _draw_collinear(ax, [B, D, Mc], constr_color, lw_fine)
                    ax.plot(Mc[0], Mc[1], 'kx', markersize=ms_P,
                            markeredgewidth=lw_link, zorder=4)
                if np.all(np.isfinite(Nc)):
                    _draw_collinear(ax, [I, F, Nc], constr_color, lw_fine)
                    _draw_collinear(ax, [G, J, Nc], constr_color, lw_fine)
                    ax.plot(Nc[0], Nc[1], 'kx', markersize=ms_P,
                            markeredgewidth=lw_link, zorder=4)

            # Joints as white-filled circles
            Jp = np.array([A, B, C, D, E, F, G, K, J, I])
            ax.scatter(Jp[:, 0], Jp[:, 1], s=ms_joint, facecolors='white',
                       edgecolors='black', linewidths=lw_joint, zorder=5)

            if opts.get('show_labels', True):
                pts = list(zip((A, B, C, D, E, F, G, K, J, I), 'ABCDEFGKJI'))
                if has_P:
                    pts.append((P, 'P'))
                if show_constr:
                    if np.all(np.isfinite(Mc)):
                        pts.append((Mc, LBL_ACBD))
                    if np.all(np.isfinite(Nc)):
                        pts.append((Nc, LBL_IFGJ))
                for pt, lbl in pts:
                    ax.text(pt[0] + dxl, pt[1] + dxl, lbl, fontsize=fs_label,
                            ha='left', va='bottom', color='k', zorder=6)

    ax.set_aspect('equal')
    lims = opts.get('limits')
    if lims is not None and len(lims) == 4:
        ax.set_xlim(lims[0], lims[1])
        ax.set_ylim(lims[2], lims[3])
    return ax, sol, kin, constr


# ── Helpers ──────────────────────────────────────────────────────────────────
def matplotlib_color_to_rgb(c):
    """RGB part of a matplotlib color (the axes background)."""
    from matplotlib.colors import to_rgb
    return to_rgb(c)


def _line_intersection(P1, P2, P3, P4):
    """Intersection of line (P1,P2) with line (P3,P4); NaN if parallel."""
    d1 = P2 - P1
    d2 = P4 - P3
    den = d1[0] * d2[1] - d1[1] * d2[0]
    if (not np.all(np.isfinite(np.r_[d1, d2])) or
            abs(den) <= 1e-12 * np.linalg.norm(d1) * np.linalg.norm(d2) or
            np.linalg.norm(d1) == 0 or np.linalg.norm(d2) == 0):
        return np.full(2, np.nan)
    w = P3 - P1
    return P1 + (w[0] * d2[1] - w[1] * d2[0]) / den * d1


def _draw_collinear(ax, pts, col, lw):
    """Fine line through collinear points, between the two extreme ones."""
    pts = np.array(pts, dtype=float)
    d = pts[1] - pts[0]
    if np.linalg.norm(d) == 0:
        d = pts[2] - pts[0]
    if np.linalg.norm(d) == 0:
        return
    d = d / np.linalg.norm(d)
    t = (pts - pts[0]) @ d
    p0 = pts[0] + t.min() * d
    p1 = pts[0] + t.max() * d
    ax.plot([p0[0], p1[0]], [p0[1], p1[1]], '-', color=col, linewidth=lw,
            zorder=3)


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
    p = dict(AB=150, AC=90, BD=120, CD=100, FG=80, CF=90, BE=50, GK=50,
             GJ=80, IF=80, IJ=100, IP=50, DCF=np.deg2rad(80),
             GFC=np.deg2rad(110), DBE=np.deg2rad(45), JGK=np.deg2rad(30),
             JIP=-np.deg2rad(30))
    fig, ax = plt.subplots()
    Q2_leg_mechanism_plot(p, 'direct', [np.deg2rad(-80), 220], {'solutions': [2]}, ax)
    plt.show()
