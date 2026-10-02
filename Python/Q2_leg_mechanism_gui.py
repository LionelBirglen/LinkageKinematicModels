"""
Q2_leg_mechanism_gui.py
Interactive Tkinter GUI for the planar Q2 leg mechanism.

Layout and behaviour match Q2_leg_mechanism_gui.m:
 - 17 independent parameters (3 per row; angles DCF, GFC, DBE, JGK, JIP
   in radians, as stored)
 - "Show M (AC∩BD), N (IF∩GJ) and lines": line intersections M and N,
   their construction lines, and the distance |MN| in the info text
 - Input angle thetaA (deg) and prismatic length rho (mm): entry + slider
 - "Show workspace of P" over given thetaA / rho ranges and grid, cached
   (recomputed only when the parameters, ranges or grid change)
 - Animate: thetaA and rho move together along sines around predefined
   centers (see ANIM_* below)
 - Display solutions: 8 checkboxes, one per assembly branch, greyed out
   when the branch does not assemble
 - Fixed view computed from the geometry; a zoom/pan made with the
   toolbar is kept while the linkage moves, until View > Reset View
 - Sessions saved/opened as .json files
 The inverse kinematics is not available yet: the GUI runs in direct
 mode only, as the MATLAB version.

BY:
Prof. Lionel Birglen
Polytechnique Montreal, 2025
Contact: lionel.birglen@polymtl.ca
Code provided under GNU Affero General Public License v3.0
"""

import json, math, time
import numpy as np
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import matplotlib
matplotlib.use('TkAgg')
from matplotlib.backends.backend_tkagg import (FigureCanvasTkAgg,
                                               NavigationToolbar2Tk)
from matplotlib.figure import Figure

from Q2_leg_mechanism_direct_kinematics import PARAM_NAMES
from Q2_leg_mechanism_plot import Q2_leg_mechanism_plot, DEFAULT_COLORS
from Q2_leg_mechanism_workspace import Q2_leg_mechanism_workspace

# ── Defaults (match Q2_leg_mechanism_gui.m) ──────────────────────────────────
DEF_PARAMS = {
    'AB': 150, 'AC': 90, 'BD': 120, 'CD': 100, 'FG': 80, 'CF': 90,
    'BE': 50, 'GK': 50, 'GJ': 80, 'IF': 80, 'IJ': 100, 'IP': 50,
    'DCF': math.radians(80), 'GFC': math.radians(110),
    'DBE': math.radians(45), 'JGK': math.radians(30),
    'JIP': -0.5235987755982988,          # -30 deg
}
DEF_THETA_A = -80.0    # deg
DEF_RHO     = 220.0    # mm
N_SOLS      = 8

# Animation: both inputs move along sines of the same period T
ANIM_THETA_C   = -80.0   # deg: central position of thetaA
ANIM_THETA_AMP =  40.0   # deg: amplitude of thetaA
ANIM_RHO_C     = 220.0   # mm : central position of rho
ANIM_RHO_AMP   =  20.0   # mm : amplitude of rho
ANIM_T         =   6.0   # s  : period, common to both inputs
ANIM_PHASE     =   0.0   # deg: phase of rho relative to thetaA

# ──────────────────────────────────────────────────────────────────────────────


def fmt_num(v):
    """Shortest string that reads back as exactly v ('150', not '150.0')."""
    v = float(v)
    return str(int(v)) if v.is_integer() and abs(v) < 1e15 else repr(v)


def compute_limits(p, margin=0.05):
    """
    Fixed axis limits [xmin, xmax, ymin, ymax] enclosing every joint and P
    for ANY thetaA, rho and assembly branch (triangle-inequality bound,
    same as computeLimits in Q2_leg_mechanism_gui.m).
    """
    A = np.array([0.0, 0.0]); B = np.array([p['AB'], 0.0])
    E = np.array([[0.0, -1.0], [1.0, 0.0]])
    Cl = np.array([0.0, 0.0]); Dl = np.array([p['CD'], 0.0])
    xC = np.array([1.0, 0.0]); yC = E @ xC
    F = Cl + p['CF'] * (math.cos(p['DCF']) * xC - math.sin(p['DCF']) * yC)
    xF = (Cl - F) / np.linalg.norm(Cl - F); yF = E @ xF
    G = F + p['FG'] * (math.cos(p['GFC']) * xF - math.sin(p['GFC']) * yF)
    rCF, rDF = p['CF'], np.linalg.norm(F - Dl)
    rCG, rDG = np.linalg.norm(G - Cl), np.linalg.norm(G - Dl)

    disc = lambda c, r: np.array([c[0] - r, c[0] + r, c[1] - r, c[1] + r])
    grow = lambda b, r: b + np.array([-r, r, -r, r])

    def isect(b1, b2):
        b = np.array([max(b1[0], b2[0]), min(b1[1], b2[1]),
                      max(b1[2], b2[2]), min(b1[3], b2[3])])
        return b1 if (b[0] > b[1] or b[2] > b[3]) else b

    bC = disc(A, p['AC'])
    bD = isect(disc(A, p['AC'] + p['CD']), disc(B, p['BD']))
    bF = isect(disc(A, p['AC'] + rCF), disc(B, p['BD'] + rDF))
    bG = isect(disc(A, p['AC'] + rCG), disc(B, p['BD'] + rDG))
    bE = disc(B, p['BE'])
    bK = grow(bG, p['GK'])
    bJ = grow(bG, p['GJ'])
    bI = isect(grow(bF, p['IF']), grow(bJ, p['IJ']))
    boxes = np.array([bC, bD, bE, bF, bG, bK, bJ, bI, disc(A, 0), disc(B, 0),
                      grow(bI, abs(p['IP']))])
    lims = np.array([boxes[:, 0].min(), boxes[:, 1].max(),
                     boxes[:, 2].min(), boxes[:, 3].max()])
    m = margin * max(lims[1] - lims[0], lims[3] - lims[2])
    return list(lims + np.array([-m, m, -m, m]))


class Q2LegMechanismGUI:
    LP = 300          # width of the left panel (px)

    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Q2 Leg Mechanism GUI")
        self.root.resizable(False, False)
        self.W, self.H = 1000, 680
        self.root.geometry(f"{self.W}x{self.H}")

        self.params      = dict(DEF_PARAMS)
        # Current inputs (the entries are the reference, as in the MATLAB
        # GUI: a typed value may exceed the slider range)
        self.val         = {'thetaA': DEF_THETA_A, 'rho': DEF_RHO}
        self.animating   = False
        self._anim_id    = None
        self._anim_t0    = None
        self.user_view   = False     # True once the user zoomed/panned
        self.last_limits = None      # limits in effect after the last redraw
        self.ws_data     = None      # workspace cache
        self.ws_key      = None

        self._build_menu()
        self._build_left()
        self._build_canvas()
        self._update_plot()

    # ── Menu ─────────────────────────────────────────────────────────────────
    def _build_menu(self):
        mb = tk.Menu(self.root)
        self.root.config(menu=mb)
        fm = tk.Menu(mb, tearoff=0)
        mb.add_cascade(label="File", menu=fm)
        fm.add_command(label="Open",       command=self._cb_open)
        fm.add_command(label="Save",       command=self._cb_save)
        fm.add_command(label="Export PNG", command=self._cb_export_png)
        fm.add_separator()
        fm.add_command(label="Exit",       command=self._cb_exit)
        vm = tk.Menu(mb, tearoff=0)
        mb.add_cascade(label="View", menu=vm)
        vm.add_command(label="Reset View", command=self._cb_reset_view)
        om = tk.Menu(mb, tearoff=0)
        mb.add_cascade(label="Options", menu=om)
        om.add_command(label="Toggle Grid", command=self._cb_toggle_grid)

    # ── Left panel ───────────────────────────────────────────────────────────
    def _build_left(self):
        LP = self.LP
        left = tk.Frame(self.root, width=LP, height=self.H)
        left.place(x=0, y=0, width=LP, height=self.H)
        font = ('TkDefaultFont', 9)

        # 17 parameters, 3 per row
        pf = tk.Frame(left)
        pf.place(x=4, y=4, width=LP - 8, height=146)
        self.param_vars = {}
        for k, name in enumerate(PARAM_NAMES):
            r, c = divmod(k, 3)
            tk.Label(pf, text=name, font=font, width=4, anchor='e') \
                .grid(row=r, column=2 * c, padx=(2, 1), pady=1)
            v = tk.StringVar(value=fmt_num(self.params[name]))
            e = tk.Entry(pf, textvariable=v, width=7, font=font)
            e.grid(row=r, column=2 * c + 1, padx=(0, 4), pady=1)
            e.bind('<Return>',   lambda _e, n=name: self._cb_param(n))
            e.bind('<FocusOut>', lambda _e, n=name: self._cb_param(n))
            self.param_vars[name] = v

        # Construction points M, N
        self.constr_var = tk.BooleanVar(value=False)
        tk.Checkbutton(left, text='Show M (AC∩BD), N (IF∩GJ) and lines',
                       variable=self.constr_var, font=font,
                       command=self._update_plot).place(x=4, y=150)

        # Inputs: thetaA and rho (entry + slider)
        self.thetaA_var = tk.DoubleVar(value=DEF_THETA_A)
        self.thetaA_ev = tk.StringVar(value=fmt_num(DEF_THETA_A))
        self.rho_var = tk.DoubleVar(value=DEF_RHO)
        self.rho_ev = tk.StringVar(value=fmt_num(DEF_RHO))
        self.sliders = {}
        for (key, lbl, var, ev, lo, hi, y) in (
                ('thetaA', 'Input angle θA (deg)',    self.thetaA_var, self.thetaA_ev, -180, 180, 176),
                ('rho',    'Prismatic length ρ (mm)', self.rho_var,    self.rho_ev,     50, 400, 226)):
            tk.Label(left, text=lbl, font=font).place(x=6, y=y + 2)
            e = tk.Entry(left, textvariable=ev, width=11, font=font)
            e.place(x=170, y=y)
            e.bind('<Return>',   lambda _e, k=key: self._on_entry(k))
            e.bind('<FocusOut>', lambda _e, k=key: self._on_entry(k))
            sc = tk.Scale(left, variable=var, from_=lo, to=hi, orient='horizontal',
                          resolution=0.1, showvalue=False, length=280,
                          command=lambda _v, k=key: self._on_slider(k))
            sc.place(x=6, y=y + 22)
            self.sliders[key] = (sc, var, ev, lo, hi)

        # Workspace of P
        self.ws_var = tk.BooleanVar(value=False)
        tk.Checkbutton(left, text='Show workspace of P', variable=self.ws_var,
                       font=font, command=self._update_plot).place(x=4, y=274)
        self.ws_vars = {}
        for (lbl, keys, defs, y) in (
                ('θA range (deg)', ('tmin', 'tmax'), (-180, 180), 298),
                ('ρ range (mm)',   ('rmin', 'rmax'), (50, 400),   322),
                ('Grid nθ × nρ',   ('nt', 'nr'),     (91, 51),    346)):
            tk.Label(left, text=lbl, font=font).place(x=6, y=y + 2)
            for j, (key, d) in enumerate(zip(keys, defs)):
                v = tk.StringVar(value=str(d))
                e = tk.Entry(left, textvariable=v, width=7, font=font)
                e.place(x=130 + 70 * j, y=y)
                e.bind('<Return>',   lambda _e: self._update_plot())
                e.bind('<FocusOut>', lambda _e: self._update_plot())
                self.ws_vars[key] = v
        self.ws_status = tk.StringVar(value='')
        tk.Label(left, textvariable=self.ws_status, font=('TkDefaultFont', 8),
                 justify='left', anchor='nw', wraplength=285) \
            .place(x=6, y=370, width=288, height=30)

        # Animate
        self.anim_btn = tk.Button(left, text='Animate', command=self._cb_animate)
        self.anim_btn.place(x=6, y=402, width=288, height=26)

        # Display solutions: 8 checkboxes (2 rows of 4); solution 2 by default
        sf = ttk.LabelFrame(left, text="Display solutions:")
        sf.place(x=4, y=432, width=292, height=62)
        self.sol_vars, self.sol_cbs = [], []
        for k in range(N_SOLS):
            v = tk.BooleanVar(value=(k == 1))
            cb = tk.Checkbutton(sf, text=str(k + 1), variable=v, font=font,
                                command=self._update_plot)
            cb.grid(row=k // 4, column=k % 4, padx=12, pady=0, sticky='w')
            self.sol_vars.append(v)
            self.sol_cbs.append(cb)

        # Info text
        self.info_var = tk.StringVar()
        tk.Label(left, textvariable=self.info_var, justify='left', anchor='nw',
                 font=('TkDefaultFont', 8)) \
            .place(x=6, y=498, width=290, height=self.H - 502)

    # ── Canvas with the matplotlib toolbar (zoom / pan) ──────────────────────
    def _build_canvas(self):
        LP = self.LP
        self.fig = Figure(figsize=((self.W - LP) / 100, self.H / 100), dpi=100)
        self.ax = self.fig.add_subplot(111)
        frame = tk.Frame(self.root)
        frame.place(x=LP, y=0, width=self.W - LP, height=self.H)
        self.canvas = FigureCanvasTkAgg(self.fig, master=frame)
        self.toolbar = NavigationToolbar2Tk(self.canvas, frame)
        self.toolbar.update()
        self.toolbar.pack(side='bottom', fill='x')
        self.canvas.get_tk_widget().pack(side='top', fill='both', expand=True)

    # ── Callbacks ────────────────────────────────────────────────────────────
    def _cb_param(self, name):
        try:
            val = float(self.param_vars[name].get())
        except ValueError:
            self.param_vars[name].set(fmt_num(self.params[name]))
            return
        if val != self.params[name]:
            self.params[name] = val
            self._update_plot()

    def _on_slider(self, key):
        _, var, ev, lo, hi = self.sliders[key]
        # Ignore the callbacks caused by programmatic updates (the slider
        # shows the input clamped to its range and rounded to 0.1)
        if abs(var.get() - min(max(self.val[key], lo), hi)) < 0.051:
            return
        self.val[key] = var.get()
        ev.set(f"{self.val[key]:g}")
        self._update_plot()

    def _on_entry(self, key):
        _, var, ev, lo, hi = self.sliders[key]
        try:
            v = float(ev.get())
        except ValueError:
            ev.set(fmt_num(self.val[key]))
            return
        if v == self.val[key]:
            return
        self._set_input(key, v)
        self._update_plot()

    def _set_input(self, key, v, text=None):
        """Set an input; the slider shows it clamped to its range."""
        _, var, ev, lo, hi = self.sliders[key]
        self.val[key] = float(v)
        ev.set(fmt_num(v) if text is None else text)
        var.set(min(max(float(v), lo), hi))

    def _cb_reset_view(self):
        self.user_view = False
        self.last_limits = None
        self.toolbar.update()              # forget the toolbar's view history
        self._update_plot()

    def _cb_toggle_grid(self):
        lines = self.ax.xaxis.get_gridlines()
        self.ax.grid(not lines[0].get_visible() if lines else True)
        self.canvas.draw_idle()

    # ── Workspace (cached) ───────────────────────────────────────────────────
    def _get_workspace(self):
        if not self.ws_var.get():
            self.ws_status.set('')
            return None
        try:
            tl = [float(self.ws_vars['tmin'].get()), float(self.ws_vars['tmax'].get())]
            rl = [float(self.ws_vars['rmin'].get()), float(self.ws_vars['rmax'].get())]
            ng = [int(round(float(self.ws_vars['nt'].get()))),
                  int(round(float(self.ws_vars['nr'].get())))]
        except ValueError:
            tl = None
        if tl is None or tl[0] >= tl[1] or rl[0] >= rl[1] or min(ng) < 2:
            self.ws_status.set('Workspace: invalid ranges or grid '
                               '(need min < max, n >= 2).')
            return None
        key = (tuple(self.params[n] for n in PARAM_NAMES), tuple(tl), tuple(rl), tuple(ng))
        if key == self.ws_key and self.ws_data is not None:
            return self.ws_data
        self.ws_status.set(f'Computing workspace ({ng[0]} x {ng[1]})...')
        self.root.config(cursor='watch')
        self.root.update_idletasks()
        try:
            W = Q2_leg_mechanism_workspace(self.params, np.radians(tl), rl, ng)
            self.ws_data, self.ws_key = W, key
            self.ws_status.set(
                f"Workspace: {W['n'][0]} x {W['n'][1]} grid, {W['time']:.1f} s\n"
                f"nodes/branch: {' '.join(str(v) for v in W['nValid'])}")
        except Exception as ex:
            W, self.ws_data, self.ws_key = None, None, None
            self.ws_status.set(f'Workspace error: {ex}')
        self.root.config(cursor='')
        return W

    # ── Plot ─────────────────────────────────────────────────────────────────
    def _update_plot(self):
        thA = self.val['thetaA']
        rho = self.val['rho']
        try:
            # Axis limits: fixed from geometry unless the user zoomed/panned
            cur = list(self.ax.get_xlim()) + list(self.ax.get_ylim())
            if not self.user_view and self.last_limits is not None:
                tol = 1e-6 * max(abs(self.last_limits[1] - self.last_limits[0]), 1)
                self.user_view = any(abs(c - l) > tol
                                     for c, l in zip(cur, self.last_limits))
            lims = cur if self.user_view else compute_limits(self.params)

            sel = [k + 1 for k, v in enumerate(self.sol_vars) if v.get()]
            show_mn = self.constr_var.get()
            opts = {'clear_axes': True, 'show_labels': True, 'limits': lims,
                    'colors': DEFAULT_COLORS, 'solutions': sel if sel else [0],
                    'show_construction': show_mn,
                    'workspace': self._get_workspace(),
                    'workspace_style': 'both'}
            _, sols, kin, constr = Q2_leg_mechanism_plot(
                self.params, 'direct', [math.radians(thA), rho], opts, self.ax)

            self.ax.set_title('Q2 Leg Mechanism - Direct Mode')
            self.ax.set_xlabel('X [mm]'); self.ax.set_ylabel('Y [mm]')
            self.last_limits = list(self.ax.get_xlim()) + list(self.ax.get_ylim())

            # Enable only the checkboxes of branches that assemble
            valid = [s['valid'] for s in sols]
            for k, cb in enumerate(self.sol_cbs):
                cb.config(state='normal' if k < len(valid) and valid[k] else 'disabled')

            info = f"θA = {math.degrees(kin['thetaA']):.1f}°, ρ = {kin['rho']:.1f} mm\n"
            if not any(valid):
                info += 'Unreachable configuration (no branch assembles).'
                self.ax.text(0.5, 0.5, 'Unreachable configuration',
                             color='r', ha='center', fontsize=12,
                             transform=self.ax.transAxes)
            else:
                mn = []    # |MN| of each displayed solution, listed compactly
                for ii in sel:
                    s = sols[ii - 1]
                    if not s['valid']:
                        info += f"Sol {ii}: invalid\n"
                        continue
                    I = s['Positions']['I']
                    info += f"Sol {ii}: I=({I[0]:.1f}, {I[1]:.1f})"
                    if np.all(np.isfinite(s['P'])):
                        info += f" P=({s['P'][0]:.1f}, {s['P'][1]:.1f})"
                    info += f" Φ={math.degrees(s['phi']):.1f}°\n"
                    if show_mn:
                        d = constr[ii - 1]['dMN']
                        mn.append(f"{ii}: {d:.2f}" if np.isfinite(d) else f"{ii}: ∞")
                # |MN| values: 3 per line (fits the panel width)
                for q in range(0, len(mn), 3):
                    lead = '|MN| (mm): ' if q == 0 else '                  '
                    info += lead + '   '.join(mn[q:q + 3]) + '\n'
            self.info_var.set(info)
            self.canvas.draw_idle()
        except Exception as ex:
            self.ax.cla()
            self.ax.text(0.5, 0.5, f"Error: {ex}", color='r', ha='center',
                         transform=self.ax.transAxes)
            self.info_var.set('Invalid configuration.')
            self.canvas.draw_idle()

    # ── Animation (as in the MATLAB GUI) ─────────────────────────────────────
    def _cb_animate(self):
        if self.animating:
            self._stop_animation()
            return
        self.animating = True
        self.anim_btn.config(text='Stop')
        self._anim_t0 = time.time()
        self._anim_step()

    def _stop_animation(self):
        self.animating = False
        self.anim_btn.config(text='Animate')
        if self._anim_id is not None:
            self.root.after_cancel(self._anim_id)
            self._anim_id = None

    def _anim_step(self):
        if not self.animating:
            return
        w = 2 * math.pi * (time.time() - self._anim_t0) / ANIM_T
        thA = ANIM_THETA_C + ANIM_THETA_AMP * math.sin(w)
        rho = ANIM_RHO_C + ANIM_RHO_AMP * math.sin(w + math.radians(ANIM_PHASE))
        self._set_input('thetaA', thA, f"{thA:g}")
        self._set_input('rho', rho, f"{rho:g}")
        self._update_plot()
        self._anim_id = self.root.after(33, self._anim_step)

    # ── File callbacks (sessions as .json) ───────────────────────────────────
    def _cb_open(self):
        self._stop_animation()
        path = filedialog.askopenfilename(
            filetypes=[("JSON session", "*.json"), ("All files", "*.*")])
        if not path:
            return
        try:
            with open(path) as f:
                sess = json.load(f)
            if 'parameters' in sess:
                par = dict(sess['parameters'])
            elif all(n in sess for n in PARAM_NAMES):
                par = sess                    # bare parameter dict
                sess = {'parameters': par}
            else:
                raise ValueError('not a Q2 leg mechanism session')
            # previous parameter set: FCD -> DCF, FI -> IF (same values)
            for old, new in (('FCD', 'DCF'), ('FI', 'IF')):
                if old in par and new not in par:
                    par[new] = par[old]
            for n in PARAM_NAMES:
                if n in par and np.isfinite(float(par[n])):
                    self.params[n] = float(par[n])
                    self.param_vars[n].set(fmt_num(self.params[n]))
            if 'thetaA' in sess:
                self._set_input('thetaA', sess['thetaA'])
            if 'rho' in sess:
                self._set_input('rho', sess['rho'])
            if 'showConstruction' in sess:
                self.constr_var.set(bool(sess['showConstruction']))
            for key, fld in (('tmin', 'wsThetaMin'), ('tmax', 'wsThetaMax'),
                             ('rmin', 'wsRhoMin'), ('rmax', 'wsRhoMax'),
                             ('nt', 'wsNTheta'), ('nr', 'wsNRho')):
                if fld in sess:
                    self.ws_vars[key].set(fmt_num(sess[fld]))
            if 'showWorkspace' in sess:
                self.ws_var.set(bool(sess['showWorkspace']))
            sv = sess.get('solsVisible')
            if sv:
                if len(sv) == N_SOLS and all(v in (0, 1, True, False) for v in sv):
                    flags = [bool(v) for v in sv]
                else:                          # list of indices
                    flags = [(k + 1) in sv for k in range(N_SOLS)]
                for k in range(N_SOLS):
                    self.sol_vars[k].set(flags[k])
            # View: saved limits restored (and kept) if they differ from the
            # geometry-based ones
            self.user_view, self.last_limits = False, None
            xl, yl = sess.get('axesXLim'), sess.get('axesYLim')
            if xl and yl and len(xl) == 2 and len(yl) == 2:
                g = compute_limits(self.params)
                sav = list(xl) + list(yl)
                tol = 1e-6 * max(abs(g[1] - g[0]), 1)
                if any(abs(s - c) > tol for s, c in zip(sav, g)):
                    self.ax.set_xlim(*xl); self.ax.set_ylim(*yl)
                    self.user_view = True
            self._update_plot()
        except Exception as ex:
            messagebox.showerror("Open Error", str(ex))

    def _cb_save(self):
        path = filedialog.asksaveasfilename(
            defaultextension=".json", filetypes=[("JSON session", "*.json")],
            initialfile="Q2_leg_session.json")
        if not path:
            return
        try:
            sess = {
                'name':             'Q2 Leg Mechanism',
                'version':          1,
                'parameters':       dict(self.params),     # angles in rad
                'modeStr':          'Direct',
                'thetaA':           self.val['thetaA'],   # deg
                'rho':              self.val['rho'],
                'solsVisible':      [int(v.get()) for v in self.sol_vars],
                'showConstruction': int(self.constr_var.get()),
                'showWorkspace':    int(self.ws_var.get()),
                'wsThetaMin':       float(self.ws_vars['tmin'].get()),
                'wsThetaMax':       float(self.ws_vars['tmax'].get()),
                'wsRhoMin':         float(self.ws_vars['rmin'].get()),
                'wsRhoMax':         float(self.ws_vars['rmax'].get()),
                'wsNTheta':         float(self.ws_vars['nt'].get()),
                'wsNRho':           float(self.ws_vars['nr'].get()),
                'axesXLim':         list(self.ax.get_xlim()),
                'axesYLim':         list(self.ax.get_ylim()),
            }
            with open(path, 'w') as f:
                json.dump(sess, f, indent=2)
        except Exception as ex:
            messagebox.showerror("Save Error", str(ex))

    def _cb_export_png(self):
        path = filedialog.asksaveasfilename(
            defaultextension=".png", filetypes=[("PNG Image", "*.png")],
            initialfile="Q2_leg_mechanism.png")
        if path:
            self.fig.savefig(path, dpi=150, bbox_inches='tight')

    def _cb_exit(self):
        if messagebox.askyesno("Exit", "Are you sure you want to exit?"):
            self._stop_animation()
            self.root.destroy()

    def run(self):
        self.root.mainloop()


if __name__ == '__main__':
    Q2LegMechanismGUI().run()
