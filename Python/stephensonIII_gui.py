"""
stephensonIII_gui.py
Interactive Tkinter GUI for a planar Stephenson III six-bar linkage.

Layout and behaviour match stephensonIII_gui.m:
 - Geometry panel: 12 parameters [OA Bx By OC CD DA | BF FE DE EP eta delta]
   (eta and delta in degrees)
 - Mode radio buttons (Direct / Inverse)
 - Direct slider: crank angle thetaO; inverse slider: angle thetaB of B-F
 - Display solutions: 6 checkboxes (at most 4 solutions in direct mode,
   6 in inverse mode), greyed out when a solution does not exist
 - Animate button: thetaO (direct) or thetaB (inverse) turns at 10 deg/s.
   Each solution keeps its number and color from one frame to the next
   (solution tracking), also when the sliders are moved; numbering
   restarts when the geometry or the mode changes.
 - Fixed view computed from the geometry; a zoom/pan made with the
   toolbar is kept while the linkage moves, until View > Reset View
 - Sessions saved/opened as .json files

BY:
Prof. Lionel Birglen
Polytechnique Montreal, 2025
Contact: lionel.birglen@polymtl.ca
Code provided under GNU Affero General Public License v3.0
"""

import json, time
import numpy as np
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import matplotlib
matplotlib.use('TkAgg')
from matplotlib.backends.backend_tkagg import (FigureCanvasTkAgg,
                                               NavigationToolbar2Tk)
from matplotlib.figure import Figure

from stephensonIII_plot import stephensonIII_plot

# ── Defaults (match stephensonIII_gui.m) ─────────────────────────────────────
DEF_GEO    = [40, 70, 30, 50, 20, 50, 30, 30, -30, 20, 30, 60]
GEO_LABELS = ['OA', 'Bx', 'By', 'OC', 'CD', 'DA',
              'BF', 'FE', 'DE', 'EP', 'η (°)', 'δ (°)']
DEF_THETA_O = 90.0    # deg
DEF_THETA_B = 69.0    # deg
N_SLOTS     = 6       # solution checkboxes
ANIM_SPEED  = 10.0    # deg/s, as in the MATLAB GUIs

# ──────────────────────────────────────────────────────────────────────────────


def fmt_num(v):
    """Shortest string that reads back as exactly v ('40', not '40.0')."""
    v = float(v)
    return str(int(v)) if v.is_integer() and abs(v) < 1e15 else repr(v)


def compute_limits(geo, margin=0.05):
    """
    Fixed axis limits [xmin, xmax, ymin, ymax] enclosing every reachable
    pose (all joints and P, any input and assembly mode), from the
    triangle inequality along the chains of rigid bodies starting at the
    ground pivots. Same bound as computeLimits in stephensonIII_gui.m.
    """
    OA, Bx, By, OC, CD, DA, BF, FE, DE, EP, eta, delta = geo
    O = np.array([0.0, 0.0]); A = np.array([OA, 0.0]); B = np.array([Bx, By])
    OC, DA, BF = abs(OC), abs(DA), abs(BF)

    # Branch-independent distances C-E (body C-D-E) and F-P (body F-E-P)
    d, e = np.radians(delta), np.radians(eta)
    Eloc = np.array([CD, 0.0]) + DE * np.array([-np.cos(d), -np.sin(d)])
    CE = np.linalg.norm(Eloc)
    FP = np.linalg.norm(np.array([FE, 0.0]) - EP * np.array([np.cos(e), np.sin(e)]))

    disc = lambda c, r: np.array([c[0] - r, c[0] + r, c[1] - r, c[1] + r])
    grow = lambda b, r: b + np.array([-r, r, -r, r])

    def isect(b1, b2):
        b = np.array([max(b1[0], b2[0]), min(b1[1], b2[1]),
                      max(b1[2], b2[2]), min(b1[3], b2[3])])
        return b1 if (b[0] > b[1] or b[2] > b[3]) else b

    bC = disc(O, OC)
    bD = isect(disc(A, DA), disc(O, OC + abs(CD)))
    bE = isect(disc(A, DA + abs(DE)), disc(O, OC + CE))
    bF = isect(disc(B, BF), grow(bE, abs(FE)))
    bP = isect(grow(bE, abs(EP)), disc(B, BF + FP))
    boxes = np.array([bC, bD, bE, bF, bP, disc(O, 0), disc(A, 0), disc(B, 0)])
    lims = np.array([boxes[:, 0].min(), boxes[:, 1].max(),
                     boxes[:, 2].min(), boxes[:, 3].max()])
    m = margin * max(lims[1] - lims[0], lims[3] - lims[2])
    return list(lims + np.array([-m, m, -m, m]))


class StephensonIIIGUI:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Stephenson III Linkage GUI")
        self.root.resizable(False, False)
        self.W, self.H = 900, 620
        self.root.geometry(f"{self.W}x{self.H}")

        self.mode        = tk.StringVar(value='Direct')
        self.animating   = False
        self._anim_id    = None
        self._anim_t0    = None
        self._anim_x0    = 0.0
        self.prev_sols   = None     # previous frame's solutions (tracking)
        self.user_view   = False    # True once the user zoomed/panned
        self.last_limits = None     # limits in effect after the last redraw

        self._build_menu()
        self._build_left()
        self._build_canvas()
        self._apply_mode_state()
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

    # ── Left panel (matches the layout of stephensonIII_gui.m) ──────────────
    def _build_left(self):
        LP = 300
        left = tk.Frame(self.root, width=LP, height=self.H, bg='white')
        left.place(x=0, y=0, width=LP, height=self.H)

        # Geometry: two columns of 6 parameters
        gf = ttk.LabelFrame(left, text="Geometry")
        gf.place(relx=0.02, rely=0.01, relwidth=0.96, relheight=0.30)
        gf.columnconfigure(2, minsize=14)
        self.geo_vars = []
        for k, (lbl, val) in enumerate(zip(GEO_LABELS, DEF_GEO)):
            row, col = k % 6, (0 if k < 6 else 3)
            tk.Label(gf, text=lbl, font=('TkDefaultFont', 9), anchor='e') \
                .grid(row=row, column=col, padx=(6, 2), pady=1, sticky='e')
            v = tk.StringVar(value=fmt_num(val))
            self.geo_vars.append(v)
            e = tk.Entry(gf, textvariable=v, width=9, font=('TkDefaultFont', 9))
            e.grid(row=row, column=col + 1, padx=(0, 6), pady=1, sticky='w')
            e.bind('<Return>',   lambda *_: self._cb_geometry())
            e.bind('<FocusOut>', lambda *_: self._cb_geometry())

        # Mode
        mf = ttk.LabelFrame(left, text="Mode")
        mf.place(relx=0.02, rely=0.32, relwidth=0.96, relheight=0.08)
        tk.Radiobutton(mf, text='Direct', variable=self.mode, value='Direct',
                       command=self._cb_mode).pack(side='left', padx=20)
        tk.Radiobutton(mf, text='Inverse', variable=self.mode, value='Inverse',
                       command=self._cb_mode).pack(side='left', padx=20)

        # Direct kinematics slider (thetaO)
        self.thetaO_var, self.thetaO_ev, self.thetaO_sl, self.thetaO_en = \
            self._slider(left, "Direct Kinematics", 'θO', DEF_THETA_O, 0.41)
        # Inverse kinematics slider (thetaB)
        self.thetaB_var, self.thetaB_ev, self.thetaB_sl, self.thetaB_en = \
            self._slider(left, "Inverse Kinematics", 'θB', DEF_THETA_B, 0.50)

        # Display solutions: 6 checkboxes, 2 rows of 3; solution 2 by default
        sf = ttk.LabelFrame(left, text="Display solutions:")
        sf.place(relx=0.02, rely=0.59, relwidth=0.96, relheight=0.10)
        self.sol_vars, self.sol_cbs = [], []
        for k in range(N_SLOTS):
            v = tk.BooleanVar(value=(k == 1))
            cb = tk.Checkbutton(sf, text=str(k + 1), variable=v,
                                command=self._update_plot)
            cb.grid(row=k // 3, column=k % 3, padx=22, pady=0, sticky='w')
            self.sol_vars.append(v)
            self.sol_cbs.append(cb)

        # Animate
        self.anim_btn = tk.Button(left, text='Animate', command=self._cb_animate)
        self.anim_btn.place(relx=0.02, rely=0.70, relwidth=0.96, height=28)

        # Info text
        self.info_var = tk.StringVar()
        tk.Label(left, textvariable=self.info_var, justify='left', anchor='nw',
                 bg='white', font=('TkDefaultFont', 9), wraplength=285) \
            .place(relx=0.02, rely=0.76, relwidth=0.96, relheight=0.23)

    def _slider(self, parent, title, label, default, rely):
        f = ttk.LabelFrame(parent, text=title)
        f.place(relx=0.02, rely=rely, relwidth=0.96, relheight=0.085)
        tk.Label(f, text=label).grid(row=0, column=0, padx=4, sticky='w')
        var = tk.DoubleVar(value=default)
        ev = tk.StringVar(value=f"{default:.1f}")
        sl = tk.Scale(f, variable=var, from_=-180, to=180, orient='horizontal',
                      resolution=0.1, showvalue=False,
                      command=lambda _v: self._on_slider(var, ev))
        sl.grid(row=0, column=1, sticky='ew', padx=4)
        f.columnconfigure(1, weight=1)
        en = tk.Entry(f, textvariable=ev, width=7)
        en.grid(row=0, column=2, padx=4)
        en.bind('<Return>',   lambda *_: self._on_entry(var, ev))
        en.bind('<FocusOut>', lambda *_: self._on_entry(var, ev))
        return var, ev, sl, en

    # ── Canvas with the matplotlib toolbar (zoom / pan) ──────────────────────
    def _build_canvas(self):
        LP = 300
        self.fig = Figure(figsize=((self.W - LP) / 100, self.H / 100), dpi=100)
        self.ax = self.fig.add_subplot(111)
        frame = tk.Frame(self.root)
        frame.place(x=LP, y=0, width=self.W - LP, height=self.H)
        self.canvas = FigureCanvasTkAgg(self.fig, master=frame)
        self.toolbar = NavigationToolbar2Tk(self.canvas, frame)
        self.toolbar.update()
        self.toolbar.pack(side='bottom', fill='x')
        self.canvas.get_tk_widget().pack(side='top', fill='both', expand=True)

    # ── Helpers ──────────────────────────────────────────────────────────────
    def _read_geo(self):
        vals = []
        for k, v in enumerate(self.geo_vars):
            try:
                vals.append(float(v.get()))
            except ValueError:
                raise ValueError(f"Invalid value for {GEO_LABELS[k]}")
        return np.array(vals)

    def _apply_mode_state(self):
        direct = self.mode.get() == 'Direct'
        for w in (self.thetaO_sl, self.thetaO_en):
            w.config(state='normal' if direct else 'disabled')
        for w in (self.thetaB_sl, self.thetaB_en):
            w.config(state='disabled' if direct else 'normal')

    # ── Callbacks ────────────────────────────────────────────────────────────
    def _cb_mode(self):
        self._stop_animation()
        self.prev_sols = None          # new mode: restart numbering
        self._apply_mode_state()
        self._update_plot()

    def _cb_geometry(self):
        self.prev_sols = None          # new geometry: restart numbering
        self.user_view = False         # and new default view
        self.last_limits = None
        self._update_plot()

    def _on_slider(self, var, ev):
        ev.set(f"{var.get():.1f}")
        self._update_plot()

    def _on_entry(self, var, ev):
        try:
            var.set(max(-180.0, min(180.0, float(ev.get()))))
        except ValueError:
            pass
        self._update_plot()

    def _cb_reset_view(self):
        self.user_view = False
        self.last_limits = None
        self.toolbar.update()          # forget the toolbar's view history
        self._update_plot()

    def _cb_toggle_grid(self):
        lines = self.ax.xaxis.get_gridlines()
        self.ax.grid(not lines[0].get_visible() if lines else True)
        self.canvas.draw_idle()

    # ── Plot ─────────────────────────────────────────────────────────────────
    def _update_plot(self):
        try:
            geo = self._read_geo()

            # Axis limits: fixed from geometry unless the user zoomed/panned
            cur = list(self.ax.get_xlim()) + list(self.ax.get_ylim())
            if not self.user_view and self.last_limits is not None:
                tol = 1e-6 * max(abs(self.last_limits[1] - self.last_limits[0]), 1)
                self.user_view = any(abs(c - l) > tol
                                     for c, l in zip(cur, self.last_limits))
            lims = cur if self.user_view else compute_limits(geo)

            sel = [k + 1 for k, v in enumerate(self.sol_vars) if v.get()]
            opts = {'clear_axes': True, 'show_labels': True, 'limits': lims,
                    'solutions': sel if sel else [0],
                    'track': self.prev_sols, 'n_slots': N_SLOTS}

            if self.mode.get() == 'Direct':
                thO = self.thetaO_var.get()
                self.thetaO_ev.set(f"{thO:.1f}")
                _, sols = stephensonIII_plot(geo, 'direct', thO, opts, self.ax)
                info = f"Direct mode: θO = {thO:.1f}°\n"
            else:
                thB = self.thetaB_var.get()
                self.thetaB_ev.set(f"{thB:.1f}")
                _, sols = stephensonIII_plot(geo, 'inverse', thB, opts, self.ax)
                info = f"Inverse mode: θB = {thB:.1f}°\n"
            self.prev_sols = sols

            self.ax.set_xlabel('X'); self.ax.set_ylabel('Y')
            self.ax.set_title(f"Stephenson III Linkage - {self.mode.get()} Mode")
            self.last_limits = list(self.ax.get_xlim()) + list(self.ax.get_ylim())

            # Enable only the checkboxes of existing, valid solutions
            valid = [s['valid'] for s in sols]
            for k, cb in enumerate(self.sol_cbs):
                ok = k < len(valid) and valid[k]
                cb.config(state='normal' if ok else 'disabled')

            if not any(valid):
                info += "No valid solution."
            else:
                # one line per solution: P and the angle that is not the
                # input (thetaB in direct mode, thetaO in inverse mode)
                key, name = (('thetaB', 'θB') if self.mode.get() == 'Direct'
                             else ('thetaO', 'θO'))
                for k, s in enumerate(sols):
                    if s['valid']:
                        P = s['Positions']['P']
                        info += (f"Sol {k+1}: P=[{P[0]:.2f}; {P[1]:.2f}]  "
                                 f"{name}={s['Angles'][key]:.1f}°\n")
            self.info_var.set(info)
            self.canvas.draw_idle()
        except Exception as ex:
            self.ax.cla()
            self.ax.text(0.5, 0.5, f"Error: {ex}", color='r', ha='center',
                         transform=self.ax.transAxes)
            self.info_var.set("Unreachable configuration")
            self.canvas.draw_idle()

    # ── Animation (same scheme as the MATLAB GUIs) ───────────────────────────
    def _cb_animate(self):
        if self.animating:
            self._stop_animation()
            return
        self.animating = True
        self.anim_btn.config(text='Stop')
        self._anim_t0 = time.time()
        self._anim_x0 = (self.thetaO_var if self.mode.get() == 'Direct'
                         else self.thetaB_var).get()
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
        val = (self._anim_x0 + ANIM_SPEED * (time.time() - self._anim_t0)
               + 180) % 360 - 180
        var = self.thetaO_var if self.mode.get() == 'Direct' else self.thetaB_var
        var.set(val)
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
            if len(sess['geo']) != 12:
                raise ValueError('not a Stephenson III session (geo must have 12 values)')
            for k in range(12):
                self.geo_vars[k].set(fmt_num(sess['geo'][k]))
            if 'thetaO' in sess:
                self.thetaO_var.set(max(-180.0, min(180.0, sess['thetaO'])))
            if 'thetaB' in sess:
                self.thetaB_var.set(max(-180.0, min(180.0, sess['thetaB'])))
            if sess.get('modeStr') in ('Direct', 'Inverse'):
                self.mode.set(sess['modeStr'])
            for k, v in enumerate(sess.get('solsVisible', [])[:N_SLOTS]):
                self.sol_vars[k].set(bool(v))
            self._apply_mode_state()
            self.prev_sols = None
            # View: saved limits are restored (and kept) if they differ
            # from the geometry-based ones
            self.user_view, self.last_limits = False, None
            xl, yl = sess.get('axesXLim'), sess.get('axesYLim')
            if xl and yl and len(xl) == 2 and len(yl) == 2:
                g = compute_limits(self._read_geo())
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
            initialfile="stephensonIII_session.json")
        if not path:
            return
        try:
            sess = {
                'name':        'Stephenson III Linkage',
                'version':     1,
                'geo':         [float(v.get()) for v in self.geo_vars],
                'thetaO':      self.thetaO_var.get(),
                'thetaB':      self.thetaB_var.get(),
                'modeStr':     self.mode.get(),
                'solsVisible': [int(v.get()) for v in self.sol_vars],
                'axesXLim':    list(self.ax.get_xlim()),
                'axesYLim':    list(self.ax.get_ylim()),
            }
            with open(path, 'w') as f:
                json.dump(sess, f, indent=2)
        except Exception as ex:
            messagebox.showerror("Save Error", str(ex))

    def _cb_export_png(self):
        path = filedialog.asksaveasfilename(
            defaultextension=".png", filetypes=[("PNG Image", "*.png")],
            initialfile="stephensonIII.png")
        if path:
            self.fig.savefig(path, dpi=150, bbox_inches='tight')

    def _cb_exit(self):
        if messagebox.askyesno("Exit", "Are you sure you want to exit?"):
            self._stop_animation()
            self.root.destroy()

    def run(self):
        self.root.mainloop()


if __name__ == '__main__':
    StephensonIIIGUI().run()
