"""
fivebar_gui.py
Interactive Tkinter GUI for a Planar Five-Bar Linkage.

Layout matches fivebar_gui.m:
 - Geometry panel: 8 params in two columns
 - Mode radio buttons (Direct / Inverse)
 - Direct Mode Sliders: θ1, θ2
 - Inverse Mode Sliders: Px, Py
 - Display solutions: 4 checkboxes
 - Animate button + info text + matplotlib canvas

BY:
Prof. Lionel Birglen
Polytechnique Montreal, 2025
Contact: lionel.birglen@polymtl.ca
Code provided under GNU Affero General Public License v3.0
"""

import math, json, time
import numpy as np
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import matplotlib
matplotlib.use('TkAgg')
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

from fivebar_direct_kinematics import fivebar_direct_kinematics
from fivebar_inverse_kinematics import fivebar_inverse_kinematics
from fivebar_plot import fivebar_plot

DEF_GEO   = [0.6, 0.7, 0.9, 0.6, 1.0, 0.0, 0.5, 45.0]
GEO_NAMES = ['a (O→A)', 'b (A→B)', 'c (B→C)', 'd (C→D)',
             'e (O→D)', 'α (deg)',  'h (A→P)', 'η (deg)']
DEF_TH1   = 120.0
DEF_TH2   =  75.0
DEF_PX    =   0.2
DEF_PY    =   1.0


class FiveBarGUI:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Five-Bar Linkage")
        self.root.resizable(False, False)
        self.W, self.H = 900, 620
        self.root.geometry(f"{self.W}x{self.H}")

        self.mode      = tk.StringVar(value='Direct')
        self.animating = False
        self._anim_id  = None
        self._anim_t0  = None
        self._th1_off  = DEF_TH1
        self._th2_off  = DEF_TH2
        self._px_off   = DEF_PX
        self._py_off   = DEF_PY

        self._build_menu()
        self._build_left()
        self._build_canvas()
        self._update_plot()

    # ── Menu ──────────────────────────────────────────────────────────────────
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
        vm.add_command(label="Reset View",  command=self._cb_reset_view)
        om = tk.Menu(mb, tearoff=0)
        mb.add_cascade(label="Options", menu=om)
        om.add_command(label="Toggle Grid", command=self._cb_toggle_grid)

    # ── Left panel ────────────────────────────────────────────────────────────
    def _build_left(self):
        LP = 305
        left = tk.Frame(self.root, width=LP, height=self.H, bg='white')
        left.place(x=0, y=0, width=LP, height=self.H)
        left.pack_propagate(False)

        # Geometry: 2 columns of 4 (left: a,b,c,d | right: e,α,h,η)
        gf = ttk.LabelFrame(left, text="Geometry")
        gf.place(relx=0.02, rely=0.01, relwidth=0.96, relheight=0.22)
        self.geo_vars = []
        params = [
            ('a (O→A)', DEF_GEO[0], 0, 0),
            ('b (A→B)', DEF_GEO[1], 1, 0),
            ('c (B→C)', DEF_GEO[2], 2, 0),
            ('d (C→D)', DEF_GEO[3], 3, 0),
            ('e (O→D)', DEF_GEO[4], 0, 3),
            ('α (deg)', DEF_GEO[5], 1, 3),
            ('h (A→P)', DEF_GEO[6], 2, 3),
            ('η (deg)', DEF_GEO[7], 3, 3),
        ]
        gf.columnconfigure(2, minsize=8)
        for lbl, val, row, col in params:
            tk.Label(gf, text=lbl, bg='white', font=('TkDefaultFont', 9),
                     anchor='e').grid(row=row, column=col,
                                      padx=(6,2), pady=3, sticky='e')
            v = tk.StringVar(value=str(val))
            self.geo_vars.append(v)
            e = tk.Entry(gf, textvariable=v, width=7,
                         font=('TkDefaultFont', 9))
            e.grid(row=row, column=col+1, padx=(0,6), pady=3, sticky='w')
            e.bind('<Return>',   lambda *_: self._update_plot())
            e.bind('<FocusOut>', lambda *_: self._update_plot())

        # Mode
        mf = ttk.LabelFrame(left, text="Mode")
        mf.place(relx=0.02, rely=0.24, relwidth=0.96, relheight=0.08)
        tk.Radiobutton(mf, text='Direct',  variable=self.mode, value='Direct',
                       command=self._cb_mode, bg='white').pack(side='left', padx=20, pady=4)
        tk.Radiobutton(mf, text='Inverse', variable=self.mode, value='Inverse',
                       command=self._cb_mode, bg='white').pack(side='left', padx=20, pady=4)

        # Direct sliders: θ1, θ2
        df = ttk.LabelFrame(left, text="Direct Mode Sliders")
        df.place(relx=0.02, rely=0.33, relwidth=0.96, relheight=0.16)
        self.dir_vars       = []
        self.dir_entry_vars = []
        self.dir_sliders    = []
        self.dir_entries    = []
        for k, (lbl, val) in enumerate([('θ1', DEF_TH1), ('θ2', DEF_TH2)]):
            tk.Label(df, text=lbl).grid(row=k, column=0, padx=4, pady=4, sticky='w')
            sv = tk.DoubleVar(value=val)
            self.dir_vars.append(sv)
            sl = tk.Scale(df, variable=sv, from_=-180, to=180,
                          orient='horizontal', resolution=0.1, showvalue=False,
                          command=lambda v, _k=k: self._on_dir_slider(_k))
            sl.grid(row=k, column=1, sticky='ew', padx=4, pady=4)
            df.columnconfigure(1, weight=1)
            ev = tk.StringVar(value=f"{val:.1f}")
            self.dir_entry_vars.append(ev)
            e = tk.Entry(df, textvariable=ev, width=7)
            e.grid(row=k, column=2, padx=4, pady=4)
            e.bind('<Return>',   lambda ev2, _k=k: self._on_dir_entry(_k))
            e.bind('<FocusOut>', lambda ev2, _k=k: self._on_dir_entry(_k))
            self.dir_sliders.append(sl)
            self.dir_entries.append(e)

        # Inverse sliders: Px, Py
        ivf = ttk.LabelFrame(left, text="Inverse Mode Sliders")
        ivf.place(relx=0.02, rely=0.50, relwidth=0.96, relheight=0.16)
        self.inv_vars       = []
        self.inv_entry_vars = []
        self.inv_sliders    = []
        self.inv_entries    = []
        for k, (lbl, val) in enumerate([('Px', DEF_PX), ('Py', DEF_PY)]):
            tk.Label(ivf, text=lbl).grid(row=k, column=0, padx=4, pady=4, sticky='w')
            sv = tk.DoubleVar(value=val)
            self.inv_vars.append(sv)
            sl = tk.Scale(ivf, variable=sv, from_=-1.5, to=1.5,
                          orient='horizontal', resolution=0.01, showvalue=False,
                          command=lambda v, _k=k: self._on_inv_slider(_k))
            sl.grid(row=k, column=1, sticky='ew', padx=4, pady=4)
            ivf.columnconfigure(1, weight=1)
            ev = tk.StringVar(value=f"{val:.2f}")
            self.inv_entry_vars.append(ev)
            e = tk.Entry(ivf, textvariable=ev, width=7)
            e.grid(row=k, column=2, padx=4, pady=4)
            e.bind('<Return>',   lambda ev2, _k=k: self._on_inv_entry(_k))
            e.bind('<FocusOut>', lambda ev2, _k=k: self._on_inv_entry(_k))
            self.inv_sliders.append(sl)
            self.inv_entries.append(e)
        self._set_inv_state('disabled')

        # Display solutions: 4 checkboxes
        sf = ttk.LabelFrame(left, text="Display solutions:")
        sf.place(relx=0.02, rely=0.67, relwidth=0.96, relheight=0.09)
        self.sol_vars = [tk.BooleanVar(value=True) for _ in range(4)]
        for k in range(4):
            tk.Checkbutton(sf, text=str(k+1), variable=self.sol_vars[k],
                           command=self._update_plot, bg='white') \
                .pack(side='left', padx=12, pady=5)

        # Animate
        self.anim_btn = tk.Button(left, text='Animate', command=self._cb_animate)
        self.anim_btn.place(relx=0.02, rely=0.77, relwidth=0.96, height=26)

        # Info text
        self.info_var = tk.StringVar()
        tk.Label(left, textvariable=self.info_var, justify='left',
                 anchor='nw', bg='white', font=('TkDefaultFont', 9),
                 wraplength=292).place(relx=0.02, rely=0.83,
                                       relwidth=0.96, relheight=0.16)

    # ── Canvas ────────────────────────────────────────────────────────────────
    def _build_canvas(self):
        LP = 305
        self.fig = Figure(figsize=((self.W-LP)/100, self.H/100), dpi=100)
        self.ax  = self.fig.add_subplot(111)
        self.ax.set_aspect('equal'); self.ax.grid(True)
        self.ax.set_xlabel('X'); self.ax.set_ylabel('Y')
        self.ax.set_title('Five-Bar Linkage')
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.get_tk_widget().place(x=LP, y=0,
                                          width=self.W-LP, height=self.H)

    # ── Helpers ───────────────────────────────────────────────────────────────
    def _read_geo(self):
        vals = []
        for k, v in enumerate(self.geo_vars):
            try:
                vals.append(float(v.get()))
            except ValueError:
                vals.append(DEF_GEO[k])
        return np.array(vals)

    def _compute_limits(self, geo):
        a, b, c, d, e, alpha, h, eta = geo
        alpha_rad = math.radians(alpha)
        Dx = e * math.cos(alpha_rad)
        Dy = e * math.sin(alpha_rad)
        x_lo = max(-(a+b+h), Dx-(c+d))
        x_hi = min( (a+b+h), Dx+(c+d))
        y_lo = max(-(a+b+h), Dy-(c+d))
        y_hi = min( (a+b+h), Dy+(c+d))
        m = (a+b+c+d) * 0.08
        x_lo -= m; x_hi += m; y_lo -= m; y_hi += m
        rx = (x_hi-x_lo)/2; cx = (x_lo+x_hi)/2
        ry = (y_hi-y_lo)/2; cy = (y_lo+y_hi)/2
        r = max(rx, ry)
        return [cx-r, cx+r, cy-r, cy+r]

    def _set_dir_state(self, state):
        for sl, e in zip(self.dir_sliders, self.dir_entries):
            sl.config(state=state); e.config(state=state)

    def _set_inv_state(self, state):
        for sl, e in zip(self.inv_sliders, self.inv_entries):
            sl.config(state=state); e.config(state=state)

    # ── Mode ──────────────────────────────────────────────────────────────────
    def _cb_mode(self):
        if self.mode.get() == 'Direct':
            self._set_dir_state('normal')
            self._set_inv_state('disabled')
            # IK → set theta sliders from current Px,Py
            geo  = self._read_geo()
            Px   = self.inv_vars[0].get()
            Py   = self.inv_vars[1].get()
            sols = fivebar_inverse_kinematics(geo, [Px, Py])
            if sols:
                for k, attr in enumerate(['theta']):
                    th = sols[0]['theta']
                    for k2 in range(2):
                        v = max(-180, min(180, th[k2]))
                        self.dir_vars[k2].set(round(v, 1))
                        self.dir_entry_vars[k2].set(f"{v:.1f}")
        else:
            self._set_dir_state('disabled')
            self._set_inv_state('normal')
            # DK → set Px,Py from current thetas
            geo  = self._read_geo()
            th1  = self.dir_vars[0].get()
            th2  = self.dir_vars[1].get()
            sols = fivebar_direct_kinematics(geo, [th1, th2])
            if sols and sols[0]['valid']:
                P = sols[0]['P']
                lim = self._compute_limits(geo)
                for k, val in enumerate([P[0], P[1]]):
                    v = max(lim[k*2], min(lim[k*2+1], val))
                    self.inv_vars[k].set(round(v, 2))
                    self.inv_entry_vars[k].set(f"{v:.2f}")
        self._update_plot()

    def _on_dir_slider(self, k):
        self.dir_entry_vars[k].set(f"{self.dir_vars[k].get():.1f}")
        self._update_plot()

    def _on_dir_entry(self, k):
        try:
            v = max(-180, min(180, float(self.dir_entry_vars[k].get())))
            self.dir_vars[k].set(v)
        except ValueError:
            pass
        self._update_plot()

    def _on_inv_slider(self, k):
        self.inv_entry_vars[k].set(f"{self.inv_vars[k].get():.2f}")
        self._update_plot()

    def _on_inv_entry(self, k):
        try:
            v = max(-1.5, min(1.5, float(self.inv_entry_vars[k].get())))
            self.inv_vars[k].set(v)
        except ValueError:
            pass
        self._update_plot()

    # ── Plot ──────────────────────────────────────────────────────────────────
    def _update_plot(self):
        try:
            geo  = self._read_geo()
            lim  = self._compute_limits(geo)
            sols_to_draw = [k+1 for k, v in enumerate(self.sol_vars) if v.get()]

            self.ax.cla()
            self.ax.set_aspect('equal'); self.ax.grid(True)
            self.ax.set_xlabel('X'); self.ax.set_ylabel('Y')
            self.ax.set_title('Five-Bar Linkage')

            opts = {'clear_axes': False, 'show_labels': True,
                    'limits': lim, 'solutions': sols_to_draw}

            if self.mode.get() == 'Direct':
                th1 = self.dir_vars[0].get()
                th2 = self.dir_vars[1].get()
                self.dir_entry_vars[0].set(f"{th1:.1f}")
                self.dir_entry_vars[1].set(f"{th2:.1f}")
                _, sols = fivebar_plot(geo, 'direct', [th1, th2], opts, self.ax)
                info = f"Direct mode:\nth1={th1:.1f}°  th2={th2:.1f}°"
                for i, s in enumerate(sols):
                    if s['valid']:
                        P = s['P']
                        info += f"\nSol {i+1}: P=({P[0]:.3f}, {P[1]:.3f})"
                    else:
                        info += f"\nSol {i+1}: invalid"
            else:
                Px = self.inv_vars[0].get()
                Py = self.inv_vars[1].get()
                self.inv_entry_vars[0].set(f"{Px:.2f}")
                self.inv_entry_vars[1].set(f"{Py:.2f}")
                _, sols = fivebar_plot(geo, 'inverse', [Px, Py], opts, self.ax)
                info = f"Inverse mode:\nP=({Px:.3f}, {Py:.3f})"
                if not sols:
                    info += "\nUnreachable"
                for i, s in enumerate(sols):
                    th = s['theta']
                    info += f"\nSol {i+1}: th1={th[0]:.1f}°  th2={th[1]:.1f}°"

            self.ax.set_xlim(lim[0], lim[1])
            self.ax.set_ylim(lim[2], lim[3])
            self.info_var.set(info)
            self.canvas.draw_idle()
        except Exception as ex:
            self.info_var.set(str(ex))

    # ── Animation ─────────────────────────────────────────────────────────────
    def _cb_animate(self):
        if self.animating:
            self.animating = False
            self.anim_btn.config(text='Animate')
            if self._anim_id:
                self.root.after_cancel(self._anim_id)
        else:
            self.animating = True
            self.anim_btn.config(text='Stop')
            self._anim_t0 = time.time()
            self._th1_off = self.dir_vars[0].get()
            self._th2_off = self.dir_vars[1].get()
            self._px_off  = self.inv_vars[0].get()
            self._py_off  = self.inv_vars[1].get()
            self._anim_step()

    def _anim_step(self):
        if not self.animating:
            return
        dt = time.time() - self._anim_t0
        if self.mode.get() == 'Direct':
            A1, f1, A2, f2 = 60, 0.1, 45, 0.15
            th1 = self._th1_off + A1 * math.sin(2*math.pi*f1*dt)
            th2 = self._th2_off + A2 * math.sin(2*math.pi*f2*dt)
            th1 = max(-180, min(180, th1))
            th2 = max(-180, min(180, th2))
            self.dir_vars[0].set(round(th1, 1))
            self.dir_entry_vars[0].set(f"{th1:.1f}")
            self.dir_vars[1].set(round(th2, 1))
            self.dir_entry_vars[1].set(f"{th2:.1f}")
        else:
            R, f = 0.25, 0.05
            px = self._px_off + R * math.cos(2*math.pi*f*dt)
            py = self._py_off + R * math.sin(2*math.pi*f*dt)
            px = max(-1.5, min(1.5, px))
            py = max(-1.5, min(1.5, py))
            self.inv_vars[0].set(round(px, 2))
            self.inv_entry_vars[0].set(f"{px:.2f}")
            self.inv_vars[1].set(round(py, 2))
            self.inv_entry_vars[1].set(f"{py:.2f}")
        self._update_plot()
        self._anim_id = self.root.after(33, self._anim_step)

    # ── File callbacks ────────────────────────────────────────────────────────
    def _cb_open(self):
        path = filedialog.askopenfilename(
            filetypes=[("JSON session", "*.json"), ("All files", "*.*")])
        if not path:
            return
        try:
            with open(path) as f:
                sess = json.load(f)
            for k in range(8):
                self.geo_vars[k].set(str(sess['geo'][k]))
            for k in range(2):
                self.dir_vars[k].set(sess['theta'][k])
                self.dir_entry_vars[k].set(f"{sess['theta'][k]:.1f}")
            self.inv_vars[0].set(sess['Px']); self.inv_entry_vars[0].set(f"{sess['Px']:.2f}")
            self.inv_vars[1].set(sess['Py']); self.inv_entry_vars[1].set(f"{sess['Py']:.2f}")
            self.mode.set(sess.get('modeStr', 'Direct'))
            sv = sess.get('solsVisible', [True]*4)
            for k in range(4):
                self.sol_vars[k].set(sv[k])
            self._cb_mode()
        except Exception as ex:
            messagebox.showerror("Open Error", str(ex))

    def _cb_save(self):
        path = filedialog.asksaveasfilename(
            defaultextension=".json",
            filetypes=[("JSON session", "*.json")],
            initialfile="fivebar_session.json")
        if not path:
            return
        try:
            sess = {
                'geo':         [float(v.get()) for v in self.geo_vars],
                'theta':       [v.get() for v in self.dir_vars],
                'Px':          self.inv_vars[0].get(),
                'Py':          self.inv_vars[1].get(),
                'modeStr':     self.mode.get(),
                'solsVisible': [v.get() for v in self.sol_vars],
            }
            with open(path, 'w') as f:
                json.dump(sess, f, indent=2)
        except Exception as ex:
            messagebox.showerror("Save Error", str(ex))

    def _cb_export_png(self):
        path = filedialog.asksaveasfilename(
            defaultextension=".png",
            filetypes=[("PNG Image", "*.png")],
            initialfile="fivebar.png")
        if not path:
            return
        self.fig.savefig(path, dpi=150, bbox_inches='tight')

    def _cb_exit(self):
        if messagebox.askyesno("Exit", "Close the Five-Bar GUI?"):
            self.root.destroy()

    def _cb_reset_view(self):
        geo = self._read_geo()
        lim = self._compute_limits(geo)
        self.ax.set_xlim(lim[0], lim[1])
        self.ax.set_ylim(lim[2], lim[3])
        self.canvas.draw_idle()

    def _cb_toggle_grid(self):
        lines = self.ax.xaxis.get_gridlines()
        self.ax.grid(not lines[0].get_visible() if lines else True)
        self.canvas.draw_idle()

    def run(self):
        self.root.mainloop()


if __name__ == '__main__':
    FiveBarGUI().run()
