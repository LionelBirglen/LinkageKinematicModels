"""
fourbar_gui.py
Interactive Tkinter GUI for a Planar Four-Bar Linkage.

Layout matches fourbar_gui.m:
 - Geometry panel: 7 params (a,b,c,d,e,ε°,δ°) in two columns
 - Mode radio buttons (Direct / Inverse)
 - Direct Mode Slider panel: single θ slider
 - Inverse Mode Slider panel: single α slider
 - Display solutions: 2 checkboxes
 - Show P trajectory checkbox
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

from fourbar_direct_kinematics import fourbar_direct_kinematics
from fourbar_inverse_kinematics import fourbar_inverse_kinematics
from fourbar_plot import fourbar_plot

# ── Defaults (match fourbar_gui.m) ────────────────────────────────────────────
DEF_GEO    = [0.81, 0.88, 0.92, 1.51, 0.80, 30.0, -10.0]
GEO_LABELS = ['a', 'b', 'c', 'd', 'e', 'ε (°)', 'δ (°)']
DEF_THETA  = 106.0   # deg
DEF_ALPHA  = 120.0   # deg (matches alphaSlider default Value=120)

# ──────────────────────────────────────────────────────────────────────────────

class FourBarGUI:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Four-Bar Linkage")
        self.root.resizable(False, False)
        self.W, self.H = 900, 600
        self.root.geometry(f"{self.W}x{self.H}")

        self.mode      = tk.StringVar(value='Direct')
        self.animating = False
        self._anim_id  = None
        self._anim_t0  = None
        self._th_off   = DEF_THETA
        self._al_off   = DEF_ALPHA

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

    # ── Left panel (matches normalized positions in fourbar_gui.m) ────────────
    def _build_left(self):
        LP = 300
        H  = self.H
        left = tk.Frame(self.root, width=LP, height=H, bg='white')
        left.place(x=0, y=0, width=LP, height=H)
        left.pack_propagate(False)

        # ── Geometry: two columns, label left / entry right
        # Left col: a, b, c, d  |  Right col: e, ε(°), δ(°)
        gf = ttk.LabelFrame(left, text="Geometry")
        gf.place(relx=0.02, rely=0.01, relwidth=0.96, relheight=0.23)
        self.geo_vars = []
        # (label, default, grid_row, base_col)
        # Left column uses cols 0,1; right column uses cols 3,4; col 2 = spacer
        params = [
            ('a',     DEF_GEO[0], 0, 0),
            ('b',     DEF_GEO[1], 1, 0),
            ('c',     DEF_GEO[2], 2, 0),
            ('d',     DEF_GEO[3], 3, 0),
            ('e',     DEF_GEO[4], 0, 3),
            ('ε (°)', DEF_GEO[5], 1, 3),
            ('δ (°)', DEF_GEO[6], 2, 3),
        ]
        gf.columnconfigure(2, minsize=10)  # spacer between columns
        for lbl, val, row, col in params:
            tk.Label(gf, text=lbl, bg='white',
                     font=('TkDefaultFont',9), anchor='e')\
                .grid(row=row, column=col, padx=(6,2), pady=3, sticky='e')
            v = tk.StringVar(value=str(val))
            self.geo_vars.append(v)
            e = tk.Entry(gf, textvariable=v, width=7,
                         font=('TkDefaultFont',9))
            e.grid(row=row, column=col+1, padx=(0,6), pady=3, sticky='w')
            e.bind('<Return>',   lambda *_: self._update_plot())
            e.bind('<FocusOut>', lambda *_: self._update_plot())

        # ── Mode panel  .m: 'Position',[0.02 0.70 0.33 0.07]
        mf = ttk.LabelFrame(left, text="Mode")
        mf.place(relx=0.02, rely=0.24, relwidth=0.96, relheight=0.08)
        tk.Radiobutton(mf, text='Direct',  variable=self.mode, value='Direct',
                       command=self._cb_mode, bg='white').pack(side='left', padx=20, pady=4)
        tk.Radiobutton(mf, text='Inverse', variable=self.mode, value='Inverse',
                       command=self._cb_mode, bg='white').pack(side='left', padx=20, pady=4)

        # ── Direct Mode Slider  .m: 'Position',[0.02 0.61 0.33 0.08]
        df = ttk.LabelFrame(left, text="Direct Mode Slider")
        df.place(relx=0.02, rely=0.33, relwidth=0.96, relheight=0.10)
        tk.Label(df, text='θ').grid(row=0, column=0, padx=4, pady=6, sticky='w')
        self.theta_var = tk.DoubleVar(value=DEF_THETA)
        self.theta_sl  = tk.Scale(df, variable=self.theta_var, from_=-360, to=360,
                                   orient='horizontal', resolution=0.1, showvalue=False,
                                   command=lambda v: self._on_theta_slider())
        self.theta_sl.grid(row=0, column=1, sticky='ew', padx=4, pady=6)
        df.columnconfigure(1, weight=1)
        self.theta_ev = tk.StringVar(value=f"{DEF_THETA:.1f}")
        self.theta_entry = tk.Entry(df, textvariable=self.theta_ev, width=7)
        self.theta_entry.grid(row=0, column=2, padx=4, pady=6)
        self.theta_entry.bind('<Return>',   lambda *_: self._on_theta_entry())
        self.theta_entry.bind('<FocusOut>', lambda *_: self._on_theta_entry())

        # ── Inverse Mode Slider  .m: 'Position',[0.02 0.52 0.33 0.08]
        ivf = ttk.LabelFrame(left, text="Inverse Mode Slider")
        ivf.place(relx=0.02, rely=0.44, relwidth=0.96, relheight=0.10)
        tk.Label(ivf, text='α').grid(row=0, column=0, padx=4, pady=6, sticky='w')
        self.alpha_var = tk.DoubleVar(value=DEF_ALPHA)
        self.alpha_sl  = tk.Scale(ivf, variable=self.alpha_var, from_=-180, to=180,
                                   orient='horizontal', resolution=0.1, showvalue=False,
                                   command=lambda v: self._on_alpha_slider())
        self.alpha_sl.grid(row=0, column=1, sticky='ew', padx=4, pady=6)
        ivf.columnconfigure(1, weight=1)
        self.alpha_ev = tk.StringVar(value=f"{DEF_ALPHA:.1f}")
        self.alpha_entry = tk.Entry(ivf, textvariable=self.alpha_ev, width=7)
        self.alpha_entry.grid(row=0, column=2, padx=4, pady=6)
        self.alpha_entry.bind('<Return>',   lambda *_: self._on_alpha_entry())
        self.alpha_entry.bind('<FocusOut>', lambda *_: self._on_alpha_entry())
        # Start disabled  .m: set(alphaSlider,'Enable','off')
        self.alpha_sl.config(state='disabled')
        self.alpha_entry.config(state='disabled')

        # ── Display solutions  .m: solsPanel 'Position',[0.02 0.43 0.33 0.08]
        sf = ttk.LabelFrame(left, text="Display solutions:")
        sf.place(relx=0.02, rely=0.55, relwidth=0.96, relheight=0.09)
        self.sol_vars = [tk.BooleanVar(value=True), tk.BooleanVar(value=True)]
        for k in range(2):
            tk.Checkbutton(sf, text=str(k+1), variable=self.sol_vars[k],
                           command=self._update_plot, bg='white') \
                .pack(side='left', padx=25, pady=5)

        # ── Show P trajectory  .m: traj_checkbox 'Position',[20 235 270 20]
        self.traj_var = tk.BooleanVar(value=False)
        tk.Checkbutton(left, text='Show P trajectory', variable=self.traj_var,
                       command=self._update_plot, bg='white') \
            .place(relx=0.04, rely=0.65, relwidth=0.80, height=22)

        # ── Animate  .m: 'Position',[20 200 295 30]
        self.anim_btn = tk.Button(left, text='Animate', command=self._cb_animate)
        self.anim_btn.place(relx=0.02, rely=0.72, relwidth=0.96, height=28)

        # ── Info text  .m: 'Position',[20 95 295 100]
        self.info_var = tk.StringVar()
        tk.Label(left, textvariable=self.info_var, justify='left',
                 anchor='nw', bg='white', font=('TkDefaultFont', 9),
                 wraplength=285) \
            .place(relx=0.02, rely=0.80, relwidth=0.96, relheight=0.18)

    # ── Canvas  .m: axes 'Position',[320 100 550 450]
    def _build_canvas(self):
        LP = 300
        self.fig = Figure(figsize=((self.W-LP)/100, self.H/100), dpi=100)
        self.ax  = self.fig.add_subplot(111)
        self.ax.set_aspect('equal'); self.ax.grid(True)
        self.ax.set_xlabel('X'); self.ax.set_ylabel('Y')
        self.ax.set_title('Four-Bar Linkage')
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.get_tk_widget().place(x=LP, y=0,
                                          width=self.W-LP, height=self.H)

    # ── Helpers ───────────────────────────────────────────────────────────────
    def _read_geo(self):
        vals = []
        for k, v in enumerate(self.geo_vars):
            try:
                x = float(v.get())
            except ValueError:
                x = DEF_GEO[k]
            if k >= 5:
                x = math.radians(x)
            vals.append(x)
        return np.array(vals)

    def _auto_limits(self, geo):
        # Tight limits: ground link + longest of remaining links
        reach = (geo[3] + max(geo[0], geo[1], geo[2], geo[4])) * 1.1
        return [-reach, reach, -reach, reach]

    # ── Mode callbacks ────────────────────────────────────────────────────────
    def _cb_mode(self):
        if self.mode.get() == 'Direct':
            self.theta_sl.config(state='normal')
            self.theta_entry.config(state='normal')
            self.alpha_sl.config(state='disabled')
            self.alpha_entry.config(state='disabled')
            geo = self._read_geo()
            alpha = math.radians(self.alpha_var.get())
            sols = fourbar_inverse_kinematics(geo, alpha)
            for s in sols:
                if s['valid']:
                    th_d = max(-360, min(360, math.degrees(s['theta'])))
                    self.theta_var.set(round(th_d, 1))
                    self.theta_ev.set(f"{th_d:.1f}")
                    break
        else:
            self.theta_sl.config(state='disabled')
            self.theta_entry.config(state='disabled')
            self.alpha_sl.config(state='normal')
            self.alpha_entry.config(state='normal')
            geo = self._read_geo()
            theta = math.radians(self.theta_var.get())
            sols = fourbar_direct_kinematics(geo, theta)
            for s in sols:
                if s['valid']:
                    al_d = max(-180, min(180, math.degrees(s['alpha'])))
                    self.alpha_var.set(round(al_d, 1))
                    self.alpha_ev.set(f"{al_d:.1f}")
                    break
        self._update_plot()

    def _on_theta_slider(self):
        self.theta_ev.set(f"{self.theta_var.get():.1f}")
        self._update_plot()

    def _on_theta_entry(self):
        try:
            v = max(-360, min(360, float(self.theta_ev.get())))
            self.theta_var.set(v)
        except ValueError:
            pass
        self._update_plot()

    def _on_alpha_slider(self):
        self.alpha_ev.set(f"{self.alpha_var.get():.1f}")
        self._update_plot()

    def _on_alpha_entry(self):
        try:
            v = max(-180, min(180, float(self.alpha_ev.get())))
            self.alpha_var.set(v)
        except ValueError:
            pass
        self._update_plot()

    # ── Plot ──────────────────────────────────────────────────────────────────
    def _update_plot(self):
        try:
            geo = self._read_geo()
            lim = self._auto_limits(geo)
            sols_to_draw = [k+1 for k, v in enumerate(self.sol_vars) if v.get()]

            self.ax.cla()
            self.ax.set_aspect('equal'); self.ax.grid(True)
            self.ax.set_xlabel('X'); self.ax.set_ylabel('Y')
            self.ax.set_title('Four-Bar Linkage')

            opts = {'clear_axes': False, 'show_labels': True,
                    'limits': lim, 'solutions': sols_to_draw}

            if self.mode.get() == 'Direct':
                theta = math.radians(self.theta_var.get())
                self.theta_ev.set(f"{self.theta_var.get():.1f}")
                fourbar_plot(geo, 'direct', theta, opts, self.ax)
                sols = fourbar_direct_kinematics(geo, theta)
                info = f'Direct mode:\nθ = {self.theta_var.get():.2f}°'
                for i, s in enumerate(sols):
                    if (i+1) in sols_to_draw:
                        if s['valid']:
                            P = s['P']
                            info += (f"\nSol {i+1}: P=[{P[0]:.2f}; {P[1]:.2f}]"
                                     f"  α={math.degrees(s['alpha']):.2f}°"
                                     f"  φ={math.degrees(s['phi']):.2f}°")
                        else:
                            info += f"\nSol {i+1}: invalid"

                if self.traj_var.get():
                    thetas = np.linspace(-math.pi, math.pi, 360)
                    for si in sols_to_draw:
                        px_list, py_list = [], []
                        for th in thetas:
                            try:
                                ss = fourbar_direct_kinematics(geo, th)
                                s0 = ss[si - 1]
                                if s0['valid']:
                                    px_list.append(s0['P'][0])
                                    py_list.append(s0['P'][1])
                                else:
                                    px_list.append(float('nan'))
                                    py_list.append(float('nan'))
                            except Exception:
                                px_list.append(float('nan'))
                                py_list.append(float('nan'))
                        ls = 'k:' if si == 1 else 'k--'
                        if any(not math.isnan(x) for x in px_list):
                            self.ax.plot(px_list, py_list, ls, linewidth=1)
            else:
                alpha = math.radians(self.alpha_var.get())
                self.alpha_ev.set(f"{self.alpha_var.get():.1f}")
                fourbar_plot(geo, 'inverse', alpha, opts, self.ax)
                sols = fourbar_inverse_kinematics(geo, alpha)
                info = f'Inverse mode:\nα = {self.alpha_var.get():.2f}°'
                for i, s in enumerate(sols):
                    if (i+1) in sols_to_draw:
                        if s['valid']:
                            P = s['P']
                            info += (f"\nSol {i+1}: P=[{P[0]:.2f}; {P[1]:.2f}]"
                                     f"  θ={math.degrees(s['theta']):.2f}°"
                                     f"  φ={math.degrees(s['phi']):.2f}°")
                        else:
                            info += f"\nSol {i+1}: invalid"

            self.ax.set_xlim(lim[0], lim[1]); self.ax.set_ylim(lim[2], lim[3])
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
            self._th_off  = self.theta_var.get()
            self._al_off  = self.alpha_var.get()
            self._anim_step()

    def _anim_step(self):
        if not self.animating:
            return
        dt    = time.time() - self._anim_t0
        speed = 30.0
        if self.mode.get() == 'Direct':
            th = (self._th_off + speed * dt + 180) % 360 - 180
            self.theta_var.set(round(th, 1))
            self.theta_ev.set(f"{th:.1f}")
        else:
            al = (self._al_off + speed * dt + 180) % 360 - 180
            self.alpha_var.set(round(al, 1))
            self.alpha_ev.set(f"{al:.1f}")
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
            for k in range(7):
                self.geo_vars[k].set(str(sess['geo'][k]))
            self.theta_var.set(sess['theta'])
            self.theta_ev.set(f"{sess['theta']:.1f}")
            self.alpha_var.set(sess['alpha'])
            self.alpha_ev.set(f"{sess['alpha']:.1f}")
            self.mode.set(sess.get('modeStr', 'Direct'))
            sv = sess.get('solsVisible', [True, True])
            for k in range(2):
                self.sol_vars[k].set(sv[k])
            self.traj_var.set(sess.get('showTraj', False))
            self._cb_mode()
        except Exception as ex:
            messagebox.showerror("Open Error", str(ex))

    def _cb_save(self):
        path = filedialog.asksaveasfilename(
            defaultextension=".json",
            filetypes=[("JSON session", "*.json")],
            initialfile="fourbar_session.json")
        if not path:
            return
        try:
            sess = {
                'geo':         [float(v.get()) for v in self.geo_vars],
                'theta':       self.theta_var.get(),
                'alpha':       self.alpha_var.get(),
                'modeStr':     self.mode.get(),
                'solsVisible': [v.get() for v in self.sol_vars],
                'showTraj':    self.traj_var.get(),
            }
            with open(path, 'w') as f:
                json.dump(sess, f, indent=2)
        except Exception as ex:
            messagebox.showerror("Save Error", str(ex))

    def _cb_export_png(self):
        path = filedialog.asksaveasfilename(
            defaultextension=".png",
            filetypes=[("PNG Image", "*.png")],
            initialfile="fourbar.png")
        if not path:
            return
        self.fig.savefig(path, dpi=150, bbox_inches='tight')

    def _cb_exit(self):
        if messagebox.askyesno("Exit", "Close the Four-Bar GUI?"):
            self.root.destroy()

    def _cb_reset_view(self):
        geo = self._read_geo()
        lim = self._auto_limits(geo)
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
    FourBarGUI().run()
