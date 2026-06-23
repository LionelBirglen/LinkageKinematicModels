"""
slidercrank_gui.py
Interactive Tkinter GUI for a Planar Slider-Crank Mechanism.

Layout matches slidercrank_gui.m:
 - Geometry panel: a, b, c, Slider angle (deg) in one row
 - Mode radio buttons (Direct / Inverse)
 - Direct Mode Sliders panel: 1 slider (φ)
 - Inverse Mode Sliders panel: 1 slider (x of P)
 - Display solutions: 2 checkboxes (elbow-down / elbow-up)
 - Animate button + info text + matplotlib canvas

BY:
Prof. Lionel Birglen
Polytechnique Montreal, 2025
Contact: lionel.birglen@polymtl.ca
Code provided under GNU Affero General Public License v3.0
"""

import math, json, time
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import matplotlib
matplotlib.use('TkAgg')
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

from slidercrank_direct_kinematics import slidercrank_direct_kinematics
from slidercrank_inverse_kinematics import slidercrank_inverse_kinematics
from slidercrank_plot import slidercrank_plot

DEF_A    = 50.0
DEF_B    = 120.0
DEF_C    = 30.0
DEF_SANG = 0.0
DEF_PHI  = 30.0
DEF_XS   = 140.0


class SliderCrankGUI:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Slider-Crank Linkage")
        self.root.resizable(False, False)
        self.W, self.H = 900, 600
        self.root.geometry(f"{self.W}x{self.H}")

        self.mode      = tk.StringVar(value='Direct')
        self.animating = False
        self._anim_id  = None
        self._anim_t0  = None
        self._phi_off  = DEF_PHI
        self._xs_off   = DEF_XS

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
        LP = 300
        H  = self.H
        left = tk.Frame(self.root, width=LP, height=H, bg='white')
        left.place(x=0, y=0, width=LP, height=H)
        left.pack_propagate(False)

        # ── Geometry: a, b, c, Slider° in one row
        # .m: geoPanel 'Position',[0.02 0.90 0.32 0.08]
        gf = ttk.LabelFrame(left, text="Geometry")
        gf.place(relx=0.02, rely=0.01, relwidth=0.96, relheight=0.10)
        geo_names   = ['a', 'b', 'c', 'ang°']
        geo_defs    = [DEF_A, DEF_B, DEF_C, DEF_SANG]
        self.geo_vars = []
        for k, (lbl, val) in enumerate(zip(geo_names, geo_defs)):
            tk.Label(gf, text=lbl, bg='white', font=('TkDefaultFont',9))\
                .grid(row=0, column=k*2, padx=(6,1), pady=6, sticky='e')
            v = tk.StringVar(value=str(val))
            self.geo_vars.append(v)
            e = tk.Entry(gf, textvariable=v, width=5, font=('TkDefaultFont',9))
            e.grid(row=0, column=k*2+1, padx=(0,4), pady=6)
            e.bind('<Return>',   lambda *_: self._update_plot())
            e.bind('<FocusOut>', lambda *_: self._update_plot())

        # ── Mode  .m: modePanel 'Position',[0.02 0.82 0.32 0.07]
        mf = ttk.LabelFrame(left, text="Mode")
        mf.place(relx=0.02, rely=0.12, relwidth=0.96, relheight=0.08)
        tk.Radiobutton(mf, text='Direct',  variable=self.mode, value='Direct',
                       command=self._cb_mode, bg='white').pack(side='left', padx=20, pady=4)
        tk.Radiobutton(mf, text='Inverse', variable=self.mode, value='Inverse',
                       command=self._cb_mode, bg='white').pack(side='left', padx=20, pady=4)

        # ── Direct Mode Sliders  .m: directPanel 'Position',[0.02 0.73 0.32 0.08]
        df = ttk.LabelFrame(left, text="Direct Mode Sliders")
        df.place(relx=0.02, rely=0.21, relwidth=0.96, relheight=0.10)
        tk.Label(df, text='θ (deg)').grid(row=0, column=0, padx=4, pady=6, sticky='w')
        self.dir_var = tk.DoubleVar(value=DEF_PHI)
        self.dir_sl  = tk.Scale(df, variable=self.dir_var, from_=-180, to=180,
                                 orient='horizontal', resolution=0.1, showvalue=False,
                                 command=lambda v: self._on_dir_slider())
        self.dir_sl.grid(row=0, column=1, sticky='ew', padx=4, pady=6)
        df.columnconfigure(1, weight=1)
        self.dir_ev = tk.StringVar(value=f"{DEF_PHI:.1f}")
        self.dir_entry = tk.Entry(df, textvariable=self.dir_ev, width=7)
        self.dir_entry.grid(row=0, column=2, padx=4, pady=6)
        self.dir_entry.bind('<Return>',   lambda *_: self._on_dir_entry())
        self.dir_entry.bind('<FocusOut>', lambda *_: self._on_dir_entry())

        # ── Inverse Mode Sliders  .m: inversePanel 'Position',[0.02 0.64 0.32 0.08]
        ivf = ttk.LabelFrame(left, text="Inverse Mode Sliders")
        ivf.place(relx=0.02, rely=0.32, relwidth=0.96, relheight=0.10)
        tk.Label(ivf, text='x').grid(row=0, column=0, padx=4, pady=6, sticky='w')
        self.inv_var = tk.DoubleVar(value=DEF_XS)
        self.inv_sl  = tk.Scale(ivf, variable=self.inv_var,
                                 from_=-(DEF_A+DEF_B+abs(DEF_C)),
                                 to=   (DEF_A+DEF_B+abs(DEF_C)),
                                 orient='horizontal', resolution=0.1, showvalue=False,
                                 command=lambda v: self._on_inv_slider())
        self.inv_sl.grid(row=0, column=1, sticky='ew', padx=4, pady=6)
        ivf.columnconfigure(1, weight=1)
        self.inv_ev = tk.StringVar(value=f"{DEF_XS:.1f}")
        self.inv_entry = tk.Entry(ivf, textvariable=self.inv_ev, width=7)
        self.inv_entry.grid(row=0, column=2, padx=4, pady=6)
        self.inv_entry.bind('<Return>',   lambda *_: self._on_inv_entry())
        self.inv_entry.bind('<FocusOut>', lambda *_: self._on_inv_entry())
        # Start disabled  .m: set(invSl,'Enable','off')
        self.inv_sl.config(state='disabled')
        self.inv_entry.config(state='disabled')

        # ── Display solutions  .m: solsPanel 'Position',[0.02 0.55 0.32 0.08]
        sf = ttk.LabelFrame(left, text="Display solutions:")
        sf.place(relx=0.02, rely=0.43, relwidth=0.96, relheight=0.10)
        self.sol_vars  = [tk.BooleanVar(value=True),
                          tk.BooleanVar(value=False)]
        sol_labels = ['1', '2']
        for k in range(2):
            tk.Checkbutton(sf, text=sol_labels[k], variable=self.sol_vars[k],
                           command=self._update_plot, bg='white') \
                .pack(side='left', padx=10, pady=5)

        # ── Animate  .m: 'Position',[20 290 285 30]
        self.anim_btn = tk.Button(left, text='Animate', command=self._cb_animate)
        self.anim_btn.place(relx=0.02, rely=0.55, relwidth=0.96, height=28)

        # ── Info text  .m: 'Position',[20 225 285 60]
        self.info_var = tk.StringVar()
        tk.Label(left, textvariable=self.info_var, justify='left',
                 anchor='nw', bg='white', font=('TkDefaultFont', 9),
                 wraplength=285).place(relx=0.02, rely=0.63,
                                       relwidth=0.96, relheight=0.35)

    # ── Canvas  .m: axes 'Position',[310 50 560 500]
    def _build_canvas(self):
        LP = 300
        self.fig = Figure(figsize=((self.W-LP)/100, self.H/100), dpi=100)
        self.ax  = self.fig.add_subplot(111)
        self.ax.set_aspect('equal'); self.ax.grid(True)
        self.ax.set_xlabel('X'); self.ax.set_ylabel('Y')
        self.ax.set_title('Slider-Crank Linkage')
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.get_tk_widget().place(x=LP, y=0,
                                          width=self.W-LP, height=self.H)

    # ── Helpers ───────────────────────────────────────────────────────────────
    def _read_geo(self):
        defs = [DEF_A, DEF_B, DEF_C, DEF_SANG]
        vals = []
        for k, v in enumerate(self.geo_vars):
            try:
                x = float(v.get())
                if k < 2 and x <= 0:
                    x = defs[k]
            except ValueError:
                x = defs[k]
            vals.append(x)
        return {'a': vals[0], 'b': vals[1], 'c': vals[2],
                'slider_angle': math.radians(vals[3])}

    # ── Mode ──────────────────────────────────────────────────────────────────
    def _cb_mode(self):
        if self.mode.get() == 'Direct':
            self.dir_sl.config(state='normal')
            self.dir_entry.config(state='normal')
            self.inv_sl.config(state='disabled')
            self.inv_entry.config(state='disabled')
            # Convert xP -> phi
            geo = self._read_geo()
            xP  = self.inv_var.get()
            xs  = xP - geo['c']
            sol = slidercrank_inverse_kinematics(geo['a'], geo['b'], geo['c'], xs, +1, geo['slider_angle'])
            if sol['valid']:
                phi_d = max(-180, min(180, math.degrees(sol['phi'])))
                self.dir_var.set(round(phi_d, 1))
                self.dir_ev.set(f"{phi_d:.1f}")
        else:
            self.dir_sl.config(state='disabled')
            self.dir_entry.config(state='disabled')
            self.inv_sl.config(state='normal')
            self.inv_entry.config(state='normal')
            # Convert phi -> xP
            geo = self._read_geo()
            phi = math.radians(self.dir_var.get())
            sol = slidercrank_direct_kinematics(geo['a'], geo['b'], geo['c'], phi, geo['slider_angle'], +1)
            if sol['valid']:
                P  = sol['Positions']['P']
                sd = sol['slider_dir']
                xP = float(P[0]*sd[0] + P[1]*sd[1])
                a, b, c = geo['a'], geo['b'], geo['c']
                xP = max(-(a+b)+c, min((a+b)+c, xP))
                self.inv_var.set(round(xP, 1))
                self.inv_ev.set(f"{xP:.1f}")
            self._update_inv_range(geo)
        self._update_plot()

    def _update_inv_range(self, geo):
        a, b, c = geo['a'], geo['b'], geo['c']
        self.inv_sl.config(from_=-(a+b)+c, to=(a+b)+c)

    def _on_dir_slider(self):
        self.dir_ev.set(f"{self.dir_var.get():.1f}")
        self._update_plot()

    def _on_dir_entry(self):
        try:
            v = max(-180, min(180, float(self.dir_ev.get())))
            self.dir_var.set(v)
        except ValueError:
            pass
        self._update_plot()

    def _on_inv_slider(self):
        self.inv_ev.set(f"{self.inv_var.get():.1f}")
        self._update_plot()

    def _on_inv_entry(self):
        try:
            geo = self._read_geo()
            a, b, c = geo['a'], geo['b'], geo['c']
            v = max(-(a+b)+c, min((a+b)+c, float(self.inv_ev.get())))
            self.inv_var.set(v)
        except ValueError:
            pass
        self._update_plot()

    # ── Plot ──────────────────────────────────────────────────────────────────
    def _update_plot(self):
        try:
            geo  = self._read_geo()
            a, b, c = geo['a'], geo['b'], geo['c']
            R    = (a + b + abs(c)) * 1.15
            lim  = [-R, R, -R, R]
            self._update_inv_range(geo)

            self.ax.cla()
            self.ax.set_aspect('equal'); self.ax.grid(True)
            self.ax.set_xlabel('X'); self.ax.set_ylabel('Y')
            self.ax.set_title('Slider-Crank Linkage')

            sol1_on = self.sol_vars[0].get()
            sol2_on = self.sol_vars[1].get()
            info    = ''

            if self.mode.get() == 'Direct':
                phi_deg = self.dir_var.get()
                self.dir_ev.set(f"{phi_deg:.1f}")
                phi  = math.radians(phi_deg)
                opts = {'clear_axes': True, 'show_labels': True, 'limits': lim,
                        'show_first': sol1_on, 'show_both': sol2_on}
                slidercrank_plot(geo, 'direct', [phi, +1], opts, self.ax)
                info = 'Direct mode:'
                for cfg, label in [(+1,'Sol 1'), (-1,'Sol 2')]:
                    on = sol1_on if cfg == +1 else sol2_on
                    if on:
                        s = slidercrank_direct_kinematics(geo['a'], geo['b'], geo['c'], phi, geo['slider_angle'], cfg)
                        if s['valid']:
                            P  = s['Positions']['P']
                            sd = s['slider_dir']
                            xP = float(P[0]*sd[0] + P[1]*sd[1])
                            info += f"\n{label}: φ={phi_deg:.2f}°   x={xP:.2f}"
                        else:
                            info += f"\n{label}: unreachable"
            else:
                xP  = self.inv_var.get()
                self.inv_ev.set(f"{xP:.1f}")
                xs  = xP - c
                opts = {'clear_axes': True, 'show_labels': True, 'limits': lim,
                        'show_first': sol1_on, 'show_both': sol2_on}
                slidercrank_plot(geo, 'inverse', [xs, +1], opts, self.ax)
                info = 'Inverse mode:'
                for cfg, label in [(+1,'Sol 1'), (-1,'Sol 2')]:
                    on = sol1_on if cfg == +1 else sol2_on
                    if on:
                        s = slidercrank_inverse_kinematics(geo['a'], geo['b'], geo['c'], xs, cfg, geo['slider_angle'])
                        if s['valid']:
                            info += f"\n{label}: x={xP:.2f}   φ={math.degrees(s['phi']):.2f}°"
                        else:
                            info += f"\n{label}: unreachable"

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
            self._phi_off = self.dir_var.get()
            self._xs_off  = self.inv_var.get()
            self._anim_step()

    def _anim_step(self):
        if not self.animating:
            return
        dt    = time.time() - self._anim_t0
        speed = 40.0
        geo   = self._read_geo()
        a, b, c = geo['a'], geo['b'], geo['c']
        if self.mode.get() == 'Direct':
            phi = (self._phi_off + speed * dt + 180) % 360 - 180
            self.dir_var.set(round(phi, 1))
            self.dir_ev.set(f"{phi:.1f}")
        else:
            xP_min = -(a+b) + c
            xP_max =  (a+b) + c
            xP_mid = (xP_min + xP_max) / 2
            xP_amp = (xP_max - xP_min) / 2
            xP = xP_mid + xP_amp * math.sin(0.8 * dt)
            self.inv_var.set(round(xP, 1))
            self.inv_ev.set(f"{xP:.1f}")
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
            for k, key in enumerate(['a', 'b', 'c', 'sang']):
                self.geo_vars[k].set(str(sess.get(key,
                                    [DEF_A,DEF_B,DEF_C,DEF_SANG][k])))
            self.dir_var.set(sess.get('phi', DEF_PHI))
            self.dir_ev.set(f"{sess.get('phi', DEF_PHI):.1f}")
            self.inv_var.set(sess.get('xs', DEF_XS))
            self.inv_ev.set(f"{sess.get('xs', DEF_XS):.1f}")
            self.mode.set(sess.get('modeStr', 'Direct'))
            sv = sess.get('solsVisible', [True, False])
            for k in range(2):
                self.sol_vars[k].set(sv[k])
            self._cb_mode()
        except Exception as ex:
            messagebox.showerror("Open Error", str(ex))

    def _cb_save(self):
        path = filedialog.asksaveasfilename(
            defaultextension=".json",
            filetypes=[("JSON session", "*.json")],
            initialfile="slidercrank_session.json")
        if not path:
            return
        try:
            sess = {
                'a':           float(self.geo_vars[0].get()),
                'b':           float(self.geo_vars[1].get()),
                'c':           float(self.geo_vars[2].get()),
                'sang':        float(self.geo_vars[3].get()),
                'phi':         self.dir_var.get(),
                'xs':          self.inv_var.get(),
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
            initialfile="slidercrank.png")
        if not path:
            return
        self.fig.savefig(path, dpi=150, bbox_inches='tight')

    def _cb_exit(self):
        if messagebox.askyesno("Exit", "Close the Slider-Crank GUI?"):
            self.root.destroy()

    def _cb_reset_view(self):
        geo = self._read_geo()
        R   = (geo['a'] + geo['b'] + abs(geo['c'])) * 1.15
        self.ax.set_xlim(-R, R); self.ax.set_ylim(-R, R)
        self.canvas.draw_idle()

    def _cb_toggle_grid(self):
        lines = self.ax.xaxis.get_gridlines()
        self.ax.grid(not lines[0].get_visible() if lines else True)
        self.canvas.draw_idle()

    def run(self):
        self.root.mainloop()


if __name__ == '__main__':
    SliderCrankGUI().run()
