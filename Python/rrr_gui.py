"""
rrr_gui.py
Interactive Tkinter GUI for a Planar RRR Serial Linkage.

Layout matches rrr_gui.m:
 - Geometry panel: L1, L2, L3 side-by-side
 - Mode radio buttons (Direct / Inverse)
 - Elbow: Up/Down button  +  Show both checkbox
 - Direct Mode Sliders panel: 3 sliders (θ1, θ2, θ3)
 - Inverse Mode Sliders panel: 3 sliders (X, Y, φ)
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

from rrr_direct_kinematics import rrr_direct_kinematics
from rrr_inverse_kinematics import rrr_inverse_kinematics
from rrr_plot import rrr_plot

DEF_L   = [57.0, 46.0, 51.0]
DEF_T   = [39.0, 37.0, 40.0]
DEF_INV = [35.0, 125.0, 116.0]
DEF_CFG = 1   # +1 elbow-up


class RRRGUI:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Planar RRR Linkage")
        self.root.resizable(False, False)
        self.W, self.H = 900, 600
        self.root.geometry(f"{self.W}x{self.H}")

        self.mode      = tk.StringVar(value='Direct')
        self.cfg_state = DEF_CFG
        self.animating = False
        self._anim_id  = None
        self._anim_t0  = None
        self._th_off   = list(DEF_T)
        self._inv_off  = list(DEF_INV)

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

        # ── Geometry: L1 L2 L3 side by side
        # .m: geoPanel 'Position',[0.02 0.90 0.32 0.09]
        gf = ttk.LabelFrame(left, text="Geometry")
        gf.place(relx=0.02, rely=0.01, relwidth=0.96, relheight=0.10)
        self.geo_vars = []
        for k, (lbl, val) in enumerate(zip(['L1','L2','L3'], DEF_L)):
            tk.Label(gf, text=lbl, bg='white').grid(row=0, column=k*2,
                                                     padx=(8,2), pady=6, sticky='e')
            v = tk.StringVar(value=str(val))
            self.geo_vars.append(v)
            e = tk.Entry(gf, textvariable=v, width=6)
            e.grid(row=0, column=k*2+1, padx=(0,6), pady=6)
            e.bind('<Return>',   lambda *_: self._update_plot())
            e.bind('<FocusOut>', lambda *_: self._update_plot())

        # ── Mode  .m: modePanel 'Position',[0.02 0.82 0.32 0.07]
        mf = ttk.LabelFrame(left, text="Mode")
        mf.place(relx=0.02, rely=0.12, relwidth=0.96, relheight=0.08)
        tk.Radiobutton(mf, text='Direct',  variable=self.mode, value='Direct',
                       command=self._cb_mode, bg='white').pack(side='left', padx=20, pady=4)
        tk.Radiobutton(mf, text='Inverse', variable=self.mode, value='Inverse',
                       command=self._cb_mode, bg='white').pack(side='left', padx=20, pady=4)

        # ── Elbow button + Show both  .m: Position [0.02 0.760 ...] [0.20 0.760 ...]
        ef = tk.Frame(left, bg='white')
        ef.place(relx=0.02, rely=0.21, relwidth=0.96, relheight=0.07)
        self.elbow_btn = tk.Button(ef, text='Config: 1', command=self._cb_elbow, width=14)
        self.elbow_btn.pack(side='left', padx=4, pady=4)
        self.show_both_var = tk.BooleanVar(value=False)
        tk.Checkbutton(ef, text='Show both', variable=self.show_both_var,
                       command=self._update_plot, bg='white').pack(side='left', padx=8)

        # ── Direct Mode Sliders  .m: directPanel 'Position',[0.02 0.58 0.32 0.17]
        # 3 rows inside panel with labels θ1 θ2 θ3
        df = ttk.LabelFrame(left, text="Direct Mode Sliders")
        df.place(relx=0.02, rely=0.29, relwidth=0.96, relheight=0.22)
        self.dir_vars       = []
        self.dir_entry_vars = []
        self.dir_sliders    = []
        self.dir_entries    = []
        dir_lbls = ['θ1', 'θ2', 'θ3']
        for k, (lbl, val) in enumerate(zip(dir_lbls, DEF_T)):
            tk.Label(df, text=lbl).grid(row=k, column=0, padx=4, pady=3, sticky='w')
            sv = tk.DoubleVar(value=val)
            self.dir_vars.append(sv)
            sl = tk.Scale(df, variable=sv, from_=-360, to=360,
                          orient='horizontal', resolution=0.1, showvalue=False,
                          command=lambda v, _k=k: self._on_dir_slider(_k))
            sl.grid(row=k, column=1, sticky='ew', padx=4, pady=3)
            df.columnconfigure(1, weight=1)
            ev = tk.StringVar(value=f"{val:.1f}")
            self.dir_entry_vars.append(ev)
            e = tk.Entry(df, textvariable=ev, width=7)
            e.grid(row=k, column=2, padx=4, pady=3)
            e.bind('<Return>',   lambda ev2, _k=k: self._on_dir_entry(_k))
            e.bind('<FocusOut>', lambda ev2, _k=k: self._on_dir_entry(_k))
            self.dir_sliders.append(sl)
            self.dir_entries.append(e)

        # ── Inverse Mode Sliders  .m: inversePanel 'Position',[0.02 0.40 0.32 0.17]
        ivf = ttk.LabelFrame(left, text="Inverse Mode Sliders")
        ivf.place(relx=0.02, rely=0.52, relwidth=0.96, relheight=0.22)
        inv_lbls = ['X', 'Y', 'φ']
        inv_mins = [-500, -500, -360]
        inv_maxs = [ 500,  500,  360]
        self.inv_vars       = []
        self.inv_entry_vars = []
        self.inv_sliders    = []
        self.inv_entries    = []
        for k, (lbl, val, mn, mx) in enumerate(zip(inv_lbls, DEF_INV, inv_mins, inv_maxs)):
            tk.Label(ivf, text=lbl).grid(row=k, column=0, padx=4, pady=3, sticky='w')
            sv = tk.DoubleVar(value=val)
            self.inv_vars.append(sv)
            sl = tk.Scale(ivf, variable=sv, from_=mn, to=mx,
                          orient='horizontal', resolution=0.1, showvalue=False,
                          command=lambda v, _k=k: self._on_inv_slider(_k))
            sl.grid(row=k, column=1, sticky='ew', padx=4, pady=3)
            ivf.columnconfigure(1, weight=1)
            ev = tk.StringVar(value=f"{val:.1f}")
            self.inv_entry_vars.append(ev)
            e = tk.Entry(ivf, textvariable=ev, width=7)
            e.grid(row=k, column=2, padx=4, pady=3)
            e.bind('<Return>',   lambda ev2, _k=k: self._on_inv_entry(_k))
            e.bind('<FocusOut>', lambda ev2, _k=k: self._on_inv_entry(_k))
            self.inv_sliders.append(sl)
            self.inv_entries.append(e)
        # Start in Direct mode: disable inverse  .m: set(invSl,'Enable','off')
        self._set_inv_state('disabled')

        # ── Animate  .m: 'Position',[20 210 285 26]
        self.anim_btn = tk.Button(left, text='Animate', command=self._cb_animate)
        self.anim_btn.place(relx=0.02, rely=0.76, relwidth=0.96, height=26)

        # ── Info text  .m: 'Position',[20 140 285 68]
        self.info_var = tk.StringVar()
        tk.Label(left, textvariable=self.info_var, justify='left',
                 anchor='nw', bg='white', font=('TkDefaultFont', 9),
                 wraplength=285).place(relx=0.02, rely=0.83,
                                       relwidth=0.96, relheight=0.15)

    # ── Canvas  .m: axes 'Position',[310 50 560 500]
    def _build_canvas(self):
        LP = 300
        self.fig = Figure(figsize=((self.W-LP)/100, self.H/100), dpi=100)
        self.ax  = self.fig.add_subplot(111)
        self.ax.set_aspect('equal'); self.ax.grid(True)
        self.ax.set_xlabel('X'); self.ax.set_ylabel('Y')
        self.ax.set_title('Planar RRR Linkage')
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.get_tk_widget().place(x=LP, y=0,
                                          width=self.W-LP, height=self.H)

    # ── Helpers ───────────────────────────────────────────────────────────────
    def _read_geo(self):
        vals = []
        for k, v in enumerate(self.geo_vars):
            try:
                x = float(v.get())
                if x <= 0:
                    x = DEF_L[k]
            except ValueError:
                x = DEF_L[k]
            vals.append(x)
        return {'L1': vals[0], 'L2': vals[1], 'L3': vals[2]}

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
            geo = self._read_geo()
            Px  = self.inv_vars[0].get()
            Py  = self.inv_vars[1].get()
            phi = math.radians(self.inv_vars[2].get())
            sol = rrr_inverse_kinematics(geo['L1'], geo['L2'], geo['L3'], Px, Py, phi, self.cfg_state)
            if sol['valid']:
                for k, attr in enumerate(['theta1', 'theta2', 'theta3']):
                    d = math.degrees(sol[attr])
                    self.dir_vars[k].set(round(d, 1))
                    self.dir_entry_vars[k].set(f"{d:.1f}")
        else:
            self._set_dir_state('disabled')
            self._set_inv_state('normal')
            geo = self._read_geo()
            t1  = math.radians(self.dir_vars[0].get())
            t2  = math.radians(self.dir_vars[1].get())
            t3  = math.radians(self.dir_vars[2].get())
            sol = rrr_direct_kinematics(geo['L1'], geo['L2'], geo['L3'], t1, t2, t3)
            P   = sol['Positions']['P']
            phi_deg = (math.degrees(sol['phi']) + 180) % 360 - 180
            for k, val in enumerate([P[0], P[1], phi_deg]):
                v = max([-500,-500,-360][k], min([500,500,360][k], val))
                self.inv_vars[k].set(round(v, 1))
                self.inv_entry_vars[k].set(f"{val:.1f}")
        self._update_plot()

    def _cb_elbow(self):
        self.cfg_state = -self.cfg_state
        self.elbow_btn.config(
            text='Config: 1' if self.cfg_state == 1 else 'Config: 2')
        self._update_plot()

    def _on_dir_slider(self, k):
        self.dir_entry_vars[k].set(f"{self.dir_vars[k].get():.1f}")
        self._update_plot()

    def _on_dir_entry(self, k):
        try:
            v = max(-360, min(360, float(self.dir_entry_vars[k].get())))
            self.dir_vars[k].set(v)
        except ValueError:
            pass
        self._update_plot()

    def _on_inv_slider(self, k):
        self.inv_entry_vars[k].set(f"{self.inv_vars[k].get():.1f}")
        self._update_plot()

    def _on_inv_entry(self, k):
        mins = [-500, -500, -360]
        maxs = [ 500,  500,  360]
        try:
            v = max(mins[k], min(maxs[k], float(self.inv_entry_vars[k].get())))
            self.inv_vars[k].set(v)
        except ValueError:
            pass
        self._update_plot()

    # ── Plot ──────────────────────────────────────────────────────────────────
    def _update_plot(self):
        try:
            geo = self._read_geo()
            R   = (geo['L1'] + geo['L2'] + geo['L3']) * 1.1
            lim = [-R, R, -R, R]

            self.ax.cla()
            self.ax.set_aspect('equal'); self.ax.grid(True)
            self.ax.set_xlabel('X'); self.ax.set_ylabel('Y')
            self.ax.set_title('Planar RRR Linkage')

            opts = {'clear_axes': True, 'show_labels': True, 'limits': lim}

            if self.mode.get() == 'Direct':
                t1 = math.radians(self.dir_vars[0].get())
                t2 = math.radians(self.dir_vars[1].get())
                t3 = math.radians(self.dir_vars[2].get())
                for k in range(3):
                    self.dir_entry_vars[k].set(
                        f"{math.degrees([t1,t2,t3][k]):.1f}")
                rrr_plot(geo, 'direct', [t1, t2, t3], opts, self.ax)
                sol = rrr_direct_kinematics(geo['L1'], geo['L2'], geo['L3'], t1, t2, t3)
                P   = sol['Positions']['P']
                phi_d = (math.degrees(sol['phi']) + 180) % 360 - 180
                info = (f"Direct mode:\n"
                        f"P=[{P[0]:.2f}; {P[1]:.2f}]  φ={phi_d:.2f}°")
            else:
                Px  = self.inv_vars[0].get()
                Py  = self.inv_vars[1].get()
                phi = math.radians(self.inv_vars[2].get())
                for k, val in enumerate([Px, Py, math.degrees(phi)]):
                    self.inv_entry_vars[k].set(f"{val:.1f}")
                opts['show_both'] = self.show_both_var.get()
                rrr_plot(geo, 'inverse', [Px, Py, phi, self.cfg_state], opts, self.ax)
                sol = rrr_inverse_kinematics(geo['L1'], geo['L2'], geo['L3'], Px, Py, phi, self.cfg_state)
                if sol['valid']:
                    info = (f"Inverse mode:\n"
                            f"Sol 1: θ1={math.degrees(sol['theta1']):.1f}°  "
                            f"θ2={math.degrees(sol['theta2']):.1f}°  "
                            f"θ3={math.degrees(sol['theta3']):.1f}°")
                else:
                    info = "Inverse mode: unreachable"
                if self.show_both_var.get():
                    sol2 = rrr_inverse_kinematics(geo['L1'], geo['L2'], geo['L3'], Px, Py, phi, -self.cfg_state)
                    if sol2['valid']:
                        info += (f"\nSol 2: θ1={math.degrees(sol2['theta1']):.1f}°  "
                                 f"θ2={math.degrees(sol2['theta2']):.1f}°  "
                                 f"θ3={math.degrees(sol2['theta3']):.1f}°")
                    else:
                        info += "\nSol 2: unreachable"

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
            self._th_off  = [v.get() for v in self.dir_vars]
            self._inv_off = [v.get() for v in self.inv_vars]
            self._anim_step()

    def _anim_step(self):
        if not self.animating:
            return
        dt    = time.time() - self._anim_t0
        speed = 30.0
        if self.mode.get() == 'Direct':
            rates = [1.0, 0.7, 0.5]
            for k in range(3):
                th = (self._th_off[k] + rates[k]*speed*dt + 180) % 360 - 180
                self.dir_vars[k].set(round(th, 1))
                self.dir_entry_vars[k].set(f"{th:.1f}")
        else:
            geo = self._read_geo()
            R   = geo['L1'] + geo['L2'] + geo['L3']
            Px  = self._inv_off[0] + R*0.3*math.sin(0.7*dt)
            Px  = max(-R, min(R, Px))
            self.inv_vars[0].set(round(Px, 1))
            self.inv_entry_vars[0].set(f"{Px:.1f}")
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
            for k in range(3):
                self.geo_vars[k].set(str(sess['geo'][k]))
            for k in range(3):
                self.dir_vars[k].set(sess['theta'][k])
                self.dir_entry_vars[k].set(f"{sess['theta'][k]:.1f}")
            self.inv_vars[0].set(sess['Px']); self.inv_entry_vars[0].set(f"{sess['Px']:.1f}")
            self.inv_vars[1].set(sess['Py']); self.inv_entry_vars[1].set(f"{sess['Py']:.1f}")
            self.inv_vars[2].set(sess['phi']); self.inv_entry_vars[2].set(f"{sess['phi']:.1f}")
            self.mode.set(sess.get('modeStr', 'Direct'))
            self.cfg_state = sess.get('configState', 1)
            self.elbow_btn.config(
                text='Config: 1' if self.cfg_state == 1 else 'Config: 2')
            self.show_both_var.set(sess.get('showBoth', False))
            self._cb_mode()
        except Exception as ex:
            messagebox.showerror("Open Error", str(ex))

    def _cb_save(self):
        path = filedialog.asksaveasfilename(
            defaultextension=".json",
            filetypes=[("JSON session", "*.json")],
            initialfile="rrr_session.json")
        if not path:
            return
        try:
            sess = {
                'geo':         [float(v.get()) for v in self.geo_vars],
                'theta':       [v.get() for v in self.dir_vars],
                'Px':          self.inv_vars[0].get(),
                'Py':          self.inv_vars[1].get(),
                'phi':         self.inv_vars[2].get(),
                'modeStr':     self.mode.get(),
                'configState': self.cfg_state,
                'showBoth':    self.show_both_var.get(),
            }
            with open(path, 'w') as f:
                json.dump(sess, f, indent=2)
        except Exception as ex:
            messagebox.showerror("Save Error", str(ex))

    def _cb_export_png(self):
        path = filedialog.asksaveasfilename(
            defaultextension=".png",
            filetypes=[("PNG Image", "*.png")],
            initialfile="rrr_linkage.png")
        if not path:
            return
        self.fig.savefig(path, dpi=150, bbox_inches='tight')

    def _cb_exit(self):
        if messagebox.askyesno("Exit", "Close the RRR GUI?"):
            self.root.destroy()

    def _cb_reset_view(self):
        geo = self._read_geo()
        R   = (geo['L1'] + geo['L2'] + geo['L3']) * 1.1
        self.ax.set_xlim(-R, R); self.ax.set_ylim(-R, R)
        self.canvas.draw_idle()

    def _cb_toggle_grid(self):
        lines = self.ax.xaxis.get_gridlines()
        self.ax.grid(not lines[0].get_visible() if lines else True)
        self.canvas.draw_idle()

    def run(self):
        self.root.mainloop()


if __name__ == '__main__':
    RRRGUI().run()
