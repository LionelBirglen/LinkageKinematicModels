import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, TextBox, RadioButtons
from rrr_direct_kinematics import rrr_direct_kinematics
from rrr_inverse_kinematics import rrr_inverse_kinematics

# Initial parameters
L1, L2, L3 = 57.0, 46.0, 51.0
elbow_config = +1
mode = 'Direct'

# Create figure and plot
fig, ax = plt.subplots(figsize=(6,6))
plt.subplots_adjust(left=0.3, bottom=0.4)
ax.set_aspect('equal')
ax.set_xlim(-200, 200)
ax.set_ylim(-200, 200)

# Link line objects with distinct colors
link1_line, = ax.plot([], [], lw=3, color='red')
link2_line, = ax.plot([], [], lw=3, color='green')
link3_line, = ax.plot([], [], lw=3, color='blue')
joint_markers, = ax.plot([], [], 'o', color='black')

# End-effector pose text under sliders (figure coordinates)
fig_text = fig.text(0.3, 0.08, '', fontsize=10, va='bottom')

# Mode radio buttons
ax_mode = plt.axes([0.02, 0.7, 0.2, 0.15])
mode_radio = RadioButtons(ax_mode, ['Direct', 'Inverse'])

# Elbow config radio buttons
ax_elbow = plt.axes([0.02, 0.5, 0.2, 0.15])
elbow_radio = RadioButtons(ax_elbow, ['Elbow-up', 'Elbow-down'])

# TextBoxes for link lengths
ax_L1 = plt.axes([0.06, 0.92, 0.1, 0.05])
tb_L1 = TextBox(ax_L1, 'L1 ', initial=str(L1))
ax_L2 = plt.axes([0.22, 0.92, 0.1, 0.05])
tb_L2 = TextBox(ax_L2, 'L2 ', initial=str(L2))
ax_L3 = plt.axes([0.38, 0.92, 0.1, 0.05])
tb_L3 = TextBox(ax_L3, 'L3 ', initial=str(L3))

# Sliders for Direct mode (θ1, θ2, θ3)
ax_t1 = plt.axes([0.3, 0.25, 0.6, 0.03])
ax_t2 = plt.axes([0.3, 0.20, 0.6, 0.03])
ax_t3 = plt.axes([0.3, 0.15, 0.6, 0.03])
s_t1 = Slider(ax_t1, 'θ1 (deg)', -180, 180, valinit=39)
s_t2 = Slider(ax_t2, 'θ2 (deg)', -180, 180, valinit=37)
s_t3 = Slider(ax_t3, 'θ3 (deg)', -180, 180, valinit=40)

# Sliders for Inverse mode (Px, Py, φ)
ax_px = plt.axes([0.3, 0.25, 0.6, 0.03])
ax_py = plt.axes([0.3, 0.20, 0.6, 0.03])
ax_phi = plt.axes([0.3, 0.15, 0.6, 0.03])
s_px = Slider(ax_px, 'Px', -200, 200, valinit=100)
s_py = Slider(ax_py, 'Py', -200, 200, valinit=0)
s_phi = Slider(ax_phi, 'φ (deg)', -180, 180, valinit=0)

def update(val=None):
    global L1, L2, L3, elbow_config, mode
    # Update link lengths
    try:
        L1 = float(tb_L1.text)
        L2 = float(tb_L2.text)
        L3 = float(tb_L3.text)
    except:
        pass

    if mode == 'Direct':
        # Direct FK
        t1 = np.deg2rad(s_t1.val)
        t2 = np.deg2rad(s_t2.val)
        t3 = np.deg2rad(s_t3.val)
        P1, P2, P3 = rrr_direct_kinematics(L1, L2, L3, t1, t2, t3)
        link1_line.set_data([0, P1[0]], [0, P1[1]])
        link2_line.set_data([P1[0], P2[0]], [P1[1], P2[1]])
        link3_line.set_data([P2[0], P3[0]], [P2[1], P3[1]])
        joint_markers.set_data([0, P1[0], P2[0], P3[0]], [0, P1[1], P2[1], P3[1]])
        fig_text.set_text(f"X={P3[0]:.2f}, Y={P3[1]:.2f}, φ={np.rad2deg(t1+t2+t3):.1f}°")
    else:
        # Inverse IK
        Px = s_px.val
        Py = s_py.val
        phi = np.deg2rad(s_phi.val)
        t1, t2, t3, ok = rrr_inverse_kinematics(L1, L2, L3, Px, Py, phi, elbow_config)
        if ok:
            P1 = np.array([L1*np.cos(t1), L1*np.sin(t1)])
            P2 = P1 + np.array([L2*np.cos(t1+t2), L2*np.sin(t1+t2)])
            P3 = np.array([Px, Py])
            link1_line.set_data([0, P1[0]], [0, P1[1]])
            link2_line.set_data([P1[0], P2[0]], [P1[1], P2[1]])
            link3_line.set_data([P2[0], P3[0]], [P2[1], P3[1]])
            joint_markers.set_data([0, P1[0], P2[0], P3[0]], [0, P1[1], P2[1], P3[1]])
            fig_text.set_text(f"θ1={np.rad2deg(t1):.1f}°, θ2={np.rad2deg(t2):.1f}°, θ3={np.rad2deg(t3):.1f}°")
        else:
            fig_text.set_text("No IK solution")
    fig.canvas.draw_idle()

def mode_func(label):
    global mode
    mode = label
    # Show/hide sliders
    for s in [s_t1, s_t2, s_t3]:
        s.ax.set_visible(mode == 'Direct')
    for s in [s_px, s_py, s_phi]:
        s.ax.set_visible(mode == 'Inverse')
    # Disable elbow when direct
    if mode == 'Direct':
        ax_elbow.patch.set_alpha(0.3)
    else:
        ax_elbow.patch.set_alpha(1.0)
    update()

def elbow_func(label):
    global elbow_config, mode
    if mode != 'Inverse':
        return
    elbow_config = +1 if label == 'Elbow-up' else -1
    update()

# Connect callbacks
mode_radio.on_clicked(mode_func)
elbow_radio.on_clicked(elbow_func)
for w in [s_t1, s_t2, s_t3, s_px, s_py, s_phi]:
    w.on_changed(update)
for tb in [tb_L1, tb_L2, tb_L3]:
    tb.on_submit(lambda text: update())

# Initialize
mode_func('Direct')
update()
plt.show()