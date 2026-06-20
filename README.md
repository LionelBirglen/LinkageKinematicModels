# Common Linkages' Kinematic Models

Support files for the kinematic analysis of a few sample linkages and support files for the MEC6319 course at Polytechnique Montreal:

- the planar RRR serial chain
- the planar fourbar linkage
- the slider-crank linkage
- spherical RRR serial chain
- spherical fourbar linkage

Each linkage ships with a **direct (forward) kinematics** function, an **inverse kinematics** function, and an interactive **GUI** that lets you move and animate the mechanism. All MATLAB functions are documented with input/output descriptions and a runnable usage example in their header comments. The forward and inverse functions can be used as standalone and called by external functions (e.g. an optimization) without the gui.

## Repository Structure

```
LinkageKinematicModels/
├── Matlab/    # Direct/inverse kinematics + GUI for all 5 linkages
├── Python/    # Direct/inverse kinematics + GUI for the planar RRR chain
├── Media/     # GUI screenshots
└── LICENSE    # GNU AGPL v3.0
```

## MATLAB Functions

Run `xxx_gui.m` from MATLAB to open a graphical interface for a given linkage, or call the direct/inverse functions standalone. Every function file documents the exact input/output format and a worked example — see the function header for details.

| Linkage | Direct kinematics | Inverse kinematics | GUI |
|---|---|---|---|
| Planar RRR (3R serial chain) | `rrr_direct_kinematics(L1, L2, L3, theta1, theta2, theta3)` → `P1, P2, P3` | `rrr_inverse_kinematics(L1, L2, L3, Px, Py, phi, elbow_config)` → `theta1, theta2, theta3, success` | `rrr_gui()` |
| Planar fourbar | `fourbar_direct_kinematics(geo, theta, config)` → `phi, alpha, A, r_P` | `fourbar_inverse_kinematics(geo, alpha, config)` → `theta, phi, A, r_P` | `fourbar_gui()` |
| Slider-crank | `slidercrank_direct_kinematics(a, b, phi, slider_angle, config)` → `x_slider, B, P` | `slidercrank_inverse_kinematics(a, b, x_slider, config, slider_angle)` → `phi, B` | `slidercrank_gui()` |
| Spherical RRR | `rrr_spherical_direct_kinematics(geometry, angles)` → `P, R_full, eul` | `rrr_spherical_inverse_kinematics(geometry, eul)` → `theta, ok` | `rrr_spherical_gui()` |
| Spherical fourbar | `fourbar_spherical_direct_kinematics(arcAngles, theta)` → `alpha, ok, P1, P2` | `fourbar_spherical_inverse_kinematics(arcAngles, alpha)` → `theta, ok, P1, P2` | `fourbar_spherical_gui()` |

Notes on each mechanism's geometry parameters:

- **Planar RRR**: `L1, L2, L3` are the three link lengths; `theta1` is the absolute angle of link 1, `theta2`/`theta3` are the relative angles between successive links. The inverse function also takes a desired end-effector pose `(Px, Py, phi)` and an `elbow_config` flag (+1 elbow-up, -1 elbow-down).
- **Planar fourbar**: geometry is packed as `geo = [a, b, c, d, e, epsilon]`, i.e. output link, coupler, input crank, ground link, distance to coupler point P, and the angle of P relative to the coupler centerline. `config` selects the open (+1) or crossed (-1) assembly mode.
- **Slider-crank**: `a` and `b` are the crank and coupler lengths, `slider_angle` sets the orientation of the prismatic joint axis, and `config` selects elbow-down (+1) or elbow-up (-1).
- **Spherical RRR**: `geometry = [alpha1, alpha2, alpha3]` are the spherical arc angles between successive joint axes (degrees); orientation is returned both as a rotation matrix and as Z-Y-X Euler angles. The inverse function returns both elbow-up and elbow-down branches, with an `ok` flag signaling singular configurations.
- **Spherical fourbar**: `arcAngles = [eta1, eta2, eta3, eta4]` are the four link arc lengths (degrees) of the spherical four-bar; both direct and inverse functions return the two possible solution branches.

## Python Functions

The `Python/` folder currently provides a NumPy/Matplotlib port of the **planar RRR** chain only for now:

- `rrr_direct_kinematics.py` — same signature and behavior as its MATLAB counterpart
- `rrr_inverse_kinematics.py` — same signature and behavior as its MATLAB counterpart
- `rrr_gui.py` — interactive Matplotlib GUI with sliders/text boxes for link lengths, direct/inverse mode toggle, and elbow-up/elbow-down configuration switching

Requires `numpy` and `matplotlib`. Run with `python rrr_gui.py` from within the `Python/` folder.

## GUI Screenshots

<a href="/LionelBirglen/LinkageKinematicModels/blob/main/Media/RRRGUI.png"><img src="https://github.com/LionelBirglen/LinkageKinematicModels/raw/main/Media/RRRGUI.png" alt="Planar RRR GUI" width="350"></a> <a href="/LionelBirglen/LinkageKinematicModels/blob/main/Media/FourbarGUI.png"><img src="https://github.com/LionelBirglen/LinkageKinematicModels/raw/main/Media/FourbarGUI.png" alt="Fourbar GUI" width="350"></a> <a href="/LionelBirglen/LinkageKinematicModels/blob/main/Media/SliderCrankGUI.png"><img src="https://github.com/LionelBirglen/LinkageKinematicModels/raw/main/Media/SliderCrankGUI.png" alt="Slider-Crank GUI" width="350"></a>

*Left to right: planar RRR linkage, planar fourbar linkage, and slider-crank mechanism GUIs. Each lets you switch between direct and inverse kinematics modes, toggle elbow/crossed configurations, view the alternate solution branch, and animate the motion.*

## License

All files are released under the GNU Affero General Public License v3.0, see LICENSE file.

Prof. Lionel Birglen<br/>
Polytechnique Montreal<br/>
Contact: lionel.birglen@polymtl.ca
