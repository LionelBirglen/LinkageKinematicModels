# Common Linkages' Kinematic Models

Support files for the kinematic analysis of a few sample linkages and support files for the MEC6319 course at Polytechnique Montreal:

- the planar RRR serial chain
- the planar fourbar linkage
- the slider-crank linkage
- spherical RRR serial chain
- spherical fourbar linkage
- planar five-bar linkage
- Stephenson III six-bar linkage

Each linkage ships with a **direct (forward) kinematics** function, an **inverse kinematics** function, and an interactive **GUI** that lets you move and animate the mechanism. All MATLAB functions are documented with input/output descriptions and a runnable usage example in their header comments. The forward and inverse functions can be used as standalone and called by external functions (e.g. an optimization) without the gui.

## Repository Structure

```
LinkageKinematicModels/
├── Matlab/    # Direct/inverse kinematics + GUI for all 7 linkages
├── Python/    # Direct/inverse kinematics + GUI for the planar RRR chain
├── Media/     # GUI screenshots
└── LICENSE    # GNU AGPL v3.0
```

## MATLAB Functions

Run `xxx_gui.m` from MATLAB to open a graphical interface for a given linkage, or call the direct/inverse functions standalone. Every function file documents the exact input/output format and a worked example — see the function header for details.

| Linkage | Direct kinematics | Inverse kinematics | GUI |
|---|---|---|---|
| Planar RRR (3R serial chain) | `rrr_direct_kinematics(L1, L2, L3, theta1, theta2, theta3)` → `P1, P2, P3` | `rrr_inverse_kinematics(L1, L2, L3, Px, Py, phi, elbow_config)` → `theta1, theta2, theta3, success` | `rrr_gui()` |
| Planar fourbar | `fourbar_direct_kinematics(geo, theta)` → `sol` (2-entry struct array, both assembly modes) | `fourbar_inverse_kinematics(geo, alpha)` → `sol` (2-entry struct array, both assembly modes) | `fourbar_gui()` |
| Slider-crank | `slidercrank_direct_kinematics(a, b, phi, slider_angle, config)` → `x_slider, B, P` | `slidercrank_inverse_kinematics(a, b, x_slider, config, slider_angle)` → `phi, B` | `slidercrank_gui()` |
| Spherical RRR | `rrr_spherical_direct_kinematics(geometry, angles)` → `P, R_full, eul` | `rrr_spherical_inverse_kinematics(geometry, eul)` → `theta, ok` | `rrr_spherical_gui()` |
| Spherical fourbar | `fourbar_spherical_direct_kinematics(arcAngles, theta)` → `alpha, ok, P1, P2` | `fourbar_spherical_inverse_kinematics(arcAngles, alpha)` → `theta, ok, P1, P2` | `fourbar_spherical_gui()` |
| Planar five-bar | `fivebar_direct_kinematics(geo, theta)` → `sol` (struct array, one entry per assembly mode) | `fivebar_inverse_kinematics(geo, P_des)` → `invSol` (struct array, up to 4 solutions) | `fivebar_gui()` |
| Stephenson III six-bar | `stephensonIII_direct_kinematics(geo, theta_O)` → `sols` (struct array, up to 4 solutions) | `stephensonIII_inverse_kinematics(geo, thetaB)` → `sols` (struct array, up to 4 solutions) | `stephensonIII_gui()` |

Notes on each mechanism's geometry parameters:

- **Planar RRR**: `L1, L2, L3` are the three link lengths; `theta1` is the absolute angle of link 1, `theta2`/`theta3` are the relative angles between successive links. The inverse function also takes a desired end-effector pose `(Px, Py, phi)` and an `elbow_config` flag (+1 elbow-up, -1 elbow-down).
- **Planar fourbar**: geometry is now packed as `geo = [a, b, c, d, e, epsilon, delta]` — output link, coupler, input crank, ground link, distance to coupler point P, the angle of P relative to the coupler centerline, and a new `delta` angle locating the ground pivot C (`C = [d*cos(delta), d*sin(delta)]`, previously fixed along the x-axis). Both direct and inverse functions now always return a 2-entry struct array `sol` covering both assembly modes (open/crossed) in one call, rather than requiring a separate `config` argument to pick one. Each `sol(k)` holds the joint positions, link angles `phi`/`alpha`, point `P`, planar twists, and a `valid` flag.
- **Slider-crank**: `a` and `b` are the crank and coupler lengths, `slider_angle` sets the orientation of the prismatic joint axis, and `config` selects elbow-down (+1) or elbow-up (-1).
- **Spherical RRR**: `geometry = [alpha1, alpha2, alpha3]` are the spherical arc angles between successive joint axes (degrees); orientation is returned both as a rotation matrix and as Z-Y-X Euler angles. The inverse function returns both elbow-up and elbow-down branches, with an `ok` flag signaling singular configurations.
- **Spherical fourbar**: `arcAngles = [eta1, eta2, eta3, eta4]` are the four link arc lengths (degrees) of the spherical four-bar; both direct and inverse functions return the two possible solution branches.
- **Planar five-bar**: `geo = [a, b, c, d, e, alpha, h, eta]` packs the two input cranks `a` (O→A) and `d` (D→C), the two couplers `b` (A→B, treated as the output link) and `c` (C→B), the distance `e` between the two fixed ground pivots O and D oriented at angle `alpha`, and a point of interest P located at radial distance `h` from A and angle `eta` from the A→B direction. `theta = [theta1, theta2]` are the two input crank angles for the direct function; the inverse function instead takes a desired point location `P_des`. Both return a struct array (one entry per assembly mode) with joint positions, output-link orientation `phi`, point `P`, and a `valid` flag.
- **Stephenson III six-bar**: `geo = [OA, Bx, By, OC, CD, DA, BE, EM, DM, MP, eta, delta]` sets the two ground pivots (O at the origin, A on the x-axis, B at `[Bx, By]`), the input crank `OC`, the remaining link lengths `CD, DA, BE, EM, DM, MP`, and two angular offsets `eta`/`delta`. The direct function takes the input crank angle `theta_O`; the inverse function takes the desired output-link angle `thetaB`. Both return a struct array of valid assembly solutions, each with full joint positions, link angles, and a `valid` flag.

## Python Functions

The `Python/` folder currently provides a NumPy/Matplotlib port of the **planar RRR** chain only for now:

- `rrr_direct_kinematics.py` — same signature and behavior as its MATLAB counterpart
- `rrr_inverse_kinematics.py` — same signature and behavior as its MATLAB counterpart
- `rrr_gui.py` — interactive Matplotlib GUI with sliders/text boxes for link lengths, direct/inverse mode toggle, and elbow-up/elbow-down configuration switching

Requires `numpy` and `matplotlib`. Run with `python rrr_gui.py` from within the `Python/` folder. Python functions are lagging behind Matlab's since the former are a port of the latter. Main programming language for this repository is Matlab.

## GUI Screenshots

<a href="/LionelBirglen/LinkageKinematicModels/blob/main/Media/RRRGUI.png"><img src="https://github.com/LionelBirglen/LinkageKinematicModels/raw/main/Media/RRRGUI.png" alt="Planar RRR GUI" width="350"></a> <a href="/LionelBirglen/LinkageKinematicModels/blob/main/Media/fourbar_gui.png"><img src="https://github.com/LionelBirglen/LinkageKinematicModels/raw/main/Media/fourbar_gui.png" alt="Fourbar GUI" width="350"></a> <a href="/LionelBirglen/LinkageKinematicModels/blob/main/Media/SliderCrankGUI.png"><img src="https://github.com/LionelBirglen/LinkageKinematicModels/raw/main/Media/SliderCrankGUI.png" alt="Slider-Crank GUI" width="350"></a>

*Left to right: planar RRR linkage, planar fourbar linkage, and slider-crank mechanism GUIs. The fourbar GUI now displays both assembly-mode solutions side by side, with a File/Edit/View/Options/Help menu bar, session save/load, and timer-based animation.*

<a href="/LionelBirglen/LinkageKinematicModels/blob/main/Media/fivebar_gui.png"><img src="https://github.com/LionelBirglen/LinkageKinematicModels/raw/main/Media/fivebar_gui.png" alt="Five-Bar GUI" width="350"></a> <a href="/LionelBirglen/LinkageKinematicModels/blob/main/Media/StephensonIII_gui.png"><img src="https://github.com/LionelBirglen/LinkageKinematicModels/raw/main/Media/StephensonIII_gui.png" alt="Stephenson III GUI" width="350"></a>

*Left to right: planar five-bar linkage and Stephenson III six-bar linkage GUIs. Both display every valid assembly solution branch side by side and let you pick which ones to show and animate.*

## License

All files are released under the GNU Affero General Public License v3.0, see LICENSE file.

---

Prof. Lionel Birglen<br/>
Polytechnique Montreal<br/>
Contact: lionel.birglen@polymtl.ca
