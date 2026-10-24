# Project A: MATLAB code for the R-P-R-R robot (steps 1–4)

Industrial Robotics 2026/27, Prof. Giberti, group E. Written 2026-10-14; step 4 added 2026-10-24.

This folder holds the MATLAB part of the Project done so far:

1. The wavy surface ("the bucket") and the letter E on its inner wall.
2. A first sizing of the robot.
3. The kinematic model: direct and inverse kinematics, Jacobian, workspace and quality maps.
4. The trajectory: Home → arc → E → straight line → Home, as joint values over time.

**Every number in this code is a placeholder** unless this file says otherwise. The code is written so that every number can be changed in one place and everything re-run in under half a minute. The only decision taken so far is that our robot and the Epson C4 work in the same bucket (decided 2026-10-14).

---

## 1. Quick start

Open MATLAB (written and tested with R2025b), set `project/` as the current folder, and run the four scripts in order:

```matlab
step1_surface_E      % bucket + letter E           (~2 s)
step2_size_robot     % does the robot reach the E?  (~5 s)
step3_kinematics     % Jacobian, workspace, score   (~15 s)
step4_trajectory     % trajectory for the task      (~30 s)
```

Each script:

- prints a short summary in the Command Window,
- opens its figures and saves them as PNG in `figures/`,
- saves its results in a `.mat` file that later steps load.

If you change a number, re-run from the step where that number lives onward (see section 4).

---

## 2. Files

| File | What it is |
|---|---|
| `step1_surface_E.m` | Builds the surface and wraps the E onto it. Computes the pen direction at each point. Saves `step1_E_points.mat`. |
| `step2_size_robot.m` | Checks that our robot and the Epson C4 can reach every point of the E. Draws a design map of usable shoulder heights and rail angles. Saves `step2_robot.mat`. |
| `step3_kinematics.m` | Tests all kinematic functions. Draws the workspace (requirement ④) and the kinematic quality maps (requirement ⑤). Saves `step3_kinematics.mat`. |
| `step4_trajectory.m` | Plans the whole task in joint space: arc and line with cubic splines, the E with lines and parabolas. Checks joint limits, motor limits, pen on the wall and collisions. Saves `step4_trajectory.mat`. |
| `traj_lfpb.m` | Lines and parabolas through via points for one joint (start and stop at rest). |
| `traj_spline.m` | Cubic spline through via points in joint space, velocity 0 at both ends, with velocity and acceleration. |
| `traj_arc.m` | Points on the circular arc through three points. |
| `robot_params.m` | **The robot's dimensions, joint limits and score setting, in one place.** Every script reads them from here. |
| `RPRRdir.m` | Direct kinematics: joint values → pen tip, pen direction, joint positions, S = [x; y; z; θ]. |
| `RPRRinv.m` | Inverse kinematics: pen tip + pen direction → joint values. Returns `ok = false` if the point is out of reach. |
| `RPRRjac.m` | Jacobian J, with Ṡ = J·Q̇. |
| `RPRRjacP.m` | Time derivative of the Jacobian J̇, with S̈ = J̇·Q̇ + J·Q̈. |
| `RPRRscore.m` | Kinematic quality at one pose: 1/κ, the inverse condition number of the normalized Jacobian. |
| `figures/` | All plots, regenerated on every run. |

The function names and roles follow the course handout *SCARA kinematics* (28/09): `SCARAdir`, `SCARAinv`, `SCARAjac`, `SCARAjacP`. There is one difference: the dimensions are passed as a struct `L` (from `robot_params`) instead of a vector, because this robot has more of them.

---

## 3. Coordinates and conventions

Units: **mm and rad** everywhere inside the code. Degrees appear only in printed output and in plots.

**World frame** (as in the Project PDF, p.3):
- z is vertical, pointing up along the surface axis.
- The origin is on the axis at the bottom of the surface.
- The robot base sits at the origin. That placement is itself a placeholder.

**Surface:**

    r(z) = R0 + A·sin(2π·k·z/H),   k = 2.5,   0 ≤ z ≤ H

**Joints** (structure fixed by the Project PDF, p.2):

| Joint | Type | Meaning | Zero / positive direction |
|---|---|---|---|
| q1 | revolute | turns the whole robot about z | q1 = 0 → arm plane along +y (as drawn in the PDF); positive = counterclockwise seen from above |
| q2 | prismatic | slider position along the rail [mm] | measured from the axis at shoulder height, along the rail |
| q3 | revolute | angle of the q3 link relative to the rail | positive = the link tilts up |
| q4 | revolute | angle of the pen relative to the q3 link | positive = the pen tilts up |

Fixed geometry, set in `robot_params.m`:
- `h`: shoulder height, where the rail starts on the axis.
- `alpha`: rail angle above horizontal; negative means the rail tilts down.
- `L3`: q3 joint → q4 joint.
- `L4`: q4 joint → pen tip.

**Workspace vector:** S = [x; y; z; θ], where θ is the pen angle above horizontal. Four joints match four task coordinates. The pen's rotation about its own axis doesn't matter for the task.

**Pen direction n:** a unit vector perpendicular to the wall, pointing from the robot into the wall.

---

## 4. Where the numbers live (what you can change)

| Number(s) | Where | Status |
|---|---|---|
| Bucket: `R0 = 390`, `A/R0 = 0.045`, `H/R0 = 1.15` | top of `step1_surface_E.m` | placeholder |
| E: height 60% of H, bottom at 20% of H, width 0.6 × height, middle bar 0.8 × width, centred at +y | top of `step1_surface_E.m` | placeholder (the PDF does not specify letter size or position) |
| Points per wave on the vertical stroke: `pts_per_wave = 12` (gives 19 points) | `step1_surface_E.m` | placeholder; step 4 adds more where needed (see 6.10) |
| Robot: `h = 200`, `alpha = 10°`, `L3 = 250`, `L4 = 100`, solution `-1` | `robot_params.m` | placeholder (a working example from step 2) |
| Joint limits: q2 30–180 mm, q3 ±100°, q4 ±120° | `robot_params.m` | placeholder (real limits come from the mechanical design) |
| Score normalization: `Lc = 250` mm | `robot_params.m` | placeholder (see 6.6) |
| Slider minimum 30 mm, rail end at least 40 mm inside the wall | `step2_size_robot.m` | placeholder |
| C4 tool length (wrist centre → pen tip) 100 mm | `step2_size_robot.m` | placeholder |
| Joint speed / acceleration limits: 90°/s and 300°/s² (q1, q3, q4), 100 mm/s and 500 mm/s² (q2) | top of `step4_trajectory.m` | placeholder (not from a datasheet; set by the transmission, steps 5–6) |
| Home: `Q_home = [0; 70 mm; −50°; 80°]` | top of `step4_trajectory.m` | placeholder (see 6.10) |
| Pen speed on the E 50 mm/s, arc bulge 30 mm, 11 via points on arc and line, wall tolerance 0.5 mm | top of `step4_trajectory.m` | placeholder |

What to re-run after a change:
- Changed `step1_surface_E.m` → run steps 1, 2, 3 and 4.
- Changed `robot_params.m` or the step 2 check settings → run steps 2, 3 and 4.
- Changed only the settings in `step4_trajectory.m` → run step 4.

---

## 5. How to read the results

### Step 1: bucket and E

Console output:
- Wall distance from the axis, bucket height, height of one wave.
- **Steepest wall tilt**: how far the pen has to tip up or down to stay perpendicular to the wall (31.6° now).
- Size of the E and how many degrees around the axis it covers (23.7° now).
- **Pen perpendicular … largest error**: must be ~1e-16. Anything bigger means the normal computation is broken.

Figures:
- `step1_3d.png`: the half bucket, the E on it, and the pen directions as arrows.
- `step1_side_cut.png`: a cut through the vertical stroke. The arrows should be square to the wall everywhere.

### Step 2: can the robot reach it?

Console output:
- **Reaches N of 25 points**: must be 25 of 25.
- **Round trip error**: inverse then direct kinematics. Must be tiny (~1e-13).
- **Joint ranges over the E**: q1 ±11.9°, q2 31–141 mm (110 mm slider travel), q3 −58.5°…38.3°, q4 −80°…80° with the current numbers.
- **Rail end vs wall**: the rail must stay inside the bucket.
- **"Only q1 moves" on the three horizontal strokes**: q2–q4 change by ~1e-13. This confirms that the horizontal strokes are pure rotations about the axis (see 6.1).
- **Epson C4 shoulder-to-wrist distance**: must lie between 170 and 500 mm for every point. If it does, the same bucket works for the C4.

Figures:
- `step2_side_cut.png`: our robot (solid) and the simplified C4 (dashed), drawn at every third point of the vertical stroke. The thick light-blue bar is the part of the rail actually used.
- `step2_joint_values.png`: q2, q3 and q4 while tracing the vertical stroke. Here q2 goes back and forth and q4 swings ±80°; whether that costs time or torque shows up in steps 4–5.
- `step2_design_map.png`: x = shoulder height, y = rail angle, with L3 and L4 as in `robot_params`. A coloured square means "reaches the whole E, slider stays out of the column, rail stays inside the bucket"; the colour is the slider travel needed. The star is the current choice. Usable combinations form a diagonal band: the higher the shoulder, the more the rail has to tilt down.

### Step 3: kinematic model

Console output:
- **Six tests on 200 random poses**: J against numerical derivatives, J̇ against numerical derivatives, inverse → direct round trip, the determinant formula, velocity inversion, acceleration inversion. All must be ≤ ~1e-9. A larger value means a function was broken by an edit.
- **Score along the E**: 0.15–0.28 now. The slide threshold is 0.2.
- **Distance from the singularity**: q3 stays 32° away from ±90°.
- **All joints within limits**: true or false.
- **Best worst-case score in the design map** and where it is.

Figures:
- `step3_workspace.png` (requirement ④): where the pen tip can go in the arm plane, with the joint limits. The colour is how wide a range of pen angles that spot can be reached with (0–180°). The wall and the E are drawn on top; the E sits in the most flexible (yellow) region. The 3D workspace is this region turned about the axis over q1's range. Small stripy spots at the edges come from sampling the pen angle in 5° steps.
- `step3_score_map.png` (requirement ⑤): 1/κ at each spot with the pen horizontal. The red line marks 1/κ = 0.2.
- `step3_score_along_E.png`: 1/κ at each point of the E, with the pen really perpendicular to the wall. This is the number that matters for the task. The bottom of the E is the weakest point.
- `step3_design_score.png`: the step 2 design map again, but the colour is now the **lowest score along the E**. The middle of the band is best: rail at 4° instead of 10° raises the worst score from 0.15 to 0.19. `robot_params` was **not** changed, because this compares kinematics only; torque (step 5) may prefer something else.

### Step 4: trajectory

Console output:
- **Time of each part**: arc, the six strokes of the E, line. 39 s in total now.
- **Joint ranges** against the limits in `robot_params`. Now everything is inside except **q2, which goes down to 29.3 mm** on the lower half of the spine (limit 30). That is the same breach the dense check found in step 2; it must be fixed in `robot_params` (h or alpha), not in step 4.
- **Highest speed and acceleration as a fraction of the motor limit**: must be ≤ 1. Accelerations show 1.00 because the parabolas use the full limit by design.
- **Pen tip vs wall along the E**: must stay within `tol_wall` (now −0.07 to +0.49 mm). Positive = pen pushed into the wall.
- **Pen angle vs wall normal along the E**: 1.3° at most now.
- **Arc and line inside the bucket** and **all joints inside the wall and above the floor**: both must hold.

Figures:
- `step4_joints.png`: q, q̇, q̈ of all four joints against time, with the limits in red and the start of each part dotted. On the E the velocity is piecewise constant and the acceleration is ±limit or 0 (lines and parabolas).
- `step4_path_3d.png`: the pen tip's path in the bucket, with the via points of the arc and the line and the robot at Home.
- `step4_pen_on_wall.png`: distance and angle between pen and wall while drawing the E.
- `step4_pen_speed.png`: pen tip speed along the whole task.

`step4_trajectory.mat` holds `t`, `Q`, `Qd`, `Qdd` (every 1 ms), `part` (which part each sample belongs to), `names`, `t_start` and the settings. It is the input for the dynamics (step 5) and for the Simscape model.

The code was tested in GNU Octave 8, not in MATLAB. The MATLAB-only plot commands (`exportgraphics`, `sgtitle`, `xline`, `yline`) were not run.

---

## 6. Why it is designed this way

### 6.1 Wrapping the E onto the wall ("label on a can")
The PDF leaves the projection method to us (p.4). We draw the E flat and wrap it around the inside of the bucket. A flat horizontal distance becomes an angle around the axis (measured on the mean radius R0); a flat height becomes height z.

The reason: at a fixed height the wall is a circle around the robot's own axis. Each horizontal stroke is then a pure q1 rotation with q2–q4 frozen. The pen stays exactly on the wall and perpendicular to it, and two end points describe the whole stroke. Only the vertical stroke rides over the waves and needs many points. Step 2 confirms numerically that q2–q4 don't move on the horizontal strokes. A side effect: checking the vertical stroke in the side cut covers the whole E.

The E is placed to read correctly as seen from the robot (inside the bucket).

### 6.2 Pen perpendicular to the wall
The PDF requires it at every point (p.4). The wall's normal is n ∝ [cos φ, sin φ, −r′(z)], normalized. The script checks that n is at right angles to both directions lying in the wall: up the wall and around the axis.

### 6.3 Inverse kinematics in four steps (`RPRRinv`)
1. q1 turns the arm plane to face the target.
2. The pen direction is known, so step back one pen length L4 from the tip to get the q4 joint W.
3. The q3 joint lies on the rail at distance L3 from W, where a circle around W meets the rail line. There are two solutions; `L.sol = -1` takes the one nearer the axis. In step 2 the other one mostly pushed the slider through the wall.
4. Read q2, q3 and q4 from the points. `atan2` is used throughout, as the course warns (9/17).

Every inverse result is checked by feeding it into `RPRRdir`; the professor stressed checking every result (9/28).

### 6.4 Jacobian and singularity (`RPRRjac`, `RPRRjacP`)
The Jacobian is written in closed form, like `SCARAjac` in the handout. Velocities and accelerations are inverted as on Lez_02 p.22: Q̇ = J⁻¹Ṡ and Q̈ = J⁻¹(S̈ − J̇Q̇).

The determinant comes out as **det J = −ρ·L3·cos q3**, checked numerically to 3e-15. So the robot is singular only when:
- q3 = ±90°, meaning the q3 link is perpendicular to the rail (the circle in step 3 of 6.3 just touches the rail line); or
- the pen tip is on the axis (ρ = 0).

Step 3 reports how far the E stays from q3 = ±90°.

### 6.5 Workspace (requirement ④)
This uses the numerical method from Lez_02 p.18: sample points, solve the inverse kinematics, keep the points that have a solution within the joint limits. The robot's useful workspace depends on orientation as well as position. So each spot is coloured by the range of pen angles it can be reached with, as suggested in class for robots with an orientation coordinate (9/17). The map is drawn in one arm plane because robot and bucket are symmetric about the same axis; the professor pointed out that one plane is enough (9/28).

### 6.6 Kinematic quality score (requirement ⑤)
The score is the inverse condition number of J, 1/κ = σ_min/σ_max (Lez_04 p.23):
- 1 means the pen moves equally easily in every direction.
- 0 means it is stuck in one direction (singular).
- Above 0.2 is acceptable.

The condition number is used rather than det J because it does not depend on the robot's size (same slide).

**Normalization (open decision).** J mixes units: q2 is in mm while the other joints are in rad, and the θ row is in rad while the others are in mm. Lez_04 p.24 says that with motors of different nature, their performance must be normalized before comparing. For now `RPRRscore` uses a characteristic length `Lc`:
- a rotary joint turning by dq counts like a slider moving by Lc·dq;
- a pen tilt dθ counts like a pen-tip motion of Lc·dθ.

The absolute score depends on Lc: with Lc between 100 and 500 mm, the score along the E ranges from 0.08 to 0.28. But **the bottom of the E is the weakest point for every Lc**. Once motors and transmissions are chosen, the alternative is to normalize by each joint's maximum speed, as the slide suggests. This is worth asking the TA.

### 6.7 Design maps instead of guessing
Steps 2 and 3 sweep shoulder height and rail angle and show which combinations work and how well. This way the dimensions can be argued for in the report instead of just picked.

### 6.8 One bucket for both robots, and the Epson C4 check
Decision (2026-10-14): use the same surface for our robot and for the Epson C4 part, so the Epson part does not need its own resized bucket (PDF p.6 allows resizing if needed).

Step 2 checks the C4 with its real dimensions from the Epson C4 manipulator manual, Rev.20 (§2.4.1 p.15, §2.5.1 p.17):
- shoulder (Joint #2) 320 mm high and 100 mm out from the axis;
- upper arm 250 mm and forearm 250 mm to the wrist centre;
- wrist centre to flange 65 mm.

The C4 is reduced to its two arm links in the vertical plane. Our robot is **not** a copy of the C4. The structure is fixed by the PDF, and the C4's shoulder motors are 400 W with built-in gearboxes, while we are limited to ≤ 200 W and no commercial gearboxes.

### 6.9 One parameter file
`robot_params.m` is the single source of the robot's numbers, so steps 2, 3 and later steps can never disagree about the robot.

### 6.10 Trajectory (step 4)
**Interpretation.** The PDF (p.4–5) asks for a circular arc to the E and a straight line back, planned with cubic splines in joint space. We read the arc and the line as shapes in space: via points are taken on them, solved with `RPRRinv`, and a spline is put through the joint values. The pen follows the shape exactly at the via points and approximately in between. To confirm with the TA.

**Order of the E, no pen lifts.** Top bar right → left, spine top → middle, middle bar out and back, spine middle → bottom, bottom bar left → right. Each stroke starts and stops at rest, so the corners are hit exactly. The horizontal strokes need only their two end points (only q1 moves, see 6.1).

**Lines and parabolas on the E** (Lez_05, Craig ch. 7). All joints share the same segment times, set from the pen speed. Each joint blends with its own acceleration limit. If a parabola does not fit or a joint is too fast, the stroke's times are stretched by 5 % and it is planned again.

**Check points** (Lez_05). Between via points the pen leaves the wall, because a straight move in joint space is not a straight move on the wall. Where it leaves by more than `tol_wall`, a via point is added in the middle of that segment. With 12 points per wave the pen was up to 5 mm off the wall; with check points each half of the spine ends up with about 22 via points and stays within 0.5 mm. The arc and the line use the same idea for joint limits and the wall.

**Home and the narrow workspace.** With the current dimensions the robot reaches only a band about 80 mm wide along the wall (step 3 workspace). In addition, the top and bottom of the E sit exactly where the wall is steepest (31.6°, because E_z0 = 0.2·H and the top is at 0.8·H). A straight line from the bottom-right corner up to a high Home cuts through the wall, since the wall leans inward faster than the line. So Home is low: pen tip about 55 mm inside the wall, just above the bottom of the E, pen 40° up. Moving the E so its top and bottom sit on a wave crest (wall vertical there) would remove this constraint. That is a group decision.

**Pen angle on the arc and the line.** The task fixes the pen angle only on the E. In between, many angles are out of reach near the wall. At each via point the script tries every angle in 1° steps, keeps those that respect the joint limits (with a small margin, since the spline can overshoot) and keep every joint inside the wall, and picks the smoothest sequence (dynamic programming: smallest sum of squared angle changes).

**Timing of the splines.** The time between via points is what the slowest joint needs at full speed. Then the whole spline is stretched or shrunk so the fastest joint just meets its speed or acceleration limit (speed scales with 1/k, acceleration with 1/k²).

---

## 7. Known limitations (not done yet)

- **Only rough collision checks.** Step 4 checks that the joint points stay inside the wall and above the floor; the links between the joints and the column are not checked. The C4's elbow comes within about 5 cm of the wall in some poses (visible in `step2_side_cut.png`). This must be checked in the Epson simulator.
- **The C4 check is simplified:** wrist-centre distance only, no joint-angle limits, no wrist limits.
- **Joint limits are invented** to cover the E with margin; real ones come from the mechanical design.
- **The score map uses a horizontal pen**; the score along the E uses the real pen angles.
- **q2 breaches its 30 mm limit** (29.3 mm) on the lower half of the spine. Fix in `robot_params` and re-run steps 2–4.
- **Motor limits in step 4 are invented.** Real ones depend on the transmission (steps 5–6); re-run step 4 when they are known.
- **Step 4 tested in Octave only**, not in MATLAB.
- **Possible typo in the PDF:** it gives H/R0 ≥ 0.095, which may be a misprint for 0.95. The current value 1.15 is valid either way.

---

## 8. What comes next

| Step | Content | Project requirement | Can it be done with what has been taught? |
|---|---|---|---|
| 4 | **Done (2026-10-24), see 6.10.** Trajectory: Home → arc → E → straight line → Home. Cubic splines in joint space for Home ↔ E, lines + parabolas for the E. Each segment's time set by the motor limits. | ② | Yes. Choices made as placeholders: stroke order without pen lifts, stops at the corners, Home, motor limits. The group should confirm them. |
| 5 | Dynamics: motor torques along the trajectory, checked against the motors; vary payload (0.2–1 kg) and cycle time. | ①, ⑨, ⑩ | Torques yes (9/21, 10/01). Checking against motor curves and gravity compensation not taught yet. |
| 6 | Mechanical design in CAD, including our own speed reduction (belts, gears) and the real joint limits. | ③ (and ④ with real limits) | Can start; component lectures still to come. |
| 7 | Simscape Multibody model, same task, compare torques with the MATLAB model and explain the differences. | ⑥, ⑦, ⑧ | Started (2026-10-24): the kinematic chain is built in `robot_model.slx` and matches `RPRRdir` for two test poses. Next: drive it with `step4_trajectory.mat`, add masses and gravity. |
| 8 | Epson C4 in the Epson simulator: same task in the same bucket, plus a short document with joint and end-effector plots and a code description. | Project A | Not taught yet. |

Steps 2–5 form a loop. If a motor cannot deliver the torque in step 5, go back and change the dimensions (step 2), the transmission, or the time.

Deadlines (professor: "approximately", to be confirmed): Word report + PPT uploaded by **Fri 2026-12-11**; defense **Mon 2026-12-14** at Politecnico.

---

## 9. Sources

- Project brief: `../slides/Progetto2026_2027_IR.pdf` (p.2 robot, p.3–4 task, p.5 requests, p.6 Project A).
- Kinematics: `../slides/Lez_02_Kinematics.pdf` (p.15–23); handout `../slides/prctices/SCARA kinematics.pdf`.
- Ellipses and indices: `../slides/Lez_04_Ellipses.pdf` (p.22–24).
- Lecture notes (Chinese): `../上课重点.md`; study log and open-decision list: `../学习计划_20261009.md`.
- Epson C4 manipulator manual, Rev.20: https://files.support.epson.com/far/docs/epson_c4-series_robot_manual(r20).pdf
