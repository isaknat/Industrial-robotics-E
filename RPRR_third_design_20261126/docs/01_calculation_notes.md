# Calculation notes: method, course basis, procedure and results

Project A, group E, Industrial Robotics 2026/27 (Prof. Giberti). State of 2026-11-26, third mechanical design.

This file explains, step by step, what was calculated, which lecture or exercise each method comes from, how the scripts do it, how each result is checked, and what the numbers are now. All numbers are from the runs of 2026-11-26 (logs in `project/logs/`). The code is in `project/`.

Lecture files are cited by their file name and page, as they are on the course site (for example "Lez_05 p.25"). Dates such as "(10/22)" are the dates of the lessons in which the professor or the assistant said something that we used.

## 0. The task and the requirements

The brief (`Progetto2026_2027_IR.pdf`) fixes the robot structure (p.2: revolute, prismatic, revolute, revolute), the task (p.3–4: write a letter on the inside of a wavy cylindrical surface, pen perpendicular to the surface) and ten requests (p.5). The brief does not number them; we number them in the order of the list, and the code comments use the same numbers.

| No. | Request (brief p.5, shortened) | Answered in |
|---|---|---|
| 1 | Kinematic and dynamic model of the robot in MATLAB | steps 2 and 3 (kinematics), step 5 (dynamics) |
| 2 | Plan the trajectory of the task, considering the motor limits | step 4 |
| 3 | Mechanical design: 3D models, drawings of parts and assemblies, exploded view | step 6 (CAD, see `02_mechanical_design.md`) |
| 4 | Map the effective workspace, with the mechanical limits of the design | step 3 (joint limits from the CAD) |
| 5 | Map the quality of the kinematics with an index of our choice | step 3, step 3b |
| 6 | Kinematic and dynamic model in Simscape Multibody | step 7, step 7b |
| 7 | Carry out the task with the MATLAB model and the Simscape model | step 7, step 7b |
| 8 | Compare the motor torques from MATLAB with those from Simscape | step 7, step 7b (the professor also asked on 9/28 for an explanation of why they differ or not) |
| 9 | Compute the motor torques and check them against the motor performance | step 5, step 7b |
| 10 | Analyse the performance at varying payload (0.2 to 1 kg) and cycle time | step 5 |

Project A (brief p.6) adds: the same task on the Epson C4 with the Epson programming software, the surface imported as a scenario, and a short document with plots of the joint variables and of the end-effector motion and a description of the code. That part (step 8) has not been started.

Motors allowed by the brief (p.2): Mitsubishi HG-KR and HG-MR up to 200 W, no commercial gearboxes. Motor data: `motor_catalog.m`, from the Mitsubishi *Servo Motor Instruction Manual (Vol. 3)*, chapter 6, section 6.3.1.

## 1. Conventions

- Units in the code: mm and rad for kinematics and trajectory; SI inside the dynamics functions (the scripts convert).
- World frame (brief p.3): z vertical along the axis of the surface, origin on the axis at the bottom of the surface. The robot base sits at the origin.
- Surface: r(z) = R0 + A·sin(2π·k·z/H), with R0 = 390 mm, A = 17.55 mm (A/R0 = 0.045), H = 448.5 mm (H/R0 = 1.15), k = 2.5.

| Joint | Type | Meaning | Zero and positive direction |
|---|---|---|---|
| q1 | revolute | turns the whole robot about the vertical axis | zero = arm plane along +y; positive = counter-clockwise seen from above |
| q2 | prismatic | slider position along the horizontal rail [mm] | distance of the q3 axis from the q1 axis |
| q3 | revolute | angle of the arm (q3 link) to the rail | positive = arm tilts up |
| q4 | revolute | angle of the pen to the arm | positive = pen tilts up |

Fixed dimensions (`robot_params.m`): shoulder height h = 225 mm, rail angle alpha = 0 (horizontal), L3 = 300 mm (q3 axis to q4 axis), L4 = 50 mm (q4 axis to pen tip, the TCP).

Task vector: S = [x; y; z; θ], θ = pen angle above horizontal. Four joints, four task coordinates. The rotation of the pen about its own axis does not matter.

Function names follow the course handouts *SCARA kinematics* and *SCARA Dynamics*: `RPRRdir`, `RPRRinv`, `RPRRjac`, `RPRRjacP`, `RPRRdirdin`, `RPRRjacdin`, `RPRRjacPdin`, `RPRRdyn` play the roles of `SCARAdir`, `SCARAinv`, and so on. The dimensions are passed as a struct `L` from `robot_params`.

## 2. Step 1: the surface and the letter (`step1_surface_E.m`)

**Idea.** The brief leaves the projection of the letter to us (p.4). We draw the E flat and wrap it around the inside of the bucket like a label on a can: a flat horizontal distance becomes an angle about the axis (measured on the mean radius R0), a flat height becomes the height z.

**Why.** At a fixed height the wall is a circle about the robot's own q1 axis. Each horizontal stroke of the E is then a pure rotation of q1 with q2, q3, q4 frozen, and its two end points describe the whole stroke. Only the vertical stroke runs over the waves and needs many points.

**Pen direction.** The brief requires the pen perpendicular to the wall at every point (p.4). The normal is n ∝ [cos φ, sin φ, −r′(z)], normalised. The script checks that n is at right angles to both directions lying in the wall.

**Our choices** (the brief gives no size or position): letter height 60 % of H, bottom at 20 % of H, width 0.6 × height, middle bar 0.8 × width, centred on +y; 24 points per wave on the vertical stroke.

**Result.** Wall 372 to 408 mm from the axis; E 269 mm tall, 161 mm wide, from z = 90 to 359 mm, spanning 23.7° about the axis; steepest wall tilt 31.6°; 43 points (37 on the vertical stroke, 2 on each bar); largest error of "perpendicular" 1.1e-16.

## 3. Step 2: can the robot reach the letter? (`step2_size_robot.m`)

**Idea.** Inverse kinematics of every letter point, then checks: all points reached, slider not inside the column, rail end inside the bucket.

**Inverse kinematics in four steps** (`RPRRinv`, built like `SCARAinv`, Lez_02 p.15–23):
1. q1 turns the arm plane to face the target.
2. The pen direction is known, so stepping back one pen length L4 from the tip gives the q4 joint W.
3. The q3 joint lies on the rail at distance L3 from W: a circle about W cut with the rail line. Of the two solutions `L.sol = -1` takes the one nearer the axis.
4. q2, q3, q4 are read from these points with `atan2` (the course's warning of 9/17).

Every inverse result is fed back into `RPRRdir` (the professor asked on 9/28 that every result be checked).

**Result.** 43 of 43 points reached; round trip error 1.2e-13. Joint ranges over the letter: q1 ±11.9°, q2 32.8 to 94.6 mm (62 mm of slider travel), q3 −32.6 to 32.2°, q4 −63.8 to 64.1°. On the three horizontal strokes q2, q3, q4 change by about 1e-13, which confirms that they are pure q1 rotations.

**Epson C4.** The same bucket is used for the C4 part (decided 2026-10-14; the brief p.6 allows resizing). The C4 is reduced to its two arm links with the dimensions of the Epson C4 manual Rev.20 (§2.4.1, §2.5.1): the shoulder-to-wrist distance needed is 173 to 349 mm, inside the allowed 170 to 500 mm. This is a reach check only.

## 4. Step 3: kinematic model, workspace and quality (`step3_kinematics.m`)

**Jacobian** (`RPRRjac`, `RPRRjacP`). Written in closed form like `SCARAjac`. Velocities and accelerations are inverted as on Lez_02 p.22: Q̇ = J⁻¹·Ṡ and Q̈ = J⁻¹·(S̈ − J̇·Q̇).

**Singularities.** The determinant is det J = −ρ·L3·cos q3 (ρ = distance of the pen tip from the axis). The robot is singular only when q3 = ±90° (arm perpendicular to the rail) or when the pen tip is on the axis.

**Tests** on 200 random poses: J and J̇ against numerical derivatives (2e-10, 6e-10), inverse-direct round trip (1e-13), the determinant formula (5e-16), velocity and acceleration inversion (3e-14, 7e-15).

**Workspace (request 4).** The numerical method of Lez_02 p.18: sample points, solve the inverse kinematics, keep the points with a solution inside the joint limits. Because the task also fixes the pen angle, each spot is coloured by the range of pen angles it can be reached with (suggested in class on 9/17 for robots with an orientation coordinate). One arm plane is enough because robot and bucket share the same axis (9/28). Figure `step3_workspace.png`.

**Kinematic quality (request 5).** The score is the inverse condition number of the Jacobian, 1/κ = σ_min/σ_max (Lez_04 p.22–23): 1 means the pen moves equally easily in every direction, 0 means singular, and above 0.2 is the usual acceptable value. The assistant said on 11/16 that 0.2 is the common value in the literature and not a hard requirement. The condition number is used instead of det J because it does not depend on the robot's size.

J mixes units (q2 in mm, the other joints in rad; the θ row in rad). Lez_04 p.24 says to normalise before comparing motors of different nature. `RPRRscore` uses a characteristic length Lc = 250 mm: a rotary joint turning by dq counts like a slider moving Lc·dq, and a pen tilt dθ counts like a tip motion Lc·dθ. The absolute score depends on Lc; this is our own choice and is stated as such.

**Result.** Along the letter 1/κ is 0.29 to 0.34; q3 stays 57° away from ±90°; all joints inside their limits. Figures `step3_score_map.png`, `step3_score_along_E.png`.

## 5. Step 3b: choosing the link lengths (`step3b_design_search.m`)

**Idea.** Instead of guessing the dimensions, a coarse grid over h, alpha, L3 and L4 is evaluated: a design is kept if it reaches the whole letter, the slider stays out of the column, the rail stays inside the bucket and the q4 joint stays at least 30 mm from the wall. The designs are ranked by their lowest score along the letter, taking the worst of three normalisation lengths (Lc = 150, 250, 400 mm) so that the ranking does not depend on one choice of Lc.

**Result.** 7605 combinations, 564 can be built. Longer L3 and shorter L4 score better; L3 of 325 mm and more never works because the q3 joint would sit inside the column. The best design is the one we use: h = 225 mm, alpha = 0°, L3 = 300 mm, L4 = 50 mm, with lowest score 0.24 / 0.29 / 0.25 for the three Lc and only 62 mm of slider travel. The search looks at kinematics only.

## 6. Step 4: trajectory of the whole task (`step4_trajectory.m`)

**What the brief fixes (p.4).** Home to the first letter point on a circular path, last letter point to Home on a straight path, both with cubic splines in joint space; the letter with "lines and parabolas". Home, the stroke order, what happens at corners and the pen angle away from the letter are left to us.

**Letter** (`traj_lines_parabolas.m`, Lez_05 p.25–29, exercise Ex3.2). The via points are the corners plus the points of the vertical stroke; inverse kinematics gives their joint values. Each segment gets the shortest rest-to-rest time of its slowest joint (`traj_min_time.m`, Lez_05 p.14–23, Ex3.1), so all joints reach every via point together; this is the rule of the course code `lines_parabolas_main.m`. A run starts and ends at rest exactly on its end points; the points in between are only passed nearby, which is the known property of the method.

**Corners.** Three ways are computed and compared on every run: stop at every corner, stop only where the pen turns back, never stop. We stop at every corner: it takes the same time (5.37 s before the stretch described below) and never stopping leaves 16.8 mm of the middle bar undrawn.

**Home moves** (`traj_cubic_spline.m`, Lez_05 p.30–34). Points are taken on the arc and on the line, inverse kinematics gives joint values, and a cubic spline joins them, starting and ending at rest. The total time is the shortest one for which no joint exceeds its speed or acceleration limit, using the time scaling of Lez_05 p.11 (speeds scale with 1/T, accelerations with 1/T²). The true tip path is then compared with the ideal arc and line (Lez_05 p.35–38).

**Pen angle on the Home moves** (our own method, not from the course). The pen must be perpendicular to the wall only on the letter. At every point of the Home moves all pen angles from −120° to 120° are tried, the ones that break a joint limit or bring the arm too close to the wall are discarded, and the sequence with the smallest sum of squared angle steps is kept (dynamic programming).

**Speed and acceleration limits.** The professor's rule (10/22): maximum acceleration = motor torque / inertia seen by the motor. The assistant (11/09): choose a value, size the motor, check, repeat. The loop was run: first guess 2 rad/s and 8 rad/s², the motors of step 5 could give 6.3 rad/s and about 80 rad/s², so the limits were set to 3 rad/s and 30 rad/s² (slider 300 mm/s and 3000 mm/s²), about half of what the motors can give.

**Third design: the wrist motor sits on the slider.** The q4 motor drives the wrist through a pulley on the q3 axis and a long belt along the arm, so the motor turns by 6·q4 + 3·q3 (see section 9). Its "joint-side" coordinate is w = q4 + 0.5·q3. The trajectory is planned as above and then slowed down by one factor so that w also respects q4's limits: factor = max(1, max|ẇ|/3, sqrt(max|ẅ|/30)) = √1.5 = 1.225. Without this stretch the cycle would be 8.72 s.

**Third design: slider bound that depends on q3 and q4.** With the slider far out, the fork, the wrist pulley, the long belt and the pen carrier can touch the wavy wall, and with the arm lowered the tube can touch the deck. `robot_coupled_limits.m` gives the largest and smallest q2 for each (q3, q4); every sample of the trajectory is checked against it. Figure `Fusion/v3/coupled_limits.png`.

**Result.** Stroke order A (top bar right to left, down to the middle, middle bar out and back, down, bottom bar): 41 via points, 850 mm of path, 129 mm drawn twice. Home: pen tip 350 mm from the axis at z = 120 mm, pen angle −48° (q1 −11.9°, q2 24.3 mm, q3 −13.1°, q4 −34.9°).

| Part | Time |
|---|---|
| Home to letter (arc) | 1.97 s |
| Letter | 6.57 s |
| Letter to Home (line) | 2.13 s |
| Whole cycle | 10.67 s (5339 samples) |

Checks on the whole cycle: joints inside their limits (q2 1.9 to 94.6 mm of 0 to 141; q3 −32.6 to 32.6° of −42 to 45; q4 −81.0 to 64.1° of ±131); largest speed / limit 0.82, 0.47, 0.22, 0.45; largest acceleration / limit 0.67 for all four; arm at least 24.7 mm from the wall and 63.5 mm above the floor; tip within 0.02 mm of the ideal arc and 0.03 mm of the ideal line. Between the via points of the vertical stroke the tip strays up to 0.84 mm from the ideal letter and dips up to 0.95 mm into the wall; the pen is up to 0.25° off perpendicular. The pen has 3 mm of spring travel for this.

## 7. Step 5: dynamic model and motor sizing (`step5_dynamics.m`, `RPRRdyn.m`)

**Dynamic model (request 1).** The method of Lez_03 p.4–7: collect every mass in an extended vector S_e, write the extended Jacobian J_e, treat inertia as a force, and get the joint loads from F_q = −J_eᵀ·(−M·S̈_e + F_se). The functions follow the handout *SCARA Dynamics*.

**Point masses** (Lez_03 p.16–19). Each body is replaced by point masses that keep its mass, its first moment along the link and its moment of inertia about the q3 axis. With point masses no inertia tensor is needed although the robot moves in 3D: each point has x, y, z and its mass three times on the diagonal of M. The only moment of inertia is J1, for the parts that turn with q1 without sliding. The points and J1 are computed in `robot_masses.m` from the Fusion mass properties (`Fusion/v3/step7_bodies/mass_properties.json`).

| Quantity | Value |
|---|---|
| Slider (D_03) | 2.612 kg |
| Arm, long belt and q3 pulley (D_04) | 0.772 kg |
| Pen head (D_05) | 0.067 kg |
| J1 (column, rail and slider about the q1 axis) | 0.0542 kg·m² |
| Payload at the pen tip | 0.2 to 1 kg (1 kg for the motor choice) |

**Reusing the kinematics.** A point at a given fraction of the arm or of the tool is the "pen tip" of a shorter robot, so its position and its rows of J_e come from `RPRRdir`, `RPRRjac`, `RPRRjacP` with shortened lengths. The result is still checked against numerical differences (the check shown on Lez_10 p.35).

**Tests.** Extended Jacobian and its derivative against numerical differences (1e-10); joint loads at rest against weight × lever arm worked out by hand (9e-16; at Home q3 holds 3.89 N·m and q4 0.33 N·m); work of the joints against the change of kinetic plus potential energy over the cycle (0.016 J out of 4.13 J).

**Motor and reduction (request 9)**, Lez_08 MotorSizingNotes p.21–28 (lesson of 10/22). For each joint the load C_r* is the joint torque from the dynamic model. Then:

- β = 2·[RMS(ω̇_r)·RMS(C_r*) + mean(ω̇_r·C_r*)] for the joint, and α = C_n²/J_m for each motor. A motor can drive the joint if α > β.
- The usable transmission ratios τ lie between τ_min and τ_max (p.26), with τ_opt = sqrt(J_m·RMS(ω̇_r)/RMS(C_r*)) (p.25) and τ ≥ ω_r,max/ω_m,max (p.27). For the slider τ is in metres per radian (p.32).
- The three conditions of p.10 are then checked directly with C_m = τ·C_r* + J_m·ω̇_r/τ: RMS torque below the rated torque, peak torque below the maximum torque, speed below the maximum speed. We keep the first two below 0.8. The check plot of p.29 is `step5_motor_checks.png`.

β of the joints: q1 36.3, q2 2.05, q3 12.4, q4 3.85 W/s. α of the smallest motor (HG-KR053, 50 W): 5689 W/s. So the 50 W motor is enough for every joint; what matters is the reduction.

**Why the reductions are large.** The joints move slowly and q3 mostly holds weight, so τ_opt is a reduction of several hundred to one (q1 1:234, q3 1:716, q4 1:146), far more than belt stages can give. We take the largest reduction that we can build with stock pulleys.

**Choice** (all four fixed by us; HG-KR053: rated 0.16 N·m, maximum 0.56 N·m, 3000 / 6000 r/min, rotor 4.5e-6 kg·m², 0.34 kg):

| Joint | Motor | Reduction | RMS / rated | Peak / maximum | Top motor speed |
|---|---|---|---|---|---|
| q1 | HG-KR053 | 1:50 (18→120, then 16→120 teeth) | 0.20 | 0.19 | 1170 r/min |
| q2 | HG-KR053 | ball screw, 10 mm per turn | 0.02 | 0.05 | 841 r/min |
| q3 | HG-KR053 | 1:42.19 (16→90, then 16→120 teeth) | 0.54 | 0.30 | 268 r/min |
| q4 | HG-KR053 | 1:6 (20→60, then 20→40 teeth) | 0.45 | 0.26 | 60 r/min |

Joint loads with 1 kg: q1 peak 5.00 N·m; q2 peak 12.4 N; q3 peak 7.30 N·m, of which 4.15 N·m is weight; q4 peak 0.86 N·m, of which 0.49 N·m is weight. Least reduction for the RMS check at 100 % of the rated torque with the 50 W motor: q1 1:9.4, q3 1:22.9, q4 1:2.7.

**Payload and cycle time (request 10).** The joint loads are linear in the masses, and their moving part scales with 1/T² when the same path is played in a different time (Lez_05 p.11). So four runs of the dynamics (robot alone and 1 kg alone, each split into weight and motion) give the loads for any payload and any cycle time.

| Payload | Largest motor use at 10.67 s | Shortest cycle that passes every check |
|---|---|---|
| 0.2 kg | 0.19 (q1) | 3.4 s |
| 0.4 kg | 0.28 (q3) | 3.8 s |
| 0.6 kg | 0.37 (q3) | 4.2 s |
| 0.8 kg | 0.45 (q3) | 4.4 s |
| 1.0 kg | 0.54 (q3) | 4.8 s |

The shortest cycles are limited by q1's RMS torque. They leave out the inertia of the transmissions (see step 7b), so they are somewhat optimistic.

**Approximations, as listed in class on 10/22:** rated and maximum torque constant over the speed range, transmission inertia ignored, transmission efficiency 1.

## 8. Step 5b: three ways to drive q3 (`step5b_q3_options.m`)

Same trajectory, method and margin as step 5, for every payload from 0.2 to 1 kg. For each option the script finds the smallest reduction of q3 that passes.

| Option | q3 motor | Smallest reduction | RMS / rated (0.2 to 1 kg) | Peak / maximum | q3 must hold |
|---|---|---|---|---|---|
| A | HG-KR053, 50 W | 1:30 | 0.27 to 0.76 | 0.14 to 0.41 | 3.90 N·m |
| B | HG-KR13, 100 W | 1:15 | 0.27 to 0.76 | 0.14 to 0.42 | 3.90 N·m |
| C | HG-KR053 with a balancing spring | 1:12 | 0.10 to 0.70 | 0.29 to 0.66 | 1.42 N·m |

The spring of option C follows Lez_08_Components p.51 (gravity balancing, lesson of 11/05). **We use option A**, built as 1:42.19, which is more reduction than the smallest that passes and uses stock pulleys on the existing shafts. The earlier designs used option B (100 W, 1:25); the 50 W motor became possible when the third design took the q4 motor off the arm. Option C is kept as an examined alternative: it adds a mechanism that balances only one payload.

## 9. The coupled wrist drive of the third design

The q4 motor is on the slider. It turns a 20-tooth pulley; a belt takes this to a 60/20-tooth double pulley that idles on the q3 axis; a long belt along the arm takes it to the 40-tooth pulley on the wrist. Consequences, all in the code:

- **Kinematics of the drive.** Relative to the arm the double pulley turns by 2·q4, and the arm itself turns by q3, so relative to the slider the double pulley turns by q3 + 2·q4 and the motor by θ_m4 = 6·q4 + 3·q3. The q3 motor turns by θ_m3 = n3·q3 with n3 = 42.19.
- **Motor torques by virtual work.** τ3·δq3 + τ4·δq4 = T_m3·n3·δq3 + T_m4·(6·δq4 + 3·δq3), so T_m4 = τ4/6 and T_m3 = (τ3 − τ4/2)/n3. In `step5_dynamics.m` and `step7b_simscape_cad.m` this is the matrix `C = eye(4); C(4,3) = 0.5`.
- **Timing.** The wrist motor's speed and acceleration follow q4 + 0.5·q3, which is why step 4 stretches the trajectory (section 6).
- **Rotor of the wrist motor.** In the Simscape model it is driven at 6·q̈4 + 3·q̈3 on the slider.

## 10. Step 7: Simscape Multibody with point masses (`step7_simscape.m`, `build_rprr_simscape.m`)

**Method of the exercise of 10/19** (Ex4.1, Ex4.2): each joint has its motion provided by input and its torque automatically computed. The motion is given as three time series per joint, [t, q], [t, q̇], [t, q̈] (Ex4.2 p.16); the computed torque (force for the slider) is sensed; a Transform Sensor reads where the pen tip is. Payload as in Ex4.2 p.19–20.

**Built by a script.** `build_rprr_simscape.m` adds and connects every block, so the model always matches the MATLAB functions. The numbers are variables set by the step script, not typed into the blocks. `robot_rprr.slx` is regenerated on every run.

**Purpose.** This version uses the same J1 and point masses as `RPRRdyn`, so the two tools must agree. It is the check of our own equations, frames, signs and units by an independent tool.

**Result.** Pen tip of Simscape against `RPRRdir`: 3.7e-13 mm apart at most. Joint loads at 1 kg and 0.2 kg: largest difference 1e-14 N·m, about 1e-15 of the peak, for every joint.

## 11. Step 7b: Simscape with the CAD bodies and the drives (`step7b_simscape_cad.m`, `build_rprr_simscape_cad.m`)

**What the model contains** (the assistant asked on 10/26 and 11/16 for solid bodies, the designed tool with the TCP at its tip, and the inertia of motors and transmissions):

- **Bodies.** The four moving sub-assemblies of the Fusion model (D_02 column and rail, D_03 slider, D_04 arm, D_05 pen head) as rigid bodies with the mass, centre of mass and full inertia tensor that Fusion computes from the materials. Their STEP shapes are shown; base and bucket are shown fixed. Exported by `Fusion/step7_export_bodies.py`.
- **Payload.** A point mass at the pen tip.
- **Drives.** One equivalent rotor per joint on the body that carries the motor, turning at the motor speed: the motor's rotor plus every turning part of the transmission (coupling, shafts, pulleys, belts, ball screw) reduced to the motor speed, J·k² for a part turning k times as fast as the motor and m·k² for a belt. The rotors get their motion from the trajectory, so no gear constraint is needed; for timing belts this is exact because input and output turn the same way.

**Checks (all passed).**
- Pen tip against `RPRRdir`: 6.5e-13 mm.
- Against Fusion: in three test poses Fusion itself reports the gravity torques at q3 and q4 and the moment of inertia about q1 (`Fusion/step7_pose_check.py`); Simscape agrees to 4.5e-10 (relative). This shows that every CAD body sits in the right place, turned the right way, with the right mass properties.
- The equivalent rotors' torques equal J_r·q̈/τ to 3e-16 (relative).

**Drive inertia at the motor shaft** (kg·m²):

| Joint | Rotor | Transmission | Total / rotor | Seen at the joint |
|---|---|---|---|---|
| q1 | 4.50e-6 | 8.64e-6 | 2.92 | 0.0329 kg·m² (rotor alone 0.0112) |
| q2 | 4.50e-6 | 3.74e-6 | 1.83 | 3.25 kg (rotor alone 1.78 kg) |
| q3 | 4.50e-6 | 4.78e-6 | 2.06 | 0.0165 kg·m² (rotor alone 0.0080) |
| q4 | 4.50e-6 | 3.65e-6 | 1.81 | 0.0003 kg·m² |

The largest single part is the 120-tooth pulley on q1's intermediate shaft: 6.73e-6 kg·m² at the motor.

**Results with 1 kg** (MATLAB = step 5 with point masses and rotor only; Simscape = CAD bodies and drives):

| Joint | Peak joint load, MATLAB → Simscape | Largest difference | Motor use RMS/rated, peak/max: MATLAB | Simscape |
|---|---|---|---|---|
| q1 | 4.999 → 5.024 N·m | 0.101 N·m | 0.20 / 0.19 | 0.21 / 0.20 |
| q2 | 12.449 → 12.456 N | 0.940 N | 0.02 / 0.05 | 0.03 / 0.05 |
| q3 | 7.303 → 7.304 N·m | 0.010 N·m | 0.54 / 0.30 | 0.54 / 0.30 |
| q4 | 0.863 → 0.863 N·m | below 0.001 N·m | 0.45 / 0.26 | 0.45 / 0.26 |

With 0.2 kg the Simscape motor use is 0.12 / 0.11, 0.02 / 0.04, 0.19 / 0.11, 0.09 / 0.05. All four motors stay inside the 0.8 margin: no motor or ratio has to change.

**Why the two models differ (request 8 and the professor's question of 9/28).** Each cause is computed separately, with runs with and without gravity.

1. **Gravity, only at q3.** The centre of mass of the arm body is 0.5 mm off the link line, while the point masses sit on the line. This changes q3's holding torque by −m·g·y·sin q3, at most 0.002 N·m. Simscape agrees with this formula to 1e-14 N·m. q1 and q2 carry no weight (vertical axis, horizontal rail).
2. **Inertia about q1.** The point masses keep each body's inertia about the q3 axis, not about the vertical axis. The CAD bodies' inertia about q1 is 0.5 to 0.8 % larger (0.1765 to 0.2512 against 0.1752 to 0.2499 kg·m² over the cycle). Times q̈1 this explains the 0.10 N·m at q1, together with terms that couple q1 with the other joints.
3. **Slider force.** The total mass on the slider is the same, but centres of mass off the link line and beside the rail couple the slider with the accelerations of q1 and q3: up to 0.94 N.
4. **Drives (Simscape only).** The transmission parts add (J_total − J_rotor)·q̈/τ to the motor torque: 0.009 N·m at q1, 0.005 at q2, 0.004 at q3, 0.001 at q4. For q1 the lecture's simplification "transmission inertia ignored" is the largest single approximation of step 5: it takes q1's use from 0.20 / 0.19 to 0.21 / 0.20.

**Not modelled** (in both models): friction and efficiency of bearings, belts, screw and guide; belt stretch; the drop of the motors' maximum torque near maximum speed; cables.

## 12. Limitations to state in the report

- The score map uses a horizontal pen; the score along the letter uses the real pen angles.
- Step 4 times every short segment of the vertical stroke as if it started and ended at rest; a rule based on cruise speed would be faster.
- The Home moves start and end with a jump in acceleration (the spline is only told to start and end at rest, Lez_05 p.33).
- Step 3b looks at kinematics only; its best designs sit on the edge of the grid for L4.
- The C4 check of step 2 is a reach check; joint and wrist limits are not included.
- Two recommendations of the motor manual are not applied (the checks taught in the course are used instead): load inertia at most 17 times the rotor inertia, and holding torque at most 70 % of the rated torque.
- The brief gives H/R0 ≥ 0.095, which may be a misprint for 0.95; our 1.15 is valid either way.

## 13. Sources

- Brief: `Progetto2026_2027_IR.pdf` (p.2 robot and motors, p.3–4 task, p.5 requests, p.6 Project A).
- Kinematics: `Lez_02_Kinematics.pdf` (p.15–23, p.18 workspace, p.22 inversion); handout *SCARA kinematics*.
- Indices: `Lez_04_Ellipses.pdf` (p.22–24).
- Trajectory: `Lez_05_MotionPlanning.pdf` (p.11 time scaling, p.14–23 minimum time, p.25–29 lines and parabolas, p.30–34 cubic splines, p.35–38 movement along a path); exercises Ex3.1 and Ex3.2 with the course MATLAB code.
- Dynamics: `Lez_03_Dynamics.pdf` (p.4–7 method, p.16–19 point masses); handout *SCARA Dynamics*; `Lez_10_ParallelRobot.pdf` p.35 (numerical check).
- Motor sizing: `Lez_08 MotorSizingNotes.pdf` (p.10 checks, p.21–28 α, β and τ, p.29 check plot, p.32 linear axis).
- Components: `Lez_08_Components.pdf` (p.51 gravity balancing).
- Simscape: exercises `Ex4.1IntroductionToSimscape.pdf`, `Ex4.2Simscape.SCARA.pdf` (p.16 time series as joint input, p.18 model, p.19–20 payload).
- Epson: `Ex6_EPSON_RC+.pdf`, `Epson exercise.pdf` (not used yet).
- Motor data: Mitsubishi *Servo Motor Instruction Manual (Vol. 3)*, chapter 6, 6.3.1.
- Epson C4 manipulator manual, Rev.20.
