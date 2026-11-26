%% Step 7, second version — Simscape with the CAD bodies and the drives, compared with the MATLAB model
% after CAD edition 1 (2026-11-24). Project requirements 6 (Simscape model), 7 (carry out the task with both
% models) and 8 (compare the motor torques and explain the differences).
%
% The first version (step7_simscape.m, point masses) stays as the check of our equations: with the
% same masses it agrees with step 5 to 1e-15. This version replaces the point masses with what the
% robot really is, as far as the CAD knows it:
%   - the four moving sub-assemblies of the Fusion model as rigid bodies with their full inertia
%     (Fusion/step7_export_bodies.py writes the shapes and the mass properties);
%   - the motors' rotors and every turning part of the transmissions (couplings, shafts, pulleys,
%     belts, ball screw), as one equivalent rotor per joint at the motor's speed. Decided on
%     2026-11-24 (assistant, 10/26: what changes the dynamics must be in the Simscape model).
% Step 5 is left as it is (point masses, rotor only, as the lecture's method), so the differences
% between the two models are the subject of requirement 8. They are split into their causes below.
%
% Checks before the comparison:
%   1. the pen tip follows RPRRdir (frames, zero positions, units of the joints);
%   2. in three poses, the gravity torques at q3 and q4 and the moment of inertia about q1 agree with
%      what Fusion itself computes for the same poses (Fusion/step7_pose_check.py). This checks that
%      every CAD body sits in the right place, turned the right way, with the right mass properties.

clear; close all; clc;

%% Settings
mdl      = 'robot_rprr_cad';
payloads = [1.0 0.2];               % [kg] payloads run with gravity
here     = fileparts(mfilename('fullpath'));
geom     = fullfile(here, 'Fusion', 'v3', 'step7_bodies');
show     = false;                    % true: at the end, play the 1 kg run in the Mechanics Explorer
                                     % (MATLAB desktop only, works there (2026-11-24); it hangs in matlab -batch)
video    = ~isempty(getenv('RPRR_VIDEO'));   % set the environment variable RPRR_VIDEO to record the 1 kg run as
                                     % robot_rprr_cad.avi at the end (smwritevideo; MATLAB with a display, not -batch)

%% Inputs
S4 = load(fullfile(here, 'step4_trajectory.mat'));
S5 = load(fullfile(here, 'step5_dynamics.mat'));
L  = S4.L;
t  = S4.traj.t(:);   ns = numel(t);
to_si = [1; 1e-3; 1; 1];
Q = S4.traj.Q.*to_si;   Qd = S4.traj.Qd.*to_si;   Qdd = S4.traj.Qdd.*to_si;          % rad, m
P  = jsondecode(fileread(fullfile(geom, 'mass_properties.json')));
PC = jsondecode(fileread(fullfile(geom, 'pose_check.json')));
g  = S5.D.g;
ch = S5.choice;                                     % motor and ratio of each joint (step 5)
tau = [ch.tau];  n_r = 1./tau;                      % joint speed / motor speed, and its inverse
Jm  = arrayfun(@(c) c.motor.Jm, ch);
motor_names = arrayfun(@(c) string(c.motor.name), ch);

%% Equivalent rotor of each drive: rotor + turning parts of the transmission, reduced to the motor speed
% Every part turning at k times the motor speed adds J*k^2 (J about its own axis, from Fusion; the parts
% are drawn with their axis along their own z). A belt moves all its mass at one speed v = k*w_motor
% (k = pulley pitch radius times that pulley's speed ratio), so it adds m*k^2. The idlers' outside
% diameter is taken as 15 mm (their part name). Parts that turn with a robot body (the last pulley of
% each drive, the q3 shaft, the q4 wrist shaft) are already in the bodies; idler axles do not turn.
rp = @(z) z*3e-3/(2*pi);                            % pitch radius of a 3 mm GT pulley with z teeth [m]
drive = {  % joint  sub-assembly  part                       k (speed / motor speed)       kind
    1 'D_01' 'D_XGT2_19C_8x6'          1                         'turn'
    1 'D_01' 'D_Q1_Input_shaft'        1                         'turn'
    1 'D_01' 'D_GT3M_18T_W9_D6'        1                         'turn'
    1 'D_01' 'D_Belt_Q1_stage1'        rp(18)                    'belt'
    1 'D_01' 'D_Q1_Intermediate_shaft' 18/120                    'turn'
    1 'D_01' 'D_GT3M_120T_W9_D8'       18/120                    'turn'
    1 'D_01' 'D_GT3M_16T_W15_D6'       18/120                    'turn'
    1 'D_01' 'D_Belt_Q1_stage2'        18/120*rp(16)             'belt'
    1 'D_01' 'D_Belt_idler_15'         18/120*rp(16)/7.5e-3      'turn'
    2 'D_02' 'D_XGT2_19C_8x6'          1                         'turn'
    2 'D_02' 'D_Q2_Ballscrew_12x10'    1                         'turn'
    3 'D_03' 'D_XGT2_19C_8x6'          1                         'turn'
    3 'D_03' 'D_Q3_Input_shaft'        1                         'turn'
    3 'D_03' 'D_GT3M_16T_W9_D6'        1                         'turn'
    3 'D_03' 'D_Belt_Q3_stage1'        rp(16)                    'belt'
    3 'D_03' 'D_Q3_Intermediate_shaft' 16/90                     'turn'
    3 'D_03' 'D_GT3M_90T_W9_D8'        16/90                     'turn'
    3 'D_03' 'D_GT3M_16T_W15_D6'       16/90                     'turn'
    3 'D_03' 'D_Belt_Q3_stage2'        16/90*rp(16)              'belt'
    3 'D_03' 'D_Belt_idler_15'         16/90*rp(16)/7.5e-3       'turn'
    1 'D_01' 'D_Q1_Stage1_idler'       rp(18)/8e-3               'turn'
    3 'D_03' 'D_Q3_First_stage_idler'  rp(16)/8e-3               'turn'
    4 'D_03' 'D_XGT2_19C_8x6'          1                         'turn'
    4 'D_03' 'D_Q4_Input_shaft_v3'     1                         'turn'
    4 'D_03' 'D_GT3M_20T_W6_D6'        1                         'turn'
    4 'D_03' 'D_Belt_Q4_stage1_W6'     rp(20)                    'belt'
    4 'D_03' 'D_Q4_Transfer_pulley_60T20T' 20/60                 'turn'};
nd = size(drive, 1);  Jpart = zeros(nd, 1);
for i = 1:nd
    parts = P.subassemblies.(drive{i, 2}).transmission_parts;
    k = find(strcmp({parts.part}, drive{i, 3}), 1);
    assert(~isempty(k), 'Part %s not found in %s', drive{i, 3}, drive{i, 2});
    if strcmp(drive{i, 5}, 'turn'), Jpart(i) = parts(k).I_about_own_z*drive{i, 4}^2;
    else,                           Jpart(i) = parts(k).mass*drive{i, 4}^2; end
end
jd = cell2mat(drive(:, 1));
Jtr = accumarray(jd, Jpart, [4 1]).';               % transmission, at the motor speed
J_r = Jm + Jtr;                                     % equivalent rotor

% The long belt and its backside idler move relative to the pitching arm.
% Their rigid-body inertia is already in D_04. Add only the internal-motion
% terms: integral(r cross tangent) around the pitch loop equals 2*loop area.
parts = P.subassemblies.D_04.transmission_parts;
long = parts(strcmp({parts.part},'D_Belt_Q4_long_690'));
idler = parts(strcmp({parts.part},'D_Q4_Long_idler'));
r1 = rp(20); r2 = rp(40); spacing = L.L3/1000;
a = asin((r2-r1)/spacing); h = spacing*cos(a);
area = (r1+r2)*h + r1^2*(pi/2-a) + r2^2*(pi/2+a);
perimeter = 2*(h + r1*(pi/2-a) + r2*(pi/2+a));
kidler = -r2/.008;
cross_I = long.mass*2*area/perimeter*r2 + idler.I_about_own_z*kidler;
wrist_I = long.mass*r2^2 + idler.I_about_own_z*kidler^2;
long_load = zeros(size(Qdd));
long_load(3,:) = cross_I*Qdd(4,:);
long_load(4,:) = cross_I*Qdd(3,:) + wrist_I*Qdd(4,:);

%% Where the motors sit, and the turn that lays each rotor joint's z on the motor shaft (positive like the joint)
carrier = {'D_01', 'D_02', 'D_03', 'D_03'};
motor_axis_xy = {[], [], [-0.1066 0.07829], [-0.0225 0.03897]};   % [m] slider frame: q3 input shaft, q4 input shaft
p_r = cell(1, 4);  R_r = cell(1, 4);
for j = 1:4
    mo = P.subassemblies.(carrier{j}).motors;
    mo = mo(strcmp({mo.part}, ['D_' strrep(char(motor_names(j)), '-', '_')]));
    if numel(mo) > 1        % q3 and q4 use the same motor type on the slider (since 2026-11-26): tell them apart by their axis
        o = reshape([mo.origin_m], 3, []);
        [~, k] = min(vecnorm(o(1:2,:) - motor_axis_xy{j}(:)));
        mo = mo(k);
    end
    assert(isscalar(mo),'Motor carrier must identify one installed motor');
    p_r{j} = mo.origin_m(:).';
    a = abs(round(mo.shaft_axis(:).'));
    if isequal(a, [0 1 0]),     R_r{j} = [1 0 0; 0 0 1; 0 -1 0];        % z onto +Y
    elseif isequal(a, [1 0 0]), R_r{j} = [0 0 1; 0 1 0; -1 0 0];        % z onto +X
    else,                       R_r{j} = eye(3);                        % already along Z
    end
end

%% Build the model (always from scratch)
build_rprr_simscape_cad(mdl, here, geom);
load_system(fullfile(here, [mdl '.slx']));

% Variables shared by every run
base = Simulink.SimulationInput(mdl);
base = base.setVariable('h_m', L.h/1000).setVariable('alpha_deg', rad2deg(L.alpha));
base = base.setVariable('L3_m', L.L3/1000).setVariable('L4_m', L.L4/1000);
base = base.setVariable('R_W', [0 0 1; 1 0 0; 0 1 0]).setVariable('R_23', [0 0 1; 0 -1 0; 1 0 0]).setVariable('R_34', diag([-1 -1 1]));
keys = {'D_01', 'D01'; 'Wave_Drum', 'drum'; 'D_02', 'D02'; 'D_03', 'D03'; 'D_04', 'D04'; 'D_05', 'D05'};
for i = 1:size(keys, 1)
    s = P.subassemblies.(keys{i, 1});  I = s.I_com;
    base = base.setVariable(['m_' keys{i, 2}], s.mass).setVariable(['c_' keys{i, 2}], s.com_m(:).');
    base = base.setVariable(['Im_' keys{i, 2}], diag(I).').setVariable(['Ip_' keys{i, 2}], [I(2,3) I(3,1) I(1,2)]);
end
base = base.setVariable('n_r', n_r);
for j = 1:4, base = base.setVariable(sprintf('p_r%d', j), p_r{j}).setVariable(sprintf('R_r%d', j), R_r{j}); end

%% Check 2 first: three poses against Fusion
np = numel(PC.poses);  chk = zeros(np, 6);           % [q3 Fusion, q3 Simscape, q4 F, q4 S, I1 F, I1 S]
for p = 1:np
    pc = PC.poses(p);  qp = pc.q(:).*[pi/180; 1e-3; pi/180; pi/180];
    B = pc.bodies;  gv = [0; -g; 0];                 % Fusion assembly frame: Y up
    O3 = pc.q3_axis.origin_m(:);  a3 = pc.q3_axis.z_axis(:);
    O4 = pc.q4_axis.origin_m(:);  a4 = pc.q4_axis.z_axis(:);
    c4 = B.D_04.com_m(:);  c5 = B.D_05.com_m(:);
    chk(p, 1) = -dot(cross(c4 - O3, B.D_04.mass*gv) + cross(c5 - O3, B.D_05.mass*gv), a3);   % joint torque that holds gravity
    chk(p, 3) = -dot(cross(c5 - O4, B.D_05.mass*gv), a4);
    chk(p, 5) = B.D_02.I_origin(2,2) + B.D_03.I_origin(2,2) + B.D_04.I_origin(2,2) + B.D_05.I_origin(2,2);   % about the Y axis = q1 axis
    for trial = 1:2                                    % 1: still, with gravity; 2: no gravity, q1 accelerating at 1 rad/s^2
        in = base.setVariable('t_out', [0; 0.5; 1]).setVariable('m_pay', 1e-12).setVariable('J_r', 1e-15*ones(1, 4));
        in = in.setVariable('g', g*(trial == 1));
        for j = 1:4
            in = in.setVariable(sprintf('q_%d', j), [0 qp(j); 1 qp(j)]).setVariable(sprintf('w_%d', j), [0 0; 1 0]);
            in = in.setVariable(sprintf('b_%d', j), [0 (trial == 2 && j == 1); 1 (trial == 2 && j == 1)]);
        end
        tic; out = sim(in); fprintf('  pose check %d, trial %d: %.0f s\n', p, trial, toc);
        at = @(ts) interp1(ts.Time, squeeze(ts.Data), 0.5);
        if trial == 1
            chk(p, 2) = at(out.F_q3);  chk(p, 4) = at(out.F_q4);
        else
            chk(p, 6) = at(out.F_q1);
        end
    end
end
err_pose = max(abs(chk(:, [2 4 6]) - chk(:, [1 3 5]))./max(abs(chk(:, [1 3 5])), 1e-3), [], 'all');

%% The task: trajectory of step 4, payload 1 kg and 0.2 kg with gravity, and 1 kg without gravity
runs = [payloads(1) 1; payloads(2) 1; payloads(1) 0];          % [payload, gravity on]
Fs = zeros(4, ns, 3);  Tr = Fs;  tcp = zeros(3, ns);
for r = 1:3
    in = base.setVariable('t_out', t).setVariable('m_pay', runs(r, 1)).setVariable('J_r', J_r).setVariable('g', g*runs(r, 2));
    for j = 1:4
        in = in.setVariable(sprintf('q_%d', j), [t, Q(j,:).']).setVariable(sprintf('w_%d', j), [t, Qd(j,:).']);
        in = in.setVariable(sprintf('b_%d', j), [t, Qdd(j,:).']);
    end
    tic; out = sim(in); fprintf('  simulation %d of 3 (payload %.1f kg, gravity %s): %.0f s\n', r, runs(r, 1), tern(runs(r, 2), 'on', 'off'), toc);
    for j = 1:4
        a = out.(sprintf('F_q%d', j));  Fs(j,:,r) = interp1(a.Time, squeeze(a.Data), t).';
        a = out.(sprintf('T_r%d', j));  Tr(j,:,r) = interp1(a.Time, squeeze(a.Data), t).';
    end
    if r == 1
        tcp = [interp1(out.tcp_x.Time, squeeze(out.tcp_x.Data), t), interp1(out.tcp_y.Time, squeeze(out.tcp_y.Data), t), ...
               interp1(out.tcp_z.Time, squeeze(out.tcp_z.Data), t)].';
        in_show = in;
    end
end
if ~show, close_system(mdl, 0); end

%% MATLAB model (step 5) for the same payloads
Fm  = @(p) S5.dyn.Fin0 + p*S5.dyn.Finu + S5.dyn.Fgr0 + p*S5.dyn.Fgru;
Fgm = @(p) S5.dyn.Fgr0 + p*S5.dyn.Fgru;
Fim = @(p) S5.dyn.Fin0 + p*S5.dyn.Finu;

%% Check 1: the pen tip
Tip = zeros(3, ns);
for i = 1:ns, Tip(:,i) = RPRRdir(S4.traj.Q(:,i), L)/1000; end
err_tip = max(vecnorm(tcp - Tip))*1000;

%% Differences in the joint loads (payload 1 kg) and their causes
F1  = Fs(:,:,1);  Fmat = Fm(1);
dF  = F1 - Fmat;                                 % total
dFg = (Fs(:,:,1) - Fs(:,:,3)) - Fgm(1);          % from gravity
dFi = Fs(:,:,3) - Fim(1);                        % from accelerations and speeds
% Gravity: only q3 and q4 carry weight (the rail is horizontal). Holding torque = g * (mass * horizontal
% arm). Terms along the link are retained by the point masses. Off-axis
% centres of mass contribute the sine terms from the Fusion measurements.
s4 = P.subassemblies.D_04;  s5 = P.subassemblies.D_05;  Dm = S5.D.m;  L3 = L.L3/1000;  L4 = L.L4/1000;
dG3_offset = -s4.mass*g*s4.com_m(2)*sin(Q(3,:));
fm_link = s4.mass*s4.com_m(1) + s5.mass*L3 - (Dm(2)*L3/2 + Dm(3)*L3 + Dm(4)*L3);   % first moments along the link [kg m]
fm_pen  = s5.mass*s5.com_m(1) - Dm(4)*L4/2;                                        % and along the pen
dG3_round = g*(fm_link*cos(Q(3,:)) + fm_pen*cos(Q(3,:) + Q(4,:)));
pen_offset = -s5.mass*g*s5.com_m(2)*sin(Q(3,:)+Q(4,:));
dG4_round = g*fm_pen*cos(Q(3,:) + Q(4,:)) + pen_offset;
dG3_pred = dG3_offset + dG3_round + pen_offset;
err_g3 = max(abs(dFg(3,:) - dG3_pred));
err_g4 = max(abs(dFg(4,:) - dG4_round));
err_g_other = max(abs(dFg([1 2],:)), [], 2);

%% Why q1 differs: moment of inertia about the q1 axis, CAD bodies against point masses (payload 1 kg in both)
B = P.subassemblies;
I1_cad = zeros(1, ns);  I1_pts = zeros(1, ns);
for i = 1:ns
    I1_cad(i) = q1_inertia_cad(Q(:,i), B, L.h/1000, L3) + 1.0*(Q(2,i) + L3*cos(Q(3,i)) + L4*cos(Q(3,i) + Q(4,i)))^2;
    I1_pts(i) = q1_inertia_pts(Q(:,i), S5.D, L3, L4);
end
err_I1_fusion = max(abs(arrayfun(@(p) q1_inertia_cad(PC.poses(p).q(:).*[pi/180; 1e-3; pi/180; pi/180], B, L.h/1000, L3), 1:np).' - chk(:, 5))./chk(:, 5));
err_I1_step5  = max(abs(I1_pts - S5.dyn.Mdiag(1,:))./S5.dyn.Mdiag(1,:));   % step 5's own mass matrix, payload 1 kg
dI1 = I1_cad - I1_pts;
rest_q1 = max(abs(dFi(1,:) - dI1.*Qdd(1,:)));                             % what dI1*qdd1 does not explain (coupling with the other joints)

%% Motor torques: MATLAB C_m = tau*C_r + J_m*qdd/tau; Simscape C_m = tau*(joint load) + (equivalent rotor torque)
C = eye(4); C(4,3) = 0.5;
Adrive = C*Qdd;
Cm_mat = zeros(4, ns, 2);  Cm_sim = Cm_mat;  Cm_body = Cm_mat;  use_mat = zeros(4, 2, 2);  use_sim = use_mat;  use_body = use_mat;   % use: [rms peak]
for a = 1:2
    Fa = Fm(payloads(a));
    Cm_mat(:,:,a) = tau(:).*(C.'\Fa) + Jm(:).*Adrive./tau(:);
    Cm_sim(:,:,a) = tau(:).*(C.'\(Fs(:,:,a)+long_load)) + Tr(:,:,a);
    Cm_body(:,:,a) = tau(:).*(C.'\Fs(:,:,a)) + Jm(:).*Adrive./tau(:);      % CAD bodies, rotor only (as step 5)
    for j = 1:4
        m = ch(j).motor;
        use_mat(j,:,a) = [rms_t(t, Cm_mat(j,:,a))/m.Cn, max(abs(Cm_mat(j,:,a)))/m.Cmax];
        use_sim(j,:,a) = [rms_t(t, Cm_sim(j,:,a))/m.Cn, max(abs(Cm_sim(j,:,a)))/m.Cmax];
        use_body(j,:,a) = [rms_t(t, Cm_body(j,:,a))/m.Cn, max(abs(Cm_body(j,:,a)))/m.Cmax];
    end
end
use5 = [arrayfun(@(c) c.use.rms, ch).', arrayfun(@(c) c.use.peak, ch).'];          % saved by step 5 (1 kg)
assert(max(abs(use_mat(:,:,1) - use5), [], 'all') < 1e-9, 'MATLAB motor use differs from step 5');
dC_body  = tau(:).*(C.'\dF);                                  % from the bodies (1 kg)
dC_drive = Tr(:,:,1) - Jm(:).*Adrive./tau(:) + tau(:).*(C.'\long_load);
rot_dev  = max(abs(Tr(:,:,1) - J_r(:).*Adrive./tau(:)), [], 2);   % rotor torque against J_r*qdd/tau [N m]
rot_rel  = rot_dev./max(abs(Tr(:,:,1)), [], 2);
rot_dev4 = max(abs(Tr(4,:,1) - J_r(4)*(Adrive(4,:)/tau(4))));   % q4's rotor also turns with the q3 link

%% Print
unit_F = ["N m", "N", "N m", "N m"];
fprintf('Simscape model %s.slx: CAD bodies D_02-D_05 (Fusion), payload at the pen tip, one equivalent rotor per drive\n', mdl);
fprintf('Trajectory of step 4: %.2f s, %d samples\n', t(end), ns);
fprintf('\nEquivalent rotor of each drive, at the motor speed [kg m^2] (transmission parts from Fusion):\n');
fprintf('  %-5s %-9s %11s %13s %11s %8s %22s\n', 'joint', 'motor', 'rotor J_m', 'transmission', 'total', 'x rotor', 'seen at the joint (J n^2)');
for j = 1:4
    if j == 2, sj = sprintf('%.2f kg (rotor %.2f kg)', J_r(j)*n_r(j)^2, Jm(j)*n_r(j)^2);
    else,      sj = sprintf('%.4f (rotor %.4f)', J_r(j)*n_r(j)^2, Jm(j)*n_r(j)^2); end
    fprintf('  q%d    %-9s %11.3e %13.3e %11.3e %8.2f   %s\n', j, motor_names(j), Jm(j), Jtr(j), J_r(j), J_r(j)/Jm(j), sj);
end
[~, big] = max(Jpart);
fprintf('  largest single part: %s of q%d, %.2e kg m^2 at the motor\n', drive{big, 3}, drive{big, 1}, Jpart(big));

fprintf('\nCheck 1. Pen tip, Simscape against RPRRdir: largest distance %.2e mm\n', err_tip);
fprintf('\nCheck 2. Three poses, Simscape against Fusion (q in deg, mm, deg, deg):\n');
fprintf('  %-26s %22s %22s %24s\n', 'pose', 'q3 gravity [N m]', 'q4 gravity [N m]', 'inertia about q1 [kg m^2]');
for p = 1:np
    fprintf('  %-26s %10.5f %10.5f  %10.6f %10.6f  %11.6f %11.6f\n', mat2str(PC.poses(p).q(:).'), chk(p, :));
end
fprintf('  largest relative difference: %.1e\n', err_pose);

fprintf('\nJoint loads, payload 1 kg: Simscape (CAD bodies) against MATLAB (point masses)\n');
fprintf('  %-5s %15s %15s %17s %15s %15s\n', 'joint', 'peak MATLAB', 'peak Simscape', 'largest diff.', 'from gravity', 'from motion');
for j = 1:4
    fprintf('  q%d %14.3f %-3s %11.3f %-3s %12.3f %-3s %11.3f %-3s %11.3f %-3s\n', j, max(abs(Fmat(j,:))), unit_F(j), ...
        max(abs(F1(j,:))), unit_F(j), max(abs(dF(j,:))), unit_F(j), max(abs(dFg(j,:))), unit_F(j), max(abs(dFi(j,:))), unit_F(j));
end
fprintf('  gravity part at q3 = -m g y sin(q3) (q3 body, m = %.4f kg, y = %.1f mm above the link line): up to %.3f N m\n', ...
    s4.mass, s4.com_m(2)*1000, max(abs(dG3_offset)));
fprintf('                     + axial moment residual from the point-mass model: up to %.1e N m\n', max(abs(dG3_round)));
fprintf('    Simscape against these two terms: largest difference %.1e N m; at q4: %.1e N m\n', err_g3, err_g4);
fprintf('  gravity part at q1 and q2: %.1e N m, %.1e N (none expected: vertical axis, horizontal rail)\n', err_g_other);
fprintf('  inertia about the q1 axis over the cycle, payload 1 kg: point masses %.4f to %.4f kg m^2, CAD bodies %.4f to %.4f\n', ...
    min(I1_pts), max(I1_pts), min(I1_cad), max(I1_cad));
fprintf('    CAD minus points: %+.4f to %+.4f kg m^2 (%+.1f%% to %+.1f%%); times qdd1 it leaves %.3f N m of the q1 difference (terms with the speeds and accelerations of the other joints)\n', ...
    min(dI1), max(dI1), 100*min(dI1./I1_pts), 100*max(dI1./I1_pts), rest_q1);
fprintf('    formulas checked: CAD against Fusion %.1e, points against step 5''s mass matrix %.1e (relative)\n', err_I1_fusion, err_I1_step5);
fprintf('  for comparison, the q1 drive (rotor + transmission) seen at the joint: %.4f kg m^2\n', J_r(1)*n_r(1)^2);

fprintf('\nMotor torques, payload 1 kg [N m]\n');
fprintf('  %-5s %13s %15s %20s %22s\n', 'joint', 'peak MATLAB', 'peak Simscape', 'diff. from bodies', 'diff. from transmission');
for j = 1:4
    fprintf('  q%d %14.4f %15.4f %20.4f %22.4f\n', j, max(abs(Cm_mat(j,:,1))), max(abs(Cm_sim(j,:,1))), max(abs(dC_body(j,:))), max(abs(dC_drive(j,:))));
end
fprintf('  equivalent rotor torque against J_r*qdd/tau, per joint: %.1e %.1e %.1e %.1e N m (%.0e %.0e %.0e %.0e relative)\n', rot_dev, rot_rel);
fprintf('    q4: slider-mounted rotor driven at 6*qdd4 + 3*qdd3: %.1e N m\n', rot_dev4);

fprintf('\nMotor use (RMS/rated, peak/max; must be <= 0.8 with our margin)\n');
fprintf('  %-5s %-9s %-8s %18s %24s %22s\n', 'joint', 'motor', 'payload', 'MATLAB (step 5)', 'CAD bodies, rotor only', 'CAD bodies + drives');
for a = 1:2
    for j = 1:4
        fprintf('  q%d    %-9s %4.1f kg %9.2f / %4.2f %16.2f / %4.2f %16.2f / %4.2f\n', j, motor_names(j), payloads(a), use_mat(j,:,a), use_body(j,:,a), use_sim(j,:,a));
    end
end
fprintf('  all within 0.8 in Simscape: %s\n', tern(all(use_sim <= 0.8, 'all'), 'yes', 'NO'));

assert(err_tip < 1e-3, 'The Simscape pen tip does not follow the MATLAB one');
assert(err_pose < 1e-5, 'The CAD bodies in Simscape do not match Fusion in the test poses');
assert(err_g3 < 1e-6 && err_g4 < 1e-6, 'The gravity differences are not explained by the Fusion centres of mass');
assert(err_I1_fusion < 1e-6 && err_I1_step5 < 1e-9, 'The inertia formulas about q1 do not match Fusion or step 5');
assert(rot_dev4 < 1e-9 && all(rot_dev(1:3) < 1e-9), 'The rotor torques are not what the equivalent rotors should give');

%% Figure 1 — joint loads: both models and the difference with its causes (payload 1 kg)
fig1 = figure('Name', 'Step 7b — joint loads', 'Color', 'w', 'Position', [80 80 1250 800]);
names = ["q1 (base turn)", "q2 (slider)", "q3", "q4 (pen angle)"];
for j = 1:4
    subplot(4, 2, 2*j - 1); hold on; grid on;
    plot(t, Fmat(j,:), '-', 'Color', [0.2 0.45 0.8], 'LineWidth', 2.2);
    plot(t, F1(j,:), '-', 'Color', [0.9 0.5 0.1], 'LineWidth', 1);
    ylabel(sprintf('%s [%s]', names(j), unit_F(j))); xlim([0 t(end)]);
    if j == 1, title('Joint loads, payload 1 kg'); legend({'MATLAB (point masses)', 'Simscape (CAD bodies)'}, 'Location', 'best'); end
    if j == 4, xlabel('t [s]'); end
    subplot(4, 2, 2*j); hold on; grid on;
    plot(t, dF(j,:), 'k-', 'LineWidth', 1.2);
    plot(t, dFg(j,:), '-', 'Color', [0.2 0.6 0.3]);
    plot(t, dFi(j,:), '-', 'Color', [0.75 0.3 0.6]);
    if j == 3, plot(t, dG3_pred, '--', 'Color', [0.1 0.3 0.1]); end
    ylabel(sprintf('[%s]', unit_F(j))); xlim([0 t(end)]);
    if j == 1, title('Simscape minus MATLAB'); legend({'total', 'from gravity', 'from motion'}, 'Location', 'best'); end
    if j == 3, legend({'total', 'from gravity', 'from motion', 'gravity from CAD centre-of-mass offset'}, 'Location', 'best'); end
    if j == 4, xlabel('t [s]'); end
end
exportgraphics(fig1, fullfile(here, 'figures', 'step7b_joint_loads.png'), 'Resolution', 150);

%% Figure 2 — motor torques and motor use
fig2 = figure('Name', 'Step 7b — motor torques', 'Color', 'w', 'Position', [80 80 1250 800]);
for j = 1:4
    subplot(4, 3, 3*j - [2 1]); hold on; grid on;
    plot(t, Cm_mat(j,:,1), '-', 'Color', [0.2 0.45 0.8], 'LineWidth', 2.2);
    plot(t, Cm_sim(j,:,1), '-', 'Color', [0.9 0.5 0.1], 'LineWidth', 1);
    yline([-1 1]*ch(j).motor.Cn, ':', 'Color', [0.4 0.4 0.4]);
    ylabel(sprintf('q%d motor [N m]', j)); xlim([0 t(end)]);
    if j == 1, title('Motor torques, payload 1 kg (dotted: rated torque)'); legend({'MATLAB (step 5)', 'Simscape (CAD bodies + drives)'}, 'Location', 'best'); end
    if j == 4, xlabel('t [s]'); end
end
subplot(4, 3, [3 6]); hold on; grid on;
bar(1:4, [use_mat(:,1,1) use_sim(:,1,1)]); yline(0.8, 'r--');
set(gca, 'XTick', 1:4, 'XTickLabel', {'q1', 'q2', 'q3', 'q4'}); ylabel('RMS / rated'); title('Motor use, 1 kg');
legend({'MATLAB', 'Simscape', 'limit 0.8'}, 'Location', 'northwest');
subplot(4, 3, [9 12]); hold on; grid on;
bar(1:4, [use_mat(:,2,1) use_sim(:,2,1)]); yline(0.8, 'r--');
set(gca, 'XTick', 1:4, 'XTickLabel', {'q1', 'q2', 'q3', 'q4'}); ylabel('peak / maximum');
exportgraphics(fig2, fullfile(here, 'figures', 'step7b_motor_torques.png'), 'Resolution', 150);

%% Save
result = struct('t', t.', 'payloads', payloads, 'runs', runs, 'Fsim', Fs, 'Trotor', Tr, 'Fmat', Fmat, 'dF', dF, 'dFg', dFg, 'dFi', dFi, ...
    'Cm_mat', Cm_mat, 'Cm_sim', Cm_sim, 'Cm_body', Cm_body, 'use_mat', use_mat, 'use_sim', use_sim, 'use_body', use_body, 'J_rotor', Jm, 'J_transmission', Jtr, 'J_r', J_r, ...
    'n_r', n_r, 'drive_parts', {drive}, 'drive_part_J', Jpart, 'pose_check', chk, 'err_pose', err_pose, 'err_tip_mm', err_tip, ...
    'dG3_offset', dG3_offset, 'dG3_round', dG3_round, 'err_g3', err_g3, 'I1_cad', I1_cad, 'I1_pts', I1_pts, 'dC_body', dC_body, 'dC_drive', dC_drive);
save(fullfile(here, 'step7b_simscape_cad.mat'), 'result');
fprintf('\nSaved %s.slx, step7b_simscape_cad.mat, figures/step7b_joint_loads.png and figures/step7b_motor_torques.png\n', mdl);

%% Optional: watch the robot write the E with its CAD bodies
if show
    sim(in_show.setModelParameter('SimMechanicsOpenEditorOnUpdate', 'on'));   % the model stays open with its Mechanics Explorer
end

%% Optional: record the 1 kg run as a video (added 2026-11-26)
if video
    vars = in_show.Variables;
    for k = 1:numel(vars), assignin('base', vars(k).Name, vars(k).Value); end
    load_system(fullfile(here, [mdl '.slx']));
    % smwritevideo needs a finished run with the Mechanics Explorer open. The explorer redraws at every output
    % time, so the run for the video outputs 30 times a second only (with the 5339 samples of step 4 it had not
    % finished after 90 minutes, 2026-11-26).
    tv = linspace(t(1), t(end), round(30*(t(end) - t(1))) + 1).';
    assignin('base', 't_out', tv);
    sim(in_show.setVariable('t_out', tv).setModelParameter('SimMechanicsOpenEditorOnUpdate', 'on'));
    smwritevideo(mdl, fullfile(here, mdl), 'PlaybackSpeedRatio', 1, 'FrameRate', 30, 'VideoFormat', 'motion jpeg avi');
    close_system(mdl, 0);
    fprintf('Saved %s.avi\n', mdl);
end

function I = q1_inertia_cad(q, B, h, L3)
% Moment of inertia of the four moving CAD bodies about the q1 axis (no payload, no rotors), q in SI.
% Frames as in Fusion (Y up); q1 itself does not change it.
Rz = @(a) [cos(a) -sin(a) 0; sin(a) cos(a) 0; 0 0 1];
pose = {'D_02', eye(3), [0; 0; 0]; 'D_03', eye(3), [q(2); h; 0]; 'D_04', Rz(q(3)), [q(2); h; 0];
        'D_05', Rz(q(3) + q(4)), [q(2) + L3*cos(q(3)); h + L3*sin(q(3)); 0]};
I = 0;
for k = 1:4
    b = B.(pose{k, 1});  R = pose{k, 2};
    c = pose{k, 3} + R*b.com_m(:);  Ic = R*b.I_com*R.';
    I = I + Ic(2,2) + b.mass*(c(1)^2 + c(3)^2);
end
end

function I = q1_inertia_pts(q, D, L3, L4)
% The same for the point masses of robot_masses (J1 included, payload as in D), q in SI.
rho = q(2) + D.pts(:,3) + D.pts(:,1)*L3*cos(q(3)) + D.pts(:,2)*L4*cos(q(3) + q(4));
I = D.J1 + sum(D.m(:).*rho.^2);
end

function v = rms_t(t, x)
% RMS over the cycle, as in motor_use (trapezoid rule).
v = sqrt(trapz(t(:), x(:).^2)/(t(end) - t(1)));
end

function s = tern(c, a, b)
if c, s = a; else, s = b; end
end
