%% Step 4 — trajectory for the whole task (Project requirement 2)
% Home -> circular arc -> letter E -> straight line -> Home   (Project PDF p.4)
%   Home <-> letter : cubic splines in joint space through points taken on the
%                     arc / on the line (Lez_05 p.30-38)
%   letter          : "lines and parabolas" in joint space (Lez_05 p.25-29, Ex3.2)
% The time of every part comes from the joint speed and acceleration limits in
% robot_params (requirement 2: "considering the motor limits").
%
% The pen must be perpendicular to the wall on the letter only. Between Home
% and the letter its angle is free, so it is chosen here: at each point on the
% way every pen angle is tried, and the smoothest sequence that keeps all
% joints inside their limits and the arm off the wall is kept.
%
% Everything marked PLACEHOLDER is a starting choice that can still change.
% Saves step4_trajectory.mat: t, Q, Qd, Qdd for steps 5 (torques) and 7 (Simscape).

clear; close all; clc;

%% Settings
cfg.order      = "A";      % stroke order (see "Letter path" below)              DECIDED 2026-11-21
cfg.stops      = "all";    % where the pen stops on the letter:                  DECIDED 2026-11-21
                           %   "all" corners | "reversals" only | "none"
cfg.time_scale = 1;        % 1 = as fast as the limits allow; 2 = twice as slow (requirement 10)
cfg.dt         = 0.002;    % [s] sample time of the saved trajectory
cfg.spine_pts_per_wave = [];   % [] = the spine points of step 1; a number resamples the spine here (to test)

% Home: pen tip position, given as distance from the axis and height. It lies
% in the same vertical plane as the first point of the letter.
% DECIDED 2026-11-21 to keep this Home for now; pick it again when L4 changes.
% Chosen from a scan of Home positions and arc shapes (2026-11-21): this one
% lets the pen tip come off the wall at a clear angle at both ends of the letter.
home.rho  = 350;           % [mm]
home.z    = 120;           % [mm]
arc_bulge = 55;            % [mm] how far the arc's middle is pulled towards the axis
n_via     = 17;            % points taken on the arc and on the line

% Safety margins used when choosing the pen angle between Home and the letter   PLACEHOLDER
chk.body  = 12;                              % [mm] q3 link and pen body stay this far from the wall
chk.floor = 10;                              % [mm] ... and this far above the bucket floor
chk.q     = [0; 2; deg2rad(3); deg2rad(3)];  % stay this far inside the joint limits [rad; mm; rad; rad]
th_grid   = deg2rad(-120:1:120);             % pen angles that are tried

%% Inputs
here = fileparts(mfilename('fullpath'));
S1  = load(fullfile(here, 'step1_E_points.mat'));
prm = S1.params;
L   = robot_params();
w      = 2*pi*prm.k/prm.H;
r_wall = @(z) prm.R0 + prm.A*sin(w*z);
dr_wall = @(z) prm.A*w*cos(w*z);
feas   = @(T, th) pose_ok(T, th, L, r_wall, chk);

% wall_point must give the same points as step 1
Pchk = wall_point(S1.strokes(1).u, S1.strokes(1).v, prm);
assert(max(vecnorm(Pchk - S1.strokes(1).P)) < 1e-9, 'wall_point disagrees with step 1: re-run step 1');

%% Letter path: corners and stroke order
% Corners in flat coordinates (u right, v up from the letter's bottom):
%   TL --------- TR
%   |
%   ML ------ MR
%   |
%   BL --------- BR
% Order A: TR-TL-ML-MR-ML-BL-BR  (middle bar drawn out and back: 129 mm drawn twice)
% Order B: TR-TL-BL-BR-BL-ML-MR  (bottom bar and lower half of the spine drawn twice: 296 mm)
uL = -prm.E_w/2;  uR = prm.E_w/2;  uM = uL + prm.E_mid*prm.E_w;
node.TL = [uL; prm.E_h];  node.TR = [uR; prm.E_h];
node.ML = [uL; prm.E_h/2]; node.MR = [uM; prm.E_h/2];
node.BL = [uL; 0];         node.BR = [uR; 0];
switch cfg.order
    case "A", seq = ["TR" "TL" "ML" "MR" "ML" "BL" "BR"];
    case "B", seq = ["TR" "TL" "BL" "BR" "BL" "ML" "MR"];
    otherwise, error('Unknown stroke order %s', cfg.order);
end

% Via points: the corners, plus the sampled points of step 1 along the spine
% (the horizontal strokes are pure q1 rotations, so their two ends are enough)
v_spine = S1.strokes(1).v;
if ~isempty(cfg.spine_pts_per_wave)
    v_spine = linspace(prm.E_h, 0, max(2, ceil(cfg.spine_pts_per_wave*prm.E_h/(prm.H/prm.k)) + 1));
end
UV = node.(seq(1));  is_node = true;
for s = 2:numel(seq)
    a = node.(seq(s-1));  b = node.(seq(s));
    if a(1) == uL && b(1) == uL
        vv = sort(v_spine(v_spine > min(a(2), b(2)) + 1e-9 & v_spine < max(a(2), b(2)) - 1e-9), 'descend');
        if b(2) > a(2), vv = fliplr(vv); end
        UV = [UV, [uL*ones(1, numel(vv)); vv]];  is_node = [is_node, false(1, numel(vv))]; %#ok<AGROW>
    end
    UV = [UV, b];  is_node = [is_node, true]; %#ok<AGROW>
end
[Pl, Nl] = wall_point(UV(1,:), UV(2,:), prm);
nl = size(Pl, 2);
Ql = zeros(4, nl);
for i = 1:nl
    [Ql(:,i), ok] = RPRRinv(Pl(:,i), Nl(:,i), L);
    assert(ok && all(Ql(:,i) >= L.qmin & Ql(:,i) <= L.qmax), 'Letter point %d is out of reach or outside the joint limits', i);
end

% Dense ideal letter, for measuring how well it is traced
len_seg = vecnorm(diff(UV, 1, 2));
UVd = UV(:,1);
for i = 1:nl-1
    m = max(1, ceil(len_seg(i)/0.5));
    UVd = [UVd, UV(:,i) + (UV(:,i+1) - UV(:,i))*(1:m)/m]; %#ok<AGROW>
end
Pideal = wall_point(UVd(1,:), UVd(2,:), prm);

%% Letter: lines and parabolas, for the three ways of handling the corners
idx_node = find(is_node);
rev = false(size(idx_node));
for s = 2:numel(seq)-1, rev(s) = seq(s-1) == seq(s+1); end
stop_sets = struct('name', {"all", "reversals", "none"}, ...
                   'idx',  {idx_node, idx_node(rev | [true, false(1, numel(seq)-2), true]), [1, nl]});
for m = 1:numel(stop_sets)
    [pc, stretch] = plan_letter(Ql, stop_sets(m).idx, L, cfg.time_scale);
    [tm, Qm] = sample_pieces(pc, cfg.dt);
    e = letter_errors(Qm, L, Pideal, r_wall, dr_wall);
    stop_sets(m).pieces = pc;  stop_sets(m).T = tm(end);  stop_sets(m).err = e;  stop_sets(m).stretch = stretch;
    stop_sets(m).t = tm;  stop_sets(m).Q = Qm;
end
sel = find([stop_sets.name] == cfg.stops);
assert(~isempty(sel), 'Unknown cfg.stops %s', cfg.stops);
letter_pieces = stop_sets(sel).pieces;

%% Home <-> letter: points on the arc and on the line
P_first = Pl(:,1);   P_last = Pl(:,end);
phi_H   = atan2(P_first(2), P_first(1));
P_home  = [home.rho*cos(phi_H); home.rho*sin(phi_H); home.z];
u_via   = linspace(0, 1, n_via);
u_via   = 3*u_via.^2 - 2*u_via.^3;            % closer together near both ends: gentle start and stop

assert(arc_bulge ~= 0, 'arc_bulge = 0 gives a straight line; the PDF asks for a circular path');
mid   = (P_home + P_first)/2;
P_mid = mid - arc_bulge*[mid(1:2)/norm(mid(1:2)); 0];
P_arc  = arc_3pt(P_home, P_mid, P_first, u_via);          % Home -> first letter point
P_line = P_last + (P_home - P_last)*u_via;                % last letter point -> Home

%% Pen angle between Home and the letter
elev = @(n, P) atan2(n(3), (P(1:2).'*n(1:2))/norm(P(1:2)));   % pen angle above horizontal
th_first = elev(Nl(:,1), P_first);   th_last = elev(Nl(:,end), P_last);
% Both searches run from the letter towards Home; Home must get one common pen angle.
[C_arc,  ptr_arc]  = dp_pen(P_arc(:, end:-1:1), th_first, th_grid, feas);
[C_line, ptr_line] = dp_pen(P_line,             th_last,  th_grid, feas);
[c_best, gH] = min(C_arc + C_line);
assert(isfinite(c_best), ['No pen-angle sequence connects this Home to both ends of the letter ', ...
       'within the joint limits and wall margins. Move Home, change arc_bulge or the stroke order.']);
th_arc  = fliplr(smooth_pen(dp_back(ptr_arc,  gH, th_grid, th_first), P_arc(:, end:-1:1), feas));
th_line =        smooth_pen(dp_back(ptr_line, gH, th_grid, th_last),  P_line,             feas);
th_home = th_grid(gH);

Q_arc  = via_to_joints(P_arc,  th_arc,  L);
Q_line = via_to_joints(P_line, th_line, L);
Q_arc(:,end) = Ql(:,1);   Q_line(:,1) = Ql(:,end);        % exactly the letter's joint values
Q_home = Q_arc(:,1);
assert(norm(Q_home - Q_line(:,end)) < 1e-9, 'Home differs between the arc and the line');

%% Home <-> letter: cubic splines in joint space, time from the joint limits
[T_arc,  lim_arc]  = spline_time(Q_arc,  L);
[T_line, lim_line] = spline_time(Q_line, L);
T_arc  = T_arc*cfg.time_scale;   T_line = T_line*cfg.time_scale;
tk_arc = linspace(0, T_arc, n_via);   tk_line = linspace(0, T_line, n_via);
arc_piece  = struct('T', T_arc,  'fun', @(tl) traj_cubic_spline(tk_arc,  Q_arc,  tl), 'phase', 1);
line_piece = struct('T', T_line, 'fun', @(tl) traj_cubic_spline(tk_line, Q_line, tl), 'phase', 3);

%% Whole cycle
pieces = [arc_piece, letter_pieces, line_piece];
[t, Q, Qd, Qdd, phase] = sample_pieces(pieces, cfg.dt);
% Enforce the wrist motor envelope along the combined shoulder/wrist path.
wrist_v = Qd(4,:) + 0.5*Qd(3,:);
wrist_a = Qdd(4,:) + 0.5*Qdd(3,:);
coupling_stretch = max([1, max(abs(wrist_v))/L.vmax(4), sqrt(max(abs(wrist_a))/L.amax(4))]);
[t, Q, Qd, Qdd, phase] = sample_pieces(pieces, cfg.dt/coupling_stretch);
t = t*coupling_stretch;
Qd = Qd/coupling_stretch;
Qdd = Qdd/coupling_stretch^2;
phase_names = ["Home to letter (arc, cubic spline)", "letter (lines and parabolas)", "letter to Home (line, cubic spline)"];

%% Checks on the sampled trajectory
% 1. joint limits, speed and acceleration limits
tol = 1e-6;
in_limits = all(Q >= L.qmin - tol & Q <= L.qmax + tol, 'all');
[wall_bound,floor_clearance,base_bound] = robot_coupled_limits(Q(3,:),Q(4,:),L,prm);
assert(all(Q(2,:) <= wall_bound+tol & Q(2,:) >= base_bound-tol) && all(floor_clearance>0), ...
       'A sampled pose exceeds the coupled drum or floor limit');
v_use = max(abs(Qd), [], 2)./L.vmax;     a_use = max(abs(Qdd), [], 2)./L.amax;
% 2. direct kinematics of every sample: pen tip, pen direction, distance of the arm from the wall
ns = numel(t);
Tip = zeros(3, ns);  Npen = zeros(3, ns);  body_clear = zeros(1, ns);  low = zeros(1, ns);
for i = 1:ns
    [Tip(:,i), Npen(:,i), Pj] = RPRRdir(Q(:,i), L);
    [body_clear(i), low(i)] = body_clearance(Pj, r_wall);
end
tip_rho = vecnorm(Tip(1:2,:));
tip_in  = r_wall(Tip(3,:)) - tip_rho;                    % > 0: pen tip inside the bucket
d_arc   = vecnorm(Tip - P_first);   d_line = vecnorm(Tip - P_last);   % distance from the letter's ends
near    = @(idx, d, x) interp1(d(idx), tip_in(idx), x);  % tip-to-wall distance at x mm from the letter
% 3. how far the pen tip strays from the ideal arc and from the ideal line
i1 = phase == 1;  i3 = phase == 3;
dev_arc  = max(abs(dist_to_circle(Tip(:,i1), P_home, P_mid, P_first)));
dev_line = max(dist_to_line(Tip(:,i3), P_last, P_home));
% 4. start and end at Home, at rest; the pieces join without jumps
[T0, n0] = RPRRdir(Q(:,1), L);   [T1, ~] = RPRRdir(Q(:,end), L);
jump_q  = max(abs(diff(Q, 1, 2)), [], 2);
% 5. the spline function against MATLAB's own clamped spline
tt = linspace(0, T_arc, 400);
q_ref = spline(tk_arc, [zeros(4,1), Q_arc, zeros(4,1)], tt);
spline_err = max(abs(traj_cubic_spline(tk_arc, Q_arc, tt) - q_ref), [], 'all');

%% Summary
T_arc = T_arc*coupling_stretch;
T_line = T_line*coupling_stretch;
T_letter = stop_sets(sel).T*coupling_stretch;
assert(max(abs(Qd(4,:)+0.5*Qd(3,:))) <= L.vmax(4)+1e-6);
assert(max(abs(Qdd(4,:)+0.5*Qdd(3,:))) <= L.amax(4)+1e-6);
fprintf('Stroke order %s: %s   (stops: %s)\n', cfg.order, strjoin(seq, '-'), cfg.stops);
fprintf('Letter: %d via points, %.0f mm of path (%.0f mm drawn twice)\n', nl, sum(len_seg), sum(len_seg) - (2*prm.E_w + prm.E_mid*prm.E_w + prm.E_h));
fprintf('Home: pen tip %.0f mm from the axis, z = %.0f mm, pen angle %.0f deg; joints q1 %.1f deg, q2 %.1f mm, q3 %.1f deg, q4 %.1f deg\n', ...
        home.rho, home.z, rad2deg(th_home), rad2deg(Q_home(1)), Q_home(2), rad2deg(Q_home(3)), rad2deg(Q_home(4)));
fprintf('Pen angle on the arc:  %.0f to %.0f deg;  on the line: %.0f to %.0f deg\n', ...
        rad2deg(min(th_arc)), rad2deg(max(th_arc)), rad2deg(min(th_line)), rad2deg(max(th_line)));
fprintf('\nTimes [s] (time_scale %.2f):\n', cfg.time_scale);
fprintf('  Home -> letter (arc)   %6.2f   limited by %s\n', T_arc, lim_arc);
fprintf('  letter                 %6.2f\n', T_letter);
fprintf('  letter -> Home (line)  %6.2f   limited by %s\n', T_line, lim_line);
fprintf('  whole cycle            %6.2f\n', t(end));
fprintf('\nLetter, three ways of handling the corners (same limits):\n');
fprintf('  %-10s %8s %14s %14s %14s %12s\n', 'stops', 'time [s]', 'off path [mm]', 'not drawn [mm]', 'into wall [mm]', 'pen tilt [deg]');
for m = 1:numel(stop_sets)
    e = stop_sets(m).err;
    fprintf('  %-10s %8.2f %14.2f %14.2f %14.2f %12.2f\n', stop_sets(m).name, stop_sets(m).T, e.off_path, e.not_drawn, e.into_wall, e.tilt_deg);
end
fprintf('  off path = farthest the pen tip gets from the ideal letter; not drawn = largest piece of the letter the tip never reaches;\n');
fprintf('  into wall = deepest the tip goes past the wall between via points; pen tilt = largest error of "perpendicular to the wall".\n');
fprintf('\nChecks on the whole cycle (%d samples):\n', ns);
fprintf('  joints inside their limits: %s\n', tern(in_limits, 'yes', 'NO'));
fprintf('  q2 %.1f to %.1f mm (limits %.0f to %.0f);  q3 %.1f to %.1f deg (+-%.0f);  q4 %.1f to %.1f deg (+-%.0f)\n', ...
        min(Q(2,:)), max(Q(2,:)), L.qmin(2), L.qmax(2), rad2deg(min(Q(3,:))), rad2deg(max(Q(3,:))), rad2deg(L.qmax(3)), ...
        rad2deg(min(Q(4,:))), rad2deg(max(Q(4,:))), rad2deg(L.qmax(4)));
fprintf('  largest speed / limit:        q1 %.2f, q2 %.2f, q3 %.2f, q4 %.2f\n', v_use);
fprintf('  largest acceleration / limit: q1 %.2f, q2 %.2f, q3 %.2f, q4 %.2f\n', a_use);
fprintf('  arm (q3 link and pen body) closest to the wall: %.1f mm;  lowest point above the floor: %.1f mm\n', min(body_clear), min(low));
past_wall = max(0, -min(tip_in(i1 | i3)));
fprintf('  pen tip between Home and the letter: deepest past the wall %.3f mm (must be ~0)\n', past_wall);
fprintf('  pen tip to wall, 10 / 20 mm before the first letter point: %.1f / %.1f mm;  10 / 20 mm after the last: %.1f / %.1f mm\n', ...
        near(i1, d_arc, 10), near(i1, d_arc, 20), near(i3, d_line, 10), near(i3, d_line, 20));
fprintf('  pen tip strays from the ideal arc by at most %.2f mm, from the ideal line by at most %.2f mm\n', dev_arc, dev_line);
fprintf('  starts and ends at Home (%.1e mm apart), at rest (%.1e)\n', norm(T0 - T1), max(abs([Qd(:,1); Qd(:,end)])));
fprintf('  largest joint step between samples: %.2e rad, %.2e mm (no jumps)\n', max(jump_q([1 3 4])), jump_q(2));
fprintf('  spline function vs MATLAB spline(): %.1e\n', spline_err);
assert(in_limits, 'A joint leaves its limits');
assert(all(v_use < 1 + 1e-6) && all(a_use < 1 + 1e-6), 'A speed or acceleration limit is exceeded');
assert(spline_err < 1e-9, 'traj_cubic_spline disagrees with spline()');
assert(past_wall < 0.05, 'The pen tip goes through the wall between Home and the letter');
assert(norm(T0 - T1) < 1e-9 && max(abs([Qd(:,1); Qd(:,end)])) < 1e-9, 'The cycle does not start and end at Home at rest');

%% Figure 1 — 3D view of the whole task
fig1 = figure('Name', 'Step 4 — task in 3D', 'Color', 'w');
hold on; axis equal; grid on; view(40, 18);
thh = linspace(prm.phi_c - pi/2, prm.phi_c + pi/2, 90);  zz = linspace(0, prm.H, 120);
[TH, ZZ] = meshgrid(thh, zz);
surf(r_wall(ZZ).*cos(TH), r_wall(ZZ).*sin(TH), ZZ, 'FaceAlpha', 0.2, 'EdgeColor', 'none', 'FaceColor', [0.45 0.6 0.9]);
plot3([0 0], [0 0], [0 prm.H], 'k--');
c1 = [0.2 0.6 0.3];  c2 = [0.83 0.33 0.49];  c3 = [0.9 0.55 0.1];
plot3(Tip(1,i1), Tip(2,i1), Tip(3,i1), '-', 'Color', c1, 'LineWidth', 1.5);
plot3(Tip(1,phase == 2), Tip(2,phase == 2), Tip(3,phase == 2), '-', 'Color', c2, 'LineWidth', 2);
plot3(Tip(1,i3), Tip(2,i3), Tip(3,i3), '-', 'Color', c3, 'LineWidth', 1.5);
plot3(P_home(1), P_home(2), P_home(3), 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 6);
[~, ~, Pj] = RPRRdir(Q_home, L);
plot3(Pj(1,:), Pj(2,:), Pj(3,:), '-o', 'Color', [0.3 0.3 0.3], 'LineWidth', 2, 'MarkerSize', 4, 'MarkerFaceColor', [0.3 0.3 0.3]);
legend({'', '', 'Home \rightarrow letter (arc)', 'letter', 'letter \rightarrow Home (line)', 'Home', 'robot at Home'}, 'Location', 'northeast');
xlabel('x [mm]'); ylabel('y [mm]'); zlabel('z [mm]');
title(sprintf('Pen tip path, order %s, cycle %.1f s', cfg.order, t(end)));

%% Figure 2 — side cut: the arm on the way from Home and back
fig2 = figure('Name', 'Step 4 — Home to letter and back', 'Color', 'w', 'Position', [100 100 1000 520]);
names = {'Home \rightarrow letter (arc)', 'letter \rightarrow Home (line)'};
Qv2 = {Q_arc, Q_line};  idx2 = {i1, i3};  col2 = {c1, c3};
for p = 1:2
    subplot(1, 2, p); hold on; axis equal; grid on;
    plot(r_wall(zz), zz, 'k-', 'LineWidth', 1.2);  plot([0 0], [0 prm.H], 'k--');
    plot([0 0], [0 L.h], '-', 'Color', [0.6 0.6 0.6], 'LineWidth', 5);                       % column
    plot([0 L.qmax(2)*cos(L.alpha)], L.h + [0 L.qmax(2)*sin(L.alpha)], '-', 'Color', [0.52 0.72 0.92], 'LineWidth', 4);   % rail
    for k = 1:2:n_via
        [~, ~, Pj] = RPRRdir(Qv2{p}(:,k), L);
        plot(vecnorm(Pj(1:2, 3:5)), Pj(3, 3:5), '-o', 'Color', [0.55 0.55 0.55], 'LineWidth', 1, 'MarkerSize', 3);
    end
    plot(tip_rho(idx2{p}), Tip(3, idx2{p}), '-', 'Color', col2{p}, 'LineWidth', 2);
    plot(home.rho, home.z, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 6);
    plot(vecnorm(Pl(1:2, [1 end])), Pl(3, [1 end]), 's', 'Color', c2, 'MarkerFaceColor', c2, 'MarkerSize', 6);
    xlim([-20, prm.R0 + prm.A + 30]); ylim([-10, prm.H + 10]);
    xlabel('distance from the axis [mm]'); ylabel('z [mm]'); title(names{p});
end
sgtitle('Side cut: q3 link and pen at every second via point (grey), pen tip path (colour)');

%% Figure 3 — joint position, speed, acceleration over the cycle
fig3 = figure('Name', 'Step 4 — joints over time', 'Color', 'w', 'Position', [100 100 1200 800]);
unit_q = {'deg', 'mm', 'deg', 'deg'};  sc = [180/pi, 1, 180/pi, 180/pi];
tb = [0, T_arc, T_arc + T_letter, t(end)];
for j = 1:4
    data = {Q(j,:)*sc(j), Qd(j,:)*sc(j), Qdd(j,:)*sc(j)};
    lims = {[L.qmin(j) L.qmax(j)]*sc(j), [-1 1]*L.vmax(j)*sc(j), [-1 1]*L.amax(j)*sc(j)};
    labs = {sprintf('q_%d [%s]', j, unit_q{j}), sprintf('speed [%s/s]', unit_q{j}), sprintf('acceleration [%s/s^2]', unit_q{j})};
    for c = 1:3
        subplot(4, 3, (j-1)*3 + c); hold on; grid on;
        plot(t, data{c}, 'b-', 'LineWidth', 1);
        if ~(j == 1 && c == 1), yline(lims{c}(1), 'r--'); yline(lims{c}(2), 'r--'); end
        xline(tb(2), 'k:'); xline(tb(3), 'k:');
        ylabel(labs{c}); xlim([0 t(end)]);
        if j == 4, xlabel('t [s]'); end
    end
end
sgtitle('Joints over the cycle (red dashed: limits; dotted: arc | letter | line)');

%% Figure 4 — the letter as traced, for the three ways of handling the corners
fig4 = figure('Name', 'Step 4 — letter as traced', 'Color', 'w', 'Position', [100 100 1200 480]);
for m = 1:numel(stop_sets)
    subplot(1, 3, m); hold on; axis equal; grid on;
    plot(UVd(1,:), UVd(2,:), '-', 'Color', [0.75 0.75 0.75], 'LineWidth', 4);
    Tm = zeros(3, size(stop_sets(m).Q, 2));
    for i = 1:size(Tm, 2), Tm(:,i) = RPRRdir(stop_sets(m).Q(:,i), L); end
    plot((prm.phi_c - atan2(Tm(2,:), Tm(1,:)))*prm.R0, Tm(3,:) - prm.E_z0, '-', 'Color', c2, 'LineWidth', 1.2);
    plot(UV(1,:), UV(2,:), 'k.', 'MarkerSize', 8);
    xlabel('u [mm]'); ylabel('v [mm]');
    title(sprintf('stops: %s — %.1f s, off path %.1f mm, not drawn %.1f mm', stop_sets(m).name, stop_sets(m).T, ...
          stop_sets(m).err.off_path, stop_sets(m).err.not_drawn), 'FontSize', 9);
end
sgtitle('Letter unrolled flat: ideal (grey), via points (black), pen tip (colour)');

%% Save
traj = struct('t', t, 'Q', Q, 'Qd', Qd, 'Qdd', Qdd, 'phase', phase, 'phase_names', phase_names, ...
              'T_arc', T_arc, 'T_letter', T_letter, 'T_line', T_line);
v3folder = fullfile(here,'Fusion','v3');
track = struct('t',t,'q',(Q.*[1;.001;1;1]).');
fid = fopen(fullfile(v3folder,'writing_track.json'),'w');
assert(fid >= 0); fwrite(fid,jsonencode(track),'char'); fclose(fid);
idx = unique([1:10:numel(t), numel(t)]);
track = struct('t',t(idx),'q',(Q(:,idx).*[180/pi;1;180/pi;180/pi]).');
fid = fopen(fullfile(v3folder,'trajectory_check_poses.json'),'w');
assert(fid >= 0); fwrite(fid,jsonencode(track),'char'); fclose(fid);
via  = struct('seq', seq, 'UV', UV, 'is_node', is_node, 'P_letter', Pl, 'N_letter', Nl, 'Q_letter', Ql, ...
              'P_arc', P_arc, 'th_arc', th_arc, 'Q_arc', Q_arc, 'P_line', P_line, 'th_line', th_line, 'Q_line', Q_line, ...
              'P_home', P_home, 'th_home', th_home, 'Q_home', Q_home, 'P_mid', P_mid);
compare = rmfield(stop_sets, {'pieces', 't', 'Q'});
save(fullfile(here, 'step4_trajectory.mat'), 'traj', 'via', 'cfg', 'home', 'arc_bulge', 'n_via', 'chk', 'L', 'compare');
figdir = fullfile(here, 'figures');
if ~exist(figdir, 'dir'), mkdir(figdir); end
exportgraphics(fig1, fullfile(figdir, 'step4_task_3d.png'), 'Resolution', 150);
exportgraphics(fig2, fullfile(figdir, 'step4_home_moves.png'), 'Resolution', 150);
exportgraphics(fig3, fullfile(figdir, 'step4_joints.png'), 'Resolution', 150);
exportgraphics(fig4, fullfile(figdir, 'step4_letter_traced.png'), 'Resolution', 150);
fprintf('\nSaved step4_trajectory.mat and figures/step4_*.png\n');

%% Local functions
function [pieces, stretch] = plan_letter(Ql, stop_idx, L, time_scale)
% One "lines and parabolas" run between consecutive stops. The time of each
% segment is the minimum rest-to-rest time of its slowest joint (Ex3.2), so all
% joints reach every via point together. If the parabolas of a run do not fit
% (they would overlap), the run's times are stretched by 5% until they do.
pieces = struct('T', {}, 'fun', {}, 'phase', {});  stretch = 1;
for s = 1:numel(stop_idx)-1
    Qv  = Ql(:, stop_idx(s):stop_idx(s+1));
    dQ  = abs(diff(Qv, 1, 2));
    tau = time_scale*max(traj_min_time(dQ, repmat(L.vmax, 1, size(dQ, 2)), repmat(L.amax, 1, size(dQ, 2))), [], 1);
    f = 1;
    for it = 1:200
        [~, ~, ~, info] = traj_lines_parabolas(Qv, tau*f, L.amax, 0);
        if info.ok, break; end
        f = f*1.05;
    end
    assert(info.ok, 'lines and parabolas: no feasible timing found');
    tau = tau*f;  stretch = max(stretch, f);
    pieces(end+1) = struct('T', sum(tau), 'fun', @(tl) traj_lines_parabolas(Qv, tau, L.amax, tl), 'phase', 2); %#ok<AGROW>
end
end

function [t, Q, Qd, Qdd, phase] = sample_pieces(pieces, dt)
% Put the pieces one after the other and sample them every dt.
Ttot = sum([pieces.T]);
t = 0:dt:Ttot;
if t(end) < Ttot - 1e-9, t(end+1) = Ttot; end
Q = zeros(4, numel(t));  Qd = Q;  Qdd = Q;  phase = zeros(1, numel(t));
t0 = 0;
for k = 1:numel(pieces)
    idx = t >= t0 - 1e-12 & t < t0 + pieces(k).T - 1e-12;
    if k == numel(pieces), idx = t >= t0 - 1e-12; end
    [Q(:,idx), Qd(:,idx), Qdd(:,idx)] = pieces(k).fun(min(max(t(idx) - t0, 0), pieces(k).T));
    phase(idx) = pieces(k).phase;
    t0 = t0 + pieces(k).T;
end
end

function e = letter_errors(Q, L, Pideal, r_wall, dr_wall)
% How well the pen tip traces the ideal letter.
n = size(Q, 2);  T = zeros(3, n);  N = zeros(3, n);
for i = 1:n, [T(:,i), N(:,i)] = RPRRdir(Q(:,i), L); end
d2 = zeros(n, size(Pideal, 2));
for c = 1:3, d2 = d2 + (T(c,:).' - Pideal(c,:)).^2; end
e.off_path  = sqrt(max(min(d2, [], 2)));      % tip -> nearest ideal point
e.not_drawn = sqrt(max(min(d2, [], 1)));      % ideal point -> nearest tip position
rho = vecnorm(T(1:2,:));
e.into_wall = max(rho - r_wall(T(3,:)));      % > 0: past the wall
nw = [T(1,:)./rho; T(2,:)./rho; -dr_wall(T(3,:))];  nw = nw./vecnorm(nw);
e.tilt_deg  = max(acosd(min(1, sum(nw.*N))));
end

function ok = pose_ok(T, th, L, r_wall, chk)
% Can the pen tip be at T with pen angle th? Joint limits with margin, arm off the wall.
rho = norm(T(1:2));
n = [cos(th)*T(1:2)/rho; sin(th)];
[Q, ok] = RPRRinv(T, n, L);
if ~ok, return; end
ok = all(Q >= L.qmin + chk.q & Q <= L.qmax - chk.q);
if ~ok, return; end
[~, ~, Pj] = RPRRdir(Q, L);
[c, low] = body_clearance(Pj, r_wall);
ok = c >= chk.body && low >= chk.floor && r_wall(T(3)) - rho >= -1e-9;
end

function [c, low] = body_clearance(Pj, r_wall)
% Smallest distance (measured horizontally) between the wall and the q3 link
% or the pen body, and the lowest height of those points. The last half of
% the pen is left out: on the letter its tip touches the wall.
J = Pj(:,3);  W = Pj(:,4);  T = Pj(:,5);
s = linspace(0, 1, 7);
pts = [J + (W - J)*s, W + (T - W)*[0.25 0.5]];
c   = min(r_wall(pts(3,:)) - vecnorm(pts(1:2,:)));
low = min(pts(3,:));
end

function [C, ptr] = dp_pen(P, th_end, thg, feas)
% Smoothest pen-angle sequence along the points P (dynamic programming).
% P(:,1) is on the letter, where the pen angle is th_end; P(:,end) is Home.
% C(g): smallest sum of squared angle steps that ends at Home with angle thg(g).
N = size(P, 2);  ng = numel(thg);
ok = false(N, ng);
for k = 2:N
    for g = 1:ng, ok(k,g) = feas(P(:,k), thg(g)); end
end
D = (thg.' - thg).^2;                          % D(i,j): step from angle j to angle i
C = (thg - th_end).^2;  C(~ok(2,:)) = inf;
ptr = zeros(N, ng);
for k = 3:N
    [Cn, p] = min(D + C, [], 2);
    C = Cn.';  C(~ok(k,:)) = inf;  ptr(k,:) = p.';
end
end

function th = dp_back(ptr, gH, thg, th_end)
% Read the best sequence back from Home (angle index gH) to the letter.
N = size(ptr, 1);  g = zeros(1, N);  g(N) = gH;
for k = N:-1:3, g(k-1) = ptr(k, g(k)); end
th = [th_end, thg(g(2:N))];
end

function th = smooth_pen(th, P, feas)
% The search works on a 1-degree grid; this removes the small steps it leaves:
% each angle moves towards the mean of its neighbours as long as the pose stays allowed.
for sweep = 1:300
    for k = 2:numel(th)-1
        target = (th(k-1) + th(k+1))/2;
        for lam = [1 0.5 0.25]
            cand = th(k) + lam*(target - th(k));
            if feas(P(:,k), cand), th(k) = cand; break; end
        end
    end
end
end

function Qv = via_to_joints(P, th, L)
Qv = zeros(4, size(P, 2));
for k = 1:size(P, 2)
    rho = norm(P(1:2,k));
    [Qv(:,k), ok] = RPRRinv(P(:,k), [cos(th(k))*P(1:2,k)/rho; sin(th(k))], L);
    assert(ok, 'Via point %d is out of reach', k);
end
end

function [T, lim] = spline_time(Qv, L)
% Shortest duration of the spline through Qv (points equally spaced in time)
% that respects every joint's speed and acceleration limit. Speeds scale with
% 1/T and accelerations with 1/T^2 (Lez_05 p.11), so one evaluation is enough.
n = size(Qv, 2);
[~, qd, qdd] = traj_cubic_spline(linspace(0, 1, n), Qv, linspace(0, 1, 4000));
Tv = max(abs(qd), [], 2)./L.vmax;
Ta = sqrt(max(abs(qdd), [], 2)./L.amax);
[T, k] = max([Tv; Ta]);
kinds = ["speed" "acceleration"];
lim = sprintf('q%d %s', mod(k-1, 4) + 1, kinds(ceil(k/4)));
end

function P = arc_3pt(A, M, B, u)
% Points on the circle through A, M, B, from A (u = 0) through M to B (u = 1).
e1 = (B - A)/norm(B - A);
wv = M - A;  e2 = wv - (wv.'*e1)*e1;  e2 = e2/norm(e2);
b  = [wv.'*e1; wv.'*e2];  c = [norm(B - A); 0];
cen = [b.'; c.'] \ ([b.'*b; c.'*c]/2);          % centre, in the plane's own coordinates (A at the origin)
R   = norm(cen);
ang = @(p) atan2(p(2) - cen(2), p(1) - cen(1));
wrap = @(a) atan2(sin(a), cos(a));
a0  = ang([0; 0]);
sweep = wrap(ang(b) - a0) + wrap(ang(c) - ang(b));
a   = a0 + u*sweep;
P   = A + e1*(cen(1) + R*cos(a)) + e2*(cen(2) + R*sin(a));
end

function d = dist_to_circle(X, A, M, B)
% Distance of the points X from the circle through A, M, B (in 3D).
e1 = (B - A)/norm(B - A);
wv = M - A;  e2 = wv - (wv.'*e1)*e1;  e2 = e2/norm(e2);  e3 = cross(e1, e2);
b  = [wv.'*e1; wv.'*e2];  c = [norm(B - A); 0];
cen = [b.'; c.'] \ ([b.'*b; c.'*c]/2);  R = norm(cen);
Y = X - A;
in_plane = sqrt((e1.'*Y - cen(1)).^2 + (e2.'*Y - cen(2)).^2) - R;
d = sqrt(in_plane.^2 + (e3.'*Y).^2);
end

function d = dist_to_line(X, A, B)
e = (B - A)/norm(B - A);
Y = X - A;
d = vecnorm(Y - e*(e.'*Y));
end

function s = tern(c, a, b)
if c, s = a; else, s = b; end
end
