%% Step 4 — trajectory: Home -> arc -> letter E -> straight line -> Home
% Project PDF p.4–5 (requirement 2):
%   A  Home -> first point of the E along a circular arc,   cubic spline in joint space
%   B  the E itself,                                         lines and parabolas in joint space
%   C  last point of the E -> Home along a straight line,    cubic spline in joint space
%   D  checks (joint limits, motor limits, pen on the wall), plots, save for step 5
%
% Interpretation (to confirm with the TA): the arc and the line are shapes in space.
% We pick via points on them, solve the inverse kinematics there, and put a cubic
% spline through the joint values. The pen follows the shape exactly at the via
% points and approximately in between.
%
% Order of the E (no pen lifts, each stroke starts and stops at rest):
%   1 top bar right -> left, 2 spine top -> middle, 3 middle bar out,
%   4 middle bar back, 5 spine middle -> bottom, 6 bottom bar left -> right.
%
% Check points (Lez_05): after planning, the script looks between the via points.
% Where the pen leaves the wall by more than tol_wall on the E, or where the arc or
% line crosses a joint limit or the wall, it adds a via point there and plans again.
%
% Every value marked PLACEHOLDER is a starting number, NOT a group decision.

clear; close all; clc;

%% Inputs
S1 = load('step1_E_points.mat');           % bucket + E from step 1
prm = S1.params;
L = robot_params();

% Motor limits at the JOINTS [q1; q2; q3; q4] in [rad; mm; rad; rad] per s and s^2.
% Not from a datasheet: they depend on the transmission chosen in steps 5–6.  PLACEHOLDER
vmax = [deg2rad(90);  100; deg2rad(90);  deg2rad(90)];
amax = [deg2rad(300); 500; deg2rad(300); deg2rad(300)];

% Home: pen tip about 55 mm inside the wall, just above the bottom of the E, pen tilted
% 40 deg up. The robot only reaches a band about 80 mm wide along the wall (see the
% step 3 workspace), so Home cannot be far from the wall. It sits low because the
% straight line back from the last point of the E (bottom right, where the wall
% leans in at 31.6 deg) must go down or sideways, or it would cut through the wall.  PLACEHOLDER
Q_home = [0; 70; deg2rad(-50); deg2rad(80)];

v_pen        = 50;    % [mm/s] wanted pen speed along the E (slower if a motor limit needs it)   PLACEHOLDER
pts_per_wave = prm.pts_per_wave;  % via points per wave length on the spine (step 1 uses 12)
arc_bulge    = 30;    % [mm] the arc's middle point is this much closer to the axis than the chord   PLACEHOLDER
n_arc        = 11;    % via points on the arc to start with (check points may add more)  PLACEHOLDER
n_line       = 11;    % via points on the line to start with (check points may add more) PLACEHOLDER
dt           = 1e-3;  % [s] time step of the saved trajectory
tol_wall     = 0.5;   % [mm] allowed distance between pen tip and wall along the E (check points)  PLACEHOLDER

r_wall = @(z) prm.R0 + prm.A*sin(2*pi*prm.k*z/prm.H);
dr     = @(z) prm.A*(2*pi*prm.k/prm.H)*cos(2*pi*prm.k*z/prm.H);

%% Points of the E on the wall (same wrapping as step 1)
u_L = -prm.E_w/2;  u_R = prm.E_w/2;  u_M = u_L + prm.E_mid*prm.E_w;
h_E = prm.E_h;
n_half = max(2, ceil(pts_per_wave*(h_E/2)/(prm.H/prm.k)) + 1);   % points on half the spine

strokes = struct('name', {}, 'u', {}, 'v', {});
strokes(1) = struct('name', "top bar",          'u', [u_R, u_L], 'v', [h_E, h_E]);
strokes(2) = struct('name', "spine, upper half", 'u', u_L*ones(1, n_half), 'v', linspace(h_E, h_E/2, n_half));
strokes(3) = struct('name', "middle bar, out",  'u', [u_L, u_M], 'v', [h_E/2, h_E/2]);
strokes(4) = struct('name', "middle bar, back", 'u', [u_M, u_L], 'v', [h_E/2, h_E/2]);
strokes(5) = struct('name', "spine, lower half", 'u', u_L*ones(1, n_half), 'v', linspace(h_E/2, 0, n_half));
strokes(6) = struct('name', "bottom bar",       'u', [u_L, u_R], 'v', [0, 0]);

%% Home
[T_home, n_home] = RPRRdir(Q_home, L);
[Q_chk, ok] = RPRRinv(T_home, n_home, L);
assert(ok && norm(Q_chk - Q_home) < 1e-9, ...
       'Q_home is not on the inverse kinematics branch L.sol = %d; choose another Home', L.sol);

%% Part B — the E: lines and parabolas, one stroke at a time
% All joints use the same times td (synchronized). Each joint blends with its own
% acceleration limit. If a parabola does not fit or a joint is too fast, all td of
% that stroke are stretched by 5 % and the stroke is planned again.
% Between via points the pen leaves the wall a little (joint space is not the wall).
% Check points (Lez_05): where it leaves by more than tol_wall, a via point is added
% in the middle of that segment and the stroke is planned again.
partB = struct('t', {}, 'Q', {}, 'Qd', {}, 'Qdd', {});
for s = 1:numel(strokes)
    u = strokes(s).u;  v = strokes(s).v;
    for added = 0:200
        [P, n, Qv] = stroke_points(u, v, prm, r_wall, dr, L);
        td = vecnorm(diff(P, 1, 2))/v_pen;
        for iter = 1:500
            tt = time_grid(sum(td), dt);
            [Q, Qd, Qdd, ok] = lfpb_all(Qv, td, amax, tt);
            if ok && all(max(abs(Qd), [], 2) <= vmax + 1e-9), break; end
            td = 1.05*td;
        end
        assert(ok, 'Stroke "%s": no feasible timing found', strokes(s).name);
        % pen tip vs wall, every 10th sample
        ks = 1:10:numel(tt);
        dev_s = zeros(size(ks));
        for i = 1:numel(ks)
            Ti = RPRRdir(Q(:,ks(i)), L);
            dev_s(i) = norm(Ti(1:2)) - r_wall(Ti(3));
        end
        [worst, iw] = max(abs(dev_s));
        if worst <= tol_wall, break; end
        j = find(tt(ks(iw)) >= [0, cumsum(td)], 1, 'last');     % segment with the worst point
        j = min(j, numel(u) - 1);
        u = [u(1:j), (u(j) + u(j+1))/2, u(j+1:end)];
        v = [v(1:j), (v(j) + v(j+1))/2, v(j+1:end)];
    end
    partB(s) = struct('t', tt, 'Q', Q, 'Qd', Qd, 'Qdd', Qdd);
    strokes(s).u = u;  strokes(s).v = v;
    strokes(s).P = P;  strokes(s).n = n;  strokes(s).Qv = Qv;  strokes(s).td = td;
end

%% Part A — Home -> first point along a circular arc, cubic spline in joint space
P1 = strokes(1).P(:,1);  n1 = strokes(1).n(:,1);
Pm = (T_home + P1)/2;
Pm(1:2) = Pm(1:2) - arc_bulge*Pm(1:2)/norm(Pm(1:2));      % pull the middle point towards the axis
arc = @(s) traj_arc(T_home, Pm, P1, s);
[partA, Parc] = path_spline(arc, n_arc, pen_angle(n_home), pen_angle(n1), L, vmax, amax, dt, r_wall);
assert(norm(partA.Q(:,end) - strokes(1).Qv(:,1)) < 1e-9, 'Arc does not end at the first point of the E');

%% Part C — last point -> Home along a straight line, cubic spline in joint space
Pg = strokes(end).P(:,end);  ng = strokes(end).n(:,end);
straight = @(s) Pg + (T_home - Pg)*s;
[partC, Pline] = path_spline(straight, n_line, pen_angle(ng), pen_angle(n_home), L, vmax, amax, dt, r_wall);
assert(norm(partC.Q(:,end) - Q_home) < 1e-9, 'Line does not end at Home');

%% Put the parts one after the other in time
parts = [partA, partB, partC];
names = ["arc", [strokes.name], "line"];
t = [];  Q = [];  Qd = [];  Qdd = [];  part = [];
t_start = zeros(1, numel(parts));
for p = 1:numel(parts)
    k0 = 1 + (p > 1);                      % skip the first sample: it equals the previous part's last
    t0 = 0;  if p > 1, t0 = t(end); end
    t_start(p) = t0;
    t    = [t,    t0 + parts(p).t(k0:end)];
    Q    = [Q,    parts(p).Q(:,   k0:end)];
    Qd   = [Qd,   parts(p).Qd(:,  k0:end)];
    Qdd  = [Qdd,  parts(p).Qdd(:, k0:end)];
    part = [part, p*ones(1, numel(parts(p).t) - k0 + 1)];
end
isE = part >= 2 & part <= 1 + numel(strokes);

%% Part D — checks on the whole trajectory
% Pen tip, pen direction and joint points at every time step
m = numel(t);
T = zeros(3, m);  N = zeros(3, m);  Pj = zeros(3, 5, m);
for i = 1:m
    [T(:,i), N(:,i), Pj(:,:,i)] = RPRRdir(Q(:,i), L);
end

% 1. Joint limits
lim_lo = min(Q, [], 2);  lim_hi = max(Q, [], 2);
in_limits = lim_lo >= L.qmin - 1e-9 & lim_hi <= L.qmax + 1e-9;

% 2. Motor limits
v_ratio = max(abs(Qd),  [], 2)./vmax;
a_ratio = max(abs(Qdd), [], 2)./amax;

% 3. Pen on the wall along the E: distance from the axis minus wall radius
rho  = vecnorm(T(1:2,:));
dev  = rho - r_wall(T(3,:));               % > 0: pen tip pushed into the wall, < 0: lifted off
% 4. Pen square to the wall: angle between pen direction and wall normal
phi  = atan2(T(2,:), T(1,:));
nw   = [cos(phi); sin(phi); -dr(T(3,:))];  nw = nw./vecnorm(nw);
ang_err = acosd(min(1, sum(N.*nw)));

% 5. Rough collision check: every joint point inside the wall and above the floor
Pr = squeeze(vecnorm(Pj(1:2,:,:)));        % [5 x m] distance from the axis
Pz = squeeze(Pj(3,:,:));
inside = Pr <= r_wall(Pz) + 1e-6 & Pz >= -1e-6;
inside(end,isE) = true;                    % the pen tip touches the wall on the E by design
clear_AC = min(r_wall(T(3,~isE)) - rho(~isE));

%% Summary
fprintf('Trajectory: %.2f s in total, %d samples (dt = %g s)\n', t(end), m, dt);
for p = 1:numel(parts)
    fprintf('  %-20s %6.2f s\n', names(p), parts(p).t(end));
end
fprintf('Joint ranges (limits from robot_params):\n');
unit = ["deg", "mm", "deg", "deg"];  sc = [180/pi, 1, 180/pi, 180/pi];
for j = 1:4
    fprintf('  q%d %8.1f to %8.1f %s  (limit %6.1f to %6.1f) %s\n', j, sc(j)*lim_lo(j), sc(j)*lim_hi(j), ...
            unit(j), sc(j)*L.qmin(j), sc(j)*L.qmax(j), tern(in_limits(j), 'OK', 'OUTSIDE LIMIT'));
end
fprintf('Highest speed and acceleration as a fraction of the motor limit:\n');
for j = 1:4
    fprintf('  q%d  speed %4.2f   acceleration %4.2f\n', j, v_ratio(j), a_ratio(j));
end
fprintf('Pen tip vs wall along the E: %.3f to %.3f mm (tolerance %.1f mm) %s\n', ...
        min(dev(isE)), max(dev(isE)), tol_wall, tern(max(abs(dev(isE))) <= tol_wall, 'OK', 'TOO FAR: add via points'));
fprintf('Pen angle vs wall normal along the E: largest error %.2f deg\n', max(ang_err(isE)));
fprintf('Arc and line stay inside the bucket: closest pen distance to the wall %.1f mm (0 = touching, at the E)\n', clear_AC);
fprintf('All joints inside the wall and above the floor: %s\n', tern(all(inside(:)), 'yes', 'NO'));

%% Figure 1 — joint position, velocity, acceleration
fig1 = figure('Name', 'Step 4 — joints', 'Color', 'w', 'Position', [100 100 1100 800]);
lbl = {'q_1 [deg]', 'q_2 [mm]', 'q_3 [deg]', 'q_4 [deg]'};
ttl = {'position', 'velocity (per s)', 'acceleration (per s^2)'};
data = {Q, Qd, Qdd};
for j = 1:4
    for c = 1:3
        subplot(4, 3, 3*(j-1) + c); hold on; grid on;
        plot(t, sc(j)*data{c}(j,:), 'LineWidth', 1.1);
        if c == 2, yline( sc(j)*vmax(j), 'r--'); yline(-sc(j)*vmax(j), 'r--'); end
        if c == 3, yline( sc(j)*amax(j), 'r--'); yline(-sc(j)*amax(j), 'r--'); end
        if c == 1, yline(sc(j)*L.qmin(j), 'r--'); yline(sc(j)*L.qmax(j), 'r--'); end
        for p = 2:numel(parts), xline(t_start(p), ':', 'Color', [0.5 0.5 0.5]); end
        if j == 1, title(ttl{c}); end
        if c == 1, ylabel(lbl{j}); end
        if j == 4, xlabel('t [s]'); end
    end
end
sgtitle('Joints along the whole task (dotted: start of each part, red: limits)');

%% Figure 2 — path in 3D with the bucket
fig2 = figure('Name', 'Step 4 — path', 'Color', 'w');
hold on; axis equal; grid on; view(35, 20);
th = linspace(prm.phi_c - pi/2, prm.phi_c + pi/2, 90);
zz = linspace(0, prm.H, 120);
[TH, ZZ] = meshgrid(th, zz);  RR = r_wall(ZZ);
surf(RR.*cos(TH), RR.*sin(TH), ZZ, 'FaceAlpha', 0.2, 'EdgeColor', 'none', 'FaceColor', [0.45 0.6 0.9]);
plot3([0 0], [0 0], [0 prm.H], 'k--');
col = [0.9 0.6 0.1; 0.83 0.33 0.49; 0.2 0.6 0.3];
grp = 1 + isE + 2*(part == numel(parts));  % 1 arc, 2 E, 3 line
for g = 1:3
    k = grp == g;
    plot3(T(1,k), T(2,k), T(3,k), '.', 'Color', col(g,:), 'MarkerSize', 4);
end
plot3(Parc(1,:), Parc(2,:), Parc(3,:), 'o', 'Color', col(1,:));
plot3(Pline(1,:), Pline(2,:), Pline(3,:), 'o', 'Color', col(3,:));
plot3(T_home(1), T_home(2), T_home(3), 'ks', 'MarkerFaceColor', 'k');
Ph = Pj(:,:,1);  plot3(Ph(1,:), Ph(2,:), Ph(3,:), 'k-o', 'LineWidth', 2);    % robot at Home
xlabel('x [mm]'); ylabel('y [mm]'); zlabel('z [mm]');
title('Pen tip: arc (orange), E (red), line (green); circles = via points');

%% Figure 3 — pen on the wall along the E
fig3 = figure('Name', 'Step 4 — pen on the wall', 'Color', 'w');
subplot(2, 1, 1); hold on; grid on;
plot(t(isE), dev(isE), 'LineWidth', 1.1);
yline(tol_wall, 'r--'); yline(-tol_wall, 'r--');
ylabel('pen tip - wall [mm]'); title('Distance between pen tip and wall while drawing the E');
subplot(2, 1, 2); hold on; grid on;
plot(t(isE), ang_err(isE), 'LineWidth', 1.1);
ylabel('angle error [deg]'); xlabel('t [s]'); title('Angle between pen and wall normal');

%% Figure 4 — pen tip speed
Td = zeros(3, m);
for i = 1:m
    Sd = RPRRjac(Q(:,i), L)*Qd(:,i);       % S_dot = J*Q_dot; rows 1-3 are the tip velocity
    Td(:,i) = Sd(1:3);
end
fig4 = figure('Name', 'Step 4 — pen speed', 'Color', 'w');
hold on; grid on;
plot(t, vecnorm(Td(1:3,:)), 'LineWidth', 1.1);
for p = 2:numel(parts), xline(t_start(p), ':', 'Color', [0.5 0.5 0.5]); end
xlabel('t [s]'); ylabel('pen tip speed [mm/s]'); title('Pen tip speed along the whole task');

%% Save for step 5 (dynamics) and the Simscape model
outdir = fileparts(mfilename('fullpath'));
settings = struct('vmax', vmax, 'amax', amax, 'Q_home', Q_home, 'v_pen', v_pen, ...
                  'pts_per_wave', pts_per_wave, 'arc_bulge', arc_bulge, 'n_arc', n_arc, ...
                  'n_line', n_line, 'dt', dt);
save(fullfile(outdir, 'step4_trajectory.mat'), 't', 'Q', 'Qd', 'Qdd', 'part', 'names', 't_start', 'settings');
figdir = fullfile(outdir, 'figures');
if ~exist(figdir, 'dir'), mkdir(figdir); end
exportgraphics(fig1, fullfile(figdir, 'step4_joints.png'), 'Resolution', 150);
exportgraphics(fig2, fullfile(figdir, 'step4_path_3d.png'), 'Resolution', 150);
exportgraphics(fig3, fullfile(figdir, 'step4_pen_on_wall.png'), 'Resolution', 150);
exportgraphics(fig4, fullfile(figdir, 'step4_pen_speed.png'), 'Resolution', 150);
fprintf('Saved step4_trajectory.mat and figures/step4_*.png\n');

%% Local functions
function [P, n] = wall_point(u, v, prm, r_wall, dr)
% Flat letter point (u, v) -> point on the wall and pen direction (as in step 1)
z   = prm.E_z0 + v;
phi = prm.phi_c - u/prm.R0;
rho = r_wall(z);
P   = [rho.*cos(phi); rho.*sin(phi); z];
n   = [cos(phi); sin(phi); -dr(z)];
n   = n./vecnorm(n);
end

function [part, Pv] = path_spline(path_fun, n_via, th0, th1, L, vmax, amax, dt, r_wall)
% Spline in joint space along a path in space (arc or line), with check points (Lez_05).
% path_fun(s) gives the point at fraction s of the path (0 = start, 1 = end).
% Between via points the spline can leave the path, cross a joint limit or push the
% pen into the wall. Where that happens, a via point is added in the middle of that
% segment and the spline is planned again.
sv = linspace(0, 1, n_via);                % positions of the via points along the path (0 to 1)
for added = 0:50
    Pv = path_fun(sv);
    [part, tv] = spline_through(Pv, th0, th1, L, vmax, amax, dt, r_wall);
    bad = false(1, numel(part.t));
    for i = 1:10:numel(part.t) - 1
        [T, ~, Pj] = RPRRdir(part.Q(:,i), L);
        bad(i) = any(part.Q(:,i) < L.qmin | part.Q(:,i) > L.qmax) ...
              || norm(T(1:2)) > r_wall(T(3)) + 0.01 ...              % pen into the wall
              || any(vecnorm(Pj(1:2,2:4)) > r_wall(Pj(3,2:4))) || any(Pj(3,:) < 0);
    end
    if ~any(bad), return; end
    j = find(part.t(find(bad, 1)) >= tv, 1, 'last');    % segment of the first bad sample
    sv = [sv(1:j), (sv(j) + sv(j+1))/2, sv(j+1:end)];
end
warning('Arc or line: still leaves the limits or the wall after adding 50 check points');
end

function [P, n, Qv] = stroke_points(u, v, prm, r_wall, dr, L)
% Via points of one stroke on the wall, with pen direction and joint values
[P, n] = wall_point(u, v, prm, r_wall, dr);
Qv = zeros(4, size(P, 2));
for i = 1:size(P, 2)
    [Qv(:,i), ok] = RPRRinv(P(:,i), n(:,i), L);
    assert(ok, 'A point of the E is out of reach');
end
end

function th = pen_angle(n)
% Pen angle above horizontal
th = atan2(n(3), norm(n(1:2)));
end

function tt = time_grid(T_end, dt)
% Times from 0 to T_end in steps of dt, always ending exactly at T_end
tt = 0:dt:T_end;
if T_end - tt(end) > 1e-12, tt = [tt, T_end]; end
end

function [Q, Qd, Qdd, ok] = lfpb_all(Qv, td, amax, tt)
% Lines and parabolas for all four joints with the same times td
Q = zeros(4, numel(tt));  Qd = Q;  Qdd = Q;  ok = true;
for j = 1:4
    [Q(j,:), Qd(j,:), Qdd(j,:), okj] = traj_lfpb(Qv(j,:), td, amax(j), tt);
    ok = ok && okj;
end
end

function [part, tv] = spline_through(Pv, th0, th1, L, vmax, amax, dt, r_wall)
% Via points Pv on a path in space -> joint values -> clamped cubic spline in joint space.
% The pen stays in the arm's vertical plane. Its angle is free between the end points
% (the task only fixes it on the E), and near the wall the robot can only reach some
% angles. So the pen angle at each via point is chosen as smoothly as possible among
% the angles that keep every joint within its limits and inside the wall
% (dynamic programming over a 1 deg grid: smallest sum of squared angle changes).
% The time is then stretched (or shrunk) so the fastest joint just meets its limit.
m   = size(Pv, 2);
ths = deg2rad(-85:85);                     % candidate pen angles
margin = [deg2rad(3); 5; deg2rad(3); deg2rad(3)];   % keep via points a bit inside the joint limits,
                                                    % since the spline can overshoot between them
nth = numel(ths);
Qc  = nan(4, nth, m);                      % joint values for every via point and angle
for i = 1:m
    q1 = atan2(-Pv(1,i), Pv(2,i));
    cand = 1:nth;
    if i == 1, [~, cand] = min(abs(ths - th0)); end   % end points: the angle is given
    if i == m, [~, cand] = min(abs(ths - th1)); end
    for c = cand
        th = ths(c);
        if i == 1, th = th0; elseif i == m, th = th1; end
        n = [-sin(q1)*cos(th); cos(q1)*cos(th); sin(th)];
        [Qi, ok] = RPRRinv(Pv(:,i), n, L);
        if ~ok || any(Qi < L.qmin + margin | Qi > L.qmax - margin), continue; end
        [~, ~, Pj] = RPRRdir(Qi, L);
        if all(vecnorm(Pj(1:2,2:4)) <= r_wall(Pj(3,2:4))) && all(Pj(3,:) >= 0)
            Qc(:,c,i) = Qi;
        end
    end
    assert(any(~isnan(Qc(1,:,i))), 'Via point %d of the arc or line cannot be reached with any pen angle', i);
end
% Dynamic programming: cost(c, i) = smallest sum of squared angle steps to reach angle c at point i
cost = inf(nth, m);  from = zeros(nth, m);
cost(~isnan(Qc(1,:,1)), 1) = 0;
D2 = (ths.' - ths).^2;                     % D2(a, b) = squared step from angle b to angle a
for i = 2:m
    ok_i = ~isnan(Qc(1,:,i));
    [best, arg] = min(D2 + cost(:, i-1).', [], 2);
    cost(ok_i, i) = best(ok_i);  from(ok_i, i) = arg(ok_i);
end
[~, c] = min(cost(:, m));
assert(isfinite(cost(c, m)), 'No smooth sequence of pen angles reaches the end of the arc or line');
Qv = zeros(4, m);
for i = m:-1:1
    Qv(:,i) = Qc(:,c,i);
    c = from(c, i);
end
% Time between via points: what the slowest joint needs at full speed (Lez_05: the
% joint that needs the most time sets the time; the others are synchronized to it)
tv = [0, cumsum(max(abs(diff(Qv, 1, 2))./vmax, [], 1))];
tv = tv/tv(end);                           % 1 s in total for a first try
tt = linspace(0, 1, 2001);
[~, Qd, Qdd] = traj_spline(tv, Qv, tt);
k  = 1.01*max([max(abs(Qd), [], 2)./vmax; sqrt(max(abs(Qdd), [], 2)./amax)]);   % 1 % margin for sampling
tv = k*tv;                                 % speed / k, acceleration / k^2
tt = time_grid(tv(end), dt);
[Q, Qd, Qdd] = traj_spline(tv, Qv, tt);
part = struct('t', tt, 'Q', Q, 'Qd', Qd, 'Qdd', Qdd);
end

function out = tern(c, a, b)
if c, out = a; else, out = b; end
end
