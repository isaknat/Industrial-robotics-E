%% Step 2 — size the R-P-R-R robot so it reaches the whole E
% Decision (2026-10-14): ONE bucket for both robots, so the same surface
% is used for our robot and for the Epson C4 part (Project A).
% The bucket and the E come from step 1 (step1_E_points.mat); the robot's
% dimensions come from robot_params.m. Both are still PLACEHOLDERS.
%
% Checks, for every point of the E:
%   1. our robot reaches it with the pen perpendicular to the wall (inverse kinematics)
%   2. inverse and direct kinematics agree (round trip)
%   3. the slider stays out of the column and the rail stays inside the bucket
%   4. the Epson C4 (simplified) can also reach it -> same bucket works for both
% Then a design map shows which shoulder heights and rail angles would work.

clear; close all; clc;

%% Inputs
S1 = load('step1_E_points.mat');           % bucket + E from step 1
prm = S1.params;  strokes = S1.strokes;
L = robot_params();                         % our robot's dimensions

% Check settings
q2_min_allowed = 30;    % [mm] slider cannot sit closer than this to the axis (column)   PLACEHOLDER
wall_clear     = 40;    % [mm] the rail end must stay this far inside the wall           PLACEHOLDER

% Epson C4-A601, simplified to its two arm links in the vertical plane
% (C4 manipulator manual Rev.20, §2.4.1 p.15 and §2.5.1 p.17). Base on the bucket axis at z = 0.
C4.J2     = [100; 320]; % [mm] shoulder (Joint #2) position: 100 mm out from the axis, 320 mm up
C4.Lu     = 250;        % [mm] upper arm, J2 -> J3
C4.Lf     = 250;        % [mm] forearm, J3 -> P point (wrist centre)
C4.dmin   = 170;        % [mm] closest the P point gets to J2 (manual p.17: R167 / R191.5)
C4.L_tool = 100;        % [mm] P point -> pen tip = 65 to the flange + 35 pen          PLACEHOLDER

r_wall = @(z) prm.R0 + prm.A*sin(2*pi*prm.k*z/prm.H);

%% 1–2. Inverse kinematics for every point, checked with direct kinematics
for s = 1:numel(strokes)
    m = size(strokes(s).P, 2);
    strokes(s).Q   = nan(4, m);
    strokes(s).ok  = false(1, m);
    strokes(s).err = nan(1, m);
    for i = 1:m
        [Q, ok] = RPRRinv(strokes(s).P(:,i), strokes(s).n(:,i), L);
        strokes(s).Q(:,i) = Q;  strokes(s).ok(i) = ok;
        if ok
            [T, n] = RPRRdir(Q, L);
            strokes(s).err(i) = max(norm(T - strokes(s).P(:,i)), norm(n - strokes(s).n(:,i)));
        end
    end
end
Qall  = [strokes.Q];
okall = [strokes.ok];

%% 3. Slider and rail
q2_lo = min(Qall(2,:));  q2_hi = max(Qall(2,:));
rail_end_r = max(q2_hi, 0)*cos(L.alpha);            % horizontal reach of the rail end
rail_end_z = L.h + max(q2_hi, 0)*sin(L.alpha);
rail_ok = all(okall) && q2_lo >= q2_min_allowed && rail_end_r <= r_wall(rail_end_z) - wall_clear;

%% 4. Epson C4: can its wrist centre reach every point with the pen square to the wall?
Pall = [strokes.P];  Nall = [strokes.n];
rho_T = vecnorm(Pall(1:2,:));                        % distance from the axis
n_rho = sum(Pall(1:2,:).*Nall(1:2,:))./rho_T;        % pen direction in the vertical plane
P_c4  = [rho_T - C4.L_tool*n_rho;  Pall(3,:) - C4.L_tool*Nall(3,:)];
d_c4  = vecnorm(P_c4 - C4.J2);
c4_ok = all(d_c4 >= C4.dmin & d_c4 <= C4.Lu + C4.Lf);

%% Summary
fprintf('Bucket: R0 = %.0f mm, A = %.1f mm, H = %.0f mm (from step 1)\n', prm.R0, prm.A, prm.H);
fprintf('Robot:  h = %.0f mm, alpha = %.0f deg, L3 = %.0f mm, L4 = %.0f mm, solution %+d\n', ...
        L.h, rad2deg(L.alpha), L.L3, L.L4, L.sol);
fprintf('Reaches %d of %d points of the E\n', nnz(okall), numel(okall));
fprintf('Round trip (inverse -> direct) largest error: %.1e\n', max([strokes.err]));
fprintf('Joint ranges over the whole E:\n');
fprintf('  q1 %7.1f to %7.1f deg\n',  rad2deg(min(Qall(1,:))), rad2deg(max(Qall(1,:))));
fprintf('  q2 %7.1f to %7.1f mm  (slider travel %.0f mm)\n', q2_lo, q2_hi, q2_hi - q2_lo);
fprintf('  q3 %7.1f to %7.1f deg\n',  rad2deg(min(Qall(3,:))), rad2deg(max(Qall(3,:))));
fprintf('  q4 %7.1f to %7.1f deg\n',  rad2deg(min(Qall(4,:))), rad2deg(max(Qall(4,:))));
fprintf('Rail end at %.0f mm from the axis, wall there at %.0f mm -> %s\n', ...
        rail_end_r, r_wall(rail_end_z), tern(rail_ok, 'OK', 'NOT OK'));
for s = 2:4                                          % horizontal strokes: only q1 should move
    fprintf('  %-6s stroke: q2–q4 change by at most %.1e (only q1 moves)\n', ...
            strokes(s).name, max(max(abs(strokes(s).Q(2:4,:) - strokes(s).Q(2:4,1)))));
end
fprintf('Epson C4: shoulder-to-wrist distance %.0f to %.0f mm (allowed %.0f to %.0f) -> %s\n', ...
        min(d_c4), max(d_c4), C4.dmin, C4.Lu + C4.Lf, tern(c4_ok, 'same bucket OK', 'NOT OK'));

%% Figure 1 — side cut: both robots at several points of the vertical stroke
fig1 = figure('Name', 'Step 2 — side cut', 'Color', 'w');
hold on; axis equal; grid on;
zz = linspace(0, prm.H, 200);
plot(r_wall(zz), zz, 'k-', 'LineWidth', 1.2);
plot([0 0], [0 max(prm.H, C4.J2(2) + C4.Lu) + 30], 'k--');
sp = strokes(1);
plot(vecnorm(sp.P(1:2,:)), sp.P(3,:), 'o', 'Color', [0.83 0.33 0.49], 'MarkerFaceColor', [0.83 0.33 0.49], 'MarkerSize', 4);
plot([0 rail_end_r], [L.h rail_end_z], '-', 'Color', [0.52 0.72 0.92], 'LineWidth', 6);   % rail
show = 1:3:size(sp.P, 2);
for i = show
    % C4 (elbow up), dashed
    P = P_c4(:, i);  d = norm(P - C4.J2);
    if d <= C4.Lu + C4.Lf
        th = atan2(P(2) - C4.J2(2), P(1) - C4.J2(1));  be = acos(d/(C4.Lu + C4.Lf));
        J3 = C4.J2 + C4.Lu*[cos(th + be); sin(th + be)];
        plot([0 0 C4.J2(1) J3(1) P(1) vecnorm(sp.P(1:2,i))], [0 C4.J2(2) C4.J2(2) J3(2) P(2) sp.P(3,i)], ...
             '--', 'Color', [0.85 0.35 0.19 0.6], 'LineWidth', 1.2);
    end
    % our robot, solid
    if sp.ok(i)
        [~, ~, Pr] = RPRRdir(sp.Q(:,i), L);
        rho = vecnorm(Pr(1:2,:));
        plot(rho(2:4), Pr(3,2:4), '-o', 'Color', [0.22 0.54 0.87], 'LineWidth', 2, 'MarkerSize', 4);
        plot(rho(4:5), Pr(3,4:5), '-', 'Color', [0.5 0.47 0.87], 'LineWidth', 2);
    end
end
plot([0 0], [0 L.h], '-', 'Color', [0.22 0.54 0.87], 'LineWidth', 5);   % column
xlabel('distance from the axis [mm]'); ylabel('z [mm]');
title('Side cut: our robot (solid) and Epson C4 (dashed) at every 3rd point');

%% Figure 2 — joint values along the vertical stroke
fig2 = figure('Name', 'Step 2 — joint values', 'Color', 'w');
zs = sp.P(3,:);
subplot(3,1,1); plot(zs, sp.Q(2,:), '.-'); grid on; ylabel('q2 [mm]'); title('Vertical stroke of the E (q1 fixed)');
subplot(3,1,2); plot(zs, rad2deg(sp.Q(3,:)), '.-'); grid on; ylabel('q3 [deg]');
subplot(3,1,3); plot(zs, rad2deg(sp.Q(4,:)), '.-'); grid on; ylabel('q4 [deg]'); xlabel('height z [mm]');

%% Figure 3 — design map: which shoulder heights and rail angles work (L3, L4 as in robot_params)
h_grid = 100:5:400;  a_grid = -30:1:45;
travel = nan(numel(a_grid), numel(h_grid));
for ih = 1:numel(h_grid)
    for ia = 1:numel(a_grid)
        Lt = L;  Lt.h = h_grid(ih);  Lt.alpha = deg2rad(a_grid(ia));
        q2 = nan(1, size(sp.P, 2));  good = true;
        for i = 1:size(sp.P, 2)                     % the vertical stroke is enough (see summary)
            [Q, ok] = RPRRinv(sp.P(:,i), sp.n(:,i), Lt);
            if ~ok, good = false; break; end
            q2(i) = Q(2);
        end
        if ~good, continue; end
        re_r = max(q2)*cos(Lt.alpha);  re_z = Lt.h + max(q2)*sin(Lt.alpha);
        if min(q2) >= q2_min_allowed && re_r <= r_wall(re_z) - wall_clear
            travel(ia, ih) = max(q2) - min(q2);
        end
    end
end
fig3 = figure('Name', 'Step 2 — design map', 'Color', 'w');
imagesc(h_grid, a_grid, travel, 'AlphaData', ~isnan(travel)); axis xy; hold on;
cb = colorbar; cb.Label.String = 'slider travel q2 needed [mm]';
plot(L.h, rad2deg(L.alpha), 'kp', 'MarkerFaceColor', 'y', 'MarkerSize', 14);
xline(C4.J2(2), 'r--', 'C4 shoulder height');
xlabel('shoulder height h [mm]'); ylabel('rail angle \alpha [deg]');
title(sprintf('Coloured = works (reach + rail fits), for L3 = %.0f mm, L4 = %.0f mm. Star = current', L.L3, L.L4));
set(gca, 'Color', [0.94 0.94 0.94]);

%% Save
outdir = fileparts(mfilename('fullpath'));
save(fullfile(outdir, 'step2_robot.mat'), 'L', 'strokes', 'C4', 'q2_min_allowed', 'wall_clear');
figdir = fullfile(outdir, 'figures');
if ~exist(figdir, 'dir'), mkdir(figdir); end
exportgraphics(fig1, fullfile(figdir, 'step2_side_cut.png'), 'Resolution', 150);
exportgraphics(fig2, fullfile(figdir, 'step2_joint_values.png'), 'Resolution', 150);
exportgraphics(fig3, fullfile(figdir, 'step2_design_map.png'), 'Resolution', 150);
fprintf('Saved step2_robot.mat and figures/step2_*.png\n');

%% Local functions
function out = tern(c, a, b)
if c, out = a; else, out = b; end
end
