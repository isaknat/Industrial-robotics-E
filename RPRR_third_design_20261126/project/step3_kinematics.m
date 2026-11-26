%% Step 3 — kinematic model of the R-P-R-R robot (Project requirements 1, 4, 5)
% Functions (same roles as SCARAdir / SCARAinv / SCARAjac / SCARAjacP in the
% 28/09 handout):  RPRRdir, RPRRinv, RPRRjac, RPRRjacP, plus RPRRscore.
% This script
%   1. tests them against each other and against numerical derivatives
%   2. draws the workspace in the arm plane, with joint limits (requirement 4)
%   3. draws the kinematic quality map, 1/condition number (requirement 5)
%   4. checks the quality and the distance from singularity along the E
% Robot dimensions, joint limits and Lc come from robot_params (PLACEHOLDERS);
% the bucket and the E come from step 1.

clear; close all; clc;
L   = robot_params();
S1  = load('step1_E_points.mat');
prm = S1.params;  strokes = S1.strokes;
r_wall = @(z) prm.R0 + prm.A*sin(2*pi*prm.k*z/prm.H);
inlim  = @(Q) all(Q >= L.qmin - 1e-9 & Q <= L.qmax + 1e-9);

%% 1. Tests
rng(1);
nt = 200;  err = zeros(nt, 6);
for t = 1:nt
    Q   = L.qmin + rand(4,1).*(L.qmax - L.qmin);       % random pose within the limits
    Qp  = randn(4,1).*[1; 200; 1; 1];                  % random joint speeds
    Qpp = randn(4,1).*[1; 200; 1; 1];                  % random joint accelerations
    [~, ~, ~, S] = RPRRdir(Q, L);
    J  = RPRRjac(Q, L);
    Jp = RPRRjacP(Q, Qp, L);
    % (a) J against central differences of S = F(Q)
    Jnum = zeros(4);
    for i = 1:4
        dq = zeros(4,1);  dq(i) = 1e-6*max(1, abs(L.qmax(i)));
        [~, ~, ~, Sa] = RPRRdir(Q + dq, L);  [~, ~, ~, Sb] = RPRRdir(Q - dq, L);
        Jnum(:,i) = (Sa - Sb)/(2*dq(i));
    end
    err(t,1) = norm(J - Jnum)/norm(J);
    % (b) Jp against the change of J along the motion
    dt = 1e-6;
    Jpnum = (RPRRjac(Q + Qp*dt, L) - RPRRjac(Q - Qp*dt, L))/(2*dt);
    err(t,2) = norm(Jp - Jpnum)/max(norm(Jp), 1e-12);
    th3 = L.alpha + Q(3);  th4 = th3 + Q(4);
    rho = Q(2)*cos(L.alpha) + L.L3*cos(th3) + L.L4*cos(th4);   % pen tip distance from the axis
    % (c) inverse kinematics -> direct kinematics round trip (pen tip in front of the axis)
    [T, n] = RPRRdir(Q, L);
    Lt = L;  Lt.sol = -1;  [Qa, oka] = RPRRinv(T, n, Lt);
    Lt.sol = +1;           [Qb, okb] = RPRRinv(T, n, Lt);
    wrap = @(v) [atan2(sin(v(1)), cos(v(1))); v(2); atan2(sin(v(3:4)), cos(v(3:4)))];
    if rho > 1                                     % behind the axis q1 would flip by 180 deg
        err(t,3) = min([norm(wrap(Qa - Q)), norm(wrap(Qb - Q))]);  % the pose is one of the two solutions
    end
    % (d) determinant formula
    err(t,4) = abs(det(J) - (-rho*L.L3*cos(Q(3))))/max(abs(det(J)), 1);
    % (e) velocity and acceleration inversion (Lez_02 p.22): Q_dot = J\S_dot, Q_ddot = J\(S_ddot - Jp*Q_dot)
    Sp  = J*Qp;  Spp = Jp*Qp + J*Qpp;
    err(t,5) = norm(J\Sp - Qp)/norm(Qp);
    err(t,6) = norm(J\(Spp - Jp*Qp) - Qpp)/norm(Qpp);
end
names = ["J vs numerical derivative", "Jp vs numerical derivative", "inverse -> direct round trip", ...
         "det(J) = -rho*L3*cos(q3)", "velocity inversion", "acceleration inversion"];
fprintf('Tests on %d random poses (largest error):\n', nt);
for i = 1:6, fprintf('  %-30s %.1e\n', names(i), max(err(:,i))); end

%% Joint values and score along the E
for s = 1:numel(strokes)
    m = size(strokes(s).P, 2);
    strokes(s).Q = nan(4, m);  strokes(s).score = nan(1, m);
    for i = 1:m
        [Q, ok] = RPRRinv(strokes(s).P(:,i), strokes(s).n(:,i), L);
        if ok
            strokes(s).Q(:,i) = Q;
            strokes(s).score(i) = RPRRscore(Q, L);
        end
    end
end
Qall = [strokes.Q];  score_E = [strokes.score];
fprintf('Along the E: score 1/kappa from %.2f to %.2f (0.2 = acceptable, Lez_04 p.23)\n', min(score_E), max(score_E));
fprintf('Along the E: q3 stays %.0f deg away from the singular +-90 deg\n', 90 - rad2deg(max(abs(Qall(3,:)))));
fprintf('Along the E: all joints within the limits: %s\n', string(all(arrayfun(@(i) inlim(Qall(:,i)), 1:size(Qall,2)))));

%% 2. Workspace in the arm plane (q1 = 0, so the plane is y-z; it turns with q1)
% Pen angles from straight down to straight up, split into 36 slices of 5 deg;
% each slice is tested at its middle angle. cover = degrees of pen tilt that work (0-180).
rho_g = 0:5:500;  z_g = -50:5:600;  th_g = deg2rad(-87.5:5:87.5);
cover = zeros(numel(z_g), numel(rho_g));             % reachable pen tilt [deg, out of 180]
for ir = 1:numel(rho_g)
    for iz = 1:numel(z_g)
        T = [0; rho_g(ir); z_g(iz)];
        nok = 0;
        for it = 1:numel(th_g)
            n = [0; cos(th_g(it)); sin(th_g(it))];
            for sol = [-1 1]                          % either of the two solutions
                Lt = L;  Lt.sol = sol;
                [Q, ok] = RPRRinv(T, n, Lt);
                if ok && inlim(Q), nok = nok + 1; break; end
            end
        end
        cover(iz, ir) = nok*5;
    end
end
reach = cover > 0;

% Score map with the pen horizontal (solution and limits as for the task)
score_map = nan(numel(z_g), numel(rho_g));
for ir = 1:numel(rho_g)
    for iz = 1:numel(z_g)
        [Q, ok] = RPRRinv([0; rho_g(ir); z_g(iz)], [0; 1; 0], L);
        if ok && inlim(Q), score_map(iz, ir) = RPRRscore(Q, L); end
    end
end

zz = linspace(0, prm.H, 200);
spine = strokes(1);
rho_E = vecnorm(spine.P(1:2,:));  z_E = spine.P(3,:);

fig1 = figure('Name', 'Step 3 — workspace', 'Color', 'w');
imagesc(rho_g, z_g, cover, 'AlphaData', reach); axis xy equal tight; hold on;
cb = colorbar; cb.Label.String = 'range of reachable pen angles [deg]';
plot(r_wall(zz), zz, 'k-', 'LineWidth', 1.5);
plot(rho_E, z_E, 'o', 'Color', [0.83 0.33 0.49], 'MarkerFaceColor', [0.83 0.33 0.49], 'MarkerSize', 3);
plot([0 0], [0 L.h], '-', 'Color', [0.22 0.54 0.87], 'LineWidth', 4);
xlabel('distance from the axis [mm]'); ylabel('z [mm]');
title('Workspace (pen tip) in the arm plane, with joint limits; wall and E on top');
set(gca, 'Color', [0.94 0.94 0.94]);

fig2 = figure('Name', 'Step 3 — score map', 'Color', 'w');
imagesc(rho_g, z_g, score_map, 'AlphaData', ~isnan(score_map)); axis xy equal tight; hold on;
clim([0 0.4]);  cb = colorbar; cb.Label.String = '1/\kappa (0 = singular; scale stops at 0.4 so differences show)';
contour(rho_g, z_g, score_map, [0.2 0.2], 'r-', 'LineWidth', 1.2);
plot(r_wall(zz), zz, 'k-', 'LineWidth', 1.5);
plot(rho_E, z_E, 'o', 'Color', [0.83 0.33 0.49], 'MarkerFaceColor', [0.83 0.33 0.49], 'MarkerSize', 3);
plot([0 0], [0 L.h], '-', 'Color', [0.22 0.54 0.87], 'LineWidth', 4);
xlabel('distance from the axis [mm]'); ylabel('z [mm]');
title(sprintf('Score map, pen horizontal (red line: 1/\\kappa = 0.2), L_c = %.0f mm', L.Lc));
set(gca, 'Color', [0.94 0.94 0.94]);

fig3 = figure('Name', 'Step 3 — score along the E', 'Color', 'w');
hl = plot(z_E, spine.score, '.-', 'LineWidth', 1.2); hold on; grid on;
yline(0.2, 'r--', 'acceptable (0.2)');
for s = 2:4
    hl(end+1) = plot(prm.E_z0 + strokes(s).v(1), strokes(s).score(1), 's', 'MarkerSize', 8, 'LineWidth', 1.2); %#ok<SAGROW>
end
ylim([0 1]); xlabel('height z [mm]'); ylabel('1/\kappa');
legend(hl, 'vertical stroke', 'top bar', 'middle bar', 'bottom bar', 'Location', 'best');
title('Score along the E (pen perpendicular to the wall)');

%% 5. Design map with the score: worst score along the E for each shoulder height and rail angle
% Same feasibility rules as the step 2 design map (reach, slider out of the column,
% rail inside the bucket); colour = lowest 1/kappa along the vertical stroke
% (the horizontal strokes have the same in-plane poses as its end and middle points).
S2 = load('step2_robot.mat', 'q2_min_allowed', 'wall_clear');
h_grid = 100:5:400;  a_grid = -30:1:45;
worst = nan(numel(a_grid), numel(h_grid));
for ih = 1:numel(h_grid)
    for ia = 1:numel(a_grid)
        Lt = L;  Lt.h = h_grid(ih);  Lt.alpha = deg2rad(a_grid(ia));
        q2 = nan(1, size(spine.P, 2));  sc = q2;  good = true;
        for i = 1:size(spine.P, 2)
            [Q, ok] = RPRRinv(spine.P(:,i), spine.n(:,i), Lt);
            if ~ok, good = false; break; end
            q2(i) = Q(2);  sc(i) = RPRRscore(Q, Lt);
        end
        if ~good, continue; end
        re_r = max(q2)*cos(Lt.alpha);  re_z = Lt.h + max(q2)*sin(Lt.alpha);
        if min(q2) >= S2.q2_min_allowed && re_r <= r_wall(re_z) - S2.wall_clear
            worst(ia, ih) = min(sc);
        end
    end
end
[best, ib] = max(worst(:));  [ia_b, ih_b] = ind2sub(size(worst), ib);
fprintf('Design map: best worst-case score %.2f at h = %d mm, alpha = %d deg (current: %.2f)\n', ...
        best, h_grid(ih_b), a_grid(ia_b), min(spine.score));

fig4 = figure('Name', 'Step 3 — design map with score', 'Color', 'w');
imagesc(h_grid, a_grid, worst, 'AlphaData', ~isnan(worst)); axis xy; hold on;
cb = colorbar; cb.Label.String = 'lowest 1/\kappa along the E';
plot(L.h, rad2deg(L.alpha), 'kp', 'MarkerFaceColor', 'y', 'MarkerSize', 14);
plot(h_grid(ih_b), a_grid(ia_b), 'ko', 'MarkerFaceColor', 'w', 'MarkerSize', 8);
xlabel('shoulder height h [mm]'); ylabel('rail angle \alpha [deg]');
title(sprintf('Worst score along the E (L3 = %.0f, L4 = %.0f, L_c = %.0f mm). Star = current, circle = best', L.L3, L.L4, L.Lc));
set(gca, 'Color', [0.94 0.94 0.94]);

%% Save
outdir = fileparts(mfilename('fullpath'));
save(fullfile(outdir, 'step3_kinematics.mat'), 'L', 'strokes', 'rho_g', 'z_g', 'cover', 'reach', 'score_map', ...
     'h_grid', 'a_grid', 'worst');
figdir = fullfile(outdir, 'figures');
exportgraphics(fig1, fullfile(figdir, 'step3_workspace.png'), 'Resolution', 150);
exportgraphics(fig2, fullfile(figdir, 'step3_score_map.png'), 'Resolution', 150);
exportgraphics(fig3, fullfile(figdir, 'step3_score_along_E.png'), 'Resolution', 150);
exportgraphics(fig4, fullfile(figdir, 'step3_design_score.png'), 'Resolution', 150);
fprintf('Saved step3_kinematics.mat and figures/step3_*.png\n');
