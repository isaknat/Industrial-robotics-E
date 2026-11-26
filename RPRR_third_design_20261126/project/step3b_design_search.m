%% Step 3b — coarse search over the robot's dimensions for the best kinematic score
% Tries every combination of shoulder height h, rail angle alpha, q3 link L3 and
% pen length L4 on a COARSE grid. For each one that can be built (rules below),
% computes the score 1/kappa (RPRRscore) at every point of the E, and keeps the
% lowest and highest value. The weakest point is what matters, so designs are
% ranked by their lowest score.
%
% The score depends on the characteristic length Lc (our own choice, see
% RPRRscore), so the lowest score is computed for three values of Lc and the
% ranking uses the worst of the three. A design that is good only for one Lc
% does not win.
%
% Rules a design must satisfy (the first three are the step 2 rules):
%   1. reaches every point of the E with the pen perpendicular to the wall
%   2. the slider stays out of the column (q2 >= q2_min_allowed)
%   3. the rail end stays inside the bucket (wall_clear)
%   4. the q4 joint stays at least wrist_clear away from the wall
% Joint limits are NOT used as a rule (they are placeholders); the joint ranges
% each design needs are reported instead.
%
% Only kinematics: torque (step 5) may prefer other dimensions. robot_params is
% NOT changed by this script. A finer grid around the good region comes later.
%
% Only the vertical stroke is evaluated: the horizontal strokes have the same
% poses in the arm plane as points of the vertical stroke, and the score does
% not depend on q1.

clear; close all; clc;
L0  = robot_params();
S1  = load('step1_E_points.mat');
prm = S1.params;  spine = S1.strokes(1);
S2  = load('step2_robot.mat', 'q2_min_allowed', 'wall_clear');
r_wall = @(z) prm.R0 + prm.A*sin(2*pi*prm.k*z/prm.H);

%% Search grid (coarse)
h_grid  = 100:25:400;      % [mm]  shoulder height
a_grid  = -30:5:30;        % [deg] rail angle
L3_grid = 150:25:350;      % [mm]  q3 joint -> q4 joint
L4_grid = 50:25:150;       % [mm]  q4 joint -> pen tip
Lc_set  = [150 250 400];   % [mm]  characteristic lengths for the score; 250 = the one used in step 3
wrist_clear = 30;          % [mm]  q4 joint must stay this far from the wall      PLACEHOLDER

[HH, AA, L3s, L4s] = ndgrid(h_grid, a_grid, L3_grid, L4_grid);
N  = numel(HH);
np = size(spine.P, 2);

% Wall profile in the arm plane, for the q4-joint clearance check
zw = linspace(0, prm.H, 400);
rw = r_wall(zw);

%% Search
reason = zeros(N, 1);              % 0 = buildable, 1-4 = first rule that failed
worst  = nan(N, numel(Lc_set));    % lowest score along the E, for each Lc
best   = nan(N, 1);                % highest score along the E (Lc = 250)
q2r    = nan(N, 2);  q3r = nan(N, 2);  q4r = nan(N, 2);   % joint ranges [min max]
clear_min = nan(N, 1);             % closest the q4 joint gets to the wall [mm]
iLc = find(Lc_set == 250);

tic;
for c = 1:N
    L = L0;  L.h = HH(c);  L.alpha = deg2rad(AA(c));  L.L3 = L3s(c);  L.L4 = L4s(c);
    Q = nan(4, np);  ok = true;
    for i = 1:np
        [Q(:,i), oki] = RPRRinv(spine.P(:,i), spine.n(:,i), L);
        if ~oki, ok = false; break; end
    end
    if ~ok, reason(c) = 1; continue; end

    if min(Q(2,:)) < S2.q2_min_allowed, reason(c) = 2; continue; end
    re_r = max(Q(2,:))*cos(L.alpha);  re_z = L.h + max(Q(2,:))*sin(L.alpha);
    if re_r > r_wall(re_z) - S2.wall_clear, reason(c) = 3; continue; end

    % q4 joint position in the arm plane, and its distance to the wall profile
    th3 = L.alpha + Q(3,:);
    Wr  = Q(2,:)*cos(L.alpha) + L.L3*cos(th3);
    Wz  = L.h + Q(2,:)*sin(L.alpha) + L.L3*sin(th3);
    dW  = sqrt((rw(:) - Wr).^2 + (zw(:) - Wz).^2);   % 400 x np
    clear_min(c) = min(dW(:));
    if clear_min(c) < wrist_clear, reason(c) = 4; continue; end

    sc = nan(numel(Lc_set), np);
    for j = 1:numel(Lc_set)
        L.Lc = Lc_set(j);
        for i = 1:np, sc(j,i) = RPRRscore(Q(:,i), L); end
    end
    worst(c,:) = min(sc, [], 2).';
    best(c)    = max(sc(iLc,:));
    q2r(c,:) = [min(Q(2,:)), max(Q(2,:))];
    q3r(c,:) = rad2deg([min(Q(3,:)), max(Q(3,:))]);
    q4r(c,:) = rad2deg([min(Q(4,:)), max(Q(4,:))]);
end
t_run = toc;

robust = min(worst, [], 2);        % worst point of the E, worst of the three Lc
travel = q2r(:,2) - q2r(:,1);

%% Summary
fprintf('Coarse search: %d combinations in %.1f s\n', N, t_run);
fprintf('  h %d-%d mm step %d | alpha %d to %d deg step %d | L3 %d-%d mm step %d | L4 %d-%d mm step %d\n', ...
        h_grid(1), h_grid(end), h_grid(2)-h_grid(1), a_grid(1), a_grid(end), a_grid(2)-a_grid(1), ...
        L3_grid(1), L3_grid(end), L3_grid(2)-L3_grid(1), L4_grid(1), L4_grid(end), L4_grid(2)-L4_grid(1));
fprintf('  rejected: cannot reach the E %d, slider into the column %d, rail outside the bucket %d, q4 joint closer than %d mm to the wall %d\n', ...
        nnz(reason == 1), nnz(reason == 2), nnz(reason == 3), wrist_clear, nnz(reason == 4));
fprintf('  buildable: %d\n', nnz(reason == 0));

cur = find(HH == L0.h & AA == round(rad2deg(L0.alpha)) & L3s == L0.L3 & L4s == L0.L4);
% Best robust score first. Scores equal to two decimals count as a tie (the grid is
% coarse), and a tie goes to the shorter slider travel.
[~, order] = sortrows([-round(robust, 2), travel]);
order = order(~isnan(robust(order)));
if ~isempty(cur) && reason(cur) == 0
    fprintf('Current design (robot_params): rank %d of %d, lowest score %.2f / %.2f / %.2f at Lc = %d / %d / %d mm\n', ...
            find(order == cur), numel(order), worst(cur,:), Lc_set);
end

edge = @(c) [HH(c) == h_grid(1) || HH(c) == h_grid(end), AA(c) == a_grid(1) || AA(c) == a_grid(end), ...
             L3s(c) == L3_grid(1) || L3s(c) == L3_grid(end), L4s(c) == L4_grid(1) || L4s(c) == L4_grid(end)];
names = ["h", "alpha", "L3", "L4"];
fprintf('\nTop 10 (ranked by the lowest score over the E and over Lc = %d / %d / %d mm):\n', Lc_set);
fprintf('  rank    h  alpha   L3   L4 | score along E (Lc 250) | lowest at Lc %d/%d/%d | slider q2 [mm]  | q3 [deg]   | q4 [deg]   | q4 joint to wall\n', Lc_set);
for r = 1:min(10, numel(order))
    c = order(r);
    e = edge(c);
    fprintf('  %4d  %3d  %5d  %3d  %3d |      %.2f to %.2f      |   %.2f / %.2f / %.2f   | %3.0f-%3.0f (%3.0f) | %4.0f..%4.0f | %4.0f..%4.0f | %3.0f mm %s\n', ...
            r, HH(c), AA(c), L3s(c), L4s(c), worst(c,iLc), best(c), worst(c,:), q2r(c,:), travel(c), ...
            q3r(c,:), q4r(c,:), clear_min(c), tern(any(e), "<- on the edge: " + strjoin(names(e), ", "), ""));
end

%% Figure 1 — landscape over L3 and L4: best design for each pair (any h, alpha)
land = nan(numel(L4_grid), numel(L3_grid));
for i3 = 1:numel(L3_grid)
    for i4 = 1:numel(L4_grid)
        m = L3s == L3_grid(i3) & L4s == L4_grid(i4);
        if any(~isnan(robust(m))), land(i4, i3) = max(robust(m)); end
    end
end
fig1 = figure('Name', 'Step 3b — landscape L3 x L4', 'Color', 'w');
imagesc(L3_grid, L4_grid, land, 'AlphaData', ~isnan(land)); axis xy; hold on;
cb = colorbar; cb.Label.String = 'best lowest score (any h, \alpha; worst of the three L_c)';
for i3 = 1:numel(L3_grid)
    for i4 = 1:numel(L4_grid)
        if ~isnan(land(i4, i3))
            text(L3_grid(i3), L4_grid(i4), sprintf('%.2f', land(i4, i3)), 'HorizontalAlignment', 'center', 'FontSize', 9);
        end
    end
end
c1 = order(1);
plot(L3s(c1), L4s(c1), 'ko', 'MarkerSize', 22, 'LineWidth', 1.5);
plot(L0.L3 + 8, L0.L4 + 8, 'kp', 'MarkerFaceColor', 'y', 'MarkerSize', 12);   % in the cell's corner, clear of its number
xticks(L3_grid); yticks(L4_grid);
xlabel('L3: q3 joint \rightarrow q4 joint [mm]'); ylabel('L4: q4 joint \rightarrow pen tip [mm]');
title('Coarse search: best score for each pair of link lengths. Circle = best, star = current');
set(gca, 'Color', [0.94 0.94 0.94]);

%% Figure 2 — h x alpha map for the best L3, L4 (where to search more finely)
m2 = nan(numel(a_grid), numel(h_grid));
for ih = 1:numel(h_grid)
    for ia = 1:numel(a_grid)
        c = find(HH == h_grid(ih) & AA == a_grid(ia) & L3s == L3s(c1) & L4s == L4s(c1));
        m2(ia, ih) = robust(c);
    end
end
fig2 = figure('Name', 'Step 3b — h x alpha for the best link lengths', 'Color', 'w');
imagesc(h_grid, a_grid, m2, 'AlphaData', ~isnan(m2)); axis xy; hold on;
cb = colorbar; cb.Label.String = 'lowest score (worst of the three L_c)';
plot(HH(c1), AA(c1), 'ko', 'MarkerFaceColor', 'w', 'MarkerSize', 9);
xlabel('shoulder height h [mm]'); ylabel('rail angle \alpha [deg]');
title(sprintf('L3 = %d mm, L4 = %d mm: which h and \\alpha work, and how well. Circle = best', L3s(c1), L4s(c1)));
set(gca, 'Color', [0.94 0.94 0.94]);

%% Save
outdir = fileparts(mfilename('fullpath'));
search_grid = struct('h', h_grid, 'alpha_deg', a_grid, 'L3', L3_grid, 'L4', L4_grid, 'Lc', Lc_set, 'wrist_clear', wrist_clear);
save(fullfile(outdir, 'step3b_design_search.mat'), 'search_grid', 'HH', 'AA', 'L3s', 'L4s', 'reason', 'worst', 'best', ...
     'robust', 'q2r', 'q3r', 'q4r', 'travel', 'clear_min', 'order');
figdir = fullfile(outdir, 'figures');
exportgraphics(fig1, fullfile(figdir, 'step3b_landscape_L3_L4.png'), 'Resolution', 150);
exportgraphics(fig2, fullfile(figdir, 'step3b_best_h_alpha.png'), 'Resolution', 150);
fprintf('\nSaved step3b_design_search.mat and figures/step3b_*.png\n');

%% Local functions
function out = tern(c, a, b)
if c, out = a; else, out = b; end
end
