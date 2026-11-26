%% Step 1 — wavy surface + letter E projected onto its inner wall
% Builds the surface r(z) = R0 + A*sin(2*pi*k*z/H) (Project PDF p.3),
% sticks the letter E onto the inner wall ("label wrapped around a can"),
% computes the pen direction at every point (perpendicular to the wall, PDF p.4),
% plots everything and saves the points for the next steps.
%
% Frame (as in the PDF p.3 figure): z vertical up along the surface axis,
% origin on the axis at the bottom of the surface, robot base at the origin.
%
% The surface and the letter's size and position were decided by the group on
% 2026-10-24 (marked DECIDED). Every value marked PLACEHOLDER is a starting
% number, NOT a group decision. Change it here and re-run; everything else follows.

clear; close all; clc;

%% Parameters
% Surface (PDF p.3: k fixed, R0/A/H chosen by students within the ratio limits)
k  = 2.5;            % number of waves over the height (fixed for all groups)
R0 = 390;            % [mm] mean radius                     DECIDED 2026-10-24
aR = 0.045;          % A/R0, allowed 0.035–0.055            DECIDED 2026-10-24
hR = 1.15;           % H/R0, allowed 0.095(?)–1.15          DECIDED 2026-10-24
A  = aR*R0;          % [mm] wave depth (amplitude)
H  = hR*R0;          % [mm] surface height

% Letter E (size and position are not specified in the PDF)
E_h      = 0.60*H;   % [mm] letter height                   DECIDED 2026-10-24
E_z0     = 0.20*H;   % [mm] height of the letter's bottom    DECIDED 2026-10-24
E_w      = 0.60*E_h; % [mm] letter width (flat)              DECIDED 2026-10-24
E_mid    = 0.80;     % middle bar length / top bar length    PLACEHOLDER
phi_c    = pi/2;     % [rad] direction of the letter's centre seen from the axis (+y)  DECIDED 2026-10-24

% Sampling of the vertical stroke (the horizontal strokes need only their two
% end points: at constant z the wall is a circle around the axis).
% Step 4 measures how far the pen tip strays between these points.
pts_per_wave = 24;   % points per wave length on the vertical stroke   DECIDED 2026-11-21 (was 12: tip cut 2.1 mm into the wall, see step 4)

% Plot settings
pen_len = 60;        % [mm] length of the drawn pen arrows (display only)

%% Checks against the PDF limits
assert(aR >= 0.035 && aR <= 0.055, 'A/R0 = %.3f is outside 0.035–0.055', aR);
assert(hR >= 0.095 && hR <= 1.15,  'H/R0 = %.3f is outside 0.095–1.15', hR);
if hR < 0.95
    warning('H/R0 < 0.95: legal only if the PDF lower bound 0.095 is not a typo for 0.95');
end
assert(E_z0 >= 0 && E_z0 + E_h <= H, 'The letter does not fit inside the surface height');

%% Surface profile and its slope
r  = @(z) R0 + A*sin(2*pi*k*z/H);                 % wall distance from the axis
dr = @(z) A*(2*pi*k/H)*cos(2*pi*k*z/H);           % dr/dz

%% Letter E in flat coordinates (u to the right as seen from the robot, v up)
% Each stroke starts at the spine. Stroke order and pen lifts are decided in step 4.
u_L = -E_w/2;  u_R = E_w/2;
n_spine = max(2, ceil(pts_per_wave*E_h/(H/k)) + 1);
strokes = struct('name', {}, 'u', {}, 'v', {});
strokes(1) = struct('name', "spine",  'u', u_L*ones(1, n_spine), 'v', linspace(E_h, 0, n_spine));
strokes(2) = struct('name', "top",    'u', [u_L, u_R],                   'v', [E_h, E_h]);
strokes(3) = struct('name', "middle", 'u', [u_L, u_L + E_mid*E_w],       'v', [E_h/2, E_h/2]);
strokes(4) = struct('name', "bottom", 'u', [u_L, u_R],                   'v', [0, 0]);

%% Wrap onto the wall ("label on a can")
% Horizontal flat distance u -> angle around the axis, measured on the mean radius.
% Seen from the axis looking at +y, "right" is +x, i.e. decreasing angle.
for s = 1:numel(strokes)
    [strokes(s).P, strokes(s).n, strokes(s).phi] = wrap_on_wall(strokes(s).u, strokes(s).v, E_z0, phi_c, R0, r, dr);
end

%% Check: the pen is perpendicular to the wall
% Two directions lie in the wall at each point: going up along the wall and going
% around the axis. The pen must be at right angles to both (dot product = 0).
dot_max = 0;
for s = 1:numel(strokes)
    z   = E_z0 + strokes(s).v;  phi = strokes(s).phi;
    t_up     = [dr(z).*cos(phi); dr(z).*sin(phi); ones(size(z))];
    t_around = [-sin(phi); cos(phi); zeros(size(z))];
    dot_max  = max([dot_max, abs(sum(t_up.*strokes(s).n)), abs(sum(t_around.*strokes(s).n))]);
end
assert(dot_max < 1e-9, 'Pen direction is not perpendicular to the wall');

%% Summary
z_all = E_z0 + [strokes.v];
tilt_max_surface = atand(2*pi*k*A/H);
tilt_E = atand(abs(dr(linspace(E_z0, E_z0 + E_h, 500))));
fprintf('Surface: R0 = %.0f mm, A = %.1f mm, H = %.0f mm, k = %.1f\n', R0, A, H, k);
fprintf('Wall distance from axis: %.0f to %.0f mm\n', R0 - A, R0 + A);
fprintf('One wave is %.0f mm tall\n', H/k);
fprintf('Steepest wall tilt: %.1f deg on the whole surface, %.1f deg along the E\n', tilt_max_surface, max(tilt_E));
fprintf('Letter E: %.0f mm tall, %.0f mm wide (flat), from z = %.0f to %.0f mm\n', E_h, E_w, min(z_all), max(z_all));
fprintf('Letter E spans %.1f deg around the axis\n', rad2deg(E_w/R0));
fprintf('Pen perpendicular to the wall at every point (largest error %.1e)\n', dot_max);
fprintf('Points: spine %d, top %d, middle %d, bottom %d\n', numel(strokes(1).u), numel(strokes(2).u), numel(strokes(3).u), numel(strokes(4).u));

%% Figure 1 — 3D view
fig1 = figure('Name', 'Step 1 — surface and letter E', 'Color', 'w');
hold on; axis equal; grid on; view(35, 20);
th = linspace(phi_c - pi/2, phi_c + pi/2, 90);            % half of the surface, behind the letter
zz = linspace(0, H, 120);
[TH, ZZ] = meshgrid(th, zz);
RR = r(ZZ);
surf(RR.*cos(TH), RR.*sin(TH), ZZ, 'FaceAlpha', 0.25, 'EdgeColor', 'none', 'FaceColor', [0.45 0.6 0.9]);
plot3([0 0], [0 0], [0 H], 'k--');                        % axis
plot3(0, 0, 0, 'ks', 'MarkerFaceColor', [0.2 0.5 0.85], 'MarkerSize', 10);   % robot base
for s = 1:numel(strokes)
    % draw each stroke as a dense curve on the wall, plus the sampled points
    uu = linspace(strokes(s).u(1), strokes(s).u(end), 100);
    vv = linspace(strokes(s).v(1), strokes(s).v(end), 100);
    Pd = wrap_on_wall(uu, vv, E_z0, phi_c, R0, r, dr);
    plot3(Pd(1,:), Pd(2,:), Pd(3,:), '-', 'Color', [0.83 0.33 0.49], 'LineWidth', 2);
    P = strokes(s).P;  N = strokes(s).n;
    plot3(P(1,:), P(2,:), P(3,:), 'k.', 'MarkerSize', 10);
    quiver3(P(1,:) - pen_len*N(1,:), P(2,:) - pen_len*N(2,:), P(3,:) - pen_len*N(3,:), ...
            pen_len*N(1,:), pen_len*N(2,:), pen_len*N(3,:), 0, 'Color', [0.5 0.47 0.87], 'MaxHeadSize', 0.08);
end
xlabel('x [mm]'); ylabel('y [mm]'); zlabel('z [mm]');
title(sprintf('R_0 = %.0f mm, A = %.1f mm, H = %.0f mm', R0, A, H));

%% Figure 2 — side cut through the letter's spine
fig2 = figure('Name', 'Step 1 — side cut', 'Color', 'w');
hold on; axis equal; grid on;
plot(r(zz), zz, 'k-', 'LineWidth', 1.2);                   % wall profile
plot([0 0], [0 H], 'k--');                                 % axis
plot(0, 0, 'ks', 'MarkerFaceColor', [0.2 0.5 0.85], 'MarkerSize', 10);
zs = E_z0 + strokes(1).v;                                  % spine heights
rho = r(zs);                                               % distance from the axis
Nr = 1./sqrt(1 + dr(zs).^2);  Nz = -dr(zs)./sqrt(1 + dr(zs).^2);   % pen direction in the cut plane
plot(rho, zs, 'o', 'Color', [0.83 0.33 0.49], 'MarkerFaceColor', [0.83 0.33 0.49], 'MarkerSize', 4);
quiver(rho - pen_len*Nr, zs - pen_len*Nz, pen_len*Nr, pen_len*Nz, 0, 'Color', [0.5 0.47 0.87], 'MaxHeadSize', 0.08);
xlim([-20, R0 + A + 40]); ylim([-10, H + 10]);
xlabel('distance from the axis [mm]'); ylabel('z [mm]');
title('Side cut: wall, spine points of the E, pen directions');

%% Save for the next steps
outdir = fileparts(mfilename('fullpath'));
params = struct('k', k, 'R0', R0, 'A', A, 'H', H, 'E_h', E_h, 'E_w', E_w, 'E_z0', E_z0, ...
                'E_mid', E_mid, 'phi_c', phi_c, 'pts_per_wave', pts_per_wave);
save(fullfile(outdir, 'step1_E_points.mat'), 'params', 'strokes');
figdir = fullfile(outdir, 'figures');
if ~exist(figdir, 'dir'), mkdir(figdir); end
exportgraphics(fig1, fullfile(figdir, 'step1_3d.png'), 'Resolution', 150);
exportgraphics(fig2, fullfile(figdir, 'step1_side_cut.png'), 'Resolution', 150);
fprintf('Saved step1_E_points.mat and figures/step1_*.png\n');

%% Local functions
function [P, n, phi] = wrap_on_wall(u, v, z0, phi_c, R0, r, dr)
% Flat letter point (u, v) -> point P on the wall and pen direction n (unit,
% perpendicular to the wall, pointing from the robot into the wall).
z   = z0 + v;
phi = phi_c - u/R0;
rho = r(z);
P   = [rho.*cos(phi); rho.*sin(phi); z];
n   = [cos(phi); sin(phi); -dr(z)];
n   = n ./ vecnorm(n);
end
